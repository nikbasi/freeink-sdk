#include "PowerManager.h"

#include <Arduino.h>
#include <BoardConfig.h>
#include <driver/gpio.h>
#include <esp_sleep.h>
#include <esp_system.h>
#include <soc/soc_caps.h>
#if FREEINK_DEVICE_WS397
#include <Axp2101.h>
#endif

namespace freeink {
namespace {
int8_t powerPin() { return BoardConfig::ACTIVE.input.power; }
bool powerActiveHigh() { return BoardConfig::ACTIVE.input.powerActiveHigh; }

PowerManager::SleepParkHook s_parkHook = nullptr;

// Consumed by the next deep sleep; see armDebugTimerWake(). Plain RAM, so it
// cannot survive a sleep and re-arm itself by accident.
uint32_t s_debugTimerWakeSeconds = 0;
bool s_debugSimulateHeld = false;

// Sleep-entry telemetry. RTC_NOINIT rather than plain statics because every
// interesting value here is produced on the way INTO sleep, after the SD card is
// unmounted and the console is down — the next boot is the only reader it will
// ever have.
constexpr uint32_t PM_COUNTER_MAGIC = 0x50574D31u;  // 'PWM1'
constexpr uint32_t FLAG_TIMED_OUT = 1u << 0;
constexpr uint32_t FLAG_STUCK_TIMER = 1u << 1;

RTC_NOINIT_ATTR uint32_t s_counterMagic;
RTC_NOINIT_ATTR uint32_t s_releaseWaitMs;
RTC_NOINIT_ATTR uint32_t s_releaseWaitTotalMs;
RTC_NOINIT_ATTR uint32_t s_releaseTimeouts;
RTC_NOINIT_ATTR uint32_t s_stuckStreak;
RTC_NOINIT_ATTR uint32_t s_flags;

// RTC_NOINIT is uninitialised on a cold boot, so nothing may be read before the
// magic has been checked once.
void ensureCounters() {
  if (s_counterMagic == PM_COUNTER_MAGIC) return;
  s_counterMagic = PM_COUNTER_MAGIC;
  s_releaseWaitMs = 0;
  s_releaseWaitTotalMs = 0;
  s_releaseTimeouts = 0;
  s_stuckStreak = 0;
  s_flags = 0;
}

// Records a deepSleep() abort (see deepSleep()) across the esp_restart() that
// follows it, so the next boot can report it via takeAbortedSleepInfo().
// RTC_NOINIT_ATTR survives esp_restart() (a warm reset, same RTC-memory domain)
// but is garbage on a cold boot -- magic-guarded like main.cpp's silentRebootMagic.
constexpr uint32_t ABORTED_SLEEP_MAGIC = 0x41424f52;  // 'ABOR'

struct AbortedSleepRecord {
  bool aborted;
  int wakeupCause;
  int wakePinLevel;
};
}  // namespace

RTC_NOINIT_ATTR uint32_t abortedSleepMagic;
RTC_NOINIT_ATTR AbortedSleepRecord abortedSleepRecord;

void PowerManager::setSleepParkHook(const SleepParkHook hook) { s_parkHook = hook; }

void PowerManager::armDebugTimerWake(const uint32_t seconds, const bool simulateHeld) {
  s_debugTimerWakeSeconds = seconds;
  s_debugSimulateHeld = simulateHeld;
}

void PowerManager::armWakeOnPins(uint64_t gpioMask, bool wakeLow) {
#if SOC_PM_SUPPORT_EXT1_WAKEUP
  // Xtensa (S3/S2, classic ESP32): RTC ext1. Pins must be RTC GPIOs.
  //
  // The classic ESP32 RTC has no "any low" mode — only ESP_EXT1_WAKEUP_ALL_LOW
  // ("wake when ALL selected pins are low"). For a single wake pin (the common
  // power-button case) ALL_LOW and ANY_LOW are identical; a multi-pin low wake on
  // classic ESP32 fires only when every pin is low. S2/S3 expose ANY_LOW directly.
#if defined(CONFIG_IDF_TARGET_ESP32)
  const esp_sleep_ext1_wakeup_mode_t lowMode = ESP_EXT1_WAKEUP_ALL_LOW;
#else
  const esp_sleep_ext1_wakeup_mode_t lowMode = ESP_EXT1_WAKEUP_ANY_LOW;
#endif
  esp_sleep_enable_ext1_wakeup(gpioMask, wakeLow ? lowMode : ESP_EXT1_WAKEUP_ANY_HIGH);
#elif SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
  // RISC-V (C3/C6/H2): the deep-sleep "gpio" wakeup source.
  esp_deep_sleep_enable_gpio_wakeup(gpioMask, wakeLow ? ESP_GPIO_WAKEUP_GPIO_LOW : ESP_GPIO_WAKEUP_GPIO_HIGH);
#else
#error "FreeInk PowerManager: target has no supported deep-sleep GPIO wakeup source"
#endif
}

bool PowerManager::armPowerButtonWakeup() {
  const int8_t pin = powerPin();
  if (pin < 0) return false;
  const bool activeHigh = powerActiveHigh();

  // Hold the idle level with the opposite pull so the line is defined in sleep.
  pinMode(pin, activeHigh ? INPUT_PULLDOWN : INPUT_PULLUP);
  armWakeOnPins(1ULL << pin, /*wakeLow=*/!activeHigh);
  return true;
}

bool PowerManager::powerButtonHeld() {
  if (s_debugSimulateHeld) return true;
  const int8_t pin = powerPin();
  if (pin < 0) return false;
  const bool activeHigh = powerActiveHigh();
  pinMode(pin, activeHigh ? INPUT_PULLDOWN : INPUT_PULLUP);
  return digitalRead(pin) == (activeHigh ? HIGH : LOW);
}

uint32_t PowerManager::waitForPowerButtonRelease() {
  ensureCounters();
  const int8_t pin = powerPin();
  if (pin < 0) {
    s_releaseWaitMs = 0;
    return 0;
  }
  const bool activeHigh = powerActiveHigh();

  pinMode(pin, activeHigh ? INPUT_PULLDOWN : INPUT_PULLUP);
  const int pressedLevel = activeHigh ? HIGH : LOW;
  const uint32_t start = millis();
  // 10 ms rather than the old 50: the wait is now bounded, so the poll interval
  // is bounded work too, and a shorter one hands an ordinary release back to the
  // sleep path faster.
  while (digitalRead(pin) == pressedLevel && (millis() - start) < RELEASE_WAIT_MAX_MS) {
    delay(10);
  }
  const uint32_t waited = millis() - start;
  s_releaseWaitMs = waited;
  s_releaseWaitTotalMs += waited;
  if (waited >= RELEASE_WAIT_MAX_MS) ++s_releaseTimeouts;
  return waited;
}

namespace {
// Drive a rail-enable pin to `offLevel` and latch it so the level survives deep
// sleep (requires gpio_deep_sleep_hold_en(), done in deepSleep()). gpio_hold_dis
// first: a hold left over from a previous cycle would make the writes no-ops.
void holdRailOff(int8_t pin, uint8_t offLevel) {
  if (pin < 0) return;
  const auto g = static_cast<gpio_num_t>(pin);
  gpio_hold_dis(g);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, offLevel);
  gpio_hold_en(g);
}
}  // namespace

void PowerManager::powerDownRailsForSleep() {
  const auto& b = BoardConfig::ACTIVE;
#if FREEINK_DEVICE_WS397
  // The EPD rail is an AXP2101 LDO, not a GPIO, so holdRailOff() below cannot
  // reach it — drop it here or the panel stays powered all through deep sleep.
  axp2101::setEpdPower(false);
#endif
  // Keep RESET defined through deep sleep, but never drive an unpowered panel's
  // input HIGH: on boards with a gated EPD rail (Sticky), that can back-power the
  // controller through its RESET protection diode and turn sleep into a
  // milliamp-level drain. Hold RESET LOW alongside a switched-off rail. Boards
  // whose panel rail remains powered (X4 Pro) keep RESET HIGH so a UC8179 cannot
  // drift out of DSLP and restart its analog booster. EpdBus and XteinkDetect
  // release the hold before issuing a reset pulse on wake.
  const uint8_t resetSleepLevel = b.display.powerEnable >= 0 ? LOW : HIGH;
  holdRailOff(b.display.rst, resetSleepLevel);
  holdRailOff(b.display.powerEnable, LOW);
  // SD enable OFF = the inactive level: LOW for active-high enables, HIGH for the
  // active-low ones (e.g. X4 Pro's GPIO5, which powers the card while held LOW).
  holdRailOff(b.sd.powerEnable, b.sd.powerActiveHigh ? LOW : HIGH);
  // With no rail to cut, the card stays powered through sleep, and
  // esp_sleep_config_gpio_isolate() would leave its chip-select floating — an
  // undefined selection state for a card that is still listening. Hold CS
  // DEASSERTED (HIGH) instead, so it idles deselected. Also hold CLK and MOSI
  // at SPI mode-0 idle (LOW): with the card still on VDD, floating clock/data
  // lines can clock spurious edges into a powered card. MISO stays an input —
  // driving it would fight the card's release and risk back-power. Skipped
  // where the rail IS cut, for the same reason RESET is not held HIGH there:
  // driving an input of an unpowered chip can back-power it through its
  // protection diode. SDCardManager::begin() and BoardConfig::releaseSdRail()
  // drop the holds.
  if (b.sd.powerEnable < 0) {
    holdRailOff(b.sd.cs, HIGH);
    holdRailOff(b.sd.sclk, LOW);
    holdRailOff(b.sd.mosi, LOW);
  }
  holdRailOff(b.touch.powerEnable, b.touch.powerEnableActiveHigh ? LOW : HIGH);
  // Boards with no touch rail have nothing to cut, so the digitizer would keep
  // scanning all through deep sleep — a GT911 costs several mA there, which on
  // its own is the difference between a milliamp-class and a microamp-class
  // sleep. Park it in reset instead (asserted LOW). Opt-in per board: see
  // TouchConfig::holdResetInSleep for why this is not inferred from a missing
  // powerEnable. InputManager's touch bring-up releases the hold on wake.
  if (b.touch.holdResetInSleep) holdRailOff(b.touch.reset, LOW);
  // The mic enable also carries a polarity flag; OFF is the inactive level.
  holdRailOff(b.mic.enable, b.mic.enableActiveHigh ? LOW : HIGH);
}

void PowerManager::deepSleep() {
  // Board park first, while I2C and the pad matrix are still alive: on boards
  // whose panel PMIC hangs off an I2C expander this is the only unconditional
  // power-down it gets. The display driver's own sleep call is guarded by a
  // cached "power is already off" flag and short-circuits in the normal case,
  // so a single missed power-down would otherwise stand for the whole night.
  if (s_parkHook) s_parkHook();

  esp_sleep_config_gpio_isolate();
#if !FREEINK_MCU_C61
  gpio_deep_sleep_hold_en();
#endif
  esp_deep_sleep_start();
  // esp_deep_sleep_start() does not return in normal operation -- reaching
  // this line means sleep entry was rejected/aborted by the SoC rather than
  // the next boot being a fresh wake. A busy-loop here draws full-clock
  // current (~20-30mA) indefinitely and never responds to the power button,
  // making an aborted sleep entry indistinguishable from a real drain bug.
  // Record the abort (RTC_NOINIT_ATTR survives the reset below) so the next
  // boot can report it via takeAbortedSleepInfo(), then reset for real via
  // esp_restart() -- itself [[noreturn]], satisfying this function's contract.
  const int8_t pin = powerPin();
  abortedSleepRecord.aborted = true;
  abortedSleepRecord.wakeupCause = static_cast<int>(esp_sleep_get_wakeup_cause());
  abortedSleepRecord.wakePinLevel = pin >= 0 ? digitalRead(pin) : -1;
  abortedSleepMagic = ABORTED_SLEEP_MAGIC;
  esp_restart();
}

void PowerManager::deepSleepUntilPowerButton() {
  ensureCounters();

  // Start from no wake sources at all, then arm exactly what this sleep needs.
  //
  // Every esp_sleep_enable_*_wakeup() writes the same global trigger word, and
  // that word is not scoped to the sleep that armed it: light sleep and deep
  // sleep read the identical configuration. The idle light sleep between page
  // turns arms a 150 ms timer wake every time it runs, and nothing ever cleared
  // it -- so the first deep sleep after any light sleep inherited that timer and
  // woke 150 ms later, which the user sees as the sleep screen appearing and the
  // device restarting itself on the spot.
  //
  // That went unnoticed for as long as it did because two faults were hiding
  // each other: a key mapped to a pad the LCD peripheral owns pinned the
  // inactivity clock, so light sleep never ran, so the timer was never armed.
  // Fixing the key is what exposed this one. The GPIO trigger leaks the same way
  // (esp_sleep_enable_gpio_wakeup() is documented light-sleep-only but sets a bit
  // here too, where its only effect is to keep RTC_PERIPH powered).
  //
  // Hence ALL rather than naming sources: the next one added to a light-sleep
  // path should not be able to reintroduce this. Everything this sleep actually
  // wants is armed below, after the clear.
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

  waitForPowerButtonRelease();
  s_flags &= ~FLAG_STUCK_TIMER;

  // Additive, and armed before the button so a test sleep still exercises the
  // real wake source rather than replacing it.
  if (s_debugTimerWakeSeconds != 0) {
    esp_sleep_enable_timer_wakeup(static_cast<uint64_t>(s_debugTimerWakeSeconds) * 1000000ULL);
    s_debugTimerWakeSeconds = 0;
  }
  // Read once and clear: the flag lives in plain RAM, but every branch below
  // sleeps without returning, so clearing it here is what guarantees the next
  // real sleep sees the actual pad.
  const bool simulateHeld = s_debugSimulateHeld;
  s_debugSimulateHeld = false;

  if (!simulateHeld && !powerButtonHeld()) {
    // The ordinary case: released, so the level-triggered wake source can be
    // armed without waking us the instant we sleep.
    s_stuckStreak = 0;
    s_flags &= ~FLAG_TIMED_OUT;
    armPowerButtonWakeup();
    deepSleep();
  }

  s_flags |= FLAG_TIMED_OUT;
  ++s_stuckStreak;

  if (s_stuckStreak < STUCK_STREAK_LIMIT) {
    // Arming the wake on a line already at the wake level means waking straight
    // back up. That is deliberate for the first couple of tries: a boot costs a
    // second or two, and it is how a genuinely long press resolves itself.
    armPowerButtonWakeup();
    deepSleep();
  }

  // Persistently asserted. Neither remaining option is free, so take the cheap
  // one: waking immediately burns a boot per cycle, and staying up here to poll
  // burns ~25 mA indefinitely. Sleep on a timer instead — deep-sleep current
  // while it holds — and re-test the line when it fires. wokeFromStuckRetry()
  // tells the consumer that firing was ours, not the user's.
  s_flags |= FLAG_STUCK_TIMER;
  esp_sleep_enable_timer_wakeup(static_cast<uint64_t>(STUCK_RETRY_SECONDS) * 1000000ULL);
  deepSleep();
}

PowerManager::SleepReport PowerManager::sleepReport() {
  ensureCounters();
  SleepReport out;
  out.releaseWaitMs = s_releaseWaitMs;
  out.releaseWaitTotalMs = s_releaseWaitTotalMs;
  out.timeouts = s_releaseTimeouts;
  out.stuckStreak = static_cast<uint8_t>(s_stuckStreak);
  out.lastTimedOut = (s_flags & FLAG_TIMED_OUT) != 0;
  out.sleptOnStuckTimer = (s_flags & FLAG_STUCK_TIMER) != 0;
  return out;
}

void PowerManager::clearSleepCounters() {
  ensureCounters();
  s_releaseWaitMs = 0;
  s_releaseWaitTotalMs = 0;
  s_releaseTimeouts = 0;
}

bool PowerManager::wokeFromStuckRetry() {
  ensureCounters();
  if ((s_flags & FLAG_STUCK_TIMER) == 0) return false;
  return esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER;
}

PowerManager::AbortedSleepInfo PowerManager::takeAbortedSleepInfo() {
  AbortedSleepInfo info;
  if (abortedSleepMagic == ABORTED_SLEEP_MAGIC && abortedSleepRecord.aborted) {
    info.aborted = true;
    info.wakeupCause = abortedSleepRecord.wakeupCause;
    info.wakePinLevel = abortedSleepRecord.wakePinLevel;
  }
  // Clear so a stale record doesn't get reported again on a later boot, and
  // re-stamp the magic so an uninitialized cold-boot read still reports false
  // rather than following a garbage `aborted` bit.
  abortedSleepRecord.aborted = false;
  abortedSleepMagic = ABORTED_SLEEP_MAGIC;
  return info;
}

}  // namespace freeink

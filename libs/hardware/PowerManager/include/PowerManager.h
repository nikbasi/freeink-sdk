#pragma once

#include <cstdint>

// FreeInk SDK — deep-sleep / wake power management.
//
// Owns the one hardware concern the rest of the SDK leaves to the consumer: the
// per-SoC deep-sleep GPIO-wakeup difference. RISC-V parts (C3/C6/H2) wake from
// deep sleep via the "gpio" source (esp_deep_sleep_enable_gpio_wakeup); Xtensa
// parts (S3/S2, classic ESP32) wake via RTC ext1 (esp_sleep_enable_ext1_wakeup).
// Hardcoding either one blocks a multi-MCU build (see docs/consumer-mcu-portability.md).
//
// This picks the right source at compile time from SoC capability macros, and
// reads the wake pin + active level from BoardConfig::ACTIVE.input, so the same
// consumer code deep-sleeps correctly on every supported board. The wake pin must
// be RTC-capable on ext1 parts (true for the de-link power button).

namespace freeink {

class PowerManager {
 public:
  // Ceiling on the release poll below. The poll used to be unbounded, and that
  // is a far more expensive thing to get wrong than it looks: it runs AFTER the
  // battery-log row is written and after every peripheral has been torn down, so
  // a line that never reads released parks the device in a CPU-idle loop —
  // roughly 25 mA, six times deep sleep — for as long as the condition lasts,
  // and leaves nothing behind but a sleep gap the log cannot tell apart from a
  // real sleep. Measured on a T5 S3: two such nights cost 240 mAh and 110 mAh.
  static constexpr uint32_t RELEASE_WAIT_MAX_MS = 4000;
  // How long the device parks on a timer once the line looks genuinely stuck.
  static constexpr uint32_t STUCK_RETRY_SECONDS = 60;
  // Timed-out waits before the button wake is swapped for that timer. ONE,
  // deliberately: arming a level-triggered wake on a line that is still at the
  // wake level wakes the chip the instant it sleeps, and the boot that follows
  // sees the button genuinely held, so it is a full visible restart -- "the
  // sleep screen came up and then it rebooted itself". Retrying that even twice
  // before parking on the timer would ship exactly the symptom the bounded wait
  // was added to prevent. There is no information in the second attempt that the
  // first did not already have.
  static constexpr uint8_t STUCK_STREAK_LIMIT = 1;

  // Board-specific "park everything that draws" step, run as the last thing
  // before the chip stops. Registered once at boot; called from deepSleep(), so
  // no sleep path — not the reader's sleep gesture, not the idle timeout, not
  // the two early returns in setup() that never reach display init — can skip
  // it. Runs while I2C and the GPIO matrix are still alive.
  using SleepParkHook = void (*)();
  static void setSleepParkHook(SleepParkHook hook);

  // Arm wake-on-power-button using the SoC-correct wakeup source and the active
  // board's power pin + polarity (powerActiveHigh -> wake on HIGH, else LOW).
  // Returns false if the board has no power pin (PIN_UNASSIGNED); nothing armed.
  static bool armPowerButtonWakeup();

  // Arm deep-sleep wake on an arbitrary set of GPIOs (gpioMask, wakeLow = wake on
  // the low level) using the SoC-correct source (ext1 on Xtensa, gpio on RISC-V).
  // Use for extra wake lines beyond the power button — a touch INT, a second
  // button, an IO-expander INT. The pins must be RTC-capable on ext1 parts.
  static void armWakeOnPins(uint64_t gpioMask, bool wakeLow = true);

  // Poll the power-button GPIO (raw read, with the matching pull) until released,
  // so deep sleep isn't immediately cancelled by a still-held press. Bounded by
  // RELEASE_WAIT_MAX_MS. Returns the milliseconds spent waiting; the caller
  // decides what a timeout means (see deepSleepUntilPowerButton()).
  static uint32_t waitForPowerButtonRelease();

  // True while the power line still reads asserted.
  static bool powerButtonHeld();

  // Drive every assigned peripheral power-rail enable in the active board
  // profile (display / SD / touch / mic) to its OFF level and latch it with
  // gpio_hold_en() so the load switches stay off through deep sleep (deepSleep()
  // enables gpio_deep_sleep_hold_en(), which makes the holds persist). Without
  // this, boards with gated rails (e.g. Sticky: GT911 on TP_PWR_EN, SD on
  // SD_PWR_EN, EPD on EP_PWR_EN) leave those peripherals powered all through
  // deep sleep — milliamps of standby drain. No-op on boards whose rails are
  // PIN_UNASSIGNED (X4/X3). Call after the display driver's deep-sleep command
  // and before deepSleep(); wake is a chip reset, so rails re-enable in the
  // normal init path. Display RESET is held LOW when its rail is cut (avoids
  // back-powering an unpowered controller) and HIGH when its rail remains on
  // (keeps deep-sleep state stable). NOTE: cutting the touch rail forfeits
  // touch-to-wake.
  //
  // Boards with NO gated rail are not left alone either, because there the
  // peripheral simply stays powered and something has to quiet it:
  //   * Touch — the controller is parked in reset (RESET held asserted) on
  //     profiles that opt in via TouchConfig::holdResetInSleep. A GT911 left
  //     scanning costs several mA, dwarfing every other sleep load.
  //   * SD — chip-select is held DEASSERTED so a still-powered card idles
  //     deselected instead of floating into an undefined selection state.
  //     On those same ungated boards, SCLK and MOSI are held at SPI mode-0
  //     idle (LOW) so a floating clock cannot tick the card while it still
  //     has VDD. MISO is left alone (card-driven when selected).
  // Every one of these holds is released again by the corresponding bring-up
  // path (InputManager's touch reset, SDCardManager::begin(),
  // BoardConfig::releaseSdRail()) — gpio_hold_en survives the wake reset, and a
  // held pad silently swallows writes, so a missing release means dead hardware
  // after the first sleep and only after a sleep.
  static void powerDownRailsForSleep();

  // Isolate floating GPIOs to cut sleep current, then enter deep sleep. Does not
  // return — the chip resets on wake.
  [[noreturn]] static void deepSleep();

  // Convenience: wait for release, arm the power-button wakeup, then deep sleep.
  [[noreturn]] static void deepSleepUntilPowerButton();

  // --- sleep-entry telemetry --------------------------------------------------
  // Kept in RTC_NOINIT, so it survives the sleep itself and the wake reset and
  // can be read on the next boot — which is the only chance anything has to see
  // it, since the card is unmounted and the console is down by the time the
  // sleep path runs.
  struct SleepReport {
    uint32_t releaseWaitMs = 0;       // the last wait
    uint32_t releaseWaitTotalMs = 0;  // cumulative since clearSleepCounters()
    uint32_t timeouts = 0;            // waits that hit RELEASE_WAIT_MAX_MS
    uint8_t stuckStreak = 0;          // consecutive timeouts, reset by a clean sleep
    bool lastTimedOut = false;
    bool sleptOnStuckTimer = false;  // the last sleep parked on the retry timer
  };
  static SleepReport sleepReport();
  static void clearSleepCounters();

  // Arm an additional timer wake on the NEXT deep sleep, then forget it.
  //
  // Exists because the deep-sleep path is otherwise untestable on a bench: the
  // only way in is a real button, the only way out is a real button, and the
  // USB console dies on the way down. With this, a serial command can drive the
  // complete path -- park hook, release poll, pad isolation, the sleep itself --
  // and the device comes back on its own with its RTC_NOINIT findings intact for
  // the next boot to report. Zero disables.
  // simulateHeld makes powerButtonHeld() answer true for that one sleep, which
  // is the only way to reach the "line never released" branch without a wedged
  // button: the poll reads a real pad, and a bench cannot hold one down. The
  // timer wake is what makes reaching that branch survivable.
  static void armDebugTimerWake(uint32_t seconds, bool simulateHeld = false);

  // True when this boot is the retry timer firing after a stuck power line,
  // rather than anything the user did. The consumer should go straight back to
  // sleep instead of lighting the screen up in front of nobody.
  static bool wokeFromStuckRetry();
  // esp_deep_sleep_start() is documented as not returning, but deepSleep()
  // calls esp_restart() if it ever does (instead of a busy-loop -- see that
  // function). This reports whether the *previous* boot took that abort
  // path, and clears the record so it only surfaces once. wakeupCause/
  // wakePinLevel are the values captured at the moment of the abort.
  struct AbortedSleepInfo {
    bool aborted = false;
    int wakeupCause = 0;
    int wakePinLevel = 0;
  };
  static AbortedSleepInfo takeAbortedSleepInfo();
};

}  // namespace freeink

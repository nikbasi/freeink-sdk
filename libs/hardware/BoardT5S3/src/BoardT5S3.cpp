#include <BoardT5S3.h>
#include <InputManager.h>
#include <SPI.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <cassert>

namespace BoardT5S3 {
namespace {
constexpr uint8_t PCA_REG_INPUT0 = 0x00;
constexpr uint8_t PCA_REG_OUTPUT0 = 0x02;
constexpr uint8_t PCA_REG_CONFIG0 = 0x06;

SemaphoreHandle_t i2cMutex = nullptr;

// What parkEpdPowerForSleep() last saw. Read back on the next boot by the
// battery log; see the header for why the sleep path cannot report it itself.
// RTC_NOINIT survives deep sleep and the wake reset but is garbage on a cold
// boot, hence the magic.
constexpr uint32_t PARK_MAGIC = 0x50524B31u;  // 'PRK1'
RTC_NOINIT_ATTR uint32_t s_parkMagic;
RTC_NOINIT_ATTR uint8_t s_lastParkState;
RTC_NOINIT_ATTR bool s_lastParkOk;

void ensureParkState() {
  if (s_parkMagic == PARK_MAGIC) return;
  s_parkMagic = PARK_MAGIC;
  s_lastParkState = 0;
  s_lastParkOk = false;
}

SemaphoreHandle_t ensureI2CMutex() {
  if (i2cMutex == nullptr) {
    i2cMutex = xSemaphoreCreateRecursiveMutex();
    assert(i2cMutex != nullptr && "Failed to create I2C mutex");
  }
  return i2cMutex;
}

bool i2cWriteReg(uint8_t addr, uint8_t reg, const uint8_t* data, size_t len) {
  ScopedI2CLock lock;
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (data != nullptr && len > 0) {
    Wire.write(data, len);
  }
  return Wire.endTransmission() == 0;
}

bool i2cReadReg(uint8_t addr, uint8_t reg, uint8_t* data, size_t len) {
  ScopedI2CLock lock;
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  const uint8_t requested = static_cast<uint8_t>(len);
  if (Wire.requestFrom(addr, requested) != requested) {
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }
  for (size_t i = 0; i < len; ++i) {
    data[i] = Wire.read();
  }
  return true;
}

bool updatePca9535Bit(uint8_t baseReg, uint8_t pin, bool high) {
  const uint8_t port = pin / 8;
  const uint8_t bit = pin % 8;
  uint8_t value = 0;
  if (!i2cReadReg(T5S3_PCA9535_ADDR, baseReg + port, &value, 1)) {
    return false;
  }
  if (high) {
    value |= static_cast<uint8_t>(1U << bit);
  } else {
    value &= static_cast<uint8_t>(~(1U << bit));
  }
  return i2cWriteReg(T5S3_PCA9535_ADDR, baseReg + port, &value, 1);
}

uint8_t inputButtonHook() { return readButton() ? static_cast<uint8_t>(1U << InputManager::BTN_DOWN) : 0; }
}  // namespace

ScopedI2CLock::ScopedI2CLock() {
  xSemaphoreTakeRecursive(ensureI2CMutex(), portMAX_DELAY);
  locked_ = true;
}

ScopedI2CLock::~ScopedI2CLock() {
  if (locked_) {
    xSemaphoreGiveRecursive(ensureI2CMutex());
    locked_ = false;
  }
}

void beginI2C() {
  ensureI2CMutex();
  Wire.begin(T5S3_SDA, T5S3_SCL);
  Wire.setClock(T5S3_I2C_FREQ);
  Wire.setTimeOut(50);
}

void prepareSdBus() {
#if T5S3_HAS_LORA_GPS
  // Deselect the radio before the SD card shares the bus with it.
  pinMode(T5S3_LORA_CS, OUTPUT);
  digitalWrite(T5S3_LORA_CS, HIGH);
#endif
  pinMode(T5S3_SD_CS, OUTPUT);
  digitalWrite(T5S3_SD_CS, HIGH);
  SPI.begin(T5S3_SPI_SCLK, T5S3_SPI_MISO, T5S3_SPI_MOSI, T5S3_SD_CS);
}

// Cuts power to the LoRa/GPS module and parks the pins it would drive. On a
// Pro Lite there is no module to cut power to, and its four pins are keys:
// InputManager has already pulled them up and is polling them, so touching
// them here would be taking them back off the user.
void disableGpsLora() {
#if T5S3_HAS_LORA_GPS
  pinMode(T5S3_LORA_CS, OUTPUT);
  digitalWrite(T5S3_LORA_CS, HIGH);
  pinMode(T5S3_LORA_RST, OUTPUT);
  digitalWrite(T5S3_LORA_RST, LOW);
  pinMode(T5S3_LORA_IRQ, INPUT);
  pinMode(T5S3_LORA_BUSY, INPUT);
#endif
  pinMode(T5S3_GPS_RXD, INPUT);
  pinMode(T5S3_GPS_TXD, INPUT);

  // Left low on both builds: the rail feeds an absent module on the Lite, and
  // the enable line is the one pin the radio does not share with anything.
  writePca9535Pin(PCA9535_IO00_LORA_GPS_EN, false);
  setPca9535PinMode(PCA9535_IO00_LORA_GPS_EN, OUTPUT);
}

void begin() {
  beginI2C();
  pinMode(T5S3_BOOT_BTN, INPUT_PULLUP);
  if (T5S3_PCA9535_INT > 0) {
    pinMode(T5S3_PCA9535_INT, INPUT_PULLUP);
  }

  prepareSdBus();
  disableGpsLora();
  setPca9535PinMode(PCA9535_IO12_BUTTON, INPUT);
  InputManager::setButtonHook(inputButtonHook);
}

bool pca9535Present() {
  ScopedI2CLock lock;
  Wire.beginTransmission(T5S3_PCA9535_ADDR);
  return Wire.endTransmission() == 0;
}

bool setPca9535PinMode(uint8_t pin, uint8_t mode) {
  const bool inputMode = mode != OUTPUT;
  return updatePca9535Bit(PCA_REG_CONFIG0, pin, inputMode);
}

bool writePca9535Pin(uint8_t pin, bool high) { return updatePca9535Bit(PCA_REG_OUTPUT0, pin, high); }

bool readPca9535Pin(uint8_t pin, bool* high) {
  if (!high) {
    return false;
  }
  const uint8_t port = pin / 8;
  const uint8_t bit = pin % 8;
  uint8_t value = 0;
  if (!i2cReadReg(T5S3_PCA9535_ADDR, PCA_REG_INPUT0 + port, &value, 1)) {
    return false;
  }
  *high = (value & (1U << bit)) != 0;
  return true;
}

bool readButton() {
  bool high = true;
  if (!readPca9535Pin(PCA9535_IO12_BUTTON, &high)) {
    return false;
  }
  return !high;
}

bool parkEpdPowerForSleep() {
  // All five EPD control lines live in port 1 (linear indexes 8..15).
  //
  // VCOM has to fall before the boost rails do, and the gate-start pin has to
  // be low before either. Dropping them in one write leaves the common plane
  // biased while a gate is still selected, and the glass then drifts to grey
  // over a few seconds on a page that had just looked clean.
  constexpr uint8_t OE_BIT = static_cast<uint8_t>(1U << (PCA9535_IO10_EP_OE - 8));
  constexpr uint8_t MODE_BIT = static_cast<uint8_t>(1U << (PCA9535_IO11_EP_MODE - 8));
  constexpr uint8_t PWRUP_BIT = static_cast<uint8_t>(1U << (PCA9535_IO13_TPS_PWRUP - 8));
  constexpr uint8_t VCOM_BIT = static_cast<uint8_t>(1U << (PCA9535_IO14_VCOM_CTRL - 8));
  constexpr uint8_t WAKEUP_BIT = static_cast<uint8_t>(1U << (PCA9535_IO15_TPS_WAKEUP - 8));
  constexpr uint8_t EPD_MASK = static_cast<uint8_t>(OE_BIT | MODE_BIT | PWRUP_BIT | VCOM_BIT | WAKEUP_BIT);

  // Stop the gate driver before the supply moves.
  pinMode(EP_STV, OUTPUT);
  digitalWrite(EP_STV, LOW);

  ensureParkState();
  s_lastParkOk = false;
  for (int attempt = 0; attempt < 3; ++attempt) {
    uint8_t config = 0;
    uint8_t output = 0;
    if (!i2cReadReg(T5S3_PCA9535_ADDR, PCA_REG_CONFIG0 + 1, &config, 1)) continue;
    if (!i2cReadReg(T5S3_PCA9535_ADDR, PCA_REG_OUTPUT0 + 1, &output, 1)) continue;

    // Outputs, or driving them means nothing.
    const uint8_t wantConfig = static_cast<uint8_t>(config & ~EPD_MASK);
    if (wantConfig != config && !i2cWriteReg(T5S3_PCA9535_ADDR, PCA_REG_CONFIG0 + 1, &wantConfig, 1)) continue;

    // Already parked (the post-refresh retry). The discharge waits below are
    // only there for a rail that is still up.
    if ((output & EPD_MASK) == 0) {
      s_lastParkState = output;
      s_lastParkOk = true;
      break;
    }

    // OE, MODE, and VCOM off. PWRUP and WAKEUP stay so the negative boost can
    // discharge the VCOM capacitor (about 4.7µF into 1k) before the rails fall.
    const uint8_t vcomOff =
        static_cast<uint8_t>((output & ~EPD_MASK) | (output & (PWRUP_BIT | WAKEUP_BIT)));
    if (!i2cWriteReg(T5S3_PCA9535_ADDR, PCA_REG_OUTPUT0 + 1, &vcomOff, 1)) continue;
    delay(40);

    const uint8_t railsOff = static_cast<uint8_t>(vcomOff & ~PWRUP_BIT);
    if (!i2cWriteReg(T5S3_PCA9535_ADDR, PCA_REG_OUTPUT0 + 1, &railsOff, 1)) continue;

    // Default power-down keeps the negative rail up until the positive rail
    // has fallen, then holds it another 100ms to finish the discharge.
    // Dropping WAKEUP inside that gap cuts one side first and walks a latched
    // page back toward grey. PWR_GOOD goes low when the sequenced rails are
    // down; the extra wait is the negative-rail tail. A stuck pin must not
    // hang the refresh.
    const uint32_t railsDownAt = millis();
    bool powerGood = true;
    while (millis() - railsDownAt < 250) {
      if (readPca9535Pin(PCA9535_IO16_TPS_PWR_GOOD, &powerGood) && !powerGood) break;
      delay(5);
    }
    delay(100);

    // WAKEUP last: the PMIC wants its outputs disabled before it loses supply.
    const uint8_t parked = static_cast<uint8_t>(railsOff & ~WAKEUP_BIT);
    if (!i2cWriteReg(T5S3_PCA9535_ADDR, PCA_REG_OUTPUT0 + 1, &parked, 1)) continue;

    uint8_t readback = 0;
    if (!i2cReadReg(T5S3_PCA9535_ADDR, PCA_REG_OUTPUT0 + 1, &readback, 1)) continue;
    s_lastParkState = readback;
    if ((readback & EPD_MASK) == 0) {
      s_lastParkOk = true;
      break;
    }
  }

  return s_lastParkOk;
}

uint8_t lastEpdParkState() {
  ensureParkState();
  return s_lastParkState;
}

bool lastEpdParkOk() {
  ensureParkState();
  return s_lastParkOk;
}

}  // namespace BoardT5S3

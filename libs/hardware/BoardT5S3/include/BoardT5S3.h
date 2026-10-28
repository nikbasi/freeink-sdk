#pragma once

#include <Arduino.h>

#include "BoardT5S3Pins.h"

// Feature markers. See docs/launcher-features.md.
// parkEpdPowerForSleep() drops STV, then VCOM, waits for the VCOM discharge,
// then the boost rails, and WAKEUP last, so a settled page does not fade.
#define FREEINK_T5S3_STAGED_RAIL_PARK 1
// Warm panels still get the room-temperature drive length: shorter impulses do
// not latch and the page relaxes to grey.
#define FREEINK_T5S3_WAVEFORM_FLOOR_C 26

namespace BoardT5S3 {

class ScopedI2CLock {
 public:
  ScopedI2CLock();
  ~ScopedI2CLock();

  ScopedI2CLock(const ScopedI2CLock&) = delete;
  ScopedI2CLock& operator=(const ScopedI2CLock&) = delete;

 private:
  bool locked_ = false;
};

void begin();
void beginI2C();
void prepareSdBus();
void disableGpsLora();
bool pca9535Present();
bool readPca9535Pin(uint8_t pin, bool* high);
bool writePca9535Pin(uint8_t pin, bool high);
bool setPca9535PinMode(uint8_t pin, uint8_t mode);
bool readButton();

// Drive the EPD PMIC control lines (EP_OE, EP_MODE, TPS PWRUP, VCOM, TPS WAKEUP)
// to their off state, verify the expander took the write, and retry if it did
// not. Idempotent, and safe from the moment begin() has brought I2C up — which
// matters, because the two early returns in setup() reach deep sleep before the
// display driver has run prepareEpdPower() even once.
//
// Why this exists as its own entry point rather than being left to the driver:
// the PCA9535 sits on a rail that never goes down, so its output latches survive
// the ESP's deep sleep AND the wake reset. Whatever the last session left in
// them is what the board sleeps with. A TPS65185 left awake there costs tens of
// milliamps for as long as the sleep lasts, and nothing in the log can see it —
// the panel is re-initialised on the next boot, so the awake current reads
// completely normal afterwards.
//
// Returns true when the expander read back the parked state.
bool parkEpdPowerForSleep();

// Port-1 output byte read back by the last parkEpdPowerForSleep(), and whether
// that readback matched. Recorded for the battery log: the sleep path has no
// console and no card, so this is the only way the next boot can say what the
// board actually slept with.
uint8_t lastEpdParkState();
bool lastEpdParkOk();

}  // namespace BoardT5S3

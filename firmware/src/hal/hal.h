// Hardware layer: one small class per job, one .cpp per part (VL53L5CX x2, Grove 3-axis
// accelerometer, PWM motors, on-board LED). The logic above it (lib/saferoute) never sees which
// part it is.
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "core/params.h"
#include "core/types.h"

namespace hal {

// ---- I2C ----------------------------------------------------------------
bool i2cBegin();                 // I2C0: both ToF sensors and the accelerometer
TwoWire& bus0();
void i2cScan(Print& out);

// ---- two VL53L5CX on I2C0, separated at boot with their LPn pins
class Tof {
 public:
  bool begin();                  // true if both came up (ground only if FEATURE_FORWARD_TOF is 0)
  void poll(sr::RangeReading& ground, sr::RangeReading& forward);  // non-blocking
  // Re-initialise faulty sensors, each one alone; a full reload blocks ~1-2.5 s, so it is
  // rate-limited.
  bool reinit(bool ground, bool forward);
  void shutdown();               // sensors off / asleep (deep sleep)
  const char* name() const;
  bool groundOk = false, forwardOk = false;
  void (*onLongBlock)() = nullptr;   // called just before a reinit that blocks (main: motors off)
};

// ---- IMU ----------------------------------------------------------------
class Imu {
 public:
  bool begin();
  bool read(sr::ImuSample& s);   // m/s2, sensor frame (gyro always 0)
  const char* name() const;
  uint8_t whoAmI = 0;
};

// ---- two haptic channels ---------------------------------------------------
class Haptics {
 public:
  bool begin();
  void set(uint8_t left, uint8_t right);   // 0-255, writes only on change (right: HAPTIC_STAGGER_MS late)
  void sleep();                            // both off at once
  const char* name() const;
  bool leftOk = false, rightOk = false;
 private:
  uint8_t delayedRight(uint8_t r, uint32_t now);
  static const uint8_t kHist = 16;         // >= HAPTIC_STAGGER_MS / loop period + margin
  uint32_t hist_t_[kHist] = {0};
  uint8_t hist_v_[kHist] = {0};
  uint8_t hist_i_ = 0, hist_n_ = 0;
  int16_t last_l_ = -1, last_r_ = -1;
  int16_t out_l_ = -1, out_r_ = -1;
  uint32_t kick_l_ = 0, kick_r_ = 0;
};

// ---- status LED ---------------------------------------------------------------
class StatusLed {
 public:
  void begin();
  void show(sr::DeviceState st, sr::Mode mode, uint32_t now);
  void on();
  void off();
};

// ---- battery, storage ------------------------------------------------------------
float readBatteryVolts();        // NAN when no divider is fitted

namespace store {
void load(sr::Params& p, bool* calibrated);
void saveCalibration(float L, float beta_deg);
void saveShaft(const sr::Vec3& shaft);
void saveFloat(const char* key, float v);
void clear();
}  // namespace store

}  // namespace hal

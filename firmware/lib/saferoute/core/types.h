// Data types shared by the logic, the hardware layer and the replay tool.
#pragma once
#include <math.h>
#include <stdint.h>
#include "core/vec3.h"

namespace sr {

// One IMU sample in the sensor's own frame.
struct ImuSample {
  bool ok;        // false = read failed this tick
  Vec3 acc;       // m/s2, gravity included (reads +g "up" when at rest)
  Vec3 gyro;      // rad/s, right-handed
};

enum class RangeStatus : uint8_t {
  None = 0,       // no new frame this tick
  Ok = 1,         // valid distance
  NoTarget = 2,   // sensor works but saw nothing in range (signal / out-of-bounds fail)
  Invalid = 3,    // noisy or bad frame (sigma fail, wrap-around, ...)
  Fault = 4,      // I2C error / timeout / sensor not initialised
};

struct RangeReading {
  RangeStatus status;
  uint16_t mm;
  // VL53L5CX ground beam only: the plane the four central zones see (NAN = none / not flat):
  // the beam's angle below that plane and the sensor's height above it
  float plane_dep_deg;
  float plane_h_m;
  RangeReading() : status(RangeStatus::None), mm(0), plane_dep_deg(NAN), plane_h_m(NAN) {}
  RangeReading(RangeStatus s, uint16_t d) : status(s), mm(d), plane_dep_deg(NAN), plane_h_m(NAN) {}
  bool fresh() const { return status != RangeStatus::None; }
};

// What the alert filter decided. Values are the telemetry "alert" codes.
enum class Hazard : uint8_t {
  None = 0,
  Drop = 1,
  HeadLeft = 2,
  HeadRight = 3,
  HeadCenter = 4,
  DropUrgent = 5,   // drop with little time left at the current speed (haptics/telemetry only)
};

// How the cane is being used (telemetry "tech": - S W T H)
enum class Technique : uint8_t { Unknown = 0, Still = 1, Sweep = 2, Tap = 3, Hold = 4 };

// Overall device state, drives the status LED. Values are the telemetry "st" codes.
enum class DeviceState : uint8_t {
  Boot = 0,
  Ok = 1,
  Paused = 2,       // cane near vertical
  Alert = 3,
  Calibrating = 4,
  Fault = 5,        // a sensor is down: no alerts from it, auto re-init
  LowBattery = 6,
  NeedsCal = 7,     // running on default geometry
  Off = 8,
  Standby = 9,      // waiting for the start taps: no alerts
};

enum class Mode : uint8_t { Smart = 0, Naive = 1 };

}  // namespace sr

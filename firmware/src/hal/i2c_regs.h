// Plain I2C register access (no library), used by the accelerometer driver (hal/imu_accel.cpp).
#pragma once
#include <Arduino.h>
#include <Wire.h>

namespace i2creg {

inline bool wr(TwoWire& w, uint8_t addr, uint8_t reg, uint8_t v) {
  w.beginTransmission(addr);
  w.write(reg);
  w.write(v);
  return w.endTransmission() == 0;
}

inline bool rd(TwoWire& w, uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t n) {
  w.beginTransmission(addr);
  w.write(reg);
  if (w.endTransmission(false) != 0) return false;
  if (w.requestFrom(addr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = w.read();
  return true;
}

inline bool present(TwoWire& w, uint8_t addr) {
  w.beginTransmission(addr);
  return w.endTransmission() == 0;
}

}  // namespace i2creg

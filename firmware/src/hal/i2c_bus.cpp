#include "hal/hal.h"

namespace hal {

bool i2cBegin() { return Wire.begin(PIN_I2C0_SDA, PIN_I2C0_SCL, I2C_FREQ_HZ); }

TwoWire& bus0() { return Wire; }

void i2cScan(Print& out) {
  // expected: 0x29 forward ToF, 0x30 ground ToF, the Grove accelerometer
  // (0x19 LIS3DHTR, 0x53 ADXL345, 0x18 H3LIS331DL or 0x4C MMA7660FC)
  out.printf("I2C0:");
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      out.printf(" 0x%02X", a);
      n++;
    }
  }
  out.printf("%s\n", n ? "" : " (nothing)");
}

}  // namespace hal

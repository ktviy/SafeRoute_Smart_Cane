// MakeCU kit IMU: a Grove "3-Axis Digital Accelerometer" - accelerometer only, NO gyro
// Seeed sold several boards under that name, so the
// chip is found at boot:
//
//   chip         address      ID register         range used   resolution   notes
//   LIS3DHTR     0x19 / 0x18  WHO_AM_I 0x0F = 0x33  +-8 g      4 mg         current Grove board
//   ADXL345      0x53 / 0x1D  DEVID 0x00 = 0xE5     +-16 g     3.9 mg       older Grove board
//   H3LIS331DL   0x18 / 0x19  WHO_AM_I 0x0F = 0x32  +-100 g    49 mg        "+-400 g" board: coarse
//   MMA7660FC    0x4C         (none)                +-1.5 g    47 mg        6-bit: coarse, clips taps
//
// The gyro is reported as 0 (the logic never uses it). Fixed-angle mode: head-level maths use the
// trained angle, drop maths the accelerometer angle corrected by the ground laser's floor plane;
// the accelerometer also gives the sweep position (sideways acceleration), "cane raised", the tap
// impacts and the walking bounce. The coarse chips (H3LIS331DL, MMA7660FC: ~0.5 m/s2 steps)
// make the sweep position noisier - prefer the LIS3DHTR or ADXL345.
//
// Wiring (Grove cable): red VCC -> 3V3, black GND -> GND, yellow SCL -> GPIO22,
// white SDA -> GPIO21. The Grove board has its own pull-ups.
#include "hal/hal.h"
#include "hal/i2c_regs.h"

namespace hal {

enum class Chip : uint8_t { None, Lis3dh, Adxl345, H3lis331, Mma7660 };
static Chip s_chip = Chip::None;
static uint8_t s_addr = 0;
static float s_g_per_lsb = 0;   // after the chip's own alignment shift

static bool whoAmI8(uint8_t addr, uint8_t reg, uint8_t* id) {
  return i2creg::present(bus0(), addr) && i2creg::rd(bus0(), addr, reg, id, 1);
}

static bool beginLis(uint8_t addr, bool h3lis) {
  TwoWire& w = bus0();
  if (h3lis) {
    // CTRL_REG1: normal mode, 100 Hz, XYZ on; CTRL_REG4: block data update, +-100 g
    if (!i2creg::wr(w, addr, 0x20, 0x2F) || !i2creg::wr(w, addr, 0x23, 0x80)) return false;
    s_g_per_lsb = 0.049f;     // 12-bit left-justified, read >> 4
  } else {
    // CTRL_REG1: 100 Hz, normal power, XYZ on; CTRL_REG4: BDU, +-8 g, high resolution.
    // +-8 g (4 mg) rather than +-16 g (12 mg) - the sweep position comes from
    // ~1 m/s2 of sideways acceleration; tip taps still fit (clipping would not hide them)
    if (!i2creg::wr(w, addr, 0x20, 0x57) || !i2creg::wr(w, addr, 0x23, 0xA8)) return false;
    s_g_per_lsb = 0.004f;     // 12-bit left-justified, read >> 4
  }
  return true;
}

static bool beginMma(uint8_t addr) {
  TwoWire& w = bus0();
  s_g_per_lsb = 1.0f / 21.33f;   // 6-bit, +-1.5 g
  return i2creg::wr(w, addr, 0x07, 0x00)      // MODE: standby while configuring
      && i2creg::wr(w, addr, 0x08, 0x00)      // SR: 120 samples/s
      && i2creg::wr(w, addr, 0x07, 0x01);     // MODE: active
}

bool Imu::begin() {
  s_chip = Chip::None;
  uint8_t id = 0;
  const uint8_t lis_addr[2] = {0x19, 0x18};
  for (uint8_t a : lis_addr) {
    if (!whoAmI8(a, 0x0F, &id)) continue;
    if (id == 0x33 && beginLis(a, false)) { s_chip = Chip::Lis3dh; s_addr = a; break; }
    if (id == 0x32 && beginLis(a, true)) { s_chip = Chip::H3lis331; s_addr = a; break; }
  }
  if (s_chip == Chip::None) {
    const uint8_t adxl_addr[2] = {0x53, 0x1D};
    for (uint8_t a : adxl_addr) {
      if (!whoAmI8(a, 0x00, &id) || id != 0xE5) continue;
      TwoWire& w = bus0();
      if (i2creg::wr(w, a, 0x2D, 0x00) && i2creg::wr(w, a, 0x31, 0x0B) && i2creg::wr(w, a, 0x2C, 0x0A /* BW_RATE 100 Hz */) &&
          i2creg::wr(w, a, 0x38, 0x00) && i2creg::wr(w, a, 0x2D, 0x08)) {
        s_chip = Chip::Adxl345;
        s_addr = a;
        s_g_per_lsb = 0.0039f /* full resolution */;
      }
      break;
    }
  }
  if (s_chip == Chip::None && i2creg::present(bus0(), 0x4C) && beginMma(0x4C)) {
    s_chip = Chip::Mma7660;
    s_addr = 0x4C;
    id = 0x4C;
  }
  whoAmI = s_chip == Chip::None ? 0 : id;
  return s_chip != Chip::None;
}

bool Imu::read(sr::ImuSample& s) {
  s.gyro = sr::Vec3();
  s.ok = false;
  float g[3];
  uint8_t b[6];
  TwoWire& w = bus0();
  switch (s_chip) {
    case Chip::Lis3dh:
    case Chip::H3lis331:
      if (!i2creg::rd(w, s_addr, 0x28 | 0x80, b, 6)) return false;   // OUT_X_L.., auto-increment
      for (int i = 0; i < 3; i++) g[i] = (int16_t)(b[2 * i] | (b[2 * i + 1] << 8)) / 16 * s_g_per_lsb;
      break;
    case Chip::Adxl345:
      if (!i2creg::rd(w, s_addr, 0x32, b, 6)) return false;          // DATAX0..
      for (int i = 0; i < 3; i++) g[i] = (int16_t)(b[2 * i] | (b[2 * i + 1] << 8)) * s_g_per_lsb;
      break;
    case Chip::Mma7660:
      if (!i2creg::rd(w, s_addr, 0x00, b, 3)) return false;
      for (int i = 0; i < 3; i++) {
        if (b[i] & 0x40) return false;                              // alert bit: read during update
        int v = b[i] & 0x3F;
        g[i] = (v >= 32 ? v - 64 : v) * s_g_per_lsb;
      }
      break;
    default:
      return false;
  }
  s.acc = sr::Vec3(g[0], g[1], g[2]) * sr::kG;
  s.ok = true;
  return true;
}

const char* Imu::name() const {
  switch (s_chip) {
    case Chip::Lis3dh: return "LIS3DHTR (accel only)";
    case Chip::Adxl345: return "ADXL345 (accel only)";
    case Chip::H3lis331: return "H3LIS331DL (accel only, coarse)";
    case Chip::Mma7660: return "MMA7660FC (accel only, coarse)";
    default: return "3-axis accel (not found)";
  }
}

}  // namespace hal

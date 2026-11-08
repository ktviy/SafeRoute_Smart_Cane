#include "power/battery.h"

namespace sr {

uint8_t lipoPercent(float v) {
  static const float kV[] = {3.30f, 3.50f, 3.61f, 3.69f, 3.73f, 3.77f, 3.80f, 3.84f,
                             3.87f, 3.91f, 3.95f, 4.02f, 4.08f, 4.15f, 4.20f};
  static const uint8_t kP[] = {0, 3, 5, 10, 15, 25, 35, 50, 55, 65, 70, 80, 85, 95, 100};
  const int n = sizeof(kV) / sizeof(kV[0]);
  if (!(v > kV[0])) return 0;
  if (v >= kV[n - 1]) return 100;
  for (int i = 1; i < n; i++) {
    if (v < kV[i]) {
      float f = (v - kV[i - 1]) / (kV[i] - kV[i - 1]);
      return (uint8_t)(kP[i - 1] + f * (kP[i] - kP[i - 1]) + 0.5f);
    }
  }
  return 100;
}

void BatteryMonitor::configure(const Params& p) {
  p_ = p;
  known_ = critical_ = crit_timing_ = warned_ = false;
}

void BatteryMonitor::update(float volts, uint32_t now) {
  if (!(volts > 2.5f && volts < 4.6f)) return;   // NAN or not connected
  v_ = known_ ? v_ * 0.8f + volts * 0.2f : volts;  // ~5 s smoothing at 1 Hz
  known_ = true;
  pct_ = lipoPercent(v_);
  if (pct_ <= p_.critical_battery_pct) {
    if (!crit_timing_) { crit_timing_ = true; crit_since_ = now; }
    if (now - crit_since_ >= 10000) critical_ = true;   // 10 s: not just a motor dip
  } else {
    crit_timing_ = false;
    critical_ = false;
  }
}

float BatteryMonitor::hoursLeft() const {
  if (!known_ || p_.avg_current_ma == 0) return 0;
  return pct_ / 100.0f * p_.battery_mah / p_.avg_current_ma;
}

uint8_t BatteryMonitor::reportPulses() const {
  if (!known_) return 0;
  if (p_.battery_report_hours) {
    float h = hoursLeft();
    return h >= 9.0f ? 9 : (uint8_t)(h + 0.5f);
  }
  uint8_t n = (uint8_t)((pct_ + 19) / 20);        // 1-20% -> 1 ... 81-100% -> 5
  return n < 1 ? 1 : (n > 5 ? 5 : n);
}

bool BatteryMonitor::lowWarningDue(uint32_t now) {
  if (!low()) { warned_ = false; return false; }
  if (!warned_ || now - last_warn_ >= p_.low_battery_repeat_ms) {
    warned_ = true;
    last_warn_ = now;
    return true;
  }
  return false;
}

}  // namespace sr

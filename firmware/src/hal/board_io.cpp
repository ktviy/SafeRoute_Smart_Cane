// Battery divider and non-volatile storage (NVS).
#include <Preferences.h>
#include "hal/hal.h"

namespace hal {

float readBatteryVolts() {
  if (PIN_BATT_ADC < 0) return NAN;
  uint32_t mv = 0;
  for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(PIN_BATT_ADC);
  float v = mv / 8.0f / 1000.0f * BATT_DIVIDER;
  return v > 2.5f ? v : NAN;          // nothing connected -> unknown
}

namespace store {

static Preferences prefs;
static const char* kNs = "saferoute";

void load(sr::Params& p, bool* calibrated) {
  prefs.begin(kNs, true);
  float L = prefs.getFloat("L", NAN), beta = prefs.getFloat("beta", NAN);
  if (!std::isnan(L) && !std::isnan(beta)) {
    p.tip_to_pod_m = L;
    p.ground_beam_deg = beta;
    if (calibrated) *calibrated = true;
  }
  float sx = prefs.getFloat("sx", NAN), sy = prefs.getFloat("sy", NAN), sz = prefs.getFloat("sz", NAN);
  if (!std::isnan(sx) && !std::isnan(sy) && !std::isnan(sz)) p.shaft = sr::Vec3(sx, sy, sz).normalized();
  p.forward_beam_deg = prefs.getFloat("gamma", p.forward_beam_deg);
  p.head_height_m = prefs.getFloat("head", p.head_height_m);
  p.haptic_plan = (char)prefs.getUChar("plan", (uint8_t)p.haptic_plan);
  p.stride_sweep_m = prefs.getFloat("ssw", p.stride_sweep_m);   // speed calibration walk
  p.stride_tap_m = prefs.getFloat("stp", p.stride_tap_m);
  p.haptic_strength = (uint8_t)prefs.getFloat("hstr", p.haptic_strength);
  prefs.end();
}

void saveCalibration(float L, float beta_deg) {
  prefs.begin(kNs, false);
  prefs.putFloat("L", L);
  prefs.putFloat("beta", beta_deg);
  prefs.end();
}

void saveShaft(const sr::Vec3& s) {
  prefs.begin(kNs, false);
  prefs.putFloat("sx", s.x);
  prefs.putFloat("sy", s.y);
  prefs.putFloat("sz", s.z);
  prefs.end();
}

void saveFloat(const char* key, float v) {
  prefs.begin(kNs, false);
  if (strcmp(key, "plan") == 0) prefs.putUChar("plan", (uint8_t)v);
  else prefs.putFloat(key, v);
  prefs.end();
}

void clear() {
  prefs.begin(kNs, false);
  prefs.clear();
  prefs.end();
}

}  // namespace store
}  // namespace hal

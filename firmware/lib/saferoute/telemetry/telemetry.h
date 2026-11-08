// Telemetry line (JSON, one per loop tick). The first eight keys are the
// agreed format (EE Task conventions) that the dashboard reads:
//   {"t":ms,"a":deg,"yaw":deg,"dg":mm,"df":mm,"dh":mm,"alert":code,"batt":pct,...}
// Extra keys: yo = yaw offset (deg), hf = forward obstacle height (mm),
// m = mode ("s" smart / "n" naive), st = DeviceState, hl/hr = motor levels,
// tech = technique ("-" unknown, "S" still, "W" sweep, "T" tap, "H" hold),
// v = estimated walking speed (m/s), gq = ground-return quality (% of recent ground
// frames with a floor reading; low = dark/shiny floor or sunlight). alert 5 = urgent drop.
// hp/hn/hs = last one-shot haptic cue (Pattern id, count, start counter: the dashboard
// plays the cue when hs changes), pl = haptic plan "A"/"B".
// Values that are not available are written as null.
#pragma once
#include <stddef.h>
#include <stdint.h>
#include "core/types.h"

namespace sr {

struct TelemetryFrame {
  uint32_t t_ms;
  float alpha_deg;
  float yaw_deg;
  float yaw_off_deg;
  int32_t dg_mm;        // ground ToF, -1 = no valid frame this tick
  int32_t df_mm;        // forward ToF, -1 = none
  float dh_m;           // NAN = none
  float fwd_h_m;        // NAN = none
  uint8_t alert;        // Hazard
  int16_t batt_pct;     // -1 = unknown
  Mode mode;
  DeviceState state;
  uint8_t hl, hr;
  Technique tech;
  float speed_mps;      // NAN = unknown
  int16_t ground_q_pct; // -1 = unknown
  uint8_t cue;          // last one-shot haptic cue (Pattern id), see haptic_engine.h
  uint8_t cue_n;        // its count (battery pulses, mode 2/3, test side 1/2/3)
  uint8_t cue_seq;      // changes every time a one-shot cue starts
  char plan;            // haptic plan 'A' / 'B'
  uint16_t head_buzz_ms; // >0: head-level alerts are one continuous buzz this long ("hb")
};

// Returns the number of characters written (without the newline it adds).
int formatTelemetry(char* buf, size_t n, const TelemetryFrame& f, bool with_haptics = true);

}  // namespace sr

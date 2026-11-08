#include "telemetry/telemetry.h"
#include <cmath>
#include <stdio.h>

namespace sr {

static int putInt(char* b, size_t n, const char* key, bool ok, long v) {
  return ok ? snprintf(b, n, ",\"%s\":%ld", key, v) : snprintf(b, n, ",\"%s\":null", key);
}

int formatTelemetry(char* buf, size_t n, const TelemetryFrame& f, bool with_haptics) {
  int k = snprintf(buf, n, "{\"t\":%lu,\"a\":%.1f,\"yaw\":%.1f", (unsigned long)f.t_ms,
                   (double)f.alpha_deg, (double)f.yaw_deg);
#define ROOM (k < (int)n ? n - (size_t)k : 0)
  k += putInt(buf + k, ROOM, "dg", f.dg_mm >= 0, f.dg_mm);
  k += putInt(buf + k, ROOM, "df", f.df_mm >= 0, f.df_mm);
  k += putInt(buf + k, ROOM, "dh", !std::isnan(f.dh_m), std::isnan(f.dh_m) ? 0 : std::lround(f.dh_m * 1000));
  k += snprintf(buf + k, ROOM, ",\"alert\":%u", (unsigned)f.alert);
  k += putInt(buf + k, ROOM, "batt", f.batt_pct >= 0, f.batt_pct);
  k += snprintf(buf + k, ROOM, ",\"yo\":%.1f", (double)f.yaw_off_deg);
  k += putInt(buf + k, ROOM, "hf", !std::isnan(f.fwd_h_m), std::isnan(f.fwd_h_m) ? 0 : std::lround(f.fwd_h_m * 1000));
  k += snprintf(buf + k, ROOM, ",\"m\":\"%c\",\"st\":%u", f.mode == Mode::Naive ? 'n' : 's',
                (unsigned)f.state);
  k += snprintf(buf + k, ROOM, ",\"tech\":\"%c\"", "-SWTH"[(int)f.tech < 5 ? (int)f.tech : 0]);
  k += std::isnan(f.speed_mps) ? snprintf(buf + k, ROOM, ",\"v\":null")
                               : snprintf(buf + k, ROOM, ",\"v\":%.2f", (double)f.speed_mps);
  k += putInt(buf + k, ROOM, "gq", f.ground_q_pct >= 0, f.ground_q_pct);
  k += snprintf(buf + k, ROOM, ",\"hp\":%u,\"hn\":%u,\"hs\":%u,\"pl\":\"%c\"", (unsigned)f.cue, (unsigned)f.cue_n,
                (unsigned)f.cue_seq, f.plan == 'B' ? 'B' : 'A');
  if (f.head_buzz_ms) k += snprintf(buf + k, ROOM, ",\"hb\":%u", (unsigned)f.head_buzz_ms);
  if (with_haptics) k += snprintf(buf + k, ROOM, ",\"hl\":%u,\"hr\":%u", f.hl, f.hr);
  k += snprintf(buf + k, ROOM, "}");
#undef ROOM
  return k < (int)n ? k : (int)n - 1;
}

}  // namespace sr

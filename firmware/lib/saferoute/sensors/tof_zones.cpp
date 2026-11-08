#include "sensors/tof_zones.h"

namespace sr {

ZoneClass classifyZone(const ZoneSample& z) {
  if (z.targets == 0 || z.status == kZoneStatusNoTarget) return ZoneClass::Empty;
  if ((z.status == kZoneStatusValid || z.status == kZoneStatusValidLargePulse) && z.mm > 0)
    return ZoneClass::Ok;
  return ZoneClass::Bad;   // low signal, sigma, consistency or wrap-around failures
}

static uint16_t clampMm(float mm) { return mm < 0.0f ? 0 : mm > 65535.0f ? 65535 : (uint16_t)(mm + 0.5f); }

// zone column / row sign of each central zone (27 28 / 35 36)
static const int8_t kSa[4] = {-1, 1, -1, 1};
static const int8_t kSb[4] = {-1, -1, 1, 1};

ZonePlane planeFromZones(const ZoneSample z[4]) {
  ZonePlane r = {false, NAN, NAN, NAN};
  // 1/d_i = cos_off * (qb + t*sa*q1 + t*sb*q2), q = floor normal / height (sensor frame)
  float s = 0, sa = 0, sb = 0, sab = 0;
  for (int i = 0; i < 4; i++) {
    if (classifyZone(z[i]) != ZoneClass::Ok) return r;
    float inv = 1000.0f / z[i].mm;
    s += inv; sa += kSa[i] * inv; sb += kSb[i] * inv; sab += kSa[i] * kSb[i] * inv;
  }
  float qb = s / (4.0f * kZoneCosOffAxis);
  float q1 = sa / (4.0f * kZoneCosOffAxis * kZoneTanOffAxis);
  float q2 = sb / (4.0f * kZoneCosOffAxis * kZoneTanOffAxis);
  float qn = std::sqrt(qb * qb + q1 * q1 + q2 * q2);
  r.twist = sab / s;
  if (!(qn > 0.0f) || qb <= 0.0f) return r;
  r.h_m = 1.0f / qn;
  r.dep_deg = rad2deg(std::asin(clampf(qb / qn, -1.0f, 1.0f)));
  r.ok = std::fabs(r.twist) <= kZonePlaneMaxTwist;
  return r;
}

RangeReading groundFromZones(const ZoneSample z[4]) {
  bool ok[4];
  int empty = 0;
  for (int i = 0; i < 4; i++) {
    ZoneClass c = classifyZone(z[i]);
    ok[i] = c == ZoneClass::Ok;
    empty += c == ZoneClass::Empty;
  }
  // harmonic mean over every valid diagonal pair (both pairs = all four zones)
  float inv = 0.0f;
  int n = 0;
  if (ok[0] && ok[3]) { inv += 1.0f / z[0].mm + 1.0f / z[3].mm; n += 2; }
  if (ok[1] && ok[2]) { inv += 1.0f / z[1].mm + 1.0f / z[2].mm; n += 2; }
  if (n) {
    RangeReading out(RangeStatus::Ok, clampMm(kZoneCosOffAxis * n / inv));
    ZonePlane pl = planeFromZones(z);
    if (pl.ok) { out.plane_dep_deg = pl.dep_deg; out.plane_h_m = pl.h_m; }
    return out;
  }
  if (empty >= 3) return RangeReading(RangeStatus::NoTarget, 0);
  return RangeReading(RangeStatus::Invalid, 0);
}

RangeReading forwardFromZones(const ZoneSample z[4]) {
  int empty = 0;
  int32_t best = -1;
  for (int i = 0; i < 4; i++) {
    ZoneClass c = classifyZone(z[i]);
    if (c == ZoneClass::Ok && (best < 0 || z[i].mm < best)) best = z[i].mm;
    empty += c == ZoneClass::Empty;
  }
  if (best >= 0) return RangeReading(RangeStatus::Ok, clampMm((float)best));
  return RangeReading(empty >= 3 ? RangeStatus::NoTarget : RangeStatus::Invalid, 0);
}

}  // namespace sr

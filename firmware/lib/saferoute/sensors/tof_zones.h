// VL53L5CX multizone ToF -> one range per beam.
//
// The pod runs each VL53L5CX in 8x8 mode (64 zones over its 45 x 45 deg field of view,
// 5.6 deg per zone, 15 Hz) and reads only the central 2x2 zones (27, 28, 35, 36): four rays
// 2.8 deg off the beam axis in both directions, an 11 deg window - close to the VL53L1X
// ROI the geometry was designed for. These functions turn them into the same RangeReading
// the alert filter already takes, so nothing above the hardware layer changes.
//
// Why 8x8 at 15 Hz and not 4x4 at 30 Hz: the 4x4 central zones are 5.6 deg off axis (a 22 deg
// window), wide enough for the head beam to see people standing beside the path and signs above
// head height. The slower frame rate costs less than the wider window.
//
// Ground beam. The geometry (expected floor range, drop depth, calibration) assumes the
// beam's centre ray, and the beam is grazing (36.9 - 19 = 18 deg below horizontal), so the
// zones read differently on a flat floor: at 37 deg the near zones ~1.22 m, the far ones
// ~1.66 m, the centre 1.40 m. On any plane, 1/range is linear in the ray direction, so the
// harmonic mean of two zones placed symmetrically about the axis (a diagonal pair: [0]+[3]
// or [1]+[2]) is exactly centre range / cos(off-axis angle), whatever the cane angle and
// whichever way the sensor is mounted (a diagonal pair always holds one near and one far
// zone). No IMU input needed, and calibration stays exact. (A plain mean reads 1-2.5 cm
// "too deep" at 37-30 deg.)
//   Edges: the harmonic mean moves little while only the far zones are past a curb, so a
//   drop is confirmed once the near zones cross too: at 37 deg they land ~0.57 m past the
//   tip (the centre ray 0.76 m). Empty zones: the far zones are the weakest returns (most
//   grazing), so two empty zones are not trusted as an edge (Invalid: no alert, counts
//   toward "poor surface"); three or four empty = NoTarget, which the filter's no-return
//   logic handles as before.
//
// Forward (head) beam: any obstacle in the window matters, and a thin bar may fill only
// one zone, so the nearest valid zone wins (what the VL53L1X did with its first peak).
// The beam points 36 deg above horizontal, so no zone meets the floor. The hit height is
// computed on the beam axis, so a hit in a zone is placed ~4 deg off (about +-7 cm at
// 1.5 m); the head band filter absorbs it.
//
// Floor plane (no-gyro pods): on a plane, 1/range of each zone is linear in its ray
// direction, so the four zones give the plane exactly: the beam's angle below it and the
// sensor's height above it (and one spare equation, the "twist", which is ~0 only when all
// four zones see one plane). With a flat floor under the tip this is the cane angle measured
// by the laser itself - what an accelerometer alone cannot give while the cane sweeps.
//
// Pure logic, no hardware: unit-tested on the laptop (test/test_core).
#pragma once
#include <stdint.h>
#include "core/types.h"

namespace sr {

// VL53L5CX per-zone target_status codes (ST UM2884)
constexpr uint8_t kZoneStatusValid = 5;           // 100% valid
constexpr uint8_t kZoneStatusValidLargePulse = 9; // valid, maybe merged targets
constexpr uint8_t kZoneStatusNoTarget = 255;      // nothing detected

// 8x8 mode: the central 2x2 zones (rows 3-4, columns 3-4), in the order the functions
// below expect. Diagonal (point-symmetric) pairs: [0]+[3] and [1]+[2].
constexpr uint8_t kZoneResolution = 64;           // VL53L5CX_RESOLUTION_8X8
constexpr uint8_t kCentreZones[4] = {27, 28, 35, 36};
constexpr float kZoneOffAxisDeg = 2.8125f;        // half a zone (45 / 8 / 2)
// each central zone's ray is kZoneOffAxisDeg off axis in both directions:
// cos(off-axis) = 1 / sqrt(1 + 2 tan^2(2.8125 deg))
constexpr float kZoneCosOffAxis = 0.997595f;

struct ZoneSample {
  int16_t mm;        // distance_mm
  uint8_t status;    // target_status
  uint8_t targets;   // nb_target_detected (0 = nothing)
};

enum class ZoneClass : uint8_t { Ok, Empty, Bad };
ZoneClass classifyZone(const ZoneSample& z);

constexpr float kZoneTanOffAxis = 0.0491268f;     // tan(kZoneOffAxisDeg)
constexpr float kZonePlaneMaxTwist = 0.02f;       // above this the four zones are not one plane

struct ZonePlane {
  bool ok;         // all four zones valid and coplanar
  float dep_deg;   // beam axis angle below the plane
  float h_m;       // sensor height above the plane (perpendicular)
  float twist;     // planarity residual (0 = perfect plane)
};
ZonePlane planeFromZones(const ZoneSample z[4]);

// z: the four central zones in kCentreZones order. The ground reading also carries the
// plane (plane_dep_deg / plane_h_m) when planeFromZones() finds one.
RangeReading groundFromZones(const ZoneSample z[4]);
RangeReading forwardFromZones(const ZoneSample z[4]);

}  // namespace sr

// Beam geometry: ground profile (drop height), forward obstacle height, and the
// one-press calibration solver. All angles in radians unless the name says deg.
//
//   alpha = cane angle below horizontal       L = tip -> pod along the shaft
//   beta  = ground beam above the shaft axis  gamma = forward beam above the shaft axis
//
//   pod height        h  = L sin(alpha)                (tip on the ground)
//   ground beam       dh = L sin(alpha) - d sin(alpha - beta)   (< 0 = drop-off)
//   forward beam      e  = gamma - alpha (elevation)   H = h + d sin(e),  x = d cos(e)
#pragma once
#include <stdint.h>
#include "core/params.h"

namespace sr {

inline float podHeight(float L, float alpha) { return L * std::sin(alpha); }

inline float groundDeltaH(float L, float beta, float alpha, float d) {
  return L * std::sin(alpha) - d * std::sin(alpha - beta);
}

// Range the ground beam should read on flat ground; < 0 if it does not hit the floor.
inline float expectedGroundRange(float L, float beta, float alpha) {
  float s = std::sin(alpha - beta);
  return s > 0.035f ? L * std::sin(alpha) / s : -1.0f;
}

struct ForwardHit {
  float height_m;   // above the ground the tip stands on
  float x_m;        // horizontal distance ahead of the pod
};

inline ForwardHit forwardHit(float L, float gamma, float alpha, float d) {
  float e = gamma - alpha;
  ForwardHit h;
  h.height_m = L * std::sin(alpha) + d * std::sin(e);
  h.x_m = d * std::cos(e);
  return h;
}

enum CalError : uint8_t { CAL_OK = 0, CAL_TOO_FEW = 1, CAL_BAD_L = 2, CAL_BAD_FIT = 3 };

struct CalResult {
  bool ok;
  bool solved_beta;   // true: beta solved from the data; false: beta from config
  float L_m;
  float beta_deg;
  float rms_m;        // flat-floor residual of dh after the fit
  uint8_t error;      // CalError
  uint16_t n;
};

// Calibration with the cane held STILL (tip on a flat floor, ~3 s). At rest
// the accelerometer is an exact inclinometer, so each still frame gives
//   beta = alpha(accelerometer) - dep(floor plane),  L = d sin(alpha - beta) / sin(alpha)
// (d = centre range). Averaged over the frames. (Fitting L and beta from a rocked cane and
// the floor plane alone is biased 2-3 deg: zone noise moves dep and height together.)
// alpha_deg: accelerometer angle, dep_deg: beam angle below the floor plane, d_m: centre range.
CalResult solveStillCalibration(const float* alpha_deg, const float* dep_deg, const float* d_m, int n,
                                const Params& p);

}  // namespace sr

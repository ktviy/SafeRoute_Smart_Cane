#include "kinematics/geometry.h"

namespace sr {

CalResult solveStillCalibration(const float* alpha_deg, const float* dep_deg, const float* d_m, int n,
                                const Params& p) {
  CalResult r = {};
  r.n = (uint16_t)n;
  if (n < (int)p.cal_min_samples) {
    r.error = CAL_TOO_FEW;
    return r;
  }
  double sb = 0;
  for (int i = 0; i < n; i++) sb += alpha_deg[i] - dep_deg[i];
  const float beta = (float)(sb / n);
  double sl = 0;
  for (int i = 0; i < n; i++) {
    float a = deg2rad(alpha_deg[i]);
    sl += d_m[i] * std::sin(a - deg2rad(beta)) / std::sin(a);
  }
  r.L_m = (float)(sl / n);
  r.beta_deg = beta;
  r.solved_beta = true;
  double s = 0;
  for (int i = 0; i < n; i++) {
    float e = groundDeltaH(r.L_m, deg2rad(beta), deg2rad(alpha_deg[i]), d_m[i]);
    s += (double)e * e;
  }
  r.rms_m = (float)std::sqrt(s / n);
  if (r.L_m < p.cal_l_min_m || r.L_m > p.cal_l_max_m) r.error = CAL_BAD_L;
  else if (std::fabs(beta - p.ground_beam_deg) > p.cal_beta_tol_deg) r.error = CAL_BAD_FIT;
  else if (r.rms_m > p.cal_max_rms_m) r.error = CAL_BAD_FIT;
  else { r.error = CAL_OK; r.ok = true; }
  return r;
}

}  // namespace sr

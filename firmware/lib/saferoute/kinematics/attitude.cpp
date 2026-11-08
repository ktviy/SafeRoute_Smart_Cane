#include "kinematics/attitude.h"

namespace sr {

void Attitude::configure(const Params& p) {
  shaft_ = p.shaft.normalized();
  tau_s_ = p.acc_only_tau_s;
  alpha0_ = p.fixed_alpha_deg;
  hw_deg_ = p.sweep_half_width_deg;
  sw_g_tau_ = p.acc_sweep_g_tau_s;
  sw_lp_tau_ = p.acc_sweep_lp_tau_s;
  sw_rms_s_ = p.acc_sweep_rms_s;
  sw_min_ = p.acc_sweep_min_mps2;
  reset();
}

void Attitude::reset() {
  init_ = false;
  run_s_ = 0;
  yaw_off_deg_ = 0;
  off_deg_ = off_anchor_deg_ = 0;
  off_known_ = false;
  sweep_known_ = false;
  sw_g_ = Vec3();
  sw_lp1_ = sw_lp2_ = sw_ms_ = 0;
}

void Attitude::floorFix(float alpha_deg, float gain, bool anchor) {
  float off = alpha_deg - alpha_acc_deg_;
  if (anchor) off_anchor_deg_ = off_known_ ? off_anchor_deg_ + gain * (off - off_anchor_deg_) : off;
  else if (!off_known_) return;                 // continuity frames need an anchor first
  off_deg_ = off_known_ ? off_deg_ + gain * (off - off_deg_) : off;
  off_known_ = true;
  alpha_deg_ = alpha_acc_deg_ + off_deg_;
}

// Sweep position. Pod at the right end of the arc -> it is turning back -> sideways
// acceleration to the left. The sideways axis is shaft x up, with "up" from a slow gravity
// reference (so the sweep's own acceleration does not cancel itself, and any roll of the cane
// in the hand is followed).
void Attitude::sweepFromAccel(const Vec3& acc, float dt) {
  if (sw_g_.norm() == 0.0f) sw_g_ = acc;
  sw_g_ = sw_g_ + (acc - sw_g_) * (dt / (sw_g_tau_ + dt));
  const float gn = sw_g_.norm();
  Vec3 right = gn > 0.5f * kG ? shaft_.cross(sw_g_ * (1.0f / gn)) : Vec3();
  const float rn = right.norm();
  if (rn < 0.3f) {                            // cane near vertical: no sideways axis
    sweep_known_ = false;
    yaw_off_deg_ = 0;
    return;
  }
  const float a_side = (acc - sw_g_).dot(right * (1.0f / rn));   // + = to the right
  const float k = dt / (sw_lp_tau_ + dt);
  sw_lp1_ += k * (a_side - sw_lp1_);
  sw_lp2_ += k * (sw_lp1_ - sw_lp2_);
  const float s = -sw_lp2_;
  sw_ms_ += (s * s - sw_ms_) * (dt / (sw_rms_s_ + dt));
  const float rms = std::sqrt(sw_ms_);
  sweep_known_ = rms >= sw_min_;
  yaw_off_deg_ = sweep_known_ ? hw_deg_ * clampf(s / (1.41421356f * rms), -1.3f, 1.3f) : 0.0f;
}

void Attitude::update(const ImuSample& s, float dt) {
  if (!s.ok) return;
  float an = s.acc.norm();
  if (!init_) {
    if (an < 0.5f * kG || an > 1.5f * kG) return;
    up_ = s.acc * (1.0f / an);
    init_ = true;
  }

  // "up" = accelerometer low-pass. Fast start: trust the accelerometer more right after
  // power-on (the user may already be walking), then settle to the full time constant.
  run_s_ += dt;
  if (an > 0.5f * kG && an < 1.5f * kG) {
    float tau = run_s_ < kSettleS ? tau_s_ * (0.05f + 0.95f * run_s_ / kSettleS) : tau_s_;
    float k = tau / (tau + dt);
    up_ = (up_ * k + s.acc * ((1.0f - k) / an)).normalized();
  }
  alpha_acc_deg_ = rad2deg(std::asin(clampf(-up_.dot(shaft_), -1.0f, 1.0f)));
  alpha_deg_ = alpha_acc_deg_ + off_deg_;

  sweepFromAccel(s.acc, dt);
}

}  // namespace sr

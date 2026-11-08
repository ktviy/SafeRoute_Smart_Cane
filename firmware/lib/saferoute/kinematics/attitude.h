// Cane attitude from the 3-axis accelerometer (no gyro): cane angle and sweep position.
//
// Cane angle (alphaDeg): "up" is the accelerometer low-passed with acc_only_tau_s. Sweeping adds
// 2-7 deg of error to that angle, so floorFix() - the cane angle the ground laser measures on a
// flat floor (pipeline, ~15 Hz) - corrects it through an offset. This measured angle is used for
// the drop maths, the "cane raised" pause and the too-flat check.
// Fixed angle (fixedAlphaDeg): the trained cane angle, used for the head-level maths.
//
// Sweep position (yawOffsetDeg, + = right of the sweep centre): from the sideways acceleration.
// A swept cane turns back at each end of its arc, so at the right end the pod accelerates to
// the left: offset = half-width * (-a_side) / (sqrt2 * RMS(a_side)), limited to +-1.3
// half-widths; 0 while the cane is not swept (sweepKnown() false).
#pragma once
#include "core/params.h"
#include "core/types.h"

namespace sr {

class Attitude {
 public:
  void configure(const Params& p);
  void reset();
  void update(const ImuSample& s, float dt);
  // a trusted cane angle from the floor plane. anchor = the frame also passed the "tip on this
  // floor" check; only those move the anchor angle that the weaker continuity check is measured
  // against (no runaway).
  void floorFix(float alpha_deg, float gain, bool anchor);
  bool floorKnown() const { return off_known_; }
  float anchorAlphaDeg() const { return alpha_acc_deg_ + off_anchor_deg_; }
  float accelAlphaDeg() const { return alpha_acc_deg_; }

  bool ready() const { return init_; }
  // filter converged: safe to raise alerts
  bool settled() const { return init_ && run_s_ >= kSettleS; }
  float alphaDeg() const { return alpha_deg_; }      // measured cane angle below horizontal
  float fixedAlphaDeg() const { return alpha0_; }    // trained cane angle (head-level maths)
  float yawOffsetDeg() const { return yaw_off_deg_; } // + = right of the sweep centre
  bool sweepKnown() const { return sweep_known_; }
  float sweepAccelRms() const { return std::sqrt(sw_ms_); }
  Vec3 up() const { return up_; }

 private:
  static constexpr float kSettleS = 2.0f;   // fast start: tau ramps up over this time
  void sweepFromAccel(const Vec3& acc, float dt);

  Vec3 shaft_;
  float tau_s_ = 0.25f, alpha0_ = 38.0f;
  float off_deg_ = 0, off_anchor_deg_ = 0, alpha_acc_deg_ = 0;
  bool off_known_ = false;

  bool init_ = false;
  Vec3 up_;
  float run_s_ = 0;
  float alpha_deg_ = 0, yaw_off_deg_ = 0;

  bool sweep_known_ = false;
  float hw_deg_ = 23.0f, sw_g_tau_ = 0.8f, sw_lp_tau_ = 0.08f, sw_rms_s_ = 3.0f, sw_min_ = 0.15f;
  Vec3 sw_g_;
  float sw_lp1_ = 0, sw_lp2_ = 0, sw_ms_ = 0;
};

}  // namespace sr

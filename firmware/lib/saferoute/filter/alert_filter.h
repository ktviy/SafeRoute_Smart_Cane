// The alert filter: the pod vibrates only when all three rules pass.
//
//   1. Confirmation  - the hazard is seen in N consecutive ToF frames (3 ground, 2 forward)
//   2. Height band   - ground: drop deeper than 8 cm; forward: 0.9 m < H < head + 0.1 m, x < 1.7 m
//   3. Sweep corridor - the hazard lies within the walking path (sweep centre +- corridor;
//                       forward obstacles also within 0.3 m of the walking line). The sweep
//                       position is the accelerometer estimate: a head-level frame that only
//                       the corridor rejects neither counts nor breaks the run, and the side
//                       (left / right / ahead) is the mean over the confirming frames.
// Head-level maths use the fixed trained cane angle; drops use the measured angle.
//
// Plus: near-vertical pause (alpha > PAUSE_ON_DEG 52 until < PAUSE_OFF_DEG 49), and fail-safe -
// an invalid frame or a sensor fault never raises an alert. A ground beam that
// sees nothing counts as a deep drop (platform edge) only while the floor normally
// answers: on a dark or shiny floor, or in sunlight, empty frames are routine.
// Naive mode (for the smart-vs-naive comparison) alerts on any single frame:
// any ground drop, any forward return closer than 1.5 m, no pause, no corridor.
#pragma once
#include "core/params.h"
#include "core/types.h"

namespace sr {

class AlertFilter {
 public:
  void configure(const Params& p);
  void reset();
  void setMode(Mode m) { mode_ = m; reset(); }
  Mode mode() const { return mode_; }
  // per-technique drop threshold (the pipeline sets it every tick)
  void setDropThreshold(float m) { p_.drop_threshold_m = m; }
  float dropThreshold() const { return p_.drop_threshold_m; }

  // Once per loop tick. ground/forward carry a frame only when the ToF has a new one.
  // tip_down: false while a two-point tap has the tip in the air (gate mode); those
  // ground frames are skipped (they neither count nor break the run).
  // tip_lift_m: estimated tip height above the floor (compensate mode), added to dh.
  // alpha_deg: the measured cane angle (drop maths, pause, too-flat check).
  // ground_alpha_deg: the cane angle measured by this ground frame's own floor plane
  // (NAN = use alpha_deg).
  void update(uint32_t now_ms, bool attitude_ok, float alpha_deg, float yaw_off_deg,
              const RangeReading& ground, const RangeReading& forward, bool tip_down = true,
              float tip_lift_m = 0.0f, float ground_alpha_deg = NAN);

  Hazard hazard() const { return hazard_; }
  bool paused() const { return paused_; }
  bool groundFault() const { return g_fault_; }
  bool forwardFault() const { return f_fault_; }
  // last computed values, NAN when not available (telemetry)
  float lastDeltaH() const { return dh_; }
  float lastForwardHeight() const { return fh_; }
  float lastForwardX() const { return fx_; }
  // share of recent ground frames with no usable return (0-1, ~1 s average)
  float groundEmptyShare() const { return nr_frac_; }
  bool groundSurfacePoor() const { return surface_poor_; }

 private:
  void groundFrame(uint32_t now, bool usable, float alpha, float yaw_off, const RangeReading& r,
                   float tip_lift);
  void forwardFrame(uint32_t now, bool usable, float alpha, float yaw_off, const RangeReading& r);

  Params p_;
  Mode mode_ = Mode::Smart;
  bool paused_ = false;
  bool g_fault_ = true, f_fault_ = true;
  uint32_t g_last_ = 0, f_last_ = 0;
  bool g_seen_ = false, f_seen_ = false;
  uint8_t g_count_ = 0, nr_count_ = 0, f_count_ = 0;
  uint32_t drop_until_ = 0, head_until_ = 0;
  float nr_frac_ = 0;
  uint32_t nr_t_ = 0;
  bool nr_init_ = false, nr_run_ = false, nr_run_ok_ = true, surface_poor_ = false;
  uint8_t ok_streak_ = 3;
  bool drop_on_ = false, head_on_ = false;
  Hazard head_side_ = Hazard::HeadCenter;
  float lat_sum_ = 0;              // sideways distance summed over the run of hits
  Hazard hazard_ = Hazard::None;
  float dh_, fh_, fx_;
};

}  // namespace sr

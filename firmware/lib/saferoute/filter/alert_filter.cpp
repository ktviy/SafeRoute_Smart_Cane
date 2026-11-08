#include "filter/alert_filter.h"
#include "kinematics/geometry.h"

namespace sr {

void AlertFilter::configure(const Params& p) {
  p_ = p;
  reset();
}

void AlertFilter::reset() {
  g_count_ = nr_count_ = f_count_ = 0;
  drop_on_ = head_on_ = false;
  hazard_ = Hazard::None;
  paused_ = false;
  dh_ = fh_ = fx_ = NAN;
  lat_sum_ = 0;
  nr_frac_ = 0;
  nr_init_ = nr_run_ = surface_poor_ = false;
  nr_run_ok_ = true;
  ok_streak_ = 3;
}

static bool before(uint32_t now, uint32_t until) { return (int32_t)(until - now) > 0; }

void AlertFilter::groundFrame(uint32_t now, bool usable, float alpha, float yaw_off,
                              const RangeReading& r, float tip_lift) {
  const bool naive = mode_ == Mode::Naive;
  const float L = p_.tip_to_pod_m, beta = deg2rad(p_.ground_beam_deg);
  bool hit = false, nr_hit = false;
  dh_ = NAN;

  // How often the floor fails to answer (1 s average over ground frames). A platform
  // edge is an unbroken run of empty frames that starts after the floor was answering
  // well; a dark or shiny floor, or sunlight, scatters empty frames all the time. The
  // call is made once, at the first empty frame of a run, and kept until the floor
  // answers again - so a real edge keeps alerting however long the beam hangs over it.
  const bool empty = r.status == RangeStatus::NoTarget || r.status == RangeStatus::Invalid;
  if (empty && !nr_run_) {
    nr_run_ = true;
    nr_run_ok_ = p_.no_return_max_frac >= 1.0f ||       // 1.0 = surface check off
                 (!surface_poor_ && ok_streak_ >= 3);    // an edge: solid floor, then nothing
  } else if (r.status == RangeStatus::Ok) {
    nr_run_ = false;
  }
  if (r.status == RangeStatus::Ok) ok_streak_ = ok_streak_ < 250 ? ok_streak_ + 1 : ok_streak_;
  else if (empty) ok_streak_ = 0;
  if (r.status == RangeStatus::Ok || empty) {
    float dt = nr_init_ ? clampf((now - nr_t_) * 0.001f, 0.0f, 0.2f) : 0.0f;
    nr_frac_ += ((empty ? 1.0f : 0.0f) - nr_frac_) * (dt / (1.0f + dt));
    nr_t_ = now;
    nr_init_ = true;
    // hysteresis: the 1 s average wobbles (40% +- 6% on a 40% floor), so poor until it
    // falls below half the limit
    if (!surface_poor_ && nr_frac_ > p_.no_return_max_frac) surface_poor_ = true;
    else if (surface_poor_ && nr_frac_ < 0.5f * p_.no_return_max_frac) surface_poor_ = false;
  }

  if (r.status == RangeStatus::Ok) {
    dh_ = groundDeltaH(L, beta, alpha, r.mm * 0.001f) + (naive ? 0.0f : tip_lift);
    bool deep = dh_ < -p_.drop_threshold_m && dh_ > -p_.tof_max_range_m;
    hit = naive ? deep : (usable && deep && std::fabs(yaw_off) <= p_.ground_corridor_deg);
  } else if (r.status == RangeStatus::NoTarget && !naive && p_.no_return_is_drop) {
    // the beam should be on the floor and well inside range, yet sees nothing
    float expect = expectedGroundRange(L, beta, alpha);
    nr_hit = usable && nr_run_ok_ && expect > 0 && expect < 0.8f * p_.tof_max_range_m &&
             std::fabs(yaw_off) <= p_.ground_corridor_deg;
  }
  // Invalid frames (and NoTarget in naive mode) break the run: fail-safe
  g_count_ = hit ? (uint8_t)(g_count_ < 250 ? g_count_ + 1 : g_count_) : 0;
  nr_count_ = nr_hit ? (uint8_t)(nr_count_ < 250 ? nr_count_ + 1 : nr_count_) : 0;

  uint8_t need = naive ? 1 : p_.n_confirm_ground;
  if (g_count_ >= need || nr_count_ >= need + p_.no_return_extra) {
    drop_until_ = now + (naive ? p_.naive_hold_ms : p_.alert_hold_ms);
    drop_on_ = true;
  }
}

void AlertFilter::forwardFrame(uint32_t now, bool usable, float alpha, float yaw_off,
                               const RangeReading& r) {
  const bool naive = mode_ == Mode::Naive;
  bool hit = false;
  float lateral = 0;
  fh_ = fx_ = NAN;
  if (r.status == RangeStatus::Ok) {
    float d = r.mm * 0.001f;
    ForwardHit h = forwardHit(p_.tip_to_pod_m, deg2rad(p_.forward_beam_deg), alpha, d);
    fh_ = h.height_m;
    fx_ = h.x_m;
    // sideways distance of the hit from the walking line (+ = right); the sweep
    // pivots at the grip, which is grip_to_pod behind the pod
    lateral = (h.x_m + p_.grip_to_pod_m * std::cos(alpha)) * std::sin(deg2rad(yaw_off));
    if (naive) {
      hit = d < p_.naive_fwd_range_m;
    } else {
      const bool in_band = usable && h.height_m > p_.head_min_m &&
                           h.height_m < p_.head_height_m + p_.head_margin_m && h.x_m < p_.forward_max_x_m;
      const bool in_corr = std::fabs(yaw_off) <= p_.forward_corridor_deg &&
                           std::fabs(lateral) <= p_.forward_corridor_m;
      hit = in_band && in_corr;
      // the sweep position is an estimate that flickers near the corridor edge: a frame that
      // only the corridor rejects neither counts nor breaks the run
      if (in_band && !in_corr) return;
    }
  }
  f_count_ = hit ? (uint8_t)(f_count_ < 250 ? f_count_ + 1 : f_count_) : 0;
  // the sweep position is an estimate (accelerometer): the side is the mean over the whole
  // run of hits, not the last frame alone
  lat_sum_ = hit ? (f_count_ == 1 ? lateral : lat_sum_ + lateral) : 0.0f;
  if (hit) lateral = lat_sum_ / f_count_;
  uint8_t need = naive ? 1 : p_.n_confirm_forward;
  if (f_count_ >= need) {
    head_until_ = now + (naive ? p_.naive_hold_ms : p_.alert_hold_ms);
    head_on_ = true;
    head_side_ = lateral > p_.side_deadband_m ? Hazard::HeadRight
               : lateral < -p_.side_deadband_m ? Hazard::HeadLeft
               : Hazard::HeadCenter;
  }
}

void AlertFilter::update(uint32_t now, bool attitude_ok, float alpha_deg, float yaw_off_deg,
                         const RangeReading& ground, const RangeReading& forward, bool tip_down,
                         float tip_lift_m, float ground_alpha_deg) {
  const bool naive = mode_ == Mode::Naive;
  const float a_meas = alpha_deg;

  // near-vertical pause with hysteresis
  if (!paused_ && a_meas > p_.pause_on_deg) paused_ = true;
  else if (paused_ && a_meas < p_.pause_off_deg) paused_ = false;
  if (!attitude_ok) paused_ = false;

  bool usable = attitude_ok && !paused_ && a_meas >= p_.alpha_min_deg;
  float alpha = deg2rad(alpha_deg);

  // sensor health: a Fault frame or no frame for tof_stale_ms
  if (ground.fresh()) {
    g_seen_ = true;
    g_last_ = now;
    g_fault_ = ground.status == RangeStatus::Fault;
  } else if (!g_seen_ || now - g_last_ > p_.tof_stale_ms) {
    g_fault_ = true;
  }
  if (!p_.forward_enabled) {
    f_fault_ = false;                     // not fitted (cut list): not a fault, no head alerts
  } else if (forward.fresh()) {
    f_seen_ = true;
    f_last_ = now;
    f_fault_ = forward.status == RangeStatus::Fault;
  } else if (!f_seen_ || now - f_last_ > p_.tof_stale_ms) {
    f_fault_ = true;
  }

  if (ground.fresh() && (naive || tip_down))
    groundFrame(now, naive ? attitude_ok : usable, std::isnan(ground_alpha_deg) ? alpha : deg2rad(ground_alpha_deg),
                yaw_off_deg, ground, tip_lift_m);
  // head-level maths with the fixed trained cane angle
  const float fwd_alpha = deg2rad(p_.fixed_alpha_deg);
  if (forward.fresh() && p_.forward_enabled)
    forwardFrame(now, naive ? attitude_ok : usable, fwd_alpha, yaw_off_deg, forward);

  if (g_fault_ || !attitude_ok) { g_count_ = nr_count_ = 0; drop_on_ = false; }
  if (f_fault_ || !attitude_ok) { f_count_ = 0; head_on_ = false; }
  if (paused_ && !naive) { drop_on_ = head_on_ = false; g_count_ = nr_count_ = f_count_ = 0; }
  if (drop_on_ && !before(now, drop_until_)) drop_on_ = false;
  if (head_on_ && !before(now, head_until_)) head_on_ = false;

  hazard_ = drop_on_ ? Hazard::Drop : head_on_ ? head_side_ : Hazard::None;
}

}  // namespace sr

#include "kinematics/gait.h"

namespace sr {

int Gait::Ring::countSince(uint32_t since) const {
  int c = 0;
  for (int k = 0; k < n; k++)
    if ((int32_t)(t[k] - since) >= 0) c++;
  return c;
}

bool Gait::Ring::span(uint32_t since, uint32_t* first, uint32_t* last, int* count) const {
  int c = 0;
  for (int k = 0; k < n; k++) {
    uint32_t x = t[k];
    if ((int32_t)(x - since) < 0) continue;
    if (!c || (int32_t)(x - *first) < 0) *first = x;
    if (!c || (int32_t)(x - *last) > 0) *last = x;
    c++;
  }
  *count = c;
  return c > 0;
}

void Gait::configure(const Params& p) {
  p_ = p;
  stride_sweep_ = p.stride_sweep_m;
  stride_tap_ = p.stride_tap_m;
  reset();
}

void Gait::reset() {
  init_ = walking_ = cal_on_ = false;
  yaw_ms_ = yaw_rms_ = lp_ = slow_ = bounce_ms_ = bounce_ = rate_ = 0;
  yaw_ = ext_ = 0;
  ext_t_ = 0;
  dir_ = 1;
  arcs_ = Ring();
  taps_ = Ring();
  tech_ = cand_ = Technique::Unknown;
  speed_ = NAN;
}

void Gait::setStride(Technique t, float m) {
  if (!(m > 0.2f && m < 2.0f)) return;
  if (t == Technique::Tap) stride_tap_ = m;
  else stride_sweep_ = m;
}

static float ema(float y, float x, float dt, float tau) { return y + (x - y) * (dt / (tau + dt)); }

bool Gait::tapping() const {
  if (taps_.n < 3) return false;
  uint32_t t0 = taps_.back(2), t1 = taps_.back(1), t2 = taps_.back(0);
  uint32_t i1 = t1 - t0, i2 = t2 - t1;
  if (i1 < p_.tap_min_interval_ms || i1 > p_.tap_max_interval_ms) return false;
  if (i2 < p_.tap_min_interval_ms || i2 > p_.tap_max_interval_ms) return false;
  return true;    // no gyro: rhythm alone (sweep turning points unknown)
}

Technique Gait::classify(uint32_t now) const {
  if (taps_.n >= 3 && now - taps_.back(0) < (uint32_t)p_.tap_max_interval_ms + 200u && tapping()) return Technique::Tap;
  if (yaw_rms_ > p_.sweep_yaw_rms_dps) return Technique::Sweep;
  if (walking_) return Technique::Hold;
  return Technique::Still;
}

void Gait::update(uint32_t now, float dt, float acc_mag, float yaw_rate_dps, bool impact) {
  if (!init_) {
    lp_ = slow_ = acc_mag;
    init_ = true;
    cand_since_ = now;
  }
  // sweep energy and step bounce (impact spikes filtered out by the 0.15 s low-pass)
  yaw_ms_ = ema(yaw_ms_, yaw_rate_dps * yaw_rate_dps, dt, 2.0f);
  yaw_rms_ = std::sqrt(yaw_ms_);
  // step bounce: clip tip-impact spikes (they are not steps), light low-pass, then the
  // RMS around the 2 s mean. Walking at 1.8 steps/s keeps ~87% of its bounce.
  float clipped = clampf(acc_mag, slow_ - 3.0f, slow_ + 3.0f);
  lp_ = ema(lp_, clipped, dt, 0.05f);
  slow_ = ema(slow_, lp_, dt, 2.0f);
  bounce_ms_ = ema(bounce_ms_, (lp_ - slow_) * (lp_ - slow_), dt, 2.0f);
  bounce_ = std::sqrt(bounce_ms_);
  if (!walking_ && bounce_ > p_.walk_bounce_on) walking_ = true;
  else if (walking_ && bounce_ < p_.walk_bounce_off) walking_ = false;

  // events: a sweep arc ends at a turning point, once the yaw angle has come back
  // arc_turn_deg from its extreme (counting rate sign flips overcounts: the yaw rate
  // wobbles inside one arc). The arc is stamped at the extreme itself.
  yaw_ += yaw_rate_dps * dt;
  if (dir_ > 0) {
    if (yaw_ > ext_) { ext_ = yaw_; ext_t_ = now; }
    else if (yaw_ < ext_ - p_.arc_turn_deg) {
      arcs_.push(ext_t_);
      if (cal_on_) cal_arcs_++;
      dir_ = -1; ext_ = yaw_; ext_t_ = now;
    }
  } else {
    if (yaw_ < ext_) { ext_ = yaw_; ext_t_ = now; }
    else if (yaw_ > ext_ + p_.arc_turn_deg) {
      arcs_.push(ext_t_);
      if (cal_on_) cal_arcs_++;
      dir_ = 1; ext_ = yaw_; ext_t_ = now;
    }
  }
  if (yaw_ > 1e4f || yaw_ < -1e4f) { ext_ -= yaw_; yaw_ = 0; }
  if (impact) {
    taps_.push(now);
    if (cal_on_) cal_taps_++;
  }

  // technique with hysteresis
  Technique c = classify(now);
  if (c != cand_) { cand_ = c; cand_since_ = now; }
  if (cand_ != tech_ && (now - cand_since_ >= p_.technique_hold_ms || tech_ == Technique::Unknown)) tech_ = cand_;

  // speed from the event rate of the current technique
  float stride = 0;
  const Ring* r = nullptr;
  if (tech_ == Technique::Tap) { r = &taps_; stride = stride_tap_; }
  else if (tech_ == Technique::Sweep) { r = &arcs_; stride = stride_sweep_; }
  float v = NAN;
  rate_ = 0;
  if (!walking_ || tech_ == Technique::Still) {
    v = 0;
  } else if (r) {
    uint32_t first = 0, last = 0;
    int n = 0;
    r->span(now - p_.speed_window_ms, &first, &last, &n);
    if (n >= 3 && last != first) {
      rate_ = (n - 1) * 1000.0f / (last - first);
      v = rate_ * stride;
    }
  }
  if (std::isnan(v)) speed_ = NAN;
  else speed_ = std::isnan(speed_) ? v : ema(speed_, v, dt, 1.5f);
}

void Gait::startCalWalk() {
  cal_on_ = true;
  cal_arcs_ = cal_taps_ = 0;
}

bool Gait::finishCalWalk(float distance_m, Technique* tech_out, float* stride_out) {
  if (!cal_on_) return false;
  cal_on_ = false;
  // the technique with more events during the walk is the one that was used
  bool tap = cal_taps_ > cal_arcs_;
  int n = tap ? cal_taps_ : cal_arcs_;
  cal_taps_ = cal_arcs_ = 0;
  if (n < 5 || !(distance_m > 1.0f)) return false;
  Technique t = tap ? Technique::Tap : Technique::Sweep;
  float s = distance_m / n;
  if (!(s > 0.2f && s < 2.0f)) return false;
  setStride(t, s);
  if (tech_out) *tech_out = t;
  if (stride_out) *stride_out = s;
  return true;
}

}  // namespace sr

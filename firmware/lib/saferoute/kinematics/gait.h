// How the user is using the cane, and how fast they walk - from the IMU alone.
//
// Technique (with TECHNIQUE_HOLD_MS hysteresis so it doesn't flicker):
//   Tap   the last 3 tip impacts are rhythmic (0.5-1.8 s apart) and land on sweep
//         turning points (two-point touch); random bumps while sweeping don't qualify
//   Sweep yaw-rate RMS over 2 s above 20 deg/s (constant contact, or sweeping while standing)
//   Hold  walking (step bounce) without sweeping or tapping - diagonal cane
//   Still none of the above
//
// Walking speed: a cane IMU cannot integrate speed (drifts in seconds), but every
// sweep arc and every tap is one step, so
//   speed = (arcs or taps per second) x metres per arc
// with metres per arc per technique (P1 defaults; one calibration walk per user).
// Speed is 0 when not walking and NAN when unknown (Hold: no arcs to count).
#pragma once
#include "core/params.h"
#include "core/types.h"

namespace sr {

class Gait {
 public:
  void configure(const Params& p);
  void reset();
  // Once per tick: |acc| (m/s2), yaw rate (deg/s, + right), impact = a tip impact this tick.
  void update(uint32_t now, float dt, float acc_mag, float yaw_rate_dps, bool impact);

  Technique technique() const { return tech_; }
  bool walking() const { return walking_; }
  float speedMps() const { return speed_; }      // NAN = unknown
  float eventRateHz() const { return rate_; }    // arcs or taps per second
  float yawRms() const { return yaw_rms_; }
  float bounce() const { return bounce_; }
  float stride(Technique t) const { return t == Technique::Tap ? stride_tap_ : stride_sweep_; }
  void setStride(Technique t, float m);

  // Calibration walk: start, walk a measured distance, finish with the distance.
  void startCalWalk();
  bool calWalking() const { return cal_on_; }
  // Returns false if too few arcs/taps were counted; on success sets that technique's stride.
  bool finishCalWalk(float distance_m, Technique* tech_out, float* stride_out);

 private:
  static const int kRing = 24;
  struct Ring {
    uint32_t t[kRing];
    int n = 0, i = 0;
    void push(uint32_t x) { t[i] = x; i = (i + 1) % kRing; if (n < kRing) n++; }
    int countSince(uint32_t since) const;
    bool span(uint32_t since, uint32_t* first, uint32_t* last, int* count) const;
    uint32_t back(int k) const { return t[(i - 1 - k + kRing) % kRing]; }   // k-th newest
  };
  Technique classify(uint32_t now) const;

  Params p_;
  float stride_sweep_ = 0.83f, stride_tap_ = 0.89f;
  float yaw_ms_ = 0, yaw_rms_ = 0;
  float lp_ = 0, slow_ = 0, bounce_ms_ = 0, bounce_ = 0;
  bool init_ = false, walking_ = false;
  float yaw_ = 0, ext_ = 0;
  uint32_t ext_t_ = 0;
  int dir_ = 1;
  bool tapping() const;
  Ring arcs_, taps_;
  Technique tech_ = Technique::Unknown, cand_ = Technique::Unknown;
  uint32_t cand_since_ = 0;
  float rate_ = 0, speed_ = NAN;
  bool cal_on_ = false;
  int cal_arcs_ = 0, cal_taps_ = 0;
};

}  // namespace sr

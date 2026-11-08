// Haptic language: a small state machine that turns hazards and UI events into
// left/right motor intensities (0-255) over time, played as PWM on the two DC motors.
//
// Patterns (Plan A = left/right; Plan B = rhythm only, both motors):
//   Drop         3 fast strong pulses, both motors, repeats while the drop is seen
//   DropUrgent   rapid strong pulses: drop and little time left at this walking speed
//   Head L/R/C   2 medium pulses on that side (Plan B: slow long buzz, both)
//   Battery      N pulses = battery level (20% steps, or hours left)
//   LowBattery   3 soft long pulses, repeated every 5 min
//   Ack / Error  calibration OK (left-then-right sweep) / failed (one long buzz)
//   SlowDown     2 soft long pulses: walking faster than the look-ahead covers
//   PodBlind     1 soft 600 ms pulse: a sensor is down, the pod is not protecting you
//   + Mode (2 = smart, 3 = naive), Boot tick, Test (haptic gate test)
#pragma once
#include "core/params.h"
#include "core/types.h"

namespace sr {

enum class Pattern : uint8_t {
  None, Drop, HeadLeft, HeadRight, HeadCenter,
  Battery, LowBattery, Ack, Error, Boot, Mode, Test,
  DropUrgent, SlowDown, PodBlind,
};

struct HapticStep {
  uint8_t left, right;
  uint16_t ms;
};

class HapticEngine {
 public:
  void configure(const Params& p);
  void setPlan(char plan) { plan_ = plan == 'B' ? 'B' : 'A'; }
  // 0 soft, 1 normal, 2 strong: scales the levels, never the rhythms
  void setStrength(uint8_t s);
  uint8_t strength() const { return strength_; }
  char plan() const { return plan_; }

  // Continuous alert: plays (and repeats) the hazard's pattern until the hazard
  // changes. Preempts one-shots; an interrupted one-shot replays afterwards.
  void setHazard(Hazard h, uint32_t now);
  // One-shot pattern. count: pulses for Battery/Mode, side for Test (1 L, 2 R, 3 both).
  void play(Pattern p, uint32_t now, uint8_t count = 1);
  void tick(uint32_t now);
  void stop();

  uint8_t left() const { return left_; }
  uint8_t right() const { return right_; }
  Pattern current() const { return cur_; }
  bool busy() const { return cur_ != Pattern::None || queued_ != Pattern::None; }
  // Last one-shot cue that STARTED playing (not hazards): pattern, its count, and a
  // counter that changes on every start, so the dashboard can play the same cue.
  Pattern lastCue() const { return cue_; }
  uint8_t lastCueCount() const { return cue_count_; }
  uint8_t cueSeq() const { return cue_seq_; }

 private:
  static const int kMaxSteps = 32;
  void load(Pattern p, uint8_t count, uint32_t now);
  void add(uint8_t l, uint8_t r, uint16_t ms);
  void apply();
  static Pattern hazardPattern(Hazard h);

  uint8_t strong_ = 255, medium_ = 200, soft_ = 130;
  uint8_t base_strong_ = 255, base_medium_ = 200, base_soft_ = 130, strength_ = 1;
  char plan_ = 'A';
  uint16_t head_buzz_ms_ = 0;
  uint8_t head_buzz_level_ = 255;
  HapticStep steps_[kMaxSteps];
  uint8_t n_ = 0, idx_ = 0;
  uint32_t step_start_ = 0;
  bool repeat_ = false;
  Pattern cur_ = Pattern::None;
  uint8_t cur_count_ = 0;
  Hazard hazard_ = Hazard::None;
  Pattern queued_ = Pattern::None;
  uint8_t queued_count_ = 0;
  uint8_t left_ = 0, right_ = 0;
  Pattern cue_ = Pattern::None;
  uint8_t cue_count_ = 0, cue_seq_ = 0;
};

}  // namespace sr

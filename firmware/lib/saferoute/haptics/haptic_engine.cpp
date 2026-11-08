#include "haptics/haptic_engine.h"

namespace sr {

void HapticEngine::configure(const Params& p) {
  base_strong_ = p.haptic_strong;
  base_medium_ = p.haptic_medium;
  base_soft_ = p.haptic_soft;
  head_buzz_ms_ = p.head_buzz_ms;
  head_buzz_level_ = p.head_buzz_level;
  setStrength(p.haptic_strength);
  setPlan(p.haptic_plan);
  stop();
}

static uint8_t scaled(uint8_t v, float k) {
  float x = v * k;
  return (uint8_t)(x > 255.0f ? 255.0f : x);
}

void HapticEngine::setStrength(uint8_t s) {
  strength_ = s > 2 ? 2 : s;
  // soft: 25% weaker; strong: 30% stronger (capped). Strong safety pulses stay well above soft cues.
  const float k = strength_ == 0 ? 0.75f : strength_ == 2 ? 1.3f : 1.0f;
  strong_ = scaled(base_strong_, strength_ == 0 ? 0.8f : 1.0f);
  medium_ = scaled(base_medium_, k);
  soft_ = scaled(base_soft_, k);
}

void HapticEngine::stop() {
  n_ = idx_ = 0;
  cur_ = queued_ = Pattern::None;
  hazard_ = Hazard::None;
  repeat_ = false;
  left_ = right_ = 0;
}

void HapticEngine::add(uint8_t l, uint8_t r, uint16_t ms) {
  if (n_ < kMaxSteps) steps_[n_++] = HapticStep{l, r, ms};
}

Pattern HapticEngine::hazardPattern(Hazard h) {
  switch (h) {
    case Hazard::Drop: return Pattern::Drop;
    case Hazard::DropUrgent: return Pattern::DropUrgent;
    case Hazard::HeadLeft: return Pattern::HeadLeft;
    case Hazard::HeadRight: return Pattern::HeadRight;
    case Hazard::HeadCenter: return Pattern::HeadCenter;
    default: return Pattern::None;
  }
}

void HapticEngine::load(Pattern p, uint8_t count, uint32_t now) {
  n_ = idx_ = 0;
  repeat_ = false;
  const uint8_t S = strong_, M = medium_, s = soft_;
  const bool A = plan_ == 'A';
  switch (p) {
    case Pattern::Drop:
      for (int i = 0; i < 3; i++) { add(S, S, 80); add(0, 0, 60); }
      steps_[n_ - 1].ms = 240;
      repeat_ = true;
      break;
    case Pattern::DropUrgent:
      for (int i = 0; i < 4; i++) { add(S, S, 60); add(0, 0, 40); }
      steps_[n_ - 1].ms = 120;
      repeat_ = true;
      break;
    case Pattern::SlowDown:
      for (int i = 0; i < 2; i++) { add(s, s, 400); add(0, 0, 300); }
      break;
    case Pattern::PodBlind:
      add(s, s, 600); add(0, 0, 300);
      break;
    case Pattern::HeadLeft:
    case Pattern::HeadRight:
    case Pattern::HeadCenter: {
      if (head_buzz_ms_ > 0) {             // one continuous buzz, then quiet while the hazard stays
        const uint8_t L = head_buzz_level_;
        add(p == Pattern::HeadRight ? 0 : L, p == Pattern::HeadLeft ? 0 : L, head_buzz_ms_);
        break;
      }
      if (A) {
        uint8_t l = p == Pattern::HeadRight ? 0 : M, r = p == Pattern::HeadLeft ? 0 : M;
        add(l, r, 250); add(0, 0, 120); add(l, r, 250); add(0, 0, 380);
      } else {
        add(M, M, 500); add(0, 0, 500);
      }
      repeat_ = true;
      break;
    }
    case Pattern::Battery:
      if (count == 0) { add(M, M, 900); }        // empty / unknown: one long buzz
      for (int i = 0; i < count && i < 10; i++) { add(M, M, 150); add(0, 0, 250); }
      add(0, 0, 300);
      break;
    case Pattern::LowBattery:
      for (int i = 0; i < 3; i++) { add(s, s, 500); add(0, 0, 400); }
      break;
    case Pattern::Ack:
      if (A) { add(M, 0, 100); add(0, 0, 60); add(0, M, 100); }
      else { add(M, M, 80); add(0, 0, 80); add(M, M, 80); }
      add(0, 0, 200);
      break;
    case Pattern::Error:
      add(M, M, 700); add(0, 0, 300);
      break;
    case Pattern::Boot:
      add(s, s, 150); add(0, 0, 100);
      break;
    case Pattern::Mode:
      for (int i = 0; i < count && i < 5; i++) { add(M, M, 80); add(0, 0, 100); }
      break;
    case Pattern::Test:
      add(count & 1 ? S : 0, count & 2 ? S : 0, 300); add(0, 0, 50);
      break;
    default:
      break;
  }
  cur_ = n_ ? p : Pattern::None;
  cur_count_ = count;
  if (cur_ != Pattern::None && !repeat_ && p != hazardPattern(hazard_)) {   // one-shot cue started
    cue_ = p;
    cue_count_ = count;
    cue_seq_++;
  }
  step_start_ = now;
  apply();
}

void HapticEngine::apply() {
  if (cur_ != Pattern::None && idx_ < n_) {
    left_ = steps_[idx_].left;
    right_ = steps_[idx_].right;
  } else {
    left_ = right_ = 0;
  }
}

void HapticEngine::setHazard(Hazard h, uint32_t now) {
  if (h == hazard_) return;
  bool cur_is_hazard = cur_ == hazardPattern(hazard_) && cur_ != Pattern::None;
  hazard_ = h;
  if (h != Hazard::None) {
    if (!cur_is_hazard && cur_ != Pattern::None && queued_ == Pattern::None) {
      queued_ = cur_;                      // replay the interrupted one-shot later
      queued_count_ = cur_count_;
    }
    load(hazardPattern(h), 1, now);        // starts this tick: no added latency
  } else if (cur_is_hazard) {
    cur_ = Pattern::None;
    n_ = idx_ = 0;
    apply();
    if (queued_ != Pattern::None) {
      Pattern q = queued_;
      queued_ = Pattern::None;
      load(q, queued_count_, now);
    }
  } else if (cur_ == Pattern::None && queued_ != Pattern::None) {   // a one-shot hazard buzz ended earlier
    Pattern q = queued_;
    queued_ = Pattern::None;
    load(q, queued_count_, now);
  }
}

void HapticEngine::play(Pattern p, uint32_t now, uint8_t count) {
  if (hazard_ != Hazard::None || cur_ != Pattern::None) {
    queued_ = p;                           // latest request wins the single queue slot
    queued_count_ = count;
    return;
  }
  load(p, count, now);
}

void HapticEngine::tick(uint32_t now) {
  while (cur_ != Pattern::None && now - step_start_ >= steps_[idx_].ms) {
    step_start_ += steps_[idx_].ms;
    if (++idx_ >= n_) {
      if (repeat_) {
        idx_ = 0;
      } else {
        cur_ = Pattern::None;
        n_ = idx_ = 0;
        if (queued_ != Pattern::None && hazard_ == Hazard::None) {
          Pattern q = queued_;
          queued_ = Pattern::None;
          load(q, queued_count_, now);
          return;
        }
      }
    }
  }
  apply();
}

}  // namespace sr

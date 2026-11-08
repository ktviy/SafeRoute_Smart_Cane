// Vibration motors (coin ERM, or a small DC motor with an off-centre weight on its shaft)
// driven by PWM through an NPN/N-MOSFET each, with a flyback diode across each motor.
// 20 kHz so the drive itself is silent.
//
// Kick: a motor starting from rest needs time to spin up (a DC motor with a weight ~50-100
// ms), which smears the short pulses of the haptic language. Each start gets
// HAPTIC_PWM_KICK_MS at HAPTIC_PWM_KICK_LEVEL, then the pattern's level.
//
// Stagger: two DC motors starting from rest at the same instant (head-level AHEAD, drop knocks)
// draw their stall current together and pull the 5 V supply down far enough to brown out the
// ESP32 (it resets and boots OFF: no buzz at all). So the right motor plays the pattern
// HAPTIC_STAGGER_MS late: every pulse keeps its full length (a drop knock is only 80 ms), and the
// left motor is always past its start-up current when the right one starts.
#include "hal/hal.h"

namespace hal {

static const uint32_t kFreq = 20000;
static const uint8_t kBits = 8;

static void pwmWrite(int pin, int ch, uint8_t v) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  ledcWrite(pin, v);
#else
  (void)pin;
  ledcWrite(ch, v);
#endif
}

bool Haptics::begin() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  leftOk = ledcAttach(PIN_ERM_LEFT, kFreq, kBits);
  rightOk = ledcAttach(PIN_ERM_RIGHT, kFreq, kBits);
#else
  ledcSetup(0, kFreq, kBits);
  ledcAttachPin(PIN_ERM_LEFT, 0);
  ledcSetup(1, kFreq, kBits);
  ledcAttachPin(PIN_ERM_RIGHT, 1);
  leftOk = rightOk = true;
#endif
  sleep();
  return leftOk && rightOk;
}

// level actually written, given the pattern's level and when this motor last started
static uint8_t kicked(uint8_t want, int16_t last, uint32_t& kick_until, uint32_t now) {
  if (want == 0) return 0;
  if (last <= 0 && HAPTIC_PWM_KICK_MS > 0) kick_until = now + HAPTIC_PWM_KICK_MS;   // starting from rest
  return (int32_t)(kick_until - now) > 0 && want < HAPTIC_PWM_KICK_LEVEL ? HAPTIC_PWM_KICK_LEVEL : want;
}

// the right motor's wanted level HAPTIC_STAGGER_MS ago (0 before there is any history)
uint8_t Haptics::delayedRight(uint8_t r, uint32_t now) {
  if (HAPTIC_STAGGER_MS <= 0) return r;
  hist_t_[hist_i_] = now;
  hist_v_[hist_i_] = r;
  hist_i_ = (uint8_t)((hist_i_ + 1) % kHist);
  if (hist_n_ < kHist) hist_n_++;
  for (uint8_t k = 1; k <= hist_n_; k++) {      // newest first
    uint8_t j = (uint8_t)((hist_i_ + kHist - k) % kHist);
    if (now - hist_t_[j] >= (uint32_t)HAPTIC_STAGGER_MS) return hist_v_[j];
  }
  return 0;
}

void Haptics::set(uint8_t l, uint8_t r) {
  const uint32_t now = millis();
  r = delayedRight(r, now);
  uint8_t vl = kicked(l, last_l_, kick_l_, now), vr = kicked(r, last_r_, kick_r_, now);
  last_l_ = l;
  last_r_ = r;
  if (vl != out_l_) { pwmWrite(PIN_ERM_LEFT, 0, vl); out_l_ = vl; }   // write only on change
  if (vr != out_r_) { pwmWrite(PIN_ERM_RIGHT, 1, vr); out_r_ = vr; }
}

// both motors off NOW (pod off, a blocking sensor reload, power down): no delayed tail
void Haptics::sleep() {
  hist_n_ = hist_i_ = 0;
  last_l_ = last_r_ = 0;
  pwmWrite(PIN_ERM_LEFT, 0, 0);
  pwmWrite(PIN_ERM_RIGHT, 1, 0);
  out_l_ = out_r_ = 0;
}

const char* Haptics::name() const { return "PWM motors x2"; }

}  // namespace hal

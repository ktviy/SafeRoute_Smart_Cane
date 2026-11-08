// Haptic gate test firmware (EE Task F3; General Task "haptic transmission test").
// Build: pio run -e haptic_test_pwm -t upload   (the ELEGOO motors on GPIO18 / GPIO19)
//
// The operator types 'n' to start a trial. The pod
// plays a random stimulus; the tester (looking away, music in ears) says what
// they felt; the operator types it. Every line is CSV, paste it into a sheet.
//
//   mode side    : left / right / both, 3 pulses of 200 ms   -> "side correct >= 85%"
//   mode detect  : vibration on or off (50/50)               -> "felt >= 95%"
//   mode pattern : the real Plan A head-level patterns, 1.5 s
// Answers: l = left, r = right, b = both, n = nothing.   Other: mode <m>, level <0-255>, score
#include <Arduino.h>
#include <esp_random.h>
#include "config.h"
#include "core/params.h"
#include "haptics/haptic_engine.h"
#include "hal/hal.h"

static hal::Haptics motors;
static sr::HapticEngine engine;
static enum { SIDE, DETECT, PATTERN } mode = SIDE;
static uint8_t level = HAPTIC_STRONG;
static int trial = 0, answered = 0, correct = 0;
static char truth = 0;              // 'l', 'r', 'b' or 'n'
static uint32_t stim_end = 0;
static bool waiting = false;

static void stimulus(char side, uint32_t now) {
  if (mode == PATTERN) {
    sr::Hazard h = side == 'l' ? sr::Hazard::HeadLeft : side == 'r' ? sr::Hazard::HeadRight
                 : side == 'b' ? sr::Hazard::HeadCenter : sr::Hazard::None;
    engine.setHazard(h, now);
    stim_end = now + 1500;
    return;
  }
  stim_end = now + 1000;
  if (side == 'n') return;
  uint8_t mask = side == 'l' ? 1 : side == 'r' ? 2 : 3;
  for (int i = 0; i < 3; i++) {     // 3 x 200 ms on, 150 ms off
    motors.set(mask & 1 ? level : 0, mask & 2 ? level : 0);
    delay(200);
    motors.set(0, 0);
    delay(150);
  }
}

static void startTrial() {
  uint32_t r = esp_random();
  if (mode == DETECT) truth = (r & 1) ? 'b' : 'n';
  else truth = "lrb"[r % 3];
  trial++;
  waiting = true;
  Serial.printf("trial,%d,%s,%lu\n", trial, mode == SIDE ? "side" : mode == DETECT ? "detect" : "pattern",
                (unsigned long)millis());
  stimulus(truth, millis());
}

static void answer(char a) {
  if (!waiting) { Serial.println("# no trial running: type n"); return; }
  bool ok = mode == DETECT ? ((a != 'n') == (truth != 'n')) : a == truth;
  answered++;
  correct += ok;
  waiting = false;
  Serial.printf("answer,%d,truth=%c,answer=%c,%s,score=%d/%d (%.0f%%)\n", trial, truth, a, ok ? "OK" : "WRONG",
                correct, answered, 100.0f * correct / answered);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  hal::i2cBegin();
  bool ok = motors.begin();
  engine.configure(sr::Params::defaults());
  engine.setPlan('A');
  Serial.printf("\n# SafeRoute haptic gate test  backend=%s left=%d right=%d\n", motors.name(), motors.leftOk,
                motors.rightOk);
  if (!ok) Serial.println("# WARNING: a motor channel did not start - check the PWM pins (GPIO18 left, GPIO19 right)");
  Serial.println("# 'n' = next trial; answers l/r/b/n; 'mode side|detect|pattern'; 'level 0-255'; 'score'");
}

void loop() {
  static char buf[32];
  static size_t n = 0;
  uint32_t now = millis();

  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') { if (n < sizeof(buf) - 1) buf[n++] = c; continue; }
    if (!n) continue;
    buf[n] = 0;
    n = 0;
    if (!strncmp(buf, "mode ", 5)) {
      mode = buf[5] == 'd' ? DETECT : buf[5] == 'p' ? PATTERN : SIDE;
      trial = answered = correct = 0;
      Serial.printf("# mode %s, score reset\n", mode == SIDE ? "side" : mode == DETECT ? "detect" : "pattern");
    } else if (!strncmp(buf, "level ", 6)) {
      level = (uint8_t)constrain(atoi(buf + 6), 0, 255);
      Serial.printf("# level %u\n", level);
    } else if (!strcmp(buf, "score")) {
      Serial.printf("# score %d/%d\n", correct, answered);
    } else if (strlen(buf) == 1 && strchr("lrbn", buf[0])) {
      if (buf[0] == 'n' && !waiting) startTrial();
      else answer(buf[0]);
    }
  }

  if (mode == PATTERN) {
    if (stim_end && (int32_t)(now - stim_end) >= 0) { engine.setHazard(sr::Hazard::None, now); stim_end = 0; }
    engine.tick(now);
    motors.set(engine.left(), engine.right());
  }
  delay(5);
}

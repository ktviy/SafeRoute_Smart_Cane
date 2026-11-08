// SafeRoute Micro Pod firmware - ESP32 main loop.
//
// Fixed-angle build: 3-axis accelerometer (no gyro), sweep position from the sideways
// acceleration, head-level maths at the fixed trained cane angle (config.h section 5).
// 50 Hz loop: read IMU -> poll both ToFs (15 Hz, non-blocking) -> pipeline
// (attitude, alert filter, haptics, battery, tap to start) -> motors, LED, telemetry.
// All decisions live in lib/saferoute (unit-tested on the laptop); this file
// only moves data between the hardware and the pipeline.
#include <Arduino.h>
#include <esp_system.h>
#include <esp_sleep.h>
#include "app/pipeline.h"
#include "config.h"
#include "hal/hal.h"
#include "telemetry/ble_uart.h"

using namespace sr;

static Pipeline pod;   // the whole pod logic (lib/saferoute)
static hal::Tof tof;
static hal::Imu imu;
static hal::Haptics motors;
static hal::StatusLed led;

static uint32_t next_tick_us = 0;
static uint32_t last_batt_ms = 0, last_tof_reinit = 0, last_imu_reinit = 0;
static bool telemetry_on = TELEMETRY_SERIAL;
static uint32_t ticks = 0, overruns = 0, max_exec_us = 0, tof_frames = 0, dropped_lines = 0;
static uint32_t stats_since = 0;

// OFF at power-on: no buzzing, LED off, no telemetry (the dashboard shows nothing) until a click
static bool pod_on = false;
// The two VL53L5CX lasers draw ~100 mA each while ranging, and their firmware upload is the biggest
// current spike of the boot: started together with the ESP32 on a power bank it browned out (power,
// but it never ran). So they stay off (LPn low) until the pod is switched on.
static bool tof_started = false, tof_ok = false;
static uint32_t off_frames_until = 0;   // after switching off: "st":8 frames for 1 s (dashboard -> off)

// ------------------------------------------------------------------ helpers
static void feedWatchdog() {
  feedLoopWDT();                                // the loop task feeds it between loop() calls
}

static void printStatus(Print& out) {
  const Params& p = pod.params();
  float secs = (millis() - stats_since) / 1000.0f;
  out.printf("# SafeRoute  tof=%s(g:%d f:%d) imu=%s(0x%02X) haptics=%s(L:%d R:%d)\n", tof.name(),
             tof.groundOk, tof.forwardOk, imu.name(), imu.whoAmI, motors.name(), motors.leftOk,
             motors.rightOk);
  out.printf("# loop %.1f Hz (target %d), max exec %lu us, overruns %lu, ToF frames %.1f/s, dropped lines %lu\n",
             ticks / secs, LOOP_HZ, (unsigned long)max_exec_us, (unsigned long)overruns,
             tof_frames / secs, (unsigned long)dropped_lines);
  out.printf("# L=%.3f m beta=%.1f gamma=%.1f head=%.2f shaft=(%.2f,%.2f,%.2f) mode=%s plan=%c state=%u\n",
             p.tip_to_pod_m, p.ground_beam_deg, p.forward_beam_deg, p.head_height_m, p.shaft.x,
             p.shaft.y, p.shaft.z, pod.filter().mode() == Mode::Smart ? "smart" : "naive",
             p.haptic_plan, (unsigned)pod.state());
  const Gait& g = pod.gait();
  const Pipeline::Stats& st = pod.stats();
  static const char* kTech[] = {"unknown", "still", "sweep", "tap", "hold"};
  out.printf("# technique=%s walking=%d speed=%.2f m/s (%.2f arcs|taps/s) stride sweep=%.2f tap=%.2f m\n",
             kTech[(int)g.technique()], g.walking(), g.speedMps(), g.eventRateHz(), g.stride(Technique::Sweep),
             g.stride(Technique::Tap));
  out.printf("# session: still %lus sweep %lus tap %lus hold %lus, walked ~%.0f m, mean speed %.2f m/s, "
             "drops %u (urgent %u), head %u, slow-down cues %u\n",
             (unsigned long)(st.ms_by_tech[1] / 1000), (unsigned long)(st.ms_by_tech[2] / 1000),
             (unsigned long)(st.ms_by_tech[3] / 1000), (unsigned long)(st.ms_by_tech[4] / 1000), st.walked_m,
             st.speed_n > 0 ? st.speed_sum / st.speed_n : 0.0f, st.drop_alerts, st.urgent_alerts, st.head_alerts,
             st.slow_down_cues);
  out.printf("# alpha=%.1f (fixed %.1f for head level) sweep_off=%.1f (%s, rms %.2f m/s2) batt=%d%% %.2fV %.1fh\n",
             pod.attitude().alphaDeg(), pod.attitude().fixedAlphaDeg(), pod.attitude().yawOffsetDeg(),
             pod.attitude().sweepKnown() ? "sweeping" : "not swept", pod.attitude().sweepAccelRms(),
             pod.battery().percent(), pod.battery().volts(), pod.battery().hoursLeft());
  ticks = overruns = max_exec_us = tof_frames = 0;
  stats_since = millis();
}

// ------------------------------------------------------------------ the button
// One button (GPIO27 to GND, or the DevKit's BOOT button - either works):
//   power on   OFF: lasers off, no buzzing, LED off, no telemetry (the dashboard shows nothing)
//   click      ON: lasers start, one left-right ring, LED on, alerts on, telemetry starts
//   double click (or hold 1.5 s): OFF again - lasers, motors, LED and dashboard off
// A single click is acted on BUTTON_DOUBLE_MS after its release (it may be the start of a double click).
static void startRealSensors() {
  if (tof_started) return;
  feedWatchdog();
  tof_ok = tof.begin();                 // VL53L5CX: ~1-2.5 s per sensor (firmware upload)
  feedWatchdog();
  tof_started = true;
  last_tof_reinit = millis();
  Serial.printf("# lasers %s (g:%d f:%d)\n", tof_ok ? "on" : "FAIL", tof.groundOk, tof.forwardOk);
}

static void stopRealSensors() {
  if (!tof_started) return;
  tof.shutdown();                       // stop ranging, sleep, LPn low
  tof_started = false;
}

static void podOn(uint32_t now) {
  if (pod_on) return;
  startRealSensors();                   // blocks ~2-5 s (laser firmware upload); motors held off
  now = millis();
  pod_on = true;
  off_frames_until = 0;
  pod.command(Command::Start, now);     // left-right ring: "working", alerts on
  Serial.println("# ON (double click or hold: off)");
}

static void podOff(uint32_t now) {
  if (!pod_on) return;
  stopRealSensors();
  pod.command(Command::Standby, now);
  pod_on = false;
  off_frames_until = now + 1000;
  motors.sleep();
  led.off();
  Serial.println("# OFF (click to start again)");
}

static void buttonClick(uint32_t now) {
  if (!pod_on) podOn(now);              // a click while ON does nothing (double click = off)
}

static void buttonDouble(uint32_t now) { podOff(now); }
static void buttonHold(uint32_t now) { podOff(now); }

// Acts on release only, so a serial monitor that holds GPIO0 low through the DevKit's auto-reset
// circuit (or a stuck line) never triggers anything.
static bool buttonDown() {
  return digitalRead(PIN_BUTTON) == LOW || digitalRead(PIN_BOOT_BUTTON) == LOW;
}

static void pollButton(uint32_t now) {
  static bool down = false;
  static uint32_t since = 0, released = 0;
  static uint8_t clicks = 0;
  if (now < 1500) return;               // boot: GPIO0 may still be settling
  bool pressed = buttonDown();
  if (pressed && !down) { down = true; since = now; }
  else if (!pressed && down) {
    down = false;
    uint32_t held = now - since;
    if (held >= BUTTON_HOLD_MS) { clicks = 0; buttonHold(now); }
    else if (held >= BUTTON_DEBOUNCE_MS) {   // shorter = contact bounce
      released = now;
      if (++clicks >= 2) { clicks = 0; buttonDouble(now); }
    }
  }
  if (clicks == 1 && !down && now - released > BUTTON_DOUBLE_MS) { clicks = 0; buttonClick(now); }
}

// Cane hanging tip-down: gravity reaction points from the tip to the grip = -shaft.
static void learnShaftAxis(Print& out) {
  Vec3 sum;
  int n = 0;
  for (int i = 0; i < 50; i++) {
    ImuSample s;
    if (imu.read(s)) { sum += s.acc; n++; }
    delay(10);
  }
  if (!n) { out.println("# axis: IMU not responding"); return; }
  Vec3 shaft = (sum * (-1.0f / n)).normalized();
  // snap to the nearest sensor axis when the mount is square (within ~15 deg)
  float ax = fabsf(shaft.x), ay = fabsf(shaft.y), az = fabsf(shaft.z);
  float m = fmaxf(ax, fmaxf(ay, az));
  if (m > 0.966f) shaft = ax == m ? Vec3(shaft.x > 0 ? 1 : -1, 0, 0)
                        : ay == m ? Vec3(0, shaft.y > 0 ? 1 : -1, 0)
                                  : Vec3(0, 0, shaft.z > 0 ? 1 : -1);
  pod.params().shaft = shaft;
  pod.applyParams();
  hal::store::saveShaft(shaft);
  out.printf("# shaft axis = (%.3f, %.3f, %.3f), saved\n", shaft.x, shaft.y, shaft.z);
}

static void help(Print& out) {
  out.println(F("# commands: stat | scan | cal | bat | smart | naive | plan a|b | test l|r|b\n"
                "#   axis (cane hanging tip-down) | set L|beta|gamma|head <value> | save | reset\n"
                "#   walk start, then walk <metres> (speed calibration walk) | strength soft|normal|strong\n"
                "#   tele on|off | on | off (same as the button: click / double click) | click | dclick"));
}

static void handleCommand(char* line, Print& out) {
  uint32_t now = millis();
  char* cmd = strtok(line, " \t");
  char* a1 = strtok(nullptr, " \t");
  char* a2 = strtok(nullptr, " \t");
  if (!cmd) return;
  Params& p = pod.params();
  if (!strcmp(cmd, "stat")) printStatus(out);
  else if (!strcmp(cmd, "scan")) hal::i2cScan(out);
  else if (!strcmp(cmd, "cal")) { pod.command(Command::Calibrate, now); out.println("# calibrating 3 s: tip on flat floor, hold the cane still"); }
  else if (!strcmp(cmd, "bat")) pod.command(Command::BatteryReport, now);
  else if (!strcmp(cmd, "smart")) pod.setMode(Mode::Smart);
  else if (!strcmp(cmd, "naive")) pod.setMode(Mode::Naive);
  else if (!strcmp(cmd, "plan") && a1) { pod.setPlan(toupper(a1[0])); hal::store::saveFloat("plan", (float)toupper(a1[0])); }
  else if (!strcmp(cmd, "test") && a1) pod.command(a1[0] == 'l' ? Command::TestLeft : a1[0] == 'r' ? Command::TestRight : Command::TestBoth, now);
  else if (!strcmp(cmd, "axis")) learnShaftAxis(out);
  else if (!strcmp(cmd, "strength") && a1) {
    uint8_t s = a1[0] == 's' && a1[1] == 'o' ? 0 : a1[0] == 's' ? 2 : 1;   // soft / strong / normal
    pod.setStrength(s);
    hal::store::saveFloat("hstr", s);
    out.printf("# vibration strength: %s (saved)\n", s == 0 ? "soft" : s == 2 ? "strong" : "normal");
  }
  else if (!strcmp(cmd, "walk") && a1) {
    if (!strcmp(a1, "start")) {
      pod.command(Command::WalkCalStart, now);
      out.println("# speed calibration: walk a measured distance with your usual technique, then type: walk <metres>");
    } else if (pod.finishWalkCal(atof(a1), now)) {
      out.printf("# speed calibration OK: sweep %.2f m/arc, tap %.2f m/tap (saved)\n", pod.params().stride_sweep_m,
                 pod.params().stride_tap_m);
    } else {
      out.println("# speed calibration failed: type 'walk start' first and walk at least 5 arcs or taps");
    }
  }
  else if (!strcmp(cmd, "set") && a1 && a2) {
    float v = atof(a2);
    if (!strcmp(a1, "L")) p.tip_to_pod_m = v;
    else if (!strcmp(a1, "beta")) p.ground_beam_deg = v;
    else if (!strcmp(a1, "gamma")) p.forward_beam_deg = v;
    else if (!strcmp(a1, "head")) p.head_height_m = v;
    else { out.println("# unknown parameter"); return; }
    pod.applyParams();
    out.printf("# %s = %.3f (type 'save' to keep it)\n", a1, v);
  }
  else if (!strcmp(cmd, "save")) {
    hal::store::saveCalibration(p.tip_to_pod_m, p.ground_beam_deg);
    hal::store::saveFloat("gamma", p.forward_beam_deg);
    hal::store::saveFloat("head", p.head_height_m);
    out.println("# saved");
  }
  else if (!strcmp(cmd, "reset")) { hal::store::clear(); out.println("# stored settings cleared; reboot"); }
  else if (!strcmp(cmd, "tele") && a1) telemetry_on = !strcmp(a1, "on");
  else if (!strcmp(cmd, "off")) podOff(now);
  else if (!strcmp(cmd, "on")) podOn(now);
  else if (!strcmp(cmd, "click")) buttonClick(now);
  else if (!strcmp(cmd, "dclick")) buttonDouble(now);
  else help(out);
}

static void pollSerial() {
  static char buf[64];
  static size_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (n) { buf[n] = 0; handleCommand(buf, Serial); n = 0; }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }
  char line[64];
  if (ble::readLine(line, sizeof line)) handleCommand(line, Serial);
}

// Battery critical only (serial `off` = standby). No wake source: power-cycle to restart.
static void powerDown() {
  Serial.println("# power off (battery critical): power-cycle the pod to restart");
  Serial.flush();
  motors.set(0, 0);
  motors.sleep();
  tof.shutdown();
  led.off();
  esp_deep_sleep_start();
}

// Why the last reset happened. "brownout" = the supply dipped (motors starting, weak power bank):
// the pod reboots OFF, which looks like "no buzz".
static void printResetReason() {
  const esp_reset_reason_t r = esp_reset_reason();
  const char* why = r == ESP_RST_POWERON ? "power on" : r == ESP_RST_BROWNOUT ? "BROWNOUT (supply dipped: motors / power bank)"
                  : r == ESP_RST_SW ? "software" : r == ESP_RST_PANIC ? "CRASH (panic)"
                  : r == ESP_RST_INT_WDT || r == ESP_RST_TASK_WDT || r == ESP_RST_WDT ? "WATCHDOG (loop froze)"
                  : r == ESP_RST_EXT ? "EN button" : r == ESP_RST_DEEPSLEEP ? "deep sleep" : "other";
  Serial.printf("# last reset: %s\n", why);
}

// ------------------------------------------------------------------ setup / loop
void setup() {
  // Without a TX ring buffer the ESP32 UART only has its 128-byte hardware FIFO, so
  // availableForWrite() never exceeds 128 and every ~150-byte telemetry line would be dropped.
  Serial.setTxBufferSize(SERIAL_TX_BUFFER);   // must come before begin()
  Serial.begin(SERIAL_BAUD);
  setCpuFrequencyMhz(CPU_MHZ);
  led.begin();
  bool hap_ok = motors.begin();                 // first: motor pins driven low (off) from the start
  hal::i2cBegin();
  delay(50);

  bool calibrated = false;
  hal::store::load(pod.params(), &calibrated);
  pod.applyParams();
  pod.setPlan(pod.params().haptic_plan);
  pod.setCalibrated(calibrated);

  // The VL53L5CX driver can hang forever if the bus fails mid-command (hal/tof_vl53l5cx.cpp):
  // a frozen loop now reboots the pod after 5 s instead of leaving it dead (or buzzing).
  enableLoopWDT();
  tof.onLongBlock = [] { motors.sleep(); };   // a reload blocks 1-2.5 s: never leave a motor on
  bool imu_ok = imu.begin();
  // lasers held off (LPn low: no ranging, ~no current) until the pod is switched on (podOn)
  pinMode(PIN_TOF_GND_XSHUT, OUTPUT);
  digitalWrite(PIN_TOF_GND_XSHUT, LOW);
  pinMode(PIN_TOF_FWD_XSHUT, OUTPUT);
  digitalWrite(PIN_TOF_FWD_XSHUT, LOW);
  feedWatchdog();
  Serial.printf("\n# SafeRoute Micro Pod  imu:%s tof:off until ON haptics:%s calibrated:%s\n", imu_ok ? "ok" : "FAIL",
                hap_ok ? "ok" : "FAIL", calibrated ? "yes" : "no (type 'cal' with the tip on a flat floor)");
  printResetReason();
  hal::i2cScan(Serial);
  help(Serial);

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_BOOT_BUTTON, INPUT_PULLUP);
  if (FEATURE_BLE) ble::begin(BLE_DEVICE_NAME);
  feedWatchdog();
  pod.begin(millis());
  stats_since = millis();
  next_tick_us = micros();
}

void loop() {
  pollSerial();
  uint32_t now_us = micros();
  if ((int32_t)(now_us - next_tick_us) < 0) return;
  next_tick_us += 1000000UL / LOOP_HZ;
  if ((int32_t)(now_us - next_tick_us) > 0) {      // fell behind by a whole tick: resync
    overruns++;
    next_tick_us = now_us + 1000000UL / LOOP_HZ;
  }
  uint32_t now = millis();

  pollButton(now);
  now = millis();                                  // podOn() may have blocked while the lasers loaded
  PipelineInput in;
  in.now_ms = now;
  in.imu.ok = false;                               // OFF: no sensor read at all
  const bool real = pod_on;
  if (real) {
    imu.read(in.imu);
    if (tof_started) tof.poll(in.ground, in.forward);
  }
  if (in.ground.fresh()) tof_frames++;
  in.batt_volts = NAN;
  if (real && now - last_batt_ms >= 1000) { last_batt_ms = now; in.batt_volts = hal::readBatteryVolts(); }

  pod.step(in);
  static bool was_active = true;                  // so a standby boot prints its hint once
  static uint8_t taps_shown = 0;
  if (pod.startTaps() != taps_shown) {
    taps_shown = pod.startTaps();
    if (taps_shown) Serial.printf("# start tap %u/%u\n", taps_shown, (unsigned)pod.params().start_taps);
  }
  if (pod.active() != was_active) {
    was_active = pod.active();
    if (was_active) Serial.println("# started: alerts on");
    else Serial.printf("# standby: tap the cane tip on the floor %u times to start\n", (unsigned)pod.params().start_taps);
  }
  if (pod_on) {
    motors.set(pod.hapticLeft(), pod.hapticRight());
    led.on();
  } else {                                        // OFF: silent and dark
    motors.set(0, 0);
    led.off();
  }

  // fail-safe: re-initialise dead sensors (no alerts come from them meanwhile)
  if (real && tof_started && (pod.groundFault() || pod.forwardFault()) && now - last_tof_reinit > TOF_REINIT_MS) {
    last_tof_reinit = now;
    tof.reinit(pod.groundFault(), pod.forwardFault());   // a forward glitch no longer blinds the ground beam
  }
  if (real && pod.imuFault() && now - last_imu_reinit > TOF_REINIT_MS && now > 2000) {
    last_imu_reinit = now;
    imu.begin();
  }

  if (pod.takeStride()) {
    hal::store::saveFloat("ssw", pod.params().stride_sweep_m);
    hal::store::saveFloat("stp", pod.params().stride_tap_m);
  }

  CalResult cal = {};
  const bool cal_new = pod.takeCalibration(&cal);
  if (cal_new) {
    hal::store::saveCalibration(cal.L_m, cal.beta_deg);
    Serial.printf("# calibration OK: L=%.3f m beta=%.1f deg (%s) rms=%.1f cm n=%u\n", cal.L_m, cal.beta_deg,
                  cal.solved_beta ? "solved" : "from config", cal.rms_m * 100, cal.n);
  } else if (!pod.calibrating() && pod.lastCalibration().n && !pod.lastCalibration().ok) {
    static uint16_t reported = 0;
    if (reported != pod.lastCalibration().n) {
      reported = pod.lastCalibration().n;
      Serial.printf("# calibration FAILED (error %u, n=%u, L=%.2f, rms=%.1f cm): tip on a flat floor, beam unobstructed\n",
                    pod.lastCalibration().error, pod.lastCalibration().n, pod.lastCalibration().L_m,
                    pod.lastCalibration().rms_m * 100);
    }
  }

  static char line[256];                           // longest line ~210 chars (with hl/hr)
  static uint8_t ble_div = 0;
  int n = 0;
  if (pod_on) {
    n = formatTelemetry(line, sizeof line, pod.frame(), TELEMETRY_HAPTICS != 0);
  } else if ((int32_t)(off_frames_until - now) > 0) {   // just switched off: the dashboard goes blank
    n = snprintf(line, sizeof line, "{\"t\":%lu,\"st\":8}", (unsigned long)now);
  }
  if (n > 0) {
    if (telemetry_on) {
      if (Serial.availableForWrite() > n + 1) { Serial.write(line, n); Serial.write('\n'); }
      else dropped_lines++;
    }
    if (FEATURE_BLE && ++ble_div >= BLE_TELEMETRY_DIV) { ble_div = 0; ble::send(line, n); }
  }

  if (pod.powerOffRequested()) powerDown();

  ticks++;
  uint32_t exec = micros() - now_us;
  if (exec > max_exec_us) max_exec_us = exec;
}

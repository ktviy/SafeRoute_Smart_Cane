// Unit tests for the pod logic. Run on the laptop: pio test -e native
#include <unity.h>
#include <cmath>
#include <cstring>
#include "app/pipeline.h"
#include "config.h"
#include "kinematics/geometry.h"
#include "sensors/tof_zones.h"

using namespace sr;

// Unity calls these around every test; MinGW (Windows) has no weak defaults, so
// without them `pio test -e native` fails to link on a Windows laptop.
void setUp() {}
void tearDown() {}

// start_taps = 0: no tap-to-start standby (the tap-to-start tests turn it back on)
static Params P() {
  Params p = Params::defaults();
  p.start_taps = 0;
  return p;
}

// ---------------------------------------------------------------- geometry
void test_flat_floor_reads_zero_drop() {
  Params p = P();
  float L = p.tip_to_pod_m, b = deg2rad(p.ground_beam_deg);
  for (float a_deg = 38; a_deg <= 53; a_deg += 5) {
    float a = deg2rad(a_deg);
    float d = expectedGroundRange(L, b, a);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, groundDeltaH(L, b, a, d));
  }
}

void test_15cm_drop_reads_minus_15cm() {
  Params p = P();
  float L = p.tip_to_pod_m, b = deg2rad(p.ground_beam_deg), a = deg2rad(47);
  float h = L * std::sin(a) + 0.15f;                 // pod is 15 cm higher above the lower floor
  float d = h / std::sin(a - b);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.15f, groundDeltaH(L, b, a, d));
}

void test_data_report_numbers() {
  // DATA REPORT v3 (130 cm cane), walking alpha 36.9 deg, L 0.72, beta 19:
  // pod height 0.432 m; ToF flat 1.41 m / 14 cm step 1.86 m / 15 cm curb 1.90 m;
  // look-ahead 0.76 m past the tip
  float L = 0.72f, b = deg2rad(19), a = deg2rad(36.9f);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.432f, podHeight(L, a));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.41f, expectedGroundRange(L, b, a));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -0.14f, groundDeltaH(L, b, a, 1.86f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -0.15f, groundDeltaH(L, b, a, 1.90f));
  float d = expectedGroundRange(L, b, a);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.76f, d * std::cos(a - b) - L * std::cos(a));
}

void test_forward_hit_height() {
  // DATA REPORT v3: forward beam 73 deg above the shaft at alpha 37 deg -> 36 deg
  // elevation; a 1.5 m overhang is met about 1.46 m ahead of the pod
  ForwardHit h = forwardHit(0.72f, deg2rad(73), deg2rad(37), 1.81f);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.433f + 1.81f * std::sin(deg2rad(36)), h.height_m);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.50f, h.height_m);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.81f * std::cos(deg2rad(36)), h.x_m);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.46f, h.x_m);
}

// ---------------------------------------------------------------- calibration
// ---------------------------------------------------------------- attitude
static ImuSample still(float alpha_deg, const Vec3& shaft) {
  // sensor frame = shaft axis is -y; build "up" for a cane at alpha below horizontal
  (void)shaft;
  float a = deg2rad(alpha_deg);
  ImuSample s;
  s.ok = true;
  s.acc = Vec3(0, std::sin(a), std::cos(a)) * kG;   // up has -shaft component sin(a)
  return s;
}

void test_attitude_static_angle() {
  Params p = P();
  p.shaft = Vec3(0, -1, 0);
  Attitude att;
  att.configure(p);
  for (int i = 0; i < 200; i++) att.update(still(45, p.shaft), 0.02f);
  TEST_ASSERT_FLOAT_WITHIN(0.2f, 45.0f, att.alphaDeg());
}

// ---------------------------------------------------------------- alert filter
static Params filterParams() {
  Params p = P();
  p.tip_to_pod_m = 0.72f;
  p.ground_beam_deg = 19;
  p.forward_beam_deg = 73;   // DATA REPORT v3, 130 cm cane
  return p;
}

static RangeReading flatFrame(float alpha_deg) {
  float d = expectedGroundRange(0.72f, deg2rad(19), deg2rad(alpha_deg));
  return RangeReading(RangeStatus::Ok, (uint16_t)(d * 1000));
}

static RangeReading dropFrame(float alpha_deg, float drop_m) {
  float a = deg2rad(alpha_deg);
  float d = (0.72f * std::sin(a) + drop_m) / std::sin(a - deg2rad(19));
  return RangeReading(RangeStatus::Ok, (uint16_t)(d * 1000));
}

static RangeReading fwdAtHeight(float alpha_deg, float H) {
  float e = deg2rad(73 - alpha_deg), h = 0.72f * std::sin(deg2rad(alpha_deg));
  return RangeReading(RangeStatus::Ok, (uint16_t)((H - h) / std::sin(e) * 1000));
}

void test_drop_needs_two_frames() {
  AlertFilter f;
  Params fp = filterParams();
  fp.n_confirm_ground = 2;
  f.configure(fp);
  RangeReading none;
  uint32_t t = 0;
  f.update(t += 33, true, 45, 0, dropFrame(45, 0.15f), none);
  TEST_ASSERT_EQUAL(Hazard::None, f.hazard());
  f.update(t += 33, true, 45, 0, flatFrame(45), none);    // single noisy frame: rejected
  f.update(t += 33, true, 45, 0, dropFrame(45, 0.15f), none);
  TEST_ASSERT_EQUAL(Hazard::None, f.hazard());
  f.update(t += 33, true, 45, 0, dropFrame(45, 0.15f), none);
  TEST_ASSERT_EQUAL(Hazard::Drop, f.hazard());
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -0.15f, f.lastDeltaH());
}

void test_small_step_ignored() {
  AlertFilter f;
  f.configure(filterParams());
  RangeReading none;
  uint32_t t = 0;
  for (int i = 0; i < 5; i++) f.update(t += 33, true, 45, 0, dropFrame(45, 0.05f), none);
  TEST_ASSERT_EQUAL(Hazard::None, f.hazard());
}

void test_pause_when_cane_vertical() {
  AlertFilter f;
  f.configure(filterParams());
  RangeReading none;
  uint32_t t = 0;
  // 130 cm cane: walking never above 45.7 deg; after finding an edge the cane goes to 52-65 deg
  for (int i = 0; i < 4; i++) f.update(t += 33, true, 56, 0, dropFrame(56, 0.3f), none);
  TEST_ASSERT_TRUE(f.paused());
  TEST_ASSERT_EQUAL(Hazard::None, f.hazard());
  f.update(t += 33, true, 50, 0, dropFrame(50, 0.3f), none);   // hysteresis: still paused
  TEST_ASSERT_TRUE(f.paused());
  f.update(t += 33, true, 47, 0, dropFrame(47, 0.3f), none);
  TEST_ASSERT_FALSE(f.paused());
}

void test_drop_outside_corridor_ignored() {
  AlertFilter f;
  f.configure(filterParams());
  RangeReading none;
  uint32_t t = 0;
  for (int i = 0; i < 4; i++) f.update(t += 33, true, 45, 50, dropFrame(45, 0.2f), none);
  TEST_ASSERT_EQUAL(Hazard::None, f.hazard());
}

void test_head_band_and_side() {
  AlertFilter f;
  f.configure(filterParams());
  RangeReading none;
  uint32_t t = 0;
  // floor-level box (0.5 m) ignored
  for (int i = 0; i < 4; i++) f.update(t += 33, true, 40, 0, flatFrame(40), fwdAtHeight(40, 0.5f));
  TEST_ASSERT_EQUAL(Hazard::None, f.hazard());
  // 1.4 m bar slightly right of the centre (at 40 deg the 73 deg beam meets a 1.5 m bar
  // 1.6 m ahead - DATA REPORT v3: 1.32-1.61 m over the alpha range, inside the 1.7 m limit)
  for (int i = 0; i < 2; i++) f.update(t += 33, true, 40, 8, flatFrame(40), fwdAtHeight(40, 1.4f));
  TEST_ASSERT_EQUAL(Hazard::HeadRight, f.hazard());
  // above head (2.2 m) ignored
  t += 1000;
  for (int i = 0; i < 4; i++) f.update(t += 33, true, 40, -8, flatFrame(40), fwdAtHeight(40, 2.2f));
  TEST_ASSERT_EQUAL(Hazard::None, f.hazard());
  for (int i = 0; i < 2; i++) f.update(t += 33, true, 40, -8, flatFrame(40), fwdAtHeight(40, 1.3f));
  TEST_ASSERT_EQUAL(Hazard::HeadLeft, f.hazard());
  // person standing 0.75 m beside the path, inside the 25 deg sweep corridor: ignored
  t += 1000;
  for (int i = 0; i < 4; i++) f.update(t += 33, true, 40, 22, flatFrame(40), fwdAtHeight(40, 1.4f));
  TEST_ASSERT_EQUAL(Hazard::None, f.hazard());
}

void test_naive_mode_alerts_on_everything() {
  AlertFilter f;
  f.configure(filterParams());
  f.setMode(Mode::Naive);
  RangeReading none;
  f.update(33, true, 45, 50, flatFrame(45), fwdAtHeight(45, 1.0f));   // object far off the path
  TEST_ASSERT_TRUE(f.hazard() != Hazard::None);
}

void test_faults_never_alert() {
  AlertFilter f;
  f.configure(filterParams());
  RangeReading none, bad(RangeStatus::Invalid, 0), fault(RangeStatus::Fault, 0);
  uint32_t t = 0;
  for (int i = 0; i < 10; i++) f.update(t += 33, true, 45, 0, i % 2 ? bad : fault, fault);
  TEST_ASSERT_EQUAL(Hazard::None, f.hazard());
  TEST_ASSERT_TRUE(f.forwardFault());
  // stale: no frames at all for > 250 ms
  for (int i = 0; i < 20; i++) f.update(t += 20, true, 45, 0, none, none);
  TEST_ASSERT_TRUE(f.groundFault());
  // drop frames while the IMU is down: no alert
  for (int i = 0; i < 4; i++) f.update(t += 33, false, 45, 0, dropFrame(45, 0.3f), none);
  TEST_ASSERT_EQUAL(Hazard::None, f.hazard());
}

void test_no_return_counts_as_deep_drop() {
  AlertFilter f;
  f.configure(filterParams());
  RangeReading none, nt(RangeStatus::NoTarget, 0);
  uint32_t t = 0;
  for (int i = 0; i < 4; i++) f.update(t += 33, true, 45, 0, nt, none);
  TEST_ASSERT_EQUAL(Hazard::None, f.hazard());              // needs 3 + 2 frames
  f.update(t += 33, true, 45, 0, nt, none);
  TEST_ASSERT_EQUAL(Hazard::Drop, f.hazard());
}

// Dark carpet / glossy tiles / sunlight: the ground beam loses the floor in runs
// (here 6 empty frames, 2 good ones, over and over). Without the surface check
// every run of 5 empty frames is a "platform edge" - a false alarm every ~0.3 s.
static int darkFloorAlertFrames(float max_frac) {
  Params fp = filterParams();
  fp.no_return_max_frac = max_frac;
  AlertFilter f;
  f.configure(fp);
  RangeReading none, nt(RangeStatus::NoTarget, 0);
  uint32_t t = 0;
  int alerted = 0;
  for (int i = 0; i < 180; i++) {                           // 6 s at 30 Hz
    f.update(t += 33, true, 45, 0, i % 8 < 6 ? nt : flatFrame(45), none);
    if (t > 2000 && f.hazard() == Hazard::Drop) alerted++;  // after the 2 s boot settle
  }
  return alerted;
}

void test_dark_floor_no_return_is_not_an_edge() {
  TEST_ASSERT_TRUE(darkFloorAlertFrames(1.0f) > 100);       // gate off: buzzing nearly all the time
  TEST_ASSERT_EQUAL(0, darkFloorAlertFrames(P().no_return_max_frac));
}

void test_platform_edge_after_floor_still_alerts() {
  AlertFilter f;
  f.configure(filterParams());
  RangeReading none, nt(RangeStatus::NoTarget, 0);
  uint32_t t = 0;
  for (int i = 0; i < 90; i++) f.update(t += 33, true, 45, 0, flatFrame(45), none);   // 3 s of floor
  TEST_ASSERT_FALSE(f.groundSurfacePoor());
  int frames = 0;
  while (f.hazard() != Hazard::Drop && frames < 20) { f.update(t += 33, true, 45, 0, nt, none); frames++; }
  TEST_ASSERT_EQUAL(Hazard::Drop, f.hazard());
  TEST_ASSERT_EQUAL(5, frames);                             // 3 + 2 frames, as before
  for (int i = 0; i < 60; i++) {                           // beam hangs over the void for 2 s:
    f.update(t += 33, true, 45, 0, nt, none);              // the warning must not fade out
    TEST_ASSERT_EQUAL(Hazard::Drop, f.hazard());
  }
  // a measured drop still alerts on a poor surface
  AlertFilter g;
  g.configure(filterParams());
  for (int i = 0; i < 120; i++) g.update(t += 33, true, 45, 0, i % 8 < 6 ? nt : flatFrame(45), none);
  TEST_ASSERT_TRUE(g.groundSurfacePoor());
  for (int i = 0; i < 3; i++) g.update(t += 33, true, 45, 0, dropFrame(45, 0.15f), none);
  TEST_ASSERT_EQUAL(Hazard::Drop, g.hazard());
}

void test_no_forward_sensor_is_not_a_fault() {
  // EE cut list step 4: forward beam removed. Ground drop-offs still alert; the
  // missing forward sensor is neither a fault (amber LED, re-init) nor an alert.
  Params fp = filterParams();
  fp.forward_enabled = false;
  AlertFilter f;
  f.configure(fp);
  RangeReading none;
  uint32_t t = 0;
  for (int i = 0; i < 20; i++) f.update(t += 33, true, 45, 0, flatFrame(45), none);
  TEST_ASSERT_FALSE(f.forwardFault());
  TEST_ASSERT_FALSE(f.groundFault());
  for (int i = 0; i < 4; i++) f.update(t += 33, true, 45, 0, flatFrame(45), fwdAtHeight(45, 1.4f));
  TEST_ASSERT_EQUAL(Hazard::None, f.hazard());              // stray forward frames ignored
  for (int i = 0; i < 3; i++) f.update(t += 33, true, 45, 0, dropFrame(45, 0.15f), none);
  TEST_ASSERT_EQUAL(Hazard::Drop, f.hazard());
}

// ---------------------------------------------------------------- haptics
void test_drop_pattern_timing() {
  HapticEngine h;
  const Params p = P();
  const int S = p.haptic_strong;
  h.configure(p);
  h.setHazard(Hazard::Drop, 1000);
  TEST_ASSERT_EQUAL(S, h.left());                         // starts immediately
  TEST_ASSERT_EQUAL(S, h.right());
  h.tick(1079); TEST_ASSERT_EQUAL(S, h.left());
  h.tick(1080); TEST_ASSERT_EQUAL(0, h.left());
  h.tick(1140); TEST_ASSERT_EQUAL(S, h.left());
  h.tick(1580); TEST_ASSERT_EQUAL(0, h.left());             // 3 x (80 + 60) - 60 + 240 = 600 ms cycle
  h.tick(1600); TEST_ASSERT_EQUAL(S, h.left());           // repeats
  h.setHazard(Hazard::None, 1610);
  TEST_ASSERT_EQUAL(0, h.left());
}

void test_head_side_plan_a_and_b() {
  HapticEngine h;
  const Params p = P();
  const int S = p.haptic_strong;
  h.configure(p);
  h.setPlan('A');
  h.setHazard(Hazard::HeadLeft, 0);
  TEST_ASSERT_TRUE(h.left() > 0);
  TEST_ASSERT_EQUAL(0, h.right());
  h.setHazard(Hazard::HeadRight, 10);
  TEST_ASSERT_EQUAL(0, h.left());
  TEST_ASSERT_TRUE(h.right() > 0);
  h.setHazard(Hazard::None, 20);
  h.setPlan('B');
  h.setHazard(Hazard::HeadLeft, 30);
  TEST_ASSERT_TRUE(h.left() > 0 && h.right() > 0);          // rhythm only, both sides
}

static int countPulses(HapticEngine& h, uint32_t t0, uint32_t t1) {
  int n = 0;
  bool on = false;
  for (uint32_t t = t0; t < t1; t++) {
    h.tick(t);
    bool now_on = h.left() > 0;
    if (now_on && !on) n++;
    on = now_on;
  }
  return n;
}

void test_battery_report_pulses_and_preemption() {
  HapticEngine h;
  h.configure(P());
  h.play(Pattern::Battery, 0, 4);
  TEST_ASSERT_EQUAL(4, countPulses(h, 0, 3000));
  h.play(Pattern::Battery, 3000, 3);
  h.setHazard(Hazard::Drop, 3100);                          // a drop interrupts the report
  TEST_ASSERT_EQUAL(Pattern::Drop, h.current());
  h.setHazard(Hazard::None, 3500);
  TEST_ASSERT_EQUAL(3, countPulses(h, 3500, 6000));         // report replays afterwards
}

// The dashboard plays a one-shot cue when cueSeq() changes: hazards must not bump it,
// a queued cue bumps it only when it actually starts.
void test_cue_sequence_for_dashboard() {
  HapticEngine h;
  h.configure(P());
  uint8_t s0 = h.cueSeq();
  h.setHazard(Hazard::Drop, 0);
  TEST_ASSERT_EQUAL(s0, h.cueSeq());                        // hazard: no cue
  h.play(Pattern::Battery, 10, 4);                          // queued behind the hazard
  TEST_ASSERT_EQUAL(s0, h.cueSeq());
  h.setHazard(Hazard::None, 500);                           // hazard over: the report starts now
  TEST_ASSERT_EQUAL((uint8_t)(s0 + 1), h.cueSeq());
  TEST_ASSERT_EQUAL(Pattern::Battery, h.lastCue());
  TEST_ASSERT_EQUAL(4, h.lastCueCount());
  countPulses(h, 500, 3000);
  h.play(Pattern::Test, 3000, 2);
  TEST_ASSERT_EQUAL((uint8_t)(s0 + 2), h.cueSeq());
  TEST_ASSERT_EQUAL(Pattern::Test, h.lastCue());
  TEST_ASSERT_EQUAL(2, h.lastCueCount());
}

// ---------------------------------------------------------------- battery
void test_battery_table() {
  TEST_ASSERT_EQUAL(100, lipoPercent(4.20f));
  TEST_ASSERT_EQUAL(0, lipoPercent(3.20f));
  TEST_ASSERT_UINT8_WITHIN(5, 50, lipoPercent(3.84f));
  TEST_ASSERT_TRUE(lipoPercent(3.75f) < lipoPercent(3.95f));
}

void test_low_battery_warning_every_5_min() {
  Params p = P();
  BatteryMonitor b;
  b.configure(p);
  b.update(3.72f, 0);
  TEST_ASSERT_TRUE(b.low());
  TEST_ASSERT_TRUE(b.lowWarningDue(0));
  TEST_ASSERT_FALSE(b.lowWarningDue(60000));
  TEST_ASSERT_TRUE(b.lowWarningDue(5UL * 60UL * 1000UL));
  TEST_ASSERT_EQUAL(1, b.reportPulses());
  BatteryMonitor full;
  full.configure(p);
  full.update(4.18f, 0);
  TEST_ASSERT_EQUAL(5, full.reportPulses());
  TEST_ASSERT_FALSE(full.lowWarningDue(0));
}

// ---------------------------------------------------------------- tap to start
// One 20 ms tick, cane still at 45 deg. A tap = |a| x1.5 for one tick: ~5 m/s2 above
// gravity, what the +-1.5 g MMA7660FC reports for a hard tap (it clips).
static void standbyTick(Pipeline& pod, uint32_t& t, const Params& p, bool tap, float drop_m = 0) {
  PipelineInput in;
  in.now_ms = t += 20;
  in.imu = still(45, p.shaft);
  if (tap) in.imu.acc = in.imu.acc * 1.5f;
  in.ground = drop_m > 0 ? dropFrame(45, drop_m) : flatFrame(45);
  in.forward = RangeReading();
  in.batt_volts = NAN;
  pod.step(in);
}

static void standbyTaps(Pipeline& pod, uint32_t& t, const Params& p, int n, int gap_ticks) {
  for (int i = 0; i < n; i++)
    for (int k = 0; k < gap_ticks; k++) standbyTick(pod, t, p, k == 0);
}

void test_standby_until_five_quick_taps() {
  Params p = filterParams();
  p.shaft = Vec3(0, -1, 0);
  p.start_taps = 5;
  Pipeline pod(p);
  pod.setCalibrated(true);
  uint32_t t = 0;
  for (int i = 0; i < 150; i++) standbyTick(pod, t, p, false);   // 3 s still
  TEST_ASSERT_FALSE(pod.active());
  TEST_ASSERT_EQUAL(DeviceState::Standby, pod.state());
  standbyTaps(pod, t, p, 4, 20);                                 // 4 taps 0.4 s apart: not yet
  TEST_ASSERT_FALSE(pod.active());
  TEST_ASSERT_EQUAL(4, pod.startTaps());
  for (int i = 0; i < 50; i++) standbyTick(pod, t, p, false);    // 1 s pause: the count is lost
  TEST_ASSERT_EQUAL(0, pod.startTaps());
  standbyTaps(pod, t, p, 6, 55);                                 // walking rhythm, 1.1 s apart
  TEST_ASSERT_FALSE(pod.active());
  standbyTaps(pod, t, p, 5, 20);                                 // 5 quick taps: on
  TEST_ASSERT_TRUE(pod.active());
  for (int i = 0; i < 100; i++) standbyTick(pod, t, p, false);
  TEST_ASSERT_TRUE(pod.state() != DeviceState::Standby);
  pod.command(Command::Standby, t);                              // serial `off`
  TEST_ASSERT_FALSE(pod.active());
  for (int i = 0; i < 5; i++) standbyTick(pod, t, p, false);
  TEST_ASSERT_EQUAL(DeviceState::Standby, pod.state());
}

void test_standby_is_silent_until_started() {
  Params p = filterParams();
  p.shaft = Vec3(0, -1, 0);
  p.start_taps = 5;
  Pipeline pod(p);
  pod.setCalibrated(true);
  uint32_t t = 0;
  for (int i = 0; i < 150; i++) standbyTick(pod, t, p, false);
  int buzz = 0;
  for (int i = 0; i < 100; i++) {                                // 15 cm edge ahead, in standby
    standbyTick(pod, t, p, false, 0.15f);
    buzz += pod.hapticLeft() > 0;
  }
  TEST_ASSERT_EQUAL(Hazard::Drop, pod.hazard());                 // seen...
  TEST_ASSERT_EQUAL(0, buzz);                                    // ...but never buzzed
  standbyTaps(pod, t, p, 5, 20);                                 // start (flat floor while tapping)
  TEST_ASSERT_TRUE(pod.active());
  bool alerted = false;
  for (int i = 0; i < 100; i++) {
    standbyTick(pod, t, p, false, 0.15f);
    alerted |= pod.hapticLeft() > 0 && pod.hazard() == Hazard::Drop;
  }
  TEST_ASSERT_TRUE(alerted);
}

// ---------------------------------------------------------------- telemetry
void test_telemetry_format() {
  TelemetryFrame f = {};
  f.t_ms = 1234; f.alpha_deg = 45.25f; f.yaw_deg = -3; f.dg_mm = 1200; f.df_mm = -1;
  f.dh_m = -0.153f; f.fwd_h_m = NAN; f.alert = 1; f.batt_pct = 87;
  char buf[256];
  formatTelemetry(buf, sizeof buf, f, false);
  TEST_ASSERT_NOT_NULL(strstr(buf, "{\"t\":1234,\"a\":45."));
  TEST_ASSERT_NOT_NULL(strstr(buf, "\"dg\":1200,\"df\":null,\"dh\":-153,\"alert\":1,\"batt\":87"));
  TEST_ASSERT_NOT_NULL(strstr(buf, "\"hf\":null"));
  f.cue = (uint8_t)Pattern::Battery; f.cue_n = 3; f.cue_seq = 7; f.plan = 'B';
  formatTelemetry(buf, sizeof buf, f, false);
  TEST_ASSERT_NOT_NULL(strstr(buf, "\"hp\":5,\"hn\":3,\"hs\":7,\"pl\":\"B\"}"));
}

// ---------------------------------------------------------------- end to end
void test_pipeline_latency_under_100ms() {
  Params p = filterParams();
  p.shaft = Vec3(0, -1, 0);
  Pipeline pipe(p);
  pipe.setCalibrated(true);
  PipelineInput in;
  in.batt_volts = NAN;
  uint32_t t = 0;
  int tick = 0;
  // 3 s walking on flat floor
  for (; tick < 150; tick++) {
    in.now_ms = t += 20;
    in.imu = still(45, p.shaft);
    in.ground = (tick % 3 != 2) ? flatFrame(45) : RangeReading();
    in.forward = (tick % 3 != 2) ? RangeReading(RangeStatus::NoTarget, 0) : RangeReading();
    pipe.step(in);
    TEST_ASSERT_EQUAL(Hazard::None, pipe.hazard());
  }
  // the beam goes over a 15 cm edge
  uint32_t edge_t = t + 20, motor_t = 0;
  for (int k = 0; k < 20 && !motor_t; k++, tick++) {
    in.now_ms = t += 20;
    in.imu = still(45, p.shaft);
    in.ground = (tick % 3 != 2) ? dropFrame(45, 0.15f) : RangeReading();
    pipe.step(in);
    if (pipe.hapticLeft() > 0 && pipe.hazard() == Hazard::Drop) motor_t = t;
  }
  TEST_ASSERT_TRUE(motor_t > 0);
  TEST_ASSERT_TRUE(motor_t - edge_t <= 100);
}

// Two-point tapping, P1 style: the cane pivots in the hand between taps, the tip
// rises ~10.5 cm (130 cm cane, hand 1.15 m from the tip) and the cane angle drops from 45 to 38 deg.
static void tapTick(Pipeline& pod, PipelineInput& in, uint32_t& t, int tick, const Params& p,
                    float floor_drop_m) {
  const float Lc = p.tip_to_pod_m + p.grip_to_pod_m;
  const float hand = Lc * std::sin(deg2rad(45));          // hand height, tip on the floor at a tap
  int k = tick % 55;                                       // a tap every 1.1 s
  float a_deg = 45 - 7.0f * std::sin(3.14159f * k / 55);   // tip up between taps
  float a = deg2rad(a_deg);
  in.now_ms = t += 20;
  in.imu = still(a_deg, p.shaft);
  if (k == 0) in.imu.acc = in.imu.acc * 2.0f;              // tap impact
  float pod_h = hand - p.grip_to_pod_m * std::sin(a);      // pod height above the walking floor
  float d = (pod_h + floor_drop_m) / std::sin(a - deg2rad(p.ground_beam_deg));
  in.ground = tick % 3 != 2 ? RangeReading(RangeStatus::Ok, (uint16_t)(d * 1000)) : RangeReading();
  if (in.ground.status == RangeStatus::Ok) {               // floor plane seen by the zones: the
    in.ground.plane_dep_deg = a_deg - p.ground_beam_deg;   // true cane angle (no gyro: the pod's
    in.ground.plane_h_m = pod_h + floor_drop_m;            // tap reference comes from this)
  }
  in.forward = RangeReading();
  pod.step(in);
}

void test_tap_lift_compensated_curb_still_seen() {
  Params p = filterParams();
  p.shaft = Vec3(0, -1, 0);
  p.tap_mode = 1;
  Pipeline pod(p);
  pod.setCalibrated(true);
  PipelineInput in;
  in.batt_volts = NAN;
  uint32_t t = 0;
  bool alerted = false;
  float max_lift = 0;
  for (int tick = 0; tick < 500; tick++) {                 // flat floor, tapping
    tapTick(pod, in, t, tick, p, 0.0f);
    if (pod.tipLift() > max_lift) max_lift = pod.tipLift();
    if (tick > 200 && pod.hazard() == Hazard::Drop) alerted = true;
  }
  TEST_ASSERT_TRUE(pod.tapping());
  TEST_ASSERT_FLOAT_WITHIN(0.01f, (p.tip_to_pod_m + p.grip_to_pod_m) * (std::sin(deg2rad(45)) - std::sin(deg2rad(38))),
                           max_lift);                      // ~10.5 cm lift recognised
  TEST_ASSERT_FALSE(alerted);
  // a 15 cm curb appears between two taps: seen at once, no need to wait for a tap
  int tick = 500 + 20, first = -1;
  for (; tick < 560; tick++) {
    tapTick(pod, in, t, tick, p, 0.15f);
    if (pod.hazard() == Hazard::Drop) { first = tick; break; }
  }
  TEST_ASSERT_TRUE(first >= 0);
  TEST_ASSERT_TRUE(first - 520 <= 6);                      // 3 frames at 30 Hz = 5 ticks
}

void test_tap_lift_without_compensation_false_alarms() {
  Params p = filterParams();
  p.shaft = Vec3(0, -1, 0);
  p.tap_mode = 0;
  Pipeline pod(p);
  pod.setCalibrated(true);
  PipelineInput in;
  in.batt_volts = NAN;
  uint32_t t = 0;
  bool alerted = false;
  for (int tick = 0; tick < 500; tick++) {
    tapTick(pod, in, t, tick, p, 0.0f);
    if (tick > 200 && pod.hazard() == Hazard::Drop) alerted = true;
  }
  TEST_ASSERT_TRUE(alerted);                               // why the compensation exists
}

// ---------------------------------------------------------------- technique and speed
// Synthetic walking: sweep of +-25 deg at f Hz (2f arcs/s), step bounce 0.8 m/s2 at
// 1.8 Hz, optional tip impacts at each sweep turning point (two-point touch).
struct WalkSim {
  float sweep_hz = 0.49f, sweep_deg = 25, bounce = 1.0f;
  bool taps_on_turns = false;
  void tick(Gait& g, uint32_t& t) {
    t += 20;
    float ts = t * 0.001f;
    float rate = sweep_deg * 2 * 3.14159f * sweep_hz * std::cos(2 * 3.14159f * sweep_hz * ts);
    float mag = kG + bounce * std::sin(2 * 3.14159f * 1.8f * ts);
    bool impact = false;
    if (taps_on_turns) {
      float phase = std::fmod(ts * 2 * sweep_hz + 0.5f, 1.0f);   // 0 at each turning point
      impact = phase < 0.02f * 2 * sweep_hz;
    }
    if (impact) mag += 12;
    g.update(t, 0.02f, mag, sweep_deg > 0 ? rate : 0.0f, impact);
  }
};

void test_gait_sweep_speed() {
  Gait g;
  g.configure(P());
  WalkSim w;
  uint32_t t = 0;
  for (int i = 0; i < 600; i++) w.tick(g, t);
  TEST_ASSERT_EQUAL(Technique::Sweep, g.technique());
  TEST_ASSERT_TRUE(g.walking());
  TEST_ASSERT_FLOAT_WITHIN(0.06f, 0.98f, g.eventRateHz());
  TEST_ASSERT_FLOAT_WITHIN(0.08f, 0.81f, g.speedMps());     // 0.98 arcs/s x 0.83 m
}

void test_gait_tap_speed() {
  Gait g;
  g.configure(P());
  WalkSim w;
  w.sweep_hz = 0.45f;
  w.bounce = 1.3f;
  w.taps_on_turns = true;
  uint32_t t = 0;
  for (int i = 0; i < 600; i++) w.tick(g, t);
  TEST_ASSERT_EQUAL(Technique::Tap, g.technique());
  TEST_ASSERT_FLOAT_WITHIN(0.08f, 0.80f, g.speedMps());     // 0.90 taps/s x 0.89 m
}

void test_gait_still_standing_and_hold() {
  Gait g;
  g.configure(P());
  WalkSim still;
  still.sweep_deg = 0;
  still.bounce = 0;
  uint32_t t = 0;
  for (int i = 0; i < 300; i++) still.tick(g, t);
  TEST_ASSERT_EQUAL(Technique::Still, g.technique());
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, g.speedMps());

  Gait s;
  s.configure(P());
  WalkSim stand;                                            // sweeping while standing
  stand.bounce = 0.4f;                                     // arm motion, no steps (rms 0.25)
  t = 0;
  for (int i = 0; i < 400; i++) stand.tick(s, t);
  TEST_ASSERT_EQUAL(Technique::Sweep, s.technique());
  TEST_ASSERT_FALSE(s.walking());
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, s.speedMps());

  Gait h;
  h.configure(P());
  WalkSim diag;                                             // diagonal cane: walking, no sweep
  diag.sweep_deg = 0;
  t = 0;
  for (int i = 0; i < 400; i++) diag.tick(h, t);
  TEST_ASSERT_EQUAL(Technique::Hold, h.technique());
  TEST_ASSERT_TRUE(std::isnan(h.speedMps()));               // no arcs to count: unknown
}

void test_gait_calibration_walk() {
  Gait g;
  g.configure(P());
  WalkSim w;
  uint32_t t = 0;
  for (int i = 0; i < 200; i++) w.tick(g, t);
  g.startCalWalk();
  for (int i = 0; i < 1000; i++) w.tick(g, t);             // 20 s at 0.98 arcs/s = ~20 arcs
  Technique tech;
  float stride;
  TEST_ASSERT_TRUE(g.finishCalWalk(12.0f, &tech, &stride));
  TEST_ASSERT_EQUAL(Technique::Sweep, tech);
  TEST_ASSERT_FLOAT_WITHIN(0.06f, 12.0f / 19.6f, stride);
  TEST_ASSERT_FALSE(g.finishCalWalk(12.0f, &tech, &stride));   // not started again: too few arcs
}

void test_telemetry_has_technique_and_speed() {
  TelemetryFrame f = {};
  f.dh_m = f.fwd_h_m = NAN;
  f.tech = Technique::Tap;
  f.speed_mps = 0.812f;
  char buf[256];
  formatTelemetry(buf, sizeof buf, f, false);
  TEST_ASSERT_NOT_NULL(strstr(buf, "\"tech\":\"T\",\"v\":0.81"));
}

void test_pod_blind_cue_when_ground_sensor_dies() {
  Params p = filterParams();
  p.shaft = Vec3(0, -1, 0);
  Pipeline pod(p);
  pod.setCalibrated(true);
  PipelineInput in;
  in.batt_volts = NAN;
  uint32_t t = 0;
  for (int i = 0; i < 400; i++) {                      // 8 s healthy
    in.now_ms = t += 20;
    in.imu = still(45, p.shaft);
    in.ground = i % 3 != 2 ? flatFrame(45) : RangeReading();
    in.forward = RangeReading();
    pod.step(in);
  }
  TEST_ASSERT_EQUAL(0, pod.stats().pod_blind_cues);
  int cue_tick = -1;
  for (int i = 0; i < 300; i++) {                      // ground ToF stops answering
    in.now_ms = t += 20;
    in.imu = still(45, p.shaft);
    in.ground = RangeReading();
    pod.step(in);
    if (cue_tick < 0 && pod.stats().pod_blind_cues == 1) cue_tick = i;
  }
  TEST_ASSERT_TRUE(cue_tick > 0);
  TEST_ASSERT_TRUE(cue_tick * 20 >= 3000 && cue_tick * 20 <= 3500);   // 250 ms stale + 3 s
  TEST_ASSERT_EQUAL(1, pod.stats().pod_blind_cues);    // once, not every tick
  TEST_ASSERT_EQUAL(Hazard::None, pod.hazard());       // and never an alert
}

void test_strength_scales_levels_not_rhythm() {
  Params p = P();
  HapticEngine h;
  h.configure(p);
  h.play(Pattern::Battery, 0, 1);
  uint8_t normal = h.left();
  h.stop();
  h.setStrength(2);
  h.play(Pattern::Battery, 0, 1);
  uint8_t strong = h.left();
  TEST_ASSERT_TRUE(strong > normal);
  h.tick(149); TEST_ASSERT_TRUE(h.left() > 0);          // same 150 ms pulse
  h.tick(150); TEST_ASSERT_EQUAL(0, h.left());
  h.stop();
  h.setStrength(0);
  h.play(Pattern::Battery, 0, 1);
  TEST_ASSERT_TRUE(h.left() < normal);
  h.stop();
  h.setHazard(Hazard::Drop, 0);
  TEST_ASSERT_TRUE(h.left() >= p.haptic_strong * 8 / 10);   // safety pulses stay strong even on soft
}

// ---- VL53L5CX zones -> one beam (lib/saferoute/sensors/tof_zones) -------------------
static ZoneSample zs(int mm, uint8_t st = kZoneStatusValid, uint8_t n = 1) {
  ZoneSample z;
  z.mm = (int16_t)mm;
  z.status = st;
  z.targets = n;
  return z;
}

// The four central zones on a flat floor, as rays kZoneOffAxisDeg off axis in pitch and yaw.
// near_first: which diagonal slot holds the near zone (the mount can be rotated / flipped).
static void flatFloorZones(float alpha_deg, ZoneSample z[4], bool near_first) {
  const Params p = P();
  const float L = p.tip_to_pod_m, beta = deg2rad(p.ground_beam_deg), a = deg2rad(alpha_deg);
  const float t = std::tan(deg2rad(kZoneOffAxisDeg)), ct = 1.0f / std::sqrt(1.0f + 2.0f * t * t);
  const float H = L * std::sin(a), s = std::sin(a - beta), c = std::cos(a - beta);
  int near = (int)std::lround(1000.0f * H / (ct * (s + t * c)));
  int far = (int)std::lround(1000.0f * H / (ct * (s - t * c)));
  z[0] = zs(near_first ? near : far);
  z[3] = zs(near_first ? far : near);
  z[1] = zs(near_first ? far : near);
  z[2] = zs(near_first ? near : far);
}

static float ct_of(float off_deg) {
  const float t = std::tan(deg2rad(off_deg));
  return 1.0f / std::sqrt(1.0f + 2.0f * t * t);
}

void test_zone_classify() {
  TEST_ASSERT_TRUE(classifyZone(zs(900)) == ZoneClass::Ok);
  TEST_ASSERT_TRUE(classifyZone(zs(900, kZoneStatusValidLargePulse)) == ZoneClass::Ok);
  TEST_ASSERT_TRUE(classifyZone(zs(900, kZoneStatusNoTarget)) == ZoneClass::Empty);
  TEST_ASSERT_TRUE(classifyZone(zs(900, kZoneStatusValid, 0)) == ZoneClass::Empty);
  TEST_ASSERT_TRUE(classifyZone(zs(900, 4)) == ZoneClass::Bad);     // consistency failed
  TEST_ASSERT_TRUE(classifyZone(zs(900, 6)) == ZoneClass::Bad);     // wrap-around not checked
  TEST_ASSERT_TRUE(classifyZone(zs(0)) == ZoneClass::Bad);
  TEST_ASSERT_TRUE(classifyZone(zs(-5)) == ZoneClass::Bad);
}

// Flat floor: the combined ground range must be the centre ray at every walking angle,
// for either mount orientation - a plain mean would read 1-2.5 cm "deep".
void test_zone_ground_flat_floor_is_centre_ray() {
  const Params prm = P();
  const float L = prm.tip_to_pod_m, beta = deg2rad(prm.ground_beam_deg);
  for (float alpha = 28.0f; alpha <= 48.0f; alpha += 2.0f) {
    for (int o = 0; o < 2; o++) {
      ZoneSample z[4];
      flatFloorZones(alpha, z, o == 0);
      RangeReading r = groundFromZones(z);
      TEST_ASSERT_TRUE(r.status == RangeStatus::Ok);
      float dh = groundDeltaH(L, beta, deg2rad(alpha), r.mm * 0.001f);
      TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.0f, dh);
      // one diagonal pair alone gives the same answer
      ZoneSample p[4] = {z[0], zs(0, 4), zs(0, 4), z[3]};
      RangeReading rp = groundFromZones(p);
      TEST_ASSERT_TRUE(rp.status == RangeStatus::Ok);
      TEST_ASSERT_INT_WITHIN(3, r.mm, rp.mm);
      // the arithmetic mean really is biased at the low angles
      if (alpha <= 32.0f) {
        float mean = (z[0].mm + z[1].mm + z[2].mm + z[3].mm) * 0.25f * 0.001f;
        TEST_ASSERT_TRUE(groundDeltaH(L, beta, deg2rad(alpha), mean) < -0.015f);
      }
      // and the constant matches the zone geometry
      TEST_ASSERT_FLOAT_WITHIN(1e-5f, ct_of(kZoneOffAxisDeg), kZoneCosOffAxis);
    }
  }
}

void test_zone_ground_drop_and_empty_rules() {
  const Params prm = P();
  const float L = prm.tip_to_pod_m, beta = deg2rad(prm.ground_beam_deg), a = deg2rad(37.0f);
  ZoneSample z[4];
  flatFloorZones(37.0f, z, true);
  // whole window over a 20 cm drop: every zone reads 1 + 0.2/H longer -> full depth
  const float k = 1.0f + 0.20f / (L * std::sin(a));
  ZoneSample d[4];
  for (int i = 0; i < 4; i++) d[i] = zs((int)std::lround(z[i].mm * k));
  RangeReading r = groundFromZones(d);
  TEST_ASSERT_TRUE(r.status == RangeStatus::Ok);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -0.20f, groundDeltaH(L, beta, a, r.mm * 0.001f));
  // two empty zones (the weak far ones on a dark floor): not trusted as an edge
  ZoneSample e2[4] = {z[0], zs(0, kZoneStatusNoTarget, 0), z[2], zs(0, kZoneStatusNoTarget, 0)};
  TEST_ASSERT_TRUE(groundFromZones(e2).status == RangeStatus::Invalid);
  // ... unless a diagonal pair still answers
  ZoneSample e2d[4] = {z[0], zs(0, kZoneStatusNoTarget, 0), zs(0, kZoneStatusNoTarget, 0), z[3]};
  TEST_ASSERT_TRUE(groundFromZones(e2d).status == RangeStatus::Ok);
  // three or four empty: nothing under the beam
  ZoneSample e3[4] = {z[0], zs(0, kZoneStatusNoTarget), zs(0, kZoneStatusNoTarget), zs(0, kZoneStatusNoTarget)};
  TEST_ASSERT_TRUE(groundFromZones(e3).status == RangeStatus::NoTarget);
  // all bad
  ZoneSample b[4] = {zs(800, 3), zs(800, 4), zs(800, 7), zs(800, 1)};
  TEST_ASSERT_TRUE(groundFromZones(b).status == RangeStatus::Invalid);
}

// The four zones give the floor plane exactly (beam angle below it, sensor
// height), for any walking angle and either mount orientation.
void test_zone_plane_flat_floor() {
  const Params prm = P();
  const float L = prm.tip_to_pod_m;
  for (float alpha = 28.0f; alpha <= 50.0f; alpha += 2.0f) {
    for (int o = 0; o < 2; o++) {
      ZoneSample z[4];
      flatFloorZones(alpha, z, o == 0);
      ZonePlane pl = planeFromZones(z);
      TEST_ASSERT_TRUE(pl.ok);
      TEST_ASSERT_FLOAT_WITHIN(0.3f, alpha - prm.ground_beam_deg, pl.dep_deg);
      TEST_ASSERT_FLOAT_WITHIN(0.005f, L * std::sin(deg2rad(alpha)), pl.h_m);
      RangeReading r = groundFromZones(z);
      TEST_ASSERT_FLOAT_WITHIN(0.01f, pl.dep_deg, r.plane_dep_deg);
      TEST_ASSERT_FLOAT_WITHIN(0.001f, pl.h_m, r.plane_h_m);
    }
  }
}

void test_zone_plane_rejects_and_parallel_floor() {
  ZoneSample z[4];
  flatFloorZones(37.0f, z, true);
  ZonePlane flat = planeFromZones(z);
  // the whole window over a 15 cm drop: same tilt, sensor 15 cm higher above that floor
  const float k = 1.0f + 0.15f / flat.h_m;
  ZoneSample d[4];
  for (int i = 0; i < 4; i++) d[i] = zs((int)std::lround(z[i].mm * k));
  ZonePlane low = planeFromZones(d);
  TEST_ASSERT_TRUE(low.ok);
  TEST_ASSERT_FLOAT_WITHIN(0.2f, flat.dep_deg, low.dep_deg);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, flat.h_m + 0.15f, low.h_m);
  // one zone alone over the edge: not one plane
  ZoneSample e[4] = {z[0], z[1], z[2], zs((int)(z[3].mm * 1.3f))};
  TEST_ASSERT_FALSE(planeFromZones(e).ok);
  TEST_ASSERT_TRUE(std::isnan(groundFromZones(e).plane_dep_deg));
  TEST_ASSERT_TRUE(groundFromZones(e).status == RangeStatus::Ok);   // the range itself still counts
  // an empty zone: no plane
  ZoneSample m[4] = {z[0], z[1], z[2], zs(0, kZoneStatusNoTarget, 0)};
  TEST_ASSERT_FALSE(planeFromZones(m).ok);
}

void test_still_calibration() {
  Params p = P();
  const float L = 0.66f, beta = 20.5f;
  float a[40], dep[40], d[40];
  for (int i = 0; i < 40; i++) {
    a[i] = 40.0f + 0.4f * std::sin(i * 0.7f);                    // hand tremor
    dep[i] = a[i] - beta + 0.5f * std::sin(i * 2.3f);           // laser tilt noise
    float ar = deg2rad(a[i]);
    d[i] = L * std::sin(ar) / std::sin(ar - deg2rad(beta)) + 0.004f * std::sin(i * 1.7f);
  }
  CalResult r = solveStillCalibration(a, dep, d, 40, p);
  TEST_ASSERT_TRUE(r.ok);
  TEST_ASSERT_FLOAT_WITHIN(0.2f, beta, r.beta_deg);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, L, r.L_m);
  TEST_ASSERT_EQUAL_INT(CAL_TOO_FEW, solveStillCalibration(a, dep, d, 5, p).error);
}

// The accelerometer reads the cane angle 5 deg too steep (sweep forces); the floor
// plane must pull the angle back to the truth, and yaw stays 0.
void test_pipeline_angle_from_floor() {
  Params p = P();
  Pipeline pod(p);
  pod.setCalibrated(true);
  const float alpha = 38.0f, acc_alpha = deg2rad(alpha + 5.0f);
  ZoneSample z[4];
  flatFloorZones(alpha, z, false);
  for (int k = 0; k < 250; k++) {
    PipelineInput in;
    in.now_ms = 1000 + k * 20;
    in.imu.ok = true;
    in.imu.acc = Vec3(0, std::sin(acc_alpha), std::cos(acc_alpha)) * kG;
    in.imu.gyro = Vec3(0.3f, -0.2f, 0.5f);                       // no gyro on the pod: ignored
    if (k % 4 == 0) in.ground = groundFromZones(z);
    in.batt_volts = NAN;
    pod.step(in);
  }
  TEST_ASSERT_FLOAT_WITHIN(0.6f, alpha, pod.attitude().alphaDeg());
  TEST_ASSERT_FLOAT_WITHIN(0.6f, alpha + 5.0f, pod.attitude().accelAlphaDeg());
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, pod.attitude().yawOffsetDeg());
  TEST_ASSERT_TRUE(pod.frame().alert == 0);
}

// Cane swept +-23 deg at 0.47 Hz about the grip (0.43 m behind the pod) at 36 deg. The
// sideways acceleration alone must give the side of the sweep; the measured angle stays
// measured and the head-level angle is the fixed trained one.
void test_fixed_angle_accel_sweep_side() {
  Params p = P();
  p.fixed_alpha_deg = 38.0f;
  p.sweep_half_width_deg = 23.0f;
  Attitude att;
  att.configure(p);
  const float a = deg2rad(36.0f), A = deg2rad(23.0f), w = 2 * kPi * 0.47f;
  const float rho = 0.43f * std::cos(a);                     // horizontal radius of the pod
  const Vec3 up(0, std::sin(a), std::cos(a)), ahead(0, -std::cos(a), std::sin(a)), right(-1, 0, 0);
  int n = 0, ok = 0;
  float peak = 0;
  for (int k = 0; k < 600; k++) {                             // 12 s at 50 Hz
    float t = k * 0.02f;
    float psi = A * std::sin(w * t), psi_d = A * w * std::cos(w * t), psi_dd = -w * w * psi;
    ImuSample s;
    s.ok = true;
    s.acc = up * kG + ahead * (-rho * psi_d * psi_d) + right * (rho * psi_dd);
    s.gyro = Vec3();
    att.update(s, 0.02f);
    if (t > 6.0f && std::fabs(psi) > deg2rad(12.0f)) {
      n++;
      ok += (att.yawOffsetDeg() > 0) == (psi > 0);
    }
    if (t > 6.0f) peak = std::fmax(peak, std::fabs(att.yawOffsetDeg()));
  }
  TEST_ASSERT_TRUE(att.sweepKnown());
  TEST_ASSERT_TRUE(ok >= 0.95f * n);
  TEST_ASSERT_FLOAT_WITHIN(5.0f, 23.0f * 1.15f, peak);       // ~half-width at the arc ends
  TEST_ASSERT_EQUAL_FLOAT(38.0f, att.fixedAlphaDeg());       // head-level maths
  TEST_ASSERT_TRUE(std::fabs(att.alphaDeg() - 36.0f) < 3.0f); // measured angle

  // cane held still: no sweep, offset 0 (corridor open, side "ahead"); the sweep size
  // (RMS, 3 s on the square) forgets the sweep in ~10-15 s
  for (int k = 0; k < 1000; k++) {
    ImuSample s;
    s.ok = true;
    s.acc = up * kG;
    att.update(s, 0.02f);
  }
  TEST_ASSERT_FALSE(att.sweepKnown());
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, att.yawOffsetDeg());
}

// The head maths use the fixed angle (never rises), so the pause must follow the measured
// angle (cane raised after finding an edge).
void test_fixed_angle_pause_uses_measured_angle() {
  AlertFilter f;
  f.configure(filterParams());
  RangeReading none;
  uint32_t t = 0;
  for (int i = 0; i < 4; i++) f.update(t += 33, true, 58, 0, dropFrame(38, 0.3f), none, true, 0.0f, NAN);
  TEST_ASSERT_TRUE(f.paused());
  TEST_ASSERT_EQUAL(Hazard::None, f.hazard());
  for (int i = 0; i < 4; i++) f.update(t += 33, true, 38, 0, dropFrame(38, 0.3f), none, true, 0.0f, NAN);
  TEST_ASSERT_FALSE(f.paused());
  TEST_ASSERT_EQUAL(Hazard::Drop, f.hazard());
}

void test_zone_forward_nearest_wins() {
  ZoneSample z[4] = {zs(2400), zs(1300), zs(0, kZoneStatusNoTarget, 0), zs(2600)};
  RangeReading r = forwardFromZones(z);
  TEST_ASSERT_TRUE(r.status == RangeStatus::Ok);
  TEST_ASSERT_EQUAL_UINT16(1300, r.mm);
  ZoneSample thin[4] = {zs(0, kZoneStatusNoTarget, 0), zs(0, kZoneStatusNoTarget, 0), zs(1100),
                        zs(0, kZoneStatusNoTarget, 0)};   // a bar in one zone only
  TEST_ASSERT_EQUAL_UINT16(1100, forwardFromZones(thin).mm);
  ZoneSample none[4] = {zs(0, kZoneStatusNoTarget, 0), zs(0, kZoneStatusNoTarget, 0),
                        zs(0, kZoneStatusNoTarget, 0), zs(0, 4)};
  TEST_ASSERT_TRUE(forwardFromZones(none).status == RangeStatus::NoTarget);
  ZoneSample bad[4] = {zs(900, 4), zs(900, 3), zs(0, kZoneStatusNoTarget, 0), zs(900, 7)};
  TEST_ASSERT_TRUE(forwardFromZones(bad).status == RangeStatus::Invalid);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_flat_floor_reads_zero_drop);
  RUN_TEST(test_15cm_drop_reads_minus_15cm);
  RUN_TEST(test_data_report_numbers);
  RUN_TEST(test_forward_hit_height);
  RUN_TEST(test_attitude_static_angle);
  RUN_TEST(test_drop_needs_two_frames);
  RUN_TEST(test_small_step_ignored);
  RUN_TEST(test_pause_when_cane_vertical);
  RUN_TEST(test_drop_outside_corridor_ignored);
  RUN_TEST(test_head_band_and_side);
  RUN_TEST(test_naive_mode_alerts_on_everything);
  RUN_TEST(test_faults_never_alert);
  RUN_TEST(test_no_return_counts_as_deep_drop);
  RUN_TEST(test_dark_floor_no_return_is_not_an_edge);
  RUN_TEST(test_platform_edge_after_floor_still_alerts);
  RUN_TEST(test_no_forward_sensor_is_not_a_fault);
  RUN_TEST(test_drop_pattern_timing);
  RUN_TEST(test_head_side_plan_a_and_b);
  RUN_TEST(test_battery_report_pulses_and_preemption);
  RUN_TEST(test_battery_table);
  RUN_TEST(test_low_battery_warning_every_5_min);
  RUN_TEST(test_standby_until_five_quick_taps);
  RUN_TEST(test_standby_is_silent_until_started);
  RUN_TEST(test_telemetry_format);
  RUN_TEST(test_cue_sequence_for_dashboard);
  RUN_TEST(test_pipeline_latency_under_100ms);
  RUN_TEST(test_tap_lift_compensated_curb_still_seen);
  RUN_TEST(test_tap_lift_without_compensation_false_alarms);
  RUN_TEST(test_gait_sweep_speed);
  RUN_TEST(test_gait_tap_speed);
  RUN_TEST(test_gait_still_standing_and_hold);
  RUN_TEST(test_gait_calibration_walk);
  RUN_TEST(test_telemetry_has_technique_and_speed);
  RUN_TEST(test_pod_blind_cue_when_ground_sensor_dies);
  RUN_TEST(test_strength_scales_levels_not_rhythm);
  RUN_TEST(test_zone_classify);
  RUN_TEST(test_zone_ground_flat_floor_is_centre_ray);
  RUN_TEST(test_zone_ground_drop_and_empty_rules);
  RUN_TEST(test_zone_forward_nearest_wins);
  RUN_TEST(test_zone_plane_flat_floor);
  RUN_TEST(test_zone_plane_rejects_and_parallel_floor);
  RUN_TEST(test_still_calibration);
  RUN_TEST(test_pipeline_angle_from_floor);
  RUN_TEST(test_fixed_angle_accel_sweep_side);
  RUN_TEST(test_fixed_angle_pause_uses_measured_angle);
  return UNITY_END();
}

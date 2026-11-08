// 2x VL53L5CX multizone ToF (SparkFun library, ST ULD driver inside).
//
// Each sensor runs in 8x8 mode, continuous, TOF_L5_HZ (15 Hz). A beam = the central 2x2 zones,
// turned into one range by lib/saferoute/sensors/tof_zones (ground: harmonic mean of the
// diagonal pairs = the centre ray on any flat floor; forward: nearest zone). The ground
// sensor ranks targets by strength (the floor), the forward one by distance (a thin bar in
// front of a wall).
//
// Addressing: both sensors start at 0x29. Their LPn pins go to the old XSHUT pins
// (GPIO17 ground, GPIO16 forward). LPn low only stops a sensor listening on I2C - it keeps
// its state and keeps ranging - so each one is talked to alone and the ground sensor is
// moved to 0x30. The new address lasts until the sensor loses power. A sensor that is not
// working is kept at LPn low so it can never answer for the other one at 0x29.
//
// Start-up: every begin() uploads ~84 KB of firmware to the sensor: ~1 s per sensor at
// 1 MHz, ~2.5 s at 400 kHz. The upload runs at TOF_L5_LOAD_I2C_HZ and falls back to
// I2C_FREQ_HZ; ranging always runs at I2C_FREQ_HZ.
//
// Watchdog: in this ST driver version, start/stop/init/settings poll the sensor in loops
// that never give up if the bus fails half-way (a loose wire would freeze the pod with the
// motors in whatever state they were). main.cpp enables the loop watchdog (5 s): a freeze
// reboots the pod. An RTC note records which sensor was being loaded; after two such
// reboots in a row that sensor is left off until the pod is power-cycled, so a broken
// sensor cannot reboot the pod forever (a dead ground sensor = pod-blind cue, as before).
#include "hal/hal.h"
#include <SparkFun_VL53L5CX_Library.h>
#include <esp_attr.h>
#include <esp_system.h>
#include "sensors/tof_zones.h"

extern bool loopTaskWDTEnabled;   // Arduino core (main.cpp): set by enableLoopWDT()

namespace hal {
namespace {

constexpr int G = 0, F = 1;
constexpr uint8_t kDefaultAddr = 0x29;
const uint8_t kAddr[2] = {TOF_GND_ADDR, TOF_FWD_ADDR};
const int kLpn[2] = {PIN_TOF_GND_XSHUT, PIN_TOF_FWD_XSHUT};
const char* const kLabel[2] = {"ground", "forward"};
#ifdef I2C_BUFFER_LENGTH
constexpr uint8_t kPacket = I2C_BUFFER_LENGTH >= 128 ? 128 : I2C_BUFFER_LENGTH;   // Wire buffer
#else
constexpr uint8_t kPacket = 32;
#endif

SparkFun_VL53L5CX s_dev[2];
bool s_allocated[2] = {false, false};
bool s_skip[2] = {false, false};
uint32_t s_last_frame[2] = {0, 0};
uint32_t s_next_load[2] = {0, 0};
uint8_t s_fails[2] = {0, 0};       // failed reloads in a row (back-off doubles each time)
void (*s_before_block)() = nullptr; // runtime reloads only: called once a load really starts
VL53L5CX_ResultsData s_res;        // one buffer: the sensors are read one after the other

// survives a watchdog reset (not a power cycle)
constexpr uint32_t kRtcMagic = 0x5AFE15C5u;
RTC_NOINIT_ATTR uint32_t s_rtc_magic;
RTC_NOINIT_ATTR uint32_t s_rtc_loading;   // 0, or 1 + sensor index while a load runs
RTC_NOINIT_ATTR uint32_t s_rtc_hangs[2];

void feed() {
  if (loopTaskWDTEnabled) feedLoopWDT();
}

void lpn(int u, bool on) {
  if (kLpn[u] >= 0) digitalWrite(kLpn[u], on ? HIGH : LOW);
}

bool probe(uint8_t addr) {
  bus0().beginTransmission(addr);
  return bus0().endTransmission() == 0;
}

// SparkFun's begin() news its I2C helper and the ~1.5 KB driver state every time and never
// frees them: free the previous ones so retries do not leak.
void freeDev(int u) {
  if (!s_allocated[u]) return;
  delete s_dev[u].Dev;
  delete s_dev[u].VL53L5CX_i2c;
  s_dev[u].Dev = nullptr;
  s_dev[u].VL53L5CX_i2c = nullptr;
  s_allocated[u] = false;
}

void bootCheck() {
  static bool done = false;
  if (done) return;
  done = true;
  esp_reset_reason_t why = esp_reset_reason();
  if (s_rtc_magic != kRtcMagic || why == ESP_RST_POWERON || why == ESP_RST_BROWNOUT) {
    s_rtc_magic = kRtcMagic;               // power cycle: every sensor gets a fresh try
    s_rtc_hangs[G] = s_rtc_hangs[F] = 0;
  } else if (why == ESP_RST_TASK_WDT && (s_rtc_loading == 1 || s_rtc_loading == 2)) {
    int u = (int)s_rtc_loading - 1;
    s_rtc_hangs[u]++;
    Serial.printf("# ToF %s sensor froze the pod while loading (%lu time(s) in a row)\n", kLabel[u],
                  (unsigned long)s_rtc_hangs[u]);
  }
  s_rtc_loading = 0;
  for (int u = 0; u < 2; u++) {
    s_skip[u] = s_rtc_hangs[u] >= 2;
    if (s_skip[u]) Serial.printf("# ToF %s sensor left OFF: check its wiring, then power-cycle the pod\n", kLabel[u]);
  }
}

bool configure(int u, uint8_t from) {
  SparkFun_VL53L5CX& d = s_dev[u];
  d.setWireMaxPacketSize(kPacket);
  if (from != kAddr[u]) {
    if (!d.setAddress(kAddr[u])) return false;
    if (!probe(kAddr[u])) return false;
  }
  return d.setResolution(sr::kZoneResolution) &&                      // 8x8, before the frequency
         d.setRangingFrequency(TOF_L5_HZ) &&                          // 8x8: up to 15 Hz
         d.setRangingMode(SF_VL53L5CX_RANGING_MODE::CONTINUOUS) &&     // longest integration
         d.setTargetOrder(u == F ? SF_VL53L5CX_TARGET_ORDER::CLOSEST : SF_VL53L5CX_TARGET_ORDER::STRONGEST) &&
         d.startRanging();
}

// Full bring-up of one sensor: firmware upload, address, settings, start. The other
// sensor must be at LPn low (0x29 must mean this one).
uint8_t findAddr(int u) {
  const uint8_t want = kAddr[u];
  return probe(want) ? want : (want != kDefaultAddr && probe(kDefaultAddr)) ? kDefaultAddr : 0;
}

bool load(int u) {
  uint8_t from = findAddr(u);
  if (!from) return false;                 // not answering: cheap, nothing to load
  if (s_before_block) {
    Serial.printf("# ToF %s sensor: reloading\n", kLabel[u]);
    s_before_block();
  }
  s_rtc_loading = (uint32_t)u + 1;
  bool ok = false;
  const uint32_t clocks[2] = {TOF_L5_LOAD_I2C_HZ, I2C_FREQ_HZ};
  for (int i = 0; i < 2 && !ok; i++) {
    if (i == 1) {
      if (clocks[1] == clocks[0]) break;
      bus0().setClock(I2C_FREQ_HZ);
      from = findAddr(u);                  // in case the failed attempt left it at 0x29
      if (!from) break;
    }
    feed();
    freeDev(u);
    bus0().setClock(clocks[i]);
    ok = s_dev[u].begin(from, bus0());
    s_allocated[u] = true;
  }
  bus0().setClock(I2C_FREQ_HZ);
  feed();
  if (ok) ok = configure(u, from);
  feed();
  s_rtc_loading = 0;
  if (ok) {
    s_rtc_hangs[u] = 0;
    s_last_frame[u] = millis();
  }
  return ok;
}

// (Re)load sensor u alone on the bus; afterwards only working sensors listen.
bool bringUp(int u, bool& ok_u, bool other_ok) {
  const int other = 1 - u;
  lpn(other, false);
  lpn(u, true);
  delay(2);
  ok_u = !s_skip[u] && load(u);
  lpn(u, ok_u);
  lpn(other, other_ok);
  return ok_u;
}

bool alive(int u) { return probe(kAddr[u]) && s_dev[u].isConnected(); }

sr::RangeReading readUnit(int u, bool ok) {
  using sr::RangeStatus;
  if (!ok) return sr::RangeReading(RangeStatus::Fault, 0);
  SparkFun_VL53L5CX& d = s_dev[u];
  bool ready = d.isDataReady();
  if (d.lastError.lastErrorCode != SF_VL53L5CX_ERROR_TYPE::NO_ERROR) return sr::RangeReading(RangeStatus::Fault, 0);
  if (!ready) return sr::RangeReading();
  if (!d.getRangingData(&s_res)) return sr::RangeReading(RangeStatus::Fault, 0);
  s_last_frame[u] = millis();
  sr::ZoneSample z[4];
  for (int i = 0; i < 4; i++) {
    const uint8_t zone = sr::kCentreZones[i];
    const int t = zone * VL53L5CX_NB_TARGET_PER_ZONE;   // first (only) target of the zone
    z[i].mm = s_res.distance_mm[t];
    z[i].status = s_res.target_status[t];
    z[i].targets = s_res.nb_target_detected[zone];
  }
  return u == G ? sr::groundFromZones(z) : sr::forwardFromZones(z);
}

}  // namespace

bool Tof::begin() {
  bootCheck();
  pinMode(PIN_TOF_GND_XSHUT, OUTPUT);
  pinMode(PIN_TOF_FWD_XSHUT, OUTPUT);
  lpn(G, false);
  lpn(F, false);
  delay(5);
  groundOk = forwardOk = false;
  bringUp(G, groundOk, false);             // ground first, alone: 0x29 -> 0x30
#if FEATURE_FORWARD_TOF
  bringUp(F, forwardOk, groundOk);         // forward stays at 0x29
  return groundOk && forwardOk;
#else
  return groundOk;                         // forward not fitted: kept at LPn low
#endif
}

// Called by main.cpp when the pipeline reports a sensor fault (at most every TOF_REINIT_MS).
// A full reload blocks the whole loop (both beams, motors) for 1-2.5 s, so it is the last
// resort: a sensor that still answers and delivered a frame in the last TOF_L5_STALL_MS had a
// passing I2C error and is left alone. A sensor that is not on the bus costs only a probe.
// Reload attempts are spaced TOF_L5_RELOAD_MS, doubling after each failure (up to 16x), so a
// sensor that answers but will not load cannot keep blinding the other beam.
bool Tof::reinit(bool ground, bool forward) {
  bool all = true;
  for (int u = 0; u < 2; u++) {
    if (!(u == G ? ground : (forward && FEATURE_FORWARD_TOF))) continue;
    bool& ok = u == G ? groundOk : forwardOk;
    const bool other_ok = u == G ? forwardOk : groundOk;
    const uint32_t now = millis();
    if (s_skip[u]) { all = false; continue; }
    if (ok && now - s_last_frame[u] < TOF_L5_STALL_MS && alive(u)) continue;
    if ((int32_t)(now - s_next_load[u]) < 0) { all = false; continue; }
    s_before_block = onLongBlock ? onLongBlock : +[] {};
    const bool up = bringUp(u, ok, other_ok);
    s_before_block = nullptr;
    s_fails[u] = up ? 0 : (uint8_t)(s_fails[u] < 4 ? s_fails[u] + 1 : 4);
    s_next_load[u] = millis() + ((uint32_t)TOF_L5_RELOAD_MS << s_fails[u]);
    all &= up;
  }
  return all;
}

void Tof::poll(sr::RangeReading& ground, sr::RangeReading& forward) {
  ground = readUnit(G, groundOk);
  forward = FEATURE_FORWARD_TOF ? readUnit(F, forwardOk) : sr::RangeReading();
}

void Tof::shutdown() {
  // LPn low alone would leave both ranging (~100 mA each) through deep sleep: stop them and
  // put them in low-power sleep first. The next begin() wakes them (its reboot sequence).
  for (int u = 0; u < 2; u++) {
    bool ok = u == G ? groundOk : forwardOk;
    if (!ok) continue;
    s_dev[u].stopRanging();
    s_dev[u].setPowerMode(SF_VL53L5CX_POWER_MODE::SLEEP);
  }
  lpn(G, false);
  lpn(F, false);
  groundOk = forwardOk = false;
}

const char* Tof::name() const { return "VL53L5CX x2"; }

}  // namespace hal

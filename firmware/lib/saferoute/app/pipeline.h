// The whole pod logic in one object, with no hardware inside. The ESP32 main
// loop feeds it sensor readings; the unit tests (test/) feed it scripted readings.
// Same code on the pod and on the laptop.
//
//   IMU ----> Attitude (alpha, yaw offset) --+
//   ToF ground/forward ---------------------+--> AlertFilter --> HapticEngine --> motors
//   tip taps (IMU) --> standby until start_taps quick taps, then alerts on
//   battery voltage --> BatteryMonitor --> low-battery warning / shutdown
#pragma once
#include "core/params.h"
#include "core/types.h"
#include "filter/alert_filter.h"
#include "haptics/haptic_engine.h"
#include "kinematics/attitude.h"
#include "kinematics/gait.h"
#include "kinematics/geometry.h"
#include "power/battery.h"
#include "telemetry/telemetry.h"

namespace sr {

struct PipelineInput {
  uint32_t now_ms;
  ImuSample imu;
  RangeReading ground;     // status None when there is no new frame this tick
  RangeReading forward;
  float batt_volts;        // NAN on ticks without a battery reading
};

enum class Command : uint8_t {
  Calibrate, BatteryReport, ToggleMode, TogglePlan, Standby,
  TestLeft, TestRight, TestBoth, WalkCalStart,
  Start,   // leave standby without the start taps (button click, serial `on`)
};

class Pipeline {
 public:
  explicit Pipeline(const Params& p = Params::defaults());
  void begin(uint32_t now_ms);               // plays the boot tick
  void step(const PipelineInput& in);
  void command(Command c, uint32_t now_ms);

  // configuration
  Params& params() { return p_; }
  void applyParams();                        // after changing params()
  void setMode(Mode m);
  void setPlan(char plan);
  void setCalibrated(bool c) { calibrated_ = c; }
  bool calibrated() const { return calibrated_; }
  void setStrength(uint8_t s) { p_.haptic_strength = s; haptics_.setStrength(s); haptics_.play(Pattern::Ack, last_ms_); }

  // outputs
  const TelemetryFrame& frame() const { return frame_; }
  uint8_t hapticLeft() const { return haptics_.left(); }
  uint8_t hapticRight() const { return haptics_.right(); }
  DeviceState state() const { return state_; }
  Hazard hazard() const { return filter_.hazard(); }
  const Attitude& attitude() const { return att_; }
  const AlertFilter& filter() const { return filter_; }
  const BatteryMonitor& battery() const { return batt_; }
  bool calibrating() const { return calibrating_; }
  bool groundFault() const { return filter_.groundFault(); }
  bool forwardFault() const { return filter_.forwardFault(); }
  bool imuFault() const { return imu_fault_; }
  bool tapping() const { return tapping_; }
  bool active() const { return active_; }               // false = standby (waiting for the start taps)
  uint8_t startTaps() const { return start_taps_n_; }   // start taps counted so far in standby
  const Gait& gait() const { return gait_; }
  Hazard alertOut() const { return hazard_out_; }      // filter hazard, with Drop -> DropUrgent

  // Calibration walk (serial `walk start`, walk a measured distance, `walk <metres>`).
  // On success the stride is in params() and takeStride() returns true once (to save it).
  bool finishWalkCal(float distance_m, uint32_t now);
  bool takeStride() { bool r = stride_ready_; stride_ready_ = false; return r; }

  // Session statistics (serial `stat`): real-world numbers for the poster.
  struct Stats {
    uint32_t ms_by_tech[5];   // indexed by Technique
    float walked_m;           // integrated estimated speed
    float speed_sum, speed_n; // mean speed while walking
    uint16_t drop_alerts, urgent_alerts, head_alerts, slow_down_cues, pod_blind_cues;
  };
  const Stats& stats() const { return stats_; }
  float tipLift() const { return tip_lift_; }
  float groundAlphaDeg() const { return a_ground_dbg_; }   // no gyro: last trusted laser angle (NAN = none)

  // one-shot flags for the main loop
  bool takeCalibration(CalResult* out);      // true once after a successful calibration
  bool powerOffRequested() const { return off_requested_ && !haptics_.busy(); }
  const CalResult& lastCalibration() const { return last_cal_; }

 private:
  static const int kMaxCal = 200;
  static constexpr float kStillMps2 = 0.25f;   // no-gyro calibration: "at rest" (accelerometer)
  void startCalibration(uint32_t now);
  void finishCalibration(uint32_t now);
  void updateState();

  Params p_;
  Attitude att_;
  AlertFilter filter_;
  HapticEngine haptics_;
  BatteryMonitor batt_;
  TelemetryFrame frame_;
  DeviceState state_ = DeviceState::Boot;

  uint32_t last_ms_ = 0, imu_last_ok_ = 0;
  bool started_ = false, imu_fault_ = true, calibrated_ = false;
  bool off_requested_ = false;

  // tap to start: standby until start_taps impacts, each within start_tap_gap_ms of the last
  static const uint32_t kStartTapRefractoryMs = 250;   // one tap rings for a few samples
  bool active_ = false;
  uint8_t start_taps_n_ = 0;
  uint32_t start_tap_last_ = 0;

  // two-point tapping: impacts and tip-lift compensation
  float amag_mean_ = 0;
  uint32_t last_impact_ = 0, prev_impact_ = 0;
  bool have_impact_ = false, tapping_ = false;
  float sin_alpha_tap_ = 0, tip_lift_ = 0;

  // technique, speed, urgency
  Gait gait_;
  Hazard hazard_out_ = Hazard::None, prev_hazard_ = Hazard::None;
  bool was_paused_ = false;            // pause_end_ack_ms: the pause that just ended
  uint32_t paused_since_ = 0;
  bool urgent_ = false, stride_ready_ = false;
  uint32_t fast_since_ = 0, last_slow_cue_ = 0;
  bool fast_ = false, slow_cued_ = false;
  uint32_t blind_since_ = 0, last_blind_cue_ = 0;
  bool blind_ = false, blind_cued_ = false;
  Stats stats_ = {};

  bool calibrating_ = false;
  uint32_t cal_end_ = 0;
  float cal_alpha_[kMaxCal], cal_d_[kMaxCal], cal_dep_[kMaxCal];
  Vec3 still_acc_;
  uint32_t anchor_ms_ = 0, last_tilt_ms_ = 0;
  float a_tap_ref_ = -90.0f, last_tilt_ = 0, a_ground_dbg_ = NAN;
  static const uint32_t kTapRefMs = 300;    // no gyro: tap reference = steepest laser angle this soon after the impact
  static const uint32_t kAnchorMs = 1500;   // continuity frames only this soon after a tip-on-floor frame
  float still_dev_ = 0;
  int cal_n_ = 0;
  CalResult last_cal_ = {};
  bool cal_ready_ = false;
};

}  // namespace sr

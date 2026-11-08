#include "app/pipeline.h"

namespace sr {

Pipeline::Pipeline(const Params& p) : p_(p) {
  applyParams();
  frame_ = TelemetryFrame();
  frame_.dh_m = frame_.fwd_h_m = NAN;
  frame_.dg_mm = frame_.df_mm = -1;
  frame_.batt_pct = -1;
  frame_.speed_mps = NAN;
  frame_.tech = Technique::Unknown;
  frame_.ground_q_pct = -1;
}

void Pipeline::applyParams() {
  att_.configure(p_);
  Mode m = filter_.mode();
  filter_.configure(p_);
  filter_.setMode(m);
  haptics_.configure(p_);
  batt_.configure(p_);
  gait_.configure(p_);
  still_acc_ = Vec3();                 // calibration stillness: start over
  still_dev_ = 0;
  was_paused_ = false;
}

void Pipeline::begin(uint32_t now) {
  haptics_.play(Pattern::Boot, now);
  last_ms_ = now;
  started_ = true;
  active_ = p_.start_taps == 0;
  start_taps_n_ = 0;
}

void Pipeline::setMode(Mode m) { filter_.setMode(m); }
void Pipeline::setPlan(char plan) { p_.haptic_plan = plan; haptics_.setPlan(plan); }

void Pipeline::command(Command c, uint32_t now) {
  switch (c) {
    case Command::Calibrate: startCalibration(now); break;
    case Command::BatteryReport: haptics_.play(Pattern::Battery, now, batt_.reportPulses()); break;
    case Command::ToggleMode: {
      Mode m = filter_.mode() == Mode::Smart ? Mode::Naive : Mode::Smart;
      filter_.setMode(m);
      haptics_.play(Pattern::Mode, now, m == Mode::Smart ? 2 : 3);
      break;
    }
    case Command::TogglePlan: setPlan(p_.haptic_plan == 'A' ? 'B' : 'A'); haptics_.play(Pattern::Ack, now); break;
    case Command::Standby:
      active_ = false;
      start_taps_n_ = 0;
      haptics_.setHazard(Hazard::None, now);
      haptics_.play(Pattern::Boot, now);    // soft tick: "standing by"
      break;
    case Command::Start:   // left-right ring every time: "working"
      haptics_.play(Pattern::Ack, now);
      active_ = true;
      start_taps_n_ = 0;
      break;
    case Command::TestLeft: haptics_.play(Pattern::Test, now, 1); break;
    case Command::TestRight: haptics_.play(Pattern::Test, now, 2); break;
    case Command::TestBoth: haptics_.play(Pattern::Test, now, 3); break;
    case Command::WalkCalStart: gait_.startCalWalk(); haptics_.play(Pattern::Boot, now); break;
  }
}

void Pipeline::startCalibration(uint32_t now) {
  calibrating_ = true;
  cal_n_ = 0;
  cal_end_ = now + p_.cal_duration_ms;
  haptics_.setHazard(Hazard::None, now);
}

void Pipeline::finishCalibration(uint32_t now) {
  calibrating_ = false;
  last_cal_ = solveStillCalibration(cal_alpha_, cal_dep_, cal_d_, cal_n_, p_);
  if (last_cal_.ok) {
    p_.tip_to_pod_m = last_cal_.L_m;
    p_.ground_beam_deg = last_cal_.beta_deg;
    Mode m = filter_.mode();
    filter_.configure(p_);
    filter_.setMode(m);
    calibrated_ = true;
    cal_ready_ = true;
    haptics_.play(Pattern::Ack, now);
  } else {
    haptics_.play(Pattern::Error, now);
  }
}

bool Pipeline::finishWalkCal(float distance_m, uint32_t now) {
  Technique t;
  float s;
  if (!gait_.finishCalWalk(distance_m, &t, &s)) {
    haptics_.play(Pattern::Error, now);
    return false;
  }
  if (t == Technique::Tap) p_.stride_tap_m = s;
  else p_.stride_sweep_m = s;
  stride_ready_ = true;
  haptics_.play(Pattern::Ack, now);
  return true;
}

bool Pipeline::takeCalibration(CalResult* out) {
  if (!cal_ready_) return false;
  cal_ready_ = false;
  if (out) *out = last_cal_;
  return true;
}

void Pipeline::step(const PipelineInput& in) {
  const uint32_t now = in.now_ms;
  if (!started_) begin(now);
  float dt = (now - last_ms_) * 0.001f;
  if (dt <= 0.0f || dt > 0.1f) dt = 1.0f / p_.loop_hz;
  last_ms_ = now;

  // attitude
  if (in.imu.ok) {
    att_.update(in.imu, dt);
    imu_last_ok_ = now;
    // stillness for the no-gyro calibration: how far the accelerometer strays from its
    // 0.3 s mean (peak-following, decays over ~0.5 s)
    if (still_acc_.norm() == 0.0f) still_acc_ = in.imu.acc;
    still_acc_ = still_acc_ + (in.imu.acc - still_acc_) * (dt / (0.3f + dt));
    float dev = (in.imu.acc - still_acc_).norm();
    still_dev_ = dev > still_dev_ ? dev : still_dev_ + (dev - still_dev_) * (dt / (0.5f + dt));
  }
  imu_fault_ = !att_.ready() || now - imu_last_ok_ > p_.imu_stale_ms;
  const bool att_ok = !imu_fault_ && att_.settled();   // no alerts until the filter has converged

  // no-gyro pods: the cane angle from the floor plane the ground laser sees (the plane's
  // tilt is the true cane angle on a flat floor, with the tip up too). A frame is trusted
  //  - "anchor": its tilt and its height both say "tip on this floor" (a drop, a step, a
  //    lifted tip or a wall moves the height, not the tilt), or
  //  - "continuity" (two-point tapping only, tip in the air): within 1.5 s of an anchor, its
  //    tilt is within tof_att_cont_deg of the anchor angle. Not used otherwise: an edge
  //    partly inside the window bends the tilt by 4-10 deg, which this check cannot catch.
  // A trusted frame's tilt is used for its own drop maths and corrects the accelerometer
  // angle that carries alpha between frames.
  const bool plane = !std::isnan(in.ground.plane_dep_deg) && in.ground.status == RangeStatus::Ok;
  float a_ground = NAN;
  if (plane && att_.ready() && !calibrating_) {
    const float a_tilt = p_.ground_beam_deg + in.ground.plane_dep_deg;
    const float s = in.ground.plane_h_m / p_.tip_to_pod_m;
    const float a_h = s > 0.0f && s < 1.0f ? rad2deg(std::asin(s)) : NAN;
    if (std::fabs(a_tilt - a_h) < p_.tof_att_gate_deg) {          // tip on this floor
      att_.floorFix(0.5f * (a_tilt + a_h), p_.tof_att_gain, true);
      anchor_ms_ = now;
      a_ground = a_tilt;
    } else if (tapping_ && att_.floorKnown() && now - anchor_ms_ < kAnchorMs &&
               std::fabs(a_tilt - att_.anchorAlphaDeg()) < p_.tof_att_cont_deg) {
      att_.floorFix(a_tilt, p_.tof_att_gain, false);              // tip up, same floor plane
      a_ground = a_tilt;
    }
    if (!std::isnan(a_ground)) {
      last_tilt_ = a_ground;
      last_tilt_ms_ = now;
      // two-point tapping reference pose: lifting the tip lowers the cane angle (it pivots
      // in the hand), so the tip-down pose is the steepest trusted angle around the impact
      // (an accelerometer angle at the impact itself is spoiled by the impact)
      if (have_impact_ && now - last_impact_ <= kTapRefMs && a_ground > a_tap_ref_) {
        a_tap_ref_ = a_ground;
        sin_alpha_tap_ = std::sin(deg2rad(a_ground));
      }
    }
  }
  const float a_tilt = a_ground;
  if (in.ground.fresh()) a_ground_dbg_ = a_ground;

  // calibration: collect flat-floor ground frames while the cane is held still (tip on the
  // floor). At rest the accelerometer is an exact inclinometer.
  if (calibrating_) {
    if (plane && att_.ready() && still_dev_ < kStillMps2 && cal_n_ < kMaxCal) {
      cal_alpha_[cal_n_] = att_.accelAlphaDeg();
      cal_dep_[cal_n_] = in.ground.plane_dep_deg;
      cal_d_[cal_n_] = in.ground.mm * 0.001f;
      cal_n_++;
    }
    if ((int32_t)(now - cal_end_) >= 0) finishCalibration(now);
  }

  // two-point tapping: find tip impacts (tip on the floor). Between taps the cane
  // pivots in the hand, so the tip height follows from the cane angle:
  //   lift = (L + grip_to_pod) * (sin(alpha at the last tap) - sin(alpha now))
  // Compensate mode adds it back to dh; gate mode skips frames while the tip is up.
  bool tip_down = true;
  float sin_a = std::sin(deg2rad(att_.alphaDeg()));
  if (in.imu.ok) {
    float m = in.imu.acc.norm();
    bool impact = false;
    amag_mean_ = amag_mean_ == 0 ? m : amag_mean_ + (m - amag_mean_) * (dt / 0.5f);
    if (m - amag_mean_ > p_.tap_impact_mps2 && (!have_impact_ || now - last_impact_ > 250)) {
      prev_impact_ = have_impact_ ? last_impact_ : now - 100000;
      last_impact_ = now;
      have_impact_ = true;
      // reference pose: the laser frame just before the impact, if any
      a_tap_ref_ = now - last_tilt_ms_ <= kTapRefMs / 2 ? last_tilt_ : -90.0f;
      if (a_tap_ref_ > -90.0f) sin_alpha_tap_ = std::sin(deg2rad(a_tap_ref_));
      impact = true;
    }
    // standby: start_taps quick tip taps on the floor switch the alerts on (own, lower
    // threshold: a coarse accelerometer clips the tap spike)
    if (!active_ && m - amag_mean_ > p_.start_tap_mps2 && now - start_tap_last_ > kStartTapRefractoryMs) {
      start_taps_n_ = start_taps_n_ && now - start_tap_last_ <= p_.start_tap_gap_ms ? start_taps_n_ + 1 : 1;
      start_tap_last_ = now;
      if (start_taps_n_ >= p_.start_taps) {
        active_ = true;
        start_taps_n_ = 0;
        haptics_.play(Pattern::Ack, now);
      }
    }
    if (!active_ && start_taps_n_ && now - start_tap_last_ > p_.start_tap_gap_ms) start_taps_n_ = 0;
    // technique (still / sweep / tap / hold) and walking speed
    gait_.update(now, dt, m, 0.0f, impact);       // no gyro: no yaw rate
  }
  const Technique tech = gait_.technique();
  filter_.setDropThreshold(tech == Technique::Tap ? p_.tap_drop_threshold_m : p_.drop_threshold_m);
  tapping_ = p_.tap_mode != 0 && have_impact_ && now - prev_impact_ <= p_.tap_detect_window_ms &&
             now - last_impact_ <= p_.tap_detect_window_ms;
  tip_lift_ = 0;
  if (tapping_ && p_.tap_mode == 1) {
    float s_now = std::isnan(a_tilt) ? sin_a : std::sin(deg2rad(a_tilt));
    float lift = (p_.tip_to_pod_m + p_.grip_to_pod_m) * (sin_alpha_tap_ - s_now);
    tip_lift_ = clampf(lift, 0.0f, p_.tap_max_lift_m);
  } else if (tapping_ && p_.tap_mode == 2) {
    tip_down = now - last_impact_ <= p_.tap_contact_ms;
  }
  // tapping: a ground frame without a trusted laser angle has only the accelerometer's
  // angle, which a tap swing spoils - skip it (neither counts nor breaks a run)
  if (tapping_ && tech == Technique::Tap && in.ground.fresh() && std::isnan(a_ground))
    tip_down = false;

  // alerts
  filter_.update(now, att_ok, att_.alphaDeg(), att_.yawOffsetDeg(), in.ground, in.forward, tip_down,
                 tip_lift_, a_tilt);

  // low-vision urgency: when a drop is confirmed, time to the edge at the current
  // speed = how far past the tip the beam is / speed. Short -> urgent pattern.
  Hazard h = filter_.hazard();
  const float v = gait_.speedMps();
  if (h == Hazard::Drop && prev_hazard_ != Hazard::Drop) {
    urgent_ = false;
    if (!std::isnan(v) && v > 0.2f) {
      // the edge lies where the beam meets the walking floor: the look-ahead past the tip
      float a = deg2rad(att_.alphaDeg()), beta = deg2rad(p_.ground_beam_deg);
      float d_floor = expectedGroundRange(p_.tip_to_pod_m, beta, a);
      if (d_floor > 0) {
        float ahead = d_floor * std::cos(a - beta) - p_.tip_to_pod_m * std::cos(a);
        urgent_ = ahead / v < p_.urgent_time_s;
      }
    }
    stats_.drop_alerts++;
    if (urgent_) stats_.urgent_alerts++;
  }
  if (h != Hazard::None && h != Hazard::Drop && (prev_hazard_ == Hazard::None || prev_hazard_ == Hazard::Drop))
    stats_.head_alerts++;
  prev_hazard_ = h;
  hazard_out_ = (h == Hazard::Drop && urgent_) ? Hazard::DropUrgent : h;
  if (!calibrating_ && !off_requested_) haptics_.setHazard(active_ ? hazard_out_ : Hazard::None, now);

  // cane back down after a long pause (stair mode: the stairs are done) -> one ack ring
  const bool paused_now = filter_.paused();
  if (paused_now && !was_paused_) paused_since_ = now;
  if (!paused_now && was_paused_ && p_.pause_end_ack_ms && active_ && !calibrating_ &&
      now - paused_since_ >= p_.pause_end_ack_ms)
    haptics_.play(Pattern::Ack, now);
  was_paused_ = paused_now;

  // walking faster than the look-ahead covers: one soft cue, at most once a minute
  bool fast = active_ && filter_.mode() == Mode::Smart && gait_.walking() && !std::isnan(v) && v > p_.speed_warn_mps;
  if (fast && !fast_) fast_since_ = now;
  fast_ = fast;
  if (fast && now - fast_since_ >= p_.speed_warn_hold_ms && h == Hazard::None &&
      (!slow_cued_ || now - last_slow_cue_ >= p_.speed_warn_repeat_ms)) {
    haptics_.play(Pattern::SlowDown, now);
    last_slow_cue_ = now;
    slow_cued_ = true;
    stats_.slow_down_cues++;
  }

  // "pod blind": the ground ToF or the IMU is down, so drop-offs can't be detected.
  // The user must know they are on their own: one soft long pulse, then every minute.
  bool blind = started_ && active_ && !calibrating_ && (imu_fault_ || filter_.groundFault()) && now > 5000;
  if (blind && !blind_) blind_since_ = now;
  blind_ = blind;
  if (!blind) blind_cued_ = false;
  if (blind && now - blind_since_ >= p_.pod_blind_after_ms &&
      (!blind_cued_ || now - last_blind_cue_ >= p_.pod_blind_repeat_ms)) {
    haptics_.play(Pattern::PodBlind, now);
    last_blind_cue_ = now;
    blind_cued_ = true;
    stats_.pod_blind_cues++;
  }

  // session statistics
  stats_.ms_by_tech[(int)tech < 5 ? (int)tech : 0] += (uint32_t)(dt * 1000);
  if (gait_.walking() && !std::isnan(v) && v > 0) {
    stats_.walked_m += v * dt;
    stats_.speed_sum += v;
    stats_.speed_n += 1;
  }

  // battery
  if (!std::isnan(in.batt_volts)) {
    batt_.update(in.batt_volts, now);
    if (batt_.lowWarningDue(now)) haptics_.play(Pattern::LowBattery, now);
    if (batt_.critical() && !off_requested_) {
      haptics_.play(Pattern::LowBattery, now);
      off_requested_ = true;
    }
  }

  haptics_.tick(now);
  updateState();

  // telemetry
  frame_.t_ms = now;
  frame_.alpha_deg = att_.alphaDeg();
  frame_.yaw_deg = 0;                            // no gyro: no heading
  frame_.yaw_off_deg = att_.yawOffsetDeg();
  if (in.ground.fresh()) frame_.dg_mm = in.ground.status == RangeStatus::Ok ? in.ground.mm : -1;
  if (in.forward.fresh()) frame_.df_mm = in.forward.status == RangeStatus::Ok ? in.forward.mm : -1;
  if (in.ground.fresh()) frame_.dh_m = filter_.lastDeltaH();
  if (in.forward.fresh()) frame_.fwd_h_m = filter_.lastForwardHeight();
  frame_.alert = (uint8_t)hazard_out_;
  frame_.tech = tech;
  frame_.speed_mps = v;
  if (!filter_.groundFault()) frame_.ground_q_pct = (int16_t)std::lround(100.0f * (1.0f - filter_.groundEmptyShare()));
  frame_.batt_pct = batt_.known() ? batt_.percent() : -1;
  frame_.mode = filter_.mode();
  frame_.state = state_;
  frame_.hl = haptics_.left();
  frame_.hr = haptics_.right();
  frame_.cue = (uint8_t)haptics_.lastCue();
  frame_.cue_n = haptics_.lastCueCount();
  frame_.cue_seq = haptics_.cueSeq();
  frame_.plan = p_.haptic_plan;
  frame_.head_buzz_ms = p_.head_buzz_ms;
}

void Pipeline::updateState() {
  if (off_requested_) state_ = DeviceState::Off;
  else if (calibrating_) state_ = DeviceState::Calibrating;
  else if (!active_) state_ = DeviceState::Standby;
  else if (!imu_fault_ && !att_.settled()) state_ = DeviceState::Boot;
  else if (hazard_out_ != Hazard::None) state_ = DeviceState::Alert;
  else if (imu_fault_ || filter_.groundFault() || filter_.forwardFault()) state_ = DeviceState::Fault;
  else if (filter_.paused()) state_ = DeviceState::Paused;
  else if (batt_.low()) state_ = DeviceState::LowBattery;
  else if (!calibrated_) state_ = DeviceState::NeedsCal;
  else state_ = DeviceState::Ok;
}

}  // namespace sr

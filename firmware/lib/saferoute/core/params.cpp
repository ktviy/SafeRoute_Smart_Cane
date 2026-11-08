#include "core/params.h"
#include "config.h"

namespace sr {

Params Params::defaults() {
  Params p;
  p.shaft = Vec3(IMU_SHAFT_AXIS_X, IMU_SHAFT_AXIS_Y, IMU_SHAFT_AXIS_Z).normalized();
  p.tip_to_pod_m = POD_TIP_TO_POD_M;
  p.grip_to_pod_m = POD_GRIP_TO_POD_M;
  p.ground_beam_deg = GROUND_BEAM_DEG;
  p.forward_beam_deg = FORWARD_BEAM_DEG;
  p.head_height_m = USER_HEAD_HEIGHT_M;

  p.loop_hz = LOOP_HZ;
  p.acc_only_tau_s = ACC_ONLY_TAU_S;
  p.tof_att_gate_deg = TOF_ATT_GATE_DEG;
  p.tof_att_cont_deg = TOF_ATT_CONT_DEG;
  p.tof_att_gain = TOF_ATT_GAIN;
  p.fixed_alpha_deg = FIXED_ALPHA_DEG;
  p.sweep_half_width_deg = SWEEP_HALF_WIDTH_DEG;
  p.acc_sweep_g_tau_s = ACC_SWEEP_G_TAU_S;
  p.acc_sweep_lp_tau_s = ACC_SWEEP_LP_TAU_S;
  p.acc_sweep_rms_s = ACC_SWEEP_RMS_S;
  p.acc_sweep_min_mps2 = ACC_SWEEP_MIN_MPS2;

  p.forward_enabled = FEATURE_FORWARD_TOF != 0;
  p.drop_threshold_m = DROP_THRESHOLD_M;
  p.n_confirm_ground = N_CONFIRM_GROUND;
  p.n_confirm_forward = N_CONFIRM_FORWARD;
  p.no_return_is_drop = NO_RETURN_IS_DROP != 0;
  p.no_return_extra = NO_RETURN_EXTRA;
  p.no_return_max_frac = NO_RETURN_MAX_FRAC;
  p.ground_corridor_deg = GROUND_CORRIDOR_DEG;
  p.forward_corridor_deg = FORWARD_CORRIDOR_DEG;
  p.forward_corridor_m = FORWARD_CORRIDOR_M;
  p.side_deadband_m = SIDE_DEADBAND_M;
  p.head_min_m = HEAD_MIN_HEIGHT_M;
  p.head_margin_m = HEAD_MARGIN_M;
  p.forward_max_x_m = FORWARD_MAX_X_M;
  p.pause_on_deg = PAUSE_ON_DEG;
  p.pause_off_deg = PAUSE_OFF_DEG;
  p.pause_end_ack_ms = PAUSE_END_ACK_MS;
  p.alpha_min_deg = ALPHA_MIN_DEG;
  p.tof_max_range_m = TOF_MAX_RANGE_M;
  p.alert_hold_ms = ALERT_HOLD_MS;
  p.naive_fwd_range_m = NAIVE_FWD_RANGE_M;
  p.naive_hold_ms = NAIVE_ALERT_HOLD_MS;
  p.tap_mode = TAP_MODE;
  p.tap_impact_mps2 = TAP_IMPACT_MPS2;
  p.tap_max_lift_m = TAP_MAX_LIFT_M;
  p.tap_contact_ms = TAP_CONTACT_MS;
  p.tap_detect_window_ms = TAP_DETECT_WINDOW_MS;
  p.sweep_yaw_rms_dps = SWEEP_YAW_RMS_DPS;
  p.walk_bounce_on = WALK_BOUNCE_ON_MPS2;
  p.walk_bounce_off = WALK_BOUNCE_OFF_MPS2;
  p.arc_turn_deg = ARC_TURN_DEG;
  p.tap_min_interval_ms = TAP_MIN_INTERVAL_MS;
  p.tap_max_interval_ms = TAP_MAX_INTERVAL_MS;
  p.tap_arc_tol_ms = TAP_ARC_TOL_MS;
  p.technique_hold_ms = TECHNIQUE_HOLD_MS;
  p.speed_window_ms = SPEED_WINDOW_MS;
  p.tap_drop_threshold_m = TAP_DROP_THRESHOLD_M;
  p.stride_sweep_m = STRIDE_SWEEP_M;
  p.stride_tap_m = STRIDE_TAP_M;
  p.urgent_time_s = URGENT_TIME_S;
  p.speed_warn_mps = SPEED_WARN_MPS;
  p.speed_warn_hold_ms = SPEED_WARN_HOLD_MS;
  p.speed_warn_repeat_ms = SPEED_WARN_REPEAT_MS;
  p.tof_stale_ms = TOF_STALE_MS;
  p.imu_stale_ms = IMU_STALE_MS;

  p.cal_duration_ms = CAL_DURATION_MS;
  p.cal_min_samples = CAL_MIN_SAMPLES;
  p.cal_beta_tol_deg = CAL_BETA_TOL_DEG;
  p.cal_l_min_m = CAL_L_MIN_M;
  p.cal_l_max_m = CAL_L_MAX_M;
  p.cal_max_rms_m = CAL_MAX_RMS_M;

  p.battery_mah = BATTERY_MAH;
  p.avg_current_ma = AVG_CURRENT_MA;
  p.low_battery_pct = LOW_BATTERY_PCT;
  p.critical_battery_pct = CRITICAL_BATTERY_PCT;
  p.low_battery_repeat_ms = LOW_BATTERY_REPEAT_MS;
  p.battery_report_hours = BATTERY_REPORT_HOURS != 0;

  p.haptic_plan = HAPTIC_PLAN;
  p.haptic_strong = HAPTIC_STRONG;
  p.haptic_medium = HAPTIC_MEDIUM;
  p.haptic_soft = HAPTIC_SOFT;
  p.head_buzz_ms = HEAD_BUZZ_MS;
  p.head_buzz_level = HEAD_BUZZ_LEVEL;
  p.haptic_strength = HAPTIC_STRENGTH;
  p.pod_blind_after_ms = POD_BLIND_AFTER_MS;
  p.pod_blind_repeat_ms = POD_BLIND_REPEAT_MS;
  p.start_taps = START_TAPS;
  p.start_tap_gap_ms = START_TAP_GAP_MS;
  p.start_tap_mps2 = START_TAP_MPS2;
  return p;
}

}  // namespace sr

// Runtime parameters. Defaults come from include/config.h; calibration and
// serial commands change them at run time (and calibration saves them).
#pragma once
#include <stdint.h>
#include "core/vec3.h"

namespace sr {

struct Params {
  // geometry
  Vec3 shaft;                 // IMU axis toward the tip
  float tip_to_pod_m;         // L
  float grip_to_pod_m;
  float ground_beam_deg;      // beta
  float forward_beam_deg;     // gamma
  float head_height_m;
  // attitude (3-axis accelerometer, no gyro)
  float loop_hz;
  float acc_only_tau_s;        // accelerometer low-pass for the cane angle
  float tof_att_gate_deg;      // floor-plane frame trusted: tilt and height agree within this
  float tof_att_cont_deg;
  float tof_att_gain;
  // fixed-angle mode
  float fixed_alpha_deg;       // the trained cane angle (head-level maths)
  float sweep_half_width_deg;  // the trained sweep half-width (sweep position scale)
  float acc_sweep_g_tau_s;
  float acc_sweep_lp_tau_s;
  float acc_sweep_rms_s;
  float acc_sweep_min_mps2;
  // alert filter
  bool forward_enabled;        // false: no forward ToF fitted (ground drop-off only)
  float drop_threshold_m;
  uint8_t n_confirm_ground;
  uint8_t n_confirm_forward;
  bool no_return_is_drop;
  uint8_t no_return_extra;
  float no_return_max_frac;    // no-return drops only while fewer ground frames than this are empty
  float ground_corridor_deg;
  float forward_corridor_deg;
  float forward_corridor_m;
  float side_deadband_m;
  float head_min_m;
  float head_margin_m;
  float forward_max_x_m;
  float pause_on_deg;
  float pause_off_deg;
  uint16_t pause_end_ack_ms;   // >0: pause at least this long, then released -> one ack ring
  float alpha_min_deg;
  float tof_max_range_m;
  uint16_t alert_hold_ms;
  float naive_fwd_range_m;
  uint16_t naive_hold_ms;      // naive mode alert hold (alert_hold_ms in smart mode)
  uint8_t tap_mode;            // 0 off, 1 compensate, 2 gate
  float tap_impact_mps2;
  float tap_max_lift_m;
  uint16_t tap_contact_ms;
  uint16_t tap_detect_window_ms;
  // technique and speed
  float sweep_yaw_rms_dps, walk_bounce_on, walk_bounce_off, arc_turn_deg;
  uint16_t tap_min_interval_ms, tap_max_interval_ms, tap_arc_tol_ms;
  uint16_t technique_hold_ms, speed_window_ms;
  float tap_drop_threshold_m;
  float stride_sweep_m, stride_tap_m;
  float urgent_time_s;
  float speed_warn_mps;
  uint16_t speed_warn_hold_ms;
  uint32_t speed_warn_repeat_ms;
  uint16_t tof_stale_ms;
  uint16_t imu_stale_ms;
  // calibration
  uint16_t cal_duration_ms;
  uint16_t cal_min_samples;
  float cal_beta_tol_deg;
  float cal_l_min_m;
  float cal_l_max_m;
  float cal_max_rms_m;
  // battery
  uint16_t battery_mah;
  uint16_t avg_current_ma;
  uint8_t low_battery_pct;
  uint8_t critical_battery_pct;
  uint32_t low_battery_repeat_ms;
  bool battery_report_hours;
  // haptics / tap to start
  char haptic_plan;           // 'A' or 'B'
  uint8_t haptic_strong, haptic_medium, haptic_soft;
  uint16_t head_buzz_ms;       // >0: head-level alert = one continuous buzz this long (0 = buzz-buzz)
  uint8_t head_buzz_level;     // its level (0-255)
  uint8_t haptic_strength;    // 0 soft, 1 normal, 2 strong
  uint16_t pod_blind_after_ms;
  uint32_t pod_blind_repeat_ms;
  uint8_t start_taps;         // 0 = no standby, start at once
  uint16_t start_tap_gap_ms;
  float start_tap_mps2;

  static Params defaults();   // everything from config.h
};

}  // namespace sr

// SafeRoute Micro Pod firmware (fixed-angle mode) - every tunable number lives here.
//
// Board: ESP32 DevKit V1 (DOIT, ESP32-WROOM-32, 30 pins, CP2102 USB) - the only board.
// Parts (MakeCU 2026 kit): 2x VL53L5CX multizone ToF, a Grove 3-axis digital ACCELEROMETER
// (no gyro), 2x ELEGOO DC motors (off-centre weight) on PN2222 transistors from VIN 5 V,
// USB power bank, on-board LED.
// Fixed-angle mode: the user holds the cane at one trained angle (section 5); the sweep position
// comes from the accelerometer's sideways acceleration.
// Numbers marked [DATA] come from the P1 phone recordings (docs/SafeRoute_DATA_REPORT.pdf,
// experiment_data/results/report.md); [130] marks numbers mapped to the 130 cm cane
// (experiment_data/cane_130cm.py, DATA REPORT v3); [tuned] = set by bench tuning.
#pragma once

// ============================================================================
// 1. Parts
// ============================================================================
// Motors: ELEGOO DC motors with an off-centre weight (behave like an ERM), PWM via PN2222.
// HAPTIC_PWM_KICK_LEVEL for this long at every start so the motor is up to speed within
// the pulse. 40 ms suits a small DC motor with an off-centre weight (ELEGOO / Type 130);
// coin ERMs are fine with it too. 0 = off.
#ifndef HAPTIC_PWM_KICK_MS
#define HAPTIC_PWM_KICK_MS 40
#endif
// Start kick level (255 = full power). Lower it if both motors starting together brown out
// the ESP32 on a weak supply.
#ifndef HAPTIC_PWM_KICK_LEVEL
#define HAPTIC_PWM_KICK_LEVEL 180
#endif
// Never start both motors at the same instant: the right motor plays every pattern this much later
// (both stall currents at once brown out the ESP32 on a USB power bank). 0 = off.
#ifndef HAPTIC_STAGGER_MS
#define HAPTIC_STAGGER_MS 50
#endif

// Forward (head-level) ToF fitted? 0 = EE cut list step 4 "ground drop-off only":
// the forward sensor is not started, never counts as a fault and never triggers a re-init.
#ifndef FEATURE_FORWARD_TOF
#define FEATURE_FORWARD_TOF 1
#endif

// ============================================================================
// 2. Pin map - ESP32 DevKit V1 (30-pin). Every pin below is on the header.
//    Avoided: GPIO 0/2/12/15 for inputs (boot straps), 6-11 (flash), 1/3 (USB serial).
//    GPIO 2 = on-board blue LED (keep it low at boot).
// ============================================================================
#define PIN_I2C0_SDA      21   // ToF x2 + accelerometer
#define PIN_I2C0_SCL      22
#define PIN_TOF_FWD_XSHUT 16   // RX2 on the silkscreen: forward VL53L5CX LPn
#define PIN_TOF_GND_XSHUT 17   // TX2 on the silkscreen: ground VL53L5CX LPn
#define PIN_LED_SINGLE    2    // on-board blue LED
#ifndef PIN_BATT_ADC
#define PIN_BATT_ADC      -1   // MakeCU kit: USB power bank, no divider -> battery % unknown.
                               // LiPo build: 34 (ADC1, input only; battery via 100k/100k divider)
#endif
#define PIN_ERM_LEFT      18   // left ELEGOO DC motor: 470 ohm to the base of its PN2222
#define PIN_ERM_RIGHT     19   // right ELEGOO DC motor (same)
#define PIN_BUTTON        27   // the pod button: to GND, internal pull-up.
                               // Nothing wired = never pressed.
#define PIN_BOOT_BUTTON   0    // the DevKit's BOOT button does the same (nothing to wire)

#define I2C_FREQ_HZ        400000
#define TOF_FWD_ADDR       0x29   // default address
#define TOF_GND_ADDR       0x30   // re-addressed at boot (LPn); lasts until power-off
#define LED_ACTIVE_LOW     0      // 1 for an LED wired to 3V3 (on when the pin is low)
#define BATT_DIVIDER       2.0f   // V_batt = V_adc * BATT_DIVIDER

// ============================================================================
// 3. Timing
// ============================================================================
#define LOOP_HZ            50     // IMU + filter + telemetry rate
#define TOF_L5_HZ          15     // VL53L5CX frame rate: 8x8 zones, its maximum (sensors/tof_zones.h)
#define TOF_STALE_MS       250    // no new frame for this long = sensor fault
#define TOF_REINIT_MS      1000   // retry a faulty sensor this often
// every VL53L5CX start uploads ~84 KB of sensor firmware (blocks the loop ~1-2.5 s)
#define TOF_L5_LOAD_I2C_HZ 400000  // upload clock (falls back to I2C_FREQ_HZ if it fails)
#define TOF_L5_STALL_MS    1000   // a fault with a frame this recent = passing I2C error: no reload
#define TOF_L5_RELOAD_MS   5000   // at most one full reload per sensor this often
#define IMU_STALE_MS       100
#define CPU_MHZ            80     // lower clock = lower power; 80 is the minimum with BLE

// ============================================================================
// 4. Geometry (130 cm cane; Andy's beam angles; recalibrated by the serial `cal` command)
// ============================================================================
// Cane 130 cm (P1 recorded on 120 cm). Same walker, same hand height: the hand pivots
// 1.15 m from the tip instead of 1.05 m, so the walking angle drops 41.1 -> 36.9 deg
// (32.9-40.3) [130]. The pod stays 0.72 m from the tip (58 cm below the top).
#define POD_TIP_TO_POD_M     0.72f  // L, tip -> sensor window along the shaft [DATA]
#define POD_GRIP_TO_POD_M    0.43f  // grip (hand, 15 cm below the top) -> pod: tip-lift pivot [130]
#define GROUND_BEAM_DEG      19.0f  // beta: lands 0.76 m past the tip at the 36.9 deg walking angle
                                    // (0.65-0.98 m over 32.9-40.3 deg); 18.8 deg exact [130]
                                    // (low-vision users walk 0.8 m/s: ~0.84 s to look down and stop)
#define FORWARD_BEAM_DEG     73.0f  // gamma: 36 deg above horizontal, 1.5 m bar met 1.46 m ahead
                                    // (1.32-1.61 m), head height 1.87 m ahead [130]
#define USER_HEAD_HEIGHT_M   1.80f  // walker height; set per user
// IMU axis that points along the shaft TOWARD THE TIP (x, y, z). Find it with
// the serial command `axis` while the cane hangs tip-down; it is then saved.
#define IMU_SHAFT_AXIS_X     0.0f
#define IMU_SHAFT_AXIS_Y    -1.0f
#define IMU_SHAFT_AXIS_Z     0.0f

// ============================================================================
// 5. Attitude filter
// ============================================================================
// No gyro. Sweeping adds 2-7 deg of error to an accelerometer-only cane angle, which the drop
// maths cannot take, so the measured angle comes from the ground laser: the four central
// VL53L5CX zones give the floor plane (beam angle below it + sensor height). A frame is trusted
// when both agree with "tip on this floor": |alpha(tilt) - alpha(height)| < TOF_ATT_GATE_DEG.
// Between trusted frames the accelerometer carries the angle.
#define ACC_ONLY_TAU_S       0.25f  // accelerometer low-pass [tuned]
#define TOF_ATT_GATE_DEG     3.0f   // [tuned]
// While two-point tapping lifts the tip (height check fails, tilt still right): trusted when
// its tilt is within this of the last tip-on-floor angle. 0 = off.
#define TOF_ATT_CONT_DEG     6.0f   // [tuned]
#define TOF_ATT_GAIN         1.0f   // share of each trusted frame taken into the angle offset [tuned]

// ---- Fixed-angle mode -----------------------------------------------------------------
// The user holds the cane at one trained angle and sweeps a trained width. Then:
//  * head-level maths: the fixed trained angle FIXED_ALPHA_DEG. A few degrees off moves the
//    obstacle height by a few cm only (P1: 3.8 cm RMS at 2 m with 38 deg).
//  * drop maths, "cane raised" (pause) and "cane too flat": the measured angle above. A fixed
//    angle is too coarse for 8 cm drops and cannot follow the cane pivoting in the hand when a
//    two-point tap lifts the tip (false alarms at the first taps in testing).
//  * sweep position: from the accelerometer's sideways acceleration. A swept cane turns
//    back at each end of the arc, so at the right end the pod accelerates to the left:
//    yaw offset ~ -lateral accel, scaled to the trained half-width SWEEP_HALF_WIDTH_DEG.
//    Gives the walking corridor (head-level only) and left / right head alerts.
// [DATA] P1 S04 + S05 (130 cm mapping): cane angle median 38.2-38.6 deg, p5-p95 33.9-42.0;
// sweep |offset| p90 22-23 deg (half-width 22.5-25.6). Accelerometer side vs gyro truth:
// correlation 0.84 / 0.85, side right 93-97% when the cane is > 10 deg off centre.
#ifndef FIXED_ALPHA_DEG
#define FIXED_ALPHA_DEG      38.0f  // [DATA] walking median; false-drop = missed-drop risk (3.8 / 3.7%)
#endif
#ifndef SWEEP_HALF_WIDTH_DEG
#define SWEEP_HALF_WIDTH_DEG 23.0f  // [DATA] trained sweep half-width (P1: |offset| p90 22-23 deg)
#endif
#define ACC_SWEEP_G_TAU_S    0.8f   // gravity reference low-pass [DATA: best of 0.6-3 s]
#define ACC_SWEEP_LP_TAU_S   0.08f  // sideways accel, 2 x first-order low-pass [DATA: best of 0.05-0.2]
#define ACC_SWEEP_RMS_S      3.0f   // sweep size = RMS over ~1.5 sweeps
#define ACC_SWEEP_MIN_MPS2   0.15f  // below this RMS the cane is not swept: offset unknown (= 0)

// ============================================================================
// 6. Alert filter (all three rules must pass: confirmation, height band, corridor)
// ============================================================================
#define DROP_THRESHOLD_M      0.08f // ground lower than expected by more than this = drop
#define N_CONFIRM_GROUND      3     // consecutive ToF frames (200 ms at 15 Hz) [DATA: the worst sweep
                                    // keeps a 30 cm drop in view for 3.6-4.3 frames (S4 3.9, S5 4.3)]
#define N_CONFIRM_FORWARD     2     // thin, low-contrast overhangs (branches, bars) are what low-vision
                                    // users miss. A head-level frame that only the corridor rejects
                                    // (estimate flickers at its edge) neither counts nor breaks the run.
#define NO_RETURN_IS_DROP     1     // ground beam sees nothing while it should see the floor:
                                    // deep drop (platform edge) - or a black/shiny floor
#define NO_RETURN_EXTRA       2     // ... needs this many more frames than a measured drop
#define NO_RETURN_MAX_FRAC    0.30f // ... and only if the floor was answering when the empty run began:
                                    // if more than this share of the ground frames (1 s average) were
                                    // empty or invalid, the floor is dark/shiny or in sunlight, not a
                                    // platform edge (back to normal below half this). Decided at the
                                    // start of each empty run and kept while it lasts, so a real edge
                                    // keeps alerting. Measured drops always alert. 1.0 = off.
#define GROUND_CORRIDOR_DEG   35.0f // drop must be within sweep centre +- this
#define FORWARD_CORRIDOR_DEG  25.0f // head-level obstacle within sweep centre +- this ...
#define FORWARD_CORRIDOR_M    0.30f // ... and within this distance of the walking line (half body
                                    // width + margin: no alerts for people beside the path)
#define SIDE_DEADBAND_M       0.05f // obstacle within +- this of the walking line = straight ahead
                                    // (side = mean over the confirming frames)
#define HEAD_MIN_HEIGHT_M     0.90f // obstacles lower than this are the cane's job
#define HEAD_MARGIN_M         0.10f // ... and higher than head + this are not a risk
#define FORWARD_MAX_X_M       1.70f // only obstacles closer than this (horizontal). 1.7, not 1.5: at
                                    // the steepest walking angle the 73 deg beam meets a 1.5 m overhang
                                    // 1.61 m ahead [130] (120 cm cane, 77 deg: 1.59 m)
#define PAUSE_ON_DEG          52.0f // cane pulled up: no alerts. [130] walking never above 45.7 deg;
                                    // after finding an edge the cane goes to 52-60 (curb), 54-57 (stair
                                    // top), 56-65 (stairs) -> the user has found it: stay quiet.
                                    // (120 cm cane: walking <= 51.6, after the edge 60-86 -> 60/55)
#define PAUSE_END_ACK_MS      0     // >0: cane back down after a pause at least this long -> one
                                    // "ack" ring (stairs done). 0 = off
#define PAUSE_OFF_DEG         49.0f // = walking max 45.7 + alpha error p95 3.4 deg [130]
#define ALPHA_MIN_DEG         20.0f // below this the ground beam geometry is meaningless
#define TOF_MAX_RANGE_M       3.5f
#define ALERT_HOLD_MS         350   // keep a confirmed alert this long after the last hit
#define NAIVE_FWD_RANGE_M     1.50f // naive mode: any forward return closer than this alerts
#define NAIVE_ALERT_HOLD_MS   2500  // naive mode: keep the alert this long after the last hit (a sweep
                                    // is 2 s, so an obstacle hit on every sweep buzzes without a break)
// Two-point tapping: between taps the tip is in the air (P1: 8 cm by tape,
// median 8.6 cm, p90 14 cm from the cane-angle change), and a lifted tip reads exactly like
// a drop of that depth. Tapping lifts the tip by pivoting the cane in the hand,
// so the lift = (L + grip_to_pod) * (sin(alpha at the last tap) - sin(alpha now)).
//   TAP_MODE 1 = compensate: add that lift back to dh; every frame still counts,
//                so a curb stays visible between taps (default)
//   TAP_MODE 2 = gate: only use ground frames for TAP_CONTACT_MS after a tap
//                (curbs are only seen in those windows: later alerts)
//   TAP_MODE 0 = off
#define TAP_MODE              1
#define TAP_IMPACT_MPS2       6.0f  // |a| spike above its 0.5 s mean = tip impact
#define TAP_CONTACT_MS        200   // gate mode: ground frames count this long after an impact
#define TAP_DETECT_WINDOW_MS  2500  // >= 2 impacts in this window = the user is tapping
#define TAP_MAX_LIFT_M        0.20f // larger computed lifts are clipped

// ============================================================================
// 6b. Technique and walking speed (P1 data, 2026-10-30)
// ============================================================================
// The pod tells how the cane is used, every tick:
//   Still  - not walking, cane not moving
//   Sweep  - constant-contact sweeping (yaw-rate RMS > 20 deg/s; needs a gyro, so this pod
//            reports Hold for a swept cane - only tapping and walking speed use this section)
//   Tap    - two-point touch (>= 3 tip impacts in 3 s)
//   Hold   - walking without sweeping (diagonal cane, common with low vision)
// [DATA] yaw RMS: still < 2, standing sweep 33-40, walking 40-53 deg/s;
//        step bounce (as the firmware filters it): still 0.03, standing sweep 0.23-0.41,
//        walking 0.47-1.46 m/s2 (5th-95th percentile)
#define SWEEP_YAW_RMS_DPS     20.0f // above this the cane is sweeping
#define WALK_BOUNCE_ON_MPS2   0.45f // step bounce above this = walking ...
#define WALK_BOUNCE_OFF_MPS2  0.36f // ... and below this = stopped
// Tapping = the last 3 tip impacts come in a steady rhythm (0.5-1.8 s apart) and at
// least 2 of them land on a sweep turning point (two-point touch taps at each end of
// the arc). Random bumps while sweeping don't. [DATA] S5 tapping: 94% detected;
// sweeping recordings: <= 7% (gait_check.md).
#define TAP_MIN_INTERVAL_MS   500
#define TAP_MAX_INTERVAL_MS   1800
#define TAP_ARC_TOL_MS        250   // impact within this of a sweep turning point
#define TECHNIQUE_HOLD_MS     1000  // a new technique must last this long before it switches
#define ARC_TURN_DEG          12.0f // a sweep arc ends when yaw turns back this far from its extreme
                                    // [DATA] S4 1.01 arcs/s (sweep 0.49 Hz), S5 0.90 arcs/s
// Per-technique drop threshold: tapping keeps a little arm lift that the pivot
// compensation can't see, so it gets 1 cm more margin (curb/stair still read -14 cm)
#define TAP_DROP_THRESHOLD_M  0.09f
// Speed = (sweep arcs or taps per second) x metres per arc. P1: 0.81 m/s with 0.98
// arcs/s sweeping and 0.91 taps/s tapping. Recalibrate per user: serial `walk start`,
// walk a measured distance, `walk 12.16`.
#define STRIDE_SWEEP_M        0.83f
#define STRIDE_TAP_M          0.89f
#define SPEED_WINDOW_MS       4000
// Low-vision urgency: when a drop is confirmed, time to the edge = beam look-ahead /
// speed. Below this the drop pattern turns urgent (faster, stronger).
#define URGENT_TIME_S         0.60f
// Walking faster than the look-ahead can cover: one soft "slow down" cue.
#define SPEED_WARN_MPS        1.30f // 0.75 m look-ahead -> 0.48 s warning at 1.3 m/s
#define SPEED_WARN_HOLD_MS    5000  // sustained this long
#define SPEED_WARN_REPEAT_MS  60000 // at most once a minute

// ============================================================================
// 7. Calibration (long press on a flat floor, tip down; hold the cane still)
// ============================================================================
#define CAL_DURATION_MS       3000
#define CAL_MIN_SAMPLES       30
#define CAL_BETA_TOL_DEG      6.0f  // solved beta must be within this of GROUND_BEAM_DEG
#define CAL_L_MIN_M           0.35f
#define CAL_L_MAX_M           1.10f
#define CAL_MAX_RMS_M         0.03f // flat-floor residual must be below this

// ============================================================================
// 8. Battery (only used when PIN_BATT_ADC is wired to a LiPo divider; the power bank has none)
// ============================================================================
#define BATTERY_MAH           600
#define AVG_CURRENT_MA        250   // ESP32 @80 MHz + BLE, 2x VL53L5CX, accelerometer, motors
#define LOW_BATTERY_PCT       15
#define CRITICAL_BATTERY_PCT  3     // below this: warn, then deep sleep (cane works passively)
#define LOW_BATTERY_REPEAT_MS (5UL * 60UL * 1000UL)
#define BATTERY_REPORT_HOURS  0     // 0: pulses = 20% steps (1-5); 1: pulses = hours left

// ============================================================================
// 9. Haptics and tap to start
// ============================================================================
#define HAPTIC_PLAN           'A'   // 'A' left/right, 'B' rhythm only (haptic gate result)
// ~30% under the original 255/200/130: at full power both motors starting together dropped
// the USB supply enough to knock out the ToF sensors and reset the ESP32
#define HAPTIC_STRONG         180
#define HAPTIC_MEDIUM         140
#define HAPTIC_SOFT           95
// Vibration strength setting (serial/BLE `strength soft|normal|strong`, saved). Rhythms never
// change, only levels: older hands and winter gloves need more (Kim & Cho 2013: <= 4 levels).
#define HAPTIC_STRENGTH       1     // 0 soft, 1 normal, 2 strong
// Head-level alert as ONE continuous buzz instead of the repeating buzz-buzz: HEAD_BUZZ_MS > 0 =
// buzz this long at HEAD_BUZZ_LEVEL, then quiet while the obstacle stays (a new alert buzzes again).
// 0 = the repeating buzz-buzz (default). Keep the level <= HAPTIC_MEDIUM on a power bank: full power
// on both motors browned out the ESP32 in bench tests.
#define HEAD_BUZZ_MS          0
#define HEAD_BUZZ_LEVEL       HAPTIC_MEDIUM
// "Pod blind": a sensor or the IMU is down, so the pod is not protecting the user.
// One soft long pulse after POD_BLIND_AFTER_MS, then every POD_BLIND_REPEAT_MS.
#define POD_BLIND_AFTER_MS    3000
#define POD_BLIND_REPEAT_MS   60000
// The button (PIN_BUTTON or BOOT, main.cpp): power on = OFF (lasers off, LED off); click = ON
// (lasers start, left-right ring, LED on); double click (2nd press within BUTTON_DOUBLE_MS) or hold = OFF.
#define BUTTON_DEBOUNCE_MS    30    // shorter presses are contact bounce
#define BUTTON_DOUBLE_MS      350   // a single click is acted on this long after its release
#define BUTTON_HOLD_MS        1500
// Tap to start (no button needed): the pod boots in standby (sensors on, no alerts, no buzzing)
// until the tip taps the floor START_TAPS times, each within START_TAP_GAP_MS of the last.
// Serial/BLE `off` goes back to standby. 0 = start at once.
// START_TAP_MPS2 is below TAP_IMPACT_MPS2: the MMA7660FC clips at +-1.5 g, so a hard tap only
// reaches |a| ~15 m/s2 (~5 above gravity).
#define START_TAPS            5
#define START_TAP_GAP_MS      800
#define START_TAP_MPS2        4.0f  // |a| spike above its 0.5 s mean = one start tap

// ============================================================================
// 10. Telemetry
// ============================================================================
#define SERIAL_BAUD           460800
#define SERIAL_TX_BUFFER      2048  // bytes; the UART FIFO alone (128) is smaller than one telemetry line
#define TELEMETRY_SERIAL      1
#ifndef FEATURE_BLE
#define FEATURE_BLE           1     // Nordic UART service "SafeRoute"
#endif
#ifndef TELEMETRY_HAPTICS
#define TELEMETRY_HAPTICS     0     // 1: add motor levels "hl"/"hr" to the serial telemetry
#endif
#define BLE_TELEMETRY_DIV     5     // 50 Hz / 5 = 10 Hz over BLE
#define BLE_DEVICE_NAME       "SafeRoute"

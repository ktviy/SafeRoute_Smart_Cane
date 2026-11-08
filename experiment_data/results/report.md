# Cane motion: numbers for the team

Walkers: P1. Every event below is found in the sensor data; videos are lined up by sound (sync_video.py).

## Video sync (sensor time = video time + offset)

| recording | video | offset (s) | confidence |
| --- | --- | --- | --- |
| S06_curb_stepdown_swipe | S06_curb_stepdown_swipe.mp4 | +1.61 | high |
| S08_curb_stepup_tap | S08_curb_stepup_tap.mp4 | +1.57 | high |
| S08_curb_stepup_swipe | S08_curb_stepup_swipe.mp4 | +2.06 | high |
| S09_side_object_left | S09_side_object_left.mp4 | +0.48 | high |
| S09_side_object_right | S09_side_object_right.mp4 | -0.49 | high |
| S11_stairtop_swipe | S11_stairtop_swipe.mp4 | +1.91 | high |
| S12_stairtop_tap | S12_stairtop_tap.mp4 | +1.55 | high |
| S13a_stair_descent | S13a_stair_descent.mp4 | +1.65 | high |
| S13b_stair_descent | S13b_stair_descent.mp4 | +1.45 | high |
| S13c_stair_descent | S13c_stair_descent.mp4 | +1.56 | high |
| S15_stairtop_parallel | S15_stairtop_parallel.mp4 | +1.28 | high |

## For Andy: cane angle while walking (S4, S5)
- Median **43.0 deg**, typical range **38.4 to 47.0 deg** (5th to 95th percentile), as the phone measured it.
- The static checks say the phone reads +1.9 deg against the inclinometer (mount offset); the shaft itself was at about **41.1 deg** (36.5 to 45.1).
- Design the ground beam so it lands 0.5 to 1.0 m past the tip across this whole range.

## Sweep by technique (means)

| technique | sweep freq (Hz) | sweep width p5-p95 (deg) | half-width p95 (deg) | ToF frames in view, worst | pitch swing (deg) | cadence (steps/min) |
| --- | --- | --- | --- | --- | --- | --- |
| constant | 0.49 | 44.4 | 23.8 | 3.9 | 5.8 | 59 |
| two-point | 0.45 | 41.7 | 23.0 | 4.3 | 10.2 | 55 |

**N = 3 check:** a 30 cm drop stays in the ground beam for at least **3.9 ToF frames** at 30 Hz in the fastest sweeps: **PASS**. (margin +0.9 frames). A curb or stair edge runs across the whole path, so it stays in view much longer.

## Firmware pitch filter while walking (vs the phone's own fused angle)
- Basic filter (K = 0.98 at 100 Hz, pitch axis only): p95 error **6.6 deg**.
- Firmware filter (gravity vector, tau = 2 s): **3.4 deg**; same plus centripetal correction: **2.4 deg** (worst of S4, S5).
- 1 deg of alpha error is about 1 cm of drop-height error; the alert threshold is 8 cm.

## Walking speed and warning time

| source | speed (m/s) | warning time (s) |
| --- | --- | --- |
| S10 timed run 1 | 0.78 | 0.86 |
| S10 timed run 2 | 0.84 | 0.79 |

Warning time = look-ahead 0.75 m / speed, minus 100 ms confirmation (3 ToF frames at 30 Hz).

Cadence 57 steps/min at a median speed of 0.81 m/s gives a step length of about 0.86 m.

## Pitch accuracy vs inclinometer (S1)

| recording | inclinometer (deg) | phone (deg) | difference (deg) | firmware filter (deg) |
| --- | --- | --- | --- | --- |
| S01_natural_angle | 43.53 | 45.80 | +2.27 | 45.82 |
| S01_high_angle | 59.45 | 60.98 | +1.53 | 61.05 |

Mean difference **+1.9 deg**: the same sign at both angles, so it is the phone sitting slightly tilted on the shaft, not filter error. Lying flat (S2) the phone reads 3.8 deg. The pod's one-press calibration (L and beta) absorbs this offset; the spread between the two angles (0.7 deg) is the real accuracy.

## Edges found in the sensor data

| scenario | edges | walking alpha before (deg) | edge -> alpha > 57 deg (s) | peak alpha after the edge (deg) | walk before (s) |
| --- | --- | --- | --- | --- | --- |
| S11 | 3 | 42.3 (40.3-43.4) | 0.63 | 64 (63-64) | 9.2 |
| S12 | 3 | 38.7 (35.1-40.7) | 0.59 | 65 (62-67) | 9.6 |
| S13 | 3 | 42.9 (42.2-43.8) | 0.68 | 84 (83-86) | 10.3 |
| S6 | 2 | 45.5 (44.6-46.3) | 0.88 | 63 (60-65) | 11.8 |
| S8 | 3 | 43.0 (41.7-44.8) | 0.85 | 67 (61-72) | 12.2 |

Outside the edge scenarios the cane went past 57 deg **1 times** (turns and pauses on flat ground).

## Stairs (from tip impacts)

- S13a_stair_descent (descent): 13 impacts over 19 s, **1.55 s per step**
- S13b_stair_descent (descent): 13 impacts over 17 s, **1.47 s per step**
- S13c_stair_descent (descent): 13 impacts over 17 s, **1.39 s per step**

Mean **1.47 s per step**.

## Two-point tip lift

- Measured by hand: **5 to 8 cm**.
- S05_walk_tap: from the cane angle (pivot in the hand), median **8.6 cm**, 90th percentile 14.1 cm over 102 taps.
- S08_curb_stepup_tap: from the cane angle (pivot in the hand), median **10.1 cm**, 90th percentile 14.0 cm over 25 taps.
- S12_stairtop_tap: from the cane angle (pivot in the hand), median **11.6 cm**, 90th percentile 17.0 cm over 23 taps.
- A lifted tip reads exactly like a drop of the same depth (dh = -lift), and the alert threshold is 8 cm. The firmware adds the computed lift back to dh while the user taps (TAP_MODE 1), so every frame still counts and a curb stays visible between taps.

## Near-vertical pause

- Highest cane angle while walking: **51.6 deg**. After finding an edge or curb the cane goes to **60-86 deg**.
- So a pause at 60 deg (back on below 55) never interrupts walking, and keeps the pod quiet once the user has found the edge and is checking it.

## Data quality

- S01_natural_angle: iPhone export: accel (g) + gravity combined
- S01_high_angle: iPhone export: accel (g) + gravity combined
- S02_gyro: iPhone export: accel (g) + gravity combined
- S03_swipe: iPhone export: accel (g) + gravity combined
- S04_walk_swipe: iPhone export: accel (g) + gravity combined
- S05_walk_tap: iPhone export: accel (g) + gravity combined
- S06_curb_stepdown_swipe: iPhone export: accel (g) + gravity combined
- S08_curb_stepup_tap: iPhone export: accel (g) + gravity combined
- S08_curb_stepup_swipe: iPhone export: accel (g) + gravity combined
- S09_side_object_left: iPhone export: accel (g) + gravity combined
- S09_side_object_right: iPhone export: accel (g) + gravity combined
- S10_walk_speed: no raw file; speed from 2 timed runs over 12.16 m
- S11_stairtop_swipe: iPhone export: accel (g) + gravity combined
- S12_stairtop_tap: iPhone export: accel (g) + gravity combined
- S13a_stair_descent: iPhone export: accel (g) + gravity combined
- S13b_stair_descent: iPhone export: accel (g) + gravity combined
- S13c_stair_descent: iPhone export: accel (g) + gravity combined
- S15_stairtop_parallel: iPhone export: accel (g) + gravity combined

# SafeRoute cane motion analysis (experiment data)

All recordings here are real: walker P1, phone (Sensor Logger app, 100 Hz) strapped to a
120 cm white cane, 30 Oct - 3 Nov 2026. The plots in `results/plots/` are made from them.

Turns the phone recordings into the numbers for the DATA REPORT: cane angle
range, sweep rate and width, the N-frame confirmation check, walking speed and
warning time, pitch accuracy and mount offset, the firmware filter's error,
edges, step-ups and stair rhythm. Events are found in the sensor data; the
videos are lined up with the sensors by sound, so every event also gets its
time in the video.

## 1. Install (once)

Python 3.9 or newer, then:

    pip install numpy pandas matplotlib imageio-ffmpeg

(`imageio-ffmpeg` is only needed by `sync_video.py`; it brings its own ffmpeg.)

## 2. Folder

    cane_analysis/
      analyze_cane.py      the analysis
      sync_video.py        lines each video up with its sensor recording (by sound)
      log.csv              the recording log: one row per recording, measurements (open in Excel)
      video_sync.csv       written by sync_video.py: offset per video
      events.csv           written by analyze_cane.py: every event with sensor AND video time
      raw/                 one Sensor Logger export per recording (zip, folder or csv)
      video/               the videos (S06-S15) - NOT in this repository (179 MB); video_sync.csv
                           and events.csv already hold the video times they produced
      results/             written by analyze_cane.py

The `file` column of log.csv must match the export name in raw/ exactly, and
`video_file` the name in video/ (typos in the names are fine, as long as they match).

If you edit log.csv in Excel: write the phone axis as `y-` (Excel turns `-y` into
#NAME?) and ranges as `5 to 8` (Excel turns `5-8` into a date).

## 3. Run

    python sync_video.py              # only when videos are added or changed
    python analyze_cane.py analyze

Results in `results/`:

- `report.md`: the headline numbers, plus a data-quality list
- `summary.csv`: one row per recording, every computed value
- `event_metrics.csv`: cane angle before each edge, time to pass 57 deg, peak angle after it
- `events_synced.csv` (= `events.csv`): every event with `sensor_time_s` and `video_time_s`
- `approaches.csv`: every walk up to an edge or curb
- `plots/`: cane angle (phone fusion and firmware filter) and sweep for each recording

## How things are measured

| What | How |
| --- | --- |
| Sync taps | 3 hard impacts 0.4-1.2 s apart, at the start and at the end |
| Video sync | `sync_video.py` finds the shift that puts the most accelerometer impacts on an audio click (taps, steps, curb), and checks that all 3 sync taps land on clicks. sensor time = video time + offset |
| Edge found (S6, S11-S13) | the cane angle leaves the walking angle and passes 57 deg (tip over the edge, cane pulled in) after >= 3 s of walking |
| Step-up (S8) | same rise, labelled `step_up`. Two recordings: `S08_curb_stepup_swipe` (constant contact) and `S08_curb_stepup_tap` (two-point; no two-point step-down was recorded) |
| Mount offset | S1: phone angle minus inclinometer, at the natural and the high angle |
| Walking speed | S10 timed runs: `walk_speed_mps` (e.g. `0.78;0.84`) or `walk_time_s` over `walk_distance_m` |
| Cadence | two-point: one tip tap per step; constant contact: one sweep arc per step |
| Stairs (S13) | hard tip impacts on each step while the cane is near vertical; median spacing = seconds per step |
| Two-point tip lift | between taps: (grip -> tip) x (sin alpha at the tap - sin alpha), the same estimate the firmware uses |
| Pause threshold | highest cane angle while plainly walking vs the peak angle after each edge |

## Adding events by hand

To mark something you see in a video (e.g. `foot_on_lower_level`), add a row to
events.csv with `source` = `video`, the `file`, the `event` name and the
`video_time_s`. The next analysis keeps the row, converts it to sensor time with
the video's offset and notes how far it is from the nearest sensor event.

## Settings

Top of analyze_cane.py: `FRAME_HZ` (30, ToF
rate), `N_CONFIRM` (3), `LOOKAHEAD_PAST_TIP_M` (0.75 m), `FEATURE_WIDTH_M` (0.30 m), `CF_TAU_S` (2 s, as the
firmware), `EDGE_ALPHA_DEG` (57), `EDGE_BASE_MAX_DEG` (52), `IMPACT_MIN` (8 m/s2).

## 130 cm cane (DATA REPORT v3, 2026-11-06)

P1 recorded on a **120 cm** cane (hand 1.05 m from the tip). The pod now goes on a **130 cm** cane,
still 0.72 m from the tip, so the hand is 1.15 m from the tip. Same walker, same hand height:

    (L + grip)_130 * sin(alpha_130) = (L + grip)_120 * sin(alpha_120)

`python cane_130cm.py --check` maps every measured angle across and re-derives the mounts:
walking 41.1 -> 36.9 deg (32.9-40.3), so the ground beam needs 18.8-19.1 deg (19) and the forward
beam 72.9-73.4 deg (73), whatever exactly the hand sits; the pause moves 60/55 -> 52/49 deg.
Everything in `results/` is still the 120 cm recording itself.

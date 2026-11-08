# SafeRoute Smart Cane

SafeRoute is a small pod that clips onto a standard white cane. It warns the user about two things a cane alone misses:

- **drop-offs** (curbs, stair tops, platform edges) a step before the tip reaches them;
- **head-level obstacles** (signs, branches, open truck doors) that the cane passes under.

Each warning is a vibration pattern on two motors (left and right) that travels up the shaft to the hand. A filter keeps the pod quiet during normal sweeping, so it only buzzes when it matters.

Built for **MakeCU 2026**.

**Live dashboard:** https://ktviy.github.io/SafeRoute_Smart_Cane/dashboard/
Open it in Chrome or Edge, power the pod, then press **USB** or **Bluetooth**.

![Schematic](hardware/schematic_as_built.png)

## What is in this repository

| Folder | Contents |
| --- | --- |
| [`firmware/`](firmware) | ESP32 firmware (PlatformIO). Pod logic in `lib/saferoute`, 45 unit tests, ESP32 main loop. [Build and wiring guide](firmware/README.md) |
| [`hardware/`](hardware) | As-built schematic ([SVG](hardware/schematic_as_built.svg) / [PNG](hardware/schematic_as_built.png)) and the original KiCad design |
| [`experiment_data/`](experiment_data) | Real cane-motion recordings (phone IMU on a white cane), analysis scripts, results and plots |
| [`dashboard/`](dashboard) | Live web dashboard (no install: Web Serial / Web Bluetooth) and a Python version |
| [`docs/`](docs) | [DATA REPORT](docs/SafeRoute_DATA_REPORT.pdf): the design numbers and where each one comes from |

## Hardware

| Part | Qty | Role |
| --- | --- | --- |
| ESP32 DevKit V1 (ESP32-WROOM-32, CP2102) | 1 | Controller. 50 Hz loop, USB and BLE telemetry |
| VL53L5CX 8x8 multizone ToF | 2 | Ground beam (19 deg above the shaft: drop-offs) and forward beam (73 deg: head level) |
| Grove 3-axis digital accelerometer (LIS3DHTR) | 1 | Cane angle, sweep position, tip taps (no gyro) |
| ELEGOO DC motor with an off-centre weight | 2 | Left / right vibration |
| PN2222 + 470 ohm + 1N5817 | 2 sets | Motor drivers with flyback diodes |
| 100 uF capacitor | 1 | Motor supply buffer |
| Push button, LED + resistor | 1 each | On/off and status |
| USB power bank | 1 | 5 V supply |
| 3D-printed pod and cane | - | The pod sits 0.72 m from the tip of a 130 cm cane |

Pin map:

| Signal | Pin |
| --- | --- |
| I2C (SDA, SCL) | GPIO21, GPIO22 |
| ToF LPn (ground, forward) | GPIO17, GPIO16 |
| Motors (left, right) | GPIO18, GPIO19 |
| Button | GPIO27 |
| LED | GPIO2 |

Full details are in [`firmware/README.md`](firmware/README.md#wiring-esp32-devkit-v1).

## How it works

1. **Sensors.** The **ground beam** measures the distance to the floor about 0.76 m past the tip. From that distance and the cane angle the pod computes the floor height ahead (`dh`). A drop deeper than 8 cm is a drop-off. The **forward beam** measures how high an obstacle in front is.
2. **Sweep position.** The accelerometer gives the cane angle and the position in the sweep. The user holds the cane at a trained 38 deg and sweeps +-23 deg; both numbers come from the recordings below.
3. **Filter.** An alert must pass three rules:
   - it is seen in 3 frames in a row (2 for head level);
   - its height is in the right band;
   - it is inside the walking corridor, not beside the path.

   When the cane is lifted above 52 deg (the user has found the edge), the pod goes quiet.
4. **Haptics.** A drop-off gives 3 fast strong pulses on both motors. A head-level obstacle gives 2 medium pulses on the side it is on.

**Using the pod:**
- Power on: the pod is OFF.
- **Click**: the pod turns ON (lasers on, one ring, LED on).
- **Double click**: the pod turns OFF.

## Experiment data

All numbers come from real recordings. Walker P1 walked with a phone (Sensor Logger, 100 Hz) strapped to a 120 cm white cane, 30 Oct - 3 Nov 2026. That gave 18 recordings:
- static angles;
- sweeping and walking with both techniques (constant contact and two-point touch);
- curb step-down and step-up;
- objects beside the path;
- stair tops and stair descents;
- timed walks.

See [`experiment_data/`](experiment_data) and the [full results](experiment_data/results/report.md).

| Result | Value |
| --- | --- |
| Cane angle while walking (120 cm cane, shaft) | 41.1 deg (36.5-45.1); 36.9 deg on the 130 cm cane |
| Sweep frequency / half-width (p95) | 0.45-0.49 Hz / 23-24 deg |
| Frames a 30 cm drop stays in the beam (fastest sweep) | 3.9 at 30 Hz: 3-frame confirmation **passes** |
| Firmware angle filter error while walking (p95) | 2.4-3.4 deg (about 1 cm of drop height per deg) |
| Walking speed / warning time | 0.78-0.84 m/s / 0.79-0.86 s |
| Cane angle after finding an edge | 63-84 deg peak, past 57 deg within 0.6-0.9 s |
| Stair descent rhythm | 1.47 s per step |

| Walking, constant contact (S04) | Walking, two-point touch (S05) |
| --- | --- |
| ![S04](experiment_data/results/plots/S04_walk_swipe.png) | ![S05](experiment_data/results/plots/S05_walk_tap.png) |
| **Stair top found (S11)** | **Curb step-down (S06)** |
| ![S11](experiment_data/results/plots/S11_stairtop_swipe.png) | ![S06](experiment_data/results/plots/S06_curb_stepdown_swipe.png) |

All 17 plots are in [`experiment_data/results/plots/`](experiment_data/results/plots). The videos used to time-align the events (179 MB) are not included; `video_sync.csv` and `events.csv` keep the times they produced.

## Quick start

```bash
# firmware
cd firmware
pio test -e native              # unit tests on the laptop
pio run -e esp32dev -t upload   # flash the pod
pio device monitor              # 460800 baud, type 'help'

# experiment analysis
cd experiment_data
pip install numpy pandas matplotlib
python analyze_cane.py analyze  # rebuilds results/ and results/plots/
```

Dashboard: open `dashboard/web_dashboard.html` in Chrome or Edge, or use the live link above. Web Serial and Web Bluetooth need HTTPS or a local file.

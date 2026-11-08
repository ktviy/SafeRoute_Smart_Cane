# SafeRoute Micro Pod firmware (fixed-angle build)

This is the ESP32 firmware for the cane pod. It runs on an **ESP32 DevKit V1** (30 pins, CP2102) mounted on a **130 cm** cane. The pod has two ToF beams and a 3-axis accelerometer. From them it gives drop-off and head-level alerts through two vibration motors. A filter keeps it quiet during normal sweeping.

**Parts:**
- 2x **VL53L5CX** multizone ToF sensors.
- 1x Grove **3-axis digital accelerometer**. It measures acceleration only (**no gyro**).
- 2x **ELEGOO DC motors** with an off-centre weight, each switched by a PN2222 transistor from VIN 5 V.
- 1x push button and 1x status LED.
- A USB power bank.

Wiring: [`../hardware/schematic_as_built.svg`](../hardware/schematic_as_built.svg).

**Fixed-angle mode:** the user holds the cane at one trained angle (**38 deg**) and sweeps a trained width (**+-23 deg**). The accelerometer's sideways acceleration then tells where the cane is in its sweep (left or right of the walking line). That makes the walking corridor and the left / right head-level alerts work without a gyro. Both numbers come from the real walking recordings in [`../experiment_data`](../experiment_data). The [DATA REPORT](../docs/SafeRoute_DATA_REPORT.pdf) explains how.

All the decision logic is plain C++ in `lib/saferoute`. The same code runs on a laptop, so it is unit-tested there.

```
firmware/
  include/config.h          every tunable number and pin (the ONLY place to change them)
  lib/saferoute/            the pod logic, no hardware inside
    core/                   types, vec3, Params (defaults from config.h)
    kinematics/attitude     accelerometer -> measured cane angle (+ floor-plane fix), sweep position
    kinematics/geometry     drop height dh, forward obstacle height, still-cane calibration solver
    kinematics/gait         technique (still / tap / hold) and walking speed from the IMU
    sensors/tof_zones       VL53L5CX: 4 central zones -> one beam range and the floor plane
    filter/alert_filter     the 3-rule filter (confirmation, height band, corridor), pause, fail-safe
    haptics/haptic_engine   haptic language (patterns, priorities, Plan A/B)
    power/battery           voltage -> %, low-battery warning (no sense pin on the power-bank build)
    telemetry/telemetry     the JSON telemetry line
    app/pipeline            glues all of the above; the main loop calls it once per tick
  src/
    main.cpp                ESP32 main loop: 50 Hz scheduler, button, serial/BLE commands, re-init
    hal/                    I2C bus, VL53L5CX, accelerometer (LIS3DH / ADXL345 / H3LIS331 / MMA7660),
                            PWM motors, status LED, settings in flash
    telemetry/ble_uart      BLE Nordic UART service ("SafeRoute")
    haptic_test/            motor bench-test firmware
  test/test_core/           45 unit tests (pio test -e native)
```

## Build, flash, test

Install PlatformIO, either as the VS Code extension or with `pip install platformio`. Then run these from this folder:

| What | Command |
| --- | --- |
| **The pod** | `pio run -e esp32dev -t upload` |
| Motor bench test (motors on GPIO18 / GPIO19) | `pio run -e haptic_test_pwm -t upload` |
| Unit tests on the laptop (needs gcc) | `pio test -e native` |
| Serial monitor (telemetry + commands) | `pio device monitor` (460800 baud) |
| Web dashboard | open [`../dashboard/web_dashboard.html`](../dashboard/web_dashboard.html) in Chrome or Edge, press USB or Bluetooth |

On Windows, run `pio` from PowerShell. The platform is pinned to `espressif32@8.0.0` (Arduino core 3.3), so it builds the same way on a clean laptop. With BLE the image is about 1.4 MB, so the ESP32 envs use `huge_app.csv` (3 MB app, no OTA).

| Problem | Fix |
| --- | --- |
| Stuck at `Connecting....___` | Hold BOOT, tap EN, and release BOOT when `Writing at 0x...` starts. |
| `could not open port ... Access is denied` | Close the serial monitor or web dashboard first. |
| No COM port | Use a data cable (not a charge-only cable). Install the CP210x driver (CH340 on clones). |
| Garbage on the monitor | The baud rate is 460800. The ROM boot log at 115200 always looks like garbage. |

## Wiring (ESP32 DevKit V1)

| Signal | ESP32 pin | Goes to |
| --- | --- | --- |
| I2C SDA / SCL | GPIO21 / GPIO22 | Both VL53L5CX and the Grove accelerometer. The breakouts have their own pull-ups. |
| Grove accelerometer | Grove cable | red -> 3V3, black -> GND, white (SDA) -> GPIO21, yellow (SCL) -> GPIO22 |
| Ground ToF **LPn** | GPIO17 (`TX2`) | Ground VL53L5CX. It is moved to address 0x30 at boot. |
| Forward ToF **LPn** | GPIO16 (`RX2`) | Forward VL53L5CX. It stays at 0x29. |
| VL53L5CX I2C_RST / INT / PWREN | - | Leave unconnected. |
| Motor left / right | GPIO18 / GPIO19 | GPIO -> **470 ohm** -> base of a **PN2222**. Emitter -> GND. Collector -> motor **black** lead. Motor **red** lead -> **VIN (5 V)**. Put a 1N5817 across each motor, stripe to red, and a 100 uF capacitor across VIN-GND near the motors. |
| Button | GPIO27 | Wire it to GND (internal pull-up). The DevKit's BOOT button (GPIO0) does the same thing. |
| Status LED | GPIO2 | Through a resistor to the LED, then to GND. On = pod ON. |
| Power | USB | USB power bank into the board's USB port. There is no battery-sense pin, so battery % shows "-". |

Both VL53L5CX wake up at address 0x29, so **the two LPn wires are required**.

**Mounting** (DATA REPORT v3):
- The pod sits 0.72 m from the tip.
- The ground ToF points 19 deg above the shaft axis.
- The forward ToF points 73 deg above the shaft axis.

## Using the pod

| Button | What the pod does |
| --- | --- |
| power on | **OFF**. The lasers are held off (LPn low), so the pod draws little current. No buzzing, LED off, no telemetry. |
| click | **ON**. Both lasers load their firmware (~2-5 s), then one left-then-right ring. The LED turns on, alerts turn on, and telemetry starts. |
| double click (or hold 1.5 s) | **OFF** again. Lasers, motors and LED turn off, and the dashboard shows "Pod off". |

The lasers start only on the click, and the right motor starts 50 ms after the left one. Starting all of it at once made the power-bank supply dip far enough to reset the ESP32.

| Haptic pattern | Meaning |
| --- | --- |
| 3 fast strong pulses, both motors, repeating | drop-off ahead |
| 4 rapid strong pulses, repeating | drop-off ahead, little time left at this walking speed |
| 2 medium pulses on one side (Plan A) / slow long buzz (Plan B) | head-level obstacle on that side (both sides = ahead) |
| left-then-right ring | pod ON / calibration OK |
| one long buzz | calibration failed |
| 2 soft long pulses | walking faster than the look-ahead covers |
| 1 soft 600 ms pulse, then once a minute | "pod blind": the ground sensor or the accelerometer is down |

## Bring-up checklist

1. **Flash and open the monitor.** The banner `# SafeRoute Micro Pod imu:ok tof:off until ON haptics:ok` appears. Click the button and wait for `# lasers on (g:1 f:1)`.
2. **I2C scan** (`scan` while ON). You should see `0x19 0x29 0x30` (the accelerometer may show as 0x18, 0x53 or 0x4C instead of 0x19). Missing 0x30 = check the ground sensor's LPn on GPIO17. Missing 0x29 = check the forward sensor's LPn on GPIO16.
3. **Accelerometer.** Telemetry `"a"` is the cane angle. With the tip on the floor at ~45 deg, held still, it should read 45 +- 2. If it reads nonsense, hang the cane tip-down and type `axis`.
4. **ToF.** On a flat floor, `"dg"` is ~1410 mm at 37 deg. `"df"` is null in open space and shows a number when your hand is 1 m in front of the forward window.
5. **Motors.** Type `test l`, `test r` and `test b`. If a motor hums but does not shake, its off-centre weight is missing.
6. **Calibrate.** Put the tip on a flat floor, type `cal` and hold the cane still at ~40 deg for 3 s.
7. **Full loop.** Walk to an edge: you should feel 3 fast pulses on both motors and see `"alert":1`.

## Serial / BLE commands

- `stat`, `scan`, `cal`, `bat`
- `smart`, `naive`, `plan a|b`
- `test l|r|b`, `axis`
- `set L|beta|gamma|head <value>`, `save`, `reset`
- `walk start` + `walk <metres>` (speed calibration)
- `strength soft|normal|strong`
- `tele on|off`
- `on` / `off` (same as click / double click), `click`, `dclick`

**Telemetry:** one JSON line per 20 ms tick:
`{"t":ms,"a":deg,"yaw":0,"dg":mm,"df":mm,"dh":mm,"alert":code,"batt":pct,"yo":deg,"hf":mm,"m":"s","st":state,...}`
- Alert codes: 0 none, 1 drop, 2 head-left, 3 head-right, 4 head-ahead, 5 urgent drop.
- After a switch-off the pod sends `{"t":ms,"st":8}` for 1 s.
- Over USB: 50 lines/s at 460800 baud.
- Over Bluetooth (Nordic UART "SafeRoute"): every 5th line (10 Hz).

## How the logic works

1. **Measured cane angle** (`attitude.cpp`). The accelerometer is low-passed (0.25 s). On a flat floor the four central ground-sensor zones give the floor plane, and that corrects the angle error that sweeping adds.
2. **Sweep position** (`sweepFromAccel`). A swept cane turns back at each end of its arc, so the sideways acceleration gives offset = 23 deg x (-a_side) / (sqrt2 x RMS(a_side)), limited to +-30 deg. The offset is 0 when the cane is not being swept.
3. **Ground profile.** `dh = L sin(alpha) - d sin(alpha - beta)`. Anything below -8 cm is a drop. If the ground beam sees nothing where the floor should be, that counts as a deep drop.
4. **Forward.** Obstacle height `H = L sin(alpha0) + d sin(gamma - alpha0)`, using the **fixed** angle alpha0 = 38 deg.
5. **Filter.** Three rules:
   - **Confirmation:** 3 consecutive ToF frames for a drop, 2 for a head-level obstacle.
   - **Height band:** a head-level obstacle counts at 0.9 m < H < head + 0.1 m and x < 1.7 m; a drop counts below 8 cm.
   - **Corridor:** drops within +-35 deg of the sweep centre, head-level obstacles within +-25 deg and 0.3 m of the walking line.

   When the cane goes above 52 deg (the user has found an edge), alerts pause until it is back below 49 deg. Invalid frames and faulty sensors never alert, and faulty sensors are re-initialised.
6. **Two-point tip lift.** When the user is tapping, the tip height between taps follows from the cane angle and is added back to dh.
7. **Calibration.** With the cane held still on a flat floor, the floor plane and the accelerometer solve L and beta.

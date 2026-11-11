# Hardware

The SafeRoute pod schematic, drawn in KiCad. `schematic.svg` / `schematic.png` are rendered from [`kicad/smart_cane.kicad_sch`](kicad/smart_cane.kicad_sch) by [`render_schematic.py`](render_schematic.py), which draws every symbol, wire and label at its KiCad position in KiCad's default colours.

![SafeRoute schematic](schematic.png)

## Bill of Materials

| Ref | Part | Qty | Role |
| --- | --- | :---: | --- |
| U1 | ESP32-DevKitC (ESP32-WROOM-32) | 1 | Controller: sensor loop, haptics, USB and BLE telemetry |
| ToF_GND, ToF_FWD | VL53L1X time-of-flight sensor | 2 | Ground beam (drop-offs) and forward beam (head-level obstacles) |
| MPU | MPU-6050 6-axis IMU (accelerometer + gyro) | 1 | Cane angle, sweep position, tip taps |
| UDRV_L, DRV_R | DRV2605L haptic motor driver | 2 | Left / right haptic waveforms |
| M1, M2 | Vibrating mini motor disc (Adafruit 1201, ERM) | 2 | Left / right vibration |
| D1 | RGB LED, common cathode | 1 | Status |
| BTN1 | Push button | 1 | On / off |
| — | USB power bank | 1 | 5 V supply |

## Pin Map

| Net | ESP32 pin | Connects to |
| --- | --- | --- |
| `I2C0_SDA` / `I2C0_SCL` | GPIO21 / GPIO22 | Both VL53L1X, MPU-6050, left DRV2605L |
| `I2C1_SDA` / `I2C1_SCL` | GPIO25 / GPIO26 | Right DRV2605L |
| `TOF_GND_XSHUT` | GPIO17 | Ground VL53L1X XSHUT |
| `TOF_FWD_XSHUT` | GPIO16 | Forward VL53L1X XSHUT |
| `BTN_IN` | GPIO27 | Push button (other side to GND, internal pull-up) |
| `R_LED_R` / `R_LED_G` / `R_LED_B` | GPIO2 / GPIO4 / GPIO15 | RGB LED anodes (cathode to GND) |
| `3.3V` | 3V3 | Sensor, IMU and driver supply |
| `+5V` | 5V | USB power bank input |

## Design Notes

- **Two I2C buses.** The DRV2605L has a fixed I2C address (0x5A), so the two drivers cannot share a bus. The left driver sits on `I2C0` with the sensors; the right driver has its own bus, `I2C1`.
- **Separate XSHUT lines.** Both VL53L1X sensors power up at address 0x29. Holding one in reset with its XSHUT line lets the other be given a new address at boot, so both can share `I2C0`.
- **Dedicated haptic drivers.** The DRV2605L drives the ERM motors with overdrive and active braking, so pulses start and stop sharply. That keeps the left / right and short / long rhythms easy to tell apart through the cane shaft.
- **6-axis IMU.** The MPU-6050 gyro tracks the cane's sweep directly, for an accurate walking-corridor filter with both constant-contact and two-point technique.

## Mounting

| Parameter | Value |
| --- | --- |
| Cane length | 130 cm |
| Pod position | 0.72 m from the tip |
| Ground ToF angle | 19° above the shaft axis |
| Forward ToF angle | 73° above the shaft axis |

These values come from the real cane recordings in [`../experiment_data`](../experiment_data); the [DATA REPORT](../docs/SafeRoute_DATA_REPORT.pdf) explains how.

## Opening the Project

Open [`kicad/smart_cane.kicad_pro`](kicad/smart_cane.kicad_pro) in KiCad 10 or newer (the file was saved by KiCad 10.0). Custom symbols (VL53L1X, MPU-6050, DRV2605L, motor disc, button) are in [`kicad/library/`](kicad/library).

To regenerate the images after editing the schematic:

```bash
cd hardware
python render_schematic.py kicad/smart_cane.kicad_sch schematic
```

# Dashboard

Live view of the pod's telemetry: cane angle, drop height, alerts and pod state.

**Live version:** https://ktviy.github.io/SafeRoute_Smart_Cane/dashboard/

## Web Dashboard (no install)

Open the live link above, or open [`web_dashboard.html`](web_dashboard.html) locally, in **Chrome or Edge**. Power the pod, then press one of:

| Button | Connection |
| --- | --- |
| **USB** | Web Serial, 460800 baud, 50 Hz telemetry |
| **Bluetooth** | Web Bluetooth, Nordic UART service `SafeRoute`, 10 Hz |
| **Replay** | A `.jsonl` telemetry log recorded from the pod (use **Save log**, or `dashboard.py`) |

Web Serial and Web Bluetooth only work over HTTPS or from a local file. When served over HTTP you can also open `web_dashboard.html?replay=<log.jsonl>`.

[`index.html`](index.html) is the same page, served by GitHub Pages at the live link.

## Python Dashboard

[`dashboard.py`](dashboard.py) shows the same data in a matplotlib window and appends every line to `dashboard_log.jsonl`.

```bash
pip install matplotlib pyserial bleak

python dashboard/dashboard.py --serial COM5             # USB
python dashboard/dashboard.py --ble                     # Bluetooth
python dashboard/dashboard.py --file dashboard_log.jsonl  # replay a log
```

Keys over serial and BLE: `s` smart mode, `n` naive mode, `c` calibrate, `b` battery report.

## Telemetry Format

One JSON line per tick. The fields and alert codes are listed in the [firmware guide](../firmware/README.md#serial--ble-commands).

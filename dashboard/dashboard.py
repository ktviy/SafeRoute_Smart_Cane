#!/usr/bin/env python3
"""
SafeRoute live dashboard (EE Task F14): cane angle, drop height, alerts, mode.

  python dashboard/dashboard.py --serial COM5              # USB (pip install pyserial)
  python dashboard/dashboard.py --ble                      # Bluetooth (pip install bleak)
  python dashboard/dashboard.py --file dashboard_log.jsonl          # replay a recorded log

Keys (serial and BLE): s = smart mode, n = naive mode, c = calibrate, b = battery report.
Every line received is also appended to dashboard_log.jsonl for the poster numbers.
"""
import argparse
import asyncio
import collections
import json
import queue
import threading
import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
ALERTS = {0: "", 1: "DROP-OFF", 2: "HEAD LEFT", 3: "HEAD RIGHT", 4: "HEAD AHEAD", 5: "DROP-OFF (URGENT)"}
TECH = {"-": "?", "S": "still", "W": "sweeping", "T": "tapping", "H": "holding"}
STATES = {0: "booting", 1: "ok", 2: "paused (cane vertical)", 3: "ALERT", 4: "calibrating",
          5: "SENSOR FAULT", 6: "low battery", 7: "not calibrated", 8: "off",
          9: "standby (tap the tip 5 times to start)"}
WINDOW_S = 15

lines = queue.Queue()
send = None                       # function(str) for commands, set by the source


def serial_source(port, baud):
    global send
    import serial
    ser = serial.Serial(port, baud, timeout=0.1)
    send = lambda s: ser.write((s + "\n").encode())
    def run():
        buf = b""
        while True:
            buf += ser.read(512)
            *done, buf = buf.split(b"\n")
            for x in done:
                lines.put(x.decode(errors="ignore").strip())
    threading.Thread(target=run, daemon=True).start()


def ble_source(name):
    global send
    from bleak import BleakClient, BleakScanner
    loop = asyncio.new_event_loop()
    state = {"buf": "", "client": None}

    async def main():
        dev = await BleakScanner.find_device_by_name(name, timeout=15)
        if dev is None:
            print(f"no BLE device called {name}")
            return
        async with BleakClient(dev) as client:
            state["client"] = client
            def on_data(_, data):
                state["buf"] += data.decode(errors="ignore")
                *done, state["buf"] = state["buf"].split("\n")
                for x in done:
                    lines.put(x.strip())
            await client.start_notify(NUS_TX, on_data)
            while client.is_connected:
                await asyncio.sleep(0.5)

    def write(s):
        if state["client"]:
            asyncio.run_coroutine_threadsafe(state["client"].write_gatt_char(NUS_RX, (s + "\n").encode()), loop)
    send = write
    threading.Thread(target=lambda: loop.run_until_complete(main()), daemon=True).start()


def file_source(path):
    def run():
        t_prev = None
        for x in open(path):
            try:
                t = json.loads(x)["t"]
            except (ValueError, KeyError):
                continue
            if t_prev is not None:
                time.sleep(max(0, (t - t_prev) / 1000))
            t_prev = t
            lines.put(x.strip())
    threading.Thread(target=run, daemon=True).start()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--serial")
    p.add_argument("--baud", type=int, default=460800)
    p.add_argument("--ble", action="store_true")
    p.add_argument("--name", default="SafeRoute")
    p.add_argument("--file")
    a = p.parse_args()
    if a.serial:
        serial_source(a.serial, a.baud)
    elif a.ble:
        ble_source(a.name)
    elif a.file:
        file_source(a.file)
    else:
        p.error("choose --serial PORT, --ble or --file")

    n = int(WINDOW_S * 50)
    T, A, DH, AL = (collections.deque(maxlen=n) for _ in range(4))
    last = {}
    log = open("dashboard_log.jsonl", "a")
    alerts_seen = collections.Counter()

    fig, ax = plt.subplots(3, 1, figsize=(11, 7), sharex=True, gridspec_kw=dict(height_ratios=[2, 2, 1]))
    la, = ax[0].plot([], [], lw=1.5)
    ax[0].axhspan(52, 90, color="0.9")           # PAUSE_ON_DEG (130 cm cane): alerts paused (edge found, stairs)
    ax[0].set_ylabel("cane angle (deg)")
    ax[0].set_ylim(0, 90)
    ld, = ax[1].plot([], [], ".", ms=3)
    ax[1].axhline(-8, color="tab:red", ls=":", lw=1)
    ax[1].set_ylabel("ground drop dh (cm)")
    ax[1].set_ylim(-40, 15)
    lal, = ax[2].plot([], [], drawstyle="steps-post", color="tab:red")
    ax[2].set_ylim(-0.2, 5.5)
    ax[2].set_yticks(range(6), ["-", "drop", "head L", "head R", "head", "drop!"])
    ax[2].set_xlabel("time (s)")
    title = fig.suptitle("")

    def on_key(e):
        cmd = {"s": "smart", "n": "naive", "c": "cal", "b": "bat"}.get(e.key)
        if cmd and send:
            send(cmd)
    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_):
        while not lines.empty():
            x = lines.get()
            if not x.startswith("{"):
                if x:
                    print(x)
                continue
            try:
                f = json.loads(x)
            except ValueError:
                continue
            log.write(x + "\n")
            if f.get("alert") and f["alert"] != last.get("alert"):
                alerts_seen[ALERTS.get(f["alert"], "?")] += 1
            last.update(f)
            T.append(f["t"] / 1000)
            A.append(f["a"])
            DH.append(f["dh"] / 10 if f.get("dh") is not None else float("nan"))
            AL.append(f["alert"])
        if T:
            la.set_data(T, A)
            ld.set_data(T, DH)
            lal.set_data(T, AL)
            ax[2].set_xlim(T[-1] - WINDOW_S, T[-1] + 0.5)
            mode = "SMART" if last.get("m") == "s" else "NAIVE"
            batt = f"{last['batt']}%" if last.get("batt") is not None else "?"
            v = f"{last['v']:.2f} m/s" if last.get("v") is not None else "? m/s"
            title.set_text(f"{mode} mode | {TECH.get(last.get('tech'), '?')} {v} | "
                           f"{STATES.get(last.get('st'), '?')} | battery {batt} | "
                           f"{ALERTS.get(last.get('alert'), '')}   alerts so far: "
                           + ", ".join(f"{k} {v}" for k, v in alerts_seen.items()))
            title.set_color("tab:red" if last.get("alert") else "black")
        return la, ld, lal, title

    _anim = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    main()

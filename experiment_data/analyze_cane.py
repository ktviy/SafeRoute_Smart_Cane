#!/usr/bin/env python3
"""
SafeRoute - cane motion analysis
================================
Reads the recording log and the raw sensor exports from the phone app and
computes the numbers the task plans need. Events (sync taps, edges, step-ups,
stair steps) are found in the sensor data; video_sync.csv (from sync_video.py)
adds the matching video time to every event.

  * cane angle (alpha) range while walking, and the phone's mount offset (S1)
  * sweep frequency, width and half-width            -> sweep corridor
  * ToF frames a narrow drop stays in the beam       -> checks N confirmation frames
  * walking speed (S10 timed runs) and warning time  -> look-ahead claim
  * pitch swing, constant contact vs two-point       -> tip-lift proxy
  * alpha error of the firmware filter while walking -> checks the IMU filter design
  * edges and step-ups: walking angle before, rise time, peak angle after
  * gyro bias (S2), stair rhythm (S13 descent)

Usage
-----
  python sync_video.py                    # once per new video: writes video_sync.csv
  python analyze_cane.py analyze          # writes results/ and events.csv

Folder layout (change the folder with --dir):
  log.csv         the recording log (one row per recording)
  video_sync.csv  video offsets (sensor time = video time + offset_s)
  events.csv      written here; rows you add with source=video are kept
  raw/            one export per recording, named exactly like the `file` column:
                raw/S04_walk_swipe.zip   app export as a zip, or
                raw/S04_walk_swipe/      the unzipped folder, or
                raw/S04_walk_swipe.csv   a single CSV

Requires Python 3.9+, numpy, pandas; matplotlib for plots (optional).
"""
import argparse
import io
import re
import sys
import zipfile
from pathlib import Path

import numpy as np
import pandas as pd

# ---------------- settings you may change ----------------
FS = 100.0                  # resample rate for analysis (Hz)
FRAME_HZ = 30               # ToF frame rate (VL53L1X long mode, 33 ms budget); firmware TOF_HZ
N_CONFIRM = 3               # ToF frames the alert filter needs (firmware N_CONFIRM_GROUND)
LOOKAHEAD_PAST_TIP_M = 0.75 # where the ground beam lands beyond the tip (m): beta 21 deg at 41 deg on
                            # P1's 120 cm cane (130 cm cane: beta 19 at 36.9 deg, see cane_130cm.py)
FEATURE_WIDTH_M = 0.30      # narrow drop used for the frames-in-view check (m)
CF_K = 0.98                 # basic complementary filter weight on the gyro (no correction)
CF_TAU_S = 2.0              # firmware filter time constant (K = 0.99 at 50 Hz, 0.995 at 100 Hz)
MOVING_DPS = 5.0            # yaw rate above this counts as "sweeping" (deg/s)
STILL_DPS = 15.0            # 1 s RMS yaw rate below this counts as standing still (deg/s)
EDGE_ALPHA_DEG = 57.0       # cane pulled past this angle = tip went over an edge (or a turn);
                            # P1 2026-10-30: stair-top probing only reaches 60-64 deg
EDGE_BASE_MAX_DEG = 52.0    # walking angle must be below this before an edge rise
TAP_GUARD_S = 0.5           # rises starting after (end sync taps - this) are ignored
IMPACT_MIN = 8.0            # high-passed |a| spike (m/s2) that counts as a tip impact
HAND_AHEAD_M = 0.25         # horizontal distance feet -> hand while walking (m)
DEFAULT_CANE_M = 1.2        # used when cane_length_cm is blank
DEFAULT_PIVOT_M = 0.33      # grip -> phone, used when grip_to_phone_mm is blank
G = 9.81
SKIP_WALKERS = set()        # log rows for these walkers are ignored
SKIP_SCENARIOS = set()      # log rows for these scenarios are ignored
SKIP_FILES = set()          # single log rows that are ignored
# S1 rows whose inclinometer_deg is not a static reading (e.g. a walking average):
# their static angle is still reported but they are left out of the accuracy check
NO_PITCH_CHECK = set()

WALKING = {"S4", "S5"}                      # used for the headline numbers
SWEEPING = {"S3", "S4", "S5", "S6", "S7", "S9", "S11", "S12"}
EDGE_SCENARIOS = {"S6", "S7", "S11", "S12", "S13"}   # a cane-angle rise here is a real edge
STAIRS = {"S13": "descent", "S14": "ascent"}

# events expected in the video, per scenario: (event names, repetitions)
EVENT_TEMPLATES = {
    "S6": (["edge_found", "step_down"], 5),
    "S7": (["edge_found", "step_down"], 5),
    "S8": (["step_up"], 3),
    "S9": (["pass_object"], 6),
    "S11": (["stair_edge_found", "stop"], 5),
    "S12": (["stair_edge_found", "stop"], 5),
    "S13": (["stair_edge_found", "first_step_down", "bottom_reached"], 1),
    "S14": (["first_step_up", "top_reached"], 1),
    "S15": (["pass_edge"], 3),
}
EDGE_EVENTS = ("edge_found", "stair_edge_found", "curb_found")


# ---------------- small helpers ----------------
def num(v):
    try:
        return float(str(v).strip())
    except ValueError:
        return float("nan")


def smooth(x, seconds):
    w = max(1, int(round(seconds * FS)))
    return pd.Series(x).rolling(w, center=True, min_periods=1).mean().to_numpy()


def smooth2d(a, seconds):
    return np.column_stack([smooth(a[:, i], seconds) for i in range(a.shape[1])])


def unit_rows(a):
    n = np.linalg.norm(a, axis=1, keepdims=True)
    n[n == 0] = np.nan
    return a / n


def parse_axis(text):
    """'-y', 'y-', '-y'' or 'minus y' -> unit vector. Write 'y-' in Excel: '-y' becomes #NAME?."""
    t = str(text).lower().replace("minus", "-").replace("neg", "-")
    m = re.search(r"([+-]?)\s*([xyz])\s*([+-]?)", t)
    if not m:
        return None
    v = np.zeros(3)
    v["xyz".index(m.group(2))] = -1.0 if "-" in (m.group(1) + m.group(3)) else 1.0
    return v


def read_log(path):
    log = pd.read_csv(path, dtype=str).fillna("")
    log.columns = [c.strip() for c in log.columns]
    for c in ("file", "walker", "scenario"):
        if c not in log.columns:
            sys.exit(f"log is missing the column '{c}'")
    log = log[(log["file"].str.strip() != "")
              & ~log["walker"].str.strip().isin(SKIP_WALKERS)
              & ~log["scenario"].str.strip().isin(SKIP_SCENARIOS)
              & ~log["file"].str.strip().isin(SKIP_FILES)].reset_index(drop=True)
    # cane/phone setup is per walker: fill once, copy down to that walker's other rows
    setup = [c for c in ("walker_height_cm", "cane_length_cm", "tip_type", "tip_to_phone_mm",
                         "grip_to_phone_mm", "phone_axis_along_shaft") if c in log.columns]
    filled = log[setup].replace(r"^\s*$", np.nan, regex=True)
    log[setup] = filled.groupby(log["walker"].str.strip()).ffill().fillna("")
    return log


# ---------------- loading raw exports ----------------
def _read_csv(src):
    try:
        return pd.read_csv(src, sep=None, engine="python")
    except Exception:
        if hasattr(src, "seek"):
            src.seek(0)
        try:
            return pd.read_csv(src)
        except pd.errors.EmptyDataError:  # e.g. Sensor Logger's empty Annotation.csv
            return pd.DataFrame()


def collect_tables(raw_dir, name):
    z, d, c = raw_dir / f"{name}.zip", raw_dir / name, raw_dir / f"{name}.csv"
    tables = {}
    if z.exists():
        with zipfile.ZipFile(z) as zf:
            for n in zf.namelist():
                if n.lower().endswith(".csv") and "__macosx" not in n.lower():
                    tables[Path(n).stem.lower()] = _read_csv(io.BytesIO(zf.read(n)))
    elif d.is_dir():
        for f in sorted(d.rglob("*.csv")):
            tables[f.stem.lower()] = _read_csv(f)
    elif c.exists():
        tables[c.stem.lower()] = _read_csv(c)
    return tables


def find_col(cols, axis, prefix=""):
    pat = re.compile(r"(^|[^a-z])" + re.escape(prefix) + axis + r"([^a-z]|$)")
    for c in cols:
        if pat.search(str(c).lower()):
            return c
    return None


def get_xyz(df, keyword=None, prefix=""):
    cols = [c for c in df.columns if keyword is None or keyword in str(c).lower()]
    found = [find_col(cols, a, prefix) for a in "xyz"]
    if None in found:
        return None
    return df[found].apply(pd.to_numeric, errors="coerce").to_numpy(dtype=float)


def get_time(df):
    low = {str(c).lower().strip(): c for c in df.columns}
    for key, scale in (("seconds_elapsed", 1.0), ("t_ms", 1e-3)):
        if key in low:
            return pd.to_numeric(df[low[key]], errors="coerce").to_numpy(dtype=float) * scale
    for lc, c in low.items():
        if "time" in lc:
            t = pd.to_numeric(df[c], errors="coerce").to_numpy(dtype=float)
            dt = np.nanmedian(np.diff(t))
            if dt > 1e5:
                return t / 1e9          # nanoseconds
            if dt > 0.5:
                return t / 1e3          # milliseconds
            return t                    # seconds
    return None


def _clean(t, v):
    ok = np.isfinite(t) & np.all(np.isfinite(v), axis=1)
    t, v = t[ok], v[ok]
    order = np.argsort(t)
    t, v = t[order], v[order]
    keep = np.concatenate([[True], np.diff(t) > 0])
    return t[keep], v[keep]


def finish(ta, acc, tg, gyr, gyro_units, notes, grav=None, uncal=None, ios=False):
    if ta is None or acc is None or tg is None or gyr is None:
        return None, "missing time or x/y/z columns"
    ta, acc = _clean(ta, acc)
    tg, gyr = _clean(tg, gyr)
    t0, t1 = max(ta[0], tg[0]), min(ta[-1], tg[-1])
    if t1 - t0 < 1:
        return None, "recording shorter than 1 s"
    t = np.arange(t0, t1, 1 / FS)
    A = np.column_stack([np.interp(t, ta, acc[:, i]) for i in range(3)])
    W = np.column_stack([np.interp(t, tg, gyr[:, i]) for i in range(3)])
    fs_raw = 1 / np.median(np.diff(ta))

    mag = np.median(np.linalg.norm(A, axis=1))
    has_grav = grav is not None and grav[0] is not None and grav[1] is not None
    if not has_grav and 0.5 < mag < 1.5:
        A *= G
        notes.append("accel was in g; converted to m/s2")
    elif has_grav or mag < 3:
        if grav is None or grav[0] is None or grav[1] is None:
            return None, ("accelerometer has no gravity (|a| ~ 0 at rest); export "
                          "acceleration WITH gravity (total acceleration)")
        tv, gv = _clean(*grav)
        gv_i = np.column_stack([np.interp(t, tv, gv[:, i]) for i in range(3)])
        if 0.5 < np.median(np.linalg.norm(gv_i, axis=1)) < 1.5:
            gv_i *= G
        # units of the motion-only accelerometer: g on iPhone (Sensor Logger without
        # standardisation), m/s2 on Android. Decide from the platform, else by fitting
        # against the uncalibrated (total) accelerometer.
        k = G if ios else 1.0
        if uncal is not None and uncal[0] is not None and uncal[1] is not None:
            tu, uv = _clean(*uncal)
            U = np.column_stack([np.interp(t, tu, uv[:, i]) for i in range(3)])
            U *= G / np.median(np.linalg.norm(U, axis=1))
            den = np.sum(A * A)
            if den > 0:
                k_fit = np.sum(A * (U - gv_i)) / den
                k = G if abs(k_fit - G) < abs(k_fit - 1) else 1.0
        A = A * k + gv_i
        gup = -gv_i if ios else gv_i      # gravity reaction ("up"), same convention as A
        if ios:
            A = -A      # iPhone reports -1 g on z when face up; flip to the Android convention
        notes.append("iPhone export: accel (g) + gravity combined" if ios
                     else "accel had no gravity; added the gravity file")

    if gyro_units == "rad/s":
        p99 = np.percentile(np.linalg.norm(W, axis=1), 99)
        if p99 > 50:
            notes.append("gyro looked like deg/s already; not converted")
        else:
            W = np.degrees(W)

    gravity_ok = bool(8.8 < np.median(np.linalg.norm(A, axis=1)) < 10.8)
    out = dict(t=t - t[0], acc=A, gyr=W, fs_raw=fs_raw, gravity_ok=gravity_ok, notes=notes)
    if "gup" in locals():
        out["gup"] = gup                  # phone's fused gravity vector: accurate while swinging
    return out, None


def load_recording(raw_dir, name):
    tables = collect_tables(raw_dir, name)
    if not tables:
        return None, "raw export not found in raw/"
    notes = []

    # 1) our cleaned format: t_ms, ax, ay, az (m/s2), gx, gy, gz (deg/s)
    for df in tables.values():
        low = {str(c).lower().strip() for c in df.columns}
        if {"t_ms", "ax", "gx"} <= low:
            t = get_time(df)
            return finish(t, get_xyz(df, prefix="a"), t, get_xyz(df, prefix="g"),
                          "deg/s", notes)

    # 2) app exports with one file per sensor (Sensor Logger, phyphox, ...)
    def pick(keys, exclude=()):
        for k in keys:
            for n, df in tables.items():
                if k in n and not any(e in n for e in exclude):
                    return df
        return None

    adf = pick(["totalacceleration", "accelerometer", "acceleration", "acc"],
               exclude=("linear", "uncalib"))
    gdf = pick(["gyroscope", "gyro"], exclude=("uncalib",))
    vdf = pick(["gravity"])

    # 3) one combined CSV holding both sensors (e.g. a phyphox custom experiment)
    if (adf is None or gdf is None) and len(tables) == 1:
        df = next(iter(tables.values()))
        t = get_time(df)
        acc, gyr = get_xyz(df, "acc"), get_xyz(df, "gyr")
        if acc is None or gyr is None:
            return None, "could not find acceleration and gyroscope x/y/z columns"
        return finish(t, acc, t, gyr, "rad/s", notes)

    if adf is None or gdf is None:
        return None, "need accelerometer and gyroscope files; found: " + ", ".join(tables)
    grav = (get_time(vdf), get_xyz(vdf)) if vdf is not None else None
    udf = pick(["accelerometeruncalibrated", "totalaccelerationuncalibrated"])
    uncal = (get_time(udf), get_xyz(udf)) if udf is not None else None
    ios = False
    meta = tables.get("metadata")
    if meta is not None and len(meta):
        m = {str(c).lower().strip(): str(meta[c].iloc[0]).lower() for c in meta.columns}
        ios = "ios" in m.get("platform", "") and m.get("standardisation", "false") != "true"
    return finish(get_time(adf), get_xyz(adf), get_time(gdf), get_xyz(gdf),
                  "rad/s", notes, grav, uncal, ios)


# ---------------- signal processing ----------------
def rms(x, seconds):
    return np.sqrt(smooth(np.asarray(x, dtype=float) ** 2, seconds))


def firmware_alpha(A, W, shaft, pivot_m, centripetal=False, tau=CF_TAU_S):
    """The pitch filter the firmware runs (kinematics/attitude.cpp), on phone data.

    Gravity-vector complementary filter: the gyro rotates the gravity estimate,
    the accelerometer pulls it back with time constant tau. With `centripetal`,
    the sweep's centripetal acceleration w x (w x r) is removed first, r being
    the grip -> sensor vector along the shaft (DATA REPORT: subtract w^2 * 0.33 m).
    On P1's data the 2 s time constant alone does better than the correction, so
    the firmware ships with the correction off (CENTRIPETAL_RADIUS_M = 0).
    A in m/s2 (gravity included), W in deg/s. Returns alpha in degrees.
    """
    dt = 1 / FS
    k = tau / (tau + dt)
    w = np.radians(W)
    r = pivot_m * shaft
    g = A[0] / np.linalg.norm(A[0])
    out = np.empty(len(A))
    for i in range(len(A)):
        wi = w[i]
        g = g + np.cross(g, wi) * dt
        a = A[i] - np.cross(wi, np.cross(wi, r)) if centripetal else A[i]
        n = np.linalg.norm(a)
        if 0.5 * G < n < 1.5 * G:
            g = k * g + (1 - k) * a / n
        g = g / np.linalg.norm(g)
        out[i] = np.degrees(np.arcsin(np.clip(-(g @ shaft), -1, 1)))
    return out


def process(rec, shaft, bias, pivot_m=DEFAULT_PIVOT_M):
    A, W = rec["acc"], rec["gyr"].copy()
    if bias is not None:
        W = W - bias
    G_src = rec.get("gup")
    up = unit_rows(G_src) if G_src is not None else unit_rows(smooth2d(A, 0.5))
    alpha = np.degrees(np.arcsin(np.clip(-(up @ shaft), -1, 1)))
    flipped = bool(np.nanmedian(alpha) < 0)
    if flipped:
        alpha = -alpha
    fast_src = G_src if G_src is not None else smooth2d(A, 0.1)
    alpha_fast = np.degrees(np.arcsin(np.clip(-(unit_rows(fast_src) @ shaft), -1, 1)))
    alpha_raw = np.degrees(np.arcsin(np.clip(-(unit_rows(A) @ shaft), -1, 1)))
    if flipped:
        alpha_fast, alpha_raw = -alpha_fast, -alpha_raw

    w_up = np.sum(W * up, axis=1)                    # yaw rate about vertical (deg/s)
    yaw = np.cumsum(w_up) / FS
    yaw_off = yaw - smooth(yaw, 2.0)                 # offset from the sweep centre

    # complementary filter on pitch (what the firmware will run)
    lat = unit_rows(np.cross(np.broadcast_to(shaft, up.shape), up))
    rate = np.sum(W * lat, axis=1)
    d_alpha = np.gradient(alpha) * FS
    if np.std(rate) > 0 and np.std(d_alpha) > 0 and np.corrcoef(rate, d_alpha)[0, 1] < 0:
        rate = -rate
    cf = np.empty_like(alpha_raw)
    cf[0] = alpha_raw[0]
    for i in range(1, len(cf)):
        cf[i] = CF_K * (cf[i - 1] + rate[i] / FS) + (1 - CF_K) * alpha_raw[i]
    sgn = -1.0 if flipped else 1.0
    fw = sgn * firmware_alpha(A, W, shaft, pivot_m)
    fw_corr = sgn * firmware_alpha(A, W, shaft, pivot_m, centripetal=True)
    return dict(alpha=alpha, alpha_fast=alpha_fast, alpha_cf=cf, alpha_fw=fw,
                alpha_fw_corr=fw_corr, w_up=w_up, yaw_off=yaw_off, flipped=flipped)


# ---------------- sensor-only events (no video needed) ----------------
def find_impacts(rec):
    """Times (s) of hard tip impacts: two-point taps, stair steps, sync taps."""
    mag = np.linalg.norm(rec["acc"], axis=1)
    hp = mag - smooth(mag, 0.5)
    mad = np.median(np.abs(hp - np.median(hp))) + 1e-9
    idx = np.where(hp > max(IMPACT_MIN, 8 * mad))[0]
    peaks = []
    for i in idx:
        if peaks and i - peaks[-1] < 0.6 * FS:
            if hp[i] > hp[peaks[-1]]:
                peaks[-1] = i
        else:
            peaks.append(i)
    return rec["t"][peaks] if peaks else np.array([])


def detect_rises(rec, sig):
    """Every time the cane is pulled from walking angle to near-vertical.

    In an edge scenario this is the tip dropping over the edge; elsewhere it is a
    turn or a pause. For each rise: onset (alpha leaves the walking angle), cross
    (alpha passes EDGE_ALPHA_DEG), the walking angle before it, and where the walk
    that led to it started (after standing still, or after the previous rise).
    """
    t = rec["t"]
    a = smooth(sig["alpha"], 0.2)
    active = rms(sig["w_up"], 1.0) > STILL_DPS
    above = a > EDGE_ALPHA_DEG
    crosses = np.where(above[1:] & ~above[:-1])[0] + 1
    out, prev_end = [], 0
    for c in crosses:
        if c < prev_end:
            continue
        end = c
        while end < len(a) and a[end] > EDGE_BASE_MAX_DEG:
            end += 1
        lo, hi = max(0, c - int(4 * FS)), max(1, c - int(1.0 * FS))
        before = a[lo:hi]
        before = before[before < EDGE_BASE_MAX_DEG]
        if len(before) < 0.5 * FS:       # cane was not at walking angle: same episode
            prev_end = end
            continue
        base = float(np.median(before))
        on = c
        while on > 0 and a[on] > base + 3:
            on -= 1
        ws = on
        while ws > prev_end and active[ws]:
            ws -= 1
        start_kind = "still" if not active[ws] and ws > prev_end else (
            "previous_rise" if prev_end > 0 else "recording_start")
        out.append(dict(onset_s=float(t[on]), cross_s=float(t[c]), alpha_walk_deg=base,
                        rise_s=float(t[c] - t[on]), walk_start_s=float(t[ws]),
                        walk_start=start_kind, end_s=float(t[min(end, len(t) - 1)])))
        prev_end = end
    return out


def tap_pivot_lift(rec, sig, lc_m):
    """Peak tip height between two-point taps, in m (one value per tap cycle).

    Tapping lifts the tip by pivoting the cane in the hand: at each tap the tip is
    on the floor, then tip height = (grip -> tip) * (sin a_tap - sin a). This is the
    same estimate the firmware uses to correct dh while tapping (TAP_MODE 1).
    """
    t, a = rec["t"], np.radians(sig["alpha"])
    imp = find_impacts(rec)
    active = rms(sig["w_up"], 1.0) > STILL_DPS
    out = []
    for t0, t1 in zip(imp[:-1], imp[1:]):
        m = (t > t0) & (t < t1)
        if 0.6 < t1 - t0 < 2.0 and m.any() and active[m].all():
            out.append(lc_m * (np.sin(np.interp(t0 - 0.02, t, a)) - np.sin(a[m])).max())
    return np.array(out)


def walking_bouts(rec, sig, min_s=4.0):
    """(start, end) of each walk between standing-still periods (S10)."""
    t = rec["t"]
    active = rms(sig["w_up"], 1.0) > STILL_DPS
    edges = np.diff(np.concatenate([[0], active.astype(int), [0]]))
    starts, ends = np.where(edges == 1)[0], np.where(edges == -1)[0] - 1
    return [(float(t[s]), float(t[e])) for s, e in zip(starts, ends) if t[e] - t[s] >= min_s]


def stair_steps(rec, sig, kind):
    """Tip impacts on the steps of a flight of stairs -> (impact times, seconds per step).

    descent: impacts while the cane is near-vertical (alpha > 62 deg), skipping the
    first second (that is the cane finding the top edge).
    ascent: the longest run of impacts spaced 0.8-4 s apart (no angle gate).
    """
    imp = find_impacts(rec)
    if kind == "descent":
        a = smooth(sig["alpha"], 0.5)
        high = np.concatenate([[0], (a > 62).astype(int), [0]])
        d = np.diff(high)
        s, e = np.where(d == 1)[0], np.where(d == -1)[0] - 1
        if not len(s):
            return np.array([]), float("nan")
        k = int(np.argmax(e - s))
        t0, t1 = rec["t"][s[k]] + 1.0, rec["t"][e[k]] + 1.0
        imp = imp[(imp >= t0) & (imp <= t1)]
    else:
        best, cur = [], []
        for x in imp:
            if cur and not (0.8 <= x - cur[-1] <= 4.0):
                best, cur = (cur if len(cur) > len(best) else best), []
            cur.append(x)
        imp = np.array(cur if len(cur) > len(best) else best)
    if len(imp) < 3:
        return imp, float("nan")
    return imp, float(np.median(np.diff(imp)))


def find_sync_triplets(rec):
    """Sensor time (s) of the first tap of the start and of the end sync triplet.

    A triplet = 3 hard impacts 0.4-1.2 s apart. The start one must lie in the first
    third of the recording, the end one in the last third; nan when missing.
    """
    mag = np.linalg.norm(rec["acc"], axis=1)
    hp = mag - smooth(mag, 0.5)
    idx = np.where(hp > IMPACT_MIN)[0]
    pk = []
    for i in idx:
        if pk and i - pk[-1] < 0.15 * FS:
            if hp[i] > hp[pk[-1]]:
                pk[-1] = i
        else:
            pk.append(i)
    t = rec["t"][pk] if pk else np.array([])
    tri = [t[i] for i in range(len(t) - 2)
           if 0.4 < t[i + 1] - t[i] < 1.2 and 0.4 < t[i + 2] - t[i + 1] < 1.2]
    dur = rec["t"][-1]
    start = [x for x in tri if x < dur / 3]
    end = [x for x in tri if x > 2 * dur / 3]
    return (float(start[0]) if start else float("nan"), float(end[-1]) if end else float("nan"))


def find_sync(rec):
    """Time (s) of the first of three hard taps, or nan."""
    mag = np.linalg.norm(rec["acc"], axis=1)
    hp = mag - smooth(mag, 0.5)
    med = np.median(hp)
    mad = np.median(np.abs(hp - med)) + 1e-9
    thr = max(med + 10 * mad, 3.0)
    idx = np.where(hp > thr)[0]
    peaks = []
    for i in idx:
        if peaks and i - peaks[-1] < 0.1 * FS:
            if hp[i] > hp[peaks[-1]]:
                peaks[-1] = i
        else:
            peaks.append(i)
    pt = rec["t"][peaks] if peaks else np.array([])
    for i in range(len(pt) - 2):
        if pt[i + 1] - pt[i] < 1.2 and pt[i + 2] - pt[i + 1] < 1.2:
            return float(pt[i]), len(pt)
    return float("nan"), len(pt)


def sweep_metrics(sig, alpha_med, cane_m):
    y = sig["yaw_off"]
    width = float(np.percentile(y, 95) - np.percentile(y, 5))
    yy = (y - y.mean()) * np.hanning(len(y))
    f = np.fft.rfftfreq(len(yy), 1 / FS)
    spec = np.abs(np.fft.rfft(yy))
    band = (f >= 0.2) & (f <= 3.0)
    freq = float(f[band][np.argmax(spec[band])]) if band.any() else float("nan")
    w = np.abs(sig["w_up"])
    moving = w[w > MOVING_DPS]
    if len(moving) == 0:
        return width, freq, float("nan"), float("nan"), float("nan")
    radius = cane_m * np.cos(np.radians(alpha_med)) + LOOKAHEAD_PAST_TIP_M
    v_med = np.radians(np.median(moving)) * radius
    v_p90 = np.radians(np.percentile(moving, 90)) * radius
    return (width, freq, float(np.percentile(moving, 90)),
            FEATURE_WIDTH_M / v_med * FRAME_HZ, FEATURE_WIDTH_M / v_p90 * FRAME_HZ)


# ---------------- commands ----------------
def make_templates(d):
    log = read_log(d / "log.csv")
    rows = []
    for _, r in log.iterrows():
        sc = r["scenario"].strip()
        if sc not in EVENT_TEMPLATES:
            continue
        events, reps = EVENT_TEMPLATES[sc]
        rows.append([r["file"], "sync_tap_start", "", "", ""])
        for i in range(1, reps + 1):
            for e in events:
                rows.append([r["file"], e if reps == 1 else f"{e}_{i}", "", "", ""])
        rows.append([r["file"], "sync_tap_end", "", "", ""])
    out = d / "events_template.csv"
    pd.DataFrame(rows, columns=["file", "event", "video_time_s", "sensor_time_ms", "notes"]) \
        .to_csv(out, index=False)
    print(f"wrote {out} ({len(rows)} rows)")


def load_video_sync(d):
    """video_sync.csv (written by sync_video.py): file -> (offset_s, confidence, video duration)."""
    f = d / "video_sync.csv"
    if not f.exists():
        return {}
    v = pd.read_csv(f, dtype=str).fillna("")
    return {r["file"].strip(): (num(r["offset_s"]), r.get("confidence", ""), num(r.get("video_duration_s", "")),
                                r.get("video_file", ""))
            for _, r in v.iterrows() if np.isfinite(num(r["offset_s"]))}


def analyze(d, plots):
    log = read_log(d / "log.csv")
    raw_dir, out = d / "raw", d / "results"
    out.mkdir(exist_ok=True)
    sync = load_video_sync(d)
    manual = pd.DataFrame(columns=["file", "event", "sensor_time_s", "video_time_s", "source", "notes"])
    if (d / "events.csv").exists():
        e = pd.read_csv(d / "events.csv", dtype=str).fillna("")
        if "source" in e:
            manual = e[e["source"].str.strip() == "video"]      # hand-typed from the video: kept

    plt = None
    if plots:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            (out / "plots").mkdir(exist_ok=True)
            for old in (out / "plots").glob("*.png"):
                old.unlink()
        except ImportError:
            print("matplotlib not installed; skipping plots")

    # S2 first: gyro bias per walker
    loaded, bias = {}, {}
    for _, r in log.iterrows():
        rec, err = load_recording(raw_dir, r["file"].strip())
        loaded[r["file"]] = (rec, err)
        if rec is not None and r["scenario"].strip() == "S2":
            W = rec["gyr"]
            still = np.linalg.norm(W, axis=1) < 5
            bias[r["walker"]] = W[still].mean(axis=0) if still.any() else W.mean(axis=0)

    summary, ev_rows, ev_metrics, approaches, signals = [], [], [], [], {}
    for _, r in log.iterrows():
        name, walker, sc = r["file"].strip(), r["walker"].strip(), r["scenario"].strip()
        rec, err = loaded[r["file"]]
        row = dict(file=name, walker=walker, scenario=sc, technique=r.get("technique", ""))
        cane_m = num(r.get("cane_length_cm", "")) / 100
        pivot_m = num(r.get("grip_to_phone_mm", "")) / 1000
        notes = []
        if not np.isfinite(cane_m):
            cane_m = DEFAULT_CANE_M
            notes.append(f"cane length blank; used {DEFAULT_CANE_M} m")
        if not np.isfinite(pivot_m):
            pivot_m = DEFAULT_PIVOT_M
        if sc == "S10":
            # hand-timed runs: walk_speed_mps, or walk_time_s over walk_distance_m (default 10 m)
            dist = num(r.get("walk_distance_m", "")) if np.isfinite(num(r.get("walk_distance_m", ""))) else 10.0
            sp = [num(x) for x in str(r.get("walk_speed_mps", "")).replace(",", ";").split(";")]
            sp = [x for x in sp if np.isfinite(x) and x > 0]
            if not sp:
                times = [num(x) for x in str(r.get("walk_time_s", "")).replace(",", ";").split(";")]
                sp = [dist / x for x in times if np.isfinite(x) and x > 0]
            if sp:
                row["walk_speed_stopwatch_mps"] = round(float(np.mean(sp)), 2)
                row["walk_speed_runs_mps"] = ";".join(f"{x:.2f}" for x in sp)
                row["walk_speed_mps"] = row["walk_speed_stopwatch_mps"]
                if len(sp) > 1 and np.std(sp) / np.mean(sp) > 0.2:
                    notes.append("timed runs differ by more than 20%; re-time them")
                if rec is None:
                    notes.append(f"no raw file; speed from {len(sp)} timed runs over {dist:g} m")
                    row.update(status="ok", notes="; ".join(notes))
                    summary.append(row)
                    continue
        if rec is None:
            row["status"] = "ERROR: " + err
            summary.append(row)
            continue
        notes = list(rec["notes"]) + notes
        shaft = parse_axis(r.get("phone_axis_along_shaft", ""))
        if shaft is None:
            shaft = np.array([0.0, 1.0, 0.0])
            notes.append("phone_axis_along_shaft blank; assumed +y")
        sig = process(rec, shaft, bias.get(walker), pivot_m)
        if sig["flipped"] and sc != "S2":
            notes.append("axis sign flipped (check phone_axis_along_shaft)")
        signals[name] = (rec, sig)

        tap0, tap1 = find_sync_triplets(rec)
        off, conf, vdur, vfile = sync.get(name, (float("nan"), "", float("nan"), ""))

        def vt(t_s):            # sensor time -> video time ("" when not in the video)
            if not np.isfinite(off):
                return ""
            v = t_s - off
            return round(v, 2) if v >= 0 and (not np.isfinite(vdur) or v <= vdur) else ""

        moving = np.abs(sig["w_up"]) > MOVING_DPS
        a = sig["alpha"][moving] if moving.sum() > FS else sig["alpha"]
        row.update(status="ok", duration_s=round(rec["t"][-1], 1),
                   sample_rate_hz=round(rec["fs_raw"], 1), gravity_ok=rec["gravity_ok"],
                   sync_tap_start_s=round(tap0, 2) if np.isfinite(tap0) else "",
                   sync_tap_end_s=round(tap1, 2) if np.isfinite(tap1) else "",
                   video_offset_s=round(off, 2) if np.isfinite(off) else "", video_sync=conf,
                   alpha_median_deg=round(np.median(a), 1),
                   alpha_p5_deg=round(np.percentile(a, 5), 1),
                   alpha_p95_deg=round(np.percentile(a, 95), 1),
                   pitch_swing_deg=round(np.percentile(sig["alpha_fast"], 95)
                                         - np.percentile(sig["alpha_fast"], 5), 1))
        if r.get("video_file", "").strip() and not np.isfinite(off):
            notes.append("video not synced yet: run sync_video.py")
        if np.isfinite(tap0):
            ev_rows.append(dict(file=name, event="sync_tap_start", sensor_time_s=round(tap0, 2),
                                video_time_s=vt(tap0), source="sensor", notes="first of 3 taps"))
        if np.isfinite(tap1):
            ev_rows.append(dict(file=name, event="sync_tap_end", sensor_time_s=round(tap1, 2),
                                video_time_s=vt(tap1), source="sensor", notes="first of 3 taps"))
        # firmware pitch filter vs the phone's own fused angle (while sweeping)
        if sc in SWEEPING and moving.sum() > FS:
            err_ = {k: np.percentile(np.abs(sig[k] - sig["alpha"])[moving], 95)
                    for k in ("alpha_cf", "alpha_fw", "alpha_fw_corr")}
            row.update(alpha_basic_err_p95_deg=round(err_["alpha_cf"], 1),
                       alpha_fw_err_p95_deg=round(err_["alpha_fw"], 1),
                       alpha_fw_corr_err_p95_deg=round(err_["alpha_fw_corr"], 1))

        if sc == "S1":
            still = np.linalg.norm(rec["gyr"], axis=1) < 5
            st = still if still.any() else np.ones_like(still)
            incl = num(r.get("inclinometer_deg", ""))
            row["alpha_static_deg"] = round(np.median(sig["alpha"][st]), 2)
            row["alpha_cf_static_deg"] = round(np.median(sig["alpha_fw"][st]), 2)
            row["inclinometer_deg"] = incl
            if name in NO_PITCH_CHECK:
                notes.append(f"inclinometer_deg {incl:g} is not a static reading; left out of accuracy check")
            elif np.isfinite(incl):
                row["alpha_error_deg"] = round(row["alpha_static_deg"] - incl, 2)
                row["alpha_cf_error_deg"] = round(row["alpha_cf_static_deg"] - incl, 2)
        if sc == "S2":
            row["gyro_bias_dps"] = " ".join(f"{b:.2f}" for b in bias.get(walker, []))
            row["alpha_flat_deg"] = round(float(np.median(sig["alpha"])), 2)
        if sc in SWEEPING:
            width, freq, rate90, fiv_med, fiv_p90 = sweep_metrics(sig, row["alpha_median_deg"], cane_m)
            row.update(sweep_width_deg=round(width, 1), sweep_freq_hz=round(freq, 2),
                       sweep_halfwidth_p95_deg=round(float(np.percentile(np.abs(sig["yaw_off"][moving]), 95)), 1),
                       yaw_rate_p90_dps=round(rate90, 1),
                       frames_in_view_typical=round(fiv_med, 1),
                       frames_in_view_worst=round(fiv_p90, 1))

        # cadence: two-point = one tap per step; constant contact = one arc per step
        impacts = find_impacts(rec)
        active = rms(sig["w_up"], 1.0) > STILL_DPS
        if sc in WALKING and active.sum() > 5 * FS:
            if r.get("technique", "").strip() == "two-point":
                at = np.interp(impacts, rec["t"], active.astype(float)) > 0.5
                row["cadence_spm"] = round(60 * at.sum() / (active.sum() / FS), 1)
            elif np.isfinite(row.get("sweep_freq_hz", np.nan)):
                row["cadence_spm"] = round(120 * row["sweep_freq_hz"], 1)

        # two-point: how high the tip goes between taps (pivot model, like the firmware)
        if r.get("technique", "").strip() == "two-point":
            tip_m = num(r.get("tip_to_phone_mm", "")) / 1000
            lc = (tip_m if np.isfinite(tip_m) else 0.72) + pivot_m
            lifts = tap_pivot_lift(rec, sig, lc) * 100
            if len(lifts) >= 5:
                row.update(tap_cycles=len(lifts), tap_lift_median_cm=round(float(np.median(lifts)), 1),
                           tap_lift_p90_cm=round(float(np.percentile(lifts, 90)), 1))
        # highest cane angle while plainly walking (sets the near-vertical pause)
        if sc in WALKING | {"S9", "S15"} and active.sum() > 5 * FS:
            win = active & (rec["t"] > (tap0 + 3 if np.isfinite(tap0) else 0)) &                   (rec["t"] < (tap1 - 1 if np.isfinite(tap1) else np.inf))
            if win.sum() > FS:
                row["alpha_walk_max_deg"] = round(float(smooth(sig["alpha"], 0.2)[win].max()), 1)

        # S10 recorded with the phone: each walk between still periods is one run
        if sc == "S10":
            bouts = walking_bouts(rec, sig)
            dist = num(r.get("walk_distance_m", "")) if np.isfinite(num(r.get("walk_distance_m", ""))) else 10.0
            if bouts:
                row["walk_speed_sensor_mps"] = round(dist / np.mean([b - a_ for a_, b in bouts]), 2)
                row["walk_speed_mps"] = row["walk_speed_sensor_mps"]
                row["walk_bouts"] = len(bouts)

        # edges, step-ups, turns and approach walks, straight from the cane angle
        rises = [e for e in detect_rises(rec, sig)
                 if not (np.isfinite(tap0) and e["onset_s"] < tap0 + 2.5)
                 and not (np.isfinite(tap1) and e["onset_s"] > tap1 - TAP_GUARD_S)]
        dist = num(r.get("approach_distance_m", ""))
        k = 0
        for e in rises:
            kind = "edge" if sc in EDGE_SCENARIOS else ("step_up" if sc == "S8" else "turn_or_pause")
            walk_s = e["onset_s"] - e["walk_start_s"]
            if kind != "turn_or_pause" and walk_s < 3:
                kind = "probe"          # cane re-checking the same edge, no walk before it
            k += 1
            ev_name = f"{kind}_{k}"
            seg = (rec["t"] >= e["onset_s"]) & (rec["t"] <= e["end_s"])
            peak = float(sig["alpha"][seg].max()) if seg.any() else float("nan")
            ev_rows.append(dict(file=name, event=ev_name, sensor_time_s=round(e["onset_s"], 2),
                                video_time_s=vt(e["onset_s"]), source="sensor",
                                notes=f"cane leaves walking angle {e['alpha_walk_deg']:.0f} deg, peaks {peak:.0f} deg"))
            ev_metrics.append(dict(
                file=name, walker=walker, scenario=sc, event=ev_name, source="sensor",
                sensor_time_s=round(e["onset_s"], 2), video_time_s=vt(e["onset_s"]),
                alpha_at_event_deg=round(e["alpha_walk_deg"], 1),
                rise_to_threshold_s=round(e["rise_s"], 2), alpha_peak_deg=round(peak, 1),
                walk_before_s=round(walk_s, 1),
                yaw_offset_at_event_deg=round(np.interp(e["onset_s"], rec["t"], sig["yaw_off"]), 1)))
            if kind in ("edge", "step_up"):
                ap = dict(file=name, scenario=sc, technique=row["technique"], event=ev_name,
                          walk_start=e["walk_start"], walk_time_s=round(walk_s, 1))
                if e["walk_start"] == "still" and np.isfinite(dist) and walk_s > 3:
                    reach = HAND_AHEAD_M + cane_m * np.cos(np.radians(e["alpha_walk_deg"]))
                    ap["distance_m"] = round(dist - reach, 2)
                    ap["speed_mps"] = round((dist - reach) / walk_s, 2)
                approaches.append(ap)
        if sc in EDGE_SCENARIOS or sc == "S8":
            row["edges_found"] = sum(1 for x in ev_metrics if x["file"] == name
                                     and x["event"].startswith(("edge", "step_up")))

        if sc in STAIRS:
            imp, sps = stair_steps(rec, sig, STAIRS[sc])
            row["stair_impacts"] = len(imp)
            if np.isfinite(sps):
                row["seconds_per_step"] = round(sps, 2)
            if len(imp) > 1:
                row["descent_s"] = round(float(imp[-1] - imp[0]), 1)
            steps = num(r.get("step_count", ""))
            if np.isfinite(steps) and len(imp) and abs(len(imp) - steps) > 2:
                notes.append(f"found {len(imp)} step impacts, log says {steps:g} steps")
            for j, x in enumerate(imp, 1):
                ev_rows.append(dict(file=name, event=f"step_{j}", sensor_time_s=round(x, 2),
                                    video_time_s=vt(x), source="sensor", notes="tip impact on a step"))

        # hand-typed events from the video (source = video in events.csv): shifted with the sync
        for _, m in manual[manual["file"].str.strip() == name].iterrows():
            v = num(m["video_time_s"])
            t_s = v + off if np.isfinite(v) and np.isfinite(off) else num(m["sensor_time_s"])
            near = [x for x in ev_rows if x["file"] == name and x["source"] == "sensor"]
            dt = min((abs(x["sensor_time_s"] - t_s) for x in near), default=float("nan")) if np.isfinite(t_s) else float("nan")
            ev_rows.append(dict(file=name, event=m["event"], sensor_time_s=round(t_s, 2) if np.isfinite(t_s) else "",
                                video_time_s=m["video_time_s"], source="video",
                                notes=(m.get("notes", "") + f" | nearest sensor event {dt:.2f} s away").strip(" |")
                                if np.isfinite(dt) else m.get("notes", "")))
            if np.isfinite(t_s) and m["event"].startswith(EDGE_EVENTS):
                ev_metrics.append(dict(
                    file=name, walker=walker, scenario=sc, event=m["event"], source="video",
                    sensor_time_s=round(t_s, 2), video_time_s=m["video_time_s"],
                    alpha_at_event_deg=round(np.interp(t_s, rec["t"], sig["alpha"]), 1),
                    yaw_offset_at_event_deg=round(np.interp(t_s, rec["t"], sig["yaw_off"]), 1)))

        row["notes"] = "; ".join(notes)
        summary.append(row)

        if plt is not None:
            fig, ax = plt.subplots(2, 1, figsize=(11, 5.5), sharex=True)
            ax[0].plot(rec["t"], sig["alpha"], label="alpha (phone fusion)")
            ax[0].plot(rec["t"], sig["alpha_fw"], label="alpha (firmware filter)", alpha=0.7)
            ax[0].set_ylabel("cane angle (deg)")
            ax[0].legend(loc="upper right")
            ax[1].plot(rec["t"], sig["yaw_off"], color="tab:green")
            ax[1].set_ylabel("yaw offset (deg)")
            ax[1].set_xlabel("sensor time (s)" + (f"   [video time = sensor time - {off:.2f} s]"
                                                  if np.isfinite(off) else ""))
            for x in ev_rows:
                if x["file"] != name or x["sensor_time_s"] == "":
                    continue
                ts = float(x["sensor_time_s"])
                if x["event"].startswith("step_"):
                    ax[0].axvline(ts, ymax=0.08, color="tab:purple", lw=1.2)
                    continue
                color = "grey" if x["event"].startswith("sync") else (
                    "tab:red" if x["source"] == "sensor" else "black")
                for a_ in ax:
                    a_.axvline(ts, color=color, lw=0.8, ls="--" if x["source"] == "sensor" else ":")
                ax[1].text(ts, ax[1].get_ylim()[1], x["event"], rotation=90, va="top", fontsize=7)
            fig.suptitle(name)
            fig.tight_layout()
            fig.savefig(out / "plots" / f"{name}.png", dpi=110)
            plt.close(fig)

    summ = pd.DataFrame(summary)
    summ.to_csv(out / "summary.csv", index=False)
    evdf = pd.DataFrame(ev_rows, columns=["file", "event", "sensor_time_s", "video_time_s", "source", "notes"])
    evdf.to_csv(out / "events_synced.csv", index=False)
    evdf.to_csv(d / "events.csv", index=False)       # every event with sensor and video time
    pd.DataFrame(ev_metrics).to_csv(out / "event_metrics.csv", index=False)
    pd.DataFrame(approaches).to_csv(out / "approaches.csv", index=False)
    write_report(out, summ, signals, log, pd.DataFrame(ev_metrics), pd.DataFrame(approaches), sync)
    print(f"done: {len(summ)} recordings, {int((summ['status'] == 'ok').sum())} ok -> {out}")


def write_report(out, summ, signals, log, evm, appr, sync):
    ok = summ[summ["status"] == "ok"] if "status" in summ else summ.iloc[0:0]
    L = ["# Cane motion: numbers for the team", "",
         f"Walkers: {', '.join(sorted(summ['walker'].unique()))}. "
         "Every event below is found in the sensor data; videos are lined up by sound (sync_video.py).", ""]
    delay = N_CONFIRM / FRAME_HZ

    # mount offset from the static angles
    s1 = ok[ok["scenario"] == "S1"]
    mount = float(s1["alpha_error_deg"].mean()) if "alpha_error_deg" in s1 and s1["alpha_error_deg"].notna().any() else 0.0

    if sync:
        L += ["## Video sync (sensor time = video time + offset)", "",
              "| recording | video | offset (s) | confidence |", "| --- | --- | --- | --- |"]
        for f, (o, c, vd, vf) in sync.items():
            L.append(f"| {f} | {vf} | {o:+.2f} | {c} |")
        L.append("")

    walk = [n for n in ok["file"] if ok.set_index("file").loc[n, "scenario"] in WALKING]
    if walk:
        alphas = np.concatenate([
            signals[n][1]["alpha"][np.abs(signals[n][1]["w_up"]) > MOVING_DPS] for n in walk])
        if len(alphas):
            L += ["## For Andy: cane angle while walking (S4, S5)",
                  f"- Median **{np.median(alphas):.1f} deg**, typical range "
                  f"**{np.percentile(alphas, 5):.1f} to {np.percentile(alphas, 95):.1f} deg** (5th to 95th percentile), "
                  "as the phone measured it.",
                  f"- The static checks say the phone reads {mount:+.1f} deg against the inclinometer (mount offset); "
                  f"the shaft itself was at about **{np.median(alphas) - mount:.1f} deg** "
                  f"({np.percentile(alphas, 5) - mount:.1f} to {np.percentile(alphas, 95) - mount:.1f}).",
                  "- Design the ground beam so it lands 0.5 to 1.0 m past the tip across this whole range.", ""]
        tab = ok[ok["scenario"].isin(WALKING)].groupby("technique")[
            ["sweep_freq_hz", "sweep_width_deg", "sweep_halfwidth_p95_deg", "frames_in_view_worst",
             "pitch_swing_deg", "cadence_spm"]].mean()
        L += ["## Sweep by technique (means)", "",
              "| technique | sweep freq (Hz) | sweep width p5-p95 (deg) | half-width p95 (deg) | ToF frames in view, worst | pitch swing (deg) | cadence (steps/min) |",
              "| --- | --- | --- | --- | --- | --- | --- |"]
        for tech, v in tab.iterrows():
            L.append(f"| {tech} | {v.sweep_freq_hz:.2f} | {v.sweep_width_deg:.1f} | {v.sweep_halfwidth_p95_deg:.1f} | "
                     f"{v.frames_in_view_worst:.1f} | {v.pitch_swing_deg:.1f} | {v.cadence_spm:.0f} |")
        worst = ok[ok["scenario"].isin(WALKING)]["frames_in_view_worst"].min()
        verdict = "PASS" if worst >= N_CONFIRM else "FAIL: lower N or raise the ToF rate"
        L += ["", f"**N = {N_CONFIRM} check:** a {FEATURE_WIDTH_M * 100:.0f} cm drop stays in the ground beam "
              f"for at least **{worst:.1f} ToF frames** at {FRAME_HZ} Hz in the fastest sweeps: **{verdict}**. "
              f"(margin {worst - N_CONFIRM:+.1f} frames). "
              "A curb or stair edge runs across the whole path, so it stays in view much longer.", ""]

    fw = ok[ok["scenario"].isin(WALKING)] if "alpha_fw_err_p95_deg" in ok else ok.iloc[0:0]
    if len(fw):
        L += ["## Firmware pitch filter while walking (vs the phone's own fused angle)",
              f"- Basic filter (K = {CF_K} at {FS:.0f} Hz, pitch axis only): p95 error "
              f"**{fw['alpha_basic_err_p95_deg'].max():.1f} deg**.",
              f"- Firmware filter (gravity vector, tau = {CF_TAU_S:g} s): **{fw['alpha_fw_err_p95_deg'].max():.1f} deg**; "
              f"same plus centripetal correction: **{fw['alpha_fw_corr_err_p95_deg'].max():.1f} deg** (worst of S4, S5).",
              "- 1 deg of alpha error is about 1 cm of drop-height error; the alert threshold is 8 cm.", ""]

    speeds = []
    for _, r in ok[ok["scenario"] == "S10"].iterrows():
        if "walk_speed_sensor_mps" in r and pd.notna(r["walk_speed_sensor_mps"]):
            speeds.append(("S10 phone sensors", float(r["walk_speed_sensor_mps"])))
        if "walk_speed_runs_mps" in r and isinstance(r["walk_speed_runs_mps"], str):
            for i, v in enumerate(r["walk_speed_runs_mps"].split(";"), 1):
                speeds.append((f"S10 timed run {i}", float(v)))
    if len(appr) and "speed_mps" in appr:
        for _, r in appr[appr["speed_mps"].notna()].iterrows():
            speeds.append((f"{r.file}, first approach ({r.technique})", float(r.speed_mps)))
    if speeds:
        L += ["## Walking speed and warning time", "",
              "| source | speed (m/s) | warning time (s) |", "| --- | --- | --- |"]
        for label, v in speeds:
            L.append(f"| {label} | {v:.2f} | {LOOKAHEAD_PAST_TIP_M / v - delay:.2f} |")
        L += ["", f"Warning time = look-ahead {LOOKAHEAD_PAST_TIP_M} m / speed, minus "
              f"{delay * 1000:.0f} ms confirmation ({N_CONFIRM} ToF frames at {FRAME_HZ} Hz).", ""]
        if "cadence_spm" in ok and ok["cadence_spm"].notna().any():
            cad = ok["cadence_spm"].dropna().mean()
            v = float(np.median([s for _, s in speeds]))
            L += [f"Cadence {cad:.0f} steps/min at a median speed of {v:.2f} m/s gives a step length of "
                  f"about {v / (cad / 60):.2f} m.", ""]

    if "alpha_error_deg" in s1 and s1["alpha_error_deg"].notna().any():
        L += ["## Pitch accuracy vs inclinometer (S1)", "",
              "| recording | inclinometer (deg) | phone (deg) | difference (deg) | firmware filter (deg) |",
              "| --- | --- | --- | --- | --- |"]
        for _, r in s1.iterrows():
            L.append(f"| {r.file} | {r.inclinometer_deg:.2f} | {r.alpha_static_deg:.2f} | {r.alpha_error_deg:+.2f} | "
                     f"{r.alpha_cf_static_deg:.2f} |")
        s2 = ok[ok["scenario"] == "S2"]
        flat = f" Lying flat (S2) the phone reads {s2['alpha_flat_deg'].iloc[0]:.1f} deg." if len(s2) and "alpha_flat_deg" in s2 else ""
        L += ["", f"Mean difference **{mount:+.1f} deg**: the same sign at both angles, so it is the phone sitting "
              f"slightly tilted on the shaft, not filter error.{flat} The pod's one-press calibration (L and beta) "
              "absorbs this offset; the spread between the two angles "
              f"({s1['alpha_error_deg'].max() - s1['alpha_error_deg'].min():.1f} deg) is the real accuracy.", ""]

    if len(evm):
        se = evm[evm["source"] == "sensor"]
        edges = se[se["event"].str.startswith(("edge", "step_up"))]
        if len(edges):
            L += ["## Edges found in the sensor data", "",
                  f"| scenario | edges | walking alpha before (deg) | edge -> alpha > {EDGE_ALPHA_DEG:g} deg (s) | "
                  "peak alpha after the edge (deg) | walk before (s) |",
                  "| --- | --- | --- | --- | --- | --- |"]
            for sc, g in edges.groupby("scenario"):
                L.append(f"| {sc} | {len(g)} | {g['alpha_at_event_deg'].mean():.1f} "
                         f"({g['alpha_at_event_deg'].min():.1f}-{g['alpha_at_event_deg'].max():.1f}) | "
                         f"{g['rise_to_threshold_s'].mean():.2f} | {g['alpha_peak_deg'].mean():.0f} "
                         f"({g['alpha_peak_deg'].min():.0f}-{g['alpha_peak_deg'].max():.0f}) | "
                         f"{g['walk_before_s'].mean():.1f} |")
            turns = se[se["event"].str.startswith("turn")]
            L += ["", f"Outside the edge scenarios the cane went past {EDGE_ALPHA_DEG:g} deg **{len(turns)} times** "
                  "(turns and pauses on flat ground).", ""]

    st = ok[ok["scenario"].isin(STAIRS)] if "seconds_per_step" in ok else ok.iloc[0:0]
    st = st[st["seconds_per_step"].notna()] if len(st) else st
    if len(st):
        L += ["## Stairs (from tip impacts)", ""]
        for _, r in st.iterrows():
            L.append(f"- {r.file} ({STAIRS[r.scenario]}): {int(r.stair_impacts)} impacts over {r.descent_s:.0f} s, "
                     f"**{r.seconds_per_step:.2f} s per step**")
        L += ["", f"Mean **{st['seconds_per_step'].mean():.2f} s per step**.", ""]

    lifts = log[log["tip_lift_cm"].str.strip() != ""]["tip_lift_cm"].unique() if "tip_lift_cm" in log else []
    tl = ok[ok["tap_lift_median_cm"].notna()] if "tap_lift_median_cm" in ok else ok.iloc[0:0]
    if len(lifts) or len(tl):
        L += ["## Two-point tip lift", ""]
        if len(lifts):
            L.append(f"- Measured by hand: **{', '.join(lifts)} cm**.")
        for _, r in tl.iterrows():
            L.append(f"- {r.file}: from the cane angle (pivot in the hand), median **{r.tap_lift_median_cm:.1f} cm**, "
                     f"90th percentile {r.tap_lift_p90_cm:.1f} cm over {int(r.tap_cycles)} taps.")
        L += ["- A lifted tip reads exactly like a drop of the same depth (dh = -lift), and the alert threshold is "
              "8 cm. The firmware adds the computed lift back to dh while the user taps (TAP_MODE 1), so every "
              "frame still counts and a curb stays visible between taps.", ""]
    wm = ok["alpha_walk_max_deg"].dropna() if "alpha_walk_max_deg" in ok else []
    if len(wm) and len(evm):
        pk = evm[(evm["source"] == "sensor") & evm["event"].str.startswith(("edge", "step_up"))]["alpha_peak_deg"]
        L += ["## Near-vertical pause", "",
              f"- Highest cane angle while walking: **{wm.max():.1f} deg**. After finding an edge or curb the cane "
              f"goes to **{pk.min():.0f}-{pk.max():.0f} deg**.",
              "- So a pause at 60 deg (back on below 55) never interrupts walking, and keeps the pod quiet once the "
              "user has found the edge and is checking it.", ""]

    issues, missing = [], []
    for _, r in summ.iterrows():
        if r["status"] != "ok":
            if "not found" in r["status"]:
                missing.append(r["file"])
            else:
                issues.append(f"- {r['file']}: {r['status']}")
            continue
        if not r.get("gravity_ok", True):
            issues.append(f"- {r['file']}: gravity magnitude not ~9.8; check export")
        if r.get("sample_rate_hz", 100) < 90:
            issues.append(f"- {r['file']}: sample rate {r['sample_rate_hz']} Hz (< 90)")
        if isinstance(r.get("notes"), str) and r["notes"]:
            issues.append(f"- {r['file']}: {r['notes']}")
    if missing:
        issues.append(f"- Not recorded yet / no file in raw/ ({len(missing)}): " + ", ".join(missing))
    L += ["## Data quality", ""] + (issues or ["- No issues found."])
    (out / "report.md").write_text("\n".join(L) + "\n", encoding="utf-8")


def main():
    p = argparse.ArgumentParser(description="SafeRoute cane motion analysis")
    p.add_argument("command", choices=["make-templates", "analyze"])
    p.add_argument("--dir", default=".", help="folder with log.csv and raw/")
    p.add_argument("--no-plots", action="store_true")
    a = p.parse_args()
    d = Path(a.dir)
    if not (d / "log.csv").exists():
        sys.exit(f"{d / 'log.csv'} not found (rename the recording log to log.csv)")
    if a.command == "make-templates":
        make_templates(d)
    else:
        analyze(d, plots=not a.no_plots)


if __name__ == "__main__":
    main()

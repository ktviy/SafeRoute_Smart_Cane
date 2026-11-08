#!/usr/bin/env python3
"""
Line up each video with its sensor recording, using sound.

Every cane impact (the 3 sync taps, two-point taps, steps, the curb) is a spike
in the phone's accelerometer AND a click in the video's audio. This script finds
the time shift that makes the most accelerometer impacts land on an audio click,
then writes it to video_sync.csv:

    sensor_time = video_time + offset_s

analyze_cane.py reads video_sync.csv and gives every event both times, so you can
jump to any event in the video.

Usage:
    pip install imageio-ffmpeg          (once; or have ffmpeg on PATH)
    python sync_video.py                (reads log.csv: file + video_file, videos in video/)

Confidence "high" = most impacts match AND (clearly better than any other shift, or
all 3 sync taps of the start or end triplet land on audio clicks).
For "low" rows, check the first sync tap in the video by eye and type the shift
into video_sync.csv (offset_s = sensor time of tap 1 - video time of tap 1).
"""
import argparse
import shutil
import subprocess
import sys
import tempfile
import wave
from pathlib import Path

import numpy as np
import pandas as pd

import analyze_cane as ac

AUDIO_FS = 8000
ENV_HZ = 200            # audio envelope rate
CLICK_DB = 5.0          # audio click = this many dB above the 1 s local level
MATCH_S = 0.05          # impact and click must be this close
SEARCH = (-10.0, 15.0)  # offsets tried (s)


def ffmpeg_exe():
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except ImportError:
        exe = shutil.which("ffmpeg")
        if not exe:
            sys.exit("need ffmpeg: pip install imageio-ffmpeg")
        return exe


def audio_clicks(video, tmp):
    wav = Path(tmp) / (video.stem + ".wav")
    subprocess.run([ffmpeg_exe(), "-hide_banner", "-loglevel", "error", "-y", "-i", str(video),
                    "-vn", "-ac", "1", "-ar", str(AUDIO_FS), str(wav)], check=True)
    with wave.open(str(wav)) as w:
        x = np.frombuffer(w.readframes(w.getnframes()), np.int16).astype(float)
    x = np.diff(x)                                   # emphasise sharp clicks
    hop = AUDIO_FS // ENV_HZ
    n = len(x) // hop
    level = 20 * np.log10(np.sqrt((x[:n * hop].reshape(n, hop) ** 2).mean(1)) + 1e-3)
    onset = level - np.convolve(level, np.ones(ENV_HZ) / ENV_HZ, "same")
    t = np.arange(n) / ENV_HZ
    onset[(t < 0.6) | (t > t[-1] - 0.6)] = 0         # record-button sounds
    return peaks(t, onset, CLICK_DB, 0.1), t[-1]


def peaks(t, v, thr, gap):
    out = []
    for i in np.where(v > thr)[0]:
        if out and t[i] - t[out[-1]] < gap:
            if v[i] > v[out[-1]]:
                out[-1] = i
        else:
            out.append(i)
    return t[out]


def sensor_impacts(rec, thr=8.0):
    mag = np.linalg.norm(rec["acc"], axis=1)
    hp = mag - ac.smooth(mag, 0.5)
    return peaks(rec["t"], hp, thr, 0.15)


def score(imp, clicks, off, dur):
    v = imp - off
    v = v[(v > 0.6) & (v < dur - 0.6)]
    if len(v) < 5 or not len(clicks):
        return 0.0, len(v), 0
    d = np.abs(clicks[None, :] - v[:, None]).min(1)
    hit = int((d < MATCH_S).sum())
    return hit / len(v), len(v), hit


def find_offset(imp, clicks, dur):
    offs = np.arange(SEARCH[0], SEARCH[1], 0.01)
    sc = np.array([score(imp, clicks, o, dur)[0] for o in offs])
    i = int(np.argmax(sc))
    best = offs[i]
    runner = sc[np.abs(offs - best) > 0.3].max() if (np.abs(offs - best) > 0.3).any() else 0.0
    # refine: median residual of the matched impacts
    v = imp - best
    v = v[(v > 0.6) & (v < dur - 0.6)]
    d = clicks[None, :] - v[:, None]
    j = np.abs(d).argmin(1)
    r = d[np.arange(len(v)), j]
    r = r[np.abs(r) < MATCH_S]
    best = best - (np.median(r) if len(r) else 0.0)
    frac, n, hit = score(imp, clicks, best, dur)
    return best, frac, runner, n, hit


def main():
    p = argparse.ArgumentParser(description="sync videos to sensor recordings by sound")
    p.add_argument("--dir", default=".", help="folder with log.csv, raw/ and video/")
    a = p.parse_args()
    d = Path(a.dir)
    log = ac.read_log(d / "log.csv")
    rows = []
    with tempfile.TemporaryDirectory() as tmp:
        for _, r in log.iterrows():
            name, vf = r["file"].strip(), r.get("video_file", "").strip()
            if not vf:
                continue
            video = d / "video" / vf
            rec, err = ac.load_recording(d / "raw", name)
            if rec is None or not video.exists():
                print(f"{name}: skipped ({err or 'video not found: ' + str(video)})")
                continue
            clicks, dur = audio_clicks(video, tmp)
            imp = sensor_impacts(rec)
            off, frac, runner, n, hit = find_offset(imp, clicks, dur)
            tri = ac.find_sync_triplets(rec)
            # do all 3 taps of a sync triplet land on audio clicks at this shift?
            taps_ok = []
            for t0 in tri:
                if t0 == t0:
                    taps = imp[(imp >= t0 - 0.01) & (imp < t0 + 2.6)][:3] - off
                    if len(taps) == 3 and taps.min() > 0.6 and taps.max() < dur - 0.6:
                        taps_ok.append(bool((np.abs(clicks[None, :] - taps[:, None]).min(1) < MATCH_S).all()))
            conf = "high" if frac >= 0.4 and (frac - runner >= 0.2 or any(taps_ok)) else "low"
            row = dict(file=name, video_file=vf, offset_s=round(off, 3), impacts_matched=hit, impacts_in_video=n,
                       sync_taps_on_clicks="yes" if any(taps_ok) else "no",
                       match_fraction=round(frac, 2), best_other_shift=round(runner, 2), confidence=conf,
                       video_duration_s=round(dur, 2), sensor_duration_s=round(rec["t"][-1], 2),
                       sensor_tap_start_s=round(tri[0], 2) if tri[0] == tri[0] else "",
                       video_tap_start_s=round(tri[0] - off, 2) if tri[0] == tri[0] else "",
                       sensor_tap_end_s=round(tri[1], 2) if tri[1] == tri[1] else "",
                       video_tap_end_s=round(tri[1] - off, 2) if tri[1] == tri[1] and tri[1] - off < dur else "")
            rows.append(row)
            print(f"{name:28s} offset {off:+.2f} s  {hit}/{n} impacts on a click (other shifts <= {runner:.0%})  {conf}")
    out = pd.DataFrame(rows)
    out.to_csv(d / "video_sync.csv", index=False)
    print(f"wrote {d / 'video_sync.csv'}")


if __name__ == "__main__":
    main()

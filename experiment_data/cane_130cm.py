#!/usr/bin/env python3
"""
SafeRoute - re-run the beam geometry for a 130 cm cane (P1 recorded on a 120 cm cane)
=====================================================================================
P1's 17 recordings (2026-10-30) were made on a 120 cm cane: tip -> phone L = 0.72 m,
grip -> phone 0.33 m, so the hand pivots 1.05 m from the tip. The pod now goes on a
130 cm cane. Same walker, same arm: the hand stays at the same height above the floor,
the pod stays 0.72 m from the tip (same ToF ranges), so the hand is now 1.15 m from the
tip (grip -> pod 0.43 m) and every tip-on-ground cane angle obeys

    (L + grip)_130 * sin(alpha_130) = (L + grip)_120 * sin(alpha_120)   (same hand height)

The transform is monotonic, so every percentile, minimum and maximum in the DATA
REPORT maps straight across. Frames-in-view scale with 1 / (hand -> beam spot radius).

  python cane_130cm.py            # prints the tables for the DATA REPORT (markdown)
  python cane_130cm.py --check    # also checks the sensitivity to where the hand sits
"""
import argparse
import math

D = math.radians
R = math.degrees

L = 0.72                  # tip -> pod (m), unchanged
PIVOT_120 = 0.72 + 0.33   # tip -> hand on P1's 120 cm cane (log.csv: tip_to_phone 720, grip_to_phone 330)
PIVOT_130 = 0.72 + 0.43   # tip -> hand on the 130 cm cane (hand 15 cm below the top, as on the 120 cm cane)
LOOKAHEAD = 0.75          # design target: ground spot 0.75 m past the tip
OVERHANG = 1.50           # head-level test object height (m)
HEAD = 1.80               # walker head height (m)
ALPHA_ERR_P95 = 3.4       # firmware filter, two-point tapping (deg), DATA REPORT section 1
SPEEDS = (0.78, 0.81, 0.84, 1.0)
CONFIRM_S = 0.1           # 3 frames at 30 Hz

# DATA REPORT section 1 (shaft angles, 120 cm cane)
WALK_MED, WALK_LO, WALK_HI = 41.1, 36.5, 45.1
WALK_MAX = 51.6
EDGE_FOUND = (35 - 1.9, 46 - 1.9)          # phone angles minus the 1.9 deg mount tilt
AFTER = {"curb": (60, 72), "stair top": (62, 67), "stair descent (p5-p95)": (65, 83), "stair descent peaks": (83, 86)}
FRAMES_WORST = (3.6, 4.3)                  # frames a 30 cm drop stays in view (worst approach / walk)


def to130(a120, p120=PIVOT_120, p130=PIVOT_130):
    """Cane angle on the 130 cm cane for the same hand height (deg)."""
    return R(math.asin(min(1.0, p120 / p130 * math.sin(D(a120)))))


def ground(alpha, beta):
    """Pod height, ToF range on flat floor, landing distance past the tip."""
    a, dep = D(alpha), D(alpha - beta)
    h = L * math.sin(a)
    d = h / math.sin(dep)
    past = d * math.cos(dep) - L * math.cos(a)
    return h, d, past


def beta_for(alpha, past=LOOKAHEAD):
    """Ground-beam mount angle that lands `past` m beyond the tip at `alpha`."""
    a = D(alpha)
    h, x = L * math.sin(a), L * math.cos(a) + past
    return alpha - R(math.atan2(h, x))


def forward(alpha, gamma, height):
    """Horizontal distance ahead of the pod where the forward beam reaches `height`."""
    a, e = D(alpha), D(gamma - alpha)
    return (height - L * math.sin(a)) / math.tan(e)


def sens(alpha, beta):
    """dh sensitivity: cm per deg of alpha error, cm per cm of ToF error."""
    _, d, _ = ground(alpha, beta)
    a, dep = D(alpha), D(alpha - beta)
    per_deg = abs(L * math.cos(a) - d * math.cos(dep)) * D(1) * 100
    return per_deg, math.sin(dep)


def max_lookahead(alpha, beta_budget_cm=7.0, err_deg=ALPHA_ERR_P95):
    """Largest look-ahead whose dh error from err_deg alpha error stays within the budget."""
    best = 0.0
    for i in range(1, 400):
        past = i / 100
        b = beta_for(alpha, past)
        if sens(alpha, b)[0] * err_deg <= beta_budget_cm:
            best = past
    return best


def frames_scale(alpha120, alpha130):
    """Frames-in-view scale: spot radius from the hand, 120 cm vs 130 cm cane."""
    r120 = PIVOT_120 * math.cos(D(alpha120)) + LOOKAHEAD
    r130 = PIVOT_130 * math.cos(D(alpha130)) + LOOKAHEAD
    return r120 / r130


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--beta", type=float, default=19.0)
    ap.add_argument("--gamma", type=float, default=73.0)
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    B, G = a.beta, a.gamma

    med, lo, hi = to130(WALK_MED), to130(WALK_LO), to130(WALK_HI)
    wmax = to130(WALK_MAX)
    print(f"# 130 cm cane, L = {L} m, tip->hand {PIVOT_130:.2f} m (P1 120 cm cane: {PIVOT_120:.2f} m)\n")
    print("## Cane angle (shaft), 120 cm -> 130 cm")
    print("| Item | 120 cm (measured) | 130 cm (same hand height) |\n|---|---|---|")
    print(f"| Walking median | {WALK_MED} | {med:.1f} |")
    print(f"| Walking range | {WALK_LO}-{WALK_HI} | {lo:.1f}-{hi:.1f} |")
    print(f"| Highest while walking | {WALK_MAX} | {wmax:.1f} |")
    print(f"| Edge found | {EDGE_FOUND[0]:.1f}-{EDGE_FOUND[1]:.1f} | {to130(EDGE_FOUND[0]):.1f}-{to130(EDGE_FOUND[1]):.1f} |")
    for k, (x, y) in AFTER.items():
        print(f"| After the edge: {k} | {x}-{y} | {to130(x):.1f}-{to130(y):.1f} |")

    print("\n## Mount angles that keep the v2 design (0.75 m past the tip, forward 36 deg above horizontal)")
    print(f"beta for 0.75 m at {med:.1f} deg: {beta_for(med):.2f} deg   gamma for 36 deg up: {med + 36:.1f} deg")
    print(f"(v2 check: beta for 0.75 m at 41.1 deg on L 0.72: {beta_for(41.1):.2f} deg)")

    print(f"\n## Ground beam, beta {B:g} deg")
    print("| alpha | pod h (m) | flat ToF (m) | past tip (m) | 14 cm step ToF | 15 cm curb ToF | 30 cm drop ToF | cm/deg | cm/cm |")
    print("|---|---|---|---|---|---|---|---|---|")
    for al in (lo, med, hi):
        h, d, past = ground(al, B)
        s = math.sin(D(al - B))
        pd, pc = sens(al, B)
        print(f"| {al:.1f} | {h:.3f} | {d:.2f} | {past:.2f} | {(h + .14) / s:.2f} | {(h + .15) / s:.2f} | "
              f"{(h + .30) / s:.2f} | {pd:.2f} | {pc:.2f} |")
    print(f"Max look-ahead for a 7 cm budget at {ALPHA_ERR_P95} deg alpha error: ~{max_lookahead(med):.2f} m")
    _, _, past = ground(med, B)
    for v in SPEEDS:
        print(f"warning at {v} m/s: {past / v - CONFIRM_S:.2f} s", end="   ")
    print()
    for bb in (18, 19, 20, 21):
        print(f"  beta {bb}: past tip {ground(lo, bb)[2]:.2f}-{ground(hi, bb)[2]:.2f} m (design {ground(med, bb)[2]:.2f})")

    print(f"\n## Forward beam, gamma {G:g} deg")
    print(f"elevation {G - hi:.1f}-{G - lo:.1f} deg (design {G - med:.1f})")
    print(f"1.5 m overhang met {forward(lo, G, OVERHANG):.2f}-{forward(hi, G, OVERHANG):.2f} m ahead of the pod "
          f"(design {forward(med, G, OVERHANG):.2f})")
    print(f"head {HEAD} m reached {forward(lo, G, HEAD):.2f}-{forward(hi, G, HEAD):.2f} m ahead (design "
          f"{forward(med, G, HEAD):.2f})")
    for gg in (71, 73, 75, 77):
        print(f"  gamma {gg}: 1.5 m overhang {forward(lo, gg, OVERHANG):.2f}-{forward(hi, gg, OVERHANG):.2f} m")

    print("\n## Pause thresholds (v2 rule: off = walking max + alpha error p95; on = lowest after-edge angle)")
    off = wmax + ALPHA_ERR_P95
    on = min(to130(v[0]) for v in AFTER.values())
    print(f"walking max {wmax:.1f} + {ALPHA_ERR_P95} = {off:.1f} -> PAUSE_OFF; lowest after-edge {on:.1f} -> PAUSE_ON")
    print(f"frames a 30 cm drop stays in view: x{frames_scale(WALK_MED, med):.3f} -> "
          f"{FRAMES_WORST[0] * frames_scale(WALK_MED, med):.1f}-{FRAMES_WORST[1] * frames_scale(WALK_MED, med):.1f}")
    print(f"calibration rocking range (v2 40-52 deg on 120 cm): {to130(40):.0f}-{to130(52):.0f} deg")

    if a.check:
        print("\n## Sensitivity: where the hand really sits")
        for p120, label in ((1.05, "hand 15 cm below top"), (1.12, "hand 8 cm below top"), (1.20, "hand at the top")):
            m = to130(WALK_MED, p120, p120 + 0.10)
            print(f"{label}: alpha {m:.1f} deg -> beta {beta_for(m):.1f}, gamma {m + 36:.1f}")


if __name__ == "__main__":
    main()

"""Print the lab tables as Markdown and write the golden CSV files.

Run from lab/:  python -m fmelab.report  [--no-golden] [--mc N]
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import statistics

from . import ball as B
from . import body as Y
from . import execution as X

SPEEDS, ANGLES = (10, 15, 20, 25, 30), (10, 20, 30, 42)
NONE, BACK4 = (0.0, 0.0, 0.0), (0.0, -4 * B.REV, 0.0)
TOP3, TOP39 = (0.0, 3 * B.REV, 0.0), (0.0, 3.9 * B.REV, 0.0)
SPINS = {"none": NONE, "back 5": (0.0, -5 * B.REV, 0.0), "side 5": (0.0, 0.0, 5 * B.REV)}
GOLDEN = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "golden")


# --- output helpers ---
def table(headers, rows) -> None:
    lines = ["| " + " | ".join(headers) + " |", "|" + "---|" * len(headers)]
    print("\n".join(lines + ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]) + "\n")


def f(x, n: int = 2) -> str:
    return "n/a" if x is None or not math.isfinite(x) else f"{x:.{n}f}"


def write_csv(out, name, header, rows) -> None:
    if out:
        with open(os.path.join(out, name), "w", newline="") as fh:
            w = csv.writer(fh, lineterminator="\n")
            w.writerow(header)
            w.writerows([f"{v:.6f}" if isinstance(v, float) else v for v in r] for r in rows)


def bisect(fn, lo, hi, target, n=40):
    """Root of fn(x) = target for a monotonic fn on [lo, hi]."""
    up = fn(hi) > fn(lo)
    for _ in range(n):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if (fn(mid) < target) == up else (lo, mid)
    return 0.5 * (lo + hi)


# --- ball helpers ---
def kick(v, a, spin=NONE, z0=None, p=B.P) -> B.Ball:
    b = B.launch(v, a, spin, p)
    b.p = b.p if z0 is None else (0.0, 0.0, z0)
    return b


def land(v, a, spin=NONE, z0=None, p=B.P) -> B.Landing:
    return B.flight(kick(v, a, spin, z0, p), p)


def cross(v, a, spin, x_line, p=B.P):
    """(y, z) where the ball crosses x = x_line in flight, else None."""
    b = kick(v, a, spin, p=p)
    while not b.events:
        p0 = b.p
        B.advance(b, B.DT, p)
        if b.p[0] >= x_line and not b.events:
            k = (x_line - p0[0]) / (b.p[0] - p0[0])
            return p0[1] + k * (b.p[1] - p0[1]), p0[2] + k * (b.p[2] - p0[2])
    return None


def curl(v, spin, x_line, z_line=1.84):
    """Launch angle and bend of a kick that crosses x = x_line at height z_line."""
    a = bisect(lambda ang: (cross(v, ang, spin, x_line) or (0.0, -1.0))[1], 1.0, 35.0, z_line)
    return a, cross(v, a, spin, x_line)[0]


def mean_coefs(v, a, spin, p=B.P):
    """Time means of C_D and C_L over the flight (sampled at the sub-step starts)."""
    b, n, cd, cl = kick(v, a, spin, p=p), 0, 0.0, 0.0
    while not b.events:
        s, sp = B.norm(b.v), B.spin_ratio(b.v, b.w, p)
        cd, cl, n = cd + B.drag_coef(s, sp, p), cl + B.lift_coef(s, sp, p), n + 1
        B.advance(b, B.DT, p)
    return cd / n, cl / n


def best_angle(v, spin) -> int:
    return max(range(10, 66), key=lambda a: land(v, a, spin).x)


def rollout(v0, p=B.P, marks=(10.0, 20.0, 30.0)):
    """Launch on the ground with no spin and enter Roll. Returns ({mark: t}, stop x, stop t)."""
    b, times = B.Ball((0.0, 0.0, p.r), (v0, 0.0, 0.0)), {}
    B.enter_roll(b, 0.0, p)
    while b.mode != B.REST and b.t < 60.0:
        x0, t0 = b.p[0], b.t
        B.advance(b, B.DT, p)
        times.update({m: t0 + (m - x0) / (b.p[0] - x0) * B.DT for m in marks if m not in times and x0 < m <= b.p[0]})
    return times, b.p[0], b.t


def mean_speed(v0, dist, p=B.P) -> float:
    t = rollout(v0, p, (dist,))[0].get(dist)
    return dist / t if t else 0.0


# --- 1. ball flight ---
def section_flight(out):
    print("## 1. Ball flight\n")
    print("Launch from the ground (centre at z = r) along +x. Backspin is about −y, sidespin about +z (bends to +y). "
          "Carry = first ground contact; apex = highest centre height. Engine integrator (RK2, 240 Hz).\n")
    grid = [(f"grid {n}", v, a, None, s) for n, s in SPINS.items() for v in SPEEDS for a in ANGLES]
    rows = []
    for case, v, a, _, s in grid:
        L = land(v, a, s)
        rows.append((case[5:], v, a, f(L.x), f(L.y), f(L.t), f(L.apex), f(L.speed)))
    table(("spin", "v0 m/s", "angle °", "carry m", "lateral m", "time s", "apex m", "v_land m/s"), rows)

    print("### 1b. Calibration checks\n")
    rows = []
    for p, tag in ((B.P, "fallback C_D0"), (B.BallParams(cd0_curve="brazuca"), "Brazuca C_D0")):
        for s, name, tgt in ((NONE, "no spin", "37 m, 2.0 s"), (BACK4, "4 rev/s back", "51 m, 3.1 s")):
            L = land(25, 25, s, p=p)
            rows.append((f"25 m/s, 25°, {name} ({tag})", f"{f(L.x, 1)} m, {f(L.t)} s", f"{tgt} "))
    box = [land(v, a, (0.0, -r * B.REV, 0.0)) for v in (21.7, 26.4) for a in (17, 29.5) for r in (2, 6)]
    span = lambda xs: f"{f(min(xs), 1)}–{f(max(xs), 1)}"   # noqa: E731
    rows.append(("JSME14 box: 21.7–26.4 m/s, 17–29.5°, 2–6 rev/s back (8 corners)",
                 f"carry {span([L.x for L in box])} m, apex {span([L.apex for L in box])} m",
                 "carry 39–51 m, apex 5–10 m [JSME14]"))
    cd, cl = mean_coefs(24, 23, BACK4)
    rows += [("mean C_D, C_L of a 24 m/s, 23°, 4 rev/s back flight", f"{f(cd, 3)}, {f(cl, 3)}",
              "0.205 (0.11–0.26), 0.237 (0.11–0.31) [JSME14]"),
             ("LP11 punt: 27.1 m/s, 43.9°, 3.9 rev/s top, from 0.55 m", f"{f(land(27.1, 43.9, TOP39, 0.55).x, 1)} m",
              "39.5 ± 2.7 m [LP11]"),
             ("GK punt 24 m/s, 42°: no spin / 3 rev/s top", f"{f(land(24, 42).x, 1)} / {f(land(24, 42, TOP3).x, 1)} m",
              "no carry in calibration"),
             ("best punt angle, 24 m/s, 3 rev/s top", f"{best_angle(24, TOP3)}°", "≈ 45° optimum; 40–44° used [LP11]"),
             ("goal kick 25 m/s, 27°: no spin / 4 rev/s back",
              f"{f(land(25, 27).x, 1)} / {f(land(25, 27, BACK4).x, 1)} m", "no carry in calibration"),
             ("best goal-kick angle, 25 m/s: no spin / 4 rev/s back",
              f"{best_angle(25, NONE)}° / {best_angle(25, BACK4)}°", "max carry at 25–30° [SAK26]")]
    wz = 36.4 * math.sin(math.radians(62.6))      # WH10: 36.4 rad/s, axis 62.6° above the horizontal
    curls = [(19.2, wz, 20.12, "deflection 1.49 m (1.37–1.55) [WH10], from the outermost point, so ≤ the bend")]
    curls += [(v, r * B.REV, 20.0, "20–28 m/s, 5–10 rev/s (no bend target)") for v in (20, 24, 28) for r in (5, 10)]
    for v, w, x_line, tgt in curls:
        a, y = curl(v, (0.0, 0.0, w), x_line)
        rows.append((f"curl {v} m/s, {f(w / B.REV, 1)} rev/s about z, crosses {x_line} m at 1.84 m",
                     f"angle {f(a, 1)}°, bend {f(y)} m", tgt))
    table(("case", "model", "target"), rows)

    print(f"### 1c. Spin-drag slope k in C_D = C_D0 + k·min(Sp, {B.P.sp_max})\n")
    rows = []
    for k in (0.5, 0.3, 0.2, 0.1, 0.0):
        p = B.BallParams(cd_spin=k)
        sp = B.spin_ratio((26.0 * math.cos(0.26), 0.0, 26.0 * math.sin(0.26)), (0.0, 0.0, 7.5 * B.REV), p)
        rows.append((f"{k} (spec)" if k == B.P.cd_spin else k, f"{f(land(25, 25, BACK4, p=p).x, 1)} m",
                     f(mean_coefs(24, 23, BACK4, p)[0], 3), f(B.drag_coef(26.0, sp, p), 3),
                     f"{f(land(27.1, 43.9, TOP39, 0.55, p).x, 1)} m"))
    table(("k", "25 m/s 25° 4 rev/s back (51 m)", "mean C_D lofted (0.205; 0.11–0.26)",
           "C_D curl 26 m/s 7.5 rev/s (0.25–0.30)", "LP11 punt (39.5 ± 2.7 m)"), rows)

    extra = [("cal 25@25 none", 25, 25, None, NONE), ("cal 25@25 back4", 25, 25, None, BACK4),
             ("LP11 punt", 27.1, 43.9, 0.55, TOP39), ("GK punt top3", 24, 42, None, TOP3),
             ("goal kick", 25, 27, None, NONE), ("WH10 curl", 19.2, 28.0, None, (0.0, 0.0, wz)),
             ("FK curl 28 side10", 28, 15, None, (0.0, 0.0, 10 * B.REV))]
    rows = []
    for case, v, a, z0, s in grid + extra:
        L, pos = land(v, a, s, z0), B.run(kick(v, a, s, z0), 2.0, samples=(0.5, 1.0, 2.0))
        rows.append((case, float(v), float(a), B.P.r if z0 is None else z0, *map(float, s),
                     L.x, L.y, L.t, L.apex, L.speed, *pos[0.5], *pos[1.0], *pos[2.0]))
    write_csv(out, "ball_flights.csv", ("case", "speed", "angle_deg", "z0", "wx", "wy", "wz", "land_x", "land_y",
                                        "flight_t", "apex_z", "land_speed")
              + tuple(f"{c}_{t}" for t in ("0.5", "1.0", "2.0") for c in "xyz"), rows)


# --- 2. integrator accuracy ---
def section_integrators():
    print("## 2. Integrator accuracy\n")
    print("Landing-point distance from RK4 at 1 kHz, in mm. RK2 = engine (midpoint, 240 Hz, exact spin decay, "
          "contact by linear interpolation in the sub-step). Euler = semi-implicit Euler at 240 Hz, comparison only.\n")
    rows, lofted = [], []
    for v in SPEEDS:
        for a in ANGLES:
            row = [v, a]
            for name, s in SPINS.items():
                ref, r2 = B.flight_ref(kick(v, a, s)), land(v, a, s)
                eu = B.flight(kick(v, a, s), air_step=B.euler_air)
                d2, de = math.hypot(r2.x - ref.x, r2.y - ref.y), math.hypot(eu.x - ref.x, eu.y - ref.y)
                row += [f(1000 * d2, 2), f(1000 * de, 0)]
                if 40.0 <= ref.x <= 60.0:
                    lofted.append((d2, de, v, a, name, ref.x))
            rows.append(row)
    table(("v0", "angle", "RK2 none", "Euler none", "RK2 back", "Euler back", "RK2 side", "Euler side"), rows)
    w2, we = max(lofted), max(lofted, key=lambda x: x[1])
    ref = B.flight_ref(kick(25, 25, BACK4))
    fine = B.flight_ref(kick(25, 25, BACK4), B.rk4_step, B.DT_REF / 4)
    e480 = B.flight(kick(25, 25, BACK4), h=B.DT / 2, air_step=B.euler_air)
    print(f"Carries of 40–60 m ({len(lofted)} cases): RK2 max {f(1000 * w2[0], 2)} mm ({w2[2]} m/s, {w2[3]}°, "
          f"{w2[4]}, carry {f(w2[5], 1)} m); Euler max {f(1000 * we[1], 0)} mm ({we[2]} m/s, {we[3]}°, {we[4]}, "
          f"carry {f(we[5], 1)} m). Target < 100 mm.\n"
          f"Reference check (25 m/s, 25°, 4 rev/s back): RK4 at 1 kHz vs 4 kHz {f(1e6 * abs(ref.x - fine.x), 2)} µm; "
          f"Euler at 480 Hz misses by {f(1000 * abs(e480.x - ref.x), 0)} mm.\n")


# --- 3. roll-out ---
def section_roll(out):
    print("## 3. Roll-out\n")
    print("Launch at z = r with no spin, enter Roll. The entry grip rule sees no normal impulse, so the ball keeps its "
          "speed. Rolling drag uses C_D0 (Sp = 0). Engine integrator (RK2, 240 Hz). Rows 3.2 and 3.46 m/s are the "
          "FIFA ramp test (measured and ideal exit speed).\n")
    rows, gold = [], []
    for v0 in (3.2, 3.46, 8.0, 10.0, 12.0, 14.0, 16.0, 17.0, 20.0):
        t, xs, ts = rollout(v0)
        gold.append((v0, *(t.get(d, math.nan) for d in (10.0, 20.0, 30.0)), xs, ts,
                     20.0 / t[20.0] if 20.0 in t else math.nan))
        rows.append(tuple(f(x) for x in gold[-1]))
    table(("v0 m/s", "t 10 m", "t 20 m", "t 30 m", "stop m", "stop s", "mean 0–20 m"), rows)
    write_csv(out, "rollouts.csv", ("v0", "t_10", "t_20", "t_30", "stop_x", "stop_t", "mean_speed_20"), gold)
    table(("mean speed target", "launch speed needed", "source"),
          [(f"{tgt} m/s over {d} m", f(bisect(lambda v: mean_speed(v, d), 3.0, 40.0, tgt), 1), src) for tgt, d, src in
           ((13.7, 20.0, "16 m/s launch"), (10.1, 10.0, "SB median, 5–15 m"),
            (13.4, 22.5, "SB median, 15–30 m"), (15.2, 37.5, "SB median, 30–45 m"))])
    variants = [("spec (a_roll 0.6)", B.P)] + [(f"a_roll {a}", B.BallParams(a_roll=a)) for a in (0.45, 0.7, 1.0, 1.5, 0.0)]
    variants += [("rolling drag with the rolling spin (Sp = 1, capped at 0.4)", B.BallParams(roll_drag_sp=1.0)),
                 ("literal: full grip on Roll entry", B.BallParams(entry_full_grip=True)),
                 ("Brazuca C_D0", B.BallParams(cd0_curve="brazuca"))]
    table(("variant", "mean 0–20 m, 14 m/s", "mean 0–20 m, 16 m/s", "FIFA roll from 3.2 m/s"),
          [(n, f(mean_speed(14.0, 20.0, p)), f(mean_speed(16.0, 20.0, p)), f(rollout(3.2, p)[1])) for n, p in variants])
    a_fit = bisect(lambda a: rollout(3.2, B.BallParams(a_roll=a))[1], 0.3, 1.5, 6.23)
    print(f"FIFA roll targets: turf Pro 4–8 m, grass 4–10 m, three stadiums 5.8–6.7 m "
          f"(their mean 6.23 m needs a_roll {f(a_fit)}).\n")


# --- 4. bounce ---
def section_bounce():
    print("## 4. Bounce\n")
    p = B.P
    b = B.Ball((0.0, 0.0, 2.0 + p.r), (0.0, 0.0, 0.0))
    peaks, peak, nev = [], -math.inf, 0
    while b.mode == B.AIR and b.t < 20.0:
        B.advance(b, B.DT, p)
        if len(b.events) > nev:
            peaks, nev, peak = peaks + [peak] * (nev > 0), len(b.events), b.p[2]
        peak = max(peak, b.p[2])
    rows = []
    for i, ev in enumerate(e for e in b.events if e[0] == "bounce"):
        vin, vout = -ev[3][2], ev[4][2]
        rows.append((i + 1, f(vin), f(B.e_n(vin), 3), f(vout / vin, 3),
                     f(peaks[i] - p.r if i < len(peaks) else 0.0, 3), f(vout * vout / (2 * p.g), 3),
                     "Roll" if vout == 0.0 else "Air"))
    print("Drop from 2.0 m (ball bottom), no spin. Rebound = height of the ball bottom at the next apex.\n")
    table(("bounce", "impact m/s", "e_n formula", "v_out/v_in", "rebound m", "no-drag m", "state after"), rows)
    print(f"Targets: first rebound 0.60–0.85 m (turf Pro), 0.60–1.00 m (grass); rebound resilience 31.7–44.4 % of "
          f"the drop [BAK90]. Air → Roll below |v_z'| = {p.v_to_roll} m/s: the last hop is at most "
          f"{f(1000 * p.v_to_roll ** 2 / (2 * p.g), 1)} mm high and {f(2 * p.v_to_roll / p.g, 3)} s long.\n")
    rows = []
    for name, v, a, rev, mu in (("45°, 15 m/s, no spin", 15.0, 45.0, 0.0, p.mu),
                                ("45°, 15 m/s, 10 rev/s back", 15.0, 45.0, -10.0, p.mu),
                                ("45°, 15 m/s, 10 rev/s top", 15.0, 45.0, 10.0, p.mu),
                                ("FIFA angle test, 13.89 m/s at 30°, dry μ 0.6", 13.89, 30.0, 0.0, 0.6),
                                ("FIFA angle test, 13.89 m/s at 30°, wet μ 0.3", 13.89, 30.0, 0.0, 0.3)):
        r = math.radians(a)
        vo, wo, grip = B.bounce((v * math.cos(r), 0.0, -v * math.sin(r)), (0.0, rev * B.REV, 0.0), p, mu)
        rows.append((name, f(B.norm(vo)), f(math.degrees(math.atan2(vo[2], vo[0])), 1), f(wo[1] / B.REV),
                     f(100 * B.norm(vo) / v, 1), "grips" if grip else "slides"))
    print("Incoming along +x and down. Out spin about +y in rev/s (+ = topspin).\n")
    table(("case", "out m/s", "out angle °", "out spin rev/s", "out/in %", "contact"), rows)
    print("FIFA angle test targets (out/in): turf dry 45–60 % (Pro), wet 45–80 %, natural grass 60–64 %.\n")


# --- 5. sprints ---
def lead(fast: Y.Player, slow: Y.Player, dist: float) -> float:
    """Metres the slow player is behind when the fast one reaches dist (both from rest)."""
    t = Y.sprint(fast, 0.0, (dist,))[dist][0]
    k = int(t / Y.DT)
    xs = Y.run_positions(slow, 0.0, k + 1)
    return dist - (xs[k] + (t / Y.DT - k) * (xs[k + 1] - xs[k]))


def section_sprint(out):
    print("## 5. Sprints\n")
    print("Sim at 60 Hz, sprint effort, straight along +x. Times interpolated inside the tick (s).\n")
    marks, rows, gold = (5.0, 10.0, 20.0, 30.0), [], []
    for pa, ac in ((-1, -1), (0, 0), (1, 1), (1, -1), (-1, 1)):
        for v0 in (0.0, 3.6):
            r = Y.sprint(Y.Player(pa, ac), v0, marks)
            rows.append((pa, ac, v0, *(f(r[d][0], 3) for d in marks)))
            gold.append((float(pa), float(ac), v0, *(r[d][0] for d in marks), *(r[d][1] for d in marks)))
    table(("Pace x", "Acc x", "v0 m/s", "5 m", "10 m", "20 m", "30 m"), rows)
    write_csv(out, "sprints.csv", ("pace_x", "acc_x", "v0", "t_5", "t_10", "t_20", "t_30",
                                   "ticks_5", "ticks_10", "ticks_20", "ticks_30"), gold)
    print("By display value (Pace = Acceleration), from standing, against the calibration splits:\n")
    rows = []
    for disp in (1, 5, 10, 15, 20):
        x = Y.x_attr(disp)
        r = Y.sprint(Y.Player(x, x), 0.0, marks)
        rows.append((disp, f(x, 3), *(f(r[d][0]) for d in marks)))
    rows.append(("calibration", "v 9.0, τ 1.2", "–", "2.10", "3.35", "4.51"))
    table(("display", "x", "5 m", "10 m", "20 m", "30 m"), rows)
    print("Lead of the faster player when he reaches the mark, from rest (m, and m per display point):\n")
    disp = lambda d: Y.Player(Y.x_attr(d), Y.x_attr(d))   # noqa: E731
    rows = []
    for name, fast, slow, pts in (("(+1,+1) over (−1,−1)", Y.Player(1, 1), Y.Player(-1, -1), 19),
                                  ("display 11 over 10", disp(11), disp(10), 1),
                                  ("display 20 over 1", disp(20), disp(1), 19),
                                  ("Pace only: (+1,0) over (−1,0)", Y.Player(1, 0), Y.Player(-1, 0), 19),
                                  ("Acc only: (0,+1) over (0,−1)", Y.Player(0, 1), Y.Player(0, -1), 19)):
        rows.append((name, *(f"{f(x)} ({f(x / pts, 3)}/pt)" for x in (lead(fast, slow, d) for d in (10.0, 20.0, 30.0)))))
    a, b = Y.Player(1, -1), Y.Player(-1, 1)
    ahead = lambda d: lead(a, b, d) if Y.sprint(a, 0.0, (d,))[d][0] < Y.sprint(b, 0.0, (d,))[d][0] else -lead(b, a, d)  # noqa: E731
    rows += [("(+1,−1) vs (−1,+1): + = Pace side ahead", *(f(ahead(d)) for d in (10.0, 20.0, 30.0)))]
    table(("pair", "10 m", "20 m", "30 m"), rows)


# --- 6. reach time ---
def reach_cases():
    """(label, player, v0, target, react, effort cap). Foot reach 1.0 m in all cases."""
    pl, s, c = Y.Player(), math.sqrt(0.5), []
    for pa in (-1.0, 0.0, 1.0):
        c += [("straight", Y.Player(pa, pa), (0.0, 0.0), (d, 0.0), 0.0, None) for d in (2.0, 5.0, 10.0, 20.0, 30.0, 40.0)]
    c += [("straight, jog", pl, (0.0, 0.0), (d, 0.0), 0.0, Y.EFFORT["jog"]) for d in (5.0, 10.0)]
    c += [("straight, react 0.25", pl, (0.0, 0.0), (10.0, 0.0), 0.25, None)]
    for v0 in (3.6, 6.0):
        for d in (5.0, 10.0, 20.0):
            c += [("straight", pl, (v0, 0.0), (d, 0.0), 0.0, None), ("turn 45°", pl, (v0, 0.0), (d * s, d * s), 0.0, None),
                  ("turn 90°", pl, (v0, 0.0), (0.0, d), 0.0, None), ("reversal 180°", pl, (v0, 0.0), (-d, 0.0), 0.0, None)]
        c += [("turn 45°, react 0.25", pl, (v0, 0.0), (10.0 * s, 10.0 * s), 0.25, None),
              ("reversal 179°", pl, (v0, 0.0), (-10.0 * math.cos(math.radians(1)), 10.0 * math.sin(math.radians(1))),
               0.0, None)]
    for d in (3.0, 5.0, 10.0):
        c += [("turn 30°, sprinting", pl, (7.5, 0.0), (d * math.cos(math.radians(30)), d * 0.5), 0.0, None)]
    return c


def section_reach(out):
    print("## 6. Reach time: closed form vs 60 Hz simulation\n")
    print("Seek at the effort cap (sprint unless noted) with the steering rules (turning-circle cap, "
          "brake first for a target behind, per-part limits then one elliptical limit, the across part only turns v, "
          "trapezoid position); facing = direction of motion at start; foot reach 1.0 m. Closed form = (no "
          "turn term; a braking phase when u < 0). err = closed − sim (sim time interpolated in the tick).\n")
    rows, gold, groups = [], [], {}
    for label, pl, v0, q, react, cap in reach_cases():
        ts, ticks, path = Y.simulate_reach(pl, (0.0, 0.0), v0, 0.0, q, 1.0, react, cap)
        tc = Y.reach_time(pl, (0.0, 0.0), v0, q, 1.0, react, cap)
        d = math.hypot(*q)
        rows.append((label, f(pl.pace, 0), f(v0[0], 1), f(d, 0), f(ts, 3), ticks, f(tc, 3), f(tc - ts, 3),
                     f(path / (d - 1.0))))
        gold.append((label, pl.pace, pl.acc, pl.agi, *v0, 0.0, *q, 1.0, react, cap or pl.v_top, ts, ticks, tc, path))
        groups.setdefault((label.split(",")[0], v0[0]), []).append((tc - ts, ticks * Y.DT - tc))
    table(("case", "Pace=Acc x", "v0", "dist", "sim s", "ticks", "closed s", "err", "path / (dist − reach)"), rows)
    write_csv(out, "reach_times.csv", ("case", "pace_x", "acc_x", "agi_x", "v0x", "v0y", "facing_deg", "qx", "qy",
                                       "reach", "react", "v_cap", "t_sim", "ticks_sim", "t_closed", "path_sim"), gold)
    print("Summary (s). 'vs tick' compares with the first tick at which the body is in reach. 'max late' is the "
          "largest positive err (closed form later than the simulation).\n")
    stats = ((max, 0), (statistics.fmean, 0), (max, 1))
    table(("group", "v0", "n", "max |err|", "mean |err|", "max |err| vs tick", "max late"),
          [(g, f(v0, 1), len(e), *(f(fn([abs(x[i]) for x in e]), 3) for fn, i in stats), f(max(x[0] for x in e), 3))
           for (g, v0), e in groups.items()])
    print("Spec targets: ≤ 1 tick (0.0167 s) for straight runs from rest; ≤ 0.05 s for 180° reversals; "
          "never more than 0.1 s late in any case, so the solver can prune with it. Turns are early: the "
          "solver confirms a turning player's first candidate with a rollout of the body model.\n")
    late, early, case, worst_c, t_med, t_max = lower_bound_sweep()
    print(f"Lower-bound sweep (27 attribute sets × 8 start speeds 0–9 m/s × 13 angles 0–180° × 7 distances "
          f"1.5–40 m): the closed form is at most {f(late, 3)} s late and at most {f(early, 2)} s early "
          f"(worst early case: {case}). The checked reach time (rollout while the speed across the line to q is "
          f"at least 1 m/s or q is more than 90° from the facing, then the closed form) is within "
          f"{f(worst_c, 3)} s of the simulation; its rollout is "
          f"{t_med} ticks at the median and {t_max} at most.\n")


def lower_bound_sweep():
    """Closed form and checked reach time against the simulation over a grid. Returns (max late, max early,
    worst early case) of the closed form, then (max |err|, median and max rollout ticks) of the checked one."""
    late, early, case, worst_c, ticks = 0.0, 0.0, "", 0.0, []
    for pa in (-1.0, 0.0, 1.0):
        for ac in (-1.0, 0.0, 1.0):
            for ag in (-1.0, 0.0, 1.0):
                pl = Y.Player(pa, ac, ag)
                for v0 in (0.0, 1.0, 2.0, 3.6, 5.0, 6.0, 7.5, 9.0):
                    if v0 > pl.v_top:
                        continue
                    for deg in (0, 10, 20, 30, 45, 60, 90, 119, 121, 135, 150, 179, 180):
                        for d in (1.5, 3.0, 5.0, 10.0, 20.0, 30.0, 40.0):
                            q = (d * math.cos(math.radians(deg)), d * math.sin(math.radians(deg)))
                            ts = Y.simulate_reach(pl, (0.0, 0.0), (v0, 0.0), 0.0, q, 1.0)[0]
                            e = Y.reach_time(pl, (0.0, 0.0), (v0, 0.0), q, 1.0) - ts
                            late = max(late, e)
                            if -e > early:
                                early, case = -e, f"{v0} m/s, {deg}°, {d} m"
                            tc, n, _ = Y.reach_time_checked(pl, (0.0, 0.0), (v0, 0.0), q, 1.0)
                            worst_c = max(worst_c, abs(tc - ts))
                            if n > 0:
                                ticks.append(n)
    ticks.sort()
    return late, early, case, worst_c, ticks[len(ticks) // 2], ticks[-1]


def section_conditioning():
    print("### 6b. Conditioning of the steering rule\n")
    print("The spec rule: the wanted change across v only turns v onto the direction of v_des, "
          "min(|v|, |v_des|)·sin δ (δ = angle between v and v_des). 'old split' = the earlier rule, comparison only: "
          "the part of Δ = v_des − v across v, |v_des|·sin δ. Below |v_des|/2 that turns v past v_des, so a heading "
          "error flips sign and grows every tick until a_n reaches a_lat; the elliptical limit then cuts a_t. "
          "Rounding seeds it. 'noise' = the velocity turned by a random angle (σ 1e-7 rad, like float rounding) "
          "after each tick. err = closed − sim (s).\n")
    pl, r30, rows = Y.Player(), math.radians(30.0), []
    cases = [(f"from rest, {d:.0f} m along {name}", (0.0, 0.0), (d * math.cos(a), d * math.sin(a)), a)
             for d in (10.0, 30.0) for name, a in (("x", 0.0), ("30°", r30))]
    cases += [(f"reversal 180°, {v:.1f} m/s, 10 m{tag}", (v, 0.0), (-10.0, off), 0.0)
              for v in (3.6, 6.0) for tag, off in (("", 0.0), (", target 1e-15 m off the axis", 1e-15))]
    cases += [("turn 90°, 3.6 m/s, 5 m", (3.6, 0.0), (0.0, 5.0), 0.0),
              ("turn 45°, 6.0 m/s, 10 m", (6.0, 0.0), (10.0 * math.sqrt(0.5), 10.0 * math.sqrt(0.5)), 0.0)]
    for label, v0, q, face in cases:
        tc = Y.reach_time(pl, (0.0, 0.0), v0, q, 1.0)
        errs = [tc - Y.simulate_reach(pl, (0.0, 0.0), v0, face, q, 1.0, rotate_only=old, noise=nz)[0]
                for old in (False, True) for nz in (0.0, 1e-7)]
        rows.append((label, f(tc, 3), *(f(e, 3) for e in errs)))
    table(("case", "closed s", "err spec rule", "err spec rule, noise", "err old split", "err old split, noise"),
          rows)


# --- 7. pass error ---
def section_pass(mc_n):
    print("## 7. Pass error (ground pass σ0, no pressure, strong foot)\n")
    xs, Ds, Ls = (-1.0, 0.0, 1.0), (0.0, 0.5, 1.0), (10.0, 20.0, 30.0, 40.0)
    print("Lateral miss at L for a 1σ direction error, L·tan(σ_dir), in m. σ_speed = 4 %·(1 + D)·e^(−0.55x).\n")
    table(("L m", *(f"x {x:+.0f}, D {D}" for x in xs for D in Ds)),
          [(L, *(f(L * math.tan(X.sigmas("ground_pass", D, x).dir)) for x in xs for D in Ds)) for L in Ls])
    print("Mishit probability (%):\n")
    table(("D", "x −1, p̂ 0", "x 0, p̂ 0", "x +1, p̂ 0", "x −1, p̂ 1", "x 0, p̂ 1", "x +1, p̂ 1"),
          [(D, *(f(100 * X.p_mishit(D, x, ph), 1) for ph in (0.0, 1.0) for x in xs)) for D in (0.0, 0.5, 1.0, 2.0)])
    print("Spec table check: " + ", ".join(f"(D {D}, x {x}, p̂ {ph}) {f(100 * X.p_mishit(D, x, ph), 1)} %"
                                            for D, x, ph in ((0, 0, 0), (1, 0, 0), (1, -1, 1), (2, -1, 0)))
          + " (spec: 1.1, 4.7, 23, 38 %).\n")
    print(f"P_exec, r_adj = 1.5 m, tol_speed = 15 %: closed form / Monte Carlo ({mc_n} samples, seed 7).\n")
    rows, worst = [], 0.0
    for L in Ls:
        for x in xs:
            cells = []
            for D in Ds:
                s, pm, td = X.sigmas("ground_pass", D, x), X.p_mishit(D, x), X.pass_tol_dir(L)
                pc, pmc = X.p_exec(s, pm, td, 0.15), X.p_exec_mc(s, pm, td, 0.15, mc_n)
                worst = max(worst, abs(pc - pmc))
                cells.append(f"{f(pc, 3)} / {f(pmc, 3)}")
            rows.append((L, f(x, 0), *cells))
    table(("L m", "Passing x", "D 0", "D 0.5", "D 1"), rows)
    p15 = X.p_exec(X.sigmas("ground_pass", 0.0, 0.0), X.p_mishit(0.0, 0.0), X.pass_tol_dir(15.0), 0.15)
    print(f"Max |closed − MC| = {f(worst, 4)} (target ≤ 0.01). Open 15 m pass, x 0, D 0: P_exec {f(p15, 3)} "
          "(scenario target: completion ≥ 0.97).\n")


# --- 8. keeper dive ---
def section_dive():
    print("## 8. Keeper dive (closed form)\n")
    print("Lateral reach (m) = hands 1.4 m + push-off speed (2.7 + 0.3·x m/s) × (t − 0.10 s set). "
          "Upper bound: no loss in the slide.\n")
    table(("Agility x", "0.8 s", "1.0 s", "1.2 s"), [(x, *(f(Y.dive_reach(t, x)) for t in (0.8, 1.0, 1.2)))
                                                    for x in (-1.0, 0.0, 1.0)])
    print(f"Target 3.8 m at 1.0 s after onset: Agility x = 0 reaches {f(Y.dive_reach(1.0, 0.0))} m. "
          f"With 2.7 m/s the target needs hands of {f(3.8 - 2.7 * 0.9)} m.")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--no-golden", action="store_true", help="do not write the golden CSV files")
    ap.add_argument("--mc", type=int, default=20000, help="Monte Carlo samples per P_exec case")
    args = ap.parse_args(argv)
    out = None if args.no_golden else GOLDEN
    if out:
        os.makedirs(out, exist_ok=True)
    print("# Lab tables\n")
    section_flight(out)
    section_integrators()
    section_roll(out)
    section_bounce()
    section_sprint(out)
    section_reach(out)
    section_conditioning()
    section_pass(args.mc)
    section_dive()


if __name__ == "__main__":
    main()
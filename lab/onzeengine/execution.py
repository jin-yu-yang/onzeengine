"""Touch error model.

Angles are in radians inside the code. The σ0 table keeps the spec's degrees.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass

B_SKILL = 0.55                              # σ × exp(−b·x_skill)
MISHIT = (-4.5, 1.5, 1.0, 0.8)              # c0, c1, c2, c3 of logit(p_mishit)
HARD_TERMS = ("incoming", "redirect", "height", "body", "air")   # Technique reduces these
D_AIR = {"upright": 0.0, "airborne": 0.3, "sliding": 0.4}

# Start values: σ0_dir (deg), σ0_loft (deg), σ0_speed (share), σ0_spin (share), speed floor (m/s)
SIGMA0 = {
    "ground_pass": (1.5, 0.5, 0.04, None, None),
    "lofted_pass": (2.5, 2.0, 0.06, 0.20, None),
    "shot_power": (2.5, 2.0, 0.05, 0.20, None),
    "shot_placed": (1.5, 1.2, 0.04, 0.20, None),
    "first_touch": (6.0, 4.0, 0.15, None, 0.4),
    "dribble": (5.0, None, 0.12, None, 0.3),
    "header": (5.0, 4.0, 0.12, None, None),
    "throw": (2.0, 2.0, 0.05, None, None),
    "keeper_kick": (3.0, 2.5, 0.06, 0.20, None),
    "tackle": (15.0, None, 0.25, None, None),
}


# --- difficulty terms; each is dimensionless and ≥ 0 ---
def d_power(v_d: float, v_max: float) -> float: return (v_d / v_max) ** 2
def d_incoming(v_rel: float) -> float: return (v_rel / 20.0) ** 2
def d_redirect(theta_r: float, v_in: float) -> float: return theta_r / math.pi * v_in / 20.0
def d_body(direction: float) -> float: return (direction / math.pi) ** 2
def d_stretch(stretch: float) -> float: return max(0.0, stretch - 0.6) ** 2 / 0.16
def d_rush(t_done: float, t_setup: float) -> float: return max(0.0, 1.0 - t_done / t_setup)
def d_first(t_since_react: float) -> float: return 0.5 * max(0.0, 1.0 - t_since_react)
def d_motion(v_p: float, v_top: float) -> float: return 0.5 * (v_p / v_top) ** 2


def difficulty(terms: dict, x_technique: float = 0.0, weights: dict | None = None) -> float:
    """D = Σ w_i·d_i. Technique scales the hard-contact terms by (1 − 0.25·x_Technique)."""
    total = 0.0
    for name, d in terms.items():
        if name in HARD_TERMS:
            d *= 1.0 - 0.25 * x_technique
        total += (1.0 if weights is None else weights.get(name, 1.0)) * d
    return total


@dataclass(frozen=True)
class Sigmas:
    dir: float      # rad
    loft: float     # rad
    speed: float    # share of |v_d|
    spin: float     # share of |ω_d|


def sigmas(kind: str, D: float, x_skill: float, pressure: float = 0.0, x_composure: float = 0.0,
           weak_foot: int | None = None) -> Sigmas:
    """σ_k = σ0_k·(1 + D)·exp(−b·x_skill)·(1 + p̂·(0.6 − 0.4·x_Composure))·m_weak.
    weak_foot: the weak-foot rating 1–20 when the weaker foot kicks, else None."""
    s_dir, s_loft, s_speed, s_spin, _ = SIGMA0[kind]
    m_weak = 1.0 if weak_foot is None else 1.0 + 0.04 * (20 - weak_foot)
    f = (1.0 + D) * math.exp(-B_SKILL * x_skill) * (1.0 + pressure * (0.6 - 0.4 * x_composure)) * m_weak
    return Sigmas(math.radians(s_dir) * f, math.radians(s_loft or 0.0) * f, s_speed * f, (s_spin or 0.0) * f)


def p_mishit(D: float, x_skill: float, pressure: float = 0.0) -> float:
    """logit(p) = c0 + c1·D − c2·x_skill + c3·p̂."""
    c0, c1, c2, c3 = MISHIT
    return 1.0 / (1.0 + math.exp(-(c0 + c1 * D - c2 * x_skill + c3 * pressure)))


def p_exec(sig: Sigmas, p_mis: float, tol_dir: float, tol_speed: float) -> float:
    """Closed form: (1 − p_mishit)·(2Φ(tol_dir/σ_dir) − 1)·(2Φ(tol_speed/σ_speed) − 1)."""
    r2 = math.sqrt(2.0)
    return (1.0 - p_mis) * math.erf(tol_dir / sig.dir / r2) * math.erf(tol_speed / sig.speed / r2)


def pass_tol_dir(length: float, r_adj: float = 1.5) -> float:
    """tol_dir = atan(r_adj/L) for a pass of length L."""
    return math.atan(r_adj / length)


def _rotate(v, yaw: float, loft: float):
    """Turn v by yaw about z and by loft about the horizontal axis across v."""
    h, s = math.hypot(v[0], v[1]), math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    az, el = math.atan2(v[1], v[0]) + yaw, math.atan2(v[2], h) + loft
    return (s * math.cos(el) * math.cos(az), s * math.cos(el) * math.sin(az), s * math.sin(el))


def _clipped_normal(rng: random.Random) -> float:
    return max(-3.0, min(3.0, rng.gauss(0.0, 1.0)))


def realize_kick(v_d, sig: Sigmas, p_mis: float, rng: random.Random, lofted: bool = False):
    """Sample one kick. Draw order: n_dir, n_loft, n_speed, n_spin, mishit test, outcome.
    Returns (out velocity, mishit flag)."""
    n_dir, n_loft, n_speed, _n_spin = (_clipped_normal(rng) for _ in range(4))
    if rng.random() < p_mis:
        scale = rng.uniform(0.30, 0.75)
        yaw = rng.gauss(0.0, math.radians(20.0))
        # Spec gives no size for the "goes long" loft error of a lofted mishit; the lab uses |σ_loft·n|.
        loft = abs(sig.loft * _clipped_normal(rng)) if lofted else rng.uniform(0.0, math.radians(10.0))
        out = _rotate(v_d, yaw, loft)
        return (out[0] * scale, out[1] * scale, out[2] * scale), True
    out = _rotate(v_d, sig.dir * n_dir, sig.loft * n_loft)
    k = 1.0 + sig.speed * n_speed
    return (out[0] * k, out[1] * k, out[2] * k), False


def p_exec_mc(sig: Sigmas, p_mis: float, tol_dir: float, tol_speed: float, n: int = 20000,
              seed: int = 7, speed: float = 15.0) -> float:
    """Monte Carlo of a ground pass along +x: success = direction and speed inside the tolerances."""
    rng = random.Random(seed)
    ok = 0
    for _ in range(n):
        out, _ = realize_kick((speed, 0.0, 0.0), sig, p_mis, rng)
        s = math.sqrt(out[0] ** 2 + out[1] ** 2 + out[2] ** 2)
        if abs(math.atan2(out[1], out[0])) <= tol_dir and abs(s / speed - 1.0) <= tol_speed:
            ok += 1
    return ok / n
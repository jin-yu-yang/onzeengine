"""Ball model

World frame: x along the pitch, y across, z up. SI units. Spin w is in rad/s.
The engine integrator is RK2 (midpoint) at 240 Hz (4 sub-steps per 1/60 s tick) for position and
velocity, with exact exponential spin decay), in flight and while rolling.
Semi-implicit Euler at the same step is kept only as a comparison for the accuracy table.
RK4 at 1 kHz uses the same forces and is the reference.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

DT = 1.0 / 240.0        # engine sub-step
DT_REF = 1.0 / 1000.0   # RK4 reference step
REV = 2.0 * math.pi     # rad per revolution
AIR, ROLL, REST = "air", "roll", "rest"


@dataclass(frozen=True)
class BallParams:
    """Constants. The defaults are the spec values."""
    m: float = 0.430              # kg
    r: float = 0.110              # m
    rho: float = 1.20             # kg/m³
    g: float = 9.81               # m/s²
    k: float = 2.0 / 3.0          # I = k·m·r² (thin shell)
    cd_spin: float = 0.3          # C_D = C_D0(v) + cd_spin·min(Sp, sp_max)
    cl_max: float = 0.32          # C_L = cl_max·(1 − e^(−min(Sp, sp_max)/cl_sp))·clamp((v − 10.5)/3.5, 0, 1)
    cl_sp: float = 0.13
    sp_max: float = 0.4           # Sp cap in C_D and C_L (the fit data end near Sp 0.4)
    tau_spin: float = 25.0        # s, spin decay in flight
    mu: float = 0.6               # bounce friction: 0.6 dry, 0.3 wet
    mu_slide: float = 0.4         # grip rule on entry to Roll
    a_roll: float = 0.6           # m/s², rolling resistance
    v_to_roll: float = 0.30       # m/s: after a bounce, |v_z'| below this → Roll
    v_stop: float = 0.05          # m/s
    side_decay: float = 5.0       # 1/s, sidespin decay while rolling
    cd0_curve: str = "fallback"   # "fallback" (two regimes) or "brazuca" (KL18 fit)
    roll_drag_sp: float = 0.0     # Sp in the rolling drag. 0: C_D0 only (spec); 1: the rolling spin (capped at sp_max)
    entry_full_grip: bool = False  # True: Roll entry removes all slip (no friction limit)

    @property
    def area(self) -> float:
        return math.pi * self.r * self.r

    @property
    def kq(self) -> float:
        """ρ·A/(2m): drag or lift acceleration per unit C·v²."""
        return 0.5 * self.rho * self.area / self.m


P = BallParams()


# --- small 3D vector helpers (tuples) ---
def add(a, b): return (a[0] + b[0], a[1] + b[1], a[2] + b[2])
def mul(a, s): return (a[0] * s, a[1] * s, a[2] * s)
def dot(a, b): return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
def norm(a): return math.sqrt(dot(a, a))
def cross(a, b): return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
def clamp(x, lo, hi): return lo if x < lo else hi if x > hi else x


# --- coefficients ---
def cd0_fallback(v: float) -> float:
    """Two regimes with a linear drag crisis: 0.47 below 11 m/s, 0.18 at 15 m/s, slow rise after."""
    if v < 11.0:
        return 0.47
    if v < 15.0:
        return 0.47 - 0.29 * (v - 11.0) / 4.0
    return 0.18 + 0.0015 * (v - 15.0)


def cd0_brazuca(v: float) -> float:
    """Kiratidis & Leinweber fit for the Brazuca. Used only for comparison."""
    a, vc, vs, bmin, bmax, vmin, vmax = 0.4740, 12.92, 1.000, 0.1657, 0.2112, 14.61, 35.00
    return ((a - bmin) / (1.0 + math.exp((v - vc) / vs)) + bmin
            + (v - vmin) / (1.0 + math.exp(-(v - vmin) / vs)) * (bmax - bmin) / (vmax - vmin))


def cd0(v: float, p: BallParams = P) -> float:
    return cd0_brazuca(v) if p.cd0_curve == "brazuca" else cd0_fallback(v)


def spin_ratio(v, w, p: BallParams = P) -> float:
    """Sp = r·|ω⊥|/|v|, ω⊥ = spin part perpendicular to the velocity (not capped; the coefficients cap it)."""
    s = norm(v)
    if s == 0.0:
        return 0.0
    return p.r * norm(cross(w, v)) / (s * s)       # |ω × v| = |ω⊥|·|v|


def drag_coef(v: float, sp: float, p: BallParams = P) -> float:
    """C_D = C_D0(v) + cd_spin·min(Sp, sp_max)."""
    return cd0(v, p) + p.cd_spin * min(sp, p.sp_max)


def lift_coef(v: float, sp: float, p: BallParams = P) -> float:
    """C_L = cl_max·(1 − e^(−min(Sp, sp_max)/cl_sp))·clamp((v − 10.5)/3.5, 0, 1): no lift in laminar flow."""
    return p.cl_max * (1.0 - math.exp(-min(sp, p.sp_max) / p.cl_sp)) * clamp((v - 10.5) / 3.5, 0.0, 1.0)


def air_acc(v, w, p: BallParams = P):
    """Gravity + drag + Magnus lift."""
    s = norm(v)
    if s == 0.0:
        return (0.0, 0.0, -p.g)
    wxv = cross(w, v)
    n = norm(wxv)
    sp = p.r * n / (s * s)
    acc = mul(v, -p.kq * drag_coef(s, sp, p) * s)
    if n > 0.0:
        acc = add(acc, mul(wxv, p.kq * lift_coef(s, sp, p) * s * s / n))
    return (acc[0], acc[1], acc[2] - p.g)


def decay(w, h: float, p: BallParams = P):
    """Exact exponential spin decay over h seconds."""
    return mul(w, math.exp(-h / p.tau_spin))


# --- ground contact ---
def e_n(vz: float) -> float:
    return clamp(0.69 - 0.012 * abs(vz), 0.45, 0.71)


def grip(v, w, jn: float, mu: float, p: BallParams = P):
    """Friction impulse at the contact point −r·ẑ. jn = normal impulse per unit mass (m/s).
    Returns (v', w', gripped). gripped means the impulse ended the slip (J_roll ≤ J_max)."""
    ux, uy = v[0] - p.r * w[1], v[1] + p.r * w[0]      # u = v_t + ω × (−r·ẑ)
    us = math.hypot(ux, uy)
    if us == 0.0:
        return v, w, True
    j_roll = p.k / (1.0 + p.k) * us
    j_max = mu * jn
    j = min(j_roll, j_max)
    jx, jy = -ux / us * j, -uy / us * j
    kr = p.k * p.r                                      # Δω = ((−r·ẑ) × J)/I = (j_y, −j_x, 0)/(k·r)
    return (v[0] + jx, v[1] + jy, v[2]), (w[0] + jy / kr, w[1] - jx / kr, w[2]), j_roll <= j_max


def bounce(v, w, p: BallParams = P, mu: float | None = None):
    """Normal restitution, then the grip rule with μ and the normal impulse (1 + e_n)·|v_z|."""
    en = e_n(v[2])
    return grip((v[0], v[1], -en * v[2]), w, (1.0 + en) * abs(v[2]), p.mu if mu is None else mu, p)


def rolling_spin(v, wz: float, p: BallParams = P):
    """ω = (ẑ × v_xy)/r, plus the remaining sidespin about z."""
    return (-v[1] / p.r, v[0] / p.r, wz)


def roll_acc(v, p: BallParams = P):
    """−a_roll·unit(v_xy) + drag. Drag uses C_D(v, roll_drag_sp): C_D0 for the default 0."""
    s = math.hypot(v[0], v[1])
    if s == 0.0:
        return (0.0, 0.0, 0.0)
    f = -p.a_roll / s - p.kq * drag_coef(s, p.roll_drag_sp, p) * s
    return (v[0] * f, v[1] * f, 0.0)


# --- sub-steps ---
def rk2_air(pos, v, w, h: float, p: BallParams = P):
    """Engine flight step: RK2 (midpoint) for position and velocity; the spin decays exactly
    (the midpoint force uses the spin at the midpoint time)."""
    vm = add(v, mul(air_acc(v, w, p), 0.5 * h))
    am = air_acc(vm, decay(w, 0.5 * h, p), p)
    return add(pos, mul(vm, h)), add(v, mul(am, h)), decay(w, h, p)


def euler_air(pos, v, w, h: float, p: BallParams = P):
    """Semi-implicit Euler flight step (forces → velocity → position). Comparison only."""
    v1 = add(v, mul(air_acc(v, w, p), h))
    return add(pos, mul(v1, h)), v1, decay(w, h, p)


def rk2_roll(pos, v, h: float, p: BallParams = P):
    """Engine rolling step: RK2 (midpoint) for position and velocity, z = r."""
    a0 = roll_acc(v, p)
    vm = (v[0] + 0.5 * h * a0[0], v[1] + 0.5 * h * a0[1], 0.0)
    am = roll_acc(vm, p)
    return (pos[0] + h * vm[0], pos[1] + h * vm[1], p.r), (v[0] + h * am[0], v[1] + h * am[1], 0.0)


def lerp(a, b, f: float):
    return tuple(x + f * (y - x) for x, y in zip(a, b))


# --- state and integrator ---
@dataclass
class Ball:
    p: tuple                      # position of the centre
    v: tuple                      # velocity
    w: tuple = (0.0, 0.0, 0.0)    # spin, rad/s
    mode: str = AIR
    t: float = 0.0
    events: list = field(default_factory=list)   # (kind, t, pos, v_in, v_out, w_out)


def launch(speed: float, angle_deg: float, spin=(0.0, 0.0, 0.0), p: BallParams = P) -> Ball:
    """Ball on the ground at the origin, launched along +x at angle_deg above the horizontal."""
    a = math.radians(angle_deg)
    return Ball((0.0, 0.0, p.r), (speed * math.cos(a), 0.0, speed * math.sin(a)), tuple(spin))


def enter_roll(b: Ball, vz_in: float, p: BallParams = P) -> None:
    """Roll entry: grip rule once with μ_slide, then rolling spin.
    vz_in is the normal speed of the contact that ends the flight (0 for a launch on the ground)."""
    jn = math.inf if p.entry_full_grip else (1.0 + e_n(vz_in)) * abs(vz_in)
    v, w, _ = grip((b.v[0], b.v[1], 0.0), b.w, jn, p.mu_slide, p)
    b.p, b.v, b.w, b.mode = (b.p[0], b.p[1], p.r), v, rolling_spin(v, w[2], p), ROLL


def _land(b: Ball, v_in, p: BallParams) -> None:
    b.v, b.w, _ = bounce(v_in, b.w, p)
    if abs(b.v[2]) < p.v_to_roll:
        enter_roll(b, v_in[2], p)
    b.events.append(("bounce", b.t, b.p, v_in, b.v, b.w))


def advance(b: Ball, h: float, p: BallParams = P, air_step=rk2_air) -> None:
    """Move the ball by h seconds: one engine sub-step (RK2 in flight and while rolling).
    A ground contact inside a flight sub-step splits it. The contact fraction f is the linear
    interpolation of the centre height between the sub-step start and end; position and velocity at the
    contact are the same interpolation of the start and end states, and the spin decays exactly to it.
    The bounce applies there, and the rest of the sub-step continues in the new state (Air or Roll)."""
    if b.mode == AIR:
        p1, v1, w1 = air_step(b.p, b.v, b.w, h, p)
        if p1[2] <= p.r and v1[2] < 0.0:
            dz = b.p[2] - p1[2]
            f = clamp((b.p[2] - p.r) / dz, 0.0, 1.0) if dz > 0.0 else 0.0
            pc, vc = lerp(b.p, p1, f), lerp(b.v, v1, f)
            b.p, b.w = (pc[0], pc[1], p.r), decay(b.w, f * h, p)
            b.t += f * h
            _land(b, vc, p)
            advance(b, (1.0 - f) * h, p, air_step)
            return
        b.p, b.v, b.w = p1, v1, w1
    elif b.mode == ROLL:
        p1, v1 = rk2_roll(b.p, b.v, h, p)
        if math.hypot(v1[0], v1[1]) < p.v_stop:
            v1, b.mode = (0.0, 0.0, 0.0), REST
            b.events.append(("stop", b.t + h, p1, b.v, v1, (0.0, 0.0, 0.0)))
        b.p, b.v = p1, v1
        b.w = rolling_spin(v1, b.w[2] * math.exp(-p.side_decay * h), p) if b.mode == ROLL else (0.0, 0.0, 0.0)
    b.t += h


@dataclass(frozen=True)
class Landing:
    x: float          # first ground contact point
    y: float
    t: float          # flight time
    apex: float       # highest centre height
    speed: float      # speed just before the contact


def flight(b: Ball, p: BallParams = P, h: float = DT, t_max: float = 30.0, air_step=rk2_air) -> Landing:
    """Engine integrator (or air_step = euler_air for the comparison) until the first ground contact."""
    apex = b.p[2]
    n0 = len(b.events)
    while len(b.events) == n0 and b.t < t_max:
        advance(b, h, p, air_step)
        apex = max(apex, b.p[2])
    _, t, pos, v_in, *_ = b.events[n0]
    return Landing(pos[0], pos[1], t, apex, norm(v_in))


def run(b: Ball, t_end: float, p: BallParams = P, h: float = DT, samples=()) -> dict:
    """Advance in fixed sub-steps to t_end. Return {t: position} at the sample times."""
    marks = {round(t / h): t for t in samples}
    out = {}
    for n in range(1, round(t_end / h) + 1):
        advance(b, h, p)
        if n in marks:
            out[marks[n]] = b.p
    return out


# --- reference integrator (flight only, same forces) ---
def _deriv(y, p: BallParams):
    v, w = y[3:6], y[6:9]
    return v + air_acc(v, w, p) + mul(w, -1.0 / p.tau_spin)


def _axpy(y, k, h):
    return tuple(a + h * b for a, b in zip(y, k))


def rk4_step(y, h, p: BallParams = P):
    k1 = _deriv(y, p)
    k2 = _deriv(_axpy(y, k1, 0.5 * h), p)
    k3 = _deriv(_axpy(y, k2, 0.5 * h), p)
    k4 = _deriv(_axpy(y, k3, h), p)
    return tuple(a + h / 6.0 * (b1 + 2.0 * b2 + 2.0 * b3 + b4) for a, b1, b2, b3, b4 in zip(y, k1, k2, k3, k4))


def flight_ref(b: Ball, step=rk4_step, h: float = DT_REF, p: BallParams = P) -> Landing:
    """Reference flight (RK4 at 1 kHz) to the first ground contact. Linear interpolation inside the last
    step (error ≈ g·h²/8 in height: 1e-6 m at 1 kHz)."""
    y, t, apex = b.p + b.v + b.w, 0.0, b.p[2]
    while True:
        y1 = step(y, h, p)
        if y1[2] <= p.r and y1[5] < 0.0:
            f = (y[2] - p.r) / (y[2] - y1[2])
            yc = tuple(a + f * (c - a) for a, c in zip(y, y1))
            return Landing(yc[0], yc[1], t + f * h, apex, norm(yc[3:6]))
        y, t = y1, t + h
        apex = max(apex, y[2])
"""Upright locomotion, and the reach times

Bodies move in the ground plane (x, y). One tick is 1/60 s. Angles are in radians.
The reach times are those of: the closed form (braking phase, above-cap branch, turn term), reach_lb
and the checked reach time.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass

DT = 1.0 / 60.0
F_DIR = ((0.0, 1.0), (45.0, 0.85), (90.0, 0.55), (135.0, 0.58), (180.0, 0.65))   # θ in degrees → speed factor
K_BACK = 0.7                  # acceleration factor at θ = 180° (1.0 up to 90°, linear between 90° and 180°)
EFFORT = {"walk": 2.0, "jog": 4.0, "run": 5.5}                                      # m/s; sprint = v_top
V_ALONG = 0.5                 # m/s: below this speed the velocity gives no "along v" direction
V_BRAKE_END = 1.0             # m/s: a target behind is braked for along v until the speed is below this
PHI_BEHIND = math.radians(120.0)   # target behind: angle between v and (q − p) above this
BETA_TURN = math.radians(120.0)    # the turn form up to this angle ...
BETA_SLOW = math.radians(150.0)    # ... and up to this one for a body at 1 m/s or less (brake-first rule)
W_TURN = 1.0                  # m/s: speed across the line to q from which the player must turn
N_CHECK = 1200                # default step limit of a checked reach time (20 s)


def x_attr(display: float) -> float:
    """Attribute form x = (attr − 10.5)/9.5"""
    return (display - 10.5) / 9.5


@dataclass(frozen=True)
class Player:
    """Capability from attribute x values in [−1, 1]
    dribble: the player controls the ball at his feet. Then v_top × (0.90 + 0.04·x_Dribbling) and the
    acceleration × 0.75 (τ / 0.75); drib = x_Dribbling."""
    pace: float = 0.0
    acc: float = 0.0
    agi: float = 0.0
    dribble: bool = False
    drib: float = 0.0

    @property
    def v_top(self) -> float:                               # top speed, m/s
        v = 9.2 + 0.5 * self.pace
        return v * (0.90 + 0.04 * self.drib) if self.dribble else v

    @property
    def tau(self) -> float:                                 # acceleration time constant, s
        t = 1.15 - 0.15 * self.acc
        return t / 0.75 if self.dribble else t

    a_brake = property(lambda s: 5.5 + 1.0 * s.agi)         # braking limit, m/s²

    def a_lat(self, v: float) -> float:
        """Lateral (normal) acceleration limit at speed v, m/s²."""
        return min(10.9 - 0.89 * v, 9.0) * (1.0 + 0.15 * self.agi)

    def turn_rate(self, v: float) -> float:
        """Facing turn-rate limit at speed v, rad/s."""
        return 8.0 * (1.0 + 0.15 * self.agi) / (1.0 + 0.25 * v)


def f_dir(theta: float) -> float:
    """Speed direction factor, linear between the table points. theta = angle between velocity and facing."""
    deg = min(abs(math.degrees(theta)), 180.0)
    for (a0, f0), (a1, f1) in zip(F_DIR, F_DIR[1:]):
        if deg <= a1:
            return f0 + (f1 - f0) * (deg - a0) / (a1 - a0)
    return F_DIR[-1][1]


def acc_factor(theta: float) -> float:
    """Acceleration direction factor: 1.0 up to θ = 90°, then linear to K_BACK = 0.7 at 180°."""
    t = min(abs(theta), math.pi)
    return 1.0 if t <= 0.5 * math.pi else 1.0 - (1.0 - K_BACK) * (t - 0.5 * math.pi) / (0.5 * math.pi)


def wrap(a: float) -> float:
    """Angle to [−π, π]."""
    return math.remainder(a, 2.0 * math.pi)


@dataclass
class Body:
    p: tuple = (0.0, 0.0)
    v: tuple = (0.0, 0.0)
    facing: float = 0.0


def body_step(b: Body, pl: Player, v_des, psi_des: float, cap: float, dt: float = DT, target=None,
              rotate_only: bool = False) -> None:
    """One 60 Hz tick of upright locomotion.

    v_des: wanted velocity; psi_des: wanted facing; cap: effort cap (m/s).
    target: the Seek/Arrive point q. It adds the turning-circle cap and the brake-first rule for a target
    behind. The wanted velocity is scaled down to v_cap.
    The part of the wanted change across v only turns v onto the direction of v_des: its size is
    min(|v|, |v_des|)·sin δ (δ = angle between v and v_des).
    rotate_only: comparison only (the earlier spec rule). The across part is |v_des|·sin δ; below |v_des|/2
    it turns v past v_des, and a heading error grows every tick.
    """
    vx, vy = b.v
    s = math.hypot(vx, vy)
    theta = abs(wrap(math.atan2(vy, vx) - b.facing)) if s > 1e-9 else 0.0
    v_cap = min(cap, pl.v_top * f_dir(theta))
    k_acc = acc_factor(theta)
    behind = False
    if target is not None and s > V_ALONG:
        ex, ey = target[0] - b.p[0], target[1] - b.p[1]
        d = math.hypot(ex, ey)
        if d > 0.0:
            sin_phi = abs(vx * ey - vy * ex) / (s * d)
            if sin_phi > 0.0:                       # turning circle through q: v²/a_lat = d/(2·sin φ)
                v_cap = min(v_cap, math.sqrt(pl.a_lat(s) * d / (2.0 * sin_phi)))
            behind = math.atan2(sin_phi, (vx * ex + vy * ey) / (s * d)) > PHI_BEHIND
    wx, wy = v_des
    wn = math.hypot(wx, wy)
    if wn > v_cap:
        wx, wy = wx * v_cap / wn, wy * v_cap / wn
    if not rotate_only and s >= V_ALONG and math.hypot(wx, wy) > s:
        k = s / math.hypot(wx, wy)                  # shrink the part of v_des across v by |v|/|v_des|
        along = (wx * vx + wy * vy) / (s * s)
        wx, wy = along * vx + k * (wx - along * vx), along * vy + k * (wy - along * vy)
    dx, dy = wx - vx, wy - vy
    if behind and s >= V_BRAKE_END:                 # target behind: brake along v first
        ax, ay = -pl.a_brake * vx / s, -pl.a_brake * vy / s
    elif s < V_ALONG:                               # no "along v" direction yet
        dn = math.hypot(dx, dy)
        if dn > 0.0:                                # so the acceleration factor uses the push direction
            k_acc = acc_factor(wrap(math.atan2(dy, dx) - b.facing))
        a = min(dn / dt, k_acc * max(v_cap - s, 0.0) / pl.tau)
        ax, ay = (dx / dn * a, dy / dn * a) if dn > 0.0 else (0.0, 0.0)
    else:
        tx, ty = vx / s, vy / s
        d_t = dx * tx + dy * ty                     # part of Δ along v
        a_t = d_t / dt
        nx, ny = (dx - d_t * tx) / dt, (dy - d_t * ty) / dt
        if d_t > 0.0:                               # speeding up (v_cap > |v| here, as |v_des| ≤ v_cap)
            a_t_max = k_acc * (v_cap - s) / pl.tau
            a_t = min(a_t, a_t_max)
        else:                                       # slowing
            a_t_max = pl.a_brake
            a_t = max(a_t, -a_t_max)
        lat = pl.a_lat(s)
        an = math.hypot(nx, ny)
        if an > lat:
            nx, ny, an = nx * lat / an, ny * lat / an, lat
        e = (a_t / a_t_max) ** 2 if a_t_max > 0.0 else 0.0
        e += (an / lat) ** 2
        if e > 1.0:                                 # one combined (elliptical) limit
            k = 1.0 / math.sqrt(e)
            a_t, nx, ny = a_t * k, nx * k, ny * k
        ax, ay = a_t * tx + nx, a_t * ty + ny
    nvx, nvy = vx + ax * dt, vy + ay * dt
    s_new = math.hypot(nvx, nvy)
    lim = max(v_cap, s - pl.a_brake * dt)           # a lower cap slows the body at a_brake, not at once
    if s_new > lim:
        nvx, nvy, s_new = nvx * lim / s_new, nvy * lim / s_new, lim
    b.p = (b.p[0] + 0.5 * (vx + nvx) * dt, b.p[1] + 0.5 * (vy + nvy) * dt)    # trapezoid
    b.v = (nvx, nvy)
    turn = pl.turn_rate(s_new) * dt
    b.facing = wrap(b.facing + max(-turn, min(turn, wrap(psi_des - b.facing))))


def _segment_hit(p0, p1, q, reach: float):
    """First fraction s in [0, 1] where the segment p0→p1 comes within `reach` of q, else None."""
    ax, ay = p0[0] - q[0], p0[1] - q[1]
    bx, by = p1[0] - p0[0], p1[1] - p0[1]
    aa, ab, bb = ax * ax + ay * ay, ax * bx + ay * by, bx * bx + by * by
    if aa <= reach * reach:
        return 0.0
    if bb == 0.0:
        return None
    sc = min(max(-ab / bb, 0.0), 1.0)               # closest approach
    if math.hypot(ax + sc * bx, ay + sc * by) > reach + 1e-9:
        return None
    disc = max(ab * ab - bb * (aa - reach * reach), 0.0)
    return min(max((-ab - math.sqrt(disc)) / bb, 0.0), sc)


def simulate_reach(pl: Player, p0, v0, facing: float, q, reach: float = 0.0, react: float = 0.0,
                   cap: float | None = None, t_max: float = 20.0, rotate_only: bool = False, noise: float = 0.0,
                   seed: int = 1):
    """Seek q at the effort cap; the body keeps its velocity and facing during the reaction delay.
    Returns (t, ticks, path): t is interpolated inside the tick (positions linear in the tick), ticks
    counts the ticks until within reach, path is the distance travelled (m).
    Conditioning checks only: rotate_only (see body_step); noise turns the velocity by a random angle
    (normal, σ = noise rad, fixed seed) after each tick, like float rounding."""
    cap = pl.v_top if cap is None else cap
    b = Body(tuple(p0), tuple(v0), facing)
    rng = random.Random(seed)
    n_react = round(react / DT)
    if _segment_hit(b.p, b.p, q, reach) is not None:
        return 0.0, 0, 0.0
    path = 0.0
    for n in range(round(t_max / DT)):
        if n < n_react:
            v_des, psi, target = b.v, b.facing, None
        else:
            ex, ey = q[0] - b.p[0], q[1] - b.p[1]
            d = math.hypot(ex, ey)
            v_des, psi, target = (cap * ex / d, cap * ey / d), math.atan2(ey, ex), q
        p_old = b.p
        body_step(b, pl, v_des, psi, cap, DT, target, rotate_only)
        if noise:
            turn = rng.gauss(0.0, noise)
            c, sn = math.cos(turn), math.sin(turn)
            b.v = (b.v[0] * c - b.v[1] * sn, b.v[0] * sn + b.v[1] * c)
        s = _segment_hit(p_old, b.p, q, reach)
        if s is not None:
            return (n + s) * DT, n + 1, path + s * math.dist(p_old, b.p)
        path += math.dist(p_old, b.p)
    return math.inf, -1, path


def sprint(pl: Player, v0: float = 0.0, marks=(5.0, 10.0, 20.0, 30.0), cap: float | None = None):
    """Straight sprint along +x from x = 0 with speed v0. Returns {mark: (t, ticks)}."""
    out = {}
    for d in marks:
        out[d] = simulate_reach(pl, (0.0, 0.0), (v0, 0.0), 0.0, (d, 0.0), 0.0, 0.0, cap)
    return out


def run_positions(pl: Player, v0: float, n: int, cap: float | None = None):
    """x position at ticks 0..n of a straight sprint along +x."""
    cap = pl.v_top if cap is None else cap
    b, xs = Body((0.0, 0.0), (v0, 0.0), 0.0), [0.0]
    for _ in range(n):
        body_step(b, pl, (cap, 0.0), 0.0, cap)
        xs.append(b.p[0])
    return xs


# ---  reach time to a point ---
def effort_cap(pl: Player, cap: float | None) -> float:
    """v_cap of an effort level: cap None = Sprint (v_top); a lower level never exceeds v_top."""
    return pl.v_top if cap is None else min(cap, pl.v_top)


def turn_term_on(pl: Player, cap: float | None) -> bool:
    """step 5 is on for Walk, Jog and Run (cap given) or while dribbling. A flag, not a float test:
    Sprint keeps steps 1–4 even when v_cap < v_top."""
    return cap is not None or pl.dribble


def run_time(pl: Player, u0: float, dist: float, v_cap: float, newton: int = 3) -> float:
    """step 4, run(u0, D): time to cover D toward q from the speed u0 ≥ 0 toward q.
    D ≤ 0 gives 0. At or below the cap: s(t) = v_cap·t − (v_cap − u0)·τ·(1 − e^(−t/τ)) = D, 3 Newton steps
    from t_q = 2D / (u0 + √(u0² + 2D·(v_cap − u0)/τ)), the positive root of u0·t + (v_cap − u0)·t²/(2τ) = D
    (no branch at u0 = v_cap). Above the cap: brake at a_brake to v_cap, then run at v_cap."""
    if dist <= 0.0:
        return 0.0
    if u0 > v_cap:
        ab = pl.a_brake
        t_d = (u0 - v_cap) / ab
        s_d = (u0 + v_cap) * t_d / 2.0
        if dist <= s_d:
            return (u0 - math.sqrt(u0 * u0 - 2.0 * ab * dist)) / ab
        return t_d + (dist - s_d) / v_cap
    tau = pl.tau
    t = 2.0 * dist / (u0 + math.sqrt(u0 * u0 + 2.0 * dist * (v_cap - u0) / tau))
    c = (v_cap - u0) * tau
    for _ in range(newton):
        e = math.exp(-t / tau)
        t -= (v_cap * t - c * (1.0 - e) - dist) / (v_cap - (v_cap - u0) * e)
    return t


def reach_time(pl: Player, p, v, q, reach: float = 0.0, react: float = 0.0, cap: float | None = None,
               newton: int = 3, turn_term: bool = False) -> float:
    """Closed-form reach time of:
    1. p_r = p + v·r.  2. d = |q − p_r| − ρ; d ≤ 0 gives r.
    3. u = v·ê; if u < 0: t_b = −u/a_brake and d_0 = d + u²/(2·a_brake); t_0 = t_b + run(max(u, 0), d_0).
    5. Turn term (C2), only when turn_term_on (Walk, Jog, Run, dribbling): the body turns its speed across ê
       onto ê. The turn form keeps all of it; t_0 becomes the mean of t_0 and min(t_0, t_1).
    6. t_reach = r + t_0.
    It is not a bound: late by up to about 0.1 s at Sprint and more at low caps; early when he must turn. turn overrides the flag (None = turn_term_on)."""
    v_cap = effort_cap(pl, cap)
    if turn_term is None:
        turn = turn_term_on(pl, cap)
    px, py = p[0] + v[0] * react, p[1] + v[1] * react           # 1. reaction
    ex, ey = q[0] - px, q[1] - py
    dist = math.hypot(ex, ey)
    d = dist - reach                                            # 2. geometry
    if d <= 0.0:
        return react
    ex, ey = ex / dist, ey / dist
    u = v[0] * ex + v[1] * ey                                   # 3. speed toward q
    if u < 0.0:
        t_b = -u / pl.a_brake                                   # stop the speed away from q first
        d0 = d + u * u / (2.0 * pl.a_brake)                     # and win back the drift
    else:
        t_b, d0 = 0.0, d
    t0 = t_b + run_time(pl, max(u, 0.0), d0, v_cap, newton)     # 4. run
    if not turn_term:
        return react + t0
    w = math.hypot(v[0] - u * ex, v[1] - u * ey)                # 5. turn term: speed across ê
    beta = math.atan2(w, u)
    s_v = math.hypot(v[0], v[1])
    t_pre = 0.0
    if BETA_TURN < beta <= BETA_SLOW and s_v >= 1.0:           # brake-first rule: along v to 1 m/s
        t_pre = (s_v - 1.0) / pl.a_brake
        x1 = (s_v * s_v - 1.0) / (2.0 * pl.a_brake)
        px, py = px + x1 * v[0] / s_v, py + x1 * v[1] / s_v
        ex, ey = q[0] - px, q[1] - py
        dist = math.hypot(ex, ey)
        d = dist - reach
        if dist > 0.0:
            ex, ey = ex / dist, ey / dist
        u = v[0] * ex + v[1] * ey
        w = math.hypot(v[0] - u * ex, v[1] - u * ey)
        beta = math.atan2(w, u)
        s_v = 1.0
    if beta <= BETA_TURN or (s_v <= 1.0 and beta <= BETA_SLOW):
        s_t = min(s_v, v_cap)
        if s_t > max(s_v * math.cos(beta), 0.0):
            om = pl.a_lat(s_t) / s_t                            # turn rate at the turning speed
            t1 = t_pre + beta / om + run_time(pl, s_t, d - s_t * math.sin(beta) / om, v_cap, newton)
            t0 = (t0 + min(t0, t1)) / 2.0
    return react + t0                                           # 6. result


def reach_lb(pl: Player, p, v, q, reach: float = 0.0, react: float = 0.0, cap: float | None = None) -> float:
    """reach_lb(q) = r + max(|q − p_r| − ρ, 0) / max(v_cap, |v|). The body never moves faster than
    max(v_cap, |v|), so this is a true lower bound of the body; it is also never above the closed form."""
    px, py = p[0] + v[0] * react, p[1] + v[1] * react
    return react + max(math.hypot(q[0] - px, q[1] - py) - reach, 0.0) / max(effort_cap(pl, cap),
                                                                            math.hypot(v[0], v[1]))


def turn_need(p, v, facing: float, q, reach: float = 0.0):
    """The numbers of the must-turn test from position p: (|q − p| − ρ, w, angle of q from the facing, u).
    w = speed across the line to q; u = speed toward q (negative: he moves away from q)."""
    ex, ey = q[0] - p[0], q[1] - p[1]
    dist = math.hypot(ex, ey)
    if dist == 0.0:
        return -reach, 0.0, 0.0, 0.0
    return (dist - reach, abs(v[0] * ey - v[1] * ex) / dist, abs(wrap(math.atan2(ey, ex) - facing)),
            (v[0] * ex + v[1] * ey) / dist)


def must_turn(p, v, facing: float, q, reach: float = 0.0, away: bool = False) -> bool:
    """must_turn from p (= p_r): d > 0 and (w ≥ 1 m/s, or q more than 90° from the facing, or, when away
    is set, u < 0). away = the turn flag (Walk, Jog, Run, dribbling): there a player who moves away
    from q (u = v·ê < 0) must reverse through the low-speed rule, which accelerates at only
    k_a·(v_cap − |v|)/τ below 0.5 m/s, far below the a_brake of the closed form's braking phase. A point inside
    the reach needs no turn."""
    d, w, ang, u = turn_need(p, v, facing, q, reach)
    return d > 0.0 and (w >= W_TURN or ang > 0.5 * math.pi or (away and u < 0.0))


def reach_time_checked(pl: Player, p, v, q, reach: float = 0.0, react: float = 0.0, cap: float | None = None,
                       facing: float | None = None, n_max: int = N_CHECK, turn: bool | None = None,
                       w_stop: float = W_TURN, away: bool | None = None):
    """Checked reach time. While he must turn (must_turn without the reach: the speed across the line
    to q is at least w_stop, or q is more than 90° from the facing, or, with the turn flag, he moves away from q),
    run the body model (Seek toward q at the effort cap, after the reaction delay). Then add the closed form (with
    the same turn flag) from the state at that tick.
    Returns (t, ticks, done). A check that reaches n_max steps stops: (inf, n_max, False), not feasible.
    Tick times are ticks / 60.0. facing defaults to the direction of v (0 at rest). turn overrides the turn flag
    (None = turn_term_on). away: the 'moves away' part of the stop test (None = the turn flag; False = the stop test
    without the 'moves away' part, an earlier rule, comparison only)."""
    cap_v = pl.v_top if cap is None else cap
    if away is None:
        away = turn_term_on(pl, cap) if turn is None else turn
    if facing is None:
        facing = math.atan2(v[1], v[0]) if any(v) else 0.0
    b = Body((p[0] + v[0] * react, p[1] + v[1] * react), tuple(v), facing)
    if math.hypot(q[0] - b.p[0], q[1] - b.p[1]) <= reach:
        return react, 0, True
    n = 0
    while True:
        ex, ey = q[0] - b.p[0], q[1] - b.p[1]
        d = math.hypot(ex, ey)
        if abs(b.v[0] * ey - b.v[1] * ex) / d < w_stop and abs(wrap(math.atan2(ey, ex) - b.facing)) <= 0.5 * math.pi \
                and not (away and (b.v[0] * ex + b.v[1] * ey) / d < 0.0):
            return react + n / 60.0 + reach_time(pl, b.p, b.v, q, reach, 0.0, cap, turn_term=True), n, True
        if n >= n_max:
            return math.inf, n, False
        p_old = b.p
        body_step(b, pl, (cap_v * ex / d, cap_v * ey / d), math.atan2(ey, ex), cap_v, DT, q)
        n += 1
        hit = _segment_hit(p_old, b.p, q, reach)
        if hit is not None:
            return react + (n - 1 + hit) / 60.0, n, True


def arrival_velocity(pl: Player, p, v, q, reach: float = 0.0, react: float = 0.0, cap: float | None = None):
    """t_control: the closed form's arrival velocity, along ê. Speed v_cap − (v_cap − u')·e^(−t_run/τ),
    or max(v_cap, u' − a_brake·t_run) above the cap; t_run = run(u', d_0) of step 4 (no turn term)."""
    v_cap = effort_cap(pl, cap)
    px, py = p[0] + v[0] * react, p[1] + v[1] * react
    ex, ey = q[0] - px, q[1] - py
    dist = math.hypot(ex, ey)
    if dist == 0.0:
        return 0.0, 0.0
    ex, ey = ex / dist, ey / dist
    u = v[0] * ex + v[1] * ey
    d0 = dist - reach + (u * u / (2.0 * pl.a_brake) if u < 0.0 else 0.0)
    u1 = max(u, 0.0)
    t_run = run_time(pl, u1, d0, v_cap)
    if u1 <= v_cap:
        s = v_cap - (v_cap - u1) * math.exp(-t_run / pl.tau)
    else:
        s = max(v_cap, u1 - pl.a_brake * t_run)
    return s * ex, s * ey


def dive_reach(t: float, x_agi: float = 0.0, v_side: float | None = None, hands: float = 1.4,
               t_set: float = 0.10) -> float:
    """Keeper dive: lateral reach t seconds after onset. Hands reach 1.4 m beyond the
    body along the dive; push-off 2.7 + 0.3·x m/s sideways after a 0.10 s set.
    Upper bound: the body keeps its sideways push-off speed (the slide after landing has no stated loss)."""
    v = 2.7 + 0.3 * x_agi if v_side is None else v_side
    return hands + v * max(0.0, t - t_set)


def dive_time(dist: float, x_agi: float = 0.0, hands: float = 1.4, t_set: float = 0.10) -> float:
    """Inverse of dive_reach: the earliest time after onset at which the dive reaches dist (0 inside the hands)."""
    if dist <= hands:
        return 0.0
    return t_set + (dist - hands) / (2.7 + 0.3 * x_agi)

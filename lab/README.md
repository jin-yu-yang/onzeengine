# Lab

The lab is plain Python. It implements a few models of the spec

- the ball (flight, bounce, roll, states),
- upright locomotion and the reach-time closed form,
- the touch error model.

## Run

Python 3.11 or later, standard library only. Run from `lab/`:

```sh
python -m onzeengine.report > tables.md         # print the tables, write golden/*.csv
python -m onzeengine.report --no-golden          # tables only
python -m unittest discover -s tests -v      # checks
```

The report takes about 4 s. The Monte Carlo uses `random.Random` with fixed seeds, so every run gives the same numbers.

`REPORT.md` = the findings, then the output of `python -m onzeengine.report`. When a constant changes, run the report again, paste the new tables, and check the findings.

## Layout

| File | Content |
|---|---|
| `onzeengine/ball.py` | `BallParams` (spec constants), forces, bounce, roll, the engine integrator (semi-implicit Euler, 240 Hz), RK2 and RK4 references |
| `onzeengine/body.py` | `Player` capability, `body_step` (one 60 Hz tick), `simulate_reach`, `sprint`, `reach_time` (closed form), `dive_reach` |
| `onzeengine/execution.py` | difficulty terms, `sigmas`, `p_mishit`, `realize_kick` (sampling), `p_exec` and its Monte Carlo |
| `onzeengine/report.py` | all tables and the golden files |
| `golden/*.csv` | reference numbers for the C++ tests |
| `tests/test_lab.py` | unittest checks |

## Conventions

- SI units. World frame: x along the pitch, y across, z up. Positions are of the ball centre.
- A kick starts at the origin with the centre at z = r = 0.11 m, along +x, at `angle_deg` above the horizontal.
- Spin in rad/s. Backspin of a ball moving along +x is about −y. Sidespin is about +z and bends the ball to +y. Topspin is about +y.
- Bodies move in the (x, y) plane; facing in radians (degrees in the CSV files).
- Attribute values are `x = (attr − 10.5)/9.5`.

## Lab readings of the spec

Where the spec is open, the lab chose one reading. REPORT.md marks each one with "Spec issue:".

- C_D0 uses the two-regime fallback. It is piecewise linear with knots at 11 and 15 m/s, so a 0.25 m/s table gives the same values. The Brazuca fit is only for comparison.
- Other curves use their analytic forms (C_L, f_dir, e_n). `f_dir` is linear between its five points.
- A ground contact inside a sub-step: the semi-implicit Euler path is a straight segment, so the contact time is a linear interpolation on it. The bounce applies at that time. The rest of the sub-step continues in the new state.
- Rolling drag uses C_D0 only (Sp = 0).
- Roll entry applies the grip rule with μ_slide and the normal impulse of the contact that ends the flight. A launch on the ground has no normal impulse, so it rolls on at its launch speed.
- Seek: each tick `v_des = v_cap·unit(q − p)` and `ψ_des` = direction of q. During the reaction delay the body keeps its velocity and facing.
- Arrival in the body simulation: the first time the body axis comes within `reach` of q. Positions are linear inside a tick.

## Tables (REPORT.md)

| § | Table | What it shows |
|---|---|---|
| 1 | Ball flight | carry, lateral bend, flight time, apex and landing speed for 5 speeds × 4 angles × 3 spins; calibration checks; a sweep of the spin-drag slope |
| 2 | Integrator accuracy | landing-point error of Euler (240 Hz) and RK2 (240 Hz) against RK4 (1 kHz) |
| 3 | Roll-out | time to 10/20/30 m, stop point, mean speed over 20 m; launch speeds for the StatsBomb mean speeds; a_roll and reading variants |
| 4 | Bounce | 2 m drop test; 45° bounces with and without spin; FIFA angle test |
| 5 | Sprints | time to 5/10/20/30 m by Pace and Acceleration; FM27 table; leads in metres |
| 6 | Reach time | closed form against the 60 Hz simulation; error summary |
| 7 | Pass error | lateral σ by distance, mishit table, P_exec closed form against Monte Carlo |
| 8 | Keeper dive | closed-form reach against the 3.8 m target |

## Golden files

All files have one header row. Numbers have six decimals. `nan` means "not reached". `case` is a label.

| File | Inputs | Outputs |
|---|---|---|
| `ball_flights.csv` | `speed`, `angle_deg`, `z0` (start height of the centre), spin `wx wy wz` (rad/s); state Air at t = 0 | `land_x land_y flight_t` (first ground contact), `apex_z` (highest centre before it), `land_speed` (just before it); centre position `x_t y_t z_t` at t = 0.5, 1.0, 2.0 s (after 120, 240, 480 sub-steps, with bounces and rolling) |
| `rollouts.csv` | `v0`: centre at (0, 0, r), velocity (v0, 0, 0), no spin, then Roll entry | `t_10 t_20 t_30` (x crosses the mark, linear inside the sub-step), `stop_x stop_t` (first sub-step with speed below 0.05 m/s), `mean_speed_20` = 20 / t_20 |
| `sprints.csv` | `pace_x acc_x`, `v0` along +x; each tick `v_des` = (v_top, 0), `ψ_des` = 0, sprint effort | `t_5 … t_30` (crossing time, linear inside the tick), `ticks_5 … ticks_30` (ticks until x ≥ mark) |
| `reach_times.csv` | `pace_x acc_x agi_x`, start at the origin with `v0x v0y` and `facing_deg`, target `qx qy`, `reach` (m), `react` (s), `v_cap` (effort) | `t_sim ticks_sim` (Seek simulation), `t_closed` (09 §5.1 as written), `t_proposed` (lab proposal), `path_sim` (m travelled) |

### How the C++ tests use them

1. Read a row. Build the start state from the input columns.
2. Run the engine code: the ball integrator at 240 Hz, `body_step` at 60 Hz with the Seek command above, or `reach_time`.
3. Compare each output column with a tolerance.

Suggested tolerances for a float build with table lookups:

| Quantity | Tolerance |
|---|---|
| landing point, positions at t | 0.02 m (0.05 m for positions after a bounce) |
| flight time, crossing times | 0.002 s |
| apex | 0.01 m; landing speed 0.01 m/s |
| stop point | 0.05 m; stop time 1 sub-step |
| tick counts | ± 1 tick |
| `t_closed` | 0.001 s |

Two rows need care:

- `reversal 180°` rows start exactly on the x axis. The body then brakes in a straight line. Any rounding that moves y off zero turns the reversal into a U-turn with a very different time (REPORT.md, spec issues). Keep exact zeros in the test setup.
- Some `turn` rows at 6 m/s spiral into the target (`path_sim` 1.3–1.8 × the distance). This is the spec's behaviour and it is well conditioned, but these rows will change when the Seek issue in REPORT.md is fixed.

The golden files follow the lab's reading of the spec. When the spec changes, run the report again and review the diff of `golden/` like code.
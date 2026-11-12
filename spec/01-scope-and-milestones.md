# 01 · Scope and plan

## 1. MVP in brief

**In:**

- The core: a deterministic, headless C++20 library. It plays a full 11 v 11 match with a 3D ball and 2.5D players. Attributes set how well players execute. A readable team and player AI makes the decisions. The core applies offside, fouls, cards and basic restarts.
- The CLI: it runs, verifies and measures matches.
- The UE 5.8 viewer: it shows live matches and replays, with a decision overlay.

**Out:** management, substitutions, injuries, extra time, full set-piece routines, perception noise beyond reaction delay, deep search and RL training (both keep their interfaces).

## 2. In scope

| Area | MVP content | Spec |
|---|---|---|
| Data and loop | `WorldState`, 60 Hz ticks, 10 Hz brains, inputs, facade and `Env` | [02](02-data-model.md), [03](03-simulation-loop.md) |
| Physics | ball: gravity, drag crisis, Magnus, spin decay, bounce, roll, posts, crossbar, net; bodies: speed, acceleration, turning, jump, dive, slide, fall; collisions | [10](10-physics.md) |
| Ball control | touch-based control, reach volumes, trajectory cache, interception solver, possession | [09](09-ball-control.md) |
| Actions | `Action` (locomotion, body, ball), helpers, rule checks | [07](07-action-contract.md) |
| Execution | set-up and recovery, difficulty, capability, Gaussian errors, mishit branch, fatigue | [08](08-execution.md) |
| Perception | exact, plus reaction delay to ball epochs and lapses | [05](05-perception.md) |
| Decision | team modes and plans; on-ball option table; pass candidates by field of view and scans; V; off-ball option table; noise; commitment; keeper; restart templates | [06](06-decision.md) |
| Conflicts | contests, collisions, balance, foul candidates, referee ordering | [04](04-conflict-resolution.md) |
| Referee | phases, clock and added time, all restarts, goals, out of play, offside, fouls, advantage, cards, penalties | [11](11-referee.md) |
| Determinism | keyed RNG, hashes, fixed FP flags, tests | [12](12-determinism.md) |
| Output | events, snapshots, traces, replay file, SPADL and stats exports | [13](13-events-snapshots-replay.md) |
| UE viewer | prebuilt library, live mode with run-ahead, interpolation, blend-space locomotion, kick and header montages, overlay, replay timeline | [14](14-unreal-integration.md) |
| Config | TOML with `extends`, parameter registry, config hash | [15](15-configuration.md) |
| Tests | unit, golden, determinism, scenarios, calibration, balance | [16](16-testing-and-tuning.md) |

## 3. Out of scope (MVP)

- Season, transfers, club management, training.
- Substitutions, injuries, extra time, penalty shoot-outs, VAR, handball.
- Full set pieces: routines, rehearsed moves, complex wall and marking schemes.
- Perception noise, belief states, communication.
- Field of view and scans as a perception filter. In the MVP, they act only in the decision layer: the memory of seen team-mates.
- Search deeper than one step; RL training (the `Env` API exists).
- Weather, pitch condition, crowd, morale, personality.
- A user-facing explanation UI (the developer overlay is in).
- Cross-platform floating-point determinism.

## 4. Milestones

Each milestone ends with a runnable state.

| Milestone | Content | Exit check |
|---|---|---|
| **M0 Skeleton** | repo, CMake, core types, keyed RNG, `WorldState` with visitor and hash, ball physics, CLI that runs ball-only scenarios, replay header | golden ball flights match the lab within tolerance; twice-run hashes equal |
| **M1 Bodies and touches** | locomotion, energy v1, postures, reach volumes with the surface stretch ([09](09-ball-control.md) §3), contact search, the touch rule (a)–(d) ([09](09-ball-control.md) §4.1), `Touch` without error, control and possession, trajectory cache, interception solver with `first_touch` ([09](09-ball-control.md) §5.1–§5.3), `plan_contact` ([09](09-ball-control.md) §5.5), out of play, goals; kick-off, throw-in (release point, [10](10-physics.md) §2.5), goal kick, corner: the referee's default taker takes ([11](11-referee.md) §3), and the others stand at their kick-off layout spots ([06](06-decision.md) §11); a trivial brain (chase and kick at goal) that plans its touches with `plan_contact` | a full match runs end to end with restarts, and no kick-off is retaken because of the layout; the golden reach-time, reach-sweep, intercept and `first_touch` tests pass (foot rows, circular reach; [16](16-testing-and-tuning.md) §1); the planned-contact scenarios pass their stretch and touch-trigger checks ([16](16-testing-and-tuning.md) §2) |
| **M2 Brains** | team brain (modes, block, slots, assignments), on-ball options with two-stage evaluation ([06](06-decision.md) §4.5), V table, off-ball option table with the duel table (s_tk = 1, P_foul = 0; [06](06-decision.md) §6.4), the Block row ([06](06-decision.md) §6.1) and the contact-part rule ([06](06-decision.md) §1.2), keeper basics, set-piece modes (the taker and Shape at every restart, the kick-off template; [06](06-decision.md) §11), decision noise, commitment, decision traces | scenario set: open pass, counter 3 v 2, counter-press after loss; the count test passes on the Count match ([03](03-simulation-loop.md) §7, [16](16-testing-and-tuning.md) §2), with its caps reset to 1.15 × the first full build; the stage-agreement test passes ([06](06-decision.md) §4.5); the modelled cost and the timed full match are reported, with no time gate |
| **M3 Execution and rules** | error model with mishit, set-up and recovery, fatigue, contests, collisions, balance, fouls, advantage, cards, offside, free kicks, tackle success and P_foul in the duel table ([06](06-decision.md) §6.4), restart templates, wall, penalty, delivery rule and second phase ([06](06-decision.md) §11), events, replay file, SPADL export | scenario set passes; offside and restart tables pass; the planned-contact scenarios pass their d_stretch, tackle-success and offside checks ([16](16-testing-and-tuning.md) §2) |
| **M4 UE viewer** | plugin with the prebuilt library, live mode, render delay, interpolation, blend-space locomotion, kick and header montages with IK, overlay, replay timeline | a live match at 60 fps with the overlay; a CLI replay plays in UE |
| **M5 Calibrate** | matchlab backend, calibration runs, first attribute sweep, performance work (engineering first, then the reserves R3 and R4; [03](03-simulation-loop.md) §7) | calibration bands (§6, item 4); the first balance report; a timed full match ≤ 90 s (§6, item 1), after the [03](03-simulation-loop.md) §7 remedies in order |

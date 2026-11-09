# OnzeEngine

A physics-driven 11v11 football (soccer) match engine.

> **Status: pre-alpha.**
> OnzeEngine currently exists as a detailed specification paired with a Python verification lab used to validate the physics, mathematics, and tick-cost models. The C++20 engine is not built yet—implementation begins at Milestone M0. APIs, file formats, and tunable parameters are subject to change without notice.

## Overview
Most football simulators are either arcade-like, heavily abstracted, probability-driven, or closed-source. **OnzeEngine** simulates a full 11v11 match with physics-based models fitted to public measurements and with bounded human decisions, not outcome tables, while staying fast and explainable.

### What Makes OnzeEngine Different
* **Physics and Decisions, Not Dice Rolls:** A pass succeeds because the ball physically arrived before an opponent could intercept it. The ball flies in 3D with aerodynamic drag, Magnus lift, spin, bounce, and roll curves. Players are 2.5D bodies governed by realistic speed, acceleration, and turn-rate limits.
* **Possession is Earned by Touch:** There is no "magnet" ball. A player controls the ball only when it enters their physical reach. Every touch introduces execution error scaled by player skill, incoming ball velocity, body orientation, defensive pressure, and preparation time (e.g., stepping around the ball and opening the body vs. rushing an awkward kick).
* **Strict Brain–Body Separation:** Brains run up to 10 Hz and perceive the pitch through an `Observation` (incorporating field-of-view limits and human reaction delays) and act solely by emitting an `Action`. This clean boundary makes it straightforward to plug in scripted tactics, external controllers, or reinforcement learning (RL) agents.
* **Bit-for-Bit Determinism:** Given the same build, configuration, seed and inputs, OnzeEngine produces exactly the same match, bit for bit, whatever the trace level or thread count. Randomness is restricted to human execution and perception errors, making replays, regression testing, and edge-case debugging exact.
* **Debug Visibility First:** Every tactical and physical decision is traceable. You can inspect why a player chose a specific pass, how they evaluated its safety, and what outcome they expected at the moment of the kick.

## Technical Architecture

* **Core Engine:** C++20 core library, headless CLI runner, single match per process.
* **Simulation Rates:** Fixed 60 Hz match physics, 240 Hz RK2 sub-stepping for 3D ball integration, and 10 Hz decision ticks.
* **Referee & Rules:** Rule-based referee enforcing the Laws of the Game, handling fouls, advantage, and restarts.
* **Viewer Decoupling:** The engine outputs lightweight snapshots, event streams, and replay files. An Unreal Engine viewer renders outputs only.

## Milestones

| Milestone | Content |
|---|---|
| M0 | core skeleton, deterministic math (`onze::dmath`), ball physics, roll curve and kick tables, `solve_kick`; golden ball-flight and kick-table tests |
| M1 | players, reach and intercept solver, touches, simple brains, basic restarts |
| M2 | team and player brains, cost-count and stage-agreement tests |
| M3 | execution errors, duels, referee, set pieces |
| M4 | replay and the UE viewer |
| M5 | calibration and performance |

## Inspiration & Sources

OnzeEngine is built on public knowledge and its own tunable parameters.

* **Research & Simulators:** Inspired by open research platforms including the RoboCup 2D Soccer Simulation Server, the HELIOS base team, and Google Research Football.
* **Football Analytics & Science:** Grounded in the IFAB Laws of the Game, public event and tracking datasets (such as StatsBomb open data), sports-science literature on biomechanics, and modern spatial analytics (pitch control, Expected Threat / xT, and xG). Any parameter lacking direct empirical backing is explicitly marked as a tunable baseline.

## License

BSD Zero Clause License (0BSD). See [LICENSE](LICENSE).

# 02 · Data model

The core keeps all simulation state in one struct, `WorldState`.
Static inputs live in `MatchConfig`.
Derived caches live outside both and are rebuilt from them.

## 1. Rules

1. `WorldState` is trivially copyable. A keyframe is one copy of it.
   `static_assert(std::is_trivially_copyable_v<WorldState>)`.
2. No pointers, references, `std::vector`, `std::string`, virtual functions or heap memory in `WorldState`.
3. Objects refer to each other by index: `PlayerId` 0–21, `TeamId` 0–1. `kNone = 0xFF`.
4. Players 0–10 are the home team and 11–21 the away team. `team = id / 11`, `slot = id % 11`. The slot index is the player's place in the `[[players]]` list of his team file: the first player is slot 0. Each slot name (`slot = "GK"`) appears once per team. Formation tables and set-piece takers name a slot by this name, so their key order does not matter.
5. Units are SI: metres, seconds, radians, kilograms. Angles are counter-clockwise from +x.
6. Time is `Tick` (`uint32_t`, 60 per second). Floats never hold absolute time. A sub-tick time is a float in [0, 1).
7. Only `float` in state. Search and prediction code may use `double` locally.
8. One visitor function lists every field of `WorldState` in a fixed order: `visit(WorldState&, V&)`. Hashing, keyframe writing, snapshot projection and field-level diffs all use it. Padding bytes never enter a hash.
9. Every enum has an explicit `uint8_t` base and stable numeric values. Replays store these values.

## 2. Frames

- **World frame.** Origin at the centre spot. x along the length, y across, z up, right-handed.
  The default pitch spans x ∈ [−52.5, 52.5] m and y ∈ [−34, 34] m.
  This is the FIFA standard size of 105 × 68 m.
  The home team attacks +x in period 1.
- **Team frame.** The frame in which a team attacks +x.
  For a team that attacks −x, the team frame is the world frame turned 180° about z: `(x, y) → (−x, −y)`, `yaw → yaw + π`.
  It is a rotation, not a mirror, so left and right stay the same. Wings and feet keep their meaning.
- Perception writes observations in the team frame. Execution turns actions back into the world frame. Brains never see the world frame.

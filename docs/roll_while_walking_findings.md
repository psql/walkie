# Roll-While-Walking: Phase 0 findings and outcome

Date: 2026-06-18. SDK: bosdyn 5.1.4. Robot: spot-BD-61330023.

## TL;DR

**Path A.** The realtime Custom Gait command natively carries a live body-rotation
axis, so roll-while-walking needs no re-staging. It is **already implemented** in
`server/custom_gait.py` (`CustomGaitWalker.steer`), which sends the slider
pitch/roll/yaw/height through the realtime command on every ~20 Hz tick.

The one blocker is licensing, not code: this robot has **no choreography license**,
so Custom Gait cannot run at all and the walk path falls back to the mobility
backend (pitch holds, roll does not). Roll-while-walking starts working with **zero
code changes** the moment a choreography license is installed. See
[[project-roll-debug]].

## Phase 0, Probe 1: does the live command support body pose?

```python
from bosdyn.api.spot import choreography_params_pb2 as cp
list(cp.CustomGaitCommand.DESCRIPTOR.fields_by_name.keys())
```

Result (bosdyn 5.1.4):

```
['drive_velocity_body', 'finished', 'body_translation_offset', 'body_orientation_offset']
```

- `body_orientation_offset` is an `EulerZYX` (roll, pitch, yaw) — a **live** body
  rotation axis on the realtime command.
- `body_translation_offset` is a `Vec3` (live height via z).
- `CustomGaitCommandLimits` also reports `maximum_body_orientation_offset` and
  `maximum_body_translation_offset`, so the live offset has reported limits too.

**Conclusion: Probe 1 finds a body field, so the path is A.** This spec's framing
("the live command channel carries translation, turning, and stop only; there is no
live body-rotation axis") was based on the Choreographer Xbox path / older docs and
is **not true for the installed SDK**. Live roll is directly commandable.

## Phase 0, Probe 2: re-stage transition quality

**Not run, and not needed.** Probe 2 only matters for Path B (re-stage on roll
change). Path A makes re-staging unnecessary. (It also could not be run here: Probe 2
requires Custom Gait to execute, which requires the choreography license this robot
lacks.)

## Path A implementation (already present)

`CustomGaitWalker.steer(vx, vy, v_rot, pitch, roll, yaw, height)` builds a
`CustomGaitCommand` every command-loop tick and sets:

```python
gait_cmd.drive_velocity_body.linear.x = vx      # drive (live)
gait_cmd.drive_velocity_body.linear.y = vy
gait_cmd.drive_velocity_body.angular  = v_rot
gait_cmd.body_orientation_offset.roll  = roll   # <-- live roll (Path A)
gait_cmd.body_orientation_offset.pitch = pitch
gait_cmd.body_orientation_offset.yaw   = yaw
gait_cmd.body_translation_offset.z     = height
```

Then sends it via `choreography_command([MoveCommand(custom_gait_command=...)], ...)`.
Velocities and offsets are clamped to the live `CustomGaitCommandLimits` first, then
to the gait caps. The slider value reaches this method through:

`roll slider -> WS control msg -> SpotController.update(roll=) -> state.roll ->
_send_command (walking branch) -> CustomGaitWalker.steer(roll=)`.

So roll is a continuous live axis alongside live pitch and drive: the puppeteering
feel the spec asks for, with no re-stage, no gait restart, driving uninterrupted.

## Acceptance criteria mapping

1. Roll while walking, driving unaffected — yes (live offset on each steer tick;
   driving uses `drive_velocity_body` on the same command).
2. Roll tracks the slider across its range — yes, **up to the clamp**. Note: the
   roll slider is +/-17 deg but the gait fallback clamp `MAX_ROLL_GAIT` is +/-11 deg
   (and the robot's live `maximum_body_orientation_offset.roll` caps it further).
   For 1:1 tracking across the full slider on hardware, raise `MAX_ROLL_GAIT` toward
   the slider range and let the live limit protect stability. Kept conservative here
   per the gait spec ("start conservative, tune up on hardware").
3. Pitch + roll together while walking — yes, both ride `body_orientation_offset`.
4. Diagnostics: commanded vs actual roll within a few degrees — `_log_pose_diagnostic`
   already logs commanded vs actual roll and warns on ROLL DRIFT during walk; with
   Path A running, that warning should not fire.
5. Safety unchanged — yes; lease/E-Stop/keepalive and the one-paradigm-at-a-time
   command loop are untouched; the steer path only sends realtime commands while the
   gait is running.

## What is left to do

Nothing in code for Path A. To exercise and tune on hardware:

1. Install a choreography license on the robot (in progress with BD support).
2. With the license present, `bring_up_control` keeps `WALK_BACKEND="custom_gait"`
   instead of falling back to mobility, and walking uses `CustomGaitWalker`.
3. Tune `MAX_ROLL_GAIT` / `cycle_duration` / velocity caps to taste; confirm roll
   tracks the slider with no stumble (acceptance 2 and 4).

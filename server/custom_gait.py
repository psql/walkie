"""
Custom Gait walker: drives Spot with a Choreography Custom Gait so that a
commanded body pitch AND roll are held through the step cycle.

Why this exists: the standard mobility velocity path (synchro_velocity_command +
BodyControlParams) honors a pitch/height offset while stepping but reserves roll
and yaw for the dynamic balancer, so a commanded roll gets washed out. Custom
Gait is the documented paradigm for a posed, steerable walk that keeps a body
rotation offset through the gait.

Reference: https://dev.bostondynamics.com/docs/concepts/choreography/custom_gait.html

API surface pinned against the installed bosdyn-choreography-client 5.1.4 (the
docs name parameters, not python method/field names, so every name below was
confirmed by introspecting the installed SDK):
  - ChoreographyClient.license_name == "choreography"; not auto-registered in the
    standard SDK, so register_service_client(ChoreographyClient) is required.
  - Build:   choreography_sequence_pb2.ChoreographySequence with one MoveParams
             (type="custom_gait", custom_gait_params=CustomGaitParams).
  - Upload:  ChoreographyClient.upload_choreography(seq).
  - Execute: ChoreographyClient.execute_choreography(name, client_start_time,
             choreography_starting_slice).
  - Realtime (the "ChoreographyCommand RPC" in the docs):
             ChoreographyClient.choreography_command([MoveCommand], client_end_time).
             MoveCommand.custom_gait_command (CustomGaitCommand) carries
             drive_velocity_body (SE2Velocity), body_orientation_offset (EulerZYX),
             body_translation_offset (Vec3) and finished (bool) LIVE every tick.
  - Status:  ChoreographyClient.get_choreography_status() -> (status, client_time);
             status.active_moves[].custom_gait_command_limits is a
             CustomGaitCommandLimits with maximum_drive_velocity_body /
             maximum_body_orientation_offset / maximum_body_translation_offset.

Spec field-name mappings worth noting:
  - "body_rotation_offset" in the spec is the params-side field
    CustomGaitParams.body_rotation_offset (EulerZYXValue). For the LIVE pose we use
    the realtime CustomGaitCommand.body_orientation_offset (EulerZYX) instead, so
    the params offset is baked to zero to avoid additive double-counting.
  - Height maps to the realtime CustomGaitCommand.body_translation_offset.z (Vec3),
    not com_height; com_height is left at its gait default.
"""

import logging
import time

from bosdyn.api.spot import choreography_sequence_pb2 as cs
from bosdyn.api.spot import choreography_params_pb2 as cp
from bosdyn.choreography.client.choreography import ChoreographyClient
from bosdyn.client.license import LicenseClient
from bosdyn.client.robot_command import blocking_stand

logger = logging.getLogger(__name__)

# Body-offset clamps while stepping. Tighter than the mobility-mode clamps because
# offsets carried through a gait have their own stability limits. Tune up on hardware.
MAX_PITCH_GAIT = 0.25   # rad (~14 deg)
MAX_ROLL_GAIT = 0.20    # rad (~11 deg)
MAX_YAW_GAIT = 0.20     # rad (~11 deg)

# Gait pattern. The python SDK ships no programmatic "gait preset" loader (presets
# live in the Choreographer GUI), so the swing phases below hand-author a stable
# high duty-factor lateral-sequence amble that mimics a walk preset: each leg swings
# for ~0.4 of the cycle, sequenced a quarter-cycle apart, so >=2 feet are always
# down. liftoff/touchdown are cycle fractions; the last leg's touchdown wraps past
# 1.0 which the controller interprets modulo the cycle. Tune on hardware.
GAIT_CYCLE_DURATION = 0.8      # s; faster stepping is easier to stabilize, slow to taste
GAIT_SWING_FRACTION = 0.40     # swing duration as a fraction of the cycle (duty factor 0.60)
SWING_PHASE_OFFSETS = {        # liftoff phase per leg, a quarter cycle apart
    "fl_swing": 0.00,
    "hr_swing": 0.25,
    "fr_swing": 0.50,
    "hl_swing": 0.75,
}

GAIT_MAX_VELOCITY = 0.5        # m/s; conservative during bring-up
GAIT_MAX_YAW_RATE = 0.6        # rad/s; conservative during bring-up
GAIT_ACCEL_SCALING = 0.5       # <1 = gentler accel during bring-up

SEQUENCE_NAME = "walkie_custom_gait"
MOVE_ID = 1                    # links the uploaded move to the realtime MoveCommand
SLICES_PER_MINUTE = 600        # standard choreography resolution (10 slices/s)
# The gait is stopped on demand via the realtime "finished" flag, so the move is
# made long enough to never expire during an activation.
GAIT_RUN_SECONDS = 1800
COMMAND_VALIDITY_S = 0.5       # how long each realtime command stays valid
START_DELAY_S = 0.25           # client_start_time lead so the robot has the command in hand
STOP_SETTLE_S = 1.5            # time for standard_final_stance to complete after finished
LIMITS_REFRESH_S = 0.5         # throttle the status/limits poll (steering ticks ~20 Hz)


def _clamp(value: float, limit: float) -> float:
    """Clamp value to the symmetric range [-limit, limit]."""
    return max(-limit, min(limit, value))


class CustomGaitWalker:
    """Owns the Custom Gait lifecycle: preflight, start, steer, pose, stop.

    All public methods are called from the single command-loop thread, so no
    internal locking is needed. The only shared field is is_running, read by the
    loop to decide whether a transition is needed.
    """

    def __init__(self, robot, command_client):
        self._robot = robot
        self._command_client = command_client
        self._choreo = None
        self._running = False
        self._limits = None          # cached CustomGaitCommandLimits or None
        self._limits_logged = False
        self._last_limits_poll = 0.0

    # ------------------------------------------------------------------
    # Setup / preflight
    # ------------------------------------------------------------------

    def setup(self, sdk):
        """Register + create the choreography client and fail fast on a missing license.

        Raises RuntimeError if the choreography license is absent, which is the
        documented preflight for Custom Gait.
        """
        # The choreography service is not part of the standard SDK registration.
        sdk.register_service_client(ChoreographyClient)

        license_client = self._robot.ensure_client(LicenseClient.default_service_name)
        feature = ChoreographyClient.license_name   # == "choreography"
        enabled = license_client.get_feature_enabled([feature])
        if not enabled.get(feature, False):
            raise RuntimeError(
                "Choreography license is NOT enabled on this robot. Custom Gait walk "
                "backend is unavailable. Install a choreography license or set "
                "WALK_BACKEND='mobility'."
            )

        # ensure_client -> update_from(robot) wires the timesync endpoint the
        # choreography client needs to stamp client_start_time / client_end_time.
        self._choreo = self._robot.ensure_client(ChoreographyClient.default_service_name)
        logger.info("Custom Gait preflight OK: choreography license enabled, client ready")

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def limits(self):
        return self._limits

    # ------------------------------------------------------------------
    # Sequence construction
    # ------------------------------------------------------------------

    def _build_params(self) -> cp.CustomGaitParams:
        """Build the CustomGaitParams. The body rotation offset is baked to ZERO;
        pitch/roll/yaw and height are driven live via the realtime command instead,
        so there is no additive double-counting between params and realtime."""
        params = cp.CustomGaitParams()
        params.max_velocity.x.value = GAIT_MAX_VELOCITY
        params.max_velocity.y.value = GAIT_MAX_VELOCITY
        params.max_yaw_rate.value = GAIT_MAX_YAW_RATE
        params.acceleration_scaling.value = GAIT_ACCEL_SCALING
        params.cycle_duration.value = GAIT_CYCLE_DURATION

        for leg, liftoff in SWING_PHASE_OFFSETS.items():
            swing = getattr(params, leg)
            swing.liftoff_phase.value = liftoff
            swing.touchdown_phase.value = liftoff + GAIT_SWING_FRACTION

        # Hold a posed stand when stopped instead of marching in place; return to a
        # rectangular stance when the gait ends.
        params.stand_in_place.value = True
        params.standard_final_stance.value = True

        # Flat activation floor: disable perception so step placement is predictable.
        params.enable_perception_terrain_height.value = False
        params.enable_perception_step_placement.value = False
        # obstacle avoidance left at its default (off).

        # body_rotation_offset / body_translation_offset deliberately left at zero;
        # driven live via the realtime CustomGaitCommand.
        return params

    def _build_sequence(self) -> cs.ChoreographySequence:
        requested_slices = int(GAIT_RUN_SECONDS * SLICES_PER_MINUTE / 60)
        move = cs.MoveParams(
            type="custom_gait",
            start_slice=0,
            requested_slices=requested_slices,
            id=MOVE_ID,
        )
        move.custom_gait_params.CopyFrom(self._build_params())
        return cs.ChoreographySequence(
            name=SEQUENCE_NAME,
            slices_per_minute=SLICES_PER_MINUTE,
            moves=[move],
            entrance_state=cs.MoveInfo.TRANSITION_STATE_STAND,
        )

    # ------------------------------------------------------------------
    # Lifecycle: start / steer / stop
    # ------------------------------------------------------------------

    def start(self):
        """From a standing state, upload and execute the custom gait sequence.

        Guarantees a settled stand first via blocking_stand, since the gait's
        entrance state is STAND. Idempotent: a no-op if already running.
        """
        if self._running:
            return
        # Require a settled stand before entering the gait.
        blocking_stand(self._command_client, timeout_sec=10)

        seq = self._build_sequence()
        upload_resp = self._choreo.upload_choreography(seq, non_strict_parsing=True)
        warnings = list(getattr(upload_resp, "warnings", []) or [])
        if warnings:
            logger.info(f"Custom gait upload warnings: {warnings}")

        self._choreo.execute_choreography(
            choreography_name=SEQUENCE_NAME,
            client_start_time=time.time() + START_DELAY_S,
            choreography_starting_slice=0.0,
        )
        self._running = True
        self._limits = None
        self._limits_logged = False
        self._last_limits_poll = 0.0   # poll limits on the first steering tick
        logger.info("Custom gait STARTED (upload + execute)")

    def steer(self, vx: float, vy: float, v_rot: float,
              pitch: float, roll: float, yaw: float, height: float):
        """Push one realtime ChoreographyCommand with steering + live body offset.

        Velocities are clamped first to the live CustomGaitCommandLimits (which may
        be lower than the configured maxima depending on what the pattern supports),
        then offsets to the gait clamps. Called at the command-loop rate (~20 Hz).
        """
        if not self._running:
            return

        self._refresh_limits()

        # Clamp steering to the live limits, where available.
        if self._limits is not None:
            mv = self._limits.maximum_drive_velocity_body
            vx = _clamp(vx, abs(mv.linear.x) or GAIT_MAX_VELOCITY)
            vy = _clamp(vy, abs(mv.linear.y) or GAIT_MAX_VELOCITY)
            v_rot = _clamp(v_rot, abs(mv.angular) or GAIT_MAX_YAW_RATE)
            mo = self._limits.maximum_body_orientation_offset
            pitch = _clamp(pitch, abs(mo.pitch) or MAX_PITCH_GAIT)
            roll = _clamp(roll, abs(mo.roll) or MAX_ROLL_GAIT)
            yaw = _clamp(yaw, abs(mo.yaw) or MAX_YAW_GAIT)
        else:
            pitch = _clamp(pitch, MAX_PITCH_GAIT)
            roll = _clamp(roll, MAX_ROLL_GAIT)
            yaw = _clamp(yaw, MAX_YAW_GAIT)

        gait_cmd = cp.CustomGaitCommand()
        gait_cmd.drive_velocity_body.linear.x = vx
        gait_cmd.drive_velocity_body.linear.y = vy
        gait_cmd.drive_velocity_body.angular = v_rot
        # Live roll/pitch/yaw and height while walking (the "roll-while-walking"
        # Path A): CustomGaitCommand.body_orientation_offset is a realtime EulerZYX
        # axis in this SDK, so roll varies continuously per tick with no re-staging.
        # See docs/roll_while_walking_findings.md.
        gait_cmd.body_orientation_offset.roll = roll
        gait_cmd.body_orientation_offset.pitch = pitch
        gait_cmd.body_orientation_offset.yaw = yaw
        gait_cmd.body_translation_offset.z = height
        gait_cmd.finished = False

        move_cmd = cs.MoveCommand(move_type="custom_gait", move_id=MOVE_ID)
        move_cmd.custom_gait_command.CopyFrom(gait_cmd)

        self._choreo.choreography_command(
            command_list=[move_cmd],
            client_end_time=time.time() + COMMAND_VALIDITY_S,
        )

    def stop(self):
        """Gracefully end the gait (equivalent to Choreographer's d-pad-down).

        Sends finished=True, then waits for standard_final_stance to settle so the
        caller can hand back to synchro_stand without slamming the controller.
        Idempotent: a no-op if not running.
        """
        if not self._running:
            return
        try:
            gait_cmd = cp.CustomGaitCommand()
            gait_cmd.finished = True
            move_cmd = cs.MoveCommand(move_type="custom_gait", move_id=MOVE_ID)
            move_cmd.custom_gait_command.CopyFrom(gait_cmd)
            self._choreo.choreography_command(
                command_list=[move_cmd],
                client_end_time=time.time() + COMMAND_VALIDITY_S,
            )
        except Exception as e:
            logger.warning(f"Custom gait stop command failed (continuing to settle): {e}")
        finally:
            self._running = False
            self._limits = None

        # Let the final stance complete before the caller stands.
        time.sleep(STOP_SETTLE_S)
        logger.info("Custom gait STOPPED (finished + settled)")

    def mark_stopped(self):
        """Clear running state WITHOUT sending or sleeping. For E-stop, where power
        is already cut so the gait is physically halted and a graceful stop would
        only block on a command that cannot land."""
        self._running = False
        self._limits = None

    # ------------------------------------------------------------------
    # Status / limits
    # ------------------------------------------------------------------

    def _refresh_limits(self):
        """Pull the live CustomGaitCommandLimits from choreography status, throttled
        so it does not issue a status RPC on every ~20 Hz steering tick."""
        now = time.time()
        if now - self._last_limits_poll < LIMITS_REFRESH_S:
            return
        self._last_limits_poll = now
        try:
            status, _ = self._choreo.get_choreography_status()
        except Exception as e:
            logger.debug(f"Choreography status poll skipped: {e}")
            return
        for active in status.active_moves:
            if active.HasField("custom_gait_command_limits"):
                self._limits = active.custom_gait_command_limits
                if not self._limits_logged:
                    mv = self._limits.maximum_drive_velocity_body
                    mo = self._limits.maximum_body_orientation_offset
                    logger.info(
                        "CustomGaitCommandLimits  "
                        f"v=({mv.linear.x:.2f},{mv.linear.y:.2f}) yaw={mv.angular:.2f}  "
                        f"orient(r/p/y)=({mo.roll:.2f},{mo.pitch:.2f},{mo.yaw:.2f})"
                    )
                    self._limits_logged = True
                return

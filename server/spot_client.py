"""
Spot SDK wrapper — manages lease, E-stop, and sends body-posed velocity commands.

Key capability: walk with a persistent body pose (pitch/roll/height) via
BodyControlParams.base_offset_rt_footprint, rather than snapping to upright.
"""

import logging
import math
import time
import threading
from dataclasses import dataclass
from typing import Optional

import bosdyn.client
import bosdyn.client.estop
import bosdyn.client.lease
import bosdyn.geometry
from bosdyn.api import geometry_pb2
from bosdyn.api import trajectory_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.api import robot_state_pb2
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.keepalive import KeepaliveClient, Policy, PolicyKeepalive, remove_all_policies
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.robot_command import (
    RobotCommandBuilder,
    RobotCommandClient,
    blocking_stand,
)
from bosdyn.client.frame_helpers import (
    get_a_tform_b,
    BODY_FRAME_NAME,
    GRAV_ALIGNED_BODY_FRAME_NAME,
)
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.client.license import LicenseClient
from bosdyn.choreography.client.choreography import ChoreographyClient
from bosdyn.api import image_pb2

from custom_gait import CustomGaitWalker

logger = logging.getLogger(__name__)

# Walk backend: "custom_gait" holds pitch AND roll through the step cycle via the
# Choreography Custom Gait paradigm; "mobility" is the legacy velocity-command path
# (pitch holds, roll washes out under the balancer) kept as a known-good fallback.
WALK_BACKEND = "custom_gait"   # "custom_gait" | "mobility"

# Safety limits
MAX_VX = 1.5        # m/s forward/back
MAX_VY = 0.5        # m/s strafe
MAX_VROT = 1.0      # rad/s yaw
MAX_PITCH = 0.5     # rad (~28 deg)
MAX_ROLL = 0.3      # rad (~17 deg)
MAX_HEIGHT = 0.15   # m above nominal
MIN_HEIGHT = -0.10  # m below nominal

COMMAND_HZ = 20
COMMAND_PERIOD = 1.0 / COMMAND_HZ
# Each command is valid for 4x the period; robot stops if we miss 4 cycles
COMMAND_TIMEOUT_FACTOR = 4


@dataclass
class ControlState:
    """Desired robot state, updated from WebSocket commands."""
    # Velocity (walk mode)
    vx: float = 0.0
    vy: float = 0.0
    v_rot: float = 0.0
    # Body pose (stand/walk modes)
    pitch: float = 0.0
    roll: float = 0.0
    yaw_offset: float = 0.0
    height: float = 0.0
    # Mode — exactly one of these should be True
    walking: bool = False
    sitting: bool = True   # default: sit until operator explicitly stands/walks
    # Freeze all output (E-stop or no WebSocket client)
    frozen: bool = True


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _license_info_to_dict(info) -> dict:
    """Convert a LicenseInfo proto to a UI-friendly dict. licensed_features holds
    the exact feature-code strings (e.g. for Joint Level Control)."""
    nvb, nva = info.not_valid_before, info.not_valid_after
    return {
        "status": info.Status.Name(info.status),
        "id": info.id,
        "robot_serial": info.robot_serial,
        "not_valid_before": nvb.ToDatetime().isoformat() if nvb.seconds else None,
        "not_valid_after": nva.ToDatetime().isoformat() if nva.seconds else None,
        "licensed_features": list(info.licensed_features),
    }


# A license read is a passive query: it only needs authentication, NOT a lease,
# E-Stop, or motor power. This standalone path lets the UI read the license even
# when the full control connection (lease/E-Stop/power/stand) is not up. The
# authenticated robot is cached so repeat reads skip re-auth.
_license_robot = None
_license_lock = threading.Lock()


def read_license(hostname: str, username: str, password: str) -> dict:
    """Read the installed license over a read-only, auth-only connection.

    Never acquires a lease, registers an E-Stop, powers motors, or moves the
    robot. Returns {"error": ...} on failure rather than raising.
    """
    global _license_robot
    with _license_lock:
        try:
            if _license_robot is None:
                sdk = bosdyn.client.create_standard_sdk("spot-license-read")
                robot = sdk.create_robot(hostname)
                robot.authenticate(username, password)   # auth only; no lease/E-Stop/power
                _license_robot = robot
            client = _license_robot.ensure_client(LicenseClient.default_service_name)
            return _license_info_to_dict(client.get_license_info())
        except Exception as e:
            _license_robot = None   # drop the cached robot so the next read re-auths
            return {"error": str(e)}


def _euler_to_quaternion(roll: float, pitch: float, yaw: float):
    """ZXY convention (matches SDK EulerZXY used in stand mode) → (w, x, y, z)."""
    cr = math.cos(roll / 2);  sr = math.sin(roll / 2)
    cp = math.cos(pitch / 2); sp = math.sin(pitch / 2)
    cy = math.cos(yaw / 2);   sy = math.sin(yaw / 2)
    return (
        cy * cr * cp - sy * sr * sp,
        cy * sr * cp - sy * cr * sp,
        cy * cr * sp + sy * sr * cp,
        cy * sr * sp + sy * cr * cp,
    )


class SpotController:
    # Base-unit grayscale fisheye cameras served by the image service. These need
    # no lease and are independent of the command loop and E-stop. Used as an
    # allowlist so the camera endpoint cannot request arbitrary services.
    CAMERA_SOURCES = {
        "frontleft_fisheye_image", "frontright_fisheye_image",
        "left_fisheye_image", "right_fisheye_image", "back_fisheye_image",
    }

    def __init__(self, hostname: str):
        self.hostname = hostname
        self.sdk = bosdyn.client.create_standard_sdk("spot-controller")
        # The Choreography service (Custom Gait) is not part of the standard SDK
        # client registration, and register_service_client MUST run BEFORE
        # create_robot: the Robot caches the service-type map at creation, so a
        # later registration (e.g. inside the walker's setup) does not reach it and
        # ensure_client raises UnregisteredServiceTypeError. Registering here is
        # harmless for the mobility backend.
        self.sdk.register_service_client(ChoreographyClient)
        self.robot = self.sdk.create_robot(hostname)
        self.state = ControlState()
        self._lock = threading.Lock()

        # robot_connected: reachable + authed + read-only clients up (cameras,
        #   license, telemetry available). control_ready: lease/E-Stop/power/stand
        #   up and the command loop running (sit/stand/walk available). The two are
        #   separate so cameras/license/telemetry keep working when control bring-up
        #   (e.g. power-on) fails.
        self.robot_connected = False
        self.control_ready = False
        # Effective walk backend for this session. Starts at the configured default
        # but falls back to "mobility" if the robot lacks the choreography license.
        self.walk_backend = WALK_BACKEND
        self._control_lock = threading.Lock()   # serialize control bring-up / recovery
        self._running = False
        self._command_thread: Optional[threading.Thread] = None
        self._lease_keepalive: Optional[LeaseKeepAlive] = None
        self._estop_keepalive: Optional[EstopKeepAlive] = None
        self._policy_keepalive: Optional[PolicyKeepalive] = None
        self._command_client: Optional[RobotCommandClient] = None
        self._state_client: Optional[RobotStateClient] = None
        self._lease_client: Optional[LeaseClient] = None
        self._estop_client = None
        self._keepalive_client = None
        self._image_client: Optional[ImageClient] = None   # base-unit cameras (no lease needed)
        self._license_client: Optional[LicenseClient] = None   # passive license read (no lease)
        self._last_robot_state = None   # cached by get_full_status, read by diagnostics
        self._walker: Optional[CustomGaitWalker] = None   # Custom Gait backend (None until setup)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def authenticate(self, username: str, password: str):
        self.robot.authenticate(username, password)
        self.robot.time_sync.wait_for_sync()
        logger.info("Authenticated and time-synced")

    def setup(self):
        """Read-only connect, then control bring-up. Kept for callers that want the
        full sequence; the server stages these separately so cameras/license/
        telemetry stay available even if control bring-up fails."""
        self.connect()
        self.bring_up_control()

    def connect(self):
        """Create the read-only clients: command/state/lease stubs plus image and
        (lazily) license. Requires only authentication, no lease or power. This is
        enough for cameras, the license read, and telemetry. Idempotent."""
        self._lease_client = self.robot.ensure_client(LeaseClient.default_service_name)
        self._command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
        self._state_client = self.robot.ensure_client(RobotStateClient.default_service_name)
        self._estop_client = self.robot.ensure_client(EstopClient.default_service_name)
        self._keepalive_client = self.robot.ensure_client(KeepaliveClient.default_service_name)

        # Image client for the base cameras. Lease-free and decoupled from control.
        try:
            self._image_client = self.robot.ensure_client(ImageClient.default_service_name)
            available = {s.name for s in self._image_client.list_image_sources()}
            missing = self.CAMERA_SOURCES - available
            if missing:
                logger.warning(f"Camera sources not advertised by robot: {sorted(missing)}")
            logger.info(f"Image client ready. Base camera sources: {sorted(self.CAMERA_SOURCES & available)}")
        except Exception as e:
            self._image_client = None
            logger.warning(f"Image client unavailable, cameras disabled: {e}")

        self.robot_connected = True
        logger.info("Read-only connect complete (cameras, license, telemetry available)")

    def bring_up_control(self):
        """Acquire lease + E-Stop, clear clearable faults, power on, stand, and start
        the command loop. Separate from connect() so a failure here (e.g. a power
        fault) leaves cameras/license/telemetry working. Idempotent and serialized by
        _control_lock, so the startup retry loop and the manual recover button cannot
        run it concurrently."""
        with self._control_lock:
            if self.control_ready:
                return
            self._ensure_control_infra()
            self._power_and_stand()

    def recover_control(self) -> dict:
        """Operator-triggered recovery: clear faults, power on, stand. Forced (does
        not early-return when control_ready) so it can recover after a mid-session
        fault or motors-off. Returns a result dict instead of raising."""
        if not self.robot_connected:
            return {"ok": False, "error": "robot not connected"}
        with self._control_lock:
            try:
                self._ensure_control_infra()
                self._power_and_stand()
                return {"ok": True, "control_ready": self.control_ready}
            except Exception as e:
                return {"ok": False, "error": str(e)}

    def _ensure_control_infra(self):
        """Idempotently set up the keepalive policy, E-Stop, and lease. Safe to call
        repeatedly: each sub-step is skipped if already done, so retries do not stack
        keepalive/E-Stop/lease threads."""
        # Clear any keepalive policies from the tablet (these can block power-on),
        # then register ours: controlled sit + motors off after 30 s of no check-in.
        if not self._policy_keepalive:
            remove_all_policies(self._keepalive_client, attempts=3)
            logger.info("Cleared existing keepalive policies")
            our_policy = Policy()
            our_policy.name = "spot-controller-web"
            our_policy.add_controlled_motors_off_action(after=30)
            self._policy_keepalive = PolicyKeepalive(
                self._keepalive_client, our_policy,
                rpc_interval_seconds=5,
                remove_policy_on_exit=True,
            )
            self._policy_keepalive.__enter__()
            logger.info("Keepalive policy registered")

        # E-stop: 9 s timeout, check-ins every ~2 s
        if not self._estop_keepalive:
            estop_endpoint = EstopEndpoint(
                client=self._estop_client,
                name="spot-controller-web",
                estop_timeout=9.0,
            )
            estop_endpoint.force_simple_setup()
            self._estop_keepalive = EstopKeepAlive(estop_endpoint)

        # Take the lease forcefully — works whether or not another client holds it
        if not self._lease_keepalive:
            self._lease = self._lease_client.take()
            self._lease_keepalive = LeaseKeepAlive(
                self._lease_client, must_acquire=False, return_at_exit=True
            )

    def _power_and_stand(self):
        """Clear clearable faults, power on, stand, and ensure the command loop runs.
        Raises on power-on failure (after logging the FaultState)."""
        # Clear any clearable behavior faults (e.g. from a prior fall or hard stop),
        # which otherwise make power-on fail with a FaultedError, then power on.
        if not self.robot.is_powered_on():
            self._clear_behavior_faults()
            try:
                self.robot.power_on(timeout_sec=20)
            except Exception:
                self._log_fault_state()
                raise
            assert self.robot.is_powered_on(), "Motor power failed"
        else:
            logger.info("Motors already on")

        # Custom Gait preflight: register + verify choreography license. If the
        # license is missing this is NOT fatal: fall back to the mobility walk
        # backend so control still comes up (walk via the standard velocity path).
        if self.walk_backend == "custom_gait" and not self._walker:
            try:
                walker = CustomGaitWalker(self.robot, self._command_client)
                walker.setup(self.sdk)
                self._walker = walker
            except Exception as e:
                logger.warning(
                    f"Custom Gait unavailable ({e}). Falling back to the mobility "
                    "walk backend (pitch holds while walking; roll does not)."
                )
                self.walk_backend = "mobility"

        blocking_stand(self._command_client, timeout_sec=10)
        logger.info("=" * 60)
        logger.info(f"CONTROL READY - walk backend: {self.walk_backend}")
        logger.info("=" * 60)

        if not self._command_thread:
            self._running = True
            with self._lock:
                # Start in stand mode, unfrozen; operator can sit, walk, or pose.
                self.state.sitting = False
                self.state.walking = False
                self.state.frozen = False
            self._command_thread = threading.Thread(target=self._command_loop, daemon=True)
            self._command_thread.start()
        self.control_ready = True

    def _clear_behavior_faults(self):
        """Clear any CLEARABLE behavior faults so power-on can proceed. Needs the
        lease (already held by the time this runs). Best-effort: logs and continues."""
        try:
            rs = self._state_client.get_robot_state()
        except Exception as e:
            logger.warning(f"Could not read robot state to clear faults: {e}")
            return
        faults = rs.behavior_fault_state.faults
        for bf in faults:
            cause = bf.Cause.Name(bf.cause).replace("CAUSE_", "")
            if bf.status == bf.STATUS_CLEARABLE:
                try:
                    self._command_client.clear_behavior_fault(behavior_fault_id=bf.behavior_fault_id)
                    logger.info(f"Cleared behavior fault id={bf.behavior_fault_id} (cause={cause})")
                except Exception as e:
                    logger.warning(f"Failed to clear behavior fault id={bf.behavior_fault_id}: {e}")
            else:
                logger.warning(f"Behavior fault id={bf.behavior_fault_id} (cause={cause}) is NOT clearable via API")

    def _log_fault_state(self):
        """Log the robot's current fault state to explain a failed power-on."""
        try:
            rs = self._state_client.get_robot_state()
        except Exception as e:
            logger.warning(f"Power-on failed and robot state unavailable: {e}")
            return
        bfaults = [
            f"{bf.Cause.Name(bf.cause).replace('CAUSE_', '')}"
            f"/{bf.Status.Name(bf.status).replace('STATUS_', '')}"
            for bf in rs.behavior_fault_state.faults
        ]
        sfaults = [
            f.error_message for f in rs.system_fault_state.faults
            if f.severity >= robot_state_pb2.SystemFault.SEVERITY_WARN
        ]
        logger.error(
            f"Power-on blocked by fault. behavior_faults={bfaults or 'none'}  "
            f"system_faults={sfaults or 'none'}"
        )

    def shutdown(self):
        """Sit, power off, release everything."""
        self._running = False
        with self._lock:
            self.state.frozen = True

        # End any running gait before sitting so we never sit straight from a
        # running choreography.
        if self._walker and self._walker.is_running:
            try:
                self._walker.stop()
            except Exception:
                self._walker.mark_stopped()

        if self._command_client:
            try:
                cmd = RobotCommandBuilder.synchro_sit_command()
                self._command_client.robot_command(command=cmd)
                time.sleep(3)
            except Exception:
                pass

        try:
            self.robot.power_off(cut_immediately=False, timeout_sec=20)
        except Exception:
            pass

        if self._estop_keepalive:
            self._estop_keepalive.shutdown()
        if self._lease_keepalive:
            self._lease_keepalive.shutdown()
        if self._policy_keepalive:
            self._policy_keepalive.__exit__(None, None, None)

        logger.info("Shutdown complete")

    def trigger_estop(self):
        """Cut power immediately via E-stop."""
        if self._estop_keepalive:
            self._estop_keepalive.stop()
        with self._lock:
            self.state.frozen = True
        # Power is cut so any running gait is physically halted; clear its state
        # (without a graceful stop, which could not land a command anyway) so a
        # later walk starts cleanly. The frozen flag short-circuits steering ticks.
        if self._walker:
            self._walker.mark_stopped()
        logger.warning("E-STOP triggered")

    # ------------------------------------------------------------------
    # State updates (called from WebSocket handler)
    # ------------------------------------------------------------------

    def update(self, **kwargs):
        """Thread-safe state update. Unknown keys are silently ignored."""
        with self._lock:
            for k, v in kwargs.items():
                if not hasattr(self.state, k):
                    continue
                if k == "vx":
                    v = _clamp(v, -MAX_VX, MAX_VX)
                elif k == "vy":
                    v = _clamp(v, -MAX_VY, MAX_VY)
                elif k == "v_rot":
                    v = _clamp(v, -MAX_VROT, MAX_VROT)
                elif k == "pitch":
                    v = _clamp(v, -MAX_PITCH, MAX_PITCH)
                elif k == "roll":
                    v = _clamp(v, -MAX_ROLL, MAX_ROLL)
                elif k == "height":
                    v = _clamp(v, MIN_HEIGHT, MAX_HEIGHT)
                setattr(self.state, k, v)
            # Entering walk or stand always clears sit mode
            if kwargs.get("walking") is True or kwargs.get("sitting") is False:
                self.state.sitting = False

    def sit(self):
        """Sit down. Command loop holds sit until mode explicitly changes."""
        with self._lock:
            self.state.sitting = True
            self.state.walking = False
            self.state.vx = 0.0
            self.state.vy = 0.0
            self.state.v_rot = 0.0
        logger.info("Sit mode activated")

    def stand_up(self):
        """Rise from sit to stand. Clears sit flag so loop switches to stand command."""
        with self._lock:
            self.state.sitting = False
            self.state.walking = False
        logger.info("Stand mode activated")

    def safe_stop(self):
        """Sit and freeze — called on unexpected WebSocket disconnect."""
        self.sit()
        logger.warning("Safe stop: sitting due to client disconnect")

    # ------------------------------------------------------------------
    # Command building
    # ------------------------------------------------------------------

    def _build_body_control(self, pitch: float, roll: float, yaw: float, height: float):
        """
        Build BodyControlParams using base_offset_rt_footprint.

        Matches the official SDK mobility_params() pattern exactly: a single
        SE3TrajectoryPoint with NO time_since_reference and NO reference_time.
        Setting those fields places the point in the future, causing the robot to
        discard the rotation entirely — which was the root cause of the roll/pitch
        not being applied during walk mode.
        """
        rotation = bosdyn.geometry.EulerZXY(
            yaw=yaw, roll=roll, pitch=pitch
        ).to_quaternion()
        pose = geometry_pb2.SE3Pose(
            position=geometry_pb2.Vec3(z=height),
            rotation=rotation,
        )
        traj = trajectory_pb2.SE3Trajectory(
            points=[trajectory_pb2.SE3TrajectoryPoint(pose=pose)]
        )
        return spot_command_pb2.BodyControlParams(base_offset_rt_footprint=traj)

    def _build_mobility_params(self, s: ControlState, log_this: bool = False) -> spot_command_pb2.MobilityParams:
        body_control = self._build_body_control(s.pitch, s.roll, s.yaw_offset, s.height)
        has_pose = abs(s.pitch) > 0.01 or abs(s.roll) > 0.01 or abs(s.height) > 0.005
        hint = spot_command_pb2.HINT_CRAWL if has_pose else spot_command_pb2.HINT_AUTO
        params = spot_command_pb2.MobilityParams(
            body_control=body_control,
            locomotion_hint=hint,
        )
        if log_this:
            # Read back from proto to confirm fields were set correctly
            bc = params.body_control
            rs = bc.rotation_setting
            rs_name = {0: "UNKNOWN", 1: "OFFSET", 2: "ABSOLUTE"}.get(rs, str(rs))
            hint_name = {0: "UNKNOWN", 1: "AUTO", 2: "TROT", 3: "SPEED_SELECT_TROT",
                         4: "CRAWL", 5: "AMBLE", 6: "SPEED_SELECT_AMBLE", 7: "JOG",
                         8: "HOP", 10: "SPEED_SELECT_CRAWL"}.get(hint, str(hint))
            which = bc.WhichOneof("param")
            if which == "body_pose":
                q = bc.body_pose.base_offset_rt_root.points[0].pose.rotation
                frame = bc.body_pose.root_frame_name
            elif which == "base_offset_rt_footprint":
                pts = bc.base_offset_rt_footprint.points
                q = pts[0].pose.rotation if pts else None
                tsref = pts[0].time_since_reference if pts else None
                frame = f"footprint  tsr={tsref.seconds if tsref else 'none'}s"
            else:
                q = None
                frame = "NONE — no param set!"
            q_str = f"w{q.w:+.4f} x{q.x:+.4f} y{q.y:+.4f} z{q.z:+.4f}" if q else "N/A"
            logger.info(
                f"PROTO  param={which}  frame={frame}"
                f"  rot_setting={rs}({rs_name})  hint={hint}({hint_name})"
                f"  q={q_str}"
            )
        return params

    def _actual_body_tilt(self):
        """Return (pitch_rad, roll_rad) of actual body from latest cached robot state.
        Uses the flat_body→body transform: flat_body is gravity-aligned at the body
        centre, so this rotation is exactly the body's pitch and roll relative to gravity.
        Returns (None, None) if state is unavailable.
        """
        rs = self._last_robot_state
        if rs is None:
            return None, None
        try:
            tform = get_a_tform_b(
                rs.kinematic_state.transforms_snapshot,
                GRAV_ALIGNED_BODY_FRAME_NAME,
                BODY_FRAME_NAME,
            )
            if tform is None:
                return None, None
            return tform.rot.to_pitch(), tform.rot.to_roll()
        except Exception:
            return None, None

    def _log_pose_diagnostic(self, s: ControlState):
        """Three log lines per cycle: input state, proto params, actual vs commanded."""
        mode = "WALK" if s.walking else "SIT" if s.sitting else "STAND"
        has_pose = abs(s.pitch) > 0.01 or abs(s.roll) > 0.01 or abs(s.height) > 0.005
        if s.walking and self._use_custom_gait():
            running = self._walker.is_running
            gait = f"CGAIT{'+' if running else '-'}"   # + = gait running, - = starting
        elif s.walking:
            gait = "CRAWL" if has_pose else "AUTO"     # legacy mobility backend
        else:
            gait = "----"

        cmd_p = math.degrees(s.pitch)
        cmd_r = math.degrees(s.roll)
        cmd_h = s.height * 100

        # Line 1 — what the controller state currently holds (sourced from WS input)
        logger.info(
            f"[{mode}/{gait}] STATE  pitch={cmd_p:+6.1f}°  roll={cmd_r:+6.1f}°"
            f"  yaw={math.degrees(s.yaw_offset):+5.1f}°  h={cmd_h:+4.1f}cm"
            f"  vx={s.vx:+.2f}  vy={s.vy:+.2f}  vrot={s.v_rot:+.2f}"
        )
        # (PROTO line already emitted by _send_command → _build_mobility_params)

        # Line 2 — actual body tilt from robot state
        actual_p, actual_r = self._actual_body_tilt()
        if actual_p is not None:
            act_p = math.degrees(actual_p)
            act_r = math.degrees(actual_r)
            err_p = cmd_p - act_p
            err_r = cmd_r - act_r
            logger.info(
                f"[{mode}/{gait}] ACTUAL pitch={act_p:+6.1f}°  roll={act_r:+6.1f}°"
                f"  err_p={err_p:+5.1f}°  err_r={err_r:+5.1f}°"
            )
            if s.walking and abs(cmd_r) > 2.0 and abs(err_r) > 4.0:
                logger.warning(
                    f"ROLL DRIFT [{gait}]  cmd={cmd_r:+.1f}°  actual={act_r:+.1f}°"
                    f"  err={err_r:+.1f}°"
                )
            if s.walking and abs(cmd_p) > 2.0 and abs(err_p) > 4.0:
                logger.warning(
                    f"PITCH DRIFT [{gait}]  cmd={cmd_p:+.1f}°  actual={act_p:+.1f}°"
                    f"  err={err_p:+.1f}°"
                )
        else:
            logger.warning(f"[{mode}/{gait}] ACTUAL unavailable — no robot state yet")

    def _use_custom_gait(self) -> bool:
        return self.walk_backend == "custom_gait" and self._walker is not None

    def _send_command(self, s: ControlState, log_this: bool = False):
        end_time = time.time() + COMMAND_PERIOD * COMMAND_TIMEOUT_FACTOR

        # Custom Gait and the mobility velocity command must never run concurrently
        # against the body lease, so every non-walk branch first stops any running
        # gait before issuing a synchro command.
        if s.sitting:
            if self._walker and self._walker.is_running:
                self._walker.stop()   # walk -> sit: end gait, then sit
            cmd = RobotCommandBuilder.synchro_sit_command()
            self._command_client.robot_command(command=cmd, end_time_secs=end_time)

        elif s.walking and self._use_custom_gait():
            # Drive Custom Gait: ensure it is running, then push steering + live pose.
            # Do NOT send synchro_velocity_command on this path.
            if not self._walker.is_running:
                self._walker.start()
            self._walker.steer(
                vx=s.vx, vy=s.vy, v_rot=s.v_rot,
                pitch=s.pitch, roll=s.roll, yaw=s.yaw_offset, height=s.height,
            )

        elif s.walking:
            # Legacy mobility backend (roll washes out under the balancer).
            params = self._build_mobility_params(s, log_this=log_this)
            cmd = RobotCommandBuilder.synchro_velocity_command(
                v_x=s.vx,
                v_y=s.vy,
                v_rot=s.v_rot,
                params=params,
            )
            self._command_client.robot_command(command=cmd, end_time_secs=end_time)

        else:
            if self._walker and self._walker.is_running:
                self._walker.stop()   # walk -> stand: end gait before standing
            footprint_R_body = bosdyn.geometry.EulerZXY(
                yaw=s.yaw_offset, roll=s.roll, pitch=s.pitch
            )
            cmd = RobotCommandBuilder.synchro_stand_command(
                footprint_R_body=footprint_R_body,
                body_height=s.height,
            )
            self._command_client.robot_command(command=cmd, end_time_secs=end_time)

    # ------------------------------------------------------------------
    # Command loop (background thread)
    # ------------------------------------------------------------------

    def _command_loop(self):
        loop_count = 0
        while self._running:
            loop_start = time.monotonic()
            try:
                with self._lock:
                    s = ControlState(**self.state.__dict__)
                if not s.frozen:
                    log_now = (loop_count % 5 == 0)   # 4 Hz diagnostic
                    if log_now:
                        self._log_pose_diagnostic(s)   # STATE + ACTUAL first
                    self._send_command(s, log_this=log_now)  # PROTO after
                    loop_count += 1
            except Exception as e:
                logger.error(f"Command loop error: {e}", exc_info=True)

            elapsed = time.monotonic() - loop_start
            sleep_time = COMMAND_PERIOD - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    @property
    def status(self) -> dict:
        with self._lock:
            s = self.state
            return {
                "connected": True,
                "robot_connected": self.robot_connected,
                "control_ready": self.control_ready,
                "walk_backend": self.walk_backend,
                "gait_running": bool(self._walker and self._walker.is_running),
                "walking": s.walking,
                "sitting": s.sitting,
                "frozen": s.frozen,
                "vx": round(s.vx, 3),
                "vy": round(s.vy, 3),
                "v_rot": round(s.v_rot, 3),
                "pitch": round(s.pitch, 3),
                "roll": round(s.roll, 3),
                "yaw_offset": round(s.yaw_offset, 3),
                "height": round(s.height, 3),
            }

    def get_full_status(self) -> dict:
        """Control state + live robot telemetry. Called from a thread — all gRPC is synchronous."""
        out = self.status

        if not self._state_client:
            return out

        try:
            rs = self._state_client.get_robot_state()
            self._last_robot_state = rs

            # Battery
            if rs.battery_states:
                b = rs.battery_states[0]
                try:
                    out["battery_pct"] = round(b.charge_percentage.value, 1)
                except AttributeError:
                    out["battery_pct"] = round(float(b.charge_percentage), 1)
                out["battery_status"] = b.Status.Name(b.status).replace("STATUS_", "")
                runtime_sec = b.estimated_runtime.seconds
                out["battery_runtime_min"] = int(runtime_sec // 60) if runtime_sec > 0 else None

            # Motor power state
            mp = rs.power_state.motor_power_state
            mp_name = rs.power_state.MotorPowerState.Name(mp)
            out["motor_state"] = mp_name.replace("MOTOR_POWER_STATE_", "").replace("STATE_", "")

            # Behavior state (what the robot is actually doing)
            beh = rs.behavior_state.state
            beh_name = rs.behavior_state.State.Name(beh).replace("STATE_", "")
            out["behavior_state"] = beh_name

            # E-stop
            out["estopped"] = any(e.state == e.STATE_ESTOPPED for e in rs.estop_states)

            # Behavior faults (falls, hardware issues, etc.)
            out["behavior_faults"] = [
                bf.Cause.Name(bf.cause).replace("CAUSE_", "")
                for bf in rs.behavior_fault_state.faults
            ]

            # System faults — only WARN or CRITICAL
            out["system_faults"] = [
                f.error_message
                for f in rs.system_fault_state.faults
                if f.severity >= robot_state_pb2.SystemFault.SEVERITY_WARN
            ]

            # Foot contact: True = in contact with ground (FL, FR, RL, RR)
            out["foot_contact"] = [f.contact == f.CONTACT_MADE for f in rs.foot_state]

            # Actual body tilt from flat_body→body frame transform
            try:
                tform = get_a_tform_b(
                    rs.kinematic_state.transforms_snapshot,
                    GRAV_ALIGNED_BODY_FRAME_NAME,
                    BODY_FRAME_NAME,
                )
                if tform:
                    out["pitch_actual"] = round(math.degrees(tform.rot.to_pitch()), 1)
                    out["roll_actual"]  = round(math.degrees(tform.rot.to_roll()),  1)
            except Exception:
                pass

        except Exception as e:
            logger.debug(f"State poll skipped: {e}")

        return out

    # ------------------------------------------------------------------
    # Cameras (base unit, image service)
    # ------------------------------------------------------------------

    def get_camera_jpeg(self, source: str, quality: int = 50) -> Optional[bytes]:
        """Grab one JPEG frame from a base camera. Returns None on failure.

        Runs synchronously; call via asyncio.to_thread from the endpoint so it never
        blocks the 20 Hz command loop. Requires no lease and is fully independent of
        the command loop and E-stop. JPEG is requested directly from the service, so
        there is no server-side decode.
        """
        if source not in self.CAMERA_SOURCES or not self._image_client:
            return None
        try:
            req = build_image_request(
                source,
                quality_percent=quality,
                image_format=image_pb2.Image.FORMAT_JPEG,
            )
            responses = self._image_client.get_image([req])
            return responses[0].shot.image.data  # JPEG bytes
        except Exception as e:
            logger.debug(f"Image grab failed for {source}: {e}")
            return None

    # ------------------------------------------------------------------
    # License (passive read; no lease, never moves the robot)
    # ------------------------------------------------------------------

    def get_license(self) -> dict:
        """Read the robot's installed license as a structured dict for the UI.

        A passive query: no lease, no E-Stop, no power-on. The licensed_features
        list holds the exact feature-code strings (e.g. for Joint Level Control).
        Returns {"error": ...} on failure rather than raising.
        """
        if not self._license_client:
            try:
                self._license_client = self.robot.ensure_client(LicenseClient.default_service_name)
            except Exception as e:
                return {"error": f"license service unavailable: {e}"}
        try:
            return _license_info_to_dict(self._license_client.get_license_info())
        except Exception as e:
            return {"error": str(e)}

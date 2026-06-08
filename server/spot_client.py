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
from google.protobuf.duration_pb2 import Duration
from google.protobuf.timestamp_pb2 import Timestamp

logger = logging.getLogger(__name__)

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
    def __init__(self, hostname: str):
        self.hostname = hostname
        self.sdk = bosdyn.client.create_standard_sdk("spot-controller")
        self.robot = self.sdk.create_robot(hostname)
        self.state = ControlState()
        self._lock = threading.Lock()

        self._running = False
        self._command_thread: Optional[threading.Thread] = None
        self._lease_keepalive: Optional[LeaseKeepAlive] = None
        self._estop_keepalive: Optional[EstopKeepAlive] = None
        self._policy_keepalive: Optional[PolicyKeepalive] = None
        self._command_client: Optional[RobotCommandClient] = None
        self._state_client: Optional[RobotStateClient] = None
        self._lease_client: Optional[LeaseClient] = None
        self._last_robot_state = None   # cached by get_full_status, read by diagnostics

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def authenticate(self, username: str, password: str):
        self.robot.authenticate(username, password)
        self.robot.time_sync.wait_for_sync()
        logger.info("Authenticated and time-synced")

    def setup(self):
        """Clear blocking keepalive policies, acquire lease, configure E-stop, power on, stand."""
        self._lease_client = self.robot.ensure_client(LeaseClient.default_service_name)
        self._command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
        self._state_client = self.robot.ensure_client(RobotStateClient.default_service_name)
        estop_client = self.robot.ensure_client(EstopClient.default_service_name)
        keepalive_client = self.robot.ensure_client(KeepaliveClient.default_service_name)

        # Clear any keepalive policies from the tablet (these can block power-on)
        remove_all_policies(keepalive_client, attempts=3)
        logger.info("Cleared existing keepalive policies")

        # Register our own policy: controlled sit + motors off after 30 s of no check-in
        our_policy = Policy()
        our_policy.name = "spot-controller-web"
        our_policy.add_controlled_motors_off_action(after=30)
        self._policy_keepalive = PolicyKeepalive(
            keepalive_client, our_policy,
            rpc_interval_seconds=5,
            remove_policy_on_exit=True,
        )
        self._policy_keepalive.__enter__()
        logger.info("Keepalive policy registered")

        # E-stop: 9 s timeout, check-ins every ~2 s
        estop_endpoint = EstopEndpoint(
            client=estop_client,
            name="spot-controller-web",
            estop_timeout=9.0,
        )
        estop_endpoint.force_simple_setup()
        self._estop_keepalive = EstopKeepAlive(estop_endpoint)

        # Take the lease forcefully — works whether or not another client holds it
        self._lease = self._lease_client.take()
        self._lease_keepalive = LeaseKeepAlive(
            self._lease_client, must_acquire=False, return_at_exit=True
        )

        if not self.robot.is_powered_on():
            self.robot.power_on(timeout_sec=20)
            assert self.robot.is_powered_on(), "Motor power failed"
        else:
            logger.info("Motors already on")

        blocking_stand(self._command_client, timeout_sec=10)
        logger.info("Robot standing — starting command loop")

        self._running = True
        with self._lock:
            # Start in stand mode, unfrozen; operator sees the robot standing
            # and can choose to sit, walk, or pose via the UI
            self.state.sitting = False
            self.state.walking = False
            self.state.frozen = False
        self._command_thread = threading.Thread(target=self._command_loop, daemon=True)
        self._command_thread.start()

    def shutdown(self):
        """Sit, power off, release everything."""
        self._running = False
        with self._lock:
            self.state.frozen = True

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
        Build a BodyControlParams holding the body at (pitch, roll, yaw, height)
        relative to the footprint frame.

        The trajectory point is placed 0.5 s ahead of now so the locomotion
        controller always has a live future target.  The SDK auto-converts
        reference_time from local time to robot time before the gRPC call.
        """
        w, x, y, z = _euler_to_quaternion(roll, pitch, yaw)

        point = trajectory_pb2.SE3TrajectoryPoint()
        point.pose.CopyFrom(
            geometry_pb2.SE3Pose(
                position=geometry_pb2.Vec3(x=0.0, y=0.0, z=height),
                rotation=geometry_pb2.Quaternion(w=w, x=x, y=y, z=z),
            )
        )
        point.time_since_reference.CopyFrom(Duration(seconds=0, nanos=500_000_000))

        now = time.time()
        traj = trajectory_pb2.SE3Trajectory()
        traj.reference_time.CopyFrom(Timestamp(
            seconds=int(now),
            nanos=int((now % 1) * 1e9),
        ))
        traj.ang_interpolation = trajectory_pb2.ANG_INTERP_LINEAR
        traj.points.append(point)

        return spot_command_pb2.BodyControlParams(base_offset_rt_footprint=traj)

    def _build_mobility_params(self, s: ControlState) -> spot_command_pb2.MobilityParams:
        body_control = self._build_body_control(s.pitch, s.roll, s.yaw_offset, s.height)
        # Use crawl gait (3 feet always planted) when any pose offset is active.
        # Crawl's stable base lets the body-control constraint dominate over the
        # locomotion planner's balance compensation, which in trot gait can temporarily
        # override the body pose during the single-support phase.
        has_pose = abs(s.pitch) > 0.01 or abs(s.roll) > 0.01 or abs(s.height) > 0.005
        hint = spot_command_pb2.HINT_CRAWL if has_pose else spot_command_pb2.HINT_AUTO
        return spot_command_pb2.MobilityParams(
            body_control=body_control,
            locomotion_hint=hint,
        )

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
        """Emit one structured log line: commanded vs actual pitch/roll + error."""
        mode = "WALK" if s.walking else "SIT" if s.sitting else "STAND"
        has_pose = abs(s.pitch) > 0.01 or abs(s.roll) > 0.01 or abs(s.height) > 0.005
        gait = "CRAWL" if (s.walking and has_pose) else "AUTO " if s.walking else "---- "

        cmd_p = math.degrees(s.pitch)
        cmd_r = math.degrees(s.roll)
        cmd_h = s.height * 100

        actual_p, actual_r = self._actual_body_tilt()
        if actual_p is not None:
            act_p = math.degrees(actual_p)
            act_r = math.degrees(actual_r)
            err_p = cmd_p - act_p
            err_r = cmd_r - act_r
            actual_str = (f"actual pitch={act_p:+5.1f}° roll={act_r:+5.1f}°"
                          f"  err p={err_p:+5.1f}° r={err_r:+5.1f}°")
        else:
            actual_str = "actual=unavailable (no state yet)"

        logger.info(
            f"[{mode}/{gait}] cmd pitch={cmd_p:+5.1f}° roll={cmd_r:+5.1f}° h={cmd_h:+4.1f}cm"
            f"  |  {actual_str}"
        )

    def _send_command(self, s: ControlState):
        end_time = time.time() + COMMAND_PERIOD * COMMAND_TIMEOUT_FACTOR

        if s.sitting:
            # Sit takes priority — loop keeps sending so it can't be overridden
            cmd = RobotCommandBuilder.synchro_sit_command()
        elif s.walking:
            params = self._build_mobility_params(s)
            cmd = RobotCommandBuilder.synchro_velocity_command(
                v_x=s.vx,
                v_y=s.vy,
                v_rot=s.v_rot,
                params=params,
            )
        else:
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
                    self._send_command(s)
                    loop_count += 1
                    if loop_count % 10 == 0:   # log at ~2 Hz
                        self._log_pose_diagnostic(s)
            except Exception as e:
                logger.error(f"Command loop error: {e}")

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

"""
FastAPI server — bridges WebSocket control messages to the Spot SDK.

Run:  uvicorn main:app --host 0.0.0.0 --port 8000 --reload
Then open http://localhost:8000 in a browser on the same Mac.
"""

import asyncio
import collections
import json
import logging
import os
from contextlib import asynccontextmanager
from typing import Optional, Set

from dotenv import load_dotenv
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

from spot_client import SpotController, read_license

load_dotenv()

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-8s  %(name)s  %(message)s",
)
logger = logging.getLogger(__name__)


class _LogBuffer(logging.Handler):
    """Captures recent log lines for delivery to browser clients."""
    def __init__(self, maxlen: int = 120):
        super().__init__()
        self.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(module)s: %(message)s",
                                            datefmt="%H:%M:%S"))
        self._buf: collections.deque = collections.deque(maxlen=maxlen)

    def emit(self, record: logging.LogRecord) -> None:
        try:
            self._buf.append(self.format(record))
        except Exception:
            pass

    def recent(self, n: int = 30) -> list:
        return list(self._buf)[-n:]


log_buffer = _LogBuffer(maxlen=300)
logging.getLogger().addHandler(log_buffer)

spot: Optional[SpotController] = None
spot_error: Optional[str] = None   # last robot-connection error, surfaced to the UI
connected_clients: Set[WebSocket] = set()
_ws_control_counter: int = 0   # throttle input logging


# ------------------------------------------------------------------
# Transport selection — Ethernet (shore port) preferred, Wi-Fi fallback
# ------------------------------------------------------------------
# Spot is reachable on two links: the rear "shore" Ethernet port (default
# 10.0.0.3, must be enabled in the robot's Network Setup → Ethernet tab) and its
# own Wi-Fi access point (192.168.80.3). We probe the API port (443) on each
# candidate in preference order and connect to the first that answers, so a
# plugged-in Ethernet cable is used automatically while Wi-Fi keeps working when
# it is not. Re-resolved on every reconnect, so swapping links is transparent.

SPOT_API_PORT = 443
_PROBE_TIMEOUT_S = 1.0


def _candidate_hostnames() -> list:
    """Spot addresses to try, in preference order (Ethernet first).

    Override with SPOT_HOSTNAMES (comma-separated) for full control. Otherwise
    default to the shore-Ethernet IP first, then SPOT_HOSTNAME (the Wi-Fi AP
    address, kept for backward compatibility), deduped preserving order.
    """
    explicit = os.environ.get("SPOT_HOSTNAMES")
    if explicit:
        cands = [h.strip() for h in explicit.split(",") if h.strip()]
    else:
        cands = ["10.0.0.3"]   # rear shore-Ethernet port (preferred when plugged in)
        cands.append(os.environ.get("SPOT_HOSTNAME", "192.168.80.3"))   # Wi-Fi AP
    seen: Set[str] = set()
    return [c for c in cands if not (c in seen or seen.add(c))]


def _reachable(host: str, port: int = SPOT_API_PORT, timeout: float = _PROBE_TIMEOUT_S) -> bool:
    """True if a TCP connection to host:port succeeds within `timeout` seconds."""
    import socket
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def resolve_spot_hostname() -> tuple:
    """Return (hostname, candidates): the first reachable Spot address, or the
    first candidate as a fallback so the caller still surfaces a clear connection
    error when nothing answers. Blocking (socket probes) — call via to_thread."""
    cands = _candidate_hostnames()
    for h in cands:
        if _reachable(h):
            return h, cands
    return cands[0], cands


async def _status_payload() -> dict:
    """Status dict for the UI, valid whether or not the robot is connected.

    When the robot link is down, `spot` may be None; we still return a payload so
    the browser can render and show the robot as offline.
    """
    if spot:
        status = await asyncio.to_thread(spot.get_full_status)
    else:
        status = {"connected": True, "robot_connected": False}
    if spot_error:
        status["robot_error"] = spot_error
    return status


# ------------------------------------------------------------------
# Status broadcast — pushes robot telemetry to all clients at 1 Hz
# ------------------------------------------------------------------

async def _status_broadcast_loop():
    while True:
        await asyncio.sleep(1.0)
        if not connected_clients:
            continue
        try:
            status = await _status_payload()
            msg = json.dumps({"event": "status", **status, "log_lines": log_buffer.recent(50)})
            dead: Set[WebSocket] = set()
            for ws in list(connected_clients):
                try:
                    await ws.send_text(msg)
                except Exception:
                    dead.add(ws)
            connected_clients -= dead
        except Exception as e:
            logger.warning(f"Status broadcast error: {e}")


ROBOT_RETRY_S = 10   # how often to retry the robot link while it is down


async def _robot_connect_loop():
    """Connect to the robot, retrying in the background.

    The web UI must be reachable whether or not the robot is online, so a failed
    connection never aborts startup. `spot` is set only on a fully successful
    connect; until then the UI serves in a degraded mode and shows the robot as
    offline. A robot that is simply unreachable fails fast at authenticate (its
    first network call) before any control threads start, so retrying with a fresh
    controller is clean.
    """
    global spot, spot_error
    username = os.environ.get("SPOT_USERNAME", "user")
    password = os.environ.get("SPOT_PASSWORD")

    if not password:
        spot_error = "SPOT_PASSWORD not set"
        logger.error(f"Robot config missing: {spot_error}. Serving UI without robot control.")
        return

    # Stage 1: authenticate + read-only connect (cameras, license, telemetry).
    # Retry the whole thing until the robot is reachable. The link (Ethernet vs
    # Wi-Fi) is re-resolved each attempt, so swapping cables is picked up here.
    controller = None
    while True:
        try:
            hostname, cands = await asyncio.to_thread(resolve_spot_hostname)
            if controller is None or controller.hostname != hostname:
                controller = SpotController(hostname)
                logger.info(f"Spot link selected: {hostname}  (candidates: {', '.join(cands)})")
            logger.info(f"Connecting to Spot at {hostname}")
            await asyncio.to_thread(controller.authenticate, username, password)
            await asyncio.to_thread(controller.connect)
            spot = controller
            spot_error = None
            break
        except Exception as e:
            spot_error = str(e)
            logger.error(f"Robot connect failed: {e}. Retrying in {ROBOT_RETRY_S}s.")
            await asyncio.sleep(ROBOT_RETRY_S)

    # Stage 2: control bring-up (lease/E-Stop/power/stand). Non-fatal: if this
    # fails (e.g. a power fault), cameras/license/telemetry keep working and we
    # retry. bring_up_control is idempotent, so retries do not stack threads.
    while True:
        try:
            await asyncio.to_thread(controller.bring_up_control)
            spot_error = None
            logger.info("Robot control ready")
            return
        except Exception as e:
            spot_error = str(e)
            logger.error(
                f"Control bring-up failed: {e}. Cameras/license/telemetry still "
                f"available; retrying control in {ROBOT_RETRY_S}s."
            )
            await asyncio.sleep(ROBOT_RETRY_S)


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Start the robot connection in the background so the UI serves immediately,
    # online or not. Status reflects the live robot link.
    connect_task = asyncio.create_task(_robot_connect_loop())
    broadcast_task = asyncio.create_task(_status_broadcast_loop())

    yield

    connect_task.cancel()
    broadcast_task.cancel()
    logger.info("Shutting down Spot connection")
    if spot:
        spot.shutdown()


app = FastAPI(title="Spot Controller", lifespan=lifespan)


# ------------------------------------------------------------------
# WebSocket endpoint
# ------------------------------------------------------------------

async def _remove_client(websocket: WebSocket):
    connected_clients.discard(websocket)
    if spot:
        # Any disconnect — intentional or not — sits the robot
        await asyncio.to_thread(spot.safe_stop)
    logger.info(f"Client removed. Active clients: {len(connected_clients)}")


@app.websocket("/ws")
async def ws_control(websocket: WebSocket):
    await websocket.accept()
    connected_clients.add(websocket)
    logger.info(f"Controller connected: {websocket.client}. Active: {len(connected_clients)}")

    # Send current state immediately so UI can sync (works even if robot is offline).
    # Send the full log buffer here so startup lines (e.g. a power-on fault logged
    # before any browser connected) are visible without digging in the terminal.
    try:
        status = await _status_payload()
        await websocket.send_text(json.dumps({"event": "status", **status, "log_lines": log_buffer.recent(300)}))
    except Exception:
        pass

    try:
        while True:
            raw = await websocket.receive_text()
            msg: dict = json.loads(raw)
            cmd = msg.get("cmd")

            # Control commands need a connected robot. When offline, ack and ignore
            # so the UI stays responsive; the status feed shows the robot as offline.
            if cmd in ("control", "stand", "sit", "walk", "estop", "disconnect") and not spot:
                if cmd == "disconnect":
                    await websocket.send_text(json.dumps({"event": "disconnecting"}))
                    break
                await websocket.send_text(json.dumps({"ok": True, "robot_connected": False}))
                continue

            if cmd == "control":
                global _ws_control_counter
                roll_in    = float(msg.get("roll",  0.0))
                pitch_in   = float(msg.get("pitch", 0.0))
                walking_in = bool(msg.get("walking", False))
                _ws_control_counter += 1
                if _ws_control_counter % 20 == 1:   # ~1 Hz at 20 Hz send rate
                    import math as _math
                    logger.info(
                        f"WS IN  walking={walking_in}"
                        f"  roll={_math.degrees(roll_in):+.1f}°"
                        f"  pitch={_math.degrees(pitch_in):+.1f}°"
                        f"  vx={float(msg.get('vx',0)):+.2f}"
                        f"  vy={float(msg.get('vy',0)):+.2f}"
                        f"  vrot={float(msg.get('v_rot',0)):+.2f}"
                    )
                spot.update(
                    vx=float(msg.get("vx", 0.0)),
                    vy=float(msg.get("vy", 0.0)),
                    v_rot=float(msg.get("v_rot", 0.0)),
                    pitch=pitch_in,
                    roll=roll_in,
                    yaw_offset=float(msg.get("yaw_offset", 0.0)),
                    height=float(msg.get("height", 0.0)),
                    walking=walking_in,
                )

            elif cmd == "stand":
                logger.info("WS CMD  stand")
                await asyncio.to_thread(spot.stand_up)

            elif cmd == "sit":
                logger.info("WS CMD  sit")
                await asyncio.to_thread(spot.sit)

            elif cmd == "walk":
                logger.info("WS CMD  walk")
                spot.update(walking=True)

            elif cmd == "disconnect":
                # Operator-initiated disconnect: sit, then close
                logger.info("Operator disconnect requested")
                await asyncio.to_thread(spot.safe_stop)
                await websocket.send_text(json.dumps({"event": "disconnecting"}))
                break  # exits the loop → finally removes client

            elif cmd == "estop":
                spot.trigger_estop()
                await websocket.send_text(json.dumps({"event": "estop_triggered"}))
                continue

            elif cmd == "status":
                status = await _status_payload()
                await websocket.send_text(json.dumps({"event": "status", **status}))
                continue

            await websocket.send_text(json.dumps({"ok": True}))

    except WebSocketDisconnect:
        logger.warning("Client disconnected unexpectedly")
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
    finally:
        await _remove_client(websocket)


# ------------------------------------------------------------------
# REST endpoints
# ------------------------------------------------------------------

@app.get("/health")
async def health():
    return {"status": "ok"}


@app.get("/status")
async def status():
    # Always 200 with a payload so the UI can render and show robot link state.
    return await _status_payload()


@app.post("/recover")
async def recover_control():
    # Operator-triggered: clear clearable faults, power on, stand. Needs the robot
    # reachable (read-only connected); runs in a worker thread (power-on can take
    # ~20s). Serialized server-side against the auto-retry loop.
    if not spot or not spot.robot_connected:
        return JSONResponse({"ok": False, "error": "robot not reachable"}, status_code=503)
    result = await asyncio.to_thread(spot.recover_control)
    return JSONResponse(result, status_code=200 if result.get("ok") else 502)


@app.get("/license")
async def license_info():
    # A license read is passive: it only needs authentication, NOT a lease,
    # E-Stop, or motor power. Reuse the live control connection if it is up;
    # otherwise do an independent auth-only read so the license is still
    # readable before/without full bring-up.
    if spot and spot.robot_connected:
        result = await asyncio.to_thread(spot.get_license)
    else:
        username = os.environ.get("SPOT_USERNAME", "user")
        password = os.environ.get("SPOT_PASSWORD")
        if not password:
            return JSONResponse(
                {"error": "SPOT_PASSWORD not set"}, status_code=503
            )
        hostname, _ = await asyncio.to_thread(resolve_spot_hostname)
        result = await asyncio.to_thread(read_license, hostname, username, password)
    status_code = 502 if "error" in result else 200
    return JSONResponse(result, status_code=status_code)


# ------------------------------------------------------------------
# Camera feeds: base-unit cameras republished as MJPEG. Lease-free and
# decoupled from the control loop; each open feed polls frames in a worker
# thread so it never blocks the 20 Hz command loop.
# ------------------------------------------------------------------

CAMERA_FPS = 8   # poll cap per feed; lower if the control link gets laggy (control wins)


@app.get("/camera/{source}")
async def camera_feed(source: str):
    if not spot or source not in spot.CAMERA_SOURCES:
        return JSONResponse({"error": "unknown source"}, status_code=404)

    async def gen():
        period = 1.0 / CAMERA_FPS
        while True:
            # Threaded grab: never touches the command loop, lease, or E-stop.
            frame = await asyncio.to_thread(spot.get_camera_jpeg, source)
            if frame:
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                       + frame + b"\r\n")
            await asyncio.sleep(period)

    return StreamingResponse(
        gen(), media_type="multipart/x-mixed-replace; boundary=frame"
    )


# ------------------------------------------------------------------
# Static frontend — mounted last so API routes take priority
# ------------------------------------------------------------------

app.mount("/", StaticFiles(directory="../web", html=True), name="static")

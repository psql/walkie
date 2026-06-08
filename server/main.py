"""
FastAPI server — bridges WebSocket control messages to the Spot SDK.

Run:  uvicorn main:app --host 0.0.0.0 --port 8000 --reload
Then open http://localhost:8000 in a browser on the same Mac.
"""

import asyncio
import json
import logging
import os
from contextlib import asynccontextmanager
from typing import Optional, Set

from dotenv import load_dotenv
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles

from spot_client import SpotController

load_dotenv()

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-8s  %(name)s  %(message)s",
)
logger = logging.getLogger(__name__)

spot: Optional[SpotController] = None
connected_clients: Set[WebSocket] = set()


# ------------------------------------------------------------------
# Status broadcast — pushes robot telemetry to all clients at 1 Hz
# ------------------------------------------------------------------

async def _status_broadcast_loop():
    while True:
        await asyncio.sleep(1.0)
        if not spot or not connected_clients:
            continue
        try:
            status = await asyncio.to_thread(spot.get_full_status)
            msg = json.dumps({"event": "status", **status})
            dead: Set[WebSocket] = set()
            for ws in list(connected_clients):
                try:
                    await ws.send_text(msg)
                except Exception:
                    dead.add(ws)
            connected_clients -= dead
        except Exception as e:
            logger.warning(f"Status broadcast error: {e}")


@asynccontextmanager
async def lifespan(app: FastAPI):
    global spot
    hostname = os.environ["SPOT_HOSTNAME"]
    username = os.environ.get("SPOT_USERNAME", "user")
    password = os.environ["SPOT_PASSWORD"]

    logger.info(f"Connecting to Spot at {hostname}")
    spot = SpotController(hostname)
    spot.authenticate(username, password)
    spot.setup()

    broadcast_task = asyncio.create_task(_status_broadcast_loop())

    yield

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

    # Send current state immediately so UI can sync
    try:
        status = await asyncio.to_thread(spot.get_full_status)
        await websocket.send_text(json.dumps({"event": "status", **status}))
    except Exception:
        pass

    try:
        while True:
            raw = await websocket.receive_text()
            msg: dict = json.loads(raw)
            cmd = msg.get("cmd")

            if cmd == "control":
                spot.update(
                    vx=float(msg.get("vx", 0.0)),
                    vy=float(msg.get("vy", 0.0)),
                    v_rot=float(msg.get("v_rot", 0.0)),
                    pitch=float(msg.get("pitch", 0.0)),
                    roll=float(msg.get("roll", 0.0)),
                    yaw_offset=float(msg.get("yaw_offset", 0.0)),
                    height=float(msg.get("height", 0.0)),
                    walking=bool(msg.get("walking", False)),
                )

            elif cmd == "stand":
                await asyncio.to_thread(spot.stand_up)

            elif cmd == "sit":
                await asyncio.to_thread(spot.sit)

            elif cmd == "walk":
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
                status = await asyncio.to_thread(spot.get_full_status)
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
    if not spot:
        return JSONResponse({"connected": False}, status_code=503)
    return await asyncio.to_thread(spot.get_full_status)


# ------------------------------------------------------------------
# Static frontend — mounted last so API routes take priority
# ------------------------------------------------------------------

app.mount("/", StaticFiles(directory="../web", html=True), name="static")

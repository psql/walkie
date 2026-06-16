# walkie — Spot Controller

Web-based controller for Boston Dynamics Spot with posed-walk support.
The full interactive setup guide is also served at **http://localhost:8000/setup** once the server is running.

---

## What makes this different

Standard Spot controllers reset the body to perfectly upright before walking.
Walkie holds your pitch **and** roll through the full gait cycle, so the body
keeps its personality while stepping. Set the pose sliders, hit Walk, and the
attitude comes with it.

### Walk backends

The mobility velocity path (`synchro_velocity_command` + `BodyControlParams`)
holds pitch and height while stepping, but Spot's balancer reserves roll for
dynamic balance and washes out a commanded roll. To hold roll too, Walkie drives
walking with a **Custom Gait** choreography (Boston Dynamics' Choreography API),
which carries a live body rotation offset through the gait.

Set the backend at the top of `server/spot_client.py`:

```python
WALK_BACKEND = "custom_gait"   # "custom_gait" | "mobility"
```

- `custom_gait` (default) — holds pitch + roll while walking. **Requires a
  choreography license on the robot** (checked at startup; the server fails fast
  with a clear message if it is missing). Driven via `server/custom_gait.py`.
- `mobility` — the legacy velocity path. Holds pitch only; roll washes out.
  Kept as a known-good fallback and for comparison.

The active backend is shown as a chip in the header (`CGAIT●` when a custom gait
is live, `MOBILITY` otherwise) and in the `walk_backend` status field.

---

## Quick-start (do steps 1–3 while you still have internet)

### 1. Install dependencies

```bash
cd ~/dev/walkie/server
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

All packages are cached in `.venv` — no internet needed after this.

### 2. Create `.env`

```bash
cp server/.env.example server/.env
```

Edit `server/.env`:

```
SPOT_HOSTNAME=192.168.80.3
SPOT_USERNAME=user
SPOT_PASSWORD=<your password>
```

### 3. Connect to Spot's WiFi

Credentials are on the sticker inside the battery bay.
Once connected you lose internet — that's expected. Everything runs locally.

```bash
ping 192.168.80.3   # should respond before starting the server
```

### 4. Power on the robot

Use the physical power button or the tablet app. Place the robot on flat ground
with room to stand. The server calls `power_on()` and `blocking_stand()` on
startup — **the robot stands automatically**. Clear the area first.

### 5. Start the server

```bash
cd ~/dev/walkie/server
source .venv/bin/activate
uvicorn main:app --host 0.0.0.0 --port 8000
```

Expected output:
```
INFO  Connecting to Spot at 192.168.80.3
INFO  Authenticated and time-synced
INFO  Custom Gait preflight OK: choreography license enabled, client ready
INFO  ============================================================
INFO  SPOT READY — walk backend: custom_gait
INFO  ============================================================
INFO  Uvicorn running on http://0.0.0.0:8000
```

If the robot has no choreography license, the server stops at startup with:
`Choreography license is NOT enabled on this robot...`. Either install a
license or set `WALK_BACKEND = "mobility"` in `server/spot_client.py`.

### 6. Open the controller

Open **http://localhost:8000** in Safari or Chrome.
Hit **Connect** — green dot confirms the WebSocket is live.
Hit **http://localhost:8000/setup** for the full interactive guide.

---

## Cameras

The base-unit grayscale fisheye cameras are shown as live tiles in the UI. These
come from the image service (request-response, republished to the browser as
MJPEG), need **no lease**, and are fully decoupled from the control loop and
E-stop, so they cannot affect robot control. All frame grabs run in worker
threads.

- The two **forward** feeds (front-left + front-right driver view) are always on.
- **Left**, **Right**, and **Back** tiles toggle on/off. Toggling a tile off closes
  its stream so it stops using robot wifi.
- **Cameras: On/Off** master toggle drops all feeds instantly if the link gets tight.
- Defaults: `CAMERA_FPS = 8` (in `server/main.py`) and JPEG `quality = 50`. Lower
  either if control latency suffers; control always wins over image smoothness.
- Front and right feeds are rotated upright in the browser via CSS (angles negated
  from the official `get_image --auto-rotate` map: front-left 78°, front-right 102°,
  right 180°).

## Robot link and offline behavior

The web UI loads and stays reachable **whether or not the robot is connected**.
The server connects to the robot in the background and retries every 10 s while
the link is down. The header shows two independent indicators:

- The **dot** is the browser-to-server link (the WebSocket / Connect button).
- The **ROBOT** chip is the server-to-robot link: green `ROBOT ●` when connected,
  red `ROBOT ○` when offline (hover for the reason). Control commands are ignored
  while offline; they resume automatically once the robot connects.

---

## Controls

| Input | Action |
|-------|--------|
| Left joystick | Move (forward / back / strafe) |
| Right joystick | Rotate |
| Pitch slider | Tilt body — persists while walking |
| Roll slider | Bank body — persists while walking |
| Height slider | Body height offset |
| Yaw slider | Body yaw (stand mode only) |
| Stand / Sit / Walk | Mode — pose is retained across transitions |
| Disconnect | Sit robot and release control |
| E-STOP | Cut motor power immediately |

**Keyboard:** WASD move · QE turn · ↑↓ pitch · ←→ roll · +- height · Space walk · X sit · Esc E-stop

**Gamepad (PS5/Xbox):** Left stick move · Right stick rotate · L2/R2 roll · Cross/A stand · Triangle/Y walk · Square/X sit · Circle/B E-stop

---

## Safety

- **Disconnect** (button or tab close): robot sits down before connection closes.
- **Unexpected drop** (WiFi blip): server's disconnect handler sits the robot.
- **Keepalive**: robot does a controlled motors-off if server goes silent for 30 s.
- **Command timeout**: each velocity command expires in 200 ms — robot stops if the loop stalls.
- All inputs are server-side clamped: ±28° pitch, ±17° roll, −10/+15 cm height.
- **In custom-gait walk**, body offsets are additionally clamped tighter (±14° pitch,
  ±11° roll) and to the robot's live gait limits, since offsets carried through a
  step cycle have their own stability bounds. Transitions stop the gait before
  sitting or standing, so choreography and the mobility path never run at once.

---

## Troubleshooting

**`KeepaliveMotorsOffError`** — Tablet keepalive policy is blocking power-on. The server clears it automatically; retry once.

**`ResourceAlreadyClaimedError`** — Another client holds the lease. Release from the tablet app or restart the robot.

**`Motor power failed`** — Robot must be powered on before starting the server.

**Disconnected dot** — Check you are on Spot's WiFi (`ping 192.168.80.3`), not your home network.

**Body pose resets while walking** — Confirm Walk mode is active (green badge). Stand mode uses a separate command path.

**`Choreography license is NOT enabled`** — The robot lacks a choreography license, required by the `custom_gait` backend. Install one, or set `WALK_BACKEND = "mobility"` in `server/spot_client.py` (roll will not hold while walking).

**Roll doesn't hold while walking** — Check the header chip reads `CGAIT●` (custom gait live). If it reads `MOBILITY`, switch `WALK_BACKEND` to `custom_gait`. Watch the server log for `ROLL DRIFT` warnings.

---

## Road to iOS

The Python server is the stable backend. The iOS app replaces the HTML frontend —
connect `URLSessionWebSocketTask` to `ws://<host>:8000/ws` and send the same JSON messages.
Run the server on a Mac mini or payload computer on Spot's own WiFi.

---

## Credits

Built by [Lingonberry Jam PBC](https://lingonberry.org).

# walkie — Spot Controller

Web-based controller for Boston Dynamics Spot with posed-walk support.
The full interactive setup guide is also served at **http://localhost:8000/setup** once the server is running.

---

## What makes this different

Standard Spot controllers reset the body to perfectly upright before walking.
Walkie uses `BodyControlParams.base_offset_rt_footprint` — a constant SE3Trajectory
offset in the footprint frame — so the body holds your pitch/roll/height through
the full gait cycle. Set the pose sliders, hit Walk, and the personality comes with it.

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
INFO  Robot standing — starting command loop
INFO  Uvicorn running on http://0.0.0.0:8000
```

### 6. Open the controller

Open **http://localhost:8000** in Safari or Chrome.
Hit **Connect** — green dot confirms the WebSocket is live.
Hit **http://localhost:8000/setup** for the full interactive guide.

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

---

## Troubleshooting

**`KeepaliveMotorsOffError`** — Tablet keepalive policy is blocking power-on. The server clears it automatically; retry once.

**`ResourceAlreadyClaimedError`** — Another client holds the lease. Release from the tablet app or restart the robot.

**`Motor power failed`** — Robot must be powered on before starting the server.

**Disconnected dot** — Check you are on Spot's WiFi (`ping 192.168.80.3`), not your home network.

**Body pose resets while walking** — Confirm Walk mode is active (green badge). Stand mode uses a separate command path.

---

## Road to iOS

The Python server is the stable backend. The iOS app replaces the HTML frontend —
connect `URLSessionWebSocketTask` to `ws://<host>:8000/ws` and send the same JSON messages.
Run the server on a Mac mini or payload computer on Spot's own WiFi.

---

## Credits

Built by [Lingonberry Jam PBC](https://lingonberryjam.com).

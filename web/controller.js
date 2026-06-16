"use strict";

const WS_URL = `ws://${location.host}/ws`;
const SEND_HZ = 20;
const DEG = Math.PI / 180;
const RAD = 180 / Math.PI;

// -----------------------------------------------------------------------
// State
// -----------------------------------------------------------------------

const state = {
  // Mode: "sit" | "stand" | "walk"
  mode: "sit",
  // Joystick axes (-1..1)
  leftX: 0, leftY: 0,
  rightX: 0, rightY: 0,
  // Body pose (degrees / cm)
  pitch: 0, roll: 0, yaw: 0, height: 0,
};

// -----------------------------------------------------------------------
// WebSocket + connect/disconnect
// -----------------------------------------------------------------------

let ws = null;
let intentionalDisconnect = false;  // true → don't auto-reconnect
let reconnectTimer = null;

function connect() {
  if (ws && ws.readyState === WebSocket.OPEN) return;
  intentionalDisconnect = false;
  clearTimeout(reconnectTimer);

  ws = new WebSocket(WS_URL);

  ws.onopen = () => {
    setConnectedUI(true);
    startSendLoop();
  };

  ws.onclose = () => {
    setConnectedUI(false);
    stopSendLoop();
    if (!intentionalDisconnect) {
      // Unexpected drop — retry
      reconnectTimer = setTimeout(connect, 2000);
    }
  };

  ws.onerror = () => { ws.close(); };

  ws.onmessage = ({ data }) => {
    const msg = JSON.parse(data);
    if (msg.event === "status")         updateStatus(msg);
    else if (msg.event === "estop_triggered") onEstop();
    else if (msg.event === "disconnecting")   onServerDisconnect();
  };
}

function disconnect() {
  intentionalDisconnect = true;
  stopSendLoop();
  send({ cmd: "disconnect" });   // server sits robot, then we close
  // ws.onclose will fire after server closes → setConnectedUI(false)
}

function send(obj) {
  if (ws?.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(obj));
  }
}

function onServerDisconnect() {
  // Server confirmed disconnect after sitting
  ws.close();
  setConnectedUI(false);
}

function onEstop() {
  document.getElementById("conn-indicator").textContent = "⚡";
  setMode("sit");
}

// -----------------------------------------------------------------------
// Send loop — 20 Hz control commands
// -----------------------------------------------------------------------

let sendLoop = null;

function startSendLoop() {
  if (sendLoop) return;
  sendLoop = setInterval(flushState, 1000 / SEND_HZ);
}
function stopSendLoop() {
  clearInterval(sendLoop);
  sendLoop = null;
}

function flushState() {
  const walking = state.mode === "walk";
  const sitting = state.mode === "sit";

  // In sit mode, keep sending sit so the command loop can't be overridden
  if (sitting) {
    send({ cmd: "control", walking: false, sitting: true,
           vx: 0, vy: 0, v_rot: 0,
           pitch: 0, roll: 0, yaw_offset: 0, height: 0 });
    return;
  }

  send({
    cmd: "control",
    walking,
    sitting: false,
    vx:          -state.leftY  * 1.5,
    vy:          -state.leftX  * 0.5,
    v_rot:       -state.rightX * 1.0,
    pitch:        state.pitch  * DEG,
    roll:         state.roll   * DEG,
    yaw_offset:   state.yaw    * DEG,
    height:       state.height / 100,
  });
}

// -----------------------------------------------------------------------
// Mode state machine
// -----------------------------------------------------------------------

function setMode(newMode) {
  state.mode = newMode;

  const badge = document.getElementById("mode-badge");
  badge.className = `badge ${newMode}`;
  badge.textContent = newMode.toUpperCase();

  document.getElementById("stand-btn").classList.toggle("active", newMode === "stand");
  document.getElementById("sit-btn").classList.toggle("active",   newMode === "sit");
  document.getElementById("walk-btn").classList.toggle("active",  newMode === "walk");

  // Yaw slider only useful in stand/pose
  document.getElementById("yaw-label").classList.toggle("active", newMode === "stand");

  // Zero joystick when entering sit/stand so robot doesn't lurch
  if (newMode !== "walk") {
    state.leftX = state.leftY = state.rightX = state.rightY = 0;
  }
}

// -----------------------------------------------------------------------
// UI: connected / disconnected
// -----------------------------------------------------------------------

function setConnectedUI(connected) {
  const dot = document.getElementById("conn-indicator");
  dot.className = `indicator ${connected ? "connected" : "disconnected"}`;
  dot.textContent = "●";

  const btn = document.getElementById("conn-btn");
  if (connected) {
    btn.textContent = "Disconnect";
    btn.className = "conn-btn connected";
  } else {
    btn.textContent = "Connect";
    btn.className = "conn-btn disconnected";
    setMode("sit");   // reflect that robot is (or will be) sitting
    // Server link is down, so the robot link is unknown: show offline until a
    // fresh status arrives.
    setRobotStatus(false, "server unreachable");
  }
}

// -----------------------------------------------------------------------
// Status display
// -----------------------------------------------------------------------

function setRobotStatus(connected, error) {
  const el = document.getElementById("robot-status");
  el.className = `robot-status ${connected ? "online" : "offline"}`;
  el.textContent = connected ? "ROBOT ●" : "ROBOT ○";
  el.title = connected
    ? "Server link to robot: connected"
    : `Server link to robot: offline${error ? " (" + error + ")" : ""}`;
}

function updateStatus(s) {
  // Robot link state (server to robot), distinct from the browser to server dot
  if ("robot_connected" in s) setRobotStatus(s.robot_connected, s.robot_error);

  // Sync mode badge with server state if it differs (e.g. after reconnect)
  if (s.sitting && state.mode !== "sit")   setMode("sit");
  else if (s.walking && state.mode !== "walk") setMode("walk");
  else if (!s.sitting && !s.walking && state.mode === "sit") {
    // Server is standing but client thinks sitting — happens on first connect
    setMode("stand");
  }

  // Walk backend chip: shows which paradigm the server is using, and (for
  // custom gait) whether the gait is actually running.
  if (s.walk_backend != null) {
    const chip = document.getElementById("backend-chip");
    const custom = s.walk_backend === "custom_gait";
    const live = custom && s.gait_running;
    chip.textContent = custom ? (live ? "CGAIT●" : "CGAIT") : "MOBILITY";
    chip.className = `backend-chip${custom ? " custom" : " mobility"}${live ? " live" : ""}`;
  }

  // Battery
  if (s.battery_pct != null) {
    const pct = s.battery_pct;
    const fill = document.getElementById("batt-fill");
    const txt  = document.getElementById("batt-pct");
    fill.style.width = `${pct}%`;
    fill.className = `batt-fill${pct < 20 ? " danger" : pct < 40 ? " warn" : ""}`;
    txt.textContent = `${pct}%`;
    txt.className   = `stat-val${pct < 20 ? " danger" : pct < 40 ? " warn" : " ok"}`;
  }

  // Runtime
  const rt = document.getElementById("batt-runtime");
  if (s.battery_runtime_min != null) {
    const h = Math.floor(s.battery_runtime_min / 60);
    const m = s.battery_runtime_min % 60;
    rt.textContent = h > 0 ? `${h}h ${m}m` : `${m}m`;
  } else {
    rt.textContent = "--";
  }

  // Motor state
  const ms = document.getElementById("motor-state");
  ms.textContent  = s.motor_state ?? "--";
  ms.className    = `stat-val${s.motor_state === "ON" ? " ok" : s.motor_state === "ERROR" ? " danger" : ""}`;

  // Behavior state
  const bs = document.getElementById("behavior-state");
  bs.textContent = s.behavior_state ?? "--";
  bs.className   = "stat-val" + (
    s.behavior_state === "STEPPING"  ? " ok"   :
    s.behavior_state === "STANDING"  ? " ok"   :
    s.behavior_state === "NOT_READY" ? " warn" : ""
  );

  // E-stop
  const es = document.getElementById("estop-state");
  const estopped = s.estopped ?? false;
  es.textContent = estopped ? "ESTOPPED" : "CLEAR";
  es.className   = `stat-val${estopped ? " danger" : " ok"}`;

  // Foot contact
  const footIds = ["foot-fl","foot-fr","foot-rl","foot-rr"];
  if (Array.isArray(s.foot_contact)) {
    s.foot_contact.forEach((c, i) => {
      document.getElementById(footIds[i])?.classList.toggle("contact", c);
    });
  }

  // Velocity readout
  document.getElementById("v-vx").textContent   = (s.vx    ?? 0).toFixed(2);
  document.getElementById("v-vy").textContent   = (s.vy    ?? 0).toFixed(2);
  document.getElementById("v-vrot").textContent = (s.v_rot ?? 0).toFixed(2);

  // Faults
  const allFaults = [
    ...(s.behavior_faults ?? []).map(f => `BEHAVIOR: ${f}`),
    ...(s.system_faults   ?? []).map(f => `SYSTEM: ${f}`),
  ];
  const fb = document.getElementById("fault-box");
  fb.textContent = allFaults.join("\n");
  fb.classList.toggle("hidden", allFaults.length === 0);

  // Commanded vs actual pose diagnostic rows
  const fmtPR = (p, r) => {
    const sign = v => (v >= 0 ? "+" : "") + v.toFixed(1);
    return `${sign(p)}° / ${sign(r)}°`;
  };
  if (s.pitch != null) {
    document.getElementById("cmd-pr").textContent =
      fmtPR(+(s.pitch * RAD).toFixed(1), +(s.roll ?? 0) * RAD);
  }
  if (s.pitch_actual != null) {
    const el = document.getElementById("actual-pr");
    el.textContent = fmtPR(s.pitch_actual, s.roll_actual ?? 0);
    const errR = Math.abs((s.roll_actual ?? 0) - (s.roll ?? 0) * RAD);
    el.className = `stat-val${errR > 3 ? " warn" : " ok"}`;
  }

  // 3D orientation from server-confirmed commanded values
  if (s.pitch != null) updateOrientation(s.pitch, s.roll ?? 0, s.yaw_offset ?? 0, s.height ?? 0);

  // Server logs
  if (Array.isArray(s.log_lines) && s.log_lines.length) appendLogs(s.log_lines);
}

// -----------------------------------------------------------------------
// 3D Orientation widget
// -----------------------------------------------------------------------

const orientStage = document.getElementById("orient-stage");
const VIEW_PITCH = -30;
const VIEW_YAW   =  22;

function updateOrientation(pitch_rad, roll_rad, yaw_rad, height_m) {
  const p = -(pitch_rad * RAD);
  const r = -(roll_rad  * RAD);
  const y =   yaw_rad   * RAD;
  orientStage.style.transform =
    `rotateY(${VIEW_YAW + y}deg) rotateX(${VIEW_PITCH + p}deg) rotateZ(${r}deg)`;
  document.getElementById("o-pitch").textContent  = `${(pitch_rad * RAD).toFixed(1)}°`;
  document.getElementById("o-roll").textContent   = `${(roll_rad  * RAD).toFixed(1)}°`;
  document.getElementById("o-height").textContent = `${Math.round(height_m * 100)}cm`;
}

function orientFromSliders() {
  updateOrientation(
    state.pitch  * DEG,
    state.roll   * DEG,
    state.yaw    * DEG,
    state.height / 100,
  );
}

// -----------------------------------------------------------------------
// UI button wiring
// -----------------------------------------------------------------------

// Connect / Disconnect
document.getElementById("conn-btn").addEventListener("click", () => {
  if (ws?.readyState === WebSocket.OPEN) {
    disconnect();
  } else {
    connect();
  }
});

// Stand
document.getElementById("stand-btn").addEventListener("click", () => {
  setMode("stand");
  send({ cmd: "stand" });
});

// Sit
document.getElementById("sit-btn").addEventListener("click", () => {
  setMode("sit");
  send({ cmd: "sit" });
});

// Walk
document.getElementById("walk-btn").addEventListener("click", () => {
  setMode("walk");
  send({ cmd: "walk" });
});

// E-stop
document.getElementById("estop-btn").addEventListener("click", () => {
  send({ cmd: "estop" });
});

// Neutral pose — zero all sliders instantly
document.getElementById("neutral-btn").addEventListener("click", () => {
  sync("pitch",  0, "pitch-val",  v => `${v.toFixed(1)}°`);
  sync("roll",   0, "roll-val",   v => `${v.toFixed(1)}°`);
  sync("height", 0, "height-val", v => `${v.toFixed(0)} cm`);
  sync("yaw",    0, "yaw-val",    v => `${v.toFixed(1)}°`);
});

// Sliders
function bindSlider(id, stateKey, displayId, fmt) {
  const el   = document.getElementById(id);
  const disp = document.getElementById(displayId);
  el.addEventListener("input", () => {
    state[stateKey] = parseFloat(el.value);
    disp.textContent = fmt(state[stateKey]);
    orientFromSliders();
  });
}
bindSlider("pitch",  "pitch",  "pitch-val",  v => `${v.toFixed(1)}°`);
bindSlider("roll",   "roll",   "roll-val",   v => `${v.toFixed(1)}°`);
bindSlider("height", "height", "height-val", v => `${v.toFixed(0)} cm`);
bindSlider("yaw",    "yaw",    "yaw-val",    v => `${v.toFixed(1)}°`);

// -----------------------------------------------------------------------
// Virtual joysticks
// -----------------------------------------------------------------------

const joyLeft = nipplejs.create({
  zone: document.getElementById("joy-left"),
  mode: "static",
  position: { left: "50%", top: "50%" },
  color: "rgba(0, 198, 255, 0.8)",
  size: 120,
});
const joyRight = nipplejs.create({
  zone: document.getElementById("joy-right"),
  mode: "static",
  position: { left: "50%", top: "50%" },
  color: "rgba(0, 232, 122, 0.8)",
  size: 120,
});

function bindJoy(manager, xKey, yKey) {
  manager.on("move", (_, data) => { state[xKey] = data.vector.x; state[yKey] = data.vector.y; });
  manager.on("end",  ()        => { state[xKey] = 0;             state[yKey] = 0; });
}
bindJoy(joyLeft,  "leftX", "leftY");
bindJoy(joyRight, "rightX", "rightY");

// -----------------------------------------------------------------------
// Keyboard
// -----------------------------------------------------------------------

const KV = 0.6, KP = 1.0;
const keysDown = new Set();

document.addEventListener("keydown", e => {
  if (e.repeat) return;
  keysDown.add(e.key.toLowerCase());
  switch (e.key) {
    case " ":      e.preventDefault(); toggleWalk(); break;
    case "x": case "X": document.getElementById("sit-btn").click(); break;
    case "Escape": send({ cmd: "estop" }); break;
  }
  updateKeyVelocity();
  updateKeyPose(e.key);
});
document.addEventListener("keyup", e => { keysDown.delete(e.key.toLowerCase()); updateKeyVelocity(); });

function toggleWalk() {
  state.mode === "walk" ? document.getElementById("stand-btn").click()
                        : document.getElementById("walk-btn").click();
}

function updateKeyVelocity() {
  state.leftY  = ((keysDown.has("s") ? 1 : 0) - (keysDown.has("w") ? 1 : 0)) * KV;
  state.leftX  = ((keysDown.has("d") ? 1 : 0) - (keysDown.has("a") ? 1 : 0)) * KV;
  state.rightX = ((keysDown.has("e") ? 1 : 0) - (keysDown.has("q") ? 1 : 0)) * KV;
}

function updateKeyPose(key) {
  const cl = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const map = {
    ArrowUp:    () => sync("pitch",  cl(state.pitch  + KP, -28, 28), "pitch-val",  v => `${v.toFixed(1)}°`),
    ArrowDown:  () => sync("pitch",  cl(state.pitch  - KP, -28, 28), "pitch-val",  v => `${v.toFixed(1)}°`),
    ArrowLeft:  () => sync("roll",   cl(state.roll   - KP, -17, 17), "roll-val",   v => `${v.toFixed(1)}°`),
    ArrowRight: () => sync("roll",   cl(state.roll   + KP, -17, 17), "roll-val",   v => `${v.toFixed(1)}°`),
    "+":        () => sync("height", cl(state.height + 1,  -10, 15), "height-val", v => `${v.toFixed(0)} cm`),
    "-":        () => sync("height", cl(state.height - 1,  -10, 15), "height-val", v => `${v.toFixed(0)} cm`),
  };
  if (map[key]) map[key]();
}

function sync(id, value, displayId, fmt) {
  state[id] = value;
  document.getElementById(id).value = value;
  document.getElementById(displayId).textContent = fmt(value);
  orientFromSliders();
}

// -----------------------------------------------------------------------
// Gamepad
// -----------------------------------------------------------------------

const DEADZONE = 0.12;
const dz = v => Math.abs(v) < DEADZONE ? 0 : v;
let gamepadLoop = null;

window.addEventListener("gamepadconnected",    () => { if (!gamepadLoop) gamepadLoop = setInterval(pollGamepad, 1000/SEND_HZ); });
window.addEventListener("gamepaddisconnected", () => { clearInterval(gamepadLoop); gamepadLoop = null; state.leftX = state.leftY = state.rightX = state.rightY = 0; });

function pollGamepad() {
  const gp = navigator.getGamepads()[0];
  if (!gp) return;
  state.leftX  = dz(gp.axes[0]);
  state.leftY  = dz(gp.axes[1]);
  state.rightX = dz(gp.axes[2]);
  if (gp.buttons[0]?.pressed) document.getElementById("stand-btn").click();  // Cross/A
  if (gp.buttons[3]?.pressed) document.getElementById("walk-btn").click();   // Triangle/Y
  if (gp.buttons[2]?.pressed) document.getElementById("sit-btn").click();    // Square/X
  if (gp.buttons[1]?.pressed) send({ cmd: "estop" });                        // Circle/B
  const l2 = gp.buttons[6]?.value ?? 0, r2 = gp.buttons[7]?.value ?? 0;
  if (Math.abs(l2 - r2) > 0.05) sync("roll", Math.max(-17, Math.min(17, (r2-l2)*17)), "roll-val", v => `${v.toFixed(1)}°`);
}

// -----------------------------------------------------------------------
// Log panel
// -----------------------------------------------------------------------

let _logSeen = new Set();   // dedup lines already rendered

function appendLogs(lines) {
  const container = document.getElementById("log-lines");
  let added = false;
  for (const line of lines) {
    if (_logSeen.has(line)) continue;
    _logSeen.add(line);
    const el = document.createElement("div");
    el.textContent = line;
    if (/ERROR/.test(line))   el.className = "log-line-error";
    else if (/WARNING/.test(line)) el.className = "log-line-warn";
    container.appendChild(el);
    added = true;
  }
  if (added) {
    // Keep at most 200 lines in DOM
    while (container.childElementCount > 200) container.firstElementChild.remove();
    container.scrollTop = container.scrollHeight;
  }
}

document.getElementById("log-toggle").addEventListener("click", () => {
  const body  = document.getElementById("log-body");
  const arrow = document.getElementById("log-toggle-arrow");
  const open  = !body.classList.contains("hidden");
  body.classList.toggle("hidden", open);
  arrow.textContent = open ? "▶" : "▼";
  document.getElementById("log-toggle").setAttribute("aria-expanded", String(!open));
  if (!open) document.getElementById("log-lines").scrollTop = document.getElementById("log-lines").scrollHeight;
});

document.getElementById("log-copy").addEventListener("click", () => {
  const lines = document.getElementById("log-lines");
  const text  = Array.from(lines.children).map(el => el.textContent).join("\n");
  navigator.clipboard.writeText(text).then(() => {
    const btn = document.getElementById("log-copy");
    btn.textContent = "copied!";
    btn.classList.add("copied");
    setTimeout(() => { btn.textContent = "copy"; btn.classList.remove("copied"); }, 1500);
  }).catch(() => {
    // Fallback: select all text in the log box
    const range = document.createRange();
    range.selectNodeContents(lines);
    window.getSelection().removeAllRanges();
    window.getSelection().addRange(range);
  });
});

document.getElementById("log-clear").addEventListener("click", () => {
  document.getElementById("log-lines").innerHTML = "";
  _logSeen.clear();
});

// -----------------------------------------------------------------------
// Cameras: MJPEG <img> feeds, lease-free and independent of control.
// Forward driver view (front L + R) is always on; side/rear toggle.
// Clearing src closes the MJPEG connection so a hidden feed frees bandwidth.
// -----------------------------------------------------------------------

const CAM_FORWARD = ["frontleft_fisheye_image", "frontright_fisheye_image"];
const camActive = new Set();   // sources currently streaming
let camMasterOn = true;

const camUrl = src => `/camera/${src}`;
const camImg = src => document.querySelector(`.cam-tile[data-src="${src}"] .cam-img`);

function startCam(src) {
  const img = camImg(src);
  if (!img) return;
  camActive.add(src);
  // Cache-bust so a reconnect always opens a fresh stream rather than a dead one.
  img.src = camUrl(src) + `?t=${Date.now()}`;
}

function stopCam(src) {
  const img = camImg(src);
  if (!img) return;
  camActive.delete(src);
  img.removeAttribute("src");   // closes the MJPEG connection (frees robot wifi)
}

// Side/rear toggle: show the tile and stream it (when master is on), or hide + stop.
function toggleOptional(src, btn) {
  const tile = document.querySelector(`.cam-tile[data-src="${src}"]`);
  const show = tile.classList.contains("hidden");
  tile.classList.toggle("hidden", !show);
  btn.classList.toggle("active", show);
  if (show && camMasterOn) startCam(src);
  else stopCam(src);
}

// Master switch: drop or restore every feed instantly if wifi gets tight.
function setCamMaster(on) {
  camMasterOn = on;
  const btn = document.getElementById("cam-master");
  btn.textContent = on ? "Cameras: On" : "Cameras: Off";
  btn.classList.toggle("off", !on);
  if (on) {
    CAM_FORWARD.forEach(startCam);
    document.querySelectorAll(".cam-tile.opt:not(.hidden)")
      .forEach(t => startCam(t.dataset.src));
  } else {
    [...camActive].forEach(stopCam);
  }
}

document.querySelectorAll(".cam-toggle").forEach(btn => {
  btn.addEventListener("click", () => toggleOptional(btn.dataset.src, btn));
});
document.getElementById("cam-master").addEventListener("click", () => setCamMaster(!camMasterOn));

// Reconnect a dropped feed (wifi blip) without crashing: retry while still active.
document.querySelectorAll(".cam-img").forEach(img => {
  img.addEventListener("error", () => {
    const src = img.closest(".cam-tile").dataset.src;
    if (!camMasterOn || !camActive.has(src)) return;
    setTimeout(() => { if (camActive.has(src)) startCam(src); }, 1500);
  });
});

// -----------------------------------------------------------------------
// Boot
// -----------------------------------------------------------------------

orientFromSliders();
setCamMaster(true);   // forward feeds start immediately, independent of Connect
connect();

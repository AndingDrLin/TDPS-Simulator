const canvas = document.getElementById("simCanvas");
const ctx = canvas.getContext("2d");
const wheelGrid = document.getElementById("wheelGrid");
const metrics = document.getElementById("metrics");
const codeInput = document.getElementById("codeInput");
const codeStatus = document.getElementById("codeStatus");
const memoryView = document.getElementById("memoryView");
const debugLog = document.getElementById("debugLog");
const clearDebug = document.getElementById("clearDebug");
const loadLfV1Template = document.getElementById("loadLfV1Template");
const camOffsetX = document.getElementById("camOffsetX");
const camOffsetY = document.getElementById("camOffsetY");
const camZoom = document.getElementById("camZoom");
const camOffsetXVal = document.getElementById("camOffsetXVal");
const camOffsetYVal = document.getElementById("camOffsetYVal");
const camZoomVal = document.getElementById("camZoomVal");
const camFollowToggle = document.getElementById("camFollowToggle");
const camReset = document.getElementById("camReset");
const cfgTrackWidth = document.getElementById("cfgTrackWidth");
const cfgWheelBase = document.getElementById("cfgWheelBase");
const cfgMaxWheelSpeed = document.getElementById("cfgMaxWheelSpeed");
const cfgWheelVisualScale = document.getElementById("cfgWheelVisualScale");
const cfgTrailPoints = document.getElementById("cfgTrailPoints");
const cfgMaxDt = document.getElementById("cfgMaxDt");
const cfgTrackPreset = document.getElementById("cfgTrackPreset");
const cfgLineWidth = document.getElementById("cfgLineWidth");
const cfgSvgPath = document.getElementById("cfgSvgPath");
const cfgSensors = document.getElementById("cfgSensors");
const applyTrackSensors = document.getElementById("applyTrackSensors");
const startTrackDraw = document.getElementById("startTrackDraw");
const finishTrackDraw = document.getElementById("finishTrackDraw");
const clearTrackDraw = document.getElementById("clearTrackDraw");
const trackStatus = document.getElementById("trackStatus");
const autoTestDuration = document.getElementById("autoTestDuration");
const autoTestDt = document.getElementById("autoTestDt");
const autoTestLineThreshold = document.getElementById("autoTestLineThreshold");
const runAutoTest = document.getElementById("runAutoTest");
const stopAutoTest = document.getElementById("stopAutoTest");
const copyAutoTestReport = document.getElementById("copyAutoTestReport");
const autoTestStatus = document.getElementById("autoTestStatus");
const autoTestReport = document.getElementById("autoTestReport");

const wheelNames = ["FL", "FR", "RL", "RR"];
const wheelValueEls = {};
let codeEditor = null;

const LF_V1_TEMPLATE = `// LF v1 compatibility template (ported from ../firmware).
// Expects 8 line sensors in order S0..S7 (left -> right).

const CFG = {
  control_period_ms: 10,
  auto_start_delay_ms: 1200,
  calibration_duration_ms: 3000,
  calibration_switch_interval_ms: 300,
  calibration_spin_speed: 180,
  sensor_filter_alpha: 0.35,
  line_detect_min_sum: 700,
  sensor_weights: [-1500, -500, 500, 1500],
  kp: 0.42,
  ki: 0.0,
  kd: 1.65,
  base_speed: 360,
  max_correction: 300,
  max_motor_cmd: 900,
  motor_deadband: 120,
  recover_turn_speed: 220,
  recover_timeout_ms: 900,
};

const STATE = {
  WAIT_START: 1,
  CALIBRATING: 2,
  RUNNING: 3,
  RECOVERING: 4,
  STOPPED: 5,
  FAULT: 6,
};

function clampI(value, minV, maxV) {
  return Math.min(maxV, Math.max(minV, value));
}

function createInitialCtx(nowMs) {
  return {
    state: STATE.WAIT_START,
    boot_ms: nowMs,
    last_step_ms: nowMs,
    calib_start_ms: 0,
    recover_start_ms: 0,
    last_seen_dir: 1,
    calibration_ok: false,
    pid: {
      integral: 0,
      prev_error: 0,
      initialized: false,
    },
    calib: {
      min_raw: [65535, 65535, 65535, 65535],
      max_raw: [0, 0, 0, 0],
      calibrated: false,
    },
    filtered: [0, 0, 0, 0],
    last_position: 0,
    last_frame: null,
    last_motor: { left: 0, right: 0 },
  };
}

function resetCalibration(ctx) {
  ctx.calib.min_raw = [65535, 65535, 65535, 65535];
  ctx.calib.max_raw = [0, 0, 0, 0];
  ctx.calib.calibrated = false;
}

function readRawSensors4() {
  const raw = api.getLineSensorRawArray();
  const out = [0, 0, 0, 0];
  for (let i = 0; i < 4; i += 1) {
    const value = raw[i];
    out[i] = Number.isFinite(value) ? clampI(Math.round(value), 0, 4095) : 0;
  }
  return out;
}

function updateCalibration(ctx, raw) {
  for (let i = 0; i < 4; i += 1) {
    if (raw[i] < ctx.calib.min_raw[i]) ctx.calib.min_raw[i] = raw[i];
    if (raw[i] > ctx.calib.max_raw[i]) ctx.calib.max_raw[i] = raw[i];
  }
}

function finishCalibration(ctx) {
  let ok = true;
  for (let i = 0; i < 4; i += 1) {
    if (ctx.calib.max_raw[i] <= ctx.calib.min_raw[i] + 20) {
      ok = false;
      break;
    }
  }
  ctx.calib.calibrated = ok;
  ctx.calibration_ok = ok;
}

function normalizeWithCalib(raw, minV, maxV) {
  if (maxV <= minV + 4) {
    return clampI(Math.round((raw * 1000) / 4095), 0, 1000);
  }
  return clampI(Math.round(((raw - minV) * 1000) / (maxV - minV)), 0, 1000);
}

function readSensorFrame(ctx) {
  const raw = readRawSensors4();
  const norm = [0, 0, 0, 0];
  const filtered = [0, 0, 0, 0];
  const filteredU16 = [0, 0, 0, 0];
  let weightedSum = 0;
  let signalSum = 0;

  for (let i = 0; i < 4; i += 1) {
    const n = normalizeWithCalib(raw[i], ctx.calib.min_raw[i], ctx.calib.max_raw[i]);
    const f = CFG.sensor_filter_alpha * n + (1 - CFG.sensor_filter_alpha) * ctx.filtered[i];
    const fu16 = clampI(Math.round(f), 0, 1000);

    ctx.filtered[i] = f;
    norm[i] = n;
    filtered[i] = f;
    filteredU16[i] = fu16;

    signalSum += fu16;
    weightedSum += CFG.sensor_weights[i] * fu16;
  }

  const lineDetected = signalSum >= CFG.line_detect_min_sum;
  let position = ctx.last_position;
  if (lineDetected && signalSum > 0) {
    position = Math.round(weightedSum / signalSum);
    ctx.last_position = position;
  }

  return {
    raw,
    norm,
    filtered,
    filtered_u16: filteredU16,
    signal_sum: signalSum,
    position,
    line_detected: lineDetected,
  };
}

function resetPid(ctx) {
  ctx.pid.integral = 0;
  ctx.pid.prev_error = 0;
  ctx.pid.initialized = false;
}

function updatePid(error, dtS, ctx) {
  if (!(dtS > 0)) return 0;

  if (!ctx.pid.initialized) {
    ctx.pid.prev_error = error;
    ctx.pid.initialized = true;
  }

  ctx.pid.integral += error * dtS;
  ctx.pid.integral = clampI(ctx.pid.integral, -5000, 5000);

  const derivative = (error - ctx.pid.prev_error) / dtS;
  const output = CFG.kp * error + CFG.ki * ctx.pid.integral + CFG.kd * derivative;
  ctx.pid.prev_error = error;

  return clampI(Math.round(output), -CFG.max_correction, CFG.max_correction);
}

function computeMotorCmd(correction) {
  const left = clampI(CFG.base_speed - correction, -CFG.max_motor_cmd, CFG.max_motor_cmd);
  const right = clampI(CFG.base_speed + correction, -CFG.max_motor_cmd, CFG.max_motor_cmd);
  return { left, right };
}

function applyDeadband(cmd) {
  if (cmd === 0) return 0;
  if (cmd > 0 && cmd < CFG.motor_deadband) return CFG.motor_deadband;
  if (cmd < 0 && cmd > -CFG.motor_deadband) return -CFG.motor_deadband;
  return cmd;
}

function chassisSetCommand(ctx, leftCmd, rightCmd) {
  let left = clampI(Math.round(leftCmd), -CFG.max_motor_cmd, CFG.max_motor_cmd);
  let right = clampI(Math.round(rightCmd), -CFG.max_motor_cmd, CFG.max_motor_cmd);
  left = applyDeadband(left);
  right = applyDeadband(right);
  api.setMotorCommand(left, right);
  ctx.last_motor = { left, right };
}

function chassisStop(ctx) {
  api.setMotorCommand(0, 0);
  ctx.last_motor = { left: 0, right: 0 };
}

function runCalibrationMotion(ctx, nowMs) {
  const elapsed = nowMs - ctx.calib_start_ms;
  const slot = Math.floor(elapsed / CFG.calibration_switch_interval_ms);
  const spin = CFG.calibration_spin_speed;
  if ((slot & 1) === 0) {
    chassisSetCommand(ctx, spin, -spin);
  } else {
    chassisSetCommand(ctx, -spin, spin);
  }
}

let ctx = api.mem.get("lfv1.ctx", null);
if (!ctx) {
  const now = api.getMillis();
  ctx = createInitialCtx(now);
  api.log("LF_V1 init");
}

const nowMs = api.getMillis();
if ((nowMs - ctx.last_step_ms) < CFG.control_period_ms) {
  api.mem.set("lfv1.ctx", ctx);
  return;
}

const dtS = Math.max(0.0001, (nowMs - ctx.last_step_ms) / 1000);
ctx.last_step_ms = nowMs;

switch (ctx.state) {
  case STATE.WAIT_START: {
    if ((nowMs - ctx.boot_ms) >= CFG.auto_start_delay_ms) {
      resetCalibration(ctx);
      ctx.calib_start_ms = nowMs;
      ctx.state = STATE.CALIBRATING;
      api.log("Calibration start");
    }
    break;
  }

  case STATE.CALIBRATING: {
    runCalibrationMotion(ctx, nowMs);
    const raw = readRawSensors4();
    updateCalibration(ctx, raw);

    if ((nowMs - ctx.calib_start_ms) >= CFG.calibration_duration_ms) {
      chassisStop(ctx);
      finishCalibration(ctx);
      resetPid(ctx);
      ctx.state = STATE.RUNNING;
      api.log("Calibration end -> RUNNING", { calibration_ok: ctx.calibration_ok });
    }
    break;
  }

  case STATE.RUNNING: {
    const frame = readSensorFrame(ctx);
    ctx.last_frame = frame;

    if (!frame.line_detected) {
      ctx.recover_start_ms = nowMs;
      ctx.state = STATE.RECOVERING;
      api.log("Line lost -> RECOVERING");
      break;
    }

    const error = -frame.position;
    const correction = updatePid(error, dtS, ctx);
    const cmd = computeMotorCmd(correction);

    if (frame.position < 0) ctx.last_seen_dir = -1;
    else if (frame.position > 0) ctx.last_seen_dir = 1;

    chassisSetCommand(ctx, cmd.left, cmd.right);
    break;
  }

  case STATE.RECOVERING: {
    const turn = CFG.recover_turn_speed;
    if (ctx.last_seen_dir < 0) {
      chassisSetCommand(ctx, -turn, turn);
    } else {
      chassisSetCommand(ctx, turn, -turn);
    }

    const frame = readSensorFrame(ctx);
    ctx.last_frame = frame;

    if (frame.line_detected) {
      resetPid(ctx);
      ctx.state = STATE.RUNNING;
      api.log("Line recovered -> RUNNING");
      break;
    }

    if ((nowMs - ctx.recover_start_ms) > CFG.recover_timeout_ms) {
      chassisStop(ctx);
      ctx.state = STATE.STOPPED;
      api.log("Recovery timeout -> STOPPED");
    }
    break;
  }

  case STATE.STOPPED:
    chassisStop(ctx);
    break;

  default:
    chassisStop(ctx);
    ctx.state = STATE.FAULT;
    break;
}

api.mem.set("lfv1.ctx", ctx);
`;

const TRACK_MASK_SIZE = 2400;
const TRACK_PX_PER_M = 280;
const trackMask = document.createElement("canvas");
trackMask.width = TRACK_MASK_SIZE;
trackMask.height = TRACK_MASK_SIZE;
const trackMaskCtx = trackMask.getContext("2d", { willReadFrequently: true });

const dragState = {
  active: false,
  lastX: 0,
  lastY: 0,
};

const sim = {
  running: true,
  codeMode: false,
  time: 0,
  pose: { x: 0, y: 0, theta: Math.PI / 2 },
  wheelSpeeds: { FL: 0, FR: 0, RL: 0, RR: 0 },
  trackWidth: 0.2,
  wheelBase: 0.4,
  trail: [],
  lastKinematics: null,
  lastDiagnostics: null,
  controller: null,
  maxTrailPoints: 900,
  maxAbsWheelSpeed: 4.0,
  wheelVisualScale: 1.0,
  maxDt: 0.04,
  camera: {
    offsetX: 0,
    offsetY: 0,
    scale: 220,
    followCar: false,
  },
  track: {
    preset: "circle",
    lineWidth: 0.03,
    svgPath: cfgSvgPath.value,
    sensors: [],
    sensorReadings: [],
    svgPath2D: null,
    drawMode: false,
    draftPoints: [],
  },
  debugLines: [],
  maxDebugLines: 180,
  scriptMemory: {},
  autoTest: {
    running: false,
    abortRequested: false,
    lastReport: null,
  },
};

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function readNumberInput(inputEl, min, max, fallback) {
  const value = Number(inputEl.value);
  if (!Number.isFinite(value)) return fallback;
  return clamp(value, min, max);
}

function setWheelSpeed(name, value) {
  if (!wheelNames.includes(name)) return;
  sim.wheelSpeeds[name] = clamp(value, -sim.maxAbsWheelSpeed, sim.maxAbsWheelSpeed);
  updateWheelValue(name);
}

function setWheelSpeeds(vFL, vFR, vRL, vRR) {
  setWheelSpeed("FL", vFL);
  setWheelSpeed("FR", vFR);
  setWheelSpeed("RL", vRL);
  setWheelSpeed("RR", vRR);
}

function sensorValueToRaw(value) {
  return Math.round(clamp(value, 0, 1) * 4095);
}

function commandToWheelSpeed(cmd) {
  const numeric = Number(cmd);
  const safeCmd = Number.isFinite(numeric) ? numeric : 0;
  return (clamp(safeCmd, -1000, 1000) / 1000) * sim.maxAbsWheelSpeed;
}

function setMotorCommand(leftCmd, rightCmd) {
  const leftSpeed = commandToWheelSpeed(leftCmd);
  const rightSpeed = commandToWheelSpeed(rightCmd);
  setWheelSpeeds(leftSpeed, rightSpeed, leftSpeed, rightSpeed);
}

function n(value, digits = 2) {
  return value.toFixed(digits);
}

function updateWheelValue(name) {
  const valueEl = wheelValueEls[name];
  if (valueEl) {
    valueEl.textContent = `${n(sim.wheelSpeeds[name], 2)} m/s`;
  }
}

function setTrackStatus(message, isError = false) {
  trackStatus.textContent = message;
  trackStatus.style.color = isError ? "#9e352f" : "#27465f";
}

function setAutoTestStatus(message, isError = false) {
  if (autoTestStatus == null) return;
  autoTestStatus.textContent = message;
  autoTestStatus.style.color = isError ? "#9e352f" : "#27465f";
}

function formatAutoTestReport(report) {
  if (report == null) return "(no report)";

  const summary = report.summary || {};
  const lines = [
    "Overall score: " + Number(summary.overallScore || 0).toFixed(1) + " / 100",
    "Scenarios: " + (summary.scenarioCount || 0) + ", completed: " + (summary.completedCount || 0) + ", aborted: " + (summary.aborted ? "yes" : "no"),
    "Avg line detection: " + Number((summary.avgLineDetectionRate || 0) * 100).toFixed(1) + "%",
    "Max longest line loss: " + Number(summary.maxLongestLostSec || 0).toFixed(3) + " s",
  ];

  if (Array.isArray(report.issues) && report.issues.length > 0) {
    lines.push("Issues:");
    for (const issue of report.issues) {
      lines.push("- " + issue);
    }
  }

  lines.push("", "JSON:", JSON.stringify(report, null, 2));
  return lines.join("\n");
}

function renderAutoTestReport(report) {
  if (autoTestReport == null) return;
  autoTestReport.textContent = formatAutoTestReport(report);
}

function syncAutoTestButtons() {
  if (runAutoTest) runAutoTest.disabled = sim.autoTest.running;
  if (stopAutoTest) stopAutoTest.disabled = sim.autoTest.running === false;
  if (copyAutoTestReport) copyAutoTestReport.disabled = sim.autoTest.lastReport == null;
}

function bodyToWorld(localX, localY, pose) {
  const c = Math.cos(pose.theta);
  const s = Math.sin(pose.theta);
  return {
    x: pose.x + localX * c - localY * s,
    y: pose.y + localX * s + localY * c,
  };
}

function parseSensorConfig(text) {
  let parsed;
  try {
    parsed = JSON.parse(text);
  } catch {
    throw new Error("Sensor JSON is invalid.");
  }
  if (!Array.isArray(parsed) || parsed.length === 0) {
    throw new Error("Sensor config must be a non-empty JSON array.");
  }

  const sensors = parsed.map((item, index) => {
    if (!item || typeof item !== "object") {
      throw new Error(`Sensor ${index} must be an object.`);
    }
    const name = typeof item.name === "string" && item.name.trim() ? item.name.trim() : `S${index}`;
    const x = Number(item.x);
    const y = Number(item.y);
    if (!Number.isFinite(x) || !Number.isFinite(y)) {
      throw new Error(`Sensor ${name} must contain numeric x and y.`);
    }
    return { name, x, y };
  });

  return sensors;
}

function buildWheelGrid() {
  wheelGrid.innerHTML = "";

  for (const name of wheelNames) {
    const label = document.createElement("div");
    label.className = "wheel-label";
    label.textContent = name;

    const value = document.createElement("div");
    value.className = "wheel-value";
    wheelValueEls[name] = value;
    updateWheelValue(name);

    const minus = document.createElement("button");
    minus.className = "small-btn secondary";
    minus.textContent = "-0.2";
    minus.addEventListener("click", () => {
      setWheelSpeed(name, sim.wheelSpeeds[name] - 0.2);
    });

    const plus = document.createElement("button");
    plus.className = "small-btn secondary";
    plus.textContent = "+0.2";
    plus.addEventListener("click", () => {
      setWheelSpeed(name, sim.wheelSpeeds[name] + 0.2);
    });

    wheelGrid.append(label, value, minus, plus);
  }
}

function updateMetrics() {
  const p = sim.pose;
  const k = sim.lastKinematics || { vx: 0, omega: 0, radius: Infinity };
  const d = sim.lastDiagnostics || { motionState: "STATIONARY", lateralSlipIndicator: 0 };
  const sensorSummary = sim.track.sensorReadings
    .map((s) => `${s.name}:${n(s.value, 2)}`)
    .join(" ");

  const values = [
    ["x", `${n(p.x, 2)} m`],
    ["y", `${n(p.y, 2)} m`],
    ["theta", `${n(p.theta, 2)} rad`],
    ["vx", `${n(k.vx, 2)} m/s`],
    ["omega", `${n(k.omega, 2)} rad/s`],
    ["radius", Number.isFinite(k.radius) ? `${n(k.radius, 2)} m` : "inf"],
    ["motion", d.motionState],
    ["sensors", sensorSummary || "n/a"],
  ];

  metrics.innerHTML = "";
  for (const [label, val] of values) {
    const row = document.createElement("div");
    const left = document.createElement("span");
    left.textContent = label;
    const right = document.createElement("strong");
    right.textContent = val;
    row.append(left, right);
    metrics.appendChild(row);
  }
}

function setCodeEditorValue(source) {
  if (codeEditor) {
    codeEditor.setValue(source);
    return;
  }
  codeInput.value = source;
}

function loadLfV1TemplateIntoEditor(showStatus = true) {
  setCodeEditorValue(LF_V1_TEMPLATE);
  if (showStatus) {
    setStatus("Loaded LF V1 compatibility template.");
  }
}

function compileController() {
  const source = codeEditor ? codeEditor.getValue() : codeInput.value;
  sim.controller = new Function("api", source);
}

function executeController(dt) {
  if (!sim.controller) return;

  const sensors = sim.track.sensorReadings.map((sensor) => ({ ...sensor }));
  const sensorMap = new Map(sensors.map((sensor) => [sensor.name, sensor]));
  const sensorRaw = sensors.map((sensor) => sensorValueToRaw(sensor.value));
  const millis = Math.round(sim.time * 1000);

  const api = {
    dt,
    time: sim.time,
    millis,
    getMillis: () => millis,
    pose: { ...sim.pose },
    wheelSpeeds: { ...sim.wheelSpeeds },
    sensors,
    sensorRaw: sensorRaw.slice(),
    getSensor: (name) => sensorMap.get(name),
    getSensorRaw: (nameOrIndex) => {
      if (typeof nameOrIndex === "number" && Number.isInteger(nameOrIndex)) {
        return sensorRaw[nameOrIndex];
      }
      const sensor = sensorMap.get(String(nameOrIndex));
      return sensor ? sensorValueToRaw(sensor.value) : undefined;
    },
    getLineSensorRaw: (index) => {
      const i = Number(index);
      if (!Number.isInteger(i)) return undefined;
      return sensorRaw[i];
    },
    getLineSensorRawArray: () => sensorRaw.slice(),
    mem: createScriptMemoryApi(),
    setWheelSpeed,
    setWheelSpeeds,
    commandToWheelSpeed,
    setMotorCommand,
    log: (...parts) => addDebugLine(...parts),
    clearLog: clearDebugLog,
    clamp,
    Math,
  };

  sim.controller(api);
}

function setStatus(message, isError = false) {
  codeStatus.textContent = message;
  codeStatus.style.color = isError ? "#9e352f" : "#27465f";
}

function renderDebugLog() {
  debugLog.textContent = sim.debugLines.join("\n");
  debugLog.scrollTop = debugLog.scrollHeight;
}

function addDebugLine(...parts) {
  const text = parts.map((part) => {
    if (typeof part === "string") return part;
    try {
      return JSON.stringify(part);
    } catch {
      return String(part);
    }
  }).join(" ");

  const stamp = `[${n(sim.time, 2)}s]`;
  sim.debugLines.push(`${stamp} ${text}`);
  if (sim.debugLines.length > sim.maxDebugLines) {
    sim.debugLines.shift();
  }
  renderDebugLog();
}

function clearDebugLog() {
  sim.debugLines = [];
  renderDebugLog();
}

function renderMemoryView() {
  const keys = Object.keys(sim.scriptMemory);
  if (keys.length === 0) {
    memoryView.textContent = "(empty)";
    return;
  }

  const lines = keys.map((key) => {
    let valueText;
    try {
      valueText = JSON.stringify(sim.scriptMemory[key]);
    } catch {
      valueText = String(sim.scriptMemory[key]);
    }
    return `${key}: ${valueText}`;
  });
  memoryView.textContent = lines.join("\n");
}

function cloneForMemory(value) {
  if (typeof structuredClone === "function") {
    return structuredClone(value);
  }
  if (value === undefined) return undefined;
  try {
    return JSON.parse(JSON.stringify(value));
  } catch {
    return value;
  }
}

function createScriptMemoryApi() {
  return {
    get(key, defaultValue = undefined) {
      if (Object.prototype.hasOwnProperty.call(sim.scriptMemory, key)) {
        return cloneForMemory(sim.scriptMemory[key]);
      }
      return cloneForMemory(defaultValue);
    },
    set(key, value) {
      sim.scriptMemory[key] = cloneForMemory(value);
      renderMemoryView();
      return cloneForMemory(sim.scriptMemory[key]);
    },
    remove(key) {
      delete sim.scriptMemory[key];
      renderMemoryView();
    },
    clear() {
      sim.scriptMemory = {};
      renderMemoryView();
    },
    keys() {
      return Object.keys(sim.scriptMemory);
    },
  };
}

function captureAutoTestTrackInputs() {
  return {
    preset: cfgTrackPreset.value,
    lineWidth: cfgLineWidth.value,
    svgPath: cfgSvgPath.value,
    sensors: cfgSensors.value,
  };
}

function applyAutoTestTrackInputs(trackInputs) {
  cfgTrackPreset.value = trackInputs.preset;
  cfgLineWidth.value = trackInputs.lineWidth;
  cfgSvgPath.value = trackInputs.svgPath;
  cfgSensors.value = trackInputs.sensors;
  return applyTrackAndSensorConfig();
}

function captureAutoTestSnapshot() {
  return {
    running: sim.running,
    codeMode: sim.codeMode,
    controller: sim.controller,
    time: sim.time,
    pose: { ...sim.pose },
    wheelSpeeds: { ...sim.wheelSpeeds },
    trail: sim.trail.map((p) => ({ ...p })),
    lastKinematics: cloneForMemory(sim.lastKinematics),
    lastDiagnostics: cloneForMemory(sim.lastDiagnostics),
    scriptMemory: cloneForMemory(sim.scriptMemory),
    debugLines: sim.debugLines.slice(),
    trackInputs: captureAutoTestTrackInputs(),
    codeStatus: {
      text: codeStatus.textContent,
      color: codeStatus.style.color,
    },
  };
}

function restoreAutoTestSnapshot(snapshot) {
  applyAutoTestTrackInputs(snapshot.trackInputs);

  sim.running = snapshot.running;
  sim.codeMode = snapshot.codeMode;
  sim.controller = snapshot.controller;
  sim.time = snapshot.time;
  sim.pose = { ...snapshot.pose };
  sim.wheelSpeeds = { ...snapshot.wheelSpeeds };
  for (const name of wheelNames) {
    updateWheelValue(name);
  }

  sim.trail = snapshot.trail.map((p) => ({ ...p }));
  sim.lastKinematics = cloneForMemory(snapshot.lastKinematics);
  sim.lastDiagnostics = cloneForMemory(snapshot.lastDiagnostics);
  sim.scriptMemory = cloneForMemory(snapshot.scriptMemory) || {};
  sim.debugLines = snapshot.debugLines.slice(-sim.maxDebugLines);

  codeStatus.textContent = snapshot.codeStatus.text;
  codeStatus.style.color = snapshot.codeStatus.color;

  renderMemoryView();
  renderDebugLog();
  updateSensorReadings();
  updateMetrics();
}

function buildAutoTestScenarios(baseTrackInputs, basePose) {
  const lineWidthValue = Number(baseTrackInputs.lineWidth);
  const lineWidth = Number.isFinite(lineWidthValue) ? String(lineWidthValue) : "0.03";
  const currentTrack = {
    preset: baseTrackInputs.preset,
    lineWidth,
    svgPath: baseTrackInputs.svgPath,
    sensors: baseTrackInputs.sensors,
  };

  const leftOffset = 0.04;
  const offsetPose = {
    x: basePose.x - Math.sin(basePose.theta) * leftOffset,
    y: basePose.y + Math.cos(basePose.theta) * leftOffset,
    theta: basePose.theta,
  };

  return [
    {
      id: "current_baseline",
      name: `Current track (${currentTrack.preset}) baseline`,
      trackInputs: currentTrack,
      startPose: { ...basePose },
    },
    {
      id: "current_left_offset",
      name: `Current track (${currentTrack.preset}) left offset`,
      trackInputs: currentTrack,
      startPose: offsetPose,
    },
    {
      id: "circle_reference",
      name: "Circle reference",
      trackInputs: {
        preset: "circle",
        lineWidth,
        svgPath: currentTrack.svgPath,
        sensors: currentTrack.sensors,
      },
      startPose: { x: 0, y: 0, theta: Math.PI / 2 },
    },
    {
      id: "figure8_reference",
      name: "Figure8 reference",
      trackInputs: {
        preset: "figure8",
        lineWidth,
        svgPath: currentTrack.svgPath,
        sensors: currentTrack.sensors,
      },
      startPose: { x: 0, y: 0, theta: Math.PI / 2 },
    },
  ];
}

function resetAutoTestScenarioState(startPose) {
  sim.time = 0;
  sim.pose = { ...startPose };
  sim.trail = [];
  sim.lastKinematics = null;
  sim.lastDiagnostics = null;
  sim.scriptMemory = {};
  setWheelSpeeds(0, 0, 0, 0);
  renderMemoryView();
  updateSensorReadings();
}

function estimateAutoTestLineState(sensorReadings, lineThreshold) {
  let signalSum = 0;
  let weightedY = 0;
  let maxValue = 0;

  for (const sensor of sensorReadings) {
    const value = clamp(sensor.value, 0, 1);
    signalSum += value;
    weightedY += sensor.y * value;
    if (value > maxValue) {
      maxValue = value;
    }
  }

  const lineDetected = maxValue >= lineThreshold;
  const lineErrorM = signalSum > 1e-9 ? weightedY / signalSum : 0;
  return { lineDetected, lineErrorM, confidence: maxValue };
}

function scoreAutoTestScenario(result) {
  if (result.runtimeError == null) {
    let score = 100;
    score -= (1 - result.lineDetectionRate) * 60;
    score -= Math.min(result.meanAbsErrorM / 0.08, 1) * 20;
    score -= Math.min(result.longestLostSec / 2.0, 1) * 15;
    score -= Math.min(result.motorSaturationRate / 0.6, 1) * 5;
    return clamp(score, 0, 100);
  }
  return 0;
}

async function runAutoTestScenario(scenario, options, scenarioIndex, scenarioCount) {
  const { durationSec, dt, lineThreshold } = options;
  const targetSteps = Math.max(1, Math.round(durationSec / dt));

  let steps = 0;
  let lineDetectedSteps = 0;
  let lineLostTransitions = 0;
  let lineRecoveredTransitions = 0;
  let longestLostSec = 0;
  let currentLostSec = 0;
  let totalLostSec = 0;
  let absErrorAccum = 0;
  let sqErrorAccum = 0;
  let errorSamples = 0;
  let maxAbsError = 0;
  let motorSaturationSteps = 0;
  let distanceM = 0;
  let runtimeError = null;
  let prevLineDetected = null;

  for (let step = 0; step < targetSteps; step += 1) {
    if (sim.autoTest.abortRequested) {
      break;
    }

    let stepResult;
    try {
      stepResult = simulateStep(dt, { catchControllerError: false });
    } catch (err) {
      runtimeError = err && err.message ? err.message : String(err);
      break;
    }

    sim.time += dt;
    steps += 1;
    distanceM += Math.hypot(stepResult.delta.dx, stepResult.delta.dy);

    const lineState = estimateAutoTestLineState(sim.track.sensorReadings, lineThreshold);

    if (lineState.lineDetected) {
      lineDetectedSteps += 1;
      absErrorAccum += Math.abs(lineState.lineErrorM);
      sqErrorAccum += lineState.lineErrorM * lineState.lineErrorM;
      maxAbsError = Math.max(maxAbsError, Math.abs(lineState.lineErrorM));
      errorSamples += 1;
      if (currentLostSec > 0) {
        longestLostSec = Math.max(longestLostSec, currentLostSec);
        currentLostSec = 0;
      }
    } else {
      currentLostSec += dt;
      totalLostSec += dt;
      if (prevLineDetected === true) {
        lineLostTransitions += 1;
      }
    }

    if (prevLineDetected === false && lineState.lineDetected) {
      lineRecoveredTransitions += 1;
    }
    prevLineDetected = lineState.lineDetected;

    const saturated = wheelNames.some((name) => Math.abs(sim.wheelSpeeds[name]) >= sim.maxAbsWheelSpeed - 1e-6);
    if (saturated) {
      motorSaturationSteps += 1;
    }

    if ((step + 1) % 250 === 0 || step + 1 === targetSteps) {
      const percent = Math.round(((step + 1) / targetSteps) * 100);
      setAutoTestStatus(`Running ${scenarioIndex + 1}/${scenarioCount}: ${scenario.name} (${percent}%)`);
      await new Promise((resolve) => setTimeout(resolve, 0));
    }
  }

  if (currentLostSec > 0) {
    longestLostSec = Math.max(longestLostSec, currentLostSec);
  }

  const lineDetectionRate = steps > 0 ? lineDetectedSteps / steps : 0;
  const meanAbsErrorM = errorSamples > 0 ? absErrorAccum / errorSamples : 0;
  const rmsErrorM = errorSamples > 0 ? Math.sqrt(sqErrorAccum / errorSamples) : 0;
  const motorSaturationRate = steps > 0 ? motorSaturationSteps / steps : 0;

  const result = {
    id: scenario.id,
    name: scenario.name,
    preset: scenario.trackInputs.preset,
    durationTargetSec: durationSec,
    durationSimulatedSec: steps * dt,
    steps,
    lineDetectionRate,
    lineLostTransitions,
    lineRecoveredTransitions,
    longestLostSec,
    totalLostSec,
    meanAbsErrorM,
    rmsErrorM,
    maxAbsErrorM: maxAbsError,
    motorSaturationRate,
    distanceM,
    runtimeError,
  };

  result.score = scoreAutoTestScenario(result);
  return result;
}

function buildAutoTestReport({
  startedAt,
  durationSec,
  dt,
  lineThreshold,
  scenarios,
  results,
  aborted,
  suiteError,
}) {
  const completed = results.filter((r) => (r.runtimeError == null)).length;
  const avgLineDetectionRate =
    results.length > 0 ? results.reduce((sum, r) => sum + r.lineDetectionRate, 0) / results.length : 0;
  const overallScore =
    results.length > 0 ? results.reduce((sum, r) => sum + r.score, 0) / results.length : 0;
  const maxLongestLostSec =
    results.length > 0 ? Math.max(...results.map((r) => r.longestLostSec || 0)) : 0;

  const issues = [];
  if (aborted) {
    issues.push("Suite aborted manually.");
  }
  if (suiteError) {
    issues.push(`Suite internal error: ${suiteError}`);
  }

  const runtimeFailures = results.filter((r) => (r.runtimeError == null ? false : true));
  if (runtimeFailures.length > 0) {
    issues.push(`Runtime errors in: ${runtimeFailures.map((r) => r.id).join(", ")}`);
  }

  if (avgLineDetectionRate < 0.85) {
    issues.push("Average line detection is below 85%; check threshold, sensor layout, or recovery logic.");
  }
  if (maxLongestLostSec > 1.0) {
    issues.push("Line loss duration exceeds 1s in at least one scenario.");
  }

  return {
    version: 1,
    generatedAt: new Date().toISOString(),
    startedAt,
    config: {
      durationSec,
      dt,
      lineThreshold,
      scenarioIds: scenarios.map((s) => s.id),
    },
    summary: {
      scenarioCount: scenarios.length,
      completedCount: completed,
      aborted,
      overallScore,
      avgLineDetectionRate,
      maxLongestLostSec,
    },
    issues,
    scenarios: results,
  };
}

async function runAutoTestSuite() {
  if (sim.autoTest.running) {
    return;
  }

  if (autoTestDuration == null || autoTestDt == null || autoTestLineThreshold == null) {
    setAutoTestStatus("Auto test controls are not available.", true);
    return;
  }

  const durationSec = readNumberInput(autoTestDuration, 1, 180, 15);
  const dt = readNumberInput(autoTestDt, 0.002, 0.05, 0.01);
  const lineThreshold = readNumberInput(autoTestLineThreshold, 0.01, 0.95, 0.12);

  autoTestDuration.value = String(durationSec);
  autoTestDt.value = String(dt);
  autoTestLineThreshold.value = String(lineThreshold);

  try {
    compileController();
  } catch (err) {
    const message = err && err.message ? err.message : String(err);
    setAutoTestStatus(`Compile error: ${message}`, true);
    setStatus(`Compile error: ${message}`, true);
    return;
  }

  const snapshot = captureAutoTestSnapshot();
  const startedAt = new Date().toISOString();
  const baseTrackInputs = captureAutoTestTrackInputs();
  const scenarios = buildAutoTestScenarios(baseTrackInputs, snapshot.pose);

  sim.autoTest.running = true;
  sim.autoTest.abortRequested = false;
  sim.autoTest.lastReport = null;
  sim.running = false;
  sim.codeMode = true;

  renderAutoTestReport(null);
  syncAutoTestButtons();
  clearDebugLog();
  setAutoTestStatus(`Preparing ${scenarios.length} scenarios...`);

  const results = [];
  let suiteError = null;

  try {
    for (let i = 0; i < scenarios.length; i += 1) {
      if (sim.autoTest.abortRequested) {
        break;
      }

      const scenario = scenarios[i];
      const trackApplied = applyAutoTestTrackInputs(scenario.trackInputs);
      if (trackApplied === false) {
        results.push({
          id: scenario.id,
          name: scenario.name,
          preset: scenario.trackInputs.preset,
          durationTargetSec: durationSec,
          durationSimulatedSec: 0,
          steps: 0,
          lineDetectionRate: 0,
          lineLostTransitions: 0,
          lineRecoveredTransitions: 0,
          longestLostSec: 0,
          totalLostSec: 0,
          meanAbsErrorM: 0,
          rmsErrorM: 0,
          maxAbsErrorM: 0,
          motorSaturationRate: 0,
          distanceM: 0,
          runtimeError: "Track/sensor config apply failed",
          score: 0,
        });
        continue;
      }

      resetAutoTestScenarioState(scenario.startPose);
      const scenarioResult = await runAutoTestScenario(
        scenario,
        { durationSec, dt, lineThreshold },
        i,
        scenarios.length
      );
      results.push(scenarioResult);
    }
  } catch (err) {
    suiteError = err && err.message ? err.message : String(err);
  } finally {
    restoreAutoTestSnapshot(snapshot);
  }

  const aborted = sim.autoTest.abortRequested ? true : false;
  const report = buildAutoTestReport({
    startedAt,
    durationSec,
    dt,
    lineThreshold,
    scenarios,
    results,
    aborted,
    suiteError,
  });

  sim.autoTest.lastReport = report;
  sim.autoTest.running = false;
  sim.autoTest.abortRequested = false;
  renderAutoTestReport(report);
  syncAutoTestButtons();

  if (suiteError) {
    setAutoTestStatus(`Auto test failed: ${suiteError}`, true);
  } else if (aborted) {
    setAutoTestStatus("Auto test aborted by user.", true);
  } else {
    setAutoTestStatus(
      `Auto test done. Score ${n(report.summary.overallScore, 1)}/100, avg detection ${(report.summary.avgLineDetectionRate * 100).toFixed(1)}%.`
    );
  }
}

function drawPresetTrackPath(targetCtx, preset) {
  if (preset === "circle") {
    const r = 1.15;
    // Circle centered at (r, 0) so the loop goes through (0, 0).
    targetCtx.moveTo(0, 0);
    targetCtx.arc(r, 0, r, Math.PI, Math.PI + 2 * Math.PI);
    return;
  }
  if (preset === "figure8") {
    const r = 0.7;
    targetCtx.arc(-r, 0, r, 0, 2 * Math.PI);
    targetCtx.moveTo(0, 0);
    targetCtx.arc(r, 0, r, 0, 2 * Math.PI);
    return;
  }

  // oval
  targetCtx.ellipse(0, 0, 1.45, 0.9, 0, 0, 2 * Math.PI);
}

function drawTrackGeometry(targetCtx) {
  targetCtx.beginPath();
  if (sim.track.preset === "svg" && sim.track.svgPath2D) {
    targetCtx.stroke(sim.track.svgPath2D);
    return;
  }
  drawPresetTrackPath(targetCtx, sim.track.preset);
  targetCtx.stroke();
}

function rebuildTrackMask() {
  trackMaskCtx.save();
  trackMaskCtx.fillStyle = "black";
  trackMaskCtx.fillRect(0, 0, trackMask.width, trackMask.height);
  trackMaskCtx.translate(trackMask.width / 2, trackMask.height / 2);
  trackMaskCtx.scale(TRACK_PX_PER_M, -TRACK_PX_PER_M);
  trackMaskCtx.lineCap = "round";
  trackMaskCtx.lineJoin = "round";
  trackMaskCtx.strokeStyle = "white";
  trackMaskCtx.lineWidth = sim.track.lineWidth;
  drawTrackGeometry(trackMaskCtx);
  trackMaskCtx.restore();
}

function updateSensorReadings() {
  sim.track.sensorReadings = sim.track.sensors.map((sensor) => {
    const world = bodyToWorld(sensor.x, sensor.y, sim.pose);
    const px = Math.round(trackMask.width / 2 + world.x * TRACK_PX_PER_M);
    const py = Math.round(trackMask.height / 2 - world.y * TRACK_PX_PER_M);

    let value = 0;
    if (px >= 0 && px < trackMask.width && py >= 0 && py < trackMask.height) {
      const rgba = trackMaskCtx.getImageData(px, py, 1, 1).data;
      value = rgba[0] / 255;
    }

    return {
      name: sensor.name,
      x: sensor.x,
      y: sensor.y,
      worldX: world.x,
      worldY: world.y,
      value,
    };
  });
}

function applyTrackAndSensorConfig() {
  try {
    sim.track.preset = cfgTrackPreset.value;
    sim.track.lineWidth = readNumberInput(cfgLineWidth, 0.005, 0.2, sim.track.lineWidth);
    sim.track.svgPath = cfgSvgPath.value.trim();
    sim.track.sensors = parseSensorConfig(cfgSensors.value);

    sim.track.svgPath2D = null;
    if (sim.track.preset === "svg") {
      if (sim.track.svgPath === "") {
        throw new Error("SVG path is empty.");
      }
      sim.track.svgPath2D = new Path2D(sim.track.svgPath);
    }

    cfgLineWidth.value = String(sim.track.lineWidth);
    rebuildTrackMask();
    updateSensorReadings();
    setTrackStatus(
      `Track applied: ${sim.track.preset}, line width ${n(sim.track.lineWidth, 3)}m, sensors ${sim.track.sensors.length}.`
    );
    return true;
  } catch (err) {
    setTrackStatus(`Track/sensor config error: ${err.message}`, true);
    return false;
  }
}

function simulateStep(dt, options = {}) {
  const { catchControllerError = true } = options;

  updateSensorReadings();

  if (sim.codeMode) {
    if (catchControllerError) {
      try {
        executeController(dt);
      } catch (err) {
        sim.codeMode = false;
        setStatus(`Code error: ${err.message}`, true);
        throw err;
      }
    } else {
      executeController(dt);
    }
  }

  const result = calculatePositionChange(
    sim.pose,
    sim.wheelSpeeds.FL,
    sim.wheelSpeeds.FR,
    sim.wheelSpeeds.RL,
    sim.wheelSpeeds.RR,
    sim.trackWidth,
    sim.wheelBase,
    dt
  );

  sim.pose = result.pose;
  sim.lastKinematics = result.kinematics;
  sim.lastDiagnostics = result.diagnostics;

  sim.trail.push({ x: sim.pose.x, y: sim.pose.y });
  if (sim.trail.length > sim.maxTrailPoints) {
    sim.trail.shift();
  }

  return result;
}

function stepSimulation(dt) {
  try {
    simulateStep(dt, { catchControllerError: true });
  } catch {
    // Error already handled in simulateStep for realtime mode.
  }
}

function drawGrid(scale, cx, cy) {
  const step = scale;
  const majorEvery = 5;
  const xStart = ((cx % step) + step) % step;
  const yStart = ((cy % step) + step) % step;

  ctx.save();
  ctx.lineWidth = 1;

  let index = 0;
  for (let x = xStart; x <= canvas.width; x += step, index += 1) {
    const isMajor = index % majorEvery === 0;
    ctx.strokeStyle = isMajor ? "rgba(22, 34, 47, 0.12)" : "rgba(22, 34, 47, 0.05)";
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, canvas.height);
    ctx.stroke();
  }

  index = 0;
  for (let y = yStart; y <= canvas.height; y += step, index += 1) {
    const isMajor = index % majorEvery === 0;
    ctx.strokeStyle = isMajor ? "rgba(22, 34, 47, 0.12)" : "rgba(22, 34, 47, 0.05)";
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(canvas.width, y);
    ctx.stroke();
  }

  ctx.strokeStyle = "rgba(230, 103, 46, 0.28)";
  ctx.lineWidth = 1.4;
  if (cx >= 0 && cx <= canvas.width) {
    ctx.beginPath();
    ctx.moveTo(cx, 0);
    ctx.lineTo(cx, canvas.height);
    ctx.stroke();
  }
  if (cy >= 0 && cy <= canvas.height) {
    ctx.beginPath();
    ctx.moveTo(0, cy);
    ctx.lineTo(canvas.width, cy);
    ctx.stroke();
  }
  ctx.restore();
}

function worldToScreen(x, y, camCenterX, camCenterY, scale) {
  return {
    x: camCenterX + x * scale,
    y: camCenterY - y * scale,
  };
}

function screenToWorld(screenX, screenY, camCenterX, camCenterY, cameraWorldX, cameraWorldY, scale) {
  return {
    x: (screenX - camCenterX) / scale + cameraWorldX,
    y: -(screenY - camCenterY) / scale + cameraWorldY,
  };
}

function updateCameraLabels() {
  camOffsetXVal.textContent = `${sim.camera.offsetX}`;
  camOffsetYVal.textContent = `${sim.camera.offsetY}`;
  camZoomVal.textContent = `${sim.camera.scale}`;
  camFollowToggle.textContent = `Follow Car: ${sim.camera.followCar ? "On" : "Off"}`;
}

function syncSimConfigInputs() {
  cfgTrackWidth.value = String(sim.trackWidth);
  cfgWheelBase.value = String(sim.wheelBase);
  cfgMaxWheelSpeed.value = String(sim.maxAbsWheelSpeed);
  cfgWheelVisualScale.value = String(sim.wheelVisualScale);
  cfgTrailPoints.value = String(sim.maxTrailPoints);
  cfgMaxDt.value = String(sim.maxDt);
}

function applySimConfigFromInputs() {
  sim.trackWidth = readNumberInput(cfgTrackWidth, 0.1, 5, sim.trackWidth);
  sim.wheelBase = readNumberInput(cfgWheelBase, 0.1, 8, sim.wheelBase);
  sim.maxAbsWheelSpeed = readNumberInput(cfgMaxWheelSpeed, 0.2, 20, sim.maxAbsWheelSpeed);
  sim.wheelVisualScale = readNumberInput(cfgWheelVisualScale, 0.2, 3, sim.wheelVisualScale);
  sim.maxTrailPoints = Math.round(readNumberInput(cfgTrailPoints, 50, 20000, sim.maxTrailPoints));
  sim.maxDt = readNumberInput(cfgMaxDt, 0.005, 0.2, sim.maxDt);

  for (const name of wheelNames) {
    sim.wheelSpeeds[name] = clamp(sim.wheelSpeeds[name], -sim.maxAbsWheelSpeed, sim.maxAbsWheelSpeed);
    updateWheelValue(name);
  }

  if (sim.trail.length > sim.maxTrailPoints) {
    sim.trail = sim.trail.slice(sim.trail.length - sim.maxTrailPoints);
  }

  syncSimConfigInputs();
}

function syncCameraInputs() {
  camOffsetX.value = String(sim.camera.offsetX);
  camOffsetY.value = String(sim.camera.offsetY);
  camZoom.value = String(sim.camera.scale);
}

function drawTrack(camCenterX, camCenterY, cameraWorldX, cameraWorldY, scale) {
  ctx.save();
  ctx.translate(camCenterX, camCenterY);
  ctx.scale(scale, -scale);
  ctx.translate(-cameraWorldX, -cameraWorldY);
  ctx.strokeStyle = "#222831";
  ctx.lineCap = "round";
  ctx.lineJoin = "round";
  ctx.globalAlpha = 0.9;
  ctx.lineWidth = sim.track.lineWidth;
  drawTrackGeometry(ctx);
  ctx.restore();

  if (sim.track.draftPoints.length > 0) {
    ctx.save();
    ctx.strokeStyle = "rgba(31, 136, 82, 0.95)";
    ctx.fillStyle = "rgba(31, 136, 82, 0.95)";
    ctx.lineWidth = 2;

    ctx.beginPath();
    for (let i = 0; i < sim.track.draftPoints.length; i += 1) {
      const p = sim.track.draftPoints[i];
      const s = worldToScreen(
        p.x - cameraWorldX,
        p.y - cameraWorldY,
        camCenterX,
        camCenterY,
        scale
      );
      if (i === 0) {
        ctx.moveTo(s.x, s.y);
      } else {
        ctx.lineTo(s.x, s.y);
      }
    }
    ctx.stroke();

    for (const p of sim.track.draftPoints) {
      const s = worldToScreen(
        p.x - cameraWorldX,
        p.y - cameraWorldY,
        camCenterX,
        camCenterY,
        scale
      );
      ctx.beginPath();
      ctx.arc(s.x, s.y, 3.5, 0, 2 * Math.PI);
      ctx.fill();
    }
    ctx.restore();
  }
}

function draftPointsToSvgPath(points) {
  if (points.length < 2) return "";
  const parts = [`M ${points[0].x.toFixed(3)} ${points[0].y.toFixed(3)}`];
  for (let i = 1; i < points.length; i += 1) {
    parts.push(`L ${points[i].x.toFixed(3)} ${points[i].y.toFixed(3)}`);
  }
  return parts.join(" ");
}

function beginTrackDraw() {
  sim.track.drawMode = true;
  sim.track.draftPoints = [];
  canvas.classList.add("draw-mode");
  setTrackStatus("Draw mode enabled: click canvas to add points, then Finish Draw.");
}

function clearTrackDraft() {
  sim.track.draftPoints = [];
  setTrackStatus("Draft points cleared.");
}

function finishTrackDrawMode() {
  if (sim.track.draftPoints.length < 2) {
    setTrackStatus("Need at least 2 points to build a track path.", true);
    return;
  }

  const svgPath = draftPointsToSvgPath(sim.track.draftPoints);
  cfgSvgPath.value = svgPath;
  cfgTrackPreset.value = "svg";
  sim.track.drawMode = false;
  canvas.classList.remove("draw-mode");
  applyTrackAndSensorConfig();
  setTrackStatus(`Draw complete with ${sim.track.draftPoints.length} points. SVG path applied.`);
}

function drawCar(camCenterX, camCenterY, cameraWorldX, cameraWorldY, scale) {
  if (sim.trail.length > 1) {
    ctx.save();
    ctx.strokeStyle = "rgba(230, 103, 46, 0.5)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    for (let i = 0; i < sim.trail.length; i += 1) {
      const p = sim.trail[i];
      const screen = worldToScreen(
        p.x - cameraWorldX,
        p.y - cameraWorldY,
        camCenterX,
        camCenterY,
        scale
      );
      if (i === 0) {
        ctx.moveTo(screen.x, screen.y);
      } else {
        ctx.lineTo(screen.x, screen.y);
      }
    }
    ctx.stroke();
    ctx.restore();
  }

  ctx.save();
  const carScreen = worldToScreen(
    sim.pose.x - cameraWorldX,
    sim.pose.y - cameraWorldY,
    camCenterX,
    camCenterY,
    scale
  );
  ctx.translate(carScreen.x, carScreen.y);
  ctx.rotate(-sim.pose.theta);

  const bodyLength = sim.wheelBase * scale;
  const bodyWidth = sim.trackWidth * scale;

  ctx.fillStyle = "#3b6d90";
  ctx.strokeStyle = "#1f3344";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.roundRect(-bodyLength / 2, -bodyWidth / 2, bodyLength, bodyWidth, 12);
  ctx.fill();
  ctx.stroke();

  ctx.fillStyle = "#e6672e";
  ctx.beginPath();
  ctx.moveTo(bodyLength / 2 - 8, 0);
  ctx.lineTo(bodyLength / 2 - 20, -9);
  ctx.lineTo(bodyLength / 2 - 20, 9);
  ctx.closePath();
  ctx.fill();

  const wheelL = 0.14 * scale * sim.wheelVisualScale;
  const wheelW = 0.07 * scale * sim.wheelVisualScale;
  const xOffset = bodyLength / 2 - wheelL * 0.65;
  const yOffset = bodyWidth / 2 + wheelW * 0.45;
  ctx.fillStyle = "#1f3344";

  ctx.fillRect(-xOffset - wheelL / 2, -yOffset, wheelL, wheelW);
  ctx.fillRect(-xOffset - wheelL / 2, yOffset - wheelW, wheelL, wheelW);
  ctx.fillRect(xOffset - wheelL / 2, -yOffset, wheelL, wheelW);
  ctx.fillRect(xOffset - wheelL / 2, yOffset - wheelW, wheelL, wheelW);

  // Draw IR sensor dots in local frame (x forward, y left).
  for (const sensor of sim.track.sensorReadings) {
    const sx = sensor.x * scale;
    const sy = -sensor.y * scale;
    const intensity = clamp(sensor.value, 0, 1);
    const r = 4;
    ctx.fillStyle = `rgba(${Math.round(220 - 170 * intensity)}, ${Math.round(80 + 120 * intensity)}, 52, 0.95)`;
    ctx.beginPath();
    ctx.arc(sx, sy, r, 0, 2 * Math.PI);
    ctx.fill();
  }

  ctx.restore();
}

function renderScene() {
  const scale = sim.camera.scale;
  const camCenterX = canvas.width / 2 + sim.camera.offsetX;
  const camCenterY = canvas.height / 2 + sim.camera.offsetY;
  const cameraWorldX = sim.camera.followCar ? sim.pose.x : 0;
  const cameraWorldY = sim.camera.followCar ? sim.pose.y : 0;
  const viewCenter = worldToScreen(-cameraWorldX, -cameraWorldY, camCenterX, camCenterY, scale);

  ctx.clearRect(0, 0, canvas.width, canvas.height);
  drawGrid(scale, viewCenter.x, viewCenter.y);
  drawTrack(camCenterX, camCenterY, cameraWorldX, cameraWorldY, scale);
  drawCar(camCenterX, camCenterY, cameraWorldX, cameraWorldY, scale);
}

camOffsetX.addEventListener("input", (event) => {
  sim.camera.offsetX = Number(event.target.value);
  updateCameraLabels();
});

camOffsetY.addEventListener("input", (event) => {
  sim.camera.offsetY = Number(event.target.value);
  updateCameraLabels();
});

camZoom.addEventListener("input", (event) => {
  sim.camera.scale = Number(event.target.value);
  updateCameraLabels();
});

camFollowToggle.addEventListener("click", () => {
  sim.camera.followCar = !sim.camera.followCar;
  updateCameraLabels();
});

camReset.addEventListener("click", () => {
  sim.camera.offsetX = 0;
  sim.camera.offsetY = 0;
  sim.camera.scale = 220;
  sim.camera.followCar = false;
  syncCameraInputs();
  updateCameraLabels();
});

clearDebug.addEventListener("click", clearDebugLog);

if (runAutoTest) {
  runAutoTest.addEventListener("click", () => {
    runAutoTestSuite();
  });
}

if (stopAutoTest) {
  stopAutoTest.addEventListener("click", () => {
    if (sim.autoTest.running === false) return;
    sim.autoTest.abortRequested = true;
    setAutoTestStatus("Stop requested. Finishing current step...");
  });
}

if (copyAutoTestReport) {
  copyAutoTestReport.addEventListener("click", async () => {
    if (sim.autoTest.lastReport == null) {
      setAutoTestStatus("No report available to copy.", true);
      return;
    }

    const text = JSON.stringify(sim.autoTest.lastReport, null, 2);
    try {
      if (navigator.clipboard && navigator.clipboard.writeText) {
        await navigator.clipboard.writeText(text);
        setAutoTestStatus("Auto test report copied.");
      } else {
        throw new Error("Clipboard API unavailable");
      }
    } catch {
      setAutoTestStatus("Copy failed. Please copy from the report panel manually.", true);
    }
  });
}

if (loadLfV1Template) {
  loadLfV1Template.addEventListener("click", () => {
    loadLfV1TemplateIntoEditor(true);
  });
}

for (const configInput of [
  cfgTrackWidth,
  cfgWheelBase,
  cfgMaxWheelSpeed,
  cfgWheelVisualScale,
  cfgTrailPoints,
  cfgMaxDt,
]) {
  configInput.addEventListener("change", applySimConfigFromInputs);
}

applyTrackSensors.addEventListener("click", applyTrackAndSensorConfig);
startTrackDraw.addEventListener("click", beginTrackDraw);
finishTrackDraw.addEventListener("click", finishTrackDrawMode);
clearTrackDraw.addEventListener("click", clearTrackDraft);

canvas.addEventListener("mousedown", (event) => {
  if (sim.track.drawMode) {
    const rect = canvas.getBoundingClientRect();
    const x = ((event.clientX - rect.left) / rect.width) * canvas.width;
    const y = ((event.clientY - rect.top) / rect.height) * canvas.height;
    const camCenterX = canvas.width / 2 + sim.camera.offsetX;
    const camCenterY = canvas.height / 2 + sim.camera.offsetY;
    const cameraWorldX = sim.camera.followCar ? sim.pose.x : 0;
    const cameraWorldY = sim.camera.followCar ? sim.pose.y : 0;
    const world = screenToWorld(x, y, camCenterX, camCenterY, cameraWorldX, cameraWorldY, sim.camera.scale);
    sim.track.draftPoints.push(world);
    setTrackStatus(`Added point ${sim.track.draftPoints.length}: (${n(world.x, 2)}, ${n(world.y, 2)})`);
    return;
  }

  dragState.active = true;
  dragState.lastX = event.clientX;
  dragState.lastY = event.clientY;
  canvas.classList.add("dragging");
});

window.addEventListener("mouseup", () => {
  dragState.active = false;
  canvas.classList.remove("dragging");
});

window.addEventListener("mousemove", (event) => {
  if (!dragState.active) return;
  const dx = event.clientX - dragState.lastX;
  const dy = event.clientY - dragState.lastY;
  dragState.lastX = event.clientX;
  dragState.lastY = event.clientY;

  sim.camera.offsetX = clamp(sim.camera.offsetX + dx, -5000, 5000);
  sim.camera.offsetY = clamp(sim.camera.offsetY + dy, -5000, 5000);
  syncCameraInputs();
  updateCameraLabels();
});

canvas.addEventListener("wheel", (event) => {
  event.preventDefault();
  const delta = event.deltaY < 0 ? 4 : -4;
  sim.camera.scale = clamp(sim.camera.scale + delta, 10, 300);
  syncCameraInputs();
  updateCameraLabels();
}, { passive: false });

document.getElementById("toggleRun").addEventListener("click", (event) => {
  sim.running = !sim.running;
  event.target.textContent = sim.running ? "Pause" : "Resume";
});

document.getElementById("zeroSpeeds").addEventListener("click", () => {
  setWheelSpeeds(0, 0, 0, 0);
});

document.getElementById("resetPose").addEventListener("click", () => {
  sim.pose = { x: 0, y: 0, theta: Math.PI / 2 };
  sim.trail = [];
  sim.lastKinematics = null;
  sim.lastDiagnostics = null;
  updateSensorReadings();
  updateMetrics();
});

document.getElementById("enableCode").addEventListener("click", () => {
  try {
    compileController();
    sim.codeMode = true;
    setStatus("Code mode enabled. Your script is running every frame.");
  } catch (err) {
    sim.codeMode = false;
    setStatus(`Compile error: ${err.message}`, true);
  }
});

document.getElementById("runOnce").addEventListener("click", () => {
  try {
    compileController();
    updateSensorReadings();
    executeController(0.016);
    setStatus("Script executed once.");
  } catch (err) {
    setStatus(`Run error: ${err.message}`, true);
  }
});

document.getElementById("disableCode").addEventListener("click", () => {
  sim.codeMode = false;
  setStatus("Code mode disabled.");
});

let lastTime = performance.now();

function loop(now) {
  const dt = Math.min(sim.maxDt, (now - lastTime) / 1000);
  lastTime = now;

  if (sim.running) {
    stepSimulation(dt);
    sim.time += dt;
  }

  renderScene();
  updateMetrics();

  requestAnimationFrame(loop);
}

function initializeCodeEditor() {
  if (typeof window.CodeMirror !== "function") return;

  codeEditor = window.CodeMirror.fromTextArea(codeInput, {
    mode: "javascript",
    theme: "neo",
    lineNumbers: true,
    lineWrapping: true,
    tabSize: 2,
    indentUnit: 2,
  });
}

initializeCodeEditor();
loadLfV1TemplateIntoEditor(false);
buildWheelGrid();
updateMetrics();
syncCameraInputs();
updateCameraLabels();
syncSimConfigInputs();
renderMemoryView();
applyTrackAndSensorConfig();
renderAutoTestReport(sim.autoTest.lastReport);
syncAutoTestButtons();
addDebugLine("Debug ready. Use api.log(...) and api.clearLog().");
requestAnimationFrame(loop);

const COMMON_TRACE_FIELDS = [
  "time_s",
  "forward_position_m",
  "forward_velocity_m_per_s",
  "body_pitch_rad",
  "body_pitch_rate_rad_per_s",
  "body_roll_rad",
  "body_roll_rate_rad_per_s",
  "reaction_position_rad",
  "reaction_rate_rad_per_s",
  "drive_torque_nm",
  "reaction_torque_nm",
];

const $ = (id) => document.getElementById(id);
const state = {
  trace: [],
  index: 0,
  playing: false,
  speed: 1,
  wallStartedMs: 0,
  simStartedS: 0,
  animationFrame: null,
  filename: null,
  view: "split",
};

function finiteNumber(value, field) {
  const number = Number(value);
  if (!Number.isFinite(number)) throw new Error(`${field} is not finite`);
  return number;
}

function normalizeRecord(record, lineNumber) {
  if (!record || typeof record !== "object" || Array.isArray(record)) {
    throw new Error(`record ${lineNumber} is not an object`);
  }
  const keys = Object.keys(record).sort();
  const expected = [...COMMON_TRACE_FIELDS].sort();
  const missing = expected.filter((key) => !keys.includes(key));
  const extra = keys.filter((key) => !expected.includes(key));
  if (missing.length || extra.length) {
    throw new Error(
      `record ${lineNumber} field mismatch` +
      `${missing.length ? `; missing=${missing.join(",")}` : ""}` +
      `${extra.length ? `; extra=${extra.join(",")}` : ""}`
    );
  }
  return Object.fromEntries(COMMON_TRACE_FIELDS.map((field) => [field, finiteNumber(record[field], field)]));
}

function validateTraceOrder(records) {
  if (records.length < 2) throw new Error("trace must contain at least two records");
  for (let i = 1; i < records.length; i += 1) {
    if (records[i].time_s <= records[i - 1].time_s) {
      throw new Error(`time_s must increase strictly at record ${i + 1}`);
    }
  }
  return records;
}

function parseTrace(text) {
  const trimmed = text.trim();
  if (!trimmed) throw new Error("trace is empty");

  let records;
  if (trimmed.startsWith("[")) {
    const values = JSON.parse(trimmed);
    if (!Array.isArray(values)) throw new Error("JSON trace must be an array");
    records = values.map((record, index) => normalizeRecord(record, index + 1));
  } else {
    records = trimmed
      .split(/\r?\n/)
      .filter((line) => line.trim())
      .map((line, index) => normalizeRecord(JSON.parse(line), index + 1));
  }
  return validateTraceOrder(records);
}

function format(value, digits, unit) {
  return `${value.toFixed(digits)} ${unit}`;
}

function setText(id, text) {
  $(id).textContent = text;
}

function logEvent(message, timeS = null) {
  const log = $("eventLog");
  const row = document.createElement("div");
  const time = document.createElement("span");
  const body = document.createElement("span");
  time.className = "log-time";
  time.textContent = timeS === null ? "console" : `${timeS.toFixed(3)} s`;
  body.textContent = message;
  row.append(time, body);
  log.prepend(row);
  while (log.children.length > 10) log.removeChild(log.lastElementChild);
}

function setTraceStatus(text, kind = "neutral") {
  setText("traceStatus", text);
  const dot = $("traceStatus").parentElement.querySelector(".status-dot");
  dot.className = `status-dot ${kind}`;
}

function renderGround(forwardPositionM) {
  const group = $("groundTicks");
  const spacingPx = 70;
  const shift = ((-forwardPositionM * 240) % spacingPx + spacingPx) % spacingPx;
  const lines = [];
  for (let x = -30 + shift; x < 670; x += spacingPx) {
    lines.push(`<line class="ground-tick" x1="${x.toFixed(1)}" y1="295" x2="${x.toFixed(1)}" y2="307"/>`);
  }
  group.innerHTML = lines.join("");
}

function renderRecord(record) {
  const pitchDeg = record.body_pitch_rad * 180 / Math.PI;
  const rollDeg = record.body_roll_rad * 180 / Math.PI;
  const reactionDeg = record.reaction_position_rad * 180 / Math.PI;

  $("pitchBody").setAttribute("transform", `rotate(${pitchDeg.toFixed(4)} 0 206)`);
  $("rollBody").setAttribute("transform", `rotate(${-rollDeg.toFixed(4)} 0 207)`);
  $("reactionWheel").setAttribute("transform", `translate(0 132) rotate(${reactionDeg.toFixed(4)})`);
  renderGround(record.forward_position_m);

  setText("clock", `t = ${record.time_s.toFixed(3)} s`);
  setText("pitchValue", `Pitch ${pitchDeg.toFixed(2)}°`);
  setText("rollValue", `Roll ${rollDeg.toFixed(2)}°`);
  setText("forwardPosition", format(record.forward_position_m, 4, "m"));
  setText("forwardVelocity", format(record.forward_velocity_m_per_s, 4, "m/s"));
  setText("pitchRate", format(record.body_pitch_rate_rad_per_s, 4, "rad/s"));
  setText("rollRate", format(record.body_roll_rate_rad_per_s, 4, "rad/s"));
  setText("reactionAngle", format(record.reaction_position_rad, 4, "rad"));
  setText("reactionRate", format(record.reaction_rate_rad_per_s, 4, "rad/s"));
  setText("driveTorque", format(record.drive_torque_nm, 4, "N·m"));
  setText("reactionTorque", format(record.reaction_torque_nm, 4, "N·m"));
  setText("sampleIndex", `${state.index + 1} / ${state.trace.length}`);
}

function setIndex(index) {
  if (!state.trace.length) return;
  state.index = Math.max(0, Math.min(index, state.trace.length - 1));
  $("scrubber").value = String(state.index);
  renderRecord(state.trace[state.index]);
}

function stopPlayback() {
  state.playing = false;
  $("playPause").textContent = "Play";
  if (state.animationFrame !== null) cancelAnimationFrame(state.animationFrame);
  state.animationFrame = null;
}

function frame(nowMs) {
  if (!state.playing || !state.trace.length) return;
  const targetTimeS = state.simStartedS + ((nowMs - state.wallStartedMs) / 1000) * state.speed;
  let index = state.index;
  while (index + 1 < state.trace.length && state.trace[index + 1].time_s <= targetTimeS) index += 1;
  setIndex(index);

  if (index >= state.trace.length - 1) {
    stopPlayback();
    logEvent("Playback reached the end of the evidence trace.", state.trace[index].time_s);
    return;
  }
  state.animationFrame = requestAnimationFrame(frame);
}

function startPlayback() {
  if (!state.trace.length) return;
  if (state.index >= state.trace.length - 1) setIndex(0);
  state.playing = true;
  state.wallStartedMs = performance.now();
  state.simStartedS = state.trace[state.index].time_s;
  $("playPause").textContent = "Pause";
  logEvent(`Playback started at ${state.speed}×.`, state.trace[state.index].time_s);
  state.animationFrame = requestAnimationFrame(frame);
}

function showError(message) {
  stopPlayback();
  const badge = $("provenanceBadge");
  badge.textContent = message;
  badge.classList.add("error");
  setTraceStatus("TRACE ERROR", "warn");
  logEvent(`Trace rejected: ${message}`);
}

function loadTrace(trace, filename) {
  stopPlayback();
  state.trace = trace;
  state.filename = filename;
  state.speed = Number($("speed").value);
  $("scrubber").max = String(trace.length - 1);
  $("scrubber").disabled = false;
  $("playPause").disabled = false;
  $("restart").disabled = false;
  $("speed").disabled = false;

  const first = trace[0];
  const last = trace[trace.length - 1];
  const duration = last.time_s - first.time_s;
  const badge = $("provenanceBadge");
  badge.classList.remove("error");
  badge.textContent = `${filename} · ${trace.length} samples`;
  setText("traceName", filename);
  setText("sampleCount", String(trace.length));
  setText("durationValue", `${duration.toFixed(3)} s`);
  setTraceStatus("TRACE READY", "good");
  setIndex(0);
  logEvent(`Accepted ${filename}: ${trace.length} samples, ${duration.toFixed(3)} s duration.`);
}

function setView(view) {
  state.view = view;
  const viewport = $("modelViewport");
  viewport.classList.remove("split-mode", "side-mode", "front-mode");
  viewport.classList.add(`${view}-mode`);
  document.querySelectorAll(".view-button").forEach((button) => {
    button.classList.toggle("selected-view", button.dataset.view === view);
  });
}

$("traceFile").addEventListener("change", async (event) => {
  const file = event.target.files?.[0];
  if (!file) return;
  try {
    loadTrace(parseTrace(await file.text()), file.name);
  } catch (error) {
    showError(error instanceof Error ? error.message : String(error));
  }
});

$("playPause").addEventListener("click", () => {
  if (state.playing) {
    const timeS = state.trace[state.index]?.time_s ?? null;
    stopPlayback();
    logEvent("Playback paused.", timeS);
  } else {
    startPlayback();
  }
});

$("restart").addEventListener("click", () => {
  stopPlayback();
  setIndex(0);
  if (state.trace.length) logEvent("Playback returned to the first sample.", state.trace[0].time_s);
});

$("speed").addEventListener("change", () => {
  const wasPlaying = state.playing;
  if (wasPlaying) stopPlayback();
  state.speed = Number($("speed").value);
  logEvent(`Playback speed set to ${state.speed}×.`, state.trace[state.index]?.time_s ?? null);
  if (wasPlaying) startPlayback();
});

$("scrubber").addEventListener("input", () => {
  stopPlayback();
  setIndex(Number($("scrubber").value));
});

document.querySelectorAll(".view-button").forEach((button) => {
  button.addEventListener("click", () => setView(button.dataset.view));
});

renderGround(0);
setView("split");

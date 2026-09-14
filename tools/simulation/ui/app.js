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

const CLOSED_LOOP_MODE = "closed_loop_production_path";
const ESTIMATE_STATE_FIELDS = [
  "forward_position_m",
  "forward_velocity_m_per_s",
  "body_pitch_rad",
  "body_pitch_rate_rad_per_s",
  "body_roll_rad",
  "body_roll_rate_rad_per_s",
  "reaction_rate_rad_per_s",
];
const TRUTH_STATE_FIELDS = [
  "forward_position_m",
  "forward_velocity_m_per_s",
  "body_pitch_rad",
  "body_pitch_rate_rad_per_s",
  "body_roll_rad",
  "body_roll_rate_rad_per_s",
  "reaction_position_rad",
];

const $ = (id) => document.getElementById(id);
const state = {
  trace: [],
  dialect: null,
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

function requireObject(value, field) {
  if (!value || typeof value !== "object" || Array.isArray(value)) {
    throw new Error(`${field} is not an object`);
  }
  return value;
}

function normalizeStateObject(value, fields, field) {
  const source = requireObject(value, field);
  return Object.fromEntries(
    fields.map((name) => [name, finiteNumber(source[name], `${field}.${name}`)])
  );
}

function normalizeCommonRecord(record, lineNumber) {
  requireObject(record, `record ${lineNumber}`);
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
  const values = Object.fromEntries(
    COMMON_TRACE_FIELDS.map((field) => [field, finiteNumber(record[field], field)])
  );
  return {
    dialect: "simulator-neutral-v2",
    time_s: values.time_s,
    visual: values,
    common: values,
    truth: null,
    production: null,
  };
}

function normalizeClosedLoopRecord(record, lineNumber) {
  requireObject(record, `record ${lineNumber}`);
  if (record.mode !== CLOSED_LOOP_MODE) {
    throw new Error(`record ${lineNumber} has unexpected mode`);
  }

  const raw = requireObject(record.raw_device_observation, `record ${lineNumber}.raw_device_observation`);
  const production = requireObject(record.production, `record ${lineNumber}.production`);
  const truth = normalizeStateObject(
    record.webots_evidence_truth,
    TRUTH_STATE_FIELDS,
    `record ${lineNumber}.webots_evidence_truth`
  );
  if (Number(raw.schema) !== 1 || raw.mapping_id !== "webots-body-identity-v1") {
    throw new Error(`record ${lineNumber} violates the #17 raw-device bridge contract`);
  }

  const rawSampleIndex = Number(raw.sample_index);
  const productionSampleIndex = Number(production.sample_index);
  if (!Number.isInteger(rawSampleIndex) || productionSampleIndex !== rawSampleIndex) {
    throw new Error(`record ${lineNumber} sample identity mismatch`);
  }
  const timestampUs = Number(raw.timestamp_us);
  if (!Number.isFinite(timestampUs)) throw new Error(`record ${lineNumber}.raw_device_observation.timestamp_us is not finite`);

  const estimate = production.estimate === null || production.estimate === undefined
    ? null
    : normalizeStateObject(production.estimate, ESTIMATE_STATE_FIELDS, `record ${lineNumber}.production.estimate`);
  const reference = normalizeStateObject(
    production.reference,
    ESTIMATE_STATE_FIELDS,
    `record ${lineNumber}.production.reference`
  );

  const actuation = String(production.actuation);
  const authority = production.authority === null || production.authority === undefined
    ? null
    : String(production.authority);
  const runtimeFaultBits = Number(production.runtime_fault_bits);
  const authorityReasonBits = Number(production.authority_reason_bits);
  if (!Number.isInteger(runtimeFaultBits) || runtimeFaultBits < 0) {
    throw new Error(`record ${lineNumber}.production.runtime_fault_bits is invalid`);
  }
  if (!Number.isInteger(authorityReasonBits) || authorityReasonBits < 0) {
    throw new Error(`record ${lineNumber}.production.authority_reason_bits is invalid`);
  }
  if (typeof production.constrained !== "boolean" || typeof production.hold_integrator !== "boolean") {
    throw new Error(`record ${lineNumber} has invalid runtime boolean evidence`);
  }
  const productionDrive = finiteNumber(production.drive_torque_nm, `record ${lineNumber}.production.drive_torque_nm`);
  const productionReaction = finiteNumber(production.reaction_torque_nm, `record ${lineNumber}.production.reaction_torque_nm`);
  const authorizedDrive = finiteNumber(record.authorized_drive_torque_nm, `record ${lineNumber}.authorized_drive_torque_nm`);
  const authorizedReaction = finiteNumber(record.authorized_reaction_torque_nm, `record ${lineNumber}.authorized_reaction_torque_nm`);

  if (actuation === "revoke") {
    if (authorizedDrive !== 0 || authorizedReaction !== 0) {
      throw new Error(`record ${lineNumber} revoked authority carries nonzero applied torque`);
    }
  } else if (actuation === "apply") {
    if (authority !== "closed_loop") {
      throw new Error(`record ${lineNumber} applies torque without closed_loop authority`);
    }
    if (Math.abs(authorizedDrive - productionDrive) > 1e-9 || Math.abs(authorizedReaction - productionReaction) > 1e-9) {
      throw new Error(`record ${lineNumber} authorized torque differs from production output`);
    }
  } else {
    throw new Error(`record ${lineNumber} has unknown actuation ${actuation}`);
  }

  const timeS = finiteNumber(record.time_s, `record ${lineNumber}.time_s`);
  return {
    dialect: "production-semantic-v1",
    time_s: timeS,
    visual: {
      time_s: timeS,
      forward_position_m: truth.forward_position_m,
      forward_velocity_m_per_s: truth.forward_velocity_m_per_s,
      body_pitch_rad: truth.body_pitch_rad,
      body_pitch_rate_rad_per_s: truth.body_pitch_rate_rad_per_s,
      body_roll_rad: truth.body_roll_rad,
      body_roll_rate_rad_per_s: truth.body_roll_rate_rad_per_s,
      reaction_position_rad: truth.reaction_position_rad,
      reaction_rate_rad_per_s: estimate?.reaction_rate_rad_per_s ?? null,
      drive_torque_nm: authorizedDrive,
      reaction_torque_nm: authorizedReaction,
    },
    truth,
    production: {
      sample_index: rawSampleIndex,
      timestamp_us: timestampUs,
      operating_state: String(production.operating_state),
      runtime_fault_bits: runtimeFaultBits,
      sensor_timing: String(production.sensor_timing),
      estimate_validity: production.estimate_validity === null || production.estimate_validity === undefined
        ? null
        : String(production.estimate_validity),
      authority,
      authority_reason_bits: authorityReasonBits,
      constrained: Boolean(production.constrained),
      hold_integrator: Boolean(production.hold_integrator),
      actuation,
      drive_torque_nm: productionDrive,
      reaction_torque_nm: productionReaction,
      estimate,
      reference,
    },
    raw,
  };
}

function parseJsonRecords(text) {
  const trimmed = text.trim();
  if (!trimmed) throw new Error("trace is empty");
  if (trimmed.startsWith("[")) {
    const values = JSON.parse(trimmed);
    if (!Array.isArray(values)) throw new Error("JSON trace must be an array");
    return values;
  }
  return trimmed
    .split(/\r?\n/)
    .filter((line) => line.trim())
    .map((line) => JSON.parse(line));
}

function detectDialect(record) {
  if (record?.mode === CLOSED_LOOP_MODE) return "production-semantic-v1";
  return "simulator-neutral-v2";
}

function validateTraceOrder(records) {
  if (records.length < 2) throw new Error("trace must contain at least two records");
  for (let i = 1; i < records.length; i += 1) {
    if (records[i].time_s <= records[i - 1].time_s) {
      throw new Error(`time_s must increase strictly at record ${i + 1}`);
    }
    if (
      records[i].dialect === "production-semantic-v1" &&
      records[i].production.sample_index !== records[i - 1].production.sample_index + 1
    ) {
      throw new Error(`closed-loop sample index must increase by one at record ${i + 1}`);
    }
  }
  return records;
}

function parseTrace(text) {
  const rawRecords = parseJsonRecords(text);
  if (rawRecords.length === 0) throw new Error("trace is empty");
  const dialect = detectDialect(rawRecords[0]);
  const normalized = rawRecords.map((record, index) => {
    if (detectDialect(record) !== dialect) {
      throw new Error(`record ${index + 1} mixes trace dialects`);
    }
    return dialect === "production-semantic-v1"
      ? normalizeClosedLoopRecord(record, index + 1)
      : normalizeCommonRecord(record, index + 1);
  });
  return { dialect, records: validateTraceOrder(normalized) };
}

function format(value, digits, unit) {
  if (value === null || value === undefined || !Number.isFinite(Number(value))) return "—";
  return `${Number(value).toFixed(digits)} ${unit}`;
}

function formatBare(value, digits = 6) {
  if (value === null || value === undefined || !Number.isFinite(Number(value))) return "—";
  return Number(value).toFixed(digits);
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
  while (log.children.length > 12) log.removeChild(log.lastElementChild);
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

function setClosedLoopPanels(visible) {
  ["comparisonPanel", "referencePanel", "runtimePanel"].forEach((id) => {
    $(id).hidden = !visible;
  });
}

function residual(truth, estimate, field) {
  if (!truth || !estimate) return null;
  const a = truth[field];
  const b = estimate[field];
  if (!Number.isFinite(a) || !Number.isFinite(b)) return null;
  return a - b;
}

function renderClosedLoopEvidence(record) {
  const truth = record.truth;
  const production = record.production;
  const estimate = production.estimate;
  const reference = production.reference;

  setText("truthPitch", format(truth.body_pitch_rad, 6, "rad"));
  setText("estimatePitch", format(estimate?.body_pitch_rad, 6, "rad"));
  setText("pitchResidual", format(residual(truth, estimate, "body_pitch_rad"), 6, "rad"));
  setText("truthRoll", format(truth.body_roll_rad, 6, "rad"));
  setText("estimateRoll", format(estimate?.body_roll_rad, 6, "rad"));
  setText("rollResidual", format(residual(truth, estimate, "body_roll_rad"), 6, "rad"));
  setText("truthForward", format(truth.forward_position_m, 6, "m"));
  setText("estimateForward", format(estimate?.forward_position_m, 6, "m"));
  setText("forwardResidual", format(residual(truth, estimate, "forward_position_m"), 6, "m"));
  setText("estimateReactionRate", format(estimate?.reaction_rate_rad_per_s, 6, "rad/s"));
  setText("truthReactionPhase", format(truth.reaction_position_rad, 6, "rad"));

  setText("referencePitch", format(reference.body_pitch_rad, 6, "rad"));
  setText("referenceRoll", format(reference.body_roll_rad, 6, "rad"));
  setText("referenceForwardVelocity", format(reference.forward_velocity_m_per_s, 6, "m/s"));
  setText("referenceReactionRate", format(reference.reaction_rate_rad_per_s, 6, "rad/s"));

  setText("operatingState", production.operating_state);
  setText("estimateValidity", production.estimate_validity ?? "unavailable");
  setText("sensorTiming", production.sensor_timing);
  setText("authorityState", production.authority ?? "not-yet-issued");
  setText("actuationState", production.actuation);
  setText("constrainedState", String(production.constrained));
  setText("holdIntegratorState", String(production.hold_integrator));
  setText("faultBits", `0x${Number(production.runtime_fault_bits).toString(16).padStart(4, "0")}`);
  setText("authorityReasonBits", `0x${Number(production.authority_reason_bits).toString(16).padStart(4, "0")}`);
  setText("authoritySummary", `${production.authority ?? "none"} / ${production.actuation}`);
}

function renderRecord(record) {
  const visual = record.visual;
  const pitchDeg = visual.body_pitch_rad * 180 / Math.PI;
  const rollDeg = visual.body_roll_rad * 180 / Math.PI;
  const reactionDeg = visual.reaction_position_rad * 180 / Math.PI;

  $("pitchBody").setAttribute("transform", `rotate(${pitchDeg.toFixed(4)} 0 206)`);
  $("rollBody").setAttribute("transform", `rotate(${-rollDeg.toFixed(4)} 0 207)`);
  $("reactionWheel").setAttribute("transform", `translate(0 132) rotate(${reactionDeg.toFixed(4)})`);
  renderGround(visual.forward_position_m);

  setText("clock", `t = ${record.time_s.toFixed(3)} s`);
  setText("pitchValue", `Pitch ${pitchDeg.toFixed(2)}°`);
  setText("rollValue", `Roll ${rollDeg.toFixed(2)}°`);
  setText("forwardPosition", format(visual.forward_position_m, 4, "m"));
  setText("forwardVelocity", format(visual.forward_velocity_m_per_s, 4, "m/s"));
  setText("pitchRate", format(visual.body_pitch_rate_rad_per_s, 4, "rad/s"));
  setText("rollRate", format(visual.body_roll_rate_rad_per_s, 4, "rad/s"));
  setText("reactionAngle", format(visual.reaction_position_rad, 4, "rad"));
  setText("reactionRate", format(visual.reaction_rate_rad_per_s, 4, "rad/s"));
  setText("driveTorque", format(visual.drive_torque_nm, 4, "N·m"));
  setText("reactionTorque", format(visual.reaction_torque_nm, 4, "N·m"));
  setText("sampleIndex", `${state.index + 1} / ${state.trace.length}`);

  if (record.dialect === "production-semantic-v1") {
    renderClosedLoopEvidence(record);
  }
}

function logClosedLoopTransition(previous, current) {
  if (!previous || !current || current.dialect !== "production-semantic-v1") return;
  const before = previous.production;
  const after = current.production;
  const events = [];
  if (before.operating_state !== after.operating_state) {
    events.push(`operating_state ${before.operating_state} → ${after.operating_state}`);
  }
  if (before.authority !== after.authority) {
    events.push(`authority ${before.authority ?? "none"} → ${after.authority ?? "none"}`);
  }
  if (before.actuation !== after.actuation) {
    events.push(`actuation ${before.actuation} → ${after.actuation}`);
  }
  if (before.estimate_validity !== after.estimate_validity) {
    events.push(`estimate ${before.estimate_validity ?? "none"} → ${after.estimate_validity ?? "none"}`);
  }
  if (events.length) logEvent(events.join("; "), current.time_s);
}

function setIndex(index) {
  if (!state.trace.length) return;
  const oldIndex = state.index;
  const newIndex = Math.max(0, Math.min(index, state.trace.length - 1));
  if (newIndex > oldIndex && state.dialect === "production-semantic-v1") {
    for (let i = oldIndex + 1; i <= newIndex; i += 1) {
      logClosedLoopTransition(state.trace[i - 1], state.trace[i]);
    }
  }
  state.index = newIndex;
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

function configureDialectUi(dialect) {
  const closedLoop = dialect === "production-semantic-v1";
  setClosedLoopPanels(closedLoop);
  setText("traceDialect", closedLoop ? "PRODUCTION EVIDENCE" : "SIMULATOR-NEUTRAL V2");
  setText("evidenceSource", closedLoop ? "#17 production-semantic evidence" : "simulator-neutral common trace");
  setText("contractTitle", closedLoop ? "Closed-loop production evidence" : "Simulator-neutral common observables");
  setText(
    "contractFields",
    closedLoop
      ? "truth · estimate · reference · operating_state · authority · actuation · authorized torque · raw-device sample identity"
      : COMMON_TRACE_FIELDS.join(" · ")
  );
  setText("authoritySummary", closedLoop ? "waiting for sample" : "not carried by common trace");
}

function loadTrace(parsed, filename) {
  stopPlayback();
  state.trace = parsed.records;
  state.dialect = parsed.dialect;
  state.filename = filename;
  state.index = 0;
  state.speed = Number($("speed").value);
  $("scrubber").max = String(state.trace.length - 1);
  $("scrubber").disabled = false;
  $("playPause").disabled = false;
  $("restart").disabled = false;
  $("speed").disabled = false;
  configureDialectUi(state.dialect);

  const first = state.trace[0];
  const last = state.trace[state.trace.length - 1];
  const duration = last.time_s - first.time_s;
  const badge = $("provenanceBadge");
  badge.classList.remove("error");
  badge.textContent = `${filename} · ${state.trace.length} samples`;
  setText("traceName", filename);
  setText("sampleCount", String(state.trace.length));
  setText("durationValue", `${duration.toFixed(3)} s`);
  setTraceStatus("TRACE READY", "good");
  setIndex(0);
  logEvent(`Accepted ${filename}: ${state.trace.length} samples, ${duration.toFixed(3)} s duration.`);
  if (state.dialect === "production-semantic-v1") {
    logEvent("Truth / estimate / reference / authority provenance kept separate.");
  }
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
  state.index = 0;
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
setClosedLoopPanels(false);
setView("split");

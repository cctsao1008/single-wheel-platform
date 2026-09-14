(() => {
  const LIVE_HISTORY_LIMIT = 3600;
  const LIVE_TREND_REFRESH_MS = 100;
  const LIVE_MAPPING_ID = "sitl-simulation-world-v1";
  let liveSource = null;
  let liveMeta = null;
  let lastTrendRefreshMs = 0;

  function addLiveStyles() {
    const style = document.createElement("style");
    style.textContent = `
      .topbar-actions { display:flex; align-items:center; justify-content:flex-end; gap:8px; flex-wrap:wrap; }
      .live-speed-label { display:inline-flex; align-items:center; gap:6px; color:#aeb3b8; font-size:.72rem; }
      .live-speed-label select { min-height:34px; padding:0 7px; border:1px solid #4a4f55; border-radius:4px; color:#d7dadd; background:#292c30; }
      .live-button, .live-stop-button { min-height:42px; padding:0 14px; border-radius:5px; font-weight:800; cursor:pointer; }
      .live-button { border:1px solid #718e4f; color:#e8f1dd; background:#42552f; }
      .live-stop-button { border:1px solid #95554d; color:#ffd8d1; background:#552f2b; }
      .live-button:disabled, .live-stop-button:disabled { opacity:.42; cursor:not-allowed; }
      .live-boundary { border-color:#70443e !important; color:#efb0a5 !important; }
      @media (max-width: 1200px) { .topbar { grid-template-columns:1fr; } .system-strip, .topbar-actions { justify-content:flex-start; } }
    `;
    document.head.appendChild(style);
  }

  function liveRecord(record, sequence) {
    requireObject(record, `live record ${sequence}`);
    if (record.mode !== CLOSED_LOOP_MODE) {
      throw new Error(`live record ${sequence} has unexpected mode`);
    }
    const raw = requireObject(record.raw_device_observation, `live record ${sequence}.raw_device_observation`);
    if (Number(raw.schema) !== 1 || raw.mapping_id !== LIVE_MAPPING_ID) {
      throw new Error(`live record ${sequence} violates the SITL raw-observation mapping contract`);
    }
    const production = requireObject(record.production, `live record ${sequence}.production`);
    const truth = normalizeStateObject(
      record.sitl_evidence_truth,
      TRUTH_STATE_FIELDS,
      `live record ${sequence}.sitl_evidence_truth`
    );
    const estimate = production.estimate === null || production.estimate === undefined
      ? null
      : normalizeStateObject(production.estimate, ESTIMATE_STATE_FIELDS, `live record ${sequence}.production.estimate`);
    const reference = normalizeStateObject(
      production.reference,
      ESTIMATE_STATE_FIELDS,
      `live record ${sequence}.production.reference`
    );

    const rawSampleIndex = Number(raw.sample_index);
    const productionSampleIndex = Number(production.sample_index);
    if (!Number.isInteger(rawSampleIndex) || productionSampleIndex !== rawSampleIndex) {
      throw new Error(`live record ${sequence} sample identity mismatch`);
    }
    const timestampUs = finiteNumber(raw.timestamp_us, `live record ${sequence}.timestamp_us`);
    const timeS = finiteNumber(record.time_s, `live record ${sequence}.time_s`);
    const authority = production.authority === null || production.authority === undefined
      ? null
      : String(production.authority);
    const actuation = String(production.actuation);
    const authorizedDrive = finiteNumber(record.authorized_drive_torque_nm, `live record ${sequence}.authorized_drive_torque_nm`);
    const authorizedReaction = finiteNumber(record.authorized_reaction_torque_nm, `live record ${sequence}.authorized_reaction_torque_nm`);
    const productionDrive = finiteNumber(production.drive_torque_nm, `live record ${sequence}.production.drive_torque_nm`);
    const productionReaction = finiteNumber(production.reaction_torque_nm, `live record ${sequence}.production.reaction_torque_nm`);
    const runtimeFaultBits = Number(production.runtime_fault_bits);
    const authorityReasonBits = Number(production.authority_reason_bits);
    if (!Number.isInteger(runtimeFaultBits) || runtimeFaultBits < 0) throw new Error("invalid live runtime fault bits");
    if (!Number.isInteger(authorityReasonBits) || authorityReasonBits < 0) throw new Error("invalid live authority reason bits");
    if (typeof production.constrained !== "boolean" || typeof production.hold_integrator !== "boolean") {
      throw new Error("invalid live runtime boolean evidence");
    }

    if (actuation === "revoke") {
      if (authorizedDrive !== 0 || authorizedReaction !== 0) {
        throw new Error("revoked live authority carries nonzero applied torque");
      }
    } else if (actuation === "apply") {
      if (authority !== "closed_loop") throw new Error("live actuation applied without closed_loop authority");
      if (Math.abs(productionDrive - authorizedDrive) > 1e-9 || Math.abs(productionReaction - authorizedReaction) > 1e-9) {
        throw new Error("live production torque differs from applied authorized torque");
      }
    } else {
      throw new Error(`unknown live actuation ${actuation}`);
    }

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
        constrained: production.constrained,
        hold_integrator: production.hold_integrator,
        actuation,
        drive_torque_nm: productionDrive,
        reaction_torque_nm: productionReaction,
        estimate,
        reference,
      },
      raw,
      source: record.source ?? null,
    };
  }

  function updateLiveChip(text, kind = "neutral") {
    $("liveChip").textContent = text;
    const dot = $("liveChip").parentElement.querySelector(".status-dot");
    dot.className = `status-dot ${kind}`;
  }

  function setReplayControls(enabled) {
    $("traceFile").disabled = !enabled;
    $("playPause").disabled = !enabled || !state.trace.length;
    $("restart").disabled = !enabled || !state.trace.length;
    $("speed").disabled = !enabled || !state.trace.length;
    $("scrubber").disabled = !enabled || !state.trace.length;
  }

  function disableComparisonForLive() {
    const input = $("compareFiles");
    if (input) input.disabled = true;
    const button = $("compareLoadButton");
    if (button) button.classList.add("disabled");
    const panel = $("comparisonTracePanel");
    if (panel) panel.hidden = true;
  }

  function configureLiveUi(meta) {
    stopPlayback();
    state.trace = [];
    state.dialect = "production-semantic-v1";
    state.index = 0;
    state.filename = "persistent-live-sitl";
    configureDialectUi(state.dialect);
    setText("traceDialect", "LIVE PRODUCTION SEMANTICS");
    setText("evidenceSource", "persistent Rust ClosedLoopSimulation · synthetic fixture");
    setText("traceName", "persistent live SITL");
    setText("sampleCount", "0 rolling");
    setText("durationValue", "0.000 s");
    setText("contractTitle", "Persistent production-semantic SITL evidence");
    setText("contractFields", "SITL truth · production estimate · reference · operating_state · authority · faults · applied authorized torque");
    setText("authoritySummary", "waiting for live sample");
    setTraceStatus("LIVE SITL", "good");
    const badge = $("provenanceBadge");
    badge.classList.remove("error");
    badge.textContent = `${meta?.source?.backend ?? "persistent Rust SITL"} · synthetic`;
    $("scrubber").min = "0";
    $("scrubber").max = "0";
    $("scrubber").value = "0";
    setReplayControls(false);
    disableComparisonForLive();
    window.SingleConsoleTrends?.render();
  }

  function refreshLiveTrends(nowMs) {
    if (nowMs - lastTrendRefreshMs < LIVE_TREND_REFRESH_MS) return;
    lastTrendRefreshMs = nowMs;
    window.SingleConsoleTrends?.render();
  }

  function appendLiveRecord(rawRecord) {
    const normalized = liveRecord(rawRecord, state.trace.length + 1);
    const previous = state.trace.at(-1);
    if (previous) {
      if (normalized.time_s <= previous.time_s) throw new Error("live time_s did not increase strictly");
      if (normalized.production.sample_index <= previous.production.sample_index) {
        throw new Error("live sample index did not increase strictly");
      }
      logClosedLoopTransition(previous, normalized);
    }

    state.trace.push(normalized);
    if (state.trace.length > LIVE_HISTORY_LIMIT) state.trace.shift();
    state.index = state.trace.length - 1;
    $("scrubber").max = String(Math.max(0, state.trace.length - 1));
    $("scrubber").value = String(state.index);
    renderRecord(normalized);

    const first = state.trace[0];
    const duration = normalized.time_s - first.time_s;
    setText("sampleCount", `${state.trace.length} rolling`);
    setText("durationValue", `${duration.toFixed(3)} s rolling`);
    setText("traceName", `live · sample ${normalized.production.sample_index}`);
    refreshLiveTrends(performance.now());
  }

  function stopLive(label = "LIVE: stopped") {
    if (liveSource) {
      liveSource.close();
      liveSource = null;
    }
    $("liveStart").disabled = false;
    $("liveStop").disabled = true;
    $("liveSpeed").disabled = false;
    updateLiveChip(label, "neutral");
    if (state.trace.length && state.dialect === "production-semantic-v1" && state.filename === "persistent-live-sitl") {
      state.filename = "stopped-live-window";
      setReplayControls(true);
      $("scrubber").max = String(state.trace.length - 1);
      logEvent("Live transport stopped. The bounded rolling window remains available for local replay.", state.trace.at(-1).time_s);
      window.SingleConsoleTrends?.render();
    }
  }

  function failLive(message) {
    logEvent(`Live SITL error: ${message}`);
    const badge = $("provenanceBadge");
    badge.textContent = `live error · ${message}`;
    badge.classList.add("error");
    setTraceStatus("LIVE ERROR", "warn");
    stopLive("LIVE: error");
  }

  function startLive() {
    stopLive("LIVE: restarting");
    liveMeta = null;
    lastTrendRefreshMs = 0;
    $("liveStart").disabled = true;
    $("liveStop").disabled = false;
    $("liveSpeed").disabled = true;
    updateLiveChip("LIVE: connecting", "warn");
    stopPlayback();
    disableComparisonForLive();

    const speed = Number($("liveSpeed").value);
    const params = new URLSearchParams({ speed: String(speed), fps: "60" });
    liveSource = new EventSource(`/api/live?${params.toString()}`);

    liveSource.addEventListener("status", (event) => {
      const status = JSON.parse(event.data);
      if (status.phase === "starting") {
        updateLiveChip("LIVE: starting Rust SITL", "warn");
        logEvent("Starting persistent Rust ClosedLoopSimulation.");
      } else if (status.phase === "ended") {
        stopLive("LIVE: ended");
      }
    });

    liveSource.addEventListener("meta", (event) => {
      liveMeta = JSON.parse(event.data);
      if (liveMeta.dialect !== "production-semantic-v1" || liveMeta.mode !== CLOSED_LOOP_MODE) {
        failLive("unexpected live evidence dialect");
        return;
      }
      configureLiveUi(liveMeta);
      updateLiveChip(`LIVE: ${liveMeta.source?.inner_hz ?? "?"} Hz SITL → 60 fps`, "good");
      logEvent("Live path accepted: Rust semantics → localhost SSE → observer-only browser.");
    });

    liveSource.addEventListener("sample", (event) => {
      if (!liveMeta) {
        failLive("sample arrived before metadata");
        return;
      }
      try {
        appendLiveRecord(JSON.parse(event.data));
      } catch (error) {
        failLive(error instanceof Error ? error.message : String(error));
      }
    });

    liveSource.addEventListener("stream-error", (event) => {
      const detail = JSON.parse(event.data);
      failLive(detail.message ?? "live bridge error");
    });

    liveSource.onerror = () => {
      if (liveSource) failLive("localhost live bridge disconnected");
    };
  }

  $("liveStart").addEventListener("click", startLive);
  $("liveStop").addEventListener("click", () => stopLive("LIVE: stopped"));
  window.addEventListener("beforeunload", () => {
    if (liveSource) liveSource.close();
  });

  addLiveStyles();
  document.querySelector('.status-chip:has(#liveChip)')?.classList.add("live-boundary");
})();

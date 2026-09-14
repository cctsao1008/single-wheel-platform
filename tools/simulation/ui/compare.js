(() => {
  const SVG_NS = "http://www.w3.org/2000/svg";
  const WIDTH = 960;
  const HEIGHT = 270;
  const LEFT = 66;
  const RIGHT = 18;
  const PLOT_WIDTH = WIDTH - LEFT - RIGHT;
  const MAX_COMPARISONS = 3;
  const METRIC_FIELDS = [
    ["forward_position_m", "forward [m]"],
    ["body_pitch_rad", "pitch [rad]"],
    ["body_roll_rad", "roll [rad]"],
    ["drive_torque_nm", "drive torque [N·m]"],
    ["reaction_torque_nm", "reaction torque [N·m]"],
  ];
  const ROWS = {
    pitch: { top: 28, height: 92 },
    roll: { top: 148, height: 92 },
  };

  const comparisonState = {
    traces: [],
  };

  function svgElement(name, attrs = {}) {
    const node = document.createElementNS(SVG_NS, name);
    Object.entries(attrs).forEach(([key, value]) => node.setAttribute(key, String(value)));
    return node;
  }

  function addStyles() {
    const style = document.createElement("style");
    style.textContent = `
      .compare-load-button { display: inline-flex; align-items: center; justify-content: center; min-height: 34px; padding: 0 12px; border: 1px solid #676b70; border-radius: 3px; color: #d7d9dc; background: #35383c; cursor: pointer; font-size: 12px; font-weight: 600; white-space: nowrap; }
      .compare-load-button:hover { border-color: #d4a63a; color: #f0c354; }
      .compare-load-button.disabled { opacity: 0.45; cursor: not-allowed; }
      .compare-load-button input { display: none; }
      .compare-panel[hidden] { display: none; }
      .compare-toolbar { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; }
      .compare-badge { border: 1px solid #575b61; border-radius: 999px; padding: 3px 8px; font-size: 11px; color: #c5c8cc; background: #2d3034; }
      .compare-warning { margin: 0 0 10px; padding: 9px 11px; border-left: 3px solid #d8a636; background: rgba(216,166,54,0.08); color: #c6c9cd; font-size: 12px; }
      .compare-wrap { background: #1f2124; border: 1px solid #44484e; border-radius: 4px; overflow: hidden; }
      #comparisonSvg { display: block; width: 100%; height: auto; min-height: 220px; }
      .compare-grid { stroke: #45494f; stroke-width: 1; }
      .compare-zero { stroke: #686d74; stroke-width: 1; }
      .compare-label { fill: #aeb3ba; font-size: 12px; font-family: system-ui, sans-serif; }
      .compare-value { fill: #7f858d; font-size: 10px; font-family: ui-monospace, monospace; }
      .compare-line { fill: none; stroke-width: 2; vector-effect: non-scaling-stroke; }
      .compare-primary { stroke: #f0b43c; }
      .compare-0 { stroke: #7fc6ff; }
      .compare-1 { stroke: #a4d37b; }
      .compare-2 { stroke: #d79de7; }
      .compare-roll { stroke-dasharray: 6 3; }
      .compare-cursor { stroke: #f4cd69; stroke-width: 1.5; vector-effect: non-scaling-stroke; pointer-events: none; }
      .compare-table { width: 100%; border-collapse: collapse; margin-top: 12px; font-size: 12px; }
      .compare-table th, .compare-table td { padding: 6px 8px; border-top: 1px solid #45494f; text-align: right; font-family: ui-monospace, monospace; }
      .compare-table th:first-child, .compare-table td:first-child { text-align: left; font-family: system-ui, sans-serif; }
      .compare-table thead th { color: #bfc3c8; font-family: system-ui, sans-serif; }
      .compare-table tbody th { color: #d7d9dc; font-weight: 500; }
      .compare-file-note { color: #8e949c; font-size: 11px; margin-top: 8px; }
    `;
    document.head.appendChild(style);
  }

  function installUi() {
    if (document.getElementById("compareFiles")) return;

    const primaryLoad = document.querySelector(".load-button");
    if (primaryLoad) {
      const label = document.createElement("label");
      label.className = "compare-load-button disabled";
      label.id = "compareLoadButton";
      label.innerHTML = `Compare traces<input id="compareFiles" type="file" accept=".jsonl,.json,.txt" multiple disabled>`;
      primaryLoad.parentElement.insertBefore(label, primaryLoad);
    }

    const trendPanel = document.getElementById("trendPanel");
    const contractPanel = document.getElementById("contractPanel");
    const anchor = trendPanel ?? contractPanel;
    if (!anchor) return;

    const panel = document.createElement("article");
    panel.className = "panel compare-panel";
    panel.id = "comparisonTracePanel";
    panel.hidden = true;
    panel.innerHTML = `
      <div class="panel-header compact-header">
        <div>
          <div class="panel-kicker">Cross-trace discrepancy review</div>
          <h2>Exact-grid comparison</h2>
          <p>Primary versus local simulator-neutral traces. Agreement is not a vote; disagreement is an investigation target.</p>
        </div>
        <div class="compare-toolbar" id="comparisonLegend"></div>
      </div>
      <p class="compare-warning">No averaging, winner selection, backend inference, smoothing, resampling, interpolation, or majority-vote physics is performed here.</p>
      <div class="compare-wrap">
        <svg id="comparisonSvg" viewBox="0 0 ${WIDTH} ${HEIGHT}" role="img" aria-label="Exact-grid cross-trace pitch and roll comparison">
          <g id="comparisonStatic"></g>
          <g id="comparisonSeries"></g>
          <line id="comparisonCursor" class="compare-cursor" x1="${LEFT}" x2="${LEFT}" y1="18" y2="250" visibility="hidden"></line>
        </svg>
      </div>
      <table class="compare-table" id="comparisonTable">
        <thead><tr id="comparisonTableHeader"><th>max |primary − comparison|</th></tr></thead>
        <tbody id="comparisonTableBody"></tbody>
      </table>
      <p class="compare-file-note" id="comparisonFileNote">Local filenames are display labels only; they are not authoritative backend provenance.</p>
    `;
    anchor.parentElement.insertBefore(panel, contractPanel);

    const contractNav = document.querySelector('.sidebar a[href="#contractPanel"]');
    if (contractNav) {
      const nav = document.createElement("a");
      nav.className = "nav-item";
      nav.href = "#comparisonTracePanel";
      nav.innerHTML = "<span>≠</span> Compare";
      contractNav.parentElement.insertBefore(nav, contractNav);
    }

    document.getElementById("compareFiles").addEventListener("change", onComparisonFiles);
    drawStaticFrame();
  }

  function drawStaticFrame() {
    const group = document.getElementById("comparisonStatic");
    if (!group) return;
    group.textContent = "";
    Object.entries(ROWS).forEach(([name, row]) => {
      const center = row.top + row.height / 2;
      group.appendChild(svgElement("line", { class: "compare-grid", x1: LEFT, x2: WIDTH - RIGHT, y1: row.top, y2: row.top }));
      group.appendChild(svgElement("line", { class: "compare-zero", x1: LEFT, x2: WIDTH - RIGHT, y1: center, y2: center }));
      group.appendChild(svgElement("line", { class: "compare-grid", x1: LEFT, x2: WIDTH - RIGHT, y1: row.top + row.height, y2: row.top + row.height }));
      const label = svgElement("text", { class: "compare-label", x: 8, y: row.top + 15 });
      label.textContent = `${name} [rad]`;
      group.appendChild(label);
    });
  }

  function enableCompare(enabled) {
    const input = document.getElementById("compareFiles");
    const label = document.getElementById("compareLoadButton");
    if (!input || !label) return;
    input.disabled = !enabled;
    label.classList.toggle("disabled", !enabled);
  }

  function resetComparisons(reason = null) {
    comparisonState.traces = [];
    const panel = document.getElementById("comparisonTracePanel");
    if (panel) panel.hidden = true;
    const input = document.getElementById("compareFiles");
    if (input) input.value = "";
    if (reason) logEvent(reason);
    renderComparisonCursor();
  }

  function requireCompatible(parsed, filename) {
    if (state.dialect !== "simulator-neutral-v2") {
      throw new Error("primary trace must be simulator-neutral-v2 for cross-trace comparison");
    }
    if (parsed.dialect !== "simulator-neutral-v2") {
      throw new Error(`${filename}: comparison trace must be simulator-neutral-v2`);
    }
    if (parsed.records.length !== state.trace.length) {
      throw new Error(`${filename}: sample count ${parsed.records.length} != primary ${state.trace.length}`);
    }
    for (let index = 0; index < state.trace.length; index += 1) {
      if (parsed.records[index].time_s !== state.trace[index].time_s) {
        throw new Error(`${filename}: time_s grid mismatch at sample ${index}; exact alignment is required`);
      }
    }
    return parsed.records;
  }

  async function onComparisonFiles(event) {
    const files = [...(event.target.files ?? [])];
    if (!files.length) return;
    if (files.length > MAX_COMPARISONS) {
      resetComparisons();
      logEvent(`Comparison rejected: select at most ${MAX_COMPARISONS} traces.`);
      return;
    }

    try {
      const accepted = [];
      for (const file of files) {
        const parsed = parseTrace(await file.text());
        accepted.push({ filename: file.name, records: requireCompatible(parsed, file.name) });
      }
      comparisonState.traces = accepted;
      renderComparison();
      document.getElementById("comparisonTracePanel").hidden = false;
      logEvent(`Accepted ${accepted.length} exact-grid comparison trace${accepted.length === 1 ? "" : "s"}.`);
    } catch (error) {
      resetComparisons();
      logEvent(`Comparison rejected: ${error instanceof Error ? error.message : String(error)}`);
    }
  }

  function allAttitudeValues() {
    const values = [];
    const traces = [{ records: state.trace }, ...comparisonState.traces];
    traces.forEach(({ records }) => records.forEach((record) => {
      values.push(record.common.body_pitch_rad, record.common.body_roll_rad);
    }));
    return values.filter(Number.isFinite);
  }

  function symmetricLimit(values, fallback = 0.01) {
    const maximum = values.length ? Math.max(...values.map((value) => Math.abs(value))) : 0;
    return maximum > 0 ? maximum * 1.12 : fallback;
  }

  function xScale(timeS) {
    const first = state.trace[0].time_s;
    const last = state.trace[state.trace.length - 1].time_s;
    if (last <= first) return LEFT;
    return LEFT + ((timeS - first) / (last - first)) * PLOT_WIDTH;
  }

  function yScale(value, row, limit) {
    const center = row.top + row.height / 2;
    return center - (value / limit) * row.height * 0.42;
  }

  function pathFor(records, field, row, limit) {
    return records.map((record, index) => {
      const x = xScale(record.time_s).toFixed(2);
      const y = yScale(record.common[field], row, limit).toFixed(2);
      return `${index ? "L" : "M"}${x} ${y}`;
    }).join(" ");
  }

  function appendTracePair(group, records, className, limit) {
    group.appendChild(svgElement("path", {
      class: `compare-line ${className}`,
      d: pathFor(records, "body_pitch_rad", ROWS.pitch, limit),
    }));
    group.appendChild(svgElement("path", {
      class: `compare-line ${className} compare-roll`,
      d: pathFor(records, "body_roll_rad", ROWS.roll, limit),
    }));
  }

  function maxAbsDifference(comparisonRecords, field) {
    let maximum = 0;
    for (let index = 0; index < state.trace.length; index += 1) {
      maximum = Math.max(maximum, Math.abs(state.trace[index].common[field] - comparisonRecords[index].common[field]));
    }
    return maximum;
  }

  function renderComparison() {
    if (!comparisonState.traces.length || !state.trace.length) return;
    const limit = symmetricLimit(allAttitudeValues());
    const group = document.getElementById("comparisonSeries");
    group.textContent = "";
    appendTracePair(group, state.trace, "compare-primary", limit);
    comparisonState.traces.forEach((trace, index) => appendTracePair(group, trace.records, `compare-${index}`, limit));

    const staticGroup = document.getElementById("comparisonStatic");
    staticGroup.querySelectorAll(".compare-value.dynamic").forEach((node) => node.remove());
    Object.values(ROWS).forEach((row) => {
      const top = svgElement("text", { class: "compare-value dynamic", x: LEFT + 4, y: row.top + 11 });
      top.textContent = `+${limit.toPrecision(3)}`;
      const bottom = svgElement("text", { class: "compare-value dynamic", x: LEFT + 4, y: row.top + row.height - 4 });
      bottom.textContent = `-${limit.toPrecision(3)}`;
      staticGroup.append(top, bottom);
    });

    const legend = document.getElementById("comparisonLegend");
    legend.textContent = "";
    const primary = document.createElement("span");
    primary.className = "compare-badge";
    primary.textContent = `primary: ${state.filename}`;
    legend.appendChild(primary);
    comparisonState.traces.forEach((trace) => {
      const badge = document.createElement("span");
      badge.className = "compare-badge";
      badge.textContent = trace.filename;
      legend.appendChild(badge);
    });

    const header = document.getElementById("comparisonTableHeader");
    header.innerHTML = "<th>max |primary − comparison|</th>";
    comparisonState.traces.forEach((trace) => {
      const th = document.createElement("th");
      th.textContent = trace.filename;
      header.appendChild(th);
    });

    const body = document.getElementById("comparisonTableBody");
    body.textContent = "";
    METRIC_FIELDS.forEach(([field, label]) => {
      const row = document.createElement("tr");
      const th = document.createElement("th");
      th.textContent = label;
      row.appendChild(th);
      comparisonState.traces.forEach((trace) => {
        const td = document.createElement("td");
        td.textContent = maxAbsDifference(trace.records, field).toExponential(4);
        row.appendChild(td);
      });
      body.appendChild(row);
    });
    renderComparisonCursor();
  }

  function renderComparisonCursor() {
    const cursor = document.getElementById("comparisonCursor");
    if (!cursor) return;
    if (!comparisonState.traces.length || !state.trace.length) {
      cursor.setAttribute("visibility", "hidden");
      return;
    }
    const x = xScale(state.trace[state.index].time_s);
    cursor.setAttribute("x1", x.toFixed(2));
    cursor.setAttribute("x2", x.toFixed(2));
    cursor.setAttribute("visibility", "visible");
  }

  addStyles();
  installUi();

  const baseLoadTrace = loadTrace;
  loadTrace = function comparisonAwareLoadTrace(parsed, filename) {
    resetComparisons();
    baseLoadTrace(parsed, filename);
    enableCompare(parsed.dialect === "simulator-neutral-v2");
    if (parsed.dialect !== "simulator-neutral-v2") {
      logEvent("Cross-trace comparison disabled: production-semantic evidence is intentionally single-trace in #33.");
    }
  };

  const baseSetIndex = setIndex;
  setIndex = function comparisonAwareSetIndex(index) {
    baseSetIndex(index);
    renderComparisonCursor();
  };
})();

(() => {
  const SVG_NS = "http://www.w3.org/2000/svg";
  const WIDTH = 960;
  const HEIGHT = 320;
  const LEFT = 66;
  const RIGHT = 18;
  const PLOT_WIDTH = WIDTH - LEFT - RIGHT;
  const ROWS = {
    attitude: { top: 28, height: 112 },
    torque: { top: 162, height: 76 },
    runtime: { top: 260, height: 36 },
  };

  function svgElement(name, attrs = {}) {
    const node = document.createElementNS(SVG_NS, name);
    Object.entries(attrs).forEach(([key, value]) => node.setAttribute(key, String(value)));
    return node;
  }

  function addTrendStyles() {
    const style = document.createElement("style");
    style.textContent = `
      .trend-panel { overflow: hidden; }
      .trend-header { align-items: flex-start; }
      .trend-legend { display: flex; gap: 12px; flex-wrap: wrap; color: #b9bdc4; font-size: 12px; }
      .trend-legend span::before { content: ""; display: inline-block; width: 18px; height: 2px; margin-right: 6px; vertical-align: middle; background: currentColor; }
      .trend-legend .truth { color: #f0b43c; }
      .trend-legend .estimate { color: #7fc6ff; }
      .trend-legend .drive { color: #a4d37b; }
      .trend-legend .reaction { color: #d79de7; }
      .trend-wrap { background: #1f2124; border: 1px solid #44484e; border-radius: 4px; overflow: hidden; }
      #trendSvg { display: block; width: 100%; height: auto; min-height: 260px; cursor: crosshair; }
      .trend-grid { stroke: #45494f; stroke-width: 1; }
      .trend-zero { stroke: #686d74; stroke-width: 1; }
      .trend-label { fill: #aeb3ba; font-size: 12px; font-family: system-ui, sans-serif; }
      .trend-axis-value { fill: #7f858d; font-size: 10px; font-family: ui-monospace, monospace; }
      .trend-path { fill: none; stroke-width: 2; vector-effect: non-scaling-stroke; }
      .trend-truth-pitch { stroke: #f0b43c; }
      .trend-truth-roll { stroke: #d98b31; }
      .trend-estimate-pitch { stroke: #7fc6ff; stroke-dasharray: 6 4; }
      .trend-estimate-roll { stroke: #91a7ff; stroke-dasharray: 6 4; }
      .trend-drive { stroke: #a4d37b; }
      .trend-reaction { stroke: #d79de7; }
      .trend-cursor { stroke: #f4cd69; stroke-width: 1.5; vector-effect: non-scaling-stroke; pointer-events: none; }
      .runtime-band-apply { fill: rgba(130, 188, 90, 0.30); }
      .runtime-band-revoke { fill: rgba(205, 89, 74, 0.30); }
      .runtime-band-unavailable { fill: rgba(120, 125, 132, 0.18); }
      .runtime-marker { stroke: #e3b54c; stroke-width: 1; stroke-dasharray: 3 3; vector-effect: non-scaling-stroke; }
      .runtime-marker-label { fill: #d8bc71; font-size: 9px; font-family: ui-monospace, monospace; }
      .trend-note { margin: 8px 0 0; color: #8e949c; font-size: 12px; }
    `;
    document.head.appendChild(style);
  }

  function installTrendPanel() {
    const contract = document.getElementById("contractPanel");
    if (!contract || document.getElementById("trendPanel")) return;

    const panel = document.createElement("article");
    panel.className = "panel trend-panel";
    panel.id = "trendPanel";
    panel.innerHTML = `
      <div class="panel-header compact-header trend-header">
        <div>
          <div class="panel-kicker">Recorded evidence over time</div>
          <h2>Synchronized trends</h2>
          <p>Direct projection of recorded samples. No smoothing, resampling, filtering, or browser-side estimation.</p>
        </div>
        <div class="trend-legend" aria-label="trend legend">
          <span class="truth">truth / common</span>
          <span class="estimate">estimate</span>
          <span class="drive">drive torque</span>
          <span class="reaction">reaction torque</span>
        </div>
      </div>
      <div class="trend-wrap">
        <svg id="trendSvg" viewBox="0 0 ${WIDTH} ${HEIGHT}" role="img" aria-label="Synchronized trace trends">
          <g id="trendStatic"></g>
          <g id="trendRuntime"></g>
          <g id="trendSeries"></g>
          <line id="trendCursor" class="trend-cursor" x1="${LEFT}" x2="${LEFT}" y1="18" y2="300" visibility="hidden"></line>
        </svg>
      </div>
      <p class="trend-note" id="trendNote">Load evidence to populate recorded trends.</p>
    `;
    contract.parentElement.insertBefore(panel, contract);

    const nav = document.querySelector('.sidebar a[href="#contractPanel"]');
    if (nav) {
      const trendNav = document.createElement("a");
      trendNav.className = "nav-item";
      trendNav.href = "#trendPanel";
      trendNav.innerHTML = "<span>⌁</span> Trends";
      nav.parentElement.insertBefore(trendNav, nav);
    }

    document.getElementById("trendSvg").addEventListener("click", seekFromChart);
    drawStaticFrame();
  }

  function xScale(timeS) {
    if (!state.trace.length) return LEFT;
    const first = state.trace[0].time_s;
    const last = state.trace[state.trace.length - 1].time_s;
    if (last <= first) return LEFT;
    return LEFT + ((timeS - first) / (last - first)) * PLOT_WIDTH;
  }

  function finiteValues(values) {
    return values.filter((value) => Number.isFinite(value));
  }

  function symmetricLimit(values, fallback = 1) {
    const finite = finiteValues(values).map((value) => Math.abs(value));
    const maximum = finite.length ? Math.max(...finite) : 0;
    return maximum > 0 ? maximum * 1.12 : fallback;
  }

  function yScale(value, row, limit) {
    const center = row.top + row.height / 2;
    const span = row.height * 0.42;
    return center - (value / limit) * span;
  }

  function seriesPath(records, getter, row, limit) {
    let path = "";
    let drawing = false;
    records.forEach((record) => {
      const value = getter(record);
      if (!Number.isFinite(value)) {
        drawing = false;
        return;
      }
      const x = xScale(record.time_s).toFixed(2);
      const y = yScale(value, row, limit).toFixed(2);
      path += `${drawing ? " L" : "M"}${x} ${y}`;
      drawing = true;
    });
    return path;
  }

  function drawStaticFrame() {
    const group = document.getElementById("trendStatic");
    if (!group) return;
    group.textContent = "";

    Object.entries(ROWS).forEach(([name, row]) => {
      const zeroY = row.top + row.height / 2;
      group.appendChild(svgElement("line", { class: "trend-grid", x1: LEFT, x2: WIDTH - RIGHT, y1: row.top, y2: row.top }));
      group.appendChild(svgElement("line", { class: "trend-zero", x1: LEFT, x2: WIDTH - RIGHT, y1: zeroY, y2: zeroY }));
      group.appendChild(svgElement("line", { class: "trend-grid", x1: LEFT, x2: WIDTH - RIGHT, y1: row.top + row.height, y2: row.top + row.height }));
      const label = svgElement("text", { class: "trend-label", x: 8, y: row.top + 15 });
      label.textContent = name === "attitude" ? "attitude [rad]" : name === "torque" ? "torque [N·m]" : "runtime";
      group.appendChild(label);
    });
  }

  function appendPath(group, className, records, getter, row, limit) {
    const d = seriesPath(records, getter, row, limit);
    if (!d) return;
    group.appendChild(svgElement("path", { class: `trend-path ${className}`, d }));
  }

  function transitionIndices(records) {
    const result = [];
    if (!records.length || records[0].dialect !== "production-semantic-v1") return result;
    for (let index = 1; index < records.length; index += 1) {
      const previous = records[index - 1].production;
      const current = records[index].production;
      const labels = [];
      if (current.operating_state !== previous.operating_state) labels.push(current.operating_state);
      if (current.authority !== previous.authority) labels.push(`authority:${current.authority ?? "none"}`);
      if (current.actuation !== previous.actuation) labels.push(`actuation:${current.actuation}`);
      if (current.constrained !== previous.constrained && current.constrained) labels.push("constrained");
      if (current.runtime_fault_bits !== previous.runtime_fault_bits) labels.push(`fault:0x${current.runtime_fault_bits.toString(16)}`);
      if (labels.length) result.push({ index, labels });
    }
    return result;
  }

  function renderRuntimeLane(records) {
    const group = document.getElementById("trendRuntime");
    group.textContent = "";
    const row = ROWS.runtime;
    const laneY = row.top + 5;
    const laneHeight = row.height - 10;

    if (!records.length || records[0].dialect !== "production-semantic-v1") {
      group.appendChild(svgElement("rect", {
        class: "runtime-band-unavailable",
        x: LEFT,
        y: laneY,
        width: PLOT_WIDTH,
        height: laneHeight,
      }));
      const text = svgElement("text", { class: "trend-axis-value", x: LEFT + 8, y: laneY + 17 });
      text.textContent = "authority/runtime unavailable in simulator-neutral-v2";
      group.appendChild(text);
      return;
    }

    records.forEach((record, index) => {
      const startX = xScale(record.time_s);
      const endTime = index + 1 < records.length ? records[index + 1].time_s : record.time_s;
      const endX = index + 1 < records.length ? xScale(endTime) : WIDTH - RIGHT;
      const className = record.production.actuation === "apply" ? "runtime-band-apply" : "runtime-band-revoke";
      group.appendChild(svgElement("rect", {
        class: className,
        x: startX,
        y: laneY,
        width: Math.max(1, endX - startX),
        height: laneHeight,
      }));
    });

    transitionIndices(records).forEach(({ index, labels }, markerIndex) => {
      const x = xScale(records[index].time_s);
      group.appendChild(svgElement("line", {
        class: "runtime-marker",
        x1: x,
        x2: x,
        y1: ROWS.attitude.top,
        y2: row.top + row.height,
      }));
      const text = svgElement("text", {
        class: "runtime-marker-label",
        x: Math.min(x + 3, WIDTH - 150),
        y: row.top + 10 + (markerIndex % 2) * 11,
      });
      text.textContent = labels.join(" · ");
      group.appendChild(text);
    });
  }

  function renderTrends() {
    const records = state.trace;
    const series = document.getElementById("trendSeries");
    if (!series) return;
    series.textContent = "";
    renderRuntimeLane(records);

    if (!records.length) {
      updateTrendCursor();
      return;
    }

    const attitudeValues = [];
    records.forEach((record) => {
      attitudeValues.push(record.visual.body_pitch_rad, record.visual.body_roll_rad);
      if (record.production?.estimate) {
        attitudeValues.push(record.production.estimate.body_pitch_rad, record.production.estimate.body_roll_rad);
      }
    });
    const attitudeLimit = symmetricLimit(attitudeValues, 0.01);
    const torqueLimit = symmetricLimit(
      records.flatMap((record) => [record.visual.drive_torque_nm, record.visual.reaction_torque_nm]),
      0.01
    );

    appendPath(series, "trend-truth-pitch", records, (record) => record.visual.body_pitch_rad, ROWS.attitude, attitudeLimit);
    appendPath(series, "trend-truth-roll", records, (record) => record.visual.body_roll_rad, ROWS.attitude, attitudeLimit);
    appendPath(series, "trend-drive", records, (record) => record.visual.drive_torque_nm, ROWS.torque, torqueLimit);
    appendPath(series, "trend-reaction", records, (record) => record.visual.reaction_torque_nm, ROWS.torque, torqueLimit);

    if (records[0].dialect === "production-semantic-v1") {
      appendPath(series, "trend-estimate-pitch", records, (record) => record.production.estimate?.body_pitch_rad, ROWS.attitude, attitudeLimit);
      appendPath(series, "trend-estimate-roll", records, (record) => record.production.estimate?.body_roll_rad, ROWS.attitude, attitudeLimit);
      document.getElementById("trendNote").textContent =
        "Solid attitude = Webots evidence truth; dashed attitude = production estimate. Runtime markers are transition-only evidence.";
    } else {
      document.getElementById("trendNote").textContent =
        "Common trace: solid attitude and applied torque only. Runtime authority is intentionally unavailable.";
    }

    const staticGroup = document.getElementById("trendStatic");
    staticGroup.querySelectorAll(".trend-axis-value.dynamic").forEach((node) => node.remove());
    [
      { row: ROWS.attitude, limit: attitudeLimit, unit: "rad" },
      { row: ROWS.torque, limit: torqueLimit, unit: "N·m" },
    ].forEach(({ row, limit, unit }) => {
      const top = svgElement("text", { class: "trend-axis-value dynamic", x: LEFT + 4, y: row.top + 11 });
      top.textContent = `+${limit.toPrecision(3)} ${unit}`;
      const bottom = svgElement("text", { class: "trend-axis-value dynamic", x: LEFT + 4, y: row.top + row.height - 4 });
      bottom.textContent = `-${limit.toPrecision(3)} ${unit}`;
      staticGroup.append(top, bottom);
    });

    updateTrendCursor();
  }

  function updateTrendCursor() {
    const cursor = document.getElementById("trendCursor");
    if (!cursor) return;
    if (!state.trace.length) {
      cursor.setAttribute("visibility", "hidden");
      return;
    }
    const x = xScale(state.trace[state.index].time_s);
    cursor.setAttribute("x1", x.toFixed(2));
    cursor.setAttribute("x2", x.toFixed(2));
    cursor.setAttribute("visibility", "visible");
  }

  function seekFromChart(event) {
    if (!state.trace.length) return;
    const svg = document.getElementById("trendSvg");
    const point = svg.createSVGPoint();
    point.x = event.clientX;
    point.y = event.clientY;
    const local = point.matrixTransform(svg.getScreenCTM().inverse());
    const ratio = Math.max(0, Math.min(1, (local.x - LEFT) / PLOT_WIDTH));
    const first = state.trace[0].time_s;
    const last = state.trace[state.trace.length - 1].time_s;
    const target = first + ratio * (last - first);
    let best = 0;
    let bestDistance = Infinity;
    state.trace.forEach((record, index) => {
      const distance = Math.abs(record.time_s - target);
      if (distance < bestDistance) {
        bestDistance = distance;
        best = index;
      }
    });
    stopPlayback();
    setIndex(best);
    logEvent("Trend plot seeked to recorded sample.", state.trace[best].time_s);
  }

  addTrendStyles();
  installTrendPanel();

  const baseLoadTrace = loadTrace;
  loadTrace = function trendAwareLoadTrace(parsed, filename) {
    baseLoadTrace(parsed, filename);
    renderTrends();
  };

  const baseSetIndex = setIndex;
  setIndex = function trendAwareSetIndex(index) {
    baseSetIndex(index);
    updateTrendCursor();
  };
})();

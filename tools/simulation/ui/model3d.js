(() => {
  const canvas = document.getElementById("spatialCanvas");
  if (!canvas) return;

  const ctx = canvas.getContext("2d");
  const DEG = 180 / Math.PI;
  const BODY_PIVOT = [0, 0, 0.28];
  const COLORS = {
    background: "#25282c",
    grid: "#3a3f45",
    axisX: "#dc8e5e",
    axisY: "#7eb68b",
    axisZ: "#7da5d8",
    bodyFront: "#d2aa61",
    bodySide: "#b8873c",
    bodyTop: "#ead19b",
    edge: "#191b1e",
    wheel: "#202328",
    wheelEdge: "#747a82",
    hub: "#c5912f",
    reaction: "#dda52f",
    spoke: "#f6d77b",
    estimateGhost: "#7fc6ff",
    estimateGhostSoft: "rgba(127,198,255,0.20)",
    shadow: "rgba(0,0,0,0.23)",
  };

  // Project coordinate contract — presentation must not change these signs:
  // +X forward, +Y left, +Z up.
  // positive pitch = right-hand rotation about +Y -> rotateY(+pitch).
  // positive roll  = right-hand rotation about +X -> rotateX(+roll).
  // positive reaction-wheel phase = right-hand rotation about body +X.
  // The estimate ghost uses this same transform. It never receives a visual-only sign fix.
  const cameraPresets = {
    iso: { eye: [2.8, -3.8, 2.45], target: [0, 0, 0.70], focal: 680 },
    side: { eye: [0.2, -4.8, 1.20], target: [0, 0, 0.58], focal: 720 },
    front: { eye: [4.8, 0.2, 1.25], target: [0, 0, 0.58], focal: 720 },
  };
  const CAMERA_MIN_RADIUS = 1.8;
  const CAMERA_MAX_RADIUS = 7.5;
  const CAMERA_MIN_ELEVATION_RAD = 0.08;
  const CAMERA_MAX_ELEVATION_RAD = 1.45;
  const CAMERA_ORBIT_SENSITIVITY = 0.006;
  const CAMERA_ZOOM_SENSITIVITY = 0.001;

  const BODY_HALF = [0.18, 0.16, 0.33];
  const BODY_CENTER = [0, 0, 0.45];
  const BODY_EDGES = [
    [0, 1], [0, 2], [0, 4],
    [1, 3], [1, 5],
    [2, 3], [2, 6],
    [3, 7],
    [4, 5], [4, 6],
    [5, 7],
    [6, 7],
  ];

  function cloneCamera(camera) {
    return {
      eye: [...camera.eye],
      target: [...camera.target],
      focal: camera.focal,
    };
  }

  let cameraPreset = "iso";
  let cameraState = cloneCamera(cameraPresets.iso);
  let activeOrbitPointer = null;
  let latestRecord = null;
  let estimateGhostEnabled = true;

  function add(a, b) {
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
  }

  function sub(a, b) {
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  }

  function scale(v, scalar) {
    return [v[0] * scalar, v[1] * scalar, v[2] * scalar];
  }

  function dot(a, b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  }

  function cross(a, b) {
    return [
      a[1] * b[2] - a[2] * b[1],
      a[2] * b[0] - a[0] * b[2],
      a[0] * b[1] - a[1] * b[0],
    ];
  }

  function normalize(v) {
    const length = Math.hypot(v[0], v[1], v[2]);
    return length > 0 ? scale(v, 1 / length) : [0, 0, 0];
  }

  function clamp(value, minimum, maximum) {
    return Math.max(minimum, Math.min(maximum, value));
  }

  function rotateX(v, angle) {
    const c = Math.cos(angle);
    const s = Math.sin(angle);
    return [v[0], v[1] * c - v[2] * s, v[1] * s + v[2] * c];
  }

  function rotateY(v, angle) {
    const c = Math.cos(angle);
    const s = Math.sin(angle);
    return [v[0] * c + v[2] * s, v[1], -v[0] * s + v[2] * c];
  }

  function bodyRotate(v, pitch, roll) {
    // Roll and pitch are carried state coordinates, not integrated here.
    return rotateY(rotateX(v, roll), pitch);
  }

  function bodyPoint(local, pitch, roll) {
    return add(BODY_PIVOT, bodyRotate(local, pitch, roll));
  }

  function bodyVertices(pitch, roll) {
    const vertices = [];
    [-1, 1].forEach((sx) => {
      [-1, 1].forEach((sy) => {
        [-1, 1].forEach((sz) => {
          vertices.push(bodyPoint([
            BODY_CENTER[0] + sx * BODY_HALF[0],
            BODY_CENTER[1] + sy * BODY_HALF[1],
            BODY_CENTER[2] + sz * BODY_HALF[2],
          ], pitch, roll));
        });
      });
    });
    return vertices;
  }

  function cameraBasis() {
    // Camera state is presentation-only. Evidence/world coordinates are never rewritten from it.
    const camera = cameraState;
    const forward = normalize(sub(camera.target, camera.eye));
    const right = normalize(cross(forward, [0, 0, 1]));
    const up = normalize(cross(right, forward));
    return { ...camera, forward, right, up };
  }

  function cameraSpherical() {
    const relative = sub(cameraState.eye, cameraState.target);
    const radius = Math.hypot(relative[0], relative[1], relative[2]);
    return {
      radius,
      azimuth: Math.atan2(relative[1], relative[0]),
      elevation: Math.asin(clamp(relative[2] / radius, -1, 1)),
    };
  }

  function setCameraSpherical(radius, azimuth, elevation) {
    const boundedRadius = clamp(radius, CAMERA_MIN_RADIUS, CAMERA_MAX_RADIUS);
    const boundedElevation = clamp(elevation, CAMERA_MIN_ELEVATION_RAD, CAMERA_MAX_ELEVATION_RAD);
    const horizontal = boundedRadius * Math.cos(boundedElevation);
    cameraState.eye = add(cameraState.target, [
      horizontal * Math.cos(azimuth),
      horizontal * Math.sin(azimuth),
      boundedRadius * Math.sin(boundedElevation),
    ]);
  }

  function updateCameraStatus() {
    const status = document.getElementById("spatialCameraState");
    if (!status) return;
    const spherical = cameraSpherical();
    const label = cameraPreset === "custom" ? "CUSTOM" : cameraPreset.toUpperCase();
    status.textContent = `camera ${label} · r ${spherical.radius.toFixed(2)} · az ${(spherical.azimuth * DEG).toFixed(0)}° · el ${(spherical.elevation * DEG).toFixed(0)}°`;
  }

  function updatePresetButtons() {
    document.querySelectorAll("[data-camera-preset]").forEach((button) => {
      button.classList.toggle("active-camera", button.dataset.cameraPreset === cameraPreset);
    });
  }

  function selectCameraPreset(preset) {
    if (!cameraPresets[preset]) return;
    cameraPreset = preset;
    cameraState = cloneCamera(cameraPresets[preset]);
    updatePresetButtons();
    updateCameraStatus();
    draw(latestRecord);
  }

  function markCameraCustom() {
    cameraPreset = "custom";
    updatePresetButtons();
  }

  function project(point, basis, width, height) {
    const relative = sub(point, basis.eye);
    const depth = dot(relative, basis.forward);
    if (depth <= 0.02) return null;
    const x = dot(relative, basis.right);
    const y = dot(relative, basis.up);
    return {
      x: width / 2 + basis.focal * x / depth,
      y: height / 2 - basis.focal * y / depth,
      depth,
    };
  }

  function resizeCanvas() {
    const rect = canvas.getBoundingClientRect();
    const ratio = Math.min(window.devicePixelRatio || 1, 2);
    const width = Math.max(640, Math.round(rect.width * ratio));
    const height = Math.max(380, Math.round(rect.height * ratio));
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }
  }

  function line3d(a, b, basis, width, height, stroke, lineWidth = 1) {
    const pa = project(a, basis, width, height);
    const pb = project(b, basis, width, height);
    if (!pa || !pb) return;
    ctx.beginPath();
    ctx.moveTo(pa.x, pa.y);
    ctx.lineTo(pb.x, pb.y);
    ctx.strokeStyle = stroke;
    ctx.lineWidth = lineWidth;
    ctx.stroke();
  }

  function polygon3d(points, basis, width, height, fill, stroke = COLORS.edge) {
    const projected = points.map((point) => project(point, basis, width, height));
    if (projected.some((point) => point === null)) return;
    ctx.beginPath();
    ctx.moveTo(projected[0].x, projected[0].y);
    for (let index = 1; index < projected.length; index += 1) {
      ctx.lineTo(projected[index].x, projected[index].y);
    }
    ctx.closePath();
    ctx.fillStyle = fill;
    ctx.fill();
    ctx.strokeStyle = stroke;
    ctx.lineWidth = 1.5;
    ctx.stroke();
  }

  function ring3d(center, axis, radius, pitch, roll, phase, basis, width, height, stroke, lineWidth, bodyAttached) {
    const points = [];
    const segments = 48;
    for (let index = 0; index <= segments; index += 1) {
      const angle = (index / segments) * Math.PI * 2;
      let local;
      if (axis === "x") {
        local = [0, radius * Math.cos(angle), radius * Math.sin(angle)];
      } else {
        local = [radius * Math.cos(angle), 0, radius * Math.sin(angle)];
      }
      const point = bodyAttached
        ? bodyPoint(add(center, local), pitch, roll)
        : add(center, local);
      points.push(project(point, basis, width, height));
    }

    ctx.beginPath();
    let drawing = false;
    points.forEach((point) => {
      if (!point) {
        drawing = false;
        return;
      }
      if (!drawing) ctx.moveTo(point.x, point.y);
      else ctx.lineTo(point.x, point.y);
      drawing = true;
    });
    ctx.strokeStyle = stroke;
    ctx.lineWidth = lineWidth;
    ctx.stroke();

    if (axis === "x" && bodyAttached && Number.isFinite(phase)) {
      [phase, phase + Math.PI / 2].forEach((angle) => {
        const a = bodyPoint(add(center, [0, radius * Math.cos(angle), radius * Math.sin(angle)]), pitch, roll);
        const b = bodyPoint(add(center, [0, -radius * Math.cos(angle), -radius * Math.sin(angle)]), pitch, roll);
        line3d(a, b, basis, width, height, COLORS.spoke, Math.max(2, lineWidth * 0.7));
      });
    }
  }

  function drawGround(basis, width, height, forwardPosition) {
    // Forward displacement shifts only the reference grid. Drive-wheel phase is intentionally not inferred.
    const spacing = 0.25;
    const offset = ((forwardPosition % spacing) + spacing) % spacing;
    for (let index = -8; index <= 8; index += 1) {
      const x = index * spacing - offset;
      line3d([x, -1.3, 0], [x, 1.3, 0], basis, width, height, COLORS.grid, 1);
    }
    for (let index = -5; index <= 5; index += 1) {
      const y = index * spacing;
      line3d([-2.0, y, 0], [2.0, y, 0], basis, width, height, COLORS.grid, 1);
    }
  }

  function drawAxes(basis, width, height) {
    const origin = [-1.45, 0.95, 0.02];
    line3d(origin, add(origin, [0.34, 0, 0]), basis, width, height, COLORS.axisX, 3);
    line3d(origin, add(origin, [0, 0.34, 0]), basis, width, height, COLORS.axisY, 3);
    line3d(origin, add(origin, [0, 0, 0.34]), basis, width, height, COLORS.axisZ, 3);
    const labels = [
      [add(origin, [0.38, 0, 0]), "+X forward", COLORS.axisX],
      [add(origin, [0, 0.38, 0]), "+Y left", COLORS.axisY],
      [add(origin, [0, 0, 0.38]), "+Z up", COLORS.axisZ],
    ];
    ctx.font = `${Math.max(11, width / 85)}px system-ui`;
    labels.forEach(([point, text, color]) => {
      const projected = project(point, basis, width, height);
      if (!projected) return;
      ctx.fillStyle = color;
      ctx.fillText(text, projected.x + 4, projected.y - 3);
    });
  }

  function vertex(vertices, sx, sy, sz) {
    return vertices[((sx + 1) / 2) * 4 + ((sy + 1) / 2) * 2 + ((sz + 1) / 2)];
  }

  function drawBody(pitch, roll, basis, width, height) {
    const vertices = bodyVertices(pitch, roll);
    const v = (sx, sy, sz) => vertex(vertices, sx, sy, sz);
    const faces = [
      { points: [v(-1,-1,-1), v(-1,1,-1), v(-1,1,1), v(-1,-1,1)], fill: COLORS.bodySide },
      { points: [v(1,-1,-1), v(1,-1,1), v(1,1,1), v(1,1,-1)], fill: COLORS.bodyFront },
      { points: [v(-1,-1,1), v(-1,1,1), v(1,1,1), v(1,-1,1)], fill: COLORS.bodyTop },
      { points: [v(-1,-1,-1), v(1,-1,-1), v(1,-1,1), v(-1,-1,1)], fill: COLORS.bodyFront },
      { points: [v(-1,1,-1), v(-1,1,1), v(1,1,1), v(1,1,-1)], fill: COLORS.bodySide },
    ];
    faces
      .map((face) => ({
        ...face,
        depth: face.points.reduce((sum, point) => sum + (project(point, basis, width, height)?.depth ?? 0), 0) / face.points.length,
      }))
      .sort((a, b) => b.depth - a.depth)
      .forEach((face) => polygon3d(face.points, basis, width, height, face.fill));

    const stemA = bodyPoint([0, 0, 0.12], pitch, roll);
    line3d(stemA, BODY_PIVOT, basis, width, height, COLORS.edge, 7);
  }

  function drawEstimateGhost(pitch, roll, basis, width, height) {
    // Estimate ghost is attitude-only. Same schematic origin; no estimated translation is synthesized.
    const vertices = bodyVertices(pitch, roll);
    BODY_EDGES.forEach(([a, b]) => {
      line3d(vertices[a], vertices[b], basis, width, height, COLORS.estimateGhost, 2.2);
    });

    const top = bodyPoint([0, 0, BODY_CENTER[2] + BODY_HALF[2] + 0.08], pitch, roll);
    const projected = project(top, basis, width, height);
    if (projected) {
      ctx.font = `${Math.max(10, width / 92)}px ui-monospace, monospace`;
      ctx.fillStyle = COLORS.estimateGhost;
      ctx.fillText("estimate attitude", projected.x + 5, projected.y - 4);
    }
  }

  function drawDriveWheel(basis, width, height) {
    const center = [0, 0, 0.28];
    const radius = 0.28; // schematic display radius only, not an accepted ONE V2 physical parameter
    ring3d(add(center, [0, -0.085, 0]), "y", radius, 0, 0, null, basis, width, height, COLORS.wheelEdge, 9, false);
    ring3d(add(center, [0, 0.085, 0]), "y", radius, 0, 0, null, basis, width, height, COLORS.wheel, 9, false);
    line3d([0, -0.13, 0.28], [0, 0.13, 0.28], basis, width, height, COLORS.hub, 8);
  }

  function drawReactionWheel(pitch, roll, phase, basis, width, height) {
    // Reaction phase is truth/common evidence only. The production estimate does not carry this cyclic coordinate.
    const center = [0, 0, 0.58];
    ring3d(add(center, [-0.035, 0, 0]), "x", 0.20, pitch, roll, phase, basis, width, height, COLORS.reaction, 5, true);
    ring3d(add(center, [0.035, 0, 0]), "x", 0.20, pitch, roll, phase, basis, width, height, COLORS.reaction, 5, true);
  }

  function estimateAttitude(record) {
    if (record?.dialect !== "production-semantic-v1") return null;
    const estimate = record.production?.estimate;
    if (!estimate) return null;
    if (!Number.isFinite(estimate.body_pitch_rad) || !Number.isFinite(estimate.body_roll_rad)) return null;
    return {
      pitch: estimate.body_pitch_rad,
      roll: estimate.body_roll_rad,
    };
  }

  function setSpatialText(id, value) {
    const element = document.getElementById(id);
    if (element) element.textContent = value;
  }

  function updateGhostStatus(available) {
    const status = document.getElementById("spatialGhostStatus");
    if (!status) return;
    if (!estimateGhostEnabled) {
      status.textContent = "estimate ghost: hidden";
      status.dataset.kind = "hidden";
    } else if (available) {
      status.textContent = "estimate ghost: attitude only";
      status.dataset.kind = "available";
    } else {
      status.textContent = "estimate ghost: unavailable";
      status.dataset.kind = "unavailable";
    }
  }

  function draw(record) {
    resizeCanvas();
    const width = canvas.width;
    const height = canvas.height;
    const basis = cameraBasis();
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = COLORS.background;
    ctx.fillRect(0, 0, width, height);

    const visual = record?.visual ?? {
      forward_position_m: 0,
      body_pitch_rad: 0,
      body_roll_rad: 0,
      reaction_position_rad: null,
    };
    const pitch = Number.isFinite(visual.body_pitch_rad) ? visual.body_pitch_rad : 0;
    const roll = Number.isFinite(visual.body_roll_rad) ? visual.body_roll_rad : 0;
    const forward = Number.isFinite(visual.forward_position_m) ? visual.forward_position_m : 0;
    const reactionPhase = Number.isFinite(visual.reaction_position_rad) ? visual.reaction_position_rad : null;
    const estimate = estimateAttitude(record);

    drawGround(basis, width, height, forward);
    drawAxes(basis, width, height);

    const axleProjected = project([0, 0, 0.28], basis, width, height);
    if (axleProjected) {
      ctx.beginPath();
      ctx.ellipse(axleProjected.x + 28, axleProjected.y + 30, 100, 24, -0.25, 0, Math.PI * 2);
      ctx.fillStyle = COLORS.shadow;
      ctx.fill();
    }

    drawDriveWheel(basis, width, height);
    drawBody(pitch, roll, basis, width, height);
    drawReactionWheel(pitch, roll, reactionPhase, basis, width, height);
    if (estimateGhostEnabled && estimate) {
      drawEstimateGhost(estimate.pitch, estimate.roll, basis, width, height);
    }

    setSpatialText("spatialPitch", `${(pitch * DEG).toFixed(2)}°`);
    setSpatialText("spatialRoll", `${(roll * DEG).toFixed(2)}°`);
    setSpatialText(
      "spatialPhase",
      reactionPhase === null ? "unavailable" : `${reactionPhase.toFixed(4)} rad · truth/common only`
    );
    setSpatialText("spatialForward", `${forward.toFixed(4)} m`);
    updateGhostStatus(Boolean(estimate));
    updateCameraStatus();
  }

  function installEstimateGhostUi() {
    const toolbar = document.querySelector(".spatial-toolbar");
    const boundary = document.querySelector(".spatial-boundary");
    if (!toolbar || !boundary || document.getElementById("spatialEstimateGhost")) return;

    const toggle = document.createElement("label");
    toggle.className = "spatial-ghost-toggle";
    toggle.innerHTML = '<input id="spatialEstimateGhost" type="checkbox" checked> estimate ghost';
    toolbar.appendChild(toggle);

    const legend = document.createElement("div");
    legend.className = "spatial-ghost-legend";
    legend.innerHTML = `
      <span><i class="spatial-truth-key"></i>solid = truth/common</span>
      <span><i class="spatial-estimate-key"></i>cyan wireframe = production estimate attitude only</span>
      <span>reaction phase = truth/common only</span>
      <strong id="spatialGhostStatus" data-kind="unavailable">estimate ghost: unavailable</strong>
    `;
    boundary.insertAdjacentElement("afterend", legend);

    const style = document.createElement("style");
    style.textContent = `
      .spatial-ghost-toggle { display:inline-flex; align-items:center; gap:6px; min-height:32px; padding:0 9px; border:1px solid #555b61; border-radius:4px; color:#bfc4c9; background:#24272b; font-size:11px; cursor:pointer; white-space:nowrap; }
      .spatial-ghost-toggle input { accent-color:#7fc6ff; }
      .spatial-ghost-legend { display:flex; align-items:center; gap:14px; flex-wrap:wrap; padding:8px 14px; border-bottom:1px solid #43484f; background:#202327; color:#969ca3; font-size:11px; }
      .spatial-ghost-legend span { display:inline-flex; align-items:center; gap:5px; }
      .spatial-ghost-legend i { display:inline-block; width:18px; height:0; border-top:3px solid #d2aa61; }
      .spatial-ghost-legend .spatial-estimate-key { border-top-color:#7fc6ff; border-top-style:dashed; }
      #spatialGhostStatus { margin-left:auto; color:#7fc6ff; font-family:ui-monospace, monospace; font-size:10px; font-weight:600; }
      #spatialGhostStatus[data-kind="unavailable"] { color:#858b92; }
      #spatialGhostStatus[data-kind="hidden"] { color:#b08d58; }
    `;
    document.head.appendChild(style);

    document.getElementById("spatialEstimateGhost").addEventListener("change", (event) => {
      estimateGhostEnabled = event.target.checked;
      draw(latestRecord);
    });
  }

  function installCameraUi() {
    const toolbar = document.querySelector(".spatial-toolbar");
    if (!toolbar || document.getElementById("spatialCameraReset")) return;

    const reset = document.createElement("button");
    reset.id = "spatialCameraReset";
    reset.type = "button";
    reset.dataset.cameraReset = "iso";
    reset.textContent = "RESET";
    reset.title = "Reset presentation camera to ISO preset";
    toolbar.appendChild(reset);

    const status = document.createElement("span");
    status.id = "spatialCameraState";
    status.className = "spatial-camera-status";
    status.textContent = "camera ISO";
    toolbar.appendChild(status);

    reset.addEventListener("click", () => selectCameraPreset("iso"));
  }

  function beginOrbit(event) {
    if (event.button !== 0) return;
    activeOrbitPointer = {
      id: event.pointerId,
      x: event.clientX,
      y: event.clientY,
    };
    canvas.setPointerCapture?.(event.pointerId);
    canvas.classList.add("is-orbiting");
  }

  function moveOrbit(event) {
    if (!activeOrbitPointer || event.pointerId !== activeOrbitPointer.id) return;
    const deltaX = event.clientX - activeOrbitPointer.x;
    const deltaY = event.clientY - activeOrbitPointer.y;
    activeOrbitPointer.x = event.clientX;
    activeOrbitPointer.y = event.clientY;

    const spherical = cameraSpherical();
    markCameraCustom();
    setCameraSpherical(
      spherical.radius,
      spherical.azimuth - deltaX * CAMERA_ORBIT_SENSITIVITY,
      spherical.elevation - deltaY * CAMERA_ORBIT_SENSITIVITY
    );
    updateCameraStatus();
    draw(latestRecord);
  }

  function endOrbit(event) {
    if (!activeOrbitPointer || event.pointerId !== activeOrbitPointer.id) return;
    canvas.releasePointerCapture?.(event.pointerId);
    activeOrbitPointer = null;
    canvas.classList.remove("is-orbiting");
  }

  function zoomCamera(event) {
    event.preventDefault();
    const spherical = cameraSpherical();
    const factor = Math.exp(event.deltaY * CAMERA_ZOOM_SENSITIVITY);
    markCameraCustom();
    setCameraSpherical(
      clamp(spherical.radius * factor, CAMERA_MIN_RADIUS, CAMERA_MAX_RADIUS),
      spherical.azimuth,
      spherical.elevation
    );
    updateCameraStatus();
    draw(latestRecord);
  }

  document.querySelectorAll("[data-camera-preset]").forEach((button) => {
    button.addEventListener("click", () => selectCameraPreset(button.dataset.cameraPreset));
  });

  installEstimateGhostUi();
  installCameraUi();

  canvas.addEventListener("pointerdown", beginOrbit);
  canvas.addEventListener("pointermove", moveOrbit);
  canvas.addEventListener("pointerup", endOrbit);
  canvas.addEventListener("pointercancel", endOrbit);
  canvas.addEventListener("wheel", zoomCamera, { passive: false });

  const baseRenderRecord = renderRecord;
  renderRecord = function spatialAwareRenderRecord(record) {
    baseRenderRecord(record);
    latestRecord = record;
    draw(record);
  };

  const observer = new ResizeObserver(() => draw(latestRecord));
  observer.observe(canvas);
  updatePresetButtons();
  updateCameraStatus();
  draw(null);

  window.SingleSpatialView = {
    render: draw,
    resetCamera: () => selectCameraPreset("iso"),
    cameraBounds: {
      minRadius: CAMERA_MIN_RADIUS,
      maxRadius: CAMERA_MAX_RADIUS,
      minElevationRad: CAMERA_MIN_ELEVATION_RAD,
      maxElevationRad: CAMERA_MAX_ELEVATION_RAD,
    },
    coordinateContract: {
      x: "+X forward",
      y: "+Y left",
      z: "+Z up",
      pitch: "+ about +Y (right-hand rule)",
      roll: "+ about +X (right-hand rule)",
      reactionPhase: "+ about body +X (right-hand rule)",
    },
    estimateGhostContract: {
      source: "record.production.estimate attitude only",
      origin: "same schematic axle/origin as truth; no estimated translation synthesized",
      reactionPhase: "not estimated; truth/common only",
    },
    cameraContract: {
      scope: "presentation only",
      orbit: "changes camera eye around fixed presentation target",
      zoom: "bounded camera radius only",
      evidenceMutation: "none",
    },
  };
})();

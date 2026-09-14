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
    shadow: "rgba(0,0,0,0.23)",
  };

  // Project coordinate contract — presentation must not change these signs:
  // +X forward, +Y left, +Z up.
  // positive pitch = right-hand rotation about +Y -> rotateY(+pitch).
  // positive roll  = right-hand rotation about +X -> rotateX(+roll).
  // positive reaction-wheel phase = right-hand rotation about body +X.
  const cameraPresets = {
    iso: { eye: [2.8, -3.8, 2.45], target: [0, 0, 0.70], focal: 680 },
    side: { eye: [0.2, -4.8, 1.20], target: [0, 0, 0.58], focal: 720 },
    front: { eye: [4.8, 0.2, 1.25], target: [0, 0, 0.58], focal: 720 },
  };

  let cameraPreset = "iso";
  let latestRecord = null;

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

  function cameraBasis() {
    const camera = cameraPresets[cameraPreset];
    const forward = normalize(sub(camera.target, camera.eye));
    const right = normalize(cross(forward, [0, 0, 1]));
    const up = normalize(cross(right, forward));
    return { ...camera, forward, right, up };
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
      let point;
      if (bodyAttached) {
        point = bodyPoint(add(center, local), pitch, roll);
      } else {
        point = add(center, local);
      }
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

  function drawBody(pitch, roll, basis, width, height) {
    const half = [0.18, 0.16, 0.33];
    const center = [0, 0, 0.45];
    const vertices = [];
    [-1, 1].forEach((sx) => {
      [-1, 1].forEach((sy) => {
        [-1, 1].forEach((sz) => {
          vertices.push(bodyPoint([
            center[0] + sx * half[0],
            center[1] + sy * half[1],
            center[2] + sz * half[2],
          ], pitch, roll));
        });
      });
    });
    const v = (sx, sy, sz) => vertices[((sx + 1) / 2) * 4 + ((sy + 1) / 2) * 2 + ((sz + 1) / 2)];
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
    const stemB = BODY_PIVOT;
    line3d(stemA, stemB, basis, width, height, COLORS.edge, 7);
  }

  function drawDriveWheel(basis, width, height) {
    const center = [0, 0, 0.28];
    const radius = 0.28;
    ring3d(add(center, [0, -0.085, 0]), "y", radius, 0, 0, null, basis, width, height, COLORS.wheelEdge, 9, false);
    ring3d(add(center, [0, 0.085, 0]), "y", radius, 0, 0, null, basis, width, height, COLORS.wheel, 9, false);
    line3d([0, -0.13, 0.28], [0, 0.13, 0.28], basis, width, height, COLORS.hub, 8);
  }

  function drawReactionWheel(pitch, roll, phase, basis, width, height) {
    const center = [0, 0, 0.58];
    ring3d(add(center, [-0.035, 0, 0]), "x", 0.20, pitch, roll, phase, basis, width, height, COLORS.reaction, 5, true);
    ring3d(add(center, [0.035, 0, 0]), "x", 0.20, pitch, roll, phase, basis, width, height, COLORS.reaction, 5, true);
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

    document.getElementById("spatialPitch").textContent = `${(pitch * DEG).toFixed(2)}°`;
    document.getElementById("spatialRoll").textContent = `${(roll * DEG).toFixed(2)}°`;
    document.getElementById("spatialPhase").textContent = reactionPhase === null
      ? "unavailable"
      : `${reactionPhase.toFixed(4)} rad`;
    document.getElementById("spatialForward").textContent = `${forward.toFixed(4)} m`;
  }

  document.querySelectorAll("[data-camera-preset]").forEach((button) => {
    button.addEventListener("click", () => {
      const preset = button.dataset.cameraPreset;
      if (!cameraPresets[preset]) return;
      cameraPreset = preset;
      document.querySelectorAll("[data-camera-preset]").forEach((candidate) => {
        candidate.classList.toggle("active-camera", candidate === button);
      });
      draw(latestRecord);
    });
  });

  const baseRenderRecord = renderRecord;
  renderRecord = function spatialAwareRenderRecord(record) {
    baseRenderRecord(record);
    latestRecord = record;
    draw(record);
  };

  const observer = new ResizeObserver(() => draw(latestRecord));
  observer.observe(canvas);
  document.querySelector('[data-camera-preset="iso"]')?.classList.add("active-camera");
  draw(null);

  window.SingleSpatialView = {
    render: draw,
    coordinateContract: {
      x: "+X forward",
      y: "+Y left",
      z: "+Z up",
      pitch: "+ about +Y (right-hand rule)",
      roll: "+ about +X (right-hand rule)",
      reactionPhase: "+ about body +X (right-hand rule)",
    },
  };
})();

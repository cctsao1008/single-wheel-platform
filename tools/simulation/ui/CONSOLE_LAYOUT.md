# Console Information Architecture

The browser console is a presentation surface over already-produced evidence. It supports replay and a bounded live projection without moving simulation or production semantics into the browser.

```text
simulator-neutral-v2            production-semantic-v1          persistent Rust SITL
plant/common observables        saved closed-loop evidence               |
          |                              |                                |
          |                              |                         localhost SSE
          +------------------------------+---------------+----------------+
                                                         |
                                                 presentation adapter
                                                         |
                                                 Single console UI
```

```text
+--------------------------------------------------------------------------------+
| Header: project | trace | dialect | LIVE | SIMULATION ONLY | compare/live/load |
+----------+---------------------------------------------+-----------------------+
|          |                                             |                       |
| Sidebar  | Model viewport                              | Inspector             |
|          | physically signed 3-D spatial viewport      | evidence              |
| setup    | synchronized trends                         | displayed state       |
| sim      | exact-grid comparison                       | truth/estimate        |
| spatial  |                                             | reference             |
| state    | split / side / front                        | runtime               |
| inputs   |                                             | applied input         |
| evidence |                                             |                       |
| trends   |                                             |                       |
| compare  |                                             |                       |
| contract |                                             |                       |
+----------+---------------------------------------------+-----------------------+
| Transport: restart | play/pause | speed | scrub | time | event log            |
+--------------------------------------------------------------------------------+
```

## Responsibility boundary

| Surface | Responsibility | Explicit non-responsibility |
| --- | --- | --- |
| Header | identify loaded/live evidence, dialect, and select presentation mode | infer missing backend/runtime provenance or grant authority |
| Sidebar | navigate presentation surfaces and select model view | configure controller/firmware |
| Model viewport | project carried motion evidence into side/front drawings | integrate physics |
| Spatial viewport | project the same carried evidence into schematic 3-D geometry using the documented physical sign convention | infer dimensions, integrate motion, repair signs, or create state |
| Spatial estimate ghost | show production estimate pitch/roll as a wireframe attitude-only diagnostic when explicitly carried | synthesize estimated translation or reaction-wheel phase |
| Spatial camera | orbit/zoom/preset the observer viewpoint with declared bounds | change world coordinates, evidence, trace time/index, simulation, or control state |
| Trend panel | project carried samples and explicit semantic transitions over time | smooth, resample, estimate, integrate, or create runtime state |
| Comparison panel | overlay exact-grid common traces and report primary-minus-comparison discrepancies | interpolate, average, rank, vote, or identify a backend from a filename |
| Inspector | preserve truth / estimate / reference / authority provenance | synthesize missing state |
| Replay transport | replay and scrub recorded evidence | alter simulation time or rerun a backend |
| Live transport | display-rate-limit and forward localhost Rust SITL evidence | compute plant/control state or provide physical output |
| Event log | report local viewer/playback/live semantic transitions | act as a production/runtime logger |

## Dialect boundary

The flat simulator-neutral trace carries plant state and applied torque only. It does not carry estimator state, controller reference, runtime authority, or saturation provenance. Those remain visibly unavailable in that dialect, including in the trend and spatial estimate surfaces.

The nested `closed_loop_production_path` evidence carries explicit simulator truth, production estimate/reference, runtime authority, actuation, and applied torque. Saved Webots evidence and persistent Rust SITL use distinct source/mapping provenance even though the browser projects both into the same production-semantic presentation shape.

The UI does not upgrade one dialect into the other. It uses presentation adapters so provenance remains visible.

## Persistent live boundary

The live path is deliberately asymmetric:

```text
Rust ClosedLoopSimulation
      |
      | 500 Hz synthetic integration-fixture semantics
      v
single_sitl_live
      |
      | timestamped JSON samples
      v
serve_live.py
      |
      | localhost-only SSE, <= 60 display fps by default
      v
live.js
      |
      v
bounded rolling browser history
```

The initial live fixture reuses the repository's existing 500 Hz synthetic closed-loop integration fixture. That cadence is not the canonical STM32 runtime cadence and is not accepted ONE V2 physical evidence.

Display-rate limiting may drop samples from the browser view, but it does not alter or slow the Rust plant, estimator, controller, Supervisor, authority, or virtual actuation path. Live sample indices therefore need only increase strictly in the browser; display-adjacent samples are not required to be consecutive production samples.

The browser keeps at most 3600 forwarded live samples. Stop/disconnect closes the SSE connection and the localhost bridge terminates its owned Rust child. The retained rolling window can then be replayed locally.

There is no physical actuator command path from browser to bridge or from bridge to Rust. `SIMULATION ONLY / NO PHYSICAL AUTHORITY` is a structural boundary, not a cosmetic warning.

## Spatial projection boundary

The 3-D viewport is a presentation transform over the same normalized record used by the side/front views, inspector, trends, and live mode:

```text
normalized record
      |
      +--> truth/common pitch, roll, forward position, reaction phase when carried
      |
      +--> production estimate pitch/roll when explicitly carried
      |
      v
project sign transform
+X forward / +Y left / +Z up
+pitch RHS about +Y
+roll RHS about +X
+reaction phase RHS about body +X
      |
      v
schematic geometry
      |
      +--> solid truth/common body
      +--> optional cyan estimate-attitude ghost
      |
      v
presentation camera
      |
      v
pixels
```

The geometry is deliberately schematic. Its body, drive-wheel, and reaction-wheel dimensions are visual proportions, not accepted ONE V2 physical parameters. Forward position shifts the reference grid; the renderer does not infer a drive-wheel phase from a schematic or guessed radius.

The production estimate does not carry reaction-wheel phase. The spatial ghost therefore represents only estimate pitch/roll at the same schematic axle/origin as the truth/common body. It does not synthesize estimated 3-D translation or reaction phase. If the estimate is absent, the ghost is absent.

### Camera interaction boundary

Camera state is independent presentation state:

```text
world/evidence geometry ---- unchanged ----+
                                           |
                                           v
                                  presentation camera
                              preset / orbit / zoom
                                           |
                                           v
                                         pixels
```

ISO, SIDE, and FRONT are deterministic presets. Pointer drag changes only camera azimuth/elevation around the fixed presentation target. Wheel input changes only bounded camera radius. Reset returns to the ISO preset. Camera radius and elevation are clamped to declared bounds in `model3d.js`.

Camera interaction must not mutate `latestRecord`, normalized evidence, trace index, simulation time, estimator/controller data, or physical coordinate definitions. Moving the camera is not moving the robot.

## Synchronized trend boundary

The trend panel consumes the same normalized records used by the model viewport and inspector:

```text
normalized record
   |      |       |
   |      |       +--> runtime transition markers
   |      +----------> torque series
   +-----------------> attitude truth/common + optional estimate
```

The cursor is synchronized with the current carried-sample index. Clicking a replay chart seeks to the nearest carried sample; it does not interpolate a new sample or rerun a simulator.

Runtime markers are transition-only evidence. A marker is created when an explicit production-semantic field changes, such as operating state, authority, actuation, constraint state, or fault bits. The UI does not manufacture events between samples.

## Exact-grid comparison boundary

Cross-trace comparison is restricted to the simulator-neutral common dialect in its first slice. One primary trace may be compared with up to three additional local traces only after an exact compatibility gate:

```text
same dialect
+ same sample count
+ exact same time_s values
        |
        v
comparison projection
```

No interpolation, resampling, time warping, smoothing, or normalization is allowed to make incompatible evidence look comparable.

The panel overlays pitch/roll and derives max absolute primary-minus-comparison discrepancies for selected common observables. Those metrics describe the loaded traces only. They do not rank backends, determine correctness, or create a consensus truth.

A local filename is a presentation label, not provenance. The UI must not infer that `webots.jsonl`, for example, actually came from Webots. Backend/version/experiment/parameter provenance belongs to the evidence-generation contract outside this UI.

Production-semantic traces remain single-trace in the comparison surface because nested truth/estimate/reference/authority comparisons need an explicit semantic contract of their own.

## Derived diagnostics

Truth-minus-estimate residuals are permitted only where both values exist explicitly in the loaded closed-loop evidence. They are labeled UI-derived diagnostics.

Reaction-wheel phase is a useful example of the boundary: simulator truth carries it, while the reduced production estimate omits that cyclic coordinate. The UI therefore shows truth phase but does not fabricate an estimated phase.

This distinction is deliberate: a polished console is not permission to invent telemetry, upgrade evidence, move reality with the camera, or hold a majority vote on physics.

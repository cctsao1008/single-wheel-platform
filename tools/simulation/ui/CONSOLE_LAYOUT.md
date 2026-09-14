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
|          | synchronized trends                         | evidence              |
| setup    | exact-grid comparison                       | displayed state       |
| sim      |                                             | truth/estimate        |
| state    | split / side / front                        | reference             |
| inputs   |                                             | runtime               |
| evidence |                                             | applied input         |
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
| Model viewport | project carried motion evidence into a drawing | integrate physics |
| Trend panel | project carried samples and explicit semantic transitions over time | smooth, resample, estimate, integrate, or create runtime state |
| Comparison panel | overlay exact-grid common traces and report primary-minus-comparison discrepancies | interpolate, average, rank, vote, or identify a backend from a filename |
| Inspector | preserve truth / estimate / reference / authority provenance | synthesize missing state |
| Replay transport | replay and scrub recorded evidence | alter simulation time or rerun a backend |
| Live transport | display-rate-limit and forward localhost Rust SITL evidence | compute plant/control state or provide physical output |
| Event log | report local viewer/playback/live semantic transitions | act as a production/runtime logger |

## Dialect boundary

The flat simulator-neutral trace carries plant state and applied torque only. It does not carry estimator state, controller reference, runtime authority, or saturation provenance. Those remain visibly unavailable in that dialect, including in the trend panel.

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

This distinction is deliberate: a polished console is not permission to invent telemetry, upgrade evidence, or hold a majority vote on physics.

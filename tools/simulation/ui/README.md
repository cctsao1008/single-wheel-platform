# Single Control & Evidence Console

Host-only browser UI for replaying and live-viewing Single simulation/evidence in an engineering-console layout.

The console is deliberately an **observer**, not a controller. It consumes already-produced evidence and has no route to `AuthorizedActuation`, firmware I/O, Webots control, or physical hardware.

> **The UI may explain evidence. It may not upgrade evidence.**

```text
saved common / production evidence        persistent Rust SITL
                |                                |
                |                                | semantic samples
                |                                v
                |                         localhost SSE bridge
                |                                |
                +---------------+----------------+
                                |
                        UI evidence adapter
                                |
              +-----------------+------------------+
              |                 |                  |
           motion            estimate/ref       authority
              |                 |                  |
              +-----------------+------------------+
                                |
                  Single Control & Evidence Console
                                |
                                v
                         human visualization
```

The browser keeps evidence provenance visible. Replay and live operation are presentation modes over already-defined host simulation semantics; neither creates physical authority.

## 1. Simulator-neutral common trace

This is the flat `simulator-neutral-v2` plant/evidence contract used by open-loop correlation tooling:

```text
time_s
forward_position_m
forward_velocity_m_per_s
body_pitch_rad
body_pitch_rate_rad_per_s
body_roll_rad
body_roll_rate_rad_per_s
reaction_position_rad
reaction_rate_rad_per_s
drive_torque_nm
reaction_torque_nm
```

Each record must contain exactly these fields, every value must be finite, and `time_s` must increase strictly. Runtime authority is intentionally unavailable because this trace does not carry it.

## 2. Closed-loop production-semantic evidence

The console accepts the nested #17 Webots evidence with:

```text
mode = closed_loop_production_path
```

The saved-trace viewer preserves the source layers instead of flattening them together:

```text
raw_device_observation
webots_evidence_truth      # simulator truth, evidence only
production.estimate        # production reduced-state estimate, when available
production.reference       # production control reference
production authority/runtime fields
authorized_*_torque_nm     # torque actually applied to Webots
```

For comparable coordinates, the UI may display a diagnostic residual:

```text
residual = simulator evidence truth - production estimate
```

That residual is derived only for presentation. It is not fed back into the estimator or controller and is not a physical-validity claim.

The reduced production estimate does not carry reaction-wheel phase, so the console does not invent one. It may show simulator truth reaction phase and production estimated reaction rate as separate quantities.

The saved closed-loop adapter rejects obvious provenance/authority contradictions, including:

- mixed trace dialects,
- broken raw/production sample identity,
- non-monotonic time/sample sequence,
- `revoke` with nonzero applied torque,
- `apply` without `closed_loop` authority,
- mismatch between production torque and authorized simulator-applied torque.

The repository's dedicated #17 validator remains authoritative for the full Webots closed-loop evidence contract. Browser checks are presentation guards, not a replacement validator.

## 3. Persistent live SITL

Issue #34 adds a separate live presentation path that keeps one Rust `ClosedLoopSimulation` alive until Stop/disconnect:

```text
persistent ClosedLoopSimulation
        |
        | PhysicalTimeAdvance
        v
SensorSample
        v
ObservationDelivery
        v
ProductionRuntime
        v
ActuationCommit
        |
        | production-semantic sample
        v
single_sitl_live (Rust)
        |
        | JSON stdout
        v
serve_live.py
        |
        | localhost-only SSE, display-rate limited
        v
live.js
        |
        v
browser model / inspector / rolling trends
```

Ownership stays explicit:

- **Rust** owns `SimulationWorld`, production sensor adaptation, estimation, control, Supervisor/authority semantics, and applied virtual actuation.
- **Python** owns only the localhost child-process/SSE transport lifetime and display-rate pacing.
- **JavaScript** owns visualization and a bounded rolling display history.

The live bridge binds only to `127.0.0.1`. Browser Stop/disconnect closes the SSE stream and the bridge terminates its owned Rust child. The browser never receives a physical actuator handle.

### Live cadence boundary

The initial #34 live fixture intentionally reuses the existing **synthetic closed-loop integration fixture**, whose semantic tick is 500 Hz and outer velocity loop is 100 Hz. That is a host integration fixture, not the canonical STM32 runtime cadence and not a physical ONE V2 timing claim.

The bridge forwards at most 60 display samples per second by default:

```text
Rust semantic path     500 Hz synthetic integration fixture
browser display        <= 60 fps
```

Display downsampling does not change plant integration, estimator/controller execution, Supervisor state, authority, or actuation semantics. It only decides which already-produced samples are painted in the browser.

The browser retains at most 3600 live display samples. The Rust simulation may continue indefinitely; bounded UI history is not a simulation reset or evidence rewrite.

## Console layout

```text
+--------------------------------------------------------------------------------+
| SINGLE | trace | dialect | LIVE | SIMULATION ONLY | Compare | Live | Load     |
+----------+---------------------------------------------+-----------------------+
| Sidebar  | Model viewport                              | Inspector             |
|          | synchronized trends                         | Evidence              |
| Setup    | exact-grid comparison                       | State                 |
| Sim      |                                             | Truth/estimate        |
| State    | split / side / front                        | Reference             |
| Inputs   |                                             | Runtime               |
| Evidence |                                             | Applied input         |
| Trends   |                                             |                       |
| Compare  |                                             |                       |
| Contract |                                             |                       |
+----------+---------------------------------------------+-----------------------+
| restart | play/pause | speed | timeline | time | event log                    |
+--------------------------------------------------------------------------------+
```

The event log reports local viewer events plus meaningful closed-loop state/authority transitions during forward replay or live observation. It is not a production runtime logger.

The durable information-architecture boundary is recorded in [`CONSOLE_LAYOUT.md`](CONSOLE_LAYOUT.md).

## Synchronized trends

`charts.js` adds a local projection of the same normalized evidence already used by the model and inspector. It does not introduce a charting dependency or a second data model.

The trend panel contains:

- pitch/roll attitude traces,
- production truth/estimate overlays when both are explicitly available,
- authorized drive/reaction torque,
- a runtime lane for `apply` / `revoke`,
- semantic markers only when operating state, authority, actuation, constraint, or fault state changes,
- a playback cursor synchronized with the main scrubber,
- click-to-seek to the nearest recorded sample.

The trend path is deliberately sample-faithful:

```text
recorded/display sample -> UI normalization -> SVG projection
```

There is no smoothing, filtering, interpolation, resampling, integration, or browser-side estimation. The simulator-neutral dialect therefore shows attitude/torque only and labels runtime authority as unavailable.

## Exact-grid local comparison

`compare.js` adds a bounded discrepancy-review surface for the simulator-neutral dialect. After loading a primary common trace, **Compare traces** accepts up to three additional local common traces.

The first comparison slice is deliberately strict:

```text
primary common trace ---------+
comparison common trace(s) ---+--> exact sample-count + time-grid gate
                              |
                              v
                       visual overlay
                              +--> max |primary - comparison| table
```

A comparison trace is rejected if:

- it is not `simulator-neutral-v2`,
- its sample count differs from the primary trace,
- any `time_s` sample differs from the primary time grid.

The console does not interpolate, resample, time-warp, normalize, average, rank, or vote among traces. It draws pitch/roll overlays and reports max absolute discrepancies for forward position, pitch, roll, drive torque, and reaction torque.

Local filenames are display labels only. The browser does **not** infer Analytical, SimulationWorld, Webots, or any other backend identity from a filename. Backend/version provenance remains the responsibility of the evidence artifacts that produced the trace.

> Three traces agreeing with each other are three traces agreeing with each other. They are not three votes that make physics true.

Production-semantic evidence remains single-trace in the comparison surface because comparing nested runtime/authority evidence requires a separate semantic contract.

## Run

### Replay-only

From the repository root:

```bash
python3 -m http.server 8000
```

On Windows:

```powershell
py -m http.server 8000
```

Then open:

```text
http://127.0.0.1:8000/tools/simulation/ui/
```

Use **Load trace** to open a supported `.jsonl` or JSON-array trace.

### Persistent live SITL

Use the dedicated local bridge instead of the generic HTTP server:

```bash
python3 tools/simulation/ui/serve_live.py
```

On Windows:

```powershell
py tools/simulation/ui/serve_live.py
```

Open the same URL and press **Live**. **Stop** closes the live transport; the bounded rolling window remains available for local replay.

Three committed replay examples are available:

- `sample-trace.jsonl` — illustrative simulator-neutral primary UI data,
- `sample-compare-trace.jsonl` — aligned illustrative common trace for exercising comparison,
- `sample-closed-loop-trace.jsonl` — illustrative nested production-semantic UI data.

All three are synthetic presentation fixtures only. None is a recorded simulator result, controller qualification result, or ONE V2 physical claim. The #34 persistent live fixture is also explicitly synthetic integration evidence.

## Semantic boundary

The drawings, residuals, trend plots, comparison metrics, and live display are visual projections of evidence. They are not a physics backend and are not evidence by themselves.

The console must not:

- compute a replacement plant trajectory,
- implement another controller or estimator,
- infer missing physical parameters,
- infer authority from motion alone,
- fabricate absent estimate coordinates,
- infer backend identity from filenames,
- repair signs,
- smooth away counterexamples,
- average or vote across simulators,
- communicate with firmware or physical hardware,
- change simulator/control behavior.

The only live browser transport introduced by #34 is same-origin localhost SSE from `serve_live.py`; it carries display evidence outward and carries no actuation command inward.

The static contract test in `../test_simulation_ui_static.py` pins the supported evidence dialects, synchronized trends, exact-grid comparison, bounded localhost live transport, and no-physical-authority boundary.

> If the little robot falls over, the console's job is to show the fall clearly. Live mode just lets us watch Gravity submit the review in real time. XD

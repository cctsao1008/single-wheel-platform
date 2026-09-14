# Single Control & Evidence Console

Host-only browser UI for replaying Single simulation/evidence traces in an engineering-console layout.

The console is deliberately an **observer**, not a controller. It consumes already-produced evidence and has no route to `AuthorizedActuation`, firmware I/O, Webots control, or physical hardware.

```text
simulator-neutral-v2 common trace        #17 closed-loop production evidence
           |                                         |
           +----------------+------------------------+
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

The UI supports two evidence dialects. They remain distinct because they answer different questions.

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

The console also accepts the nested #17 trace produced by `run_webots_closed_loop.sh` with:

```text
mode = closed_loop_production_path
```

The viewer preserves the source layers instead of flattening them together:

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
residual = Webots evidence truth - production estimate
```

That residual is derived only for presentation. It is not fed back into the estimator or controller and is not a physical-validity claim.

The reduced production estimate does not carry reaction-wheel phase, so the console does not invent one. It may show Webots truth reaction phase and production estimated reaction rate as separate quantities.

The closed-loop adapter rejects obvious provenance/authority contradictions, including:

- mixed trace dialects,
- broken raw/production sample identity,
- non-monotonic time/sample sequence,
- `revoke` with nonzero applied torque,
- `apply` without `closed_loop` authority,
- mismatch between production torque and authorized Webots-applied torque.

The repository's dedicated #17 validator remains authoritative for the full closed-loop evidence contract. Browser checks are presentation guards, not a replacement validator.

## Console layout

```text
+-------------------------------------------------------------------+
| SINGLE | trace status | evidence dialect | observer-only | Load   |
+----------+--------------------------------------+-----------------+
| Sidebar  | Model viewport                       | Inspector       |
|          |                                      |                 |
| Setup    | side: pitch / forward motion        | Evidence        |
| Sim      | front: roll / reaction wheel        | State           |
| State    |                                      | Truth/estimate  |
| Inputs   | synchronized trends                  | Reference       |
| Evidence |                                      | Runtime         |
| Trends   | split / side / front views          | Applied input   |
| Contract |                                      |                 |
+----------+--------------------------------------+-----------------+
| restart | play/pause | speed | timeline | time | event log       |
+-------------------------------------------------------------------+
```

The event log reports local viewer events plus meaningful closed-loop state/authority transitions during forward replay. It is not a production runtime logger.

The durable information-architecture boundary is recorded in [`CONSOLE_LAYOUT.md`](CONSOLE_LAYOUT.md).

## Synchronized trends

`charts.js` adds a local-only projection of the same normalized evidence already used by the model and inspector. It does not introduce a charting dependency or a second data model.

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
recorded sample -> UI normalization -> SVG projection
```

There is no smoothing, filtering, interpolation, resampling, integration, or browser-side estimation. The simulator-neutral dialect therefore shows attitude/torque only and labels runtime authority as unavailable.

## Run

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

Two committed examples are available:

- `sample-trace.jsonl` — illustrative simulator-neutral UI data,
- `sample-closed-loop-trace.jsonl` — illustrative nested production-semantic UI data.

Both are synthetic presentation fixtures only. Neither is a recorded simulator result, controller qualification result, or ONE V2 physical claim.

## Semantic boundary

The drawings, residuals, and trend plots are visual projections of evidence. They are not a physics backend and are not evidence by themselves.

The console must not:

- compute a replacement plant trajectory,
- implement another controller or estimator,
- infer missing physical parameters,
- infer authority from motion alone,
- fabricate absent estimate coordinates,
- repair signs,
- smooth away counterexamples,
- communicate with firmware or physical hardware,
- change simulator/control behavior.

The static contract test in `../test_simulation_ui_static.py` pins both supported evidence dialects, the synchronized trend sidecar, and the observer-only/no-network boundary.

> If the little robot falls over, the console's job is to show the fall clearly. If the estimator disagrees with reality, the console's job is to preserve the disagreement long enough for us to learn something from it.
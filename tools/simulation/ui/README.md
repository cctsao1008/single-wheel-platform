# Single Control & Evidence Console

Host-only browser UI for visualizing simulator-neutral Single traces in an engineering-console layout.

The presentation borrows the useful shape of a vehicle configurator—persistent status, navigation, a central model viewport, an inspector, and a playback/log strip—but the semantics are Single's own. The console is deliberately an **observer**, not a controller.

```text
Analytical / SimulationWorld / Webots
              |
              v
simulator-neutral common trace
              |
              v
   Single Control & Evidence Console
              |
              v
       human visualization
```

It consumes already-produced evidence and has no route to `AuthorizedActuation`, firmware I/O, Webots control, or physical hardware.

## Console layout

```text
+-------------------------------------------------------------------+
| SINGLE | trace status | contract | observer-only | Load trace     |
+----------+--------------------------------------+-----------------+
| Sidebar  | Model viewport                       | Inspector       |
|          |                                      |                 |
| Setup    | side: pitch / forward motion        | Evidence        |
| Sim      | front: roll / reaction wheel        | State           |
| State    |                                      | Applied input   |
| Inputs   | split / side / front views          | Runtime boundary|
| Evidence |                                      |                 |
+----------+--------------------------------------+-----------------+
| restart | play/pause | speed | timeline | time | event log       |
+-------------------------------------------------------------------+
```

The current viewport shows:

- side view: forward motion and body pitch,
- front view: body roll and reaction-wheel phase,
- split, side-only, and front-only display modes,
- common physical state values,
- applied drive/reaction torque,
- trace filename, sample count, and duration,
- timeline scrubbing and playback speed,
- local playback/validation events.

The inspector intentionally does **not** invent fields that the common trace does not contain. For example, runtime authority is displayed as `not carried by common trace` rather than inferred from torque or motion.

The durable information-architecture boundary is recorded in [`CONSOLE_LAYOUT.md`](CONSOLE_LAYOUT.md).

## Common observable contract

The UI uses the same common observable contract as `open_loop_correlation.py`:

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

Each loaded record must contain exactly these fields, every value must be finite, and `time_s` must increase strictly. The UI does not repair malformed traces.

## Run

From the repository root:

```bash
python3 -m http.server 8000
```

On Windows, this is also fine:

```powershell
py -m http.server 8000
```

Then open:

```text
http://127.0.0.1:8000/tools/simulation/ui/
```

Use **Load trace** to open a simulator-neutral `.jsonl` or JSON-array trace produced by the existing simulation/correlation tooling.

`sample-trace.jsonl` is **illustrative synthetic UI data only**. It exists to exercise the console and is not a recorded simulator result, controller result, or ONE V2 physical claim.

## Semantic boundary

The drawings are visual projections of trace fields. They are not a physics backend and are not evidence by themselves.

The console must not:

- compute a replacement plant trajectory,
- implement another controller,
- infer missing physical parameters,
- infer authority from torque or motion,
- repair signs,
- smooth away counterexamples,
- communicate with firmware or physical hardware,
- change simulator/control behavior.

The static contract test in `../test_simulation_ui_static.py` pins the common fields and the observer-only/no-network boundary.

> If the little robot falls over, the console's job is to show the fall clearly. It is not allowed to save face for the controller.

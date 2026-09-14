# Single Simulation Viewer

Host-only browser UI for visualizing simulator-neutral Single traces.

The viewer is deliberately an **observer**, not a controller. It consumes already-produced evidence and has no route to `AuthorizedActuation`, firmware I/O, Webots control, or physical hardware.

## What it shows

- side view: forward motion and body pitch,
- front view: body roll and reaction-wheel phase,
- common physical state values,
- applied drive/reaction torque,
- timeline scrubbing and playback speed.

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

## Run

From the repository root:

```bash
python3 -m http.server 8000
```

Then open:

```text
http://127.0.0.1:8000/tools/simulation/ui/
```

Load a simulator-neutral `.jsonl` trace produced by the existing simulation/correlation tooling.

`sample-trace.jsonl` is **illustrative synthetic UI data only**. It exists to exercise the viewer and is not a recorded simulator result, controller result, or ONE V2 physical claim.

## Semantic boundary

The drawings are visual projections of trace fields. They are not a physics backend and are not evidence by themselves. The UI must not infer missing physical parameters, repair signs, smooth away counterexamples, or change simulator/control behavior.

> If the little robot falls over, the viewer's job is to draw the fall faithfully. It is not allowed to save face for the controller.

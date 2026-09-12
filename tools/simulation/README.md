# Simulation evidence contract

`tools/simulation/` defines the host-side contract shared by independent simulation backends. It does not define a fifth production domain and it does not replace `tools/sitl/SimulationWorld`.

The purpose is to make equivalent experiments comparable across the analytical reference, Rust `SimulationWorld`, Webots, and explicitly experimental scratch backends without allowing simulator convenience values to become physical facts.

```text
experiment contract
      |
      +--> analytical model
      +--> Rust SimulationWorld
      +--> Webots
      `--> PyBullet scratch work (when useful)
              |
              v
      backend evidence
              |
              v
      correlation / counterexamples
```

## Experiment contract

Schema `2` / `simulator-neutral-v2` requires:

- an explicit backend contract and allowed backend set;
- a parameter-set identifier, source, and provenance class;
- the canonical body frame and mechanical positive-direction definitions;
- duration and integration/sample step;
- the canonical eight-quantity physical initial condition;
- a piecewise input profile containing drive torque, reaction-wheel torque, and optional body-frame external force;
- named common observables with explicit physical units.

The common initial-state contract is:

```text
forward_position_m
forward_velocity_m_per_s
body_pitch_rad
body_pitch_rate_rad_per_s
body_roll_rad
body_roll_rate_rad_per_s
reaction_position_rad
reaction_rate_rad_per_s
```

This is intentionally not a list of simulator joints. In the canonical reduced model, drive-wheel relative angle is derived under local pure rolling:

```text
delta_d = s / r_drive - theta
```

A rigid-body backend may record drive-joint angle/rate as backend-native diagnostics, but those values are not interchangeable with `s` / `s_dot` and are not part of the common correlation state.

Validate an experiment with:

```bash
python3 tools/simulation/validate_experiment.py \
  tools/simulation/experiments/synthetic-small-angle.json
```

The committed example deliberately reuses values from the existing synthetic correlation fixture. Its provenance is `synthetic`; it is not ONE V2 physical evidence. The fixture therefore starts at `s = 0`, `pitch = +0.025 rad`, and `roll = -0.02 rad`, matching that existing synthetic model experiment rather than inventing a backend-specific joint perturbation.

## Provenance classes

`synthetic` means the values exist only to exercise architecture, dynamics, correlation, or controller behavior. They must never be described as measured ONE V2 properties.

`accepted_physical` means the parameter source is the canonical `parameters/reference-assembly.json`. A later backend-specific materializer may derive simulator input from that registry, but it must fail closed when required accepted values remain unknown. Simulator defaults are never a substitute for missing physical evidence.

## Coordinate contract

All backends adapt into the project semantics before comparison:

```text
body frame: +X forward, +Y left, +Z up
body roll +: right-hand rotation about body +X
body pitch +: right-hand rotation about body +Y
drive wheel +: relative rotation producing +X forward rolling with body pitch held fixed
reaction wheel +: relative rotation about body +X by right-hand rule
```

A backend-specific axis convention is an adapter concern. Hidden sign flips used only to make a controller appear stable are not allowed.

## Evidence manifest

Backend runners added later should emit a manifest alongside traces. The manifest should record at least:

```text
schema
backend name and version
repository git commit
experiment path and SHA-256
parameter-set id, provenance, source, and SHA-256/materialization hash
deterministic seed when applicable
runtime/solver settings that affect results
trace SHA-256
summary SHA-256
```

Generated traces and summaries are evidence artifacts. They are not parameter registries and must not silently feed production estimation or control.

## Simulation-truth boundary

A closed-loop simulator may expose its truth to evidence and correlation. Production estimation/control must receive device-like observations through the existing production semantic path. The simulator's ability to apply torque does not grant physical actuation authority.

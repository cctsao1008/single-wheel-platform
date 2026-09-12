# Simulation evidence contract

`tools/simulation/` defines the host-side contract shared by independent simulation backends. It does not define a fifth production domain and it does not replace `tools/sitl/SimulationWorld`.

The purpose is to make equivalent experiments comparable across the analytical reference, Rust `SimulationWorld`, Webots, and explicitly experimental scratch backends without allowing simulator convenience values to become physical facts.

```text
experiment contract
      |
      +--> analytical model
      +--> Rust SimulationWorld
      +--> Webots
      `--> scratch/reviewer backends when justified
              |
              v
      common physical observables
              |
              v
      discrepancy evidence
```

## Experiment contract

Schema `2` / `simulator-neutral-v2` requires an explicit backend set, parameter source and provenance, project coordinate/sign definitions, timing, physical initial state, piecewise torque/disturbance input, and named observables with units.

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

A rigid-body backend may retain drive-joint angle/rate as diagnostics, but those values are not interchangeable with `s` / `s_dot` and are not part of the common correlation state.

Validate the committed experiments with:

```bash
for experiment in tools/simulation/experiments/*.json; do
  python3 tools/simulation/validate_experiment.py "$experiment"
done
```

## Open-loop correlation suite

Issue #16 defines five synthetic experiments before any closed-loop controller comparison:

```text
synthetic-free-response.json
synthetic-drive-torque-pulse.json
synthetic-reaction-torque-pulse.json
synthetic-small-angle.json          # combined small disturbance
synthetic-zero-input-equilibrium.json
```

`open_loop_correlation.py` translates each neutral experiment into the reduced-model fixture format, projects analytical/Rust/Webots outputs into the common observable set, aligns time samples, checks applied inputs, computes max/RMS discrepancies, and performs experiment-specific causal/sign checks.

The reduced analytical lane uses float64 exact ZOH. `SimulationWorld` uses the independent Rust f32/RK4 implementation. Their agreement is held to a strict numerical threshold. Webots is a rigid-body/contact model and is not required to numerically equal the reduced plant; it must preserve input/sign causality or produce an explicitly classified physical-model difference.

A backend disagreement is not resolved by majority vote or by retuning one backend until it matches another.

## Synthetic cross-backend parameter source

The older `tools/model/fixtures/synthetic_correlation.json` remains the compact synthetic fixture for reduced-model implementation correlation. Its convenient inertias were never intended to describe a realizable rigid-body geometry.

Cross-backend analytical/Rust/Webots experiments instead use:

```text
tools/simulation/fixtures/synthetic-rigidbody-correlation.json
```

That fixture derives the reduced body/wheel inertias from the same synthetic box/cylinder geometry committed in the bootstrap Webots world. Unit tests pin the geometry-to-inertia formulas and require every #16 experiment to use this single source.

Both fixtures are synthetic. Neither contains ONE V2 physical facts.

## Known rigid-body difference

The bootstrap Webots drive wheel has finite width, while the reduced roll model behaves like a knife-edge rolling support. A small initial roll can therefore produce different roll evolution in the two models. The correlation summary preserves this as an `explainable_difference`; it is not hidden with a sign flip or gain change.

Independent drive-torque and reaction-torque experiments remain the polarity/causal checks. A sign failure there is a defect, not an explainable contact difference.

## Provenance classes

`synthetic` means values exist only to exercise architecture, dynamics, correlation, or controller behavior. They must never be described as measured ONE V2 properties.

`accepted_physical` means the parameter source is the canonical `parameters/reference-assembly.json`. A future backend-specific materializer may derive simulator input from that registry, but it must fail closed when required accepted values remain unknown. Simulator defaults are never a substitute for missing physical evidence.

## Coordinate contract

All backends adapt into project semantics before comparison:

```text
body frame: +X forward, +Y left, +Z up
body roll +: right-hand rotation about body +X
body pitch +: right-hand rotation about body +Y
drive wheel +: relative rotation producing +X forward rolling with body pitch held fixed
reaction wheel +: relative rotation about body +X by right-hand rule
```

A backend-specific axis convention is an adapter concern. Hidden sign flips used only to make a controller appear stable are not allowed.

## Evidence

A backend evidence bundle should identify backend/version, repository commit, experiment and SHA-256, parameter source and SHA-256, solver settings, raw/projected traces, correlation summary, and deterministic seed when applicable.

Generated traces and summaries are evidence artifacts. They are not parameter registries and must not silently feed production estimation or control.

## Simulation-truth boundary

A closed-loop simulator may expose truth to evidence/correlation. Production estimation/control must receive device-like observations through the existing production semantic path. The simulator's ability to apply torque does not grant physical actuation authority.

# Webots backend

This directory is the host-only rigid-body simulation lane for the Single platform. Webots is an independent physical counterexample generator; it does not replace the analytical reference or Rust `SimulationWorld`, and it is not a production Firmware dependency.

Two synthetic worlds serve different evidence contracts:

```text
worlds/single_wheel_synthetic.wbt
  open-loop / cross-backend correlation world (#16)

worlds/single_wheel_closed_loop_equivalent.wbt
  production-semantic closed-loop world (#17)
```

Both use the project body frame and explicitly set gravity to `9.80665 m/s^2`:

```text
+X forward
+Y left
+Z up
```

Mechanical sign mapping:

```text
drive wheel +    = relative rotation about +Y, producing +X forward rolling
reaction wheel + = relative rotation about body +X by right-hand rule
```

All geometry, mass, contact, friction, and torque-limit values in these worlds are **synthetic simulation values**. They are not ONE V2 facts and must not be copied into `parameters/reference-assembly.json`.

## Open-loop simulator-neutral experiments

The controller accepts a schema-2 experiment through `SWP_EXPERIMENT` and writes common-observable JSONL through `SWP_WEBOTS_TRACE`:

```bash
SWP_EXPERIMENT=tools/simulation/experiments/synthetic-drive-torque-pulse.json \
SWP_WEBOTS_TRACE=/tmp/single-webots.jsonl \
xvfb-run --auto-servernum webots \
  --stdout --stderr --batch --mode=fast --no-rendering \
  tools/simulation/webots/worlds/single_wheel_synthetic.wbt
```

`setTorque()` is used intentionally for ideal direct joint torque. The controller applies the experiment's piecewise drive/reaction torque profile and emits:

```text
time
forward position / velocity
pitch / pitch rate
roll / roll rate
reaction-wheel relative angle / rate
applied drive / reaction torque
```

Forward translation comes from the body world pose/velocity. The drive encoder may remain in the Webots world as a backend diagnostic device, but it is deliberately not read as the common forward coordinate.

A legacy zero-torque smoke mode remains when `SWP_EXPERIMENT` is absent; it is not the #16 correlation path.

## Open-loop parameter consistency

The #16 Webots experiments use `tools/simulation/fixtures/synthetic-rigidbody-correlation.json`. Its reduced-model inertias are analytically derived from the same committed synthetic box/cylinder geometry used by the open-loop world. This prevents analytical/Rust/Webots comparisons from accidentally using different inertial parameter sets.

The source is synthetic and exists only for correlation development.

## Closed-loop production semantic path

The #17 lane does not implement a second estimator or controller in Python. Webots emits device-like evidence and the Rust bridge executes the production semantic composition:

```text
Webots rigid-body truth
  -> Accelerometer / Gyro / PositionSensor
  -> MPU6050-like raw counts + encoder counts
  -> RawObservation
  -> production scaling / calibration / frame transform
  -> EstimatorInput
  -> production estimator
  -> production Control + velocity outer loop
  -> actuator model
  -> RuntimeSupervisor / RuntimeAuthority
  -> AuthorizedActuation or explicit revocation
  -> Webots direct torque adapter
```

Simulator pose/state truth is written only into the evidence side of the trace. It is never sent to the Rust bridge. A raw observation is consumed at most once; replay is rejected.

The raw bridge contract pins the body-axis mapping. A deliberately broken pitch-rate sign mapping is a regression failure rather than something that can be compensated with a controller gain or simulator-only sign flip.

### Aggregate-equivalent physical realization

A useful failure during #17 exposed an important distinction: the original reduced synthetic parameter tuple is algebraically valid for the reduced model but is not, component-by-component, a convenient rigid-body geometry for Webots. Feeding a high-fidelity simulator a different hidden inertia set caused the observer/controller to react to a robot that was physically not the robot described by its model.

The closed-loop world therefore uses:

```text
tools/simulation/fixtures/closed-loop-aggregate-equivalent.json
```

and is checked by:

```text
tools/simulation/validate_closed_loop_realization.py
```

The Webots component masses and geometries are physically realizable and reproduce the reduced upright quantities that actually govern the local model:

```text
gravitational first moment
vertical second moment
equivalent translation mass
pitch inertia
roll-body inertia
drive-wheel radius
reaction-wheel spin inertia
```

within the declared tolerance. This is an **aggregate-equivalent synthetic realization**, not a claim that its component dimensions describe ONE V2.

The closed-loop accelerometer is placed at the drive-wheel axle / reduced-model origin, matching the measurement model used by the production bridge. That placement is part of the evidence contract, not a hidden correction.

`tools/simulation/run_webots_closed_loop.sh` first validates this realization, then runs the pinned Webots image through the production semantic bridge and validates the resulting authority/actuation trace.

## Known roll/contact difference

The open-loop Webots drive wheel has finite width. The reduced roll model uses a knife-edge-like rolling support assumption. A small roll perturbation can therefore remain inside a lateral contact support region in Webots while the reduced model predicts immediate unstable roll evolution.

This difference is kept visible as `explainable_difference` evidence. It is not corrected with a hidden sign flip or controller/gain adjustment. Standalone drive- and reaction-torque experiments are used to verify actuator polarity and gross causal direction independently of that combined contact effect.

## Reproducible CI lane

`.github/workflows/webots.yml` uses the Cyberbotics R2025a container pinned by immutable digest:

```text
ghcr.io/cyberbotics/webots-docker/webots
@sha256:f31b128a3e4c06e54b26ce3d963a0e6b1c9634907978ae4397db9b4cde2d9f0c
```

The workflow runs both evidence lanes:

1. the five #16 experiments across analytical, Rust `SimulationWorld`, and Webots;
2. the #17 short closed-loop production-semantic scenario.

For open-loop experiments it validates contracts, materializes the reduced fixture, records raw/projected traces, verifies Webots emitted no runtime `ERROR:`, calculates discrepancy/causal metrics, and writes machine-readable summaries.

For the closed-loop path it requires startup revocation before authority, nonzero authorized actuation, zero runtime faults, bounded attitude in the short acceptance window, and exact equality between the Rust-authorized torque and the torque sent to Webots.

Evidence artifacts are uploaded even when a job fails, so a failed counterexample remains inspectable rather than disappearing with the job.

The pinned simulator image and repository commit together define the executable environment. Changing the simulator digest is an explicit evidence-boundary change, not an invisible upgrade.

## Authority boundary

Webots never gains physical motor authority. The simulation torque adapter consumes only the host-side `AuthorizedActuation` result returned by the production semantic bridge. No path is added from Webots to the STM32 electrical output owner.

```text
Webots truth -> device-like evidence -> production semantics -> host-only Webots torque
```

is valid; these are not:

```text
Webots truth -> estimator state
Webots truth -> controller state
Webots -> physical PWM/DIR
```

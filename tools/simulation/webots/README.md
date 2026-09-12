# Webots backend

This directory is the host-only rigid-body simulation lane for the Single platform. Webots is an independent physical counterexample generator; it does not replace the analytical reference or Rust `SimulationWorld`, and it is not a production Firmware dependency.

The committed world is intentionally synthetic:

```text
worlds/single_wheel_synthetic.wbt
  body rigid body
  drive wheel: hinge axis +Y, finite-width ground contact
  reaction wheel: hinge axis +X
  body InertialUnit + Gyro
  drive/reaction PositionSensor
  drive/reaction RotationalMotor in direct torque control
```

The world uses the project body frame and explicitly sets gravity to `9.80665 m/s^2`:

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

All geometry, mass, contact, friction, and torque-limit values in this bootstrap world are **synthetic simulation values**. They are not ONE V2 facts and must not be copied into `parameters/reference-assembly.json`.

## Simulator-neutral experiment run

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

## Cross-backend parameter consistency

The #16 Webots experiments use `tools/simulation/fixtures/synthetic-rigidbody-correlation.json`. Its reduced-model inertias are analytically derived from the same committed synthetic box/cylinder geometry used by this world. This prevents analytical/Rust/Webots comparisons from accidentally using different inertial parameter sets.

The source is synthetic and exists only for correlation development.

## Known roll/contact difference

The Webots drive wheel has finite width. The reduced roll model uses a knife-edge-like rolling support assumption. A small roll perturbation can therefore remain inside a lateral contact support region in Webots while the reduced model predicts immediate unstable roll evolution.

This difference is kept visible as `explainable_difference` evidence. It is not corrected with a hidden sign flip or controller/gain adjustment. Standalone drive- and reaction-torque experiments are used to verify actuator polarity and gross causal direction independently of that combined contact effect.

## Reproducible CI lane

`.github/workflows/webots.yml` uses the Cyberbotics R2025a container pinned by immutable digest:

```text
ghcr.io/cyberbotics/webots-docker/webots
@sha256:f31b128a3e4c06e54b26ce3d963a0e6b1c9634907978ae4397db9b4cde2d9f0c
```

The workflow runs the five #16 experiments across analytical, Rust `SimulationWorld`, and Webots lanes. For each experiment it validates the contract, materializes the reduced fixture, records raw/projected traces, verifies Webots emitted no runtime `ERROR:`, calculates discrepancy/causal metrics, and writes a machine-readable summary.

Evidence artifacts are uploaded even when the correlation job fails, so a failed counterexample remains inspectable rather than disappearing with the job.

The pinned simulator image and repository commit together define the executable environment. Changing the simulator digest is an explicit evidence-boundary change, not an invisible upgrade.

## Boundary

The bootstrap controller does **not** implement state estimation, LQR/LQI, the velocity loop, or runtime authority. Closing Webots through the production semantic path is a separate issue. Until then:

```text
Webots truth -> common trace evidence only
```

not:

```text
Webots truth -> production estimator/control
```

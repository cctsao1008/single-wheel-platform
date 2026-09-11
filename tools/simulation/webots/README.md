# Webots backend

This directory is the host-only high-fidelity simulation lane for the Single platform. Webots is an independent physical counterexample generator; it does not replace the analytical reference or Rust `SimulationWorld`, and it is not a production Firmware dependency.

The first committed world is intentionally synthetic:

```text
worlds/single_wheel_synthetic.wbt
  body rigid body
  drive wheel: hinge axis +Y, ground contact
  reaction wheel: hinge axis +X
  body InertialUnit + Gyro
  drive/reaction PositionSensor
  drive/reaction RotationalMotor in direct torque control
```

The world uses Webots' conventional robot axes as the project body frame:

```text
+X forward
+Y left
+Z up
```

This gives the physical sign mapping:

```text
drive wheel +    = relative rotation about +Y, producing +X forward rolling
reaction wheel + = relative rotation about body +X by right-hand rule
```

The geometry, masses, friction, and torque limits in this bootstrap world are **synthetic simulation values**. They are not promoted ONE V2 facts and must not be copied into `parameters/reference-assembly.json`.

## Minimal trace run

With Webots R2025a installed on Linux, run headless under Xvfb:

```bash
SWP_WEBOTS_TRACE=/tmp/single-webots.jsonl \
xvfb-run --auto-servernum webots \
  --stdout --stderr --batch --mode=fast --no-rendering \
  tools/simulation/webots/worlds/single_wheel_synthetic.wbt
```

The `swp_trace` controller defaults to 0.25 s and zero torque. Optional environment variables are:

```text
SWP_WEBOTS_DURATION_S
SWP_WEBOTS_DRIVE_TORQUE_NM
SWP_WEBOTS_REACTION_TORQUE_NM
SWP_WEBOTS_TRACE
```

`setTorque()` is used intentionally: Webots documents it as direct torque control that disables the internal position PID until position control is selected again.

The emitted JSONL records body roll/pitch and rates, drive/reaction positions and finite-difference rates, plus the applied ideal joint torques. This trace is simulator evidence only.

## Reproducible CI lane

`.github/workflows/webots.yml` runs the smoke model in the Cyberbotics R2025a container pinned by immutable image digest:

```text
ghcr.io/cyberbotics/webots-docker/webots
@sha256:f31b128a3e4c06e54b26ce3d963a0e6b1c9634907978ae4397db9b4cde2d9f0c
```

The high-fidelity job runs only when its workflow, Webots model/controller, or Webots trace validation changes, and it is also available through `workflow_dispatch`. It deliberately does not run on every ordinary firmware commit. The normal Rust workflow still validates the lightweight simulator-neutral contract on each `main` push.

The smoke workflow:

1. runs Webots R2025a headlessly with software rendering;
2. fails if Webots emits a load/runtime `ERROR:` line;
3. validates the generated JSONL trace structure and timing;
4. uploads the trace and Webots log as CI evidence.

The pinned simulator image and repository commit together define the executable environment for this lane. Changing the simulator digest is an explicit evidence-boundary change, not an invisible upgrade.

## Boundary

The bootstrap controller does **not** implement state estimation, LQR/LQI, the velocity loop, or runtime authority. Closing Webots through the production semantic path is a separate integration step. Until then:

```text
Webots truth/sensors -> trace evidence only
```

not:

```text
Webots truth -> production estimator/control
```

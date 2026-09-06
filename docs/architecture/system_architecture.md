# System Architecture

The platform is organized around four architectural domains:

```text
                  CONTROL
                     ▲
                     │
                 SUPERVISOR
                  ▲      ▲
                  │      │
                PLANT    │
                  ▲      │
                  └──┬───┘
                     │
                 FIRMWARE
              STM32 / RP2350
```

The arrows express architectural dependency and service relationships. They are not a single runtime data pipeline.

## Plant

Plant is the portable physical truth of the robot.

It owns:

```text
physical state and units
plant dynamics
measurement physics
raw observation semantics
actuator physical model
```

Canonical equations are:

```text
x_dot = f(x, u, p)
y     = h(x, u, p)
```

The reduced upright state is:

```text
x = [
    forward displacement,
    forward velocity,
    pitch,
    pitch rate,
    roll,
    roll rate,
    reaction-wheel relative rate,
]^T
```

The physical plant input is:

```text
u = [drive-wheel torque, reaction-wheel torque]^T
```

Plant does not know STM32, RP2350, RTIC, GPIO, PWM, BLE, or electrical polarity.

Current code lives in:

```text
plant/robot-domain
plant/plant-model
plant/measurement-model
plant/plant-observation
plant/actuator-model
```

## Control

Control defines desired behavior from estimated state and reference.

```text
EstimatedState + Reference
            |
            v
       control law
            |
            v
    GeneralizedDemand
```

The current state-feedback form is:

```text
u = u_ff - K (x_hat - x_ref)
```

LQI may add explicit integral coordinates. The controller does not decide whether physical output is currently permitted.

Current code lives in:

```text
control/state-feedback
```

## Supervisor

Supervisor owns runtime belief, operating policy, and physical-output authority.

It consumes Plant semantics, invokes Control, and decides what may become physically effective.

```text
EstimatorMeasurement
        |
        v
StateEstimator
        |
        v
EstimatedState
        |
        +------> Control
        |           |
        |       demand
        |           |
        v           v
operating state / timing / limits
              |
              v
       RuntimeAuthority
              |
              +-- denied --> no physical-output token
              |
              +-- allowed -> AuthorizedActuation
```

Supervisor owns:

```text
state estimation
operating state
sensor timing health
state validity
reaction-wheel headroom
actuator constraint interpretation
LQI integrator hold policy
previous applied input
one-step control orchestration
```

The estimator boundary is `StateEstimator`; linear observer and EKF are implementations of that contract.

Current code lives in:

```text
supervisor/state-estimator
supervisor/ekf
supervisor/runtime-state
supervisor/control-runtime
```

## Firmware

Firmware is the physical execution and target-composition domain.

It owns:

```text
device drivers and transfer functions
sensor calibration / frame projection adapters
board and installed-assembly binding
interrupts and scheduling
concrete peripheral ownership
recording / telemetry transport integration
electrical output
```

Firmware is allowed to depend on Plant, Supervisor, and Control because it is the composition boundary that creates an executable robot.

Current code includes:

```text
firmware/mpu6050
firmware/software-i2c
firmware/sensor-calibration
firmware/frame-transform
firmware/estimator-input
firmware/board-one-v2
firmware/reference-assembly
firmware/one-v2-electrical-output
firmware/stm32f103-electrical-output
firmware/stm32f103
firmware/live-shadow-stm32f103
firmware/control-footprint-stm32f103
```

A future RP2350 target belongs here and should reuse the same portable Plant / Supervisor / Control contracts.

## Infrastructure

`infrastructure/` is horizontal support, not a fifth control layer.

```text
infrastructure/dsp-kernel
infrastructure/observation-record
infrastructure/control-profile-record
```

Numerical kernels, record formats, and profiling transport do not own robot behavior.

## Dependency rules

The architecture is governed by these rules:

```text
Plant
    no target-MCU dependency
    no control-policy dependency
    no physical-output mutation

Control
    may depend on Plant semantics
    no firmware dependency
    no authority ownership

Supervisor
    may depend on Plant + Control
    no STM32/RP2350/HAL dependency
    is the only semantic source of AuthorizedActuation

Firmware
    may compose Plant + Supervisor + Control
    owns concrete devices, timers, GPIO, DMA and electrical output
```

Horizontal infrastructure may be used where appropriate but must not acquire system policy.

## Runtime causality

The physical runtime is a closed loop:

```text
        ┌─────────────────────────────────────┐
        │                                     │
        ▼                                     │
   Physical Plant                             │
        │                                     │
   observation                                │
        ▼                                     │
   Firmware input adapters                    │
        │                                     │
        ▼                                     │
   Supervisor / estimator                     │
        │                                     │
   EstimatedState                             │
        ▼                                     │
      Control                                 │
        │                                     │
   GeneralizedDemand                          │
        ▼                                     │
   Supervisor / authority                     │
        │                                     │
   AuthorizedActuation                        │
        ▼                                     │
   Firmware electrical output                 │
        │                                     │
        └──────── physical actuation ─────────┘
```

One sensor opportunity creates at most one control opportunity; missed periods are not replayed as catch-up control.

## Typed semantic boundaries

The runtime does not collapse physical meaning into anonymous numeric commands:

```text
RawObservation
    != EstimatorMeasurement
    != EstimatedState
    != GeneralizedDemand
    != BoundedActuatorCommand
    != AuthorizedActuation
    != ElectricalActuation
```

Each transition has one owner. In particular, electrical-output code accepts `AuthorizedActuation`, not an arbitrary normalized command.

## Host engineering

Model derivation, parameter identification, exact discretization, observer/controller synthesis, replay, and correlation remain host-side operations under `tools/`.

```text
measurement / recorded evidence
            |
            v
identification / correlation
            |
            v
model / estimator / control synthesis
            |
            v
canonical parameters
            |
            v
MCU runtime
```

Reference-backed nominal parameters may bootstrap the executable system. Local measured or identified values supersede lower-confidence assumptions without changing the four-domain architecture.

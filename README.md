# Self-Balancing Single-Wheel Platform

A Rust `no_std` control platform for a self-balancing single-wheel robot.

The repository separates physical evidence, sensing, estimation, control, actuator modeling, runtime authority, and board electrical output. The STM32 executes deterministic fixed-size control math; model derivation, system identification, discretization, and Riccati synthesis remain host-side engineering operations.

## Core Architecture

```text
Physical Robot
      |
      v
RawObservation
      |
      v
ScaledObservation
      |
      v
CalibratedObservation
      |
      v
BodyObservation
      |
      v
State Estimator
      |
      v
EstimatedState
      |
      v
LQR / LQI
      |
      v
GeneralizedDemand [N m]
      |
      v
Actuator Model / Inverse Model
      |
      v
Bounded Actuator Command
      |
      v
RuntimeAuthority
      |
      +-- denied ------> no physical output
      |
      +-- authorized --> AuthorizedActuation
                              |
                              v
                    Electrical Output
                              |
                              v
                     Physical Actuators
```

The model contracts are:

```text
plant-model
    x_dot = f(x, u, p)

measurement-model
    y = h(x, u, p)

state-estimator
    x_pred = A_d x_hat + B_d u
    x_hat  = x_pred + L (y - y0 - C x_pred - D u)

state-feedback
    u = u_ff - K (x_hat - x_ref)
```

`control-runtime` composes estimator, state feedback, actuator inversion, and runtime authority into one executable control opportunity. It retains the previously authorized physical effort as the next observer input and never performs catch-up control for missed periods.

## Current Runtime

Target MCU: `STM32F103C8T6`.

```text
MPU6050 DATA_RDY / PC13 EXTI13   500 Hz acquisition boundary
TIM1                              1 kHz independent timing-health supervisor
TIM2                              Encoder_1 QEI
TIM4                              Encoder_2 QEI
ADC1 / PA5                        battery observation
USART2 TX / DMA1 CH7              RecordedObservation transport
```

The primary sensor timing states are:

```text
Startup
Healthy
Late
Timeout
```

Only `Healthy` timing can participate in closed-loop authority. A missing DATA_RDY event is detected by the independent TIM1 supervisor rather than by the sensor interrupt path itself.

The current STM32 firmware is still **observation-only**. The estimator, LQR/LQI, actuator model, control-runtime composition, and authority contracts are implemented and tested, but numeric reference-platform gains and actuator parameters are not instantiated until the required physical evidence exists.

## Control Runtime

One control opportunity follows this causality:

```text
measurement[k]
      |
      v
observer using applied u[k-1]
      |
      v
estimated state[k]
      |
      v
LQR / LQI
      |
      v
requested torque[k]
      |
      v
inverse actuator model
      |
      v
bounded command[k]
      |
      v
RuntimeAuthority
      |
      v
applied u[k]
      |
      +----> observer input for k+1
```

The runtime preserves these distinct meanings:

```text
requested torque
    != bounded actuator command
    != authorized command
    != electrical output
```

LQI integral state is advanced only when the current request and the candidate updated request both remain fully authorized and unconstrained. Saturation, reaction-wheel limiting, invalid estimation, invalid timing, or an ineligible operating state therefore cannot silently accumulate integral wind-up.

## Physical Model

The reduced upright / straight-line balance state is:

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

with physical plant input:

```text
u = [drive-wheel torque, reaction-wheel torque]^T
```

The nonlinear plant contract is:

```text
M(q, p) q_ddot
+ c(q, q_dot, p)
+ g(q, p)
+ d(q_dot, p)
=
B(p) u
```

Roll and pitch are not assumed globally decoupled. The stationary-upright linearization exposes a local pitch/translation block and a local roll/reaction-wheel-momentum block as a derived property of the model.

## Measurement Model

The estimator consumes physical observations, not synthetic tilt angles.

```text
y = h(x, u, p)
```

Around stationary upright:

```text
y = y0 + C x + D u
```

The accelerometer is modeled as a specific-force sensor including gravity, translational acceleration, angular acceleration at the IMU lever arm, and actuator feedthrough. Encoder and gyro channels remain independent measurement evidence.

## Runtime Authority

`RuntimeAuthority` is the only semantic boundary that can create `AuthorizedActuation`.

Authority considers:

```text
operating state
sensor timing health
estimated-state validity
reaction-wheel speed / headroom
actuator saturation
```

Hard denial removes physical-output authority. Constrained operation remains explicit and holds LQI integration.

## Numerical Execution

Cortex-M control math uses CMSIS-DSP through `swp-dsp-kernel`.

```text
measurement-model
state-estimator
state-feedback
      |
      v
swp-dsp-kernel
      |
      v
CMSIS-DSP / Cortex-M3
```

There is no parallel scalar production backend on STM32. Non-ARM builds provide only a host semantic implementation for deterministic tests.

Host synthesis in `tools/control/` performs exact zero-order-hold discretization and observer/LQR/LQI synthesis. The MCU does not solve matrix exponentials or Riccati equations at runtime.

## Physical Evidence

Reference-platform quantities live under `parameters/` and remain unknown until supported by evidence.

```text
measured
identified
datasheet
derived
unknown
```

Unknown mass, inertia, geometry, encoder scale, IMU placement, actuator gain, friction, or delay is not replaced by a convenient nominal value.

## Repository Structure

```text
crates/
  robot-domain/          robot-semantic states, units, actuator roles
  plant-model/           physical plant dynamics and reduced balance model
  measurement-model/     sensor equations and observability model
  dsp-kernel/            CMSIS-DSP Cortex-M numerical boundary
  state-estimator/       fixed-rate discrete predictor/corrector
  state-feedback/        LQR/LQI execution
  control-runtime/       estimator -> control -> actuator -> authority composition
  actuator-model/        torque / command inverse model and saturation
  runtime-state/         operating state, timing health, physical-output authority
  plant-observation/     raw observation, timing, quality, acquisition status
  sensor-calibration/    physical sensor calibration
  frame-transform/       sensor-frame -> robot-body mapping
  reference-assembly/    installed hardware and board-to-role mapping
  observation-record/    binary recording / replay contract
  mpu6050/               MPU6050 device driver
  software-i2c/          embedded-hal software I2C
  board-one-v2/          PCB wiring and peripheral mapping

firmware/
  stm32f103/             RTIC target runtime

parameters/
  reference-assembly.json

tools/
  model/                 symbolic derivation and structural analysis
  control/               exact ZOH, observer, LQR/LQI synthesis
  actuator/              actuator identification
  recording/             record decode / replay support
  wireless/              BLE capture and live observation

docs/
  architecture/
  hardware/
  commissioning/
```

## Reference Assembly

```text
BLDC_1 / Encoder_1 -> ReactionWheel
BLDC_2 / Encoder_2 -> DriveWheel
BLDC_3             -> unused
```

Robot body frame:

```text
+X = forward
+Y = left
+Z = up
```

PCB channel identity, installed assembly role, and robot-control semantics remain separate concepts.

## Build

```bash
cargo fw
```

CI checks formatting, Cortex-M workspace compilation, Clippy, CMSIS-DSP integration, model/estimator/controller/control-runtime/authority tests, host tools, control synthesis, and release firmware linking.

## Architecture Documents

- [`docs/architecture/system_architecture.md`](docs/architecture/system_architecture.md)
- [`docs/architecture/control_runtime.md`](docs/architecture/control_runtime.md)
- [`docs/architecture/plant_model.md`](docs/architecture/plant_model.md)
- [`docs/architecture/measurement_model.md`](docs/architecture/measurement_model.md)
- [`docs/architecture/state_estimation_and_control.md`](docs/architecture/state_estimation_and_control.md)
- [`docs/architecture/runtime_authority.md`](docs/architecture/runtime_authority.md)
- [`docs/architecture/observation_time_health_replay.md`](docs/architecture/observation_time_health_replay.md)
- [`docs/hardware/pin_mapping.md`](docs/hardware/pin_mapping.md)
- [`docs/commissioning/runtime_profile.md`](docs/commissioning/runtime_profile.md)

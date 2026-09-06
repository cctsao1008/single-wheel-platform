# Self-Balancing Single-Wheel Platform

A Rust `no_std` embedded control platform for a reaction-wheel-stabilized single-wheel robot.

The platform separates physical modeling, sensing, estimation, state feedback, actuator semantics, runtime authority, electrical output, and commissioning infrastructure. Reference-backed nominal parameters may instantiate the initial system; local measurement and system identification progressively replace those assumptions.

## Architecture

```text
Physical Robot
      |
      v
RawObservation
      |
      v
Sensor Transfer / Calibration / Body Frame
      |
      v
EstimatorMeasurement
      |
      v
StateEstimator
  /      |       \
linear  lightweight  EKF
observer  fusion
  \      |       /
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
   RuntimeAuthority
     /           \
 denied       authorized
   |              |
no output   AuthorizedActuation
                  |
                  v
          Electrical Output
                  |
                  v
             PWM / DIR
                  |
                  v
          Physical Actuators
```

The estimator boundary is canonical; EKF is a production-capable implementation, not an architectural requirement. Estimator complexity is selected from measured state quality, disturbance rejection, operating envelope, and real-time cost.

## Balance model

The reduced upright state is

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

with physical input

```text
u = [drive-wheel torque, reaction-wheel torque]^T
```

The nonlinear plant contract is

```text
M(q, p) q_ddot
+ c(q, q_dot, p)
+ g(q, p)
+ d(q_dot, p)
= B(p) u
```

Roll and pitch are not assumed globally decoupled. Any local pitch/drive and roll/reaction-wheel structure must emerge from the upright model.

## Measurement model

The estimator consumes physical observations through

```text
y = h(x, u, p)
```

with the current measurement channels

```text
accel_x, accel_y, accel_z
gyro_x, gyro_y, gyro_z
drive_encoder_relative_angle
reaction_wheel_relative_rate
```

Accelerometer output is modeled as specific force, including gravity, translational acceleration, angular acceleration at the IMU lever arm, rotational terms, and actuator feedthrough.

## Estimation and control

Available estimator implementations include:

```text
swp-state-estimator   fixed-gain discrete linear observer
swp-ekf               nonlinear covariance-based estimator
```

A lightweight complementary-class estimator is also a valid production strategy where measured performance justifies it.

State feedback is

```text
u = u_ff - K (x_hat - x_ref)
```

with optional LQI integral coordinates. Runtime authority owns integrator hold semantics when actuation is constrained or denied.

The executable control causality is

```text
measurement[k]
      |
      v
StateEstimator using applied u[k-1]
      |
      v
EstimatedState[k]
      |
      v
LQR / LQI
      |
      v
requested torque[k]
      |
      v
Actuator inverse model
      |
      v
bounded command[k]
      |
      v
RuntimeAuthority
      |
      v
physical applied input[k]
      |
      +------> estimator[k+1]
```

Requested torque, bounded command, authorized actuation, and electrical output remain distinct semantics.

## Runtime authority

`RuntimeAuthority` is the semantic boundary that creates `AuthorizedActuation`.

Authority considers:

```text
operating state
sensor timing health
estimated-state validity
reaction-wheel momentum / speed headroom
actuator saturation
```

A denied step cannot reach physical output. Constrained operation remains explicit and holds LQI integration.

## Reference-backed parameters

Physical provenance is explicit:

```text
measured
identified
datasheet
reference-platform
literature
derived
nominal
unknown
```

Architecture does not wait for every parameter to become locally measured. A nominal/reference value may instantiate the initial model when its source and confidence are explicit. Local measured or identified values supersede lower-confidence assumptions.

External platforms are used for model structure, estimator/controller methods, initial parameter ranges, and commissioning workflow; their gains and physical parameters are not copied blindly.

Useful references include:

- [Mini-Wheelbot](https://github.com/wheelbot/Mini-Wheelbot): nonlinear dynamics, complementary-class estimation, upright LQR, system identification, measured datasets.
- Wheel-E: nonlinear Lagrangian model, friction model, Kalman/LQG evaluation, STM32 real-time architecture.

## Numerical boundary

```text
HOST
  model derivation
  system identification
  exact ZOH
  observer / EKF design quantities
  LQR / LQI synthesis
  correlation
        |
        v
STM32F103
  deterministic sensor processing
  estimator execution
  state feedback
  actuator inversion
  runtime authority
  electrical output
```

Cortex-M fixed-size numerical kernels use `swp-dsp-kernel` backed by CMSIS-DSP. The MCU does not solve Riccati equations or matrix exponentials at runtime.

## Real-time target

Reference MCU: `STM32F103C8T6`.

```text
MPU6050 DATA_RDY / PC13 EXTI13   500 Hz control opportunity
TIM1                              1 kHz timing-health supervisor
TIM2                              Encoder_1 QEI
TIM4                              Encoder_2 QEI
ADC1 / PA5                        battery observation
USART2 TX / DMA1 CH7              telemetry transport
```

There is no catch-up control. One DATA_RDY event creates at most one physical control opportunity.

## Commissioning modes

Observation and shadow-control are commissioning modes, not the identity of the platform.

```text
observation
    acquire and record physical sensor evidence

live-shadow
    execute the full control computation without motor electrical ownership

closed-loop
    StateEstimator -> LQR/LQI -> RuntimeAuthority -> ElectricalOutput
```

## Repository structure

```text
crates/
  robot-domain/
  plant-model/
  measurement-model/
  dsp-kernel/
  state-estimator/
  ekf/
  estimator-input/
  state-feedback/
  control-runtime/
  actuator-model/
  runtime-state/
  plant-observation/
  sensor-calibration/
  frame-transform/
  reference-assembly/
  observation-record/
  control-profile-record/
  mpu6050/
  software-i2c/
  board-one-v2/

firmware/
  stm32f103/
  live-shadow-stm32f103/
  control-footprint-stm32f103/

parameters/
  reference-assembly.json

tools/
  model/
  control/
  actuator/
  recording/
  wireless/
```

## Reference assembly

```text
BLDC_1 / Encoder_1 -> ReactionWheel
BLDC_2 / Encoder_2 -> DriveWheel
BLDC_3             -> unused
```

Body frame:

```text
+X = forward
+Y = left
+Z = up
```

Board channel identity, installed assembly role, and robot-control semantics remain separate.

## Build

```bash
cargo fw
```

Architecture details:

- [`docs/architecture/system_architecture.md`](docs/architecture/system_architecture.md)
- [`docs/architecture/state_estimation_and_control.md`](docs/architecture/state_estimation_and_control.md)
- [`docs/architecture/estimator_input.md`](docs/architecture/estimator_input.md)
- [`docs/architecture/control_runtime.md`](docs/architecture/control_runtime.md)
- [`docs/architecture/plant_model.md`](docs/architecture/plant_model.md)
- [`docs/architecture/measurement_model.md`](docs/architecture/measurement_model.md)
- [`docs/architecture/runtime_authority.md`](docs/architecture/runtime_authority.md)

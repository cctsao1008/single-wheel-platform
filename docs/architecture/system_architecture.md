# System Architecture

The platform is organized around semantic transitions and physical ownership.

## Terminology

```text
platform
    repository and engineering / test infrastructure

robot
    complete physical sensing-and-actuation system under control

plant
    dynamic system seen by estimation and control

board
    PCB capability and electrical routing

reference-assembly
    installed hardware and board-channel-to-role mapping
```

`platform`, `robot`, and `plant` are related but not interchangeable. The repository is a platform; the controlled physical system is a robot; control and estimation operate on its plant dynamics.

## Core dataflow

```text
Physical Plant
      |
      v
RawObservation
      |
      | device transfer functions
      v
ScaledObservation
      |
      | measured calibration
      v
CalibratedObservation
      |
      | sensor -> body transform
      v
BodyObservation
      |
      v
EstimatedState
      |
      v
State-Space Control Law
      |
      v
GeneralizedDemand
      |
      v
ActuatorAllocation
      |
      v
RuntimeAuthority
      |
      v
ElectricalOutput
      |
      v
Physical Actuators
```

The estimator is bounded by explicit plant and measurement models:

```text
plant-model
    x_dot = f(x, u, p)

measurement-model
    y = h(x, u, p)
```

Recording is a branch from the raw-observation boundary:

```text
RawObservation
   |\
   | +--> semantic/control path
   |
   +--> RecordedObservation --> transport/storage --> replay
```

## Semantic ownership

```text
plant-model
    full robot configuration, reduced balance coordinates,
    physical parameters, nonlinear/linear plant contracts

measurement-model
    physical sensor equation, IMU lever-arm semantics,
    encoder kinematics, local observability structure

plant-observation
    raw values, timing, quality, acquisition status

mpu6050
    device protocol and nominal transfer functions

sensor-calibration
    measured sensor-frame correction

frame-transform
    sensor-frame to robot-body rotation

robot-domain
    robot state, generalized demand, actuator roles

runtime-state
    operating state, limits, physical-output authority
```

## Plant-model boundary

The physical robot and the balance controller use different model scopes.

The full robot configuration retains planar pose, yaw, body attitude, and motor-relative wheel coordinates:

```text
q_full = [world x, world y, yaw, pitch, roll,
          drive-wheel relative angle, reaction-wheel relative angle]^T
```

The single-wheel ground contact is nonholonomic, so the full configuration does not imply independent generalized velocities.

The current upright / straight-line balance reduction is:

```text
q_b = [forward displacement, pitch, roll, reaction-wheel relative angle]^T
u_ref = [drive torque, reaction-wheel torque]^T
```

with the drive-wheel relative coordinate eliminated by the local pure-rolling relation.

The reduced nonlinear model is represented as

```text
M(q_b, p) q_b_ddot
+ c(q_b, q_b_dot, p)
+ g(q_b, p)
+ d(q_b_dot, p)
=
B(p) u_ref
```

Roll and pitch are not assumed decoupled. Any reduction or decoupling must emerge from model structure, operating-region analysis, or physical correlation.

Yaw remains part of the full robot model even though it is not currently part of the reduced balance state. Turning, path following, and finite-speed gyroscopic coupling belong to the wider nonholonomic mobility problem rather than being hidden inside ad-hoc balance terms.

See [`plant_model.md`](plant_model.md).

## Measurement-model boundary

The estimator does not treat calibrated sensor values as if they were state variables. It compares physical body-frame observations against

```text
y = h(x, u, p)
```

or the stationary-upright local form

```text
y = y_0 + C x + D u
```

The current measurement model includes:

```text
body-frame accelerometer specific force
body-frame gyroscope angular rate
drive-wheel relative encoder angle
reaction-wheel relative encoder rate
```

The accelerometer equation includes gravity, translational acceleration, angular acceleration at the IMU lever arm, and direct actuator feedthrough. Accelerometer output is therefore not promoted to a geometric tilt angle before estimation.

The ideal local seven-state balance plant is structurally observable from encoder/gyro channels alone. This does not remove the accelerometer from the estimator: real bias, scale, timing, noise, vibration, and model error make independent specific-force evidence valuable.

Encoder scale/sign and IMU placement remain physical evidence requirements. Until measured, raw counts and an assumed lever arm are not promoted into the numeric measurement model.

See [`measurement_model.md`](measurement_model.md).

## Hardware ownership

```text
board-one-v2
    PCB pins, timers, buses, connector identities

reference-assembly
    installed hardware and board-channel-to-role mapping

firmware/stm32f103
    concrete STM32 peripheral ownership and RTIC execution
```

The current reference assembly has two populated balance actuators:

```text
BLDC_1 / Encoder_1 -> ReactionWheel
BLDC_2 / Encoder_2 -> DriveWheel
BLDC_3             -> unused by the reference assembly
```

Board capability, assembly population, and robot semantics remain separate types of information.

## Body frame

```text
+X = forward
+Y = left
+Z = up
```

Roll, pitch, and yaw follow the right-hand rule about +X, +Y, and +Z respectively.

## Measurement timing and quality

Scheduler time, physical source-sample time, peripheral capture time, readout completion time, and transmission time are distinct.

The MPU6050 INT pin is physically routed to PC13 / EXTI13. The current runtime does not yet use that interrupt and still configures DATA_RDY disabled, so `source_sample_at_us` remains `Unknown`; I2C read start/completion times remain available. Hardware interrupt capability is not promoted to timing evidence until the runtime actually captures and validates it.

`MeasurementQuality` carries independent availability, I/O, timing, freshness, saturation, staleness, and retry state. An unset flag does not imply the opposite property.

The physical measurement equation and measurement timing/quality are complementary contracts: a mathematically valid sensor model does not make stale or mistimed data valid.

## Runtime authority

The operating-state model is:

```text
Boot
  -> HardwareCheck
  -> Standby
  -> CaptureWindow
  -> Balancing
       |-> MomentumLimited
       |-> Fault
```

Only authorized closed-loop states may reach physical outputs. Reaction-wheel speed/headroom is part of actuator authority.

## Real-time runtime

The STM32F103 target uses Rust `no_std`, `embedded-hal` 1.0, `stm32f1xx-hal`, and RTIC.

The target composition is:

```text
TIM1          acquisition scheduling
DWT           monotonic acquisition timing
PB8/PB9       software I2C -> MPU6050
PC13          MPU6050 INT / EXTI13 hardware route, currently unused
TIM2          Encoder_1 QEI
TIM4          Encoder_2 QEI
ADC1 / PA5    battery ADC
USART2        ECB02S2 wireless record transport
USART1        wired engineering interface
PB4/PB5       OLED status interface
```

Control/acquisition work does not block on UART, BLE, display rendering, storage, or host traffic.

## Interface roles

```text
USART2 + ECB02S2
    wireless `RecordedObservation` transport for the mobile platform

USART1
    wired bench / engineering interface

OLED
    optional local status interface
```

The host-side BLE observer reassembles the byte stream independently of BLE packet boundaries and preserves the canonical binary records for decode and replay.

Transport and UI components may observe state or submit validated requests; they do not own physical semantics or bypass runtime authority.

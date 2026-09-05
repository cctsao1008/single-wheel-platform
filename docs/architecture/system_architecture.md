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

The estimator and controller are bounded by explicit plant and measurement models:

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

state-estimator
    discrete predictor/corrector execution, measurement masks,
    estimate validity

state-feedback
    physical-state LQR/LQI execution and integrator state

tools/control
    exact ZOH discretization, Riccati synthesis,
    generated numeric design matrices

plant-observation
    raw values, timing, quality, acquisition status

mpu6050
    device protocol and nominal transfer functions

sensor-calibration
    measured sensor-frame correction

frame-transform
    sensor-frame to robot-body rotation

robot-domain
    robot state, physical control demand, actuator roles

reference-assembly
    installed actuator roles and demand-to-plant-input allocation

runtime-state
    operating state, sensor timing health, limits,
    physical-output authority
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

## Estimation and control boundary

The current reduced controller state is

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

The real-time observer uses the exact discrete plant generated for the configured sample period:

```text
x_pred[k] = A_d x_hat[k-1] + B_d u[k-1]

innovation[k]
    = y[k] - y_0 - C x_pred[k] - D u[k]

x_hat[k] = x_pred[k] + L innovation[k]
```

Prediction input and measurement-feedthrough input remain separate arguments because they refer to different physical intervals.

The state-feedback boundary is

```text
u[k] = u_ff[k] - K (x_hat[k] - x_ref[k])
```

with optional LQI integral state when tracking requires it. LQI integration can be explicitly held when runtime authority denies or constrains actuation.

`GeneralizedDemand` is expressed in physical torque units for the two populated plant inputs:

```text
drive-wheel torque
reaction-wheel torque
```

It is not PWM, duty, timer compare, or motor-driver polarity. The current reference assembly maps those two semantic efforts one-to-one into the plant input vector, while board/electrical mapping remains downstream.

Numeric `A_d`, `B_d`, `L`, `K`, and optional LQI matrices are synthesized on the host from evidenced physical parameters. The STM32 executes those matrices deterministically and does not solve matrix exponentials or Riccati equations at runtime.

See [`state_estimation_and_control.md`](state_estimation_and_control.md).

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

Physical source-sample time, DATA_RDY event service time, peripheral capture time, readout completion time, and transmission time are distinct.

The MPU6050 INT pin is routed to PC13 / EXTI13 and the current runtime enables DATA_RDY. Successful interrupt-triggered reads are freshness-verified. EXTI task entry is timestamped with DWT, but that timestamp is not the MPU6050 internal sensing instant; `source_sample_at_us` therefore remains `Unknown` and sensor-filter delay remains explicit.

DATA_RDY drives the 500 Hz acquisition boundary, while TIM1 independently supervises it at 1 kHz. `SensorTimingMonitor` requires an observed inter-event cadence before declaring the boundary healthy, then classifies elapsed DATA_RDY time as `Healthy`, `Late`, or `Timeout`. A sensor interrupt cannot be responsible for detecting its own disappearance.

`MeasurementQuality` carries independent availability, I/O, timing, freshness, saturation, staleness, and retry state. `AcquisitionStatus` exposes DATA_RDY presence and timing-health state. A mathematically valid sensor model does not make stale or mistimed data valid.

The state estimator accepts explicit timing validity and required-measurement availability. Invalid timing, missing required channels, non-finite input, or numerical failure invalidates the estimate instead of fabricating continuity.

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

Only authorized closed-loop states may reach physical outputs. Reaction-wheel speed/headroom and primary sensor timing are independent authority constraints. `Balancing` or `MomentumLimited` may receive closed-loop authority only when primary sensor timing is `Healthy`; `Startup`, `Late`, and `Timeout` deny physical output authority.

Estimated-state validity is also a required physical-output condition before control is instantiated in firmware. The estimator core already exposes `Valid` / `Invalid`; physical actuation remains disabled until runtime composition connects that validity into the authority boundary together with measured actuator limits.

The current firmware remains observation-only, so these contracts are established before motor output exists rather than added after balancing is enabled.

## Real-time runtime

The STM32F103 target uses Rust `no_std`, `embedded-hal` 1.0, `stm32f1xx-hal`, and RTIC.

The target composition is:

```text
DWT            monotonic acquisition timing
PB8/PB9        software I2C -> MPU6050
PC13/EXTI13    MPU6050 DATA_RDY -> 500 Hz acquisition task
TIM1           independent 1 kHz sensor-timing watchdog
TIM2           Encoder_1 QEI
TIM4           Encoder_2 QEI
ADC1 / PA5     battery ADC
USART2_TX      ECB02S2 wireless record transport
DMA1 channel 7 USART2 TX record transfer
USART1         wired engineering interface
PB4/PB5        OLED status interface
```

Task priority follows control relevance:

```text
TIM1 timing health     priority 3
MPU DATA_RDY / EXTI13  priority 2
USART2 TX DMA complete priority 1
```

Control/acquisition work does not block on UART, BLE, display rendering, storage, or host traffic. The previous per-byte USART2 TXE interrupt path is replaced by DMA1 channel 7; an 80-byte record is transferred without 80 CPU service interrupts.

DMA is used where the hardware request topology creates a real reduction in CPU service work. MPU acquisition remains software I2C because the board routes PB8/PB9 opposite the STM32F103 I2C1-remap SCL/SDA assignment. DMA is not treated as a goal independent of hardware semantics.

The canonical IMU path is raw 500 Hz DATA_RDY acquisition. Vendor DMP output is not part of the control architecture.

The `state-estimator` and `state-feedback` crates are implemented, but numeric reference-platform gains are not instantiated in firmware until the parameter/calibration/actuator evidence required by synthesis is available.

## Interface roles

```text
USART2 + ECB02S2
    wireless `RecordedObservation` transport for the mobile platform

USART1
    wired bench / engineering interface

OLED
    optional local status interface
```

The host-side BLE observer reassembles the byte stream independently of BLE packet boundaries and preserves the canonical binary records for decode and replay. DMA boundaries are likewise not record semantics.

Transport and UI components may observe state or submit validated requests; they do not own physical semantics or bypass runtime authority.

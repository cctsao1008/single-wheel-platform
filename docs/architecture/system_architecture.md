# System Architecture

The platform is a complete embedded self-balancing control system. Observation, replay, and shadow-control are commissioning modes around the same canonical sensing, estimation, control, authority, and electrical-output architecture.

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

## Canonical runtime

```text
Physical Robot
      |
      v
RawObservation
      |
      | device transfer
      v
ScaledObservation
      |
      | calibration
      v
CalibratedObservation
      |
      | sensor -> body transform
      v
BodyObservation
      |
      | encoder semantic projection
      v
EstimatorMeasurement
      |
      v
StateEstimator
      |
      v
EstimatedState
      |
      v
LQR / LQI
      |
      v
GeneralizedDemand
      |
      v
Actuator Model / Inverse Model
      |
      v
Bounded Actuator Command
      |
      v
RuntimeAuthority
   /             \
denied          authorized
  |                 |
no output    AuthorizedActuation
                    |
                    v
             ElectricalOutput
                    |
                    v
               PWM / DIR
                    |
                    v
            Physical Actuators
```

Only `RuntimeAuthority` may create `AuthorizedActuation`. Electrical output is downstream of that type boundary.

## Estimator boundary

`StateEstimator` is the architecture boundary. The implementation may be selected independently of the controller:

```text
linear observer
lightweight complementary-class fusion
extended Kalman filter
```

The fixed-gain observer is useful for local-upright verification. `swp-ekf` provides a nonlinear covariance-based production-capable path. A lightweight estimator remains valid where measured performance is sufficient.

The architecture does not encode “EKF is always better”; estimator complexity is justified by measured state quality, innovation behavior, disturbance rejection, operating envelope, and execution cost.

## Plant boundary

The full robot configuration retains planar pose, yaw, body attitude, drive-wheel motion, and reaction-wheel motion. The reduced upright control state is

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

with plant input

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

Roll and pitch are not assumed globally decoupled. The stationary-upright model may expose local pitch/drive and roll/reaction-wheel structure; any such decoupling is a model result rather than a platform axiom.

## Measurement boundary

The estimator compares physical body-frame observations against

```text
y = h(x, u, p)
```

or its local upright form

```text
y = y_0 + C x + D u
```

The current measurement channels are body accelerometer specific force, body gyroscope rate, drive-wheel relative encoder angle, and reaction-wheel relative encoder rate.

The accelerometer model includes gravity, translational acceleration, IMU lever-arm angular acceleration, rotational terms, and actuator feedthrough. Raw acceleration is not silently promoted into a geometric tilt angle by the architecture.

## Control causality

One physical control opportunity is

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
applied input[k]
      |
      +------> estimator[k+1]
```

Requested torque, bounded actuator command, authorized actuation, and physical electrical output are distinct meanings.

LQI integration advances only when the candidate actuation remains authorized and unconstrained. Runtime authority therefore owns anti-windup permission rather than hidden controller saturation.

## Parameter provenance

Reference-backed nominal values are valid initial engineering inputs. Evidence changes confidence and correlation quality; it does not gate the existence of the software architecture.

Supported provenance classes are

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

A value may move from nominal/reference to measured/identified without changing its physical meaning or software interface.

External platforms are used for method and range evidence, not blind gain copying. The Mini-Wheelbot is a useful executable reference for nonlinear Wheelbot dynamics, complementary-class estimation, upright LQR, and nonlinear system identification. Wheel-E is useful literature evidence for nonlinear Lagrangian modeling, friction modeling, Kalman/LQG evaluation, and STM32 real-time architecture.

## Hardware ownership

```text
board-one-v2
    PCB pins, timers, buses, connector identities

reference-assembly
    installed hardware and board-channel-to-role mapping

firmware
    concrete STM32 peripheral ownership and real-time execution
```

Reference assembly:

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

## Real-time boundary

Reference target: STM32F103C8T6.

```text
PC13 / EXTI13   MPU6050 DATA_RDY -> 500 Hz control opportunity
TIM1            1 kHz sensor-timing watchdog
TIM2            Encoder_1 QEI
TIM4            Encoder_2 QEI
ADC1 / PA5      battery observation
USART2 / DMA    asynchronous telemetry transport
```

Control never blocks on BLE, UART, display, storage, or host traffic. One DATA_RDY event creates at most one control opportunity; missed periods are not replayed as catch-up control.

The MPU6050 path remains raw DATA_RDY acquisition. Vendor DMP output is not part of the canonical estimator interface.

## Commissioning modes

```text
observation
    sensor acquisition and evidence recording

live-shadow
    full estimator/control/authority computation with motor electrical ownership absent

closed-loop
    estimator -> control -> authority -> electrical output
```

These are operating modes around one architecture, not separate platform identities.

## Host / MCU split

Host engineering owns:

```text
symbolic / nonlinear model derivation
system identification
parameter correlation
exact zero-order-hold discretization
observer / EKF design quantities
LQR / LQI synthesis
closed-loop analysis
```

STM32 execution owns:

```text
sensor semantics
state estimation
state feedback
actuator inversion
runtime authority
electrical output
```

Fixed-size Cortex-M numerical kernels use `swp-dsp-kernel` backed by CMSIS-DSP. The MCU does not solve Riccati equations or matrix exponentials at runtime.

## Engineering loop

```text
reference / datasheet / literature
              |
              v
       nominal parameter set
              |
              v
       executable model
              |
              v
 estimator + controller + authority
              |
              v
        physical robot
              |
              v
 measurement / system identification
              |
              v
       model correlation
              |
              +--------> replace nominal assumptions
```

The architecture is complete first; physical evidence progressively improves the model, estimator, controller, and authority envelope.
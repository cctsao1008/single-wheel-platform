# State Estimation and State-Space Control

The balance runtime is model-based but estimator-agnostic. The architectural boundary is `StateEstimator`; a particular estimator implementation is selected from timing, model fidelity, sensor quality, and measured closed-loop behavior.

```text
BodyObservation + Encoder Kinematics
              |
              v
     EstimatorMeasurement
              |
              v
        StateEstimator
        /     |      \
       /      |       \
linear   lightweight   EKF
observer  fusion       estimator
       \      |       /
        \     |      /
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
      RuntimeAuthority
              |
              v
       ElectricalOutput
```

The estimator boundary is canonical. EKF is a production-capable implementation, not a requirement for the architecture to exist.

## State contract

The reduced upright balance state is

```text
x = [
    s,
    s_dot,
    theta,
    theta_dot,
    phi,
    phi_dot,
    psi_r_dot,
]^T
```

where `s` is forward displacement, `theta` is body pitch, `phi` is body roll, and `psi_r_dot` is reaction-wheel rate relative to the body.

The physical plant input is

```text
u = [tau_drive, tau_reaction]^T
```

PWM, direction GPIO, timer compare values, and motor-driver polarity are downstream electrical semantics.

## Plant and measurement models

The host derives the nonlinear plant

```text
M(q, p) q_ddot + c(q, q_dot, p) + g(q, p) + d(q_dot, p) = B(p) u
```

and the local upright model used for synthesis and deterministic embedded execution.

The estimator consumes the physical measurement model

```text
y = h(x, u, p)
```

with the current channel order

```text
accel_x
accel_y
accel_z
gyro_x
gyro_y
gyro_z
drive_encoder_relative_angle
reaction_wheel_relative_rate
```

The accelerometer is treated as a specific-force sensor. Translational acceleration, gravity, angular acceleration at the IMU lever arm, rotational terms, and actuator feedthrough belong in the measurement model rather than being silently collapsed into a geometric tilt angle.

## Estimator implementations

### Linear observer

`swp-state-estimator` provides the fixed-rate discrete predictor/corrector

```text
x_pred[k] = A_d x_hat[k-1] + B_d u[k-1]
innovation[k] = y[k] - y_0 - C x_pred[k] - D u[k]
x_hat[k] = x_pred[k] + L innovation[k]
```

It is useful as a deterministic reference implementation, for local-upright verification, and where fixed-gain estimation is sufficient.

### Lightweight fusion

A complementary-class estimator is a valid production option when the physical sensor geometry and measured dynamics show that it provides adequate state quality. This is not treated as a lesser architecture.

The open Mini-Wheelbot is a useful reference: its deployed estimator combines gyro integration, accelerometer tilt correction, motor encoder state, and complementary fusion while its normal balancing controller uses state feedback. That implementation demonstrates that a reaction-wheel unicycle does not require a full EKF merely to achieve stable balancing.

Reference: <https://github.com/wheelbot/Mini-Wheelbot>

### Extended Kalman filter

`swp-ekf` provides the model-based nonlinear-estimator path. Its production design uses fixed-size `no_std` storage, local nonlinear measurement Jacobians, sequential scalar measurement updates, and covariance propagation suitable for Cortex-M execution.

EKF is preferred when its additional model/noise treatment produces measurable benefit in state error, innovation behavior, disturbance rejection, or operating envelope. It is not selected solely because it is mathematically more elaborate.

## Controller

The balance controller remains state feedback independent of the chosen estimator:

```text
u[k] = u_ff[k] - K (x_hat[k] - x_ref[k])
```

LQI may add explicit integral coordinates when zero steady-state tracking error is required. Runtime authority controls whether integration may advance so saturation, invalid estimation, momentum limits, or timing faults cannot create hidden wind-up.

The nonlinear plant is not globally decoupled. Around stationary upright, the model may expose a useful local structure:

```text
pitch / translation -> drive-wheel torque
roll / reaction-wheel momentum -> reaction-wheel torque
```

That structure may be exploited by controller synthesis when it follows from the model. It is not assumed as a global physical truth.

The Mini-Wheelbot provides a useful external example of this distinction: it uses a full nonlinear Wheelbot model for simulation/system identification while its regular balancing controller uses separate four-state roll and pitch LQR feedback paths.

## Reference-backed nominal parameters

Physical evidence does not gate architectural completeness. A complete executable controller may be instantiated from reference-backed nominal parameters and then improved by local measurements.

Accepted provenance classes are:

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

A nominal or literature-derived value must remain labeled as such. It must not be represented as measured or identified.

The precedence is generally:

```text
local measured / identified
        ^
local geometry / component datasheet
        ^
reference platform / literature
        ^
engineering nominal estimate
```

New local evidence replaces lower-confidence assumptions without changing the software architecture.

## External reference evidence

External platforms are used for method, model structure, initial ranges, and commissioning strategy; their physical gains and parameters are not copied blindly into this robot.

### Mini-Wheelbot

The public Mini-Wheelbot repository provides:

```text
nonlinear rigid-body dynamics
reaction-wheel and drive-wheel torque inputs
state-feedback balancing controller
complementary-class state estimation
nonlinear system-identification scripts
measured disturbance datasets
parameter-bounded fitting workflow
```

This is high-value executable reference evidence for software structure and system-identification workflow.

### Wheel-E

Wheel-E provides literature evidence for:

```text
full nonlinear Lagrangian modeling
explicit friction modeling
linearization and controllability analysis
Kalman filtering / LQG evaluation
sensor-placement effects
STM32 real-time control architecture
```

Its Kalman/LQG results are primarily simulation/design evidence and therefore do not supersede local robot measurements.

## Numerical execution boundary

The host owns expensive design operations:

```text
symbolic / nonlinear model derivation
system identification
exact zero-order-hold discretization
observer / EKF design quantities
LQR / LQI synthesis
closed-loop eigenvalue checks
parameter correlation
```

The STM32 owns deterministic real-time execution:

```text
measurement projection
state estimation
state feedback
actuator inversion
runtime authority
physical electrical output
```

Cortex-M fixed-size dot products and matrix/vector kernels use `swp-dsp-kernel` and CMSIS-DSP. Host scalar code exists only for semantic tests.

## Development and correlation loop

The platform follows

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
 estimator + LQR/LQI + authority
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

The goal is not to delay implementation until every parameter is measured. The goal is to keep provenance explicit while continuously improving model correlation and closed-loop performance.
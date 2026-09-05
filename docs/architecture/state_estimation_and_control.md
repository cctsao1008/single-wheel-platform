# State Estimation and State-Space Control

The upright balance controller operates on estimated physical state, not peripheral values. The real-time path is intentionally split into deterministic execution on the MCU and numerical design on the host.

```text
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
GeneralizedDemand
      |
      v
Reference-Assembly Allocation
      |
      v
RuntimeAuthority
      |
      v
ElectricalOutput
```

## State contract

The current reduced stationary-upright state is

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

with

```text
s           forward displacement
theta       body pitch
phi         body roll
psi_r_dot   reaction-wheel rate relative to the body
```

This is a local balance state, not the full mobile-robot configuration. Yaw and planar nonholonomic mobility remain in the wider plant model.

The physical input vector is

```text
u = [
    tau_drive,
    tau_reaction,
]^T
```

Controller demand is therefore expressed as physical drive-wheel and reaction-wheel torque. PWM, direction GPIO, motor-driver scaling, saturation, and electrical polarity remain downstream concerns.

## Discrete plant

The controller and observer use the exact zero-order-hold discrete model at the configured inner-loop period:

```text
x[k+1] = A_d x[k] + B_d u[k]
```

For the current 500 Hz target,

```text
T_s = 0.002 s
```

The canonical host synthesis computes

```text
A_d = exp(A_c T_s)

B_d = integral_0^T_s exp(A_c tau) B_c d(tau)
```

rather than using a forward-Euler approximation.

No numeric `A_d`, `B_d`, controller gain, or observer gain becomes a reference-platform fact until the required physical parameters and design evidence are supplied.

## Measurement equation

The estimator uses the physical local measurement model

```text
y[k] = y_0 + C x[k] + D u[k]
```

with the fixed channel order

```text
[
    accel_x,
    accel_y,
    accel_z,
    gyro_x,
    gyro_y,
    gyro_z,
    drive_encoder_relative_angle,
    reaction_wheel_relative_rate,
]
```

The direct-feedthrough matrix `D` is retained. Accelerometer specific force can change during the same sample interval as actuator effort; the estimator must not silently model that effect as state alone.

## Observer execution

`swp-state-estimator` implements the fixed-rate linear predictor/corrector:

```text
x_pred[k]
    = A_d x_hat[k-1] + B_d u[k-1]

innovation[k]
    = y[k] - y_0 - C x_pred[k] - D u[k]

x_hat[k]
    = x_pred[k] + L innovation[k]
```

`u[k-1]` and `u[k]` are explicit separate inputs because state propagation and direct sensor feedthrough have different time meanings.

The estimator also receives an explicit measurement-availability mask and timing-valid flag. A required missing channel, non-finite value, invalid primary timing boundary, or numerical fault invalidates the estimate rather than silently continuing with fabricated state.

The current estimator core does not choose `L` on the MCU. Observer gain synthesis is a host-side numerical-design function.

## LQR

`swp-state-feedback` executes the physical state-feedback law

```text
u[k]
    = u_ff[k] - K (x_hat[k] - x_ref[k])
```

The controller does not clamp, normalize, or convert torque demand into PWM. An unconstrained demand is useful information: downstream allocation and runtime authority own the distinction between desired and physically allowed actuation.

The canonical host synthesis uses the discrete algebraic Riccati equation on `(A_d, B_d)`. Initial diagonal `Q` and `R` weighting is parameterized through explicit physical state and input scales rather than inherited legacy gains.

## LQI

When zero steady-state tracking error is required, two explicit integral coordinates may be added:

```text
z[k+1]
    = z[k] + T_s C_i (x_hat[k] - x_ref[k])

u[k]
    = u_ff[k]
      - K_x (x_hat[k] - x_ref[k])
      - K_i z[k+1]
```

`C_i` is an explicit projection from the seven-state error into the two integrated quantities. The chosen projection is therefore part of the controller design rather than an implicit historical convention.

The real-time LQI API exposes `Integrate` and `Hold`. Runtime authority can freeze integration when actuation is denied or constrained so anti-windup behavior follows actual authority rather than hidden controller saturation.

## Numeric design boundary

Host-side synthesis is in:

```text
tools/control/synthesize_upright.py
```

The host owns:

```text
physical-parameter ingestion
exact zero-order-hold discretization
LQR / optional LQI Riccati synthesis
steady-state discrete observer synthesis
closed-loop eigenvalue checks
generated Rust matrix constants
```

The MCU owns:

```text
fixed-rate prediction
measurement correction
state validity
matrix state feedback
integrator state
runtime authority interaction
```

This split keeps expensive numerical synthesis off the STM32F103 while preserving a deterministic and inspectable real-time implementation.

## Current instantiation status

The estimator and state-feedback cores are implemented as reusable `no_std` components, but the reference firmware does not yet instantiate numeric gains or authorize motor output.

The missing physical evidence currently includes, at minimum:

```text
body / wheel masses and centers of mass
body roll / pitch inertia
reaction-wheel spin / transverse inertia
drive-wheel spin inertia and radius
IMU lever-arm placement
encoder scale and sign
actuator command -> physical torque behavior
measurement noise / process-residual levels
```

The synthesis template leaves unsupported quantities as `null` and refuses to generate a numeric design until they are supplied. This is deliberate: unknown parameters are identified, not invented.

## Acceptance criterion

A generated estimator/controller is not accepted because the Riccati equation has a solution. It must pass the physical correlation chain:

```text
identified parameters
      |
      v
model prediction
      |
      v
measured open-loop correlation
      |
      v
observer residual correlation
      |
      v
bounded closed-loop commissioning
      |
      v
closed-loop residual / authority analysis
```

Model disagreement is engineering evidence. It is not hidden by retuning gains until the physical cause is understood.

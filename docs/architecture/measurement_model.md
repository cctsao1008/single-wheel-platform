# Measurement Model

The measurement model defines what the physical sensors are expected to observe from the current balance state.

It is distinct from acquisition, scaling, calibration, frame transformation, and estimation:

```text
RawObservation
      ↓
Scaling / calibration
      ↓
BodyObservation
      ↓
Measurement model comparison
      ↓
State estimator
```

The measurement model predicts ideal body-frame sensor quantities. It does not own raw register decoding, sensor calibration, encoder scaling, or timestamp validity.

## Reduced balance state

The current controller-facing state is

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

with physical input

```text
u = [tau_drive, tau_reaction]^T
```

The local stationary-upright measurement equation is written as

```text
y = y_0 + C x + D u
```

The current ideal measurement vector is

```text
y = [
    accel_x,
    accel_y,
    accel_z,
    gyro_x,
    gyro_y,
    gyro_z,
    drive_encoder_relative_angle,
    reaction_wheel_relative_rate,
]^T
```

The reaction-wheel encoder physically measures relative wheel motion. Its absolute phase is cyclic for the current balance dynamics, so the reduced estimator/controller interface uses the unwrapped relative rate rather than promoting wheel phase into the control state.

## IMU frame and placement

Sensor calibration and sensor-to-body rotation occur before this model. The measurement equation therefore uses the canonical robot body frame:

```text
+X = forward
+Y = left
+Z = up
```

The remaining IMU geometry is its lever arm from the reduced-model body origin / drive-wheel axle:

```text
r_i = [x_i, y_i, z_i]^T
```

This placement is a physical parameter. It must be measured from the assembly rather than guessed.

## Gyroscope equation

For the reduced orientation convention

```text
R = R_y(theta) R_x(phi)
```

the exact body angular rate is

```text
omega_b = [
    phi_dot,
    theta_dot cos(phi),
    -theta_dot sin(phi),
]^T
```

At stationary upright, the first-order gyro equation is therefore

```text
gyro_x = phi_dot
gyro_y = theta_dot
gyro_z = 0
```

The zero `gyro_z` row is not a claim that real yaw rate is always zero. Yaw is outside the current reduced balance state and remains part of the wider robot model.

## Accelerometer equation

An accelerometer measures specific force, not tilt angle.

For a body-fixed IMU point,

```text
f_b = R^T (a_o - g_w)
    + alpha_b × r_i
    + omega_b × (omega_b × r_i)
```

where `a_o` is the translational acceleration of the reduced-model body origin.

Around stationary upright and zero rates, centripetal terms are second order. The first-order body-frame equations become

```text
accel_x = s_ddot - g theta + z_i theta_ddot

accel_y = g phi - z_i phi_ddot

accel_z = g - x_i theta_ddot + y_i phi_ddot
```

Substituting the upright plant dynamics produces both state sensitivity and direct actuator feedthrough:

```text
y = y_0 + C x + D u
```

The `D u` term is physically important. During active balancing, accelerometer output includes translational and angular acceleration caused by motor torque. Treating accelerometer vectors as pure gravity direction would therefore inject a false attitude observation during maneuvers.

## Encoder equations

The drive motor relative angle is

```text
delta_d = s / r - theta
```

so the ideal drive encoder equation is

```text
y_drive = s / r - theta
```

and its rate is

```text
delta_d_dot = s_dot / r - theta_dot
```

The reference assembly maps Encoder_2 to the drive wheel, but encoder sign and counts/revolution are not yet canonical physical facts. Raw counts cannot enter this equation until those quantities are measured.

The reaction-wheel encoder measures wheel rotation relative to the body. For the reduced balance state the relevant channel is

```text
y_reaction_rate = psi_r_dot
```

again only after count scale, sign, wrap handling, and timing are established.

## Structural observability

A useful result appears when the measurement model is combined with the upright plant.

The ideal seven-state local model is structurally observable even without using accelerometer channels.

For the pitch / translation state

```text
x_p = [s, s_dot, theta, theta_dot]^T
```

use only

```text
y_p = [
    s / r - theta,
    theta_dot,
]^T
```

corresponding to drive relative angle and pitch gyro rate.

A nonzero observability minor is

```text
O_pitch_minor = H M_s g / (Delta_pitch r^2)
```

which is nonzero for positive physical parameters and a nonsingular pitch inertia matrix.

For the roll / momentum state

```text
x_r = [phi, phi_dot, psi_r_dot]^T
```

use only

```text
y_r = [
    phi_dot,
    psi_r_dot,
]^T
```

corresponding to roll gyro rate and reaction-wheel relative rate.

A nonzero observability minor is

```text
O_roll_minor = H g / J_phi
```

which is also nonzero for the physical upright model.

Therefore the ideal stationary-upright reduced plant is generically observable from encoder/gyro information alone.

This result must not be overinterpreted. It does **not** prove that accelerometer data is unnecessary in the physical estimator. The structural calculation currently excludes:

```text
gyro bias and drift
encoder scale/sign uncertainty
quantization
measurement delay
sample-time uncertainty
model error
friction uncertainty
unmodeled yaw coupling
vibration
```

The accelerometer remains valuable as an independent gravity/specific-force observation, for bias rejection, model residual detection, and estimator robustness.

## Estimator consequence

The estimator should not implement the legacy assumption

```text
accelerometer angle + gyro integration = attitude
```

as its fundamental model.

Instead it should compare actual body-frame observations against the physical sensor equation

```text
y = h(x, u, p)
```

or its local linearization

```text
y = y_0 + C x + D u
```

This permits a Kalman-family estimator, nonlinear observer, or later model-based estimator to reason explicitly about motion-induced specific force rather than treating it as sensor error.

## Evidence still required

The measurement equation is structurally defined, but a physical numeric instance still requires:

```text
IMU lever arm from the body origin
Drive encoder counts/revolution
Drive encoder sign
Reaction-wheel encoder counts/revolution
Reaction-wheel encoder sign
Encoder wrap / unwrapping policy
Effective encoder capture timing
Residual gyro bias behavior after calibration
```

Until these are measured, no code should promote raw encoder counts into canonical radians or rad/s.

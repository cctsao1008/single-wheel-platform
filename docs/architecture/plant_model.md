# Plant Model

The plant model is the canonical physical model used by system identification, estimation, and control synthesis.

It is a current model, not a versioned artifact. When evidence changes the model, this document and the corresponding source are updated directly; Git preserves the evolution.

## Model hierarchy

The physical robot and the balance controller do not require the same state description.

The full robot configuration keeps planar pose, yaw, body attitude, and both motor-relative wheel coordinates available:

```text
q_full = [x_w, y_w, gamma, theta, phi, delta_d, psi_r]^T

x_w, y_w   world-plane position
gamma      yaw
theta      body pitch
phi        body roll
delta_d    drive-wheel angle relative to the body
psi_r      reaction-wheel angle relative to the body
```

The single-wheel ground contact is nonholonomic, so these coordinates do not imply seven independent generalized velocities. Rolling/contact constraints determine the admissible motion.

The current balance model is a local reduction for upright and straight-line balance. It uses

```text
q_b = [s, theta, phi, psi_r]^T

s       forward path displacement
theta   body pitch
phi     body roll
psi_r   reaction-wheel angle relative to the body
```

The drive-wheel relative coordinate is eliminated through the local pure-rolling relation

```text
delta_d = s / r_drive - theta
```

with signs defined by the body-frame and actuator conventions.

The reduced balance state is

```text
x_b = [
    s,
    s_dot,
    theta,
    theta_dot,
    phi,
    phi_dot,
    psi_r_dot,
]^T
```

For an axisymmetric reaction wheel, wheel phase is cyclic in the current balance model, so `psi_r` itself is omitted from the reduced state while its relative rate is retained.

This reduced state is not declared to be the full robot state. Yaw, planar path geometry, and nonholonomic coupling remain part of the wider plant description and are promoted into estimator/controller state when the operating problem requires them.

## Reference input

The populated reference assembly currently has two physical actuator inputs:

```text
u_ref = [
    tau_drive,
    tau_reaction,
]^T
```

Both are motor torques expressed with the model sign convention, not PWM values.

The board exposes a third motor channel, but the current reference assembly does not populate it. A yaw/turn input is therefore not part of `u_ref`; it is added only when a physical actuator and its mechanical input mapping are part of the reference assembly.

## Canonical nonlinear form

The reduced balance plant is represented in generalized-coordinate form:

```text
M(q_b, p) q_b_ddot
+ c(q_b, q_b_dot, p)
+ g(q_b, p)
+ d(q_b_dot, p)
=
Q(q_b, q_b_dot, u_ref, p)
```

where

```text
M   coupled inertia matrix
c   Coriolis / centrifugal terms
g   gravity terms
d   identified dissipation and low-order losses
Q   generalized actuator forces
p   physical parameter set
```

`M`, `c`, and `g` are not assumed block diagonal. Roll and pitch are separated only if the derived model, linearization, scale analysis, or measured correlation demonstrates that the coupling is negligible in a specified operating region.

## Virtual-work input mapping

The actuator mapping follows the coordinates actually chosen by the model.

For the drive motor, the motor-relative wheel angle is

```text
delta_d = s / r_drive - theta
```

so virtual work is

```text
dW_drive
    = tau_drive d(delta_d)
    = (tau_drive / r_drive) ds - tau_drive d(theta)
```

and therefore

```text
Q_drive = [
    tau_drive / r_drive,
    -tau_drive,
    0,
    0,
]^T
```

For the reaction wheel, `psi_r` is already the wheel angle relative to the body. The motor acts directly on that relative coordinate:

```text
dW_reaction = tau_reaction d(psi_r)
```

so

```text
Q_reaction = [
    0,
    0,
    0,
    tau_reaction,
]^T
```

There is intentionally no second explicit `-tau_reaction` term in the roll coordinate. The equal-and-opposite body reaction appears through the coupled kinetic energy because the reaction wheel's inertial angular rate contains both body roll rate and wheel-relative spin rate. Adding an explicit roll torque in these coordinates would double-count the internal action/reaction pair.

Electrical polarity, PWM direction, connector identity, and motor-driver sign are outside this model. Those mappings belong below actuator semantics and are verified during commissioning.

## Physical parameters

A numeric plant is created only when its required parameters are supported by measurement, identification, or sufficiently specific component data.

Current parameter classes include:

```text
body
    mass
    center-of-mass height
    roll inertia
    pitch inertia
    yaw inertia

drive wheel
    mass
    radius
    spin inertia

reaction wheel
    mass
    center height
    spin inertia
    transverse inertia

actuation / losses
    drive torque mapping
    reaction-wheel torque mapping
    friction / damping
    actuator delay
```

Unknown parameters remain unknown until measured or identified. Generic textbook values are not substituted into the canonical parameter set.

## Upright operating point

The principal balance linearization point is the stationary upright condition:

```text
theta = 0
phi   = 0
s_dot = 0
psi_r_dot = 0
u_ref = 0
```

Forward position and reaction-wheel phase are cyclic coordinates in this operating problem.

For a nonlinear state equation

```text
x_dot = f(x, u, p)
```

the continuous linear model is

```text
A = df/dx
B = df/du
```

evaluated at the selected operating point.

The real-time control model is discretized using the actual control period. For the current 500 Hz inner-loop target:

```text
T_s = 0.002 s
```

and the discrete form is

```text
x[k+1] = A_d x[k] + B_d u[k]
```

Zero-order hold is the default input assumption until actuator characterization justifies a different model.

## Controllability and observability

Controller synthesis begins from plant structure rather than legacy controller topology.

The model must establish:

```text
controllability rank
observability rank
mode locations
state and input scaling
dominant coupling terms
parameter sensitivity
```

A state that cannot be measured directly may remain usable when the estimator has sufficient observability. A state is removed only because model structure and measured behavior justify the reduction.

## Yaw and mobility

Yaw is part of the full robot configuration even though it is not currently part of the reduced balance state.

The reduced balance model is intended for upright / straight-line stabilization. It does not assert that yaw coupling is zero. Turning, path-following, finite-speed gyroscopic effects, or a future populated yaw actuator require the full nonholonomic mobility model rather than ad-hoc correction terms inside the balance controller.

Model reduction is therefore an explicit operating-region decision, not an erasure of physical degrees of freedom.

## Correlation contract

The plant model is accepted only through comparison with the physical system.

Relevant correlation data includes:

```text
free response
body angular response
reaction-wheel acceleration
drive-wheel acceleration
actuator step response
frequency response
cross-axis response
finite-speed behavior
saturation behavior
closed-loop residuals
```

The goal is not to preserve a particular equation set. The goal is to keep the canonical model aligned with measured physics.

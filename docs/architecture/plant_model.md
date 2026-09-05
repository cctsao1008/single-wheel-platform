# Plant Model

The plant model is the canonical physical model used by system identification, estimation, and control synthesis.

It is a current model, not a versioned artifact. When evidence changes the model, this document and the corresponding source are updated directly; Git preserves the evolution.

## Modeling scope

The current balance model uses the generalized coordinates

```text
q = [s, theta, phi, psi]^T

s       forward displacement

theta   body pitch angle
phi     body roll angle
psi     reaction-wheel angle relative to the body
```

The drive-wheel absolute rotation is eliminated with the local no-slip constraint

```text
alpha_drive = s / r_drive
```

The reaction-wheel angle `psi` is retained in the generalized coordinates because it is required by the mechanical energy and virtual-work formulation. For an axisymmetric reaction wheel, absolute wheel phase is cyclic, so the reduced control state omits `psi` itself and retains its rate.

The reduced state is

```text
x = [
    s,
    s_dot,
    theta,
    theta_dot,
    phi,
    phi_dot,
    psi_dot,
]^T
```

The physical input is

```text
u = [
    tau_drive,
    tau_reaction,
]^T
```

where both inputs are motor-shaft torques expressed with the model sign convention, not PWM values.

## Canonical nonlinear form

The plant is represented in generalized-coordinate form:

```text
M(q, p) q_ddot
+ c(q, q_dot, p)
+ g(q, p)
+ d(q_dot, p)
=
G(q, p) u
```

where

```text
M   coupled inertia matrix
c   Coriolis / centrifugal terms
g   gravity terms
d   identified dissipation and unmodeled low-order losses
G   physical input mapping
p   physical parameter set
```

`M`, `c`, and `g` are not assumed block diagonal. Roll and pitch are not separated merely because the legacy firmware used independent control loops. Any useful decoupling must emerge from the derived model, linearization, scale analysis, or measured correlation.

## Input mapping

Virtual work defines the current actuator-coordinate contract.

For the drive motor,

```text
alpha_drive = s / r_drive
```

and positive drive torque increases the drive-wheel rotation associated with positive forward motion while applying the equal/opposite torque to the body pitch coordinate.

Therefore the generalized contribution is

```text
Q_drive = [
    tau_drive / r_drive,
    -tau_drive,
    0,
    0,
]^T
```

For the reaction wheel, motor torque acts internally between body roll and wheel relative rotation:

```text
Q_reaction = [
    0,
    0,
    -tau_reaction,
    +tau_reaction,
]^T
```

The complete generalized force is the sum of these contributions.

Electrical polarity, PWM direction, connector identity, and motor-driver sign are intentionally outside this model. Those mappings belong below actuator semantics and are verified during commissioning.

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

Product-level facts such as total platform mass, nominal motor data, and reaction-wheel geometry constrain identification, but they do not automatically determine the separated inertial and actuator parameters required by the model.

Unknown parameters remain unknown until identified. Generic textbook values are not substituted into the canonical parameter set.

## Upright operating point

The principal linearization point for balancing is the stationary upright condition:

```text
theta = 0
phi   = 0
s_dot = 0
psi_dot = 0
u = 0
```

Absolute forward position and reaction-wheel phase are arbitrary cyclic coordinates.

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

The real-time control model is then discretized with the actual control period. For the current 500 Hz inner-loop target:

```text
T_s = 0.002 s
```

and the canonical discrete form is

```text
x[k+1] = A_d x[k] + B_d u[k]
```

Zero-order hold is the default discretization assumption unless actuator characterization supports a more accurate input model.

## Controllability and observability

Controller synthesis does not begin from legacy gain topology.

The model must first establish the relevant structure through:

```text
controllability rank
observability rank
mode locations
state and input scaling
dominant coupling terms
parameter sensitivity
```

A state that cannot be measured directly may remain usable if the estimator has sufficient observability. A state that is dynamically irrelevant should be removed because the model demonstrates that fact, not because legacy code omitted it.

## Yaw

Yaw rate is observable from the IMU, but the current reference assembly has no dedicated yaw actuator and no canonical yaw-position measurement.

The balance model therefore does not currently promote yaw position into the reduced state. This is a model-scope decision, not a claim that yaw coupling is physically zero. If measured correlation shows that yaw dynamics or gyroscopic coupling materially affects balance, the generalized-coordinate model is expanded rather than compensated with ad-hoc controller terms.

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
saturation behavior
closed-loop residuals
```

The goal is not to preserve a particular equation set. The goal is to keep the canonical model aligned with measured physics.

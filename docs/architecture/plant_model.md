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

The current balance model is a local reduction for upright and straight-line balance:

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

For an axisymmetric reaction wheel, wheel phase is cyclic in the current balance model, so `psi_r` itself is omitted from the reduced control state while its relative rate is retained.

This reduced state is not declared to be the full robot state. Yaw, planar path geometry, and nonholonomic coupling remain part of the wider plant description and are promoted when the operating problem requires them.

## Reference input

The populated reference assembly has two physical actuator inputs:

```text
u_ref = [
    tau_drive,
    tau_reaction,
]^T
```

Both are motor torques expressed with the model sign convention, not PWM values.

The board exposes a third motor channel, but the current reference assembly does not populate it. A yaw/turn input is therefore not part of `u_ref`.

## Canonical nonlinear form

The reduced balance plant is

```text
M(q_b, p) q_b_ddot
+ c(q_b, q_b_dot, p)
+ g(q_b, p)
+ d(q_b_dot, p)
=
B(p) u_ref
```

where

```text
M   coupled inertia matrix
c   Coriolis / centrifugal vector
g   gravity vector
d   identified dissipation and low-order losses
B   physical input map
p   physical parameter set
```

No roll/pitch decoupling is assumed before derivation.

## Physical parameter reduction

For the current balance derivation define

```text
H   = m_b h_b + m_r h_r
S   = m_b h_b^2 + m_r h_r^2
M_s = m_b + m_r + m_d + J_d / r^2
```

and

```text
J_theta = S + I_by + J_t
J_phi   = S + I_bx
```

with

```text
m_b    body mass excluding the rotating wheel inertias represented separately
h_b    body center-of-mass height
I_b*   body rotational inertias

m_d    drive-wheel mass
r      drive-wheel radius
J_d    drive-wheel spin inertia

m_r    reaction-wheel mass
h_r    reaction-wheel center height
J_r    reaction-wheel spin inertia
J_t    reaction-wheel transverse inertia
```

The drive-wheel gravitational potential is constant in the local rolling model and is omitted.

The current parameter partition is a model contract, not a claim that every value is already known. Unknown quantities remain unknown until measured or identified.

## Kinetic and potential energy

Using the orientation convention

```text
R = R_y(theta) R_x(phi)
```

the center of mass of a component located at height `h` on the body +Z axis is

```text
p(h) = [
    s + h sin(theta) cos(phi),
    -h sin(phi),
    h cos(theta) cos(phi),
]^T
```

The body angular velocity in body coordinates is

```text
omega_b = [
    phi_dot,
    theta_dot cos(phi),
    -theta_dot sin(phi),
]^T
```

The reaction wheel rotates about the body roll axis, so its absolute spin rate about that axis is

```text
phi_dot + psi_r_dot
```

The reduced kinetic energy is

```text
T =
    1/2 M_s s_dot^2
  + H s_dot theta_dot cos(phi) cos(theta)
  - H s_dot phi_dot sin(phi) sin(theta)

  + 1/2 S phi_dot^2
  + 1/2 S theta_dot^2 cos(phi)^2

  + 1/2 I_bx phi_dot^2
  + 1/2 I_by theta_dot^2 cos(phi)^2
  + 1/2 I_bz theta_dot^2 sin(phi)^2

  + 1/2 J_t theta_dot^2
  + 1/2 J_r (phi_dot + psi_r_dot)^2
```

and the nonconstant potential energy is

```text
V = g H cos(theta) cos(phi)
```

The Lagrangian is

```text
L = T - V
```

and the equations follow from

```text
d/dt(dL/dq_dot_i) - dL/dq_i = Q_i
```

## Exact reduced nonlinear terms

The derived inertia matrix is

```text
M(q) =

[ M_s,  H cos(phi) cos(theta), -H sin(phi) sin(theta), 0 ]
[ H cos(phi) cos(theta),
         (S + I_by) cos(phi)^2 + I_bz sin(phi)^2 + J_t, 0, 0 ]
[ -H sin(phi) sin(theta), 0, S + I_bx + J_r, J_r ]
[ 0, 0, J_r, J_r ]
```

The Coriolis / centrifugal vector is

```text
c_1 =
-H [
    phi_dot^2 sin(theta) cos(phi)
  + 2 phi_dot theta_dot sin(phi) cos(theta)
  + theta_dot^2 sin(theta) cos(phi)
]

c_2 =
phi_dot theta_dot (-I_by + I_bz - S) sin(2 phi)

c_3 =
1/2 theta_dot^2 (I_by - I_bz + S) sin(2 phi)

c_4 = 0
```

The gravity vector is

```text
g(q) = [
    0,
    -H g sin(theta) cos(phi),
    -H g cos(theta) sin(phi),
    0,
]^T
```

This model is nonlinear and coupled. In particular, translation, pitch, and roll coupling appears through both the inertia matrix and velocity terms.

## Virtual-work input mapping

For the drive motor,

```text
delta_d = s / r - theta
```

so

```text
dW_drive
    = tau_drive d(delta_d)
    = (tau_drive / r) ds - tau_drive d(theta)
```

and

```text
Q_drive = [
    tau_drive / r,
    -tau_drive,
    0,
    0,
]^T
```

For the reaction wheel, `psi_r` is already the wheel angle relative to the body:

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

Therefore

```text
B = [
    1/r,  0
    -1,   0
     0,   0
     0,   1
]
```

There is intentionally no additional explicit `-tau_reaction` in the roll coordinate. Body reaction torque is already represented by the kinetic-energy coupling through `J_r (phi_dot + psi_r_dot)^2 / 2`.

Electrical polarity, PWM direction, connector identity, and motor-driver sign remain below this physical input boundary.

## Upright linearization

The principal balance point is

```text
theta = 0
phi = 0
s_dot = 0
psi_r_dot = 0
u_ref = 0
```

Define

```text
Delta_pitch = M_s J_theta - H^2
```

Physical consistency of the upright inertia matrix requires

```text
Delta_pitch > 0
```

The upright inertia matrix is

```text
M_0 = [
    M_s, H, 0, 0
    H, J_theta, 0, 0
    0, 0, J_phi + J_r, J_r
    0, 0, J_r, J_r
]
```

and the gravity stiffness is

```text
K_g = [
    0, 0, 0, 0
    0, -H g, 0, 0
    0, 0, -H g, 0
    0, 0, 0, 0
]
```

The negative pitch and roll stiffness terms express the unstable upright equilibrium.

### Pitch / translation block

The first-order equations are

```text
M_s s_ddot + H theta_ddot
    = tau_drive / r

H s_ddot + J_theta theta_ddot - H g theta
    = -tau_drive
```

or

```text
s_ddot =
    -(H^2 g / Delta_pitch) theta
    + ((J_theta / r + H) / Delta_pitch) tau_drive

theta_ddot =
    (H M_s g / Delta_pitch) theta
    - ((H / r + M_s) / Delta_pitch) tau_drive
```

The open-loop pitch unstable modal rate is

```text
lambda_pitch^2 = H M_s g / Delta_pitch
```

with two additional zero poles associated with free forward translation.

### Roll / reaction-wheel momentum block

The first-order equations are

```text
(J_phi + J_r) phi_ddot + J_r psi_r_ddot - H g phi = 0

J_r (phi_ddot + psi_r_ddot) = tau_reaction
```

or

```text
phi_ddot =
    (H g / J_phi) phi
    - tau_reaction / J_phi

psi_r_ddot =
    -(H g / J_phi) phi
    + ((J_phi + J_r) / (J_r J_phi)) tau_reaction
```

The open-loop roll unstable modal rate is

```text
lambda_roll^2 = H g / J_phi
```

and the remaining zero mode is reaction-wheel momentum.

## Structural controllability

For the pitch state

```text
x_pitch = [s, s_dot, theta, theta_dot]^T
```

the controllability determinant is

```text
det(C_pitch)
=
H^2 g^2 (H + M_s r)^2
/
(r^4 Delta_pitch^4)
```

For the roll state

```text
x_roll = [phi, phi_dot, psi_r_dot]^T
```

the controllability determinant is

```text
det(C_roll)
=
H g
/
(J_r J_phi^3)
```

For positive finite physical parameters and `Delta_pitch > 0`, both determinants are nonzero. The seven-state upright balance reduction is therefore generically controllable with the two populated actuators.

This is a derived structural result, not an assumption inherited from the legacy controller.

## Important local result: nonlinear coupling, linear decoupling

The exact reduced nonlinear plant is coupled, but the Jacobian at stationary upright and zero wheel speed separates into pitch/translation and roll/momentum blocks.

That means:

```text
nonlinear plant
    coupled

upright first-order linearization
    pitch/translation block
    +
    roll/reaction-wheel block
```

This is not a reason to restore legacy PID topology. It means the physics itself permits a block-diagonal local linear controller when the objective weights and actuator constraints are also separable.

Cross-axis control becomes valuable when one or more of the following matter:

```text
finite attitude excursions
finite forward speed
yaw / turning
gyroscopic effects
cross-axis actuator constraints
measured model residuals
```

The model, not convention, decides when coupling matters.

## State-space and discretization

For the linear mechanical coordinates,

```text
x_8 = [q_b, q_b_dot]^T
```

the continuous model is

```text
x_dot = A_c x + B_c u
```

with

```text
A_c = [
    0, I
    -M_0^-1 K_g, -M_0^-1 D
]

B_c = [
    0
    M_0^-1 B
]
```

`psi_r` is cyclic, so the control state removes reaction-wheel phase and retains `psi_r_dot`, producing the current seven-state contract.

For the current 500 Hz inner-loop target,

```text
T_s = 0.002 s
```

and zero-order-hold discretization gives

```text
A_d = exp(A_c T_s)

B_d = integral_0^T_s exp(A_c tau) B_c d(tau)
```

so

```text
x[k+1] = A_d x[k] + B_d u[k]
```

No numeric `A_d` or `B_d` is canonical until the required physical parameters are supported by measurement or identification.

## Observability boundary

Controllability can be established from the plant and actuator model alone.

Observability cannot be claimed until the measurement model is explicit:

```text
y = h(x, u, p)
```

The next estimator-facing model must represent what the actual sensors measure, including:

```text
IMU specific force
IMU angular rate
drive encoder relative angle / rate
reaction-wheel encoder relative angle / rate
measurement timing
measurement quality
```

Treating `theta`, `phi`, or wheel speed as directly measured states without a sensor equation would hide the actual estimation problem.

## Symbolic derivation

The symbolic source for the reduced balance equations is:

```text
tools/model/derive_balance_model.py
```

It derives `M(q)`, `c(q,q_dot)`, `g(q)`, the input map, upright matrices, unstable modal rates, and controllability determinants from the current energy and coordinate definitions.

The script contains no numeric plant parameter guesses.

## Yaw and mobility

Yaw remains part of the full robot configuration even though it is not part of the current reduced balance state.

Turning, path-following, finite-speed gyroscopic effects, or a future yaw actuator require the full nonholonomic mobility model rather than ad-hoc correction terms inside the balance controller.

Model reduction is an operating-region decision, not an erasure of physical degrees of freedom.

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

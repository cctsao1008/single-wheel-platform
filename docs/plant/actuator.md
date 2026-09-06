# Actuator Model

The controller produces physical torque demand. The actuator layer owns the inverse mapping from that demand to bounded normalized motor command.

```text
EstimatedState
      |
      v
State-Space Control Law
      |
      v
GeneralizedDemand [N m]
      |
      v
Actuator Model / Inverse Model
      |
      v
BoundedActuatorCommand [-1, 1]
      |
      v
RuntimeAuthority
      |
      v
Board-Specific PWM / Direction
```

The canonical static actuator model is

```text
tau = K_u * effective(command, u_dead)
      - b * omega
      - tau_c * sign_epsilon(omega)
```

where `effective()` removes command dead zone and rescales the remaining range to `[-1, 1]`.

The inverse model solves the command required for a requested physical torque at the current actuator speed. If the required effective command exceeds unit authority, the result is clamped and `saturated=true`; saturation is evidence for runtime authority and LQI integrator hold, not a hidden controller behavior.

The current model deliberately does not invent electrical parameters. Battery dependence, back-EMF structure, current-loop dynamics, command delay, and nonlinear friction may be promoted when identified evidence supports them.

`swp-actuator-model` is the executable `no_std` boundary. `tools/actuator/` owns host-side identification. Accepted physical quantities live in `parameters/reference-assembly.json`.

This separates three meanings that must not collapse:

```text
requested torque        controller semantics
bounded command         actuator semantics
PWM / direction         board electrical semantics
```

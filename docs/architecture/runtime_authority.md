# Runtime Authority and Reaction-Wheel Headroom

## Why this is a first-class concern

The inspected robot is not a generic two-motor platform. Lateral balance uses a reaction wheel. A reaction wheel can only provide sustained corrective torque while it retains angular-momentum headroom.

The supplied product documentation explicitly warns that a sustained disturbance can accelerate the inertia wheel toward maximum speed, after which balancing authority is lost and the unit must be restarted. The software architecture therefore treats reaction-wheel saturation as plant state, not merely as a motor fault.

## Operating-state contract

`swp-runtime-state` defines the semantic operating states independently of RTIC scheduling:

```text
Boot
  -> HardwareCheck
  -> Standby
  -> CaptureWindow
  -> Balancing
       |
       +-> MomentumLimited
       |
       +-> Fault
```

The exact transition thresholds remain commissioning policy. The invariant is already fixed:

```text
Boot / HardwareCheck / Standby / CaptureWindow / Fault
    -> physical actuation denied

Balancing / MomentumLimited
    -> closed-loop actuation may be authorized
```

A future commissioning mode may add a deliberately constrained bench-test authority, but it must be explicit rather than bypassing the state machine.

## Reaction-wheel authority before inertia is known

Exact wheel angular momentum is

```text
H = J * omega
```

but the installed wheel inertia `J` is not yet verified. The runtime must not pretend otherwise.

Wheel speed is nevertheless directly observable after encoder scale is commissioned and is sufficient to expose saturation headroom. `ReactionWheelSpeedLimits` therefore classifies the measured speed as:

```text
Nominal
Warning
Exhausted
```

and computes normalized remaining speed headroom to a configured hard limit.

No product-sheet maximum-speed value is hard-coded as a reference-unit truth. Limits enter as configuration evidence and can later be replaced or augmented by a momentum-domain model once wheel inertia is measured.

## Startup consequence

The supplied operating instructions describe a deliberate capture sequence: power on, wait for initialization, hold the robot close to the equilibrium position, allow automatic control to take over, then release the robot. The re-architecture preserves that physical requirement without preserving the old implementation.

A single `motor_enabled` boolean is therefore insufficient. Startup/capture state, sensor health, actuator authority, and reaction-wheel headroom are separate semantic inputs to physical output authorization.

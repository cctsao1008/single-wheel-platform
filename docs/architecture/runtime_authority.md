# Runtime Authority and Reaction-Wheel Headroom

## Operating states

`swp-runtime-state` defines the robot operating-state model:

```text
Boot
  -> HardwareCheck
  -> Standby
  -> CaptureWindow
  -> Balancing
       |-> MomentumLimited
       |-> Fault
```

Physical actuator output is denied in:

```text
Boot
HardwareCheck
Standby
CaptureWindow
Fault
```

Closed-loop actuator authority exists only in:

```text
Balancing
MomentumLimited
```

Runtime state is independent of RTIC task scheduling and independent of raw GPIO/timer ownership.

## Reaction-wheel authority

Reaction-wheel control authority depends on remaining angular-momentum headroom:

```text
H = J * omega
```

The runtime represents available headroom with `ReactionWheelSpeedLimits` until a configured inertia model is present.

The speed-domain authority classes are:

```text
Nominal
Warning
Exhausted
```

A hard speed limit defines zero remaining speed headroom. A warning limit defines the transition into constrained authority.

## Authorization boundary

Actuator authorization combines:

```text
operating state
sensor / state validity
actuator limits
reaction-wheel headroom
fault state
```

Only an authorized actuator demand may reach electrical mapping and physical PWM/direction resources.

A generic `motor_enabled` flag is not an actuator-authority model.

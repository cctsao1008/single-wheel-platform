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

Closed-loop actuator authority is possible only in:

```text
Balancing
MomentumLimited
```

Runtime state is independent of RTIC task scheduling and independent of raw GPIO/timer ownership.

## Primary sensor timing authority

The primary balance observation clock is the MPU6050 DATA_RDY stream. Its timing health is a separate authority condition:

```text
Startup
Healthy
Late
Timeout
```

Only `Healthy` is eligible for closed-loop actuation. `Startup`, `Late`, and `Timeout` deny physical output authority even when the operating state is `Balancing` or `MomentumLimited`.

Timing health is supervised by an MCU timebase independent of DATA_RDY. A missing sensor interrupt must therefore revoke authority instead of silently stopping the control path.

The current runtime timing policy is:

```text
nominal DATA_RDY period   2 ms
late                     >= 3 ms
hard timeout             >= 6 ms
```

One complete inter-event interval is required before timing becomes `Healthy`. These thresholds are runtime safety policy, not MPU6050 device specifications.

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
primary sensor timing health
measurement / estimated-state validity
control deadline validity
actuator limits
reaction-wheel headroom
fault state
```

Only an authorized actuator demand may reach electrical mapping and physical PWM/direction resources.

A generic `motor_enabled` flag is not an actuator-authority model.

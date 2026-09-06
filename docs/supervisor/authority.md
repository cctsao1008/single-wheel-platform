# Runtime Authority and Reaction-Wheel Headroom

`RuntimeAuthority` is the only semantic boundary allowed to promote a bounded actuator request into a physical-output-authorized token.

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
ActuatorPairCommand
      |
      v
RuntimeAuthority
      |
      +-- denied ------> no physical-output token
      |
      +-- authorized --> AuthorizedActuation
                              |
                              v
                    board electrical mapping
                              |
                              v
                         PWM / direction
```

`AuthorizedActuation` has no public constructor. Downstream electrical-output code must accept that token rather than a raw normalized command. Physical-output authority is therefore a type-level ownership rule rather than a convention.

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

The control runtime never catches up missed periods. One DATA_RDY event creates at most one corresponding control opportunity. A late or timed-out cadence is handled as lost authority, not by executing backlog estimator/controller iterations back-to-back.

## Estimated-state authority

A healthy sensor clock is not sufficient by itself. Closed-loop physical output also requires `StateValidity::Valid`. Missing required measurements, invalid timing, non-finite input, or an estimator numerical fault therefore prevents `AuthorizedActuation` from being created.

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

`Warning` keeps bounded closed-loop output available but marks the decision constrained and requires the LQI integrator to be held. `Exhausted` revokes physical-output authority.

A hard speed limit defines zero remaining speed headroom. A warning limit defines the transition into constrained authority.

## Actuator saturation

The actuator model may bound a requested torque because the inverse model requires more than normalized unit command. Saturation does not disappear inside the actuator layer.

```text
requested torque
      |
      v
inverse actuator model
      |
      +-- within authority --> bounded command
      |
      +-- outside authority -> bounded command + saturated=true
```

A saturated bounded command can remain physically authorized when all hard authority conditions are healthy, but the authority decision is `constrained=true` and `hold_integrator=true`. This prevents LQI accumulation from treating unavailable actuator authority as though it were applied torque.

Drive and reaction-wheel saturation remain separate decision reasons.

## Authorization evidence

`RuntimeAuthority::evaluate()` consumes:

```text
operating state
primary-sensor timing health
estimated-state validity
reaction-wheel speed authority
drive actuator saturation
reaction-wheel actuator saturation
```

Hard denial conditions are:

```text
operating state not closed-loop eligible
sensor timing not Healthy
estimated state Invalid
reaction-wheel authority Exhausted
```

Constrained-but-authorized conditions are:

```text
reaction-wheel Warning
drive actuator saturated
reaction-wheel actuator saturated
```

Every decision carries `AuthorityReasons`, `constrained`, and `hold_integrator`. Requested and applied meanings therefore remain distinct and diagnosable.

A generic `motor_enabled` flag is not an actuator-authority model.

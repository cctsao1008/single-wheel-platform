# System Architecture

## Purpose

The Self-Balancing Single-Wheel Platform separates physical I/O, measured state, control policy, actuator qualification, and hardware authority into explicit boundaries.

The architecture is designed around three physically different actuation roles:

- **roll / lateral balance** through a reaction wheel,
- **pitch / longitudinal balance** through the ground-contact drive wheel,
- **yaw / spin** through a third actuator path.

## Target data flow

```text
Physical Sensors
    -> Sensor Acquisition
    -> State Estimation
    -> State Validation
    -> Control Pipeline
       -> Roll Controller
       -> Pitch Controller
       -> Yaw Controller
    -> Actuator Mapper
    -> Output Safety
    -> Motor Authority
    -> Board Motor API
    -> Physical Actuators
```

Non-critical services run outside this path:

```text
telemetry
UART / Bluetooth
OLED / UI
parameter management
persistent storage
maintenance commands
analysis log transfer
```

## Module ownership

### Sensor acquisition

Owns timestamped raw measurements only. It must not contain balancing policy.

### State estimation

Converts raw inertial and encoder measurements into physical state variables. Estimator algorithms must remain replaceable behind a stable state interface.

### State validation

Determines whether the current state is usable by automatic control. Invalid or stale state must not preserve an old actuator request indefinitely.

### Controllers

Controllers compute control effort in physical/control-domain terms. They do not know GPIO, timer, connector, or PWM-register details.

The initial controller decomposition is compatible with:

```text
roll  = attitude PD + reaction-wheel speed PI
pitch = attitude PD + drive-wheel speed PI
yaw   = independent spin command / future closed-loop policy
```

Future state-space or coupled controllers may replace this policy behind the same architecture.

### Actuator mapper

Converts abstract control effort into logical actuator-domain commands. It owns control sign, normalization, dead-zone compensation, and actuator-domain scaling.

### Output safety

Applies final command-domain limits such as saturation, slew rate, operating envelopes, reversal constraints, and fault-forced safe command.

### Motor authority

Owns which software path may physically command an actuator. Maintenance and automatic control must not independently write the same motor hardware.

### Platform implementation

Owns MCU peripherals, pins, timers, bus drivers, PWM electrical behavior, storage, startup, and safe-off behavior.

## Dependency rule

```text
control/
   ^
   |
 app/
   ^
   |
platform/api/
   ^
   |
platform/stm32f103/
```

`control/` must remain buildable without STM32-specific headers.

## Physical units

Control-domain interfaces should use explicit units wherever practical:

- angle: radians,
- angular rate: rad/s,
- wheel speed: rad/s,
- voltage: volts,
- time: microseconds or seconds as declared by the API,
- normalized actuator request: `[-1.0, +1.0]`.

## Controller evolution

The architecture must allow the following without a board-layer rewrite:

- complementary filter or alternative estimator,
- PD/PI baseline control,
- state-space control,
- LQR / LQI,
- observers / Kalman filtering,
- coupling compensation,
- system identification and model comparison.

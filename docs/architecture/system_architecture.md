# System Architecture

## Purpose

The Self-Balancing Single-Wheel Platform is a re-architecture of the embedded control software around the existing physical system.

The architecture separates device access, measurement, coordinate mapping, state estimation, control computation, actuator translation, and hardware ownership into explicit boundaries. The intent is to keep the critical control path understandable and deterministic while preventing control logic from depending directly on MCU-specific implementation details.

The reference platform contains three physical actuation paths:

- **roll / lateral balance** through a reaction wheel,
- **pitch / longitudinal balance** through the ground-contact drive wheel,
- **yaw / spin** through a third actuator path.

The third path is represented explicitly, but its effective control authority and coupling are properties of the physical plant and are not assumed by the software architecture.

## Primary control path

```text
Physical Sensors / Encoders
          |
          v
   Platform Bus / I/O
          |
          v
     Device Drivers
          |
          v
  Sensor Acquisition
          |
          v
 Coordinate Transform
          |
          v
   State Estimation
          |
          v
   State Validation
          |
          v
     Control Policy
          |
          v
   Actuator Mapper
          |
          v
 Output Protection
          |
          v
   Motor Authority
          |
          v
  Platform Motor I/O
          |
          v
  Physical Actuators
```

Non-critical services remain outside this path:

```text
telemetry / trace
UART / Bluetooth
OLED / UI
configuration
persistent storage
maintenance commands
log transfer / offline analysis
```

## Module ownership

### Platform API

Defines board-level hardware services such as I2C transactions, timestamps, GPIO interrupt delivery, encoder access, ADC access, serial transport, storage, and motor output.

The API describes behavior and units; it does not expose STM32 register details and it does not depend on `control/` or device-specific types.

### Platform implementation

`platform/stm32f103/` owns the STM32F103-specific realization of the platform contracts:

- peripheral initialization,
- pins and alternate functions,
- timers and PWM generation,
- interrupt wiring,
- I2C / UART / ADC implementation,
- startup and shutdown behavior,
- electrical safe-off behavior.

### Device drivers

`drivers/` owns device-specific protocol and register behavior without robot control policy.

The reference MPU6050 driver owns sensor register configuration, full-scale settings, DLPF selection, sample-rate configuration, raw-data conversion primitives, and device probing. It receives bus, delay, and timestamp functions through injected transport callbacks.

The application binds those callbacks to `platform/api/` services. This keeps the driver portable while also keeping `platform/api/` free of MPU6050-specific types.

A device driver must not depend on `platform/stm32f103/` directly.

### Sensor acquisition

Owns coherent, timestamped measurements presented to the control domain. Acquisition decides when a sample is accepted and how freshness is represented; it does not contain balancing policy.

### Coordinate transform

Maps physical sensor and encoder axes into the robot coordinate convention. Mounting orientation, encoder polarity, actuator direction, and roll/pitch/yaw sign conventions are explicit configuration or hardware-mapping data rather than implicit assumptions in controller code.

### State estimation

Converts measurements into the state required by the active control policy. Estimation is replaceable behind a stable state interface and does not command actuators.

### State validation

Determines whether the state is sufficiently fresh and valid for automatic control. Invalid or stale state must not allow an old actuator request to remain active indefinitely.

### Control policy

Consumes robot state and produces requested control effort in control-domain terms.

The control-policy interface does not assume a specific control law. Controller implementation and controller synthesis method are separate concerns; changing a controller must not require changes to board I/O, sensor drivers, or motor peripherals.

### Actuator mapper

Translates requested control effort into actuator-domain commands. It owns the mapping between control axes and physical actuators together with sign, normalization, scaling, and actuator-specific command conventions.

Plant-dependent compensation belongs here only when it is part of command translation rather than control-policy state feedback.

### Output protection

Applies command-domain limits required before physical actuation, including saturation, slew limits, reversal constraints, operating envelopes, and fault-forced safe commands.

Reaction-wheel speed and momentum limits, motor torque/current capability, and other finite actuator constraints must remain visible system properties rather than being hidden inside peripheral code.

### Motor authority

Owns which software path may command each physical actuator. Automatic control, maintenance functions, commissioning tools, and fault handling must not independently write the same motor hardware.

### Application layer

`app/` owns system orchestration and is the binding point between portable modules and the selected platform:

- startup sequencing,
- driver transport binding,
- module initialization,
- runtime mode selection,
- control-loop scheduling,
- background-service coordination,
- transitions between disabled, commissioning, maintenance, and automatic-control states.

It coordinates modules but does not absorb device-driver or controller implementation details.

## Dependency rules

The intended compile-time dependency direction is:

```text
app/ ---------------------> control/
  |
  +------------------------> drivers/
  |
  +------------------------> platform/api/

platform/stm32f103/ -------> platform/api/

control/  -----------------> no platform-specific dependency
drivers/  -----------------> no MCU-specific dependency
platform/api/ -------------> no control/device-specific dependency
```

The application supplies platform services to portable device drivers through explicit bindings rather than by making device drivers include STM32 implementation headers.

Additional rules:

- `control/` must build without STM32 headers or MCU register definitions.
- `drivers/` must build without STM32-specific implementation headers.
- `platform/api/` must not depend on `control/` or device-driver types.
- `platform/stm32f103/` implements hardware contracts; it does not own balancing policy.
- `app/` is the integration point between portable control logic, device drivers, and the selected platform implementation.

## Timing semantics

Timing is part of the interface contract.

- Sensor samples are timestamped at a defined acquisition point.
- Estimation uses declared sample timing rather than silently assuming a fixed period.
- Control-loop rate, jitter, and worst-case execution time are measured properties of the implementation.
- Blocking telemetry, formatted output, display rendering, Flash operations, and long protocol processing remain outside the critical control path.

Detailed timing rules are maintained in [`timing_architecture.md`](timing_architecture.md).

## Physical units

Control-domain interfaces use explicit units wherever practical:

- angle: radians,
- angular rate: rad/s,
- wheel speed: rad/s,
- voltage: volts,
- current: amperes when available,
- time: microseconds or seconds as declared by the API,
- normalized actuator request: `[-1.0, +1.0]` only where a normalized interface is intentional.

Raw register units and ADC counts remain inside the appropriate driver or acquisition boundary.

## Hardware mapping

Software names must follow confirmed physical roles rather than historical channel names.

The mapping between board connectors, motor channels, encoder channels, IMU axes, and robot axes is maintained in `docs/hardware/` and the platform configuration. Controller code must not infer physical meaning from names such as `motor1`, `motor2`, `x`, or `y`.

## Extension policy

The architecture allows estimation and control implementations to change without restructuring the hardware-facing layers. Modeling, system identification, replay, and more advanced control methods may be added as engineering capabilities, but they are not required to define the platform architecture.

The repository remains specific to the self-balancing single-wheel system; abstractions are introduced only when they clarify a real boundary in this physical platform.

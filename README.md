# Self-Balancing Single-Wheel Platform

A re-architected embedded control platform for a self-balancing single-wheel robot, with clear separation of sensing, estimation, control, actuation, and hardware-specific implementation.

The project reorganizes the existing platform into explicit software boundaries so that control and estimation logic can evolve without being coupled directly to MCU-specific code. Modeling, system identification, and advanced control are supported as future capabilities, but the primary goal is the platform architecture itself.

## Physical system

The reference platform combines three physical actuation paths:

- **Roll / lateral balance** — reaction-wheel actuation.
- **Pitch / longitudinal balance** — ground-contact drive-wheel actuation.
- **Yaw / spin** — a third actuation path whose control authority and coupling are treated as plant properties to characterize.

State feedback is provided by an MPU6050-class IMU and wheel encoders on the reference hardware.

## Architecture

```text
Physical Sensors / Encoders
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
  Control Policy
          |
          v
   Actuator Mapper
          |
          v
 Safety / Authority
          |
          v
   Platform I/O
          |
          v
 Physical Actuators
```

Non-critical services such as telemetry, storage, user interfaces, configuration, and maintenance traffic remain outside the critical control path.

## Design goals

- Separate device drivers from MCU- and board-specific implementation.
- Keep control and estimation logic independent of hardware registers and SDK headers.
- Make coordinate conventions, units, timing, and actuator directions explicit.
- Keep the control path deterministic and free from blocking non-critical work.
- Represent controller output as requested physical effort rather than direct motor access.
- Allow estimation and control methods to be replaced without restructuring the platform.
- Keep the architecture tied to the physical single-wheel system rather than growing into a generic robotics framework.

## Control perspective

The architecture does not assume a particular control law.

A controller consumes an estimated robot state and produces an actuator request through a defined interface. Linear state feedback, model-based methods, robust control, constrained control, or nonlinear methods can therefore be introduced without changing the hardware-facing layers.

Reaction-wheel momentum, actuator speed, torque, saturation, sensing delay, and real-time execution limits are treated as physical constraints of the platform rather than hidden implementation details.

## Reference platform

The current reference hardware is based on:

- STM32F103C8T6-class MCU
- MPU6050-class inertial sensing
- reaction-wheel actuator
- ground-contact drive-wheel actuator
- additional spin actuator
- encoder feedback
- battery and analog monitoring
- UART / serial communication and local display interfaces

Exact board mappings, polarities, timer assignments, and coordinate transforms are maintained in the platform and hardware documentation.

## Repository layout

```text
app/                 System orchestration and services
control/             Platform-independent estimation and control
  estimation/        State-estimation algorithms
  controllers/       Control-policy implementations
  safety/            State and output protection

drivers/             Device-level drivers independent of robot policy
platform/
  api/                Board and platform contracts
  stm32f103/          STM32F103 reference-platform implementation
telemetry/            Runtime telemetry and trace infrastructure
tests/                Host-side control and interface tests
tools/                Analysis, replay, plotting, and system tools
docs/
  architecture/       System, control, timing, and interface architecture
  hardware/           Hardware baseline and mapping
  commissioning/      Sensor and actuator bring-up notes
```

## Architectural rules

1. `control/` must not depend on STM32-specific registers or SDK headers.
2. Hardware pins, timers, channels, and polarities belong to the platform layer.
3. Coordinate conventions and physical units are explicit contracts.
4. Controller output is a requested control effort, not direct motor access.
5. Telemetry, storage, UI, and maintenance traffic remain outside the critical control path.
6. The control-loop rate is a measured system property, not a fixed assumption inherited from another implementation.

See [`docs/architecture/system_architecture.md`](docs/architecture/system_architecture.md) for the detailed architecture.

## Status

The repository currently establishes the platform architecture, interface boundaries, and reference-hardware structure. Device integration, hardware mapping, runtime scheduling, and control implementations are being built on top of these boundaries.

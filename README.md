# Self-Balancing Single-Wheel Platform

Embedded control and system-identification platform for a self-balancing single-wheel robot, covering state estimation, balance control, actuation, and safety.

The project separates **physical I/O**, **state estimation**, **control computation**, and **actuator authority** so that controller and estimator research can evolve without coupling directly to MCU-specific code.

## System view

```text
Sensors / Encoders / ADC
          |
          v
  Sensor Acquisition
          |
          v
   State Estimation
          |
          v
 State Validation
          |
          v
  Control Pipeline
   /      |      \
 Roll   Pitch    Yaw
   \      |      /
          v
   Actuator Mapper
          |
          v
    Output Safety
          |
          v
   Motor Authority
          |
          v
     Board I/O
          |
          v
  Physical Actuators
```

## Physical control concept

- **Roll / lateral balance** — reaction-wheel actuation.
- **Pitch / longitudinal balance** — ground-contact drive-wheel actuation.
- **Yaw / spin** — independent spin-actuator path.
- **Attitude sensing** — MPU6050-class inertial sensing on the reference platform.
- **Wheel feedback** — encoder-based speed / motion feedback.

The current control baseline is compatible with an attitude **PD** plus wheel-speed **PI** decomposition while keeping the architecture open to state-space control, LQR/LQI, observers, Kalman filtering, coupling compensation, and system-identification work.

## Repository layout

```text
app/                 System orchestration and services
control/             Platform-independent estimation and control
  estimation/        State-estimation algorithms
  controllers/       Roll / pitch / yaw and future controllers
  safety/            State and output safety

drivers/             Device-level drivers independent of robot policy
platform/
  api/                Shared board contracts
  stm32f103/          STM32F103 reference-platform implementation
telemetry/            Runtime telemetry and trace infrastructure
tests/                Host-side control and interface tests
tools/                Analysis, replay, plotting, and system-ID tools
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
6. The control-loop rate is a measured system property, not a fixed assumption inherited from another platform.

See [`docs/architecture/system_architecture.md`](docs/architecture/system_architecture.md) for the target architecture.

## Status

The repository currently defines the target architecture and interface boundaries for the platform. Hardware mapping and control parameters are promoted to confirmed project facts only after they are tied to the actual reference hardware.

# Self-Balancing Single-Wheel Platform

A Rust `no_std` embedded control platform for a two-axis self-balancing single-wheel robot.

The repository is structured around explicit physical and semantic boundaries: board wiring, installed hardware, observation, calibration, coordinate mapping, state estimation, control, actuator authority, and electrical output are separate parts of the system.

## Core Architecture

```text
Physical Plant
      |
      v
RawObservation
      |
      | device transfer functions
      v
ScaledObservation
      |
      | measured calibration
      v
CalibratedObservation
      |
      | sensor -> body transform
      v
BodyObservation
      |
      v
EstimatedState
      |
      v
GeneralizedDemand
      |
      v
ActuatorAllocation
      |
      v
RuntimeAuthority
      |
      v
ElectricalOutput
      |
      v
Physical Actuators
```

Observation and control remain independent from transport and user interfaces:

```text
RawObservation ───────> Recorder / Replay
System State ─────────> OLED status
Commissioning Command <──── BLE / USART2
```

The control path does not depend on telemetry, Bluetooth, OLED, host tools, or storage.

## Platform

```text
MCU             STM32F103C8T6
IMU             MPU6050

Reaction wheel  BLDC_1 / Encoder_1
Drive wheel     BLDC_2 / Encoder_2
Third channel   BLDC_3, unused by the reference assembly

USART1          wired recording / engineering interface
USART2          ECB02S2 BLE commissioning interface
OLED            PB4/PB5 local status interface
```

The robot body frame is right-handed:

```text
+X = forward
+Y = left
+Z = up
```

Board channels and robot roles are intentionally separate concepts:

```text
board-one-v2
    PCB pins, timers, connectors, buses

reference-assembly
    installed hardware and channel-to-role mapping

robot-domain
    reaction wheel, drive wheel, body state, control demand
```

## Runtime

The target runtime uses:

```text
Rust no_std
embedded-hal 1.0
stm32f1xx-hal
RTIC 2.x
```

RTIC owns interrupt priority and static peripheral/resource ownership. High-priority acquisition and control work is isolated from lower-priority recording and interface traffic.

The operating-state model is:

```text
Boot
  |
  v
HardwareCheck
  |
  v
Standby
  |
  v
CaptureWindow
  |
  v
Balancing
  |
  +------> MomentumLimited
  |
  +------> Fault
```

Actuator output is permitted only through runtime authority. Reaction-wheel speed is part of actuator authority because balance authority decreases as wheel momentum headroom is exhausted.

## Observation Model

Sensor acquisition preserves timing and measurement quality instead of collapsing all inputs into one synthetic sample instant.

```text
IMU
  raw accel / gyro / temperature
  source-time evidence
  read start / completion time
  measurement quality

Encoders
  raw quadrature counters
  individual capture time
  measurement quality

Battery
  raw ADC value
  read timing
  measurement quality
```

Scaling, physical calibration, and frame mapping are separate semantic transitions.

```text
raw register value
      !=
physical unit
      !=
calibrated sensor value
      !=
body-frame measurement
      !=
estimated robot state
```

## Recording and Replay

`RawObservation` can be encoded as a fixed-size CRC-protected record and replayed independently of the firmware transport implementation.

```text
RawObservation
      |
      v
RecordedObservation
      |
      +----> USART1
      |
      +----> host recording
                 |
                 v
               replay
```

UART is a transport, not the system data model.

## Repository Structure

```text
crates/
  robot-domain/          Robot state and actuator-domain types
  reference-assembly/    Installed hardware and board-to-role mapping
  plant-observation/     Raw acquisition, timing, and quality
  sensor-calibration/    Sensor scaling and measured calibration
  frame-transform/       Sensor-frame to body-frame mapping
  runtime-state/         Operating state and actuator authority
  observation-record/    Binary recording / replay contract
  mpu6050/               MPU6050 device driver
  software-i2c/          embedded-hal software I2C
  board-one-v2/          PCB wiring and peripheral mapping

firmware/
  stm32f103/             STM32F103 RTIC target runtime

tools/
  recording/             Decode and deterministic replay tools

docs/
  architecture/          System contracts and semantic boundaries
  hardware/              Board, assembly, and pin mapping
  commissioning/         Physical bring-up and characterization
```

## Build

Install a current stable Rust toolchain with the `thumbv7m-none-eabi` target.

```bash
cargo fw
```

CI checks formatting, Cortex-M workspace compilation, Clippy, host-side unit tests, protocol/replay tests, and the release firmware link.

## Documentation

- [`docs/architecture/system_architecture.md`](docs/architecture/system_architecture.md)
- [`docs/architecture/body_frame_contract.md`](docs/architecture/body_frame_contract.md)
- [`docs/architecture/runtime_authority.md`](docs/architecture/runtime_authority.md)
- [`docs/architecture/calibration_contract.md`](docs/architecture/calibration_contract.md)
- [`docs/architecture/observation_time_health_replay.md`](docs/architecture/observation_time_health_replay.md)
- [`docs/hardware/hardware_baseline.md`](docs/hardware/hardware_baseline.md)
- [`docs/hardware/pin_mapping.md`](docs/hardware/pin_mapping.md)

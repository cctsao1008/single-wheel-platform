# Self-Balancing Single-Wheel Platform

A clean re-architecture of the embedded software for a self-balancing single-wheel robot.

The repository is organized around physical truth and semantic transitions rather than inherited firmware structure. Device access, board wiring, acquisition evidence, calibration, robot-domain state, real-time scheduling, estimation, control, and actuation are separate concerns.

The current implementation uses **Rust `no_std`**, **embedded-hal 1.0**, **stm32f1xx-hal**, and **RTIC** on the STM32F103 reference target. The earlier C/CMake scaffold is not retained as a compatibility layer.

## Architectural rule

Data may acquire richer meaning only when the evidence required for that meaning exists.

```text
physical hardware
      |
      v
RawObservation + timing/quality evidence
      |
      v
CalibratedObservation
      |
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
Authority / limits
      |
      v
ElectricalOutput
```

A raw ADC count is not a voltage until its transfer function is known. A PCB encoder channel is not a reaction-wheel speed until mechanical mapping, sign, and scale are established. An I2C read that succeeded is not automatically a fresh, exactly timed IMU sample.

## Time and measurement quality

The runtime does not collapse sequential hardware reads into one fictional timestamp. `RawObservation` records acquisition start/completion, MPU read start/completion, encoder capture times, ADC read completion, per-measurement quality, and platform acquisition status.

The MPU6050 source-sample timestamp is explicitly **unknown** on the reference board because the actual MPU data-ready interrupt is not routed. Later estimation must use measurement timing evidence rather than assuming that the scheduler period is identical to physical sample time.

## Recording and deterministic replay

Recording/replay is a first-class data path:

```text
RawObservation
   |\
   | +------------------> future estimator/control path
   |
   +--> RecordedObservation --> binary log --> replay
```

`swp-observation-record` owns the deterministic binary record format. UART is only the current transport; transport is not the owner of acquisition semantics. Host tools can decode the same record stream to CSV or replay it without using host wall-clock timing.

## Repository layout

```text
crates/
  robot-domain/          Single-wheel physical/control-domain types
  plant-observation/     Raw evidence, timing, and measurement quality
  observation-record/   Deterministic binary record/replay contract
  mpu6050/               Portable embedded-hal MPU6050 driver
  software-i2c/          Portable open-drain embedded-hal I2C
  board-one-v2/          Reference-board wiring facts only

firmware/
  stm32f103/             no_std RTIC runtime and STM32 peripheral ownership

tools/
  recording/             Record decode and deterministic replay source

docs/
  architecture/          Semantic, timing, replay, and authority contracts
  hardware/              Schematic review and board mapping
  commissioning/         Bring-up and characterization notes
```

## Board/robot separation

The board crate describes PCB identities such as `BLDC_1`, `BLDC_2`, `BLDC_3`, `Encoder_1`, and `Encoder_2`. It does not promote those channels into reaction-wheel, drive-wheel, or spin semantics until the physical harness mapping is established.

Important reviewed hardware facts include:

- STM32F103C8T6 reference MCU.
- MPU6050 address `0x68`.
- MPU SDA/SCL are PB8/PB9 as drawn, requiring software I2C for this PCB wiring.
- PC13 net `MPU_INT` reaches MPU6050 **FSYNC**, while the actual MPU6050 INT pin is not routed.
- TIM2/TIM4 provide raw quadrature counts for Encoder 1/2; Encoder 3 has no MCU route shown.
- PA5 / ADC1_IN5 is the raw battery-divider node; divider scaling remains unconfirmed.
- The reviewed schematic does not label the external crystal frequency.

## Current executable path

```text
HSI 8 MHz
  -> MPU6050 software-I2C acquisition
  -> TIM2/TIM4 encoder snapshots
  -> ADC1/PA5 raw battery read
  -> explicit per-source timing and quality evidence
  -> RawObservation
  -> 80-byte CRC-protected RecordedObservation
  -> lock-free SPSC record queue
  -> lower-priority USART1 TXE record transport
  -> host recording / replay tools
```

The acquisition task is currently scheduled at 100 Hz. That period is a scheduler intent, not a claim that every physical sensor sample occurred exactly 10 ms apart.

No TIM3 motor PWM, direction, or brake output is configured. Motor activation remains outside the current passive observation cut.

## Build

Install a current stable Rust toolchain with the `thumbv7m-none-eabi` target. `cargo fw` links the STM32F103 release firmware. CI enforces formatting, full Cortex-M workspace check, Clippy with warnings denied, observation-record host tests, Python recording decoder tests, and the final release firmware link.

See [`docs/architecture/system_architecture.md`](docs/architecture/system_architecture.md), [`docs/architecture/typed_dataflow.md`](docs/architecture/typed_dataflow.md), and [`docs/architecture/observation_time_health_replay.md`](docs/architecture/observation_time_health_replay.md).

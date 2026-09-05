# Self-Balancing Single-Wheel Platform

A clean re-architecture of the embedded software for a self-balancing single-wheel robot.

The repository is organized around physical truth and semantic transitions rather than inherited firmware structure. Device access, board wiring, physical assembly, acquisition evidence, device scaling, measured calibration, robot-domain state, real-time scheduling, estimation, control, and actuation are separate concerns.

The current implementation uses **Rust `no_std`**, **embedded-hal 1.0**, **stm32f1xx-hal**, and **RTIC** on the STM32F103 reference target. The earlier C/CMake scaffold is not retained as a compatibility layer.

## Architectural rule

Data may acquire richer meaning only when the evidence required for that meaning exists.

```text
physical hardware
      |
      v
RawObservation + timing/quality evidence
      |
      | device transfer functions
      v
ScaledSensorObservation
      |
      | measured calibration
      v
CalibratedObservation
      |
      | mechanical/frame mapping
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

A raw ADC count is not a voltage until its transfer function is known. A datasheet transfer function is not a physical calibration. A PCB encoder channel is not a reaction-wheel speed until mechanical mapping, sign, and scale are established. An I2C read that succeeded is not automatically a fresh, exactly timed IMU sample.

## Time and measurement quality

The runtime does not collapse sequential hardware reads into one fictional timestamp. `RawObservation` records acquisition start/completion, MPU read start/completion, encoder capture times, ADC read completion, per-measurement quality, and platform acquisition status.

The MPU6050 source-sample timestamp is explicitly **unknown** on the reference board because the actual MPU data-ready interrupt is not routed. Later estimation must use measurement timing evidence rather than assuming that the scheduler period is identical to physical sample time.

## Scaling and calibration

The MPU6050 device crate owns nominal transfer functions for the configured full-scale ranges and temperature conversion. These functions convert register counts into SI units in the native sensor frame.

`swp-sensor-calibration` is a separate semantic layer. It applies measured three-axis affine correction and carries explicit calibration evidence/revision. Mechanical mounting orientation is not encoded in calibration parameters; frame mapping remains a later transition into `BodyObservation`.

The current target firmware intentionally continues to record raw evidence because no verified physical calibration profile has yet been established for the unit. Zero bias and an identity matrix are not treated as a production calibration merely to make the pipeline executable.

## Verified physical actuator topology

Physical inspection and manual cable tracing establish the current reference assembly as a **two-actuator plant**:

```text
PCB silk M2 / schematic BLDC_1 -> upper reaction-wheel motor
PCB silk M1 / schematic BLDC_2 -> lower ground-drive motor
PCB silk M3 / schematic BLDC_3 -> no motor installed
```

The corresponding robot-domain actuator roles are therefore:

```text
ReactionWheel
DriveWheel
```

The unpopulated third motor interface remains a board capability, not a robot actuator. `swp-board-one-v2` continues to describe only PCB wiring; `swp-reference-assembly` owns the verified transition from board channels to installed robot roles.

Encoder association follows the installed harness: Encoder 1 belongs to the reaction-wheel motor path and Encoder 2 to the drive-wheel motor path. Encoder sign and mechanical scale remain unverified commissioning facts.

## Recording and deterministic replay

Recording/replay is a first-class data path:

```text
RawObservation
   |\
   | +------------------> scaling / calibration / future estimator path
   |
   +--> RecordedObservation --> binary log --> replay
```

`swp-observation-record` owns the deterministic binary record format. UART is only the current transport; transport is not the owner of acquisition semantics. Host tools can decode the same record stream to CSV or replay it without using host wall-clock timing.

## Repository layout

```text
crates/
  robot-domain/          Verified single-wheel physical/control-domain types
  reference-assembly/    Installed actuator population and board-to-role mapping
  plant-observation/     Raw evidence, timing, and measurement quality
  sensor-calibration/    Device scaling boundary and measured IMU correction
  observation-record/   Deterministic binary record/replay contract
  mpu6050/               Portable driver and nominal transfer functions
  software-i2c/          Portable open-drain embedded-hal I2C
  board-one-v2/          Reference-board wiring facts only

firmware/
  stm32f103/             no_std RTIC runtime and STM32 peripheral ownership

tools/
  recording/             Record decode and deterministic replay source

docs/
  architecture/          Semantic, timing, replay, calibration, authority contracts
  hardware/              Schematic review, assembly observation, and board mapping
  commissioning/         Bring-up and characterization notes
```

## Board / assembly / robot separation

The project now treats these as three distinct facts:

```text
Board capability
    BLDC_1 / BLDC_2 / BLDC_3 exist

Assembly population
    BLDC_1 and BLDC_2 installed
    BLDC_3 unpopulated

Robot semantics
    BLDC_1 -> ReactionWheel
    BLDC_2 -> DriveWheel
```

Important reviewed hardware facts include:

- STM32F103C8T6 reference MCU from the schematic; physical top marking still needs direct visual confirmation.
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

No TIM3 motor PWM or direction output is configured. Motor activation remains outside the current passive observation cut.

## Build

Install a current stable Rust toolchain with the `thumbv7m-none-eabi` target. `cargo fw` links the STM32F103 release firmware. CI enforces formatting, full Cortex-M workspace check, Clippy with warnings denied, MPU transfer-function tests, sensor-calibration tests, observation-record host tests, Python recording decoder tests, and the final release firmware link.

See [`docs/architecture/system_architecture.md`](docs/architecture/system_architecture.md), [`docs/architecture/typed_dataflow.md`](docs/architecture/typed_dataflow.md), [`docs/architecture/calibration_contract.md`](docs/architecture/calibration_contract.md), [`docs/architecture/observation_time_health_replay.md`](docs/architecture/observation_time_health_replay.md), and [`docs/hardware/assembly_observation_2026-09-05.md`](docs/hardware/assembly_observation_2026-09-05.md).

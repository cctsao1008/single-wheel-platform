# Self-Balancing Single-Wheel Platform

A clean re-architecture of the embedded software for a self-balancing single-wheel robot.

The repository is organized around physical truth and semantic transitions rather than inherited firmware structure. Device access, board wiring, physical assembly, acquisition evidence, device scaling, measured calibration, frame mapping, robot-domain state, operating authority, real-time scheduling, estimation, control, and actuation are separate concerns.

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
      | evidenced mechanical/frame mapping
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
Runtime Authority / limits
      |
      v
ElectricalOutput
```

A raw ADC count is not a voltage until its transfer function is known. A datasheet transfer function is not a physical calibration. A PCB encoder channel is not a reaction-wheel speed until mechanical mapping, sign, and scale are established. An I2C read that succeeded is not automatically a fresh, exactly timed IMU sample. A Bluetooth command is not physical actuator authority.

## Canonical body frame

The project now defines one explicit right-handed body frame:

```text
+X = forward, along the ground-drive direction
+Y = left
+Z = up
```

`swp-frame-transform` is the only semantic boundary that promotes calibrated sensor-frame IMU data into this body frame. It accepts proper 3-D rotations and carries explicit frame evidence. No default reference-unit MPU rotation is published yet because the remaining sensor X/Y signs still require a physical tilt/rotation test.

Legacy behavior is consistent with MPU sensor +Z being approximately body-up at equilibrium, but legacy comments are not treated as sufficient evidence for the remaining signs.

## Time and measurement quality

The runtime does not collapse sequential hardware reads into one fictional timestamp. `RawObservation` records acquisition start/completion, MPU read start/completion, encoder capture times, ADC read completion, per-measurement quality, and platform acquisition status.

The MPU6050 source-sample timestamp is explicitly **unknown** on the reference board because the actual MPU data-ready interrupt is not routed. Later estimation must use measurement timing evidence rather than assuming that the scheduler period is identical to physical sample time.

## Scaling, calibration, and frame mapping

The MPU6050 device crate owns nominal transfer functions for the configured full-scale ranges and temperature conversion. These functions convert register counts into SI units in the native sensor frame.

`swp-sensor-calibration` applies measured three-axis affine correction and carries explicit calibration evidence/revision. `swp-frame-transform` then applies a separately evidenced sensor-to-body rotation. Mechanical mounting orientation is never hidden inside sensor bias/scale parameters.

The current target firmware intentionally continues to record raw evidence because no verified physical calibration profile or complete sensor-to-body transform has yet been established for the unit. Zero bias, identity calibration, or guessed axis signs are not promoted merely to make the pipeline executable.

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

The unpopulated third motor interface remains a board capability, not a robot actuator. `swp-board-one-v2` describes PCB wiring; `swp-reference-assembly` owns the verified transition from board channels to installed robot roles.

Encoder association follows the installed harness: Encoder 1 belongs to the reaction-wheel motor path and Encoder 2 to the drive-wheel motor path. Encoder sign and mechanical scale remain commissioning facts.

## Runtime authority and reaction-wheel saturation

The supplied operating material confirms an important plant constraint: lateral balance depends on reaction-wheel counter-torque, and sustained disturbance can drive the inertia wheel toward maximum speed until balancing authority is lost.

`swp-runtime-state` therefore makes startup/capture and reaction-wheel headroom explicit. Its initial operating-state vocabulary is:

```text
Boot
HardwareCheck
Standby
CaptureWindow
Balancing
MomentumLimited
Fault
```

Physical actuation is denied outside explicit closed-loop states. Exact reaction-wheel momentum still depends on an unverified wheel inertia, so the current authority model uses configurable wheel-speed warning/hard limits rather than inventing `J`.

## Recording and deterministic replay

Recording/replay is a first-class data path:

```text
RawObservation
   |\
   | +------------------> scaling / calibration / frame / estimator path
   |
   +--> RecordedObservation --> binary log --> replay
```

`swp-observation-record` owns the deterministic binary record format. UART is only the current transport; transport is not the owner of acquisition semantics. Host tools can decode the same record stream to CSV or replay it without using host wall-clock timing.

## Board communication and UI surfaces

The reviewed board exposes three distinct surfaces with intentionally different roles:

```text
USART1 / PA9-PA10
    -> wired recorder and engineering debug

USART2 / PA2-PA3 -> ECB02S2 BLE module
    -> wireless commissioning and validated commands

PB4-PB5 -> OLED
    -> low-rate local health/status
```

The onboard CH340 and MCU USART1 nets terminate on separate P2 pins; USB-C/CH340 is therefore not assumed to be hard-wired to the recorder UART. The ECB02 control pins are PC15 `AT_EN` and PC14 `ROLE`; its sleep input is tied low on the reviewed board. PB4 is also an STM32F103 JTAG pin after reset, so OLED use must release full JTAG while preserving the desired SWD path.

`EN_X` (PA15) and `EN_Y` (PB3) are separate board jumper/authority inputs. They are **not** the same as `EN_BLDC_*`, which are hard-wired high. Their final actuator-semantic association remains unpromoted because product labels and legacy X/Y naming are not fully consistent.

## Repository layout

```text
crates/
  robot-domain/          Verified single-wheel physical/control-domain types
  reference-assembly/    Installed actuator population and board-to-role mapping
  plant-observation/     Raw evidence, timing, and measurement quality
  sensor-calibration/    Device scaling boundary and measured IMU correction
  frame-transform/       Evidenced sensor-frame -> canonical body-frame rotation
  runtime-state/         Startup/capture/authority and reaction-wheel headroom types
  observation-record/    Deterministic binary record/replay contract
  mpu6050/               Portable driver and nominal transfer functions
  software-i2c/          Portable open-drain embedded-hal I2C
  board-one-v2/          Reference-board wiring facts only

firmware/
  stm32f103/             no_std RTIC runtime and STM32 peripheral ownership

tools/
  recording/             Record decode and deterministic replay source

docs/
  architecture/          Semantic, timing, frame, replay, calibration, authority contracts
  hardware/              Schematic review, assembly observation, and board mapping
  commissioning/         Bring-up and characterization notes
```

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

Product material and legacy source support a 72 MHz target configuration, while the current passive bring-up executable remains deliberately on HSI 8 MHz. No TIM3 motor PWM or direction output is configured yet.

## Evidence status

Enough source material now exists to continue architecture and hardware bring-up without waiting for more documents. Remaining blockers are primarily measurements:

```text
MPU X/Y body-frame signs
encoder counts/revolution and positive direction
PWM active polarity and DIR polarity
battery ADC transfer function
installed reaction-wheel dimensions/mass/inertia
physical HSE marking
```

Product-document values remain priors rather than silent control constants where physical verification matters. Known revision conflicts, including OLED size and inertia-wheel diameter, stay visible in `docs/hardware/hardware_baseline.md`.

## Build

Install a current stable Rust toolchain with the `thumbv7m-none-eabi` target. `cargo fw` links the STM32F103 release firmware. CI enforces formatting, full Cortex-M workspace check, Clippy with warnings denied, reference-assembly tests, MPU transfer-function tests, sensor-calibration tests, frame-transform tests, runtime-authority tests, observation-record tests, Python recording decoder tests, and the final release firmware link.

See [`docs/architecture/system_architecture.md`](docs/architecture/system_architecture.md), [`docs/architecture/body_frame_contract.md`](docs/architecture/body_frame_contract.md), [`docs/architecture/runtime_authority.md`](docs/architecture/runtime_authority.md), [`docs/architecture/calibration_contract.md`](docs/architecture/calibration_contract.md), [`docs/architecture/observation_time_health_replay.md`](docs/architecture/observation_time_health_replay.md), and [`docs/hardware/hardware_baseline.md`](docs/hardware/hardware_baseline.md).

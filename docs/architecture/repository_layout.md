# Repository Layout

The repository uses a Rust workspace. Directory boundaries follow ownership and semantic meaning rather than a traditional BSP/middleware/application split.

```text
Cargo.toml
rust-toolchain.toml
.cargo/
.github/workflows/

crates/
  robot-domain/
    src/lib.rs          Verified single-wheel physical/control-domain types

  reference-assembly/
    src/lib.rs          Installed population and board-channel-to-robot-role mapping

  plant-observation/
    src/lib.rs          Raw plant evidence, timing, and measurement quality

  sensor-calibration/
    src/lib.rs          SI scaling boundary and measured sensor-frame correction

  observation-record/
    src/lib.rs          Deterministic binary record/replay contract

  mpu6050/
    src/lib.rs          Generic embedded-hal driver and nominal transfer functions

  software-i2c/
    src/lib.rs          Portable open-drain embedded-hal I2C implementation

  board-one-v2/
    src/lib.rs          Schematic-derived PCB wiring facts only

firmware/
  stm32f103/
    Cargo.toml
    build.rs
    memory.x
    src/main.rs         RTIC application / concrete peripheral ownership

docs/
  architecture/
  hardware/
  commissioning/

tools/
  recording/            Canonical record decode and deterministic replay
```

The dependency meanings are:

```text
robot-domain        knows the verified single-wheel plant semantics, not STM32
reference-assembly  maps installed board channels to robot actuator roles
plant-observation   carries raw evidence and uncertainty, not robot interpretation
sensor-calibration  separates nominal device scaling from measured calibration
observation-record  serializes observation evidence for recording/replay
mpu6050             owns sensor protocol and datasheet transfer functions
software-i2c        implements the standard I2C contract, not board wiring
board-one-v2        knows PCB channels and pins, not robot actuator roles
firmware            owns STM32 peripherals and composes runtime acquisition
stm32f1xx-hal       owns STM32F1 peripheral access
RTIC                owns real-time task/resource priority and concurrency
```

The reference-assembly crate exists because three different forms of truth must not be collapsed:

```text
board capability    BLDC_1, BLDC_2, BLDC_3 exist electrically
assembly population BLDC_1 + BLDC_2 installed; BLDC_3 not installed
robot semantics     BLDC_1 = ReactionWheel; BLDC_2 = DriveWheel
```

The sensor-calibration crate deliberately does not own mounting rotation or robot-axis mapping. A value can therefore become SI-valued and physically calibrated while still remaining in the sensor frame.

UART is not an architectural layer. It is the current transport for `RecordedObservation` bytes. Replacing UART with DMA, USB, SWO, storage, or another transport must not change `RawObservation` semantics or the record/replay contract.

New abstractions are introduced only for a real change in ownership or physical meaning. Unknown facts remain representable as unknown rather than being filled with historical assumptions.

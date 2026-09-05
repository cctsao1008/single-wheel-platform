# Repository Layout

The repository uses a Rust workspace. The old C/CMake layer split has been retired rather than retained as a compatibility shell.

```text
Cargo.toml
rust-toolchain.toml
.cargo/
.github/workflows/

crates/
  robot-domain/
    src/lib.rs          Single-wheel physical state and actuator-domain types

  plant-observation/
    src/lib.rs          Raw plant evidence before calibration or coordinate mapping

  mpu6050/
    src/lib.rs          Generic embedded-hal device driver

  software-i2c/
    src/lib.rs          Portable open-drain embedded-hal I2C implementation

  telemetry-protocol/
    src/lib.rs          Versioned binary telemetry transport format

  board-one-v2/
    src/lib.rs          Schematic-derived PCB wiring facts only

firmware/
  stm32f103/
    Cargo.toml
    build.rs
    memory.x
    src/main.rs         RTIC application / peripheral ownership

docs/
  architecture/
  hardware/
  commissioning/

tools/
  telemetry/            Host-side capture decoding and analysis
```

The important boundary is semantic rather than directory depth:

```text
robot-domain       knows the single-wheel plant, not STM32
plant-observation  carries raw evidence, not calibrated robot meaning
mpu6050            knows the sensor, not the board or controller
software-i2c       implements the standard I2C contract, not board wiring
telemetry-protocol serializes observations, but does not own acquisition
board-one-v2       knows PCB channels and pins, not robot actuator roles
firmware           owns STM32 peripherals and composes the above pieces
stm32f1xx-hal      owns STM32F1 peripheral access
RTIC               owns the real-time task/resource model
```

New abstractions are added only when they represent a real change in ownership or semantic meaning. A private `board_*` HAL is not recreated when an ecosystem trait already expresses the required contract, and a PCB channel is not renamed into a robot role until that mapping is physically established.

See [`typed_dataflow.md`](typed_dataflow.md) for the runtime information-flow model.

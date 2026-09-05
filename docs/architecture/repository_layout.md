# Repository Layout

The repository uses a Rust workspace. The old C/CMake layer split has been retired rather than retained as a compatibility shell.

```text
Cargo.toml
rust-toolchain.toml
.cargo/
.github/workflows/

crates/
  robot-domain/
    src/lib.rs          Single-wheel physical/domain types

  mpu6050/
    src/lib.rs          Generic embedded-hal device driver

  software-i2c/
    src/lib.rs          Portable open-drain embedded-hal I2C implementation

  board-one-v2/
    src/lib.rs          Schematic-derived board wiring facts

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
```

The important boundary is semantic rather than directory depth:

```text
robot-domain     knows the single-wheel plant, not STM32
mpu6050          knows the sensor, not the board or controller
software-i2c     implements the standard I2C contract, not board wiring
board-one-v2     knows the reference PCB wiring, not control policy
firmware         owns STM32 peripherals and composes the above pieces
stm32f1xx-hal    owns STM32F1 peripheral access
RTIC             owns the real-time task/resource model
```

New abstractions are added only when they represent a real boundary in this platform. A private `board_*` HAL is not recreated when an `embedded-hal` trait already expresses the required contract.

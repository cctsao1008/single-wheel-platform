# Self-Balancing Single-Wheel Platform

A clean re-architecture of the embedded software for a self-balancing single-wheel robot.

The repository is intentionally organized around the physical system rather than around inherited firmware structure. Device access, board wiring, robot-domain state, real-time scheduling, estimation, control, and actuation are separated so that changes in one area do not leak through the rest of the system.

The current implementation uses **Rust `no_std`**, **embedded-hal 1.0**, **stm32f1xx-hal**, and **RTIC** on the STM32F103 reference target. The earlier C/CMake scaffold has been removed rather than carried forward as a compatibility layer.

## Physical system

The reference platform has three actuation paths:

- **Roll / lateral balance** — reaction wheel.
- **Pitch / longitudinal balance** — ground-contact drive wheel.
- **Yaw / spin** — third brushless actuator path.

The reference board also contains an MPU6050 and wheel-encoder feedback. Board facts are kept separate from controller conventions; the software does not infer robot meaning from historical names such as `motor1`, `x`, or `y`.

## Architecture

```text
                    RTIC application
                         |
        +----------------+----------------+
        |                |                |
        v                v                v
 robot-domain        device drivers    board description
        |                |                |
        |        embedded-hal traits      |
        |                |                |
        +----------------+----------------+
                         |
                  stm32f1xx-hal
                         |
                    STM32F103C8
                         |
                  physical hardware
```

At runtime the critical path remains:

```text
measurement
   -> coordinate mapping
   -> state estimation
   -> state validation
   -> control policy
   -> actuator mapping
   -> authority / limits
   -> physical output
```

RTIC owns concurrency and scheduling. Peripheral ownership is represented by Rust types instead of globally accessible registers or a custom C board API.

## Repository layout

```text
Cargo.toml
rust-toolchain.toml
.cargo/

crates/
  robot-domain/          Physical state and actuator-domain types
  mpu6050/               Portable embedded-hal MPU6050 driver
  board-one-v2/          Reference-board wiring and hardware facts

firmware/
  stm32f103/             no_std RTIC application for STM32F103C8

docs/
  architecture/          Architecture and timing contracts
  hardware/              Schematic review and board mapping
  commissioning/         Bring-up and characterization notes

tools/                   Host-side engineering tools
```

## Design rules

- Use ecosystem-standard hardware traits where a standard contract exists; do not recreate a private HAL for its own sake.
- Keep device drivers generic over `embedded-hal` and independent of STM32 types.
- Keep board-specific wiring in the board crate, not in device drivers or control code.
- Keep robot-domain state and actuator semantics independent of the MCU and HAL.
- Make coordinate conventions, physical units, timestamps, and actuator authority explicit.
- Keep blocking telemetry, display work, storage, and maintenance traffic outside the critical control path.
- Do not hard-code unconfirmed hardware facts. For example, the reviewed schematic does not label the HSE crystal frequency, and the net named `MPU_INT` is connected to MPU6050 **FSYNC**, while the actual MPU6050 INT pin is not routed.

## Reference target

- STM32F103C8T6, Cortex-M3
- 64 KiB Flash / 20 KiB SRAM
- MPU6050 at schematic-selected address `0x68`
- MPU SDA on PB8 and SCL on PB9, requiring a software-I2C implementation for this board wiring rather than the STM32F1 I2C1 remap
- TIM3 motor PWM paths, TIM2/TIM4 encoder paths
- SWD development workflow through `probe-rs`

See [`docs/hardware/pin_mapping.md`](docs/hardware/pin_mapping.md) for the reviewed board mapping and [`docs/architecture/system_architecture.md`](docs/architecture/system_architecture.md) for the software boundaries.

## Build direction

Install a current stable Rust toolchain with the `thumbv7m-none-eabi` target. The repository pins architecture dependencies in the Cargo workspace; `cargo fw` builds the STM32F103 release target.

The firmware target currently boots into a safe, inactive RTIC skeleton. Peripheral ownership and bring-up are being added directly in the new architecture rather than by porting the removed C board layer line-for-line.

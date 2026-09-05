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
  software-i2c/          Portable open-drain embedded-hal I2C
  telemetry-protocol/    Fixed binary target/host telemetry contract
  board-one-v2/          Reference-board wiring and hardware facts

firmware/
  stm32f103/             no_std RTIC application for STM32F103C8

docs/
  architecture/          Architecture and timing contracts
  hardware/              Schematic review and board mapping
  commissioning/         Bring-up and characterization notes

tools/
  telemetry/             Python capture and decode utilities
```

## Design rules

- Use ecosystem-standard hardware traits where a standard contract exists; do not recreate a private HAL for its own sake.
- Keep device drivers generic over `embedded-hal` and independent of STM32 types.
- Keep board-specific wiring in the board crate, not in device drivers or control code.
- Keep robot-domain state and actuator semantics independent of the MCU and HAL.
- Make coordinate conventions, physical units, timestamps, and actuator authority explicit.
- Keep telemetry, display work, storage, and maintenance traffic outside the high-priority acquisition/control path.
- Do not hard-code unconfirmed hardware facts. For example, the reviewed schematic does not label the HSE crystal frequency, and the net named `MPU_INT` is connected to MPU6050 **FSYNC**, while the actual MPU6050 INT pin is not routed.

## Reference target

- STM32F103C8T6, Cortex-M3
- 64 KiB Flash / 20 KiB SRAM
- MPU6050 at schematic-selected address `0x68`
- MPU SDA on PB8 and SCL on PB9, requiring a software-I2C implementation for this board wiring rather than the STM32F1 I2C1 remap
- TIM3 motor PWM paths, TIM2/TIM4 encoder paths
- USART1 on PA9/PA10 for the main UART; the CH340N pair is exposed separately on P2 rather than hard-wired to USART1
- SWD development workflow through `probe-rs`

See [`docs/hardware/pin_mapping.md`](docs/hardware/pin_mapping.md) for the reviewed board mapping and [`docs/architecture/system_architecture.md`](docs/architecture/system_architecture.md) for the software boundaries.

## Current executable path

The Rust firmware now has a complete passive sensing/observability slice:

```text
HSI 8 MHz
  -> software I2C on PB8/PB9
  -> MPU6050 probe + explicit configuration
  -> TIM1 100 Hz raw acquisition
  -> DWT timestamp
  -> fixed CRC-protected telemetry frame
  -> lock-free SPSC frame queue
  -> lower-priority USART1 TXE interrupt pump
  -> PA9 / TX
```

No motor PWM, direction, or brake output is configured in this path. The next runtime work builds on this observable base rather than reintroducing a monolithic ISR.

## Build

Install a current stable Rust toolchain with the `thumbv7m-none-eabi` target. `cargo fw` links the STM32F103 release firmware. CI also checks formatting, the full Cortex-M workspace, Clippy with warnings denied, host-side telemetry protocol tests, and the final release link.

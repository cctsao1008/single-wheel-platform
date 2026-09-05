# System Architecture

## Purpose

The project is a clean re-architecture of the embedded software around the existing single-wheel physical platform. It is not a preservation exercise for the previous source structure.

The architecture separates four kinds of knowledge:

1. **robot-domain knowledge** — roll, pitch, yaw, wheel states, actuator requests and physical units,
2. **device knowledge** — register/protocol behavior of devices such as the MPU6050,
3. **board knowledge** — which MCU pins and timer channels are wired to which physical functions,
4. **target-runtime knowledge** — STM32F103 peripheral ownership, scheduling, interrupts and execution timing.

Rust type ownership and ecosystem hardware traits are used to enforce those boundaries directly instead of recreating a custom C-style HAL.

## Dependency structure

```text
                    firmware/stm32f103
                      /      |      \
                     v       v       v
          swp-robot-domain  board-one-v2  swp-mpu6050
                                  |            |
                                  |       embedded-hal 1.0
                                  |            |
                                  +------ stm32f1xx-hal
                                              |
                                         STM32F103C8
```

RTIC is the firmware concurrency model. It owns task priority, shared/local resources and interrupt-driven execution; it does not define robot-domain behavior.

## Primary runtime path

```text
Physical Sensors / Encoders
          |
          v
       Drivers
          |
          v
  Timestamped Measurements
          |
          v
 Coordinate Transform
          |
          v
   State Estimation
          |
          v
   State Validation
          |
          v
     Control Policy
          |
          v
   Actuator Mapping
          |
          v
 Limits / Authority
          |
          v
 HAL-owned Outputs
          |
          v
  Physical Actuators
```

Telemetry, display rendering, persistent storage, maintenance commands and log transfer remain outside the timing-critical path.

## Robot domain

`swp-robot-domain` owns state and command types that have physical meaning on this system. It is `no_std` and has no dependency on an MCU, HAL, device driver or scheduler.

The domain is intentionally not generalized into a robotics framework. The state representation remains specific to the single-wheel plant: roll/pitch state, reaction-wheel speed, drive-wheel speed, yaw-rate information, battery state and validity.

## Device drivers

Device drivers are generic over standard `embedded-hal` traits. The MPU6050 driver owns MPU6050 register configuration, range selection, sample-rate configuration, raw acquisition and conversion scales. It does not own sensor mounting orientation, robot coordinates or balancing policy.

This removes the need for a project-specific transport callback layer. The driver accepts any I2C implementation that satisfies `embedded_hal::i2c::I2c`.

## Board description

`swp-board-one-v2` records reference-board wiring facts independently of the STM32 HAL implementation.

Important reviewed facts include:

- BLDC1 / lateral: PB1 TIM3_CH4, PB11 direction, encoder on PA1/PA0,
- BLDC2 / longitudinal: PA6 TIM3_CH1, PA4 direction, encoder on PB7/PB6,
- BLDC3 / spin: PB0 TIM3_CH3, PB10 direction, PA7 brake,
- MPU6050 SDA/SCL: PB8/PB9 as drawn by the board schematic,
- PC13 net `MPU_INT` connects to MPU6050 FSYNC; the actual MPU6050 INT pin is not routed,
- MPU6050 AD0 is low, selecting address `0x68`,
- PA5 is the battery ADC node,
- the reviewed schematic does not label the external crystal frequency.

Electrical behaviors that are not established by the schematic, such as motor-module brake polarity or robot-positive direction, are not promoted into board constants prematurely.

## STM32F103 firmware

`firmware/stm32f103` is the only workspace member allowed to own concrete STM32 peripheral instances. It uses `stm32f1xx-hal` rather than handwritten memory-mapped register definitions.

The current migration baseline intentionally leaves actuators inactive and does not force a 72 MHz clock configuration because the schematic itself does not identify the HSE frequency. Hardware bring-up is added directly through typed HAL ownership after the required board fact is established.

The unusual MPU wiring is handled above the GPIO layer by a software-I2C implementation satisfying `embedded-hal::i2c::I2c`; the MPU6050 driver itself remains unaware of that board constraint.

## Real-time execution

RTIC replaces ad-hoc global peripheral access and monolithic interrupt bodies with explicit local/shared resources and statically prioritized tasks.

A hardware interrupt should perform only the work necessary for deterministic acquisition/control execution or should release work to a lower-priority software task. UART formatting, display rendering, Flash operations and maintenance parsing are not part of the control ISR path.

The control-loop rate is not inherited from the previous firmware. Acquisition rate, estimator rate and controller rate are selected from measured plant, sensor, actuator and WCET requirements.

## Actuator ownership

Controller output and physical output ownership remain separate concepts. A control policy requests effort in robot-domain terms; actuator mapping and authority logic decide whether and how that request reaches a physical motor.

Rust ownership is used to make the physical PWM/timer resource have one software owner. Maintenance, commissioning and automatic control therefore do not receive independent mutable access to the same timer peripheral.

## Extension policy

New control laws, estimators, replay tools and system-identification utilities may be added without changing the board or device-driver contracts. They are capabilities enabled by the architecture, not the project identity.

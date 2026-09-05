# System Architecture

## Purpose

This project re-architects the embedded software around the physical single-wheel plant. It does not preserve the structure or assumptions of previous firmware for compatibility.

The architecture separates six kinds of knowledge:

1. **acquisition evidence** — what was measured, when it was observed, and what is known about measurement quality;
2. **sensor and frame semantics** — device scaling, measured calibration, and evidenced sensor-to-body coordinate transforms;
3. **robot-domain knowledge** — roll, pitch, yaw, wheel state, generalized effort, and physical units;
4. **device knowledge** — register/protocol behavior of devices such as the MPU6050;
5. **board / assembly knowledge** — PCB channels and pins versus what the inspected unit physically contains;
6. **target-runtime knowledge** — STM32F103 peripheral ownership, operating state, authority, scheduling, interrupts, and execution timing.

Rust types make invalid semantic shortcuts harder while RTIC and the HAL own execution/resource constraints.

## Dependency structure

```text
                           firmware/stm32f103
                    /       |        |        \
                   v        v        v         v
          board-one-v2   mpu6050  plant-observation  observation-record
               |                         |
               v                         v
        reference-assembly       sensor-calibration
               |                         |
               v                         v
          robot-domain             frame-transform
               |                         |
               +-----------+-------------+
                           v
                     runtime-state

portable device drivers -> embedded-hal 1.0
concrete target ownership -> stm32f1xx-hal + RTIC
```

The arrows above describe semantic dependency, not a requirement that every crate be used by the current passive firmware cut. `robot-domain`, `frame-transform`, and `runtime-state` remain independent of concrete STM32 peripherals.

## Primary runtime path

```text
Physical Sensors / Encoders
          |
          v
 Hardware Capture
          |
          v
 RawObservation
(raw values + timing + quality)
          |
          +---------------------------> Recorder / Replay
          |
          v
 Sensor Transfer Functions
          |
          v
 ScaledObservation
          |
          v
 Measured Sensor Calibration
          |
          v
 CalibratedObservation
          |
          v
 Evidenced Frame Transform
          |
          v
 BodyObservation
          |
          v
 State Estimation
          |
          v
 EstimatedState
          |
          v
 Control Policy
          |
          v
 GeneralizedDemand
          |
          v
 Actuator Allocation
          |
          v
 Runtime Authority / Limits
          |
          v
 Electrical Mapping
          |
          v
 Physical Actuators
```

Recording, display, Bluetooth commissioning, maintenance, and host analysis are outside the high-priority control path. They may observe semantic state or submit validated requests, but transport cannot own or mutate physical semantics directly.

## Canonical body frame

The robot-domain body frame is explicit and right-handed:

```text
+X = forward, along the ground-drive direction
+Y = left
+Z = up
```

`swp-frame-transform` owns the semantic promotion from calibrated MPU6050 sensor-frame measurements into this body frame. It accepts only proper 3-D rotations and requires explicit frame evidence. The reference assembly intentionally has no default sensor-to-body rotation yet because the remaining X/Y signs still require physical tilt/rotation confirmation.

This prevents sensor-package orientation, PCB orientation, and robot mechanics from collapsing into undocumented sign changes inside an estimator.

## Measurement time

Scheduler time, source-sample time, readout time, and transmit time are different quantities.

The current reference board cannot observe the MPU6050 data-ready event because the schematic net named `MPU_INT` is wired to FSYNC and the actual INT pin is not routed. Firmware therefore records the MPU source-sample time as unknown while preserving I2C read start/completion times. Encoder snapshots carry their own capture timestamps. Battery ADC preserves read-completion timing without claiming an exact analog sample instant.

The estimator must not use a hard-coded scheduler `dt` as a substitute for measurement timing when better timestamp evidence exists.

## Measurement quality

A single valid/invalid bit is insufficient. `MeasurementQuality` records independent evidence such as availability, successful I/O, known timing, freshness verification, saturation, staleness, or retry history.

Unset flags are not silently interpreted as proof of the opposite property. In particular, a clean MPU I2C transaction does not prove that the returned internal sample is fresh or exactly timestamped.

## Runtime state and authority

The physical plant has explicit startup/capture behavior and a finite reaction-wheel speed/momentum envelope. `swp-runtime-state` therefore separates operating state from RTIC scheduling and from raw motor GPIO ownership.

The initial semantic states are:

```text
Boot
HardwareCheck
Standby
CaptureWindow
Balancing
MomentumLimited
Fault
```

Actuation is denied in boot/check/standby/capture/fault states. Only explicit closed-loop states may request physical authority.

Reaction-wheel saturation is treated as control-authority state. Until installed wheel inertia is verified, the runtime models speed-domain headroom with configurable warning/hard limits rather than inventing momentum constants. Exact momentum-domain authority can replace or augment this once `J` is measured.

## Record/replay contract

`swp-observation-record` converts `RawObservation` into a fixed-size, CRC-protected record. The record is intended for deterministic capture and replay across estimator/controller revisions.

The current USART1 TX path is only a transport implementation. A future DMA, storage, USB, or SWO recorder should use the same observation/record semantics rather than creating a second data model.

## Device drivers

Device drivers are generic over standard `embedded-hal` traits. The MPU6050 driver owns device register behavior, configuration, ranges, and raw transfer. It does not own sensor mounting orientation, robot coordinates, or balancing policy.

## Board / assembly description

`swp-board-one-v2` records schematic-derived PCB facts. It describes BLDC and encoder channels, serial/OLED wiring, Bluetooth control pins, and EN_X/EN_Y jumper pins without assigning unverified robot meaning.

`swp-reference-assembly` records the inspected physical population:

```text
BLDC_1 / PCB M2 -> ReactionWheel
BLDC_2 / PCB M1 -> DriveWheel
BLDC_3 / PCB M3 -> not installed
```

Unconfirmed motor polarity, brake polarity, ADC scale, exact HSE marking, robot-positive encoder direction, and EN_X/EN_Y semantic association remain outside board facts.

## STM32F103 runtime

`firmware/stm32f103` is the only workspace member allowed to own concrete STM32 peripherals. It currently uses HSI 8 MHz, software I2C for the schematic PB8/PB9 MPU wiring, TIM2/TIM4 QEI, ADC1/PA5, TIM1 acquisition scheduling, DWT timestamping, and lower-priority USART1 record transport.

Product documentation and legacy source support a 72 MHz operating configuration, but the passive bring-up runtime deliberately remains on HSI until the clock transition is introduced as an explicit target configuration rather than a hidden board fact.

TIM3 and all motor GPIO remain untouched during this passive observation phase.

## Communication and UI roles

The board has three distinct observability/commissioning surfaces:

```text
USART1 / PA9-PA10
    -> wired recorder/debug transport

USART2 / PA2-PA3 -> ECB02S2
    -> wireless commissioning / validated command path

PB4-PB5 -> OLED
    -> low-rate local health/status surface
```

These are intentionally different responsibilities. Bluetooth is not allowed to bypass runtime authority; OLED is not a control owner; UART recording cannot block acquisition/control work.

## Real-time execution

RTIC owns priority and resource concurrency. Acquisition/control work must have bounded execution and must not wait on host I/O. Record transmission is lower priority and queue-backed; record drops are counted and preserved in subsequent records.

Acquisition rate, estimator rate, and controller rate are selected from sensor/plant timing and WCET evidence rather than inherited constants.

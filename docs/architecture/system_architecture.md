# System Architecture

## Purpose

This project re-architects the embedded software around the physical single-wheel plant. It does not preserve the structure or assumptions of previous firmware for compatibility.

The architecture separates five kinds of knowledge:

1. **acquisition evidence** — what was measured, when it was observed, and what is known about measurement quality;
2. **robot-domain knowledge** — roll, pitch, yaw, wheel state, generalized effort, and physical units;
3. **device knowledge** — register/protocol behavior of devices such as the MPU6050;
4. **board knowledge** — PCB channels, pins, and timer wiring;
5. **target-runtime knowledge** — STM32F103 peripheral ownership, scheduling, interrupts, and execution timing.

Rust types make invalid semantic shortcuts harder while RTIC and the HAL own execution/resource constraints.

## Dependency structure

```text
                         firmware/stm32f103
                       /    |       |       \
                      v     v       v        v
             board-one-v2  mpu6050  plant-observation  observation-record
                              |              |              |
                       embedded-hal 1.0      +--------------+
                              |
                        stm32f1xx-hal
                              |
                         STM32F103C8
```

`robot-domain` remains independent of the MCU/HAL and is consumed only when raw evidence has been promoted into physical robot meaning.

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
 Sensor Calibration
          |
          v
 CalibratedObservation
          |
          v
 Frame / Mechanical Mapping
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
 Authority / Limits
          |
          v
 Electrical Mapping
          |
          v
 Physical Actuators
```

Recording, display, maintenance, and host analysis are outside the high-priority control path. Recording may observe the same data, but transport cannot own or mutate its semantics.

## Measurement time

Scheduler time, source-sample time, readout time, and transmit time are different quantities.

The current reference board cannot observe the MPU6050 data-ready event because the schematic net named `MPU_INT` is wired to FSYNC and the actual INT pin is not routed. Firmware therefore records the MPU source-sample time as unknown while preserving I2C read start/completion times. Encoder snapshots carry their own capture timestamps. Battery ADC preserves read-completion timing without claiming an exact analog sample instant.

The estimator must not use a hard-coded scheduler `dt` as a substitute for measurement timing when better timestamp evidence exists.

## Measurement quality

A single valid/invalid bit is insufficient. `MeasurementQuality` records independent evidence such as availability, successful I/O, known timing, freshness verification, saturation, staleness, or retry history.

Unset flags are not silently interpreted as proof of the opposite property. In particular, a clean MPU I2C transaction does not prove that the returned internal sample is fresh or exactly timestamped.

## Record/replay contract

`swp-observation-record` converts `RawObservation` into a fixed-size, CRC-protected record. The record is intended for deterministic capture and replay across estimator/controller revisions.

The current USART1 TX path is only a transport implementation. A future DMA, storage, USB, or SWO recorder should use the same observation/record semantics rather than creating a second data model.

## Device drivers

Device drivers are generic over standard `embedded-hal` traits. The MPU6050 driver owns device register behavior, configuration, ranges, and raw transfer. It does not own sensor mounting orientation, robot coordinates, or balancing policy.

## Board description

`swp-board-one-v2` records schematic-derived PCB facts. It describes BLDC and encoder channels rather than assigning robot actuator roles. Unconfirmed motor polarity, brake polarity, ADC scale, external-crystal frequency, and robot-positive direction remain outside board facts.

## STM32F103 runtime

`firmware/stm32f103` is the only workspace member allowed to own concrete STM32 peripherals. It currently uses HSI 8 MHz, software I2C for the schematic PB8/PB9 MPU wiring, TIM2/TIM4 QEI, ADC1/PA5, TIM1 acquisition scheduling, DWT timestamping, and lower-priority USART1 record transport.

TIM3 and all motor GPIO remain untouched during this passive observation phase.

## Real-time execution

RTIC owns priority and resource concurrency. Acquisition/control work must have bounded execution and must not wait on host I/O. Record transmission is lower priority and queue-backed; record drops are counted and preserved in subsequent records.

Acquisition rate, estimator rate, and controller rate are selected from sensor/plant timing and WCET evidence rather than inherited constants.

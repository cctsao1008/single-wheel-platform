# System Architecture

The platform is organized around semantic transitions and physical ownership.

## Core dataflow

```text
Physical Plant
      |
      v
RawObservation
      |
      | device transfer functions
      v
ScaledObservation
      |
      | measured calibration
      v
CalibratedObservation
      |
      | sensor -> body transform
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
RuntimeAuthority
      |
      v
ElectricalOutput
      |
      v
Physical Actuators
```

Recording is a branch from the raw-observation boundary:

```text
RawObservation
   |\
   | +--> semantic/control path
   |
   +--> RecordedObservation --> transport/storage --> replay
```

## Semantic ownership

```text
plant-observation
    raw values, timing, quality, acquisition status

mpu6050
    device protocol and nominal transfer functions

sensor-calibration
    measured sensor-frame correction

frame-transform
    sensor-frame to platform-body rotation

platform-domain
    platform state, generalized demand, actuator roles

runtime-state
    operating state, limits, physical-output authority
```

## Hardware ownership

```text
board-one-v2
    PCB pins, timers, buses, connector identities

reference-assembly
    installed hardware and board-channel-to-role mapping

firmware/stm32f103
    concrete STM32 peripheral ownership and RTIC execution
```

The reference assembly is a two-actuator plant:

```text
BLDC_1 / Encoder_1 -> ReactionWheel
BLDC_2 / Encoder_2 -> DriveWheel
BLDC_3             -> unused
```

Board capability, assembly population, and platform semantics remain separate types of information.

## Body frame

```text
+X = forward
+Y = left
+Z = up
```

Roll, pitch, and yaw follow the right-hand rule about +X, +Y, and +Z respectively.

## Measurement model

Scheduler time, physical source-sample time, peripheral capture time, readout completion time, and transmission time are distinct.

The MPU6050 source-sample timestamp is `Unknown` because the reference board does not route the device data-ready interrupt. I2C read start/completion times remain available.

`MeasurementQuality` carries independent availability, I/O, timing, freshness, saturation, staleness, and retry state. An unset flag does not imply the opposite property.

## Runtime authority

The operating-state model is:

```text
Boot
  -> HardwareCheck
  -> Standby
  -> CaptureWindow
  -> Balancing
       |-> MomentumLimited
       |-> Fault
```

Only authorized closed-loop states may reach physical outputs. Reaction-wheel speed/headroom is part of actuator authority.

## Real-time runtime

The STM32F103 target uses Rust `no_std`, `embedded-hal` 1.0, `stm32f1xx-hal`, and RTIC.

The target composition is:

```text
TIM1          acquisition scheduling
DWT           monotonic acquisition timing
PB8/PB9       software I2C -> MPU6050
TIM2          Encoder_1 QEI
TIM4          Encoder_2 QEI
ADC1 / PA5    battery ADC
USART2        ECB02S2 wireless record transport
USART1        wired engineering interface
PB4/PB5       OLED status interface
```

Control/acquisition work does not block on UART, BLE, display rendering, storage, or host traffic.

## Interface roles

```text
USART2 + ECB02S2
    wireless `RecordedObservation` transport for the mobile platform

USART1
    wired bench / engineering interface

OLED
    optional local status interface
```

The host-side BLE observer reassembles the byte stream independently of BLE packet boundaries and preserves the canonical binary records for decode and replay.

Transport and UI components may observe state or submit validated requests; they do not own physical semantics or bypass runtime authority.

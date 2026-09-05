# Rust Firmware Bring-Up

The reference firmware is Rust-only. There is no parallel C implementation or compatibility board layer.

## Current executable path

The firmware now proves a passive multi-sensor acquisition path while keeping every actuator inactive:

```text
reset
  -> HSI 8 MHz clock
  -> DWT cycle counter
  -> PB8/PB9 software-I2C
  -> MPU6050 probe + configuration
  -> TIM2 Encoder 1 quadrature counter
  -> TIM4 Encoder 2 quadrature counter
  -> ADC1 / PA5 raw battery-divider sample
  -> TIM1 100 Hz acquisition interrupt
  -> coherent raw sensor snapshot
  -> CRC-protected binary telemetry
  -> lock-free SPSC frame queue
  -> USART1 TXE interrupt pump
  -> PA9 / schematic net TX
```

TIM3 and all motor PWM, direction, brake, and enable-related outputs remain untouched.

## Clock profile

The reviewed schematic confirms the STM32F103C8T6 but does not confirm the external high-speed crystal frequency. The executable therefore continues to use the MCU's internal 8 MHz oscillator. DWT cycle counting provides acquisition timestamps on that known clock basis. An HSE/PLL profile will only be introduced after the physical oscillator is confirmed from the board, BOM, or measurement.

## MPU6050 profile

The current commissioning profile is an explicit firmware choice, not a schematic fact:

```text
sample rate     100 Hz
gyro range      +/-1000 dps
accelerometer   +/-4 g
DLPF            CONFIG=3
data-ready IRQ  disabled
```

The data-ready interrupt remains disabled because the reviewed board does not route MPU6050 INT to the MCU. The net named `MPU_INT` reaches MPU6050 FSYNC instead.

## Encoder acquisition

The reviewed board routes Encoder 1 to TIM2 and Encoder 2 to TIM4. The HAL QEI input tuple is ordered timer CH1 then CH2; on this board those pins correspond to the schematic B then A nets:

```text
Encoder 1: TIM2_CH1 PA0 = B, TIM2_CH2 PA1 = A
Encoder 2: TIM4_CH1 PB6 = B, TIM4_CH2 PB7 = A
```

Firmware reports the raw wrapping 16-bit counters. It does not yet reinterpret their direction as robot-positive motion or convert counts to angular velocity. Encoder 3 is not acquired because the reviewed schematic does not show a route from its connector signals back to MCU timer inputs.

## Battery ADC acquisition

PA5 / ADC1_IN5 is the schematic node named `ADC` between divider resistors R2 and R4. Their values are not shown in the reviewed drawing, so the firmware reports only the raw ADC conversion. A voltage scale is not introduced until the divider ratio is confirmed from BOM, PCB data, or measurement.

## Telemetry architecture

TIM1 timestamps and acquires IMU, encoder, and battery-ADC data, builds one fixed-size snapshot, and attempts one SPSC enqueue. UART byte transmission is owned by the lower-priority USART1 interrupt task.

```text
TIM1_UP / priority 2
  timestamp -> acquire -> encode -> enqueue -> pend USART1

USART1 / priority 1
  dequeue frame -> TXE-driven byte pump
```

A snapshot is either enqueued whole or dropped whole; partial protocol frames are never intentionally inserted into the queue. `dropped_frames` is cumulative and reported in later successful frames.

The current Sensor Snapshot frame is protocol kind 2 and includes sequence, timestamp, raw IMU words, raw Encoder 1/2 counts, raw battery ADC, status bits, dropped-frame count, and CRC-16/CCITT-FALSE. Protocol kind 1 remains decodable for earlier raw-IMU captures.

## Host connection

USART1 TX is PA9 / schematic net `TX`, exposed on P2. The CH340N uses separate `CH340_TX` and `CH340_RX` nets that are also exposed on P2; the schematic does not short them to the MCU UART.

For a CH340 USB-UART path, the required signal relationship is:

```text
MCU TX / P2 TX -> CH340_RX
MCU RX / P2 RX <- CH340_TX
```

Only TX is used by the current firmware. See `tools/telemetry/` for capture and decode utilities.

# Rust Firmware Bring-Up

The reference firmware is Rust-only. There is no parallel C implementation or compatibility board layer.

## Current executable path

The firmware now proves a complete sensing-to-host path while keeping every actuator inactive:

```text
reset
  -> HSI 8 MHz clock
  -> DWT cycle counter
  -> PB8/PB9 open-drain ownership
  -> software-I2C bus recovery
  -> MPU6050 WHO_AM_I at 0x68
  -> explicit MPU6050 configuration
  -> TIM1 100 Hz acquisition interrupt
  -> raw 14-byte sensor read
  -> fixed binary telemetry frame
  -> lock-free SPSC frame queue
  -> USART1 TXE interrupt pump
  -> PA9 / schematic net TX
```

No motor PWM, direction, brake, or enable-related output is configured by this runtime.

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

## Why software I2C

The schematic routes:

```text
PB8 -> MPU_SDA
PB9 -> MPU_SCL
```

This is the opposite signal order from the STM32F103 I2C1 remap function on PB8/PB9. The firmware therefore treats these two pins as open-drain GPIO and exposes the resulting bus through the standard `embedded-hal` 1.0 `I2c` trait.

The portable `swp-mpu6050` crate does not know whether the bus is implemented by hardware I2C or GPIO bit-banging.

## Telemetry architecture

TIM1 only acquires the sample, timestamps it, builds a fixed-size frame, and attempts one SPSC enqueue. UART byte transmission is owned by the lower-priority USART1 interrupt task.

```text
TIM1_UP / priority 2
  acquire -> encode -> enqueue -> pend USART1

USART1 / priority 1
  dequeue frame -> TXE-driven byte pump
```

The queue stores seven complete frames. A frame is either enqueued whole or dropped whole; partial protocol frames are never intentionally inserted into the queue. `dropped_frames` is cumulative and reported in later successful frames.

Each raw-IMU frame is 38 bytes and includes sequence, DWT-derived timestamp, raw accelerometer/temperature/gyro words, status bits, dropped-frame count, and CRC-16/CCITT-FALSE.

## Host connection

USART1 TX is PA9 / schematic net `TX`, exposed on P2. The CH340N uses separate `CH340_TX` and `CH340_RX` nets that are also exposed on P2; the schematic does not short them to the MCU UART.

For a CH340 USB-UART path, the required signal relationship is:

```text
MCU TX / P2 TX -> CH340_RX
MCU RX / P2 RX <- CH340_TX
```

Only TX is used by the current firmware. See `tools/telemetry/` for capture and decode utilities.

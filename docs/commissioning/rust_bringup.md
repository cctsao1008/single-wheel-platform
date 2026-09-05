# Rust Firmware Bring-Up

The reference firmware is Rust-only. There is no parallel C implementation or compatibility board layer.

## Bring-up order

The first executable milestone deliberately proves the least destructive hardware path first:

```text
reset
  -> HSI 8 MHz clock
  -> PB8/PB9 open-drain ownership
  -> software-I2C bus recovery
  -> MPU6050 WHO_AM_I at 0x68
  -> passive idle
```

No motor PWM, direction, brake, or enable-related output is configured by this milestone.

## Why HSI first

The reviewed schematic confirms the STM32F103C8T6 but does not confirm the external high-speed crystal frequency. The first firmware therefore uses the MCU's internal 8 MHz oscillator. An HSE/PLL clock profile will be introduced only after the physical oscillator is confirmed from the board, BOM, or measurement.

## Why software I2C

The schematic routes:

```text
PB8 -> MPU_SDA
PB9 -> MPU_SCL
```

This is the opposite signal order from the STM32F103 I2C1 remap function on PB8/PB9. The firmware therefore treats these two pins as open-drain GPIO and exposes the resulting bus through the standard `embedded-hal` 1.0 `I2c` trait.

The portable `swp-mpu6050` crate does not know whether the bus is implemented by hardware I2C or GPIO bit-banging.

## First debugger-visible result

`imu_present` in the RTIC local resources is `true` only when bus recovery succeeds and MPU6050 `WHO_AM_I` returns `0x68`.

The next commissioning layer should add non-blocking telemetry so this result, timing, and raw sensor samples can be observed without a debugger.

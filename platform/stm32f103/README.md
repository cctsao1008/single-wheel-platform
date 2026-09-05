# STM32F103 Reference Platform

This directory contains the concrete STM32F103 implementation of the shared board APIs for the ONE_V2.0 reference hardware.

## Implemented services

```text
board_time_stm32f103.c       DWT-based microsecond timebase and busy-wait delays
board_i2c_stm32f103.c        MPU bus on PB8/PB9 using open-drain software I2C
board_motor_stm32f103.c      TIM3 PWM and direction outputs for the three motor paths
board_encoder_stm32f103.c    TIM2/TIM4 quadrature encoder capture
stm32f103_regs.h             Minimal peripheral-register definitions used by this layer
```

The implementation assumes a 72 MHz core clock. `board_time_us()` derives time from the Cortex-M3 DWT cycle counter.

## Important schematic details

The reference schematic routes:

- `MPU_SDA` to **PB8** and `MPU_SCL` to **PB9**. The STM32F103 I2C1 remap uses PB8 as SCL and PB9 as SDA, so this PCB cannot use hardware I2C1 for the MPU6050 without crossing the two signals. The platform therefore uses software I2C on the actual board routing.
- The net named `MPU_INT` from **PC13** is connected to MPU6050 **FSYNC (pin 11)**. MPU6050 **INT (pin 12) is marked no-connect**. The reference PCB therefore has no MPU6050 data-ready interrupt path to the MCU.
- MPU6050 `AD0` is pulled low through R11, selecting the 7-bit address `0x68`.

## Motor and encoder routing

The schematic captions identify:

- side brushless connector: `BLDC_1`, PWM PB1/TIM3_CH4, DIR PB11, Encoder 1 PA1/PA0,
- front/back brushless connector: `BLDC_2`, PWM PA6/TIM3_CH1, DIR PA4, Encoder 2 PB7/PB6,
- spin brushless connector: `BLDC_3`, PWM PB0/TIM3_CH3, DIR PB10, Brake PA7.

`EN_BLDC_1`, `EN_BLDC_2`, and `EN_BLDC_3` are tied directly to 3.3 V at their connectors; they are not MCU-controlled enable signals. `board_motor_command_t.enable` therefore gates the PWM request rather than toggling a physical enable pin.

The spin connector exposes `Ecoder_3_A/B`, but those signals are not routed to MCU pins in this schematic, so `BOARD_ENCODER_SPIN` is unsupported on the reference PCB.

PWM active level is not encoded by the schematic itself. The current board implementation preserves the established electrical convention: BLDC_1 and BLDC_2 use active-low PWM command waveforms, while BLDC_3 uses active-high PWM. This should be checked at the connector before the first powered motor run on a newly assembled board.

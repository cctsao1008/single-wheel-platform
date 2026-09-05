# STM32F103 Reference Platform

This directory contains the concrete STM32F103 implementation of the shared board APIs for the reference single-wheel hardware.

## Implemented services

```text
board_time_stm32f103.c       DWT-based microsecond timebase and busy-wait delays
board_i2c_stm32f103.c        MPU bus on PB8/PB9 using open-drain software I2C
board_gpio_irq_stm32f103.c   MPU_INT on PC13 through EXTI13
board_motor_stm32f103.c      TIM3 PWM and direction outputs for the three motor paths
board_encoder_stm32f103.c    TIM2/TIM4 quadrature encoder capture
stm32f103_regs.h             Minimal peripheral-register definitions used by this layer
```

The implementation assumes the reference MCU runs at 72 MHz. `board_time_us()` derives time from the Cortex-M3 DWT cycle counter and extends it into a 32-bit microsecond timebase; active firmware must call the time service more often than one raw DWT wrap.

## Board mapping

The current pin mapping follows the reference ONE_V2.0 schematic:

- MPU6050 SDA/SCL: PB8/PB9
- MPU6050 INT: PC13
- reaction-wheel PWM/direction: PB1 TIM3_CH4 / PB11
- drive-wheel PWM/direction: PA6 TIM3_CH1 / PA4
- spin PWM/direction: PB0 TIM3_CH3 / PB10
- reaction encoder: PA0/PA1 through TIM2
- drive encoder: PB6/PB7 through TIM4

The spin encoder signals are present at the motor connector but are not routed to MCU encoder inputs in the reference schematic, so `BOARD_ENCODER_SPIN` is currently unsupported.

Motor electrical polarity is isolated in `board_motor_stm32f103.c`; coordinate/sign convention above the board layer remains the responsibility of the actuator and coordinate mapping layers.

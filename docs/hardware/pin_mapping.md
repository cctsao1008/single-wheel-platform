# Hardware / Firmware Mapping

This document records the logical-to-physical mapping used by the STM32F103 reference platform. The mapping is based on the ONE_V2.0 reference schematic; software sign conventions above the board layer are kept separate from electrical pin polarity.

| Logical function | MCU resource | Reference net / connector role | Status |
|---|---|---|---|
| Reaction-wheel PWM | PB1 / TIM3_CH4 | `PWM_BLDC_1`, side brushless motor interface | Mapped |
| Reaction-wheel direction | PB11 | `DIR_BLDC_1` | Mapped |
| Reaction encoder A/B | PA1 / PA0 | `Ecoder_1_A` / `Ecoder_1_B` | Mapped |
| Drive-wheel PWM | PA6 / TIM3_CH1 | `PWM_BLDC_2`, front/back brushless motor interface | Mapped |
| Drive-wheel direction | PA4 | `DIR_BLDC_2` | Mapped |
| Drive encoder A/B | PB7 / PB6 | `Ecoder_2_A` / `Ecoder_2_B` | Mapped |
| Spin PWM | PB0 / TIM3_CH3 | `PWM_BLDC_3` | Mapped |
| Spin direction | PB10 | `DIR_BLDC_3` | Mapped |
| Spin brake | PA7 | `Brake` | Pin mapped; electrical polarity not frozen |
| Spin encoder A/B | Not routed to MCU encoder timer | `Ecoder_3_A` / `Ecoder_3_B` at connector | Unsupported by current board API implementation |
| MPU6050 SDA / SCL | PB8 / PB9 | `MPU_SDA` / `MPU_SCL` | Mapped |
| MPU6050 interrupt | PC13 / EXTI13 | `MPU_INT` | Mapped |
| Battery ADC | PA5 / ADC1_IN5 | `ADC` | Mapped; ADC implementation pending |

## Electrical behavior

The reference motor implementation uses TIM3 with a 72 MHz timer clock and a 7200-count period, corresponding to 10 kHz PWM. The reaction-wheel PWM path uses the board's inverted electrical convention, while drive and spin PWM use the non-inverted convention. This electrical translation is contained entirely inside the platform motor implementation.

Encoder polarity is intentionally not converted into robot-axis sign here. `board_encoder_read_delta()` returns hardware-domain signed counts; the coordinate-mapping layer defines the robot-positive direction.

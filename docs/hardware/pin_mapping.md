# Pin Mapping

## Motor and encoder channels

| Function | MCU resource | Connector / net |
|---|---|---|
| BLDC_1 PWM | PB1 / TIM3_CH4 | `PWM_BLDC_1`, PCB `M2` |
| BLDC_1 direction | PB11 | `DIR_BLDC_1`, PCB `M2` |
| BLDC_1 enable | hard-wired 3.3 V | `EN_BLDC_1` |
| Encoder_1 B | PA0 / TIM2_CH1 | `Ecoder_1_B` |
| Encoder_1 A | PA1 / TIM2_CH2 | `Ecoder_1_A` |
| BLDC_2 PWM | PA6 / TIM3_CH1 | `PWM_BLDC_2`, PCB `M1` |
| BLDC_2 direction | PA4 | `DIR_BLDC_2`, PCB `M1` |
| BLDC_2 enable | hard-wired 3.3 V | `EN_BLDC_2` |
| Encoder_2 B | PB6 / TIM4_CH1 | `Ecoder_2_B` |
| Encoder_2 A | PB7 / TIM4_CH2 | `Ecoder_2_A` |
| BLDC_3 PWM | PB0 / TIM3_CH3 | `PWM_BLDC_3`, PCB `M3` |
| BLDC_3 direction | PB10 | `DIR_BLDC_3`, PCB `M3` |
| BLDC_3 brake | PA7 | `Brake`, PCB `M3` |
| BLDC_3 enable | hard-wired 3.3 V | `EN_BLDC_3` |
| Encoder_3 A/B | no MCU route | BLDC_3 connector only |

Robot-role mapping:

```text
BLDC_1 / Encoder_1 -> ReactionWheel
BLDC_2 / Encoder_2 -> DriveWheel
BLDC_3             -> unused
```

## MPU6050

| Function | MCU resource | Net |
|---|---|---|
| SDA | PB8 | `MPU_SDA` |
| SCL | PB9 | `MPU_SCL` |
| FSYNC | PC13 | schematic net `MPU_INT` |
| INT | not routed | MPU6050 pin 12 |
| AD0 | pulled low | address `0x68` |

PB8/PB9 are used through software I2C.

## Battery ADC

| Function | MCU resource | Net |
|---|---|---|
| Battery ADC | PA5 / ADC1_IN5 | `ADC` divider node |

## Communication and UI

| Function | MCU resource | Net / device |
|---|---|---|
| USART1 TX | PA9 | `TX` |
| USART1 RX | PA10 | `RX` |
| Bluetooth TX | PA2 / USART2_TX | `T_TX` -> ECB02S2 RX |
| Bluetooth RX | PA3 / USART2_RX | `T_RX` <- ECB02S2 TX |
| Bluetooth AT_EN | PC15 | `EN_AT` |
| Bluetooth ROLE | PC14 | `ROLE` |
| OLED SDA | PB4 | `OLED_SDA` |
| OLED SCL | PB5 | `OLED_SCL` |
| SWDIO | PA13 | `SDIO` |
| SWCLK | PA14 | `SCLK` |
| EN_X | PA15 | `EN_X` |
| EN_Y | PB3 | `EN_Y` |

`EN_X` / `EN_Y` are runtime inputs. They are not the motor-interface `EN_BLDC_*` nets.

## P2 header

```text
1  RX
2  CH340_TX
3  TX
4  CH340_RX
5  BOOT0
6  3.3V
7  EN_Y
8  GND
9  EN_X
10 GND
11 SCLK / SWCLK
12 SDIO / SWDIO
13 PA12
14 PA11
15 PA8
16 PB15
17 3.3V
18 3.3V
19 GND
20 GND
```

The MCU USART1 and CH340 nets are separate on P2. A CH340 bridge uses crossed UART routing:

```text
MCU TX -> CH340_RX
MCU RX <- CH340_TX
```

## Electrical configuration boundary

PWM active polarity, direction polarity, brake polarity, encoder-positive direction, encoder mechanical scale, and battery-voltage scale are explicit actuator/sensor configuration parameters. They are not inferred from pin names.

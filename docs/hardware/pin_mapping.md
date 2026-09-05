# Pin Mapping

## Motor and encoder channels

| Function | MCU resource | Connector / net | Vendor V2.0 source use |
|---|---|---|---|
| BLDC_1 PWM | PB1 / TIM3_CH4 | `PWM_BLDC_1`, PCB `M2` | `PWM_Y = TIM3->CCR4` |
| BLDC_1 direction | PB11 | `DIR_BLDC_1`, PCB `M2` | `DIR_Y = PB11` |
| BLDC_1 enable | hard-wired 3.3 V | `EN_BLDC_1` | no GPIO control |
| Encoder_1 B | PA0 / TIM2_CH1 | `Ecoder_1_B` | TIM2 encoder mode |
| Encoder_1 A | PA1 / TIM2_CH2 | `Ecoder_1_A` | TIM2 encoder mode |
| BLDC_2 PWM | PA6 / TIM3_CH1 | `PWM_BLDC_2`, PCB `M1` | `PWM_X = TIM3->CCR1` |
| BLDC_2 direction | PA4 | `DIR_BLDC_2`, PCB `M1` | `DIR_X = PA4` |
| BLDC_2 enable | hard-wired 3.3 V | `EN_BLDC_2` | no GPIO control |
| Encoder_2 B | PB6 / TIM4_CH1 / EXTI6 | `Ecoder_2_B` | active source uses EXTI6 falling-edge pulse capture; TIM4 QEI implementation also exists |
| Encoder_2 A | PB7 / TIM4_CH2 | `Ecoder_2_A` | active source reads PB7 as direction; TIM4 QEI implementation also exists |
| BLDC_3 PWM | PB0 / TIM3_CH3 | `PWM_BLDC_3`, PCB `M3` | `PWM_Z = TIM3->CCR3` |
| BLDC_3 direction | PB10 | `DIR_BLDC_3`, PCB `M3` | `DIR_Z = PB10` |
| BLDC_3 brake | PA7 | `Brake`, PCB `M3` | PA7 PWM setup is present only as commented code; brake is not actively driven |
| BLDC_3 enable | hard-wired 3.3 V | `EN_BLDC_3` | no GPIO control |
| Encoder_3 A/B | no MCU route | BLDC_3 connector only | no active MCU capture |

Robot-role mapping:

```text
BLDC_1 / Encoder_1 -> ReactionWheel
BLDC_2 / Encoder_2 -> DriveWheel
BLDC_3             -> unused
```

The vendor V2.0 executable control dataflow is consistent with the encoder-to-motor pairing above:

```text
TIM2 / Encoder_1 -> Encoder_y -> Moto_y -> PWM_Y / TIM3_CH4 / BLDC_1
PB6/PB7 / Encoder_2 -> Encoder_x -> Moto_x -> PWM_X / TIM3_CH1 / BLDC_2
```

Some comments in the vendor source describe the two encoder roles inconsistently. Canonical association follows electrical routing, executable control dataflow, and reference-assembly mapping rather than those comments.

The current Rust runtime uses TIM2 QEI for Encoder_1 and TIM4 QEI for Encoder_2. This intentionally differs from the vendor V2.0 active implementation for Encoder_2, which uses PB6 EXTI6 plus PB7 direction sensing.

## MPU6050

| Function | MCU resource | Net / electrical state |
|---|---|---|
| SDA | PB8 | `MPU_SDA` |
| SCL | PB9 | `MPU_SCL` |
| INT | PC13 / EXTI13 | `MPU_INT` |
| FSYNC | hard-wired low | GND |
| AD0 | pulled low | address `0x68` |

PB8/PB9 are used through software I2C in both the vendor V2.0 source and the current Rust runtime.

The MPU6050 interrupt route is physically available at PC13. The vendor V2.0 firmware does not configure PC13/EXTI13, and the current Rust runtime also leaves DATA_RDY disabled at present. Hardware capability and instantiated runtime behavior are therefore distinct.

## Battery ADC

| Function | MCU resource | Net |
|---|---|---|
| Battery ADC | PA5 / ADC1_IN5 | `ADC` divider node |

The vendor implementation configures PA5 as analog input and actually reads `Get_Adc(5)`. Its header also contains a stale `Battery_Ch 6` macro; that macro is not canonical because it conflicts with both the schematic and executable ADC path.

## Communication, controls, and UI

| Function | MCU resource | Net / device | Vendor V2.0 source use |
|---|---|---|---|
| USART1 TX | PA9 | `TX` | active, 9600 baud in `USER/main.c` |
| USART1 RX | PA10 | `RX` | active |
| Bluetooth TX | PA2 / USART2_TX | `T_TX` -> ECB02S2 RX | active, 115200 baud |
| Bluetooth RX | PA3 / USART2_RX | `T_RX` <- ECB02S2 TX | active |
| Bluetooth AT_EN | PC15 | `EN_AT` | not configured by vendor main runtime |
| Bluetooth ROLE | PC14 | `ROLE` | not configured by vendor main runtime |
| OLED SDA | PB4 | `OLED_SDA` | active GPIO output |
| OLED SCL | PB5 | `OLED_SCL` | active GPIO output |
| SW2 button | PB12 | `PB12`, switch to GND | `KEY1`, input pull-up, active low |
| SW4 button | PB13 | `PB13`, switch to GND | `KEY2`, input pull-up, active low |
| D2 status LED | PB14 | PB14 -> R5 -> D2 -> GND | `LED = PB14` |
| EN_X | PA15 | `EN_X` | input pull-up, low treated as asserted |
| EN_Y | PB3 | `EN_Y` | input pull-up, low treated as asserted |
| SWDIO | PA13 | `SDIO` | SWD retained |
| SWCLK | PA14 | `SCLK` | SWD retained |

PB3, PB4, and PA15 overlap STM32F103 JTAG functions. The vendor V2.0 main runtime explicitly switches AFIO to SWD-only before initializing `EN_Y`, OLED, and `EN_X`. Any runtime that uses these GPIOs must likewise release the JTAG pins while preserving SWD.

The vendor archive also contains a USART3 implementation on PB10/PB11. `USER/main.c` does not call `uart3_init()`, and those pins are simultaneously the active BLDC direction pins, so USART3 is not part of the canonical board runtime interface.

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

PWM active polarity, direction polarity, brake polarity, encoder-positive direction, encoder mechanical scale, battery-voltage scale, and interrupt polarity/clear behavior are explicit actuator/sensor configuration parameters. They are not inferred from pin names.

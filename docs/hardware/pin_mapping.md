# Hardware / Firmware Mapping

This document records the schematic-level mapping of the ONE_V2.0 reference board. Software axis signs and control semantics are kept separate from electrical net names.

## MCU and peripheral mapping

| Schematic function | MCU resource | Net / connector detail | Status |
|---|---|---|---|
| BLDC_1 PWM | PB1 / TIM3_CH4 | `PWM_BLDC_1`, PCB silk `M2` | Schematic + assembly mapped |
| BLDC_1 direction | PB11 | `DIR_BLDC_1`, PCB silk `M2` | Schematic + assembly mapped |
| BLDC_1 enable | Not MCU-controlled | `EN_BLDC_1`, connector pin 3 tied to 3.3 V | Hard-wired high |
| Encoder 1 A/B | PA1 / PA0 | `Ecoder_1_A/B`, same BLDC_1 / PCB `M2` harness | Schematic + assembly mapped |
| BLDC_2 PWM | PA6 / TIM3_CH1 | `PWM_BLDC_2`, PCB silk `M1` | Schematic + assembly mapped |
| BLDC_2 direction | PA4 | `DIR_BLDC_2`, PCB silk `M1` | Schematic + assembly mapped |
| BLDC_2 enable | Not MCU-controlled | `EN_BLDC_2`, connector pin 3 tied to 3.3 V | Hard-wired high |
| Encoder 2 A/B | PB7 / PB6 | `Ecoder_2_A/B`, same BLDC_2 / PCB `M1` harness | Schematic + assembly mapped |
| BLDC_3 PWM | PB0 / TIM3_CH3 | `PWM_BLDC_3`, PCB silk `M3`, schematic `CN1` | Schematic + assembly mapped |
| BLDC_3 direction | PB10 | `DIR_BLDC_3`, PCB silk `M3` | Schematic + assembly mapped |
| BLDC_3 brake | PA7 | `Brake`, PCB silk `M3` | Pin mapped; active polarity not specified |
| BLDC_3 enable | Not MCU-controlled | `EN_BLDC_3`, connector pin 6 tied to 3.3 V | Hard-wired high |
| Encoder 3 A/B | No MCU route shown | `Ecoder_3_A/B`, BLDC_3 connector | Connector-only |
| MPU6050 SDA | PB8 | `MPU_SDA`; 4.7 kOhm pull-up to 3.3 V | Schematic-mapped |
| MPU6050 SCL | PB9 | `MPU_SCL`; 4.7 kOhm pull-up to 3.3 V | Schematic-mapped |
| MPU6050 AD0 | N/A | R11 pulls AD0 low | Address 0x68 |
| Net `MPU_INT` | PC13 | **Connected to MPU6050 FSYNC pin 11** | Net name is misleading |
| MPU6050 INT | No MCU route | MPU6050 pin 12 is marked no-connect | No data-ready IRQ path |
| Battery ADC | PA5 / ADC1_IN5 | Divider node `ADC` between R2/R4 | Pin mapped; divider values not given in this drawing |
| Bluetooth UART TX/RX | PA2 / PA3 | MCU `T_TX` / `T_RX` to ECB02S2 RXD/TXD | Schematic-mapped |
| Bluetooth AT enable | PC15 | `EN_AT` to ECB02S2 `AT_EN` | Schematic-mapped |
| Bluetooth role select | PC14 | `ROLE` to ECB02S2 `ROLE` | Schematic-mapped |
| Bluetooth sleep | Not MCU-controlled | ECB02S2 sleep input tied low | Module held awake |
| Main UART TX/RX | PA9 / PA10 | `TX` / `RX`, exposed on P2 pins 3/1 | Schematic-mapped |
| CH340N UART | Not hard-wired to MCU UART | `CH340_TX` / `CH340_RX`, exposed separately on P2 pins 2/4 | External cross-connection required for bridge use |
| OLED SDA/SCL | PB4 / PB5 | `OLED_SDA` / `OLED_SCL`; legacy software uses two-wire SSD130x-style traffic | Schematic + source evidence |
| EN_X jumper input | PA15 | Board silk `ENX` | Pin mapped; actuator semantics not yet promoted |
| EN_Y jumper input | PB3 | Board silk `ENY` | Pin mapped; actuator semantics not yet promoted |
| SWDIO/SWCLK | PA13 / PA14 | Nets `SDIO` / `SCLK`, exposed on P2 | Schematic-mapped |

## Verified connector / actuator topology — 2026-09-05

The physical PCB silk, schematic connector details, and manual cable tracing establish the following mapping:

```text
PCB M2 -> schematic BLDC_1 -> upper reaction-wheel motor
PCB M1 -> schematic BLDC_2 -> lower Nidec ground-drive motor
PCB M3 -> schematic BLDC_3 -> no motor installed
```

The schematic component designators are worth noting because they are not a simple `M1/M2/M3` sequence:

```text
BLDC_1 connector component: M2
BLDC_2 connector component: CN2
BLDC_3 connector component: CN1
```

The corresponding feedback-path association is:

```text
Encoder_1 -> reaction-wheel motor path
Encoder_2 -> drive-wheel motor path
Encoder_3 -> no installed actuator and no MCU route shown
```

Encoder polarity, counts per mechanical revolution, and any gearing ratio remain unverified.

## Motor connector power versus EN_X / EN_Y

Each brushless connector provides `12V_P`, GND, and 3.3 V logic/encoder supply. The three `EN_BLDC_*` lines are not driven by the MCU; each is tied to 3.3 V directly on the schematic.

`EN_X` (PA15) and `EN_Y` (PB3) are different signals. Legacy firmware configures them as pulled-up inputs and uses their jumper state to gate which motor-control path is permitted. Product material labels them as motor-enable jumpers. Because legacy X/Y naming and product physical-role labels are not perfectly consistent, the board crate records the pins but does not assign them directly to `ReactionWheel` or `DriveWheel` yet.

The BLDC_3 / PCB M3 interface additionally exposes `Brake`. The schematic establishes only the PA7-to-Brake connection; it does not establish the brake input's active polarity. The inspected assembly has no motor connected to M3.

## MPU6050 bus consequence

The board labels PB8 as SDA and PB9 as SCL. STM32F103 I2C1 remap assigns PB8=SCL and PB9=SDA, so the PCB routing is reversed relative to the hardware I2C1 alternate function. The reference platform therefore uses software I2C on PB8/PB9.

The schematic also shows that PC13 cannot be used as an MPU6050 data-ready interrupt: the `MPU_INT` net terminates on FSYNC, while the actual INT pin is no-connect.

## Bluetooth consequence

The on-board ECB02S2 is a UART-transparent BLE module. From the MCU perspective:

```text
PA2 / USART2_TX -> ECB02 RXD
PA3 / USART2_RX <- ECB02 TXD
PC15            -> ECB02 AT_EN
PC14            -> ECB02 ROLE
SLEEP           -> GND (awake)
```

The module documentation gives 115200 baud as the default UART setting and describes peripheral/slave operation for a phone acting as BLE central. Those are device-configuration facts, not pin-map constants in `swp-board-one-v2`.

## UART / CH340 consequence

The MCU USART1 nets `TX` and `RX` and the CH340N nets `CH340_TX` and `CH340_RX` terminate on separate P2 pins. They are not shorted together on the schematic.

Using the onboard CH340 as the USART1 host bridge therefore requires the normal crossed UART relationship outside those nets:

```text
P2 pin 3 / MCU TX -> P2 pin 4 / CH340_RX
P2 pin 1 / MCU RX <- P2 pin 2 / CH340_TX
```

The current recording firmware transmits only on MCU PA9 / `TX`; it does not assume that USB-C/CH340 is already electrically connected to that net.

## OLED consequence

The board exposes only the two-wire OLED data/clock path on PB4/PB5. Legacy firmware uses PB4 as SDA and PB5 as SCL and sends the SSD130x-family write address corresponding to 7-bit address `0x3C`.

PB4 is also an STM32F103 JTAG pin (`JNTRST`) after reset. Firmware that uses PB4 for OLED traffic must release the full JTAG function while preserving SWD access as required by the target HAL/runtime configuration.

The supplied OLED source set is revision-mixed: product material mentions 0.96 inch while the inspected assembly carries a larger module and a separate 2.42-inch SSD1309 module schematic is available. Display size/driver revision is therefore kept outside the board pin facts until the installed module is identified directly.

## Electrical behavior not yet physically verified

The schematic does not define the installed brushless motor modules' PWM active level, direction sign, or robot-positive encoder direction. Those are board/actuator integration properties and must not be inferred from net names alone.

Private legacy firmware suggests BLDC_1/2 active-low PWM and historical direction conventions, but those remain source-derived evidence rather than verified electrical facts. Connector waveforms must be measured before first powered actuator commissioning.

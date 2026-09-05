# Hardware Platform

## Core hardware

```text
MCU             STM32F103C8T6
IMU             MPU6050 @ 0x68
Power           3S battery platform
```

## Actuation

```text
BLDC_1 / PCB M2 -> ReactionWheel
BLDC_2 / PCB M1 -> DriveWheel
BLDC_3 / PCB M3 -> unused
```

The reference assembly therefore contains exactly two actuator roles:

```text
ReactionWheel
DriveWheel
```

Encoder association is:

```text
Encoder_1 -> ReactionWheel
Encoder_2 -> DriveWheel
Encoder_3 -> unused / no MCU route
```

Board connector identity and robot actuator identity are separate definitions. `swp-board-one-v2` owns PCB wiring; `swp-reference-assembly` owns the installed channel-to-role mapping.

## Interfaces

```text
USART1              PA9 TX / PA10 RX
USART2 / ECB02S2    PA2 TX / PA3 RX
ECB02 AT_EN          PC15
ECB02 ROLE           PC14
OLED SDA             PB4
OLED SCL             PB5
EN_X                 PA15
EN_Y                 PB3
```

USART1 is the wired recording/engineering transport. USART2 is the BLE commissioning transport. OLED is the local status interface.

`EN_X` and `EN_Y` are MCU inputs and are distinct from `EN_BLDC_1/2/3`, which are hard-wired high at the motor interfaces.

## IMU wiring

```text
MPU_SDA      PB8
MPU_SCL      PB9
MPU_FSYNC    PC13
MPU_INT      not routed
AD0          low
I2C address  0x68
```

PB8/PB9 do not match the STM32F103 hardware-I2C1 remap polarity, so the platform uses software I2C.

Because MPU6050 INT is not routed, the device source-sample timestamp is not directly observable.

## Encoder wiring

```text
Encoder_1 B  PA0 / TIM2_CH1
Encoder_1 A  PA1 / TIM2_CH2
Encoder_2 B  PB6 / TIM4_CH1
Encoder_2 A  PB7 / TIM4_CH2
```

Encoder values enter the observation model as raw wrapping timer counts. Angular scale and robot-positive sign are configuration parameters rather than board-pin properties.

## Battery sensing

```text
Battery ADC  PA5 / ADC1_IN5
```

The runtime stores the raw ADC conversion. Voltage conversion is applied only through an explicit divider/ADC transfer function.

## Physical reference values

```text
vehicle envelope        105 x 70 x 150 mm
vehicle mass            570 g
battery nominal         11.1 V
battery full            12.6 V
battery mass            107 g
reaction-wheel motor    12 V / 10 W / 3000 rpm / 0.085 N·m / 1 A stall
 drive-wheel motor       12 V / 3000 rpm / 0.075 N·m / 1 A stall
encoder specification   100 lines per installed motor
```

`100 lines` is not treated as `100 counts/revolution`; quadrature decoding and mechanical scale are separate configuration values.

## Reaction-wheel authority

Reaction-wheel speed is part of the actuator-authority state. Runtime authority transitions through `Nominal`, `Warning`, and `Exhausted` speed-headroom states before electrical output is authorized.

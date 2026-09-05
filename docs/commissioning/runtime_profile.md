# STM32F103 Runtime Profile

## Clock

```text
SYSCLK   HSI 8 MHz
PCLK1    8 MHz
PCLK2    8 MHz
```

DWT cycle counting is the firmware monotonic timebase.

## Acquisition

```text
scheduler            TIM1
rate                 100 Hz
MPU6050 bus          software I2C on PB8/PB9
MPU6050 address      0x68
gyro range           +/-1000 dps
accelerometer range  +/-4 g
DLPF                 CONFIG=3
data-ready IRQ       disabled
Encoder_1            TIM2 QEI
Encoder_2            TIM4 QEI
battery              ADC1 / PA5 raw conversion
```

The MPU6050 source-sample timestamp is `Unknown`; I2C read start/completion timestamps are recorded.

Encoder values are raw wrapping timer counts. Battery values are raw ADC conversions.

## Recording transport

```text
USART2 TX      PA2
baud           115200
module         ECB02S2
transport      BLE transparent UART
queue          heapless SPSC
TX service     USART2 TXE interrupt
```

The acquisition task owns sampling and record enqueue. USART2 byte transmission runs at lower RTIC priority.

Each `RecordedObservation` is 80 bytes and CRC16-CCITT-FALSE protected. Sequence number and cumulative dropped-record count remain part of the record contract so the host can detect wireless loss and firmware queue pressure independently.

USART1 remains available as a wired engineering interface but is not the active recording transport in this runtime profile.

## Actuation state

```text
TIM3 motor PWM       not configured
motor direction GPIO not configured
BLDC_3 brake         not configured
```

The runtime profile is observation-only until actuator electrical configuration is instantiated through runtime authority.

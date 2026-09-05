# STM32F103 Runtime Profile

## Clock

```text
HSE      8 MHz crystal
SYSCLK   72 MHz
HCLK     72 MHz
PCLK1    36 MHz
PCLK2    72 MHz
ADCCLK   12 MHz
```

The external 8 MHz crystal is multiplied to the STM32F103C8T6 maximum 72 MHz system clock. APB1 remains within its 36 MHz limit; TIM2/TIM4 receive the doubled APB1 timer clock. ADC1 runs at 12 MHz, below the 14 MHz device limit.

DWT cycle counting is the firmware monotonic timebase.

## Acquisition

```text
scheduler            TIM1
inner acquisition    500 Hz / 2 ms
MPU6050 bus          software I2C on PB8/PB9
I2C target           400 kHz nominal bit timing
MPU6050 address      0x68
gyro range           +/-1000 dps
accelerometer range  +/-4 g
DLPF                 CONFIG=3
MPU6050 sample rate  500 Hz
data-ready IRQ       disabled
Encoder_1            TIM2 QEI
Encoder_2            TIM4 QEI
battery              ADC1 / PA5 raw conversion
```

With DLPF CONFIG=3, the MPU6050 measurement path is intentionally bandwidth-limited before the 500 Hz acquisition boundary. The resulting sensor-filter delay remains part of the measured control-path latency and must be included in later timing characterization.

The MPU6050 source-sample timestamp is `Unknown`; I2C read start/completion timestamps are recorded.

The 500 Hz TIM1 boundary is the target timing boundary for MPU acquisition, state estimation, and the roll/pitch inner balance loops. The current firmware instantiates only acquisition at this rate; estimator and motor-control stages remain disabled.

Encoder capture is currently performed at the 100 Hz canonical observation boundary. The architecture permits a separate 100-200 Hz encoder-velocity task when velocity estimation is instantiated.

Encoder values are raw wrapping timer counts. Battery values are raw ADC conversions.

## Recording transport

```text
RecordedObservation  100 Hz
USART2 TX             PA2
baud                  115200
module                ECB02S2
transport             BLE transparent UART
BLE telemetry         100 Hz current record stream
queue                 heapless SPSC
TX service            USART2 TXE interrupt
```

The 500 Hz acquisition task decimates by five for canonical `RecordedObservation` generation. USART2 byte transmission runs at lower RTIC priority.

Each `RecordedObservation` is 80 bytes and CRC16-CCITT-FALSE protected. At 100 Hz this is 8000 payload bytes/s. USART2 115200 8N1 provides 11520 byte/s line capacity, so the record stream occupies about 69% of the UART line before BLE-side buffering and scheduling effects.

ECB02S2 is treated as a byte-stream transport. Its configured MTU and sustained BLE notification pacing must be measured on the assembled platform; BLE packet boundaries are not record boundaries.

Sequence number and cumulative dropped-record count remain part of the record contract so the host can detect wireless loss and firmware queue pressure independently.

USART1 remains available as a wired engineering interface but is not the active recording transport in this runtime profile.

## Multi-rate target

```text
MPU6050 acquisition     500 Hz
state estimator         500 Hz
roll balance loop       500 Hz
pitch balance loop      500 Hz
encoder velocity        100-200 Hz
outer velocity loop     100 Hz
RecordedObservation     100 Hz
BLE telemetry           50-100 Hz
OLED                    10-20 Hz
```

Only the acquisition, recording, and USART2/BLE transport portions are instantiated in the current observation-only firmware.

## Actuation state

```text
TIM3 motor PWM       not configured
motor direction GPIO not configured
BLDC_3 brake         not configured
```

The runtime profile remains observation-only until actuator electrical configuration is instantiated through runtime authority.

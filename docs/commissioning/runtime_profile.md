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

The runtime configures an 8 MHz HSE and multiplies it to a 72 MHz system clock. APB1 runs at 36 MHz; TIM1/TIM2/TIM4 receive the doubled APB1 timer clock. ADC1 runs at 12 MHz. DWT cycle counting is the firmware monotonic timebase.

## Acquisition and timing health

```text
acquisition trigger     MPU6050 DATA_RDY -> PC13 / EXTI13
MPU6050 sample rate     500 Hz / nominal 2 ms
MPU6050 bus             software I2C on PB8/PB9
I2C target              400 kHz nominal bit timing
MPU6050 address         0x68
gyro range              +/-1000 dps
accelerometer range     +/-4 g
DLPF                    CONFIG=3
INT electrical mode     active-high / push-pull / pulse
health watchdog         TIM1 / 1 kHz, independent of DATA_RDY
late threshold          3 ms since previous DATA_RDY
hard timeout            6 ms since previous DATA_RDY
Encoder_1               TIM2 QEI
Encoder_2               TIM4 QEI
battery                 ADC1 / PA5 raw conversion
```

The MPU6050 DATA_RDY route is part of the current runtime. PC13 is configured as the EXTI13 source and the MPU6050 DATA_RDY interrupt is enabled after successful device configuration. A successful interrupt-triggered register read is therefore marked `FRESHNESS_VERIFIED`.

TIM1 is a separate 1 kHz liveness timebase. It does not drive acquisition. Its job is to detect that the DATA_RDY-driven path is late or absent even when the MPU6050 produces no interrupt at all. The timing monitor remains `Startup` until one complete inter-event interval has been observed, then classifies the primary sensor clock as `Healthy`, `Late`, or `Timeout`. Closed-loop runtime authority is eligible only while primary sensor timing is `Healthy`.

The 3 ms late threshold and 6 ms hard timeout are explicit runtime policy relative to the nominal 2 ms sample period. They are not physical sensor parameters.

`acquisition_started_us` is the DWT time observed at EXTI13 task entry. It is not promoted to the MPU6050 internal source-sample instant. The sensor samples and filters internally before DATA_RDY is asserted, and EXTI does not hardware-capture that internal instant. `source_sample_at_us` therefore remains `Unknown`; I2C read start/completion times are recorded independently.

With DLPF CONFIG=3, sensor-filter delay remains part of the control-path phase and must not be collapsed into the interrupt timestamp.

The DATA_RDY boundary is the 500 Hz acquisition boundary intended for state estimation and inner balance control. The current firmware instantiates acquisition and timing supervision; estimator and motor-control stages remain disabled.

Encoder capture and battery conversion are currently performed only on every fifth IMU event, at the 100 Hz canonical observation/record boundary. Encoder velocity may later run at its own 100-200 Hz boundary without changing the 500 Hz IMU path.

## Recording transport

```text
RecordedObservation  100 Hz
USART2 TX             PA2
baud                  115200
module                ECB02S2
transport             BLE transparent UART
queue                 heapless SPSC
DMA                   DMA1 channel 7 / USART2_TX
completion IRQ        DMA1_CHANNEL7
```

The 500 Hz DATA_RDY acquisition path decimates by five for canonical `RecordedObservation` generation. USART2 transmission runs at lower RTIC priority and transfers each queued record with DMA1 channel 7. The previous byte-rate USART2 TXE interrupt pump is not part of the current runtime.

Each `RecordedObservation` is 80 bytes and CRC16-CCITT-FALSE protected. At 100 Hz this is 8000 payload bytes/s. USART2 115200 8N1 provides 11520 byte/s line capacity, so the record stream occupies about 69% of the UART line before BLE-side buffering and scheduling effects. DMA reduces MCU interrupt load; it does not change the UART/BLE bandwidth limit.

ECB02S2 is treated as a byte-stream transport. BLE packet boundaries are not record boundaries. Sequence number and cumulative dropped-record count remain part of the record contract.

USART1 remains available as a wired engineering interface but is not the active recording transport.

DMA is applied where the board mapping provides a hardware request path and materially reduces CPU service load. The MPU6050 itself remains on software I2C because the board routes PB8 as SDA and PB9 as SCL, opposite the STM32F103 I2C1 remap assignment; that path cannot be converted into normal peripheral I2C DMA without changing the electrical routing. The 100 Hz single battery conversion likewise does not justify owning ADC DMA continuously.

## Multi-rate target

```text
TIM1 timing health      1 kHz
MPU6050 acquisition     500 Hz / DATA_RDY driven
state estimator         500 Hz
roll balance loop       500 Hz
pitch balance loop      500 Hz
encoder velocity        100-200 Hz
outer velocity loop     100 Hz
RecordedObservation     100 Hz
BLE telemetry           50-100 Hz
OLED                    10-20 Hz
```

The instantiated runtime contains the DATA_RDY acquisition boundary, independent timing health, recording, and USART2/DMA/BLE transport. State estimation and actuation remain later semantic stages, not hidden inside acquisition.

## Actuation state

```text
TIM3 motor PWM       not configured
motor direction GPIO not configured
BLDC_3 brake         not configured
```

The runtime remains observation-only until actuator electrical configuration is instantiated through runtime authority.

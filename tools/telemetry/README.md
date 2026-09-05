# Telemetry tools

The STM32F103 firmware emits fixed-size binary sensor-snapshot frames on USART1 TX (PA9) at 115200 baud. Transmission is interrupt-driven and fed by a lock-free SPSC queue, so UART byte transmission does not run inside the TIM1 acquisition task.

Each current snapshot contains the raw MPU6050 sample, Encoder 1 and Encoder 2 timer counts, the raw PA5 ADC conversion, status bits, and a DWT-derived timestamp. The decoder remains compatible with the earlier raw-IMU frame kind.

## Capture

Install pyserial once:

```text
py -m pip install pyserial
```

Then capture a raw stream:

```text
py tools/telemetry/capture.py COM5 sensors.bin
```

The schematic does **not** hard-wire MCU USART1 to the onboard CH340N. P2 exposes the pairs separately. To use the onboard CH340 as the host bridge, MCU TX must be externally routed to `CH340_RX`; MCU RX would likewise route to `CH340_TX` when receive support is added.

## Decode

```text
py tools/telemetry/decode.py sensors.bin > sensors.csv
```

or from stdin:

```text
type sensors.bin | py tools/telemetry/decode.py - > sensors.csv
```

The decoder scans for frame magic, validates protocol version, frame kind, payload length, and CRC-16/CCITT-FALSE, then emits CSV rows.

`encoder_1_count` and `encoder_2_count` are raw 16-bit quadrature timer counts. No robot-positive sign convention or counts-per-revolution scale is applied yet. `battery_adc_raw` is the raw ADC conversion; it is deliberately **not** converted to volts because the reviewed schematic does not provide the R2/R4 divider values.

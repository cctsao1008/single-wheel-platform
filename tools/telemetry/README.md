# Telemetry tools

The STM32F103 firmware emits fixed-size binary raw-IMU frames on USART1 TX (PA9) at 115200 baud. Transmission is interrupt-driven and fed by a lock-free SPSC queue, so UART byte transmission does not run inside the TIM1 acquisition task.

## Capture

Install pyserial once:

```text
py -m pip install pyserial
```

Then capture a raw stream:

```text
py tools/telemetry/capture.py COM5 imu.bin
```

The schematic does **not** hard-wire MCU USART1 to the onboard CH340N. P2 exposes the pairs separately. To use the onboard CH340 as the host bridge, MCU TX must be externally routed to `CH340_RX`; MCU RX would likewise route to `CH340_TX` when receive support is added.

## Decode

```text
py tools/telemetry/decode.py imu.bin > imu.csv
```

or from stdin:

```text
type imu.bin | py tools/telemetry/decode.py - > imu.csv
```

The decoder scans for frame magic, validates the protocol version, payload length, and CRC-16/CCITT-FALSE, then emits CSV rows.

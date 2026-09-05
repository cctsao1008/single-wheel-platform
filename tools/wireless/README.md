# Wireless Observation

The reference mobile platform streams canonical `RecordedObservation` bytes over the on-board ECB02S2 BLE link.

```text
STM32F103 USART2 @ 115200
        |
        | DMA1 channel 7
        v
ECB02S2
        |
        v
BLE notification
        |
        v
observe.py
        |
        +--> raw binary capture
        +--> live decode
        +--> sequence / CRC / drop statistics
        +--> IMU timing-health state
        +--> live raw sensor view
```

Each firmware record is 80 bytes. USART2 DMA boundaries and BLE packet boundaries are transport details only; records are reassembled from the continuous byte stream.

## Install

```bash
python -m pip install -r tools/wireless/requirements.txt
```

## Scan

```bash
python tools/wireless/observe.py --scan
```

## Capture

Select by advertised name:

```bash
python tools/wireless/observe.py --name ECB02
```

Select a specific device:

```bash
python tools/wireless/observe.py --address <BLE-address-or-identifier>
```

Specify the notification characteristic when the device exposes more than one notify-capable characteristic:

```bash
python tools/wireless/observe.py --address <device> --notify-uuid <uuid>
```

A raw binary capture is written by default as `swp-YYYYMMDD-HHMMSS.bin`. An explicit path can be supplied with `--output`.

```bash
python tools/wireless/observe.py --name ECB02 --output captures/run01.bin
```

The observer reports frame rate, primary IMU timing as `STARTUP` / `OK` / `LATE` / `TIMEOUT`, sequence gaps, CRC failures, firmware-reported dropped records, raw accelerometer/gyro values, encoder counts, and battery ADC values.

The ECB02 documentation does not define a GATT UUID in the module interface contract used by this repository. The host therefore discovers notify-capable characteristics at runtime and accepts an explicit UUID override.

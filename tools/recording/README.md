# Observation Recording and Replay

`swp-observation-record::RecordedObservation` defines the host recording contract.

```text
RawObservation
      |
      +----> semantic / control path
      |
      +----> RecordedObservation
                    |
                    v
                binary log
                    |
          +---------+---------+
          |                   |
          v                   v
       decode.py            replay.py
          |                   |
          v                   v
         CSV           deterministic JSONL
```

The active mobile-platform transport is USART2 through the ECB02S2 BLE link. Transport boundaries do not define observation-record boundaries.

## Decode

```bash
python3 tools/recording/decode.py capture.bin > capture.csv
```

Unknown timing values are preserved as empty fields.

## Replay

```bash
python3 tools/recording/replay.py capture.bin > replay.jsonl
python3 tools/recording/replay.py --strict capture.bin > replay.jsonl
```

Replay preserves record order, sequence gaps, timestamps, unknown states, and CRC validation. Host wall-clock speed is not used as measurement time.

The MPU6050 source-sample timestamp is `Unknown`; I2C read timing, encoder capture timing, ADC read timing, acquisition duration, and measurement quality remain available in the record.

Wireless capture is provided by [`../wireless/observe.py`](../wireless/observe.py), which writes the received byte stream directly to the canonical binary log while decoding a parallel live view.

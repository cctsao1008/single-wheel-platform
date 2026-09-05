# Observation Recording and Replay

The canonical host artifact is the binary `RecordedObservation` stream defined by `swp-observation-record`.

This is deliberately a **record/replay contract**, not a telemetry-domain model. UART is only the current transport used to move records off the STM32F103.

```text
physical acquisition
      |
      v
RawObservation
      |
      +----> estimator / future runtime stages
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

`decode.py` converts records to CSV while preserving unknown timing as empty fields. `replay.py` emits records in stored order without using host wall-clock time; downstream estimators must use recorded measurement timestamps rather than the speed at which replay executes.

The reference-board MPU6050 has no routed data-ready interrupt, so `imu_sample_time_us` is intentionally unknown. The record still preserves I2C read start/completion times, encoder capture times, ADC read completion time, acquisition duration, and per-measurement quality flags.

Examples:

```bash
python3 tools/recording/decode.py capture.bin > capture.csv
python3 tools/recording/replay.py capture.bin > replay.jsonl
python3 tools/recording/replay.py --strict capture.bin > replay.jsonl
```

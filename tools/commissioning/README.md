# Commissioning Tools

Host-side commissioning tools convert bounded physical experiments into reproducible evidence candidates. They do not write accepted physical parameters automatically.

```text
encoder_evidence.py    analyze manually marked one-revolution encoder trials from RecordedObservation captures
```

The encoder tool consumes the existing non-actuating STM32F103 `observation` firmware stream. It verifies record continuity, encoder quality, 16-bit wrap handling, bidirectional sign consistency, and repeated one-revolution count magnitude before emitting a candidate result.

The output remains evidence. Promotion into `parameters/reference-assembly.json` is a separate review step governed by `parameters/reference-assembly-evidence.json`.
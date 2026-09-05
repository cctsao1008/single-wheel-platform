# Timing Architecture

The control loop is a deterministic real-time path. Its rate is selected from measured sensor, encoder, actuator, computation, and closed-loop requirements rather than copied from another platform.

## Critical path

```text
timestamp
  -> sensor acquisition
  -> state estimation
  -> state validation
  -> control computation
  -> actuator mapping
  -> output safety
  -> motor command
```

The critical path must avoid:

- formatted text generation,
- blocking telemetry,
- display rendering,
- Flash erase/write,
- long protocol parsing,
- human-scale delays.

Timing documentation should eventually include:

- acquisition rate,
- estimator rate,
- controller rate,
- actuator update rate,
- worst-case execution time,
- jitter,
- timestamp semantics,
- encoder quantization at the selected period.

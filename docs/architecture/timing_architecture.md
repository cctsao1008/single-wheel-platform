# Timing Architecture

The control path is deterministic and bounded.

## Critical path

```text
timestamp
  -> sensor acquisition
  -> state estimation
  -> state validation
  -> control computation
  -> actuator allocation
  -> runtime authority
  -> electrical output
```

The critical path excludes:

- formatted text generation;
- blocking telemetry;
- BLE protocol work;
- display rendering;
- Flash erase/write;
- long command parsing;
- human-scale delays.

## Runtime timing

```text
acquisition schedule  100 Hz
record transport      lower RTIC priority than acquisition
measurement time      carried in observation data
```

Scheduler period does not replace measurement timestamps.

Timing characterization is expressed as runtime parameters and measurements:

```text
acquisition period
estimator period
controller period
actuator update period
WCET
jitter
measurement latency
encoder quantization over the selected interval
```

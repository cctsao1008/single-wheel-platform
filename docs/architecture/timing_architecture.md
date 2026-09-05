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

The current observation-only firmware instantiates the 500 Hz MPU6050 acquisition boundary and decimates canonical `RecordedObservation` generation to 100 Hz. Encoder capture is currently aligned with the 100 Hz recording boundary. Estimation, balance control, outer-loop control, motor actuation, and OLED service remain uninstantiated until their corresponding runtime stages are enabled.

USART2 record transport runs at lower RTIC priority than the 500 Hz acquisition task.

Scheduler periods do not replace measurement timestamps.

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

The 500 Hz inner path is independent of the 100 Hz canonical recording path. Lower-rate transport and display work must not enter the hard real-time balance path.

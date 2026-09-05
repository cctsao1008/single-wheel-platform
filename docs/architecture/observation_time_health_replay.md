# Observation Time, Measurement Health, and Replay

## Why these are architectural concerns

A balancing controller operates on a physical system, so three kinds of information are inseparable from the measured value itself:

1. **when the physical evidence was obtained**;
2. **what is known about its quality**;
3. **whether the same evidence can be replayed deterministically later**.

Treating these as optional diagnostics would allow scheduling and transport assumptions to leak into estimation and control.

## Time model

The architecture distinguishes:

```text
acquisition_started_us
    task begins acquiring a batch

source_sample_at_us
    physical/device sample time, only when actually known

read_started_at_us / captured_at_us
    software/peripheral observation timing

read_completed_at_us
    transfer/conversion readout completion

acquisition_completed_us
    complete batch is available to later runtime stages
```

The reference MPU6050 currently has `source_sample_at_us = Unknown`. This is intentional: polling a register at time T does not prove the sensor sampled the MEMS element at T.

Encoder counter snapshots are timestamped independently because each timer count is observed at a different CPU instant. ADC read completion is also preserved independently.

## Quality model

`MeasurementQuality` is an evidence bitset rather than a single validity flag:

```text
AVAILABLE             a value was produced
IO_OK                 the acquisition operation completed without I/O error
IO_ERROR              an attempted acquisition reported an I/O error
TIMING_VALID          source/capture timing is considered usable
FRESHNESS_VERIFIED    freshness was independently established
SATURATED             saturation/clipping was detected
STALE                 staleness was detected
RETRIED               acquisition required a retry
```

The absence of `FRESHNESS_VERIFIED` means freshness is not established; it does not automatically mean stale. The absence of `SATURATED` means saturation was not flagged; it does not claim that saturation detection was performed.

Platform facts such as bus readiness, MPU presence, and MPU configuration are represented separately by `AcquisitionStatus`.

## Estimator rule

The estimator must consume timestamped observations. It may use a scheduler period as a fallback only when the measurement contract explicitly permits that approximation. It must not overwrite unknown source time with task-entry time merely to obtain a convenient `dt`.

## Canonical record

`swp-observation-record::RecordedObservation` is the canonical replayable representation. It stores one `RawObservation`, a dropped-record counter, version/kind metadata, and CRC16-CCITT-FALSE.

To keep the live 100 Hz stream practical on the current UART, one absolute acquisition-start timestamp is stored and other times are encoded as offsets. Unknown offsets remain explicit sentinels. The record is currently 80 bytes.

The wire format is a recording contract, not the runtime data model. `RawObservation` may evolve internally without forcing estimator code to manipulate wire offsets.

## Replay invariants

A replay implementation must:

- preserve record order and recorded sequence numbers;
- preserve unknown timestamp/quality states;
- use recorded timestamps for estimator time evolution;
- report sequence gaps rather than silently interpolating records;
- not use host wall-clock speed as measurement time;
- permit the same record stream to be consumed by different estimator/controller revisions.

This makes real hardware captures suitable for regression tests and algorithm comparison rather than only for plotting.

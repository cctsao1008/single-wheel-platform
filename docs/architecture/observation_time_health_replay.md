# Observation Time, Measurement Health, and Replay

## Time model

The observation model distinguishes:

```text
acquisition_started_us
source_sample_at_us
read_started_at_us / captured_at_us
read_completed_at_us
acquisition_completed_us
```

`source_sample_at_us` is present only when the physical/device sample instant is known. The MPU6050 source-sample time is `Unknown` because the reference board does not route the device data-ready interrupt.

Encoder snapshots and ADC readout timing are stored independently.

## Measurement quality

`MeasurementQuality` is an independent bitset:

```text
AVAILABLE
IO_OK
IO_ERROR
TIMING_VALID
FRESHNESS_VERIFIED
SATURATED
STALE
RETRIED
```

An unset flag is not interpreted as proof of its opposite. Acquisition-platform state such as bus readiness, device presence, and device configuration belongs to `AcquisitionStatus`.

## Estimator contract

Estimator time evolution is derived from observation timing evidence. Scheduler period is not substituted for source time unless the estimator contract explicitly permits that approximation.

Unknown source time remains unknown.

## Record contract

`swp-observation-record::RecordedObservation` is the canonical replay representation:

```text
RawObservation
+ dropped-record count
+ record version / kind
+ CRC16-CCITT-FALSE
```

The binary record is fixed at 80 bytes. One absolute acquisition-start timestamp is stored; related times are represented as offsets with explicit unknown sentinels.

The record format is a transport-independent recording contract. Runtime semantic types do not depend on wire offsets or UART framing.

## Replay invariants

Replay preserves:

- record order;
- sequence numbers and gaps;
- timestamp values and unknown states;
- measurement-quality state;
- record corruption detection.

Replay time is recorded measurement time, not host wall-clock execution speed.

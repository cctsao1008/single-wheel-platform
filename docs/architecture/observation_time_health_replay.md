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

MPU6050 acquisition is driven by DATA_RDY through PC13 / EXTI13. `acquisition_started_us` is the DWT timestamp observed at EXTI task entry. A successful DATA_RDY-triggered read proves that a fresh output register image is available, but it does not expose the exact internal sensing/filtering instant. `source_sample_at_us` therefore remains `Unknown`.

I2C read start/completion times, encoder snapshots, and ADC readout timing are stored independently.

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

A successful DATA_RDY-triggered MPU read carries `AVAILABLE | IO_OK | FRESHNESS_VERIFIED`. `FRESHNESS_VERIFIED` is not equivalent to an exact source timestamp. An unset flag is not interpreted as proof of its opposite.

`AcquisitionStatus` separately represents platform/runtime state such as bus readiness, device presence, device configuration, and DATA_RDY IRQ enablement.

## Estimator contract

Estimator time evolution uses explicit timing evidence. Scheduler period, IRQ service time, source-sample time, and read-completion time are not interchangeable.

The 500 Hz MPU rate is the device configuration and acquisition cadence. The estimator may use that timing contract, but it must not fabricate a sensor-internal timestamp from the EXTI ISR timestamp. DLPF and sensor pipeline delay remain part of the measurement phase model.

## Health contract

DATA_RDY is also a liveness signal. Closed-loop authority must not depend indefinitely on the assumption that interrupts continue to arrive. Before actuation is enabled, runtime health must define an independent timeout/watchdog for missing DATA_RDY, stale IMU state, and missed control deadlines.

## Record contract

`swp-observation-record::RecordedObservation` is the canonical replay representation:

```text
RawObservation
+ dropped-record count
+ record version / kind
+ CRC16-CCITT-FALSE
```

The binary record is fixed at 80 bytes. One absolute acquisition-start timestamp is stored; related times are represented as offsets with explicit unknown sentinels.

The record format is transport-independent. Runtime semantic types do not depend on wire offsets or UART framing.

## Replay invariants

Replay preserves:

- record order;
- sequence numbers and gaps;
- timestamp values and unknown states;
- measurement-quality state;
- record corruption detection.

Replay time is recorded measurement time, not host wall-clock execution speed.

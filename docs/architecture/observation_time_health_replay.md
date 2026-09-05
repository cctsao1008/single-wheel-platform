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

A successful DATA_RDY-triggered MPU read carries `AVAILABLE | IO_OK | FRESHNESS_VERIFIED`. `TIMING_VALID` is added only when the primary sensor cadence has reached the `Healthy` timing state. `FRESHNESS_VERIFIED` is not equivalent to an exact source timestamp. An unset flag is not interpreted as proof of its opposite.

`AcquisitionStatus` separately represents platform/runtime state:

```text
BUS_READY
IMU_PRESENT
IMU_CONFIGURED
IMU_DATA_READY_IRQ_ENABLED
IMU_DATA_READY_SEEN
IMU_TIMING_HEALTHY
IMU_TIMING_LATE
IMU_TIMING_TIMEOUT
```

The timing bits describe the primary acquisition clock at the observation boundary; they do not replace per-measurement quality flags.

## Estimator contract

Estimator time evolution uses explicit timing evidence. Scheduler period, IRQ service time, source-sample time, and read-completion time are not interchangeable.

The 500 Hz MPU rate is the device configuration and acquisition cadence. The estimator may use that timing contract, but it must not fabricate a sensor-internal timestamp from the EXTI ISR timestamp. DLPF and sensor pipeline delay remain part of the measurement phase model.

## Health contract

DATA_RDY drives acquisition but does not supervise itself. TIM1 provides an independent 1 kHz liveness boundary and polls `SensorTimingMonitor` even when no MPU interrupt occurs.

The current timing policy is:

```text
nominal DATA_RDY period   2 ms
late                     >= 3 ms
hard timeout             >= 6 ms
```

One complete inter-event interval is required before timing becomes `Healthy`. `Startup`, `Late`, and `Timeout` are not eligible for closed-loop authority. This makes a missing sensor interrupt observable to runtime authority instead of allowing the control path to stop silently.

The late/timeout values are runtime safety policy relative to the configured 500 Hz acquisition period. They are not asserted as MPU6050 physical specifications.

## Transport scheduling

Recording is downstream of observation and does not define control timing. USART2 TX uses DMA1 channel 7. A queued 80-byte record generates one DMA completion boundary instead of one TXE service interrupt per byte.

DMA changes MCU service cost, not transport capacity. UART baud rate, BLE buffering, sequence continuity, queue pressure, and dropped-record evidence remain independent constraints.

## Record contract

`swp-observation-record::RecordedObservation` is the canonical replay representation:

```text
RawObservation
+ dropped-record count
+ record version / kind
+ CRC16-CCITT-FALSE
```

The binary record is fixed at 80 bytes. One absolute acquisition-start timestamp is stored; related times are represented as offsets with explicit unknown sentinels. Timing-health state is carried inside the existing `AcquisitionStatus` field, so the wire record does not change size or version.

The record format is transport-independent. Runtime semantic types do not depend on wire offsets, DMA boundaries, UART framing, or BLE packet framing.

## Replay invariants

Replay preserves:

- record order;
- sequence numbers and gaps;
- timestamp values and unknown states;
- measurement-quality state;
- acquisition/timing-health status;
- record corruption detection.

Replay time is recorded measurement time, not host wall-clock execution speed.

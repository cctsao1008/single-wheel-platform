# Typed Dataflow Architecture

Data moves between layers only when its semantic meaning changes.

```text
physical hardware
    |
    v
RawObservation
    |  raw values + timing + quality + acquisition status
    v
ScaledObservation
    |  device transfer functions assign SI units in sensor frame
    v
CalibratedObservation
    |  measured bias / scale / cross-axis correction
    v
BodyObservation
    |  sensor-to-body transform
    v
EstimatedState
    |  estimator output in platform-domain coordinates
    v
GeneralizedDemand
    |  control demand in physical axes
    v
ActuatorAllocation
    |  platform roles mapped to installed actuator channels
    v
RuntimeAuthority
    |  operating state, health, limits, reaction-wheel headroom
    v
ElectricalOutput
```

These are semantic boundaries; crate boundaries exist only where ownership or meaning requires them.

## Raw observation

`RawObservation` contains:

- acquisition start/completion time;
- MPU6050 raw accelerometer, gyroscope, and temperature words;
- MPU read start/completion time;
- source-sample timestamp evidence;
- Encoder_1 and Encoder_2 raw timer counts and capture times;
- raw battery ADC value and read timing;
- `MeasurementQuality`;
- `AcquisitionStatus`.

Raw observation does not imply simultaneity, freshness, calibration, or platform-frame meaning.

## Sensor semantics

```text
RawImuObservation
      |
      | MPU6050 transfer functions
      v
ScaledImuObservation
      |
      | measured affine calibration
      v
CalibratedImuObservation
      |
      | SensorToBodyRotation
      v
BodyImuObservation
```

Device scaling, physical calibration, and mechanical coordinate mapping are independent transitions.

## Recording branch

```text
RawObservation
      |\
      | +--> semantic / estimator / control path
      |
      +--> RecordedObservation --> storage / transport --> replay
```

`swp-observation-record` owns the persistent record format. UART does not own observation semantics.

## Board / assembly / platform mapping

```text
board-one-v2
    BLDC_1, BLDC_2, BLDC_3, Encoder_1, Encoder_2, pins, timers

reference-assembly
    BLDC_1 -> ReactionWheel
    BLDC_2 -> DriveWheel
    BLDC_3 -> unused

platform-domain
    body state, actuator roles, generalized demand
```

## Runtime properties

Semantic types are `no_std`, statically allocated, and heap-free. Wire-format layout is isolated from estimator/controller data structures.

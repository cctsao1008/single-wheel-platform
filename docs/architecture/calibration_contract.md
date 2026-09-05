# Calibration Contract

Calibration is the semantic boundary between device-scaled sensor values and physically calibrated sensor-frame values.

```text
RawImuObservation
      |
      | device transfer functions
      v
ScaledImuObservation
      |
      | measured affine calibration
      v
CalibratedImuObservation
      |
      | sensor-to-body rotation
      v
BodyImuObservation
```

## Device scaling

`swp-mpu6050` owns the MPU6050 nominal transfer functions for the configured full-scale ranges and temperature conversion. The result is expressed in SI units but remains in the native sensor frame.

Device scaling does not imply physical calibration.

## Measured calibration

`swp-sensor-calibration` applies:

```text
corrected = matrix * (scaled - bias)
```

The affine matrix may represent per-axis scale correction and cross-axis correction. Bias and matrix values are explicit parameters.

`CalibratedImuObservation` carries `CalibrationEvidence` with a revision and calibration basis.

## Boundary rules

Calibration does not own:

- sensor mounting orientation;
- body-frame axis mapping;
- encoder sign or scale;
- actuator-channel mapping;
- state estimation;
- controller tuning.

Timing and `MeasurementQuality` propagate through scaling and calibration unchanged. Unknown source time remains unknown.

No implicit identity profile exists. Zero bias and identity scale acquire calibration semantics only when supplied as an explicit measured calibration profile.

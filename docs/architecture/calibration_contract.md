# Calibration Contract

## Purpose

Calibration is a semantic boundary between raw electrical/register evidence and physically meaningful sensor-frame quantities. It is not the same operation as device scaling, mechanical mounting correction, coordinate mapping, state estimation, or controller tuning.

The architecture therefore keeps these transitions separate:

```text
RawObservation
    |
    | device transfer function
    v
ScaledImuObservation
    |
    | measured sensor calibration
    v
CalibratedImuObservation
    |
    | mechanical/frame mapping
    v
BodyObservation
```

Each transition may only assert meaning supported by the evidence available at that stage.

## Device scaling

The MPU6050 crate owns nominal transfer functions that are properties of the configured device:

- accelerometer counts per g for the selected full-scale range,
- gyro counts per degree/second for the selected full-scale range,
- nominal temperature transfer function.

Applying those transfer functions converts raw register values to SI units, but it does **not** prove that the sensor is physically calibrated.

For that reason the output type is `ScaledImuObservation`, not `CalibratedImuObservation`.

The values remain in the MPU6050 sensor frame.

## Measured calibration

`swp-sensor-calibration` applies a three-axis affine correction:

```text
corrected = matrix * (scaled - bias)
```

The bias and matrix are explicitly supplied calibration parameters. They can represent bias, per-axis scale correction, and cross-axis correction without embedding any assumption about how the sensor is mounted in the robot.

A calibrated observation carries `CalibrationEvidence` containing a revision and a basis such as bench-measured or imported-measured data. Software cannot prove that a physical calibration procedure was performed correctly; the purpose of the evidence object is to prevent anonymous constants from silently acquiring calibration authority.

## Mechanical mapping is not calibration

The following questions are deliberately outside the calibration layer:

- Which MPU axis corresponds to robot roll, pitch, or yaw?
- Is an axis inverted by mounting orientation?
- Is the board rotated relative to the body coordinate system?
- Which physical encoder or BLDC channel corresponds to a robot actuator role?

Those are integration/frame facts and belong to a later `BodyObservation` mapping step.

This distinction prevents a sensor calibration file from becoming an undocumented mixture of electrical correction and robot geometry.

## Timing and quality propagation

Scaling and calibration preserve measurement timing and quality evidence. They do not fabricate a source sample timestamp or promote an unavailable measurement into a valid physical quantity.

The reference board does not route the MPU6050 data-ready interrupt, so `source_sample_at_us` remains `Unknown` during the current bring-up path. The I2C read start/completion timestamps remain available for latency analysis and replay.

`MeasurementQuality` is also propagated without collapsing it to one boolean. A successful transfer-function conversion does not prove freshness, absence of clipping, or timing validity.

## Current executable boundary

The STM32F103 firmware continues to record `RawObservation` as the canonical acquisition evidence. It does not currently apply a measured calibration profile on target because no verified profile has been established for the physical unit.

The calibration crate is host- and target-buildable, covered by CI, and ready to become part of the target semantic path once measured parameters exist.

This avoids the common but incorrect bootstrap shortcut of treating zero bias and identity scale as a real calibration merely to make the pipeline executable.

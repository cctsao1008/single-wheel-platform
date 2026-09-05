# Body-Frame Contract

## Canonical body frame

The project uses one explicit right-handed body frame:

```text
+X = forward, along the ground-drive direction
+Y = left
+Z = up
```

This is a robot-domain convention, not a PCB or sensor-package convention.

## Semantic transition

IMU data may enter the body frame only after two independent transformations have evidence:

```text
MPU6050 register counts
        |
        v
sensor-frame SI values
        |
        | measured sensor calibration
        v
CalibratedImuObservation
        |
        | evidenced sensor-to-body rotation
        v
BodyImuObservation
```

Sensor calibration and mechanical orientation are deliberately separate. Bias/scale/cross-axis correction does not decide which physical direction is robot forward or up.

`swp-frame-transform` represents the sensor-to-body mapping as a proper 3-D rotation. Construction rejects scale, shear, non-orthogonal matrices, and handedness reflections. A frame transform also carries explicit evidence and revision metadata.

## Current reference-unit evidence

Legacy V2.0 behavior provides useful but incomplete orientation evidence:

- active firmware reads native MPU6050 X/Y/Z channels without an active axis swap;
- the legacy balancing equations are consistent with sensor +Z being approximately body-up at the upright equilibrium;
- the legacy X/Y balance loops correlate with the ground-drive and reaction-wheel control paths.

This does **not** yet establish the signs of sensor X/Y relative to canonical body +X forward and +Y left. Legacy comments contain naming drift and are therefore not promoted into a reference transform.

The repository intentionally publishes **no default `SensorToBodyRotation` for the reference assembly** until a physical tilt/rotation test or equivalent measured survey resolves the remaining signs.

## Commissioning test that closes the gap

With motor outputs disabled, record raw/calibrated IMU data while applying two unambiguous motions:

1. positive nose/drive-direction tilt around the lateral axis;
2. positive left/right tilt around the longitudinal axis.

The observed accelerometer and gyroscope signs establish the remaining axis correspondence. The resulting transform can then be recorded with `FrameEvidenceBasis::PhysicalTiltTest` and a revision number.

The transform is thereafter a configuration/evidence artifact, not a hidden sign convention inside an estimator or controller.

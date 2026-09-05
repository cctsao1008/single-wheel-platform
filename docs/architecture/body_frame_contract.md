# Body-Frame Contract

## Body frame

The robot uses one right-handed body frame:

```text
+X = forward, along the ground-drive direction
+Y = left
+Z = up
```

Rotations follow the right-hand rule:

```text
roll  = rotation about +X
pitch = rotation about +Y
yaw   = rotation about +Z
```

## Sensor-to-body transition

IMU data enters the body frame only through the explicit sensor-to-body transform:

```text
MPU6050 register counts
        |
        v
ScaledImuObservation
        |
        v
CalibratedImuObservation
        |
        | SensorToBodyRotation
        v
BodyImuObservation
```

Sensor calibration and mechanical orientation are separate operations. Calibration corrects bias, scale, and cross-axis error in the sensor frame. `SensorToBodyRotation` maps the calibrated sensor frame into the robot body frame.

`swp-frame-transform` accepts only proper 3-D rotations. Scale, shear, non-orthogonal transforms, and handedness reflections are rejected.

## Configuration contract

The sensor-to-body rotation is an explicit configuration value with frame evidence and revision metadata. It is never hidden inside estimator equations, controller signs, calibration matrices, or board pin definitions.

A reference-unit transform is not defined until the complete physical axis/sign mapping is available. Until then, body-frame promotion is unavailable rather than approximated.

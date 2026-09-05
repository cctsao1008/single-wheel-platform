# Coordinate Convention

The platform coordinate system is right-handed:

```text
+X = forward
+Y = left
+Z = up
```

Angular coordinates follow the right-hand rule:

```text
roll  = rotation about +X
pitch = rotation about +Y
yaw   = rotation about +Z
```

Platform coordinates are independent of PCB coordinates, sensor-package coordinates, encoder phase order, and motor electrical polarity.

Mappings into this coordinate system are explicit configuration boundaries:

```text
sensor frame  -> body frame
encoder count -> actuator angular position / speed
actuator role -> board motor channel
command sign  -> electrical PWM / direction state
```

No controller equation defines or overrides these mappings.

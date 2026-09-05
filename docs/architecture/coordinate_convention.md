# Coordinate Convention

A single coordinate convention is required before controller signs, sensor axes, or motor polarities are treated as stable constants.

Proposed body frame:

```text
+x : forward
+y : left
+z : upward
```

Rotations follow the right-hand rule:

- **roll** — rotation about `+x`,
- **pitch** — rotation about `+y`,
- **yaw** — rotation about `+z`.

The final project convention must additionally define:

- positive reaction-wheel rotation,
- positive drive-wheel rotation,
- positive spin-actuator rotation,
- encoder-positive directions,
- motor-positive command directions,
- MPU6050 mounting transform.

These mappings belong in hardware / calibration contracts rather than being scattered through controller equations.

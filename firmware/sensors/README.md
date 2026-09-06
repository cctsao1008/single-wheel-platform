# Sensors

`sensors/` contains reusable sensing-device protocol and transfer-function logic.

A sensor crate owns the concrete sensing device identity and the rules required to obtain/scale its native measurements. It does not own board pin routing, mounting orientation, body-frame mapping, calibration evidence, state estimation, or control policy.

Current implementation:

```text
mpu6050/
```

Higher-level calibration and semantic projection remain under `firmware/adapters/`.

# Control Layer

`control/` contains platform-independent state estimation, control, actuator mapping, and control-domain safety.

```text
sensor sample
    -> state estimation
    -> state validation
    -> roll / pitch / yaw control
    -> actuator mapping
    -> output safety
    -> actuator request
```

The control layer operates only on explicit physical/control-domain types and must not depend on STM32 headers, GPIO names, timer channels, or connector labels.

# Actuators

`actuators/` contains reusable actuator-interface electrical/protocol semantics.

An actuator crate converts `AuthorizedActuation` into the concrete frame required by the installed actuator hardware while remaining independent of the selected MCU. Target-specific peripheral mutation belongs under `firmware/targets/` through `ActuatorIo<Frame>`.

Current implementation:

```text
one-v2-pwm-dir/
```

This category intentionally replaces a generic `drivers/` bucket: sensor, communication, UI, and actuator code are classified by system role rather than by the fact that they all contain low-level driver logic.

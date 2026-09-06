# Plant

`plant/` defines the portable physical truth of the robot.

It owns robot-domain types, nonlinear and reduced plant models, measurement physics, raw observation semantics, and actuator physical models. It does not own MCU peripherals, scheduling, controller policy, or output authority.

```text
x_dot = f(x, u, p)
y     = h(x, u, p)
```

Detailed contracts:

- [`model.md`](model.md)
- [`measurement.md`](measurement.md)
- [`actuator.md`](actuator.md)
- [`body_frame.md`](body_frame.md)
- [`coordinate_convention.md`](coordinate_convention.md)

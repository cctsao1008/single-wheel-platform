# Firmware

`firmware/` is the physical execution and target-composition domain.

It owns device drivers, sensor-to-semantic adapters, board and assembly binding, concrete peripheral ownership, scheduling, telemetry integration, and electrical output. Firmware may compose Plant, Supervisor, and Control because it is where an executable robot is created.

Current target family: STM32F103. A future RP2350 target belongs in the same domain and should reuse the portable Plant / Supervisor / Control contracts.

Detailed contracts:

- [`calibration.md`](calibration.md)
- [`estimator_input.md`](estimator_input.md)
- [`timing.md`](timing.md)
- [`electrical_output.md`](electrical_output.md)
- [`control_footprint_probe.md`](control_footprint_probe.md)

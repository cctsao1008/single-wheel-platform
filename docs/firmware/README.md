# Firmware

`firmware/` is the physical execution and target-composition domain. It is structured by system role so control boards and actuator hardware can vary independently without changing Plant / Supervisor / Control.

```text
firmware/
├── interfaces/       target-independent physical-I/O contracts
├── sensors/          sensing-device protocols and transfer functions
├── communications/   external communication modules/endpoints
├── ui/               reusable human-interface components
├── buses/            reusable bus implementations
├── actuators/        actuator electrical/protocol semantics
├── adapters/         hardware evidence -> platform semantics
├── boards/           control-board wiring and peripheral capability
├── assemblies/       installed robot-role/channel binding
└── targets/          MCU-specific executable composition and HAL ownership
```

The high-level actuation boundary is `ActuationSink`, which accepts only `AuthorizedActuation`. Actuator adapters emit actuator-specific frames through `ActuatorIo<Frame>`, and target backends implement how those frames reach GPIO, PWM, PIO, SPI, CAN, or another concrete peripheral.

Current composition:

```text
sensor      sensors/mpu6050
board       boards/one-v2
assembly    assemblies/one-v2-reference
actuator    actuators/one-v2-pwm-dir
target      targets/stm32f103
```

`communications/` and `ui/` are architectural homes for reusable external communication and human-interface components. Concrete implementations are added only when their hardware identity and reusable behavior are known; simple board-local GPIO remains board/target-owned until extraction has semantic value.

A future RP2350 control board belongs under `targets/rp2350/` with its own board description under `boards/`. A different motor-driver or actuator-interface board belongs under `actuators/`. Neither requires a new Plant, Supervisor, or Control architecture.

Detailed contracts:

- [`calibration.md`](calibration.md)
- [`estimator_input.md`](estimator_input.md)
- [`timing.md`](timing.md)
- [`actuation.md`](actuation.md)
- [`control_footprint_probe.md`](control_footprint_probe.md)

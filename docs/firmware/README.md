# Firmware

`firmware/` is the physical execution and target-composition domain. It is structured so control boards and motor-driver boards can vary independently without changing Plant / Supervisor / Control.

```text
firmware/
├── interfaces/      target-independent physical-I/O contracts
├── devices/         IC/device protocols and transfer functions
├── buses/           reusable bus implementations
├── adapters/        sensor/device evidence -> platform semantics
├── boards/          control-board wiring and peripheral capability
├── drivers/         motor-driver electrical/protocol semantics
├── assemblies/      installed robot-role/channel binding
└── targets/         MCU-specific executable composition and HAL ownership
```

The high-level actuation boundary is `ActuationSink`, which accepts only `AuthorizedActuation`. Driver adapters emit driver-specific frames through `DriverIo<Frame>`, and target backends implement how those frames reach GPIO, PWM, PIO, SPI, CAN, or another concrete peripheral.

Current composition:

```text
board      boards/one-v2
assembly   assemblies/one-v2-reference
driver     drivers/one-v2-pwm-dir
target     targets/stm32f103
```

A future RP2350 control board belongs under `targets/rp2350/` with its own board description under `boards/`. A different motor-driver board belongs under `drivers/`. Neither requires a new Plant, Supervisor, or Control architecture.

Detailed contracts:

- [`calibration.md`](calibration.md)
- [`estimator_input.md`](estimator_input.md)
- [`timing.md`](timing.md)
- [`actuation.md`](actuation.md)
- [`control_footprint_probe.md`](control_footprint_probe.md)

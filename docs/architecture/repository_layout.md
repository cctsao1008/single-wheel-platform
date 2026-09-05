# Repository Layout

```text
app/                     System orchestration and platform binding
control/                 Platform-independent control-domain logic
  estimation/            State-estimation algorithms
  controllers/           Control-policy implementations
  safety/                State and output qualification
drivers/                 Portable device-level protocol drivers
  mpu6050/               MPU6050 register/configuration driver
platform/api/            Board-level hardware contracts
platform/stm32f103/      STM32F103 reference implementation
telemetry/               Runtime trace and telemetry
tests/                   Host-side tests
tools/                   Replay, plotting, log decoding, system tools
docs/architecture/       Architecture contracts
docs/hardware/           Hardware baseline and mapping
docs/commissioning/      Bring-up and characterization notes
```

`platform/api/` exposes hardware services such as I2C, time, GPIO interrupts, encoders, ADC, UART, storage, and motor output. Device-specific register behavior belongs in `drivers/`.

Empty architectural concepts should not be represented by fake implementation claims. Directories become populated as their contracts or implementations are defined.

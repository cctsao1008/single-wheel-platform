# Repository Layout

```text
app/                     System orchestration and services
control/                 Platform-independent control-domain logic
  estimation/            State-estimation algorithms
  controllers/           Roll / pitch / yaw and future control policies
  safety/                State and output qualification
drivers/                 Device-level protocol drivers
platform/api/            Shared hardware contracts
platform/stm32f103/      STM32F103 reference implementation
telemetry/               Runtime trace and telemetry
tests/                   Host-side tests
tools/                   Replay, plotting, log decoding, system ID
docs/architecture/       Architecture contracts
docs/hardware/           Hardware baseline and mapping
docs/commissioning/      Bring-up and characterization notes
```

Empty architectural concepts should not be represented by fake implementation claims. Directories become populated as their contracts or implementations are defined.

# Platform Layer

The platform layer provides the hardware contract between the application/control stack and the physical target.

```text
control / app
     |
     v
platform/api
     |
     +-------------------+
     |                   |
     v                   v
stm32f103/          future targets
```

Board-specific pins, timers, channels, polarities, startup code, linker configuration, and MCU SDK dependencies belong below the concrete platform implementation.

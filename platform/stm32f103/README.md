# STM32F103 Reference Platform

This directory owns the concrete STM32F103 implementation of the shared board APIs.

Expected responsibilities include:

- 72 MHz clock and startup configuration,
- deterministic control timing,
- MPU6050 bus access,
- encoder capture / counting,
- PWM and direction outputs,
- battery ADC,
- UART transports,
- persistent parameter storage,
- safe motor-off behavior.

Pin and connector assignments remain uncommitted until they are reconciled with the actual reference hardware.

# Device Drivers

`drivers/` contains device-specific protocol and register implementations without robot control policy.

Current reference-device work includes:

```text
mpu6050/
  mpu6050.c
  mpu6050.h
  mpu6050_registers.h
```

Device drivers are portable C modules. Hardware access is supplied through injected transport or service callbacks that the application binds to `platform/api/` implementations.

MCU-specific pins, peripheral instances, interrupt wiring, and SDK dependencies remain in the concrete platform implementation.

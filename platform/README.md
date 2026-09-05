# Platform Layer

The platform layer provides hardware services and actuator access without exposing MCU-specific implementation details to portable code.

```text
            app/
          /   |   \
         v    v    v
   control/ drivers/ platform/api/
                 ^
                 |
        platform/stm32f103/
```

`platform/api/` defines board-level contracts such as I2C transactions, timestamps, GPIO interrupt delivery, encoder access, ADC access, serial transport, storage, and motor output.

`platform/stm32f103/` owns pins, timers, channels, alternate functions, peripheral initialization, interrupt wiring, startup code, linker configuration, and MCU SDK dependencies.

Device-specific behavior such as MPU6050 register configuration belongs in `drivers/`, not in a board-level IMU abstraction. The application binds portable driver transport callbacks to the selected platform services.

The platform API must not depend on `control/` types.

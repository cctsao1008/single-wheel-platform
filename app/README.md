# Application Layer

`app/` owns system orchestration and the binding between portable modules and the selected platform implementation.

Current application-side infrastructure includes `mpu6050_platform_binding.c`, which adapts the portable MPU6050 transport callbacks to `board_i2c` and `board_time` without making the MPU6050 driver depend on STM32-specific code.

The application layer will also own startup sequencing, runtime mode selection, control-loop scheduling, and coordination of non-critical services. It must not absorb device-register logic or controller implementation details.

# MPU6050 Driver

This directory contains the portable MPU6050 device driver used by the reference platform.

The driver owns MPU6050-specific register behavior and configuration. It does not own robot coordinate conventions, attitude estimation, or balancing policy.

## Boundary

```text
app / acquisition
      |
      v
   mpu6050
      |
      v
transport callbacks
      |
      v
platform/api
  board_i2c
  board_time
  board_gpio_irq
```

The application binds the driver transport callbacks to the selected platform implementation. This keeps the device driver independent of STM32-specific headers and also prevents `platform/api/` from depending on MPU6050-specific types.

## Configuration

The public configuration keeps gyro full-scale range, accelerometer full-scale range, DLPF setting, sample rate, and data-ready interrupt selection explicit. Register conversion scales are derived from the configured full-scale range rather than duplicated as unrelated constants elsewhere in the software.

Coordinate transforms and sensor mounting orientation belong above the device-driver boundary.

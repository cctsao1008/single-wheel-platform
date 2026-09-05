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
```

The application binds the driver transport callbacks to the selected platform implementation. This keeps the device driver independent of STM32-specific headers and prevents `platform/api/` from depending on MPU6050-specific types.

## Configuration

The public configuration keeps gyro full-scale range, accelerometer full-scale range, DLPF setting, sample rate, and the MPU6050 interrupt-enable register setting explicit. Register conversion scales are derived from the configured full-scale range rather than duplicated as unrelated constants elsewhere in the software.

The ONE_V2.0 reference PCB does **not** route MPU6050 INT (pin 12) to the MCU: the schematic net named `MPU_INT` is actually connected to FSYNC (pin 11), while INT is no-connect. Therefore `data_ready_interrupt` must remain disabled for this board unless the hardware is modified.

Coordinate transforms and sensor mounting orientation belong above the device-driver boundary.

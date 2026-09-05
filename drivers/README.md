# Device Drivers

`drivers/` contains device-level protocol implementations that are independent of robot control policy where practical.

Examples include MPU6050 register access or communication transports. MCU-specific bus and peripheral details remain behind `platform/api/` implementations.

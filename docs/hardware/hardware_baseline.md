# Hardware Baseline

Current reference hardware information supports the following platform-level components:

- **MCU:** STM32F103C8T6-class controller,
- **IMU:** MPU6050,
- **lateral actuator:** reaction-wheel mechanism,
- **longitudinal actuator:** ground-contact drive wheel,
- **third actuator path:** spin / yaw motor interface,
- **feedback:** encoder signals associated with motor / wheel motion,
- **communication:** UART-class interfaces including USB-UART / wireless serial capability,
- **local UI:** OLED-class display interface,
- **power:** nominal battery-powered embedded platform with motor and logic rails.

Exact connector numbering, MCU pin mapping, timer-channel mapping, motor polarity, and encoder polarity remain hardware-mapping properties and must be documented separately before controller code depends on them.

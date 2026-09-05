# Hardware Baseline

Current reference hardware information supports the following platform-level components:

- **MCU:** STM32F103C8T6-class controller,
- **IMU:** MPU6050,
- **lateral actuator mechanism:** reaction-wheel / flywheel assembly,
- **longitudinal actuator mechanism:** lower motor / ground-contact drive path pending final mechanical-role confirmation,
- **motor interfaces:** three PCB brushless connectors (`M1`, `M2`, `M3` / schematic `BLDC_1..3`),
- **current inspected assembly:** `M1` and `M2` are cabled; `M3` is physically present but unconnected,
- **feedback:** encoder signals associated with motor / wheel motion,
- **communication:** UART-class interfaces including USB-UART / wireless serial capability,
- **local UI:** OLED-class display interface,
- **power:** nominal battery-powered embedded platform with motor and logic rails.

The third motor channel is therefore a **PCB capability**, not an installed actuator in the currently inspected physical plant. The robot-domain model must not infer an actuator merely because a schematic connector exists.

Exact connector numbering, MCU pin mapping, timer-channel mapping, motor polarity, encoder polarity, and robot-role mapping remain hardware / mechanical integration properties and must be established before controller code depends on them.

See `assembly_observation_2026-09-05.md` for physical observations from the assembled robot.

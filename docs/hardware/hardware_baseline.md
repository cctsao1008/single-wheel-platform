# Hardware Baseline

Current reference hardware information supports the following platform-level components:

- **MCU:** STM32F103C8T6-class controller,
- **IMU:** MPU6050,
- **reaction-wheel actuator:** upper motor driving the large metal flywheel,
- **ground-drive actuator:** lower Nidec 24H404H-160 driving the ground-contact wheel,
- **motor interfaces:** three PCB brushless interfaces exist electrically, but only two are populated in the inspected assembly,
- **verified physical mapping:** PCB `M2` / schematic `BLDC_1` -> reaction wheel; PCB `M1` / schematic `BLDC_2` -> drive wheel; PCB `M3` / schematic `BLDC_3` -> unconnected,
- **feedback:** Encoder 1 is associated with the reaction-wheel motor path; Encoder 2 with the drive-wheel motor path,
- **communication:** UART-class interfaces including USB-UART / wireless serial capability,
- **local UI:** OLED-class display interface,
- **power:** nominal battery-powered embedded platform with motor and logic rails.

The third motor channel is therefore a **PCB capability**, not an installed actuator in the currently inspected physical plant. The verified robot-domain actuator set for this unit is exactly:

```text
ReactionWheel
DriveWheel
```

The board-level connector names, installed assembly, and robot-domain roles remain separate architectural facts. `swp-board-one-v2` owns schematic/PCB wiring; `swp-reference-assembly` owns the confirmed physical population and role mapping.

Still-unresolved commissioning properties include motor PWM active polarity, direction polarity, encoder sign, encoder mechanical scale, battery-divider scaling, MCU crystal frequency, and the MPU6050 sensor-frame-to-body-frame transform.

See `assembly_observation_2026-09-05.md` for the physical inspection evidence.

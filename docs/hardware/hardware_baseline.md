# Hardware Baseline

Current reference hardware information supports the following platform-level components:

- **MCU:** STM32F103C8T6-class controller,
- **IMU:** MPU6050,
- **reaction-wheel actuator:** upper motor driving the large metal flywheel,
- **ground-drive actuator:** lower Nidec 24H404H-160 driving the ground-contact wheel,
- **motor interfaces:** three PCB brushless interfaces exist electrically, but only two are populated in the inspected assembly,
- **verified physical mapping:** PCB `M2` / schematic `BLDC_1` -> reaction wheel; PCB `M1` / schematic `BLDC_2` -> drive wheel; PCB `M3` / schematic `BLDC_3` -> unconnected,
- **feedback:** Encoder 1 is associated with the reaction-wheel motor path; Encoder 2 with the drive-wheel motor path,
- **communication:** USART1-class wired recorder/debug path and USART2-to-ECB02 Bluetooth serial path,
- **local UI:** OLED two-wire interface on PB4/PB5,
- **power:** 3-cell battery-powered platform with motor and logic rails.

The third motor channel is therefore a **PCB capability**, not an installed actuator in the currently inspected physical plant. The verified robot-domain actuator set for this unit is exactly:

```text
ReactionWheel
DriveWheel
```

The board-level connector names, installed assembly, and robot-domain roles remain separate architectural facts. `swp-board-one-v2` owns schematic/PCB wiring; `swp-reference-assembly` owns the confirmed physical population and role mapping.

## Product-document evidence

The supplied product material gives useful priors for the original/reference configuration:

| Property | Documented value | Repository treatment |
|---|---:|---|
| Whole-vehicle mass | 570 g | product-spec prior; physical unit not yet weighed |
| Envelope | 105 x 70 x 150 mm | product-spec prior |
| Battery | 3S, 11.1 V nominal / 12.6 V full | supported operating prior |
| Battery mass | about 107 g | product-spec prior |
| Legacy control frequency | 100 Hz | historical timing reference, not estimator truth |
| MCU operating frequency | 72 MHz | product/legacy configuration evidence |
| Reaction-wheel motor | 12 V, 10 W, 3000 rpm max, 0.085 N*m max torque, 1 A stall | actuator prior; not yet independently measured |
| Ground-drive motor | 12 V, 3000 rpm no-load, 0.075 N*m rated torque, 1 A stall | actuator prior; not yet independently measured |
| Encoder line count | 100 lines on each installed motor | line-count evidence; **not yet counts/revolution** |

These values are intentionally not promoted into control constants merely because they appear in product material. Encoder decoding mode, gearing, installed-part tolerance, and actual mechanical assembly must still be measured where they affect state estimation or authority limits.

## Evidence conflicts that remain visible

The source set is not perfectly revision-consistent, so conflicting values stay explicit instead of being silently reconciled:

- product material describes a **0.96-inch OLED**, while the inspected unit carries a visibly larger OLED module and a separate 2.42-inch SSD1309 module schematic/example set is available;
- product material lists the inertia wheel as **4 mm x 70 mm**, while the supplied inertia-wheel DXF has an outer radius of 47.5 CAD units (95 diameter if the CAD unit is millimetres);
- product material states a 72 MHz MCU operating frequency and legacy source assumes the normal STM32F103 8 MHz HSE-to-72 MHz PLL arrangement, but the reviewed board schematic does not label the installed crystal frequency.

These conflicts are evidence-version issues, not reasons to block software architecture. Physical measurement will resolve them when the affected parameter becomes operationally necessary.

## Reaction-wheel plant consequence

The supplied operating instructions state that lateral balance depends on reaction-wheel counter-torque, that the wheel normally remains stopped or at low speed, and that sustained external force can drive it to high speed until balance authority is lost. Reaction-wheel speed/headroom is therefore part of control authority state, not merely telemetry.

`swp-runtime-state` models speed-domain authority without inventing an unverified wheel inertia. Exact momentum-domain control can be added after the installed wheel mass/inertia is measured.

## Communication and local UI

The reviewed board wiring now has explicit board-crate facts for:

```text
USART1: PA9 TX / PA10 RX
USART2 to ECB02S2: PA2 TX / PA3 RX
ECB02 control: PC15 AT_EN / PC14 ROLE
OLED: PB4 SDA / PB5 SCL
EN_X: PA15
EN_Y: PB3
```

The onboard CH340 and MCU USART1 nets terminate on separate P2 pins, so the USB-UART bridge is not assumed to be hard-wired to the recorder path.

`EN_X` / `EN_Y` are treated as board configuration/authority inputs. They are not the same signals as the BLDC connector `EN_BLDC_*` lines, which are hard-wired high. Their final semantic association with installed actuators remains a commissioning fact because product labels and legacy X/Y naming are not fully consistent.

## Remaining commissioning properties

Still-unresolved properties that matter later include motor PWM active polarity, direction polarity, encoder sign, encoder counts per mechanical revolution, battery-divider scaling, physical HSE marking, and the remaining signs in the MPU6050 sensor-frame-to-body-frame transform.

These are now **measurement gaps**, not document-search blockers. See `assembly_observation_2026-09-05.md`, `pin_mapping.md`, and `../architecture/body_frame_contract.md` for the current evidence boundaries.

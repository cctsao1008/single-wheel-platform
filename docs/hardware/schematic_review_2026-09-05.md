# Schematic Review Notes

This note captures reference-board facts visible in the ONE_V2.0 schematic that affect the current STM32F103 platform implementation.

- MPU6050 `MPU_SDA` is routed to PB8 and `MPU_SCL` to PB9.
- The STM32F103 I2C1 remap assigns PB8=SCL and PB9=SDA, so the board routing is opposite the hardware I2C1 remap. The reference platform must therefore use software I2C unless the signals are physically crossed or the PCB is revised.
- The schematic net named `MPU_INT` from PC13 terminates on MPU6050 FSYNC (pin 11). MPU6050 INT (pin 12) is explicitly no-connect. There is no data-ready interrupt route on the reference PCB.
- MPU6050 AD0 is pulled low, selecting address 0x68.
- `EN_BLDC_1`, `EN_BLDC_2`, and `EN_BLDC_3` are tied directly to 3.3 V at the motor connectors; they are not MCU-controlled enable lines.
- BLDC_3 exposes a `Brake` input on PA7. Its active polarity is not specified by this schematic.
- Encoder 3 A/B are present at the spin connector but no MCU routing for them is shown.
- The schematic captions identify BLDC_1 as the side brushless connector and BLDC_2 as the front/back brushless connector.
- The battery ADC is on PA5 at the R2/R4 divider node, but the resistor values are not specified in this drawing, so the divider ratio cannot be derived from the schematic alone.

These are board-routing facts, not control-law assumptions.

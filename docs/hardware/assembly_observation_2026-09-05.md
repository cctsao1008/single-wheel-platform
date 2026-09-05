# Physical Assembly Observation — 2026-09-05

This note records facts observed from the currently inspected assembled single-wheel robot. It is deliberately separate from schematic-derived facts.

## Connector population

The control PCB silkscreen visibly identifies three motor connectors:

- `M1` — populated / cabled,
- `M2` — populated / cabled,
- `M3` — physically present but **not cabled** in the inspected assembly.

The `M3` connector silkscreen includes `BRA` / brake alongside the motor-control signals. This uniquely matches the schematic `BLDC_3` / `CN1` interface, which is the only brushless connector carrying the `Brake` signal.

The remaining two physical connector labels require care because the schematic component designators are not numerically identical to the PCB motor-channel silkscreen:

```text
PCB silk M2  <-> schematic BLDC_1 connector (schematic component M2)
PCB silk M1  <-> schematic BLDC_2 connector (schematic component CN2)
PCB silk M3  <-> schematic BLDC_3 connector (schematic component CN1)
```

## Verified installed motor routing

The assembled unit has two installed motors, and both cable destinations have now been manually confirmed on the physical robot:

```text
PCB M2 / schematic BLDC_1
    -> upper motor
    -> large metal reaction wheel / flywheel
    -> robot role: ReactionWheel

PCB M1 / schematic BLDC_2
    -> lower Nidec 24H404H-160
    -> ground-contact drive wheel
    -> robot role: DriveWheel

PCB M3 / schematic BLDC_3
    -> no motor connected in the inspected unit
```

The lower motor label is readable as:

```text
Nidec
24H404H-160
D6831765B
LOT. M13910
```

This establishes the actuator-role topology of the inspected reference assembly without relying on the legacy X/Y naming or schematic captions alone.

## Encoder association

Because the encoder signals are carried on the same BLDC connector interfaces, the verified assembly mapping establishes the following channel association:

```text
Encoder_1 -> reaction-wheel motor feedback path
Encoder_2 -> drive-wheel motor feedback path
Encoder_3 -> no installed actuator / no MCU encoder route shown
```

This association does **not** yet establish encoder sign, counts per mechanical revolution, gearbox ratio, or robot-positive angular velocity. Those remain commissioning facts.

## Architectural consequence

The physical plant in the inspected unit has exactly two installed actuator roles:

```text
ReactionWheel
DriveWheel
```

`BLDC_3` remains a real PCB output capability but is not an installed actuator in this assembly. The architecture therefore keeps three different kinds of truth separate:

```text
PCB capability
    BLDC_1 / BLDC_2 / BLDC_3 exist

Assembly population
    BLDC_1 installed
    BLDC_2 installed
    BLDC_3 not installed

Robot semantics
    BLDC_1 -> ReactionWheel
    BLDC_2 -> DriveWheel
```

The Rust workspace records this verified transition in `swp-reference-assembly`; `swp-board-one-v2` remains schematic/PCB-only and does not own robot semantics.

## Still unresolved from the supplied views

- MCU top marking is obscured / not readable in these views;
- external crystal `Y1` marking/frequency is not readable in these views;
- MPU6050 sensor-axis-to-body-axis orientation still needs a clear board/vehicle orientation reference;
- encoder positive direction and mechanical scale are not yet measured;
- BLDC PWM active polarity / direction polarity still require powered commissioning evidence before output activation.

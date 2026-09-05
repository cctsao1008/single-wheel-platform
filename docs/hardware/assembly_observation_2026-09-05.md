# Physical Assembly Observation — 2026-09-05

This note records facts observed from the currently inspected assembled single-wheel robot. It is deliberately separate from schematic-derived facts.

## Connector population

The control PCB silkscreen visibly identifies three motor connectors:

- `M1` — populated / cabled,
- `M2` — populated / cabled,
- `M3` — physically present but **not cabled** in the inspected assembly.

The `M3` connector silkscreen includes `BRA` / brake alongside the motor-control signals. This uniquely matches the schematic `BLDC_3` / `CN1` interface, which is the only brushless connector carrying the `Brake` signal. Therefore the inspected PCB establishes:

```text
M3 <-> schematic BLDC_3 / CN1
```

The physical connector numbering is consistent with the schematic channel numbering (`M1`/`BLDC_1`, `M2`/`BLDC_2`, `M3`/`BLDC_3`), but only the M3 identification is currently considered uniquely cross-checked by the brake signal.

## Installed motors

Two brushless motors are visibly installed in the current assembly. The M1 cable can be visually followed from the PCB connector near the USB-C edge to the lower motor. The lower motor label is readable as:

```text
Nidec
24H404H-160
D6831765B
LOT. M13910
```

A second motor drives the large metal flywheel/reaction-wheel mechanism in the upper part of the assembly.

At this observation stage, the final semantic mapping is intentionally not promoted into the robot-domain model until cable routing is physically confirmed:

```text
M1 / BLDC_1 -> lower Nidec motor                 observed cable routing
M2 / BLDC_2 -> upper flywheel motor              probable; confirm cable trace
M3 / BLDC_3 -> no motor installed in this unit   observed
```

The upper flywheel motor is mechanically identifiable as the reaction-wheel actuator. The lower motor's final robot-domain role (for example ground-contact drive) should be confirmed from the mechanism rather than inferred solely from schematic captions.

## Architectural consequence

`BLDC_3` remains a real PCB output interface, but it is **not an installed actuator in the currently inspected physical plant**. Firmware must not assume that every schematic motor channel corresponds to an installed actuator.

Keep these concepts separate:

```text
PCB capability        = M1 / M2 / M3 interfaces exist
Assembly population   = M1 + M2 connected, M3 unconnected
Robot semantics       = assigned only after mechanical/cable confirmation
```

This distinction should remain explicit in actuator allocation and commissioning logic.

## Still unresolved from the supplied views

- exact M2 cable destination needs a direct physical trace/continuity confirmation;
- whether the lower Nidec motor is the ground-contact drive actuator needs mechanical confirmation;
- MCU top marking is obscured / not readable in these views;
- external crystal `Y1` marking/frequency is not readable in these views;
- MPU6050 sensor-axis-to-body-axis orientation still needs a clear board/vehicle orientation reference.

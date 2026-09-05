# Typed Dataflow Architecture

The firmware is organized around changes in **meaning**, not around traditional embedded folders such as `bsp`, `middleware`, or a monolithic `control.c`.

The core rule is simple:

> Data may move to a richer semantic layer only when the required evidence and configuration exist.

That produces the following dataflow:

```text
physical hardware
    |
    v
RawObservation
    |  register counts, timer counts, ADC counts, timestamp, validity
    v
CalibratedObservation
    |  sensor scale / bias / transfer functions are known
    v
BodyObservation
    |  sensor mounting and robot coordinate mapping are known
    v
RobotState
    |  estimator has produced the state used by control
    v
ControlEffort
    |  control policy requests physical-axis effort
    v
ActuatorAllocation
    |  robot actuator roles are mapped to physical output channels
    v
Authority / limits
    |  safety and actuator constraints approve or reject the request
    v
ElectricalOutput
```

These are semantic stages. They do not have to become one crate each; a crate is introduced only when it creates a real ownership or dependency boundary.

## Raw observation is evidence, not interpretation

`swp-plant-observation::RawObservation` is the first shared runtime object above peripheral drivers. It contains:

- MPU6050 raw acceleration, temperature, and gyro register values;
- raw TIM2 and TIM4 quadrature counts;
- raw ADC1_IN5 battery-divider count;
- monotonic timestamp and sample index;
- explicit validity/acquisition-health flags.

It intentionally does **not** contain volts, radians, radians/second, body axes, or actuator roles. Those meanings require calibration or mechanical mapping that is not yet fully established.

Telemetry is a tap on this dataflow. The UART protocol serializes a `RawObservation`; it is not the owner of acquisition semantics.

## PCB identity and robot identity are separate

The board crate describes only facts visible at the PCB boundary:

```text
BLDC_1 / BLDC_2 / BLDC_3
Encoder_1 / Encoder_2 / Encoder_3
MPU SDA/SCL
ADC node
physical pins and timer channels
```

It does not declare that `BLDC_1` is the reaction wheel or that `BLDC_2` is the drive wheel. That association belongs to an explicit actuator-allocation / robot-configuration layer and will be created only when the physical harness is confirmed.

This prevents historical variable names, schematic captions, or inherited firmware assumptions from silently becoming architectural truth.

## Zero-cost boundary

The semantic separation is intended to compile away. The runtime still uses statically allocated `no_std` data, fixed-size frames, RTIC-owned resources, and no heap allocation. Stronger meaning should cost type definitions and compile-time checking, not dynamic infrastructure.

## Current executable cut

The active STM32F103 firmware currently implements only:

```text
hardware -> RawObservation -> telemetry
```

Motor outputs remain untouched. The next layer should be calibration and coordinate mapping, followed by state estimation. Control and actuation are attached only after the observation path has explicit units, signs, timing, and validity semantics.

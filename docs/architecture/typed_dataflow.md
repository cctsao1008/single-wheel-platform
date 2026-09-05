# Typed Dataflow Architecture

The firmware is organized around changes in **meaning**, not around traditional embedded folders such as `bsp`, `middleware`, or a monolithic `control.c`.

> Data may move to a richer semantic layer only when the required evidence and configuration exist.

```text
physical hardware
    |
    v
RawObservation
    |  raw values + timing + quality + acquisition status
    v
CalibratedObservation
    |  sensor scale / bias / electrical transfer functions are known
    v
BodyObservation
    |  mounting transform, robot axes, and sign conventions are known
    v
EstimatedState
    |  estimator produces control-domain state using measurement time
    v
GeneralizedDemand
    |  control policy requests physical-axis effort
    v
ActuatorAllocation
    |  robot roles are mapped to physical output channels
    v
Authority / limits
    |  health, safety, and actuator constraints authorize output
    v
ElectricalOutput
```

These are semantic stages, not a requirement that every stage become a crate.

## Raw observation is evidence

`swp-plant-observation::RawObservation` intentionally does not claim that all sensors were sampled simultaneously. It contains:

- acquisition start and completion time;
- MPU6050 register-domain values;
- MPU source-sample timestamp evidence, which is currently unknown;
- MPU I2C read start/completion times;
- independent Encoder 1/2 count snapshots and capture times;
- raw battery ADC value and read-completion time;
- per-measurement quality flags;
- acquisition/platform status.

`AVAILABLE | IO_OK` means a value was acquired cleanly. It does **not** imply freshness, exact source timing, saturation knowledge, or calibrated physical meaning. Those properties must be established independently.

## Time is data, not scheduler folklore

The 100 Hz RTIC timer defines when acquisition work is requested. It does not define the physical source timestamp of every measurement.

The estimator must derive time evolution from measurement timestamp evidence. A future source with exact sample timing may set `TIMING_VALID` and a known source timestamp. A source without that evidence remains explicit rather than being assigned the task-entry timestamp.

## Calibration and coordinate mapping are different transitions

Calibration answers questions such as sensor scale, bias, temperature compensation, encoder scale, and ADC transfer function. Coordinate mapping answers different questions: sensor mounting rotation, robot axes, encoder sign, and which PCB channel corresponds to which physical actuator role.

The intended transition is therefore:

```text
RawObservation
      |
      v
SensorCalibration
      |
      v
CalibratedObservation
      |
      v
Frame / mechanical mapping
      |
      v
BodyObservation
```

## Recording is a branch, not the model owner

```text
RawObservation
      |\
      | +--> calibration / estimator / control
      |
      +--> RecordedObservation --> storage/transport --> replay
```

The record format is deterministic and versioned by `swp-observation-record`. UART merely transports those bytes today. Replay feeds recorded observation evidence back into later processing without depending on host wall-clock timing.

## PCB identity and robot identity remain separate

The board crate describes only PCB facts such as `BLDC_1`, `BLDC_2`, `BLDC_3`, `Encoder_1`, `Encoder_2`, pins, timers, and ADC nodes. It does not declare a reaction-wheel or drive-wheel mapping until that association is physically established.

## Zero-cost boundary

The semantic types remain `no_std`, statically allocated, and heap-free. Stronger meaning should cost compile-time checking and explicit code paths, not dynamic infrastructure. Record serialization is isolated from the runtime domain types so wire layout does not dictate estimator or controller data structures.

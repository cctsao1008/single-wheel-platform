# Self-Balancing Single-Wheel Platform

A Rust `no_std` control platform for a reaction-wheel-stabilized single-wheel robot.

The repository has four architectural domains. They describe ownership and dependency, not execution order or directory depth.

```text
                  CONTROL
                     ▲
                     │
                 SUPERVISOR
                  ▲      ▲
                  │      │
                PLANT    │
                  ▲      │
                  └──┬───┘
                     │
                 FIRMWARE
              STM32 / RP2350
```

## Domains

### Plant

`plant/` defines the physical system and the semantics that describe it:

```text
robot-domain
plant-model
measurement-model
plant-observation
actuator-model
```

It owns physical state, measurements, torque demand, actuator behavior, and model equations. It does not own MCU peripherals, scheduling, or control policy.

### Control

`control/` defines desired closed-loop behavior from estimated state and reference:

```text
EstimatedState + Reference
            |
            v
        LQR / LQI
            |
            v
    GeneralizedDemand
```

Control does not own sensors, operating-state policy, authority, or electrical output.

### Supervisor

`supervisor/` owns the robot's runtime belief and authority:

```text
measurement
    |
    v
StateEstimator
    |
    v
EstimatedState
    |
    +------> Control
    |           |
    |       requested demand
    |           |
    v           v
operating state / timing / limits
            |
            v
      RuntimeAuthority
            |
            v
   AuthorizedActuation
```

This includes state estimation, operating state, sensor timing health, reaction-wheel headroom, actuator constraints, integrator-hold policy, and orchestration of one control opportunity.

### Firmware

`firmware/` makes the portable system real on hardware. It owns device transfer, board binding, peripheral ownership, scheduling, telemetry, and electrical actuation.

Current STM32F103 resources include:

```text
MPU6050 DATA_RDY / PC13 EXTI13   500 Hz primary control opportunity
TIM1                              1 kHz timing-health supervisor
TIM2                              Encoder_1 QEI
TIM4                              Encoder_2 QEI
ADC1 / PA5                        battery observation
USART2 TX / DMA1 CH7              telemetry
TIM3_CH1 / PA6 + PA4 DIR          DriveWheel output
TIM3_CH4 / PB1 + PB11 DIR         ReactionWheel output
```

The firmware boundary is also where another target such as RP2350 plugs into the same Plant / Supervisor / Control contracts.

## Runtime loop

The runtime is a feedback loop rather than a linear layer stack:

```text
Physical Plant
     |
 observation
     v
Supervisor / Estimator
     |
 estimated state
     v
Control
     |
 physical demand
     v
Supervisor / Authority
     |
 AuthorizedActuation
     v
Firmware
     |
 electrical actuation
     +--------------------> Physical Plant
```

The semantic path remains typed:

```text
RawObservation
  -> EstimatorMeasurement
  -> EstimatedState
  -> GeneralizedDemand
  -> BoundedActuatorCommand
  -> AuthorizedActuation
  -> ElectricalActuation
  -> Physical Output
```

Requested effort, bounded command, authorized actuation, and electrical output are distinct meanings.

## Infrastructure

`infrastructure/` contains horizontal mechanisms that support the four domains without becoming another control layer:

```text
dsp-kernel
observation-record
control-profile-record
```

Host-side engineering remains under `tools/` for model derivation, system identification, control synthesis, recording, replay, and correlation.

## Repository

```text
plant/
control/
supervisor/
firmware/
infrastructure/
parameters/
tools/
docs/
```

The old flat `crates/` layout is intentionally removed. Git history preserves the previous structure; `main` represents the current architecture.

## Reference assembly

```text
BLDC_1 / Encoder_1 -> ReactionWheel
BLDC_2 / Encoder_2 -> DriveWheel
BLDC_3             -> unused
```

Body frame:

```text
+X = forward
+Y = left
+Z = up
```

## Safety boundary

Physical output can only be reached through:

```text
RuntimeAuthority
       |
       v
AuthorizedActuation
       |
       v
Electrical Output
```

Observation and live-shadow firmware do not instantiate the motor electrical-output owner.

## Build

```bash
cargo fw
```

Architecture: [`docs/architecture/system_architecture.md`](docs/architecture/system_architecture.md)

# Self-Balancing Single-Wheel Platform

A Rust `no_std` control platform for a reaction-wheel-stabilized single-wheel robot.

The repository has four architectural domains. They define ownership and dependency, not execution order or directory depth.

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
          control board + actuator hardware
```

## Domains

### Plant

`plant/` defines the physical system: state, units, dynamics, measurement physics, observations, and actuator physics. It does not know MCU peripherals, scheduling, or control policy.

### Control

`control/` defines desired closed-loop behavior from estimated state and reference. The current implementation is state feedback (`LQR` / `LQI`) producing `GeneralizedDemand` in physical units.

### Supervisor

`supervisor/` owns runtime belief and authority: state estimation, operating state, timing health, reaction-wheel headroom, actuator constraints, integrator hold policy, and the only semantic promotion to `AuthorizedActuation`.

### Firmware

`firmware/` makes the portable system real on hardware. Its top-level taxonomy is role-based so sensors, communication modules, UI, control boards, and actuator hardware do not collapse into generic `devices/` or `drivers/` buckets.

```text
firmware/
├── interfaces/       target-independent physical-I/O contracts
├── sensors/          sensing-device protocols and transfer functions
├── communications/   external communication modules/endpoints
├── ui/               human-interface components
├── buses/            reusable bus implementations
├── actuators/        actuator electrical/protocol semantics
├── adapters/         hardware evidence -> platform semantics
├── boards/           control-board wiring and peripheral capability
├── assemblies/       robot roles -> installed hardware channels
└── targets/          MCU-specific executable composition and HAL ownership
```

The actuation boundary is:

```text
Supervisor
    |
AuthorizedActuation
    |
    v
ActuationSink
    |
    v
actuator adapter
    |
 actuator-specific frame
    v
ActuatorIo<Frame>
    |
    v
control-board target backend
    |
 GPIO / PWM / PIO / SPI / CAN
    |
    v
actuator hardware
```

`ActuationSink` is target-independent. `ActuatorIo<Frame>` separates actuator electrical/protocol meaning from the MCU mechanism that emits it. A future RP2350 target can therefore reuse the same Plant / Supervisor / Control and, where electrically compatible, the same actuator adapter.

Current ONE V2 composition:

```text
sensor      firmware/sensors/mpu6050
board       firmware/boards/one-v2
assembly    firmware/assemblies/one-v2-reference
actuator    firmware/actuators/one-v2-pwm-dir
target      firmware/targets/stm32f103
```

The installed actuator mapping remains:

```text
BLDC_1 / Encoder_1 -> ReactionWheel
BLDC_2 / Encoder_2 -> DriveWheel
BLDC_3             -> unused
```

Current STM32F103 resources include:

```text
MPU6050 DATA_RDY / PC13 EXTI13   500 Hz primary control opportunity
TIM1                              1 kHz timing-health supervisor
TIM2                              Encoder_1 QEI
TIM4                              Encoder_2 QEI
ADC1 / PA5                        battery observation
USART2 TX / DMA1 CH7              telemetry
TIM3_CH1 / PA6 + PA4 DIR          DriveWheel output backend
TIM3_CH4 / PB1 + PB11 DIR         ReactionWheel output backend
```

The observation and live-shadow targets do not instantiate the motor output backend.

## Runtime loop

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
Firmware / ActuationSink
     |
 physical actuation
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
  -> actuator-specific frame
  -> physical output
```

## Infrastructure and host engineering

`infrastructure/` contains horizontal numerical and recording mechanisms. Host-side model derivation, system identification, control synthesis, replay, and correlation remain under `tools/`.

## Build

```bash
cargo fw-observation
cargo fw-live-shadow
cargo fw-control-footprint
```

Architecture: [`docs/architecture/system_architecture.md`](docs/architecture/system_architecture.md)

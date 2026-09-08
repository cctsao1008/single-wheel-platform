# Self-Balancing Single-Wheel Platform

A Rust `no_std` control platform for a reaction-wheel-stabilized single-wheel robot.

The repository has four architectural domains. They define ownership and dependency, not runtime execution order or directory depth.

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
```

## Domains

- **Plant** — physical state, units, dynamics, measurement physics, observation semantics, and actuator physics.
- **Control** — desired closed-loop behavior. The current implementation contains inner state feedback and a 100 Hz outer velocity loop; it produces `GeneralizedDemand` in physical semantics.
- **Supervisor** — estimation, runtime state, timing health, watchdog/fault handling, actuator qualification, and the only semantic promotion to `AuthorizedActuation`.
- **Firmware** — sensing-device protocols, communications, UI, buses, actuator electrical semantics, board/assembly binding, MCU target composition, and physical I/O.

The canonical typed path is:

```text
RawObservation
  -> EstimatorMeasurement
  -> EstimatedState
  -> GeneralizedDemand
  -> BoundedActuatorCommand
  -> AuthorizedActuation
  -> actuator-specific electrical/protocol frame
  -> physical output
```

## Firmware shape

```text
firmware/
├── interfaces/       target-independent physical-I/O contracts
├── sensors/          sensing-device protocols and transfer functions
├── communications/   telemetry and external communication endpoints
├── ui/               reusable status/UI behavior
├── buses/            reusable bus implementations
├── actuators/        actuator electrical/protocol semantics
├── adapters/         hardware evidence -> platform semantics
├── boards/           control-board wiring and peripheral capability
├── assemblies/       robot roles -> installed hardware channels
├── recording/        firmware-owned binary evidence formats
└── targets/          MCU-specific executable composition and HAL ownership
```

Current reusable Firmware components include:

```text
sensing        firmware/sensors/mpu6050
telemetry      firmware/communications/telemetry
BLE boundary   firmware/communications/ecb02
status view    firmware/ui/status
OLED UI        firmware/ui/oled
board          firmware/boards/one-v2
assembly       firmware/assemblies/one-v2-reference
actuator       firmware/actuators/one-v2-pwm-dir
```

The installed actuator mapping is:

```text
BLDC_1 / Encoder_1 -> ReactionWheel
BLDC_2 / Encoder_2 -> DriveWheel
BLDC_3             -> unused
```

## Actuation authority

```text
BoundedActuatorCommand
        ↓
Supervisor / RuntimeAuthority
        ↓
AuthorizedActuation
        ↓
Firmware / ActuationSink
        ↓
actuator-specific frame
        ↓
ActuatorIo<Frame>
        ↓
physical target backend
```

The existence of an actuator frame, target backend, PWM peripheral, or GPIO route never grants authority by itself.

## Current non-actuating runtime

The canonical STM32F103 runtime baseline currently implemented by `runtime-shadow` is:

```text
inner sensing / estimation / balance    200 Hz
outer velocity loop                     100 Hz
semantic RuntimeObservation             100 Hz
telemetry framework                      50 Hz
OLED UI framework                        10 Hz
```

`firmware/targets/stm32f103/runtime-shadow` executes the control path through `AuthorizedActuation` and ONE V2 electrical encoding, then terminates in RAM. It owns no motor PWM/DIR backend.

`firmware/targets/stm32f103/io-shadow` materializes the Communications/UI framework at 50 Hz telemetry and 10 Hz OLED update rates using RAM-only shadow transports. Both publishers use latest-value, drop-on-busy semantics: they own no backlog and do not replay missed output opportunities.

The ECB02 and OLED crates therefore define reusable contracts and presentation/transport behavior, but **do not claim verified ONE V2 UART/display wiring, BLE throughput, module configuration, or physical OLED operation**.

## Software-In-The-Loop

Host SITL lives under `tools/sitl/`; it is verification tooling, not a fifth production domain.

```text
Deterministic Scheduler
        |
        | PhysicalTimeAdvance
        v
SimulationWorld
        |
        | RawObservation
        v
production Firmware adapters
        |
        v
production estimator / Control / actuator model / RuntimeAuthority
        |
        | AuthorizedActuation
        v
ActuationSink
        |
        +-----------------------------> SimulationWorld
```

`SimulationWorld` owns reduced physical truth, deterministic physical-time integration, device-like MPU6050/encoder sensing, and the currently applied authorized ideal torque. Simulation truth is available to host evidence/correlation only and is never passed directly into the production estimator or Control.

The closed-loop SITL composition reuses production sensor scaling/calibration, frame transform, estimator-input, estimator, state feedback, velocity-loop reference generation, actuator model, Supervisor state/authority semantics, and `ActuationSink`. A missed runtime opportunity consumes that observation as missed and is never replayed or caught up.

SITL configurations and repository integration tests use explicitly synthetic values where physical ONE V2 parameters or calibration evidence remain unknown. Simulation evidence is not physical validation.

## Targets

```text
firmware/targets/stm32f103/
├── observation
├── control-footprint
├── live-shadow
├── control-shadow
├── runtime-shadow
├── io-shadow
└── one-v2-pwm-dir
```

The first six targets are non-actuating integration/profiling targets. `one-v2-pwm-dir` contains the separate physical motor backend; it is not composed into `runtime-shadow` or `io-shadow`.

## Support and host engineering

`support/` contains non-domain implementation support shared by production domains; `support/dsp-kernel` is the current cross-domain numerical kernel. Firmware-owned recording codecs live under `firmware/recording/`. Host-side system identification, mathematical derivation, control synthesis, SITL, recording decode/replay, and correlation live under `tools/`.

## Build

```bash
cargo fw-observation
cargo fw-control-footprint
cargo fw-live-shadow
cargo fw-control-shadow
cargo fw-runtime-shadow
cargo fw-io-shadow
```

Architecture: [`docs/architecture/system_architecture.md`](docs/architecture/system_architecture.md)

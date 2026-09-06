# System Architecture

The platform is organized around four architectural domains:

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

The arrows express ownership/dependency relationships. Runtime execution is a closed feedback loop, not a top-to-bottom software-layer pipeline.

## Plant

Plant is the portable physical truth of the robot. It owns physical state and units, plant dynamics, measurement physics, raw observation semantics, generalized physical input, and actuator physics/model constraints.

```text
x_dot = f(x, u, p)
y     = h(x, u, p)
```

The balance input is physical torque:

```text
u = [drive-wheel torque, reaction-wheel torque]^T
```

Current code:

```text
plant/robot-domain
plant/plant-model
plant/measurement-model
plant/plant-observation
plant/actuator-model
```

Plant does not know STM32, RP2350, RTIC, GPIO, PWM, BLE, display buses, or actuator electrical polarity.

## Control

Control owns desired closed-loop behavior.

```text
EstimatedState + Reference
            ↓
        control law
            ↓
    GeneralizedDemand
```

Current code:

```text
control/state-feedback
control/velocity-loop
```

The inner state-feedback path produces physical `GeneralizedDemand`. The outer velocity loop runs at 100 Hz in the current STM32 runtime composition and produces a balance-state reference rather than an actuator command.

Control does not own sensors, runtime state policy, timing/watchdog policy, or physical-output authority.

## Supervisor

Supervisor owns runtime belief, operating policy, health, causality, and physical-output authority.

```text
EstimatorMeasurement
        ↓
StateEstimator
        ↓
EstimatedState
        ↓
Control
        ↓
GeneralizedDemand
        ↓
Plant actuator model
        ↓
BoundedActuatorCommand
        ↓
RuntimeAuthority
        ├── denied  -> no token
        └── allowed -> AuthorizedActuation
```

Current code:

```text
supervisor/state-estimator
supervisor/ekf
supervisor/runtime-state
supervisor/runtime-supervisor
supervisor/control-runtime
```

`runtime-supervisor` owns watchdog, latched faults, and operating-state orchestration. Supervisor has no concrete MCU/HAL dependency.

## Firmware

Firmware is the hardware-realization and target-composition domain.

```text
firmware/
├── interfaces/
├── sensors/
├── communications/
├── ui/
├── buses/
├── actuators/
├── adapters/
├── boards/
├── assemblies/
└── targets/
```

There is intentionally no generic `devices/` or `drivers/` architectural bucket.

### Interfaces

`interfaces/` contains target-independent physical-I/O contracts.

```text
AuthorizedActuation
        ↓
ActuationSink
        ↓
actuator-specific Frame
        ↓
ActuatorIo<Frame>
        ↓
MCU target backend
```

`ActuationSink` consumes Supervisor-authorized actuation. `ActuatorIo<Frame>` separates actuator electrical/protocol meaning from the MCU mechanism that emits it.

### Sensors

Current reusable sensing-device behavior:

```text
firmware/sensors/mpu6050
```

Sensor transfer behavior, mounting/frame evidence, calibration evidence, and target pin routing remain distinct concerns.

### Communications

Current code:

```text
firmware/communications/telemetry
firmware/communications/ecb02
```

`telemetry` defines the current fixed runtime telemetry packet, version/sequence/timestamp fields, CRC, and a non-blocking latest-value publisher.

`ecb02` provides the reusable ECB02-facing byte-transport boundary. It does not own a UART instance, DMA channel, module configuration commands, or board wiring.

A busy communication transport drops the current telemetry opportunity. There is no queue and no backlog replay into later control periods.

### UI

Current code:

```text
firmware/ui/status
firmware/ui/oled
```

`status` defines the read-only presentation model for runtime/control/health information. `oled` renders that model into a fixed no-heap text frame and exposes a non-blocking display contract.

A busy display drops the current UI opportunity. UI does not mutate Control or acquire actuation authority.

The current reusable UI framework does not assert a concrete OLED controller identity, bus, address, or ONE V2 display pinout.

### Buses

Current implementation:

```text
firmware/buses/software-i2c
```

A bus owns transport mechanics, not the semantic meaning of the attached device.

### Adapters

Current semantic adapters:

```text
firmware/adapters/sensor-calibration
firmware/adapters/frame-transform
firmware/adapters/estimator-input
```

Adapters convert hardware evidence into portable Plant/Supervisor semantics.

### Boards

Current board description:

```text
firmware/boards/one-v2
```

A board owns PCB wiring and peripheral capability. It does not silently assign robot roles to connectors.

### Actuators

Current actuator electrical-semantic adapter:

```text
firmware/actuators/one-v2-pwm-dir
```

It converts `AuthorizedActuation` into ONE V2 `ElectricalActuation` semantics and emits that frame through `ActuatorIo<ElectricalActuation>`.

### Assemblies

Current installed role/channel binding:

```text
firmware/assemblies/one-v2-reference
```

```text
BLDC_1 / Encoder_1 -> ReactionWheel
BLDC_2 / Encoder_2 -> DriveWheel
BLDC_3             -> unused
```

Board identity, actuator protocol, and robot role remain separate facts.

### Targets

Current STM32F103 target family:

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

`runtime-shadow` is the canonical non-actuating control integration target. It executes sensing, estimation, 200 Hz inner control, the 100 Hz outer velocity loop, Plant actuator bounding, Supervisor runtime qualification/authority, ONE V2 electrical encoding, and 100 Hz semantic runtime recording. Authorized electrical output terminates in RAM.

`io-shadow` is the non-actuating Communications/UI framework target. A 200 Hz target-owned cadence produces 50 Hz telemetry and 10 Hz OLED opportunities through RAM-only shadow transports. It owns no ECB02 UART or physical OLED bus/pins.

`one-v2-pwm-dir` is the separate physical motor backend. It owns the concrete STM32F103 TIM3/GPIO resources but is not composed into `runtime-shadow` or `io-shadow`.

A future RP2350 target may provide different peripheral backends without changing Plant, Control, or Supervisor ownership.

## Runtime causality

```text
Physical Plant
      │ observation
      ▼
Firmware input path
      ▼
Supervisor / Estimator
      │ EstimatedState
      ▼
Control
      │ GeneralizedDemand
      ▼
Plant / Actuator Model
      │ BoundedActuatorCommand
      ▼
Supervisor / RuntimeAuthority
      │ AuthorizedActuation
      ▼
Firmware / actuator adapter
      │ actuator-specific frame
      ▼
Firmware target backend
      └──────────────► Physical Plant
```

One fresh sensor/control opportunity creates at most one control opportunity. Missed periods are not replayed as catch-up control iterations.

The current non-actuating STM32F103 timing baseline is:

```text
sensing / estimator / inner balance    200 Hz
outer velocity loop                    100 Hz
semantic RuntimeObservation            100 Hz
telemetry framework                     50 Hz
OLED UI framework                       10 Hz
```

Telemetry and UI are lower-priority observations of current runtime state. Their drop-on-busy behavior cannot create a queue that changes control causality.

## Typed semantic boundaries

```text
RawObservation
    != EstimatorMeasurement
    != EstimatedState
    != GeneralizedDemand
    != BoundedActuatorCommand
    != AuthorizedActuation
    != actuator-specific electrical/protocol frame
    != physical output
```

Each transition has one clear owner. The existence of a target backend, communication endpoint, UI component, or actuator frame never grants actuation authority by itself.

## Physical-output isolation

The non-actuating integration targets stop before physical motor output:

```text
AuthorizedActuation
      ↓
ElectricalActuation
      ↓
RAM shadow sink
      ↓
STOP
```

They do not rely on a runtime `PWM_ENABLED` flag; the physical motor backend is absent from their target composition.

The reusable ECB02/OLED frameworks likewise do not claim physical peripheral integration or verification. Concrete UART/display wiring and throughput/timing evidence belong to later target commissioning.

## Host engineering

`infrastructure/` remains horizontal support for numerical kernels, records, and profiling. Model derivation, parameter identification, control synthesis, replay, and physical correlation remain host-side under `tools/`. Reference-backed or synthetic commissioning parameters may bootstrap structural execution, but physical validity requires measured/identified evidence.

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
          control board + driver board
```

The arrows express ownership/dependency relationships. Runtime execution is a closed feedback loop, not a top-to-bottom pipeline.

## Plant

Plant is the portable physical truth of the robot. It owns physical state and units, plant dynamics, measurement physics, raw observation semantics, and actuator physics.

```text
x_dot = f(x, u, p)
y     = h(x, u, p)
```

The current balance input is physical torque:

```text
u = [drive-wheel torque, reaction-wheel torque]^T
```

Plant does not know STM32, RP2350, RTIC, GPIO, PWM, PIO, BLE, or motor-driver polarity.

Current code:

```text
plant/robot-domain
plant/plant-model
plant/measurement-model
plant/plant-observation
plant/actuator-model
```

## Control

Control defines desired behavior from estimated state and reference.

```text
EstimatedState + Reference
            |
            v
       control law
            |
            v
    GeneralizedDemand
```

The current state-feedback form is:

```text
u = u_ff - K (x_hat - x_ref)
```

Control does not own sensors, target hardware, operating-state policy, or output authority.

Current code:

```text
control/state-feedback
```

## Supervisor

Supervisor owns runtime belief, operating policy, and physical-output authority. It consumes Plant semantics, invokes Control, and is the only semantic source of `AuthorizedActuation`.

```text
EstimatorMeasurement
        |
        v
StateEstimator
        |
        v
EstimatedState
        |
        +------> Control
        |           |
        |       demand
        |           |
        v           v
operating state / timing / limits
              |
              v
       RuntimeAuthority
              |
              +-- denied --> no token
              |
              +-- allowed -> AuthorizedActuation
```

Current code:

```text
supervisor/state-estimator
supervisor/ekf
supervisor/runtime-state
supervisor/control-runtime
```

Supervisor has no STM32/RP2350/HAL dependency.

## Firmware

Firmware is the physical execution and target-composition domain. It is intentionally decomposed along hardware identities that can vary independently:

```text
firmware/
├── interfaces/
├── devices/
├── buses/
├── adapters/
├── boards/
├── drivers/
├── assemblies/
└── targets/
```

### Interfaces

`interfaces/` contains target-independent physical-I/O contracts. The actuation contract is:

```text
AuthorizedActuation
        |
        v
   ActuationSink
```

`ActuationSink` is the only public high-level physical-output contract. It accepts an `AuthorizedActuation` token and supports explicit revocation of stale output.

`DriverIo<Frame>` is the lower boundary between a driver-specific frame and the MCU mechanism that emits it:

```text
driver semantics
      |
 driver-specific Frame
      |
      v
DriverIo<Frame>
      |
      v
MCU peripheral backend
```

This keeps driver-board semantics independent of the selected control board.

### Devices and buses

`devices/` owns IC/device protocols and transfer functions. `buses/` owns reusable transport implementations.

Current examples:

```text
firmware/devices/mpu6050
firmware/buses/software-i2c
```

Neither category owns robot role or control policy.

### Adapters

`adapters/` converts device/board evidence into portable platform semantics:

```text
firmware/adapters/sensor-calibration
firmware/adapters/frame-transform
firmware/adapters/estimator-input
```

Adapters may depend on Plant/Supervisor contracts but not on a concrete target executable.

### Boards

`boards/` describes a control board's physical capabilities and wiring: MCU identity, pins, timers, connectors, and peripheral routes.

A board must not silently assign robot roles to its connectors.

Current board:

```text
firmware/boards/one-v2
```

For example, the board knows `BLDC_1`, `BLDC_2`, pins, and timer channels; it does not define `ReactionWheel` or `DriveWheel` as intrinsic PCB identities.

### Drivers

`drivers/` owns motor-driver electrical/protocol semantics independently of the MCU that emits them.

Current driver adapter:

```text
firmware/drivers/one-v2-pwm-dir
```

It owns the current ONE V2 PWM/DIR polarity and zero-effort encoding, implements `ActuationSink`, and emits a driver-specific `ElectricalActuation` frame through `DriverIo<ElectricalActuation>`.

A future driver may use a different frame entirely, for example 3-PWM, SPI, CAN, or another protocol, while preserving the same `ActuationSink` boundary.

### Assemblies

`assemblies/` binds robot roles to the physically installed board/driver channels. This is where statements such as:

```text
BLDC_1 / Encoder_1 -> ReactionWheel
BLDC_2 / Encoder_2 -> DriveWheel
```

belong.

Current assembly:

```text
firmware/assemblies/one-v2-reference
```

Board identity, driver protocol, and robot role are separate facts.

### Targets

`targets/` owns executable MCU composition, scheduling, interrupts, DMA, concrete HAL resources, and target backends for driver I/O.

Current target family:

```text
firmware/targets/stm32f103/
├── observation
├── live-shadow
├── control-footprint
└── one-v2-pwm-dir
```

The `one-v2-pwm-dir` backend owns the exact STM32F103 TIM3/GPIO resources and implements `DriverIo<ElectricalActuation>`. It does not own driver polarity or runtime authority.

A future RP2350 target belongs beside STM32F103:

```text
firmware/targets/rp2350/...
```

and may provide PWM-slice, PIO, SPI, or CAN backends without changing Plant / Supervisor / Control.

## Control-board and driver-board portability

The hardware composition is explicitly two-dimensional:

```text
                 portable robot domains
        Plant / Supervisor / Control
                    |
                    v
              ActuationSink
                    |
        +-----------+-----------+
        |                       |
   driver adapter A        driver adapter B
        |                       |
   DriverIo<FrameA>         DriverIo<FrameB>
        |                       |
   +----+-----+             +---+------+
   |          |             |          |
STM32F103   RP2350       STM32F103   RP2350
 backend     backend       backend     backend
```

A new control board should require a new target backend, not a new controller. A new motor-driver board should require a new driver adapter, not a new estimator or control law. If the new control board and driver share an existing `DriverIo<Frame>` contract, only target composition changes.

## Dependency rules

```text
Plant
    no target-MCU dependency
    no control-policy dependency
    no physical-output mutation

Control
    may depend on Plant semantics
    no firmware dependency
    no authority ownership

Supervisor
    may depend on Plant + Control
    no target-MCU/HAL dependency
    is the only semantic source of AuthorizedActuation

Firmware interfaces
    define target-independent physical-I/O boundaries

Firmware boards
    own wiring/capability, not robot roles or control policy

Firmware drivers
    own driver electrical/protocol semantics, not MCU peripheral ownership

Firmware assemblies
    own installed role/channel binding

Firmware targets
    may compose all lower contracts
    own concrete MCU peripherals and executable scheduling
```

`infrastructure/` remains horizontal support for numerical kernels, records, and profiling; it is not a fifth robot domain.

## Runtime causality

```text
        ┌─────────────────────────────────────┐
        │                                     │
        ▼                                     │
   Physical Plant                             │
        │                                     │
   observation                                │
        ▼                                     │
   Firmware input adapters                    │
        │                                     │
        ▼                                     │
   Supervisor / estimator                     │
        │                                     │
   EstimatedState                             │
        ▼                                     │
      Control                                 │
        │                                     │
   GeneralizedDemand                          │
        ▼                                     │
   Supervisor / authority                     │
        │                                     │
   AuthorizedActuation                        │
        ▼                                     │
   Firmware / ActuationSink                   │
        │                                     │
        └──────── physical actuation ─────────┘
```

One sensor opportunity creates at most one control opportunity; missed periods are not replayed as catch-up control.

## Typed semantic boundaries

```text
RawObservation
    != EstimatorMeasurement
    != EstimatedState
    != GeneralizedDemand
    != BoundedActuatorCommand
    != AuthorizedActuation
    != driver-specific frame
    != physical output
```

Each transition has one owner. The existence of a target backend or driver frame never grants actuation authority by itself.

## Host engineering

Model derivation, parameter identification, exact discretization, observer/controller synthesis, replay, and correlation remain host-side under `tools/`. Reference-backed nominal parameters may bootstrap execution; local measured or identified values supersede lower-confidence assumptions without changing this architecture.

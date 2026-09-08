# SITL

Host-side Software-In-The-Loop execution for deterministic control-system verification.

The repository keeps simulated physical truth separate from production software belief:

```text
Scenario / Parameters
        |
        v
Deterministic Scheduler
        |
        | PhysicalTimeAdvance
        v
SimulationWorld
        |
        | device-like evidence
        v
RawObservation
        |
        v
production sensor calibration
        |
        v
production frame transform
        |
        v
production estimator-input
        |
        v
production estimator / Control / actuator model / RuntimeAuthority
        |
        | AuthorizedActuation
        v
ActuationSink
        |
        | ideal physical torque in SITL
        +-------------------------------> SimulationWorld
```

Simulation truth is available only to host evidence and independent analysis:

```text
SimulationWorld truth
        |
        +--> Evidence / plots / reference correlation
```

Production estimator and Control must never receive simulation truth directly.

## Deterministic execution

The scheduler owns integer virtual time, independent sensor/runtime cadences, explicit missed runtime opportunities, semantic event ordering, and deterministic evidence generation.

The scheduler advances from the previous timestamp to the next timestamp before dispatching events at the new time:

```text
current time = t0
    -> next timestamp = t1
    -> PhysicalTimeAdvance [t0, t1)
    -> dispatch events at t1
```

Physical integration is therefore a time-transition operation, not a queued event at `t1`.

Events at one timestamp are ordered by:

```text
virtual time
    -> semantic phase
    -> insertion sequence
```

The queued semantic phases are:

```text
PhysicalOrFaultEvent
SensorSample
ObservationDelivery
ProductionRuntime
ActuationCommit
```

A nominal executable system may implement `ScenarioExecution` so these events operate on real production semantic components instead of being trace-only mechanics.

Scheduling into the past is rejected. A missed runtime opportunity is recorded and skipped; the delivered observation for that opportunity is consumed as missed and is never replayed or caught up later. The previously committed physical input remains the zero-order-held plant input until a later authorized actuation commit changes it.

## SimulationWorld

`SimulationWorld` is the physical side of SITL. It owns:

```text
physical truth state
physical-time integration
device-like virtual sensing
currently applied authorized physical input
```

The current backend uses the repository's stationary-upright reduced dynamics with deterministic RK4 integration. It synthesizes MPU6050-like raw accelerometer/gyroscope counts and 16-bit drive/reaction encoder counts. Authorized actuator output is represented as ideal physical drive/reaction torque.

The virtual sensor frame currently equals the canonical body frame:

```text
+X forward
+Y left
+Z up
```

`SimulationWorld` exposes virtual encoder slots in robot-semantic `[drive, reaction]` order. ONE V2 board-channel association remains a Firmware assembly responsibility and is not fabricated by host SITL.

The model deliberately does not invent unavailable reference-platform facts. `parameters/reference-assembly.json` remains the source for identified platform parameters, and unknown entries remain unknown. Callers constructing `SimulationWorld` must supply a complete physical/sensor configuration; repository tests use explicitly synthetic fixtures only.

The current physical backend does not claim sensor bias/noise, sensor latency, wheel slip, motor electrical dynamics, battery dynamics, or physical calibration/correlation.

## Production semantic path

`ClosedLoopSimulation` composes `SimulationWorld` with the existing production semantic components rather than implementing a second estimator/controller stack:

```text
RawObservation
    -> scale_mpu6050
    -> calibrate_imu
    -> map_calibrated_imu_to_body
    -> EstimatorInputBuilder
    -> EstimatorMeasurement
    -> ControlRuntime
         -> production estimator
         -> state feedback
         -> actuator model
         -> RuntimeAuthority
    -> AuthorizedActuation
    -> SimulationWorld ActuationSink
```

The outer `VelocityLoop` remains production Control and updates the held balance reference at its configured decimation. `RuntimeSupervisor` owns explicit operating-state progression and control-health observation around the production `ControlRuntime`.

The first accepted encoder observation only primes the production encoder trackers. No rate is invented; that runtime opportunity produces no closed-loop actuation. Closed-loop authority remains denied until sensor cadence is healthy, the estimator is valid, and the Supervisor has progressed from `CaptureWindow` to `Balancing`.

The closed-loop integration tests intentionally use a complete synthetic parameter/calibration/controller fixture. Those values prove software causality and loop closure only. They are not ONE V2 physical evidence.

## RawObservation boundary

Virtual sensing produces the same raw observation semantics used by production firmware:

```text
SimulationWorld physical truth
        |
        v
raw MPU6050-like counts
raw encoder counts
timing / quality evidence
        |
        v
RawObservation
```

Simulation truth is not substituted for an estimated angle or controller state.

## Scenario

Scenarios are external TOML files:

```text
duration_us
seed
sensor_period_us
runtime_period_us
missed_runtime_at_us[]
```

The scheduler-only CLI remains useful for deterministic execution/evidence checks when no complete physical/control configuration is supplied:

```bash
cargo run -p swp-sitl --target x86_64-unknown-linux-gnu -- \
  --scenario tools/sitl/scenarios/deterministic-baseline.toml \
  --output sitl-output
```

## Evidence

`manifest.json` records execution provenance independently from the trace, including:

```text
system_identifier
git_commit
scenario / seed / duration
sensor_period_us
runtime_period_us
missed_runtime_at_us[]
production_model_configuration
virtual_physical_truth_configuration
```

Production-model configuration and virtual physical truth configuration remain separate so model assumptions can be compared without losing provenance.

`trace.jsonl` records deterministic event order. `summary.json` records time slices, physical-time advances, sensor/runtime counts, missed opportunities, actuation commits, and pass/fail.

CI runs the deterministic scenario twice and requires all three evidence files to be byte-identical. Closed-loop integration tests additionally verify that device-like virtual sensing traverses the production semantic path, that an upright perturbation can be stabilized under an explicit synthetic fixture, and that a missed runtime opportunity is not replayed.

SITL evidence is simulation evidence, not physical validation.

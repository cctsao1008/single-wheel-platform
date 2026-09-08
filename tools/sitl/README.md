# SITL

Host-side Software-In-The-Loop execution for deterministic control-system verification.

The repository keeps simulated physical truth separate from production software belief:

```text
Scenario / Parameters
        |
        v
Deterministic Scheduler
        |
        v
SimulationWorld
        |
        | RawObservation
        v
Production Runtime
        |
        | AuthorizedActuation
        +---------------------> SimulationWorld
```

Simulation truth is available only to evidence and independent analysis:

```text
SimulationWorld truth
        |
        +--> Evidence / plots / reference correlation
```

Production runtime must never receive simulation truth directly.

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

Scheduling into the past is rejected. A missed runtime opportunity is recorded and skipped; it is never replayed or caught up later.

## SimulationWorld

`SimulationWorld` is the physical side of the current SITL architecture. It owns:

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

The model deliberately does not invent unavailable reference-platform facts. `parameters/reference-assembly.json` remains the source for identified platform parameters, and unknown entries remain unknown. Callers constructing `SimulationWorld` must supply a complete physical/sensor configuration; tests use explicitly synthetic fixtures only.

The current model does not claim sensor bias/noise, sensor latency, wheel slip, motor electrical dynamics, battery dynamics, or physical calibration/correlation.

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

The baseline scheduler-only CLI remains useful for deterministic execution/evidence checks when no physical parameter set is supplied:

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

CI runs the deterministic scenario twice and requires all three evidence files to be byte-identical. SITL evidence is simulation evidence, not physical validation.

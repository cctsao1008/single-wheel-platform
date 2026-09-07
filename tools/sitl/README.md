# SITL

Host-side Software-In-The-Loop execution for deterministic control-system verification.

The Stage-1 runner owns test mechanics only:

```text
integer virtual time
monotonic scheduler time
semantic-phase event ordering
external TOML scenarios
independent sensor/runtime cadence
explicit missed runtime opportunities
no replay / no catch-up
deterministic evidence generation
manifest.json + trace.jsonl + summary.json
```

## Causality

The scheduler advances from the previous timestamp to the next timestamp before dispatching events at the new time:

```text
current time = t0
    -> next timestamp = t1
    -> PhysicalTimeAdvance [t0, t1)
    -> dispatch events at t1
```

Plant integration is therefore a time-transition operation, not a queued event at `t1`.
The runner exposes a `PhysicalTimeAdvance` hook and uses a no-op implementation in Stage 1. Stage 2 replaces that no-op with the project-specific Virtual Plant integration while preserving the same scheduler semantics.

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

Scheduling into the past is rejected and insertion-sequence exhaustion is an error.

## Scenario

Scenarios are external TOML files. The baseline keeps sensor and runtime cadence explicit and deliberately misses one runtime opportunity:

```text
duration_us
seed
sensor_period_us
runtime_period_us
missed_runtime_at_us[]
```

The missed opportunity is recorded and skipped; it is never replayed later.

Run on the host with:

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

The last two fields intentionally remain distinct so future production-model assumptions and simulated physical truth can differ without losing reproducibility.

`trace.jsonl` records deterministic event order. `summary.json` records time slices, physical-time advances, sensor/runtime counts, missed opportunities, actuation commits, and pass/fail.

CI runs the same scenario twice and requires all three evidence files to be byte-identical.

Virtual Plant, Virtual Sensor Physics, production semantic-path reuse, and Virtual Physical Actuator integration are the next layer behind these execution mechanics. SITL evidence is simulation evidence, not physical validation.

# SITL

Host-side Software-In-The-Loop execution for deterministic control-system verification.

The current runner owns only test mechanics:

```text
integer virtual time
semantic-phase event ordering
no-replay control opportunities
deterministic evidence generation
manifest.json + trace.jsonl + summary.json
```

Same-time events are ordered by:

```text
virtual time
    -> semantic phase
    -> insertion sequence
```

The semantic phases are:

```text
IntegratePlantTo
PhysicalOrFaultEvent
SensorSample
ObservationDelivery
ProductionRuntime
ActuationCommit
```

The current deterministic baseline deliberately records one missed control opportunity and verifies that it is skipped rather than replayed.

Run on the host with:

```bash
cargo run -p swp-sitl --target x86_64-unknown-linux-gnu -- --output sitl-output
```

Virtual Plant, Virtual Sensor Physics, production semantic-path reuse, and Virtual Physical Actuator integration are added behind these deterministic execution mechanics. SITL output is simulation evidence, not physical validation.

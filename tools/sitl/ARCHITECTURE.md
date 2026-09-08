# SITL Architecture

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
        | RawObservation
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
        | ideal physical torque
        +-------------------------------> SimulationWorld
```

`SimulationWorld` truth is host-only evidence. It never enters the production estimator or Control.

A fresh observation may drive at most one production runtime opportunity. A missed opportunity consumes that observation as missed; there is no replay or catch-up. The last committed physical input is held until a later actuation commit changes it.

SITL is host tooling, not a fifth production domain. Synthetic integration fixtures prove software causality only and do not constitute ONE V2 physical calibration or validation.

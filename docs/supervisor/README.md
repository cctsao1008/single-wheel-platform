# Supervisor

`supervisor/` owns runtime belief, operating policy, and physical-output authority.

It estimates state from plant observations, invokes the control law, interprets timing and physical limits, manages integrator hold semantics, and is the only semantic source of `AuthorizedActuation`.

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
```

Detailed contracts:

- [`runtime.md`](runtime.md)
- [`authority.md`](authority.md)
- [`observation_time_health_replay.md`](observation_time_health_replay.md)

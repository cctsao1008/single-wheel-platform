# Control Runtime Composition

`swp-control-runtime` is the executable composition boundary between state estimation, state feedback, actuator inversion, and physical-output authority.

The runtime is estimator-agnostic:

```rust
ControlRuntime<E: BalanceStateEstimator>
```

The default implementation remains `LinearObserver` for compatibility and deterministic reference testing, while `ExtendedKalmanFilter` implements the same estimator contract. A lightweight estimator may use the same boundary when justified by measured performance.

```text
EstimatorMeasurement[k]
        |
        v
StateEstimator<E>
  uses applied u[k-1]
        |
        v
EstimatedBalanceState[k]
        |
        v
LQR / LQI
        |
        v
GeneralizedDemand[k] [N m]
        |
        v
ActuatorPairModel
        |
        v
ActuatorPairCommand[k]
        |
        v
RuntimeAuthority
        |
        +-- denied ------> applied u[k] = 0
        |
        +-- authorized --> AuthorizedActuation[k]
                              |
                              v
                       ElectricalOutput
                              |
                              v
                         PWM / DIR
```

## Causality

The selected estimator distinguishes the physical effort applied over the previous sample interval from the command being computed now. In the current zero-order-hold composition, the previous authorized plant effort is supplied to prediction and to the current local measurement-feedthrough input.

A controller request is not plant input merely because it was computed. The semantic sequence is:

```text
state feedback
  -> requested torque
  -> actuator inverse model
  -> bounded command
  -> runtime authority
  -> AuthorizedActuation
  -> electrical output
```

This preserves:

```text
requested torque
    != bounded command
    != authorized actuation
    != electrical output
    != applied plant input
```

The remembered estimator input represents the predicted physical effort associated with authorized actuation. If authority denies the step, the remembered input is zero.

## One call, one physical opportunity

One `ControlRuntime::step()` corresponds to one measurement/control opportunity. Missed real time is never recovered by executing backlog controller calls.

```text
late / missed DATA_RDY
        -> timing health / authority
        -> no catch-up execution
```

## Session reset

`ControlRuntime::reset()` clears dynamic state before a new balancing session:

```text
selected estimator state <- supplied initial state
estimator validity        <- implementation-defined invalid until correction
LQI integral state        <- zero
previous applied u        <- zero
```

For EKF, reset also restores its configured reset covariance.

## LQI anti-windup

LQI uses authority-aware two-stage evaluation. The current integral state is first held and evaluated through actuator limits and authority. Only a fully authorized, unconstrained request permits a candidate integration step. The candidate is committed only if its resulting request is also fully authorized and unconstrained.

```text
Hold request
   |-- denied/constrained -> keep integral
   v
candidate Integrate request
   |-- denied/constrained -> discard candidate
   v
commit integral
```

## Numeric design and provenance

The runtime owns no reference-platform gains. Host engineering supplies estimator/controller/actuator design data.

Reference-backed nominal values may instantiate an initial controller; local measurement and system identification then replace lower-confidence assumptions. Provenance is explicit rather than used as a software-completeness gate.

```text
reference / datasheet / literature / nominal
                    |
                    v
              host synthesis
                    |
                    v
 estimator design + controller gains + actuator model
                    |
                    v
            ControlRuntime<E>
                    |
          +---------+---------+
          |                   |
      live STM32          host replay
          |
          v
measurement / identification / correlation
          |
          +------> replace nominal assumptions
```

## Commissioning modes

Observation and shadow execution are commissioning modes around the same architecture:

```text
observation
    acquire and record sensor evidence

live-shadow
    execute estimator/control/authority with motor electrical ownership absent

closed-loop
    execute estimator/control/authority and deliver AuthorizedActuation to the electrical-output layer
```

The platform architecture is therefore complete independently of which commissioning mode is currently selected.
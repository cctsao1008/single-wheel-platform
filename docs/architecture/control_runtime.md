# Control Runtime Composition

`swp-control-runtime` is the executable composition boundary between estimation, state feedback, actuator inversion, and physical-output authority.

It does not own board peripherals, sensors, generated gains, or host-side synthesis. Its job is to make the causality of one real-time balance opportunity explicit and reusable by both the STM32 firmware and deterministic host replay.

```text
EstimatorMeasurement[k]
        |
        v
LinearObserver
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
                        predicted applied
                        physical torque u[k]
                              |
                              +---- stored for estimator[k+1]
```

## Causality

The observer distinguishes the input that drove the plant over the previous sample interval from the command being computed now. In the reference zero-order-hold composition, the previous physically authorized effort is used for both discrete prediction and the local direct-feedthrough term associated with the current measurement.

A new controller request is not considered applied merely because the controller computed it. It becomes plant input only after:

```text
state feedback
  -> actuator inverse model
  -> bounded command
  -> runtime authority
  -> AuthorizedActuation
```

If authority is denied, the remembered applied plant input for the next observer prediction is zero.

This preserves the distinction:

```text
requested torque
    != bounded command
    != authorized command
    != applied plant input
```

## One call, one physical opportunity

One `ControlRuntime::step()` call corresponds to one measurement opportunity. The caller must not execute backlog steps to make up for missed real-time periods.

```text
late / missed DATA_RDY
        |
        v
timing health / authority
```

not:

```text
late / missed DATA_RDY
        |
        v
multiple estimator/controller calls back-to-back
```

Physical time cannot be caught up by replaying computation.

## Session reset

`ControlRuntime::reset()` clears dynamic control history before a new balancing session:

```text
observer state      <- supplied captured state
observer validity   <- Invalid until a valid correction
LQI integral state  <- zero
previous applied u  <- zero
```

This prevents stale estimator, integral, or actuator history from surviving a Standby -> Balancing transition.

## LQI anti-windup

The LQI path uses authority-aware two-stage evaluation.

First the current integral state is held and the request is evaluated through actuator limits and runtime authority. Only if that request is fully authorized and unconstrained is a candidate integral update computed.

The updated request is then evaluated again. The integral state is committed only if the updated request also remains fully authorized and unconstrained.

```text
current integral
      |
      v
Hold request
      |
      +-- denied/constrained --------------------> keep integral
      |
      v
candidate Integrate request
      |
      +-- denied/constrained --------------------> discard candidate
      |
      v
fully authorized and unconstrained
      |
      v
commit new integral state
```

This prevents even one control sample of integral accumulation at the onset of actuator saturation or reaction-wheel authority limiting.

## Runtime vs numeric design

The crate contains no reference-platform numeric gains. Numeric observer and controller design remains host-generated from evidenced physical parameters in `tools/control/`.

The intended deployment path is:

```text
physical evidence
      |
      v
host synthesis
      |
      v
generated ObserverDesign / gains / actuator parameters
      |
      v
ControlRuntime
      |
      +--> STM32 live 500 Hz execution
      |
      +--> host deterministic replay
```

Until those physical inputs are evidenced, the STM32 reference firmware remains observation-only even though the executable control composition itself is implemented and tested.

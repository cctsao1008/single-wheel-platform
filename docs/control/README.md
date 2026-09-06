# Control

`control/` owns desired closed-loop behavior.

The current controller consumes estimated state and reference and produces physical generalized demand:

```text
EstimatedState + Reference
            |
            v
        LQR / LQI
            |
            v
    GeneralizedDemand
```

The state-feedback form is:

```text
u = u_ff - K (x_hat - x_ref)
```

Control does not own sensing, operating state, timing health, actuator authorization, or electrical output. Those responsibilities remain outside the control law.

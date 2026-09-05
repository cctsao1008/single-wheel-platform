# Actuator Authority

Controller computation and physical motor access are different responsibilities.

```text
maintenance request ----\
                         > Motor Authority -> board_motor -> actuator
control request --------/
fault condition --------/
```

A minimal ownership model is:

```c
typedef enum
{
    MOTOR_OWNER_NONE = 0,
    MOTOR_OWNER_MAINTENANCE,
    MOTOR_OWNER_CONTROL,
    MOTOR_OWNER_FAULT
} motor_owner_t;
```

Key invariant: only one software ownership boundary may command a physical actuator at a time.

The final electrical safe state — coast, brake, standby, or another behavior — belongs to the hardware contract and must be defined per actuator.

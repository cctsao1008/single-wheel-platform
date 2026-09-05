# Actuator Authority

Controller computation and physical actuator ownership are separate responsibilities.

```text
maintenance request ----\
                         > authority / limits -> typed actuator owner -> hardware
control request --------/
fault condition --------/
```

The architectural invariant is that only one runtime owner can mutate the physical PWM/direction resources for an actuator at a time.

In the Rust implementation this is represented by ownership of the concrete HAL peripheral and pin types rather than by globally reachable timer registers or a numeric owner flag. RTIC shared/local resources provide controlled access when ownership must cross task boundaries.

The final electrical safe state — PWM inactive level, coast, brake, standby or another behavior — is a board/actuator property. It must not be guessed from a generic `enabled` boolean when the hardware polarity is not established.

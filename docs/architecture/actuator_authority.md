# Actuator Authority

Controller computation and physical actuator ownership are separate responsibilities.

```text
maintenance request ----\
                         > authority / limits -> typed actuator owner -> hardware
control request --------/
fault condition --------/
```

The architectural invariant is that only one runtime owner can mutate the physical PWM/direction resources for an actuator at a time.

`RuntimeAuthority` is the semantic promotion boundary. Only `AuthorizedActuation` may enter the electrical-output layer; raw normalized commands are not accepted by the physical-output API.

For the reference ONE V2 assembly, concrete mutation is bound to:

```text
DriveWheel     -> BLDC_2 -> PA6 / TIM3_CH1 + PA4 DIR
ReactionWheel  -> BLDC_1 -> PB1 / TIM3_CH4 + PB11 DIR
```

`swp-stm32f103-electrical-output::MotorElectricalOutputs` owns those exact HAL timer-channel and GPIO types. RTIC or another target runtime may own the resulting object, but unrelated code does not receive globally reachable timer registers or a numeric owner flag.

The board's BLDC enable nets are hard-wired high, so software authority must be enforced at PWM/direction ownership rather than by pretending there is a controllable `motor_enabled` GPIO.

The final electrical safe state — PWM inactive level, coast, brake, disabled timer output or another behavior — is a board/actuator property. It must not be guessed from a generic `enabled` boolean. Channel-disable behavior and mechanical direction signs remain commissioning evidence.

The canonical line encoding and commissioning boundary are defined in [`electrical_output.md`](electrical_output.md).

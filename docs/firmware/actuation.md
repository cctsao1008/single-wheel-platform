# Actuation

Physical output begins only after Supervisor creates an `AuthorizedActuation` token.

```text
ActuatorPairCommand
      |
      v
RuntimeAuthority
      |
      +-- denied ------> no token
      |
      +-- authorized --> AuthorizedActuation
                              |
                              v
                         ActuationSink
                              |
                              v
                       actuator adapter
                              |
                     actuator-specific frame
                              |
                              v
                      ActuatorIo<Frame>
                              |
                              v
                    control-board backend
                              |
                              v
                       physical interface
```

`ActuationSink` is target-independent. It accepts no raw `NormalizedCommand`. `ActuatorIo<Frame>` is lower-level and separates actuator electrical/protocol semantics from the MCU mechanism used to emit the frame.

## Portability boundary

A control-board change and an actuator-hardware change are independent architectural operations.

```text
new control board
    -> add/replace board description + target backend

new motor-driver / actuator-interface board
    -> add/replace actuator adapter and its Frame

Plant / Supervisor / Control
    -> unchanged unless the physical plant or control problem itself changes
```

For actuator hardware that can reuse an existing frame contract, an RP2350 backend may implement the same `ActuatorIo<Frame>` currently implemented by STM32F103. Interfaces using SPI, CAN, 3-PWM, or another mechanism may define a different frame while retaining the same `ActuationSink` boundary.

## Current ONE V2 composition

```text
firmware/interfaces/actuation
        |
        v
firmware/actuators/one-v2-pwm-dir
        |
  ElectricalActuation
        |
        v
firmware/targets/stm32f103/one-v2-pwm-dir
        |
        v
TIM3 PWM + DIR GPIO
```

The installed actuator association is defined separately by:

```text
firmware/assemblies/one-v2-reference
```

and the control-board wiring by:

```text
firmware/boards/one-v2
```

Board identity, assembly role, actuator semantics, and MCU backend are intentionally separate facts.

## Current electrical resources

| Robot role | PCB interface | PWM | Direction |
|---|---|---|---|
| Reaction wheel | BLDC_1 / M2 | PB1 / TIM3_CH4 | PB11 |
| Drive wheel | BLDC_2 / M1 | PA6 / TIM3_CH1 | PA4 |

BLDC_3 is not installed in the reference assembly.

The BLDC enable nets are hard-wired to 3.3 V on the reviewed board. Runtime authority therefore cannot rely on a software motor-enable GPIO.

## ONE V2 PWM/DIR encoding

The V2.0 executable source configures TIM3 with `ARR=7199`, `PSC=0` at a 72 MHz timer clock, giving a 10 kHz carrier.

The canonical actuator adapter represents the physical line encoding as:

```text
m = abs(normalized_command)

DIR high            when command < 0
DIR low             otherwise
PWM line high       fraction = 1 - m
PWM line low        fraction = m
```

Zero effort is therefore:

```text
DIR low
PWM line continuously high
```

The vendor reaction-wheel `+100` timer-count term is not part of the canonical actuator encoding. Dead zone, static friction, and minimum effective effort belong in the Plant actuator model and require measured or identified evidence.

## Revocation

`ActuationSink::revoke()` must replace any previously applied command with the concrete actuator interface's configured zero-demand / neutral encoding. It exists so a target runtime does not leave stale physical effort latched when Supervisor withdraws authority or the control opportunity disappears.

Revocation is not a universal electrical-safe-state claim. For the current ONE V2 interface, the external behavior of zero command and disabled PWM channels still requires commissioning evidence.

## STM32F103 backend

`firmware/targets/stm32f103/one-v2-pwm-dir` owns only concrete HAL mutation:

```text
TIM3_CH1  Drive PWM
TIM3_CH4  Reaction PWM
PA4       Drive DIR
PB11      Reaction DIR
```

It implements `ActuatorIo<ElectricalActuation>`. Actuator polarity and authority are not duplicated in the target backend.

Its constructor requires the exact HAL resource types but does not configure or enable TIM3. PWM enable remains an explicit commissioning action.

## Commissioning boundary

The observation and live-shadow targets do not instantiate the motor backend. Before a closed-loop target enables physical channels, commissioning must establish at minimum:

```text
PWM pin idle waveform
PWM effort polarity
DIR-to-mechanical sign for both installed actuators
zero-command behavior
motor/ESC response around zero
behavior when the PWM channel is disabled
```

`disable_channels()` remains a target timer operation, not a claimed universal motor safe state.

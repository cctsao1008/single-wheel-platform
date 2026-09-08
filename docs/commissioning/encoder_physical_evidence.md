# Encoder Physical Evidence Contract

The first physical commissioning step establishes encoder mechanical scale and sign without motor actuation.

## Scope

The reference assembly mapping is:

```text
Encoder_1 / TIM2 -> reaction wheel
Encoder_2 / TIM4 -> drive wheel
```

The STM32F103 `observation` target is the evidence source. It samples the actual TIM2/TIM4 quadrature counters at 100 Hz and emits canonical `RecordedObservation` records over USART2. TIM3 and all motor GPIO remain untouched by that target.

## Positive mechanical directions

The mechanical direction is defined before looking at counter sign:

```text
drive wheel     positive relative rotation is the rotation that produces +X forward rolling
                when body pitch is held fixed

reaction wheel  positive relative rotation is about body +X by the right-hand rule
```

Encoder electrical phase order does not define these signs.

## One-revolution evidence

For each installed encoder:

1. Keep motor actuation electrically unavailable. Support the robot so the selected wheel can be rotated manually without unintended ground motion.
2. Mark one unambiguous mechanical reference position on the controlled rotating member and its stationary reference.
3. Start a raw `RecordedObservation` capture using the `observation` firmware.
4. Rotate exactly one mechanical revolution in the defined positive direction, return to the mark, and note the start/end record sequence numbers.
5. Repeat in the negative direction. Additional positive/negative repeats are preferred.
6. Preserve the raw binary capture; do not replace it with hand-entered count values.
7. Run `tools/commissioning/encoder_evidence.py` with a plan that binds the exact sequence windows to their declared mechanical directions.

Example plan:

```json
{
  "schema": 1,
  "assembly": "reference-assembly",
  "encoder_channel": "encoder_2",
  "mechanical_coordinate": "drive_wheel_relative_angle",
  "positive_direction_definition": "rotation that produces +X forward rolling when body pitch is held fixed",
  "trials": [
    {"name": "positive-1", "direction": "positive", "start_sequence": 120, "end_sequence": 260},
    {"name": "negative-1", "direction": "negative", "start_sequence": 410, "end_sequence": 550}
  ]
}
```

Run:

```text
python3 tools/commissioning/encoder_evidence.py capture.bin --plan drive-plan.json --output drive-evidence.json
```

## Acceptance semantics

The tool rejects a trial if:

```text
record sequence is discontinuous
a dropped-record event occurs inside the window
encoder AVAILABLE / IO_OK / TIMING_VALID evidence is missing
IO_ERROR or STALE is present
an individual modular counter step is exactly 32768 counts and therefore ambiguous
```

The tool unwraps the 16-bit timer counter only under the commissioning constraint that each observed manual-motion step is strictly less than half the counter range.

A `counts_per_mechanical_revolution_candidate` is emitted only when positive and negative one-revolution trials have opposite counter signs and identical absolute net counts. The inferred `counter_sign_for_mechanical_positive` reports whether the STM32 count increases (`+1`) or decreases (`-1`) for the declared positive mechanical rotation.

This output is still an evidence candidate. It becomes accepted physical truth only after the mark/revolution definition, assembly identity, raw capture hash, channel mapping, and trial windows are reviewed.

## Delta-bound boundary

The same capture reports `observed_manual_run_max_abs_step_counts`, but this value is **not** `encoder_max_abs_delta_counts_per_sample`.

The runtime anti-alias/wrap bound requires an evidenced maximum mechanical speed and the accepted counts/revolution at the actual sampling period. Manual commissioning motion cannot establish that operating envelope.
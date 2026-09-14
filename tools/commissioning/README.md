# Commissioning Tools

Host-side commissioning tools convert bounded physical experiments into reproducible evidence candidates. They do not write accepted physical parameters automatically.

```text
encoder_preregistration.py   validate and fingerprint pre-capture encoder hypotheses
encoder_evidence.py          analyze manually marked one-revolution encoder trials
```

## Encoder sign / counts-per-revolution commissioning

The current mainline physical experiment is intentionally non-actuating. It uses the existing STM32F103 `observation` target and manual wheel rotation only.

Reference-assembly bindings are fixed before capture:

```text
Encoder_1 / TIM2 -> ReactionWheel
Encoder_2 / TIM4 -> DriveWheel
```

Project-defined physical positive directions are also fixed before capture:

```text
DriveWheel     relative wheel rotation that produces +X forward rolling
               when body pitch is held fixed

ReactionWheel  relative wheel rotation about body +X by the right-hand rule
```

The committed preregistrations live under `tools/commissioning/plans/`:

```text
drive-encoder-one-rev.preregistration.json
reaction-encoder-one-rev.preregistration.json
```

Validate and fingerprint one before looking at a capture:

```bash
python3 tools/commissioning/encoder_preregistration.py \
  tools/commissioning/plans/drive-encoder-one-rev.preregistration.json

python3 tools/commissioning/encoder_preregistration.py \
  tools/commissioning/plans/drive-encoder-one-rev.preregistration.json \
  --hash-only
```

Preserve that SHA-256 with the lab/capture record. The exact raw counter polarity is deliberately left `unknown` where no accepted physical evidence exists. The pre-capture falsifiable expectations are instead structural: a marked positive revolution must produce nonzero motion, the marked negative revolution must reverse counter sign, and repeated exact one-revolution trials must agree in absolute count magnitude.

After capture, select the exact record sequence ranges for the positive and negative marked revolutions and analyze them with `encoder_evidence.py`. That tool verifies record continuity, encoder quality, 16-bit wrap handling, bidirectional sign consistency, and repeated one-revolution count magnitude before emitting a candidate result.

The observed slow manual-run maximum count step is evidence only. It is **not** `encoder_max_abs_delta_counts_per_sample`; that bound still requires an evidenced runtime speed envelope.

The output remains evidence. Promotion into `parameters/reference-assembly.json` is a separate review step governed by `parameters/reference-assembly-evidence.json`.

> Measure the direction first. Never repair a physical sign mistake with a prettier controller sign.

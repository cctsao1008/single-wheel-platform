# Physical Parameter Registry

`reference-assembly.json` is the machine-readable evidence boundary for physical quantities used by plant, measurement, actuator, estimator, and controller synthesis.

Each field carries both a numeric `value` and an evidence classification. `null` means the quantity is not yet supported strongly enough to become a reference-assembly fact.

Allowed evidence classes are:

```text
unknown
measured
identified
datasheet
derived
```

A value is not promoted merely because legacy firmware, a vendor comment, or a nominal component specification contains a plausible number. The consuming synthesis path must reject required `null` quantities instead of substituting defaults.

The registry is intentionally about current physical truth, not measurement history. Raw identification datasets and scripts remain under `tools/` or recorded-observation storage; this file contains the accepted result only.

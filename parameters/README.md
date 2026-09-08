# Physical Parameter Registry

`reference-assembly.json` is the machine-readable accepted-truth boundary for physical quantities used by plant, measurement, actuator, estimator, and controller synthesis.

Each registry field carries both a numeric `value` and an evidence classification. `null` means the quantity is not yet supported strongly enough to become a reference-assembly fact.

Allowed accepted-registry evidence classes are:

```text
unknown
measured
identified
datasheet
derived
```

A value is not promoted merely because legacy firmware, a vendor comment, a nominal component specification, or a plausible calculation contains a number. The consuming synthesis path must reject required `null` quantities instead of substituting defaults.

`reference-assembly-evidence.json` is the machine-readable provenance boundary for candidate claims, conflicting sources, source hashes, and the evidence still required for promotion. Entries in that file do **not** become physical truth merely by being recorded there.

The contract is:

```text
source artifact / bench evidence
        ↓
reference-assembly-evidence.json
        ↓  promotion only after the parameter definition is resolved
           and conflicting evidence is closed
reference-assembly.json
        ↓
model / estimation / synthesis / runtime
```

The registry is intentionally about current physical truth, not measurement history. Raw identification datasets and scripts remain under `tools/` or recorded-observation storage; the accepted registry contains the accepted result only.

CI runs `tools/model/check_parameter_provenance.py` so every registry leaf is classified exactly once as `known`, `derived`, or `unknown`, and a future numeric promotion cannot silently leave provenance classification stale.

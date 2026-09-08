# Plant Model Tooling

This directory contains host-side mathematical references for the canonical reduced balance plant. It is not firmware and it does not supply missing ONE V2 physical parameters.

The coordinate and input contract is:

```text
q = [s, theta, phi, psi_r]^T
x = [s, s_dot, theta, theta_dot, phi, phi_dot, psi_r_dot]^T
u = [tau_drive, tau_reaction]^T
```

`psi_r` is the reaction-wheel angle relative to the robot body. The canonical production reduced state keeps `psi_r_dot`; SITL additionally retains relative wheel phase as simulation truth so encoder evidence can be synthesized.

## Symbolic derivation

`derive_balance_model.py` uses SymPy to make the current mechanical assumptions executable and inspectable. It derives and prints:

```text
M(q)
c(q, q_dot)
g(q)
B
upright M_0
upright gravity stiffness
pitch controllability determinant
roll controllability determinant
open-loop unstable modal rates
```

Run it with:

```bash
python -m pip install -r tools/model/requirements.txt
python tools/model/derive_balance_model.py
```

The derivation corresponds to [`docs/architecture/dynamics_model.md`](../../docs/architecture/dynamics_model.md). If the physical model changes, the document and symbolic source change together; Git preserves the history.

## Independent numeric reference

`reference_balance.py` is an independent numeric oracle for the stationary-upright reduced plant currently used by `tools/sitl/SimulationWorld`.

The two implementations deliberately use different numerical engines:

```text
Rust SimulationWorld   f32 + deterministic RK4
Python reference       float64 + exact matrix exponential / ZOH
```

Both rebuild the plant from the same explicit physical parameter fixture. The comparison therefore detects sign, matrix, input-map, state-order, and integration drift without making Python part of the production runtime.

The repository correlation fixture is:

```text
tools/model/fixtures/synthetic_correlation.json
```

It is explicitly synthetic and exists only to prove cross-implementation consistency. It is not a ONE V2 calibration or validation data set.

Run the reference alone:

```bash
python tools/model/reference_balance.py \
  --fixture tools/model/fixtures/synthetic_correlation.json
```

Run cross-language correlation:

```bash
cargo run -p swp-sitl --bin plant_reference_trace \
  --target x86_64-unknown-linux-gnu -- \
  --fixture tools/model/fixtures/synthetic_correlation.json \
  --output /tmp/swp-rust-plant-trace.json

python tools/model/reference_balance.py \
  --fixture tools/model/fixtures/synthetic_correlation.json \
  --rust-trace /tmp/swp-rust-plant-trace.json \
  --max-abs-error 5e-6
```

Numeric parameter fitting, physical correlation, estimator synthesis, controller synthesis, plotting, and higher-fidelity plant effects remain downstream of this reference contract. Unknown physical values stay unknown until measured, identified, or supported by a trusted reference.

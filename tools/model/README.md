# Symbolic Plant Model

This directory contains host-side symbolic derivation for the canonical reduced balance plant.

The symbolic model is not firmware and does not contain guessed numeric parameters. Its role is to make the current mechanical assumptions executable and inspectable.

## Run

```bash
python -m pip install -r tools/model/requirements.txt
python tools/model/derive_balance_model.py
```

The script derives and prints:

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

The coordinate contract is:

```text
q = [s, theta, phi, psi_r]^T
u = [tau_drive, tau_reaction]^T
```

`psi_r` is the reaction-wheel angle relative to the robot body.

The derivation corresponds to [`docs/architecture/plant_model.md`](../../docs/architecture/plant_model.md). If the physical model changes, the document and symbolic source change together; Git preserves the history.

Numeric parameter fitting, correlation, estimator synthesis, and controller synthesis belong downstream of this symbolic contract.

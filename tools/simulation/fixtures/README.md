# Simulation fixtures

`synthetic-rigidbody-correlation.json` is the synthetic parameter source used by the cross-backend open-loop correlation suite. Its body and wheel inertias are analytically derived from the committed bootstrap Webots box/cylinder geometry so the reduced analytical/Rust model and the rigid-body backend no longer compare different inertial parameter sets by accident.

It is simulation evidence only. None of its values are ONE V2 physical facts, and nothing in this directory may promote values into `parameters/reference-assembly.json`.

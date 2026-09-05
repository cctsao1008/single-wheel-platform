# Model-Based Control Synthesis

`tools/control/synthesize_upright.py` is the canonical host-side synthesis path for the stationary-upright balance design.

It consumes evidenced physical parameters and design scales, then produces the exact 500 Hz zero-order-hold plant together with observer and LQR/LQI gains.

```text
measured / identified physical parameters
        |
        v
continuous upright plant A, B
        |
        +--> physical measurement model y0, C, D
        |
        v
exact ZOH @ configured sample period
        |
        v
A_d, B_d
        |
        +--> discrete LQR / optional LQI
        |
        +--> steady-state discrete observer gain
        |
        v
JSON design artifact + generated Rust constants
```

The tool deliberately refuses missing or non-finite physical parameters. The reference template contains `null` for quantities that are not yet supported by measured or identified evidence. Those fields are not placeholders for textbook guesses.

## Dependencies

```bash
python -m pip install -r tools/control/requirements.txt
```

## Design input

Start from:

```text
tools/control/reference_design.template.json
```

The fixed state order is:

```text
[
    forward_position_m,
    forward_velocity_m_per_s,
    pitch_rad,
    pitch_rate_rad_per_s,
    roll_rad,
    roll_rate_rad_per_s,
    reaction_wheel_rate_rad_per_s,
]
```

The physical input order is:

```text
[
    drive_torque_nm,
    reaction_wheel_torque_nm,
]
```

The measurement order is:

```text
[
    accel_x_m_per_s2,
    accel_y_m_per_s2,
    accel_z_m_per_s2,
    gyro_x_rad_per_s,
    gyro_y_rad_per_s,
    gyro_z_rad_per_s,
    drive_encoder_relative_angle_rad,
    reaction_wheel_relative_rate_rad_per_s,
]
```

`lqr.state_scale` and `lqr.input_scale_nm` are physical design scales, not legacy PID gains. They produce Bryson-form diagonal `Q` and `R` weights.

The observer uses explicit process and measurement standard deviations plus an explicit required-measurement set. Only selected channels participate in the observer correction; generated gain columns for unselected channels are zero.

Optional LQI adds two explicit integral coordinates using `lqi.integral_projection`. The projection defines which linear combinations of the seven-state regulation error are integrated.

## Synthesis

```bash
python tools/control/synthesize_upright.py control-design.json \
  --json-output generated/upright-design.json \
  --rust-output generated/upright_design.rs
```

The generated JSON reports the closed-loop spectral radius of the LQR/LQI design and the estimator error spectral radius. Synthesis fails if either discrete-time design is unstable.

The generated Rust file contains only matrices and the required-measurement bit mask. It does not authorize actuation. Firmware runtime authority remains a separate boundary.

## Design ownership

```text
plant-model
    physical equations and state/input definition

measurement-model
    physical sensor equation

tools/control
    numerical ZOH / Riccati synthesis and design correlation

state-estimator
    deterministic real-time predictor/corrector execution

state-feedback
    deterministic real-time LQR/LQI execution

runtime-state
    validity, limits, momentum headroom, and physical-output authority
```

Numeric controller/observer gains become reference-platform facts only after the design input is backed by physical measurement or identification and the resulting model is correlated against the robot.

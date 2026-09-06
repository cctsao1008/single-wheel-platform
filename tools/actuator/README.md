# Actuator Identification

The actuator boundary converts physical torque demand into normalized motor command. Parameters are identified from data rather than inherited from legacy PWM gains.

`identify_static.py` fits the current static model:

```text
torque = K_u * effective(command, deadzone)
         - b * speed
         - tau_c * sign(speed)
```

Input CSV columns:

```text
command,speed_rad_s,torque_nm
```

The command is normalized to `[-1, 1]`. Speed is actuator-relative angular velocity in rad/s. Torque must be a physically supported estimate or measurement in N m; this tool does not fabricate torque from PWM.

Example:

```bash
python3 tools/actuator/identify_static.py run.csv --output drive-actuator.json
```

The fit performs a bounded dead-zone search and least-squares estimation of torque gain, viscous friction, and Coulomb friction. Fits with negative torque gain or negative friction coefficients are rejected.

The resulting numbers are evidence candidates. They become reference-assembly parameters only after the identification dataset and model residuals are accepted.

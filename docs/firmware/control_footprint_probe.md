# Control Footprint Probe

The STM32F103 observation target remains non-actuating until physical evidence is sufficient to instantiate the estimator, controller, encoder transfer, actuator model, and output path with commissioned parameters.

A separate non-production binary measures the linked cost of the complete control path:

```text
firmware/targets/stm32f103/control-footprint
```

It links and executes:

```text
EstimatorMeasurement
      |
      v
State Estimator
      |
      v
LQR
      |
      v
GeneralizedDemand
      |
      v
Actuator Inverse Model
      |
      v
RuntimeAuthority
```

The probe uses a synthetic numeric fixture only to keep the complete path reachable by the linker. The fixture is not reference-platform evidence, is not used by the observation firmware, and must not be copied into `parameters/` or a generated control design.

The probe owns no motor peripherals and cannot create physical output.

Build both images with:

```bash
cargo fw-observation
cargo fw-control-footprint
```

Or compare them directly on a host with the ARM GNU binutils installed:

```bash
bash tools/firmware/footprint.sh
```

The meaningful comparison is:

```text
observation-only image size
vs.
full control-path probe image size
```

This establishes the incremental linked cost of the control brain without weakening the evidence rule that unknown physical parameters remain unknown.

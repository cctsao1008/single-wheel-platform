# Control Footprint Probe

The reference STM32F103 firmware remains observation-only until physical evidence is sufficient to instantiate the estimator, controller, encoder transfer, actuator model, and runtime authority with reference-platform values.

A separate non-production binary exists to measure the linked cost of the complete control path before those physical values are available:

```text
firmware/control-footprint-stm32f103
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

The probe uses a synthetic numeric fixture only to keep the complete path reachable by the linker. The fixture is not reference-platform evidence, is not used by the production firmware, and must not be copied into `parameters/` or a generated control design.

The probe owns no motor peripherals, does not configure TIM3 or motor GPIO, and cannot create an electrical output. It exists only for Cortex-M3 linkage and Flash/RAM footprint measurement through the same CMSIS-DSP numerical backend used by the control crates.

Build both images with:

```bash
cargo fw
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

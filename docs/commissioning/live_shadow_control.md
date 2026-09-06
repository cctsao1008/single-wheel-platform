# Live Shadow-Control Profiler

The live shadow-control profiler measures the real STM32F103C8 execution budget of the sensor-driven model-based control path without owning any motor peripheral.

It is a commissioning instrument, not a controller configuration.

## Purpose

The profiler answers the timing question left open by static firmware size measurements:

```text
MPU6050 DATA_RDY @ 500 Hz
        |
        v
software-I2C IMU read
        |
        v
TIM2/TIM4 encoder snapshot
        |
        v
synthetic numeric semantic projection
        |
        v
state estimator / CMSIS-DSP
        |
        v
LQR
        |
        v
actuator inverse model
        |
        v
RuntimeAuthority
        |
        v
AuthorizedActuation token, if any, is discarded
```

The 500 Hz deadline is 2 ms, or 144,000 DWT cycles at 72 MHz.

## Safety Boundary

`firmware/live-shadow-stm32f103` deliberately does not configure TIM3 or any motor GPIO. It cannot convert an `AuthorizedActuation` token into electrical output.

The shadow observer also retains zero previous applied input on every step because no physical motor effort was applied. A synthetic authority decision must never be mistaken for plant actuation.

## Evidence Boundary

The profiler uses real MPU6050 register data, real DATA_RDY timing, and real TIM2/TIM4 counter snapshots.

The following values are intentionally synthetic and exist only to force ordinary numerical branches through the embedded implementation:

- affine IMU profile transforms;
- sensor-to-profile-body rotation;
- encoder counts-per-revolution scale;
- observer matrices and gain;
- LQR gain;
- actuator parameters;
- reaction-wheel speed limits.

The profiler does not construct evidence-bearing `CalibratedImuObservation`, `BodyImuObservation`, or reference-platform parameter records from those synthetic values. None of its numeric output is control-design evidence.

## Recorded Timing

Each 20 Hz `KIND_CONTROL_PROFILE` record contains the latest 500 Hz step timing plus window and boot maxima:

```text
imu_read_cycles
encoder_snapshot_cycles
semantic_projection_cycles
estimator_cycles
feedback_cycles
actuator_authority_cycles
critical_path_cycles
window_max_critical_path_cycles
boot_max_critical_path_cycles
deadline_cycles
overrun_count
```

`critical_path_cycles` starts at EXTI13 service entry and ends after RuntimeAuthority evaluation. The independent 1 kHz TIM1 sensor-health interrupt remains enabled at higher priority, so preemption appears naturally in measured critical-path jitter.

## Build and Flash

```bash
cargo fw-live-shadow
cargo run-live-shadow
```

The configured runner uses `probe-rs` with `STM32F103C8` over SWD.

## Capture and Decode

Capture the USART2 / ECB02S2 binary stream with the existing wireless capture path or another byte-preserving capture method, then decode:

```bash
python3 tools/recording/decode_control_profile.py capture.bin > control-profile.csv
```

The decoder converts cycles to microseconds using the `cpu_hz` carried in each record and calculates instantaneous deadline headroom.

## Decision Rule

Do not optimize from code size or intuition. Use measured `boot_max_critical_path_cycles`, the distribution of `critical_path_cycles`, and observed overruns.

The reference working rule is to retain substantial margin below the 2 ms deadline for interrupt jitter, software-I2C variation, future actuator service, and fault handling. If the measured full critical path remains comfortably below the deadline, there is no evidence-based reason to replace STM32F103 or convert the control path to fixed point.

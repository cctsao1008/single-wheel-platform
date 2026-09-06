# Estimator Input Boundary

`swp-estimator-input` converts evidenced body-frame sensing and robot-semantic encoder observations into the exact measurement vector consumed by the upright observer.

```text
Raw IMU registers
      |
      v
sensor transfer functions
      |
      v
measured IMU calibration
      |
      v
sensor -> body rotation
      |
      v
BodyImuObservation -----------------------+
                                           |
Raw QEI counters                           |
      |                                    |
      v                                    |
evidenced count/revolution + sign          |
      |                                    |
      v                                    |
16-bit unwrap + rate                        |
      |                                    |
      +--> drive relative angle ------------+
      |                                    |
      +--> reaction-wheel relative rate ----+
                                           |
                                           v
                                  EstimatorMeasurement
                                           |
                                           v
                                      state-estimator
```

## Measurement order

The adapter follows the measurement-model contract exactly:

```text
0  body accel X [m/s^2]
1  body accel Y [m/s^2]
2  body accel Z [m/s^2]
3  body gyro X [rad/s]
4  body gyro Y [rad/s]
5  body gyro Z [rad/s]
6  drive-wheel motor-relative angle [rad]
7  reaction-wheel relative rate [rad/s]
```

No raw count, board-channel identity, or sensor-frame axis is promoted directly into this vector.

## Encoder transfer evidence

`EncoderTransfer` requires:

```text
counter counts per mechanical revolution
mechanical positive direction relative to counter direction
maximum evidenced absolute counter delta per sample
transfer evidence revision / basis
```

`counter_counts_per_revolution` means what the STM32 timer sees per mechanical revolution of the controlled shaft. Encoder lines, timer quadrature multiplication, gearbox ratio, and any external gearing must already be resolved before this value becomes a reference-platform fact.

The 16-bit timer count is unwrapped with modular subtraction. That operation is only physically unambiguous when the true inter-sample motion is known to stay below half the counter range. The configured `max_abs_delta_counts_per_sample` is therefore an explicit anti-aliasing contract and must remain below 32768 counts.

A sample outside that evidenced bound is rejected. The tracker state is not advanced by a rejected sample.

## Capture-window semantics

The first accepted counter sample defines the relative angle origin:

```text
first counter sample
    drive relative angle = 0
    rate = unavailable
```

Rate becomes available only after a second accepted sample with a later timestamp. The estimator input therefore does not invent zero speed at startup.

A new control session calls `EstimatorInputBuilder::reset()` before capture. This clears encoder origins and rate history so a later balancing session cannot inherit a stale unwrapped angle or derivative.

## Timing and availability

The IMU channels are available only when their body-frame observation carries usable measurement quality. Primary estimator timing is valid only when the IMU observation is both timing-valid and freshness-verified.

Encoder failures do not become numeric zeroes. They remove the affected measurement channel from the `MeasurementMask` and preserve an explicit `EncoderChannelStatus` describing whether the channel is:

```text
Primed
Ready
Rejected(error)
```

The observer then applies its own required-measurement contract. Missing required channels invalidate estimation rather than creating fictitious continuity.

## Runtime consequence

The live 500 Hz control composition should read the hardware QEI counters on each control opportunity even though BLE recording remains decimated. Reading an encoder at the 100 Hz recording boundary is not the same thing as providing 500 Hz estimator evidence.

The current reference firmware remains observation-only and does not instantiate this adapter because the required encoder counts/revolution, signs, unwrap bounds, IMU calibration, and frame evidence are still unknown in the canonical parameter registry.

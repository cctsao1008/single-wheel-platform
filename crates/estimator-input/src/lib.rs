#![no_std]

use swp_frame_transform::BodyImuObservation;
use swp_measurement_model::{
    ACCEL_X, ACCEL_Y, ACCEL_Z, DRIVE_ENCODER_RELATIVE_ANGLE, GYRO_X, GYRO_Y, GYRO_Z,
    REACTION_WHEEL_RELATIVE_RATE, UPRIGHT_MEASUREMENT_COUNT,
};
use swp_plant_observation::{MeasurementQuality, RawEncoderObservation};
use swp_sensor_calibration::encoder::{
    EncoderKinematicObservation, EncoderTracker, EncoderTrackingError, EncoderTransfer,
};
use swp_state_estimator::{EstimatorMeasurement, MeasurementMask};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EncoderChannelStatus {
    Primed,
    Ready,
    Rejected(EncoderTrackingError),
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EstimatorInputFrame {
    pub measurement: EstimatorMeasurement,
    pub drive_encoder_status: EncoderChannelStatus,
    pub reaction_encoder_status: EncoderChannelStatus,
}

/// Stateful semantic adapter from body-frame sensor evidence into the exact
/// measurement vector consumed by the upright observer.
///
/// Board channel identities are intentionally absent. The caller must already
/// have mapped installed Encoder_1/Encoder_2 hardware into the robot-semantic
/// drive/reaction roles through the reference assembly.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EstimatorInputBuilder {
    drive_encoder: EncoderTracker,
    reaction_encoder: EncoderTracker,
}

impl EstimatorInputBuilder {
    pub const fn new(drive_transfer: EncoderTransfer, reaction_transfer: EncoderTransfer) -> Self {
        Self {
            drive_encoder: EncoderTracker::new(drive_transfer),
            reaction_encoder: EncoderTracker::new(reaction_transfer),
        }
    }

    /// Clear encoder origins/rate history before a new capture window.
    pub fn reset(&mut self) {
        self.drive_encoder.reset();
        self.reaction_encoder.reset();
    }

    pub fn build(
        &mut self,
        imu: BodyImuObservation,
        drive_encoder: RawEncoderObservation,
        reaction_encoder: RawEncoderObservation,
    ) -> EstimatorInputFrame {
        let mut values = [0.0; UPRIGHT_MEASUREMENT_COUNT];
        let mut available = MeasurementMask::NONE;

        let imu_usable = measurement_usable(imu.quality);
        let imu_timing_valid = imu_usable
            && imu.quality.contains(MeasurementQuality::TIMING_VALID)
            && imu.quality.contains(MeasurementQuality::FRESHNESS_VERIFIED);

        if imu_usable {
            values[ACCEL_X] = imu.acceleration.0[0];
            values[ACCEL_Y] = imu.acceleration.0[1];
            values[ACCEL_Z] = imu.acceleration.0[2];
            values[GYRO_X] = imu.angular_rate.0[0];
            values[GYRO_Y] = imu.angular_rate.0[1];
            values[GYRO_Z] = imu.angular_rate.0[2];
            for index in [ACCEL_X, ACCEL_Y, ACCEL_Z, GYRO_X, GYRO_Y, GYRO_Z] {
                if values[index].is_finite() {
                    available = available.with(index);
                }
            }
        }

        let (drive_encoder_status, drive_sample) =
            observe_encoder(&mut self.drive_encoder, drive_encoder);
        if let Some(sample) = drive_sample {
            if sample.relative_angle_rad.is_finite() {
                values[DRIVE_ENCODER_RELATIVE_ANGLE] = sample.relative_angle_rad;
                available = available.with(DRIVE_ENCODER_RELATIVE_ANGLE);
            }
        }

        let (reaction_encoder_status, reaction_sample) =
            observe_encoder(&mut self.reaction_encoder, reaction_encoder);
        if let Some(rate) = reaction_sample.and_then(|sample| sample.relative_rate_rad_per_s) {
            if rate.is_finite() {
                values[REACTION_WHEEL_RELATIVE_RATE] = rate;
                available = available.with(REACTION_WHEEL_RELATIVE_RATE);
            }
        }

        EstimatorInputFrame {
            measurement: EstimatorMeasurement::new(values, available, imu_timing_valid),
            drive_encoder_status,
            reaction_encoder_status,
        }
    }
}

fn observe_encoder(
    tracker: &mut EncoderTracker,
    raw: RawEncoderObservation,
) -> (EncoderChannelStatus, Option<EncoderKinematicObservation>) {
    match tracker.observe(raw) {
        Ok(sample) => {
            let status = if sample.relative_rate_rad_per_s.is_some() {
                EncoderChannelStatus::Ready
            } else {
                EncoderChannelStatus::Primed
            };
            (status, Some(sample))
        }
        Err(error) => (EncoderChannelStatus::Rejected(error), None),
    }
}

fn measurement_usable(quality: MeasurementQuality) -> bool {
    quality.contains(MeasurementQuality::AVAILABLE)
        && quality.contains(MeasurementQuality::IO_OK)
        && !quality.contains(MeasurementQuality::IO_ERROR)
        && !quality.contains(MeasurementQuality::STALE)
}

#[cfg(test)]
mod tests {
    use super::*;
    use swp_frame_transform::{FrameEvidence, FrameEvidenceBasis};
    use swp_plant_observation::TimestampEvidence;
    use swp_sensor_calibration::{
        AccelerationVectorMps2, AngularRateVectorRadPerSec, CalibrationBasis, CalibrationEvidence,
        TemperatureCelsius,
        encoder::{EncoderPositiveDirection, EncoderTransferBasis, EncoderTransferEvidence},
    };

    fn transfer() -> EncoderTransfer {
        EncoderTransfer::new(
            1_000,
            EncoderPositiveDirection::CounterIncreasing,
            100,
            EncoderTransferEvidence {
                revision: 1,
                basis: EncoderTransferBasis::BenchMeasured,
            },
        )
        .unwrap()
    }

    fn imu(quality: MeasurementQuality) -> BodyImuObservation {
        BodyImuObservation {
            source_sample_at_us: TimestampEvidence::Unknown,
            read_started_at_us: TimestampEvidence::Known(1_000),
            read_completed_at_us: TimestampEvidence::Known(1_200),
            acceleration: AccelerationVectorMps2([1.0, 2.0, 3.0]),
            temperature: TemperatureCelsius(25.0),
            angular_rate: AngularRateVectorRadPerSec([4.0, 5.0, 6.0]),
            quality,
            calibration_evidence: CalibrationEvidence {
                revision: 2,
                basis: CalibrationBasis::BenchMeasured,
            },
            frame_evidence: FrameEvidence {
                revision: 3,
                basis: FrameEvidenceBasis::PhysicalTiltTest,
            },
        }
    }

    fn raw_encoder(count: u16, at_us: u64) -> RawEncoderObservation {
        RawEncoderObservation {
            captured_at_us: TimestampEvidence::Known(at_us),
            count,
            quality: MeasurementQuality::AVAILABLE
                | MeasurementQuality::IO_OK
                | MeasurementQuality::TIMING_VALID,
        }
    }

    fn healthy_imu_quality() -> MeasurementQuality {
        MeasurementQuality::AVAILABLE
            | MeasurementQuality::IO_OK
            | MeasurementQuality::TIMING_VALID
            | MeasurementQuality::FRESHNESS_VERIFIED
    }

    #[test]
    fn first_frame_primes_encoder_rate_without_inventing_it() {
        let mut builder = EstimatorInputBuilder::new(transfer(), transfer());
        let frame = builder.build(
            imu(healthy_imu_quality()),
            raw_encoder(100, 1_000),
            raw_encoder(200, 1_000),
        );

        assert_eq!(frame.drive_encoder_status, EncoderChannelStatus::Primed);
        assert_eq!(frame.reaction_encoder_status, EncoderChannelStatus::Primed);
        assert!(
            frame
                .measurement
                .available
                .contains(DRIVE_ENCODER_RELATIVE_ANGLE)
        );
        assert!(
            !frame
                .measurement
                .available
                .contains(REACTION_WHEEL_RELATIVE_RATE)
        );
        assert!(frame.measurement.timing_valid);
    }

    #[test]
    fn second_frame_populates_exact_measurement_order() {
        let mut builder = EstimatorInputBuilder::new(transfer(), transfer());
        builder.build(
            imu(healthy_imu_quality()),
            raw_encoder(100, 1_000),
            raw_encoder(200, 1_000),
        );
        let frame = builder.build(
            imu(healthy_imu_quality()),
            raw_encoder(110, 3_000),
            raw_encoder(220, 3_000),
        );

        assert_eq!(frame.drive_encoder_status, EncoderChannelStatus::Ready);
        assert_eq!(frame.reaction_encoder_status, EncoderChannelStatus::Ready);
        assert_eq!(frame.measurement.values[ACCEL_X], 1.0);
        assert_eq!(frame.measurement.values[ACCEL_Y], 2.0);
        assert_eq!(frame.measurement.values[ACCEL_Z], 3.0);
        assert_eq!(frame.measurement.values[GYRO_X], 4.0);
        assert_eq!(frame.measurement.values[GYRO_Y], 5.0);
        assert_eq!(frame.measurement.values[GYRO_Z], 6.0);
        assert!(frame.measurement.values[DRIVE_ENCODER_RELATIVE_ANGLE] > 0.0);
        assert!(frame.measurement.values[REACTION_WHEEL_RELATIVE_RATE] > 0.0);
        for index in 0..UPRIGHT_MEASUREMENT_COUNT {
            assert!(frame.measurement.available.contains(index));
        }
    }

    #[test]
    fn primary_timing_health_is_not_invented_from_available_values() {
        let mut builder = EstimatorInputBuilder::new(transfer(), transfer());
        let quality = MeasurementQuality::AVAILABLE | MeasurementQuality::IO_OK;
        let frame = builder.build(
            imu(quality),
            raw_encoder(100, 1_000),
            raw_encoder(200, 1_000),
        );
        assert!(!frame.measurement.timing_valid);
    }

    #[test]
    fn encoder_failure_becomes_channel_unavailability_not_fake_zero_measurement() {
        let mut builder = EstimatorInputBuilder::new(transfer(), transfer());
        builder.build(
            imu(healthy_imu_quality()),
            raw_encoder(100, 1_000),
            raw_encoder(200, 1_000),
        );
        let mut failed = raw_encoder(210, 3_000);
        failed.quality = MeasurementQuality::AVAILABLE | MeasurementQuality::IO_ERROR;
        let frame = builder.build(imu(healthy_imu_quality()), raw_encoder(110, 3_000), failed);

        assert_eq!(
            frame.reaction_encoder_status,
            EncoderChannelStatus::Rejected(EncoderTrackingError::InputIoError)
        );
        assert!(
            !frame
                .measurement
                .available
                .contains(REACTION_WHEEL_RELATIVE_RATE)
        );
    }

    #[test]
    fn reset_starts_a_new_relative_encoder_capture_window() {
        let mut builder = EstimatorInputBuilder::new(transfer(), transfer());
        builder.build(
            imu(healthy_imu_quality()),
            raw_encoder(100, 1_000),
            raw_encoder(200, 1_000),
        );
        builder.build(
            imu(healthy_imu_quality()),
            raw_encoder(110, 3_000),
            raw_encoder(220, 3_000),
        );
        builder.reset();

        let frame = builder.build(
            imu(healthy_imu_quality()),
            raw_encoder(50_000, 10_000),
            raw_encoder(40_000, 10_000),
        );
        assert_eq!(frame.measurement.values[DRIVE_ENCODER_RELATIVE_ANGLE], 0.0);
        assert_eq!(frame.reaction_encoder_status, EncoderChannelStatus::Primed);
    }
}

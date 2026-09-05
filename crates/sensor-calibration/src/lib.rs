#![no_std]

use swp_mpu6050::{
    AccelRange, Config as MpuConfig, GyroRange, accel_raw_to_mps2, gyro_raw_to_rad_per_sec,
    temperature_raw_to_celsius,
};
use swp_plant_observation::{MeasurementQuality, RawImuObservation, TimestampEvidence};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AccelerationVectorMps2(pub [f32; 3]);

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AngularRateVectorRadPerSec(pub [f32; 3]);

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TemperatureCelsius(pub f32);

/// Device-transfer-function output in the MPU6050 sensor frame.
///
/// Values have SI units, but no measured bias/scale correction or mechanical
/// axis mapping has been applied yet.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ScaledImuObservation {
    pub source_sample_at_us: TimestampEvidence,
    pub read_started_at_us: TimestampEvidence,
    pub read_completed_at_us: TimestampEvidence,
    pub acceleration: AccelerationVectorMps2,
    pub temperature: TemperatureCelsius,
    pub angular_rate: AngularRateVectorRadPerSec,
    pub quality: MeasurementQuality,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CalibrationBasis {
    BenchMeasured,
    ImportedMeasured,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CalibrationEvidence {
    pub revision: u32,
    pub basis: CalibrationBasis,
}

/// Three-axis affine correction: output = matrix * (input - bias).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AffineCalibration3 {
    pub bias: [f32; 3],
    pub matrix: [[f32; 3]; 3],
}

impl AffineCalibration3 {
    pub fn new(bias: [f32; 3], matrix: [[f32; 3]; 3]) -> Option<Self> {
        if bias.iter().all(|value| value.is_finite())
            && matrix
                .iter()
                .flatten()
                .all(|value| value.is_finite())
        {
            Some(Self { bias, matrix })
        } else {
            None
        }
    }

    pub fn apply(self, input: [f32; 3]) -> [f32; 3] {
        let centered = [
            input[0] - self.bias[0],
            input[1] - self.bias[1],
            input[2] - self.bias[2],
        ];

        [
            dot(self.matrix[0], centered),
            dot(self.matrix[1], centered),
            dot(self.matrix[2], centered),
        ]
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuCalibration {
    pub accelerometer: AffineCalibration3,
    pub gyroscope: AffineCalibration3,
    pub evidence: CalibrationEvidence,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CalibratedImuObservation {
    pub source_sample_at_us: TimestampEvidence,
    pub read_started_at_us: TimestampEvidence,
    pub read_completed_at_us: TimestampEvidence,
    pub acceleration: AccelerationVectorMps2,
    pub temperature: TemperatureCelsius,
    pub angular_rate: AngularRateVectorRadPerSec,
    pub quality: MeasurementQuality,
    pub calibration_evidence: Option<CalibrationEvidence>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CalibrationError {
    InputUnavailable,
    InputIoError,
}

/// Apply only MPU6050 datasheet transfer functions.
///
/// This step intentionally does not claim physical calibration. It converts
/// register counts into SI units in the sensor's native frame.
pub fn scale_mpu6050(
    raw: RawImuObservation,
    config: MpuConfig,
) -> Result<ScaledImuObservation, CalibrationError> {
    require_usable_input(raw.quality)?;

    let acceleration = raw
        .accel_raw
        .map(|value| accel_raw_to_mps2(value, config.accel_range));
    let angular_rate = raw
        .gyro_raw
        .map(|value| gyro_raw_to_rad_per_sec(value, config.gyro_range));

    Ok(ScaledImuObservation {
        source_sample_at_us: raw.source_sample_at_us,
        read_started_at_us: raw.read_started_at_us,
        read_completed_at_us: raw.read_completed_at_us,
        acceleration: AccelerationVectorMps2(acceleration),
        temperature: TemperatureCelsius(temperature_raw_to_celsius(raw.temperature_raw)),
        angular_rate: AngularRateVectorRadPerSec(angular_rate),
        quality: raw.quality,
    })
}

/// Apply measured sensor-frame calibration without changing coordinate frames.
pub fn calibrate_imu(
    scaled: ScaledImuObservation,
    calibration: ImuCalibration,
) -> CalibratedImuObservation {
    CalibratedImuObservation {
        source_sample_at_us: scaled.source_sample_at_us,
        read_started_at_us: scaled.read_started_at_us,
        read_completed_at_us: scaled.read_completed_at_us,
        acceleration: AccelerationVectorMps2(
            calibration.accelerometer.apply(scaled.acceleration.0),
        ),
        temperature: scaled.temperature,
        angular_rate: AngularRateVectorRadPerSec(
            calibration.gyroscope.apply(scaled.angular_rate.0),
        ),
        quality: scaled.quality,
        calibration_evidence: Some(calibration.evidence),
    }
}

fn require_usable_input(quality: MeasurementQuality) -> Result<(), CalibrationError> {
    if quality.contains(MeasurementQuality::IO_ERROR) || !quality.contains(MeasurementQuality::IO_OK)
    {
        return Err(CalibrationError::InputIoError);
    }
    if !quality.contains(MeasurementQuality::AVAILABLE) {
        return Err(CalibrationError::InputUnavailable);
    }
    Ok(())
}

fn dot(lhs: [f32; 3], rhs: [f32; 3]) -> f32 {
    lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2]
}

#[cfg(test)]
mod tests {
    use super::*;
    use swp_mpu6050::Dlpf;

    fn config() -> MpuConfig {
        MpuConfig {
            gyro_range: GyroRange::Dps1000,
            accel_range: AccelRange::G4,
            dlpf: Dlpf::Config3,
            sample_rate_hz: 100,
            data_ready_interrupt: false,
        }
    }

    fn raw_sample() -> RawImuObservation {
        RawImuObservation {
            source_sample_at_us: TimestampEvidence::Unknown,
            read_started_at_us: TimestampEvidence::Known(1_000),
            read_completed_at_us: TimestampEvidence::Known(1_200),
            accel_raw: [8_192, 0, -8_192],
            temperature_raw: 0,
            gyro_raw: [328, 0, -328],
            quality: MeasurementQuality::AVAILABLE | MeasurementQuality::IO_OK,
        }
    }

    #[test]
    fn nominal_scaling_is_not_physical_calibration() {
        let scaled = scale_mpu6050(raw_sample(), config()).unwrap();
        assert!((scaled.acceleration.0[0] - 9.806_65).abs() < 0.000_1);
        assert!((scaled.acceleration.0[2] + 9.806_65).abs() < 0.000_1);
        assert!((scaled.angular_rate.0[0] - 10.0_f32.to_radians()).abs() < 0.000_1);
        assert_eq!(scaled.source_sample_at_us, TimestampEvidence::Unknown);
    }

    #[test]
    fn measured_affine_calibration_preserves_timing_and_quality() {
        let scaled = scale_mpu6050(raw_sample(), config()).unwrap();
        let identity = AffineCalibration3::new(
            [0.0; 3],
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        )
        .unwrap();
        let evidence = CalibrationEvidence {
            revision: 7,
            basis: CalibrationBasis::BenchMeasured,
        };
        let calibrated = calibrate_imu(
            scaled,
            ImuCalibration {
                accelerometer: identity,
                gyroscope: identity,
                evidence,
            },
        );

        assert_eq!(calibrated.source_sample_at_us, TimestampEvidence::Unknown);
        assert_eq!(calibrated.quality, scaled.quality);
        assert_eq!(calibrated.calibration_evidence, Some(evidence));
        assert_eq!(calibrated.acceleration, scaled.acceleration);
        assert_eq!(calibrated.angular_rate, scaled.angular_rate);
    }

    #[test]
    fn io_error_blocks_semantic_upgrade() {
        let mut raw = raw_sample();
        raw.quality = MeasurementQuality::AVAILABLE | MeasurementQuality::IO_ERROR;
        assert_eq!(scale_mpu6050(raw, config()), Err(CalibrationError::InputIoError));
    }

    #[test]
    fn non_finite_calibration_is_rejected() {
        assert!(AffineCalibration3::new([f32::NAN, 0.0, 0.0], [[1.0; 3]; 3]).is_none());
    }
}

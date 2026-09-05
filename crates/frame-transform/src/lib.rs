#![no_std]

use swp_plant_observation::{MeasurementQuality, TimestampEvidence};
use swp_sensor_calibration::{
    AccelerationVectorMps2, AngularRateVectorRadPerSec, CalibratedImuObservation,
    CalibrationEvidence, TemperatureCelsius,
};

/// Canonical robot body frame used by this project.
///
/// The frame is right-handed:
/// - +X: forward, along the ground-drive direction;
/// - +Y: left;
/// - +Z: up.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BodyAxis {
    ForwardX,
    LeftY,
    UpZ,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameEvidenceBasis {
    PhysicalTiltTest,
    AssemblySurvey,
    ImportedMeasured,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FrameEvidence {
    pub revision: u32,
    pub basis: FrameEvidenceBasis,
}

/// Proper 3-D rotation from the MPU6050 sensor frame into the canonical body frame.
///
/// Construction rejects non-finite, non-orthonormal, and reflection matrices. The
/// transform therefore represents a physical orientation change rather than an
/// arbitrary scale, shear, or handedness flip.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SensorToBodyRotation {
    matrix: [[f32; 3]; 3],
}

impl SensorToBodyRotation {
    const ORTHONORMAL_TOLERANCE: f32 = 1.0e-3;
    const DETERMINANT_TOLERANCE: f32 = 5.0e-3;

    pub fn new(matrix: [[f32; 3]; 3]) -> Option<Self> {
        if !matrix.iter().flatten().all(|value| value.is_finite()) {
            return None;
        }

        let rows = matrix;
        for row in rows {
            if (dot(row, row) - 1.0).abs() > Self::ORTHONORMAL_TOLERANCE {
                return None;
            }
        }

        if dot(rows[0], rows[1]).abs() > Self::ORTHONORMAL_TOLERANCE
            || dot(rows[0], rows[2]).abs() > Self::ORTHONORMAL_TOLERANCE
            || dot(rows[1], rows[2]).abs() > Self::ORTHONORMAL_TOLERANCE
        {
            return None;
        }

        if (determinant(matrix) - 1.0).abs() > Self::DETERMINANT_TOLERANCE {
            return None;
        }

        Some(Self { matrix })
    }

    pub const fn matrix(self) -> [[f32; 3]; 3] {
        self.matrix
    }

    pub fn apply(self, sensor_vector: [f32; 3]) -> [f32; 3] {
        [
            dot(self.matrix[0], sensor_vector),
            dot(self.matrix[1], sensor_vector),
            dot(self.matrix[2], sensor_vector),
        ]
    }
}

/// Calibrated IMU data expressed in the canonical body frame.
///
/// This is a semantic upgrade from `CalibratedImuObservation`: the values have
/// measured sensor calibration and an explicit, evidenced mechanical frame transform.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyImuObservation {
    pub source_sample_at_us: TimestampEvidence,
    pub read_started_at_us: TimestampEvidence,
    pub read_completed_at_us: TimestampEvidence,
    pub acceleration: AccelerationVectorMps2,
    pub temperature: TemperatureCelsius,
    pub angular_rate: AngularRateVectorRadPerSec,
    pub quality: MeasurementQuality,
    pub calibration_evidence: CalibrationEvidence,
    pub frame_evidence: FrameEvidence,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameTransformError {
    CalibrationEvidenceMissing,
}

pub fn map_calibrated_imu_to_body(
    calibrated: CalibratedImuObservation,
    rotation: SensorToBodyRotation,
    frame_evidence: FrameEvidence,
) -> Result<BodyImuObservation, FrameTransformError> {
    let calibration_evidence = calibrated
        .calibration_evidence
        .ok_or(FrameTransformError::CalibrationEvidenceMissing)?;

    Ok(BodyImuObservation {
        source_sample_at_us: calibrated.source_sample_at_us,
        read_started_at_us: calibrated.read_started_at_us,
        read_completed_at_us: calibrated.read_completed_at_us,
        acceleration: AccelerationVectorMps2(rotation.apply(calibrated.acceleration.0)),
        temperature: calibrated.temperature,
        angular_rate: AngularRateVectorRadPerSec(rotation.apply(calibrated.angular_rate.0)),
        quality: calibrated.quality,
        calibration_evidence,
        frame_evidence,
    })
}

fn dot(lhs: [f32; 3], rhs: [f32; 3]) -> f32 {
    lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2]
}

fn determinant(matrix: [[f32; 3]; 3]) -> f32 {
    matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
}

#[cfg(test)]
mod tests {
    use super::*;
    use swp_sensor_calibration::{CalibrationBasis, CalibrationEvidence};

    fn calibrated() -> CalibratedImuObservation {
        CalibratedImuObservation {
            source_sample_at_us: TimestampEvidence::Unknown,
            read_started_at_us: TimestampEvidence::Known(100),
            read_completed_at_us: TimestampEvidence::Known(120),
            acceleration: AccelerationVectorMps2([1.0, 2.0, 3.0]),
            temperature: TemperatureCelsius(25.0),
            angular_rate: AngularRateVectorRadPerSec([4.0, 5.0, 6.0]),
            quality: MeasurementQuality::AVAILABLE | MeasurementQuality::IO_OK,
            calibration_evidence: Some(CalibrationEvidence {
                revision: 2,
                basis: CalibrationBasis::BenchMeasured,
            }),
        }
    }

    #[test]
    fn proper_signed_axis_rotation_is_accepted_and_applied() {
        // body +X <- sensor +Y, body +Y <- sensor -X, body +Z <- sensor +Z
        let rotation =
            SensorToBodyRotation::new([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
                .unwrap();

        assert_eq!(rotation.apply([1.0, 2.0, 3.0]), [2.0, -1.0, 3.0]);
    }

    #[test]
    fn reflection_is_rejected() {
        assert!(
            SensorToBodyRotation::new([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0],])
                .is_none()
        );
    }

    #[test]
    fn non_orthonormal_transform_is_rejected() {
        assert!(
            SensorToBodyRotation::new([[1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 0.0, 1.0],])
                .is_none()
        );
    }

    #[test]
    fn mapping_preserves_timing_quality_and_evidence() {
        let rotation =
            SensorToBodyRotation::new([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]).unwrap();
        let frame_evidence = FrameEvidence {
            revision: 3,
            basis: FrameEvidenceBasis::PhysicalTiltTest,
        };
        let body = map_calibrated_imu_to_body(calibrated(), rotation, frame_evidence).unwrap();

        assert_eq!(body.acceleration.0, [1.0, 2.0, 3.0]);
        assert_eq!(body.angular_rate.0, [4.0, 5.0, 6.0]);
        assert_eq!(body.frame_evidence, frame_evidence);
        assert_eq!(body.read_completed_at_us, TimestampEvidence::Known(120));
    }

    #[test]
    fn missing_calibration_evidence_blocks_body_semantics() {
        let mut input = calibrated();
        input.calibration_evidence = None;
        let rotation =
            SensorToBodyRotation::new([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]).unwrap();

        assert_eq!(
            map_calibrated_imu_to_body(
                input,
                rotation,
                FrameEvidence {
                    revision: 1,
                    basis: FrameEvidenceBasis::AssemblySurvey,
                },
            ),
            Err(FrameTransformError::CalibrationEvidenceMissing)
        );
    }
}

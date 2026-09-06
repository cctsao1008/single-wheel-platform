use core::f32::consts::TAU;

use swp_plant_observation::{MeasurementQuality, RawEncoderObservation, TimestampEvidence};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EncoderTransferBasis {
    BenchMeasured,
    ImportedMeasured,
    DatasheetDerived,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EncoderTransferEvidence {
    pub revision: u32,
    pub basis: EncoderTransferBasis,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EncoderPositiveDirection {
    CounterIncreasing,
    CounterDecreasing,
}

impl EncoderPositiveDirection {
    const fn multiplier(self) -> i32 {
        match self {
            Self::CounterIncreasing => 1,
            Self::CounterDecreasing => -1,
        }
    }
}

/// Evidenced transfer from one STM32 quadrature-counter count to mechanical angle.
///
/// `counter_counts_per_revolution` is deliberately the count seen by the STM32 timer
/// per mechanical revolution of the controlled shaft. It is not encoder lines/PPR
/// unless the timer decode mode and any gearing have already been accounted for.
///
/// `max_abs_delta_counts_per_sample` is the physical anti-aliasing contract for the
/// 16-bit counter. Signed modular unwrapping is unique only while the real inter-sample
/// motion is known to stay below half the counter range. The configured bound must
/// therefore be smaller than 32768 counts and should come from an evidenced speed/
/// sampling envelope before the transfer is instantiated on the reference robot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EncoderTransfer {
    counter_counts_per_revolution: u32,
    positive_direction: EncoderPositiveDirection,
    max_abs_delta_counts_per_sample: u16,
    evidence: EncoderTransferEvidence,
}

impl EncoderTransfer {
    pub const fn new(
        counter_counts_per_revolution: u32,
        positive_direction: EncoderPositiveDirection,
        max_abs_delta_counts_per_sample: u16,
        evidence: EncoderTransferEvidence,
    ) -> Option<Self> {
        if counter_counts_per_revolution == 0
            || max_abs_delta_counts_per_sample == 0
            || max_abs_delta_counts_per_sample >= 32_768
        {
            None
        } else {
            Some(Self {
                counter_counts_per_revolution,
                positive_direction,
                max_abs_delta_counts_per_sample,
                evidence,
            })
        }
    }

    pub const fn counter_counts_per_revolution(self) -> u32 {
        self.counter_counts_per_revolution
    }

    pub const fn positive_direction(self) -> EncoderPositiveDirection {
        self.positive_direction
    }

    pub const fn max_abs_delta_counts_per_sample(self) -> u16 {
        self.max_abs_delta_counts_per_sample
    }

    pub const fn evidence(self) -> EncoderTransferEvidence {
        self.evidence
    }

    pub fn radians_per_count(self) -> f32 {
        TAU / self.counter_counts_per_revolution as f32
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EncoderKinematicObservation {
    /// Relative mechanical angle since the tracker was primed/reset.
    pub relative_angle_rad: f32,
    /// Mechanical rate is unavailable for the first accepted counter sample.
    pub relative_rate_rad_per_s: Option<f32>,
    pub captured_at_us: u64,
    pub quality: MeasurementQuality,
    pub transfer_evidence: EncoderTransferEvidence,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EncoderTrackingError {
    InputUnavailable,
    InputIoError,
    TimingInvalid,
    TimestampUnknown,
    NonMonotonicTimestamp,
    DeltaBeyondEvidence { observed_abs: u16, allowed: u16 },
    NumericalFault,
}

/// Stateful 16-bit QEI unwrapping in robot-semantic mechanical coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EncoderTracker {
    transfer: EncoderTransfer,
    previous_count: Option<u16>,
    previous_at_us: Option<u64>,
    unwrapped_signed_counts: i64,
}

impl EncoderTracker {
    pub const fn new(transfer: EncoderTransfer) -> Self {
        Self {
            transfer,
            previous_count: None,
            previous_at_us: None,
            unwrapped_signed_counts: 0,
        }
    }

    pub const fn transfer(self) -> EncoderTransfer {
        self.transfer
    }

    pub fn reset(&mut self) {
        self.previous_count = None;
        self.previous_at_us = None;
        self.unwrapped_signed_counts = 0;
    }

    pub fn observe(
        &mut self,
        raw: RawEncoderObservation,
    ) -> Result<EncoderKinematicObservation, EncoderTrackingError> {
        require_usable_encoder(raw.quality)?;
        let TimestampEvidence::Known(captured_at_us) = raw.captured_at_us else {
            return Err(EncoderTrackingError::TimestampUnknown);
        };

        let radians_per_count = self.transfer.radians_per_count();
        if !radians_per_count.is_finite() || radians_per_count <= 0.0 {
            return Err(EncoderTrackingError::NumericalFault);
        }

        let (Some(previous_count), Some(previous_at_us)) =
            (self.previous_count, self.previous_at_us)
        else {
            self.previous_count = Some(raw.count);
            self.previous_at_us = Some(captured_at_us);
            self.unwrapped_signed_counts = 0;
            return Ok(EncoderKinematicObservation {
                relative_angle_rad: 0.0,
                relative_rate_rad_per_s: None,
                captured_at_us,
                quality: raw.quality,
                transfer_evidence: self.transfer.evidence(),
            });
        };

        let Some(delta_us) = captured_at_us.checked_sub(previous_at_us) else {
            return Err(EncoderTrackingError::NonMonotonicTimestamp);
        };
        if delta_us == 0 {
            return Err(EncoderTrackingError::NonMonotonicTimestamp);
        }

        // Modular subtraction followed by i16 interpretation gives the unique signed
        // shortest delta of a 16-bit timer counter. The configured physical bound
        // below protects the validity of that unwrapping assumption.
        let raw_delta = raw.count.wrapping_sub(previous_count) as i16 as i32;
        let observed_abs = raw_delta.unsigned_abs() as u16;
        let allowed = self.transfer.max_abs_delta_counts_per_sample();
        if observed_abs > allowed {
            return Err(EncoderTrackingError::DeltaBeyondEvidence {
                observed_abs,
                allowed,
            });
        }

        let signed_delta = raw_delta * self.transfer.positive_direction().multiplier();
        let next_unwrapped = self.unwrapped_signed_counts + i64::from(signed_delta);
        let relative_angle_rad = next_unwrapped as f32 * radians_per_count;
        let delta_s = delta_us as f32 * 1.0e-6;
        let relative_rate_rad_per_s = signed_delta as f32 * radians_per_count / delta_s;
        if !relative_angle_rad.is_finite() || !relative_rate_rad_per_s.is_finite() {
            return Err(EncoderTrackingError::NumericalFault);
        }

        self.previous_count = Some(raw.count);
        self.previous_at_us = Some(captured_at_us);
        self.unwrapped_signed_counts = next_unwrapped;

        Ok(EncoderKinematicObservation {
            relative_angle_rad,
            relative_rate_rad_per_s: Some(relative_rate_rad_per_s),
            captured_at_us,
            quality: raw.quality,
            transfer_evidence: self.transfer.evidence(),
        })
    }
}

fn require_usable_encoder(quality: MeasurementQuality) -> Result<(), EncoderTrackingError> {
    if quality.contains(MeasurementQuality::IO_ERROR)
        || !quality.contains(MeasurementQuality::IO_OK)
    {
        return Err(EncoderTrackingError::InputIoError);
    }
    if !quality.contains(MeasurementQuality::AVAILABLE) {
        return Err(EncoderTrackingError::InputUnavailable);
    }
    if !quality.contains(MeasurementQuality::TIMING_VALID) {
        return Err(EncoderTrackingError::TimingInvalid);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn transfer(direction: EncoderPositiveDirection) -> EncoderTransfer {
        EncoderTransfer::new(
            1_000,
            direction,
            100,
            EncoderTransferEvidence {
                revision: 1,
                basis: EncoderTransferBasis::BenchMeasured,
            },
        )
        .unwrap()
    }

    fn raw(count: u16, at_us: u64) -> RawEncoderObservation {
        RawEncoderObservation {
            captured_at_us: TimestampEvidence::Known(at_us),
            count,
            quality: MeasurementQuality::AVAILABLE
                | MeasurementQuality::IO_OK
                | MeasurementQuality::TIMING_VALID,
        }
    }

    #[test]
    fn first_sample_defines_relative_zero_without_inventing_rate() {
        let mut tracker = EncoderTracker::new(transfer(EncoderPositiveDirection::CounterIncreasing));
        let first = tracker.observe(raw(12_345, 1_000)).unwrap();
        assert_eq!(first.relative_angle_rad, 0.0);
        assert_eq!(first.relative_rate_rad_per_s, None);
    }

    #[test]
    fn counter_wrap_is_unwrapped_in_both_directions() {
        let mut forward = EncoderTracker::new(transfer(EncoderPositiveDirection::CounterIncreasing));
        forward.observe(raw(65_534, 1_000)).unwrap();
        let next = forward.observe(raw(2, 3_000)).unwrap();
        assert!((next.relative_angle_rad - 4.0 * TAU / 1_000.0).abs() < 1.0e-6);

        let mut reverse = EncoderTracker::new(transfer(EncoderPositiveDirection::CounterIncreasing));
        reverse.observe(raw(2, 1_000)).unwrap();
        let next = reverse.observe(raw(65_534, 3_000)).unwrap();
        assert!((next.relative_angle_rad + 4.0 * TAU / 1_000.0).abs() < 1.0e-6);
    }

    #[test]
    fn mechanical_sign_is_explicit() {
        let mut tracker = EncoderTracker::new(transfer(EncoderPositiveDirection::CounterDecreasing));
        tracker.observe(raw(100, 1_000)).unwrap();
        let next = tracker.observe(raw(110, 3_000)).unwrap();
        assert!(next.relative_angle_rad < 0.0);
        assert!(next.relative_rate_rad_per_s.unwrap() < 0.0);
    }

    #[test]
    fn delta_beyond_physical_unwrap_contract_is_rejected_without_advancing_state() {
        let mut tracker = EncoderTracker::new(transfer(EncoderPositiveDirection::CounterIncreasing));
        tracker.observe(raw(100, 1_000)).unwrap();
        assert_eq!(
            tracker.observe(raw(250, 3_000)),
            Err(EncoderTrackingError::DeltaBeyondEvidence {
                observed_abs: 150,
                allowed: 100,
            })
        );
        let recovered = tracker.observe(raw(110, 5_000)).unwrap();
        assert!((recovered.relative_angle_rad - 10.0 * TAU / 1_000.0).abs() < 1.0e-6);
    }

    #[test]
    fn unknown_or_nonmonotonic_time_is_rejected() {
        let mut tracker = EncoderTracker::new(transfer(EncoderPositiveDirection::CounterIncreasing));
        let mut sample = raw(10, 1_000);
        sample.captured_at_us = TimestampEvidence::Unknown;
        assert_eq!(
            tracker.observe(sample),
            Err(EncoderTrackingError::TimestampUnknown)
        );

        tracker.observe(raw(10, 2_000)).unwrap();
        assert_eq!(
            tracker.observe(raw(11, 2_000)),
            Err(EncoderTrackingError::NonMonotonicTimestamp)
        );
    }
}

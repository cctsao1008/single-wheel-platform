#![no_std]

use core::ops::{BitOr, BitOrAssign};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AcquisitionStatus(u16);

impl AcquisitionStatus {
    pub const NONE: Self = Self(0);
    pub const BUS_READY: Self = Self(1 << 0);
    pub const IMU_PRESENT: Self = Self(1 << 1);
    pub const IMU_CONFIGURED: Self = Self(1 << 2);

    pub const fn from_bits(bits: u16) -> Self {
        Self(bits)
    }

    pub const fn bits(self) -> u16 {
        self.0
    }

    pub const fn contains(self, flag: Self) -> bool {
        self.0 & flag.0 == flag.0
    }
}

impl BitOr for AcquisitionStatus {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self::Output {
        Self(self.0 | rhs.0)
    }
}

impl BitOrAssign for AcquisitionStatus {
    fn bitor_assign(&mut self, rhs: Self) {
        self.0 |= rhs.0;
    }
}

/// Evidence about the quality of one measurement.
///
/// Flags are deliberately not interpreted as a single valid/invalid boolean.
/// For example, an MPU read may be available and I/O-clean while freshness and
/// physical sample time remain unknown because the board does not route DRDY.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MeasurementQuality(u16);

impl MeasurementQuality {
    pub const NONE: Self = Self(0);
    pub const AVAILABLE: Self = Self(1 << 0);
    pub const IO_OK: Self = Self(1 << 1);
    pub const IO_ERROR: Self = Self(1 << 2);
    pub const TIMING_VALID: Self = Self(1 << 3);
    pub const FRESHNESS_VERIFIED: Self = Self(1 << 4);
    pub const SATURATED: Self = Self(1 << 5);
    pub const STALE: Self = Self(1 << 6);
    pub const RETRIED: Self = Self(1 << 7);

    pub const fn from_bits(bits: u16) -> Self {
        Self(bits)
    }

    pub const fn bits(self) -> u16 {
        self.0
    }

    pub const fn contains(self, flag: Self) -> bool {
        self.0 & flag.0 == flag.0
    }
}

impl BitOr for MeasurementQuality {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self::Output {
        Self(self.0 | rhs.0)
    }
}

impl BitOrAssign for MeasurementQuality {
    fn bitor_assign(&mut self, rhs: Self) {
        self.0 |= rhs.0;
    }
}

/// Timestamp evidence on the firmware monotonic timebase.
///
/// `Unknown` is a first-class state rather than a fabricated timestamp. This
/// matters for the MPU6050 on the reference board: its internal sample time is
/// not observable because the actual data-ready interrupt is not routed.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum TimestampEvidence {
    #[default]
    Unknown,
    Known(u64),
}

impl TimestampEvidence {
    pub const fn value(self) -> Option<u64> {
        match self {
            Self::Unknown => None,
            Self::Known(value) => Some(value),
        }
    }
}

/// Lossless MPU6050 register-domain observation plus timing/quality evidence.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RawImuObservation {
    pub source_sample_at_us: TimestampEvidence,
    pub read_started_at_us: TimestampEvidence,
    pub read_completed_at_us: TimestampEvidence,
    pub accel_raw: [i16; 3],
    pub temperature_raw: i16,
    pub gyro_raw: [i16; 3],
    pub quality: MeasurementQuality,
}

/// One raw quadrature-counter snapshot.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RawEncoderObservation {
    pub captured_at_us: TimestampEvidence,
    pub count: u16,
    pub quality: MeasurementQuality,
}

/// One raw battery-divider ADC observation.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RawBatteryObservation {
    pub read_completed_at_us: TimestampEvidence,
    pub adc_raw: u16,
    pub quality: MeasurementQuality,
}

/// One acquisition batch from the physical plant.
///
/// This type intentionally preserves what the firmware actually knows. It does
/// not pretend that sequential I2C, timer-register, and ADC reads happened at
/// one physical instant, and it does not upgrade raw values into SI units or
/// robot-axis semantics.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RawObservation {
    pub sample_index: u32,
    pub acquisition_started_us: u64,
    pub acquisition_completed_us: u64,
    pub imu: RawImuObservation,
    pub encoders: [RawEncoderObservation; 2],
    pub battery: RawBatteryObservation,
    pub acquisition_status: AcquisitionStatus,
}

impl RawObservation {
    pub const fn acquisition_duration_us(self) -> u64 {
        self.acquisition_completed_us
            .saturating_sub(self.acquisition_started_us)
    }
}

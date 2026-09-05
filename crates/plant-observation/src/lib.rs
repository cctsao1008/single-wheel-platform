#![no_std]

use core::ops::{BitOr, BitOrAssign};

/// Validity and acquisition-health flags attached to one raw observation.
///
/// A set bit means the corresponding evidence was available for this sample;
/// it does not upgrade raw counts into calibrated physical quantities.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ObservationFlags(u16);

impl ObservationFlags {
    pub const NONE: Self = Self(0);
    pub const BUS_READY: Self = Self(1 << 0);
    pub const IMU_PRESENT: Self = Self(1 << 1);
    pub const IMU_CONFIGURED: Self = Self(1 << 2);
    pub const IMU_SAMPLE_VALID: Self = Self(1 << 3);
    pub const ENCODER_1_VALID: Self = Self(1 << 4);
    pub const ENCODER_2_VALID: Self = Self(1 << 5);
    pub const BATTERY_ADC_VALID: Self = Self(1 << 6);

    pub const fn bits(self) -> u16 {
        self.0
    }

    pub const fn contains(self, flag: Self) -> bool {
        self.0 & flag.0 == flag.0
    }
}

impl BitOr for ObservationFlags {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self::Output {
        Self(self.0 | rhs.0)
    }
}

impl BitOrAssign for ObservationFlags {
    fn bitor_assign(&mut self, rhs: Self) {
        self.0 |= rhs.0;
    }
}

/// Lossless MPU6050 register-domain sample.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RawImuObservation {
    pub accel_raw: [i16; 3],
    pub temperature_raw: i16,
    pub gyro_raw: [i16; 3],
}

/// One coherent acquisition-time view of the physical plant.
///
/// This is intentionally below calibration, coordinate mapping, estimation,
/// and control. Values remain in the electrical/register domain until a later
/// stage has enough evidence to assign SI units and robot-axis semantics.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RawObservation {
    pub sample_index: u32,
    pub timestamp_us: u64,
    pub imu: RawImuObservation,
    pub encoder_counts: [u16; 2],
    pub battery_adc_raw: u16,
    pub validity: ObservationFlags,
}

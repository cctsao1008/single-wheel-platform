#![no_std]

pub const MAGIC: [u8; 2] = *b"SW";
pub const VERSION: u8 = 1;
pub const KIND_CONTROL_PROFILE: u8 = 2;
pub const CONTROL_PROFILE_PAYLOAD_LEN: u16 = 72;
pub const CONTROL_PROFILE_RECORD_LEN: usize = 80;

const CRC_OFFSET: usize = CONTROL_PROFILE_RECORD_LEN - 2;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ControlProfileStatus(u16);

impl ControlProfileStatus {
    pub const NONE: Self = Self(0);
    pub const SYNTHETIC_NUMERICS: Self = Self(1 << 0);
    pub const MOTOR_PERIPHERALS_ABSENT: Self = Self(1 << 1);
    pub const IMU_IO_OK: Self = Self(1 << 2);
    pub const TIMING_HEALTHY: Self = Self(1 << 3);
    pub const SEMANTIC_PROJECTION_READY: Self = Self(1 << 4);
    pub const ESTIMATOR_OK: Self = Self(1 << 5);
    pub const FEEDBACK_OK: Self = Self(1 << 6);
    pub const AUTHORITY_EVALUATED: Self = Self(1 << 7);
    pub const AUTHORIZED_TOKEN_DROPPED: Self = Self(1 << 8);
    pub const CRITICAL_PATH_OVERRUN: Self = Self(1 << 9);

    pub const fn bits(self) -> u16 {
        self.0
    }

    pub const fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }

    pub const fn with(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ControlProfileSample {
    pub sequence: u32,
    pub event_started_us: u64,
    pub imu_read_cycles: u32,
    pub encoder_snapshot_cycles: u32,
    pub semantic_projection_cycles: u32,
    pub estimator_cycles: u32,
    pub feedback_cycles: u32,
    pub actuator_authority_cycles: u32,
    pub critical_path_cycles: u32,
    pub window_max_critical_path_cycles: u32,
    pub boot_max_critical_path_cycles: u32,
    pub deadline_cycles: u32,
    pub overrun_count: u32,
    pub cpu_hz: u32,
    pub authority_reasons: u16,
    pub status: ControlProfileStatus,
    pub dropped_records: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecodeError {
    Magic,
    Version,
    Kind,
    PayloadLength,
    Crc,
}

impl ControlProfileSample {
    pub fn encode(self) -> [u8; CONTROL_PROFILE_RECORD_LEN] {
        let mut out = [0_u8; CONTROL_PROFILE_RECORD_LEN];
        out[0] = MAGIC[0];
        out[1] = MAGIC[1];
        out[2] = VERSION;
        out[3] = KIND_CONTROL_PROFILE;
        put_u16(&mut out, 4, CONTROL_PROFILE_PAYLOAD_LEN);
        put_u32(&mut out, 6, self.sequence);
        put_u64(&mut out, 10, self.event_started_us);
        put_u32(&mut out, 18, self.imu_read_cycles);
        put_u32(&mut out, 22, self.encoder_snapshot_cycles);
        put_u32(&mut out, 26, self.semantic_projection_cycles);
        put_u32(&mut out, 30, self.estimator_cycles);
        put_u32(&mut out, 34, self.feedback_cycles);
        put_u32(&mut out, 38, self.actuator_authority_cycles);
        put_u32(&mut out, 42, self.critical_path_cycles);
        put_u32(&mut out, 46, self.window_max_critical_path_cycles);
        put_u32(&mut out, 50, self.boot_max_critical_path_cycles);
        put_u32(&mut out, 54, self.deadline_cycles);
        put_u32(&mut out, 58, self.overrun_count);
        put_u32(&mut out, 62, self.cpu_hz);
        put_u16(&mut out, 66, self.authority_reasons);
        put_u16(&mut out, 68, self.status.bits());
        put_u16(&mut out, 70, self.dropped_records);
        // Bytes 72..78 are reserved for forward-compatible profiler fields.
        let crc = crc16_ccitt_false(&out[..CRC_OFFSET]);
        put_u16(&mut out, CRC_OFFSET, crc);
        out
    }

    pub fn decode(bytes: &[u8; CONTROL_PROFILE_RECORD_LEN]) -> Result<Self, DecodeError> {
        if bytes[0] != MAGIC[0] || bytes[1] != MAGIC[1] {
            return Err(DecodeError::Magic);
        }
        if bytes[2] != VERSION {
            return Err(DecodeError::Version);
        }
        if bytes[3] != KIND_CONTROL_PROFILE {
            return Err(DecodeError::Kind);
        }
        if get_u16(bytes, 4) != CONTROL_PROFILE_PAYLOAD_LEN {
            return Err(DecodeError::PayloadLength);
        }
        if crc16_ccitt_false(&bytes[..CRC_OFFSET]) != get_u16(bytes, CRC_OFFSET) {
            return Err(DecodeError::Crc);
        }

        Ok(Self {
            sequence: get_u32(bytes, 6),
            event_started_us: get_u64(bytes, 10),
            imu_read_cycles: get_u32(bytes, 18),
            encoder_snapshot_cycles: get_u32(bytes, 22),
            semantic_projection_cycles: get_u32(bytes, 26),
            estimator_cycles: get_u32(bytes, 30),
            feedback_cycles: get_u32(bytes, 34),
            actuator_authority_cycles: get_u32(bytes, 38),
            critical_path_cycles: get_u32(bytes, 42),
            window_max_critical_path_cycles: get_u32(bytes, 46),
            boot_max_critical_path_cycles: get_u32(bytes, 50),
            deadline_cycles: get_u32(bytes, 54),
            overrun_count: get_u32(bytes, 58),
            cpu_hz: get_u32(bytes, 62),
            authority_reasons: get_u16(bytes, 66),
            status: ControlProfileStatus(get_u16(bytes, 68)),
            dropped_records: get_u16(bytes, 70),
        })
    }
}

pub fn crc16_ccitt_false(bytes: &[u8]) -> u16 {
    let mut crc = 0xffff_u16;
    for &byte in bytes {
        crc ^= u16::from(byte) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

fn put_u16(dst: &mut [u8], offset: usize, value: u16) {
    dst[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
}

fn put_u32(dst: &mut [u8], offset: usize, value: u32) {
    dst[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

fn put_u64(dst: &mut [u8], offset: usize, value: u64) {
    dst[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
}

fn get_u16(src: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes([src[offset], src[offset + 1]])
}

fn get_u32(src: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes([
        src[offset],
        src[offset + 1],
        src[offset + 2],
        src[offset + 3],
    ])
}

fn get_u64(src: &[u8], offset: usize) -> u64 {
    u64::from_le_bytes([
        src[offset],
        src[offset + 1],
        src[offset + 2],
        src[offset + 3],
        src[offset + 4],
        src[offset + 5],
        src[offset + 6],
        src[offset + 7],
    ])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn profile_record_round_trips_and_crc_covers_payload() {
        let sample = ControlProfileSample {
            sequence: 42,
            event_started_us: 12_345_678,
            imu_read_cycles: 10,
            encoder_snapshot_cycles: 20,
            semantic_projection_cycles: 30,
            estimator_cycles: 40,
            feedback_cycles: 50,
            actuator_authority_cycles: 60,
            critical_path_cycles: 70,
            window_max_critical_path_cycles: 80,
            boot_max_critical_path_cycles: 90,
            deadline_cycles: 144_000,
            overrun_count: 3,
            cpu_hz: 72_000_000,
            authority_reasons: 0x1234,
            status: ControlProfileStatus::SYNTHETIC_NUMERICS
                .with(ControlProfileStatus::ESTIMATOR_OK),
            dropped_records: 5,
        };
        let encoded = sample.encode();
        assert_eq!(ControlProfileSample::decode(&encoded), Ok(sample));

        let mut corrupt = encoded;
        corrupt[33] ^= 0x80;
        assert_eq!(
            ControlProfileSample::decode(&corrupt),
            Err(DecodeError::Crc)
        );
    }
}

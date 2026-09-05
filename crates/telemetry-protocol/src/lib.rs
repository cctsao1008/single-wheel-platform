#![no_std]

pub const MAGIC: [u8; 2] = *b"SW";
pub const VERSION: u8 = 1;
pub const KIND_RAW_IMU: u8 = 1;
pub const RAW_IMU_PAYLOAD_LEN: u16 = 30;
pub const RAW_IMU_FRAME_LEN: usize = 38;
const CRC_OFFSET: usize = RAW_IMU_FRAME_LEN - 2;

pub mod status {
    pub const BUS_READY: u16 = 1 << 0;
    pub const IMU_PRESENT: u16 = 1 << 1;
    pub const IMU_CONFIGURED: u16 = 1 << 2;
    pub const SAMPLE_VALID: u16 = 1 << 3;
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RawImuFrame {
    pub sequence: u32,
    pub timestamp_us: u64,
    pub accel_raw: [i16; 3],
    pub temperature_raw: i16,
    pub gyro_raw: [i16; 3],
    pub status: u16,
    pub dropped_frames: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecodeError {
    Magic,
    Version,
    Kind,
    PayloadLength,
    Crc,
}

impl RawImuFrame {
    pub fn encode(self) -> [u8; RAW_IMU_FRAME_LEN] {
        let mut out = [0_u8; RAW_IMU_FRAME_LEN];
        out[0] = MAGIC[0];
        out[1] = MAGIC[1];
        out[2] = VERSION;
        out[3] = KIND_RAW_IMU;
        put_u16(&mut out, 4, RAW_IMU_PAYLOAD_LEN);
        put_u32(&mut out, 6, self.sequence);
        put_u64(&mut out, 10, self.timestamp_us);
        put_i16(&mut out, 18, self.accel_raw[0]);
        put_i16(&mut out, 20, self.accel_raw[1]);
        put_i16(&mut out, 22, self.accel_raw[2]);
        put_i16(&mut out, 24, self.temperature_raw);
        put_i16(&mut out, 26, self.gyro_raw[0]);
        put_i16(&mut out, 28, self.gyro_raw[1]);
        put_i16(&mut out, 30, self.gyro_raw[2]);
        put_u16(&mut out, 32, self.status);
        put_u16(&mut out, 34, self.dropped_frames);

        let crc = crc16_ccitt_false(&out[..CRC_OFFSET]);
        put_u16(&mut out, CRC_OFFSET, crc);
        out
    }

    pub fn decode(bytes: &[u8; RAW_IMU_FRAME_LEN]) -> Result<Self, DecodeError> {
        if bytes[0] != MAGIC[0] || bytes[1] != MAGIC[1] {
            return Err(DecodeError::Magic);
        }
        if bytes[2] != VERSION {
            return Err(DecodeError::Version);
        }
        if bytes[3] != KIND_RAW_IMU {
            return Err(DecodeError::Kind);
        }
        if get_u16(bytes, 4) != RAW_IMU_PAYLOAD_LEN {
            return Err(DecodeError::PayloadLength);
        }

        let expected_crc = get_u16(bytes, CRC_OFFSET);
        if crc16_ccitt_false(&bytes[..CRC_OFFSET]) != expected_crc {
            return Err(DecodeError::Crc);
        }

        Ok(Self {
            sequence: get_u32(bytes, 6),
            timestamp_us: get_u64(bytes, 10),
            accel_raw: [get_i16(bytes, 18), get_i16(bytes, 20), get_i16(bytes, 22)],
            temperature_raw: get_i16(bytes, 24),
            gyro_raw: [get_i16(bytes, 26), get_i16(bytes, 28), get_i16(bytes, 30)],
            status: get_u16(bytes, 32),
            dropped_frames: get_u16(bytes, 34),
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

fn put_i16(dst: &mut [u8], offset: usize, value: i16) {
    dst[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
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

fn get_i16(src: &[u8], offset: usize) -> i16 {
    i16::from_le_bytes([src[offset], src[offset + 1]])
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
    fn crc_matches_reference_vector() {
        assert_eq!(crc16_ccitt_false(b"123456789"), 0x29b1);
    }

    #[test]
    fn raw_imu_frame_round_trips() {
        let frame = RawImuFrame {
            sequence: 0x1234_5678,
            timestamp_us: 0x0123_4567_89ab_cdef,
            accel_raw: [-123, 456, -789],
            temperature_raw: 321,
            gyro_raw: [111, -222, 333],
            status: status::BUS_READY | status::IMU_PRESENT | status::SAMPLE_VALID,
            dropped_frames: 7,
        };

        let encoded = frame.encode();
        assert_eq!(RawImuFrame::decode(&encoded), Ok(frame));
    }

    #[test]
    fn crc_rejects_corruption() {
        let mut encoded = RawImuFrame::default().encode();
        encoded[20] ^= 0x01;
        assert_eq!(RawImuFrame::decode(&encoded), Err(DecodeError::Crc));
    }
}

#![no_std]

pub const MAGIC: [u8; 2] = *b"SW";
pub const VERSION: u8 = 1;
pub const KIND_RAW_IMU: u8 = 1;
pub const KIND_SENSOR_SNAPSHOT: u8 = 2;
pub const RAW_IMU_PAYLOAD_LEN: u16 = 30;
pub const RAW_IMU_FRAME_LEN: usize = 38;
pub const SENSOR_SNAPSHOT_PAYLOAD_LEN: u16 = 36;
pub const SENSOR_SNAPSHOT_FRAME_LEN: usize = 44;
const HEADER_LEN: usize = 6;
const CRC_LEN: usize = 2;

pub mod status {
    pub const BUS_READY: u16 = 1 << 0;
    pub const IMU_PRESENT: u16 = 1 << 1;
    pub const IMU_CONFIGURED: u16 = 1 << 2;
    pub const SAMPLE_VALID: u16 = 1 << 3;
    pub const ENCODER_1_VALID: u16 = 1 << 4;
    pub const ENCODER_2_VALID: u16 = 1 << 5;
    pub const BATTERY_ADC_VALID: u16 = 1 << 6;
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

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SensorSnapshotFrame {
    pub sequence: u32,
    pub timestamp_us: u64,
    pub accel_raw: [i16; 3],
    pub temperature_raw: i16,
    pub gyro_raw: [i16; 3],
    pub encoder_1_count: u16,
    pub encoder_2_count: u16,
    pub battery_adc_raw: u16,
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
        write_header(&mut out, KIND_RAW_IMU, RAW_IMU_PAYLOAD_LEN);
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
        write_crc(&mut out);
        out
    }

    pub fn decode(bytes: &[u8; RAW_IMU_FRAME_LEN]) -> Result<Self, DecodeError> {
        validate_frame(bytes, KIND_RAW_IMU, RAW_IMU_PAYLOAD_LEN)?;
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

impl SensorSnapshotFrame {
    pub fn encode(self) -> [u8; SENSOR_SNAPSHOT_FRAME_LEN] {
        let mut out = [0_u8; SENSOR_SNAPSHOT_FRAME_LEN];
        write_header(&mut out, KIND_SENSOR_SNAPSHOT, SENSOR_SNAPSHOT_PAYLOAD_LEN);
        put_u32(&mut out, 6, self.sequence);
        put_u64(&mut out, 10, self.timestamp_us);
        put_i16(&mut out, 18, self.accel_raw[0]);
        put_i16(&mut out, 20, self.accel_raw[1]);
        put_i16(&mut out, 22, self.accel_raw[2]);
        put_i16(&mut out, 24, self.temperature_raw);
        put_i16(&mut out, 26, self.gyro_raw[0]);
        put_i16(&mut out, 28, self.gyro_raw[1]);
        put_i16(&mut out, 30, self.gyro_raw[2]);
        put_u16(&mut out, 32, self.encoder_1_count);
        put_u16(&mut out, 34, self.encoder_2_count);
        put_u16(&mut out, 36, self.battery_adc_raw);
        put_u16(&mut out, 38, self.status);
        put_u16(&mut out, 40, self.dropped_frames);
        write_crc(&mut out);
        out
    }

    pub fn decode(bytes: &[u8; SENSOR_SNAPSHOT_FRAME_LEN]) -> Result<Self, DecodeError> {
        validate_frame(bytes, KIND_SENSOR_SNAPSHOT, SENSOR_SNAPSHOT_PAYLOAD_LEN)?;
        Ok(Self {
            sequence: get_u32(bytes, 6),
            timestamp_us: get_u64(bytes, 10),
            accel_raw: [get_i16(bytes, 18), get_i16(bytes, 20), get_i16(bytes, 22)],
            temperature_raw: get_i16(bytes, 24),
            gyro_raw: [get_i16(bytes, 26), get_i16(bytes, 28), get_i16(bytes, 30)],
            encoder_1_count: get_u16(bytes, 32),
            encoder_2_count: get_u16(bytes, 34),
            battery_adc_raw: get_u16(bytes, 36),
            status: get_u16(bytes, 38),
            dropped_frames: get_u16(bytes, 40),
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

fn write_header(dst: &mut [u8], kind: u8, payload_len: u16) {
    dst[0] = MAGIC[0];
    dst[1] = MAGIC[1];
    dst[2] = VERSION;
    dst[3] = kind;
    put_u16(dst, 4, payload_len);
}

fn write_crc(dst: &mut [u8]) {
    let crc_offset = dst.len() - CRC_LEN;
    let crc = crc16_ccitt_false(&dst[..crc_offset]);
    put_u16(dst, crc_offset, crc);
}

fn validate_frame(bytes: &[u8], kind: u8, payload_len: u16) -> Result<(), DecodeError> {
    if bytes.len() != HEADER_LEN + usize::from(payload_len) + CRC_LEN {
        return Err(DecodeError::PayloadLength);
    }
    if bytes[0] != MAGIC[0] || bytes[1] != MAGIC[1] {
        return Err(DecodeError::Magic);
    }
    if bytes[2] != VERSION {
        return Err(DecodeError::Version);
    }
    if bytes[3] != kind {
        return Err(DecodeError::Kind);
    }
    if get_u16(bytes, 4) != payload_len {
        return Err(DecodeError::PayloadLength);
    }

    let crc_offset = bytes.len() - CRC_LEN;
    if crc16_ccitt_false(&bytes[..crc_offset]) != get_u16(bytes, crc_offset) {
        return Err(DecodeError::Crc);
    }
    Ok(())
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
    fn sensor_snapshot_round_trips() {
        let frame = SensorSnapshotFrame {
            sequence: 42,
            timestamp_us: 123_456_789,
            accel_raw: [1, 2, 3],
            temperature_raw: -4,
            gyro_raw: [-5, 6, -7],
            encoder_1_count: 0xff00,
            encoder_2_count: 0x00ff,
            battery_adc_raw: 2048,
            status: status::SAMPLE_VALID
                | status::ENCODER_1_VALID
                | status::ENCODER_2_VALID
                | status::BATTERY_ADC_VALID,
            dropped_frames: 9,
        };

        let encoded = frame.encode();
        assert_eq!(SensorSnapshotFrame::decode(&encoded), Ok(frame));
    }

    #[test]
    fn crc_rejects_corruption() {
        let mut encoded = SensorSnapshotFrame::default().encode();
        encoded[20] ^= 0x01;
        assert_eq!(SensorSnapshotFrame::decode(&encoded), Err(DecodeError::Crc));
    }
}

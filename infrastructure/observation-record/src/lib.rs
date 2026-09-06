#![no_std]

use swp_plant_observation::{
    AcquisitionStatus, MeasurementQuality, RawBatteryObservation, RawEncoderObservation,
    RawImuObservation, RawObservation, TimestampEvidence,
};

pub const MAGIC: [u8; 2] = *b"SW";
pub const VERSION: u8 = 1;
pub const KIND_RAW_OBSERVATION: u8 = 1;
pub const RAW_OBSERVATION_PAYLOAD_LEN: u16 = 72;
pub const RAW_OBSERVATION_RECORD_LEN: usize = 80;
pub const UNKNOWN_OFFSET_US: u32 = u32::MAX;
pub const UNKNOWN_SAMPLE_OFFSET_US: i32 = i32::MIN;

const HEADER_LEN: usize = 6;
const CRC_LEN: usize = 2;
const CRC_OFFSET: usize = RAW_OBSERVATION_RECORD_LEN - CRC_LEN;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RecordedObservation {
    pub observation: RawObservation,
    pub dropped_records: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecodeError {
    Magic,
    Version,
    Kind,
    PayloadLength,
    Crc,
    Timing,
}

impl RecordedObservation {
    pub fn encode(self) -> [u8; RAW_OBSERVATION_RECORD_LEN] {
        let mut out = [0_u8; RAW_OBSERVATION_RECORD_LEN];
        let observation = self.observation;
        let base = observation.acquisition_started_us;

        out[0] = MAGIC[0];
        out[1] = MAGIC[1];
        out[2] = VERSION;
        out[3] = KIND_RAW_OBSERVATION;
        put_u16(&mut out, 4, RAW_OBSERVATION_PAYLOAD_LEN);
        put_u32(&mut out, 6, observation.sample_index);
        put_u64(&mut out, 10, base);
        put_u32(
            &mut out,
            18,
            forward_offset(base, observation.acquisition_completed_us),
        );
        put_i32(
            &mut out,
            22,
            signed_evidence_offset(base, observation.imu.source_sample_at_us),
        );
        put_u32(
            &mut out,
            26,
            evidence_offset(base, observation.imu.read_started_at_us),
        );
        put_u32(
            &mut out,
            30,
            evidence_offset(base, observation.imu.read_completed_at_us),
        );
        put_i16(&mut out, 34, observation.imu.accel_raw[0]);
        put_i16(&mut out, 36, observation.imu.accel_raw[1]);
        put_i16(&mut out, 38, observation.imu.accel_raw[2]);
        put_i16(&mut out, 40, observation.imu.temperature_raw);
        put_i16(&mut out, 42, observation.imu.gyro_raw[0]);
        put_i16(&mut out, 44, observation.imu.gyro_raw[1]);
        put_i16(&mut out, 46, observation.imu.gyro_raw[2]);
        put_u16(&mut out, 48, observation.imu.quality.bits());

        put_u32(
            &mut out,
            50,
            evidence_offset(base, observation.encoders[0].captured_at_us),
        );
        put_u16(&mut out, 54, observation.encoders[0].count);
        put_u16(&mut out, 56, observation.encoders[0].quality.bits());
        put_u32(
            &mut out,
            58,
            evidence_offset(base, observation.encoders[1].captured_at_us),
        );
        put_u16(&mut out, 62, observation.encoders[1].count);
        put_u16(&mut out, 64, observation.encoders[1].quality.bits());

        put_u32(
            &mut out,
            66,
            evidence_offset(base, observation.battery.read_completed_at_us),
        );
        put_u16(&mut out, 70, observation.battery.adc_raw);
        put_u16(&mut out, 72, observation.battery.quality.bits());
        put_u16(&mut out, 74, observation.acquisition_status.bits());
        put_u16(&mut out, 76, self.dropped_records);

        let crc = crc16_ccitt_false(&out[..CRC_OFFSET]);
        put_u16(&mut out, CRC_OFFSET, crc);
        out
    }

    pub fn decode(bytes: &[u8; RAW_OBSERVATION_RECORD_LEN]) -> Result<Self, DecodeError> {
        validate_record(bytes)?;

        let base = get_u64(bytes, 10);
        let acquisition_completed_us = decode_required_offset(base, get_u32(bytes, 18))?;

        Ok(Self {
            observation: RawObservation {
                sample_index: get_u32(bytes, 6),
                acquisition_started_us: base,
                acquisition_completed_us,
                imu: RawImuObservation {
                    source_sample_at_us: decode_signed_evidence(base, get_i32(bytes, 22)),
                    read_started_at_us: decode_evidence(base, get_u32(bytes, 26)),
                    read_completed_at_us: decode_evidence(base, get_u32(bytes, 30)),
                    accel_raw: [get_i16(bytes, 34), get_i16(bytes, 36), get_i16(bytes, 38)],
                    temperature_raw: get_i16(bytes, 40),
                    gyro_raw: [get_i16(bytes, 42), get_i16(bytes, 44), get_i16(bytes, 46)],
                    quality: MeasurementQuality::from_bits(get_u16(bytes, 48)),
                },
                encoders: [
                    RawEncoderObservation {
                        captured_at_us: decode_evidence(base, get_u32(bytes, 50)),
                        count: get_u16(bytes, 54),
                        quality: MeasurementQuality::from_bits(get_u16(bytes, 56)),
                    },
                    RawEncoderObservation {
                        captured_at_us: decode_evidence(base, get_u32(bytes, 58)),
                        count: get_u16(bytes, 62),
                        quality: MeasurementQuality::from_bits(get_u16(bytes, 64)),
                    },
                ],
                battery: RawBatteryObservation {
                    read_completed_at_us: decode_evidence(base, get_u32(bytes, 66)),
                    adc_raw: get_u16(bytes, 70),
                    quality: MeasurementQuality::from_bits(get_u16(bytes, 72)),
                },
                acquisition_status: AcquisitionStatus::from_bits(get_u16(bytes, 74)),
            },
            dropped_records: get_u16(bytes, 76),
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

fn validate_record(bytes: &[u8; RAW_OBSERVATION_RECORD_LEN]) -> Result<(), DecodeError> {
    if bytes[0] != MAGIC[0] || bytes[1] != MAGIC[1] {
        return Err(DecodeError::Magic);
    }
    if bytes[2] != VERSION {
        return Err(DecodeError::Version);
    }
    if bytes[3] != KIND_RAW_OBSERVATION {
        return Err(DecodeError::Kind);
    }
    if get_u16(bytes, 4) != RAW_OBSERVATION_PAYLOAD_LEN
        || RAW_OBSERVATION_RECORD_LEN
            != HEADER_LEN + usize::from(RAW_OBSERVATION_PAYLOAD_LEN) + CRC_LEN
    {
        return Err(DecodeError::PayloadLength);
    }
    if crc16_ccitt_false(&bytes[..CRC_OFFSET]) != get_u16(bytes, CRC_OFFSET) {
        return Err(DecodeError::Crc);
    }
    Ok(())
}

fn forward_offset(base: u64, timestamp: u64) -> u32 {
    let Some(delta) = timestamp.checked_sub(base) else {
        return UNKNOWN_OFFSET_US;
    };
    u32::try_from(delta).unwrap_or(UNKNOWN_OFFSET_US)
}

fn evidence_offset(base: u64, timestamp: TimestampEvidence) -> u32 {
    match timestamp {
        TimestampEvidence::Unknown => UNKNOWN_OFFSET_US,
        TimestampEvidence::Known(value) => forward_offset(base, value),
    }
}

fn signed_evidence_offset(base: u64, timestamp: TimestampEvidence) -> i32 {
    let TimestampEvidence::Known(value) = timestamp else {
        return UNKNOWN_SAMPLE_OFFSET_US;
    };

    if value >= base {
        i32::try_from(value - base).unwrap_or(UNKNOWN_SAMPLE_OFFSET_US)
    } else {
        let delta = base - value;
        if delta <= i32::MAX as u64 {
            -(delta as i32)
        } else {
            UNKNOWN_SAMPLE_OFFSET_US
        }
    }
}

fn decode_required_offset(base: u64, offset: u32) -> Result<u64, DecodeError> {
    if offset == UNKNOWN_OFFSET_US {
        return Err(DecodeError::Timing);
    }
    base.checked_add(u64::from(offset))
        .ok_or(DecodeError::Timing)
}

fn decode_evidence(base: u64, offset: u32) -> TimestampEvidence {
    if offset == UNKNOWN_OFFSET_US {
        TimestampEvidence::Unknown
    } else {
        base.checked_add(u64::from(offset))
            .map_or(TimestampEvidence::Unknown, TimestampEvidence::Known)
    }
}

fn decode_signed_evidence(base: u64, offset: i32) -> TimestampEvidence {
    if offset == UNKNOWN_SAMPLE_OFFSET_US {
        return TimestampEvidence::Unknown;
    }

    let value = if offset >= 0 {
        base.checked_add(offset as u64)
    } else {
        base.checked_sub(u64::from(offset.unsigned_abs()))
    };
    value.map_or(TimestampEvidence::Unknown, TimestampEvidence::Known)
}

fn put_i16(dst: &mut [u8], offset: usize, value: i16) {
    dst[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
}

fn put_i32(dst: &mut [u8], offset: usize, value: i32) {
    dst[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
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

fn get_i32(src: &[u8], offset: usize) -> i32 {
    i32::from_le_bytes([
        src[offset],
        src[offset + 1],
        src[offset + 2],
        src[offset + 3],
    ])
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
    fn raw_observation_round_trips_without_inventing_sample_time() {
        let observation = RawObservation {
            sample_index: 17,
            acquisition_started_us: 1_000_000,
            acquisition_completed_us: 1_001_750,
            imu: RawImuObservation {
                source_sample_at_us: TimestampEvidence::Unknown,
                read_started_at_us: TimestampEvidence::Known(1_000_040),
                read_completed_at_us: TimestampEvidence::Known(1_001_400),
                accel_raw: [-1, 2, -3],
                temperature_raw: 4,
                gyro_raw: [-5, 6, -7],
                quality: MeasurementQuality::AVAILABLE | MeasurementQuality::IO_OK,
            },
            encoders: [
                RawEncoderObservation {
                    captured_at_us: TimestampEvidence::Known(1_001_420),
                    count: 100,
                    quality: MeasurementQuality::AVAILABLE
                        | MeasurementQuality::IO_OK
                        | MeasurementQuality::TIMING_VALID,
                },
                RawEncoderObservation {
                    captured_at_us: TimestampEvidence::Known(1_001_430),
                    count: 200,
                    quality: MeasurementQuality::AVAILABLE
                        | MeasurementQuality::IO_OK
                        | MeasurementQuality::TIMING_VALID,
                },
            ],
            battery: RawBatteryObservation {
                read_completed_at_us: TimestampEvidence::Known(1_001_700),
                adc_raw: 3000,
                quality: MeasurementQuality::AVAILABLE | MeasurementQuality::IO_OK,
            },
            acquisition_status: AcquisitionStatus::BUS_READY
                | AcquisitionStatus::IMU_PRESENT
                | AcquisitionStatus::IMU_CONFIGURED,
        };
        let record = RecordedObservation {
            observation,
            dropped_records: 3,
        };
        let encoded = record.encode();
        assert_eq!(RecordedObservation::decode(&encoded), Ok(record));
        assert_eq!(
            RecordedObservation::decode(&encoded)
                .unwrap()
                .observation
                .imu
                .source_sample_at_us,
            TimestampEvidence::Unknown
        );
    }

    #[test]
    fn known_source_sample_before_acquisition_round_trips() {
        let observation = RawObservation {
            acquisition_started_us: 10_000,
            acquisition_completed_us: 10_100,
            imu: RawImuObservation {
                source_sample_at_us: TimestampEvidence::Known(9_750),
                ..RawImuObservation::default()
            },
            ..RawObservation::default()
        };
        let record = RecordedObservation {
            observation,
            dropped_records: 0,
        };
        let decoded = RecordedObservation::decode(&record.encode()).unwrap();
        assert_eq!(decoded, record);
    }

    #[test]
    fn crc_rejects_corruption() {
        let mut encoded = RecordedObservation::default().encode();
        encoded[40] ^= 0x01;
        assert_eq!(RecordedObservation::decode(&encoded), Err(DecodeError::Crc));
    }
}

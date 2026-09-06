#![no_std]

use swp_actuator_model::{ActuatorPairCommand, BoundedActuatorCommand};
use swp_plant_model::ReducedBalanceState;
use swp_robot_domain::{GeneralizedDemand, NormalizedCommand, StateValidity, TorqueNm};
use swp_runtime_state::{OperatingState, SensorTimingHealth};
use swp_runtime_supervisor::{ControlWatchdogHealth, RuntimeFaults};

pub const MAGIC: [u8; 2] = *b"SW";
pub const VERSION: u8 = 1;
pub const KIND_RUNTIME_OBSERVATION: u8 = 3;
pub const RUNTIME_OBSERVATION_PAYLOAD_LEN: u16 = 120;
pub const RUNTIME_OBSERVATION_RECORD_LEN: usize = 128;

const CRC_OFFSET: usize = RUNTIME_OBSERVATION_RECORD_LEN - 2;
const FLAG_AUTHORIZED: u16 = 1 << 0;
const FLAG_DRIVE_SATURATED: u16 = 1 << 1;
const FLAG_REACTION_SATURATED: u16 = 1 << 2;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RuntimeObservation {
    pub sample_index: u32,
    pub timestamp_us: u64,
    pub estimated_state: ReducedBalanceState,
    pub reference: ReducedBalanceState,
    pub demand: GeneralizedDemand,
    pub bounded_commands: ActuatorPairCommand,
    pub operating_state: OperatingState,
    pub timing: SensorTimingHealth,
    pub estimate_validity: StateValidity,
    pub watchdog: ControlWatchdogHealth,
    pub authority_reasons: u16,
    pub runtime_faults: RuntimeFaults,
    pub authorized: bool,
    pub outer_target_velocity_m_per_s: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RecordedRuntimeObservation {
    pub observation: RuntimeObservation,
    pub dropped_records: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecodeError {
    Magic,
    Version,
    Kind,
    PayloadLength,
    Crc,
    EnumValue,
    CommandValue,
}

impl RecordedRuntimeObservation {
    pub fn encode(self) -> [u8; RUNTIME_OBSERVATION_RECORD_LEN] {
        let mut out = [0_u8; RUNTIME_OBSERVATION_RECORD_LEN];
        let observation = self.observation;

        out[0] = MAGIC[0];
        out[1] = MAGIC[1];
        out[2] = VERSION;
        out[3] = KIND_RUNTIME_OBSERVATION;
        put_u16(&mut out, 4, RUNTIME_OBSERVATION_PAYLOAD_LEN);
        put_u32(&mut out, 6, observation.sample_index);
        put_u64(&mut out, 10, observation.timestamp_us);

        put_state(&mut out, 18, observation.estimated_state);
        put_state(&mut out, 46, observation.reference);
        put_f32(&mut out, 74, observation.demand.drive_wheel_torque.0);
        put_f32(&mut out, 78, observation.demand.reaction_wheel_torque.0);
        put_f32(
            &mut out,
            82,
            observation.bounded_commands.drive.command.get(),
        );
        put_f32(
            &mut out,
            86,
            observation.bounded_commands.reaction.command.get(),
        );
        put_f32(
            &mut out,
            90,
            observation.bounded_commands.drive.predicted_torque_nm.0,
        );
        put_f32(
            &mut out,
            94,
            observation.bounded_commands.reaction.predicted_torque_nm.0,
        );

        out[98] = encode_operating_state(observation.operating_state);
        out[99] = encode_timing(observation.timing);
        out[100] = encode_validity(observation.estimate_validity);
        out[101] = encode_watchdog(observation.watchdog);
        put_u16(&mut out, 102, observation.authority_reasons);
        put_u16(&mut out, 104, observation.runtime_faults.bits());

        let mut flags = 0_u16;
        if observation.authorized {
            flags |= FLAG_AUTHORIZED;
        }
        if observation.bounded_commands.drive.saturated {
            flags |= FLAG_DRIVE_SATURATED;
        }
        if observation.bounded_commands.reaction.saturated {
            flags |= FLAG_REACTION_SATURATED;
        }
        put_u16(&mut out, 106, flags);
        put_u16(&mut out, 108, self.dropped_records);
        put_f32(&mut out, 110, observation.outer_target_velocity_m_per_s);

        let crc = crc16_ccitt_false(&out[..CRC_OFFSET]);
        put_u16(&mut out, CRC_OFFSET, crc);
        out
    }

    pub fn decode(bytes: &[u8; RUNTIME_OBSERVATION_RECORD_LEN]) -> Result<Self, DecodeError> {
        if bytes[0] != MAGIC[0] || bytes[1] != MAGIC[1] {
            return Err(DecodeError::Magic);
        }
        if bytes[2] != VERSION {
            return Err(DecodeError::Version);
        }
        if bytes[3] != KIND_RUNTIME_OBSERVATION {
            return Err(DecodeError::Kind);
        }
        if get_u16(bytes, 4) != RUNTIME_OBSERVATION_PAYLOAD_LEN {
            return Err(DecodeError::PayloadLength);
        }
        if crc16_ccitt_false(&bytes[..CRC_OFFSET]) != get_u16(bytes, CRC_OFFSET) {
            return Err(DecodeError::Crc);
        }

        let flags = get_u16(bytes, 106);
        let drive_command =
            NormalizedCommand::new(get_f32(bytes, 82)).ok_or(DecodeError::CommandValue)?;
        let reaction_command =
            NormalizedCommand::new(get_f32(bytes, 86)).ok_or(DecodeError::CommandValue)?;

        Ok(Self {
            observation: RuntimeObservation {
                sample_index: get_u32(bytes, 6),
                timestamp_us: get_u64(bytes, 10),
                estimated_state: get_state(bytes, 18),
                reference: get_state(bytes, 46),
                demand: GeneralizedDemand {
                    drive_wheel_torque: TorqueNm(get_f32(bytes, 74)),
                    reaction_wheel_torque: TorqueNm(get_f32(bytes, 78)),
                },
                bounded_commands: ActuatorPairCommand {
                    drive: BoundedActuatorCommand {
                        command: drive_command,
                        saturated: flags & FLAG_DRIVE_SATURATED != 0,
                        predicted_torque_nm: TorqueNm(get_f32(bytes, 90)),
                    },
                    reaction: BoundedActuatorCommand {
                        command: reaction_command,
                        saturated: flags & FLAG_REACTION_SATURATED != 0,
                        predicted_torque_nm: TorqueNm(get_f32(bytes, 94)),
                    },
                },
                operating_state: decode_operating_state(bytes[98])?,
                timing: decode_timing(bytes[99])?,
                estimate_validity: decode_validity(bytes[100])?,
                watchdog: decode_watchdog(bytes[101])?,
                authority_reasons: get_u16(bytes, 102),
                runtime_faults: RuntimeFaults::from_bits(get_u16(bytes, 104)),
                authorized: flags & FLAG_AUTHORIZED != 0,
                outer_target_velocity_m_per_s: get_f32(bytes, 110),
            },
            dropped_records: get_u16(bytes, 108),
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

fn put_state(dst: &mut [u8], offset: usize, state: ReducedBalanceState) {
    for (index, value) in state.as_vector().iter().enumerate() {
        put_f32(dst, offset + index * 4, *value);
    }
}

fn get_state(src: &[u8], offset: usize) -> ReducedBalanceState {
    ReducedBalanceState {
        forward_position_m: get_f32(src, offset),
        forward_velocity_m_per_s: get_f32(src, offset + 4),
        pitch_rad: get_f32(src, offset + 8),
        pitch_rate_rad_per_s: get_f32(src, offset + 12),
        roll_rad: get_f32(src, offset + 16),
        roll_rate_rad_per_s: get_f32(src, offset + 20),
        reaction_wheel_rate_rad_per_s: get_f32(src, offset + 24),
    }
}

fn encode_operating_state(state: OperatingState) -> u8 {
    match state {
        OperatingState::Boot => 0,
        OperatingState::HardwareCheck => 1,
        OperatingState::Standby => 2,
        OperatingState::CaptureWindow => 3,
        OperatingState::Balancing => 4,
        OperatingState::MomentumLimited => 5,
        OperatingState::Fault => 6,
    }
}
fn decode_operating_state(value: u8) -> Result<OperatingState, DecodeError> {
    match value {
        0 => Ok(OperatingState::Boot),
        1 => Ok(OperatingState::HardwareCheck),
        2 => Ok(OperatingState::Standby),
        3 => Ok(OperatingState::CaptureWindow),
        4 => Ok(OperatingState::Balancing),
        5 => Ok(OperatingState::MomentumLimited),
        6 => Ok(OperatingState::Fault),
        _ => Err(DecodeError::EnumValue),
    }
}
fn encode_timing(value: SensorTimingHealth) -> u8 {
    match value {
        SensorTimingHealth::Startup => 0,
        SensorTimingHealth::Healthy => 1,
        SensorTimingHealth::Late => 2,
        SensorTimingHealth::Timeout => 3,
    }
}
fn decode_timing(value: u8) -> Result<SensorTimingHealth, DecodeError> {
    match value {
        0 => Ok(SensorTimingHealth::Startup),
        1 => Ok(SensorTimingHealth::Healthy),
        2 => Ok(SensorTimingHealth::Late),
        3 => Ok(SensorTimingHealth::Timeout),
        _ => Err(DecodeError::EnumValue),
    }
}
fn encode_validity(value: StateValidity) -> u8 {
    match value {
        StateValidity::Invalid => 0,
        StateValidity::Valid => 1,
    }
}
fn decode_validity(value: u8) -> Result<StateValidity, DecodeError> {
    match value {
        0 => Ok(StateValidity::Invalid),
        1 => Ok(StateValidity::Valid),
        _ => Err(DecodeError::EnumValue),
    }
}
fn encode_watchdog(value: ControlWatchdogHealth) -> u8 {
    match value {
        ControlWatchdogHealth::Startup => 0,
        ControlWatchdogHealth::Healthy => 1,
        ControlWatchdogHealth::Timeout => 2,
    }
}
fn decode_watchdog(value: u8) -> Result<ControlWatchdogHealth, DecodeError> {
    match value {
        0 => Ok(ControlWatchdogHealth::Startup),
        1 => Ok(ControlWatchdogHealth::Healthy),
        2 => Ok(ControlWatchdogHealth::Timeout),
        _ => Err(DecodeError::EnumValue),
    }
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
fn put_f32(dst: &mut [u8], offset: usize, value: f32) {
    put_u32(dst, offset, value.to_bits());
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
fn get_f32(src: &[u8], offset: usize) -> f32 {
    f32::from_bits(get_u32(src, offset))
}

#[cfg(test)]
mod tests {
    use super::*;
    fn bounded(command: f32, saturated: bool, predicted: f32) -> BoundedActuatorCommand {
        BoundedActuatorCommand {
            command: NormalizedCommand::new(command).unwrap(),
            saturated,
            predicted_torque_nm: TorqueNm(predicted),
        }
    }

    #[test]
    fn runtime_observation_round_trips() {
        let state = ReducedBalanceState {
            forward_position_m: 1.0,
            forward_velocity_m_per_s: 2.0,
            pitch_rad: 0.1,
            pitch_rate_rad_per_s: 0.2,
            roll_rad: -0.1,
            roll_rate_rad_per_s: -0.2,
            reaction_wheel_rate_rad_per_s: 30.0,
        };
        let record = RecordedRuntimeObservation {
            observation: RuntimeObservation {
                sample_index: 7,
                timestamp_us: 123_456,
                estimated_state: state,
                reference: ReducedBalanceState {
                    forward_velocity_m_per_s: 0.5,
                    pitch_rad: 0.03,
                    ..ReducedBalanceState::default()
                },
                demand: GeneralizedDemand {
                    drive_wheel_torque: TorqueNm(0.25),
                    reaction_wheel_torque: TorqueNm(-0.4),
                },
                bounded_commands: ActuatorPairCommand {
                    drive: bounded(0.4, false, 0.24),
                    reaction: bounded(-1.0, true, -0.35),
                },
                operating_state: OperatingState::MomentumLimited,
                timing: SensorTimingHealth::Healthy,
                estimate_validity: StateValidity::Valid,
                watchdog: ControlWatchdogHealth::Healthy,
                authority_reasons: 0x12,
                runtime_faults: RuntimeFaults::from_bits(0x20),
                authorized: true,
                outer_target_velocity_m_per_s: 0.5,
            },
            dropped_records: 3,
        };
        let encoded = record.encode();
        assert_eq!(RecordedRuntimeObservation::decode(&encoded), Ok(record));
    }

    #[test]
    fn crc_rejects_corruption() {
        let record = RecordedRuntimeObservation {
            observation: RuntimeObservation {
                sample_index: 1,
                timestamp_us: 2,
                estimated_state: ReducedBalanceState::default(),
                reference: ReducedBalanceState::default(),
                demand: GeneralizedDemand::default(),
                bounded_commands: ActuatorPairCommand {
                    drive: bounded(0.0, false, 0.0),
                    reaction: bounded(0.0, false, 0.0),
                },
                operating_state: OperatingState::Standby,
                timing: SensorTimingHealth::Startup,
                estimate_validity: StateValidity::Invalid,
                watchdog: ControlWatchdogHealth::Startup,
                authority_reasons: 0,
                runtime_faults: RuntimeFaults::NONE,
                authorized: false,
                outer_target_velocity_m_per_s: 0.0,
            },
            dropped_records: 0,
        };
        let mut encoded = record.encode();
        encoded[50] ^= 0x40;
        assert_eq!(
            RecordedRuntimeObservation::decode(&encoded),
            Err(DecodeError::Crc)
        );
    }
}

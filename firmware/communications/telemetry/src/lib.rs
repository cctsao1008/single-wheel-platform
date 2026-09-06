#![no_std]
#![forbid(unsafe_code)]

pub const TELEMETRY_PROTOCOL_VERSION: u8 = 1;
pub const TELEMETRY_PACKET_KIND_RUNTIME: u8 = 1;
pub const TELEMETRY_PACKET_LEN: usize = 48;

const MAGIC: [u8; 2] = *b"SW";

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TelemetrySnapshot {
    pub timestamp_us: u64,
    pub sample_index: u32,
    pub operating_state: u8,
    pub timing_health: u8,
    pub watchdog_health: u8,
    pub authorized: bool,
    pub runtime_fault_bits: u32,
    pub authority_reason_bits: u32,
    pub forward_velocity_mm_per_s: i16,
    pub pitch_mrad: i16,
    pub drive_demand_mnm: i16,
    pub reaction_demand_mnm: i16,
    pub drive_command_permille: i16,
    pub reaction_command_permille: i16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TelemetryTxOutcome {
    Sent,
    Busy,
}

pub trait TelemetryTransport {
    type Error;

    fn try_send(&mut self, packet: &[u8]) -> Result<TelemetryTxOutcome, Self::Error>;
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TelemetryPublisherStats {
    pub opportunities: u32,
    pub sent: u32,
    pub dropped_busy: u32,
}

pub struct TelemetryPublisher<T> {
    transport: T,
    next_sequence: u32,
    stats: TelemetryPublisherStats,
}

impl<T> TelemetryPublisher<T>
where
    T: TelemetryTransport,
{
    pub const fn new(transport: T) -> Self {
        Self {
            transport,
            next_sequence: 0,
            stats: TelemetryPublisherStats {
                opportunities: 0,
                sent: 0,
                dropped_busy: 0,
            },
        }
    }

    /// Publish only the snapshot supplied for this opportunity.
    ///
    /// The publisher intentionally owns no queue. A busy transport drops this
    /// opportunity and the next call publishes the next fresh snapshot.
    pub fn publish_latest(
        &mut self,
        snapshot: TelemetrySnapshot,
    ) -> Result<TelemetryTxOutcome, T::Error> {
        let sequence = self.next_sequence;
        self.next_sequence = self.next_sequence.wrapping_add(1);
        self.stats.opportunities = self.stats.opportunities.wrapping_add(1);
        let packet = encode_runtime_packet(sequence, snapshot);
        let outcome = self.transport.try_send(&packet)?;
        match outcome {
            TelemetryTxOutcome::Sent => self.stats.sent = self.stats.sent.wrapping_add(1),
            TelemetryTxOutcome::Busy => {
                self.stats.dropped_busy = self.stats.dropped_busy.wrapping_add(1)
            }
        }
        Ok(outcome)
    }

    pub const fn stats(&self) -> TelemetryPublisherStats {
        self.stats
    }

    pub fn transport(&self) -> &T {
        &self.transport
    }

    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.transport
    }

    pub fn into_transport(self) -> T {
        self.transport
    }
}

pub fn encode_runtime_packet(
    sequence: u32,
    snapshot: TelemetrySnapshot,
) -> [u8; TELEMETRY_PACKET_LEN] {
    let mut out = [0_u8; TELEMETRY_PACKET_LEN];
    out[0..2].copy_from_slice(&MAGIC);
    out[2] = TELEMETRY_PROTOCOL_VERSION;
    out[3] = TELEMETRY_PACKET_KIND_RUNTIME;
    put_u32(&mut out, 4, sequence);
    put_u64(&mut out, 8, snapshot.timestamp_us);
    put_u32(&mut out, 16, snapshot.sample_index);
    out[20] = snapshot.operating_state;
    out[21] = snapshot.timing_health;
    out[22] = snapshot.watchdog_health;
    out[23] = u8::from(snapshot.authorized);
    put_u32(&mut out, 24, snapshot.runtime_fault_bits);
    put_u32(&mut out, 28, snapshot.authority_reason_bits);
    put_i16(&mut out, 32, snapshot.forward_velocity_mm_per_s);
    put_i16(&mut out, 34, snapshot.pitch_mrad);
    put_i16(&mut out, 36, snapshot.drive_demand_mnm);
    put_i16(&mut out, 38, snapshot.reaction_demand_mnm);
    put_i16(&mut out, 40, snapshot.drive_command_permille);
    put_i16(&mut out, 42, snapshot.reaction_command_permille);
    let crc = crc16_ccitt_false(&out[..46]);
    put_u16(&mut out, 46, crc);
    out
}

pub fn packet_crc_is_valid(packet: &[u8; TELEMETRY_PACKET_LEN]) -> bool {
    let expected = u16::from_le_bytes([packet[46], packet[47]]);
    crc16_ccitt_false(&packet[..46]) == expected
}

fn put_u16(out: &mut [u8], offset: usize, value: u16) {
    out[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
}

fn put_i16(out: &mut [u8], offset: usize, value: i16) {
    out[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
}

fn put_u32(out: &mut [u8], offset: usize, value: u32) {
    out[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

fn put_u64(out: &mut [u8], offset: usize, value: u64) {
    out[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
}

fn crc16_ccitt_false(bytes: &[u8]) -> u16 {
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

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::vec::Vec;

    #[derive(Default)]
    struct MockTransport {
        busy_once: bool,
        packets: Vec<[u8; TELEMETRY_PACKET_LEN]>,
    }

    impl TelemetryTransport for MockTransport {
        type Error = ();

        fn try_send(&mut self, packet: &[u8]) -> Result<TelemetryTxOutcome, Self::Error> {
            if self.busy_once {
                self.busy_once = false;
                return Ok(TelemetryTxOutcome::Busy);
            }
            let mut copy = [0_u8; TELEMETRY_PACKET_LEN];
            copy.copy_from_slice(packet);
            self.packets.push(copy);
            Ok(TelemetryTxOutcome::Sent)
        }
    }

    fn snapshot(sample_index: u32) -> TelemetrySnapshot {
        TelemetrySnapshot {
            timestamp_us: u64::from(sample_index) * 5_000,
            sample_index,
            operating_state: 4,
            timing_health: 1,
            watchdog_health: 1,
            authorized: true,
            runtime_fault_bits: 0x12,
            authority_reason_bits: 0x34,
            forward_velocity_mm_per_s: 123,
            pitch_mrad: -45,
            drive_demand_mnm: 7,
            reaction_demand_mnm: -8,
            drive_command_permille: 91,
            reaction_command_permille: -92,
        }
    }

    #[test]
    fn packet_has_version_sequence_and_crc() {
        let packet = encode_runtime_packet(7, snapshot(11));
        assert_eq!(&packet[0..2], b"SW");
        assert_eq!(packet[2], TELEMETRY_PROTOCOL_VERSION);
        assert_eq!(packet[3], TELEMETRY_PACKET_KIND_RUNTIME);
        assert_eq!(u32::from_le_bytes(packet[4..8].try_into().unwrap()), 7);
        assert_eq!(u32::from_le_bytes(packet[16..20].try_into().unwrap()), 11);
        assert!(packet_crc_is_valid(&packet));
    }

    #[test]
    fn busy_drops_current_snapshot_without_replay_queue() {
        let transport = MockTransport {
            busy_once: true,
            packets: Vec::new(),
        };
        let mut publisher = TelemetryPublisher::new(transport);
        assert_eq!(
            publisher.publish_latest(snapshot(1)).unwrap(),
            TelemetryTxOutcome::Busy
        );
        assert_eq!(
            publisher.publish_latest(snapshot(2)).unwrap(),
            TelemetryTxOutcome::Sent
        );
        let stats = publisher.stats();
        assert_eq!(stats.opportunities, 2);
        assert_eq!(stats.sent, 1);
        assert_eq!(stats.dropped_busy, 1);
        let transport = publisher.into_transport();
        assert_eq!(transport.packets.len(), 1);
        let packet = transport.packets[0];
        assert_eq!(u32::from_le_bytes(packet[4..8].try_into().unwrap()), 1);
        assert_eq!(u32::from_le_bytes(packet[16..20].try_into().unwrap()), 2);
    }
}

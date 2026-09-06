#![no_std]
#![forbid(unsafe_code)]

use swp_telemetry::{TelemetryTransport, TelemetryTxOutcome};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ByteWriteOutcome {
    Written,
    Busy,
}

/// Target-owned byte transport used by the ECB02 integration boundary.
///
/// This contract deliberately says nothing about UART instances, DMA channels,
/// module AT/configuration commands, or connection-state GPIO. Those details are
/// added only when the concrete ONE V2 wiring is integrated and verified.
pub trait ByteTransport {
    type Error;

    fn try_write(&mut self, bytes: &[u8]) -> Result<ByteWriteOutcome, Self::Error>;
}

/// Reusable ECB02 telemetry endpoint.
///
/// It forwards one already-encoded telemetry packet to the target byte
/// transport. It owns no buffering and therefore cannot create a telemetry
/// backlog that feeds back into the control cadence.
pub struct Ecb02TelemetryTransport<T> {
    bytes: T,
}

impl<T> Ecb02TelemetryTransport<T> {
    pub const fn new(bytes: T) -> Self {
        Self { bytes }
    }

    pub fn bytes(&self) -> &T {
        &self.bytes
    }

    pub fn bytes_mut(&mut self) -> &mut T {
        &mut self.bytes
    }

    pub fn into_bytes(self) -> T {
        self.bytes
    }
}

impl<T> TelemetryTransport for Ecb02TelemetryTransport<T>
where
    T: ByteTransport,
{
    type Error = T::Error;

    fn try_send(&mut self, packet: &[u8]) -> Result<TelemetryTxOutcome, Self::Error> {
        match self.bytes.try_write(packet)? {
            ByteWriteOutcome::Written => Ok(TelemetryTxOutcome::Sent),
            ByteWriteOutcome::Busy => Ok(TelemetryTxOutcome::Busy),
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::vec::Vec;
    use swp_telemetry::{TelemetryPublisher, TelemetrySnapshot};

    #[derive(Default)]
    struct MockBytes {
        busy: bool,
        writes: Vec<Vec<u8>>,
    }

    impl ByteTransport for MockBytes {
        type Error = ();

        fn try_write(&mut self, bytes: &[u8]) -> Result<ByteWriteOutcome, Self::Error> {
            if self.busy {
                self.busy = false;
                return Ok(ByteWriteOutcome::Busy);
            }
            self.writes.push(bytes.to_vec());
            Ok(ByteWriteOutcome::Written)
        }
    }

    #[test]
    fn ecb02_transport_preserves_drop_on_busy_policy() {
        let transport = Ecb02TelemetryTransport::new(MockBytes {
            busy: true,
            writes: Vec::new(),
        });
        let mut publisher = TelemetryPublisher::new(transport);
        assert_eq!(
            publisher.publish_latest(TelemetrySnapshot::default()).unwrap(),
            TelemetryTxOutcome::Busy
        );
        assert_eq!(
            publisher.publish_latest(TelemetrySnapshot::default()).unwrap(),
            TelemetryTxOutcome::Sent
        );
        let bytes = publisher.into_transport().into_bytes();
        assert_eq!(bytes.writes.len(), 1);
    }
}

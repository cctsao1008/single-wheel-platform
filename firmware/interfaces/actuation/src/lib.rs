#![no_std]

use swp_runtime_state::AuthorizedActuation;

/// Firmware-owned boundary that may make a runtime-authorized command physically effective.
///
/// Supervisor code produces `AuthorizedActuation`; concrete control-board and
/// motor-driver implementations live behind this trait. A sink must never accept
/// an arbitrary normalized command as a substitute for the authority token.
pub trait ActuationSink {
    type Error;

    /// Apply one command that has already passed runtime authority.
    fn apply_authorized(&mut self, actuation: AuthorizedActuation) -> Result<(), Self::Error>;

    /// Revoke any previously applied command by driving the concrete motor interface
    /// to its configured zero-demand / neutral encoding.
    ///
    /// This is a semantic revocation requirement, not a claim that a particular
    /// electrical state is universally safe. The external hardware behavior still
    /// has to be established during commissioning.
    fn revoke(&mut self) -> Result<(), Self::Error>;
}

/// Target-side transport for one driver-specific electrical/protocol frame.
///
/// A motor-driver crate owns the meaning of `Frame`; an MCU backend owns how that
/// frame reaches pins, timers, PIO, SPI, CAN, or another concrete peripheral.
/// This keeps driver-board semantics independent of the selected control board.
pub trait DriverIo<Frame> {
    type Error;

    fn write_frame(&mut self, frame: Frame) -> Result<(), Self::Error>;
}

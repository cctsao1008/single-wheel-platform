#![no_std]
#![deny(unsafe_code)]

use core::convert::Infallible;

use embedded_hal::{
    delay::DelayNs,
    digital::{InputPin, OutputPin},
    i2c::{
        ErrorKind, ErrorType, I2c, NoAcknowledgeSource, Operation, SevenBitAddress,
    },
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    BusBusy,
    NoAcknowledge(NoAcknowledgeSource),
    ClockStretchTimeout,
}

impl embedded_hal::i2c::Error for Error {
    fn kind(&self) -> ErrorKind {
        match self {
            Self::BusBusy | Self::ClockStretchTimeout => ErrorKind::Bus,
            Self::NoAcknowledge(source) => ErrorKind::NoAcknowledge(*source),
        }
    }
}

/// Blocking, single-master software I2C implementation for open-drain GPIO.
///
/// Setting a line high means releasing it. The external pull-up then produces
/// the high level. Both pins must therefore support reading while configured as
/// open-drain outputs.
pub struct SoftwareI2c<SDA, SCL, D> {
    sda: SDA,
    scl: SCL,
    delay: D,
    half_period_ns: u32,
    stretch_timeout_us: u32,
}

impl<SDA, SCL, D> SoftwareI2c<SDA, SCL, D>
where
    SDA: InputPin<Error = Infallible> + OutputPin<Error = Infallible>,
    SCL: InputPin<Error = Infallible> + OutputPin<Error = Infallible>,
    D: DelayNs,
{
    pub fn new(
        mut sda: SDA,
        mut scl: SCL,
        delay: D,
        half_period_ns: u32,
        stretch_timeout_us: u32,
    ) -> Self {
        let _ = sda.set_high();
        let _ = scl.set_high();

        Self {
            sda,
            scl,
            delay,
            half_period_ns: half_period_ns.max(1),
            stretch_timeout_us: stretch_timeout_us.max(1),
        }
    }

    /// Attempts standard I2C bus recovery: release SDA, pulse SCL up to nine
    /// times, then issue STOP. This is useful after a reset that occurred while
    /// a slave was part-way through a byte transfer.
    pub fn recover_bus(&mut self) -> Result<(), Error> {
        self.release_sda();
        self.release_scl()?;

        if self.sda_high() {
            return Ok(());
        }

        for _ in 0..9 {
            self.drive_scl_low();
            self.half_delay();
            self.release_scl()?;
            self.half_delay();
            if self.sda_high() {
                break;
            }
        }

        self.stop()?;
        if self.sda_high() && self.scl_high() {
            Ok(())
        } else {
            Err(Error::BusBusy)
        }
    }

    pub fn release(self) -> (SDA, SCL, D) {
        (self.sda, self.scl, self.delay)
    }

    fn half_delay(&mut self) {
        self.delay.delay_ns(self.half_period_ns);
    }

    fn release_sda(&mut self) {
        let _ = self.sda.set_high();
    }

    fn drive_sda_low(&mut self) {
        let _ = self.sda.set_low();
    }

    fn release_scl(&mut self) -> Result<(), Error> {
        let _ = self.scl.set_high();

        for _ in 0..self.stretch_timeout_us {
            if self.scl_high() {
                return Ok(());
            }
            self.delay.delay_us(1);
        }

        Err(Error::ClockStretchTimeout)
    }

    fn drive_scl_low(&mut self) {
        let _ = self.scl.set_low();
    }

    fn sda_high(&mut self) -> bool {
        self.sda.is_high().unwrap_or(false)
    }

    fn scl_high(&mut self) -> bool {
        self.scl.is_high().unwrap_or(false)
    }

    fn start(&mut self) -> Result<(), Error> {
        self.release_sda();
        self.release_scl()?;
        self.half_delay();

        if !self.sda_high() {
            return Err(Error::BusBusy);
        }

        self.drive_sda_low();
        self.half_delay();
        self.drive_scl_low();
        Ok(())
    }

    fn repeated_start(&mut self) -> Result<(), Error> {
        self.release_sda();
        self.half_delay();
        self.release_scl()?;
        self.half_delay();
        self.drive_sda_low();
        self.half_delay();
        self.drive_scl_low();
        Ok(())
    }

    fn stop(&mut self) -> Result<(), Error> {
        self.drive_sda_low();
        self.half_delay();
        self.release_scl()?;
        self.half_delay();
        self.release_sda();
        self.half_delay();
        Ok(())
    }

    fn write_bit(&mut self, high: bool) -> Result<(), Error> {
        if high {
            self.release_sda();
        } else {
            self.drive_sda_low();
        }

        self.half_delay();
        self.release_scl()?;
        self.half_delay();
        self.drive_scl_low();
        Ok(())
    }

    fn read_bit(&mut self) -> Result<bool, Error> {
        self.release_sda();
        self.half_delay();
        self.release_scl()?;
        let high = self.sda_high();
        self.half_delay();
        self.drive_scl_low();
        Ok(high)
    }

    fn write_byte(&mut self, mut byte: u8) -> Result<bool, Error> {
        for _ in 0..8 {
            self.write_bit((byte & 0x80) != 0)?;
            byte <<= 1;
        }

        Ok(!self.read_bit()?)
    }

    fn read_byte(&mut self, acknowledge: bool) -> Result<u8, Error> {
        let mut byte = 0_u8;

        for _ in 0..8 {
            byte <<= 1;
            if self.read_bit()? {
                byte |= 1;
            }
        }

        // I2C ACK is active low; NACK releases SDA high.
        self.write_bit(!acknowledge)?;
        self.release_sda();
        Ok(byte)
    }

    fn address(&mut self, address: SevenBitAddress, read: bool) -> Result<(), Error> {
        let byte = (address << 1) | u8::from(read);
        if self.write_byte(byte)? {
            Ok(())
        } else {
            Err(Error::NoAcknowledge(NoAcknowledgeSource::Address))
        }
    }

    fn write_data(&mut self, bytes: &[u8]) -> Result<(), Error> {
        for &byte in bytes {
            if !self.write_byte(byte)? {
                return Err(Error::NoAcknowledge(NoAcknowledgeSource::Data));
            }
        }
        Ok(())
    }

    fn has_following_read_data(operations: &[Operation<'_>], index: usize) -> bool {
        for operation in &operations[index + 1..] {
            match operation {
                Operation::Read(bytes) if !bytes.is_empty() => return true,
                Operation::Read(_) => continue,
                Operation::Write(_) => return false,
            }
        }
        false
    }
}

impl<SDA, SCL, D> ErrorType for SoftwareI2c<SDA, SCL, D>
where
    SDA: InputPin<Error = Infallible> + OutputPin<Error = Infallible>,
    SCL: InputPin<Error = Infallible> + OutputPin<Error = Infallible>,
    D: DelayNs,
{
    type Error = Error;
}

impl<SDA, SCL, D> I2c<SevenBitAddress> for SoftwareI2c<SDA, SCL, D>
where
    SDA: InputPin<Error = Infallible> + OutputPin<Error = Infallible>,
    SCL: InputPin<Error = Infallible> + OutputPin<Error = Infallible>,
    D: DelayNs,
{
    fn transaction(
        &mut self,
        address: SevenBitAddress,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        if operations.is_empty() {
            return Ok(());
        }

        let result = (|| {
            self.start()?;

            let mut current_read: Option<bool> = None;
            for index in 0..operations.len() {
                let read = matches!(&operations[index], Operation::Read(_));
                let follow_on_read = if read {
                    Self::has_following_read_data(operations, index)
                } else {
                    false
                };

                match current_read {
                    None => self.address(address, read)?,
                    Some(previous) if previous != read => {
                        self.repeated_start()?;
                        self.address(address, read)?;
                    }
                    Some(_) => {}
                }
                current_read = Some(read);

                match &mut operations[index] {
                    Operation::Write(bytes) => self.write_data(bytes)?,
                    Operation::Read(bytes) => {
                        let length = bytes.len();
                        for (byte_index, slot) in bytes.iter_mut().enumerate() {
                            let acknowledge = byte_index + 1 < length || follow_on_read;
                            *slot = self.read_byte(acknowledge)?;
                        }
                    }
                }
            }

            self.stop()
        })();

        if result.is_err() {
            let _ = self.stop();
        }

        result
    }
}

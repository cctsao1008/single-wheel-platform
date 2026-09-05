#![no_std]

use embedded_hal::i2c::I2c;

pub const DEFAULT_ADDRESS: u8 = 0x68;

const REG_SMPLRT_DIV: u8 = 0x19;
const REG_CONFIG: u8 = 0x1a;
const REG_GYRO_CONFIG: u8 = 0x1b;
const REG_ACCEL_CONFIG: u8 = 0x1c;
const REG_INT_ENABLE: u8 = 0x38;
const REG_ACCEL_XOUT_H: u8 = 0x3b;
const REG_PWR_MGMT_1: u8 = 0x6b;
const REG_WHO_AM_I: u8 = 0x75;

const WHO_AM_I_VALUE: u8 = 0x68;
const CLOCK_PLL_X_GYRO: u8 = 0x01;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum GyroRange {
    Dps250 = 0,
    Dps500 = 1,
    Dps1000 = 2,
    Dps2000 = 3,
}

impl GyroRange {
    pub const fn lsb_per_dps(self) -> f32 {
        match self {
            Self::Dps250 => 131.0,
            Self::Dps500 => 65.5,
            Self::Dps1000 => 32.8,
            Self::Dps2000 => 16.4,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum AccelRange {
    G2 = 0,
    G4 = 1,
    G8 = 2,
    G16 = 3,
}

impl AccelRange {
    pub const fn lsb_per_g(self) -> f32 {
        match self {
            Self::G2 => 16_384.0,
            Self::G4 => 8_192.0,
            Self::G8 => 4_096.0,
            Self::G16 => 2_048.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Dlpf {
    Config0 = 0,
    Config1 = 1,
    Config2 = 2,
    Config3 = 3,
    Config4 = 4,
    Config5 = 5,
    Config6 = 6,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Config {
    pub gyro_range: GyroRange,
    pub accel_range: AccelRange,
    pub dlpf: Dlpf,
    pub sample_rate_hz: u16,
    pub data_ready_interrupt: bool,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RawSample {
    pub accel: [i16; 3],
    pub temperature: i16,
    pub gyro: [i16; 3],
}

#[derive(Debug, PartialEq, Eq)]
pub enum Error<E> {
    Bus(E),
    InvalidConfig,
    UnexpectedIdentity(u8),
}

pub struct Mpu6050<I2C> {
    i2c: I2C,
    address: u8,
    config: Option<Config>,
}

impl<I2C> Mpu6050<I2C>
where
    I2C: I2c,
{
    pub const fn new(i2c: I2C, address: u8) -> Self {
        Self {
            i2c,
            address,
            config: None,
        }
    }

    pub fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
        let identity = self.read_u8(REG_WHO_AM_I)?;
        if identity == WHO_AM_I_VALUE {
            Ok(())
        } else {
            Err(Error::UnexpectedIdentity(identity))
        }
    }

    /// Wakes the device and programs the explicit operating configuration.
    ///
    /// This function does not introduce a hidden startup delay. The application
    /// owns sequencing and decides when the first sample is considered usable.
    pub fn configure(&mut self, config: Config) -> Result<(), Error<I2C::Error>> {
        let base_rate_hz: u32 = if config.dlpf == Dlpf::Config0 {
            8_000
        } else {
            1_000
        };
        let requested_rate = u32::from(config.sample_rate_hz);

        if requested_rate == 0
            || requested_rate > base_rate_hz
            || base_rate_hz % requested_rate != 0
        {
            return Err(Error::InvalidConfig);
        }

        let divider_plus_one = base_rate_hz / requested_rate;
        if !(1..=256).contains(&divider_plus_one) {
            return Err(Error::InvalidConfig);
        }

        self.write_u8(REG_PWR_MGMT_1, CLOCK_PLL_X_GYRO)?;
        self.write_u8(REG_CONFIG, config.dlpf as u8)?;
        self.write_u8(REG_GYRO_CONFIG, (config.gyro_range as u8) << 3)?;
        self.write_u8(REG_ACCEL_CONFIG, (config.accel_range as u8) << 3)?;
        self.write_u8(REG_SMPLRT_DIV, (divider_plus_one - 1) as u8)?;
        self.write_u8(
            REG_INT_ENABLE,
            if config.data_ready_interrupt {
                0x01
            } else {
                0x00
            },
        )?;

        self.config = Some(config);
        Ok(())
    }

    pub fn read_raw(&mut self) -> Result<RawSample, Error<I2C::Error>> {
        let mut data = [0_u8; 14];
        self.i2c
            .write_read(self.address, &[REG_ACCEL_XOUT_H], &mut data)
            .map_err(Error::Bus)?;

        Ok(RawSample {
            accel: [
                be_i16(data[0], data[1]),
                be_i16(data[2], data[3]),
                be_i16(data[4], data[5]),
            ],
            temperature: be_i16(data[6], data[7]),
            gyro: [
                be_i16(data[8], data[9]),
                be_i16(data[10], data[11]),
                be_i16(data[12], data[13]),
            ],
        })
    }

    pub const fn config(&self) -> Option<Config> {
        self.config
    }

    pub fn release(self) -> I2C {
        self.i2c
    }

    fn read_u8(&mut self, register: u8) -> Result<u8, Error<I2C::Error>> {
        let mut value = [0_u8; 1];
        self.i2c
            .write_read(self.address, &[register], &mut value)
            .map_err(Error::Bus)?;
        Ok(value[0])
    }

    fn write_u8(&mut self, register: u8, value: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, &[register, value])
            .map_err(Error::Bus)
    }
}

const fn be_i16(msb: u8, lsb: u8) -> i16 {
    i16::from_be_bytes([msb, lsb])
}

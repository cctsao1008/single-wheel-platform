#![no_std]

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Port {
    A,
    B,
    C,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pin {
    pub port: Port,
    pub index: u8,
}

impl Pin {
    pub const fn new(port: Port, index: u8) -> Self {
        Self { port, index }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TimerChannel {
    Tim2Ch1,
    Tim2Ch2,
    Tim3Ch1,
    Tim3Ch3,
    Tim3Ch4,
    Tim4Ch1,
    Tim4Ch2,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SerialWiring {
    /// MCU transmit pin.
    pub tx_pin: Pin,
    /// MCU receive pin.
    pub rx_pin: Pin,
}

/// Physical brushless connector identity from the schematic.
///
/// This deliberately does not encode platform meaning such as reaction wheel or
/// drive wheel. Connector identity and actuator role are separate facts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MotorChannel {
    Bldc1,
    Bldc2,
    Bldc3,
}

/// Physical encoder connector identity from the schematic.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EncoderChannel {
    Encoder1,
    Encoder2,
    Encoder3,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MotorWiring {
    pub channel: MotorChannel,
    pub pwm_pin: Pin,
    pub pwm_timer: TimerChannel,
    pub direction_pin: Pin,
    pub brake_pin: Option<Pin>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EncoderWiring {
    pub channel: EncoderChannel,
    pub channel_a_pin: Option<Pin>,
    pub channel_a_timer: Option<TimerChannel>,
    pub channel_b_pin: Option<Pin>,
    pub channel_b_timer: Option<TimerChannel>,
}

pub const MCU: &str = "STM32F103C8T6";
pub const MPU6050_ADDRESS: u8 = 0x68;

/// The schematic label `MPU_INT` is connected to MPU6050 FSYNC, not INT.
pub const MPU_FSYNC: Pin = Pin::new(Port::C, 13);
pub const MPU_HAS_DATA_READY_IRQ: bool = false;

/// Reference-board software-I2C wiring. These pins are intentionally named by
/// the schematic net rather than by STM32 I2C-remap function.
pub const MPU_SDA: Pin = Pin::new(Port::B, 8);
pub const MPU_SCL: Pin = Pin::new(Port::B, 9);

/// MCU USART1 wiring. The MCU nets and CH340 nets terminate on separate P2 pins;
/// the schematic does not hard-wire the USB-UART bridge to this serial port.
pub const MAIN_UART: SerialWiring = SerialWiring {
    tx_pin: Pin::new(Port::A, 9),
    rx_pin: Pin::new(Port::A, 10),
};
pub const MAIN_UART_TO_CH340_HARDWIRED: bool = false;

/// MCU USART2 wiring to the on-board ECB02S2 Bluetooth serial module.
pub const BLUETOOTH_UART: SerialWiring = SerialWiring {
    tx_pin: Pin::new(Port::A, 2),
    rx_pin: Pin::new(Port::A, 3),
};
pub const BLUETOOTH_AT_ENABLE: Pin = Pin::new(Port::C, 15);
pub const BLUETOOTH_ROLE: Pin = Pin::new(Port::C, 14);
/// The reviewed schematic ties the ECB02 sleep input low, keeping the module awake.
pub const BLUETOOTH_SLEEP_HARDWIRED_LOW: bool = true;

/// OLED two-wire interface as routed by the reference board.
pub const OLED_SDA: Pin = Pin::new(Port::B, 4);
pub const OLED_SCL: Pin = Pin::new(Port::B, 5);

/// Configuration/authority jumper inputs exposed by the board as EN_X and EN_Y.
/// Their electrical pin mapping is known; the platform-semantic actuator association
/// is intentionally not encoded here because legacy naming and product labels differ.
pub const EN_X: Pin = Pin::new(Port::A, 15);
pub const EN_Y: Pin = Pin::new(Port::B, 3);

pub const SWDIO: Pin = Pin::new(Port::A, 13);
pub const SWCLK: Pin = Pin::new(Port::A, 14);

/// Schematic connector BLDC_1 (captioned side/lateral brushless interface).
pub const BLDC_1: MotorWiring = MotorWiring {
    channel: MotorChannel::Bldc1,
    pwm_pin: Pin::new(Port::B, 1),
    pwm_timer: TimerChannel::Tim3Ch4,
    direction_pin: Pin::new(Port::B, 11),
    brake_pin: None,
};

/// Schematic connector BLDC_2 (captioned front/back brushless interface).
pub const BLDC_2: MotorWiring = MotorWiring {
    channel: MotorChannel::Bldc2,
    pwm_pin: Pin::new(Port::A, 6),
    pwm_timer: TimerChannel::Tim3Ch1,
    direction_pin: Pin::new(Port::A, 4),
    brake_pin: None,
};

/// Schematic connector BLDC_3 (captioned spin brushless interface).
pub const BLDC_3: MotorWiring = MotorWiring {
    channel: MotorChannel::Bldc3,
    pwm_pin: Pin::new(Port::B, 0),
    pwm_timer: TimerChannel::Tim3Ch3,
    direction_pin: Pin::new(Port::B, 10),
    brake_pin: Some(Pin::new(Port::A, 7)),
};

pub const ENCODER_1: EncoderWiring = EncoderWiring {
    channel: EncoderChannel::Encoder1,
    channel_a_pin: Some(Pin::new(Port::A, 1)),
    channel_a_timer: Some(TimerChannel::Tim2Ch2),
    channel_b_pin: Some(Pin::new(Port::A, 0)),
    channel_b_timer: Some(TimerChannel::Tim2Ch1),
};

pub const ENCODER_2: EncoderWiring = EncoderWiring {
    channel: EncoderChannel::Encoder2,
    channel_a_pin: Some(Pin::new(Port::B, 7)),
    channel_a_timer: Some(TimerChannel::Tim4Ch2),
    channel_b_pin: Some(Pin::new(Port::B, 6)),
    channel_b_timer: Some(TimerChannel::Tim4Ch1),
};

/// Encoder-3 signals exist at the BLDC3 connector, but the reviewed schematic
/// does not show a route back to MCU encoder inputs.
pub const ENCODER_3: EncoderWiring = EncoderWiring {
    channel: EncoderChannel::Encoder3,
    channel_a_pin: None,
    channel_a_timer: None,
    channel_b_pin: None,
    channel_b_timer: None,
};

pub const BATTERY_ADC: Pin = Pin::new(Port::A, 5);

/// The schematic does not label the external crystal frequency, so the board
/// crate deliberately does not publish an HSE frequency constant.
pub const HSE_FREQUENCY_CONFIRMED: bool = false;

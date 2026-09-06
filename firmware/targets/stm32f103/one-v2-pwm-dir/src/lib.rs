#![no_std]

use core::convert::Infallible;

use stm32f1xx_hal::{
    gpio::{Output, PushPull, gpioa::PA4, gpiob::PB11},
    pac,
    timer::{C1, C4, PwmChannel},
};
use swp_actuation_interface::DriverIo;
use swp_one_v2_pwm_dir_driver::{ElectricalActuation, OneV2PwmDirDriver};

/// ONE V2.0 drive-wheel control-board resources:
///
/// - BLDC_2 PWM: PA6 / TIM3_CH1
/// - BLDC_2 DIR: PA4
pub type DrivePwm = PwmChannel<pac::TIM3, C1>;
pub type DriveDirection = PA4<Output<PushPull>>;

/// ONE V2.0 reaction-wheel control-board resources:
///
/// - BLDC_1 PWM: PB1 / TIM3_CH4
/// - BLDC_1 DIR: PB11
pub type ReactionPwm = PwmChannel<pac::TIM3, C4>;
pub type ReactionDirection = PB11<Output<PushPull>>;

/// Concrete STM32F103 backend for the ONE V2 PWM/DIR driver interface.
///
/// This type owns only target-specific peripheral mutation. Driver polarity and
/// zero-effort semantics remain in `swp-one-v2-pwm-dir-driver`, while authority
/// remains upstream in Supervisor.
pub struct MotorElectricalOutputs {
    drive_pwm: DrivePwm,
    reaction_pwm: ReactionPwm,
    drive_direction: DriveDirection,
    reaction_direction: ReactionDirection,
}

impl MotorElectricalOutputs {
    /// Construct the target backend from the exact ONE V2 TIM3 channels and DIR pins.
    ///
    /// The caller still owns TIM3/GPIO configuration. Construction does not enable
    /// PWM channels and therefore does not itself start physical motor output.
    pub fn new(
        drive_pwm: DrivePwm,
        reaction_pwm: ReactionPwm,
        drive_direction: DriveDirection,
        reaction_direction: ReactionDirection,
    ) -> Self {
        let mut outputs = Self {
            drive_pwm,
            reaction_pwm,
            drive_direction,
            reaction_direction,
        };
        outputs.hold_zero_effort_encoding();
        outputs
    }

    /// Set the current ONE V2 zero-effort electrical encoding without changing
    /// channel enable state: DIR low and PWM line continuously high.
    pub fn hold_zero_effort_encoding(&mut self) {
        self.drive_direction.set_low();
        self.reaction_direction.set_low();

        let drive_max = self.drive_pwm.get_max_duty();
        let reaction_max = self.reaction_pwm.get_max_duty();
        self.drive_pwm.set_duty(drive_max);
        self.reaction_pwm.set_duty(reaction_max);
    }

    /// Explicitly enable the two installed TIM3 PWM channels.
    ///
    /// This remains a physical commissioning action; no observation or live-shadow
    /// target constructs this backend.
    pub fn enable_channels(&mut self) {
        self.drive_pwm.enable();
        self.reaction_pwm.enable();
    }

    /// Disable both TIM3 output channels.
    ///
    /// The external driver/motor state while a channel is disabled is a measured
    /// hardware property. This method is not a universal motor safe-state claim.
    pub fn disable_channels(&mut self) {
        self.drive_pwm.disable();
        self.reaction_pwm.disable();
    }

    /// Wrap this MCU backend in the portable ONE V2 driver adapter.
    ///
    /// The returned type implements `ActuationSink` and therefore accepts only
    /// `AuthorizedActuation` at the physical-actuation boundary.
    pub fn into_actuation_sink(self) -> OneV2ActuationSink {
        OneV2PwmDirDriver::new(self)
    }

    fn apply_electrical(&mut self, electrical: ElectricalActuation) {
        if electrical.drive.direction_high {
            self.drive_direction.set_high();
        } else {
            self.drive_direction.set_low();
        }
        if electrical.reaction.direction_high {
            self.reaction_direction.set_high();
        } else {
            self.reaction_direction.set_low();
        }

        let drive_duty = duty_from_line_high_fraction(
            self.drive_pwm.get_max_duty(),
            electrical.drive.pwm_line_high_fraction,
        );
        let reaction_duty = duty_from_line_high_fraction(
            self.reaction_pwm.get_max_duty(),
            electrical.reaction.pwm_line_high_fraction,
        );
        self.drive_pwm.set_duty(drive_duty);
        self.reaction_pwm.set_duty(reaction_duty);
    }
}

impl DriverIo<ElectricalActuation> for MotorElectricalOutputs {
    type Error = Infallible;

    fn write_frame(&mut self, frame: ElectricalActuation) -> Result<(), Self::Error> {
        self.apply_electrical(frame);
        Ok(())
    }
}

pub type OneV2ActuationSink = OneV2PwmDirDriver<MotorElectricalOutputs>;

fn duty_from_line_high_fraction(max_duty: u16, fraction: f32) -> u16 {
    let bounded = fraction.clamp(0.0, 1.0);
    (f32::from(max_duty) * bounded) as u16
}

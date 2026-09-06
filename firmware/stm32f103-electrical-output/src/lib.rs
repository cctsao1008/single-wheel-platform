#![no_std]

use stm32f1xx_hal::{
    gpio::{
        Output, PushPull,
        gpioa::PA4,
        gpiob::PB11,
    },
    pac,
    timer::{C1, C4, PwmChannel},
};
use swp_one_v2_electrical_output::{ElectricalActuation, encode_authorized};
use swp_runtime_state::AuthorizedActuation;

/// ONE V2.0 drive-wheel electrical resources:
///
/// - BLDC_2 PWM: PA6 / TIM3_CH1
/// - BLDC_2 DIR: PA4
pub type DrivePwm = PwmChannel<pac::TIM3, C1>;
pub type DriveDirection = PA4<Output<PushPull>>;

/// ONE V2.0 reaction-wheel electrical resources:
///
/// - BLDC_1 PWM: PB1 / TIM3_CH4
/// - BLDC_1 DIR: PB11
pub type ReactionPwm = PwmChannel<pac::TIM3, C4>;
pub type ReactionDirection = PB11<Output<PushPull>>;

/// Concrete STM32F103 owner of the two installed motor electrical outputs.
///
/// Construction requires the exact TIM3 channels and direction pins. A caller
/// cannot substitute unrelated timer/GPIO resources, and physical commands can
/// only be applied with an `AuthorizedActuation` token.
///
/// This type deliberately does not configure TIM3 or GPIO pin modes. The owning
/// firmware must create PA6/TIM3_CH1 and PB1/TIM3_CH4 using the no-remap TIM3
/// mapping and configure PA4/PB11 as push-pull outputs. Separating configuration
/// from mutation keeps board bring-up and runtime authority explicit.
pub struct MotorElectricalOutputs {
    drive_pwm: DrivePwm,
    reaction_pwm: ReactionPwm,
    drive_direction: DriveDirection,
    reaction_direction: ReactionDirection,
}

impl MotorElectricalOutputs {
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
        outputs.hold_vendor_idle_encoding();
        outputs
    }

    /// Set the vendor V2.0 zero-effort line encoding without changing channel
    /// enable state: DIR low and PWM line continuously high.
    pub fn hold_vendor_idle_encoding(&mut self) {
        self.drive_direction.set_low();
        self.reaction_direction.set_low();

        let drive_max = self.drive_pwm.get_max_duty();
        let reaction_max = self.reaction_pwm.get_max_duty();
        self.drive_pwm.set_duty(drive_max);
        self.reaction_pwm.set_duty(reaction_max);
    }

    /// Explicitly enable the two installed TIM3 PWM channels.
    ///
    /// Calling this is a physical commissioning action. The current observation
    /// and live-shadow firmware do not construct this owner and therefore cannot
    /// enable these channels.
    pub fn enable_channels(&mut self) {
        self.drive_pwm.enable();
        self.reaction_pwm.enable();
    }

    /// Disable both TIM3 output channels. The final external motor-driver state
    /// while channels are disabled remains a hardware property and must be
    /// verified during commissioning; this method is not advertised as a generic
    /// electrical safe-state primitive.
    pub fn disable_channels(&mut self) {
        self.drive_pwm.disable();
        self.reaction_pwm.disable();
    }

    /// Apply one runtime-authorized command to TIM3 PWM and DIR resources.
    ///
    /// The semantic-to-electrical conversion is performed by
    /// `swp-one-v2-electrical-output`; this sink owns only concrete MCU mutation.
    pub fn apply(&mut self, authorized: AuthorizedActuation) {
        let electrical = encode_authorized(authorized);
        self.apply_electrical(electrical);
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

fn duty_from_line_high_fraction(max_duty: u16, fraction: f32) -> u16 {
    let bounded = fraction.clamp(0.0, 1.0);
    (f32::from(max_duty) * bounded) as u16
}

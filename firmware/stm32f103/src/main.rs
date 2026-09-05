#![no_std]
#![no_main]
#![deny(unsafe_code)]

use panic_halt as _;

#[rtic::app(device = stm32f1xx_hal::pac)]
mod app {
    use stm32f1xx_hal::{
        gpio::{OpenDrain, Output, PinState, gpiob::PB8, gpiob::PB9},
        prelude::*,
        rcc,
        timer::SysDelay,
    };
    use swp_board_one_v2 as board;
    use swp_mpu6050::Mpu6050;
    use swp_software_i2c::SoftwareI2c;

    type ImuBus = SoftwareI2c<PB8<Output<OpenDrain>>, PB9<Output<OpenDrain>>, SysDelay>;
    type Imu = Mpu6050<ImuBus>;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        _imu: Imu,
        imu_present: bool,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        // Use the known internal 8 MHz oscillator for first-board bring-up.
        // The external crystal frequency is not yet a confirmed schematic fact.
        let mut flash = ctx.device.FLASH.constrain();
        let mut rcc = ctx.device.RCC.freeze(
            rcc::Config::hsi()
                .sysclk(8.MHz())
                .pclk1(8.MHz())
                .pclk2(8.MHz()),
            &mut flash.acr,
        );

        // Schematic wiring is PB8=SDA and PB9=SCL. That is intentionally not
        // the STM32F103 I2C1 remap pin order, so this path uses software I2C.
        let mut gpiob = ctx.device.GPIOB.split(&mut rcc);
        let sda = gpiob
            .pb8
            .into_open_drain_output_with_state(&mut gpiob.crh, PinState::High);
        let scl = gpiob
            .pb9
            .into_open_drain_output_with_state(&mut gpiob.crh, PinState::High);

        let delay = ctx.core.SYST.delay(&rcc.clocks);
        let mut bus = SoftwareI2c::new(sda, scl, delay, 5_000, 100);
        let bus_ready = bus.recover_bus().is_ok();

        let mut imu = Mpu6050::new(bus, board::MPU6050_ADDRESS);
        let imu_present = bus_ready && imu.probe().is_ok();

        // No actuator peripheral is configured in this milestone. A successful
        // build boots safely, owns the physical MPU bus, recovers it if needed,
        // and performs only WHO_AM_I probing at address 0x68.
        (
            Shared {},
            Local {
                _imu: imu,
                imu_present,
            },
        )
    }

    #[idle(local = [imu_present])]
    fn idle(ctx: idle::Context) -> ! {
        // Keep the probe result debugger-visible while leaving the hardware in
        // a passive state until telemetry is added.
        let _ = *ctx.local.imu_present;

        loop {
            cortex_m::asm::wfi();
        }
    }
}

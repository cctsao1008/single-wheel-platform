#![no_std]
#![no_main]
#![deny(unsafe_code)]

use panic_halt as _;

#[rtic::app(device = stm32f1xx_hal::pac)]
mod app {
    use swp_board_one_v2 as board;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(_: init::Context) -> (Shared, Local) {
        // Deliberately keep reset-clock behavior and all actuators inactive at
        // this migration point. The reviewed schematic does not identify the
        // external crystal frequency, so 72 MHz is not hard-coded as a board
        // fact. Peripheral bring-up is added through typed HAL ownership from
        // here, not through a compatibility C board layer.
        let _ = board::MCU;
        let _ = board::MPU6050_ADDRESS;

        (Shared {}, Local {})
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }
}

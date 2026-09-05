#![no_std]
#![no_main]
#![deny(unsafe_code)]

use panic_halt as _;

#[rtic::app(device = stm32f1xx_hal::pac)]
mod app {
    use cortex_m::peripheral::DWT;
    use heapless::spsc::{Consumer, Producer, Queue};
    use stm32f1xx_hal::{
        gpio::{OpenDrain, Output, PinState, gpiob::PB8, gpiob::PB9},
        pac,
        prelude::*,
        rcc,
        serial::{Config as SerialConfig, Tx},
        timer::{CounterMs, Event, SysDelay},
    };
    use swp_board_one_v2 as board;
    use swp_mpu6050::{AccelRange, Config as MpuConfig, Dlpf, GyroRange, Mpu6050, RawSample};
    use swp_software_i2c::SoftwareI2c;
    use swp_telemetry_protocol::{RAW_IMU_FRAME_LEN, RawImuFrame, status};

    const CPU_HZ: u64 = 8_000_000;
    const CYCLES_PER_US: u64 = CPU_HZ / 1_000_000;
    const SAMPLE_PERIOD_MS: u32 = 10;
    const TELEMETRY_BAUD: u32 = 115_200;
    const TELEMETRY_QUEUE_STORAGE: usize = 8;

    type ImuBus = SoftwareI2c<PB8<Output<OpenDrain>>, PB9<Output<OpenDrain>>, SysDelay>;
    type Imu = Mpu6050<ImuBus>;
    type SampleTimer = CounterMs<pac::TIM1>;
    type TelemetryTx = Tx<pac::USART1>;
    type TelemetryProducer = Producer<'static, [u8; RAW_IMU_FRAME_LEN], TELEMETRY_QUEUE_STORAGE>;
    type TelemetryConsumer = Consumer<'static, [u8; RAW_IMU_FRAME_LEN], TELEMETRY_QUEUE_STORAGE>;

    struct TelemetryPump {
        tx: TelemetryTx,
        consumer: TelemetryConsumer,
        current_frame: Option<[u8; RAW_IMU_FRAME_LEN]>,
        frame_index: usize,
    }

    impl TelemetryPump {
        fn new(tx: TelemetryTx, consumer: TelemetryConsumer) -> Self {
            Self {
                tx,
                consumer,
                current_frame: None,
                frame_index: 0,
            }
        }

        fn on_interrupt(&mut self) {
            if self.current_frame.is_none() {
                self.current_frame = self.consumer.dequeue();
                self.frame_index = 0;

                if self.current_frame.is_none() {
                    self.tx.unlisten();
                    return;
                }
            }

            if !self.tx.is_tx_empty() {
                self.tx.listen();
                return;
            }

            let Some(frame) = self.current_frame.as_ref() else {
                self.tx.unlisten();
                return;
            };

            if self.tx.write_u8(frame[self.frame_index]).is_ok() {
                self.frame_index += 1;
                if self.frame_index == RAW_IMU_FRAME_LEN {
                    self.current_frame = None;
                    self.frame_index = 0;
                }
            }

            self.tx.listen();
        }
    }

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        imu: Imu,
        sample_timer: SampleTimer,
        telemetry_producer: TelemetryProducer,
        telemetry_pump: TelemetryPump,
        bus_ready: bool,
        imu_present: bool,
        imu_configured: bool,
        sequence: u32,
        dropped_frames: u16,
        last_cycle: u32,
        cycle_epoch: u64,
    }

    #[init(local = [telemetry_queue: Queue<[u8; RAW_IMU_FRAME_LEN], 8> = Queue::new()])]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let mut dcb = ctx.core.DCB;
        let mut dwt = ctx.core.DWT;
        dcb.enable_trace();
        dwt.enable_cycle_counter();

        // Keep the first executable clock profile on the confirmed internal
        // oscillator. The board drawing does not specify the HSE frequency.
        let mut flash = ctx.device.FLASH.constrain();
        let mut rcc = ctx.device.RCC.freeze(
            rcc::Config::hsi()
                .sysclk(8.MHz())
                .pclk1(8.MHz())
                .pclk2(8.MHz()),
            &mut flash.acr,
        );

        let mut gpioa = ctx.device.GPIOA.split(&mut rcc);
        let mut gpiob = ctx.device.GPIOB.split(&mut rcc);

        // Schematic wiring is PB8=SDA and PB9=SCL. That is intentionally not
        // the STM32F103 I2C1 remap pin order, so this path uses software I2C.
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
        let imu_configured = imu_present
            && imu
                .configure(MpuConfig {
                    gyro_range: GyroRange::Dps1000,
                    accel_range: AccelRange::G4,
                    dlpf: Dlpf::Config3,
                    sample_rate_hz: 100,
                    data_ready_interrupt: false,
                })
                .is_ok();

        // USART1 TX is the schematic net TX on PA9. The onboard CH340 nets are
        // separate at P2; using CH340 as the host bridge requires external
        // cross-connection and is not assumed by firmware.
        let uart_tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        let telemetry_tx = ctx.device.USART1.tx(
            uart_tx,
            SerialConfig::default().baudrate(TELEMETRY_BAUD.bps()),
            &mut rcc,
        );

        let mut sample_timer = ctx.device.TIM1.counter_ms(&mut rcc);
        sample_timer.start(SAMPLE_PERIOD_MS.millis()).unwrap();
        sample_timer.listen(Event::Update);

        let (telemetry_producer, telemetry_consumer) = ctx.local.telemetry_queue.split();
        let telemetry_pump = TelemetryPump::new(telemetry_tx, telemetry_consumer);
        let last_cycle = DWT::cycle_count();

        // No motor PWM, direction, or brake output is configured here. The
        // first scheduled runtime path is sensing plus observable telemetry.
        (
            Shared {},
            Local {
                imu,
                sample_timer,
                telemetry_producer,
                telemetry_pump,
                bus_ready,
                imu_present,
                imu_configured,
                sequence: 0,
                dropped_frames: 0,
                last_cycle,
                cycle_epoch: 0,
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(
        binds = TIM1_UP,
        priority = 2,
        local = [
            imu,
            sample_timer,
            telemetry_producer,
            bus_ready,
            imu_present,
            imu_configured,
            sequence,
            dropped_frames,
            last_cycle,
            cycle_epoch
        ]
    )]
    fn sample_tick(ctx: sample_tick::Context) {
        let timestamp_us = capture_timestamp_us(ctx.local.last_cycle, ctx.local.cycle_epoch);

        let mut status_bits = 0_u16;
        if *ctx.local.bus_ready {
            status_bits |= status::BUS_READY;
        }
        if *ctx.local.imu_present {
            status_bits |= status::IMU_PRESENT;
        }
        if *ctx.local.imu_configured {
            status_bits |= status::IMU_CONFIGURED;
        }

        let mut sample = RawSample::default();
        if *ctx.local.imu_configured {
            if let Ok(value) = ctx.local.imu.read_raw() {
                sample = value;
                status_bits |= status::SAMPLE_VALID;
            }
        }

        let frame = RawImuFrame {
            sequence: *ctx.local.sequence,
            timestamp_us,
            accel_raw: sample.accel,
            temperature_raw: sample.temperature,
            gyro_raw: sample.gyro,
            status: status_bits,
            dropped_frames: *ctx.local.dropped_frames,
        }
        .encode();

        if ctx.local.telemetry_producer.enqueue(frame).is_ok() {
            rtic::pend(pac::Interrupt::USART1);
        } else {
            *ctx.local.dropped_frames = ctx.local.dropped_frames.saturating_add(1);
        }

        *ctx.local.sequence = ctx.local.sequence.wrapping_add(1);
        ctx.local.sample_timer.clear_interrupt(Event::Update);
    }

    #[task(binds = USART1, priority = 1, local = [telemetry_pump])]
    fn telemetry_tx(ctx: telemetry_tx::Context) {
        ctx.local.telemetry_pump.on_interrupt();
    }

    fn capture_timestamp_us(last_cycle: &mut u32, cycle_epoch: &mut u64) -> u64 {
        let now = DWT::cycle_count();
        if now < *last_cycle {
            *cycle_epoch += 1_u64 << 32;
        }
        *last_cycle = now;
        (*cycle_epoch + u64::from(now)) / CYCLES_PER_US
    }
}

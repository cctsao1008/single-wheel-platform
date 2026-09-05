#![no_std]
#![no_main]
#![deny(unsafe_code)]

use panic_halt as _;

#[rtic::app(device = stm32f1xx_hal::pac)]
mod app {
    use cortex_m::peripheral::DWT;
    use heapless::spsc::{Consumer, Producer, Queue};
    use stm32f1xx_hal::{
        adc::Adc,
        gpio::{
            Analog, OpenDrain, Output, PinState,
            gpioa::PA5,
            gpiob::{PB8, PB9},
        },
        pac,
        prelude::*,
        rcc,
        serial::{Config as SerialConfig, Tx},
        timer::{
            CounterMs, Event, SysDelay, Timer,
            pwm_input::{Qei, QeiOptions},
        },
    };
    use swp_board_one_v2 as board;
    use swp_mpu6050::{AccelRange, Config as MpuConfig, Dlpf, GyroRange, Mpu6050, RawSample};
    use swp_plant_observation::{ObservationFlags, RawImuObservation, RawObservation};
    use swp_software_i2c::SoftwareI2c;
    use swp_telemetry_protocol::{SENSOR_SNAPSHOT_FRAME_LEN, SensorSnapshotFrame};

    const CPU_HZ: u64 = 8_000_000;
    const CYCLES_PER_US: u64 = CPU_HZ / 1_000_000;
    const SAMPLE_PERIOD_MS: u32 = 10;
    const TELEMETRY_BAUD: u32 = 115_200;
    const TELEMETRY_QUEUE_STORAGE: usize = 8;

    type ImuBus = SoftwareI2c<PB8<Output<OpenDrain>>, PB9<Output<OpenDrain>>, SysDelay>;
    type Imu = Mpu6050<ImuBus>;
    type Encoder1 = Qei<pac::TIM2>;
    type Encoder2 = Qei<pac::TIM4>;
    type BatteryAdc = Adc<pac::ADC1>;
    type BatteryAdcPin = PA5<Analog>;
    type SampleTimer = CounterMs<pac::TIM1>;
    type TelemetryTx = Tx<pac::USART1>;
    type TelemetryProducer =
        Producer<'static, [u8; SENSOR_SNAPSHOT_FRAME_LEN], TELEMETRY_QUEUE_STORAGE>;
    type TelemetryConsumer =
        Consumer<'static, [u8; SENSOR_SNAPSHOT_FRAME_LEN], TELEMETRY_QUEUE_STORAGE>;

    struct TelemetryPump {
        tx: TelemetryTx,
        consumer: TelemetryConsumer,
        current_frame: Option<[u8; SENSOR_SNAPSHOT_FRAME_LEN]>,
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
                if self.frame_index == SENSOR_SNAPSHOT_FRAME_LEN {
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
        encoder_1: Encoder1,
        encoder_2: Encoder2,
        battery_adc: BatteryAdc,
        battery_adc_pin: BatteryAdcPin,
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

    #[init(local = [telemetry_queue: Queue<[u8; SENSOR_SNAPSHOT_FRAME_LEN], 8> = Queue::new()])]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let mut dcb = ctx.core.DCB;
        let mut dwt = ctx.core.DWT;
        dcb.enable_trace();
        dwt.enable_cycle_counter();

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

        // HAL QEI tuples are ordered CH1, CH2. On this PCB those are the
        // schematic B, A nets respectively; raw count direction is therefore
        // reported without inventing a robot-positive sign convention.
        let encoder_1 = Timer::new(ctx.device.TIM2, &mut rcc)
            .qei((gpioa.pa0, gpioa.pa1), QeiOptions::default());
        let encoder_2 = Timer::new(ctx.device.TIM4, &mut rcc)
            .qei((gpiob.pb6, gpiob.pb7), QeiOptions::default());

        // PA5 is the schematic divider node ADC. Divider resistor values are
        // not present in the reviewed drawing, so firmware exposes raw counts.
        let battery_adc_pin = gpioa.pa5.into_analog(&mut gpioa.crl);
        let battery_adc = Adc::new(ctx.device.ADC1, &mut rcc);

        // USART1 TX is PA9 / schematic net TX. The onboard CH340 pair is
        // separate on P2 and is not assumed to be cross-connected here.
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

        // TIM3 and all motor GPIO remain untouched. This runtime only observes
        // the plant through IMU, encoders, battery ADC, and telemetry.
        (
            Shared {},
            Local {
                imu,
                encoder_1,
                encoder_2,
                battery_adc,
                battery_adc_pin,
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
            encoder_1,
            encoder_2,
            battery_adc,
            battery_adc_pin,
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

        let mut validity = ObservationFlags::ENCODER_1_VALID | ObservationFlags::ENCODER_2_VALID;
        if *ctx.local.bus_ready {
            validity |= ObservationFlags::BUS_READY;
        }
        if *ctx.local.imu_present {
            validity |= ObservationFlags::IMU_PRESENT;
        }
        if *ctx.local.imu_configured {
            validity |= ObservationFlags::IMU_CONFIGURED;
        }

        let mut sample = RawSample::default();
        if *ctx.local.imu_configured {
            if let Ok(value) = ctx.local.imu.read_raw() {
                sample = value;
                validity |= ObservationFlags::IMU_SAMPLE_VALID;
            }
        }

        let encoder_counts = [ctx.local.encoder_1.count(), ctx.local.encoder_2.count()];
        let battery_adc_raw = match ctx.local.battery_adc.read(ctx.local.battery_adc_pin) {
            Ok(value) => {
                validity |= ObservationFlags::BATTERY_ADC_VALID;
                value
            }
            Err(_) => 0,
        };

        let observation = RawObservation {
            sample_index: *ctx.local.sequence,
            timestamp_us,
            imu: RawImuObservation {
                accel_raw: sample.accel,
                temperature_raw: sample.temperature,
                gyro_raw: sample.gyro,
            },
            encoder_counts,
            battery_adc_raw,
            validity,
        };

        let frame = SensorSnapshotFrame::from_observation(observation, *ctx.local.dropped_frames)
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

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
        dma::{Event as DmaEvent, R as DmaRead, Transfer},
        gpio::{
            Analog, Edge, ExtiPin, Floating, Input, OpenDrain, Output, PinState,
            gpioa::PA5,
            gpiob::{PB8, PB9},
            gpioc::PC13,
        },
        pac,
        prelude::*,
        rcc,
        serial::{Config as SerialConfig, TxDma2},
        timer::{
            CounterMs, Event as TimerEvent, SysDelay, Timer,
            pwm_input::{Qei, QeiOptions},
        },
    };
    use swp_board_one_v2 as board;
    use swp_mpu6050::{AccelRange, Config as MpuConfig, Dlpf, GyroRange, Mpu6050, RawSample};
    use swp_observation_record::{RAW_OBSERVATION_RECORD_LEN, RecordedObservation};
    use swp_plant_observation::{
        AcquisitionStatus, MeasurementQuality, RawBatteryObservation, RawEncoderObservation,
        RawImuObservation, RawObservation, TimestampEvidence,
    };
    use swp_runtime_state::{SensorTimingHealth, SensorTimingLimits, SensorTimingMonitor};
    use swp_software_i2c::SoftwareI2c;

    const CPU_HZ: u64 = 72_000_000;
    const CYCLES_PER_US: u64 = CPU_HZ / 1_000_000;

    const MPU_ACQUISITION_HZ: u16 = 500;
    const MPU_EXPECTED_PERIOD_US: u32 = 2_000;
    const MPU_LATE_AFTER_US: u32 = 3_000;
    const MPU_TIMEOUT_AFTER_US: u32 = 6_000;
    const HEALTH_PERIOD_MS: u32 = 1;

    const OBSERVATION_DECIMATION: u8 = 5;
    const I2C_HALF_PERIOD_NS: u32 = 1_250;

    const RECORD_BAUD: u32 = 115_200;
    const RECORD_QUEUE_STORAGE: usize = 8;

    type ImuBus = SoftwareI2c<PB8<Output<OpenDrain>>, PB9<Output<OpenDrain>>, SysDelay>;
    type Imu = Mpu6050<ImuBus>;
    type ImuInt = PC13<Input<Floating>>;
    type Encoder1 = Qei<pac::TIM2>;
    type Encoder2 = Qei<pac::TIM4>;
    type BatteryAdc = Adc<pac::ADC1>;
    type BatteryAdcPin = PA5<Analog>;
    type HealthTimer = CounterMs<pac::TIM1>;
    type RecordDma = TxDma2;
    type RecordTransfer =
        Transfer<DmaRead, &'static mut [u8; RAW_OBSERVATION_RECORD_LEN], RecordDma>;
    type RecordProducer = Producer<'static, [u8; RAW_OBSERVATION_RECORD_LEN], RECORD_QUEUE_STORAGE>;
    type RecordConsumer = Consumer<'static, [u8; RAW_OBSERVATION_RECORD_LEN], RECORD_QUEUE_STORAGE>;

    enum RecordDmaState {
        Idle {
            dma: RecordDma,
            buffer: &'static mut [u8; RAW_OBSERVATION_RECORD_LEN],
        },
        Active(RecordTransfer),
    }

    struct UartRecordDmaPump {
        consumer: RecordConsumer,
        state: Option<RecordDmaState>,
    }

    impl UartRecordDmaPump {
        fn new(
            mut dma: RecordDma,
            consumer: RecordConsumer,
            buffer: &'static mut [u8; RAW_OBSERVATION_RECORD_LEN],
        ) -> Self {
            dma.channel.listen(DmaEvent::TransferComplete);
            Self {
                consumer,
                state: Some(RecordDmaState::Idle { dma, buffer }),
            }
        }

        fn on_interrupt(&mut self) {
            let Some(state) = self.state.take() else {
                return;
            };

            let idle = match state {
                RecordDmaState::Active(transfer) => {
                    if !transfer.is_done() {
                        self.state = Some(RecordDmaState::Active(transfer));
                        return;
                    }
                    let (buffer, dma) = transfer.wait();
                    RecordDmaState::Idle { dma, buffer }
                }
                idle @ RecordDmaState::Idle { .. } => idle,
            };

            let RecordDmaState::Idle { dma, buffer } = idle else {
                unreachable!();
            };

            if let Some(record) = self.consumer.dequeue() {
                buffer.copy_from_slice(&record);
                self.state = Some(RecordDmaState::Active(dma.write(buffer)));
            } else {
                self.state = Some(RecordDmaState::Idle { dma, buffer });
            }
        }
    }

    #[shared]
    struct Shared {
        imu_timing_monitor: SensorTimingMonitor,
    }

    #[local]
    struct Local {
        imu: Imu,
        imu_int: ImuInt,
        encoder_1: Encoder1,
        encoder_2: Encoder2,
        battery_adc: BatteryAdc,
        battery_adc_pin: BatteryAdcPin,
        health_timer: HealthTimer,
        record_producer: RecordProducer,
        record_pump: UartRecordDmaPump,
        bus_ready: bool,
        imu_present: bool,
        imu_configured: bool,
        observation_divider: u8,
        sequence: u32,
        dropped_records: u16,
        imu_last_cycle: u32,
        imu_cycle_epoch: u64,
        health_last_cycle: u32,
        health_cycle_epoch: u64,
    }

    #[init(local = [
        record_queue: Queue<[u8; RAW_OBSERVATION_RECORD_LEN], 8> = Queue::new(),
        record_dma_buffer: [u8; RAW_OBSERVATION_RECORD_LEN] = [0; RAW_OBSERVATION_RECORD_LEN]
    ])]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        let mut dcb = ctx.core.DCB;
        let mut dwt = ctx.core.DWT;
        dcb.enable_trace();
        dwt.enable_cycle_counter();

        let mut flash = ctx.device.FLASH.constrain();
        let mut rcc = ctx.device.RCC.freeze(
            rcc::Config::hse(8.MHz())
                .sysclk(72.MHz())
                .pclk1(36.MHz())
                .pclk2(72.MHz())
                .adcclk(12.MHz()),
            &mut flash.acr,
        );

        let mut afio = ctx.device.AFIO.constrain(&mut rcc);
        let dma_channels = ctx.device.DMA1.split(&mut rcc);
        let mut gpioa = ctx.device.GPIOA.split(&mut rcc);
        let mut gpiob = ctx.device.GPIOB.split(&mut rcc);
        let mut gpioc = ctx.device.GPIOC.split(&mut rcc);

        let sda = gpiob
            .pb8
            .into_open_drain_output_with_state(&mut gpiob.crh, PinState::High);
        let scl = gpiob
            .pb9
            .into_open_drain_output_with_state(&mut gpiob.crh, PinState::High);
        let delay = ctx.core.SYST.delay(&rcc.clocks);
        let mut bus = SoftwareI2c::new(sda, scl, delay, I2C_HALF_PERIOD_NS, 100);
        let bus_ready = bus.recover_bus().is_ok();

        let mut imu_int = gpioc.pc13.into_floating_input(&mut gpioc.crh);
        imu_int.make_interrupt_source(&mut afio);
        imu_int.trigger_on_edge(&mut ctx.device.EXTI, Edge::Rising);

        let mut imu = Mpu6050::new(bus, board::MPU6050_ADDRESS);
        let imu_present = bus_ready && imu.probe().is_ok();
        let imu_configured = imu_present
            && imu
                .configure(MpuConfig {
                    gyro_range: GyroRange::Dps1000,
                    accel_range: AccelRange::G4,
                    dlpf: Dlpf::Config3,
                    sample_rate_hz: MPU_ACQUISITION_HZ,
                    data_ready_interrupt: true,
                })
                .is_ok();

        if imu_configured {
            imu_int.clear_interrupt_pending_bit();
            imu_int.enable_interrupt(&mut ctx.device.EXTI);
        }

        let encoder_1 = Timer::new(ctx.device.TIM2, &mut rcc)
            .qei((gpioa.pa0, gpioa.pa1), QeiOptions::default());
        let encoder_2 = Timer::new(ctx.device.TIM4, &mut rcc)
            .qei((gpiob.pb6, gpiob.pb7), QeiOptions::default());

        let battery_adc_pin = gpioa.pa5.into_analog(&mut gpioa.crl);
        let battery_adc = Adc::new(ctx.device.ADC1, &mut rcc);

        let bluetooth_tx = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
        let record_tx = ctx.device.USART2.tx(
            bluetooth_tx,
            SerialConfig::default().baudrate(RECORD_BAUD.bps()),
            &mut rcc,
        );
        let record_dma = record_tx.with_dma(dma_channels.7);

        let mut health_timer = ctx.device.TIM1.counter_ms(&mut rcc);
        health_timer.start(HEALTH_PERIOD_MS.millis()).unwrap();
        health_timer.listen(TimerEvent::Update);

        let (record_producer, record_consumer) = ctx.local.record_queue.split();
        let record_pump =
            UartRecordDmaPump::new(record_dma, record_consumer, ctx.local.record_dma_buffer);

        let initial_cycle = DWT::cycle_count();
        let timing_started_at_us = u64::from(initial_cycle) / CYCLES_PER_US;
        let timing_limits = SensorTimingLimits::new(
            MPU_EXPECTED_PERIOD_US,
            MPU_LATE_AFTER_US,
            MPU_TIMEOUT_AFTER_US,
        )
        .unwrap();
        let imu_timing_monitor = SensorTimingMonitor::new(timing_limits, timing_started_at_us);

        // TIM3 and all motor GPIO remain untouched. MPU6050 DATA_RDY on PC13
        // defines the 500 Hz acquisition / estimator / inner-balance boundary.
        // TIM1 independently supervises that boundary; USART2 recording uses
        // DMA1 channel 7 so telemetry cannot create a byte-rate interrupt load.
        (
            Shared { imu_timing_monitor },
            Local {
                imu,
                imu_int,
                encoder_1,
                encoder_2,
                battery_adc,
                battery_adc_pin,
                health_timer,
                record_producer,
                record_pump,
                bus_ready,
                imu_present,
                imu_configured,
                observation_divider: 0,
                sequence: 0,
                dropped_records: 0,
                imu_last_cycle: initial_cycle,
                imu_cycle_epoch: 0,
                health_last_cycle: initial_cycle,
                health_cycle_epoch: 0,
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
        priority = 3,
        shared = [imu_timing_monitor],
        local = [health_timer, health_last_cycle, health_cycle_epoch]
    )]
    fn timing_health(mut ctx: timing_health::Context) {
        ctx.local.health_timer.clear_interrupt(TimerEvent::Update);
        let now_us =
            capture_timestamp_us(ctx.local.health_last_cycle, ctx.local.health_cycle_epoch);
        ctx.shared
            .imu_timing_monitor
            .lock(|monitor| monitor.poll(now_us));
    }

    #[task(
        binds = EXTI15_10,
        priority = 2,
        shared = [imu_timing_monitor],
        local = [
            imu,
            imu_int,
            encoder_1,
            encoder_2,
            battery_adc,
            battery_adc_pin,
            record_producer,
            bus_ready,
            imu_present,
            imu_configured,
            observation_divider,
            sequence,
            dropped_records,
            imu_last_cycle,
            imu_cycle_epoch
        ]
    )]
    fn imu_data_ready(mut ctx: imu_data_ready::Context) {
        let acquisition_started_us =
            capture_timestamp_us(ctx.local.imu_last_cycle, ctx.local.imu_cycle_epoch);
        ctx.local.imu_int.clear_interrupt_pending_bit();

        let timing_health = ctx
            .shared
            .imu_timing_monitor
            .lock(|monitor| monitor.on_event(acquisition_started_us));

        let mut acquisition_status = AcquisitionStatus::NONE;
        if *ctx.local.bus_ready {
            acquisition_status |= AcquisitionStatus::BUS_READY;
        }
        if *ctx.local.imu_present {
            acquisition_status |= AcquisitionStatus::IMU_PRESENT;
        }
        if *ctx.local.imu_configured {
            acquisition_status |= AcquisitionStatus::IMU_CONFIGURED;
            acquisition_status |= AcquisitionStatus::IMU_DATA_READY_IRQ_ENABLED;
            acquisition_status |= AcquisitionStatus::IMU_DATA_READY_SEEN;
        }
        acquisition_status |= timing_status(timing_health);

        let imu_read_started_at_us = TimestampEvidence::Known(capture_timestamp_us(
            ctx.local.imu_last_cycle,
            ctx.local.imu_cycle_epoch,
        ));
        let (sample, mut imu_quality) = match ctx.local.imu.read_raw() {
            Ok(value) => (
                value,
                MeasurementQuality::AVAILABLE
                    | MeasurementQuality::IO_OK
                    | MeasurementQuality::FRESHNESS_VERIFIED,
            ),
            Err(_) => (RawSample::default(), MeasurementQuality::IO_ERROR),
        };
        if timing_health == SensorTimingHealth::Healthy
            && imu_quality.contains(MeasurementQuality::IO_OK)
        {
            imu_quality |= MeasurementQuality::TIMING_VALID;
        }
        let imu_read_completed_at_us = TimestampEvidence::Known(capture_timestamp_us(
            ctx.local.imu_last_cycle,
            ctx.local.imu_cycle_epoch,
        ));

        *ctx.local.observation_divider += 1;
        if *ctx.local.observation_divider < OBSERVATION_DECIMATION {
            return;
        }
        *ctx.local.observation_divider = 0;

        // Encoder capture, outer-loop observation, and canonical recording are
        // aligned at 100 Hz. The 500 Hz IMU path remains independent of this
        // recording decimation.
        let encoder_1_count = ctx.local.encoder_1.count();
        let encoder_1_captured_at_us = TimestampEvidence::Known(capture_timestamp_us(
            ctx.local.imu_last_cycle,
            ctx.local.imu_cycle_epoch,
        ));
        let encoder_2_count = ctx.local.encoder_2.count();
        let encoder_2_captured_at_us = TimestampEvidence::Known(capture_timestamp_us(
            ctx.local.imu_last_cycle,
            ctx.local.imu_cycle_epoch,
        ));
        let encoder_quality = MeasurementQuality::AVAILABLE
            | MeasurementQuality::IO_OK
            | MeasurementQuality::TIMING_VALID;

        let (battery_adc_raw, battery_quality) =
            match ctx.local.battery_adc.read(ctx.local.battery_adc_pin) {
                Ok(value) => (
                    value,
                    MeasurementQuality::AVAILABLE | MeasurementQuality::IO_OK,
                ),
                Err(_) => (0, MeasurementQuality::IO_ERROR),
            };
        let battery_read_completed_at_us = TimestampEvidence::Known(capture_timestamp_us(
            ctx.local.imu_last_cycle,
            ctx.local.imu_cycle_epoch,
        ));

        let acquisition_completed_us =
            capture_timestamp_us(ctx.local.imu_last_cycle, ctx.local.imu_cycle_epoch);

        let observation = RawObservation {
            sample_index: *ctx.local.sequence,
            acquisition_started_us,
            acquisition_completed_us,
            imu: RawImuObservation {
                // DATA_RDY proves that this register image is fresh. EXTI13
                // timestamps ISR service, not the MPU6050's internal sample
                // instant, so source time remains explicitly unknown.
                source_sample_at_us: TimestampEvidence::Unknown,
                read_started_at_us: imu_read_started_at_us,
                read_completed_at_us: imu_read_completed_at_us,
                accel_raw: sample.accel,
                temperature_raw: sample.temperature,
                gyro_raw: sample.gyro,
                quality: imu_quality,
            },
            encoders: [
                RawEncoderObservation {
                    captured_at_us: encoder_1_captured_at_us,
                    count: encoder_1_count,
                    quality: encoder_quality,
                },
                RawEncoderObservation {
                    captured_at_us: encoder_2_captured_at_us,
                    count: encoder_2_count,
                    quality: encoder_quality,
                },
            ],
            battery: RawBatteryObservation {
                read_completed_at_us: battery_read_completed_at_us,
                adc_raw: battery_adc_raw,
                quality: battery_quality,
            },
            acquisition_status,
        };

        let record = RecordedObservation {
            observation,
            dropped_records: *ctx.local.dropped_records,
        }
        .encode();

        if ctx.local.record_producer.enqueue(record).is_ok() {
            rtic::pend(pac::Interrupt::DMA1_CHANNEL7);
        } else {
            *ctx.local.dropped_records = ctx.local.dropped_records.saturating_add(1);
        }

        *ctx.local.sequence = ctx.local.sequence.wrapping_add(1);
    }

    #[task(binds = DMA1_CHANNEL7, priority = 1, local = [record_pump])]
    fn record_tx_dma(ctx: record_tx_dma::Context) {
        ctx.local.record_pump.on_interrupt();
    }

    fn timing_status(health: SensorTimingHealth) -> AcquisitionStatus {
        match health {
            SensorTimingHealth::Startup => AcquisitionStatus::NONE,
            SensorTimingHealth::Healthy => AcquisitionStatus::IMU_TIMING_HEALTHY,
            SensorTimingHealth::Late => AcquisitionStatus::IMU_TIMING_LATE,
            SensorTimingHealth::Timeout => AcquisitionStatus::IMU_TIMING_TIMEOUT,
        }
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

#![no_std]
#![no_main]
#![deny(unsafe_code)]

use panic_halt as _;

/// Live, non-actuating commissioning firmware for measuring the real 500 Hz
/// acquisition + model-based control execution budget on STM32F103C8.
///
/// The numeric design, calibration matrix, frame rotation, and encoder scale in
/// this binary are deliberately synthetic profiling fixtures. They are never
/// reference-platform evidence and never authorize electrical output. TIM3 and
/// every motor GPIO remain unconfigured.
#[rtic::app(device = stm32f1xx_hal::pac)]
mod app {
    use core::f32::consts::TAU;

    use cortex_m::peripheral::DWT;
    use heapless::spsc::{Consumer, Producer, Queue};
    use stm32f1xx_hal::{
        dma::{Event as DmaEvent, R as DmaRead, Transfer},
        gpio::{
            Edge, ExtiPin, Floating, Input, OpenDrain, Output, PinState,
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
    use swp_actuator_model::{
        ActuatorPairModel, ActuatorPairOperatingPoint, ActuatorParameters, StaticActuatorModel,
    };
    use swp_board_one_v2 as board;
    use swp_control_profile_record::{
        CONTROL_PROFILE_RECORD_LEN, ControlProfileSample, ControlProfileStatus,
    };
    use swp_frame_transform::SensorToBodyRotation;
    use swp_measurement_model::{
        ACCEL_X, ACCEL_Y, ACCEL_Z, DRIVE_ENCODER_RELATIVE_ANGLE, GYRO_X, GYRO_Y, GYRO_Z,
        REACTION_WHEEL_RELATIVE_RATE, UPRIGHT_MEASUREMENT_COUNT, UprightMeasurementModel,
    };
    use swp_mpu6050::{AccelRange, Config as MpuConfig, Dlpf, GyroRange, Mpu6050, RawSample};
    use swp_plant_model::{
        DiscreteLinearPlant, REDUCED_BALANCE_STATE_COUNT, REFERENCE_INPUT_COUNT,
        ReducedBalanceState, ReferencePlantInput,
    };
    use swp_plant_observation::{MeasurementQuality, RawImuObservation, TimestampEvidence};
    use swp_robot_domain::{AngularRateRadPerSec, GeneralizedDemand};
    use swp_runtime_state::{
        AuthorityContext, OperatingState, ReactionWheelSpeedLimits, RuntimeAuthority,
        SensorTimingHealth, SensorTimingLimits, SensorTimingMonitor,
    };
    use swp_sensor_calibration::{AffineCalibration3, scale_mpu6050};
    use swp_software_i2c::SoftwareI2c;
    use swp_state_estimator::{
        EstimatorMeasurement, LinearObserver, MeasurementMask, ObserverDesign, ObserverGain,
    };
    use swp_state_feedback::{LqrController, StateFeedbackGain};

    const CPU_HZ: u32 = 72_000_000;
    const CYCLES_PER_US: u64 = CPU_HZ as u64 / 1_000_000;
    const CONTROL_HZ: u32 = 500;
    const CONTROL_DEADLINE_CYCLES: u32 = CPU_HZ / CONTROL_HZ;

    const MPU_EXPECTED_PERIOD_US: u32 = 2_000;
    const MPU_LATE_AFTER_US: u32 = 3_000;
    const MPU_TIMEOUT_AFTER_US: u32 = 6_000;
    const HEALTH_PERIOD_MS: u32 = 1;
    const I2C_HALF_PERIOD_NS: u32 = 1_250;

    const PROFILE_BAUD: u32 = 115_200;
    const PROFILE_DECIMATION: u8 = 25; // 20 Hz profile stream from a 500 Hz loop.
    const PROFILE_QUEUE_STORAGE: usize = 8;

    const SYNTHETIC_COUNTS_PER_REVOLUTION: f32 = 4_096.0;
    const SYNTHETIC_RAD_PER_COUNT: f32 = TAU / SYNTHETIC_COUNTS_PER_REVOLUTION;

    type ImuBus = SoftwareI2c<PB8<Output<OpenDrain>>, PB9<Output<OpenDrain>>, SysDelay>;
    type Imu = Mpu6050<ImuBus>;
    type ImuInt = PC13<Input<Floating>>;
    type Encoder1 = Qei<pac::TIM2>;
    type Encoder2 = Qei<pac::TIM4>;
    type HealthTimer = CounterMs<pac::TIM1>;
    type ProfileDma = TxDma2;
    type ProfileTransfer =
        Transfer<DmaRead, &'static mut [u8; CONTROL_PROFILE_RECORD_LEN], ProfileDma>;
    type ProfileProducer =
        Producer<'static, [u8; CONTROL_PROFILE_RECORD_LEN], PROFILE_QUEUE_STORAGE>;
    type ProfileConsumer =
        Consumer<'static, [u8; CONTROL_PROFILE_RECORD_LEN], PROFILE_QUEUE_STORAGE>;

    enum ProfileDmaState {
        Idle {
            dma: ProfileDma,
            buffer: &'static mut [u8; CONTROL_PROFILE_RECORD_LEN],
        },
        Active(ProfileTransfer),
    }

    struct ProfileDmaPump {
        consumer: ProfileConsumer,
        state: Option<ProfileDmaState>,
    }

    impl ProfileDmaPump {
        fn new(
            mut dma: ProfileDma,
            consumer: ProfileConsumer,
            buffer: &'static mut [u8; CONTROL_PROFILE_RECORD_LEN],
        ) -> Self {
            dma.channel.listen(DmaEvent::TransferComplete);
            Self {
                consumer,
                state: Some(ProfileDmaState::Idle { dma, buffer }),
            }
        }

        fn on_interrupt(&mut self) {
            let Some(state) = self.state.take() else {
                return;
            };

            let idle = match state {
                ProfileDmaState::Active(transfer) => {
                    if !transfer.is_done() {
                        self.state = Some(ProfileDmaState::Active(transfer));
                        return;
                    }
                    let (buffer, dma) = transfer.wait();
                    ProfileDmaState::Idle { dma, buffer }
                }
                idle @ ProfileDmaState::Idle { .. } => idle,
            };

            let ProfileDmaState::Idle { dma, buffer } = idle else {
                unreachable!();
            };

            if let Some(record) = self.consumer.dequeue() {
                buffer.copy_from_slice(&record);
                self.state = Some(ProfileDmaState::Active(dma.write(buffer)));
            } else {
                self.state = Some(ProfileDmaState::Idle { dma, buffer });
            }
        }
    }

    #[derive(Clone, Copy, Default)]
    struct SyntheticEncoderTracker {
        primed: bool,
        previous_drive_count: u16,
        previous_reaction_count: u16,
        previous_at_us: u64,
        drive_unwrapped_counts: i64,
    }

    impl SyntheticEncoderTracker {
        fn observe(
            &mut self,
            drive_count: u16,
            reaction_count: u16,
            captured_at_us: u64,
        ) -> (f32, f32, f32) {
            if !self.primed {
                self.primed = true;
                self.previous_drive_count = drive_count;
                self.previous_reaction_count = reaction_count;
                self.previous_at_us = captured_at_us;
                return (0.0, 0.0, 0.0);
            }

            let delta_us = captured_at_us.saturating_sub(self.previous_at_us).max(1);
            let delta_s = delta_us as f32 * 1.0e-6;
            let drive_delta = drive_count.wrapping_sub(self.previous_drive_count) as i16 as i32;
            let reaction_delta =
                reaction_count.wrapping_sub(self.previous_reaction_count) as i16 as i32;

            self.drive_unwrapped_counts += i64::from(drive_delta);
            let drive_angle = self.drive_unwrapped_counts as f32 * SYNTHETIC_RAD_PER_COUNT;
            let drive_rate = drive_delta as f32 * SYNTHETIC_RAD_PER_COUNT / delta_s;
            let reaction_rate = reaction_delta as f32 * SYNTHETIC_RAD_PER_COUNT / delta_s;

            self.previous_drive_count = drive_count;
            self.previous_reaction_count = reaction_count;
            self.previous_at_us = captured_at_us;
            (drive_angle, drive_rate, reaction_rate)
        }
    }

    struct ShadowEngine {
        observer: LinearObserver,
        controller: LqrController,
        actuators: ActuatorPairModel,
        reaction_limits: ReactionWheelSpeedLimits,
        previous_applied_input: ReferencePlantInput,
        accel_profile_transform: AffineCalibration3,
        gyro_profile_transform: AffineCalibration3,
        sensor_to_profile_body: SensorToBodyRotation,
        encoders: SyntheticEncoderTracker,
    }

    struct ShadowStepProfile {
        semantic_projection_cycles: u32,
        estimator_cycles: u32,
        feedback_cycles: u32,
        actuator_authority_cycles: u32,
        authority_reasons: u16,
        status: ControlProfileStatus,
    }

    impl ShadowEngine {
        fn new() -> Self {
            let mut a_d = [[0.0; REDUCED_BALANCE_STATE_COUNT]; REDUCED_BALANCE_STATE_COUNT];
            for (index, row) in a_d.iter_mut().enumerate() {
                row[index] = 1.0;
            }
            let plant = DiscreteLinearPlant {
                sample_period_s: 1.0 / CONTROL_HZ as f32,
                a_d,
                b_d: [[0.0; REFERENCE_INPUT_COUNT]; REDUCED_BALANCE_STATE_COUNT],
            };

            let mut c = [[0.0; REDUCED_BALANCE_STATE_COUNT]; UPRIGHT_MEASUREMENT_COUNT];
            let mut l = [[0.0; UPRIGHT_MEASUREMENT_COUNT]; REDUCED_BALANCE_STATE_COUNT];
            for index in 0..REDUCED_BALANCE_STATE_COUNT {
                c[index][index] = 1.0;
                l[index][index] = 0.05;
            }
            c[REACTION_WHEEL_RELATIVE_RATE][6] = 1.0;
            l[6][REACTION_WHEEL_RELATIVE_RATE] = 0.05;
            let required = MeasurementMask::from_bits((1_u16 << UPRIGHT_MEASUREMENT_COUNT) - 1);
            let measurement_model = UprightMeasurementModel {
                nominal: [0.0; UPRIGHT_MEASUREMENT_COUNT],
                c,
                d: [[0.0; REFERENCE_INPUT_COUNT]; UPRIGHT_MEASUREMENT_COUNT],
            };
            let design = ObserverDesign::new(
                plant,
                measurement_model,
                ObserverGain::new(l).expect("synthetic observer gain"),
                required,
            )
            .expect("synthetic observer design");
            let observer = LinearObserver::new(design, ReducedBalanceState::default())
                .expect("synthetic observer initial state");

            let mut k = [[0.0; REDUCED_BALANCE_STATE_COUNT]; 2];
            k[0][2] = 0.8;
            k[0][3] = 0.05;
            k[1][4] = 0.8;
            k[1][5] = 0.05;
            let controller = LqrController::new(
                StateFeedbackGain::new(k).expect("synthetic state-feedback gain"),
            );

            let actuator_parameters = ActuatorParameters::new(10.0, 0.05, 0.001, 0.002, 0.1)
                .expect("synthetic actuator parameters");
            let actuator =
                StaticActuatorModel::new(actuator_parameters).expect("synthetic actuator model");
            let actuators = ActuatorPairModel {
                drive: actuator,
                reaction: actuator,
            };

            let identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
            let accel_profile_transform =
                AffineCalibration3::new([0.0; 3], identity).expect("synthetic accel transform");
            let gyro_profile_transform =
                AffineCalibration3::new([0.0; 3], identity).expect("synthetic gyro transform");
            let sensor_to_profile_body =
                SensorToBodyRotation::new(identity).expect("synthetic frame rotation");

            Self {
                observer,
                controller,
                actuators,
                reaction_limits: ReactionWheelSpeedLimits::new(1_000.0, 2_000.0)
                    .expect("synthetic reaction-wheel limits"),
                previous_applied_input: ReferencePlantInput::default(),
                accel_profile_transform,
                gyro_profile_transform,
                sensor_to_profile_body,
                encoders: SyntheticEncoderTracker::default(),
            }
        }

        fn step(
            &mut self,
            raw_imu: RawImuObservation,
            drive_count: u16,
            reaction_count: u16,
            captured_at_us: u64,
            timing: SensorTimingHealth,
        ) -> ShadowStepProfile {
            let mut status = ControlProfileStatus::SYNTHETIC_NUMERICS
                .with(ControlProfileStatus::MOTOR_PERIPHERALS_ABSENT);
            if raw_imu.quality.contains(MeasurementQuality::IO_OK) {
                status = status.with(ControlProfileStatus::IMU_IO_OK);
            }
            if timing == SensorTimingHealth::Healthy {
                status = status.with(ControlProfileStatus::TIMING_HEALTHY);
            }

            let semantic_started = DWT::cycle_count();
            let scaled = match scale_mpu6050(raw_imu, mpu_config()) {
                Ok(value) => value,
                Err(_) => {
                    return ShadowStepProfile {
                        semantic_projection_cycles: elapsed_cycles(semantic_started),
                        estimator_cycles: 0,
                        feedback_cycles: 0,
                        actuator_authority_cycles: 0,
                        authority_reasons: 0,
                        status,
                    };
                }
            };

            // These transforms exercise the same CMSIS-DSP vector kernels used by
            // production calibration/frame code but intentionally do not create
            // evidence-bearing CalibratedImuObservation/BodyImuObservation values.
            let profile_accel = self
                .sensor_to_profile_body
                .apply(self.accel_profile_transform.apply(scaled.acceleration.0));
            let profile_gyro = self
                .sensor_to_profile_body
                .apply(self.gyro_profile_transform.apply(scaled.angular_rate.0));
            let (drive_angle, drive_rate, reaction_rate) =
                self.encoders
                    .observe(drive_count, reaction_count, captured_at_us);

            let mut values = [0.0; UPRIGHT_MEASUREMENT_COUNT];
            // Numeric conditioning is synthetic and exists only to keep the
            // profiler on ordinary finite branches while real sensor data changes.
            values[ACCEL_X] = profile_accel[0] * 0.01;
            values[ACCEL_Y] = profile_accel[1] * 0.01;
            values[ACCEL_Z] = (profile_accel[2] - 9.806_65) * 0.01;
            values[GYRO_X] = profile_gyro[0];
            values[GYRO_Y] = profile_gyro[1];
            values[GYRO_Z] = profile_gyro[2];
            values[DRIVE_ENCODER_RELATIVE_ANGLE] = drive_angle * 0.01;
            values[REACTION_WHEEL_RELATIVE_RATE] = reaction_rate * 0.01;
            let measurement = EstimatorMeasurement::new(
                values,
                MeasurementMask::from_bits((1_u16 << UPRIGHT_MEASUREMENT_COUNT) - 1),
                timing == SensorTimingHealth::Healthy,
            );
            status = status.with(ControlProfileStatus::SEMANTIC_PROJECTION_READY);
            let semantic_projection_cycles = elapsed_cycles(semantic_started);

            let estimator_started = DWT::cycle_count();
            let estimate = match self.observer.step(
                self.previous_applied_input,
                self.previous_applied_input,
                measurement,
            ) {
                Ok(value) => value,
                Err(_) => {
                    self.previous_applied_input = ReferencePlantInput::default();
                    return ShadowStepProfile {
                        semantic_projection_cycles,
                        estimator_cycles: elapsed_cycles(estimator_started),
                        feedback_cycles: 0,
                        actuator_authority_cycles: 0,
                        authority_reasons: 0,
                        status,
                    };
                }
            };
            let estimator_cycles = elapsed_cycles(estimator_started);
            status = status.with(ControlProfileStatus::ESTIMATOR_OK);

            let feedback_started = DWT::cycle_count();
            let demand = match self.controller.command(
                estimate.state,
                ReducedBalanceState::default(),
                GeneralizedDemand::default(),
            ) {
                Ok(value) => value,
                Err(_) => {
                    self.previous_applied_input = ReferencePlantInput::default();
                    return ShadowStepProfile {
                        semantic_projection_cycles,
                        estimator_cycles,
                        feedback_cycles: elapsed_cycles(feedback_started),
                        actuator_authority_cycles: 0,
                        authority_reasons: 0,
                        status,
                    };
                }
            };
            let feedback_cycles = elapsed_cycles(feedback_started);
            status = status.with(ControlProfileStatus::FEEDBACK_OK);

            let actuator_started = DWT::cycle_count();
            let commands = match self.actuators.command_for_demand(
                demand,
                ActuatorPairOperatingPoint {
                    drive_speed_rad_per_s: drive_rate,
                    reaction_speed_rad_per_s: reaction_rate,
                },
            ) {
                Ok(value) => value,
                Err(_) => {
                    self.previous_applied_input = ReferencePlantInput::default();
                    return ShadowStepProfile {
                        semantic_projection_cycles,
                        estimator_cycles,
                        feedback_cycles,
                        actuator_authority_cycles: elapsed_cycles(actuator_started),
                        authority_reasons: 0,
                        status,
                    };
                }
            };
            let reaction_wheel_authority = self.reaction_limits.classify(AngularRateRadPerSec(
                estimate.state.reaction_wheel_rate_rad_per_s,
            ));
            let outcome = RuntimeAuthority::evaluate(
                AuthorityContext {
                    operating_state: OperatingState::Balancing,
                    timing,
                    estimate_validity: estimate.validity,
                    reaction_wheel_authority,
                },
                commands,
            );
            let actuator_authority_cycles = elapsed_cycles(actuator_started);
            status = status.with(ControlProfileStatus::AUTHORITY_EVALUATED);
            if outcome.authorized().is_some() {
                status = status.with(ControlProfileStatus::AUTHORIZED_TOKEN_DROPPED);
            }

            // Physical truth for shadow mode: no motor command is ever applied.
            // The observer must therefore retain zero applied input even when the
            // synthetic authority evaluation produces an AuthorizedActuation token.
            self.previous_applied_input = ReferencePlantInput::default();

            ShadowStepProfile {
                semantic_projection_cycles,
                estimator_cycles,
                feedback_cycles,
                actuator_authority_cycles,
                authority_reasons: outcome.decision().reasons.bits(),
                status,
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
        health_timer: HealthTimer,
        profile_producer: ProfileProducer,
        profile_pump: ProfileDmaPump,
        shadow: ShadowEngine,
        profile_divider: u8,
        sequence: u32,
        dropped_records: u16,
        window_max_critical_path_cycles: u32,
        boot_max_critical_path_cycles: u32,
        overrun_count: u32,
        imu_last_cycle: u32,
        imu_cycle_epoch: u64,
        health_last_cycle: u32,
        health_cycle_epoch: u64,
    }

    #[init(local = [
        profile_queue: Queue<[u8; CONTROL_PROFILE_RECORD_LEN], 8> = Queue::new(),
        profile_dma_buffer: [u8; CONTROL_PROFILE_RECORD_LEN] = [0; CONTROL_PROFILE_RECORD_LEN]
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
                .pclk2(72.MHz()),
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
        let imu_configured = imu_present && imu.configure(mpu_config()).is_ok();
        if imu_configured {
            imu_int.clear_interrupt_pending_bit();
            imu_int.enable_interrupt(&mut ctx.device.EXTI);
        }

        // Installed reference assembly mapping:
        // Encoder_1/TIM2 = reaction wheel, Encoder_2/TIM4 = drive wheel.
        let encoder_1 = Timer::new(ctx.device.TIM2, &mut rcc)
            .qei((gpioa.pa0, gpioa.pa1), QeiOptions::default());
        let encoder_2 = Timer::new(ctx.device.TIM4, &mut rcc)
            .qei((gpiob.pb6, gpiob.pb7), QeiOptions::default());

        let bluetooth_tx = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
        let profile_tx = ctx.device.USART2.tx(
            bluetooth_tx,
            SerialConfig::default().baudrate(PROFILE_BAUD.bps()),
            &mut rcc,
        );
        let profile_dma = profile_tx.with_dma(dma_channels.7);

        let mut health_timer = ctx.device.TIM1.counter_ms(&mut rcc);
        health_timer.start(HEALTH_PERIOD_MS.millis()).unwrap();
        health_timer.listen(TimerEvent::Update);

        let (profile_producer, profile_consumer) = ctx.local.profile_queue.split();
        let profile_pump = ProfileDmaPump::new(
            profile_dma,
            profile_consumer,
            ctx.local.profile_dma_buffer,
        );

        let initial_cycle = DWT::cycle_count();
        let started_at_us = u64::from(initial_cycle) / CYCLES_PER_US;
        let timing_limits = SensorTimingLimits::new(
            MPU_EXPECTED_PERIOD_US,
            MPU_LATE_AFTER_US,
            MPU_TIMEOUT_AFTER_US,
        )
        .unwrap();

        // This binary intentionally owns no TIM3 channel and no motor GPIO.
        // Its only outputs are profile records on USART2 DMA.
        (
            Shared {
                imu_timing_monitor: SensorTimingMonitor::new(timing_limits, started_at_us),
            },
            Local {
                imu,
                imu_int,
                encoder_1,
                encoder_2,
                health_timer,
                profile_producer,
                profile_pump,
                shadow: ShadowEngine::new(),
                profile_divider: 0,
                sequence: 0,
                dropped_records: 0,
                window_max_critical_path_cycles: 0,
                boot_max_critical_path_cycles: 0,
                overrun_count: 0,
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
            profile_producer,
            shadow,
            profile_divider,
            sequence,
            dropped_records,
            window_max_critical_path_cycles,
            boot_max_critical_path_cycles,
            overrun_count,
            imu_last_cycle,
            imu_cycle_epoch
        ]
    )]
    fn imu_data_ready(mut ctx: imu_data_ready::Context) {
        let critical_started_cycle = DWT::cycle_count();
        let event_started_us =
            capture_timestamp_us(ctx.local.imu_last_cycle, ctx.local.imu_cycle_epoch);
        ctx.local.imu_int.clear_interrupt_pending_bit();

        let timing = ctx
            .shared
            .imu_timing_monitor
            .lock(|monitor| monitor.on_event(event_started_us));

        let imu_read_started_cycle = DWT::cycle_count();
        let read_started_at_us = TimestampEvidence::Known(capture_timestamp_us(
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
        let imu_read_cycles = elapsed_cycles(imu_read_started_cycle);
        if timing == SensorTimingHealth::Healthy
            && imu_quality.contains(MeasurementQuality::IO_OK)
        {
            imu_quality |= MeasurementQuality::TIMING_VALID;
        }
        let read_completed_at_us = TimestampEvidence::Known(capture_timestamp_us(
            ctx.local.imu_last_cycle,
            ctx.local.imu_cycle_epoch,
        ));

        let encoder_started_cycle = DWT::cycle_count();
        let reaction_encoder_count = ctx.local.encoder_1.count();
        let drive_encoder_count = ctx.local.encoder_2.count();
        let encoder_snapshot_cycles = elapsed_cycles(encoder_started_cycle);
        let encoder_captured_at_us =
            capture_timestamp_us(ctx.local.imu_last_cycle, ctx.local.imu_cycle_epoch);

        let raw_imu = RawImuObservation {
            source_sample_at_us: TimestampEvidence::Unknown,
            read_started_at_us,
            read_completed_at_us,
            accel_raw: sample.accel,
            temperature_raw: sample.temperature,
            gyro_raw: sample.gyro,
            quality: imu_quality,
        };
        let step = ctx.local.shadow.step(
            raw_imu,
            drive_encoder_count,
            reaction_encoder_count,
            encoder_captured_at_us,
            timing,
        );

        let critical_path_cycles = elapsed_cycles(critical_started_cycle);
        *ctx.local.window_max_critical_path_cycles = (*ctx
            .local
            .window_max_critical_path_cycles)
            .max(critical_path_cycles);
        *ctx.local.boot_max_critical_path_cycles = (*ctx.local.boot_max_critical_path_cycles)
            .max(critical_path_cycles);

        let mut status = step.status;
        if critical_path_cycles > CONTROL_DEADLINE_CYCLES {
            *ctx.local.overrun_count = ctx.local.overrun_count.saturating_add(1);
            status = status.with(ControlProfileStatus::CRITICAL_PATH_OVERRUN);
        }

        *ctx.local.profile_divider += 1;
        if *ctx.local.profile_divider >= PROFILE_DECIMATION {
            *ctx.local.profile_divider = 0;
            let record = ControlProfileSample {
                sequence: *ctx.local.sequence,
                event_started_us,
                imu_read_cycles,
                encoder_snapshot_cycles,
                semantic_projection_cycles: step.semantic_projection_cycles,
                estimator_cycles: step.estimator_cycles,
                feedback_cycles: step.feedback_cycles,
                actuator_authority_cycles: step.actuator_authority_cycles,
                critical_path_cycles,
                window_max_critical_path_cycles: *ctx.local.window_max_critical_path_cycles,
                boot_max_critical_path_cycles: *ctx.local.boot_max_critical_path_cycles,
                deadline_cycles: CONTROL_DEADLINE_CYCLES,
                overrun_count: *ctx.local.overrun_count,
                cpu_hz: CPU_HZ,
                authority_reasons: step.authority_reasons,
                status,
                dropped_records: *ctx.local.dropped_records,
            }
            .encode();

            if ctx.local.profile_producer.enqueue(record).is_ok() {
                rtic::pend(pac::Interrupt::DMA1_CHANNEL7);
            } else {
                *ctx.local.dropped_records = ctx.local.dropped_records.saturating_add(1);
            }
            *ctx.local.window_max_critical_path_cycles = 0;
        }

        *ctx.local.sequence = ctx.local.sequence.wrapping_add(1);
    }

    #[task(binds = DMA1_CHANNEL7, priority = 1, local = [profile_pump])]
    fn profile_tx_dma(ctx: profile_tx_dma::Context) {
        ctx.local.profile_pump.on_interrupt();
    }

    fn mpu_config() -> MpuConfig {
        MpuConfig {
            gyro_range: GyroRange::Dps1000,
            accel_range: AccelRange::G4,
            dlpf: Dlpf::Config3,
            sample_rate_hz: CONTROL_HZ as u16,
            data_ready_interrupt: true,
        }
    }

    fn elapsed_cycles(started: u32) -> u32 {
        DWT::cycle_count().wrapping_sub(started)
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

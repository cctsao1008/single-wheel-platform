#![no_std]
#![no_main]
#![deny(unsafe_code)]

use panic_halt as _;

/// Non-actuating 200 Hz control-shadow runtime.
///
/// This target executes the embedded sensing -> estimator -> Control -> Plant
/// actuator model -> Supervisor authority -> Firmware electrical-encoding path,
/// but it deliberately owns no TIM3 motor PWM channel and no motor-direction
/// GPIO. Any AuthorizedActuation token is consumed only by an in-memory
/// ActuationSink. ElectricalActuation is observable as debugger state and can
/// never reach physical motor pins from this binary.
///
/// The sensor-to-estimator numeric projection, observer/controller design, and
/// actuator parameters remain explicit synthetic commissioning fixtures until
/// physical calibration/correlation is completed. They are not reference-platform
/// evidence and must not be used to claim closed-loop physical validity.
#[rtic::app(device = stm32f1xx_hal::pac)]
mod app {
    use core::{
        convert::Infallible,
        f32::consts::TAU,
        sync::atomic::{AtomicI32, AtomicU32, Ordering},
    };

    use cortex_m::peripheral::DWT;
    use stm32f1xx_hal::{
        gpio::{
            Edge, ExtiPin, Floating, Input, OpenDrain, Output, PinState,
            gpiob::{PB8, PB9},
            gpioc::PC13,
        },
        pac,
        prelude::*,
        rcc,
        timer::{
            CounterMs, Event as TimerEvent, SysDelay, Timer,
            pwm_input::{Qei, QeiOptions},
        },
    };
    use swp_actuation_interface::{ActuationSink, ActuatorIo};
    use swp_actuator_model::{
        ActuatorPairModel, ActuatorPairOperatingPoint, ActuatorParameters, StaticActuatorModel,
    };
    use swp_board_one_v2 as board;
    use swp_measurement_model::{
        ACCEL_X, ACCEL_Y, ACCEL_Z, DRIVE_ENCODER_RELATIVE_ANGLE, GYRO_X, GYRO_Y, GYRO_Z,
        REACTION_WHEEL_RELATIVE_RATE, UPRIGHT_MEASUREMENT_COUNT, UprightMeasurementModel,
    };
    use swp_mpu6050::{AccelRange, Config as MpuConfig, Dlpf, GyroRange, Mpu6050, RawSample};
    use swp_one_v2_pwm_dir::{ElectricalActuation, OneV2PwmDirAdapter};
    use swp_plant_model::{
        DiscreteLinearPlant, REDUCED_BALANCE_STATE_COUNT, REFERENCE_INPUT_COUNT,
        ReducedBalanceState, ReferencePlantInput,
    };
    use swp_plant_observation::{
        AcquisitionStatus, MeasurementQuality, RawEncoderObservation, RawImuObservation,
        RawObservation, TimestampEvidence,
    };
    use swp_robot_domain::{AngularRateRadPerSec, GeneralizedDemand};
    use swp_runtime_state::{
        AuthorityContext, OperatingState, ReactionWheelSpeedLimits, RuntimeAuthority,
        SensorTimingHealth, SensorTimingLimits, SensorTimingMonitor,
    };
    use swp_sensor_calibration::scale_mpu6050;
    use swp_software_i2c::SoftwareI2c;
    use swp_state_estimator::{
        EstimatorMeasurement, LinearObserver, MeasurementMask, ObserverDesign, ObserverGain,
    };
    use swp_state_feedback::{LqrController, StateFeedbackGain};

    const CPU_HZ: u64 = 72_000_000;
    const CYCLES_PER_US: u64 = CPU_HZ / 1_000_000;

    const CONTROL_HZ: u16 = 200;
    const EXPECTED_PERIOD_US: u32 = 5_000;
    const LATE_AFTER_US: u32 = 7_500;
    const TIMEOUT_AFTER_US: u32 = 15_000;
    const HEALTH_PERIOD_MS: u32 = 1;
    const I2C_HALF_PERIOD_NS: u32 = 1_250;

    const SYNTHETIC_COUNTS_PER_REVOLUTION: f32 = 4_096.0;
    const SYNTHETIC_RAD_PER_COUNT: f32 = TAU / SYNTHETIC_COUNTS_PER_REVOLUTION;

    type ImuBus = SoftwareI2c<PB8<Output<OpenDrain>>, PB9<Output<OpenDrain>>, SysDelay>;
    type Imu = Mpu6050<ImuBus>;
    type ImuInt = PC13<Input<Floating>>;
    type Encoder1 = Qei<pac::TIM2>;
    type Encoder2 = Qei<pac::TIM4>;
    type HealthTimer = CounterMs<pac::TIM1>;

    /// Debugger-visible shadow results. Integer scaling avoids adding a logging
    /// transport to the hard-real-time path.
    static SHADOW_SAMPLE_INDEX: AtomicU32 = AtomicU32::new(0);
    static SHADOW_STAGE: AtomicU32 = AtomicU32::new(0);
    static SHADOW_TIMING: AtomicU32 = AtomicU32::new(0);
    static SHADOW_AUTHORITY_REASONS: AtomicU32 = AtomicU32::new(0);
    static SHADOW_AUTHORIZED: AtomicU32 = AtomicU32::new(0);
    static SHADOW_DRIVE_DEMAND_MNM: AtomicI32 = AtomicI32::new(0);
    static SHADOW_REACTION_DEMAND_MNM: AtomicI32 = AtomicI32::new(0);
    static SHADOW_DRIVE_COMMAND_PERMILLE: AtomicI32 = AtomicI32::new(0);
    static SHADOW_REACTION_COMMAND_PERMILLE: AtomicI32 = AtomicI32::new(0);
    static SHADOW_DRIVE_PWM_HIGH_PERMILLE: AtomicI32 = AtomicI32::new(1_000);
    static SHADOW_REACTION_PWM_HIGH_PERMILLE: AtomicI32 = AtomicI32::new(1_000);
    static SHADOW_DIRECTION_BITS: AtomicU32 = AtomicU32::new(0);
    static SHADOW_CRITICAL_PATH_CYCLES: AtomicU32 = AtomicU32::new(0);

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

    struct ShadowElectricalIo {
        last: ElectricalActuation,
    }

    impl ShadowElectricalIo {
        const fn new() -> Self {
            Self {
                last: ElectricalActuation::zero_effort(),
            }
        }
    }

    impl ActuatorIo<ElectricalActuation> for ShadowElectricalIo {
        type Error = Infallible;

        fn write_frame(&mut self, frame: ElectricalActuation) -> Result<(), Self::Error> {
            self.last = frame;
            Ok(())
        }
    }

    struct ShadowEngine {
        observer: LinearObserver,
        controller: LqrController,
        actuators: ActuatorPairModel,
        reaction_limits: ReactionWheelSpeedLimits,
        encoders: SyntheticEncoderTracker,
        sink: OneV2PwmDirAdapter<ShadowElectricalIo>,
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

            Self {
                observer,
                controller,
                actuators: ActuatorPairModel {
                    drive: actuator,
                    reaction: actuator,
                },
                reaction_limits: ReactionWheelSpeedLimits::new(1_000.0, 2_000.0)
                    .expect("synthetic reaction-wheel limits"),
                encoders: SyntheticEncoderTracker::default(),
                sink: OneV2PwmDirAdapter::new(ShadowElectricalIo::new()),
            }
        }

        fn step(&mut self, raw: RawObservation, timing: SensorTimingHealth) {
            SHADOW_STAGE.store(1, Ordering::Relaxed);

            let scaled = match scale_mpu6050(raw.imu, mpu_config()) {
                Ok(value) => value,
                Err(_) => return,
            };

            // Reference assembly mapping:
            // Encoder_1 = reaction wheel, Encoder_2 = drive wheel.
            let captured_at_us = raw.acquisition_started_us;
            let (drive_angle, drive_rate, reaction_rate) =
                self.encoders
                    .observe(raw.encoders[1].count, raw.encoders[0].count, captured_at_us);

            let mut values = [0.0; UPRIGHT_MEASUREMENT_COUNT];
            // This projection is intentionally synthetic until measured IMU
            // calibration, body-frame evidence, and encoder transfer evidence exist.
            // It drives real embedded code paths without promoting the values to
            // commissioned physical truth.
            values[ACCEL_X] = scaled.acceleration.0[0] * 0.01;
            values[ACCEL_Y] = scaled.acceleration.0[1] * 0.01;
            values[ACCEL_Z] = (scaled.acceleration.0[2] - 9.806_65) * 0.01;
            values[GYRO_X] = scaled.angular_rate.0[0];
            values[GYRO_Y] = scaled.angular_rate.0[1];
            values[GYRO_Z] = scaled.angular_rate.0[2];
            values[DRIVE_ENCODER_RELATIVE_ANGLE] = drive_angle * 0.01;
            values[REACTION_WHEEL_RELATIVE_RATE] = reaction_rate * 0.01;

            let measurement = EstimatorMeasurement::new(
                values,
                MeasurementMask::from_bits((1_u16 << UPRIGHT_MEASUREMENT_COUNT) - 1),
                timing == SensorTimingHealth::Healthy,
            );
            SHADOW_STAGE.store(2, Ordering::Relaxed);

            // No physical actuation exists in this target, therefore the only
            // truthful observer input is zero applied torque on every step.
            let zero_input = ReferencePlantInput::default();
            let estimate = match self.observer.step(zero_input, zero_input, measurement) {
                Ok(value) => value,
                Err(_) => return,
            };
            SHADOW_STAGE.store(3, Ordering::Relaxed);

            let demand = match self.controller.command(
                estimate.state,
                ReducedBalanceState::default(),
                GeneralizedDemand::default(),
            ) {
                Ok(value) => value,
                Err(_) => return,
            };
            SHADOW_DRIVE_DEMAND_MNM
                .store(scale_milli(demand.drive_wheel_torque.0), Ordering::Relaxed);
            SHADOW_REACTION_DEMAND_MNM.store(
                scale_milli(demand.reaction_wheel_torque.0),
                Ordering::Relaxed,
            );
            SHADOW_STAGE.store(4, Ordering::Relaxed);

            let commands = match self.actuators.command_for_demand(
                demand,
                ActuatorPairOperatingPoint {
                    drive_speed_rad_per_s: drive_rate,
                    reaction_speed_rad_per_s: reaction_rate,
                },
            ) {
                Ok(value) => value,
                Err(_) => return,
            };
            SHADOW_DRIVE_COMMAND_PERMILLE
                .store(scale_milli(commands.drive.command.get()), Ordering::Relaxed);
            SHADOW_REACTION_COMMAND_PERMILLE.store(
                scale_milli(commands.reaction.command.get()),
                Ordering::Relaxed,
            );
            SHADOW_STAGE.store(5, Ordering::Relaxed);

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
            SHADOW_AUTHORITY_REASONS.store(
                u32::from(outcome.decision().reasons.bits()),
                Ordering::Relaxed,
            );
            SHADOW_STAGE.store(6, Ordering::Relaxed);

            if let Some(authorized) = outcome.authorized() {
                let _ = self.sink.apply_authorized(authorized);
                SHADOW_AUTHORIZED.store(1, Ordering::Relaxed);
            } else {
                let _ = self.sink.revoke();
                SHADOW_AUTHORIZED.store(0, Ordering::Relaxed);
            }

            // The sink above owns only in-memory ElectricalActuation. There is no
            // ActuatorIo implementation backed by TIM3/GPIO in this binary.
            let electrical = self.sink.io_mut().last;
            publish_electrical(electrical);
            SHADOW_STAGE.store(7, Ordering::Relaxed);
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
        shadow: ShadowEngine,
        sample_index: u32,
        bus_ready: bool,
        imu_present: bool,
        imu_configured: bool,
        imu_last_cycle: u32,
        imu_cycle_epoch: u64,
        health_last_cycle: u32,
        health_cycle_epoch: u64,
    }

    #[init]
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
        let gpioa = ctx.device.GPIOA.split(&mut rcc);
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

        let mut health_timer = ctx.device.TIM1.counter_ms(&mut rcc);
        health_timer.start(HEALTH_PERIOD_MS.millis()).unwrap();
        health_timer.listen(TimerEvent::Update);

        let initial_cycle = DWT::cycle_count();
        let started_at_us = u64::from(initial_cycle) / CYCLES_PER_US;
        let timing_limits =
            SensorTimingLimits::new(EXPECTED_PERIOD_US, LATE_AFTER_US, TIMEOUT_AFTER_US).unwrap();

        // Deliberately no TIM3 setup and no PA6/PB1 PWM or PA4/PB11 direction
        // ownership. This is a hard target-composition boundary, not a runtime flag.
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
                shadow: ShadowEngine::new(),
                sample_index: 0,
                bus_ready,
                imu_present,
                imu_configured,
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
        let health = ctx
            .shared
            .imu_timing_monitor
            .lock(|monitor| monitor.poll(now_us));
        SHADOW_TIMING.store(timing_code(health), Ordering::Relaxed);
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
            shadow,
            sample_index,
            bus_ready,
            imu_present,
            imu_configured,
            imu_last_cycle,
            imu_cycle_epoch
        ]
    )]
    fn imu_data_ready(mut ctx: imu_data_ready::Context) {
        let critical_started = DWT::cycle_count();
        let acquisition_started_us =
            capture_timestamp_us(ctx.local.imu_last_cycle, ctx.local.imu_cycle_epoch);
        ctx.local.imu_int.clear_interrupt_pending_bit();

        let timing = ctx
            .shared
            .imu_timing_monitor
            .lock(|monitor| monitor.on_event(acquisition_started_us));
        SHADOW_TIMING.store(timing_code(timing), Ordering::Relaxed);

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
        if timing == SensorTimingHealth::Healthy && imu_quality.contains(MeasurementQuality::IO_OK)
        {
            imu_quality |= MeasurementQuality::TIMING_VALID;
        }
        let read_completed_at_us = TimestampEvidence::Known(capture_timestamp_us(
            ctx.local.imu_last_cycle,
            ctx.local.imu_cycle_epoch,
        ));

        let encoder_quality = MeasurementQuality::AVAILABLE
            | MeasurementQuality::IO_OK
            | MeasurementQuality::TIMING_VALID;
        let encoder_1_count = ctx.local.encoder_1.count();
        let encoder_1_at_us = TimestampEvidence::Known(capture_timestamp_us(
            ctx.local.imu_last_cycle,
            ctx.local.imu_cycle_epoch,
        ));
        let encoder_2_count = ctx.local.encoder_2.count();
        let encoder_2_at_us = TimestampEvidence::Known(capture_timestamp_us(
            ctx.local.imu_last_cycle,
            ctx.local.imu_cycle_epoch,
        ));

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
        acquisition_status |= match timing {
            SensorTimingHealth::Healthy => AcquisitionStatus::IMU_TIMING_HEALTHY,
            SensorTimingHealth::Late => AcquisitionStatus::IMU_TIMING_LATE,
            SensorTimingHealth::Timeout => AcquisitionStatus::IMU_TIMING_TIMEOUT,
            SensorTimingHealth::Startup => AcquisitionStatus::NONE,
        };

        let acquisition_completed_us =
            capture_timestamp_us(ctx.local.imu_last_cycle, ctx.local.imu_cycle_epoch);
        let raw = RawObservation {
            sample_index: *ctx.local.sample_index,
            acquisition_started_us,
            acquisition_completed_us,
            imu: RawImuObservation {
                source_sample_at_us: TimestampEvidence::Unknown,
                read_started_at_us,
                read_completed_at_us,
                accel_raw: sample.accel,
                temperature_raw: sample.temperature,
                gyro_raw: sample.gyro,
                quality: imu_quality,
            },
            encoders: [
                RawEncoderObservation {
                    captured_at_us: encoder_1_at_us,
                    count: encoder_1_count,
                    quality: encoder_quality,
                },
                RawEncoderObservation {
                    captured_at_us: encoder_2_at_us,
                    count: encoder_2_count,
                    quality: encoder_quality,
                },
            ],
            battery: Default::default(),
            acquisition_status,
        };

        SHADOW_SAMPLE_INDEX.store(*ctx.local.sample_index, Ordering::Relaxed);
        *ctx.local.sample_index = ctx.local.sample_index.wrapping_add(1);
        ctx.local.shadow.step(raw, timing);
        SHADOW_CRITICAL_PATH_CYCLES.store(
            DWT::cycle_count().wrapping_sub(critical_started),
            Ordering::Relaxed,
        );
    }

    fn mpu_config() -> MpuConfig {
        MpuConfig {
            gyro_range: GyroRange::Dps1000,
            accel_range: AccelRange::G4,
            dlpf: Dlpf::Config3,
            sample_rate_hz: CONTROL_HZ,
            data_ready_interrupt: true,
        }
    }

    fn publish_electrical(electrical: ElectricalActuation) {
        SHADOW_DRIVE_PWM_HIGH_PERMILLE.store(
            scale_milli(electrical.drive.pwm_line_high_fraction),
            Ordering::Relaxed,
        );
        SHADOW_REACTION_PWM_HIGH_PERMILLE.store(
            scale_milli(electrical.reaction.pwm_line_high_fraction),
            Ordering::Relaxed,
        );
        let direction_bits = u32::from(electrical.drive.direction_high)
            | (u32::from(electrical.reaction.direction_high) << 1);
        SHADOW_DIRECTION_BITS.store(direction_bits, Ordering::Relaxed);
    }

    fn timing_code(timing: SensorTimingHealth) -> u32 {
        match timing {
            SensorTimingHealth::Startup => 0,
            SensorTimingHealth::Healthy => 1,
            SensorTimingHealth::Late => 2,
            SensorTimingHealth::Timeout => 3,
        }
    }

    fn scale_milli(value: f32) -> i32 {
        let scaled = value * 1_000.0;
        if scaled >= i32::MAX as f32 {
            i32::MAX
        } else if scaled <= i32::MIN as f32 {
            i32::MIN
        } else {
            scaled as i32
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

#![no_std]
#![no_main]
#![deny(unsafe_code)]

use panic_halt as _;

/// Canonical non-actuating runtime integration target.
///
/// This binary materializes the 200 Hz inner stack, 100 Hz outer velocity loop,
/// Supervisor watchdog/fault/state orchestration, and 100 Hz semantic runtime
/// recording. It intentionally owns no TIM3 motor PWM channels and no motor DIR
/// GPIO, so closed-loop computation cannot reach the physical motor pins.
///
/// Sensor projection, observer/controller design, encoder scale, outer-loop
/// gains, actuator parameters, and reaction-wheel limits remain explicit
/// synthetic commissioning fixtures until physical calibration/correlation is
/// performed. The executable proves causality and ownership, not physical
/// controller validity.
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
        ActuatorPairCommand, ActuatorPairModel, ActuatorPairOperatingPoint, ActuatorParameters,
        StaticActuatorModel,
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
    use swp_robot_domain::{AngularRateRadPerSec, GeneralizedDemand, StateValidity};
    use swp_runtime_observation_record::{
        RUNTIME_OBSERVATION_RECORD_LEN, RecordedRuntimeObservation, RuntimeObservation,
    };
    use swp_runtime_state::{
        AuthorityContext, OperatingState, ReactionWheelAuthority, ReactionWheelSpeedLimits,
        RuntimeAuthority, SensorTimingHealth, SensorTimingLimits, SensorTimingMonitor,
    };
    use swp_runtime_supervisor::{
        ControlWatchdog, ControlWatchdogHealth, RuntimeFaults, RuntimeSupervisor,
    };
    use swp_sensor_calibration::scale_mpu6050;
    use swp_software_i2c::SoftwareI2c;
    use swp_state_estimator::{
        EstimatorMeasurement, LinearObserver, MeasurementMask, ObserverDesign, ObserverGain,
    };
    use swp_state_feedback::{LqrController, StateFeedbackGain};
    use swp_velocity_loop::{
        VelocityIntegratorUpdate, VelocityLoop, VelocityLoopParameters, VelocityTarget,
    };

    const CPU_HZ: u64 = 72_000_000;
    const CYCLES_PER_US: u64 = CPU_HZ / 1_000_000;
    const INNER_HZ: u16 = 200;
    const OUTER_HZ: u16 = 100;
    const EXPECTED_PERIOD_US: u32 = 5_000;
    const LATE_AFTER_US: u32 = 7_500;
    const TIMEOUT_AFTER_US: u32 = 15_000;
    const CONTROL_WATCHDOG_TIMEOUT_US: u32 = 15_000;
    const HEALTH_PERIOD_MS: u32 = 1;
    const I2C_HALF_PERIOD_NS: u32 = 1_250;
    const OUTER_DECIMATION: u8 = 2;
    const RECORD_DECIMATION: u8 = 2;
    const RUNTIME_RECORD_CAPACITY: usize = 4;
    const SYNTHETIC_COUNTS_PER_REVOLUTION: f32 = 4_096.0;
    const SYNTHETIC_RAD_PER_COUNT: f32 = TAU / SYNTHETIC_COUNTS_PER_REVOLUTION;
    const SHADOW_VELOCITY_TARGET_M_PER_S: f32 = 0.0;

    type ImuBus = SoftwareI2c<PB8<Output<OpenDrain>>, PB9<Output<OpenDrain>>, SysDelay>;
    type Imu = Mpu6050<ImuBus>;
    type ImuInt = PC13<Input<Floating>>;
    type Encoder1 = Qei<pac::TIM2>;
    type Encoder2 = Qei<pac::TIM4>;
    type HealthTimer = CounterMs<pac::TIM1>;

    static SHADOW_SAMPLE_INDEX: AtomicU32 = AtomicU32::new(0);
    static SHADOW_STAGE: AtomicU32 = AtomicU32::new(0);
    static SHADOW_TIMING: AtomicU32 = AtomicU32::new(0);
    static SHADOW_WATCHDOG: AtomicU32 = AtomicU32::new(0);
    static SHADOW_OPERATING_STATE: AtomicU32 = AtomicU32::new(0);
    static SHADOW_RUNTIME_FAULTS: AtomicU32 = AtomicU32::new(0);
    static SHADOW_AUTHORITY_REASONS: AtomicU32 = AtomicU32::new(0);
    static SHADOW_AUTHORIZED: AtomicU32 = AtomicU32::new(0);
    static SHADOW_DRIVE_DEMAND_MNM: AtomicI32 = AtomicI32::new(0);
    static SHADOW_REACTION_DEMAND_MNM: AtomicI32 = AtomicI32::new(0);
    static SHADOW_DRIVE_COMMAND_PERMILLE: AtomicI32 = AtomicI32::new(0);
    static SHADOW_REACTION_COMMAND_PERMILLE: AtomicI32 = AtomicI32::new(0);
    static SHADOW_PITCH_REFERENCE_MRAD: AtomicI32 = AtomicI32::new(0);
    static SHADOW_DRIVE_PWM_HIGH_PERMILLE: AtomicI32 = AtomicI32::new(1_000);
    static SHADOW_REACTION_PWM_HIGH_PERMILLE: AtomicI32 = AtomicI32::new(1_000);
    static SHADOW_DIRECTION_BITS: AtomicU32 = AtomicU32::new(0);
    static SHADOW_CRITICAL_PATH_CYCLES: AtomicU32 = AtomicU32::new(0);
    static SHADOW_RUNTIME_RECORD_COUNT: AtomicU32 = AtomicU32::new(0);
    static SHADOW_RUNTIME_RECORD_OVERWRITTEN: AtomicU32 = AtomicU32::new(0);

    #[derive(Clone, Copy, Default)]
    struct SyntheticEncoderTracker {
        primed: bool,
        previous_drive_count: u16,
        previous_reaction_count: u16,
        previous_at_us: u64,
        drive_unwrapped_counts: i64,
    }

    impl SyntheticEncoderTracker {
        fn observe(&mut self, drive_count: u16, reaction_count: u16, captured_at_us: u64) -> (f32, f32, f32) {
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
            let reaction_delta = reaction_count.wrapping_sub(self.previous_reaction_count) as i16 as i32;
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

    struct ShadowElectricalIo { last: ElectricalActuation }
    impl ShadowElectricalIo {
        const fn new() -> Self { Self { last: ElectricalActuation::zero_effort() } }
    }
    impl ActuatorIo<ElectricalActuation> for ShadowElectricalIo {
        type Error = Infallible;
        fn write_frame(&mut self, frame: ElectricalActuation) -> Result<(), Self::Error> {
            self.last = frame;
            Ok(())
        }
    }

    #[derive(Clone, Copy)]
    struct PreparedControlStep {
        estimate: ReducedBalanceState,
        estimate_validity: StateValidity,
        reference: ReducedBalanceState,
        demand: GeneralizedDemand,
        commands: ActuatorPairCommand,
        reaction_wheel_authority: ReactionWheelAuthority,
    }

    struct ShadowEngine {
        observer: LinearObserver,
        controller: LqrController,
        outer_loop: VelocityLoop,
        actuators: ActuatorPairModel,
        reaction_limits: ReactionWheelSpeedLimits,
        encoders: SyntheticEncoderTracker,
        sink: OneV2PwmDirAdapter<ShadowElectricalIo>,
        reference: ReducedBalanceState,
        outer_divider: u8,
        hold_outer_integrator: bool,
    }

    impl ShadowEngine {
        fn new() -> Self {
            let mut a_d = [[0.0; REDUCED_BALANCE_STATE_COUNT]; REDUCED_BALANCE_STATE_COUNT];
            for (index, row) in a_d.iter_mut().enumerate() { row[index] = 1.0; }
            let plant = DiscreteLinearPlant {
                sample_period_s: 1.0 / INNER_HZ as f32,
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
            ).expect("synthetic observer design");
            let observer = LinearObserver::new(design, ReducedBalanceState::default())
                .expect("synthetic observer initial state");
            let mut k = [[0.0; REDUCED_BALANCE_STATE_COUNT]; 2];
            k[0][2] = 0.8;
            k[0][3] = 0.05;
            k[1][4] = 0.8;
            k[1][5] = 0.05;
            let controller = LqrController::new(StateFeedbackGain::new(k).expect("synthetic state-feedback gain"));
            let outer_loop = VelocityLoop::new(
                VelocityLoopParameters::new(0.08, 0.02, 0.12, 1.0).expect("synthetic outer-loop parameters"),
                1.0 / OUTER_HZ as f32,
            ).expect("100 Hz outer loop");
            let actuator_parameters = ActuatorParameters::new(10.0, 0.05, 0.001, 0.002, 0.1)
                .expect("synthetic actuator parameters");
            let actuator = StaticActuatorModel::new(actuator_parameters).expect("synthetic actuator model");
            Self {
                observer,
                controller,
                outer_loop,
                actuators: ActuatorPairModel { drive: actuator, reaction: actuator },
                reaction_limits: ReactionWheelSpeedLimits::new(1_000.0, 2_000.0).expect("synthetic reaction-wheel limits"),
                encoders: SyntheticEncoderTracker::default(),
                sink: OneV2PwmDirAdapter::new(ShadowElectricalIo::new()),
                reference: ReducedBalanceState::default(),
                outer_divider: 0,
                hold_outer_integrator: true,
            }
        }

        fn prepare(&mut self, raw: RawObservation) -> Option<PreparedControlStep> {
            SHADOW_STAGE.store(1, Ordering::Relaxed);
            let scaled = scale_mpu6050(raw.imu, mpu_config()).ok()?;
            let captured_at_us = raw.acquisition_started_us;
            let (drive_angle, drive_rate, reaction_rate) = self.encoders.observe(
                raw.encoders[1].count,
                raw.encoders[0].count,
                captured_at_us,
            );
            let mut values = [0.0; UPRIGHT_MEASUREMENT_COUNT];
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
                true,
            );
            SHADOW_STAGE.store(2, Ordering::Relaxed);
            let zero_input = ReferencePlantInput::default();
            let estimate = self.observer.step(zero_input, zero_input, measurement).ok()?;
            SHADOW_STAGE.store(3, Ordering::Relaxed);

            self.outer_divider = self.outer_divider.wrapping_add(1);
            if self.outer_divider >= OUTER_DECIMATION {
                self.outer_divider = 0;
                let update = if self.hold_outer_integrator { VelocityIntegratorUpdate::Hold } else { VelocityIntegratorUpdate::Integrate };
                let output = self.outer_loop.update(
                    estimate.state,
                    VelocityTarget { forward_velocity_m_per_s: SHADOW_VELOCITY_TARGET_M_PER_S },
                    update,
                ).ok()?;
                self.reference = output.reference;
                SHADOW_PITCH_REFERENCE_MRAD.store(scale_milli(self.reference.pitch_rad), Ordering::Relaxed);
            }

            let demand = self.controller.command(estimate.state, self.reference, GeneralizedDemand::default()).ok()?;
            SHADOW_DRIVE_DEMAND_MNM.store(scale_milli(demand.drive_wheel_torque.0), Ordering::Relaxed);
            SHADOW_REACTION_DEMAND_MNM.store(scale_milli(demand.reaction_wheel_torque.0), Ordering::Relaxed);
            SHADOW_STAGE.store(4, Ordering::Relaxed);
            let commands = self.actuators.command_for_demand(
                demand,
                ActuatorPairOperatingPoint { drive_speed_rad_per_s: drive_rate, reaction_speed_rad_per_s: reaction_rate },
            ).ok()?;
            SHADOW_DRIVE_COMMAND_PERMILLE.store(scale_milli(commands.drive.command.get()), Ordering::Relaxed);
            SHADOW_REACTION_COMMAND_PERMILLE.store(scale_milli(commands.reaction.command.get()), Ordering::Relaxed);
            SHADOW_STAGE.store(5, Ordering::Relaxed);
            let reaction_wheel_authority = self.reaction_limits.classify(AngularRateRadPerSec(
                estimate.state.reaction_wheel_rate_rad_per_s,
            ));
            Some(PreparedControlStep {
                estimate: estimate.state,
                estimate_validity: estimate.validity,
                reference: self.reference,
                demand,
                commands,
                reaction_wheel_authority,
            })
        }

        fn apply_shadow_authority(
            &mut self,
            operating_state: OperatingState,
            timing: SensorTimingHealth,
            step: PreparedControlStep,
        ) -> (u16, bool) {
            let outcome = RuntimeAuthority::evaluate(
                AuthorityContext {
                    operating_state,
                    timing,
                    estimate_validity: step.estimate_validity,
                    reaction_wheel_authority: step.reaction_wheel_authority,
                },
                step.commands,
            );
            let decision = outcome.decision();
            self.hold_outer_integrator = decision.hold_integrator;
            SHADOW_AUTHORITY_REASONS.store(u32::from(decision.reasons.bits()), Ordering::Relaxed);
            SHADOW_STAGE.store(6, Ordering::Relaxed);
            let authorized = if let Some(authorized) = outcome.authorized() {
                let _ = self.sink.apply_authorized(authorized);
                true
            } else {
                let _ = self.sink.revoke();
                false
            };
            SHADOW_AUTHORIZED.store(authorized as u32, Ordering::Relaxed);
            publish_electrical(self.sink.io_mut().last);
            SHADOW_STAGE.store(7, Ordering::Relaxed);
            (decision.reasons.bits(), authorized)
        }
    }

    struct RuntimeRecordBuffer {
        records: [[u8; RUNTIME_OBSERVATION_RECORD_LEN]; RUNTIME_RECORD_CAPACITY],
        next: usize,
        count: usize,
        overwritten: u16,
    }
    impl RuntimeRecordBuffer {
        const fn new() -> Self {
            Self {
                records: [[0; RUNTIME_OBSERVATION_RECORD_LEN]; RUNTIME_RECORD_CAPACITY],
                next: 0,
                count: 0,
                overwritten: 0,
            }
        }
        fn push(&mut self, mut record: RecordedRuntimeObservation) {
            if self.count == RUNTIME_RECORD_CAPACITY {
                self.overwritten = self.overwritten.saturating_add(1);
            } else {
                self.count += 1;
            }
            record.dropped_records = self.overwritten;
            self.records[self.next] = record.encode();
            self.next = (self.next + 1) % RUNTIME_RECORD_CAPACITY;
            SHADOW_RUNTIME_RECORD_COUNT.store(self.count as u32, Ordering::Relaxed);
            SHADOW_RUNTIME_RECORD_OVERWRITTEN.store(u32::from(self.overwritten), Ordering::Relaxed);
        }
    }

    #[shared]
    struct Shared {
        imu_timing_monitor: SensorTimingMonitor,
        control_watchdog: ControlWatchdog,
        runtime_supervisor: RuntimeSupervisor,
    }

    #[local]
    struct Local {
        imu: Imu,
        imu_int: ImuInt,
        encoder_1: Encoder1,
        encoder_2: Encoder2,
        health_timer: HealthTimer,
        shadow: ShadowEngine,
        runtime_records: RuntimeRecordBuffer,
        record_divider: u8,
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
            rcc::Config::hse(8.MHz()).sysclk(72.MHz()).pclk1(36.MHz()).pclk2(72.MHz()),
            &mut flash.acr,
        );
        let mut afio = ctx.device.AFIO.constrain(&mut rcc);
        let gpioa = ctx.device.GPIOA.split(&mut rcc);
        let mut gpiob = ctx.device.GPIOB.split(&mut rcc);
        let mut gpioc = ctx.device.GPIOC.split(&mut rcc);
        let sda = gpiob.pb8.into_open_drain_output_with_state(&mut gpiob.crh, PinState::High);
        let scl = gpiob.pb9.into_open_drain_output_with_state(&mut gpiob.crh, PinState::High);
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
        let encoder_1 = Timer::new(ctx.device.TIM2, &mut rcc).qei((gpioa.pa0, gpioa.pa1), QeiOptions::default());
        let encoder_2 = Timer::new(ctx.device.TIM4, &mut rcc).qei((gpiob.pb6, gpiob.pb7), QeiOptions::default());
        let mut health_timer = ctx.device.TIM1.counter_ms(&mut rcc);
        health_timer.start(HEALTH_PERIOD_MS.millis()).unwrap();
        health_timer.listen(TimerEvent::Update);
        let initial_cycle = DWT::cycle_count();
        let started_at_us = u64::from(initial_cycle) / CYCLES_PER_US;
        let timing_limits = SensorTimingLimits::new(EXPECTED_PERIOD_US, LATE_AFTER_US, TIMEOUT_AFTER_US).unwrap();
        let control_watchdog = ControlWatchdog::new(CONTROL_WATCHDOG_TIMEOUT_US, started_at_us).unwrap();
        let mut runtime_supervisor = RuntimeSupervisor::new();
        let _ = runtime_supervisor.boot_complete();
        if imu_configured {
            let _ = runtime_supervisor.hardware_check_passed();
            let _ = runtime_supervisor.request_balance();
        }
        publish_runtime(runtime_supervisor);
        (
            Shared {
                imu_timing_monitor: SensorTimingMonitor::new(timing_limits, started_at_us),
                control_watchdog,
                runtime_supervisor,
            },
            Local {
                imu,
                imu_int,
                encoder_1,
                encoder_2,
                health_timer,
                shadow: ShadowEngine::new(),
                runtime_records: RuntimeRecordBuffer::new(),
                record_divider: 0,
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
        loop { cortex_m::asm::wfi(); }
    }

    #[task(
        binds = TIM1_UP,
        priority = 3,
        shared = [imu_timing_monitor, control_watchdog, runtime_supervisor],
        local = [health_timer, health_last_cycle, health_cycle_epoch]
    )]
    fn timing_health(mut ctx: timing_health::Context) {
        ctx.local.health_timer.clear_interrupt(TimerEvent::Update);
        let now_us = capture_timestamp_us(ctx.local.health_last_cycle, ctx.local.health_cycle_epoch);
        let timing = ctx.shared.imu_timing_monitor.lock(|monitor| monitor.poll(now_us));
        let watchdog = ctx.shared.control_watchdog.lock(|watchdog| watchdog.poll(now_us));
        ctx.shared.runtime_supervisor.lock(|runtime| {
            runtime.observe_independent_health(timing, watchdog);
            publish_runtime(*runtime);
        });
        SHADOW_TIMING.store(timing_code(timing), Ordering::Relaxed);
        SHADOW_WATCHDOG.store(watchdog_code(watchdog), Ordering::Relaxed);
    }

    #[task(
        binds = EXTI15_10,
        priority = 2,
        shared = [imu_timing_monitor, control_watchdog, runtime_supervisor],
        local = [imu, imu_int, encoder_1, encoder_2, shadow, runtime_records, record_divider, sample_index, bus_ready, imu_present, imu_configured, imu_last_cycle, imu_cycle_epoch]
    )]
    fn imu_data_ready(mut ctx: imu_data_ready::Context) {
        let critical_started = DWT::cycle_count();
        let acquisition_started_us = capture_timestamp_us(ctx.local.imu_last_cycle, ctx.local.imu_cycle_epoch);
        ctx.local.imu_int.clear_interrupt_pending_bit();
        let timing = ctx.shared.imu_timing_monitor.lock(|monitor| monitor.on_event(acquisition_started_us));
        SHADOW_TIMING.store(timing_code(timing), Ordering::Relaxed);
        let read_started_at_us = TimestampEvidence::Known(capture_timestamp_us(ctx.local.imu_last_cycle, ctx.local.imu_cycle_epoch));
        let (sample, mut imu_quality) = match ctx.local.imu.read_raw() {
            Ok(value) => (value, MeasurementQuality::AVAILABLE | MeasurementQuality::IO_OK | MeasurementQuality::FRESHNESS_VERIFIED),
            Err(_) => (RawSample::default(), MeasurementQuality::IO_ERROR),
        };
        if timing == SensorTimingHealth::Healthy && imu_quality.contains(MeasurementQuality::IO_OK) {
            imu_quality |= MeasurementQuality::TIMING_VALID;
        }
        let read_completed_at_us = TimestampEvidence::Known(capture_timestamp_us(ctx.local.imu_last_cycle, ctx.local.imu_cycle_epoch));
        let encoder_quality = MeasurementQuality::AVAILABLE | MeasurementQuality::IO_OK | MeasurementQuality::TIMING_VALID;
        let encoder_1_count = ctx.local.encoder_1.count();
        let encoder_1_at_us = TimestampEvidence::Known(capture_timestamp_us(ctx.local.imu_last_cycle, ctx.local.imu_cycle_epoch));
        let encoder_2_count = ctx.local.encoder_2.count();
        let encoder_2_at_us = TimestampEvidence::Known(capture_timestamp_us(ctx.local.imu_last_cycle, ctx.local.imu_cycle_epoch));
        let mut acquisition_status = AcquisitionStatus::NONE;
        if *ctx.local.bus_ready { acquisition_status |= AcquisitionStatus::BUS_READY; }
        if *ctx.local.imu_present { acquisition_status |= AcquisitionStatus::IMU_PRESENT; }
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
        let acquisition_completed_us = capture_timestamp_us(ctx.local.imu_last_cycle, ctx.local.imu_cycle_epoch);
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
                RawEncoderObservation { captured_at_us: encoder_1_at_us, count: encoder_1_count, quality: encoder_quality },
                RawEncoderObservation { captured_at_us: encoder_2_at_us, count: encoder_2_count, quality: encoder_quality },
            ],
            battery: Default::default(),
            acquisition_status,
        };
        SHADOW_SAMPLE_INDEX.store(*ctx.local.sample_index, Ordering::Relaxed);
        let current_sample = *ctx.local.sample_index;
        *ctx.local.sample_index = ctx.local.sample_index.wrapping_add(1);
        if timing != SensorTimingHealth::Healthy {
            SHADOW_CRITICAL_PATH_CYCLES.store(DWT::cycle_count().wrapping_sub(critical_started), Ordering::Relaxed);
            return;
        }
        let Some(step) = ctx.local.shadow.prepare(raw) else {
            ctx.shared.runtime_supervisor.lock(|runtime| {
                runtime.latch_fault(RuntimeFaults::CONTROL_NUMERICAL_FAULT);
                publish_runtime(*runtime);
            });
            return;
        };
        let completed_at_us = capture_timestamp_us(ctx.local.imu_last_cycle, ctx.local.imu_cycle_epoch);
        let watchdog = ctx.shared.control_watchdog.lock(|watchdog| watchdog.observe_control_completion(completed_at_us));
        SHADOW_WATCHDOG.store(watchdog_code(watchdog), Ordering::Relaxed);
        let operating_state = ctx.shared.runtime_supervisor.lock(|runtime| {
            if runtime.state() == OperatingState::CaptureWindow
                && step.estimate_validity == StateValidity::Valid
                && watchdog == ControlWatchdogHealth::Healthy
            {
                let _ = runtime.capture_ready();
            }
            runtime.observe_control_health(step.estimate_validity, step.reaction_wheel_authority);
            publish_runtime(*runtime);
            runtime.state()
        });
        let (authority_reasons, authorized) = ctx.local.shadow.apply_shadow_authority(operating_state, timing, step);
        *ctx.local.record_divider = ctx.local.record_divider.wrapping_add(1);
        if *ctx.local.record_divider >= RECORD_DECIMATION {
            *ctx.local.record_divider = 0;
            let faults = ctx.shared.runtime_supervisor.lock(|runtime| runtime.faults());
            ctx.local.runtime_records.push(RecordedRuntimeObservation {
                observation: RuntimeObservation {
                    sample_index: current_sample,
                    timestamp_us: acquisition_started_us,
                    estimated_state: step.estimate,
                    reference: step.reference,
                    demand: step.demand,
                    bounded_commands: step.commands,
                    operating_state,
                    timing,
                    estimate_validity: step.estimate_validity,
                    watchdog,
                    authority_reasons,
                    runtime_faults: faults,
                    authorized,
                    outer_target_velocity_m_per_s: SHADOW_VELOCITY_TARGET_M_PER_S,
                },
                dropped_records: ctx.local.runtime_records.overwritten,
            });
        }
        SHADOW_CRITICAL_PATH_CYCLES.store(DWT::cycle_count().wrapping_sub(critical_started), Ordering::Relaxed);
    }

    fn mpu_config() -> MpuConfig {
        MpuConfig {
            gyro_range: GyroRange::Dps1000,
            accel_range: AccelRange::G4,
            dlpf: Dlpf::Config3,
            sample_rate_hz: INNER_HZ,
            data_ready_interrupt: true,
        }
    }

    fn publish_electrical(electrical: ElectricalActuation) {
        SHADOW_DRIVE_PWM_HIGH_PERMILLE.store(scale_milli(electrical.drive.pwm_line_high_fraction), Ordering::Relaxed);
        SHADOW_REACTION_PWM_HIGH_PERMILLE.store(scale_milli(electrical.reaction.pwm_line_high_fraction), Ordering::Relaxed);
        let direction_bits = (electrical.drive.direction_high as u32) | ((electrical.reaction.direction_high as u32) << 1);
        SHADOW_DIRECTION_BITS.store(direction_bits, Ordering::Relaxed);
    }
    fn publish_runtime(runtime: RuntimeSupervisor) {
        SHADOW_OPERATING_STATE.store(operating_state_code(runtime.state()), Ordering::Relaxed);
        SHADOW_RUNTIME_FAULTS.store(u32::from(runtime.faults().bits()), Ordering::Relaxed);
    }
    fn operating_state_code(state: OperatingState) -> u32 {
        match state {
            OperatingState::Boot => 0,
            OperatingState::HardwareCheck => 1,
            OperatingState::Standby => 2,
            OperatingState::CaptureWindow => 3,
            OperatingState::Balancing => 4,
            OperatingState::MomentumLimited => 5,
            OperatingState::Fault => 6,
        }
    }
    fn timing_code(timing: SensorTimingHealth) -> u32 {
        match timing {
            SensorTimingHealth::Startup => 0,
            SensorTimingHealth::Healthy => 1,
            SensorTimingHealth::Late => 2,
            SensorTimingHealth::Timeout => 3,
        }
    }
    fn watchdog_code(watchdog: ControlWatchdogHealth) -> u32 {
        match watchdog {
            ControlWatchdogHealth::Startup => 0,
            ControlWatchdogHealth::Healthy => 1,
            ControlWatchdogHealth::Timeout => 2,
        }
    }
    fn scale_milli(value: f32) -> i32 {
        let scaled = value * 1_000.0;
        if scaled >= i32::MAX as f32 { i32::MAX } else if scaled <= i32::MIN as f32 { i32::MIN } else { scaled as i32 }
    }
    fn capture_timestamp_us(last_cycle: &mut u32, cycle_epoch: &mut u64) -> u64 {
        let now = DWT::cycle_count();
        if now < *last_cycle { *cycle_epoch += 1_u64 << 32; }
        *last_cycle = now;
        (*cycle_epoch + u64::from(now)) / CYCLES_PER_US
    }
}

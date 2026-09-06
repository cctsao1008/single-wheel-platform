#![no_std]
#![no_main]
#![deny(unsafe_code)]

use panic_halt as _;

/// Non-actuating Communications/UI framework target.
///
/// A 200 Hz target-owned cadence publishes the latest telemetry snapshot at
/// 50 Hz and the latest OLED status view at 10 Hz. Both outputs are immediate,
/// drop-on-busy shadow transports with no queue and no catch-up replay. This
/// binary owns no motor PWM/DIR resources and does not claim ECB02/OLED hardware
/// integration or verification.
#[rtic::app(device = stm32f1xx_hal::pac)]
mod app {
    use core::{
        convert::Infallible,
        sync::atomic::{AtomicU32, Ordering},
    };

    use stm32f1xx_hal::{
        pac,
        prelude::*,
        rcc,
        timer::{CounterMs, Event as TimerEvent},
    };
    use swp_ecb02::{ByteTransport, ByteWriteOutcome, Ecb02TelemetryTransport};
    use swp_oled_ui::{
        DisplayPresentOutcome, OledDisplay, OledPresenter, OledRenderer, OledTextFrame,
    };
    use swp_status_view::{HealthState, LinkState, RuntimeMode, StatusPage, StatusView};
    use swp_telemetry::{TelemetryPublisher, TelemetrySnapshot};

    const BASE_PERIOD_MS: u32 = 5;
    const TELEMETRY_DECIMATION: u32 = 4;
    const OLED_DECIMATION: u32 = 20;

    type CadenceTimer = CounterMs<pac::TIM1>;
    type ShadowTelemetry = TelemetryPublisher<Ecb02TelemetryTransport<ShadowBytes>>;
    type ShadowOled = OledPresenter<ShadowDisplay>;

    static SHADOW_BASE_TICKS: AtomicU32 = AtomicU32::new(0);
    static SHADOW_TELEMETRY_OPPORTUNITIES: AtomicU32 = AtomicU32::new(0);
    static SHADOW_TELEMETRY_SENT: AtomicU32 = AtomicU32::new(0);
    static SHADOW_TELEMETRY_DROPPED: AtomicU32 = AtomicU32::new(0);
    static SHADOW_OLED_OPPORTUNITIES: AtomicU32 = AtomicU32::new(0);
    static SHADOW_OLED_PRESENTED: AtomicU32 = AtomicU32::new(0);
    static SHADOW_OLED_DROPPED: AtomicU32 = AtomicU32::new(0);

    struct ShadowBytes {
        writes: u32,
        last_len: usize,
    }

    impl ShadowBytes {
        const fn new() -> Self {
            Self {
                writes: 0,
                last_len: 0,
            }
        }
    }

    impl ByteTransport for ShadowBytes {
        type Error = Infallible;

        fn try_write(&mut self, bytes: &[u8]) -> Result<ByteWriteOutcome, Self::Error> {
            self.writes = self.writes.wrapping_add(1);
            self.last_len = bytes.len();
            Ok(ByteWriteOutcome::Written)
        }
    }

    struct ShadowDisplay {
        frames: u32,
        last: OledTextFrame,
    }

    impl ShadowDisplay {
        const fn new() -> Self {
            Self {
                frames: 0,
                last: OledTextFrame::blank(),
            }
        }
    }

    impl OledDisplay for ShadowDisplay {
        type Error = Infallible;

        fn try_present(
            &mut self,
            frame: &OledTextFrame,
        ) -> Result<DisplayPresentOutcome, Self::Error> {
            self.frames = self.frames.wrapping_add(1);
            self.last = *frame;
            Ok(DisplayPresentOutcome::Presented)
        }
    }

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        cadence_timer: CadenceTimer,
        telemetry: ShadowTelemetry,
        oled: ShadowOled,
        page: StatusPage,
        tick: u32,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let mut flash = ctx.device.FLASH.constrain();
        let mut rcc = ctx.device.RCC.freeze(
            rcc::Config::hse(8.MHz())
                .sysclk(72.MHz())
                .pclk1(36.MHz())
                .pclk2(72.MHz()),
            &mut flash.acr,
        );
        let mut cadence_timer = ctx.device.TIM1.counter_ms(&mut rcc);
        cadence_timer.start(BASE_PERIOD_MS.millis()).unwrap();
        cadence_timer.listen(TimerEvent::Update);

        let telemetry = TelemetryPublisher::new(Ecb02TelemetryTransport::new(ShadowBytes::new()));
        let oled = OledPresenter::new(ShadowDisplay::new());

        (
            Shared {},
            Local {
                cadence_timer,
                telemetry,
                oled,
                page: StatusPage::Overview,
                tick: 0,
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
        priority = 1,
        local = [cadence_timer, telemetry, oled, page, tick]
    )]
    fn output_cadence(ctx: output_cadence::Context) {
        ctx.local.cadence_timer.clear_interrupt(TimerEvent::Update);
        *ctx.local.tick = ctx.local.tick.wrapping_add(1);
        let tick = *ctx.local.tick;
        SHADOW_BASE_TICKS.store(tick, Ordering::Relaxed);

        let view = status_view();

        if tick % TELEMETRY_DECIMATION == 0 {
            let _ = ctx
                .local
                .telemetry
                .publish_latest(telemetry_snapshot(tick, view));
            let stats = ctx.local.telemetry.stats();
            SHADOW_TELEMETRY_OPPORTUNITIES.store(stats.opportunities, Ordering::Relaxed);
            SHADOW_TELEMETRY_SENT.store(stats.sent, Ordering::Relaxed);
            SHADOW_TELEMETRY_DROPPED.store(stats.dropped_busy, Ordering::Relaxed);
        }

        if tick % OLED_DECIMATION == 0 {
            let frame = OledRenderer::render(view, *ctx.local.page);
            let _ = ctx.local.oled.present_latest(&frame);
            *ctx.local.page = ctx.local.page.next();
            let stats = ctx.local.oled.stats();
            SHADOW_OLED_OPPORTUNITIES.store(stats.opportunities, Ordering::Relaxed);
            SHADOW_OLED_PRESENTED.store(stats.presented, Ordering::Relaxed);
            SHADOW_OLED_DROPPED.store(stats.dropped_busy, Ordering::Relaxed);
        }
    }

    fn telemetry_snapshot(tick: u32, view: StatusView) -> TelemetrySnapshot {
        TelemetrySnapshot {
            timestamp_us: u64::from(tick) * u64::from(BASE_PERIOD_MS) * 1_000,
            sample_index: tick,
            operating_state: 0,
            timing_health: 1,
            watchdog_health: 1,
            authorized: view.authorized,
            runtime_fault_bits: view.runtime_fault_bits,
            authority_reason_bits: view.authority_reason_bits,
            forward_velocity_mm_per_s: view.forward_velocity_mm_per_s,
            pitch_mrad: view.pitch_mrad,
            drive_demand_mnm: view.drive_demand_mnm,
            reaction_demand_mnm: view.reaction_demand_mnm,
            drive_command_permille: view.drive_command_permille,
            reaction_command_permille: view.reaction_command_permille,
        }
    }

    fn status_view() -> StatusView {
        StatusView {
            runtime_mode: RuntimeMode::Standby,
            timing: HealthState::Ok,
            watchdog: HealthState::Ok,
            ble: LinkState::Unknown,
            authorized: false,
            runtime_fault_bits: 0,
            authority_reason_bits: 0,
            forward_velocity_mm_per_s: 0,
            pitch_mrad: 0,
            drive_demand_mnm: 0,
            reaction_demand_mnm: 0,
            drive_command_permille: 0,
            reaction_command_permille: 0,
        }
    }
}

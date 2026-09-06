#![no_std]
#![forbid(unsafe_code)]

use swp_status_view::{HealthState, LinkState, RuntimeMode, StatusPage, StatusView};

pub const OLED_TEXT_ROWS: usize = 8;
pub const OLED_TEXT_COLUMNS: usize = 21;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OledTextFrame {
    pub rows: [[u8; OLED_TEXT_COLUMNS]; OLED_TEXT_ROWS],
}

impl OledTextFrame {
    pub const fn blank() -> Self {
        Self {
            rows: [[b' '; OLED_TEXT_COLUMNS]; OLED_TEXT_ROWS],
        }
    }

    pub fn row(&self, index: usize) -> &[u8; OLED_TEXT_COLUMNS] {
        &self.rows[index]
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DisplayPresentOutcome {
    Presented,
    Busy,
}

pub trait OledDisplay {
    type Error;

    fn try_present(&mut self, frame: &OledTextFrame) -> Result<DisplayPresentOutcome, Self::Error>;
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct OledPresenterStats {
    pub opportunities: u32,
    pub presented: u32,
    pub dropped_busy: u32,
}

pub struct OledPresenter<D> {
    display: D,
    stats: OledPresenterStats,
}

impl<D> OledPresenter<D>
where
    D: OledDisplay,
{
    pub const fn new(display: D) -> Self {
        Self {
            display,
            stats: OledPresenterStats {
                opportunities: 0,
                presented: 0,
                dropped_busy: 0,
            },
        }
    }

    /// Present only the frame for this UI opportunity; no display backlog exists.
    pub fn present_latest(
        &mut self,
        frame: &OledTextFrame,
    ) -> Result<DisplayPresentOutcome, D::Error> {
        self.stats.opportunities = self.stats.opportunities.wrapping_add(1);
        let outcome = self.display.try_present(frame)?;
        match outcome {
            DisplayPresentOutcome::Presented => {
                self.stats.presented = self.stats.presented.wrapping_add(1)
            }
            DisplayPresentOutcome::Busy => {
                self.stats.dropped_busy = self.stats.dropped_busy.wrapping_add(1)
            }
        }
        Ok(outcome)
    }

    pub const fn stats(&self) -> OledPresenterStats {
        self.stats
    }

    pub fn display(&self) -> &D {
        &self.display
    }

    pub fn display_mut(&mut self) -> &mut D {
        &mut self.display
    }

    pub fn into_display(self) -> D {
        self.display
    }
}

pub struct OledRenderer;

impl OledRenderer {
    pub fn render(view: StatusView, page: StatusPage) -> OledTextFrame {
        let mut frame = OledTextFrame::blank();
        match page {
            StatusPage::Overview => render_overview(&mut frame, view),
            StatusPage::Control => render_control(&mut frame, view),
            StatusPage::Health => render_health(&mut frame, view),
        }
        frame
    }
}

fn render_overview(frame: &mut OledTextFrame, view: StatusView) {
    write_label_value(
        &mut frame.rows[0],
        b"STATE ",
        runtime_mode(view.runtime_mode),
    );
    write_label_value(&mut frame.rows[1], b"TIMING ", health(view.timing));
    write_label_value(&mut frame.rows[2], b"WATCH  ", health(view.watchdog));
    write_label_value(&mut frame.rows[3], b"BLE    ", link(view.ble));
    write_signed(
        &mut frame.rows[4],
        b"VEL mm/s ",
        i32::from(view.forward_velocity_mm_per_s),
    );
    write_signed(&mut frame.rows[5], b"PITCH mr ", i32::from(view.pitch_mrad));
    write_label_value(
        &mut frame.rows[6],
        b"AUTH   ",
        if view.authorized { b"YES" } else { b"NO" },
    );
    write_hex32(&mut frame.rows[7], b"FAULT 0x", view.runtime_fault_bits);
}

fn render_control(frame: &mut OledTextFrame, view: StatusView) {
    write_signed(
        &mut frame.rows[0],
        b"DRV dem ",
        i32::from(view.drive_demand_mnm),
    );
    write_signed(
        &mut frame.rows[1],
        b"REA dem ",
        i32::from(view.reaction_demand_mnm),
    );
    write_signed(
        &mut frame.rows[2],
        b"DRV cmd ",
        i32::from(view.drive_command_permille),
    );
    write_signed(
        &mut frame.rows[3],
        b"REA cmd ",
        i32::from(view.reaction_command_permille),
    );
    write_signed(
        &mut frame.rows[4],
        b"VEL mm/s ",
        i32::from(view.forward_velocity_mm_per_s),
    );
    write_signed(&mut frame.rows[5], b"PITCH mr ", i32::from(view.pitch_mrad));
    write_hex32(&mut frame.rows[6], b"AUTH  0x", view.authority_reason_bits);
    write_hex32(&mut frame.rows[7], b"FAULT 0x", view.runtime_fault_bits);
}

fn render_health(frame: &mut OledTextFrame, view: StatusView) {
    write_label_value(
        &mut frame.rows[0],
        b"STATE ",
        runtime_mode(view.runtime_mode),
    );
    write_label_value(&mut frame.rows[1], b"TIMING ", health(view.timing));
    write_label_value(&mut frame.rows[2], b"WATCH  ", health(view.watchdog));
    write_label_value(&mut frame.rows[3], b"BLE    ", link(view.ble));
    write_label_value(
        &mut frame.rows[4],
        b"AUTH   ",
        if view.authorized { b"YES" } else { b"NO" },
    );
    write_hex32(&mut frame.rows[5], b"AUTH  0x", view.authority_reason_bits);
    write_hex32(&mut frame.rows[6], b"FAULT 0x", view.runtime_fault_bits);
    write_label_value(&mut frame.rows[7], b"UI     ", b"SHADOW");
}

fn runtime_mode(mode: RuntimeMode) -> &'static [u8] {
    match mode {
        RuntimeMode::Boot => b"BOOT",
        RuntimeMode::HardwareCheck => b"HWCHK",
        RuntimeMode::Standby => b"STBY",
        RuntimeMode::CaptureWindow => b"CAPTURE",
        RuntimeMode::Balancing => b"BAL",
        RuntimeMode::MomentumLimited => b"MOM LIM",
        RuntimeMode::Fault => b"FAULT",
    }
}

fn health(state: HealthState) -> &'static [u8] {
    match state {
        HealthState::Unknown => b"UNKNOWN",
        HealthState::Ok => b"OK",
        HealthState::Late => b"LATE",
        HealthState::Fault => b"FAULT",
    }
}

fn link(state: LinkState) -> &'static [u8] {
    match state {
        LinkState::Unknown => b"UNKNOWN",
        LinkState::Down => b"DOWN",
        LinkState::Up => b"UP",
    }
}

fn write_label_value(row: &mut [u8; OLED_TEXT_COLUMNS], label: &[u8], value: &[u8]) {
    write_bytes(row, 0, label);
    write_bytes(row, label.len(), value);
}

fn write_signed(row: &mut [u8; OLED_TEXT_COLUMNS], label: &[u8], value: i32) {
    write_bytes(row, 0, label);
    let mut cursor = label.len();
    if value < 0 {
        if cursor < row.len() {
            row[cursor] = b'-';
            cursor += 1;
        }
    } else if cursor < row.len() {
        row[cursor] = b'+';
        cursor += 1;
    }
    let magnitude = value.unsigned_abs();
    write_u32_decimal(row, cursor, magnitude);
}

fn write_hex32(row: &mut [u8; OLED_TEXT_COLUMNS], label: &[u8], value: u32) {
    write_bytes(row, 0, label);
    for (offset, shift) in (0..8).rev().enumerate() {
        let cursor = label.len() + offset;
        if cursor >= row.len() {
            break;
        }
        let nibble = ((value >> (shift * 4)) & 0x0f) as u8;
        row[cursor] = if nibble < 10 {
            b'0' + nibble
        } else {
            b'A' + nibble - 10
        };
    }
}

fn write_u32_decimal(row: &mut [u8; OLED_TEXT_COLUMNS], start: usize, mut value: u32) {
    let mut digits = [0_u8; 10];
    let mut count = 0;
    loop {
        digits[count] = (value % 10) as u8;
        count += 1;
        value /= 10;
        if value == 0 || count == digits.len() {
            break;
        }
    }
    for index in (0..count).rev() {
        let pos = start + (count - 1 - index);
        if pos >= row.len() {
            break;
        }
        row[pos] = b'0' + digits[index];
    }
}

fn write_bytes(row: &mut [u8; OLED_TEXT_COLUMNS], start: usize, bytes: &[u8]) {
    for (offset, &byte) in bytes.iter().enumerate() {
        let index = start + offset;
        if index >= row.len() {
            break;
        }
        row[index] = byte;
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use swp_status_view::{HealthState, LinkState, RuntimeMode};

    #[derive(Default)]
    struct MockDisplay {
        busy_once: bool,
        presented: u32,
    }

    impl OledDisplay for MockDisplay {
        type Error = ();

        fn try_present(
            &mut self,
            _frame: &OledTextFrame,
        ) -> Result<DisplayPresentOutcome, Self::Error> {
            if self.busy_once {
                self.busy_once = false;
                return Ok(DisplayPresentOutcome::Busy);
            }
            self.presented += 1;
            Ok(DisplayPresentOutcome::Presented)
        }
    }

    fn view() -> StatusView {
        StatusView {
            runtime_mode: RuntimeMode::Balancing,
            timing: HealthState::Ok,
            watchdog: HealthState::Ok,
            ble: LinkState::Up,
            authorized: true,
            runtime_fault_bits: 0,
            authority_reason_bits: 0x12,
            forward_velocity_mm_per_s: 123,
            pitch_mrad: -45,
            drive_demand_mnm: 7,
            reaction_demand_mnm: -8,
            drive_command_permille: 91,
            reaction_command_permille: -92,
        }
    }

    #[test]
    fn overview_contains_status_without_heap_formatting() {
        let frame = OledRenderer::render(view(), StatusPage::Overview);
        assert!(frame.row(0).starts_with(b"STATE BAL"));
        assert!(frame.row(1).starts_with(b"TIMING OK"));
        assert!(frame.row(4).starts_with(b"VEL mm/s +123"));
        assert!(frame.row(5).starts_with(b"PITCH mr -45"));
    }

    #[test]
    fn busy_display_drops_frame_without_queue() {
        let mut presenter = OledPresenter::new(MockDisplay {
            busy_once: true,
            presented: 0,
        });
        let first = OledRenderer::render(view(), StatusPage::Overview);
        let second = OledRenderer::render(view(), StatusPage::Control);
        assert_eq!(
            presenter.present_latest(&first).unwrap(),
            DisplayPresentOutcome::Busy
        );
        assert_eq!(
            presenter.present_latest(&second).unwrap(),
            DisplayPresentOutcome::Presented
        );
        assert_eq!(presenter.stats().dropped_busy, 1);
        assert_eq!(presenter.into_display().presented, 1);
    }
}

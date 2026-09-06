#![no_std]
#![forbid(unsafe_code)]

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum HealthState {
    #[default]
    Unknown,
    Ok,
    Late,
    Fault,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum LinkState {
    #[default]
    Unknown,
    Down,
    Up,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum RuntimeMode {
    #[default]
    Boot,
    HardwareCheck,
    Standby,
    CaptureWindow,
    Balancing,
    MomentumLimited,
    Fault,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct StatusView {
    pub runtime_mode: RuntimeMode,
    pub timing: HealthState,
    pub watchdog: HealthState,
    pub ble: LinkState,
    pub authorized: bool,
    pub runtime_fault_bits: u32,
    pub authority_reason_bits: u32,
    pub forward_velocity_mm_per_s: i16,
    pub pitch_mrad: i16,
    pub drive_demand_mnm: i16,
    pub reaction_demand_mnm: i16,
    pub drive_command_permille: i16,
    pub reaction_command_permille: i16,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum StatusPage {
    #[default]
    Overview,
    Control,
    Health,
}

impl StatusPage {
    pub const fn next(self) -> Self {
        match self {
            Self::Overview => Self::Control,
            Self::Control => Self::Health,
            Self::Health => Self::Overview,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn page_cycle_is_closed() {
        let page = StatusPage::Overview.next().next().next();
        assert_eq!(page, StatusPage::Overview);
    }
}

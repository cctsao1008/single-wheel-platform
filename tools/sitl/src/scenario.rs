#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Scenario {
    pub name: &'static str,
    pub duration_us: u64,
    pub opportunity_period_us: u64,
    pub seed: u64,
    pub missed_control_at_us: Option<u64>,
}

impl Scenario {
    pub const fn deterministic_baseline() -> Self {
        Self {
            name: "deterministic-baseline",
            duration_us: 100_000,
            opportunity_period_us: 5_000,
            seed: 0x5357_5001,
            missed_control_at_us: Some(50_000),
        }
    }

    pub fn opportunity_count(self) -> u64 {
        self.duration_us / self.opportunity_period_us + 1
    }
}

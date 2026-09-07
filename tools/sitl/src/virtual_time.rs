#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct VirtualTime(u64);

impl VirtualTime {
    pub const ZERO: Self = Self(0);

    pub const fn from_micros(micros: u64) -> Self {
        Self(micros)
    }

    pub const fn as_micros(self) -> u64 {
        self.0
    }

    pub const fn checked_add_micros(self, micros: u64) -> Option<Self> {
        match self.0.checked_add(micros) {
            Some(value) => Some(Self(value)),
            None => None,
        }
    }
}

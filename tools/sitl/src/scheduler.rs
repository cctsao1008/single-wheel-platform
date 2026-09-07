use std::{cmp::Reverse, collections::BinaryHeap};

use crate::virtual_time::VirtualTime;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum SemanticPhase {
    IntegratePlantTo,
    PhysicalOrFaultEvent,
    SensorSample,
    ObservationDelivery,
    ProductionRuntime,
    ActuationCommit,
}

impl SemanticPhase {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::IntegratePlantTo => "integrate_plant_to",
            Self::PhysicalOrFaultEvent => "physical_or_fault_event",
            Self::SensorSample => "sensor_sample",
            Self::ObservationDelivery => "observation_delivery",
            Self::ProductionRuntime => "production_runtime",
            Self::ActuationCommit => "actuation_commit",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ScheduledEvent {
    pub at: VirtualTime,
    pub phase: SemanticPhase,
    pub insertion_sequence: u64,
}

impl Ord for ScheduledEvent {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        (self.at, self.phase, self.insertion_sequence).cmp(&(
            other.at,
            other.phase,
            other.insertion_sequence,
        ))
    }
}

impl PartialOrd for ScheduledEvent {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

#[derive(Default)]
pub struct DeterministicScheduler {
    queue: BinaryHeap<Reverse<ScheduledEvent>>,
    next_insertion_sequence: u64,
}

impl DeterministicScheduler {
    pub fn schedule(&mut self, at: VirtualTime, phase: SemanticPhase) {
        let event = ScheduledEvent {
            at,
            phase,
            insertion_sequence: self.next_insertion_sequence,
        };
        self.next_insertion_sequence = self.next_insertion_sequence.wrapping_add(1);
        self.queue.push(Reverse(event));
    }

    pub fn pop_next(&mut self) -> Option<ScheduledEvent> {
        self.queue.pop().map(|entry| entry.0)
    }

    pub fn is_empty(&self) -> bool {
        self.queue.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn semantic_phase_order_wins_over_insertion_order_at_same_time() {
        let mut scheduler = DeterministicScheduler::default();
        let at = VirtualTime::from_micros(5_000);

        for phase in [
            SemanticPhase::ActuationCommit,
            SemanticPhase::ProductionRuntime,
            SemanticPhase::ObservationDelivery,
            SemanticPhase::SensorSample,
            SemanticPhase::PhysicalOrFaultEvent,
            SemanticPhase::IntegratePlantTo,
        ] {
            scheduler.schedule(at, phase);
        }

        let mut observed = Vec::new();
        while let Some(event) = scheduler.pop_next() {
            observed.push(event.phase);
        }

        assert_eq!(
            observed,
            vec![
                SemanticPhase::IntegratePlantTo,
                SemanticPhase::PhysicalOrFaultEvent,
                SemanticPhase::SensorSample,
                SemanticPhase::ObservationDelivery,
                SemanticPhase::ProductionRuntime,
                SemanticPhase::ActuationCommit,
            ]
        );
    }
}

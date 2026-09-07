use std::cmp::Reverse;
use std::collections::BinaryHeap;
use std::error::Error;
use std::fmt::{Display, Formatter};

use crate::virtual_time::VirtualTime;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum SemanticPhase {
    PhysicalOrFaultEvent,
    SensorSample,
    ObservationDelivery,
    ProductionRuntime,
    ActuationCommit,
}

impl SemanticPhase {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::PhysicalOrFaultEvent => "physical_or_fault_event",
            Self::SensorSample => "sensor_sample",
            Self::ObservationDelivery => "observation_delivery",
            Self::ProductionRuntime => "production_runtime",
            Self::ActuationCommit => "actuation_commit",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EventKind {
    ScenarioStart,
    SensorSample,
    ObservationDelivery,
    ProductionRuntime,
    RuntimeOpportunityMissed,
    ActuationCommit,
}

impl EventKind {
    pub const fn phase(self) -> SemanticPhase {
        match self {
            Self::ScenarioStart => SemanticPhase::PhysicalOrFaultEvent,
            Self::SensorSample => SemanticPhase::SensorSample,
            Self::ObservationDelivery => SemanticPhase::ObservationDelivery,
            Self::ProductionRuntime | Self::RuntimeOpportunityMissed => {
                SemanticPhase::ProductionRuntime
            }
            Self::ActuationCommit => SemanticPhase::ActuationCommit,
        }
    }

    pub const fn record_kind(self) -> &'static str {
        match self {
            Self::ScenarioStart => "scenario_start",
            Self::SensorSample => "sensor_sample",
            Self::ObservationDelivery => "observation_delivery",
            Self::ProductionRuntime => "control_opportunity",
            Self::RuntimeOpportunityMissed => "missed_opportunity",
            Self::ActuationCommit => "actuation_commit",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ScheduledEvent {
    pub at: VirtualTime,
    pub phase: SemanticPhase,
    pub insertion_sequence: u64,
    pub kind: EventKind,
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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TimeAdvance {
    pub from: VirtualTime,
    pub to: VirtualTime,
}

#[derive(Debug, PartialEq, Eq)]
pub struct TimeSlice {
    pub advance: TimeAdvance,
    pub events: Vec<ScheduledEvent>,
}

#[derive(Debug, PartialEq, Eq)]
pub enum ScheduleError {
    EventInPast { now: VirtualTime, at: VirtualTime },
    SequenceExhausted,
}

impl Display for ScheduleError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::EventInPast { now, at } => write!(
                formatter,
                "cannot schedule event at {} us while virtual time is {} us",
                at.as_micros(),
                now.as_micros()
            ),
            Self::SequenceExhausted => write!(formatter, "scheduler insertion sequence exhausted"),
        }
    }
}

impl Error for ScheduleError {}

#[derive(Default)]
pub struct DeterministicScheduler {
    queue: BinaryHeap<Reverse<ScheduledEvent>>,
    next_insertion_sequence: u64,
    now: VirtualTime,
}

impl DeterministicScheduler {
    pub const fn now(&self) -> VirtualTime {
        self.now
    }

    pub fn schedule(&mut self, at: VirtualTime, kind: EventKind) -> Result<u64, ScheduleError> {
        if at < self.now {
            return Err(ScheduleError::EventInPast { now: self.now, at });
        }

        let insertion_sequence = self.next_insertion_sequence;
        self.next_insertion_sequence = self
            .next_insertion_sequence
            .checked_add(1)
            .ok_or(ScheduleError::SequenceExhausted)?;
        self.queue.push(Reverse(ScheduledEvent {
            at,
            phase: kind.phase(),
            insertion_sequence,
            kind,
        }));
        Ok(insertion_sequence)
    }

    /// Advances scheduler time to the next timestamp and returns all events at
    /// that timestamp in semantic order. `advance` is the Stage-2 boundary at
    /// which the physical world will be integrated from the previous time to
    /// the new time before any event at `to` is dispatched.
    pub fn next_slice(&mut self) -> Option<TimeSlice> {
        let first = self.queue.pop()?.0;
        let at = first.at;
        let advance = TimeAdvance {
            from: self.now,
            to: at,
        };
        self.now = at;

        let mut events = vec![first];
        while self
            .queue
            .peek()
            .is_some_and(|event| event.0.at == at)
        {
            events.push(self.queue.pop().expect("peeked event must exist").0);
        }
        events.sort_by_key(|event| (event.phase, event.insertion_sequence));

        Some(TimeSlice { advance, events })
    }

    pub fn is_empty(&self) -> bool {
        self.queue.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn same_time_events_follow_semantic_phase_not_insertion_order() {
        let mut scheduler = DeterministicScheduler::default();
        let at = VirtualTime::from_micros(5_000);

        scheduler.schedule(at, EventKind::ActuationCommit).unwrap();
        scheduler
            .schedule(at, EventKind::ProductionRuntime)
            .unwrap();
        scheduler
            .schedule(at, EventKind::ObservationDelivery)
            .unwrap();
        scheduler.schedule(at, EventKind::SensorSample).unwrap();
        scheduler.schedule(at, EventKind::ScenarioStart).unwrap();

        let slice = scheduler.next_slice().unwrap();
        let phases: Vec<_> = slice.events.iter().map(|event| event.phase).collect();
        assert_eq!(
            phases,
            vec![
                SemanticPhase::PhysicalOrFaultEvent,
                SemanticPhase::SensorSample,
                SemanticPhase::ObservationDelivery,
                SemanticPhase::ProductionRuntime,
                SemanticPhase::ActuationCommit,
            ]
        );
    }

    #[test]
    fn scheduler_reports_time_advance_before_dispatch() {
        let mut scheduler = DeterministicScheduler::default();
        scheduler
            .schedule(VirtualTime::from_micros(5_000), EventKind::SensorSample)
            .unwrap();
        scheduler
            .schedule(
                VirtualTime::from_micros(10_000),
                EventKind::SensorSample,
            )
            .unwrap();

        let first = scheduler.next_slice().unwrap();
        assert_eq!(first.advance.from, VirtualTime::ZERO);
        assert_eq!(first.advance.to, VirtualTime::from_micros(5_000));
        assert_eq!(scheduler.now(), VirtualTime::from_micros(5_000));

        let second = scheduler.next_slice().unwrap();
        assert_eq!(second.advance.from, VirtualTime::from_micros(5_000));
        assert_eq!(second.advance.to, VirtualTime::from_micros(10_000));
        assert_eq!(scheduler.now(), VirtualTime::from_micros(10_000));
    }

    #[test]
    fn scheduling_into_the_past_is_rejected() {
        let mut scheduler = DeterministicScheduler::default();
        scheduler
            .schedule(VirtualTime::from_micros(10_000), EventKind::SensorSample)
            .unwrap();
        let _ = scheduler.next_slice().unwrap();

        assert_eq!(
            scheduler.schedule(VirtualTime::from_micros(5_000), EventKind::SensorSample),
            Err(ScheduleError::EventInPast {
                now: VirtualTime::from_micros(10_000),
                at: VirtualTime::from_micros(5_000),
            })
        );
    }

    #[test]
    fn equal_phase_events_preserve_insertion_sequence() {
        let mut scheduler = DeterministicScheduler::default();
        let at = VirtualTime::from_micros(5_000);
        let first = scheduler
            .schedule(at, EventKind::ProductionRuntime)
            .unwrap();
        let second = scheduler
            .schedule(at, EventKind::RuntimeOpportunityMissed)
            .unwrap();

        let slice = scheduler.next_slice().unwrap();
        assert_eq!(slice.events[0].insertion_sequence, first);
        assert_eq!(slice.events[1].insertion_sequence, second);
    }
}

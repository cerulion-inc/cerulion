// SPDX-License-Identifier: AGPL-3.0-only
//! Event types for pub/sub notification.
//!
//! Defines bidirectional event IDs for publisher↔subscriber signaling.
//! Each variant maps to a stable `EventId` value — do not reorder.
//!
//! # Event Topology
//!
//! Both publisher and subscriber create Notifier + Listener on the SAME
//! `{topic}/event` service. The `PubSubEvent` discriminant carries direction:
//!
//! - Publisher → Subscriber: `SentSample`, `SentHistory`, `PublisherConnected`, `PublisherDisconnected`
//! - Subscriber → Publisher: `SubscriberConnected`, `SubscriberDisconnected`, `ReceivedSample`

use iceoryx2::prelude::EventId;

/// Pub/sub event identifiers.
///
/// Mapped to `EventId` for iceoryx2 event service notification.
/// Values are stable — do not reorder or change numeric assignments.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u64)]
pub enum PubSubEvent {
    /// Publisher has new data available (pub → sub).
    SentSample = 0,
    /// Publisher has finished delivering history to a late-joiner (pub → sub).
    SentHistory = 1,
    /// A new subscriber has connected (sub → pub).
    SubscriberConnected = 2,
    /// A subscriber has disconnected (sub → pub).
    SubscriberDisconnected = 3,
    /// Subscriber acknowledges receipt (sub → pub).
    ReceivedSample = 4,
    /// Publisher is online (pub → sub).
    PublisherConnected = 5,
    /// Publisher has disconnected (pub → sub). Sent on Drop.
    PublisherDisconnected = 6,
}

impl From<PubSubEvent> for EventId {
    fn from(event: PubSubEvent) -> Self {
        EventId::new(event as usize)
    }
}

impl TryFrom<EventId> for PubSubEvent {
    type Error = usize;

    fn try_from(id: EventId) -> Result<Self, Self::Error> {
        match id.as_value() {
            0 => Ok(PubSubEvent::SentSample),
            1 => Ok(PubSubEvent::SentHistory),
            2 => Ok(PubSubEvent::SubscriberConnected),
            3 => Ok(PubSubEvent::SubscriberDisconnected),
            4 => Ok(PubSubEvent::ReceivedSample),
            5 => Ok(PubSubEvent::PublisherConnected),
            6 => Ok(PubSubEvent::PublisherDisconnected),
            other => Err(other),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pub_sub_event_to_event_id() {
        let event_id: EventId = PubSubEvent::SentSample.into();
        assert_eq!(event_id.as_value(), 0);
    }

    #[test]
    fn test_event_id_to_pub_sub_event() {
        let event_id = EventId::new(0);
        let event: PubSubEvent = event_id.try_into().unwrap();
        assert_eq!(event, PubSubEvent::SentSample);
    }

    #[test]
    fn test_unknown_event_id_rejected() {
        let event_id = EventId::new(99);
        let result: Result<PubSubEvent, usize> = event_id.try_into();
        assert_eq!(result.unwrap_err(), 99);
    }

    #[test]
    fn test_event_ids_stable() {
        assert_eq!(PubSubEvent::SentSample as u64, 0);
        assert_eq!(PubSubEvent::SentHistory as u64, 1);
        assert_eq!(PubSubEvent::SubscriberConnected as u64, 2);
        assert_eq!(PubSubEvent::SubscriberDisconnected as u64, 3);
        assert_eq!(PubSubEvent::ReceivedSample as u64, 4);
        assert_eq!(PubSubEvent::PublisherConnected as u64, 5);
        assert_eq!(PubSubEvent::PublisherDisconnected as u64, 6);
    }

    #[test]
    fn test_all_event_ids_roundtrip() {
        let events = [
            PubSubEvent::SentSample,
            PubSubEvent::SentHistory,
            PubSubEvent::SubscriberConnected,
            PubSubEvent::SubscriberDisconnected,
            PubSubEvent::ReceivedSample,
            PubSubEvent::PublisherConnected,
            PubSubEvent::PublisherDisconnected,
        ];
        for event in events {
            let id: EventId = event.into();
            let back: PubSubEvent = id.try_into().unwrap();
            assert_eq!(back, event);
        }
    }

    #[test]
    fn test_all_values_7_to_255_rejected() {
        for val in 7..=255 {
            let id = EventId::new(val);
            let result: Result<PubSubEvent, usize> = id.try_into();
            assert_eq!(result.unwrap_err(), val);
        }
    }
}

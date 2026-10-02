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

impl PubSubEvent {
    /// Every variant, in discriminant order. The single list the event-id
    /// ceiling is derived from; `all_lists_every_variant_and_max_id_covers_it`
    /// fails to COMPILE if a variant is added and not listed here.
    pub(crate) const ALL: [PubSubEvent; 7] = [
        PubSubEvent::SentSample,
        PubSubEvent::SentHistory,
        PubSubEvent::SubscriberConnected,
        PubSubEvent::SubscriberDisconnected,
        PubSubEvent::ReceivedSample,
        PubSubEvent::PublisherConnected,
        PubSubEvent::PublisherDisconnected,
    ];

    /// The highest event id the transport ever mints, derived from
    /// [`Self::ALL`] rather than written down twice. Every event service is
    /// created with this as its `event_id_max_value`, which sizes the
    /// shared-memory counting bitset a listener walks on every wait; see
    /// `CERULION_MAX_EVENT_ID` in `transport::mod`.
    pub(crate) const MAX_ID: usize = Self::max_id();

    const fn max_id() -> usize {
        let mut max = 0;
        let mut i = 0;
        while i < Self::ALL.len() {
            let value = Self::ALL[i] as usize;
            if value > max {
                max = value;
            }
            i += 1;
        }
        max
    }
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
    fn all_lists_every_variant_and_max_id_covers_it() {
        for event in PubSubEvent::ALL {
            // Exhaustive on purpose: a variant added to the enum and not to
            // `ALL` makes this match fail to compile, and an id above the
            // ceiling would be refused by iceoryx2 at notify time.
            let id: usize = match event {
                PubSubEvent::SentSample => 0,
                PubSubEvent::SentHistory => 1,
                PubSubEvent::SubscriberConnected => 2,
                PubSubEvent::SubscriberDisconnected => 3,
                PubSubEvent::ReceivedSample => 4,
                PubSubEvent::PublisherConnected => 5,
                PubSubEvent::PublisherDisconnected => 6,
            };
            assert_eq!(EventId::from(event).as_value(), id);
            assert!(
                id <= PubSubEvent::MAX_ID,
                "event id {id} exceeds the ceiling {} every event service is created with",
                PubSubEvent::MAX_ID
            );
        }
        assert_eq!(PubSubEvent::MAX_ID, 6);
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

// SPDX-License-Identifier: AGPL-3.0-only
//! App subscriptions read with `poll`.
//!
//! The subscription queue is the transport's own subscriber queue, `depth`
//! messages deep: a full queue drops its oldest message. `poll` takes one
//! message off it and decodes it into JSON text; that text is an app-side
//! copy, not the zero-copy node path.
use cerulion_core::dynamic::WireLayout;
use cerulion_core::CerulionSubscriber;
use serde::Serialize;

use crate::decode::decode_json_into;
use crate::error::EmbedError;

#[derive(Debug, Clone, Copy, Default, Serialize, PartialEq, Eq)]
/// The counts of one subscription, under `subscribed` in status.
pub struct SubscriptionCounts {
    /// Messages `poll` returned.
    pub polled: u64,
    /// Messages missing from a publisher's sequence: the full queue dropped
    /// them before a poll took them. A frame whose wire header the
    /// transport cannot read is skipped, with a logged warning, before poll
    /// sees it, and is counted nowhere.
    pub dropped: u64,
    /// Messages `poll` took off the queue and refused with `DECODE`.
    pub undecodable: u64,
}

/// One `subscribe` entry of a run.
pub(crate) struct Subscription {
    subscriber: CerulionSubscriber,
    layout: WireLayout,
    drops: DropCounter,
    counts: SubscriptionCounts,
    /// Reused JSON buffer: cleared per poll, its capacity kept.
    json: String,
}

impl Subscription {
    pub(crate) fn new(subscriber: CerulionSubscriber, layout: WireLayout) -> Self {
        Self {
            subscriber,
            layout,
            drops: DropCounter::default(),
            counts: SubscriptionCounts::default(),
            json: String::new(),
        }
    }

    /// Take the oldest queued message and decode it into JSON text, or
    /// return `None` when the queue is empty. Returns the subscription's
    /// counts after this poll with the result.
    pub(crate) fn poll(&mut self) -> (SubscriptionCounts, Result<Option<&str>, EmbedError>) {
        let taken = self.take();
        (
            self.counts,
            taken.map(|taken| taken.then_some(self.json.as_str())),
        )
    }

    /// Take one message into `json`; `false` when the queue is empty.
    fn take(&mut self) -> Result<bool, EmbedError> {
        let Some(sample) = self
            .subscriber
            .try_receive_one_owned()
            .map_err(|error| EmbedError::Receive(error.to_string()))?
        else {
            return Ok(false);
        };
        if let Some(header) = sample.wire_header() {
            self.counts.dropped += self.drops.observe(sample.origin(), header.sequence);
        }
        match decode_json_into(&self.layout, sample.payload(), &mut self.json) {
            Ok(()) => {
                self.counts.polled += 1;
                Ok(true)
            }
            Err(error) => {
                self.counts.undecodable += 1;
                Err(error)
            }
        }
    }
}

/// Publishers whose baseline a subscription keeps. A topic has one
/// publisher unless it is a multi-publisher topic. When every slot is used,
/// a new publisher takes the slot of the one seen longest ago, which starts
/// over with a new baseline if it sends again.
const MAX_STREAMS: usize = 8;

/// Counts the messages a full queue dropped, from gaps in each publisher's
/// wire sequence, which goes up by one per message sent. The first message
/// from a publisher sets its baseline, so a message dropped before the first
/// one this subscription took from that publisher is not counted. A message
/// behind the baseline (a replay of history) counts nothing.
#[derive(Debug, Default)]
struct DropCounter {
    /// `(publisher, last sequence taken, observation it was last seen at)`.
    streams: Vec<(u128, u32, u64)>,
    observations: u64,
}

impl DropCounter {
    /// Record a message `sequence` from `origin` and return how many of that
    /// publisher's messages were dropped since the last one taken.
    fn observe(&mut self, origin: u128, sequence: u32) -> u64 {
        self.observations += 1;
        let now = self.observations;
        if let Some(stream) = self.streams.iter_mut().find(|stream| stream.0 == origin) {
            stream.2 = now;
            let gap = sequence.wrapping_sub(stream.1);
            if gap == 0 || gap > u32::MAX / 2 {
                return 0;
            }
            stream.1 = sequence;
            return u64::from(gap - 1);
        }
        if self.streams.len() == MAX_STREAMS {
            if let Some(oldest) = self
                .streams
                .iter()
                .enumerate()
                .min_by_key(|(_, stream)| stream.2)
                .map(|(index, _)| index)
            {
                self.streams.swap_remove(oldest);
            }
        }
        self.streams.push((origin, sequence, now));
        0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_dense_sequence_drops_nothing() {
        let mut drops = DropCounter::default();
        assert_eq!(
            [0, 1, 2, 3].map(|sequence| drops.observe(7, sequence)),
            [0, 0, 0, 0]
        );
    }

    #[test]
    fn a_gap_counts_the_messages_between() {
        let mut drops = DropCounter::default();
        assert_eq!(
            drops.observe(7, 2),
            0,
            "the first message sets the baseline"
        );
        assert_eq!(drops.observe(7, 6), 3);
        assert_eq!(drops.observe(7, 7), 0);
        assert_eq!(drops.observe(7, u32::MAX - 1), 0, "a jump back is a replay");
        assert_eq!(drops.observe(7, 8), 0, "the baseline did not move back");
    }

    #[test]
    fn the_sequence_wraps_without_a_false_count() {
        let mut drops = DropCounter::default();
        drops.observe(7, u32::MAX - 1);
        assert_eq!(drops.observe(7, u32::MAX), 0);
        assert_eq!(drops.observe(7, 1), 1, "0 was dropped");
    }

    #[test]
    fn each_publisher_keeps_its_own_baseline() {
        let mut drops = DropCounter::default();
        drops.observe(1, 10);
        drops.observe(2, 500);
        assert_eq!(drops.observe(1, 12), 1);
        assert_eq!(drops.observe(2, 501), 0);
        assert_eq!(drops.observe(2, 0), 0, "a jump back is a replay");
    }

    #[test]
    fn a_new_publisher_takes_the_slot_of_the_one_seen_longest_ago() {
        let mut drops = DropCounter::default();
        for origin in 0..MAX_STREAMS as u128 {
            drops.observe(origin, 0);
        }
        // Publisher 0 is seen again, so publisher 1 is now the oldest.
        drops.observe(0, 1);
        drops.observe(99, 0);
        assert_eq!(drops.streams.len(), MAX_STREAMS);
        assert_eq!(drops.observe(0, 3), 1, "publisher 0 kept its baseline");
        assert_eq!(drops.observe(1, 5), 0, "publisher 1 lost its baseline");
    }

    #[test]
    fn a_repeated_sequence_and_the_half_range_boundary() {
        let mut drops = DropCounter::default();
        drops.observe(7, 5);
        assert_eq!(drops.observe(7, 5), 0, "the same message again");
        assert_eq!(
            drops.observe(7, 5 + u32::MAX / 2),
            u64::from(u32::MAX / 2 - 1)
        );
        let base = 5 + u32::MAX / 2;
        assert_eq!(
            drops.observe(7, base.wrapping_add(u32::MAX / 2 + 1)),
            0,
            "more than half the range ahead reads as behind"
        );
    }
}

// SPDX-License-Identifier: AGPL-3.0-only
//! Bounded, lease-armed node-fire metadata. No payload bytes enter this ring.
//!
//! Producers make one slot-claim attempt and never wait for another producer or
//! a reader. Overwritten, superseded and contended records count as loss. Each
//! slot occupies 40 bytes; the returned record remains 32 bytes.

use crate::clock::Clock;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// Default number of retained fire slots.
pub const FIRE_TAP_DEFAULT_CAPACITY: usize = 4096;
/// How long a reader keeps recording enabled after a request.
pub const FIRE_TAP_DEFAULT_LEASE: Duration = Duration::from_secs(5);
/// Maximum records examined by one read, excluding a bulk overwrite gap.
pub const FIRE_SERVE_MAX: usize = 512;
const LAST_SEQ: u64 = (u64::MAX - 2) / 2;

/// Ring bounds fixed at construction.
#[derive(Debug, Clone, Copy)]
pub struct FireTapConfig {
    /// Power of two in the inclusive range 64..=65536.
    pub capacity: usize,
    /// Nonzero lease representable as a u64 number of nanoseconds.
    pub lease: Duration,
}
impl Default for FireTapConfig {
    fn default() -> Self {
        Self {
            capacity: FIRE_TAP_DEFAULT_CAPACITY,
            lease: FIRE_TAP_DEFAULT_LEASE,
        }
    }
}

/// Resolved trigger policy. Value 2 is reserved for catch-up.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FireCause {
    /// No known trigger policy.
    Unknown = 0,
    /// Periodic trigger.
    Period = 1,
    /// Input arrival trigger.
    Data = 3,
    /// Synchronized input trigger.
    Sync = 4,
    /// External trigger.
    External = 5,
}
impl TryFrom<u8> for FireCause {
    /// The byte that names no cause, including the reserved 2.
    type Error = u8;

    fn try_from(value: u8) -> Result<Self, u8> {
        match value {
            0 => Ok(Self::Unknown),
            1 => Ok(Self::Period),
            3 => Ok(Self::Data),
            4 => Ok(Self::Sync),
            5 => Ok(Self::External),
            other => Err(other),
        }
    }
}

/// Tick outcome and attribution bits.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FireFlags(pub u8);
impl FireFlags {
    /// The tick panicked.
    pub const PANICKED: u8 = 0x01;
    /// The tick returned an error.
    pub const TICK_ERROR: u8 = 0x02;
    /// Output commits could not be observed.
    pub const WROTE_UNKNOWN: u8 = 0x04;
    /// Outputs beyond the 64-bit mask exist.
    pub const WROTE_TRUNCATED: u8 = 0x08;
    /// Wall duration exceeded u32 nanoseconds.
    pub const DURATION_SATURATED: u8 = 0x10;
    /// More than one input could have triggered the tick.
    pub const TRIGGER_AMBIGUOUS: u8 = 0x20;
}

/// One fire, with field order matching the version-one wire row.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FireRecord {
    /// Tap-wide sequence, starting at 1.
    pub seq: u64,
    /// Tick start on the injected runtime clock.
    pub at_ns: u64,
    /// Saturated tick wall duration in nanoseconds.
    pub dur_ns: u32,
    /// Index into the run topology's node array.
    pub node: u16,
    /// Resolved trigger policy.
    pub cause: FireCause,
    /// Outcome and attribution bits.
    pub flags: FireFlags,
    /// Bit k indicates a commit on output k.
    pub wrote: u64,
}

#[derive(Clone, Copy)]
pub(crate) struct FireBody {
    pub at_ns: u64,
    pub dur_ns: u32,
    pub node: u16,
    pub cause: FireCause,
    pub flags: FireFlags,
    pub wrote: u64,
}

/// Where a reader resumes. Independent readers hold independent cursors.
#[derive(Debug, Clone, Copy)]
pub enum Cursor {
    /// Start after all sequences claimed at this request's snapshot.
    Now,
    /// Resume strictly after this sequence (0 starts at the first record).
    After(u64),
}

/// A cursor page. `records.len() + lost == next - after` for an After request.
#[derive(Debug)]
pub struct FireBatch {
    /// Runtime clock at the request.
    pub now_ns: u64,
    /// Runtime clock when the current lease interval began. Every tick the
    /// interval admits starts at or after it, and the lease runs at least the
    /// configured lease past it.
    pub armed_since_ns: u64,
    /// Last sequence accounted for; pass it to `Cursor::After` unchanged.
    pub next: u64,
    /// Overwritten, superseded or contended records in this page's range.
    pub lost: u64,
    /// Node indices and nonzero tick-start timestamps, in topology order.
    pub in_tick: Vec<(u16, u64)>,
    /// Complete records in strictly increasing sequence order.
    pub records: Vec<FireRecord>,
}

/// Current recording state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
pub struct FireTapStats {
    /// Whether the reader lease is currently live.
    pub armed: bool,
    /// Sequences claimed while armed, including records subsequently lost.
    pub recorded: u64,
    /// Number of allocated slots.
    pub capacity: usize,
}

/// Bounded multi-producer, multi-reader fire telemetry.
///
/// Only readers serialize lease-epoch bookkeeping. Producers never acquire
/// that mutex, allocate, read a clock, or retry a sequence or slot claim. The
/// stamp and data words are sequentially consistent: equal committed stamps
/// surrounding the three data loads exclude any intervening writer. The claim
/// CAS excludes overlapping writers even when a producer is lapped while paused.
pub struct FireTap {
    slots: Box<[Slot]>,
    head: AtomicU64,
    lease_until_ns: AtomicU64,
    armed_since_ns: Mutex<u64>,
    lease_ns: u64,
    in_tick: Box<[Arc<AtomicU64>]>,
    clock: Arc<dyn Clock>,
}

impl std::fmt::Debug for FireTap {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("FireTap")
            .field("capacity", &self.slots.len())
            .field("recorded", &self.assigned())
            .field(
                "lease_until_ns",
                &self.lease_until_ns.load(Ordering::Relaxed),
            )
            .field("lease_ns", &self.lease_ns)
            .field("nodes", &self.in_tick.len())
            .finish_non_exhaustive()
    }
}

#[derive(Default)]
struct Slot {
    stamp: AtomicU64,
    at_ns: AtomicU64,
    packed: AtomicU64,
    wrote: AtomicU64,
    // A dropped claim cannot write a stamp another writer owns. The highest
    // dropped seq lets readers account for that hole without waiting.
    settled: AtomicU64,
}

impl FireTap {
    pub(crate) fn new(
        config: FireTapConfig,
        node_count: u16,
        in_tick: Vec<Arc<AtomicU64>>,
        clock: Arc<dyn Clock>,
    ) -> Self {
        assert!(
            config.capacity.is_power_of_two() && (64..=65536).contains(&config.capacity),
            "fire capacity must be a power of two in 64..=65536"
        );
        assert_eq!(
            in_tick.len(),
            usize::from(node_count),
            "one in-tick atomic per node"
        );
        let lease_ns =
            u64::try_from(config.lease.as_nanos()).expect("fire lease exceeds u64 nanoseconds");
        assert_ne!(lease_ns, 0, "fire lease must be nonzero");
        Self {
            slots: (0..config.capacity).map(|_| Slot::default()).collect(),
            head: AtomicU64::new(0),
            lease_until_ns: AtomicU64::new(0),
            armed_since_ns: Mutex::new(0),
            lease_ns,
            in_tick: in_tick.into_boxed_slice(),
            clock,
        }
    }

    /// The unarmed fast path is one relaxed load and no clock call.
    #[inline]
    pub(crate) fn armed_now(&self) -> Option<u64> {
        let until = self.lease_until_ns.load(Ordering::Relaxed);
        if until == 0 {
            return None;
        }
        let now = self.clock.now_ns();
        if now < until {
            return Some(now);
        }
        // A reader's concurrent extension must not be cleared by this expiry.
        // Every nonzero deadline is a reader's clock read plus `lease_ns`, and
        // a re-arm reports that read as `armed_since_ns`, so a tick that read
        // the clock before it stays out of that lease.
        match self
            .lease_until_ns
            .compare_exchange(until, 0, Ordering::Relaxed, Ordering::Relaxed)
        {
            Err(extended) if now < extended && now >= extended.saturating_sub(self.lease_ns) => {
                Some(now)
            }
            _ => None,
        }
    }

    /// Call after `armed_now` admitted a tick. No clock read is needed again;
    /// a lease explicitly cleared during that tick suppresses its record.
    pub(crate) fn push(&self, body: FireBody) {
        self.push_with(body, |_| {});
    }

    #[inline]
    fn push_with(&self, body: FireBody, after_claim: impl FnOnce(u64)) {
        if self.lease_until_ns.load(Ordering::Relaxed) == 0 {
            return;
        }
        let Some(seq) = self.reserve(|| {}) else {
            return;
        };
        let slot = &self.slots[((seq - 1) as usize) & (self.slots.len() - 1)];
        let stamp = slot.stamp.load(Ordering::SeqCst);
        let writing = 2 * seq + 1;
        if stamp & 1 != 0
            || stamp >= writing
            || slot
                .stamp
                .compare_exchange(stamp, writing, Ordering::SeqCst, Ordering::SeqCst)
                .is_err()
        {
            slot.settled.fetch_max(seq, Ordering::Release);
            return;
        }
        after_claim(seq);
        slot.at_ns.store(body.at_ns, Ordering::SeqCst);
        let packed = u64::from(body.dur_ns)
            | (u64::from(body.node) << 32)
            | ((body.cause as u64) << 48)
            | (u64::from(body.flags.0) << 56);
        slot.packed.store(packed, Ordering::SeqCst);
        slot.wrote.store(body.wrote, Ordering::SeqCst);
        slot.stamp.store(writing + 1, Ordering::SeqCst);
    }

    /// One read-modify-write takes the next sequence, so a producer never
    /// retries against another. `contend` is a test seam that runs just before it.
    #[inline]
    fn reserve(&self, contend: impl FnOnce()) -> Option<u64> {
        contend();
        // Relaxed suffices: head assigns identities, never publishes data.
        let previous = self.head.fetch_add(1, Ordering::Relaxed);
        if previous >= LAST_SEQ {
            // Exhaustion freezes instead of recycling stamp values.
            self.head.store(LAST_SEQ, Ordering::Relaxed);
            return None;
        }
        Some(previous + 1)
    }

    /// Sequences assigned so far. After exhaustion `head` can briefly pass
    /// `LAST_SEQ`.
    #[inline]
    fn assigned(&self) -> u64 {
        self.head.load(Ordering::Relaxed).min(LAST_SEQ)
    }

    /// Arm or extend the lease, then return a bounded page after `cursor`.
    /// A pending, unsuperseded claim ends the page without advancing its cursor.
    /// A zero limit arms the lease without consuming any sequence or loss.
    /// A lost sequence takes no row of the limit, so a page with fewer rows
    /// ended at the newest assigned sequence or at a pending claim.
    pub fn serve(&self, cursor: Cursor, max: usize) -> FireBatch {
        let mut since = self
            .armed_since_ns
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        let mut old = self.lease_until_ns.load(Ordering::Relaxed);
        // The clock is read after `old` and before the CAS: an expiry that
        // clears `old` in between fails the CAS and forces a later read, and a
        // tick this CAS admits reads its clock after the CAS.
        let now_ns = loop {
            let now_ns = self.clock.now_ns();
            let until = old.max(now_ns.saturating_add(self.lease_ns));
            match self.lease_until_ns.compare_exchange(
                old,
                until,
                Ordering::Relaxed,
                Ordering::Relaxed,
            ) {
                Ok(_) => break now_ns,
                Err(actual) => old = actual,
            }
        };
        if old == 0 || old <= now_ns {
            *since = now_ns;
        }
        let armed_since_ns = *since;
        drop(since);
        let head = self.assigned();
        let mut batch = FireBatch {
            now_ns,
            armed_since_ns,
            next: match cursor {
                Cursor::Now => head,
                Cursor::After(seq) => seq,
            },
            lost: 0,
            in_tick: self
                .in_tick
                .iter()
                .enumerate()
                .filter_map(|(node, since)| {
                    let since = since.load(Ordering::Acquire);
                    (since != 0).then_some((node as u16, since))
                })
                .collect(),
            records: Vec::with_capacity(max.min(FIRE_SERVE_MAX)),
        };
        let max = max.min(FIRE_SERVE_MAX);
        if max == 0 || batch.next >= head {
            return batch;
        }
        let floor = head.saturating_sub(self.slots.len() as u64);
        if batch.next < floor {
            batch.lost = floor - batch.next;
            batch.next = floor;
        }
        while batch.records.len() < max {
            if batch.next >= head {
                break;
            }
            let seq = batch.next + 1;
            let slot = &self.slots[((seq - 1) as usize) & (self.slots.len() - 1)];
            // Only drops raise `settled`: reaching `seq` means `seq` was dropped
            // or a later claim on this slot lapped it.
            let settled = slot.settled.load(Ordering::Acquire);
            let stamp = slot.stamp.load(Ordering::SeqCst);
            let committed = 2 * seq + 2;
            if stamp == committed {
                let at_ns = slot.at_ns.load(Ordering::SeqCst);
                let packed = slot.packed.load(Ordering::SeqCst);
                let wrote = slot.wrote.load(Ordering::SeqCst);
                if slot.stamp.load(Ordering::SeqCst) == committed {
                    batch.records.push(FireRecord {
                        seq,
                        at_ns,
                        dur_ns: packed as u32,
                        node: (packed >> 32) as u16,
                        cause: FireCause::try_from((packed >> 48) as u8)
                            .unwrap_or(FireCause::Unknown),
                        flags: FireFlags((packed >> 56) as u8),
                        wrote,
                    });
                } else {
                    batch.lost += 1;
                }
            } else if stamp > committed || settled >= seq {
                batch.lost += 1;
            } else {
                break;
            }
            batch.next = seq;
        }
        batch
    }

    /// Sample lease state and the number of assigned sequences.
    pub fn stats(&self) -> FireTapStats {
        FireTapStats {
            armed: self.armed_now().is_some(),
            recorded: self.assigned(),
            capacity: self.slots.len(),
        }
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::clock::Clock;
    use std::sync::atomic::{AtomicU64, Ordering};
    use std::sync::{Arc, Barrier};
    use std::time::Duration;

    #[derive(Default)]
    struct CountingClock {
        now: AtomicU64,
        reads: AtomicU64,
    }
    impl Clock for CountingClock {
        fn now_ns(&self) -> u64 {
            self.reads.fetch_add(1, Ordering::Relaxed);
            self.now.load(Ordering::Relaxed)
        }
    }

    fn tap(capacity: usize) -> (FireTap, Arc<CountingClock>) {
        let clock = Arc::new(CountingClock::default());
        let tap = FireTap::new(
            FireTapConfig {
                capacity,
                lease: Duration::from_nanos(100),
            },
            4,
            (0..4).map(|_| Arc::new(AtomicU64::new(0))).collect(),
            clock.clone(),
        );
        (tap, clock)
    }

    struct PausingClock {
        now: AtomicU64,
        pause: std::sync::atomic::AtomicBool,
        skip: AtomicU64,
        late: std::sync::atomic::AtomicBool,
        arrived: std::sync::mpsc::SyncSender<()>,
        resume: Mutex<std::sync::mpsc::Receiver<()>>,
    }
    impl Clock for PausingClock {
        fn now_ns(&self) -> u64 {
            let now = self.now.load(Ordering::Relaxed);
            if self.pause.load(Ordering::Relaxed)
                && self
                    .skip
                    .try_update(Ordering::Relaxed, Ordering::Relaxed, |n| n.checked_sub(1))
                    .is_err()
                && self.pause.swap(false, Ordering::Relaxed)
            {
                self.arrived.send(()).unwrap();
                self.resume.lock().unwrap().recv().unwrap();
                if self.late.load(Ordering::Relaxed) {
                    return self.now.load(Ordering::Relaxed);
                }
            }
            now
        }
    }

    /// Once `pause` is set, the clock passes `skip` reads, then the next read
    /// blocks until the returned sender releases it. That read returns the time
    /// from before the block, or from after it when `late` is set.
    fn pausing_tap() -> (
        FireTap,
        Arc<PausingClock>,
        std::sync::mpsc::Receiver<()>,
        std::sync::mpsc::SyncSender<()>,
    ) {
        let (arrived_tx, arrived_rx) = std::sync::mpsc::sync_channel(1);
        let (resume_tx, resume_rx) = std::sync::mpsc::sync_channel(1);
        let clock = Arc::new(PausingClock {
            now: AtomicU64::new(0),
            pause: std::sync::atomic::AtomicBool::new(false),
            skip: AtomicU64::new(0),
            late: std::sync::atomic::AtomicBool::new(false),
            arrived: arrived_tx,
            resume: Mutex::new(resume_rx),
        });
        let tap = FireTap::new(
            FireTapConfig {
                capacity: 64,
                lease: Duration::from_nanos(100),
            },
            0,
            vec![],
            clock.clone(),
        );
        (tap, clock, arrived_rx, resume_tx)
    }

    fn body(id: u64) -> FireBody {
        FireBody {
            at_ns: id,
            dur_ns: (id as u32) ^ 0xa55a1234,
            node: (id % 4) as u16,
            cause: FireCause::Data,
            flags: FireFlags(FireFlags::TICK_ERROR | FireFlags::WROTE_TRUNCATED),
            wrote: !id,
        }
    }

    fn assert_body(record: &FireRecord) {
        assert_eq!(record.dur_ns, (record.at_ns as u32) ^ 0xa55a1234);
        assert_eq!(record.node, (record.at_ns % 4) as u16);
        assert_eq!(record.cause, FireCause::Data);
        assert_eq!(record.flags.0, 0x0a);
        assert_eq!(record.wrote, !record.at_ns);
    }

    #[test]
    fn lease_arms_extends_expires_and_disarmed_reads_no_clock() {
        let (tap, clock) = tap(64);
        assert_eq!(tap.armed_now(), None);
        tap.push(body(0));
        assert_eq!(clock.reads.load(Ordering::Relaxed), 0);
        assert_eq!(tap.stats().recorded, 0);
        let initial = tap.serve(Cursor::Now, 512);
        assert_eq!(initial.next, 0);
        assert_eq!(initial.armed_since_ns, 0);
        assert_eq!(tap.armed_now(), Some(0));
        clock.now.store(99, Ordering::Relaxed);
        tap.serve(Cursor::After(0), 512);
        clock.now.store(100, Ordering::Relaxed);
        assert_eq!(tap.armed_now(), Some(100));
        clock.now.store(199, Ordering::Relaxed);
        assert_eq!(tap.armed_now(), None);
        let reads = clock.reads.load(Ordering::Relaxed);
        tap.push(body(1));
        assert_eq!(tap.armed_now(), None);
        assert_eq!(clock.reads.load(Ordering::Relaxed), reads);
        assert_eq!(tap.stats().recorded, 0);
        clock.now.store(200, Ordering::Relaxed);
        assert_eq!(tap.serve(Cursor::Now, 0).armed_since_ns, 200);
    }

    #[test]
    fn overwrite_loss_and_inclusive_cursor_are_exact() {
        let (tap, _) = tap(64);
        tap.serve(Cursor::Now, 512);
        for id in 0..100 {
            tap.push(body(id));
        }
        let zero = tap.serve(Cursor::After(0), 0);
        assert_eq!((zero.next, zero.lost, zero.records.len()), (0, 0, 0));
        let batch = tap.serve(Cursor::After(0), 512);
        assert_eq!(batch.lost, 36);
        assert_eq!(batch.next, 100);
        assert_eq!(batch.records.len(), 64);
        for (id, record) in (36..100).zip(&batch.records) {
            assert_eq!(record.seq, id + 1);
            assert_eq!(record.at_ns, id);
            assert_body(record);
        }
        let again = tap.serve(Cursor::After(0), 512);
        assert_eq!(again.records, batch.records);
        assert!(tap.serve(Cursor::After(batch.next), 512).records.is_empty());
        assert!(tap.serve(Cursor::Now, 512).records.is_empty());
    }

    #[test]
    fn page_limit_zero_and_future_cursor_do_not_skip_records() {
        let (tap, _) = tap(1024);
        tap.serve(Cursor::Now, 0);
        for id in 0..600 {
            tap.push(body(id));
        }
        let zero = tap.serve(Cursor::After(0), 0);
        assert_eq!(zero.next, 0);
        assert_eq!(zero.lost, 0);
        let page = tap.serve(Cursor::After(0), usize::MAX);
        assert_eq!(page.records.len(), 512);
        assert_eq!(page.next, 512);
        let tail = tap.serve(Cursor::After(page.next), 512);
        assert_eq!(tail.records.len(), 88);
        assert_eq!(tail.next, 600);
        let future = tap.serve(Cursor::After(u64::MAX), 512);
        assert!(future.records.is_empty());
        assert_eq!(future.next, u64::MAX);
    }

    /// A reader reads on only after a full page, so a lost sequence must not
    /// take a row: a short page reached the newest assigned sequence.
    #[test]
    fn a_lost_sequence_inside_the_page_does_not_shorten_it() {
        let (tap, _) = tap(1024);
        tap.serve(Cursor::Now, 0);
        // A busy slot makes the claim of seq 1 drop it.
        tap.slots[0].stamp.store(1, Ordering::SeqCst);
        for id in 0..700 {
            tap.push(body(id));
        }
        let page = tap.serve(Cursor::After(0), FIRE_SERVE_MAX);
        assert_eq!((page.next, page.lost, page.records.len()), (513, 1, 512));
        assert_eq!(page.records[0].seq, 2);
        let tail = tap.serve(Cursor::After(page.next), FIRE_SERVE_MAX);
        assert_eq!((tail.next, tail.lost, tail.records.len()), (700, 0, 187));
    }

    #[test]
    fn in_tick_reports_only_nonzero_nodes() {
        let (tap, _) = tap(64);
        tap.in_tick[2].store(42, Ordering::Release);
        assert_eq!(tap.serve(Cursor::Now, 0).in_tick, vec![(2, 42)]);
        tap.in_tick[2].store(0, Ordering::Release);
        assert!(tap.serve(Cursor::Now, 0).in_tick.is_empty());
    }

    #[test]
    fn pack_round_trip_and_record_size() {
        assert_eq!(std::mem::size_of::<FireRecord>(), 32);
        let (tap, _) = tap(64);
        tap.serve(Cursor::Now, 0);
        let cases = [
            (FireCause::Unknown, 0xffff, 0xff),
            (FireCause::Period, 0x8001, 0x01),
            (FireCause::Data, 0x1234, 0x80),
            (FireCause::Sync, 0x0100, 0x3f),
            (FireCause::External, 0x00ff, 0x00),
        ];
        for (cause, node, flags) in cases {
            tap.push(FireBody {
                at_ns: u64::MAX,
                dur_ns: u32::MAX,
                node,
                cause,
                flags: FireFlags(flags),
                wrote: u64::MAX,
            });
        }
        let batch = tap.serve(Cursor::After(0), 512);
        assert_eq!(batch.records.len(), 5);
        for (record, (cause, node, flags)) in batch.records.iter().zip(cases) {
            assert_eq!(record.at_ns, u64::MAX);
            assert_eq!(record.dur_ns, u32::MAX);
            assert_eq!(record.node, node);
            assert_eq!(record.cause, cause);
            assert_eq!(record.flags.0, flags);
            assert_eq!(record.wrote, u64::MAX);
        }
    }

    #[test]
    fn cause_bytes_convert_only_for_known_causes() {
        for (byte, cause) in [
            (0, FireCause::Unknown),
            (1, FireCause::Period),
            (3, FireCause::Data),
            (4, FireCause::Sync),
            (5, FireCause::External),
        ] {
            assert_eq!(FireCause::try_from(byte), Ok(cause));
            assert_eq!(cause as u8, byte);
        }
        for byte in [2, 6, 0x7f, 0xff] {
            assert_eq!(FireCause::try_from(byte), Err(byte));
        }
    }

    #[test]
    fn stats_compare_by_value_and_debug_reads_no_clock() {
        let (tap, clock) = tap(64);
        let disarmed = tap.stats();
        let copy = disarmed;
        assert_eq!(
            disarmed,
            FireTapStats {
                armed: false,
                recorded: 0,
                capacity: 64
            }
        );
        assert_eq!(copy, disarmed);
        tap.serve(Cursor::Now, 0);
        tap.push(body(0));
        assert_eq!(
            tap.stats(),
            FireTapStats {
                armed: true,
                recorded: 1,
                capacity: 64
            }
        );
        let reads = clock.reads.load(Ordering::Relaxed);
        let debug = format!("{tap:?}");
        assert_eq!(clock.reads.load(Ordering::Relaxed), reads);
        for field in [
            "capacity: 64",
            "recorded: 1",
            "lease_until_ns: 100",
            "lease_ns: 100",
        ] {
            assert!(debug.contains(field), "{field} missing from {debug}");
        }
    }

    #[test]
    fn four_producers_two_readers_one_million_pushes() {
        const TOTAL: u64 = 1_000_000;
        let (tap, _) = tap(64);
        tap.serve(Cursor::Now, 0);
        let barrier = Barrier::new(6);
        let oracle: Vec<_> = (0..TOTAL).map(|_| AtomicU64::new(u64::MAX)).collect();
        std::thread::scope(|scope| {
            let readers: Vec<_> = (0..2)
                .map(|_| {
                    scope.spawn(|| {
                        barrier.wait();
                        let mut cursor = 0;
                        let mut served = 0;
                        let mut lost = 0;
                        let deadline = std::time::Instant::now() + Duration::from_secs(60);
                        while cursor < TOTAL {
                            assert!(
                                std::time::Instant::now() < deadline,
                                "reader stuck at {cursor}"
                            );
                            let batch = tap.serve(Cursor::After(cursor), 23);
                            assert_eq!(
                                batch.records.len() as u64 + batch.lost,
                                batch.next - cursor
                            );
                            let mut previous = cursor;
                            for record in &batch.records {
                                assert!(record.seq > previous);
                                assert!(record.at_ns < TOTAL);
                                assert_body(record);
                                assert_eq!(
                                    record.at_ns,
                                    oracle[record.seq as usize - 1].load(Ordering::Relaxed)
                                );
                                previous = record.seq;
                            }
                            served += batch.records.len() as u64;
                            lost += batch.lost;
                            cursor = batch.next;
                            std::thread::yield_now();
                        }
                        assert_eq!(served + lost, TOTAL);
                        assert!(served > 0);
                        (served, lost)
                    })
                })
                .collect();
            let tap = &tap;
            let barrier = &barrier;
            let oracle = &oracle;
            let producers: Vec<_> = (0..4)
                .map(|producer| {
                    scope.spawn(move || {
                        barrier.wait();
                        for id in (producer..TOTAL).step_by(4) {
                            tap.push_with(body(id), |seq| {
                                oracle[seq as usize - 1].store(id, Ordering::Relaxed)
                            });
                            if id % 256 < 4 {
                                std::thread::yield_now();
                            }
                        }
                    })
                })
                .collect();
            for producer in producers {
                producer.join().unwrap();
            }
            for reader in readers {
                let (served, lost) = reader.join().unwrap();
                eprintln!("fire_tap stress: served={served}, lost={lost}, total={TOTAL}");
            }
        });
        assert_eq!(tap.stats().recorded, TOTAL);
    }
    #[test]
    fn pending_claim_stops_then_lapped_dropped_tail_is_accounted() {
        let (tap, _) = tap(64);
        tap.serve(Cursor::Now, 0);
        let (claimed_tx, claimed_rx) = std::sync::mpsc::sync_channel(1);
        let (resume_tx, resume_rx) = std::sync::mpsc::sync_channel(1);
        let (pending, lapped) = std::thread::scope(|scope| {
            let tap_ref = &tap;
            scope.spawn(move || {
                tap_ref.push_with(body(0), |seq| {
                    claimed_tx.send(seq).unwrap();
                    resume_rx.recv().unwrap();
                })
            });
            let claimed = claimed_rx.recv().unwrap();
            tap.push(body(1));
            let pending = tap.serve(Cursor::After(0), 512);
            for id in 2..65 {
                tap.push(body(id));
            }
            let lapped = tap.serve(Cursor::After(0), 512);
            resume_tx.send(()).unwrap();
            assert_eq!(claimed, 1);
            (pending, lapped)
        });
        assert_eq!(
            (pending.next, pending.lost, pending.records.len()),
            (0, 0, 0)
        );
        assert_eq!(
            (lapped.next, lapped.lost, lapped.records.len()),
            (65, 2, 63)
        );
        for (id, record) in (1..64).zip(&lapped.records) {
            assert_eq!(record.seq, id + 1);
            assert_eq!(record.at_ns, id);
            assert_body(record);
        }
        assert!(tap.serve(Cursor::After(65), 512).records.is_empty());
        tap.push(body(65));
        let next = tap.serve(Cursor::After(65), 512);
        assert_eq!(next.records[0].at_ns, 65);
        assert_eq!(next.next, 66);
        assert_eq!(next.lost, 0);
    }

    #[test]
    fn a_claim_lapped_before_it_reads_its_slot_cannot_overwrite_a_newer_record() {
        let (tap, _) = tap(64);
        tap.serve(Cursor::Now, 0);
        for id in 0..65 {
            tap.push(body(id));
        }
        // Replays a producer that took seq 1 and was preempted before reading
        // slot 0, while seq 65 committed there.
        tap.head.store(0, Ordering::Relaxed);
        tap.push(body(1000));
        tap.head.store(65, Ordering::Relaxed);
        let batch = tap.serve(Cursor::After(64), 512);
        assert_eq!((batch.next, batch.lost, batch.records.len()), (65, 0, 1));
        assert_eq!((batch.records[0].seq, batch.records[0].at_ns), (65, 64));
        assert_body(&batch.records[0]);
    }

    #[test]
    fn eight_producers_forced_wraparound_never_tear_or_stall() {
        const EACH: u64 = 10_000;
        const TOTAL: u64 = 1 + 7 * EACH;
        let (tap, _) = tap(64);
        tap.serve(Cursor::Now, 0);
        let oracle: Vec<_> = (0..TOTAL).map(|_| AtomicU64::new(u64::MAX)).collect();
        let barrier = Barrier::new(9);
        let (claimed_tx, claimed_rx) = std::sync::mpsc::sync_channel(1);
        let (resume_tx, resume_rx) = std::sync::mpsc::sync_channel(1);
        std::thread::scope(|scope| {
            let tap = &tap;
            let oracle = &oracle;
            let barrier = &barrier;
            let stalled = scope.spawn(move || {
                tap.push_with(body(0), |seq| {
                    oracle[seq as usize - 1].store(0, Ordering::Relaxed);
                    claimed_tx.send(seq).unwrap();
                    resume_rx.recv().unwrap();
                })
            });
            assert_eq!(claimed_rx.recv().unwrap(), 1);
            let readers: Vec<_> = (0..2)
                .map(|_| {
                    scope.spawn(move || {
                        barrier.wait();
                        let mut cursor = 0;
                        let mut served = 0;
                        let mut lost = 0;
                        let deadline = std::time::Instant::now() + Duration::from_secs(60);
                        while cursor < TOTAL {
                            assert!(
                                std::time::Instant::now() < deadline,
                                "dropped sequence stalled reader at {cursor}"
                            );
                            let batch = tap.serve(Cursor::After(cursor), 31);
                            assert_eq!(
                                batch.records.len() as u64 + batch.lost,
                                batch.next - cursor
                            );
                            let mut previous = cursor;
                            for record in &batch.records {
                                assert!(record.seq > previous);
                                assert_eq!(
                                    record.at_ns,
                                    oracle[record.seq as usize - 1].load(Ordering::Relaxed)
                                );
                                assert_body(record);
                                previous = record.seq;
                            }
                            served += batch.records.len() as u64;
                            lost += batch.lost;
                            cursor = batch.next;
                            std::thread::yield_now();
                        }
                        assert_eq!(served + lost, TOTAL);
                        assert!(lost > 0);
                        eprintln!(
                            "fire_tap forced lapping: served={served}, lost={lost}, total={TOTAL}"
                        );
                    })
                })
                .collect();
            let producers: Vec<_> = (0..7)
                .map(|producer| {
                    scope.spawn(move || {
                        barrier.wait();
                        for id in (1 + producer * EACH)..(1 + (producer + 1) * EACH) {
                            tap.push_with(body(id), |seq| {
                                oracle[seq as usize - 1].store(id, Ordering::Relaxed)
                            });
                            if id % 256 < 4 {
                                std::thread::yield_now();
                            }
                        }
                    })
                })
                .collect();
            for producer in producers {
                producer.join().unwrap();
            }
            // Always release the paused producer even if a reader assertion fails.
            let results: Vec<_> = readers.into_iter().map(|reader| reader.join()).collect();
            resume_tx.send(()).unwrap();
            stalled.join().unwrap();
            for result in results {
                result.unwrap();
            }
        });
        assert_eq!(tap.stats().recorded, TOTAL);
    }

    #[test]
    fn configuration_rejects_invalid_bounds_and_matches_default() {
        let defaults = FireTapConfig::default();
        assert_eq!(defaults.capacity, 4096);
        assert_eq!(defaults.lease, Duration::from_secs(5));
        assert_eq!(std::mem::size_of::<Slot>(), 40);
        for capacity in [0, 1, 63, 65, 131072] {
            assert!(std::panic::catch_unwind(|| tap(capacity)).is_err());
        }
        for lease in [Duration::ZERO, Duration::MAX] {
            assert!(std::panic::catch_unwind(|| FireTap::new(
                FireTapConfig {
                    capacity: 64,
                    lease
                },
                0,
                vec![],
                Arc::new(CountingClock::default())
            ))
            .is_err());
        }
        assert!(std::panic::catch_unwind(|| FireTap::new(
            defaults,
            1,
            vec![],
            Arc::new(CountingClock::default())
        ))
        .is_err());
        assert_eq!(tap(65536).0.stats().capacity, 65536);
    }

    #[test]
    fn expiry_cannot_erase_a_concurrent_reader_extension() {
        let (tap, clock, arrived_rx, resume_tx) = pausing_tap();
        tap.serve(Cursor::Now, 0);
        clock.now.store(100, Ordering::Relaxed);
        clock.pause.store(true, Ordering::Relaxed);
        std::thread::scope(|scope| {
            let expiry = scope.spawn(|| tap.armed_now());
            arrived_rx.recv().unwrap();
            clock.now.store(150, Ordering::Relaxed);
            let extension = tap.serve(Cursor::Now, 0);
            resume_tx.send(()).unwrap();
            assert_eq!(
                expiry.join().unwrap(),
                None,
                "the tick at 100 predates the lease re-armed at 150"
            );
            assert_eq!(extension.armed_since_ns, 150);
        });
        clock.now.store(249, Ordering::Relaxed);
        assert_eq!(tap.armed_now(), Some(249));
        clock.now.store(250, Ordering::Relaxed);
        assert_eq!(tap.armed_now(), None);
    }

    #[test]
    fn a_tick_that_sees_its_lease_end_during_a_live_extension_joins_it() {
        let (tap, clock, arrived_rx, resume_tx) = pausing_tap();
        tap.serve(Cursor::Now, 0);
        clock.now.store(95, Ordering::Relaxed);
        clock.pause.store(true, Ordering::Relaxed);
        std::thread::scope(|scope| {
            let reader = scope.spawn(|| tap.serve(Cursor::Now, 0));
            arrived_rx.recv().unwrap();
            clock.now.store(100, Ordering::Relaxed);
            clock.pause.store(true, Ordering::Relaxed);
            let tick = scope.spawn(|| tap.armed_now());
            arrived_rx.recv().unwrap();
            // The reader read 95 before the lease ended at 100; it extends first.
            resume_tx.send(()).unwrap();
            let extension = reader.join().unwrap();
            resume_tx.send(()).unwrap();
            assert_eq!(tick.join().unwrap(), Some(100));
            assert_eq!(extension.armed_since_ns, 0);
        });
        clock.now.store(194, Ordering::Relaxed);
        assert_eq!(tap.armed_now(), Some(194));
        clock.now.store(195, Ordering::Relaxed);
        assert_eq!(tap.armed_now(), None);
    }

    #[test]
    fn a_tick_that_read_an_ended_lease_cannot_join_a_later_one() {
        let (tap, clock, arrived_rx, resume_tx) = pausing_tap();
        tap.serve(Cursor::Now, 0);
        clock.now.store(150, Ordering::Relaxed);
        clock.pause.store(true, Ordering::Relaxed);
        let (stale, rearm) = std::thread::scope(|scope| {
            let stale = scope.spawn(|| tap.armed_now());
            arrived_rx.recv().unwrap();
            assert_eq!(tap.armed_now(), None, "another tick ends the lease");
            clock.now.store(160, Ordering::Relaxed);
            let rearm = tap.serve(Cursor::Now, 0);
            resume_tx.send(()).unwrap();
            (stale.join().unwrap(), rearm)
        });
        assert_eq!(rearm.armed_since_ns, 160);
        assert_eq!(
            stale, None,
            "the tick at 150 predates the lease armed at 160"
        );
    }

    #[test]
    fn rearming_after_a_refused_tick_starts_the_interval_after_it() {
        let (tap, clock, arrived_rx, resume_tx) = pausing_tap();
        tap.serve(Cursor::Now, 0);
        clock.now.store(99, Ordering::Relaxed);
        clock.pause.store(true, Ordering::Relaxed);
        let (refused, rearm) = std::thread::scope(|scope| {
            let reader = scope.spawn(|| tap.serve(Cursor::After(0), 512));
            arrived_rx.recv().unwrap();
            clock.now.store(105, Ordering::Relaxed);
            let refused = tap.armed_now();
            tap.push(body(105));
            resume_tx.send(()).unwrap();
            (refused, reader.join().unwrap())
        });
        assert_eq!(refused, None, "the lease ended at 100");
        assert_eq!((rearm.next, rearm.lost, rearm.records.len()), (0, 0, 0));
        assert!(
            rearm.armed_since_ns >= 105,
            "armed_since_ns={} predates the tick refused at 105",
            rearm.armed_since_ns
        );
        assert!(rearm.armed_since_ns <= rearm.now_ns);
        clock.now.store(106, Ordering::Relaxed);
        assert_eq!(tap.armed_now(), Some(106));
        tap.push(body(106));
        let after = tap.serve(Cursor::After(0), 512);
        assert_eq!(after.armed_since_ns, rearm.armed_since_ns);
        assert_eq!((after.next, after.lost), (1, 0));
        assert_eq!(after.records[0].at_ns, 106);
    }

    #[test]
    fn a_tick_admitted_while_the_reader_arms_lies_inside_the_lease_it_reports() {
        let (tap, clock, arrived_rx, resume_tx) = pausing_tap();
        // Any reader clock read after the first one comes after its arm CAS.
        clock.skip.store(1, Ordering::Relaxed);
        clock.late.store(true, Ordering::Relaxed);
        clock.pause.store(true, Ordering::Relaxed);
        let done = clock.arrived.clone();
        let (admitted, arm) = std::thread::scope(|scope| {
            let reader = scope.spawn(|| {
                let batch = tap.serve(Cursor::Now, 0);
                done.send(()).unwrap();
                batch
            });
            arrived_rx.recv().unwrap();
            // Still set only if the reader returned without a second read.
            let paused = !clock.pause.swap(false, Ordering::Relaxed);
            clock.now.store(10, Ordering::Relaxed);
            let admitted = tap.armed_now();
            if let Some(at_ns) = admitted {
                tap.push(body(at_ns));
            }
            clock.now.store(20, Ordering::Relaxed);
            if paused {
                resume_tx.send(()).unwrap();
            }
            (admitted, reader.join().unwrap())
        });
        assert_eq!(admitted, Some(10));
        assert_eq!(arm.armed_since_ns, 0, "the tick admitted at 10 predates it");
        assert!(arm.armed_since_ns <= arm.now_ns);
        clock.now.store(arm.armed_since_ns + 99, Ordering::Relaxed);
        assert_eq!(tap.armed_now(), Some(arm.armed_since_ns + 99));
        clock.now.store(arm.armed_since_ns + 100, Ordering::Relaxed);
        assert_eq!(
            tap.armed_now(),
            None,
            "the lease runs 100 ns from its start"
        );
        let page = tap.serve(Cursor::After(0), 512);
        assert_eq!((page.next, page.lost, page.records.len()), (1, 0, 1));
        assert_eq!(page.records[0].at_ns, 10);
        assert_body(&page.records[0]);
    }

    #[test]
    fn a_producer_that_loses_the_race_for_head_still_reserves_in_one_step() {
        let (tap, _) = tap(64);
        tap.serve(Cursor::Now, 0);
        let mut attempts = 0;
        let seq = tap.reserve(|| {
            attempts += 1;
            if attempts == 1 {
                tap.push(body(0));
            }
        });
        assert_eq!(attempts, 1, "a reservation never retries");
        assert_eq!(seq, Some(2));
        assert_eq!(tap.stats().recorded, 2);
    }

    #[test]
    fn sequence_exhaustion_freezes_without_stamp_reuse() {
        let (tap, _) = tap(64);
        tap.head.store(LAST_SEQ - 1, Ordering::Relaxed);
        tap.serve(Cursor::Now, 0);
        tap.push(body(7));
        tap.push(body(8));
        let batch = tap.serve(Cursor::After(LAST_SEQ - 1), 512);
        assert_eq!(batch.next, LAST_SEQ);
        assert_eq!(batch.lost, 0);
        assert_eq!(batch.records.len(), 1);
        assert_eq!(batch.records[0].at_ns, 7);
        assert_eq!(tap.stats().recorded, LAST_SEQ);
    }

    thread_local! {
        /// This thread's allocation count while `Some`; the lib test binary's
        /// allocator counts into it.
        pub(crate) static ALLOCATIONS: std::cell::Cell<Option<u64>> = const { std::cell::Cell::new(None) };
    }

    struct CountingAllocator;

    fn count_allocation() {
        let _ = ALLOCATIONS.try_with(|count| {
            if let Some(value) = count.get() {
                count.set(Some(value + 1));
            }
        });
    }

    // SAFETY: every call is delegated unchanged to System; the counter is
    // const-initialized TLS without allocation or a destructor. Every method is
    // forwarded because this allocator serves the whole lib test binary.
    unsafe impl std::alloc::GlobalAlloc for CountingAllocator {
        unsafe fn alloc(&self, layout: std::alloc::Layout) -> *mut u8 {
            count_allocation();
            unsafe { std::alloc::System.alloc(layout) }
        }
        unsafe fn alloc_zeroed(&self, layout: std::alloc::Layout) -> *mut u8 {
            count_allocation();
            unsafe { std::alloc::System.alloc_zeroed(layout) }
        }
        unsafe fn realloc(
            &self,
            ptr: *mut u8,
            layout: std::alloc::Layout,
            new_size: usize,
        ) -> *mut u8 {
            count_allocation();
            unsafe { std::alloc::System.realloc(ptr, layout, new_size) }
        }
        unsafe fn dealloc(&self, ptr: *mut u8, layout: std::alloc::Layout) {
            unsafe { std::alloc::System.dealloc(ptr, layout) }
        }
    }

    #[global_allocator]
    static ALLOCATOR: CountingAllocator = CountingAllocator;

    #[test]
    fn push_allocates_nothing_and_reads_no_clock() {
        let (tap, clock) = tap(64);
        tap.serve(Cursor::Now, 0);
        let reads = clock.reads.load(Ordering::Relaxed);
        ALLOCATIONS.set(Some(0));
        for id in 0..10_000 {
            tap.push(body(id));
        }
        let allocations = ALLOCATIONS.replace(None).unwrap();
        assert_eq!(allocations, 0);
        assert_eq!(clock.reads.load(Ordering::Relaxed), reads);
        assert_eq!(tap.stats().recorded, 10_000);

        ALLOCATIONS.set(Some(0));
        std::hint::black_box(Box::new(7_u64));
        let mut grown = std::hint::black_box(Vec::<u8>::with_capacity(1));
        grown.reserve(4096);
        std::hint::black_box(&grown);
        std::hint::black_box(vec![0_u64; 64]);
        let allocations = ALLOCATIONS.replace(None).unwrap();
        assert_eq!(
            allocations, 4,
            "allocation counter positive control: alloc, alloc, realloc, alloc_zeroed"
        );
        ALLOCATIONS.set(Some(0));
        assert_eq!(ALLOCATIONS.replace(None), Some(0));
    }
}

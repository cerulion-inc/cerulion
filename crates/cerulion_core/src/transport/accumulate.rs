// SPDX-License-Identifier: AGPL-3.0-only
//! The keep-every-message input: `#[input(accumulate)]`.
//!
//! A plain non-trigger input is a latest-value read: the step-boundary
//! snapshot drains its queue and keeps only the newest frame. An accumulate
//! input keeps EVERY frame that arrived since the node's last fire, up to a
//! declared cap, and serves them to the next fire in one defined order. It is
//! the read a many-writer topic such as `/tf` needs, where each writer owns
//! different transforms and a latest-value read would lose all but one.
//!
//! # Order
//!
//! Frames are served oldest first by the wire publish timestamp
//! (`timestamp_ns`), then by the wire `sequence`, then by the frame bytes.
//! The key is a pure function of the frames themselves, never of arrival
//! order or of which writer's queue the transport reads first, so two runs
//! that publish the same frames serve them in the same order. Two frames
//! with an equal key are byte-identical, so their relative order cannot be
//! observed.
//!
//! # Cap and overflow
//!
//! The buffer holds at most `cap` frames. When it is full, the OLDEST frame
//! by the order above is dropped: an arriving frame older than every held
//! one is itself the one dropped, otherwise it replaces the oldest held
//! frame. Every drop is counted ([`AccumulateBuffer::dropped_total`]), with
//! no regime or rate limit on the count.
//!
//! # Storage
//!
//! Frames are copied out of the transport sample into slots the buffer
//! owns, because the transport lends a bounded number of samples per
//! subscriber and a held sample per frame would exhaust it. Each slot keeps
//! its allocation across fires, so once every slot has held a frame of the
//! largest size the topic carries, collecting and serving allocates nothing.
//! Slots are 8-byte words so a frame view is aligned the way the wire
//! format requires.

use std::cmp::Ordering;
use std::marker::PhantomData;
use std::sync::Arc;

use super::failure_regime_latch::{FailureRegimeLatch, RegimeDecision};
use super::input_view::{validate_wire_frame, InputView};
use super::shm_sample::SampleHandle;
use crate::error::{TransportError, TransportResult};
use crate::graph::node::{AnySubscriber, BackpressurePolicy, NodeContext};
use crate::graph::topology::{DEFAULT_CONSUMER_DEPTH, MAX_CONSUMER_DEPTH};
use crate::message::ShmMessage;
use crate::scheduler::BackpressureEvent;
use crate::wire::WireHeader;

/// The cap an `#[input(accumulate)]` without `cap = N` gets.
pub const DEFAULT_ACCUMULATE_CAP: usize = DEFAULT_CONSUMER_DEPTH;

/// The largest accepted `cap`. The cap is also the input's transport queue
/// depth, so it shares the queue depth ceiling.
pub const MAX_ACCUMULATE_CAP: usize = MAX_CONSUMER_DEPTH;

/// What [`AccumulateBuffer::insert`] did with one frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InsertOutcome {
    /// The buffer had room; the frame is held.
    Stored,
    /// The buffer was full; the frame is held and the oldest held frame was
    /// dropped.
    ReplacedOldest,
    /// The buffer was full and the frame was older than every held frame,
    /// so it is the one dropped.
    DroppedArrival,
}

impl InsertOutcome {
    /// Did this insert drop a frame?
    pub fn dropped(self) -> bool {
        !matches!(self, Self::Stored)
    }
}

#[derive(Debug, Default)]
struct Slot {
    words: Vec<u64>,
    len: usize,
    timestamp_ns: u64,
    sequence: u32,
}

impl Slot {
    fn bytes(&self) -> &[u8] {
        // SAFETY: `words` owns `words.len() * 8` initialised bytes and `len`
        // never exceeds that (see `fill`); u8 has no alignment requirement.
        let all = unsafe {
            std::slice::from_raw_parts(self.words.as_ptr().cast::<u8>(), self.words.len() * 8)
        };
        &all[..self.len]
    }

    fn fill(&mut self, frame: &[u8], header: &WireHeader) {
        let n_words = frame.len().div_ceil(8);
        // Grows only when this slot has never held a frame this large; the
        // capacity is kept across fires.
        self.words.resize(n_words, 0); // hot-path-alloc-ok: grows only past the largest frame this slot has held
                                       // SAFETY: `words` now holds `n_words * 8 >= frame.len()` bytes.
        let dst = unsafe {
            std::slice::from_raw_parts_mut(self.words.as_mut_ptr().cast::<u8>(), n_words * 8)
        };
        dst[..frame.len()].copy_from_slice(frame);
        self.len = frame.len();
        self.timestamp_ns = header.timestamp_ns;
        self.sequence = header.sequence;
    }
}

/// The serve order of two frames: publish timestamp, then wire sequence,
/// then the bytes. Total over frames, and independent of arrival order.
pub fn frame_order(a_ts: u64, a_seq: u32, a: &[u8], b_ts: u64, b_seq: u32, b: &[u8]) -> Ordering {
    a_ts.cmp(&b_ts)
        .then(a_seq.cmp(&b_seq))
        .then_with(|| a.cmp(b))
}

fn slot_order(a: &Slot, b: &Slot) -> Ordering {
    frame_order(
        a.timestamp_ns,
        a.sequence,
        a.bytes(),
        b.timestamp_ns,
        b.sequence,
        b.bytes(),
    )
}

/// The bounded, ordered frame store behind one accumulate input.
#[derive(Debug)]
pub struct AccumulateBuffer {
    cap: usize,
    /// `slots[..held]` are live; slots past `held` keep their storage.
    slots: Vec<Slot>,
    held: usize,
    /// The frames have been handed to a fire; the next collect starts empty.
    served: bool,
    /// The fire now running is the one the frames were handed to (a later
    /// fire in the same step sees none).
    visible: bool,
    dropped_total: u64,
    dropped_since_serve: u64,
}

impl AccumulateBuffer {
    /// A buffer that holds at most `cap` frames.
    ///
    /// # Errors
    ///
    /// [`TransportError::InvalidTransportConfig`] when `cap` is 0 or above
    /// [`MAX_ACCUMULATE_CAP`].
    pub fn new(cap: usize) -> TransportResult<Self> {
        if cap == 0 || cap > MAX_ACCUMULATE_CAP {
            return Err(TransportError::InvalidTransportConfig {
                reason: format!(
                    "accumulate cap {cap} is out of range: the accepted range is 1..={MAX_ACCUMULATE_CAP}"
                ),
            });
        }
        let mut slots = Vec::with_capacity(cap); // hot-path-alloc-ok: constructor, runs once at node init
        slots.resize_with(cap, Slot::default);
        Ok(Self {
            cap,
            slots,
            held: 0,
            served: false,
            visible: false,
            dropped_total: 0,
            dropped_since_serve: 0,
        })
    }

    /// The declared cap.
    pub fn cap(&self) -> usize {
        self.cap
    }

    /// Frames held now.
    pub fn len(&self) -> usize {
        self.held
    }

    /// No frame is held.
    pub fn is_empty(&self) -> bool {
        self.held == 0
    }

    /// Every frame this input has dropped at the cap, over its lifetime.
    pub fn dropped_total(&self) -> u64 {
        self.dropped_total
    }

    /// Frames dropped at the cap since the last [`Self::serve`].
    pub fn dropped_since_serve(&self) -> u64 {
        self.dropped_since_serve
    }

    /// Start a step-boundary collect. Frames a fire already received are
    /// released; frames no fire has received yet stay held, so a fire that
    /// did not run (its body was skipped) loses nothing.
    pub fn begin_collect(&mut self) {
        if self.served {
            self.held = 0;
            self.served = false;
            self.dropped_since_serve = 0;
        }
        self.visible = false;
    }

    /// Take one full wire frame (header included).
    ///
    /// # Errors
    ///
    /// [`TransportError::Deserialization`] (naming `topic`) when `frame` has
    /// no readable wire header or the header's `total_size` disagrees with
    /// `frame`. Nothing is held or dropped.
    pub fn insert(&mut self, topic: &str, frame: &[u8]) -> TransportResult<InsertOutcome> {
        let Some(header) =
            WireHeader::read_from_buf(frame).filter(|h| h.total_size as usize == frame.len())
        else {
            return Err(TransportError::Deserialization {
                topic: topic.to_string(),
                reason: format!(
                    "accumulate frame of {} bytes has no valid wire header",
                    frame.len()
                ),
            });
        };
        if self.held < self.cap {
            self.slots[self.held].fill(frame, &header);
            self.held += 1;
            return Ok(InsertOutcome::Stored);
        }
        let mut oldest = 0;
        for i in 1..self.held {
            if slot_order(&self.slots[i], &self.slots[oldest]) == Ordering::Less {
                oldest = i;
            }
        }
        let o = &self.slots[oldest];
        let arrival_vs_oldest = frame_order(
            header.timestamp_ns,
            header.sequence,
            frame,
            o.timestamp_ns,
            o.sequence,
            o.bytes(),
        );
        self.dropped_total += 1;
        self.dropped_since_serve += 1;
        if arrival_vs_oldest == Ordering::Greater {
            self.slots[oldest].fill(frame, &header);
            Ok(InsertOutcome::ReplacedOldest)
        } else {
            Ok(InsertOutcome::DroppedArrival)
        }
    }

    /// End a collect: put the held frames in serve order.
    pub fn finish_collect(&mut self) {
        self.slots[..self.held].sort_unstable_by(slot_order);
    }

    /// Hand the held frames to the fire now running. A second call before
    /// the next collect hands over nothing, so a node that fires several
    /// times in one step receives each frame once.
    pub fn mark_served(&mut self) {
        self.visible = !self.served;
        self.served = true;
    }

    /// The frames handed to the fire now running, oldest first (empty when
    /// [`Self::mark_served`] handed them to an earlier fire).
    pub fn frames(&self) -> Frames<'_> {
        if self.visible {
            Frames {
                slots: &self.slots[..self.held],
            }
        } else {
            Frames { slots: &[] }
        }
    }

    /// Frames dropped at the cap before the fire now running received its
    /// frames (0 for a later fire in the same step).
    pub fn dropped_for_this_fire(&self) -> u64 {
        if self.visible {
            self.dropped_since_serve
        } else {
            0
        }
    }

    /// [`Self::mark_served`] then [`Self::frames`].
    pub fn serve(&mut self) -> Frames<'_> {
        self.mark_served();
        self.frames()
    }
}

/// The frames one fire received, oldest first.
#[derive(Debug, Clone, Copy)]
pub struct Frames<'a> {
    slots: &'a [Slot],
}

impl<'a> Frames<'a> {
    /// How many frames this fire received.
    pub fn len(&self) -> usize {
        self.slots.len()
    }

    /// This fire received no frame.
    pub fn is_empty(&self) -> bool {
        self.slots.is_empty()
    }

    /// The full wire frames (8-byte aligned), oldest first.
    pub fn iter(&self) -> impl Iterator<Item = &'a [u8]> + 'a {
        self.slots.iter().map(Slot::bytes)
    }
}

/// Checks one full wire frame against an accumulate input's schema, with
/// the same frame checks a typed read applies. Monomorphised per input at
/// registration so the type-erased collect still refuses what a typed read
/// would.
type FrameCheck = fn(&str, &[u8]) -> TransportResult<()>;

fn check_frame<T: ShmMessage>(topic: &str, raw: &[u8]) -> TransportResult<()> {
    validate_wire_frame::<T>(topic, raw).map(|_| ())
}

#[derive(Debug)]
struct Accumulator {
    name: String,
    buffer: AccumulateBuffer,
    check: FrameCheck,
    /// Edge trigger for the overflow event: one event per overflow regime,
    /// re-armed by a collect that drops nothing.
    armed: bool,
    /// Flood suppression for refused frames and receive faults.
    faults: FailureRegimeLatch,
}

/// Every accumulate input of one node, keyed by input name in declaration
/// order. Owned by the node's generated entry, beside its `NodeContext`.
#[derive(Debug, Default)]
pub struct AccumulatorSet {
    inputs: Vec<Accumulator>,
}

impl AccumulatorSet {
    /// An empty set (a node with no accumulate input).
    pub fn new() -> Self {
        Self::default()
    }

    /// Declare `name` an accumulate input holding at most `cap` frames of
    /// schema `T`. Called once from the node's generated `init`, before the
    /// first step. The input's subscriber, when the graph wired one, switches
    /// to one-frame-per-read consumption so the collect pops every queued
    /// frame through the ordinary accounted read path.
    ///
    /// # Errors
    ///
    /// [`TransportError::InvalidTransportConfig`] for a cap outside
    /// `1..=MAX_ACCUMULATE_CAP`, or a name registered twice.
    pub fn register<T: ShmMessage>(
        &mut self,
        ctx: &mut NodeContext,
        name: &str,
        cap: usize,
    ) -> TransportResult<()> {
        if self.inputs.iter().any(|a| a.name == name) {
            return Err(TransportError::InvalidTransportConfig {
                reason: format!("accumulate input '{name}' is registered twice"),
            });
        }
        let buffer = AccumulateBuffer::new(cap)?;
        if let Some(sub) = ctx.subscriber_mut(name) {
            sub.mark_fifo_consume();
        }
        self.inputs.push(Accumulator {
            name: name.to_string(),
            buffer,
            check: check_frame::<T>,
            armed: true,
            faults: FailureRegimeLatch::new(),
        });
        Ok(())
    }

    /// Is `name` an accumulate input?
    pub fn contains(&self, name: &str) -> bool {
        self.inputs.iter().any(|a| a.name == name)
    }

    /// The step-boundary freeze for one node: every accumulate input in
    /// `names` collects the frames queued since the last collect, and every
    /// other name takes the ordinary latest-value snapshot. This is the one
    /// freeze both the in-process entry and the cdylib export call, so the
    /// two node kinds read accumulate inputs identically.
    pub fn snapshot(&mut self, ctx: &mut NodeContext, names: &[String]) {
        for name in names {
            match self.inputs.iter_mut().find(|a| a.name == *name) {
                Some(acc) => match ctx.subscriber_mut(name) {
                    Some(sub) => acc.collect(sub),
                    None => {
                        debug_assert!(
                            false,
                            "snapshot: accumulate input '{name}' not a wired subscriber"
                        );
                        tracing::warn!(
                            input = %name,
                            "accumulate snapshot: unknown input name skipped (wiring desync)"
                        );
                    }
                },
                None => ctx.snapshot_inputs(std::slice::from_ref(name)),
            }
        }
    }

    /// Hand `name`'s frames to the fire now running (the generated tick
    /// calls this just before the node body, and only when the body runs).
    /// A second call before the next [`Self::snapshot`] hands over nothing,
    /// so a node fired several times in one step receives each frame once.
    pub fn mark_served(&mut self, name: &str) {
        if let Some(acc) = self.inputs.iter_mut().find(|a| a.name == name) {
            acc.buffer.mark_served();
        }
    }

    /// The frames [`Self::mark_served`] handed to the fire now running.
    /// An unknown name yields an empty view (the generated tick only asks
    /// for declared accumulate inputs).
    pub fn view<T: ShmMessage>(&self, name: &str) -> Accumulated<'_, T> {
        match self.inputs.iter().find(|a| a.name == name) {
            Some(acc) => Accumulated {
                topic: acc.name.as_str(),
                frames: acc.buffer.frames(),
                dropped: acc.buffer.dropped_for_this_fire(),
                dropped_total: acc.buffer.dropped_total(),
                _schema: PhantomData,
            },
            None => Accumulated {
                topic: "",
                frames: Frames { slots: &[] },
                dropped: 0,
                dropped_total: 0,
                _schema: PhantomData,
            },
        }
    }
}

impl Accumulator {
    fn collect(&mut self, sub: &mut AnySubscriber) {
        self.buffer.begin_collect();
        let before = self.buffer.dropped_total();
        let mut newest_ts = 0u64;
        loop {
            let frame = sub.view_raw();
            let outcome = match frame {
                Ok(Some(view)) => (self.check)(&self.name, &view)
                    .and_then(|()| self.buffer.insert(&self.name, &view))
                    .map(|_| {
                        newest_ts = newest_ts.max(view.header().timestamp_ns);
                    }),
                Ok(None) => break,
                Err(e) => Err(e),
            };
            if let Err(error) = outcome {
                // A refused frame is popped and counted; a receive fault ends
                // this collect (the queue is read again at the next one).
                match self.faults.on_failure() {
                    RegimeDecision::Suppressed { .. } => tracing::debug!(
                        input = %self.name,
                        error = %error,
                        total_failures = self.faults.total_failures(),
                        "accumulate input refused a frame"
                    ),
                    _ => tracing::warn!(
                        input = %self.name,
                        error = %error,
                        total_failures = self.faults.total_failures(),
                        "accumulate input refused a frame"
                    ),
                }
                if !matches!(
                    error,
                    TransportError::Deserialization { .. }
                        | TransportError::SchemaMismatch { .. }
                        | TransportError::SchemaHashMismatch { .. }
                ) {
                    break;
                }
            } else if let Some(suppressed) = self.faults.on_success() {
                tracing::info!(
                    input = %self.name,
                    suppressed,
                    total_failures = self.faults.total_failures(),
                    "accumulate input reads frames again"
                );
            }
        }
        self.buffer.finish_collect();
        let dropped_now = self.buffer.dropped_total() - before;
        if dropped_now == 0 {
            self.armed = true;
            return;
        }
        if self.armed {
            self.armed = false;
            tracing::warn!(
                input = %self.name,
                cap = self.buffer.cap(),
                dropped = dropped_now,
                total = self.buffer.dropped_total(),
                "accumulate input overflowed its cap: oldest frames dropped"
            );
            let AnySubscriber::Ipc(ipc) = sub;
            ipc.queue_backpressure_event(BackpressureEvent {
                policy: BackpressurePolicy::DropOldest,
                input_name: Arc::from(self.name.as_str()), // hot-path-alloc-ok: once per overflow regime, never per frame
                count_total: self.buffer.dropped_total(),
                count_in_regime: dropped_now,
                dropped: dropped_now,
                regime_started_at_ns: newest_ts,
                buffer_capacity: self.buffer.cap(),
            });
        } else {
            tracing::debug!(
                input = %self.name,
                dropped = dropped_now,
                total = self.buffer.dropped_total(),
                "accumulate input still over its cap"
            );
        }
    }
}

/// What one fire received on an `#[input(accumulate)]` port: every frame
/// since the node's last fire, oldest first (see the module docs for the
/// order and the cap).
pub struct Accumulated<'a, T: ShmMessage> {
    topic: &'a str,
    frames: Frames<'a>,
    dropped: u64,
    dropped_total: u64,
    _schema: PhantomData<fn() -> T>,
}

impl<'a, T: ShmMessage + 'a> Accumulated<'a, T> {
    /// Frames this fire received.
    pub fn len(&self) -> usize {
        self.frames.len()
    }

    /// This fire received no frame (nothing arrived since the last fire).
    pub fn is_empty(&self) -> bool {
        self.frames.is_empty()
    }

    /// Frames dropped at the cap before this fire could receive them.
    pub fn dropped(&self) -> u64 {
        self.dropped
    }

    /// Every frame this input has dropped at the cap, over its lifetime.
    pub fn dropped_total(&self) -> u64 {
        self.dropped_total
    }

    /// The frames, oldest first, each read the same way a plain `#[input]`
    /// is read (`Deref` to the schema reader, `wire_header()`).
    pub fn iter(&self) -> impl Iterator<Item = InputView<'a, T>> + 'a {
        let topic = self.topic;
        self.frames.iter().filter_map(move |raw| {
            // Every held frame passed `validate_wire_frame::<T>` at collect,
            // and the slot holds the same bytes at the same alignment, so this
            // cannot refuse; a refusal would be a bug, never a frame to serve.
            match validate_wire_frame::<T>(topic, raw) {
                Ok((header, payload)) => Some(InputView::new(
                    SampleHandle::ScheduledBytes { bytes: raw },
                    T::build_reader(&raw[payload]),
                    header,
                )),
                Err(error) => {
                    debug_assert!(false, "accumulate frame failed its re-check: {error}");
                    None
                }
            }
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A hand-built wire frame: header (seq, ts) plus `body`.
    fn frame(seq: u32, ts: u64, body: &[u8]) -> Vec<u8> {
        let mut h = WireHeader::new(0xAB, seq, ts);
        h.total_size = (WireHeader::SIZE + body.len()) as u32;
        let mut out = vec![0u8; WireHeader::SIZE + body.len()];
        h.write_to_buf(&mut out[..WireHeader::SIZE]);
        out[WireHeader::SIZE..].copy_from_slice(body);
        out
    }

    /// (seq, ts, first body byte) of each served frame.
    fn served(buf: &mut AccumulateBuffer) -> Vec<(u32, u64, u8)> {
        buf.serve()
            .iter()
            .map(|f| {
                let h = WireHeader::read_from_buf(f).expect("served frames have headers");
                (h.sequence, h.timestamp_ns, f[WireHeader::SIZE])
            })
            .collect()
    }

    fn collect(buf: &mut AccumulateBuffer, frames: &[Vec<u8>]) -> Vec<InsertOutcome> {
        buf.begin_collect();
        let out = frames
            .iter()
            .map(|f| buf.insert("/t", f).expect("well-formed frame"))
            .collect();
        buf.finish_collect();
        out
    }

    #[test]
    fn two_writers_interleaved_out_of_order_are_served_by_timestamp_then_sequence() {
        // Writer A publishes body 0xA_, writer B body 0xB_. The transport
        // hands writer B's queue over first.
        let mut buf = AccumulateBuffer::new(8).unwrap();
        collect(
            &mut buf,
            &[
                frame(0, 100, &[0xB0]),
                frame(1, 300, &[0xB1]),
                frame(0, 200, &[0xA0]),
                frame(1, 300, &[0xA1]),
            ],
        );
        assert_eq!(
            served(&mut buf),
            vec![
                (0, 100, 0xB0),
                (0, 200, 0xA0),
                (1, 300, 0xA1),
                (1, 300, 0xB1)
            ],
            "ts ascending; the (300, seq 1) tie breaks on the bytes, 0xA1 < 0xB1"
        );
    }

    #[test]
    fn the_order_does_not_depend_on_arrival_order() {
        let frames = [
            frame(3, 50, &[1]),
            frame(1, 50, &[2]),
            frame(0, 10, &[3]),
            frame(1, 50, &[1]),
        ];
        let mut forward = AccumulateBuffer::new(4).unwrap();
        collect(&mut forward, &frames);
        let mut reversed_input = frames.to_vec();
        reversed_input.reverse();
        let mut backward = AccumulateBuffer::new(4).unwrap();
        collect(&mut backward, &reversed_input);
        let want = vec![(0, 10, 3), (1, 50, 1), (1, 50, 2), (3, 50, 1)];
        assert_eq!(served(&mut forward), want);
        assert_eq!(served(&mut backward), want);
    }

    #[test]
    fn an_empty_collect_serves_an_empty_fire() {
        let mut buf = AccumulateBuffer::new(4).unwrap();
        collect(&mut buf, &[]);
        let frames = buf.serve();
        assert!(frames.is_empty());
        assert_eq!(frames.len(), 0);
        assert_eq!(buf.dropped_total(), 0);
    }

    #[test]
    fn exactly_at_the_cap_holds_every_frame_and_drops_none() {
        let mut buf = AccumulateBuffer::new(3).unwrap();
        let outcomes = collect(
            &mut buf,
            &[frame(0, 1, &[0]), frame(1, 2, &[0]), frame(2, 3, &[0])],
        );
        assert_eq!(outcomes, vec![InsertOutcome::Stored; 3]);
        assert_eq!(served(&mut buf), vec![(0, 1, 0), (1, 2, 0), (2, 3, 0)]);
        assert_eq!(buf.dropped_total(), 0);
    }

    #[test]
    fn one_past_the_cap_drops_the_oldest_frame() {
        let mut buf = AccumulateBuffer::new(3).unwrap();
        let outcomes = collect(
            &mut buf,
            &[
                frame(0, 1, &[0]),
                frame(1, 2, &[0]),
                frame(2, 3, &[0]),
                frame(3, 4, &[0]),
            ],
        );
        assert_eq!(outcomes[3], InsertOutcome::ReplacedOldest);
        assert_eq!(served(&mut buf), vec![(1, 2, 0), (2, 3, 0), (3, 4, 0)]);
        assert_eq!(buf.dropped_total(), 1);
    }

    #[test]
    fn a_late_arrival_older_than_every_held_frame_is_the_one_dropped() {
        let mut buf = AccumulateBuffer::new(2).unwrap();
        let outcomes = collect(
            &mut buf,
            &[frame(5, 50, &[0]), frame(6, 60, &[0]), frame(1, 10, &[0])],
        );
        assert_eq!(outcomes[2], InsertOutcome::DroppedArrival);
        assert!(outcomes[2].dropped());
        assert_eq!(served(&mut buf), vec![(5, 50, 0), (6, 60, 0)]);
        assert_eq!(buf.dropped_total(), 1);
    }

    #[test]
    fn a_writer_flood_keeps_the_newest_cap_frames_and_counts_every_drop() {
        // Hand oracle: 20 frames ts 1..=20 from one writer plus 5 frames
        // ts 100..=104 from another, cap 4. Kept: the four newest,
        // 101..=104. Dropped: 25 - 4 = 21.
        let mut frames: Vec<Vec<u8>> = (1..=20u32)
            .map(|i| frame(i, u64::from(i), &[0xA]))
            .collect();
        frames.extend((0..5u32).map(|i| frame(i, 100 + u64::from(i), &[0xB])));
        let mut buf = AccumulateBuffer::new(4).unwrap();
        collect(&mut buf, &frames);
        assert_eq!(
            served(&mut buf),
            vec![(1, 101, 0xB), (2, 102, 0xB), (3, 103, 0xB), (4, 104, 0xB)]
        );
        assert_eq!(buf.dropped_total(), 21);
        assert_eq!(buf.dropped_since_serve(), 21);
    }

    #[test]
    fn served_frames_are_released_at_the_next_collect_and_a_second_serve_is_empty() {
        let mut buf = AccumulateBuffer::new(4).unwrap();
        collect(&mut buf, &[frame(0, 1, &[1])]);
        assert_eq!(served(&mut buf), vec![(0, 1, 1)]);
        assert!(
            buf.serve().is_empty(),
            "a second fire in the same step gets nothing"
        );
        collect(&mut buf, &[frame(1, 2, &[2])]);
        assert_eq!(
            served(&mut buf),
            vec![(1, 2, 2)],
            "only frames since the last fire"
        );
    }

    #[test]
    fn unserved_frames_carry_into_the_next_collect() {
        // A fire whose body did not run never called serve: its frames are
        // still owed and join the next collect, in order.
        let mut buf = AccumulateBuffer::new(4).unwrap();
        collect(&mut buf, &[frame(1, 20, &[0])]);
        collect(&mut buf, &[frame(0, 10, &[0])]);
        assert_eq!(served(&mut buf), vec![(0, 10, 0), (1, 20, 0)]);
    }

    #[test]
    fn the_drop_counter_survives_serves_and_the_per_fire_count_resets() {
        let mut buf = AccumulateBuffer::new(1).unwrap();
        collect(&mut buf, &[frame(0, 1, &[0]), frame(1, 2, &[0])]);
        let _ = buf.serve();
        collect(
            &mut buf,
            &[frame(2, 3, &[0]), frame(3, 4, &[0]), frame(4, 5, &[0])],
        );
        assert_eq!(buf.dropped_since_serve(), 2);
        assert_eq!(buf.dropped_total(), 3);
    }

    #[test]
    fn served_frames_are_eight_byte_aligned_and_byte_exact() {
        let original = frame(7, 9, &[1, 2, 3, 4, 5]);
        let mut buf = AccumulateBuffer::new(2).unwrap();
        collect(&mut buf, std::slice::from_ref(&original));
        let frames = buf.serve();
        let got = frames.iter().next().unwrap();
        assert_eq!(got, original.as_slice());
        assert_eq!(got.as_ptr() as usize % 8, 0);
    }

    #[test]
    fn cap_zero_and_cap_above_the_ceiling_are_refused() {
        assert_eq!(
            AccumulateBuffer::new(0).unwrap_err().to_string(),
            "Invalid transport configuration: accumulate cap 0 is out of range: \
             the accepted range is 1..=64"
        );
        assert_eq!(
            AccumulateBuffer::new(65).unwrap_err().to_string(),
            "Invalid transport configuration: accumulate cap 65 is out of range: \
             the accepted range is 1..=64"
        );
        assert_eq!(AccumulateBuffer::new(64).unwrap().cap(), 64);
        assert_eq!(DEFAULT_ACCUMULATE_CAP, 10);
    }

    #[test]
    fn a_frame_without_a_valid_header_is_refused_and_changes_nothing() {
        let mut buf = AccumulateBuffer::new(2).unwrap();
        buf.begin_collect();
        assert_eq!(
            buf.insert("/tf", &[0u8; 8]).unwrap_err().to_string(),
            "Deserialization error on topic '/tf': accumulate frame of 8 bytes has no valid wire header"
        );
        // A header whose total_size disagrees with the bytes handed over.
        let mut lying = frame(0, 1, &[0, 0]);
        lying.truncate(lying.len() - 1);
        assert_eq!(
            buf.insert("/tf", &lying).unwrap_err().to_string(),
            "Deserialization error on topic '/tf': accumulate frame of 33 bytes has no valid wire header"
        );
        buf.finish_collect();
        assert!(buf.is_empty());
        assert_eq!(buf.dropped_total(), 0);
    }

    #[test]
    fn slot_storage_is_reused_across_fires() {
        let mut buf = AccumulateBuffer::new(2).unwrap();
        collect(&mut buf, &[frame(0, 1, &[0; 40]), frame(1, 2, &[0; 40])]);
        let _ = buf.serve();
        let ptrs: Vec<*const u64> = buf.slots.iter().map(|s| s.words.as_ptr()).collect();
        collect(&mut buf, &[frame(2, 3, &[0; 16]), frame(3, 4, &[0; 40])]);
        let after: Vec<*const u64> = buf.slots.iter().map(|s| s.words.as_ptr()).collect();
        assert_eq!(
            ptrs, after,
            "no slot reallocated for frames no larger than before"
        );
    }
}

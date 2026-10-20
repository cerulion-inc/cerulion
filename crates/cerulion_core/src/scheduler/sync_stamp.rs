// SPDX-License-Identifier: AGPL-3.0-only
//! Capture-time (`header.stamp`) matching for the per-set Sync trigger.
//!
//! # Publish time versus capture time
//!
//! By default a Sync node aligns its trigger inputs on each frame's WIRE
//! stamp: the transport clock when the producer loaned the frame. That is the
//! wrong clock for sensor fusion. A camera frame and a lidar sweep captured at
//! the same moment can be published tens of milliseconds apart, so on publish
//! time they fall into different windows or pair with the wrong partner.
//!
//! A node that opts in to CAPTURE-TIME matching aligns on the message's own
//! `header.stamp` instead. Nothing else changes: the matcher in
//! [`super::sync_match`] is the same pure function over the same head tuple,
//! only the number in each head is the capture stamp. This module is the
//! translation layer between the transport (which answers with wire stamps)
//! and that matcher, and every decision in it is a pure function of the frame
//! bytes and the stamps already accepted, so two runs over the same frames
//! pick the same sets.
//!
//! Capture-time matching needs a window (`sync_window_ms`): the window is
//! what tells a frame that arrived out of capture order apart from a capture
//! clock that restarted.
//!
//! # The three rules this layer adds
//!
//! * **Decode**: [`capture_stamp_ns`] reads `header.stamp` at a location
//!   resolved once from the schema ([`HeaderStampLocation`]). A frame whose
//!   stamp cannot be decoded is DISCARDED and counted, never matched on a
//!   made-up number.
//! * **Order**: the matcher requires each input's member stamps to be
//!   non-decreasing (its in-order rule). Wire stamps are monotone per writer by
//!   construction; capture stamps are not (a driver can publish frames out of
//!   capture order). [`order_capture_stamp`] classifies a new head: a stamp
//!   below the input's newest accepted one, by at most the window, is
//!   DISCARDED and counted; a drop of MORE than the window is a clock restart
//!   (a simulator reset, a looped bag) and is accepted so the scheduler's
//!   existing epoch reset re-bases the node.
//! * **Peek**: a staged next frame that would not be accepted as a head
//!   answers "nothing to descend to". It is judged again, and counted, when it
//!   reaches the head, so every frame is counted at most once.

use crate::scheduler::{SyncAlignSite, SyncHeadOp, SyncOpAnswer};
use crate::wire::WireHeader;

/// Nanoseconds per second, the `builtin_interfaces/Time` scale.
const NS_PER_SEC: u64 = 1_000_000_000;

/// Upper bound on frames one head op may discard before it gives up.
///
/// Each discard pops a real frame, so the loop always ends when the queue
/// does; this clamp only bounds a producer that refills the queue faster than
/// the loop can drain it. Hitting it voids the last discarded frame (so it is
/// never offered, or counted, again) and answers [`SyncOpAnswer::Failed`],
/// which the align driver treats as "no head this pass"; the next boundary
/// resumes the walk.
const MAX_DISCARDS_PER_OP: u32 = 256;

/// Where `header.stamp` lives in a frame, resolved once per input from the
/// input's schema (see [`crate::codegen::header_stamp_location`]).
///
/// Offsets are measured from the first byte AFTER the [`WireHeader`], the same
/// origin the offset table uses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HeaderStampLocation {
    /// `header` is a fixed nested message, inlined in the fixed section. The
    /// stamp's `sec` sits at `offset`, `nanosec` at `offset + 4`.
    Fixed {
        /// Offset of `header.stamp.sec` in the fixed section.
        offset: usize,
    },
    /// `header` is a variable nested message (the `std_msgs/Header` case: its
    /// `frame_id` string makes it variable), stored as a headerless sub-frame
    /// through the offset table.
    Table {
        /// Size of the parent's fixed section: the offset table starts here.
        fixed_size: usize,
        /// The `header` field's index in the offset table.
        index: usize,
        /// Offset of `stamp.sec` inside the header sub-frame's fixed section.
        stamp_offset: usize,
    },
}

/// Why a frame's `header.stamp` could not be decoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum StampDecodeError {
    /// The frame ends before the bytes the location names.
    #[error("frame is {len} bytes but header.stamp needs {needed} bytes")]
    Truncated {
        /// Bytes the read needed, counted from the start of the frame.
        needed: usize,
        /// Bytes the frame has.
        len: usize,
    },
    /// `stamp.sec` is negative: a time before the epoch, which no capture
    /// clock produces and which has no nanosecond value on this scale.
    #[error("header.stamp.sec is negative ({sec})")]
    NegativeSeconds {
        /// The raw `sec` value.
        sec: i32,
    },
    /// `stamp.nanosec` is one second or more, which is not a valid
    /// `builtin_interfaces/Time`.
    #[error("header.stamp.nanosec is {nanosec}, one second or more")]
    NanosecOutOfRange {
        /// The raw `nanosec` value.
        nanosec: u32,
    },
}

/// Read four little-endian bytes at `at`, or say how short the frame is.
fn read_u32(frame: &[u8], at: usize) -> Result<u32, StampDecodeError> {
    let end = at.saturating_add(4);
    match frame.get(at..end) {
        Some(bytes) => {
            let mut word = [0u8; 4];
            word.copy_from_slice(bytes);
            Ok(u32::from_le_bytes(word))
        }
        None => Err(StampDecodeError::Truncated {
            needed: end,
            len: frame.len(),
        }),
    }
}

/// Decode `header.stamp` from a whole wire frame (WireHeader included) as
/// nanoseconds since the epoch.
///
/// Pure and allocation-free: a handful of bounds-checked slice reads.
pub fn capture_stamp_ns(
    frame: &[u8],
    location: HeaderStampLocation,
) -> Result<u64, StampDecodeError> {
    let body = WireHeader::SIZE;
    let sec_at = match location {
        HeaderStampLocation::Fixed { offset } => body.saturating_add(offset),
        HeaderStampLocation::Table {
            fixed_size,
            index,
            stamp_offset,
        } => {
            let entry = body
                .saturating_add(fixed_size)
                .saturating_add(index.saturating_mul(8));
            let sub_offset = read_u32(frame, entry)? as usize;
            let sub_len = read_u32(frame, entry.saturating_add(4))? as usize;
            let stamp_end = stamp_offset.saturating_add(8);
            if sub_len < stamp_end {
                // The sub-frame is shorter than the stamp it must hold. Reported
                // against the sub-frame's own end, which is where the read
                // would have stopped.
                return Err(StampDecodeError::Truncated {
                    needed: body.saturating_add(sub_offset).saturating_add(stamp_end),
                    len: body.saturating_add(sub_offset).saturating_add(sub_len),
                });
            }
            body.saturating_add(sub_offset).saturating_add(stamp_offset)
        }
    };
    let sec = read_u32(frame, sec_at)? as i32;
    let nanosec = read_u32(frame, sec_at.saturating_add(4))?;
    if sec < 0 {
        return Err(StampDecodeError::NegativeSeconds { sec });
    }
    if u64::from(nanosec) >= NS_PER_SEC {
        return Err(StampDecodeError::NanosecOutOfRange { nanosec });
    }
    Ok(sec as u64 * NS_PER_SEC + u64::from(nanosec))
}

/// How a new head's capture stamp relates to the newest one this input has
/// already contributed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CaptureOrder {
    /// Equal to or after the newest accepted stamp: a valid next member.
    InOrder,
    /// Before the newest accepted stamp by no more than the window: the frame
    /// arrived out of capture order. Accepting it would break the matcher's
    /// in-order rule, so it is discarded and counted.
    Backwards,
    /// Before the newest accepted stamp by MORE than the window: the capture
    /// clock restarted. Accepted, and the scheduler's epoch reset re-bases
    /// the other inputs.
    NewEpoch,
}

/// Classify a head's capture stamp against the input's newest accepted one.
///
/// The `NewEpoch` threshold is the same strict `>` the scheduler's wire-stamp
/// epoch reset uses (`ts + window < high_water`), so the two layers agree on
/// which frames start a new epoch.
#[must_use]
pub fn order_capture_stamp(high_water: Option<u64>, ts: u64, window_ns: u64) -> CaptureOrder {
    let Some(high_water) = high_water else {
        return CaptureOrder::InOrder;
    };
    if ts >= high_water {
        CaptureOrder::InOrder
    } else if high_water - ts > window_ns {
        CaptureOrder::NewEpoch
    } else {
        CaptureOrder::Backwards
    }
}

/// Which of an input's two slots a stamp is read from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SyncFrameSlot {
    /// The head frame (what a fill or an advance just produced).
    Head,
    /// The staged next frame (what a peek just popped).
    Next,
}

/// The transport and frame access one capture-time head op needs. The runtime
/// implements it over a node's subscribers; the unit tests implement it over a
/// hand-written queue.
pub trait CaptureIo {
    /// Perform one head op exactly as a publish-time node would.
    fn op(&mut self, op: SyncHeadOp) -> SyncOpAnswer;
    /// Decode the capture stamp of the frame in `slot`. `None` means no frame
    /// is held there (or the node cannot expose it), which the driver reports
    /// as a failed op.
    fn stamp(&mut self, slot: SyncFrameSlot) -> Option<Result<u64, StampDecodeError>>;
}

/// What one capture-time head op did, for the caller to count and log.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CaptureOutcome {
    /// The answer the align driver receives, carrying CAPTURE stamps.
    pub answer: SyncOpAnswer,
    /// Frames discarded because their stamp did not decode.
    pub decode_failures: u32,
    /// Frames discarded because their stamp went backwards.
    pub backwards_drops: u32,
    /// The last decode error, for the log line.
    pub last_decode_error: Option<StampDecodeError>,
}

impl CaptureOutcome {
    fn plain(answer: SyncOpAnswer) -> Self {
        Self {
            answer,
            decode_failures: 0,
            backwards_drops: 0,
            last_decode_error: None,
        }
    }
}

/// Perform one Sync head op for a capture-time input.
///
/// `high_water` is this input's newest accepted head stamp. It is advanced
/// only when a head is accepted, never by a peek.
pub fn capture_head_op(
    op: SyncHeadOp,
    window_ns: u64,
    high_water: &mut Option<u64>,
    io: &mut impl CaptureIo,
) -> CaptureOutcome {
    let discard_site = match op {
        SyncHeadOp::FillBoundary => SyncAlignSite::Boundary,
        SyncHeadOp::FillRefill => SyncAlignSite::Refill,
        SyncHeadOp::Advance { site } => site,
        SyncHeadOp::PeekNext { .. } => {
            let answer = match io.op(op) {
                SyncOpAnswer::Stamp(_) => match io.stamp(SyncFrameSlot::Next) {
                    Some(Ok(ts)) => {
                        match order_capture_stamp(*high_water, ts, window_ns) {
                            CaptureOrder::InOrder => SyncOpAnswer::Stamp(ts),
                            // Not a frame to descend to. Judged and counted
                            // when it becomes the head.
                            CaptureOrder::Backwards | CaptureOrder::NewEpoch => {
                                SyncOpAnswer::Nothing
                            }
                        }
                    }
                    Some(Err(_)) => SyncOpAnswer::Nothing,
                    None => SyncOpAnswer::Failed,
                },
                other => other,
            };
            return CaptureOutcome::plain(answer);
        }
        SyncHeadOp::ProbeNext | SyncHeadOp::Void => return CaptureOutcome::plain(io.op(op)),
    };

    let mut outcome = CaptureOutcome::plain(io.op(op));
    let mut discards = 0u32;
    loop {
        if !matches!(outcome.answer, SyncOpAnswer::Head(_)) {
            return outcome;
        }
        match io.stamp(SyncFrameSlot::Head) {
            None => {
                outcome.answer = SyncOpAnswer::Failed;
                return outcome;
            }
            Some(Ok(ts)) => match order_capture_stamp(*high_water, ts, window_ns) {
                CaptureOrder::InOrder | CaptureOrder::NewEpoch => {
                    *high_water = Some(ts);
                    outcome.answer = SyncOpAnswer::Head(ts);
                    return outcome;
                }
                CaptureOrder::Backwards => outcome.backwards_drops += 1,
            },
            Some(Err(e)) => {
                outcome.decode_failures += 1;
                outcome.last_decode_error = Some(e);
            }
        }
        discards += 1;
        if discards >= MAX_DISCARDS_PER_OP {
            // The discarded frame still sits in the head slot, and the next
            // boundary's fill re-offers an unserved head. Void it, so it is
            // never judged or counted a second time.
            io.op(SyncHeadOp::Void);
            outcome.answer = SyncOpAnswer::Failed;
            return outcome;
        }
        outcome.answer = io.op(SyncHeadOp::Advance { site: discard_site });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scheduler::sync_match::{next_sync_step, NextInfo, SyncHead, SyncStep};
    use std::collections::VecDeque;
    use std::time::Duration;

    const MS: u64 = 1_000_000;

    /// A wire frame whose fixed section holds `header.stamp` at offset 8
    /// (a fixed `header`, after one `u64` field).
    fn fixed_frame(sec: i32, nanosec: u32) -> Vec<u8> {
        let mut f = vec![0u8; WireHeader::SIZE + 16];
        let at = WireHeader::SIZE + 8;
        f[at..at + 4].copy_from_slice(&sec.to_le_bytes());
        f[at + 4..at + 8].copy_from_slice(&nanosec.to_le_bytes());
        f
    }

    const FIXED: HeaderStampLocation = HeaderStampLocation::Fixed { offset: 8 };

    /// A `geometry_msgs/PoseStamped`-shaped frame: a 56-byte fixed `pose`,
    /// one offset-table entry for `header`, then the header sub-frame
    /// (`stamp` 8 bytes, `frame_id` entry 8 bytes, the UTF-8).
    fn table_frame(sec: i32, nanosec: u32) -> Vec<u8> {
        let body_start = WireHeader::SIZE;
        let mut f = vec![0u8; body_start + 56 + 8];
        let sub_offset = (56 + 8) as u32;
        let mut sub = vec![0u8; 16];
        sub[0..4].copy_from_slice(&sec.to_le_bytes());
        sub[4..8].copy_from_slice(&nanosec.to_le_bytes());
        sub[8..12].copy_from_slice(&16u32.to_le_bytes());
        sub[12..16].copy_from_slice(&3u32.to_le_bytes());
        sub.extend_from_slice(b"cam");
        let entry = body_start + 56;
        f[entry..entry + 4].copy_from_slice(&sub_offset.to_le_bytes());
        f[entry + 4..entry + 8].copy_from_slice(&(sub.len() as u32).to_le_bytes());
        f.extend_from_slice(&sub);
        f
    }

    const TABLE: HeaderStampLocation = HeaderStampLocation::Table {
        fixed_size: 56,
        index: 0,
        stamp_offset: 0,
    };

    // ---- decode ------------------------------------------------------------

    #[test]
    fn a_fixed_header_stamp_decodes_to_nanoseconds() {
        assert_eq!(
            capture_stamp_ns(&fixed_frame(2, 500_000_000), FIXED),
            Ok(2_500_000_000)
        );
    }

    #[test]
    fn a_table_header_stamp_decodes_through_the_offset_table() {
        assert_eq!(
            capture_stamp_ns(&table_frame(7, 42), TABLE),
            Ok(7_000_000_042)
        );
    }

    #[test]
    fn a_header_sub_frame_exactly_as_long_as_the_stamp_decodes_and_one_byte_less_is_refused() {
        // Rewrite the table entry's sub-frame length: 8 bytes is exactly
        // `sec` + `nanosec`, so the stamp fits; 7 cuts `nanosec` short.
        let with_len = |len: u32| {
            let mut f = table_frame(7, 42);
            let at = WireHeader::SIZE + 56 + 4;
            f[at..at + 4].copy_from_slice(&len.to_le_bytes());
            f
        };
        assert_eq!(capture_stamp_ns(&with_len(8), TABLE), Ok(7_000_000_042));
        let sub_start = WireHeader::SIZE + 56 + 8;
        assert_eq!(
            capture_stamp_ns(&with_len(7), TABLE),
            Err(StampDecodeError::Truncated {
                needed: sub_start + 8,
                len: sub_start + 7,
            })
        );
    }

    #[test]
    fn the_largest_valid_nanosec_decodes_and_one_more_is_refused() {
        assert_eq!(
            capture_stamp_ns(&fixed_frame(0, 999_999_999), FIXED),
            Ok(999_999_999)
        );
        assert_eq!(
            capture_stamp_ns(&fixed_frame(0, 1_000_000_000), FIXED),
            Err(StampDecodeError::NanosecOutOfRange {
                nanosec: 1_000_000_000
            })
        );
    }

    #[test]
    fn a_negative_second_is_refused() {
        assert_eq!(
            capture_stamp_ns(&fixed_frame(-1, 0), FIXED),
            Err(StampDecodeError::NegativeSeconds { sec: -1 })
        );
    }

    #[test]
    fn a_frame_cut_inside_the_stamp_is_truncated() {
        let mut f = fixed_frame(1, 1);
        f.truncate(WireHeader::SIZE + 14);
        assert_eq!(
            capture_stamp_ns(&f, FIXED),
            Err(StampDecodeError::Truncated {
                needed: WireHeader::SIZE + 16,
                len: WireHeader::SIZE + 14
            })
        );
    }

    #[test]
    fn a_header_entry_pointing_past_the_frame_is_truncated() {
        let mut f = table_frame(1, 1);
        let entry = WireHeader::SIZE + 56;
        f[entry..entry + 4].copy_from_slice(&10_000u32.to_le_bytes());
        assert_eq!(
            capture_stamp_ns(&f, TABLE),
            Err(StampDecodeError::Truncated {
                needed: WireHeader::SIZE + 10_004,
                len: f.len()
            })
        );
    }

    #[test]
    fn a_header_sub_frame_shorter_than_a_stamp_is_truncated() {
        let mut f = table_frame(1, 1);
        let entry = WireHeader::SIZE + 56;
        f[entry + 4..entry + 8].copy_from_slice(&4u32.to_le_bytes());
        assert_eq!(
            capture_stamp_ns(&f, TABLE),
            Err(StampDecodeError::Truncated {
                needed: WireHeader::SIZE + 64 + 8,
                len: WireHeader::SIZE + 64 + 4
            })
        );
    }

    #[test]
    fn a_frame_with_no_offset_table_is_truncated_not_a_panic() {
        assert_eq!(
            capture_stamp_ns(&[0u8; 8], TABLE),
            Err(StampDecodeError::Truncated {
                needed: WireHeader::SIZE + 56 + 4,
                len: 8
            })
        );
    }

    // ---- order -------------------------------------------------------------

    #[test]
    fn the_first_stamp_and_equal_stamps_are_in_order() {
        assert_eq!(order_capture_stamp(None, 5, 50 * MS), CaptureOrder::InOrder);
        assert_eq!(
            order_capture_stamp(Some(5), 5, 50 * MS),
            CaptureOrder::InOrder
        );
    }

    #[test]
    fn a_backwards_stamp_within_the_window_is_dropped() {
        assert_eq!(
            order_capture_stamp(Some(100 * MS), 99 * MS, 50 * MS),
            CaptureOrder::Backwards
        );
        // Exactly the window back: still not an epoch (strict `>`).
        assert_eq!(
            order_capture_stamp(Some(100 * MS), 50 * MS, 50 * MS),
            CaptureOrder::Backwards
        );
    }

    #[test]
    fn a_drop_of_more_than_the_window_is_a_new_epoch() {
        assert_eq!(
            order_capture_stamp(Some(100 * MS), 50 * MS - 1, 50 * MS),
            CaptureOrder::NewEpoch
        );
    }

    #[test]
    fn a_restart_far_below_the_high_water_is_a_new_epoch() {
        // A capture clock that restarts at 0 after an hour: accepted, never
        // dropped until the new clock passes the old high-water.
        assert_eq!(
            order_capture_stamp(Some(3_600_000 * MS), 0, 50 * MS),
            CaptureOrder::NewEpoch
        );
    }

    // ---- the driver, over a hand-written queue -----------------------------

    /// One input's transport: a head slot, a staged slot and a FIFO queue of
    /// frames, each frame a decoded capture stamp (or a decode error) plus a
    /// distinct wire stamp, so a test can see which clock an answer carries.
    ///
    /// A fill RE-OFFERS a head that is still held, as the real trigger drain
    /// does for an unserved head; a fire serves (clears) it, see
    /// [`Queue::serve`].
    struct Queue {
        head: Option<(u64, Result<u64, StampDecodeError>)>,
        next: Option<(u64, Result<u64, StampDecodeError>)>,
        queue: VecDeque<(u64, Result<u64, StampDecodeError>)>,
        ops: Vec<SyncHeadOp>,
    }

    impl Queue {
        fn new(frames: &[(u64, Result<u64, StampDecodeError>)]) -> Self {
            Self {
                head: None,
                next: None,
                queue: frames.iter().copied().collect(),
                ops: Vec::new(),
            }
        }

        /// A fire's read: the head is served and leaves the slot.
        fn serve(&mut self) {
            self.head = None;
        }
    }

    impl CaptureIo for Queue {
        fn op(&mut self, op: SyncHeadOp) -> SyncOpAnswer {
            self.ops.push(op);
            match op {
                SyncHeadOp::FillBoundary | SyncHeadOp::FillRefill => {
                    if self.head.is_none() {
                        self.head = self.next.take().or_else(|| self.queue.pop_front());
                    }
                    self.head
                        .map_or(SyncOpAnswer::Nothing, |(w, _)| SyncOpAnswer::Head(w))
                }
                SyncHeadOp::ProbeNext => {
                    if self.next.is_some() || !self.queue.is_empty() {
                        SyncOpAnswer::Present
                    } else {
                        SyncOpAnswer::Nothing
                    }
                }
                SyncHeadOp::PeekNext { .. } => {
                    if self.next.is_none() {
                        self.next = self.queue.pop_front();
                    }
                    self.next
                        .map_or(SyncOpAnswer::Nothing, |(w, _)| SyncOpAnswer::Stamp(w))
                }
                SyncHeadOp::Advance { .. } => {
                    self.head = self.next.take().or_else(|| self.queue.pop_front());
                    self.head
                        .map_or(SyncOpAnswer::Nothing, |(w, _)| SyncOpAnswer::Head(w))
                }
                SyncHeadOp::Void => {
                    self.head = None;
                    SyncOpAnswer::Nothing
                }
            }
        }

        fn stamp(&mut self, slot: SyncFrameSlot) -> Option<Result<u64, StampDecodeError>> {
            match slot {
                SyncFrameSlot::Head => self.head.map(|(_, s)| s),
                SyncFrameSlot::Next => self.next.map(|(_, s)| s),
            }
        }
    }

    const BAD: StampDecodeError = StampDecodeError::NegativeSeconds { sec: -1 };
    const WINDOW: u64 = 50 * MS;

    #[test]
    fn a_fill_answers_the_capture_stamp_not_the_wire_stamp() {
        let mut q = Queue::new(&[(999, Ok(10 * MS))]);
        let mut hw = None;
        let out = capture_head_op(SyncHeadOp::FillBoundary, WINDOW, &mut hw, &mut q);
        assert_eq!(out.answer, SyncOpAnswer::Head(10 * MS));
        assert_eq!(hw, Some(10 * MS));
        assert_eq!((out.decode_failures, out.backwards_drops), (0, 0));
    }

    #[test]
    fn an_undecodable_head_is_discarded_counted_and_the_next_frame_serves() {
        let mut q = Queue::new(&[(1, Err(BAD)), (2, Ok(20 * MS))]);
        let mut hw = None;
        let out = capture_head_op(SyncHeadOp::FillBoundary, WINDOW, &mut hw, &mut q);
        assert_eq!(out.answer, SyncOpAnswer::Head(20 * MS));
        assert_eq!(out.decode_failures, 1);
        assert_eq!(out.last_decode_error, Some(BAD));
        assert_eq!(
            q.ops,
            vec![
                SyncHeadOp::FillBoundary,
                SyncHeadOp::Advance {
                    site: SyncAlignSite::Boundary
                }
            ],
            "the discard pops past the head at the fill's own site"
        );
    }

    #[test]
    fn a_backwards_head_is_discarded_and_counted() {
        let mut q = Queue::new(&[(1, Ok(30 * MS)), (2, Ok(40 * MS))]);
        let mut hw = Some(35 * MS);
        let out = capture_head_op(SyncHeadOp::FillRefill, WINDOW, &mut hw, &mut q);
        assert_eq!(out.answer, SyncOpAnswer::Head(40 * MS));
        assert_eq!(out.backwards_drops, 1);
        assert_eq!(hw, Some(40 * MS));
        assert_eq!(
            q.ops[1],
            SyncHeadOp::Advance {
                site: SyncAlignSite::Refill
            }
        );
    }

    #[test]
    fn an_input_with_only_bad_frames_ends_with_no_head() {
        let mut q = Queue::new(&[(1, Err(BAD)), (2, Err(BAD))]);
        let mut hw = None;
        let out = capture_head_op(SyncHeadOp::FillBoundary, WINDOW, &mut hw, &mut q);
        assert_eq!(out.answer, SyncOpAnswer::Nothing);
        assert_eq!(out.decode_failures, 2);
        assert_eq!(hw, None, "no head was accepted");
    }

    #[test]
    fn a_clock_restart_is_accepted_as_a_new_epoch() {
        let mut q = Queue::new(&[(9, Ok(0))]);
        let mut hw = Some(10_000 * MS);
        let out = capture_head_op(SyncHeadOp::FillBoundary, WINDOW, &mut hw, &mut q);
        assert_eq!(out.answer, SyncOpAnswer::Head(0));
        assert_eq!(hw, Some(0), "the high-water re-bases onto the new epoch");
    }

    #[test]
    fn a_peek_answers_the_capture_stamp_and_never_moves_the_high_water() {
        let mut q = Queue::new(&[(1, Ok(10 * MS)), (2, Ok(12 * MS))]);
        let mut hw = None;
        capture_head_op(SyncHeadOp::FillBoundary, WINDOW, &mut hw, &mut q);
        let site = SyncAlignSite::Boundary;
        let out = capture_head_op(SyncHeadOp::PeekNext { site }, WINDOW, &mut hw, &mut q);
        assert_eq!(out.answer, SyncOpAnswer::Stamp(12 * MS));
        assert_eq!(hw, Some(10 * MS));
    }

    #[test]
    fn a_bad_or_backwards_peek_refuses_descent_without_counting() {
        let site = SyncAlignSite::Boundary;
        for staged in [Err(BAD), Ok(5 * MS)] {
            let mut q = Queue::new(&[(1, Ok(10 * MS)), (2, staged)]);
            let mut hw = None;
            capture_head_op(SyncHeadOp::FillBoundary, WINDOW, &mut hw, &mut q);
            let out = capture_head_op(SyncHeadOp::PeekNext { site }, WINDOW, &mut hw, &mut q);
            assert_eq!(out.answer, SyncOpAnswer::Nothing, "staged {staged:?}");
            assert_eq!((out.decode_failures, out.backwards_drops), (0, 0));
        }
    }

    #[test]
    fn a_missing_frame_behind_a_head_answer_is_a_failed_op() {
        struct NoFrames;
        impl CaptureIo for NoFrames {
            fn op(&mut self, _: SyncHeadOp) -> SyncOpAnswer {
                SyncOpAnswer::Head(1)
            }
            fn stamp(&mut self, _: SyncFrameSlot) -> Option<Result<u64, StampDecodeError>> {
                None
            }
        }
        let mut hw = None;
        let out = capture_head_op(SyncHeadOp::FillBoundary, WINDOW, &mut hw, &mut NoFrames);
        assert_eq!(out.answer, SyncOpAnswer::Failed);
    }

    #[test]
    fn a_queue_that_never_runs_dry_of_bad_frames_is_clamped() {
        struct Endless(u32);
        impl CaptureIo for Endless {
            fn op(&mut self, _: SyncHeadOp) -> SyncOpAnswer {
                self.0 += 1;
                SyncOpAnswer::Head(1)
            }
            fn stamp(&mut self, _: SyncFrameSlot) -> Option<Result<u64, StampDecodeError>> {
                Some(Err(BAD))
            }
        }
        let mut io = Endless(0);
        let mut hw = None;
        let out = capture_head_op(SyncHeadOp::FillBoundary, WINDOW, &mut hw, &mut io);
        assert_eq!(out.answer, SyncOpAnswer::Failed);
        assert_eq!(out.decode_failures, MAX_DISCARDS_PER_OP);
        // One fill, MAX - 1 advances, then the void of the last discard.
        assert_eq!(io.0, MAX_DISCARDS_PER_OP + 1);
    }

    #[test]
    fn a_give_up_voids_the_last_discard_so_no_frame_is_counted_twice() {
        // 300 backwards frames: more than one op may discard. Each is a
        // distinct frame and must be counted exactly once across the ops.
        let frames: Vec<_> = (0..300u64).map(|w| (w, Ok(90 * MS))).collect();
        let mut q = Queue::new(&frames);
        let mut hw = Some(100 * MS);
        let first = capture_head_op(SyncHeadOp::FillBoundary, WINDOW, &mut hw, &mut q);
        assert_eq!(first.answer, SyncOpAnswer::Failed);
        assert_eq!(first.backwards_drops, MAX_DISCARDS_PER_OP);
        assert_eq!(q.ops.last(), Some(&SyncHeadOp::Void));
        let second = capture_head_op(SyncHeadOp::FillBoundary, WINDOW, &mut hw, &mut q);
        assert_eq!(second.answer, SyncOpAnswer::Nothing);
        assert_eq!(
            first.backwards_drops + second.backwards_drops,
            300,
            "one count per distinct frame"
        );
        assert_eq!(hw, Some(100 * MS), "no dropped frame moves the high-water");
    }

    #[test]
    fn probe_and_void_pass_through_untouched() {
        let mut q = Queue::new(&[(1, Ok(1))]);
        let mut hw = Some(77);
        let out = capture_head_op(SyncHeadOp::ProbeNext, WINDOW, &mut hw, &mut q);
        assert_eq!(out.answer, SyncOpAnswer::Present);
        let out = capture_head_op(SyncHeadOp::Void, WINDOW, &mut hw, &mut q);
        assert_eq!(out.answer, SyncOpAnswer::Nothing);
        assert_eq!(hw, Some(77));
    }

    // ---- capture-time sets, end to end over the matcher --------------------

    /// Drive the align loop the way the scheduler does, with every op going
    /// through [`capture_head_op`], and return each fired set's CAPTURE stamps
    /// in fire order. Two inputs, a hand-written queue each.
    fn fused_sets(
        a: &[(u64, Result<u64, StampDecodeError>)],
        b: &[(u64, Result<u64, StampDecodeError>)],
        window: Duration,
    ) -> (Vec<(u64, u64)>, [u32; 2], [u32; 2]) {
        let window_ns = window.as_nanos() as u64;
        let mut queues = [Queue::new(a), Queue::new(b)];
        let mut hw = [None, None];
        let mut heads: [Option<SyncHead>; 2] = [None, None];
        let mut decode = [0u32; 2];
        let mut backwards = [0u32; 2];
        let mut sets = Vec::new();
        let mut run =
            |i: usize, op: SyncHeadOp, queues: &mut [Queue; 2], hw: &mut [Option<u64>; 2]| {
                let out = capture_head_op(op, window_ns, &mut hw[i], &mut queues[i]);
                decode[i] += out.decode_failures;
                backwards[i] += out.backwards_drops;
                out.answer
            };
        for _boundary in 0..32 {
            for (i, head) in heads.iter_mut().enumerate() {
                if head.is_none() {
                    *head = match run(i, SyncHeadOp::FillBoundary, &mut queues, &mut hw) {
                        SyncOpAnswer::Head(ts) => Some(SyncHead::filled(ts)),
                        _ => None,
                    };
                }
            }
            let mut next = [NextInfo::Unknown, NextInfo::Unknown];
            let site = SyncAlignSite::Boundary;
            loop {
                match next_sync_step(&heads, &next, Some(window)) {
                    SyncStep::Wait => break,
                    SyncStep::Fire => {
                        let (Some(x), Some(y)) = (heads[0], heads[1]) else {
                            unreachable!("fire on a complete tuple")
                        };
                        sets.push((x.ts, y.ts));
                        heads = [None, None];
                        queues.iter_mut().for_each(Queue::serve);
                        break;
                    }
                    SyncStep::NeedNext(i, _) => {
                        next[i] = match run(i, SyncHeadOp::ProbeNext, &mut queues, &mut hw) {
                            SyncOpAnswer::Present => NextInfo::Present,
                            _ => NextInfo::None,
                        };
                    }
                    SyncStep::NeedStamp(i) => {
                        next[i] = match run(i, SyncHeadOp::PeekNext { site }, &mut queues, &mut hw)
                        {
                            SyncOpAnswer::Stamp(ts) => NextInfo::Stamp(ts),
                            _ => NextInfo::None,
                        };
                    }
                    SyncStep::Advance(i) => {
                        heads[i] = match run(i, SyncHeadOp::Advance { site }, &mut queues, &mut hw)
                        {
                            SyncOpAnswer::Head(ts) => Some(SyncHead::filled(ts)),
                            _ => None,
                        };
                        next[i] = NextInfo::Unknown;
                    }
                    SyncStep::DiscardTie(dead) => {
                        for i in dead {
                            heads[i] =
                                match run(i, SyncHeadOp::Advance { site }, &mut queues, &mut hw) {
                                    SyncOpAnswer::Head(ts) => Some(SyncHead::filled(ts)),
                                    _ => None,
                                };
                            next[i] = NextInfo::Unknown;
                        }
                    }
                }
            }
        }
        (sets, decode, backwards)
    }

    fn ok(wire: u64, capture_ms: u64) -> (u64, Result<u64, StampDecodeError>) {
        (wire, Ok(capture_ms * MS))
    }

    #[test]
    fn exact_capture_matches_pair_even_when_publish_times_are_far_apart() {
        // Camera publishes 40 ms after capture, lidar 5 ms after. On wire
        // stamps the pairs would be 35 ms apart; on capture stamps they match
        // exactly.
        let cam = [ok(140, 100), ok(240, 200)];
        let lidar = [ok(105, 100), ok(205, 200)];
        let (sets, _, _) = fused_sets(&cam, &lidar, Duration::from_millis(10));
        assert_eq!(sets, vec![(100 * MS, 100 * MS), (200 * MS, 200 * MS)]);
    }

    #[test]
    fn a_pair_exactly_at_the_window_edge_matches_and_one_past_does_not() {
        let (sets, _, _) = fused_sets(&[ok(1, 100)], &[ok(2, 110)], Duration::from_millis(10));
        assert_eq!(sets, vec![(100 * MS, 110 * MS)]);
        let (sets, _, _) = fused_sets(
            &[(1, Ok(100 * MS))],
            &[(2, Ok(110 * MS + 1))],
            Duration::from_millis(10),
        );
        assert_eq!(sets, vec![], "one nanosecond past the window is no set");
    }

    #[test]
    fn out_of_order_arrival_across_inputs_still_pairs_by_capture_time() {
        // Lidar's frame for t=100 arrives AFTER camera's t=200 frame was
        // published; capture stamps still pair 100 with 100 and 200 with 200.
        let cam = [ok(150, 100), ok(160, 200)];
        let lidar = [ok(300, 100), ok(310, 200)];
        let (sets, _, _) = fused_sets(&cam, &lidar, Duration::from_millis(20));
        assert_eq!(sets, vec![(100 * MS, 100 * MS), (200 * MS, 200 * MS)]);
    }

    #[test]
    fn equal_stamps_on_one_input_serve_in_arrival_order() {
        // Two camera frames share a capture stamp. The matcher's tie at the
        // minimum never descends, so the first-arrived one is served first and
        // the second waits for the next lidar frame: deterministic, FIFO.
        let cam = [ok(1, 100), ok(2, 100)];
        let lidar = [ok(3, 100), ok(4, 105)];
        let (sets, _, backwards) = fused_sets(&cam, &lidar, Duration::from_millis(10));
        assert_eq!(sets, vec![(100 * MS, 100 * MS), (100 * MS, 105 * MS)]);
        assert_eq!(backwards, [0, 0], "an equal stamp is in order");
    }

    #[test]
    fn a_stale_frame_that_never_finds_a_partner_is_never_fused() {
        // Camera t=0 has no lidar partner within 10 ms; it dies on the window
        // and camera t=100 pairs with lidar t=100.
        let cam = [ok(1, 0), ok(2, 100)];
        let lidar = [ok(3, 100)];
        let (sets, _, _) = fused_sets(&cam, &lidar, Duration::from_millis(10));
        assert_eq!(sets, vec![(100 * MS, 100 * MS)]);
    }

    #[test]
    fn a_stamp_that_goes_backwards_is_dropped_and_counted() {
        // Camera's second frame claims t=95 after t=100 was fused: dropped and
        // counted, and its t=200 frame pairs normally.
        let cam = [ok(1, 100), ok(2, 95), ok(3, 200)];
        let lidar = [ok(4, 100), ok(5, 200)];
        let (sets, decode, backwards) = fused_sets(&cam, &lidar, Duration::from_millis(10));
        assert_eq!(sets, vec![(100 * MS, 100 * MS), (200 * MS, 200 * MS)]);
        assert_eq!(backwards, [1, 0]);
        assert_eq!(decode, [0, 0]);
    }

    #[test]
    fn an_undecodable_frame_is_skipped_and_counted_in_the_walk() {
        let cam = [(1, Err(BAD)), ok(2, 100)];
        let lidar = [ok(3, 100)];
        let (sets, decode, _) = fused_sets(&cam, &lidar, Duration::from_millis(10));
        assert_eq!(sets, vec![(100 * MS, 100 * MS)]);
        assert_eq!(decode, [1, 0]);
    }

    #[test]
    fn the_same_frames_fuse_into_the_same_sets_on_every_run() {
        let cam = [ok(1, 100), ok(2, 95), ok(3, 130), ok(4, 200)];
        let lidar = [ok(5, 98), ok(6, 128), ok(7, 202)];
        let window = Duration::from_millis(5);
        let first = fused_sets(&cam, &lidar, window);
        let second = fused_sets(&cam, &lidar, window);
        assert_eq!(first, second);
        assert_eq!(
            first.0,
            vec![
                (100 * MS, 98 * MS),
                (130 * MS, 128 * MS),
                (200 * MS, 202 * MS)
            ]
        );
    }
}

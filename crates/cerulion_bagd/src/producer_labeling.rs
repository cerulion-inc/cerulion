// SPDX-License-Identifier: AGPL-3.0-only
//! The per-tap PRODUCER-LABELING state machine — the pure decision
//! half of record-time producer attribution.
//!
//! Its own module on the `graph/drain_latch.rs` / `transport/output_discard_latch.rs`
//! precedent: one file per pure state machine, so the rule and its oracle
//! vectors sit together and neither has to be found inside a 23 000-line
//! recorder.
//!
//! # What it decides, and why the recorder has to
//!
//! A `multi_publisher_topics` topic is written by several publishers onto ONE
//! iceoryx2 service, and the wire header's `sequence` is a PER-PUBLISHER commit
//! counter — so the bag's channel holds an interleaving that no reader can
//! attribute after the fact. iceoryx2 DOES report which port committed a sample,
//! but only while the sample is borrowed, so the attribution has to be written
//! down at record time or not at all.
//!
//! # When labeling arms
//!
//! Two ways, and the second is why every tap runs this:
//!
//! * **DECLARED** — the topic is in the graph's `multi_publisher_topics`, so the
//!   tap is armed at open and EVERY frame is labeled. There is no unlabeled
//!   prefix, so no catch-up record is ever owed.
//! * **OBSERVED** — an undeclared topic turns out to have a second writer. The
//!   first frame whose `origin` differs from the topic's first-ever origin ARMS
//!   labeling **starting with that frame**, and the run of frames before it
//!   (all of which carried the first origin — this recorder wrote every one of
//!   them and each matched) is attributed in ONE `CatchUpPrefix` record rather
//!   than retro-labeled frame by frame.
//!
//! Arming is ONE-WAY: once on, a third origin (or a return to the first) neither
//! re-arms nor emits a second catch-up.
//!
//! # Cost
//!
//! One `u128` compare per drained frame, on the recorder's own drain thread —
//! [`OwnedInboundSample::origin`](cerulion_core::transport::subscriber::OwnedInboundSample::origin)
//! is sample metadata, so it parses no wire bytes and allocates nothing. The
//! PUBLISHER pays nothing at all: no hot-path code changed. A single-writer
//! topic that never sees a second origin writes ZERO MESSAGES on the reserved
//! `__cerulion/frame_producers` channel — the channel's REGISTRATION is in every
//! bag, so an ordinary recording is not byte-identical to one written before
//! this channel existed (the registration also shifts the reserved channel
//! ids); what is unchanged between runs is that registration itself.
//!
//! Pure — no transport, no I/O, no clock. See the `producer_labeling_tests`
//! module below for the oracle vectors.

/// Whether a staged frame owes a producer label, and whose.
///
/// A dedicated enum replacing what used to be an `Option<u128>`, because in this
/// repo a `None` is the UNKNOWN reading (a value nobody could establish) and
/// that is not what the steady state means here: a single-writer topic is not a
/// topic whose producer we failed to identify, it is one that owes NO LABEL
/// because nothing on the channel is ambiguous. Spelling the two apart in the
/// type is what stops a future reader from rendering "producer: unknown" on
/// every frame of every ordinary recording.
///
/// No `Default`, for the same reason [`ProducerLabeling`] has none: a
/// `#[default] Unlabeled` makes the SILENTLY WRONG value the one a
/// `..Default::default()` or a `mem::take` reaches for, so a frame that should
/// have carried its publisher's identity would come back unlabeled with nothing
/// failing. Every construction site knows which arm it means.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum FrameOrigin {
    /// No label is owed for this frame. The steady state: the frame's topic is
    /// not being labeled, so nothing about its producer is ambiguous and no
    /// producer record is minted for it.
    Unlabeled,
    /// This frame carries the identity of the publisher that COMMITTED it
    /// (iceoryx2's `UniquePublisherId`, raw), because its topic is LABELED —
    /// declared `multi_publisher`, or observed to have a second writer.
    Labeled(u128),
}

impl FrameOrigin {
    /// The publisher id, when one is owed.
    pub(crate) fn id(self) -> Option<u128> {
        match self {
            Self::Unlabeled => None,
            Self::Labeled(id) => Some(id),
        }
    }

    /// Whether this frame owes a producer record.
    pub(crate) fn is_labeled(self) -> bool {
        matches!(self, Self::Labeled(_))
    }
}

/// One drained frame's verdict, returned by [`ProducerLabeling::observe`].
///
/// The two answers are handed back TOGETHER because they are decided together
/// and must be read in one order: the arming frame IS labeled, so a caller that
/// asked "did this arm?" and "is this labeled?" through two calls was driving
/// the very ordering it was supposed to be told about.
///
/// An ENUM rather than a struct of two independent fields, because the pairing
/// is an INVARIANT and not a coincidence: an armed prefix can only ever ride a
/// LABELED verdict (arming labels the very frame that armed it), so a struct
/// admitted a `{ label: Unlabeled, armed_prefix: Some(..) }` that says the topic
/// just became multi-writer and this frame owes no attribution — a contradiction
/// held together by nothing but statement order inside `observe`, and by one
/// behavioural pin. Here it does not exist.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum FrameVerdict {
    /// No label is owed. The steady state of a single-writer topic, and — before
    /// the arming frame — of one that later turns out to have two writers.
    Unlabeled,
    /// This frame is LABELED with the identity of the publisher that committed
    /// it.
    Labeled {
        /// The committing publisher (iceoryx2's `UniquePublisherId`, raw).
        id: u128,
        /// `Some(prefix_origin)` on the ONE call that arms labeling by OBSERVING
        /// plurality — the caller's cue to log it, carrying the origin of the
        /// unlabeled run that precedes this frame. Never set for a declared
        /// multi-publisher tap (armed at open, with no prefix to catch up) and
        /// never set twice.
        armed_prefix: Option<u128>,
    },
}

impl FrameVerdict {
    /// What the frame owes, in the form the staging buffer stores.
    pub(crate) fn label(self) -> FrameOrigin {
        match self {
            Self::Unlabeled => FrameOrigin::Unlabeled,
            Self::Labeled { id, .. } => FrameOrigin::Labeled(id),
        }
    }

    /// The origin of the unlabeled run this frame's arming closed, if this call
    /// is the one that armed.
    pub(crate) fn armed_prefix(self) -> Option<u128> {
        match self {
            Self::Unlabeled => None,
            Self::Labeled { armed_prefix, .. } => armed_prefix,
        }
    }
}

/// The per-tap producer-labeling state (see the module docs).
///
/// No `Default`: `default()` would mean `new(false)` — an UNDECLARED tap — and
/// a tap built by mistake on a declared `multi_publisher` topic would then label
/// nothing and owe no catch-up, which is a silently wrong recording rather than
/// a loud one. Every tap is opened knowing its declaration, so the constructor
/// takes it.
#[derive(Debug, Clone)]
pub(crate) struct ProducerLabeling {
    /// The first publisher identity this tap ever saw, or `None` before the
    /// first frame. It is the CATCH-UP's attribution: every frame written before
    /// the arming frame carried it (the recorder saw them all, and the arming
    /// frame is by definition the FIRST that did not match).
    first_origin: Option<u128>,
    /// Whether frames are being labeled. Set at open for a declared
    /// multi-publisher topic, or on the first observed second origin.
    labeling: bool,
    /// A `CatchUpPrefix` OWED to the writer, carrying the prefix's origin. Taken
    /// when a batch is handed off (the writer computes the prefix LENGTH, which
    /// only it knows — see `WriterCore::frame_ordinals`) and restored if that
    /// hand-off is deferred.
    catch_up: Option<u128>,
}

impl ProducerLabeling {
    /// Open a tap's labeling state. `declared_multi` arms it immediately.
    pub(crate) fn new(declared_multi: bool) -> Self {
        Self {
            first_origin: None,
            labeling: declared_multi,
            catch_up: None,
        }
    }

    /// Observe one drained frame's producer and report what it owes.
    pub(crate) fn observe(&mut self, origin: u128) -> FrameVerdict {
        let armed_prefix = match self.first_origin {
            None => {
                self.first_origin = Some(origin);
                None
            }
            Some(first) if !self.labeling && origin != first => {
                self.labeling = true;
                self.catch_up = Some(first);
                Some(first)
            }
            Some(_) => None,
        };
        if self.labeling {
            FrameVerdict::Labeled {
                id: origin,
                armed_prefix,
            }
        } else {
            // `armed_prefix` is `None` on this arm BY CONSTRUCTION: the only
            // branch above that sets it also sets `self.labeling`, so the
            // contradictory pairing the enum forbids cannot be reached here
            // either.
            FrameVerdict::Unlabeled
        }
    }

    /// Take the owed catch-up, if any, for a batch about to be handed off.
    ///
    /// `#[must_use]` because the return IS the debt: dropping it silently loses
    /// the attribution of an entire single-writer run, and the sibling
    /// [`restore_catch_up`](Self::restore_catch_up) exists precisely because
    /// that debt has to survive a deferred hand-off.
    ///
    /// It does NOT guard the confusion with [`first_origin`](Self::first_origin),
    /// which returns the same `Option<u128>`: a caller who means that one and
    /// types this one BINDS the result, so the lint never fires. What separates
    /// them is the RECEIVER — this takes `&mut self`, `first_origin` takes
    /// `&self` — so the capture snapshot's `self.taps.iter()` cannot reach this
    /// one at all. The attribute guards the other mistake: taking the debt and
    /// dropping it.
    #[must_use]
    pub(crate) fn take_catch_up(&mut self) -> Option<u128> {
        self.catch_up.take()
    }

    /// The FIRST publisher identity this tap ever saw, or `None` before its
    /// first frame.
    ///
    /// # Why this is read, and why it is not [`take_catch_up`](Self::take_catch_up)
    ///
    /// A flashback capture must be attributable from its own bytes, and a
    /// capture whose window reaches back past the arming frame
    /// holds a leading run of frames staged as [`FrameOrigin::Unlabeled`] — the
    /// id was never stored for them, deliberately (an `Unlabeled` frame owes no
    /// label, and storing 16 bytes per frame on every single-writer topic is the
    /// cost this design exists to avoid). Every one of those frames carried THIS
    /// value, because arming happens on the FIRST frame that did not match it.
    ///
    /// It is a NON-CONSUMING read and not the catch-up, for two independent
    /// reasons. The catch-up is a DEBT owed to the continuous bag's writer and
    /// taking it would pay the wrong creditor — a capture that stole it would
    /// leave the recording's own prefix unattributed. And a capture may be taken
    /// long after that debt was settled, or on a DECLARED tap that never owed one
    /// at all, while this value is set on the first frame and never changes.
    ///
    /// `None` only before the first frame, so a capture holding frames of a topic
    /// always has an answer for it.
    pub(crate) fn first_origin(&self) -> Option<u128> {
        self.first_origin
    }

    /// Put an owed catch-up BACK after a deferred hand-off.
    ///
    /// `or` rather than an assignment, and it cannot mask a newer one: a
    /// catch-up is minted only at the single arming transition, and arming
    /// cannot happen while one is outstanding (`labeling` is already true). The
    /// combinator states that rather than trusting the ordering.
    pub(crate) fn restore_catch_up(&mut self, owed: Option<u128>) {
        self.catch_up = self.catch_up.or(owed);
    }
}

/// Oracle vectors for [`ProducerLabeling`] — the pure decision half of
/// record-time producer attribution.
///
/// Pure state machine: no transport, no writer, no clock. Every expectation is a
/// hand-written verdict rather than a second run of the same code.
#[cfg(test)]
mod producer_labeling_tests {
    use super::{FrameOrigin, ProducerLabeling};

    const A: u128 = 0x1111_2222_3333_4444_5555_6666_7777_8888;
    const B: u128 = 0x9999_AAAA_BBBB_CCCC_DDDD_EEEE_FFFF_0000;
    const C: u128 = 7;

    /// An undeclared topic's FIRST frame records the origin, arms nothing, and
    /// is not labeled — the single-writer steady state, which is most topics.
    #[test]
    fn the_first_frame_sets_the_origin_and_labels_nothing() {
        let mut p = ProducerLabeling::new(false);
        let v = p.observe(A);
        assert_eq!(
            v.armed_prefix(),
            None,
            "the first frame cannot arm plurality"
        );
        assert_eq!(
            v.label(),
            FrameOrigin::Unlabeled,
            "one writer is not a reason to label"
        );
        assert_eq!(p.take_catch_up(), None);
    }

    /// The same origin, however many times, never arms and never labels. This is
    /// the case that must stay free: a single-writer recording writes ZERO
    /// producer records for its whole run.
    #[test]
    fn a_single_writer_never_arms_however_long_it_runs() {
        let mut p = ProducerLabeling::new(false);
        for _ in 0..1000 {
            let v = p.observe(A);
            assert_eq!(v.armed_prefix(), None);
            assert_eq!(v.label(), FrameOrigin::Unlabeled);
        }
        assert_eq!(p.take_catch_up(), None, "no prefix is owed to anyone");
    }

    /// A SECOND distinct origin arms labeling STARTING WITH THAT FRAME, and owes
    /// exactly one catch-up carrying the FIRST origin — not the second, which is
    /// the whole point: the catch-up attributes the run BEFORE the arming frame.
    #[test]
    fn a_second_origin_arms_once_and_owes_the_first_origins_prefix() {
        let mut p = ProducerLabeling::new(false);
        p.observe(A);
        let v = p.observe(A);
        assert_eq!(v.label(), FrameOrigin::Unlabeled, "still one writer");
        let v = p.observe(B);
        assert_eq!(
            v.armed_prefix(),
            Some(A),
            "arms, and names the PREFIX's origin"
        );
        assert_eq!(
            v.label(),
            FrameOrigin::Labeled(B),
            "the ARMING frame is itself labeled, with ITS OWN producer"
        );
        assert_eq!(p.take_catch_up(), Some(A));
        assert_eq!(p.take_catch_up(), None, "taken once, not once per read");
    }

    /// Arming is ONE-WAY and ONE-TIME: a third origin, and a return to the
    /// first, both keep labeling on and neither mints a second catch-up (which
    /// would re-attribute an already-attributed prefix).
    #[test]
    fn arming_never_repeats_however_many_writers_follow() {
        let mut p = ProducerLabeling::new(false);
        p.observe(A);
        assert_eq!(p.observe(B).armed_prefix(), Some(A));
        assert_eq!(p.take_catch_up(), Some(A));
        for origin in [C, A, B, C, A] {
            let v = p.observe(origin);
            assert_eq!(v.armed_prefix(), None, "re-armed on {origin}");
            assert_eq!(v.label(), FrameOrigin::Labeled(origin));
            assert_eq!(p.take_catch_up(), None);
        }
    }

    /// A DECLARED `multi_publisher` topic labels from its very first frame, so
    /// there is no unlabeled prefix and no catch-up can EVER be owed — including
    /// at the instant a second writer really does appear.
    #[test]
    fn a_declared_multi_publisher_tap_labels_from_the_first_frame_with_no_catch_up() {
        let mut p = ProducerLabeling::new(true);
        let v = p.observe(A);
        assert_eq!(v.armed_prefix(), None);
        assert_eq!(
            v.label(),
            FrameOrigin::Labeled(A),
            "armed at open — the first frame is already labeled"
        );
        let v = p.observe(B);
        assert_eq!(
            v.armed_prefix(),
            None,
            "already armed — nothing to announce"
        );
        assert_eq!(v.label(), FrameOrigin::Labeled(B));
        assert_eq!(p.take_catch_up(), None, "there is no prefix to catch up");
    }

    /// A deferred hand-off returns the owed catch-up, and restoring it is a
    /// no-op when nothing was owed — the two shapes the defer path produces.
    #[test]
    fn a_deferred_catch_up_comes_back_and_an_empty_restore_changes_nothing() {
        let mut p = ProducerLabeling::new(false);
        p.observe(A);
        p.observe(B);
        let owed = p.take_catch_up();
        assert_eq!(owed, Some(A));
        p.restore_catch_up(owed);
        assert_eq!(p.take_catch_up(), Some(A), "restored, not lost");

        let mut q = ProducerLabeling::new(false);
        q.observe(A);
        q.restore_catch_up(None);
        assert_eq!(q.take_catch_up(), None);
    }

    /// An EMPTY restore may never CLEAR a catch-up that is still standing.
    ///
    /// The shape: a batch is handed off WITHOUT taking the catch-up (a tap whose
    /// hand-off carried no frames of its own), that hand-off is deferred, and
    /// the restore arrives carrying `None` for this tap. `self.catch_up.or(owed)`
    /// keeps the standing debt; a bare `self.catch_up = owed` DESTROYS it, and
    /// the prefix is then never attributed in the bag — silently, since nothing
    /// downstream knows a record was owed.
    ///
    /// The sibling arm above restores the SAME value it took, so it cannot see
    /// the difference: there `catch_up` is `None` when the restore lands and
    /// both spellings agree.
    #[test]
    fn an_empty_restore_never_clears_a_standing_catch_up() {
        let mut p = ProducerLabeling::new(false);
        p.observe(A);
        assert_eq!(
            p.observe(B).armed_prefix(),
            Some(A),
            "the catch-up is minted"
        );
        // NOT taken — the debt is still standing when the deferred hand-off
        // hands back its own (empty) slot for this tap.
        p.restore_catch_up(None);
        assert_eq!(
            p.take_catch_up(),
            Some(A),
            "an empty restore must not destroy a standing catch-up"
        );
    }

    /// The verdict sequence over one realistic tap life, against a hand-written
    /// oracle — the property the writer's `frame_ordinals` bookkeeping rests on:
    /// the unlabeled run is a strict PREFIX, so its length is exactly the
    /// catch-up's `ordinal`.
    ///
    /// The stream is deliberately ASYMMETRIC (4 unlabeled, 5 labeled). At the
    /// balanced 4/4 it started out as, the prefix-length assertion below was
    /// decorative: it read `4` whichever side of the split the filter happened
    /// to be counting, so an inverted `is_labeled` passed it and only the
    /// full-vector oracle above had any teeth.
    #[test]
    fn the_labeled_run_is_a_suffix_and_the_unlabeled_one_a_prefix() {
        let mut p = ProducerLabeling::new(false);
        let stream = [A, A, A, A, B, A, B, B, B];
        let verdicts: Vec<FrameOrigin> = stream.iter().map(|&o| p.observe(o).label()).collect();
        assert_eq!(
            verdicts,
            vec![
                FrameOrigin::Unlabeled,
                FrameOrigin::Unlabeled,
                FrameOrigin::Unlabeled,
                FrameOrigin::Unlabeled,
                FrameOrigin::Labeled(B),
                FrameOrigin::Labeled(A),
                FrameOrigin::Labeled(B),
                FrameOrigin::Labeled(B),
                FrameOrigin::Labeled(B),
            ],
            "labeling arms WITH the arming frame and never turns off"
        );
        assert_eq!(
            verdicts.iter().filter(|v| !v.is_labeled()).count(),
            4,
            "the prefix the catch-up attributes is exactly 4 frames long"
        );
        assert_eq!(
            verdicts.iter().filter(|v| v.is_labeled()).count(),
            5,
            "…and the labeled suffix is the other 5 — asserted apart so neither count can be \
             satisfied by the wrong half"
        );
        assert_eq!(p.take_catch_up(), Some(A));
    }
}

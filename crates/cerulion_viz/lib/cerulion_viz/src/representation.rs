// SPDX-License-Identifier: AGPL-3.0-only
//! The per-topic REPRESENTATION choice — how a topic renders, as a
//! USER decision layered on top of the archetype ladder's automagic election.
//!
//! # The ask
//!
//! An IMU topic is a plot, but the operator also wants the option to show
//! plot-type data as text (a message dump similar to lowstate). The user
//! must be able to choose what type the data shows as.
//!
//! Today the ladder ([`crate::sink::classify_frame`]) picks FOR the topic, and
//! the only topics that also get the structured field dump are the ones an
//! election earns it for: the text rule (the message declares text) and the curation rule (the
//! curation withheld numeric fields the dump can show), both of which elect
//! [`ArchetypeKind::ScalarsWithText`]. That election answers "which topics
//! NEED the dump", which is a different question from "which topics may the
//! operator ASK for it on" — and the answer to the second is *any decodable
//! topic*, because the walker decodes a frame into a
//! [`FrameValue`](cerulion_core::codegen::FrameValue) regardless of what the
//! ladder elects from it.
//!
//! # The shape
//!
//! [`Representation`] is ADDITIVE: [`Representation::Auto`] is the default and
//! reproduces today's rendering byte for byte, so the instant-viz decision and
//! every existing election are untouched. The other three arms are the operator
//! overriding that choice for one topic.
//!
//! [`resolve_render_plan`] is the whole decision, as a pure function of
//! `(elected kind, choice)`. It is called TWICE from different processes' worth
//! of state and must agree both times:
//!
//! - `cerulion-vizd`'s LAYOUT resolution, so the blueprint places the views the
//!   render will actually fill (the rule: an archetype's render and its
//!   view set are decided from the same answer, at classify time — a render arm
//!   that logged a `TextDocument` into a topic whose views carry none would hand
//!   the operator a document nothing displays);
//! - the sink's per-frame [`render_classified`](crate::sink) dispatch.
//!
//! # What it deliberately is NOT
//!
//! It is not a per-topic archetype ASSIGNMENT. The operator picks between the
//! ladder's own visual rendering and the text dump — never "render this
//! `PointCloud2` as an `Image`". The ladder still decides WHAT the visual half
//! is; this decides WHICH HALVES render.

use crate::sink::ArchetypeKind;

/// How a topic renders — the operator's choice, or [`Auto`](Self::Auto) for the
/// archetype ladder's own.
///
/// The wire spelling (`auto` / `visual` / `text` / `both`) is the vizd control
/// protocol's, mirrored by Studio. `visual` rather than "plot" because the
/// ladder's visual half is a plot only for numeric telemetry — it is a 3D cloud
/// for a `PointCloud2`, an image for a camera, a transform tree for `/tf`. A
/// value named `plot` would be an affirmatively wrong description of what it
/// selects on most topics, and Studio labels the choice with the row's own
/// archetype noun where it knows one.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Representation {
    /// The ladder decides — today's behaviour, exactly.
    #[default]
    Auto,
    /// The ladder's VISUAL rendering only: its plot / cloud / image / transform
    /// tree, with the structured field dump suppressed.
    ///
    /// On a topic whose only rendering IS text ([`ArchetypeKind::AnyValues`],
    /// [`ArchetypeKind::TextLog`]) this is a documented NO-OP rather than a
    /// blank pane — see [`resolve_render_plan`].
    Visual,
    /// The structured field dump only, with the visual half suppressed.
    Text,
    /// Both halves.
    Both,
}

impl Representation {
    /// Every value, in menu order — the set Studio offers and the set the wire
    /// accepts. Derived here so a new arm cannot be added without both surfaces
    /// seeing it.
    pub const ALL: [Representation; 4] = [Self::Auto, Self::Visual, Self::Text, Self::Both];

    /// The wire spelling.
    pub const fn as_wire(self) -> &'static str {
        match self {
            Self::Auto => "auto",
            Self::Visual => "visual",
            Self::Text => "text",
            Self::Both => "both",
        }
    }

    /// Parse a wire spelling. `None` for anything else — the caller REFUSES it
    /// loudly rather than defaulting, because silently reading an unrecognized
    /// choice as `auto` would tell the operator their click did nothing while
    /// reporting success.
    pub fn from_wire(s: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|r| r.as_wire() == s)
    }

    /// Whether this choice is the automagic default (nothing to remember, and
    /// nothing to render on a sidebar row).
    pub const fn is_auto(self) -> bool {
        matches!(self, Self::Auto)
    }
}

/// What a topic actually renders, once a [`Representation`] has been applied to
/// the kind the ladder elected.
///
/// INVARIANT: at least one of the two halves renders — `visual.is_some() ||
/// dump`. A preference must never leave a topic rendering NOTHING; that is the
/// clamp [`resolve_render_plan`] documents on each arm.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RenderPlan {
    /// The archetype whose visual half renders, or `None` when the operator
    /// suppressed it.
    pub visual: Option<ArchetypeKind>,
    /// Whether the structured field dump renders IN ADDITION to whatever
    /// `visual` logs — an OVERLAY, so it is false for the kinds that already log
    /// a dump at the topic entity themselves (see [`renders_own_dump`]) and the
    /// document is never logged twice.
    pub dump: bool,
}

impl RenderPlan {
    /// The plan for a kind nobody overrode.
    pub const fn automagic(elected: ArchetypeKind) -> Self {
        Self {
            visual: Some(elected),
            dump: false,
        }
    }
}

/// Whether `kind`'s own render arm ALWAYS logs the structured field dump at the
/// topic entity.
///
/// Deliberately NOT [`ArchetypeKind::renders_text_document`]: that predicate is
/// about which kinds may EVER need a `text_document` view, so it also covers the
/// kinds that merely CAN degrade to a dump (an [`ArchetypeKind::Image`] whose
/// encoding is not decoded). Those render their dump on a condition this frame
/// may not meet, so treating them as already-dumping would make
/// [`Representation::Both`] silently do nothing on a camera topic — the exact
/// class of silent no-op this module exists to avoid.
///
/// [`ArchetypeKind::TextLog`] is excluded for a different reason: its rolling
/// `TextDocument` mirror lands at `<entity>/text` and carries the message's TEXT,
/// not its structure, so an overlay dump at `<entity>` adds the envelope rather
/// than duplicating anything.
///
/// COST OF THAT CHOICE, stated at its real size: on a kind that is CURRENTLY
/// degrading, [`Representation::Both`] logs the same document twice (same entity,
/// same timestamp, latest-at wins — one redundant chunk, never a wrong render).
/// For most degrading kinds that is a per-frame corner; for
/// [`ArchetypeKind::Skeleton`] it is the SHIPPING state, because the removal of
/// the last installer left every skeleton-classified topic dumping on every frame, so
/// there `Both` doubles every admitted frame and buys nothing. Widening
/// `renders_own_dump` would be the wrong lever (it would silence `Both` on a
/// camera, which the decision table pins); the fix, if it is ever worth one, is
/// for the degrade path to skip its own dump when the overlay already rendered.
pub const fn renders_own_dump(kind: ArchetypeKind) -> bool {
    matches!(
        kind,
        ArchetypeKind::ScalarsWithText | ArchetypeKind::AnyValues
    )
}

/// The visual half of `kind` with any text half stripped off.
///
/// Only [`ArchetypeKind::ScalarsWithText`] has a separable text half — it is
/// exactly [`ArchetypeKind::Scalars`] plus the dump, which is why those elections
/// could elect it without inventing a render. Every other kind is returned
/// unchanged, INCLUDING the two whose whole rendering is text: stripping those
/// would leave nothing on screen, and a preference that blanks a topic is worse
/// than one that does nothing.
const fn strip_text(kind: ArchetypeKind) -> ArchetypeKind {
    match kind {
        ArchetypeKind::ScalarsWithText => ArchetypeKind::Scalars,
        other => other,
    }
}

/// THE decision: what `elected` renders under `choice`.
///
/// | choice | visual | dump | note |
/// |---|---|---|---|
/// | [`Auto`](Representation::Auto) | the elected kind | `false` | the elected kind's own arm decides — `ScalarsWithText` still dumps, exactly as today |
/// | [`Visual`](Representation::Visual) | its visual half, text stripped | `false` | a no-op on a kind with no separable text half |
/// | [`Text`](Representation::Text) | `None` | `true` | the dump becomes the topic's whole rendering |
/// | [`Both`](Representation::Both) | the elected kind | `!`[`renders_own_dump`] | the overlay, skipped where the arm already dumps |
///
/// PURE — no clock, no state, no I/O — so vizd's layout resolution and the
/// sink's per-frame dispatch derive the same plan from the same two inputs, and
/// a replay renders what the live run did (Principle #7).
pub const fn resolve_render_plan(elected: ArchetypeKind, choice: Representation) -> RenderPlan {
    match choice {
        Representation::Auto => RenderPlan::automagic(elected),
        Representation::Visual => RenderPlan {
            visual: Some(strip_text(elected)),
            dump: false,
        },
        Representation::Text => RenderPlan {
            visual: None,
            dump: true,
        },
        Representation::Both => RenderPlan {
            visual: Some(elected),
            dump: !renders_own_dump(elected),
        },
    }
}

/// How long a FORCED dump may go without re-rendering, in WIRE nanoseconds —
/// i.e. it renders at most ~10 Hz of publisher time.
///
/// The dump the operator asked for is a markdown document a human reads; past
/// ~10 Hz a refresh is invisible to the reader and pure cost to the viewer. The
/// measured shape this bounds is the dump gate's: a ~500 Hz `/lowstate`-class bank
/// dumps a MEASURED ~2 150 bytes, so an unmetered forced dump is ~1 MB/s of
/// markdown per topic — which is precisely the load the dump gate metered the
/// automatic election down from, and there is no reason a manual election should
/// re-open it.
///
/// WIRE time, never wall time: decimation keyed on the publisher's own stamps is
/// a pure function of the frame stream, so a replay renders the same documents
/// (Principle #7 — the rule [`crate::plot_rate`] states at length).
///
/// That choice is deliberate and it BOUNDS THE BYTE FIGURE ABOVE TO REAL TIME
/// ONLY: the ~1 MB/s comparison is wall-relative, while this gate counts the
/// publisher's clock. A bag replayed at 10×, a simulated clock or a `VirtualClock`
/// worker advances stamps faster than the wall, so ~100 documents/s can leave here
/// in a wall second. The trade is still right — a rate gate that read the wall
/// would make a replay render a different document set — but the ceiling is "10
/// per publisher second", not "10 per second".
pub const FORCED_DUMP_MIN_INTERVAL_NS: u64 = 100_000_000;

/// How many refused frames a forced dump may accumulate before it renders
/// regardless — the STALLED-CLOCK backstop.
///
/// A wire-time interval alone is unsatisfiable on a publisher whose stamps never
/// advance (the `StalledClock` class): the first frame renders, and
/// every later frame is refused forever, freezing the operator's text at frame
/// one. Mirrors [`crate::plot_rate::DUMP_REFRESH_FRAME_FLOOR`] in both value and
/// reasoning — a FRAME count, because in the case it exists for the clock is
/// what has stopped.
pub const FORCED_DUMP_FRAME_FLOOR: u64 = crate::plot_rate::DUMP_REFRESH_FRAME_FLOOR;

/// One topic's forced-dump refresh gate — a pure state machine over the wire
/// stamps, with no clock of its own.
///
/// Separate from [`DumpRefreshGate`](crate::plot_rate::DumpRefreshGate) because
/// the two meter different things from different signals: that gate rides the
/// SERIES gate's verdict (the dump is a snapshot of the same message the plot
/// lines come from, so one cadence keeps one story on screen), and a forced dump
/// may sit on a topic that has no series gate at all — a camera, a cloud, a
/// transform tree. Its own interval is the right answer there, and it leaves
/// every automatic `ScalarsWithText` election metered exactly as the dump gate left it.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ForcedDumpGate {
    /// The wire stamp of the last RENDERED dump.
    last_render_ns: Option<u64>,
    /// Frames refused since that render — the floor's counter.
    since_render: u64,
    /// How many renders this gate has withheld (Principle #3: observable
    /// without reading a log line).
    suppressed: u64,
}

impl ForcedDumpGate {
    /// Decide whether the frame stamped `timestamp_ns` renders its forced dump,
    /// and record the decision.
    ///
    /// A stamp that goes BACKWARDS renders: the only way one arrives is a
    /// publisher restart or a clock epoch change, and holding the old document
    /// against a stream that has plainly restarted is the frozen-text failure
    /// this gate's floor exists to prevent.
    pub fn admit(&mut self, timestamp_ns: u64) -> bool {
        let due = match self.last_render_ns {
            None => true,
            Some(last) => {
                timestamp_ns < last
                    || timestamp_ns.saturating_sub(last) >= FORCED_DUMP_MIN_INTERVAL_NS
            }
        };
        if due || self.since_render >= FORCED_DUMP_FRAME_FLOOR {
            self.last_render_ns = Some(timestamp_ns);
            self.since_render = 0;
            return true;
        }
        self.since_render += 1;
        self.suppressed += 1;
        false
    }

    /// How many dump renders this gate has withheld so far.
    pub const fn suppressed(&self) -> u64 {
        self.suppressed
    }

    /// Frames refused since the last rendered dump (test / observability seam).
    pub const fn since_render(&self) -> u64 {
        self.since_render
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every arm of the decision, against a HAND-WRITTEN table — never the
    /// implementation re-derived.
    ///
    /// The three non-`Auto` rows for `Scalars` and `ScalarsWithText` are the
    /// requested case in both directions: an IMU-class plot topic gaining
    /// text, and a `/lowstate`-class topic (which already earned the dump) losing
    /// it.
    #[test]
    fn the_decision_table_is_exactly_what_each_arm_promises() {
        let cases: &[(ArchetypeKind, Representation, Option<ArchetypeKind>, bool)] = &[
            // Auto reproduces the election, whatever it was.
            (
                ArchetypeKind::Scalars,
                Representation::Auto,
                Some(ArchetypeKind::Scalars),
                false,
            ),
            (
                ArchetypeKind::ScalarsWithText,
                Representation::Auto,
                Some(ArchetypeKind::ScalarsWithText),
                false,
            ),
            (
                ArchetypeKind::Points3D,
                Representation::Auto,
                Some(ArchetypeKind::Points3D),
                false,
            ),
            // Visual strips the ONE separable text half and nothing else.
            (
                ArchetypeKind::ScalarsWithText,
                Representation::Visual,
                Some(ArchetypeKind::Scalars),
                false,
            ),
            (
                ArchetypeKind::Scalars,
                Representation::Visual,
                Some(ArchetypeKind::Scalars),
                false,
            ),
            (
                ArchetypeKind::Image,
                Representation::Visual,
                Some(ArchetypeKind::Image),
                false,
            ),
            // Text is the dump, on any kind.
            (ArchetypeKind::Scalars, Representation::Text, None, true),
            (ArchetypeKind::Points3D, Representation::Text, None, true),
            (ArchetypeKind::VideoStream, Representation::Text, None, true),
            // Both overlays the dump — except where the arm already dumps.
            (
                ArchetypeKind::Scalars,
                Representation::Both,
                Some(ArchetypeKind::Scalars),
                true,
            ),
            (
                ArchetypeKind::Imu,
                Representation::Both,
                Some(ArchetypeKind::Imu),
                true,
            ),
            (
                ArchetypeKind::Image,
                Representation::Both,
                Some(ArchetypeKind::Image),
                true,
            ),
            (
                ArchetypeKind::ScalarsWithText,
                Representation::Both,
                Some(ArchetypeKind::ScalarsWithText),
                false,
            ),
            (
                ArchetypeKind::AnyValues,
                Representation::Both,
                Some(ArchetypeKind::AnyValues),
                false,
            ),
        ];
        for &(elected, choice, want_visual, want_dump) in cases {
            let got = resolve_render_plan(elected, choice);
            assert_eq!(
                (got.visual, got.dump),
                (want_visual, want_dump),
                "{elected:?} under {choice:?}"
            );
        }
    }

    /// The clamp, stated as an invariant over the WHOLE cross product: no
    /// (kind, choice) pair may render nothing.
    ///
    /// This is what makes `Visual` a no-op rather than a blank pane on the two
    /// text-only kinds, and it is checked over `ArchetypeKind::ALL` so a kind
    /// added later cannot quietly acquire a blanking combination.
    #[test]
    fn no_choice_on_any_kind_renders_nothing() {
        for kind in ArchetypeKind::ALL {
            for choice in Representation::ALL {
                let plan = resolve_render_plan(kind, choice);
                assert!(
                    plan.visual.is_some() || plan.dump,
                    "{kind:?} under {choice:?} renders nothing"
                );
            }
        }
    }

    /// `Text` is the one arm whose answer does not depend on the election at
    /// all — the operator asked for the message, and the walker decodes it
    /// whatever the ladder made of it.
    #[test]
    fn text_is_the_same_answer_for_every_kind() {
        for kind in ArchetypeKind::ALL {
            assert_eq!(
                resolve_render_plan(kind, Representation::Text),
                RenderPlan {
                    visual: None,
                    dump: true
                },
                "{kind:?}"
            );
        }
    }

    /// `Auto` must be BYTE-identical to no override at all, on every kind —
    /// this is the additivity promise, and the reason the instant-viz decision is
    /// untouched.
    #[test]
    fn auto_reproduces_the_election_on_every_kind() {
        for kind in ArchetypeKind::ALL {
            let plan = resolve_render_plan(kind, Representation::Auto);
            assert_eq!(plan, RenderPlan::automagic(kind), "{kind:?}");
            assert!(!plan.dump, "{kind:?}: auto must never add an overlay");
        }
    }

    /// `Both` adds the dump on every kind that does not already log one, and on
    /// no kind that does — the anti-double-log rule, over the whole enum.
    #[test]
    fn both_overlays_exactly_the_kinds_that_do_not_dump_themselves() {
        for kind in ArchetypeKind::ALL {
            let plan = resolve_render_plan(kind, Representation::Both);
            assert_eq!(plan.visual, Some(kind), "{kind:?}");
            assert_eq!(
                plan.dump,
                !renders_own_dump(kind),
                "{kind:?}: overlay must be the complement of renders_own_dump"
            );
        }
    }

    /// The wire vocabulary round-trips, and nothing else parses.
    ///
    /// An unrecognized spelling must be `None` rather than `Auto`: a client that
    /// sent `"plot"` (or a future value an old daemon does not know) has to be
    /// told, not silently answered with the default while its request is dropped.
    #[test]
    fn the_wire_vocabulary_round_trips_and_refuses_everything_else() {
        for r in Representation::ALL {
            assert_eq!(Representation::from_wire(r.as_wire()), Some(r));
        }
        let wire: Vec<&str> = Representation::ALL.iter().map(|r| r.as_wire()).collect();
        assert_eq!(wire, ["auto", "visual", "text", "both"]);
        for bad in ["", "plot", "Auto", "AUTO", "dump", "visual ", "textual"] {
            assert_eq!(Representation::from_wire(bad), None, "{bad:?}");
        }
        assert!(Representation::default().is_auto());
    }

    /// The forced-dump gate: one render, then a refusal window one nanosecond
    /// wide of the interval, then the next render — both sides of the threshold.
    #[test]
    fn the_forced_dump_interval_is_a_threshold_pinned_on_both_sides() {
        let mut gate = ForcedDumpGate::default();
        assert!(gate.admit(1_000), "the first frame always renders");
        assert!(
            !gate.admit(1_000 + FORCED_DUMP_MIN_INTERVAL_NS - 1),
            "one ns short of the interval must be refused"
        );
        assert_eq!(gate.suppressed(), 1);
        assert!(
            gate.admit(1_000 + FORCED_DUMP_MIN_INTERVAL_NS),
            "exactly the interval renders"
        );
        assert_eq!(gate.since_render(), 0, "a render clears the floor counter");
        assert_eq!(gate.suppressed(), 1, "a render withholds nothing");
    }

    /// A publisher whose stamps never advance still refreshes, at the floor —
    /// the anti-freeze backstop, asserted at the exact frame it fires.
    #[test]
    fn a_stalled_clock_still_refreshes_at_the_frame_floor() {
        let mut gate = ForcedDumpGate::default();
        assert!(gate.admit(7), "first render");
        for i in 0..FORCED_DUMP_FRAME_FLOOR {
            assert!(!gate.admit(7), "frame {i} must be refused");
        }
        assert!(
            gate.admit(7),
            "the frame at the floor renders despite an unmoving clock"
        );
        assert_eq!(gate.suppressed(), FORCED_DUMP_FRAME_FLOOR);
    }

    /// A backwards stamp (publisher restart / clock epoch change) renders rather
    /// than holding the pre-restart document against a stream that restarted.
    #[test]
    fn a_backwards_stamp_renders_rather_than_freezing_the_document() {
        let mut gate = ForcedDumpGate::default();
        assert!(gate.admit(10 * FORCED_DUMP_MIN_INTERVAL_NS));
        assert!(
            gate.admit(5),
            "a stamp below the last render is a new epoch, not a refusal"
        );
        assert_eq!(gate.suppressed(), 0);
    }

    /// Determinism: the same stamp stream yields the same verdict stream, and
    /// the verdicts are what a hand-written oracle says (a replay renders the
    /// same documents — Principle #7).
    #[test]
    fn the_gate_is_a_pure_function_of_the_stamp_stream() {
        let step = FORCED_DUMP_MIN_INTERVAL_NS / 4;
        let stamps: Vec<u64> = (0..9).map(|i| i * step).collect();
        // 0 renders (first), 1..3 are inside the interval, 4 is exactly at it,
        // 5..7 inside again, 8 exactly at it.
        let oracle = [true, false, false, false, true, false, false, false, true];
        for _ in 0..2 {
            let mut gate = ForcedDumpGate::default();
            let got: Vec<bool> = stamps.iter().map(|&t| gate.admit(t)).collect();
            assert_eq!(got, oracle);
            assert_eq!(gate.suppressed(), 6);
        }
    }
}

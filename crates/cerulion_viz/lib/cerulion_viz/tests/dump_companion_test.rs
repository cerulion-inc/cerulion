//! The dump companion is refused on a LIVE SIGNAL, per
//! archetype — never blanket, and it comes BACK when a degradation fires.
//!
//! An always-there `text_document` companion on every degradable render arm
//! gives a frame-time degradation somewhere to land ("nothing
//! may render nothing"), and costs one empty pane on every healthy
//! camera / map / marker topic. VIDEO refuses that cost on
//! the evidence of the demux's rendition set. The decision pinned here is that
//! the same holds for every archetype that carries an equivalent live signal.
//!
//! **What each arm here is FOR.** The headline is the (kind x proof) table, driven
//! over the CLOSED [`ArchetypeKind::ALL`] set with HAND-WRITTEN expectations — a
//! new archetype variant shows up as a gap rather than escaping a sampled list.
//! Around it sit the arms a table cannot state: that the two signals are
//! per-archetype and not interchangeable, that a pane earned by a PRIMARY text
//! render is never eligible, that the field-dump archetype cannot be left with no
//! view at all, and — the one the decision calls load-bearing — that a topic which
//! rendered and then DEGRADED gets its pane back.
//!
//! Every expectation is written out, never derived from the predicate under test.

use cerulion_viz::blueprint::{views_for_archetype, views_for_render, AttachedRender, ViewKind};
use cerulion_viz::representation::Representation;
use cerulion_viz::sink::{ArchetypeKind, RenderProof};

/// The archetypes whose `text_document` pane is EARNED BY DEGRADING and that also
/// render something else, the set this change can convert. Hand-written: it is the
/// oracle for `ArchetypeKind::can_degrade_to_dump` intersected with "has a visual
/// half", so widening either silently fails the table below.
///
/// `Skeleton` is in the set on purpose. Structurally it qualifies (a
/// `spatial3d` + `text_document` kind that degrades), and if a URDF were ever
/// installed its pane WOULD be empty. What keeps its companion on a live run is
/// not an exception here but the evidence: `install_skeleton` has had no caller
/// since its node was deleted, so its arm always dumps and the signal can never turn true —
/// pinned separately by `an_archetype_that_only_ever_dumps_keeps_its_companion`.
const CONVERTIBLE: &[ArchetypeKind] = &[
    ArchetypeKind::Image,
    ArchetypeKind::OccupancyGrid,
    ArchetypeKind::Boxes3D,
    ArchetypeKind::Path3D,
    ArchetypeKind::PoseArray3D,
    ArchetypeKind::MarkerArray,
    ArchetypeKind::VideoStream,
    ArchetypeKind::Skeleton,
];

/// A topic with NO live signal — the state every attach starts in.
fn unobserved(kind: ArchetypeKind) -> AttachedRender {
    AttachedRender {
        topic: "/t".to_string(),
        entity: "world/t".to_string(),
        archetype: Some(kind),
        producer_count: None,
        video_renditions: Vec::new(),
        representation: Representation::Auto,
        render_proof: RenderProof::default(),
    }
}

/// The same topic, with the signal that PROVES its visual half is rendering.
///
/// The `match` is the per-archetype signal table stated by hand: video reads the
/// demux's rendition set (its shipped evidence, untouched), everything else
/// reads the render arm's own report. Writing it out here rather than calling the
/// production chooser is what makes `the_two_signals_are_not_interchangeable`
/// meaningful — a test that asked the code which signal to set could not tell that
/// the code had started reading the wrong one.
fn proven(kind: ArchetypeKind) -> AttachedRender {
    let mut t = unobserved(kind);
    match kind {
        ArchetypeKind::VideoStream => t.video_renditions = vec!["1280x720".to_string()],
        _ => t.render_proof.rendered_without_dumping = true,
    }
    t
}

/// A topic that rendered and then DEGRADED, the state this change's core guarantee is
/// about.
fn degraded(kind: ArchetypeKind) -> AttachedRender {
    let mut t = proven(kind);
    t.render_proof.degraded = true;
    t
}

fn without_dump(kind: ArchetypeKind) -> Vec<ViewKind> {
    views_for_archetype(kind)
        .iter()
        .copied()
        .filter(|v| *v != ViewKind::TextDocument)
        .collect()
}

fn archetype_views(kind: ArchetypeKind) -> Vec<ViewKind> {
    views_for_archetype(kind).to_vec()
}

// ── The headline: the whole (kind x proof) table, in ONE test ────────────────

#[test]
fn every_archetype_answers_the_same_three_states_the_same_way() {
    for kind in ArchetypeKind::ALL {
        let convertible = CONVERTIBLE.contains(&kind);

        // (1) NOTHING OBSERVED: the companion protection, and the state a
        //     never-seen robot's topic sits in until its first frame lands. Every
        //     archetype keeps exactly the views its KIND declares.
        assert_eq!(
            views_for_render(&unobserved(kind)),
            archetype_views(kind),
            "{kind:?}: a topic whose render nobody has observed must keep the views \
             its archetype declares — refusing the text_document companion here would be \
             the blanket drop the companion guarantee forbids"
        );

        // (2) PROVEN RENDERING — the convertible set, and only it, loses the pane.
        let want_proven = if convertible {
            without_dump(kind)
        } else {
            archetype_views(kind)
        };
        assert_eq!(
            views_for_render(&proven(kind)),
            want_proven,
            "{kind:?}: a topic proven to be rendering keeps a text_document pane \
             ONLY when that pane is not the always-there companion (a primary text \
             render, or the field dump's own only view)"
        );

        // (3) DEGRADED — the pane is back (or never left). This is the arm the
        //     decision calls load-bearing: the companion guarantee has to hold
        //     end-to-end, so a dump that fires must have somewhere to land.
        assert_eq!(
            views_for_render(&degraded(kind)),
            archetype_views(kind),
            "{kind:?}: once a frame has DEGRADED, the dump has somewhere to land — \
             for the rest of the attach"
        );

        // The convertible set really does lose something, so arm (2) is not
        // vacuously equal to arm (1) for the kinds this feature is about.
        if convertible {
            assert!(
                archetype_views(kind).contains(&ViewKind::TextDocument)
                    && !want_proven.contains(&ViewKind::TextDocument),
                "{kind:?} is declared CONVERTIBLE, so it must carry a text_document \
                 pane that a proven render actually removes"
            );
        }
    }
}

// ── The arms the table cannot state ──────────────────────────────────────────

#[test]
fn a_topic_that_rendered_then_degraded_gets_its_dump_pane_back() {
    // The decision's headline, stated once per convertible archetype as a sequence
    // rather than as three independent lookups: healthy => no companion, then a
    // degradation is observed => the companion is present and carries the dump.
    for &kind in CONVERTIBLE {
        let healthy = views_for_render(&proven(kind));
        assert!(
            !healthy.contains(&ViewKind::TextDocument),
            "{kind:?}: a provably-rendering topic must not carry an empty status pane"
        );
        assert!(
            !healthy.is_empty(),
            "{kind:?}: dropping the companion must never leave a topic with no view"
        );

        let after = views_for_render(&degraded(kind));
        assert!(
            after.contains(&ViewKind::TextDocument),
            "{kind:?}: the dump fired, so the pane it lands in must APPEAR — this is \
             the whole of the companion guarantee this change must not trade away"
        );
        assert_eq!(
            after,
            archetype_views(kind),
            "{kind:?}: and the topic is otherwise placed exactly as its archetype says"
        );
    }
}

#[test]
fn a_degradation_is_sticky_so_the_pane_cannot_flicker_away_again() {
    // A later healthy frame does NOT take the pane back: `degraded` latches for
    // the life of the attach. A pane that vanished on the next good frame could
    // never be READ, which is the operator-facing half of the guarantee — and it
    // is also what bounds the layout at two moves per topic instead of one per
    // frame (the "layout must not be a function of which frame arrived
    // first" rule).
    for &kind in CONVERTIBLE {
        let mut healed = degraded(kind);
        healed.render_proof.rendered_without_dumping = true; // more good frames
        assert_eq!(
            views_for_render(&healed),
            archetype_views(kind),
            "{kind:?}: a topic that has ever dumped keeps its pane even while it is \
             rendering again"
        );
    }
}

#[test]
fn the_two_signals_are_not_interchangeable() {
    // Video reads the rendition set; every other archetype reads the render arm's
    // report. Cross the wires and the answer must NOT change — otherwise a
    // `pub` field any caller can fill would be deciding another archetype's
    // layout, and the per-archetype table above would be fiction.

    // A NON-video degradable topic carrying a rendition set (a shape the demux
    // cannot mint, but the field is `pub`) is NOT proven.
    let mut grid = unobserved(ArchetypeKind::OccupancyGrid);
    grid.video_renditions = vec!["1280x720".to_string()];
    assert_eq!(
        views_for_render(&grid),
        archetype_views(ArchetypeKind::OccupancyGrid),
        "a rendition set is VIDEO's evidence — it says nothing about an occupancy grid"
    );

    // A VIDEO topic whose arm reported clean but whose demux opened no stream is
    // NOT proven: its evidence is the decoder, deliberately stronger than
    // the arm's return path, and this change leaves it exactly as it shipped.
    let mut video = unobserved(ArchetypeKind::VideoStream);
    video.render_proof.rendered_without_dumping = true;
    assert_eq!(
        views_for_render(&video),
        archetype_views(ArchetypeKind::VideoStream),
        "video's signal is the rendition set (video's shipped template), not the \
         general arm report"
    );
}

#[test]
fn an_archetype_whose_pane_is_a_primary_render_never_loses_it() {
    // `TextLog` mirrors a rolling `TextDocument` (rerun 0.34 ships no
    // TextLogView) and `ScalarsWithText` logs the field dump BESIDE its plots on
    // every frame. Their pane is not the companion — it is where
    // their own data lands — so a proven render must never remove it. Dropping it
    // would be the dual-view twin's silent payload drop, re-created from the other side.
    for kind in [ArchetypeKind::TextLog, ArchetypeKind::ScalarsWithText] {
        assert!(
            archetype_views(kind).contains(&ViewKind::TextDocument),
            "{kind:?} must carry a text_document view for this arm to mean anything"
        );
        assert_eq!(
            views_for_render(&proven(kind)),
            archetype_views(kind),
            "{kind:?} earns its text_document view by RENDERING text, not by degrading"
        );
    }
}

#[test]
fn the_field_dump_archetype_keeps_its_only_view() {
    // `AnyValues` IS a dump — its view set is text-only, so a refusal would leave
    // the topic with NO pane at all, strictly worse than the empty one this change
    // removes. The guard is structural (a kind must have a visual half), so a
    // future degradable-but-text-only archetype is protected without being named.
    let kind = ArchetypeKind::AnyValues;
    assert_eq!(
        archetype_views(kind),
        vec![ViewKind::TextDocument],
        "AnyValues renders as a field dump and nothing else"
    );
    for topic in [unobserved(kind), proven(kind), degraded(kind)] {
        assert_eq!(
            views_for_render(&topic),
            vec![ViewKind::TextDocument],
            "the field-dump archetype keeps its only view in every state"
        );
    }
}

#[test]
fn an_archetype_that_only_ever_dumps_keeps_its_companion() {
    // `Skeleton`'s arm is INERT on every live run: the robot-side viz removal deleted the last
    // `install_skeleton` caller, so it takes the dump branch for every frame. Its
    // companion is therefore kept by the EVIDENCE rather than by an exception:
    // whatever else is true, a topic that has dumped carries `degraded`.
    //
    // This is the shape of the decision's "where no live signal exists, the
    // companion STAYS": the predicate needs no Skeleton arm, because a skeleton
    // topic can never reach the proven-and-never-degraded state on a real run.
    let kind = ArchetypeKind::Skeleton;
    let live_reality = degraded(kind); // what every live skeleton frame produces
    assert_eq!(
        views_for_render(&live_reality),
        archetype_views(kind),
        "an archetype whose arm always dumps keeps the pane its dump lands in"
    );
    assert!(
        views_for_render(&live_reality).contains(&ViewKind::TextDocument),
        "and that pane is the text_document one"
    );
}

#[test]
fn an_operator_who_asked_for_the_dump_still_gets_it_on_a_proven_topic() {
    // The operator's REPRESENTATION choice is the discriminator, taken
    // FIRST. A forced dump never reaches the refusal, on any archetype and however
    // healthy — otherwise `representation text` / `both` would be a silent no-op
    // on exactly the topics this change touches.
    for &kind in CONVERTIBLE {
        for choice in [Representation::Text, Representation::Both] {
            let mut topic = proven(kind);
            topic.representation = choice;
            assert!(
                views_for_render(&topic).contains(&ViewKind::TextDocument),
                "{kind:?} under {choice:?}: a FORCED dump must always get its pane"
            );
        }
    }
}

#[test]
fn a_silent_topic_still_yields_no_views_in_any_proof_state() {
    // `archetype: None` means "not yet resolved", which is un-TYPED rather than
    // un-rendered — the layout treats it as un-placeable. No render proof can
    // change that (and a proof on a silent topic is not a state the daemon can
    // produce, since the proof is keyed on a route whose frames resolved it).
    let mut silent = unobserved(ArchetypeKind::Image);
    silent.archetype = None;
    silent.render_proof = RenderProof {
        rendered_without_dumping: true,
        degraded: true,
    };
    assert!(
        views_for_render(&silent).is_empty(),
        "a still-silent topic is un-placeable whatever its render proof says"
    );
}

// SPDX-License-Identifier: AGPL-3.0-only
//! The compose compiler PREFERS LIVE topics — pinned against HAND oracles
//! (never a self-compare against the function under test).
//!
//! [`cerulion_viz::blueprint::compose_layout`] reads each resolved topic's
//! `producer_count` liveness and:
//! - AUTOMAGIC (role-less) path: a DEAD route (`Some(0)`) is SKIPPED from placement
//!   and recorded in `skipped_dead` with a human reason (never silently);
//! - EXPLICIT (role-tagged) path: a dead route is PLACED (user intent wins) but the
//!   placement is FLAGGED `dead: true`;
//! - `None` (UNKNOWN) and `Some(n>0)` (LIVE) are never penalized.
//!
//! The motivating case: on the Go2, `/uslam/cloud_map` (a dead DDS route) must not
//! consume a hero-view slot while `/utlidar/cloud_deskewed` streams live.

use cerulion_viz::blueprint::{
    compose_layout, AttachedRender, ComposeError, ComposeGroup, ComposeInput, ComposeStrategy,
    ComposedLayout, GroupRole, ViewKind,
};
use cerulion_viz::representation::Representation;
use cerulion_viz::sink::ArchetypeKind;
use cerulion_viz::sink::RenderProof;

// ── Hand-built resolved topics with an explicit liveness ──────────────────────

/// A resolved topic with a chosen archetype + explicit `producer_count`.
fn resolved(topic: &str, entity: &str, arch: ArchetypeKind, pc: Option<u32>) -> AttachedRender {
    AttachedRender {
        topic: topic.to_string(),
        entity: entity.to_string(),
        archetype: Some(arch),
        producer_count: pc,
        video_renditions: Vec::new(),
        representation: Representation::Auto,
        render_proof: RenderProof::default(),
    }
}

/// A resolved topic that is DEAD (probed `Some(0)` producers).
fn dead(topic: &str, entity: &str, arch: ArchetypeKind) -> AttachedRender {
    resolved(topic, entity, arch, Some(0))
}

/// A resolved topic that is LIVE (probed `Some(n)`, n > 0).
fn live(topic: &str, entity: &str, arch: ArchetypeKind) -> AttachedRender {
    resolved(topic, entity, arch, Some(3))
}

/// A resolved topic whose liveness is UNKNOWN (probe failed / not carried).
fn unknown(topic: &str, entity: &str, arch: ArchetypeKind) -> AttachedRender {
    resolved(topic, entity, arch, None)
}

/// Bare-topics automagic input (ONE role-less group).
fn bare(topics: Vec<AttachedRender>) -> ComposeInput {
    ComposeInput {
        groups: vec![ComposeGroup {
            topics,
            role: None,
            title: None,
        }],
        strategy: ComposeStrategy::Auto,
        include_robot: false,
    }
}

/// The placed topics (any view), in placement order.
fn placed_topics(l: &ComposedLayout) -> Vec<String> {
    l.placements.iter().map(|p| p.topic.clone()).collect()
}

// The two clouds from the motivating case.
const DEAD_MAP: &str = "world/uslam/cloud_map";
const LIVE_CLOUD: &str = "world/utlidar/cloud_deskewed";

// ── 1. Automagic: a dead route is SKIPPED + reported (never placed, never silent) ─

#[test]
fn automagic_dead_route_is_skipped_and_reported_with_a_reason() {
    let l = compose_layout(&bare(vec![
        live(
            "/utlidar/cloud_deskewed",
            LIVE_CLOUD,
            ArchetypeKind::Points3D,
        ),
        dead("/uslam/cloud_map", DEAD_MAP, ArchetypeKind::Points3D),
    ]))
    .expect("one live topic composes");

    // The live topic IS placed; the dead one is NOT (in ANY view).
    assert_eq!(
        placed_topics(&l),
        vec!["/utlidar/cloud_deskewed".to_string()]
    );
    assert!(
        !placed_topics(&l).contains(&"/uslam/cloud_map".to_string()),
        "a dead route never lands in a view"
    );

    // The dead topic is surfaced EXPLICITLY with a human reason.
    assert_eq!(l.skipped_dead.len(), 1);
    assert_eq!(l.skipped_dead[0].topic, "/uslam/cloud_map");
    assert_eq!(l.skipped_dead[0].reason, "never published — 0 producers");

    // Every kept placement is NOT flagged dead (only explicit-role dead placements are).
    assert!(l.placements.iter().all(|p| !p.dead));
}

// ── 2. Motivating case: the dead cloud_map never takes the hero slot ──────────

#[test]
fn founder_case_dead_cloud_map_never_consumes_the_hero_slot() {
    // Both are Points3D (spatial3d/hero). Without this rule BOTH would land in
    // the hero view; with it only the live one does, and the dead one is reported out.
    let l = compose_layout(&bare(vec![
        dead("/uslam/cloud_map", DEAD_MAP, ArchetypeKind::Points3D),
        live(
            "/utlidar/cloud_deskewed",
            LIVE_CLOUD,
            ArchetypeKind::Points3D,
        ),
    ]))
    .expect("the live cloud composes");

    // Exactly ONE spatial3d placement — the LIVE cloud.
    let hero: Vec<&str> = l
        .placements
        .iter()
        .filter(|p| p.view == ViewKind::Spatial3d)
        .map(|p| p.topic.as_str())
        .collect();
    assert_eq!(
        hero,
        vec!["/utlidar/cloud_deskewed"],
        "only the live cloud is in the hero 3D view"
    );
    assert_eq!(
        l.skipped_dead
            .iter()
            .map(|s| s.topic.as_str())
            .collect::<Vec<_>>(),
        vec!["/uslam/cloud_map"]
    );
}

// ── 3. Automagic where EVERY topic is dead → a loud, actionable refusal ────────

#[test]
fn all_dead_automagic_errors_actionably_naming_the_dead_topics() {
    let err = compose_layout(&bare(vec![
        dead("/uslam/cloud_map", DEAD_MAP, ArchetypeKind::Points3D),
        dead("/dead_scan", "world/dead_scan", ArchetypeKind::LaserScan),
    ]))
    .expect_err("all-dead → a loud refusal, never an empty layout");

    assert_eq!(
        err,
        ComposeError::AllTopicsDead {
            dead: vec!["/uslam/cloud_map".to_string(), "/dead_scan".to_string()],
        }
    );
    let msg = err.to_string();
    assert!(msg.contains("/uslam/cloud_map") && msg.contains("/dead_scan"));
    assert!(msg.contains("DEAD route") && msg.contains("no views"));
}

// ── 4. Explicit role group: a dead topic is PLACED but FLAGGED ────────────────

#[test]
fn explicit_role_group_dead_topic_is_placed_but_flagged_dead() {
    // The user explicitly put a dead route in a `hero` group — intent wins, it IS
    // placed (so the pane is laid out for when data arrives), but flagged.
    let input = ComposeInput {
        groups: vec![ComposeGroup {
            topics: vec![dead("/uslam/cloud_map", DEAD_MAP, ArchetypeKind::Points3D)],
            role: Some(GroupRole::Hero),
            title: None,
        }],
        strategy: ComposeStrategy::Auto,
        include_robot: false,
    };
    let l = compose_layout(&input).expect("an explicit dead topic still composes");

    assert_eq!(placed_topics(&l), vec!["/uslam/cloud_map".to_string()]);
    let p = &l.placements[0];
    assert_eq!(p.view, ViewKind::Spatial3d);
    assert!(p.dead, "an explicit dead placement is flagged");
    // It is NOT in skipped_dead (only the automagic path skips).
    assert!(l.skipped_dead.is_empty());
    // And a warning names the dead route + the empty-pane consequence.
    assert!(l
        .warnings
        .iter()
        .any(|w| w.contains("/uslam/cloud_map") && w.contains("DEAD route")));
}

// ── 5. Unknown liveness (None) is NEVER penalized ─────────────────────────────

#[test]
fn unknown_liveness_is_treated_as_live_and_placed() {
    let l = compose_layout(&bare(vec![unknown(
        "/some/cloud",
        "world/utlidar/cloud",
        ArchetypeKind::Points3D,
    )]))
    .expect("an unknown-liveness topic composes (never penalized)");

    assert_eq!(placed_topics(&l), vec!["/some/cloud".to_string()]);
    assert!(l.placements.iter().all(|p| !p.dead));
    assert!(l.skipped_dead.is_empty(), "unknown is not dead");
}

// ── 6. A positive producer count is placed normally ───────────────────────────

#[test]
fn live_positive_count_is_placed_and_not_flagged() {
    let l = compose_layout(&bare(vec![resolved(
        "/cloud",
        "world/utlidar/cloud",
        ArchetypeKind::Points3D,
        Some(1),
    )]))
    .expect("a live topic composes");
    assert_eq!(placed_topics(&l), vec!["/cloud".to_string()]);
    assert!(l.placements.iter().all(|p| !p.dead));
    assert!(l.skipped_dead.is_empty());
}

// ── 7. Mixed: an explicit live group + a role-less dead group STILL succeeds ───

#[test]
fn mixed_explicit_live_and_automagic_dead_succeeds_with_the_dead_reported() {
    // "compose still succeeds with whatever the user explicitly named" — the role
    // group's live topic places; the role-less group's dead topic is skipped + noted.
    let input = ComposeInput {
        groups: vec![
            ComposeGroup {
                topics: vec![live(
                    "/utlidar/cloud_deskewed",
                    LIVE_CLOUD,
                    ArchetypeKind::Points3D,
                )],
                role: Some(GroupRole::Hero),
                title: None,
            },
            ComposeGroup {
                topics: vec![dead("/uslam/cloud_map", DEAD_MAP, ArchetypeKind::Points3D)],
                role: None,
                title: None,
            },
        ],
        strategy: ComposeStrategy::Auto,
        include_robot: false,
    };
    let l =
        compose_layout(&input).expect("the explicitly-named live topic keeps the compose alive");

    assert_eq!(
        placed_topics(&l),
        vec!["/utlidar/cloud_deskewed".to_string()]
    );
    assert_eq!(
        l.skipped_dead
            .iter()
            .map(|s| s.topic.as_str())
            .collect::<Vec<_>>(),
        vec!["/uslam/cloud_map"]
    );
    // The kept placement is a live one → not flagged.
    assert!(l.placements.iter().all(|p| !p.dead));
}

// ── 8. Dead-AND-silent reads as DEAD (the more actionable cause) ──────────────

#[test]
fn a_dead_and_silent_automagic_topic_reads_as_dead_not_silent() {
    // A topic that is BOTH dead (Some(0)) AND still-silent (archetype None). The dead
    // check precedes the archetype check, so it is reported as DEAD, not as a silent
    // "re-run once it publishes" hint (and it does not fall to NoRenderableTopics).
    let input = bare(vec![AttachedRender {
        topic: "/uslam/cloud_map".to_string(),
        entity: DEAD_MAP.to_string(),
        archetype: None,
        producer_count: Some(0),
        video_renditions: Vec::new(),
        representation: Representation::Auto,
        render_proof: RenderProof::default(),
    }]);
    let err = compose_layout(&input).expect_err("the only topic is a dead route → refuse");
    assert_eq!(
        err,
        ComposeError::AllTopicsDead {
            dead: vec!["/uslam/cloud_map".to_string()],
        }
    );
}

// ── 9. A live topic alongside a dead+silent one: dead reported, live placed ────

#[test]
fn dead_and_silent_alongside_a_live_topic_is_reported_dead_not_silent() {
    let l = compose_layout(&bare(vec![
        live(
            "/utlidar/cloud_deskewed",
            LIVE_CLOUD,
            ArchetypeKind::Points3D,
        ),
        AttachedRender {
            topic: "/uslam/cloud_map".to_string(),
            entity: DEAD_MAP.to_string(),
            archetype: None,
            producer_count: Some(0),
            video_renditions: Vec::new(),
            representation: Representation::Auto,
            render_proof: RenderProof::default(),
        },
    ]))
    .expect("the live topic composes");

    assert_eq!(
        placed_topics(&l),
        vec!["/utlidar/cloud_deskewed".to_string()]
    );
    // Reported as DEAD (skipped_dead), NOT as a silent hint in warnings.
    assert_eq!(l.skipped_dead.len(), 1);
    assert_eq!(l.skipped_dead[0].topic, "/uslam/cloud_map");
    assert!(
        !l.warnings.iter().any(|w| w.contains("still silent")),
        "a dead+silent topic is reported dead, not silent"
    );
}

// ── 8b. MIXED dead + live-but-still-SILENT: names BOTH, never the universal claim ─

#[test]
fn mixed_dead_and_live_but_silent_names_both_never_the_universal_dead_claim() {
    // The review-MEDIUM case: an automagic compose over a DEAD route (Some(0)) AND a
    // topic with LIVE producers (Some(3)) that is NOT YET typed — the bounded resolve
    // peek has not caught its first frame, so its archetype is still None (producer
    // count and typing are INDEPENDENT). The dead route is skipped; the live-but-silent
    // topic falls to `silent`. Nothing places → the error must name the dead topic AS
    // dead and the live one AS silent-with-hint — NEVER the false universal "every topic
    // is a dead route" claim (the live topic has 3 publishers).
    let err = compose_layout(&bare(vec![
        dead("/uslam/cloud_map", DEAD_MAP, ArchetypeKind::Points3D),
        AttachedRender {
            topic: "/fresh".to_string(),
            entity: "world/fresh".to_string(),
            archetype: None,         // not yet typed (first frame not caught)
            producer_count: Some(3), // but 3 LIVE publishers — NOT a dead route
            video_renditions: Vec::new(),
            representation: Representation::Auto,
            render_proof: RenderProof::default(),
        },
    ]))
    .expect_err("dead + live-but-silent → refuse");

    assert_eq!(
        err,
        ComposeError::DeadAndSilentTopics {
            dead: vec!["/uslam/cloud_map".to_string()],
            silent: vec!["/fresh".to_string()],
        }
    );
    let msg = err.to_string();
    // The dead topic is named AS dead; the fresh (live) topic AS silent — both accurately.
    assert!(
        msg.contains("/uslam/cloud_map"),
        "names the dead topic: {msg}"
    );
    assert!(msg.contains("/fresh"), "names the silent topic: {msg}");
    // NEVER the false universal-dead claim (the live topic is not a dead route).
    assert!(
        !msg.contains("every requested topic is a registered-but-DEAD route"),
        "must not claim every topic is dead when one is live-but-silent: {msg}"
    );
    // And it carries the silent "renders once it publishes" hint (never discarded).
    assert!(
        msg.contains("no data yet") && msg.contains("publish"),
        "carries the silent re-run/publish hint: {msg}"
    );
}

// ── 10. Determinism (two runs byte-identical) ─────────────────────────────────

#[test]
fn prefer_live_is_deterministic_across_runs() {
    let mk = || {
        compose_layout(&bare(vec![
            live(
                "/utlidar/cloud_deskewed",
                LIVE_CLOUD,
                ArchetypeKind::Points3D,
            ),
            dead("/uslam/cloud_map", DEAD_MAP, ArchetypeKind::Points3D),
            unknown("/tf", "world", ArchetypeKind::Transforms),
        ]))
        .expect("composes")
    };
    let a = mk();
    let b = mk();
    assert_eq!(a.placements, b.placements);
    assert_eq!(a.skipped_dead, b.skipped_dead);
    assert_eq!(a.warnings, b.warnings);
    assert_eq!(a.plan, b.plan);
}

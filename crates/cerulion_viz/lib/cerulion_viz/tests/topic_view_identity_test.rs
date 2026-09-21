// SPDX-License-Identifier: AGPL-3.0-only
//! A per-topic view must be TITLED by its topic and must show
//! ONLY that topic's data.
//!
//! Fixing the entity paths is not enough on its own. The layout
//! compiler derived two things from the entity path that both stayed broken:
//!
//! 1. **View TITLES came from the entity's LEAF segment.** With paths flattened
//!    the leaf of `world/api/audiohub/response` is `response` — and so is every
//!    other `/api/*/response` topic's, so all 15 views were still titled
//!    "response" in the viewport. "Check two topics, see one" would have survived
//!    in the view tabs.
//! 2. **View CONTENTS were a bare subtree glob.** Under the old flat scheme no
//!    topic entity was ever an ancestor of another, so `/<entity>/**` was safe.
//!    Topic-shaped paths make the canonical ROS `image_transport` family
//!    (`/camera/image_raw` alongside `/camera/image_raw/compressed`) an
//!    ancestor/descendant pair, and the parent's own view silently drew the
//!    child's data too.
//!
//! Oracles are HAND-WRITTEN expected `PlanView` lists — never derived from the
//! functions under test. The 3 `sportmodestate` topics are the corpus for the
//! title half (the reported example); the `image_transport` family is the
//! corpus for the contents half.

use cerulion_viz::blueprint::{
    compose_layout, default_layout, default_layout_excluding, AttachedRender, BlueprintPlan,
    ComposeGroup, ComposeInput, ComposeStrategy, PlanNode, PlanView, ViewKind,
};
use cerulion_viz::representation::Representation;
use cerulion_viz::sink::RenderProof;
use cerulion_viz::sink::{route_for_input, route_key_for_topic, ArchetypeKind};

/// The REAL entity a topic resolves to (so this test composes the entity
/// derivation with the layout compiler, exactly as the daemon does).
fn entity_of(topic: &str) -> String {
    route_for_input(&route_key_for_topic(topic, None)).entity
}

fn att(topic: &str, arch: ArchetypeKind) -> AttachedRender {
    AttachedRender {
        topic: topic.to_string(),
        entity: entity_of(topic),
        archetype: Some(arch),
        producer_count: None,
        video_renditions: Vec::new(),
        representation: Representation::Auto,
        render_proof: RenderProof::default(),
    }
}

fn silent(topic: &str) -> AttachedRender {
    AttachedRender {
        topic: topic.to_string(),
        entity: entity_of(topic),
        archetype: None,
        producer_count: None,
        video_renditions: Vec::new(),
        representation: Representation::Auto,
        render_proof: RenderProof::default(),
    }
}

fn leaf_views(plan: &BlueprintPlan) -> Vec<PlanView> {
    fn walk(node: &PlanNode, out: &mut Vec<PlanView>) {
        match node {
            PlanNode::View(v) => out.push(v.clone()),
            PlanNode::Container(c) => c.children.iter().for_each(|ch| walk(ch, out)),
        }
    }
    let mut out = Vec::new();
    walk(&plan.root, &mut out);
    out
}

/// The per-topic sidebar views (everything that is not the world-rooted Scene —
/// whose origin is the `WORLD_ORIGIN` literal `/world`, leading slash included).
fn sidebar_views(plan: &BlueprintPlan) -> Vec<PlanView> {
    leaf_views(plan)
        .into_iter()
        .filter(|v| {
            v.origin
                .as_deref()
                .map(|o| o.trim_matches('/') != "world")
                .unwrap_or(true)
        })
        .collect()
}

// ── 1. titles: the three reported sportmodestate topics ─────────────────────

/// THE headline. `/lf/sportmodestate`, `/mf/sportmodestate` and `/sportmodestate`
/// are three genuinely different signals (low-frequency, medium-frequency, raw).
/// Before the entity-path fix they shared ONE entity; with that fixed but titles still taken from
/// the entity leaf, they would share ONE title (`sportmodestate`) in three
/// separate view tabs — indistinguishable in the viewport.
#[test]
fn the_three_sportmodestate_topics_get_three_distinctly_titled_views() {
    let plan = default_layout(&[
        att("/lf/sportmodestate", ArchetypeKind::SportModeState),
        att("/mf/sportmodestate", ArchetypeKind::SportModeState),
        att("/sportmodestate", ArchetypeKind::SportModeState),
    ]);
    // Hand oracle: three time_series views, one per topic, each titled by its
    // TOPIC and rooted at its own entity with only its own contents.
    let expected: Vec<PlanView> = vec![
        PlanView {
            kind: ViewKind::TimeSeries,
            name: Some("/lf/sportmodestate".to_string()),
            origin: Some("world/lf/sportmodestate".to_string()),
            contents: Some(vec!["/world/lf/sportmodestate/**".to_string()]),
        },
        PlanView {
            kind: ViewKind::TimeSeries,
            name: Some("/mf/sportmodestate".to_string()),
            origin: Some("world/mf/sportmodestate".to_string()),
            contents: Some(vec!["/world/mf/sportmodestate/**".to_string()]),
        },
        PlanView {
            kind: ViewKind::TimeSeries,
            name: Some("/sportmodestate".to_string()),
            origin: Some("world/sportmodestate".to_string()),
            contents: Some(vec!["/world/sportmodestate/**".to_string()]),
        },
    ];
    assert_eq!(sidebar_views(&plan), expected);
    // The property that matters to the user: three DISTINCT titles.
    let titles: std::collections::BTreeSet<String> = sidebar_views(&plan)
        .into_iter()
        .filter_map(|v| v.name)
        .collect();
    assert_eq!(titles.len(), 3, "three topics ⇒ three distinguishable tabs");
}

/// The 15-way case at full width: `/api/*/response` topics whose entity leaf is
/// `response` for every single one.
#[test]
fn fifteen_api_response_topics_get_fifteen_distinct_titles() {
    let services = [
        "audiohub",
        "bashrunner",
        "config",
        "fourg_agent",
        "gas_sensor",
        "gpt",
        "motion_switcher",
        "obstacles_avoid",
        "programming_actuator",
        "robot_state",
        "sport",
        "sport_lease",
        "uwbswitch",
        "videohub",
        "vui",
    ];
    let attached: Vec<AttachedRender> = services
        .iter()
        .map(|s| att(&format!("/api/{s}/response"), ArchetypeKind::TextLog))
        .collect();
    let views = sidebar_views(&default_layout(&attached));
    assert_eq!(views.len(), 15, "one view per topic");
    let titles: Vec<String> = views.iter().map(|v| v.name.clone().unwrap()).collect();
    let expected: Vec<String> = services
        .iter()
        .map(|s| format!("/api/{s}/response"))
        .collect();
    assert_eq!(titles, expected, "each view is titled by its own topic");
    assert_eq!(
        titles
            .iter()
            .collect::<std::collections::BTreeSet<_>>()
            .len(),
        15,
        "before the entity-path fix all fifteen read `response`"
    );
}

// ── 2. contents: the image_transport ancestor/descendant family ─────────────

/// The canonical ROS `image_transport` family. `/camera/image_raw`'s view must NOT
/// draw `/camera/image_raw/compressed` or `/theora` — those are separate topics
/// the user did not check.
#[test]
fn a_parent_topics_view_excludes_its_child_topics() {
    let plan = default_layout(&[
        att("/camera/image_raw", ArchetypeKind::Image),
        att("/camera/image_raw/compressed", ArchetypeKind::Image),
        att("/camera/image_raw/theora", ArchetypeKind::Image),
    ]);
    let views = sidebar_views(&plan);
    // Hand oracle for the PARENT view: its own subtree, minus each child topic.
    let parent = views
        .iter()
        .find(|v| v.origin.as_deref() == Some("world/camera/image_raw"))
        .expect("the parent topic has its own view");
    assert_eq!(
        parent.contents,
        Some(vec![
            "/world/camera/image_raw/**".to_string(),
            "-/world/camera/image_raw/compressed/**".to_string(),
            "-/world/camera/image_raw/theora/**".to_string(),
        ]),
        "the parent view includes its subtree MINUS every attached child topic"
    );
    // Each CHILD's view is its own subtree only — nothing to exclude.
    for child in ["compressed", "theora"] {
        let e = format!("world/camera/image_raw/{child}");
        let v = views
            .iter()
            .find(|v| v.origin.as_deref() == Some(e.as_str()))
            .unwrap_or_else(|| panic!("{child} has its own view"));
        assert_eq!(v.contents, Some(vec![format!("/{e}/**")]));
        assert_eq!(
            v.name.as_deref(),
            Some(&*format!("/camera/image_raw/{child}"))
        );
    }
}

/// The exclusion must be SUBTRACTIVE, not an exact-path expression: an archetype's
/// OWN sub-entities have to stay in. A cloud's geometry lives at
/// `<entity>/viz-sweep/{k}` and a path's waypoints at `<entity>/viz-vertices`, so an
/// exact-path view would render EMPTY — a worse bug than the one being fixed.
#[test]
fn an_archetypes_own_sub_entities_are_never_excluded() {
    let plan = default_layout(&[
        att("/utlidar/cloud", ArchetypeKind::Points3D),
        att("/plan", ArchetypeKind::Path3D),
        att("/gnss", ArchetypeKind::TextLog),
    ]);
    // The two spatial topics fold into the Scene hero; its contents are plain
    // subtree globs (a cloud's `sweep/k` and a path's `vertices` ride them).
    let hero = leaf_views(&plan)
        .into_iter()
        .find(|v| v.kind == ViewKind::Spatial3d)
        .expect("a Scene hero exists");
    let contents = hero.contents.expect("the hero grounds explicit contents");
    assert!(contents.contains(&"/world/utlidar/cloud/**".to_string()));
    assert!(contents.contains(&"/world/plan/**".to_string()));
    assert!(
        contents.iter().all(|c| !c.starts_with('-')),
        "no exclusions: neither spatial topic is below the other ({contents:?})"
    );
    // And no glob is an exact path — every one keeps its `/**` subtree suffix, so
    // sub-entities are included by construction.
    for c in &contents {
        assert!(c.ends_with("/**"), "{c} must be a subtree glob");
    }
}

/// A STILL-SILENT child topic gets no view of its own, but its data must not be
/// swept into its parent's view either (it has an entity the moment it attaches).
#[test]
fn a_silent_child_topic_is_still_excluded_from_its_parent() {
    let plan = default_layout(&[
        att("/camera/image_raw", ArchetypeKind::Image),
        silent("/camera/image_raw/compressed"),
    ]);
    let views = sidebar_views(&plan);
    // The silent child gets NO view of its own. The parent gets two — its
    // spatial2d image pane and the status pane its render arm degrades
    // into for an encoding this build cannot decode — and the exclusion must hold
    // in BOTH, or the child's data would be swept into one of them.
    assert_eq!(
        views.iter().map(|v| v.kind).collect::<Vec<_>>(),
        vec![ViewKind::Spatial2d, ViewKind::TextDocument],
        "the silent child gets no view; the parent gets its image + dump panes"
    );
    for view in &views {
        assert_eq!(
            view.contents,
            Some(vec![
                "/world/camera/image_raw/**".to_string(),
                "-/world/camera/image_raw/compressed/**".to_string(),
            ]),
            "…but the silent child is still excluded from EVERY parent view ({:?})",
            view.kind
        );
    }
}

/// The ANTI-TAUTOLOGY control: an unrelated sibling topic is NOT excluded. Without
/// this, an implementation that excluded everything would pass the arms above.
#[test]
fn unrelated_topics_are_not_excluded() {
    let plan = default_layout(&[
        att("/camera/image_raw", ArchetypeKind::Image),
        att("/camera/image_rect", ArchetypeKind::Image),
        // A prefix-but-not-a-path-descendant name (the `/` boundary matters).
        att("/camera/image_rawX", ArchetypeKind::Image),
    ]);
    for v in sidebar_views(&plan) {
        assert_eq!(
            v.contents.as_ref().map(|c| c.len()),
            Some(1),
            "no topic here is below another, so nothing is excluded: {v:?}"
        );
    }
}

// ── 3. the compose (agent/explicit) path keeps the same identity rules ──────

/// `compose_layout`'s per-camera `spatial2d` views were ALSO titled by the entity
/// leaf, which is `image_raw` for both cameras of the canonical two-camera robot.
#[test]
fn two_cameras_sharing_a_leaf_get_distinct_pane_titles_and_scoped_contents() {
    let composed = compose_layout(&ComposeInput {
        groups: vec![ComposeGroup {
            topics: vec![
                att("/cam1/image_raw", ArchetypeKind::Image),
                att("/cam2/image_raw", ArchetypeKind::Image),
            ],
            role: None,
            title: None,
        }],
        strategy: ComposeStrategy::Grid,
        include_robot: false,
    })
    .expect("two cameras compose");
    let names: Vec<String> = leaf_views(&composed.plan)
        .into_iter()
        .filter(|v| v.kind == ViewKind::Spatial2d)
        .map(|v| v.name.unwrap())
        .collect();
    assert_eq!(
        names,
        vec![
            "Images: /cam1/image_raw".to_string(),
            "Images: /cam2/image_raw".to_string()
        ],
        "each pane names its own TOPIC (the entity leaf is `image_raw` for both)"
    );
}

/// …and the compose path's per-entity image views exclude a descendant topic too.
#[test]
fn a_composed_image_view_excludes_a_descendant_topic() {
    let composed = compose_layout(&ComposeInput {
        groups: vec![ComposeGroup {
            topics: vec![
                att("/camera/image_raw", ArchetypeKind::Image),
                att("/camera/image_raw/compressed", ArchetypeKind::Image),
            ],
            role: None,
            title: None,
        }],
        strategy: ComposeStrategy::Grid,
        include_robot: false,
    })
    .expect("the family composes");
    let parent = leaf_views(&composed.plan)
        .into_iter()
        .find(|v| v.origin.as_deref() == Some("world/camera/image_raw"))
        .expect("the parent camera has a pane");
    assert_eq!(
        parent.contents,
        Some(vec![
            "/world/camera/image_raw/**".to_string(),
            "-/world/camera/image_raw/compressed/**".to_string(),
        ])
    );
}

/// **THE DETACH ZOMBIE.**
///
/// The exclusions were computed from the CURRENTLY-attached set, and nothing in
/// vizd clears a detached topic's data from the rerun store (the uncheck tombstone
/// belongs to the Studio shell, and there are shell-free detach paths: the CLI
/// detaches only the taps IT opened, so `cerulion viz --detach /camera/image_raw`
/// then `cerulion viz /camera/image_raw/compressed` + Ctrl+C detaches the
/// DESCENDANT only). Recomposing then DROPPED the exclusion while the descendant's
/// last frame was still in the store — so it drew inside the still-attached
/// ancestor's pane, under a different topic's title. "Check two topics, see one",
/// re-entered through an UNCHECK: the same class as two earlier fixes.
#[test]
fn detaching_a_descendant_does_not_re_admit_it_into_its_ancestors_view() {
    let parent = entity_of("/camera/image_raw");
    let child = entity_of("/camera/image_raw/compressed");

    // BOTH attached: the child is excluded from the parent's view (the pre-existing
    // contract, restated here so the after-detach assert has a baseline).
    let both = default_layout_excluding(
        &[
            att("/camera/image_raw", ArchetypeKind::Image),
            att("/camera/image_raw/compressed", ArchetypeKind::Image),
        ],
        &[],
    );
    let parent_view = sidebar_views(&both)
        .into_iter()
        .find(|v| v.origin.as_deref() == Some(parent.as_str()))
        .expect("the parent has a view");
    assert_eq!(
        parent_view.contents,
        Some(vec![format!("/{parent}/**"), format!("-/{child}/**"),])
    );

    // The CHILD detaches. Its data is still in the store, so the exclusion must
    // SURVIVE — the ever-logged set is what carries it.
    let after = default_layout_excluding(
        &[att("/camera/image_raw", ArchetypeKind::Image)],
        std::slice::from_ref(&child),
    );
    let views = sidebar_views(&after);
    // Only the ancestor keeps views — its image pane and its dump pane.
    assert_eq!(
        views.iter().map(|v| v.kind).collect::<Vec<_>>(),
        vec![ViewKind::Spatial2d, ViewKind::TextDocument],
        "only the ancestor keeps views (image + the companion dump pane)"
    );
    for view in &views {
        assert_eq!(
            view.contents,
            Some(vec![format!("/{parent}/**"), format!("-/{child}/**"),]),
            "a detached descendant must not be re-admitted into ANY ancestor view ({:?})",
            view.kind
        );
    }
}

/// The Scene half of the same class: this module's own contract says "a DETACHED
/// spatial topic drops out of the union — it 'goes away' from the Scene on the next
/// reflow", which was false whenever an ANCESTOR spatial topic stayed attached (the
/// ancestor's subtree glob kept drawing it).
#[test]
fn a_detached_spatial_descendant_leaves_the_scene_even_under_an_attached_ancestor() {
    let parent = entity_of("/sensors/cloud");
    let child = entity_of("/sensors/cloud/filtered");

    let after = default_layout_excluding(
        &[att("/sensors/cloud", ArchetypeKind::Points3D)],
        std::slice::from_ref(&child),
    );
    let hero = leaf_views(&after)
        .into_iter()
        .find(|v| v.kind == ViewKind::Spatial3d)
        .expect("the Scene hero");
    let contents = hero.contents.expect("explicit hero contents");
    assert!(
        contents.contains(&format!("/{parent}/**")),
        "the attached ancestor is still drawn: {contents:?}"
    );
    assert!(
        contents.contains(&format!("-/{child}/**")),
        "the detached descendant is excluded from the Scene: {contents:?}"
    );

    // ANTI-TAUTOLOGY: while it is still ATTACHED it is INCLUDED, not excluded.
    let both = default_layout_excluding(
        &[
            att("/sensors/cloud", ArchetypeKind::Points3D),
            att("/sensors/cloud/filtered", ArchetypeKind::Points3D),
        ],
        &[],
    );
    let hero = leaf_views(&both)
        .into_iter()
        .find(|v| v.kind == ViewKind::Spatial3d)
        .expect("the Scene hero");
    let contents = hero.contents.expect("explicit hero contents");
    assert!(contents.contains(&format!("/{child}/**")), "{contents:?}");
    assert!(
        !contents.iter().any(|c| c == &format!("-/{child}/**")),
        "an ATTACHED descendant must not be excluded: {contents:?}"
    );
}

/// The ever-logged set is ADDITIVE and cannot hide a view's own subtree: an entry
/// naming the view's OWN entity (a re-attach of a topic detached earlier) excludes
/// nothing, because only STRICT descendants are excluded.
#[test]
fn an_ever_logged_entry_for_the_views_own_entity_excludes_nothing() {
    let parent = entity_of("/camera/image_raw");
    let plan = default_layout_excluding(
        &[att("/camera/image_raw", ArchetypeKind::Image)],
        &[parent.clone(), "world/unrelated".to_string()],
    );
    let views = sidebar_views(&plan);
    assert_eq!(views[0].contents, Some(vec![format!("/{parent}/**")]));
}

// SPDX-License-Identifier: AGPL-3.0-only
//! The DETERMINISTIC layout COMPILER, pinned against HAND oracles
//! (never a self-compare against the function under test).
//!
//! [`cerulion_viz::blueprint::compose_layout`] turns SEMANTIC INTENT (a set of
//! topics, or role-tagged groups) into an emit-ready `BlueprintPlan` + a placement
//! echo. These tests pin EXACT placements, `with_contents` globs, view kinds, and
//! arrangement shares over a Go2-like corpus (cloud + scan + tf + odom + cmd_vel +
//! image) for every strategy, plus the Odom/Imu dual-view, role-hint
//! incompatibility, the include_robot skeleton placement, the zero-topic refusal,
//! and determinism.
//!
//! The topic → entity mapping itself is pinned by `layout_mapping_test.rs`; the
//! entities below are re-derived HAND LITERALS (a change is caught, not followed).

use cerulion_viz::blueprint::{
    blueprint_decorations, blueprint_property_paths, build_blueprint_msgs, compose_layout,
    Arrangement, AttachedRender, BlueprintPlan, ComposeError, ComposeGroup, ComposeInput,
    ComposeStrategy, ComposedLayout, ComposedPlacement, ContainerKind, GroupRole, PlanNode,
    PlanView, ViewKind,
};
use cerulion_viz::representation::Representation;
use cerulion_viz::sink::ArchetypeKind;
use cerulion_viz::sink::RenderProof;

// ── The Go2-like corpus (hand entity literals; archetypes follow the decided table) ──

const LIDAR: &str = "world/utlidar/cloud";
const SCAN: &str = "world/scan";
const TF: &str = "world";
const ODOM: &str = "world/tf-tree/odom";
const CMD_VEL: &str = "world/cmd_vel";
const CAMERA: &str = "world/camera/image";
const ROBOT_GLOB: &str = "/world/tf-tree/robot/**";

fn att(topic: &str, entity: &str, arch: ArchetypeKind) -> AttachedRender {
    AttachedRender {
        topic: topic.to_string(),
        entity: entity.to_string(),
        archetype: Some(arch),
        producer_count: None,
        video_renditions: Vec::new(),
        representation: Representation::Auto,
        render_proof: RenderProof::default(),
    }
}

fn silent(topic: &str, entity: &str) -> AttachedRender {
    AttachedRender {
        topic: topic.to_string(),
        entity: entity.to_string(),
        archetype: None,
        producer_count: None,
        video_renditions: Vec::new(),
        representation: Representation::Auto,
        render_proof: RenderProof::default(),
    }
}

/// The 6-topic Go2-like corpus, in a fixed order.
fn go2_corpus() -> Vec<AttachedRender> {
    vec![
        att("/utlidar/cloud", LIDAR, ArchetypeKind::Points3D),
        att("/scan", SCAN, ArchetypeKind::LaserScan),
        att("/tf", TF, ArchetypeKind::Transforms),
        att("/odom", ODOM, ArchetypeKind::Odometry),
        att("/cmd_vel", CMD_VEL, ArchetypeKind::Scalars),
        att("/camera/image", CAMERA, ArchetypeKind::Image),
    ]
}

/// Bare-topics automagic input (ONE role-less group).
fn bare(
    topics: Vec<AttachedRender>,
    strategy: ComposeStrategy,
    include_robot: bool,
) -> ComposeInput {
    ComposeInput {
        groups: vec![ComposeGroup {
            topics,
            role: None,
            title: None,
        }],
        strategy,
        include_robot,
    }
}

/// A `glob` for an entity — the compiler's `/<entity>/**` leading-slash form.
fn glob(entity: &str) -> String {
    format!("/{entity}/**")
}

/// (topic, view-kind-wire) for every placement, in order.
fn placement_pairs(l: &ComposedLayout) -> Vec<(String, String)> {
    l.placements
        .iter()
        .map(|p| (p.topic.clone(), p.view.as_str().to_string()))
        .collect()
}

/// Collect every leaf view in the plan (declaration order).
fn leaf_views(l: &ComposedLayout) -> Vec<PlanView> {
    fn walk(node: &PlanNode, out: &mut Vec<PlanView>) {
        match node {
            PlanNode::View(v) => out.push(v.clone()),
            PlanNode::Container(c) => c.children.iter().for_each(|ch| walk(ch, out)),
        }
    }
    let mut out = Vec::new();
    walk(&l.plan.root, &mut out);
    out
}

/// The first leaf view of `kind`.
fn view_of(l: &ComposedLayout, kind: ViewKind) -> PlanView {
    leaf_views(l)
        .into_iter()
        .find(|v| v.kind == kind)
        .unwrap_or_else(|| panic!("no {kind:?} view in the plan"))
}

// ── 1. Bare-topics automagic over the Go2 corpus (the headline) ───────────────

#[test]
fn bare_topics_auto_places_the_go2_corpus_into_the_hero_sidebar_shape() {
    let l = compose_layout(&bare(go2_corpus(), ComposeStrategy::Auto, true))
        .expect("the go2 corpus composes");

    // (a) Placements: each topic in EVERY view its archetype renders in, in corpus
    //     order; Odometry is dual (spatial3d AND time_series). Hand oracle.
    //     The camera is dual too — its render arm degrades to the field
    //     dump for an encoding this build does not decode, so it is also placed in
    //     the status panel where that dump is displayable.
    assert_eq!(
        placement_pairs(&l),
        vec![
            ("/utlidar/cloud".to_string(), "spatial3d".to_string()),
            ("/scan".to_string(), "spatial3d".to_string()),
            ("/tf".to_string(), "spatial3d".to_string()),
            ("/odom".to_string(), "spatial3d".to_string()),
            ("/odom".to_string(), "time_series".to_string()),
            ("/cmd_vel".to_string(), "time_series".to_string()),
            ("/camera/image".to_string(), "spatial2d".to_string()),
            ("/camera/image".to_string(), "text_document".to_string()),
        ],
        "one placement per (topic, view); odom and the camera are dual-view"
    );

    // (b) auto_views is FALSE (the Twist-6-panel-explosion fix).
    assert!(!l.plan.auto_views, "auto_views must be OFF");

    // (c) Arrangement: HeroSidebar (a 3D scene exists → auto picks the split).
    assert_eq!(l.arrangement, Arrangement::HeroSidebar);
    assert_eq!(l.arrangement.as_str(), "hero_sidebar");

    // (d) The plan tree: Horizontal[3.0 : 1.5]( hero | Vertical(plots, images,
    //     status)) — the status row is the dump pane the camera earns.
    let root = match &l.plan.root {
        PlanNode::Container(c) => c,
        other => panic!("expected a horizontal root, got {other:?}"),
    };
    assert_eq!(root.kind, ContainerKind::Horizontal);
    assert_eq!(
        root.shares,
        Some(vec![3.0, 1.5]),
        "hero:sidebar column shares"
    );
    assert_eq!(root.children.len(), 2);
    // Left child: the hero view.
    let hero = match &root.children[0] {
        PlanNode::View(v) => v.clone(),
        other => panic!("hero child is a view, got {other:?}"),
    };
    assert_eq!(hero.kind, ViewKind::Spatial3d);
    assert_eq!(hero.name.as_deref(), Some("3D Scene"));
    assert_eq!(
        hero.origin.as_deref(),
        Some("/world"),
        "the hero roots at the world origin (the 3D coordinate frame)"
    );
    // The hero grounds EXACTLY the 3D topic subtrees + the robot skeleton.
    assert_eq!(
        hero.contents,
        Some(vec![
            glob(LIDAR),
            glob(SCAN),
            glob(TF),
            glob(ODOM),
            ROBOT_GLOB.to_string(),
        ]),
        "hero contents = the 3D globs + the robot skeleton"
    );
    // Right child: the sidebar Vertical(plots, images).
    let sidebar = match &root.children[1] {
        PlanNode::Container(c) => c,
        other => panic!("sidebar is a vertical container, got {other:?}"),
    };
    assert_eq!(sidebar.kind, ContainerKind::Vertical);
    assert_eq!(
        sidebar.shares, None,
        "the sidebar rows are equal (no explicit shares)"
    );
    let plots = match &sidebar.children[0] {
        PlanNode::View(v) => v.clone(),
        other => panic!("plots child, got {other:?}"),
    };
    assert_eq!(plots.kind, ViewKind::TimeSeries);
    assert_eq!(plots.name.as_deref(), Some("Plots"));
    assert_eq!(
        plots.origin.as_deref(),
        Some(ODOM),
        "a sidebar view roots at its FIRST topic entity (precise grounding)"
    );
    assert_eq!(
        plots.contents,
        Some(vec![glob(ODOM), glob(CMD_VEL)]),
        "plots grounds odom (twist series) + cmd_vel — ONE plot per topic"
    );
    let images = match &sidebar.children[1] {
        PlanNode::View(v) => v.clone(),
        other => panic!("images child, got {other:?}"),
    };
    assert_eq!(images.kind, ViewKind::Spatial2d);
    assert_eq!(images.name.as_deref(), Some("Images"));
    assert_eq!(images.origin.as_deref(), Some(CAMERA));
    assert_eq!(images.contents, Some(vec![glob(CAMERA)]));
    // The status row exists because the camera's render path can degrade
    // to the field dump, and it grounds exactly that topic — so when a raw frame
    // arrives in an encoding this build does not decode, the dump is VISIBLE
    // instead of landing in an entity no view displays.
    let status = match &sidebar.children[2] {
        PlanNode::View(v) => v.clone(),
        other => panic!("status child, got {other:?}"),
    };
    assert_eq!(status.kind, ViewKind::TextDocument);
    assert_eq!(status.origin.as_deref(), Some(CAMERA));
    assert_eq!(status.contents, Some(vec![glob(CAMERA)]));

    // (e) view_count = 4 (hero + plots + images + the status pane).
    assert_eq!(l.plan.view_count(), 4);
    assert!(l.warnings.is_empty(), "all topics resolved → no warnings");
}

// ── 2. focus_3d forces the hero-sidebar shape; grid lays an even grid ─────────

#[test]
fn focus_3d_forces_hero_sidebar_and_grid_lays_an_even_grid() {
    let focus = compose_layout(&bare(go2_corpus(), ComposeStrategy::Focus3d, true))
        .expect("focus_3d composes");
    assert_eq!(focus.arrangement, Arrangement::HeroSidebar);
    // focus_3d yields the SAME hero-sidebar tree as auto (over a 3D-having corpus).
    let auto =
        compose_layout(&bare(go2_corpus(), ComposeStrategy::Auto, true)).expect("auto composes");
    assert_eq!(focus.plan, auto.plan, "focus_3d == auto when a hero exists");

    let grid =
        compose_layout(&bare(go2_corpus(), ComposeStrategy::Grid, true)).expect("grid composes");
    assert_eq!(grid.arrangement, Arrangement::Grid);
    let root = match &grid.plan.root {
        PlanNode::Container(c) => c,
        other => panic!("grid root, got {other:?}"),
    };
    assert_eq!(root.kind, ContainerKind::Grid);
    // 4 views (hero, plots, images, status) → ceil(sqrt(4)) = 2 columns,
    // flat (no nesting).
    assert_eq!(root.columns, Some(2));
    assert_eq!(
        root.children.len(),
        4,
        "hero + plots + images + status, flat"
    );
    assert!(
        root.children.iter().all(|c| matches!(c, PlanNode::View(_))),
        "a grid is flat — every child is a view"
    );
    // The SAME four views + placements regardless of arrangement (WHERE not WHAT).
    assert_eq!(placement_pairs(&grid), placement_pairs(&auto));
    assert_eq!(grid.plan.view_count(), 4);
}

// ── 3. Odometry / Imu dual-view placement ─────────────────────────────────────

#[test]
fn odometry_and_imu_land_in_both_the_hero_and_plots_views() {
    for arch in [ArchetypeKind::Odometry, ArchetypeKind::Imu] {
        let l = compose_layout(&bare(
            vec![att("/state", "world/state", arch)],
            ComposeStrategy::Auto,
            false, // isolate: no robot glob confounding the hero contents
        ))
        .expect("dual-view topic composes");
        assert_eq!(
            placement_pairs(&l),
            vec![
                ("/state".to_string(), "spatial3d".to_string()),
                ("/state".to_string(), "time_series".to_string()),
            ],
            "{arch:?} places into BOTH the 3D scene and the plot"
        );
        // Both views ground the SAME entity subtree.
        assert_eq!(
            view_of(&l, ViewKind::Spatial3d).contents,
            Some(vec![glob("world/state")])
        );
        assert_eq!(
            view_of(&l, ViewKind::TimeSeries).contents,
            Some(vec![glob("world/state")])
        );
    }
}

// ── 4. include_robot rides the skeleton in the hero (and its absence) ─────────

#[test]
fn include_robot_adds_the_skeleton_glob_to_the_hero_only() {
    let with = compose_layout(&bare(
        vec![att("/utlidar/cloud", LIDAR, ArchetypeKind::Points3D)],
        ComposeStrategy::Auto,
        true,
    ))
    .expect("composes");
    assert_eq!(
        view_of(&with, ViewKind::Spatial3d).contents,
        Some(vec![glob(LIDAR), ROBOT_GLOB.to_string()]),
        "the robot skeleton rides in the hero contents"
    );

    let without = compose_layout(&bare(
        vec![att("/utlidar/cloud", LIDAR, ArchetypeKind::Points3D)],
        ComposeStrategy::Auto,
        false,
    ))
    .expect("composes");
    assert_eq!(
        view_of(&without, ViewKind::Spatial3d).contents,
        Some(vec![glob(LIDAR)]),
        "no robot glob when include_robot is off"
    );
}

#[test]
fn include_robot_forces_a_hero_even_with_no_3d_topics() {
    // A plots-only corpus (cmd_vel) + include_robot → a hero 3D view exists,
    // grounded by ONLY the robot skeleton, beside the plots sidebar.
    let l = compose_layout(&bare(
        vec![att("/cmd_vel", CMD_VEL, ArchetypeKind::Scalars)],
        ComposeStrategy::Auto,
        true,
    ))
    .expect("composes");
    assert_eq!(l.arrangement, Arrangement::HeroSidebar);
    assert_eq!(
        view_of(&l, ViewKind::Spatial3d).contents,
        Some(vec![ROBOT_GLOB.to_string()]),
        "the hero shows only the robot (no 3D topic)"
    );
    // But cmd_vel is the only PLACEMENT (the robot is not a placement).
    assert_eq!(
        placement_pairs(&l),
        vec![("/cmd_vel".to_string(), "time_series".to_string())]
    );
}

// ── 5. No-hero auto → grid; single view → Single ──────────────────────────────

#[test]
fn no_3d_content_without_robot_auto_picks_a_grid() {
    // cmd_vel (plots) + image (images + the dump pane), no hero, no robot
    // → auto → grid of 3.
    let l = compose_layout(&bare(
        vec![
            att("/cmd_vel", CMD_VEL, ArchetypeKind::Scalars),
            att("/camera/image", CAMERA, ArchetypeKind::Image),
        ],
        ComposeStrategy::Auto,
        false,
    ))
    .expect("composes");
    assert_eq!(l.arrangement, Arrangement::Grid);
    let root = match &l.plan.root {
        PlanNode::Container(c) => c,
        other => panic!("grid root, got {other:?}"),
    };
    assert_eq!(root.kind, ContainerKind::Grid);
    assert_eq!(root.columns, Some(2), "3 views → ceil(sqrt(3)) = 2 columns");
    assert_eq!(root.children.len(), 3);
}

#[test]
fn a_single_bucket_produces_a_single_view_root() {
    let l = compose_layout(&bare(
        vec![att("/utlidar/cloud", LIDAR, ArchetypeKind::Points3D)],
        ComposeStrategy::Auto,
        false,
    ))
    .expect("composes");
    assert_eq!(l.arrangement, Arrangement::Single);
    assert!(
        matches!(&l.plan.root, PlanNode::View(v) if v.kind == ViewKind::Spatial3d),
        "a lone 3D scene is a bare view root: {:?}",
        l.plan.root
    );
    assert_eq!(l.plan.view_count(), 1);
}

// ── 6. A text topic fills the status panel (a 3-panel sidebar) ────────────────

#[test]
fn a_text_topic_fills_the_status_panel_making_a_three_panel_sidebar() {
    let l = compose_layout(&bare(
        vec![
            att("/utlidar/cloud", LIDAR, ArchetypeKind::Points3D),
            att("/cmd_vel", CMD_VEL, ArchetypeKind::Scalars),
            att("/camera/image", CAMERA, ArchetypeKind::Image),
            att("/rosout", "world/rosout", ArchetypeKind::TextLog),
        ],
        ComposeStrategy::Auto,
        false,
    ))
    .expect("composes");
    // hero + Vertical(plots, images, status).
    let root = match &l.plan.root {
        PlanNode::Container(c) => c,
        other => panic!("root, got {other:?}"),
    };
    let sidebar = match &root.children[1] {
        PlanNode::Container(c) => c,
        other => panic!("sidebar, got {other:?}"),
    };
    let kinds: Vec<ViewKind> = sidebar
        .children
        .iter()
        .map(|c| match c {
            PlanNode::View(v) => v.kind,
            _ => panic!("sidebar children are views"),
        })
        .collect();
    assert_eq!(
        kinds,
        vec![
            ViewKind::TimeSeries,
            ViewKind::Spatial2d,
            ViewKind::TextDocument
        ],
        "plots, images, status in that canonical order"
    );
    // The status panel grounds the text topic AND the camera, whose
    // render arm can degrade to the field dump — the whole point of the issue is
    // that such a dump has somewhere to render. Entity order follows the input
    // order, so the camera's glob precedes rosout's.
    assert_eq!(
        view_of(&l, ViewKind::TextDocument).contents,
        Some(vec![glob(CAMERA), glob("world/rosout")])
    );
    assert_eq!(l.plan.view_count(), 4);
}

// ── 7. Role groups: pinned single-bucket placement + titles ───────────────────

#[test]
fn role_groups_pin_each_topic_to_one_bucket_and_apply_titles() {
    // odom is Odometry (dual-view), but placed in a PLOTS role group it goes ONLY
    // to the plots bucket (the role is an explicit override, NOT dual-view).
    let input = ComposeInput {
        groups: vec![
            ComposeGroup {
                topics: vec![
                    att("/utlidar/cloud", LIDAR, ArchetypeKind::Points3D),
                    att("/tf", TF, ArchetypeKind::Transforms),
                ],
                role: Some(GroupRole::Hero),
                title: Some("Robot Scene".to_string()),
            },
            ComposeGroup {
                topics: vec![
                    att("/odom", ODOM, ArchetypeKind::Odometry),
                    att("/cmd_vel", CMD_VEL, ArchetypeKind::Scalars),
                ],
                role: Some(GroupRole::Plots),
                title: Some("Telemetry".to_string()),
            },
        ],
        strategy: ComposeStrategy::Auto,
        include_robot: false,
    };
    let l = compose_layout(&input).expect("role groups compose");
    // Each topic → exactly ONE placement (the role's target), NOT the dual-view.
    assert_eq!(
        placement_pairs(&l),
        vec![
            ("/utlidar/cloud".to_string(), "spatial3d".to_string()),
            ("/tf".to_string(), "spatial3d".to_string()),
            ("/odom".to_string(), "time_series".to_string()),
            ("/cmd_vel".to_string(), "time_series".to_string()),
        ],
        "odom is pinned to plots ONLY by its role (no hero placement)"
    );
    // Titles name each role bucket's view.
    assert_eq!(
        view_of(&l, ViewKind::Spatial3d).name.as_deref(),
        Some("Robot Scene")
    );
    assert_eq!(
        view_of(&l, ViewKind::TimeSeries).name.as_deref(),
        Some("Telemetry")
    );
    // The hero grounds only cloud + tf (odom did NOT enter the hero).
    assert_eq!(
        view_of(&l, ViewKind::Spatial3d).contents,
        Some(vec![glob(LIDAR), glob(TF)])
    );
}

#[test]
fn a_silent_topic_in_a_role_group_is_placed_by_role_with_a_soft_warning() {
    let input = ComposeInput {
        groups: vec![ComposeGroup {
            topics: vec![silent("/lidar_boot", "world/lidar_boot")],
            role: Some(GroupRole::Hero),
            title: None,
        }],
        strategy: ComposeStrategy::Auto,
        include_robot: false,
    };
    let l = compose_layout(&input).expect("silent-by-role composes");
    assert_eq!(
        placement_pairs(&l),
        vec![("/lidar_boot".to_string(), "spatial3d".to_string())],
        "a silent topic is placed by its declared role (lay-before-data)"
    );
    assert_eq!(l.placements[0].archetype, None);
    assert_eq!(l.warnings.len(), 1);
    assert!(
        l.warnings[0].contains("/lidar_boot") && l.warnings[0].contains("silent"),
        "the soft warning names the silent topic: {:?}",
        l.warnings
    );
}

// ── 8. Role incompatibility is a loud, actionable refusal ─────────────────────

#[test]
fn an_image_in_a_hero_role_group_is_refused_naming_its_real_view() {
    let input = ComposeInput {
        groups: vec![ComposeGroup {
            topics: vec![att("/camera/image", CAMERA, ArchetypeKind::Image)],
            role: Some(GroupRole::Hero),
            title: None,
        }],
        strategy: ComposeStrategy::Auto,
        include_robot: false,
    };
    let err = compose_layout(&input).expect_err("image in hero → refuse");
    assert_eq!(
        err,
        ComposeError::RoleIncompatible {
            topic: "/camera/image".to_string(),
            archetype: "Image".to_string(),
            role: "hero",
            role_view: "spatial3d",
            // The degradation status pane also renders an image, so the accurate "renders
            // only in …" list is two kinds. The refusal itself is unchanged —
            // an image is not 3D geometry in either of them.
            real_views: "spatial2d, text_document".to_string(),
        }
    );
    let msg = err.to_string();
    assert!(
        msg.contains("/camera/image")
            && msg.contains("spatial2d")
            && msg.contains("hero")
            && msg.contains("move it"),
        "the refusal names the topic, its real view, the role, and the fix: {msg}"
    );
}

#[test]
fn a_scalar_bag_in_an_images_role_group_is_refused() {
    // A Scalars topic (time_series only) in an `images` (spatial2d) group → refuse.
    let input = ComposeInput {
        groups: vec![ComposeGroup {
            topics: vec![att("/cmd_vel", CMD_VEL, ArchetypeKind::Scalars)],
            role: Some(GroupRole::Images),
            title: None,
        }],
        strategy: ComposeStrategy::Auto,
        include_robot: false,
    };
    let err = compose_layout(&input).expect_err("scalars in images → refuse");
    assert!(matches!(err, ComposeError::RoleIncompatible { .. }));
    assert!(
        err.to_string().contains("time_series"),
        "names the real view: {err}"
    );
}

#[test]
fn odometry_is_compatible_with_both_hero_and_plots_roles() {
    // A dual-view topic is compatible with EITHER of its two roles (not refused).
    for role in [GroupRole::Hero, GroupRole::Plots] {
        let input = ComposeInput {
            groups: vec![ComposeGroup {
                topics: vec![att("/odom", ODOM, ArchetypeKind::Odometry)],
                role: Some(role),
                title: None,
            }],
            strategy: ComposeStrategy::Auto,
            include_robot: false,
        };
        let l = compose_layout(&input).unwrap_or_else(|e| panic!("odom in {role:?}: {e}"));
        assert_eq!(l.placements.len(), 1, "one placement in the role's bucket");
        assert_eq!(l.placements[0].view, role.view_kind());
    }
}

// ── 9. Zero renderable topics → a loud refusal ────────────────────────────────

#[test]
fn compose_over_only_silent_topics_refuses_loudly() {
    let input = bare(
        vec![silent("/a", "world/a"), silent("/b", "world/b")],
        ComposeStrategy::Auto,
        true, // even with the robot, silent topics are not placements
    );
    let err = compose_layout(&input).expect_err("all-silent → refuse");
    match err {
        ComposeError::NoRenderableTopics { requested, silent } => {
            assert_eq!(requested, 2);
            assert_eq!(silent, vec!["/a".to_string(), "/b".to_string()]);
        }
        other => panic!("expected NoRenderableTopics, got {other:?}"),
    }
}

// ── 10. Determinism: same intent twice → byte-identical output ────────────────

#[test]
fn composing_the_same_intent_twice_is_byte_identical() {
    let mk = || compose_layout(&bare(go2_corpus(), ComposeStrategy::Auto, true)).unwrap();
    let a = mk();
    let b = mk();
    assert_eq!(a.plan, b.plan, "plans are byte-identical");
    assert_eq!(a.placements, b.placements, "placements are byte-identical");
    assert_eq!(a.arrangement, b.arrangement);
    assert_eq!(a.warnings, b.warnings);
    // And the placement echo carries the resolved archetype for a real topic.
    assert!(a.placements.iter().any(|p| p
        == &ComposedPlacement {
            topic: "/utlidar/cloud".to_string(),
            entity: LIDAR.to_string(),
            archetype: Some(ArchetypeKind::Points3D),
            view: ViewKind::Spatial3d,
            dead: false,
        }));
}

// ── 11. Multi-camera: each image gets its OWN spatial2d view ───────

const CAM_FRONT: &str = "world/camera/front";
const CAM_WRIST: &str = "world/camera/wrist";

#[test]
fn two_cameras_get_one_spatial2d_view_each_not_a_single_shared_view() {
    // A humanoid/manipulator with /camera/front + /camera/wrist (both Image). A
    // rerun spatial2d view anchors to ONE origin coordinate space, so unioning both
    // into one view overlays the feeds — each camera MUST get its own view.
    let l = compose_layout(&bare(
        vec![
            att("/camera/front", CAM_FRONT, ArchetypeKind::Image),
            att("/camera/wrist", CAM_WRIST, ArchetypeKind::Image),
        ],
        ComposeStrategy::Grid, // no hero → a flat grid of the two image views
        false,
    ))
    .expect("two cameras compose");

    // TWO spatial2d views, each rooted at + grounding ONLY its own camera entity.
    let images: Vec<PlanView> = leaf_views(&l)
        .into_iter()
        .filter(|v| v.kind == ViewKind::Spatial2d)
        .collect();
    assert_eq!(
        images.len(),
        2,
        "one spatial2d view per camera, not one shared"
    );

    // Hand oracle: view i is rooted at camera i, contents = ONLY camera i's glob.
    assert_eq!(images[0].origin.as_deref(), Some(CAM_FRONT));
    assert_eq!(images[0].contents, Some(vec![glob(CAM_FRONT)]));
    assert_eq!(images[1].origin.as_deref(), Some(CAM_WRIST));
    assert_eq!(images[1].contents, Some(vec![glob(CAM_WRIST)]));
    // NEITHER view's contents include the OTHER camera (the overlay bug).
    assert!(
        !images[0]
            .contents
            .as_ref()
            .unwrap()
            .contains(&glob(CAM_WRIST)),
        "the front view must NOT ground the wrist camera"
    );

    // Multiple → each name is suffixed with the TOPIC (distinguishable
    // panes). It was the entity LEAF, which is `image_raw` for both cameras on the
    // canonical `/cam1/image_raw` + `/cam2/image_raw` robot.
    assert_eq!(images[0].name.as_deref(), Some("Images: /camera/front"));
    assert_eq!(images[1].name.as_deref(), Some("Images: /camera/wrist"));

    // Both placements still land in spatial2d — and each camera also
    // lands in the status panel its field dump would render in.
    assert_eq!(
        placement_pairs(&l),
        vec![
            ("/camera/front".to_string(), "spatial2d".to_string()),
            ("/camera/front".to_string(), "text_document".to_string()),
            ("/camera/wrist".to_string(), "spatial2d".to_string()),
            ("/camera/wrist".to_string(), "text_document".to_string()),
        ]
    );
    // view_count = 3: the two per-camera spatial2d views PLUS the ONE shared
    // status pane. The dump bucket is shared (it grounds both camera
    // globs) — the per-topic SPLIT is a spatial2d-only rule, because two images
    // drawn into one 2D pane hide each other while two field dumps in one text
    // pane do not.
    assert_eq!(l.plan.view_count(), 3);
}

#[test]
fn a_single_camera_keeps_the_plain_images_name_unsuffixed() {
    // The single-camera path is UNCHANGED (no leaf suffix): the go2
    // corpus one-camera oracle in `bare_topics_...` stays "Images".
    let l = compose_layout(&bare(
        vec![att("/camera/image", CAMERA, ArchetypeKind::Image)],
        ComposeStrategy::Auto,
        false,
    ))
    .expect("one camera composes");
    let images = view_of(&l, ViewKind::Spatial2d);
    assert_eq!(
        images.name.as_deref(),
        Some("Images"),
        "no leaf suffix for one"
    );
    assert_eq!(images.origin.as_deref(), Some(CAMERA));
    assert_eq!(images.contents, Some(vec![glob(CAMERA)]));
}

#[test]
fn an_images_role_group_of_two_cameras_titles_each_pane() {
    // A role=images group with a title over two cameras → the title is the base
    // name, suffixed per camera (the role-title contract).
    let l = compose_layout(&ComposeInput {
        groups: vec![ComposeGroup {
            topics: vec![
                att("/camera/front", CAM_FRONT, ArchetypeKind::Image),
                att("/camera/wrist", CAM_WRIST, ArchetypeKind::Image),
            ],
            role: Some(GroupRole::Images),
            title: Some("Cameras".to_string()),
        }],
        strategy: ComposeStrategy::Grid,
        include_robot: false,
    })
    .expect("images role group composes");
    let names: Vec<String> = leaf_views(&l)
        .into_iter()
        .filter(|v| v.kind == ViewKind::Spatial2d)
        .map(|v| v.name.unwrap())
        .collect();
    // Titled by TOPIC, not by the entity leaf.
    assert_eq!(
        names,
        vec!["Cameras: /camera/front", "Cameras: /camera/wrist"]
    );
}

// ── Trailing plot windows + stage backgrounds (compose → emit) ─────
//
// The compose compiler tags its plan `decorate = true`; the hand-rolled emission
// (`build_blueprint_msgs`) stamps a [-30s,0] trailing window on every time_series
// view and the Studio stage background on every spatial view, at each view's own
// blueprint property path — the exact path the viewer reads (asserted here via the
// emitted `.../VisibleTimeRanges` and `.../Background` entity paths). The serialized
// window VALUES are hand-pinned in `blueprint.rs`'s
// `trailing_window_ranges_are_cursor_relative_*`.

/// The blueprint property paths the composed plan emits (decode of the real blueprint
/// LogMsgs — never a re-derivation).
fn emitted_paths(l: &ComposedLayout) -> Vec<String> {
    blueprint_property_paths(&build_blueprint_msgs("f_compose", &l.plan).expect("emit"))
}

fn count_ending(paths: &[String], suffix: &str) -> usize {
    paths.iter().filter(|p| p.ends_with(suffix)).count()
}

#[test]
fn composed_plan_is_decorated_and_stamps_windows_and_backgrounds_by_kind() {
    let l = compose_layout(&bare(go2_corpus(), ComposeStrategy::Auto, true)).expect("composes");
    assert!(l.plan.decorate, "a compose product is a decorated plan");

    let leaves = leaf_views(&l);
    let time_series = leaves
        .iter()
        .filter(|v| v.kind == ViewKind::TimeSeries)
        .count();
    let spatial = leaves
        .iter()
        .filter(|v| matches!(v.kind, ViewKind::Spatial2d | ViewKind::Spatial3d))
        .count();
    assert!(
        time_series >= 1 && spatial >= 1,
        "the go2 corpus yields both plot AND spatial views"
    );

    let paths = emitted_paths(&l);
    // EXACTLY one trailing window per time_series view, one background per spatial
    // view — and a non-time-series view (the hero is spatial, status is text) never
    // gets a window.
    assert_eq!(
        count_ending(&paths, "/VisibleTimeRanges"),
        time_series,
        "one trailing (query) window per time_series view: {paths:?}"
    );
    // One DISPLAY x-axis window per time_series view too (the empty-epoch-axis fix).
    assert_eq!(
        count_ending(&paths, "/TimeAxis"),
        time_series,
        "one display x-axis window per time_series view: {paths:?}"
    );
    assert_eq!(
        count_ending(&paths, "/Background"),
        spatial,
        "one stage background per spatial view: {paths:?}"
    );
}

#[test]
fn composed_plan_decoration_component_values_match_the_hand_oracle() {
    // Decode the emitted COMPONENT VALUES from a REAL compose output
    // (not just their entity paths) and pin them against hand oracles — the stage
    // background is SolidColor #10161f and the trailing window is cursor-relative
    // [-30s, 0] on both nanosecond timelines. A value drift (e.g. wrong kind/color/
    // bounds) fails HERE even though the entity paths are unchanged.
    let l = compose_layout(&bare(go2_corpus(), ComposeStrategy::Auto, true)).expect("composes");
    let dec = blueprint_decorations(&build_blueprint_msgs("f_compose", &l.plan).expect("emit"));

    let spatial = leaf_views(&l)
        .iter()
        .filter(|v| matches!(v.kind, ViewKind::Spatial2d | ViewKind::Spatial3d))
        .count();
    let time_series = leaf_views(&l)
        .iter()
        .filter(|v| v.kind == ViewKind::TimeSeries)
        .count();
    assert!(spatial >= 1 && time_series >= 1);

    // Value pins use the rerun-type-free accessors (an integration test stays out of
    // rerun's type namespace); the full rerun-typed oracle + the SolidColor mutation
    // kill live in `blueprint.rs`'s in-module `decorated_plan_emits_solid_*` test.
    assert_eq!(
        dec.backgrounds.len(),
        spatial,
        "one background per spatial view"
    );
    for bg in &dec.backgrounds {
        assert_eq!(
            bg.kind_names(),
            vec!["SolidColor".to_string()],
            "SolidColor kind: {bg:?}"
        );
        assert_eq!(
            bg.colors,
            vec![[0x10, 0x16, 0x1f, 0xff]],
            "stage color #10161f: {bg:?}"
        );
    }

    assert_eq!(
        dec.windows.len(),
        time_series,
        "one query window per time_series view"
    );
    for w in &dec.windows {
        assert_eq!(
            w.timelines(),
            vec!["robot_time".to_string(), "log_time".to_string()],
            "the two nanosecond timelines: {w:?}"
        );
        assert_eq!(
            w.cursor_relative_ns(),
            vec![
                (Some(-30_000_000_000), Some(0)),
                (Some(-30_000_000_000), Some(0))
            ],
            "cursor-relative [-30s, 0] on both timelines: {w:?}"
        );
    }

    // One DISPLAY x-axis window per time_series view, cursor-relative [-30s, 0],
    // timeline-agnostic (a SINGLE range, not per-timeline) — the empty-epoch-axis fix's
    // real emitted value from a compose product.
    assert_eq!(
        dec.time_axes.len(),
        time_series,
        "one display x-axis per time_series view"
    );
    for ta in &dec.time_axes {
        assert_eq!(
            ta.cursor_relative_ns(),
            vec![(Some(-30_000_000_000), Some(0))],
            "cursor-relative [-30s, 0], one timeline-agnostic range: {ta:?}"
        );
    }
}

#[test]
fn the_go2_default_is_scene_only_with_the_studio_stage_background() {
    // Design decision: the built-in default/empty scene is Scene-ONLY (a
    // single decorated 3D scene) — the empty Telemetry + Status panels are DROPPED (a
    // robot-free desk has no scalar/status data, so those panels were empty and the
    // Telemetry axis showed the nonsense 1970-era epoch span). The lone spatial scene
    // carries the Studio stage `Background`, and — with no time_series view — the default
    // emits ZERO telemetry decorations (no `/VisibleTimeRanges`, no `/TimeAxis`). Those
    // ride ONLY the compose path's plots (which have data — pinned above).
    let plan = BlueprintPlan::go2_default();
    assert!(
        plan.decorate,
        "the default still carries the stage decoration"
    );
    let paths = blueprint_property_paths(&build_blueprint_msgs("f_default", &plan).expect("emit"));
    assert_eq!(
        count_ending(&paths, "/Background"),
        1,
        "the default's spatial scene carries the stage background: {paths:?}"
    );
    assert_eq!(
        count_ending(&paths, "/VisibleTimeRanges"),
        0,
        "Scene-only: no telemetry query window on the default: {paths:?}"
    );
    assert_eq!(
        count_ending(&paths, "/TimeAxis"),
        0,
        "Scene-only: no display x-axis window (no empty plot → no 1970 axis): {paths:?}"
    );
}

#[test]
fn compose_and_emit_are_deterministic() {
    let a = compose_layout(&bare(go2_corpus(), ComposeStrategy::Auto, true)).unwrap();
    let b = compose_layout(&bare(go2_corpus(), ComposeStrategy::Auto, true)).unwrap();
    assert_eq!(a.plan, b.plan, "two composes yield identical plans");
    assert_eq!(
        emitted_paths(&a),
        emitted_paths(&b),
        "deterministic node ids ⇒ identical emitted entity paths"
    );
}

#[test]
fn single_scalars_topic_composes_a_single_decorated_plot_with_the_window() {
    let l = compose_layout(&bare(
        vec![att("/cmd_vel", CMD_VEL, ArchetypeKind::Scalars)],
        ComposeStrategy::Auto,
        false,
    ))
    .expect("a single scalars topic composes");
    assert!(l.plan.decorate);
    let leaves = leaf_views(&l);
    assert_eq!(leaves.len(), 1, "one topic → one view");
    assert_eq!(leaves[0].kind, ViewKind::TimeSeries);
    let paths = emitted_paths(&l);
    assert_eq!(count_ending(&paths, "/VisibleTimeRanges"), 1, "{paths:?}");
    assert_eq!(count_ending(&paths, "/TimeAxis"), 1, "{paths:?}");
    assert_eq!(count_ending(&paths, "/Background"), 0, "{paths:?}");
}

#[test]
fn scales_to_ninety_scalars_topics_one_decorated_plot() {
    // 90 distinct scalars topics collapse into ONE plots view (the Twist-6-panel fix:
    // one plot per topic in ONE view). Compose stays decorated + bounded, and the ONE
    // time_series view carries EXACTLY one trailing window.
    let topics: Vec<AttachedRender> = (0..90)
        .map(|i| {
            att(
                &format!("/t{i}"),
                &format!("world/t{i}"),
                ArchetypeKind::Scalars,
            )
        })
        .collect();
    let l = compose_layout(&bare(topics, ComposeStrategy::Auto, false)).expect("90 topics compose");
    assert!(l.plan.decorate);
    let time_series = leaf_views(&l)
        .iter()
        .filter(|v| v.kind == ViewKind::TimeSeries)
        .count();
    assert_eq!(time_series, 1, "all scalars fold into one plots view");
    let paths = emitted_paths(&l);
    assert_eq!(count_ending(&paths, "/VisibleTimeRanges"), 1, "{paths:?}");
    assert_eq!(count_ending(&paths, "/TimeAxis"), 1, "{paths:?}");
    assert_eq!(count_ending(&paths, "/Background"), 0, "{paths:?}");
}

// ── The representation choice reaches the COMPOSE compiler too ──────

/// A plot topic asked for text is PLACED in a `text_document` bucket by the
/// semantic compiler, not only by the attach/detach reflow.
///
/// Both composers bucket by "the views this topic renders in", and that question now
/// depends on the operator's choice. The compiler is the agent's
/// path rather than the sidebar's, so it is easy to leave behind — and a layout
/// that placed the topic only in `time_series` would drop the document on the
/// floor while the sink logged it.
///
/// Reverting the bucketing loop to
/// `views_for_archetype(arch)` fails this test — the `Both` topic is placed once,
/// in `time_series`, and the `Text` topic is placed in `time_series` it does not
/// render in.
#[test]
fn the_compiler_places_a_forced_dump_in_a_text_document_view() {
    fn as_(topic: AttachedRender, representation: Representation) -> AttachedRender {
        AttachedRender {
            representation,
            ..topic
        }
    }
    let plot = || att("/cmd_vel", CMD_VEL, ArchetypeKind::Scalars);

    let views_of_topic = |input: ComposeInput| -> Vec<ViewKind> {
        let composed = compose_layout(&input).expect("compose");
        let mut v: Vec<ViewKind> = composed
            .placements
            .iter()
            .filter(|p| p.topic == "/cmd_vel")
            .map(|p| p.view)
            .collect();
        v.sort_by_key(|k| k.as_str());
        v
    };

    assert_eq!(
        views_of_topic(bare(vec![plot()], ComposeStrategy::Grid, false)),
        vec![ViewKind::TimeSeries],
        "an un-overridden plot topic is placed in its plot bucket only"
    );
    assert_eq!(
        views_of_topic(bare(
            vec![as_(plot(), Representation::Both)],
            ComposeStrategy::Grid,
            false
        )),
        vec![ViewKind::TextDocument, ViewKind::TimeSeries],
        "Both places the topic in BOTH buckets"
    );
    assert_eq!(
        views_of_topic(bare(
            vec![as_(plot(), Representation::Text)],
            ComposeStrategy::Grid,
            false
        )),
        vec![ViewKind::TextDocument],
        "Text places it in the document bucket ALONE — never in a plot it does \
         not render in"
    );
}

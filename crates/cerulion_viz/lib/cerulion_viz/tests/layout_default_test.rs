// SPDX-License-Identifier: AGPL-3.0-only
//! By design: the dynamic consolidated default layout + the wall-clock
//! default timeline, pinned against HAND oracles (never a self-compare against the
//! function under test).
//!
//! [`cerulion_viz::blueprint::default_layout`] turns the currently-attached topics
//! into the layout vizd re-applies on EVERY attach/detach when no explicit
//! `set_blueprint`/`compose_layout` is active: the primary Scene (attached spatial
//! topics + the robot) beside a GRID of per-topic views — ONE `time_series` view per
//! plot topic (all its scalar series in ONE plot, NEVER one-per-component), NEVER
//! tabs. Detaching reflows a topic OUT; the empty set returns the Scene-only
//! default. [`build_blueprint_msgs`] additionally pins the viewer's DEFAULT active
//! timeline to `log_time` (desk wall-clock receive time) while `robot_time` stays a
//! referenced timeline.

use cerulion_viz::blueprint::{
    blueprint_decorations, blueprint_panel_timeline, blueprint_property_paths,
    build_blueprint_msgs, compose_layout, default_layout, validate_plan_against_attached,
    views_for_render, AttachedRender, BlueprintPlan, ComposeGroup, ComposeInput, ComposeStrategy,
    ContainerKind, LayoutError, PlanNode, PlanView, ViewKind,
};
use cerulion_viz::representation::Representation;
use cerulion_viz::sink::ArchetypeKind;
use cerulion_viz::sink::RenderProof;

// ── Hand entity literals + a corpus (archetypes are the fixed table) ──────────

const CLOUD: &str = "world/utlidar/cloud";
const ODOM: &str = "world/lf/sportmodestate";
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

/// The `/<entity>/**` leading-slash content glob (the compiler's canonical form).
fn glob(entity: &str) -> String {
    format!("/{entity}/**")
}

/// Every leaf view in the plan, declaration order (with its container path kinds
/// unrecorded — [`container_kinds`] pins those).
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

/// Every container kind in the plan tree (so a test can pin "NO tabs, ever").
fn container_kinds(plan: &BlueprintPlan) -> Vec<ContainerKind> {
    fn walk(node: &PlanNode, out: &mut Vec<ContainerKind>) {
        if let PlanNode::Container(c) = node {
            out.push(c.kind);
            c.children.iter().for_each(|ch| walk(ch, out));
        }
    }
    let mut out = Vec::new();
    walk(&plan.root, &mut out);
    out
}

/// The leaf views of a given kind.
fn views_of(plan: &BlueprintPlan, kind: ViewKind) -> Vec<PlanView> {
    leaf_views(plan)
        .into_iter()
        .filter(|v| v.kind == kind)
        .collect()
}

/// The single Scene (spatial3d) hero view.
fn scene(plan: &BlueprintPlan) -> PlanView {
    let mut s = views_of(plan, ViewKind::Spatial3d);
    assert_eq!(s.len(), 1, "exactly one Scene hero: {plan:?}");
    s.pop().unwrap()
}

/// A universal invariant across EVERY default-layout state: never a `Tabs` container.
fn assert_no_tabs(plan: &BlueprintPlan) {
    assert!(
        !container_kinds(plan).contains(&ContainerKind::Tabs),
        "the default layout must NEVER contain a Tabs container: {plan:?}"
    );
    // And it is always auto_views OFF (so the viewer never re-explodes scalars into
    // one-per-component tabs).
    assert!(
        !plan.auto_views,
        "the default layout must keep auto_views OFF (the Twist-6-tabs fix): {plan:?}"
    );
}

// ── 1. The headline: one plot topic → Scene + ONE consolidated plot ───────────

#[test]
fn single_plot_topic_yields_scene_plus_one_consolidated_timeseries_view() {
    // `/lf/sportmodestate → Odometry` renders in spatial3d (pose) AND time_series (the
    // 6 twist scalars). Unconsolidated, those 6 scalars explode into 6 tabs; the layout
    // consolidates them into ONE time_series view.
    let plan = default_layout(&[att("/lf/sportmodestate", ODOM, ArchetypeKind::Odometry)]);
    assert_no_tabs(&plan);

    // (a) EXACTLY ONE time_series view, contents = the ONE topic glob (all 6 series
    //     land under it → 6 lines in ONE plot, not 6 views).
    let plots = views_of(&plan, ViewKind::TimeSeries);
    assert_eq!(plots.len(), 1, "one consolidated plot view: {plan:?}");
    assert_eq!(
        plots[0].contents,
        Some(vec![glob(ODOM)]),
        "the plot grounds the ONE topic subtree (all its scalar series): {plan:?}"
    );

    // (b) The dual-view Odometry ALSO feeds the Scene (its pose), grounded by the
    //     explicit union + the robot glob.
    let hero = scene(&plan);
    assert_eq!(
        hero.contents,
        Some(vec![glob(ODOM), ROBOT_GLOB.to_string()]),
        "the Scene grounds the attached spatial topic + the robot: {plan:?}"
    );
    assert_eq!(hero.origin.as_deref(), Some("/world"));

    // (c) Arrangement: Horizontal[ Scene | plot ], Scene the dominant column (3:1.5).
    let PlanNode::Container(root) = &plan.root else {
        panic!("root is the hero|sidebar split: {plan:?}");
    };
    assert_eq!(root.kind, ContainerKind::Horizontal);
    assert_eq!(root.shares, Some(vec![3.0, 1.5]));
    assert!(
        matches!(&root.children[0], PlanNode::View(v) if v.kind == ViewKind::Spatial3d),
        "the FIRST (large) child is the Scene: {plan:?}"
    );
    // decorate ON → the plot gets the trailing window, the scene the stage bg.
    assert!(plan.decorate);
}

// ── 2. Two plots + a spatial topic → Scene + a GRID of two plots ──────────────

#[test]
fn two_plots_plus_spatial_reflow_to_scene_beside_a_grid_of_two_plots() {
    let plan = default_layout(&[
        att("/utlidar/cloud", CLOUD, ArchetypeKind::Points3D), // spatial-only → Scene
        att("/cmd_vel", CMD_VEL, ArchetypeKind::Scalars),      // plot
        att("/lf/sportmodestate", ODOM, ArchetypeKind::Odometry), // pose + plot
    ]);
    assert_no_tabs(&plan);

    // TWO time_series views (one per plot topic — NEVER unioned into one).
    let plots = views_of(&plan, ViewKind::TimeSeries);
    assert_eq!(plots.len(), 2, "one plot PER plot topic: {plan:?}");
    let plot_globs: Vec<_> = plots.iter().map(|v| v.contents.clone()).collect();
    assert_eq!(
        plot_globs,
        vec![Some(vec![glob(CMD_VEL)]), Some(vec![glob(ODOM)])],
        "each plot grounds its OWN topic subtree, in attach order: {plan:?}"
    );

    // The Scene grounds BOTH spatial topics (cloud + odom pose) + the robot.
    let hero = scene(&plan);
    assert_eq!(
        hero.contents,
        Some(vec![glob(CLOUD), glob(ODOM), ROBOT_GLOB.to_string()]),
        "the Scene unions every attached spatial topic + the robot: {plan:?}"
    );

    // Arrangement: Horizontal[ Scene | Grid(plot, plot) ] — the sidebar is a GRID
    // (tiled + shrinking, NEVER tabs).
    let PlanNode::Container(root) = &plan.root else {
        panic!("root is hero|sidebar: {plan:?}");
    };
    assert_eq!(root.kind, ContainerKind::Horizontal);
    let PlanNode::Container(sidebar) = &root.children[1] else {
        panic!("sidebar is a grid container: {plan:?}");
    };
    assert_eq!(sidebar.kind, ContainerKind::Grid, "sidebar tiles as a GRID");
    assert_eq!(sidebar.children.len(), 2);
}

// ── 3. Detach reflow: dropping a plot collapses the grid; detach-all → Scene ──

#[test]
fn detach_one_plot_reflows_to_scene_plus_one_plot() {
    // Start with cloud + 2 plots; the reflow after detaching /cmd_vel is exactly the
    // layout `default_layout` derives from the REDUCED attach set (the daemon re-runs
    // it on every detach).
    let after_detach = default_layout(&[
        att("/utlidar/cloud", CLOUD, ArchetypeKind::Points3D),
        att("/lf/sportmodestate", ODOM, ArchetypeKind::Odometry),
    ]);
    assert_no_tabs(&after_detach);
    // ONE plot remains → the sidebar collapses to a single view (no grid nesting).
    let plots = views_of(&after_detach, ViewKind::TimeSeries);
    assert_eq!(plots.len(), 1, "the surviving plot: {after_detach:?}");
    assert_eq!(plots[0].contents, Some(vec![glob(ODOM)]));
    let PlanNode::Container(root) = &after_detach.root else {
        panic!("Scene|plot split: {after_detach:?}");
    };
    assert!(
        matches!(&root.children[1], PlanNode::View(v) if v.kind == ViewKind::TimeSeries),
        "a single sidebar view is placed directly (no 1-child grid): {after_detach:?}"
    );
}

#[test]
fn detach_all_returns_the_scene_only_default() {
    // The empty attach set (detach-all) is the EXACT Scene-only default.
    assert_eq!(
        default_layout(&[]),
        BlueprintPlan::go2_default(),
        "detach-all is the Scene-only default"
    );
    // Only-SILENT topics (attached, no resolved archetype) are ALSO the Scene-only
    // default — never a fabricated view.
    assert_eq!(
        default_layout(&[silent("/quiet", "world/quiet")]),
        BlueprintPlan::go2_default(),
        "an only-silent attach set is the Scene-only default (no fabricated view)"
    );
}

// ── 4. A spatial-only attach set is a lone Scene view (no empty sidebar) ──────

#[test]
fn spatial_only_topics_yield_a_lone_scene_view() {
    let plan = default_layout(&[att("/utlidar/cloud", CLOUD, ArchetypeKind::Points3D)]);
    assert_no_tabs(&plan);
    let PlanNode::View(hero) = &plan.root else {
        panic!("a lone Scene view (no sidebar): {plan:?}");
    };
    assert_eq!(hero.kind, ViewKind::Spatial3d);
    assert_eq!(
        hero.contents,
        Some(vec![glob(CLOUD), ROBOT_GLOB.to_string()]),
        "the Scene grounds the cloud + the robot: {plan:?}"
    );
}

// ── 5. Image topics get their own spatial2d view in the sidebar grid ──────────

#[test]
fn image_topic_gets_its_own_view_in_the_sidebar() {
    let plan = default_layout(&[
        att("/lf/sportmodestate", ODOM, ArchetypeKind::Odometry),
        att("/camera/image", CAMERA, ArchetypeKind::Image),
    ]);
    assert_no_tabs(&plan);
    let images = views_of(&plan, ViewKind::Spatial2d);
    assert_eq!(images.len(), 1, "one image view: {plan:?}");
    assert_eq!(images[0].contents, Some(vec![glob(CAMERA)]));
    // The camera ALSO gets the status pane its field dump renders in
    // (a raw frame in an encoding this build cannot decode), rooted at the SAME
    // entity — so a degraded camera shows its fields instead of a blank 2D pane.
    let dumps = views_of(&plan, ViewKind::TextDocument);
    assert_eq!(dumps.len(), 1, "one status view for the camera: {plan:?}");
    assert_eq!(dumps[0].origin.as_deref(), Some(CAMERA));
    assert_eq!(dumps[0].contents, Some(vec![glob(CAMERA)]));
    // Sidebar = Grid(plot, image, camera status) — three per-topic views tiled.
    let PlanNode::Container(root) = &plan.root else {
        panic!("hero|sidebar: {plan:?}");
    };
    let PlanNode::Container(sidebar) = &root.children[1] else {
        panic!("grid sidebar: {plan:?}");
    };
    assert_eq!(sidebar.kind, ContainerKind::Grid);
    assert_eq!(sidebar.children.len(), 3);
}

// ── 6. Task B: the viewer's DEFAULT active timeline is log_time (wall clock) ───

#[test]
fn every_emitted_blueprint_pins_the_default_active_timeline_to_log_time() {
    // The pin rides EVERY plan vizd emits — the Go2 default, the dynamic default, AND
    // a composed layout — so a fresh attach in ANY state opens on wall-clock receive
    // time, not the robot's "+6h10m" uptime axis.
    let plans = vec![
        ("go2_default", BlueprintPlan::go2_default()),
        (
            "default_layout",
            default_layout(&[att("/cmd_vel", CMD_VEL, ArchetypeKind::Scalars)]),
        ),
        (
            "compose_layout",
            compose_layout(&ComposeInput {
                groups: vec![ComposeGroup {
                    topics: vec![att("/cmd_vel", CMD_VEL, ArchetypeKind::Scalars)],
                    role: None,
                    title: None,
                }],
                strategy: ComposeStrategy::Auto,
                include_robot: false,
            })
            .expect("composes")
            .plan,
        ),
    ];
    for (label, plan) in plans {
        let msgs = build_blueprint_msgs("vizd", &plan).expect("emit");
        assert_eq!(
            blueprint_panel_timeline(&msgs).as_deref(),
            Some("log_time"),
            "{label}: the pinned default active timeline is log_time"
        );
        assert!(
            blueprint_property_paths(&msgs)
                .iter()
                .any(|p| p.trim_start_matches('/') == "time_panel"),
            "{label}: a time_panel chunk is emitted where the viewer reads it"
        );
    }
}

#[test]
fn robot_time_remains_a_referenced_timeline_alongside_the_log_time_default() {
    // The `log_time` default does NOT delete `robot_time`: a decorated plot's trailing
    // window is still emitted for BOTH timelines (so `robot_time` stays selectable in
    // the viewer's dropdown for cross-sensor correlation).
    let plan = default_layout(&[att("/cmd_vel", CMD_VEL, ArchetypeKind::Scalars)]);
    let msgs = build_blueprint_msgs("vizd", &plan).expect("emit");
    let dec = blueprint_decorations(&msgs);
    let window = dec
        .windows
        .iter()
        .find(|w| w.path.ends_with("/VisibleTimeRanges"))
        .expect("the decorated plot carries a trailing window");
    let timelines = window.timelines();
    assert!(
        timelines.iter().any(|t| t == "robot_time"),
        "robot_time is still a referenced timeline: {timelines:?}"
    );
    assert!(
        timelines.iter().any(|t| t == "log_time"),
        "log_time is also referenced: {timelines:?}"
    );
}

// ── The new archetypes flow through the SAME dynamic default ─────────

#[test]
fn boxes3d_folds_into_the_scene_and_occupancy_grid_gets_its_own_2d_view() {
    const DETECTIONS: &str = "world/perception/detections";
    const MAP: &str = "world/map";
    let plan = default_layout(&[
        att("/perception/detections", DETECTIONS, ArchetypeKind::Boxes3D),
        att("/map", MAP, ArchetypeKind::OccupancyGrid),
    ]);
    assert_no_tabs(&plan);

    // (a) A detection box is 3D geometry → it folds into the Scene's explicit
    //     content union (beside the cloud), never into a separate pane.
    let hero = scene(&plan);
    assert_eq!(
        hero.contents,
        Some(vec![glob(DETECTIONS), ROBOT_GLOB.to_string()]),
        "the Boxes3D topic grounds the Scene: {plan:?}"
    );

    // (b) The occupancy grid renders as a grayscale image → its OWN spatial2d
    //     sidebar view, rooted at the map entity.
    let images = views_of(&plan, ViewKind::Spatial2d);
    assert_eq!(images.len(), 1, "one 2D view for the map: {plan:?}");
    assert_eq!(images[0].origin.as_deref(), Some(MAP));
    assert_eq!(images[0].contents, Some(vec![glob(MAP)]));

    // (c) Neither archetype invents a PLOT, but the dump-companion rule gives BOTH the status
    //     pane their render arms can degrade into (a box with no extractable
    //     geometry, a grid with no `info` dimensions / a short cell buffer), one
    //     per topic, each rooted at its own entity. Without it, a robot publishing
    //     malformed detections or an unsized map shows two empty panes and no
    //     diagnostic anywhere in the viewer.
    assert!(views_of(&plan, ViewKind::TimeSeries).is_empty());
    let dumps = views_of(&plan, ViewKind::TextDocument);
    assert_eq!(
        dumps
            .iter()
            .map(|v| v.origin.clone().expect("per-topic origin"))
            .collect::<Vec<_>>(),
        vec![DETECTIONS.to_string(), MAP.to_string()],
        "one status pane per degradable topic, in attach order: {plan:?}"
    );
}

// ── The representation choice reaches the LAYOUT ────────────────────
//
// The render half is pinned in `sink_dispatch_test`. This is the other end of
// the rule: an override the layout did not apply logs a `TextDocument`
// into a topic whose views carry none, and the operator gets a document nothing
// displays. So the views a topic is given must be the views its choice resolves
// to — asserted here through the PRODUCTION reflow, not only through the pure
// helper.

fn att_as(
    topic: &str,
    entity: &str,
    arch: ArchetypeKind,
    representation: Representation,
) -> AttachedRender {
    AttachedRender {
        representation,
        ..att(topic, entity, arch)
    }
}

/// The pure view answer, per choice, on a plot topic and on a spatial one.
///
/// Hand oracles: `views_for_archetype` is the kind's table, and a choice can only
/// suppress the visual half or add the `text_document` a dump needs.
#[test]
fn views_for_render_applies_the_choice_to_the_kinds_table() {
    let cases: &[(ArchetypeKind, Representation, &[ViewKind])] = &[
        // A plain plot topic: Auto and Visual are the kind's own table.
        (
            ArchetypeKind::Scalars,
            Representation::Auto,
            &[ViewKind::TimeSeries],
        ),
        (
            ArchetypeKind::Scalars,
            Representation::Visual,
            &[ViewKind::TimeSeries],
        ),
        (
            ArchetypeKind::Scalars,
            Representation::Both,
            &[ViewKind::TimeSeries, ViewKind::TextDocument],
        ),
        (
            ArchetypeKind::Scalars,
            Representation::Text,
            &[ViewKind::TextDocument],
        ),
        // A topic the election already gave both views: Visual takes the
        // document's view away with the document.
        (
            ArchetypeKind::ScalarsWithText,
            Representation::Auto,
            &[ViewKind::TimeSeries, ViewKind::TextDocument],
        ),
        (
            ArchetypeKind::ScalarsWithText,
            Representation::Visual,
            &[ViewKind::TimeSeries],
        ),
        (
            ArchetypeKind::ScalarsWithText,
            Representation::Both,
            &[ViewKind::TimeSeries, ViewKind::TextDocument],
        ),
        // A spatial topic: the dump is additive there too, and Text leaves the
        // scene entirely.
        (
            ArchetypeKind::Points3D,
            Representation::Both,
            &[ViewKind::Spatial3d, ViewKind::TextDocument],
        ),
        (
            ArchetypeKind::Points3D,
            Representation::Text,
            &[ViewKind::TextDocument],
        ),
        // The DE-DUP guard, which is decisive on exactly the kinds that both take
        // the overlay (`!renders_own_dump`) and ALREADY carry a `text_document`
        // (the nine that can degrade to a dump, plus `TextLog`). Deleting it
        // yields `[Spatial2d, TextDocument, TextDocument]` here — and nothing
        // downstream de-dups: `default_layout_excluding` pushes one `PlanView` per
        // view, `NodeIdMinter` deliberately disambiguates a repeated identity, and
        // the daemon maps the list straight onto the wire `view_kinds`. So
        // deleting it ships a duplicated status pane AND a repeated wire entry.
        (
            ArchetypeKind::Image,
            Representation::Both,
            &[ViewKind::Spatial2d, ViewKind::TextDocument],
        ),
        // `TextLog` reaches the guard by the other route: its own view list is
        // `[TextDocument]` alone, and it is deliberately OUTSIDE `renders_own_dump`
        // (its rolling mirror logs at `<entity>/text`, not the entity), so the
        // overlay is set and the guard is what keeps the list at one.
        (
            ArchetypeKind::TextLog,
            Representation::Both,
            &[ViewKind::TextDocument],
        ),
    ];
    for &(kind, representation, want) in cases {
        let got = views_for_render(&att_as("/t", "world/t", kind, representation));
        assert_eq!(got, want, "{kind:?} under {representation:?}");
    }

    // A still-silent topic is UN-TYPED, not un-rendered: no views, whatever the
    // operator chose — the layout must never fabricate one.
    for representation in Representation::ALL {
        let mut silent = silent("/quiet", "world/quiet");
        silent.representation = representation;
        assert!(
            views_for_render(&silent).is_empty(),
            "a silent topic yields no views under {representation:?}"
        );
    }
}

/// **The motivating case, through the production reflow.** A plot topic asked for
/// text gets a `text_document` view BESIDE its plot — and asked for text alone,
/// gets the document view INSTEAD of the plot.
#[test]
fn a_plot_topic_asked_for_text_is_given_a_document_view() {
    // Baseline: the plot alone (this is `single_plot_topic_…`'s shape).
    let auto = default_layout(&[att("/cmd_vel", CMD_VEL, ArchetypeKind::Scalars)]);
    assert_eq!(views_of(&auto, ViewKind::TimeSeries).len(), 1);
    assert!(
        views_of(&auto, ViewKind::TextDocument).is_empty(),
        "an un-overridden plot topic gets no document view: {auto:?}"
    );

    // Both: the plot AND a document view, each grounded on the topic's subtree.
    let both = default_layout(&[att_as(
        "/cmd_vel",
        CMD_VEL,
        ArchetypeKind::Scalars,
        Representation::Both,
    )]);
    assert_no_tabs(&both);
    assert_eq!(views_of(&both, ViewKind::TimeSeries).len(), 1);
    let docs = views_of(&both, ViewKind::TextDocument);
    assert_eq!(docs.len(), 1, "one document view: {both:?}");
    assert_eq!(
        docs[0].contents,
        Some(vec![glob(CMD_VEL)]),
        "the document view grounds the topic's own subtree: {both:?}"
    );

    // Text: the document view REPLACES the plot.
    let text = default_layout(&[att_as(
        "/cmd_vel",
        CMD_VEL,
        ArchetypeKind::Scalars,
        Representation::Text,
    )]);
    assert!(
        views_of(&text, ViewKind::TimeSeries).is_empty(),
        "a suppressed visual half leaves no plot view behind: {text:?}"
    );
    assert_eq!(views_of(&text, ViewKind::TextDocument).len(), 1);
}

/// **Guardrail 2 must ask what the TOPIC renders, not what its
/// archetype could.**
///
/// This is NOT the only coverage the guardrail has: six guardrail-2 oracles live
/// in `blueprint.rs`'s own
/// test module, two of them asserting the full `Spatial2dRendersNothing` VALUE.
/// Those oracles are precisely there to pin that the DECISION lives in
/// `views_for_render` while both DIAGNOSTICS stay on the kind.
/// This arm is the OVERRIDE-path complement to them, not their predecessor.
///
/// `validate_plan_against_attached` refuses an explicit `spatial2d` view whose
/// grounded topics can none of them display in 2D — the empty-pane refusal. It
/// judged capability from `views_for_archetype`, so a camera under `Text` (whose
/// `render_classified` returns before ANY geometry) counted as 2D-capable and the
/// view was validated into existence, producing exactly the empty pane the
/// guardrail exists to prevent.
///
/// The `Auto` arm is the anti-tautology half: the same plan, the same topic, and
/// the refusal must NOT fire — so this reads the override rather than a guardrail
/// that refuses everything.
#[test]
fn the_spatial2d_guardrail_reads_the_topics_choice_not_its_archetype() {
    let plan = |origin: &str| BlueprintPlan {
        root: PlanNode::View(PlanView {
            kind: ViewKind::Spatial2d,
            name: Some("Camera".to_string()),
            origin: Some(format!("/{origin}")),
            contents: None,
        }),
        auto_views: false,
        decorate: false,
    };

    // A camera the operator left alone: the 2D view is legitimate.
    let auto = vec![att("/camera/image", CAMERA, ArchetypeKind::Image)];
    assert!(
        validate_plan_against_attached(&plan(CAMERA), &auto).is_ok(),
        "an un-overridden camera renders in 2D — this view must be accepted"
    );

    // The same camera under `Text`: the render logs no image at all, so a
    // spatial2d view grounded solely on it can only ever be empty.
    let text = vec![att_as(
        "/camera/image",
        CAMERA,
        ArchetypeKind::Image,
        Representation::Text,
    )];
    let err = validate_plan_against_attached(&plan(CAMERA), &text)
        .expect_err("a 2D view grounded only on a text-only topic must be refused");
    assert!(
        matches!(err, LayoutError::Spatial2dRendersNothing { .. }),
        "…and refused as the empty-pane case, naming the offender: {err:?}"
    );
}

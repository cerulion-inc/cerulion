// SPDX-License-Identifier: AGPL-3.0-only
//! Studio agent smart layouts: the DETERMINISTIC layout mapping,
//! pinned against HAND oracles (never a self-compare against the function under
//! test).
//!
//! Three pure mappings are the foundation the layout compiler + the enriched
//! control protocol rest on. The Studio agent guessed `/cloud` / `/cmd_vel` when
//! the real render paths are `world/<topic path>`; these pin the real mappings.
//!
//! - `topic → route key → entity path`
//!   ([`cerulion_viz::sink::route_key_for_topic`] +
//!   [`cerulion_viz::sink::route_for_input`]) — MUST be stable across attaches
//!   and pinned over a representative topic corpus (media / TF / odom /
//!   generic leaves / nested paths / overrides / sanitization edges).
//!   **The entity-path fix rewrote this mapping**: the entity is now `world/<full sanitized
//!   topic>` rather than `world/odom/base/<last topic segment>`, because the
//!   last-segment rule collapsed 38 of a robot's 75 topics onto 6
//!   entities. The corpus-scale oracle (the real 114-topic Go2 list → zero
//!   collisions, plus the six measured collision groups) lives in
//!   `entity_path_collision_test.rs`; what stays HERE is the per-shape
//!   contract: which names carry semantics, casing, overrides, degenerate
//!   inputs.
//! - `archetype → view kind(s)` ([`cerulion_viz::blueprint::views_for_archetype`])
//!   — the decision table, exhaustive over every [`ArchetypeKind`] variant.
//! - `archetype → rerun component families`
//!   ([`cerulion_viz::blueprint::archetype_components`]) — exhaustive.

use cerulion_viz::blueprint::{archetype_components, views_for_archetype, ViewKind};
use cerulion_viz::sink::{route_for_input, route_key_for_topic, ArchetypeKind};

// The viz root, RE-DERIVED here as a literal (a hand oracle — never read from
// the production `crate::tf` constants, so a constant change is CAUGHT, not
// silently followed).
const WORLD: &str = "world";

/// Resolve an ABSOLUTE topic (no override) all the way to its render entity —
/// exactly what the daemon does for a discovered/tapped topic.
fn entity_of(topic: &str) -> String {
    route_for_input(&route_key_for_topic(topic, None)).entity
}

// ── 1. topic → entity: the mechanical rule ───────────────────────────

#[test]
fn every_topic_gets_its_own_entity_under_the_world_root() {
    // The entity is `world/` + the topic's sanitized segments. The earlier
    // media table (any topic whose LAST segment was `cloud`/`points`/`image`/…
    // folded onto ONE canonical entity) is GONE — media names carry no path
    // meaning, so four different lidar topics get four different entities.
    for (topic, expected) in [
        ("/utlidar/cloud", "world/utlidar/cloud"),
        ("/velodyne/points", "world/velodyne/points"),
        ("/sensor/pointcloud", "world/sensor/pointcloud"),
        ("/lidar", "world/lidar"),
        ("/camera/image", "world/camera/image"),
        ("/front/camera", "world/front/camera"),
        ("/cam/jpeg", "world/cam/jpeg"),
        ("/stream/compressed", "world/stream/compressed"),
    ] {
        assert_eq!(entity_of(topic), expected, "{topic} → entity");
    }
    // Ordinary telemetry / control topics, and a deeply nested one keeping
    // EVERY segment (a last-segment rule would collapse it).
    assert_eq!(entity_of("/cmd_vel"), format!("{WORLD}/cmd_vel"));
    assert_eq!(entity_of("/imu/data"), format!("{WORLD}/imu/data"));
    assert_eq!(
        entity_of("/deep/nested/topic/name"),
        format!("{WORLD}/deep/nested/topic/name")
    );
    let leaf = route_for_input(&route_key_for_topic("/cmd_vel", None));
    assert!(!leaf.is_static && !leaf.drives_robot_root);
}

#[test]
fn tf_and_tf_static_report_the_world_root_and_differ_only_in_the_static_flag() {
    // A TFMessage does NOT render at a per-topic entity — `dispatch_transforms`
    // logs each transform at its own child-frame entity — so the accurate reported
    // entity is the tree's root. `is_static` is the only difference.
    let tf = route_for_input(&route_key_for_topic("/tf", None));
    assert_eq!(tf.entity, WORLD);
    assert!(!tf.is_static, "/tf is temporal");
    assert!(!tf.drives_robot_root);

    let tf_static = route_for_input(&route_key_for_topic("/tf_static", None));
    assert_eq!(tf_static.entity, WORLD);
    assert!(tf_static.is_static, "/tf_static is static");
    assert!(!tf_static.drives_robot_root);

    // The static knob keys on the LAST segment, so a NAMESPACED tf_static topic
    // still logs static (a whole-name match would have lost this once the route
    // key became the full topic).
    let ns = route_for_input(&route_key_for_topic("/robot1/tf_static", None));
    assert!(ns.is_static, "/robot1/tf_static must still log static");
    assert_eq!(ns.entity, WORLD);
}

#[test]
fn odom_named_topics_keep_their_own_entity_and_elect_the_robot_root() {
    // The three odom aliases (matched on the LAST segment) elect
    // `drives_robot_root` and keep their own mechanical entity with the ORIGINAL
    // case preserved.
    let cases = [
        ("/robot/odom", "world/robot/odom"),
        ("/state/odometry", "world/state/odometry"),
        ("/robot_odom", "world/robot_odom"),
    ];
    for (topic, expected) in cases {
        let r = route_for_input(&route_key_for_topic(topic, None));
        assert_eq!(r.entity, expected, "{topic} entity");
        assert!(r.drives_robot_root, "{topic} drives the robot root");
        assert!(!r.is_static, "{topic} is temporal");
    }
    // A non-exact odom-ish name is a plain topic (no robot-root election).
    for topic in ["/odometry_raw", "/base_odom", "/wheel"] {
        assert!(
            !route_for_input(&route_key_for_topic(topic, None)).drives_robot_root,
            "{topic} must not drive the robot root"
        );
    }
}

#[test]
fn knob_matching_is_case_insensitive_but_entities_keep_original_case() {
    // The knob names match case-insensitively, on the LAST segment.
    let odom = route_for_input(&route_key_for_topic("/robot/ODOM", None));
    assert!(odom.drives_robot_root, "ODOM elects the robot root");
    assert!(route_for_input(&route_key_for_topic("/r/TF_STATIC", None)).is_static);
    // Entity segments preserve the user's original casing.
    assert_eq!(odom.entity, format!("{WORLD}/robot/ODOM"));
    assert_eq!(entity_of("/WristCam"), format!("{WORLD}/WristCam"));
    assert_eq!(entity_of("/Laser_map"), format!("{WORLD}/Laser_map"));
}

#[test]
fn an_entity_override_replaces_the_derived_entity_and_round_trips() {
    // The override is a genuine ENTITY-PATH override (which is what
    // `protocol.rs` and Studio's tool schema document it as). Fed through the
    // last-segment key path instead, `world/cam` would land on
    // `world/odom/base/world_cam` — and the daemon's OWN reported entity fed back
    // as an override would land somewhere else again.
    assert_eq!(
        route_for_input(&route_key_for_topic("/some/blah", Some("world/cam"))).entity,
        "world/cam",
        "a path-shaped override IS the entity path"
    );
    assert_eq!(
        route_for_input(&route_key_for_topic("/x/y", Some("wrist"))).entity,
        format!("{WORLD}/wrist"),
        "a bare override is a path under the world root"
    );
    // THE ROUND TRIP (the property the docs promise).
    for topic in ["/some/blah", "/utlidar/cloud", "/api/vui/response"] {
        let reported = entity_of(topic);
        assert_eq!(
            route_for_input(&route_key_for_topic(topic, Some(&reported))).entity,
            reported,
            "{topic}: the reported entity must round-trip as an override"
        );
    }
}

#[test]
fn sanitization_edges_are_total_and_never_panic() {
    // Non-alphanumeric characters sanitize to `_` PLUS a 4-hex FNV suffix of the
    // raw segment (the ONE crate sanitizer's aliasing guard — without it two
    // topics differing only in separators would collapse onto one entity, which
    // is the very class the entity-path fix closes).
    assert_eq!(entity_of("/weird.name"), format!("{WORLD}/weird_name_198b"));
    assert_eq!(entity_of("/a b/c!d"), format!("{WORLD}/a_b_3892/c_d_2bb7"));
    assert_eq!(entity_of("/naïve"), format!("{WORLD}/na_ve_32ab"));
    // An ALL-invalid segment still yields a non-empty, distinguishable segment.
    assert_eq!(entity_of("/日本"), format!("{WORLD}/___ce91"));
    // Degenerate topics collapse to an empty key, never a panic — and never the
    // transform tree's root entity.
    assert_eq!(route_key_for_topic("/", None), "");
    assert_eq!(route_key_for_topic("", None), "");
    assert_eq!(route_key_for_topic("//", None), "");
    for topic in ["/", "", "//"] {
        assert_eq!(
            entity_of(topic),
            format!("{WORLD}/unknown_2325"),
            "{topic:?} → the sanitizer's empty fallback"
        );
        assert_ne!(entity_of(topic), WORLD);
    }
    // A trailing slash is trimmed (so `/robot/odom/` still elects the robot root
    // and resolves to the same entity as `/robot/odom`).
    let trailing = route_for_input(&route_key_for_topic("/robot/odom/", None));
    assert_eq!(trailing.entity, format!("{WORLD}/robot/odom"));
    assert!(trailing.drives_robot_root);
}

// ── 2. archetype → view kind(s): the decision table (exhaustive) ──────

/// Assert `oracle` covers the WHOLE closed archetype set exactly once — the
/// totality guard. Without it a new `ArchetypeKind` variant would
/// silently escape a hand-written table.
fn assert_covers_every_archetype<T>(oracle: &[(ArchetypeKind, T)]) {
    for kind in ArchetypeKind::ALL {
        assert_eq!(
            oracle.iter().filter(|(k, _)| *k == kind).count(),
            1,
            "{kind:?} must appear EXACTLY once in the oracle table"
        );
    }
    assert_eq!(
        oracle.len(),
        ArchetypeKind::ALL.len(),
        "the oracle table has entries outside the closed archetype set"
    );
}

#[test]
fn views_for_archetype_pins_the_ruled_table_for_every_variant() {
    use ArchetypeKind as A;
    use ViewKind::{Spatial2d, Spatial3d, TextDocument, TimeSeries};
    // Hand oracles, one per variant. A future ArchetypeKind variant is a compile
    // error in `views_for_archetype` itself (no wildcard arm), and
    // `assert_covers_every_archetype` makes it a FAILURE here too.
    let oracle: &[(A, &[ViewKind])] = &[
        (A::Points3D, &[Spatial3d]),
        // A LaserScan projects to a Points3D ring → the 3D scene. It
        // renders EMPTY in a 2D view, so `spatial2d` would be the bug this fixes.
        (A::LaserScan, &[Spatial3d]),
        (A::Transform3D, &[Spatial3d]),
        (A::Transforms, &[Spatial3d]),
        (A::Point3D, &[Spatial3d]),
        // The skeleton's render arm dumps whenever no URDF is loaded —
        // which is now ALWAYS (`install_skeleton` has no caller), so it
        // carries the status pane its dump lands in. Without it a `/lowstate`-class
        // topic shows a blank 3D pane and nothing else.
        (A::Skeleton, &[Spatial3d, TextDocument]),
        // An oriented detection box is 3D geometry — beside the cloud in
        // the Scene, never in a 2D pane (the LaserScan lesson applied up front).
        // A box whose geometry does not extract (and whose detection-set
        // element array is absent too) degrades to the dump.
        (A::Boxes3D, &[Spatial3d, TextDocument]),
        // Pose + scalars → BOTH views (order: 3D scene first, then the plot).
        (A::Odometry, &[Spatial3d, TimeSeries]),
        (A::Imu, &[Spatial3d, TimeSeries]),
        // An inferred spatial shape that ALSO carries sibling
        // telemetry renders the primitive AND those series, so it needs BOTH
        // views. `[Spatial3d]` alone would leave every non-pose number without a
        // home — the silent-drop regression these variants exist to close. Still
        // ONE time_series view (the Twist-6-panel-explosion decision holds).
        (A::Transform3DWithScalars, &[Spatial3d, TimeSeries]),
        (A::Point3DWithScalars, &[Spatial3d, TimeSeries]),
        // A raw image whose `encoding` this build does not decode degrades
        // to the dump, so the image family carries the status pane too.
        (A::Image, &[Spatial2d, TextDocument]),
        // The occupancy grid renders as a grayscale `rerun::Image`, so it
        // joins the SAME 2D family (a map is a picture, not 3D geometry).
        // A grid with no `info` dimensions / a short cell buffer degrades.
        (A::OccupancyGrid, &[Spatial2d, TextDocument]),
        // ONE plot per topic (the Twist-6-panel-explosion fix).
        (A::Scalars, &[TimeSeries]),
        // Numbers AND text → BOTH views (the Odometry/Imu dual-view
        // precedent). `[TimeSeries]` alone is what dropped a vendor status
        // message's string payload while plotting its numeric envelope.
        (A::ScalarsWithText, &[TimeSeries, TextDocument]),
        (A::SportModeState, &[TimeSeries]),
        // Rerun 0.34's blueprint API ships no TextLogView, so TextLog
        // maps to text_document — and the sink's TextLog arm now ALSO logs a
        // TextDocument mirror (see archetype_components below), so the view is
        // genuinely displayable, not empty.
        (A::TextLog, &[TextDocument]),
        (A::AnyValues, &[TextDocument]),
        // A decoded element array's polyline / point set is 3D geometry —
        // beside the cloud in the Scene, never a 2D pane (the LaserScan lesson).
        // An ABSENT / undecodable element array degrades to the dump.
        (A::Path3D, &[Spatial3d, TextDocument]),
        (A::PoseArray3D, &[Spatial3d, TextDocument]),
        // The viewer decodes each H.264 sample into a picture, so a video
        // topic lands in the SAME 2D family a JPEG camera does.
        // A render-side rescan disagreement degrades to the dump.
        (A::VideoStream, &[Spatial2d, TextDocument]),
        // Markers are 3D primitives on CHILD entities of the topic entity,
        // and a spatial3d view rooted at an origin includes its subtree — so ONE
        // view covers every marker with no extra blueprint work.
        // An absent `markers` array degrades to the dump.
        (A::MarkerArray, &[Spatial3d, TextDocument]),
    ];
    assert_covers_every_archetype(oracle);
    for (kind, expected) in oracle {
        assert_eq!(
            views_for_archetype(*kind),
            *expected,
            "{kind:?} → view kinds"
        );
    }
}

#[test]
fn view_kind_wire_strings_are_the_protocol_vocabulary() {
    // The daemon serializes these strings into the enriched responses; pin them
    // as the exact wire vocabulary Studio parses.
    assert_eq!(ViewKind::Spatial3d.as_str(), "spatial3d");
    assert_eq!(ViewKind::Spatial2d.as_str(), "spatial2d");
    assert_eq!(ViewKind::TimeSeries.as_str(), "time_series");
    assert_eq!(ViewKind::TextDocument.as_str(), "text_document");
    // The composed answer the daemon sends for an Odometry topic.
    let wire: Vec<&str> = views_for_archetype(ArchetypeKind::Odometry)
        .iter()
        .map(|v| v.as_str())
        .collect();
    assert_eq!(wire, vec!["spatial3d", "time_series"]);
}

// ── 3. archetype → rerun component families (exhaustive) ──────────────────────

#[test]
fn archetype_components_pins_the_rendered_families_for_every_variant() {
    use ArchetypeKind as A;
    let oracle: &[(A, &[&str])] = &[
        (A::Points3D, &["Points3D"]),
        (A::Point3D, &["Points3D"]),
        (A::LaserScan, &["Points3D"]),
        (A::Image, &["EncodedImage", "Image"]),
        // A plain TF topic logs ONLY Transform3D (single pose or the TF tree).
        (A::Transforms, &["Transform3D"]),
        (A::Transform3D, &["Transform3D"]),
        // The URDF skeleton render arm logs FOUR families — per-joint
        // Transform3D, joint-marker Points3D, bone LineStrips3D, mesh Asset3D —
        // NOT a single Transform3D.
        (
            A::Skeleton,
            &["Transform3D", "Points3D", "LineStrips3D", "Asset3D"],
        ),
        (A::Scalars, &["Scalars"]),
        // The twin logs the plots AND the structured dump on every
        // frame — understating this as `["Scalars"]` would tell the agent the
        // text half is not rendered.
        (A::ScalarsWithText, &["Scalars", "TextDocument"]),
        (A::SportModeState, &["Scalars"]),
        // Odometry / Imu log a moving transform AND the plotted scalars.
        (A::Imu, &["Transform3D", "Scalars"]),
        (A::Odometry, &["Transform3D", "Scalars"]),
        // The same two families for an inferred transform-shape
        // carrying sibling telemetry; its point twin logs `Points3D` + `Scalars`.
        (A::Transform3DWithScalars, &["Transform3D", "Scalars"]),
        (A::Point3DWithScalars, &["Points3D", "Scalars"]),
        // The TextLog arm logs a TextLog line AND a rolling-latest
        // TextDocument mirror (so its `text_document` view is displayable).
        (A::TextLog, &["TextLog", "TextDocument"]),
        (A::AnyValues, &["TextDocument"]),
        // ONE family, logged at each rendition's
        // `viz-video/<WxH>` child (a `spatial2d` view rooted at the topic includes
        // the subtree). The family is `Image`, NOT `VideoStream`: the viz pipeline moved the
        // DECODE to the desk, because rerun's viewer-side H.264 backend spawns an
        // `ffmpeg` whose default frame threading measured 567 ms of lag. What goes
        // on the wire is now a decoded picture, replaced at its entity on arrival.
        (A::VideoStream, &["Image"]),
        // One oriented box instance per frame; the occupancy grid is a
        // RAW grayscale `Image` (never an `EncodedImage` — nothing is compressed).
        (A::Boxes3D, &["Boxes3D"]),
        (A::OccupancyGrid, &["Image"]),
        // An ORDERED element array logs the polyline AND its vertices (the
        // vertices ride the `<entity>/viz-vertices` child, so a spatial3d view rooted
        // at the topic shows both) — understating it as `["LineStrips3D"]` would
        // tell the agent the waypoints are not rendered. An UNORDERED one logs ONE
        // multi-instance `Points3D`: the same family as a single point, N positions.
        (A::Path3D, &["LineStrips3D", "Points3D"]),
        (A::PoseArray3D, &["Points3D"]),
        // The marker render arm is a KIND SWITCH, so the accurate
        // archetype-level answer is the UNION — every marker logs its pose
        // `Transform3D` plus exactly one geometry family, and `Clear` rides the
        // DELETE / DELETEALL path. Which subset a given topic uses depends on its
        // data, which this mapping deliberately does not read.
        (
            A::MarkerArray,
            &[
                "Transform3D",
                "Arrows3D",
                "Boxes3D",
                "Ellipsoids3D",
                "Cylinders3D",
                "LineStrips3D",
                "Points3D",
                "Mesh3D",
                "Clear",
            ],
        ),
    ];
    assert_covers_every_archetype(oracle);
    for (kind, expected) in oracle {
        assert_eq!(
            archetype_components(*kind),
            *expected,
            "{kind:?} → component families"
        );
    }
}

// ── 4. The new archetypes' placement + wire vocabulary ────────────────────────

#[test]
fn archetype_wire_name_pins_every_variants_protocol_string() {
    use ArchetypeKind as A;
    // The protocol strings Studio parses (the daemon serializes these into every
    // enriched response and the guardrail-2 error). Unpinned, a rename of any
    // variant is a SILENT protocol
    // break for the Studio parser. Hand oracle, one row per variant, with the
    // ALL-totality guard making a new variant fail here until it is given
    // its wire string on purpose.
    let oracle: &[(A, &str)] = &[
        (A::Points3D, "Points3D"),
        (A::Image, "Image"),
        (A::Transforms, "Transforms"),
        (A::Scalars, "Scalars"),
        (A::ScalarsWithText, "ScalarsWithText"),
        (A::Transform3D, "Transform3D"),
        (A::Point3D, "Point3D"),
        (A::Transform3DWithScalars, "Transform3DWithScalars"),
        (A::Point3DWithScalars, "Point3DWithScalars"),
        (A::Imu, "Imu"),
        (A::Odometry, "Odometry"),
        (A::LaserScan, "LaserScan"),
        (A::SportModeState, "SportModeState"),
        (A::TextLog, "TextLog"),
        (A::Boxes3D, "Boxes3D"),
        (A::OccupancyGrid, "OccupancyGrid"),
        (A::Skeleton, "Skeleton"),
        (A::Path3D, "Path3D"),
        (A::PoseArray3D, "PoseArray3D"),
        (A::VideoStream, "VideoStream"),
        (A::MarkerArray, "MarkerArray"),
        (A::AnyValues, "AnyValues"),
    ];
    assert_covers_every_archetype(oracle);
    for (kind, expected) in oracle {
        assert_eq!(
            cerulion_viz::blueprint::archetype_wire_name(*kind),
            *expected,
            "{kind:?} → wire name"
        );
    }
    // Distinct strings: two variants may never collide on the wire.
    let distinct: std::collections::BTreeSet<&str> = oracle.iter().map(|(_, n)| *n).collect();
    assert_eq!(distinct.len(), oracle.len(), "wire names must be distinct");
}

// ── Nothing may render nothing ──────────────────────────────────────

#[test]
fn degradable_archetypes_are_exactly_the_hand_oracle_set() {
    // The SINGLE SOURCE OF TRUTH (`ArchetypeKind::can_degrade_to_dump`) pinned
    // against a HAND classification of every variant — the drift guard the
    // const assertion in `blueprint` cannot provide.
    //
    // Why BOTH are needed: the const assertion proves the predicate and the view
    // table AGREE, so mutating either one alone fails the BUILD. It cannot notice
    // a co-ordinated edit that flips a kind in the predicate AND drops its view —
    // exactly what a future arm-author does when they add a fallback and "fix" the
    // resulting build error the wrong way. This oracle is what fails then.
    //
    // The `expected` match is EXHAUSTIVE with no wildcard arm, so a NEW archetype
    // variant cannot compile until it is classified here on purpose.
    use ArchetypeKind as A;
    for kind in A::ALL {
        let expected = match kind {
            // ── CAN degrade: one entry per production fallback site in
            // `sink::render_classified` (and the two helpers it calls).
            // A raw buffer whose `encoding` is not decoded.
            A::Image
            // A grid with no `info` dimensions / a cell buffer shorter than w*h.
            | A::OccupancyGrid
            // No extractable single-box geometry AND no decodable detection set.
            | A::Boxes3D
            // No decodable element array (absent, or opaque element bytes).
            | A::Path3D
            | A::PoseArray3D
            // No decodable `markers` array.
            | A::MarkerArray
            // The render-side H.264 rescan disagreed with the classifier.
            | A::VideoStream
            // The skeleton is inert without a URDF — and that is now EVERY
            // run, so this arm dumps on every frame in production.
            | A::Skeleton
            // The dump IS its render.
            | A::AnyValues => true,
            // ── CANNOT: these arms always log their own archetype (or, for a
            // Scalars/TextLog frame, whatever series/line the frame carries).
            A::Points3D
            | A::Transforms
            | A::Scalars
            // Its dump is a PRIMARY render (every frame), not a
            // degradation — it earns the text view through
            // `renders_text_document`, not through this predicate.
            | A::ScalarsWithText
            | A::Transform3D
            | A::Point3D
            | A::Transform3DWithScalars
            | A::Point3DWithScalars
            | A::Imu
            | A::Odometry
            | A::LaserScan
            | A::SportModeState
            | A::TextLog => false,
        };
        assert_eq!(
            kind.can_degrade_to_dump(),
            expected,
            "{kind:?}: can_degrade_to_dump() disagrees with the hand oracle. If a render arm \
             GAINED a field-dump fallback, add it here AND to views_for_archetype's \
             text_document set; if one LOST its fallback, remove it from both"
        );
    }
}

#[test]
fn every_archetype_that_can_dump_has_a_text_document_view() {
    // THE CONTRACT, spelled as an IFF over the closed archetype set —
    // as EXECUTABLE DOCUMENTATION, not as a drift guard.
    //
    // SCOPE: the same equality is a `const` assertion in
    // `blueprint` (`VIEWS_MATCH_TEXT_DOCUMENT_CONTRACT`), so violating it is a
    // COMPILE error and this test can never be the thing that reports it — a
    // variant that breaks the IFF never produces a runnable binary. What it buys
    // is a readable statement of the rule at the public API, in the file a
    // reader opens to learn the mapping.
    //
    // The REAL drift guard is `degradable_archetypes_are_exactly_the_hand_oracle_set`
    // above: it pins the predicate against a HAND classification, which is what
    // catches the co-ordinated edit (flip a kind in the predicate AND drop its
    // view) that satisfies the const assertion and is therefore invisible here.
    //
    // Forward: a kind whose render arm can degrade logs a `TextDocument` at the
    // topic entity, so a view that displays one must exist — otherwise the dump
    // renders NOWHERE and the operator sees an empty pane with no diagnostic.
    //
    // Reverse (the layout-cleanliness half): a kind that can never dump must NOT
    // carry the view, or every one of its topics gets a permanently empty status
    // pane in the sidebar grid.
    //
    // `renders_text_document` is the union of "can degrade" and the kinds whose
    // PRIMARY render is text, so the assertion needs no exception list.
    for kind in ArchetypeKind::ALL {
        let has_view = views_for_archetype(kind).contains(&ViewKind::TextDocument);
        assert_eq!(
            has_view,
            kind.renders_text_document(),
            "{kind:?}: views_for_archetype text_document presence ({has_view}) must equal \
             renders_text_document() ({})",
            kind.renders_text_document()
        );
        if kind.can_degrade_to_dump() {
            assert!(
                has_view,
                "{kind:?} can degrade to the AnyValues field dump, so its dump would render \
                 NOWHERE without a text_document view"
            );
        }
    }
}

#[test]
fn a_degradable_archetype_keeps_its_primary_views_alongside_the_dump_view() {
    // ANTI-TAUTOLOGY: the previous test is satisfied by giving every degradable
    // kind ONLY a text_document view, which would delete the geometry. Pin that
    // the dump view is ADDITIVE — the primary family is still first, and the
    // status pane is appended.
    use ArchetypeKind as A;
    use ViewKind::{Spatial2d, Spatial3d, TextDocument};
    // Hand oracle: (kind, the view its geometry renders in).
    for (kind, primary) in [
        (A::Image, Spatial2d),
        (A::OccupancyGrid, Spatial2d),
        (A::VideoStream, Spatial2d),
        (A::Skeleton, Spatial3d),
        (A::Boxes3D, Spatial3d),
        (A::Path3D, Spatial3d),
        (A::PoseArray3D, Spatial3d),
        (A::MarkerArray, Spatial3d),
    ] {
        assert_eq!(
            views_for_archetype(kind),
            &[primary, TextDocument],
            "{kind:?} must keep its primary view FIRST and append the dump view"
        );
    }
    // `AnyValues` is the one degradable kind whose dump IS its primary render, so
    // it stays single-view — the control that keeps the rule above from being
    // read as "every degradable kind has two views".
    assert_eq!(views_for_archetype(A::AnyValues), &[TextDocument]);
}

#[test]
fn a_non_degradable_archetype_is_never_handed_an_empty_status_pane() {
    // The cleanliness half, spelled as its own oracle over the kinds an operator
    // would most notice: a healthy cloud / plot / pose topic must not sprout a
    // status pane that can never contain anything.
    use ArchetypeKind as A;
    for kind in [
        A::Points3D,
        A::LaserScan,
        A::Transforms,
        A::Transform3D,
        A::Point3D,
        A::Imu,
        A::Odometry,
        A::Scalars,
        A::SportModeState,
    ] {
        assert!(
            !views_for_archetype(kind).contains(&ViewKind::TextDocument),
            "{kind:?} can never log a TextDocument, so a text_document view would be a \
             permanently empty pane"
        );
    }
}

#[test]
fn every_archetype_lands_in_a_real_view() {
    // No archetype may resolve to ZERO views — that would be a topic the layout
    // compiler can classify but never place (the inert-view failure mode).
    for kind in ArchetypeKind::ALL {
        assert!(
            !views_for_archetype(kind).is_empty(),
            "{kind:?} must resolve to at least one view"
        );
        assert!(
            !archetype_components(kind).is_empty(),
            "{kind:?} must produce at least one rerun component family"
        );
    }
}

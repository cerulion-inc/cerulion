// SPDX-License-Identifier: AGPL-3.0-only
//! The entity path must be a function of the TOPIC IDENTITY, which is
//! unique by construction — not of the topic's last path segment.
//!
//! # The bug this file pins
//!
//! Measured on the LIVE Go2 (`vizd discover`, robot `ubuntu`, 75 topics):
//! **6 entity paths covered 38 topics.**
//!
//! ```text
//! world/odom/base/response        <- 15 topics  (/api/audiohub/response, /api/bashrunner/response, …)
//! world/odom/base/request         <- 14 topics  (/api/audiohub/request, …)
//! world/odom/base/sportmodestate  <-  3 topics  (/lf/sportmodestate, /mf/sportmodestate, /sportmodestate)
//! world/odom/base/state           <-  2 topics  (/audiohub/player/state, /rtc/state)
//! world/odom/base/lowstate        <-  2 topics  (/lf/lowstate, /lowstate)
//! world/odom/base/odom            <-  2 topics  (/uslam/frontend/odom, /uslam/localization/odom)
//! ```
//!
//! Checking two of those topics in the sidebar drew them into the SAME place in
//! the scene, each overwriting the other, with no indication anything was
//! wrong — a silent wrong answer, not a visible failure. For the two
//! `nav_msgs/Odometry` topics that is a genuinely wrong SPATIAL answer (both
//! logged a `Transform3D` to one entity).
//!
//! # What is asserted, and against what
//!
//! The corpus is the REAL robot's topic list, transcribed here as a HAND
//! literal ([`GO2_TOPICS`], 114 topics from `research/go2_topics.txt`,
//! the superset from which the live 75 were drawn — it contains every one of
//! the 6 collision groups above). The oracle is a HAND-WRITTEN expected-entity
//! table ([`HAND_ORACLE`]) written out per topic, never derived from the
//! function under test.
//!
//! `world/odom/base` and friends are re-derived here as LITERALS rather than
//! read from `cerulion_viz::tf`'s constants, so a constant change is CAUGHT
//! rather than silently followed.
//!
//! # The second half: topic vs FRAME
//!
//! Topic-vs-topic uniqueness is only half the property — the transform tree
//! writes to entities of its own, and a topic that lands on one of those is the
//! same silent wrong answer. That half is
//! [`a_topic_can_never_land_on_a_transform_tree_frame_entity`], which asserts
//! the headline `/odom` pair as HAND literals on BOTH sides and then proves
//! TOTALITY against `cerulion_viz::tf`'s reserved-subtree constants (a name
//! blacklist would reproduce the very miss it fixes — nothing enumerated
//! `/odom`).

use std::collections::{BTreeMap, BTreeSet};

use cerulion_viz::archetype::PATH_VERTICES_CHILD;
use cerulion_viz::sink::{route_for_input, route_key_for_topic, SWEEP_CHILD};
use cerulion_viz::skeleton::ROBOT_ROOT;
use cerulion_viz::tf::{
    entity_path_for_frame, BASE_ENTITY, CAMERA_ENTITY, FRAME_ROOT, LIDAR_ENTITY, ODOM_ENTITY,
};

/// Resolve an ABSOLUTE topic (no override) all the way to its render entity —
/// exactly what the daemon does for a discovered/tapped topic.
fn entity_of(topic: &str) -> String {
    route_for_input(&route_key_for_topic(topic, None)).entity
}

/// The REAL Go2 topic list (`research/go2_topics.txt`), transcribed as a
/// hand literal so this test needs no external fixture. 114 topics; the live
/// 75-topic `discover` set is a subset, and every one of the 6 measured
/// collision groups is present here.
const GO2_TOPICS: &[&str] = &[
    "/Laser_map",
    "/api/audiohub/request",
    "/api/audiohub/response",
    "/api/bashrunner/request",
    "/api/bashrunner/response",
    "/api/config/request",
    "/api/config/response",
    "/api/fourg_agent/request",
    "/api/fourg_agent/response",
    "/api/gas_sensor/request",
    "/api/gas_sensor/response",
    "/api/gpt/request",
    "/api/gpt/response",
    "/api/motion_switcher/request",
    "/api/motion_switcher/response",
    "/api/obstacles_avoid/request",
    "/api/obstacles_avoid/response",
    "/api/programming_actuator/request",
    "/api/programming_actuator/response",
    "/api/robot_state/request",
    "/api/robot_state/response",
    "/api/sport/request",
    "/api/sport/response",
    "/api/sport_lease/request",
    "/api/sport_lease/response",
    "/api/uwbswitch/request",
    "/api/uwbswitch/response",
    "/api/videohub/request",
    "/api/videohub/response",
    "/api/vui/request",
    "/api/vui/response",
    "/arm_Command",
    "/arm_Feedback",
    "/audiohub/player/state",
    "/audioreceiver",
    "/audiosender",
    "/client_count",
    "/clock",
    "/cloud_effected",
    "/cloud_registered_body",
    "/config_change_status",
    "/connected_clients",
    "/frontvideostream",
    "/gas_sensor",
    "/gnss",
    "/gpt_cmd",
    "/gptflowfeedback",
    "/lf/lowstate",
    "/lf/sportmodestate",
    "/lio_sam_ros2/mapping/odometry",
    "/lowcmd",
    "/lowstate",
    "/mf/sportmodestate",
    "/multiplestate",
    "/parameter_events",
    "/path",
    "/pctoimage_local",
    "/planner_normal",
    "/programming_actuator/command",
    "/programming_actuator/feedback",
    "/public_network_status",
    "/qt_add_edge",
    "/qt_add_node",
    "/qt_command",
    "/qt_notice",
    "/query_result_edge",
    "/query_result_node",
    "/registered_scan",
    "/rosout",
    "/rtc/state",
    "/rtc_status",
    "/selftest",
    "/servicestate",
    "/servicestateactivate",
    "/sportmodestate",
    "/state_estimation",
    "/tf",
    "/tf_static",
    "/uslam/client_command",
    "/uslam/cloud_map",
    "/uslam/frontend/cloud_world_ds",
    "/uslam/frontend/odom",
    "/uslam/localization/cloud_world",
    "/uslam/localization/odom",
    "/uslam/navigation/global_path",
    "/uslam/server_log",
    "/utlidar/cloud",
    "/utlidar/cloud_deskewed",
    "/utlidar/foot_position",
    "/utlidar/grid_map",
    "/utlidar/height_map",
    "/utlidar/height_map_array",
    "/utlidar/imu",
    "/utlidar/lidar_state",
    "/utlidar/map_state",
    "/utlidar/mapping_cmd",
    "/utlidar/range_info",
    "/utlidar/range_map",
    "/utlidar/robot_odom",
    "/utlidar/robot_pose",
    "/utlidar/switch",
    "/utlidar/transformed_cloud",
    "/utlidar/transformed_imu",
    "/utlidar/transformed_raw_imu",
    "/utlidar/voxel_map",
    "/utlidar/voxel_map_compressed",
    "/uwbstate",
    "/uwbswitch",
    "/videohub/inner",
    "/webrtcreq",
    "/webrtcres",
    "/wireless_controller",
    "/wirelesscontroller",
    "/wirelesscontroller_unprocessed",
];

/// The HEADLINE hand oracle: the exact entity every Go2 topic must render at,
/// written out by hand (never derived from the function under test). `/tf` and
/// `/tf_static` are the two deliberate exceptions — a TFMessage renders as the
/// transform TREE at per-frame entities, so its reported entity is the viz root.
const HAND_ORACLE: &[(&str, &str)] = &[
    // The three sportmodestate topics: three genuinely different signals
    // (low-frequency, medium-frequency, raw) a user would
    // reasonably compare side by side. Before the entity-path fix all three were ONE entity.
    ("/lf/sportmodestate", "world/lf/sportmodestate"),
    ("/mf/sportmodestate", "world/mf/sportmodestate"),
    ("/sportmodestate", "world/sportmodestate"),
    // The 15-way request/response collapse, spot-checked at both ends.
    ("/api/audiohub/request", "world/api/audiohub/request"),
    ("/api/audiohub/response", "world/api/audiohub/response"),
    ("/api/vui/request", "world/api/vui/request"),
    ("/api/vui/response", "world/api/vui/response"),
    // The two-way collisions.
    ("/audiohub/player/state", "world/audiohub/player/state"),
    ("/rtc/state", "world/rtc/state"),
    ("/lf/lowstate", "world/lf/lowstate"),
    ("/lowstate", "world/lowstate"),
    ("/uslam/frontend/odom", "world/uslam/frontend/odom"),
    ("/uslam/localization/odom", "world/uslam/localization/odom"),
    // Media topics: the earlier fold onto `world/odom/base/lidar` /
    // `.../camera` is GONE — each keeps its own topic path.
    ("/utlidar/cloud", "world/utlidar/cloud"),
    ("/utlidar/cloud_deskewed", "world/utlidar/cloud_deskewed"),
    ("/uslam/cloud_map", "world/uslam/cloud_map"),
    ("/Laser_map", "world/Laser_map"),
    // Top-level topics are one segment under the root.
    ("/path", "world/path"),
    ("/clock", "world/clock"),
    ("/lowcmd", "world/lowcmd"),
    // Deeply nested topics keep EVERY segment.
    (
        "/lio_sam_ros2/mapping/odometry",
        "world/lio_sam_ros2/mapping/odometry",
    ),
    (
        "/uslam/frontend/cloud_world_ds",
        "world/uslam/frontend/cloud_world_ds",
    ),
    (
        "/uslam/navigation/global_path",
        "world/uslam/navigation/global_path",
    ),
    // The TF pair: the tree, not a per-topic entity.
    ("/tf", "world"),
    ("/tf_static", "world"),
];

#[test]
fn the_go2_topic_corpus_maps_to_the_hand_oracle() {
    for (topic, expected) in HAND_ORACLE {
        assert_eq!(&entity_of(topic), expected, "{topic} → entity");
    }
    // The oracle rows must be REAL topics from the corpus (a typo'd oracle row
    // would otherwise assert a hypothetical).
    let corpus: BTreeSet<&str> = GO2_TOPICS.iter().copied().collect();
    for (topic, _) in HAND_ORACLE {
        assert!(
            corpus.contains(topic),
            "{topic} is not in the transcribed Go2 corpus — fix the oracle"
        );
    }
}

#[test]
fn every_go2_topic_gets_a_distinct_entity_zero_collisions() {
    // THE contract: 114 topics → 114 distinct entities. `/tf` + `/tf_static`
    // deliberately SHARE the transform-tree root (neither renders at a per-topic
    // entity), so they are excluded and counted separately.
    let mut by_entity: BTreeMap<String, Vec<&str>> = BTreeMap::new();
    for topic in GO2_TOPICS {
        if matches!(*topic, "/tf" | "/tf_static") {
            continue;
        }
        by_entity.entry(entity_of(topic)).or_default().push(topic);
    }
    let collisions: Vec<(&String, &Vec<&str>)> =
        by_entity.iter().filter(|(_, ts)| ts.len() > 1).collect();
    assert!(
        collisions.is_empty(),
        "entity paths must be unique by construction; collisions: {collisions:#?}"
    );
    assert_eq!(
        by_entity.len(),
        GO2_TOPICS.len() - 2,
        "every non-TF topic must own exactly one entity"
    );
    // And the TF pair reports the root.
    assert_eq!(entity_of("/tf"), "world");
    assert_eq!(entity_of("/tf_static"), "world");
}

/// The anti-tautology arm: the SIX measured collision groups, each asserted to
/// be genuinely separated now. Written as the collapsed-entity → topics table
/// from the tracked issue, so this test states the bug and its fix in one place.
#[test]
fn the_six_measured_collision_groups_are_separated() {
    // (the collapsed entity a last-segment key yields, the topics that collapse onto it)
    let measured: &[(&str, &[&str])] = &[
        (
            "world/tf-tree/odom/base/response",
            &[
                "/api/audiohub/response",
                "/api/bashrunner/response",
                "/api/config/response",
                "/api/fourg_agent/response",
                "/api/gas_sensor/response",
                "/api/gpt/response",
                "/api/motion_switcher/response",
                "/api/obstacles_avoid/response",
                "/api/programming_actuator/response",
                "/api/robot_state/response",
                "/api/sport/response",
                "/api/sport_lease/response",
                "/api/uwbswitch/response",
                "/api/videohub/response",
                "/api/vui/response",
            ],
        ),
        (
            "world/tf-tree/odom/base/request",
            &[
                "/api/audiohub/request",
                "/api/bashrunner/request",
                "/api/config/request",
                "/api/fourg_agent/request",
                "/api/gas_sensor/request",
                "/api/gpt/request",
                "/api/motion_switcher/request",
                "/api/obstacles_avoid/request",
                "/api/programming_actuator/request",
                "/api/robot_state/request",
                "/api/sport/request",
                "/api/sport_lease/request",
                "/api/uwbswitch/request",
                "/api/videohub/request",
                "/api/vui/request",
            ],
        ),
        (
            "world/tf-tree/odom/base/sportmodestate",
            &[
                "/lf/sportmodestate",
                "/mf/sportmodestate",
                "/sportmodestate",
            ],
        ),
        (
            "world/tf-tree/odom/base/state",
            &["/audiohub/player/state", "/rtc/state"],
        ),
        (
            "world/tf-tree/odom/base/lowstate",
            &["/lf/lowstate", "/lowstate"],
        ),
        (
            "world/tf-tree/odom/base/odom",
            &["/uslam/frontend/odom", "/uslam/localization/odom"],
        ),
    ];
    // Sanity: the transcription reproduces the measured group sizes (15 response
    // + 15 request here vs the live 15/14 — the live set omitted one request).
    assert_eq!(measured.len(), 6, "six measured collision groups");
    // 15 response + 15 request + 3 + 2 + 2 + 2 = 39 here; the LIVE 75-topic set
    // measured 38 because it carried only 14 of the 15 request topics.
    let covered: usize = measured.iter().map(|(_, ts)| ts.len()).sum();
    assert_eq!(covered, 39, "the six groups cover 39 topics in this corpus");

    for (old_entity, topics) in measured {
        let now: BTreeSet<String> = topics.iter().map(|t| entity_of(t)).collect();
        assert_eq!(
            now.len(),
            topics.len(),
            "{old_entity}: {} topics must yield {} DISTINCT entities, got {now:#?}",
            topics.len(),
            topics.len()
        );
        // None of them may still land on the old collapsed path.
        for t in *topics {
            assert_ne!(
                &entity_of(t),
                old_entity,
                "{t} must no longer render at the collapsed {old_entity}"
            );
        }
    }
}

/// Leaf-and-parent (`/a` alongside `/a/b`) is ALLOWED with no disambiguation:
/// rerun renders a node carrying both data and children as one row with a
/// collapsing triangle, and we already ship the extreme case (`world/odom/base`
/// carries a `Transform3D` while parenting the whole tree). Zero occurrences on
/// the Go2; the canonical ROS `image_transport` family hits it.
#[test]
fn a_parent_topic_and_its_child_topic_nest_without_disambiguation() {
    assert_eq!(entity_of("/camera/image_raw"), "world/camera/image_raw");
    assert_eq!(
        entity_of("/camera/image_raw/compressed"),
        "world/camera/image_raw/compressed"
    );
    assert_eq!(
        entity_of("/camera/image_raw/theora"),
        "world/camera/image_raw/theora"
    );
    // Distinct entities, and the child is a strict descendant of the parent
    // (which is what the blueprint's per-topic view contents must account for —
    // see `layout_default_test`'s exclusion arms).
    let parent = entity_of("/camera/image_raw");
    for child in ["/camera/image_raw/compressed", "/camera/image_raw/theora"] {
        let c = entity_of(child);
        assert_ne!(c, parent);
        assert!(
            c.starts_with(&format!("{parent}/")),
            "{c} must be a strict descendant of {parent}"
        );
    }
    // Before the entity-path fix this family was mishandled WORSE than a collision: the
    // compressed stream's last segment `compressed` hit the camera media arm and
    // landed on `world/odom/base/camera` while the raw image went to
    // `world/odom/base/image_raw` — a silent semantic MISPLACEMENT.
    assert_ne!(
        entity_of("/camera/image_raw/compressed"),
        "world/tf-tree/odom/base/camera"
    );
}

/// Sanitization edges, end to end through the real derivation. The aliasing
/// suffix is what keeps two raw names that differ only in separators apart —
/// without it the derivation would trade one collision class for another.
#[test]
fn sanitization_edges_are_total_and_collision_safe() {
    // Non-identifier characters in a segment sanitize to `_` + a 4-hex FNV
    // suffix of the RAW segment.
    assert_eq!(entity_of("/weird.name"), "world/weird_name_198b");
    assert_eq!(entity_of("/a b/c!d"), "world/a_b_3892/c_d_2bb7");
    // A single segment containing `.` vs one containing a space do NOT collapse.
    assert_ne!(entity_of("/a.b"), entity_of("/a b"));
    // Multi-byte characters map per-char and stay distinct from a same-shape
    // ASCII name.
    assert_eq!(entity_of("/naïve"), "world/na_ve_32ab");
    assert_ne!(entity_of("/naïve"), entity_of("/na_ve"));
    assert_eq!(entity_of("/na_ve"), "world/na_ve");
    // Degenerate topics never panic and never claim the transform-tree root.
    for topic in ["/", "", "//"] {
        assert_eq!(entity_of(topic), "world/unknown_2325", "{topic:?}");
        assert_ne!(entity_of(topic), "world");
    }
    // A trailing slash is trimmed, so it is the SAME entity as without.
    assert_eq!(entity_of("/robot/odom/"), entity_of("/robot/odom"));
    // An interior empty segment is dropped rather than yielding an empty
    // path element (`world//x` would be an invalid rerun path).
    assert_eq!(entity_of("/a//b"), "world/a/b");
}

/// **The TOPIC-vs-FRAME half of uniqueness.**
///
/// Every other test in this file checks topic-vs-TOPIC. That is only half the
/// property: the transform tree writes `Transform3D`s to its OWN entities, and
/// those used to sit directly under `world` — so a topic literally named
/// `/odom` (the canonical `nav_msgs/Odometry` name on essentially every nav2
/// robot) resolved to `world/odom`, which is EXACTLY where `/tf` logs its
/// `odom` child transform. Two unrelated transforms on one entity, each
/// overwriting the other under latest-at: the same silent wrong answer this
/// file exists to eliminate, invisible to a topic-vs-topic corpus check
/// (`GO2_TOPICS` has `/uslam/frontend/odom` but no bare `/odom`).
///
/// Both sides are asserted, so this cannot pass by one side moving.
#[test]
fn a_topic_can_never_land_on_a_transform_tree_frame_entity() {
    // The headline pair, asserted on BOTH sides by hand.
    assert_eq!(entity_of("/odom"), "world/odom");
    assert_eq!(entity_path_for_frame("odom").path, "world/tf-tree/odom");
    assert_ne!(entity_of("/odom"), entity_path_for_frame("odom").path);

    // The sibling names that shared the same hazard.
    assert_eq!(entity_of("/odom/base"), "world/odom/base");
    assert_eq!(
        entity_path_for_frame("base_link").path,
        "world/tf-tree/odom/base"
    );
    assert_ne!(entity_of("/odom/base"), BASE_ENTITY);
    assert_eq!(entity_of("/robot"), "world/robot");
    assert_ne!(entity_of("/robot"), ROBOT_ROOT);

    // TOTALITY, not a name list: every reserved frame entity — the alias table,
    // the unknown-frame fallback, the URDF skeleton root — lives under the
    // reserved `FRAME_ROOT` segment, and NO topic-derived entity can reach it,
    // because the sanitizer's output alphabet excludes that segment's `-`
    // (proved in `cerulion_viz::tf`'s alphabet test).
    let reserved: Vec<String> = [ODOM_ENTITY, BASE_ENTITY, LIDAR_ENTITY, CAMERA_ENTITY]
        .iter()
        .map(|s| s.to_string())
        .chain(std::iter::once(ROBOT_ROOT.to_string()))
        .chain(std::iter::once(entity_path_for_frame("velodyne_link").path))
        .collect();
    let reserved_prefix = format!("{FRAME_ROOT}/");
    for r in &reserved {
        assert!(
            r.starts_with(&reserved_prefix),
            "{r} must live under the reserved frame root {FRAME_ROOT}"
        );
    }
    // Adversarial topics: the frame names themselves, the reserved segment
    // spelled out, and the whole Go2 corpus.
    let adversarial = [
        "/odom",
        "/odom/base",
        "/odom/base/lidar",
        "/odom/base/camera",
        "/robot",
        "/robot/radar",
        "/tf-tree",
        "/tf-tree/odom",
        "/world/tf-tree/odom",
        "/tf_tree/odom",
    ];
    for topic in adversarial
        .iter()
        .copied()
        .chain(GO2_TOPICS.iter().copied())
    {
        let e = entity_of(topic);
        assert!(
            !e.starts_with(&reserved_prefix) && e != FRAME_ROOT,
            "{topic} resolved to {e}, inside the reserved frame subtree"
        );
        for r in &reserved {
            assert_ne!(&e, r, "{topic} collides with the frame entity {r}");
        }
    }
    // A hand-typed `-` topic segment cannot alias the reserved segment either:
    // the sanitizer rewrites `-` to `_` AND suffixes it.
    assert_eq!(entity_of("/tf-tree"), "world/tf_tree_df1a");
    assert_ne!(entity_of("/tf-tree"), FRAME_ROOT);
    // …and neither can an explicit `entity` OVERRIDE, which is routed through
    // the same sanitizer.
    let overridden = route_for_input(&route_key_for_topic("/x", Some("world/tf-tree/odom"))).entity;
    assert_ne!(overridden, ODOM_ENTITY);
    assert!(!overridden.starts_with(&reserved_prefix));
}

/// Stability: the entity is a pure function of the topic, so
/// repeated resolution — the daemon re-derives it on every `attach`/`list`/
/// `discover`/`status` — must be byte-identical.
#[test]
fn entity_resolution_is_stable_across_repeated_derivation() {
    for topic in GO2_TOPICS {
        let first = entity_of(topic);
        for _ in 0..3 {
            assert_eq!(entity_of(topic), first, "{topic} must be stable");
        }
    }
}

/// **The SYNTHETIC children are reserved too.**
///
/// A cloud's geometry lives at `<entity>/viz-sweep/{k}` and a path's waypoints at
/// `<entity>/viz-vertices` — segments the sink SYNTHESIZES under a topic's entity,
/// so they share a namespace with real topics. Under the earlier flat scheme
/// no topic entity could sit under another, so `sweep`/`vertices` were safe; the
/// mechanical `world/<full topic>` rule made a robot publishing BOTH `/plan` and
/// `/plan/vertices` (or `/utlidar/cloud` and `/utlidar/cloud/sweep/0`) land the
/// second topic's data on the first's synthetic child — the same silent overwrite
/// this file exists to eliminate, one level down.
///
/// Same structural argument as `FRAME_ROOT`: the sanitizer emits only
/// `[A-Za-z0-9_]`, so a segment containing `-` is unreachable from any topic name.
#[test]
fn a_topic_can_never_land_on_a_synthesized_child_entity() {
    // Both halves asserted by hand, on BOTH sides.
    assert_eq!(entity_of("/plan/vertices"), "world/plan/vertices");
    assert_eq!(PATH_VERTICES_CHILD, "viz-vertices");
    assert_ne!(
        entity_of("/plan/vertices"),
        format!("{}/{PATH_VERTICES_CHILD}", entity_of("/plan"))
    );
    assert_eq!(SWEEP_CHILD, "viz-sweep");
    assert_ne!(
        entity_of("/utlidar/cloud/sweep/0"),
        format!("{}/{SWEEP_CHILD}/0", entity_of("/utlidar/cloud"))
    );

    // TOTALITY: no topic — adversarial or real — can produce either segment.
    let adversarial = [
        "/plan/vertices",
        "/plan/viz-vertices",
        "/plan/viz_vertices",
        "/utlidar/cloud/sweep/0",
        "/utlidar/cloud/viz-sweep/0",
        "/viz-sweep",
        "/viz-vertices",
    ];
    for topic in adversarial
        .iter()
        .copied()
        .chain(GO2_TOPICS.iter().copied())
    {
        let e = entity_of(topic);
        for reserved in [PATH_VERTICES_CHILD, SWEEP_CHILD] {
            assert!(
                !e.split('/').any(|seg| seg == reserved),
                "{topic} resolved to {e}, which claims the reserved `{reserved}` segment"
            );
        }
    }
    // The reserved segments are outside the sanitizer's alphabet — the proof, not
    // a property of the corpus above.
    for reserved in [PATH_VERTICES_CHILD, SWEEP_CHILD] {
        assert!(
            reserved
                .chars()
                .any(|c| !(c.is_ascii_alphanumeric() || c == '_')),
            "{reserved} must contain a character sanitize_segment cannot emit"
        );
    }
}

// SPDX-License-Identifier: AGPL-3.0-only
//! The shape-inference ladder as recorded-chunk deltas, WITHOUT a
//! transport.
//!
//! This file used to drive `cerulion_viz::archetype::log_frame_value` — a
//! SECOND, `SinkState`-free ladder through the classification tree that existed
//! so a test could assert a render without building a wire frame. That path had
//! no production caller and is retired: every render property it witnessed now
//! rides the PRODUCTION `sink::dispatch_frame` in `sink_dispatch_test.rs`, over
//! a real wire frame, against the same hand oracles. The map, arm by arm:
//!
//! | what it pinned | where it lives now |
//! |---|---|
//! | Imu ⇒ 1 rotation + 6 scalars | `imu_frame_renders_one_rotation_transform_and_six_scalars` |
//! | Odometry ⇒ 1 pose + 6 twist scalars | `odometry_on_odom_input_also_poses_robot_root` leg (b) |
//! | unmapped numeric bag ⇒ one plot per field | `vector3_infers_to_three_scalars`, `time_infers_to_scalar_bag` |
//! | single string ⇒ TextLog + its `/text` mirror | `a_single_string_frame_renders_a_textlog_and_its_text_document_mirror` |
//! | LaserScan ⇒ one Points3D ring | `laserscan_frame_renders_one_points3d_ring` |
//! | stamped element array ⇒ polyline + vertices | `path_frame_renders_a_polyline_and_its_vertices` (mapped), `an_unmapped_stamped_element_array_renders_a_polyline_and_its_vertices` (shape ladder) |
//! | nothing-spatial element array ⇒ one dump | `a_non_spatial_element_array_takes_the_dump_instead_of_drawing_geometry` |
//! | unstamped element array ⇒ one Points3D | `pose_array_frame_renders_one_points3d_not_a_polyline` (mapped), `a_stale_path_memo_cannot_draw_a_polyline_through_an_unordered_array` (shape ladder) |
//! | detection array ⇒ one multi-instance Boxes3D | `detection3d_array_frame_renders_one_multi_instance_boxes3d` |
//! | idle array ⇒ nothing, then draws when it fills | `an_idle_name_mapped_element_array_draws_nothing_then_draws_when_it_fills` |
//! | opaque element bytes ⇒ one dump, never a guess | `an_undecodable_path_array_reports_one_field_naming_line_per_input` |
//! | name-mapped Polygon ⇒ polyline | `polygon_frame_name_mapped_to_path3d_renders_a_polyline_not_points`, `a_polygon_stamped_frame_draws_a_polyline_from_an_array_one_hop_down` |
//!
//! What remains here is the one arm that never went through `log_frame_value`:
//! `log_pose`'s own ladder, driven directly against a hand-built `FrameValue`.

use cerulion_core::codegen::{FrameValue, FrameValueKind, NamedValue};
use cerulion_viz::archetype::log_pose;

fn memory() -> (rerun::RecordingStream, rerun::sink::MemorySinkStorage) {
    rerun::RecordingStreamBuilder::new("go2_test")
        .recording_id("go2_archetype_inference_test")
        .memory()
        .expect("memory sink")
}

fn f64f(name: &str, v: f64) -> NamedValue<'static> {
    NamedValue {
        name: name.to_string(),
        value: FrameValueKind::F64(v),
    }
}

fn nested(name: &str, inner: FrameValue<'static>) -> NamedValue<'static> {
    NamedValue {
        name: name.to_string(),
        value: FrameValueKind::Nested(Box::new(inner)),
    }
}

fn vec3(x: f64, y: f64, z: f64) -> FrameValue<'static> {
    FrameValue {
        schema_name: "geometry_msgs/Vector3".to_string(),
        fields: vec![f64f("x", x), f64f("y", y), f64f("z", z)],
    }
}

/// `log_pose` draws EXACTLY the primitive the ONE spatial
/// ladder resolved, instead of re-deciding through a second ladder of its own.
/// A `{point: {x, y, z}}` value resolves to the ladder's POINT rung — a
/// private `log_pose` ladder that looks only for `position` / `translation`
/// / `theta` / a named orientation draws NOTHING for it (0 chunks). That
/// rung is reachable whenever a NAME-mapped `Transform3D` schema's frame carries
/// no readable orientation, so drawing nothing is a silent miss; the caller
/// asked for a transform, so the named point is drawn as a translation-only
/// one.
#[test]
fn log_pose_draws_the_ladders_point_rung_instead_of_nothing() {
    let (rec, storage) = memory();
    let point_only = FrameValue {
        schema_name: "acme/Waypoint".to_string(),
        fields: vec![nested("point", vec3(1.0, 2.0, 3.0))],
    };
    let base = storage.num_msgs();
    log_pose(&rec, "world/x", 1_000, &point_only);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - base,
        1,
        "a private log_pose ladder draws nothing for a nested `point`"
    );

    // ANTI-TAUTOLOGY: a value with NO spatial primitive at all still draws
    // nothing — the arm above is the ladder's answer, not an unconditional log.
    let battery = FrameValue {
        schema_name: "acme/Battery".to_string(),
        fields: vec![f64f("voltage", 12.4)],
    };
    let base = storage.num_msgs();
    log_pose(&rec, "world/x", 1_000, &battery);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - base,
        0,
        "nothing spatial ⇒ nothing drawn"
    );
}

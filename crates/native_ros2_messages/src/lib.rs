// SPDX-License-Identifier: AGPL-3.0-only
//! ROS 2 message types for Cerulion nodes.
//!
//! 22 packages and 254 messages, generated at build time from the `.msg`
//! files vendored in this crate's `msg/` directory. Most packages follow
//! ROS 2 Jazzy. `control_msgs` follows the Lyrical release, because 13 of its
//! 39 messages do not exist in Jazzy, and the two `autoware_*` packages follow
//! the Autoware message repository, which is not part of a ROS 2 distribution.
//!
//! # Using a message type in a node
//!
//! Import the type and make it the type of a port field. A graph file names
//! the same type as `sensor_msgs/Image`.
//!
//! ```rust
//! use cerulion_core::prelude::*;
//! use native_ros2_messages::sensor_msgs::Image;
//!
//! #[cerulion_node(period_ms = 33)]
//! #[derive(Default)]
//! struct CameraNode {
//!     #[output]
//!     image: Image,
//!     frame_count: u32,
//! }
//!
//! #[cerulion_node_impl]
//! impl CameraNode {
//!     fn tick(&mut self) -> Result<(), NodeError> {
//!         self.frame_count = self.frame_count.wrapping_add(1);
//!         // Fixed fields: plain assignment, straight into shared memory.
//!         self.image.height = 480;
//!         self.image.width = 640;
//!         self.image.step = 640 * 3;
//!         // A variable-length field: assignment costs one copy.
//!         self.image.encoding = "rgb8";
//!         // A leaf of a nested message, by dotted assignment.
//!         self.image.header.frame_id = "camera";
//!         // A variable-length field filled in place, with no copy: the
//!         // producer gets the loaned region and returns how much it wrote.
//!         let shade = (self.frame_count % 256) as u8;
//!         self.image.data.fill_from(|pixels: &mut [u8]| {
//!             let n = pixels.len().min(640 * 480 * 3);
//!             pixels[..n].fill(shade);
//!             Ok(n)
//!         })?;
//!         Ok(())
//!     }
//! }
//! # fn main() {}
//! ```
//!
//! A message type with variable-length fields must have EVERY one of them
//! written in a tick that writes any of it, or the frame is discarded with an
//! `error` log. `Image` has three (`header`, `encoding`, `data`), and the
//! example writes all three.
//!
//! Reading mirrors writing. A fixed field is read as a field; a
//! variable-length field is read through an accessor method of the same name:
//!
//! ```rust
//! use cerulion_core::prelude::*;
//! use native_ros2_messages::sensor_msgs::Image;
//! use native_ros2_messages::std_msgs::Float32;
//!
//! #[cerulion_node]
//! #[derive(Default)]
//! struct BrightnessNode {
//!     #[input(trigger)]
//!     image: Image,
//!     #[output]
//!     mean: Float32,
//! }
//!
//! #[cerulion_node_impl]
//! impl BrightnessNode {
//!     fn tick(&mut self) -> Result<(), NodeError> {
//!         // `data()` borrows the bytes from shared memory. Nothing is copied.
//!         let pixels: &[u8] = self.image.data();
//!         let sum: f32 = pixels.iter().map(|&p| f32::from(p)).sum();
//!         self.mean.data = sum / pixels.len().max(1) as f32;
//!         Ok(())
//!     }
//! }
//! # fn main() {}
//! ```
//!
//! # Nested messages
//!
//! A leaf of a nested message is written by dotted assignment, at any depth.
//! To make several writes into one nested message, the generated
//! `with_<field>` method hands a closure that message's own writer. Inside
//! the closure a variable-length leaf is written through its `set_<leaf>`
//! method, because dotted assignment is only rewritten on `self.<port>`:
//!
//! ```rust
//! use cerulion_core::prelude::*;
//! use native_ros2_messages::geometry_msgs::PoseStamped;
//!
//! #[cerulion_node(period_ms = 100)]
//! #[derive(Default)]
//! struct StampedPoseNode {
//!     #[output]
//!     pose: PoseStamped,
//! }
//!
//! #[cerulion_node_impl]
//! impl StampedPoseNode {
//!     fn tick(&mut self) -> Result<(), NodeError> {
//!         // `header` is a std_msgs/Header: one closure, two leaves.
//!         self.pose.with_header(|h| {
//!             h.set_frame_id("map")?;
//!             h.stamp.sec = 5;
//!             Ok(())
//!         })?;
//!         // `pose` is a fixed-size nested message: dotted assignment lands
//!         // straight in shared memory.
//!         self.pose.pose.position.x = 1.0;
//!         Ok(())
//!     }
//! }
//! # fn main() {}
//! ```
//!
//! Write each nested field by ONE mechanism per tick. Mixing the leaf forms
//! above with a whole-field write of the same field is rejected at run time
//! with `TransportError::NestedWriteConflict`.
//!
//! # Finding what a type offers
//!
//! For every message the build emits three items into its package module:
//!
//! 1. `<Name>`, a unit marker. This is what you import and write as a port
//!    field's type.
//! 2. `<Name>Shm`, the accessor over the message where it lives in shared
//!    memory. Inside `tick`, `self.<port>` dereferences to it, so its page is
//!    where a type's fields, accessor methods and `with_<field>` nested
//!    writers are listed.
//! 3. `<Name>Snapshot`, an owned copy that outlives the shared-memory sample
//!    (`Default + Clone + PartialEq`, plus serde when the schema fits).
//!    Recording and replay tooling uses it; node code rarely needs it.
//!
//! From the command line, `cerulion schema list` prints every built-in type
//! and `cerulion schema info sensor_msgs/Image` prints one type's fields.
//!
//! # Packages
//!
//! `action_msgs`, `autoware_perception_msgs`, `autoware_planning_msgs`,
//! `builtin_interfaces`, `control_msgs`, `diagnostic_msgs`, `geometry_msgs`,
//! `grid_map_msgs`, `moveit_msgs`, `nav_msgs`, `object_recognition_msgs`,
//! `octomap_msgs`, `radar_msgs`, `sensor_msgs`, `shape_msgs`,
//! `statistics_msgs`, `std_msgs`, `tf2_msgs`, `trajectory_msgs`,
//! `unique_identifier_msgs`, `vision_msgs`, `visualization_msgs`.

#![deny(dead_code, unused_imports, unused_variables)]
// Principle #12 (structured logging): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

// Include generated ROS2 message modules
include!(concat!(env!("OUT_DIR"), "/ros2_msgs.rs"));

// Embedded built-in message registry. Brings in `BUILTIN_MSGS`:
// the exact vendored `.msg` text frozen at build time so CLI schema
// introspection can never skew from what the generated types compiled
// against. See the `///` doc on `BUILTIN_MSGS` in the generated file.
include!(concat!(env!("OUT_DIR"), "/msg_registry.rs"));

/// The `schema_hash` → qualified-name binding of EVERY built-in
/// type in [`BUILTIN_MSGS`] — the ONE corpus-bindings source.
///
/// A network gateway NAMES a runtime-registered topic (one that arrives over
/// the reg-channel carrying only `(topic, schema_hash)` — an rmw publisher, a
/// `ros2 attach` raw route) by resolving that hash through a hash→name map handed
/// to it at boot. Without these bindings that map carries CUSTOM types only, so a
/// `std_msgs/String` rmw topic catalogs `schema_name: None` and every desk
/// consumer refuses it (`SchemaUnavailable`). Both the CLI (`build_schema_serving`)
/// and `cerulion-netd`'s start-booted standing gateway now fold THESE bindings
/// into the serving they hand across; the desk then resolves a named built-in
/// from its own compiled corpus with NO served doc (the local-first rule).
///
/// Hashes are the recipe-3 wire hashes (`MessageSchema::schema_hash`), resolved
/// with `resolve_fixed_nested` over the whole corpus so a fixed nested target
/// (e.g. `std_msgs/Header` inside `sensor_msgs/Imu`) folds exactly as the
/// generated type stamps it — pinned against the generated `SCHEMA_HASH`
/// constants below. A built-in never references a custom type, so resolving the
/// corpus against ITSELF is sufficient, not a shortcut. Cost: one parse of the
/// vendored corpus per call — paid once at a daemon/gateway boot, never per
/// frame; callers must not put it on a latency-bounded path.
///
/// Panics on an unparseable built-in: the corpus is frozen at build time and
/// every entry compiled into a generated type, so that is a build defect, not a
/// runtime condition.
pub fn builtin_hash_bindings() -> Vec<::cerulion_core::SchemaHashName> {
    let mut schemas: Vec<::cerulion_core::codegen::MessageSchema> = BUILTIN_MSGS
        .iter()
        .map(|&(package, name, text)| {
            ::cerulion_core::codegen::parse_rosmsg(text, name, Some(package)).unwrap_or_else(|e| {
                panic!("built-in registry: failed to parse {package}/{name}: {e}")
            })
        })
        .collect();
    let _ = ::cerulion_core::codegen::resolve_fixed_nested(&mut schemas);
    schemas
        .iter()
        .map(|s| ::cerulion_core::SchemaHashName {
            schema_hash: s.schema_hash(),
            qualified: s.qualified_name(),
        })
        .collect()
}

// Inline smoke tests for the generated codegen surface. The legacy
// version of this block exercised the deleted heap `<Name>::new()`
// constructor + `wire_size()` byte-count assertions on the legacy wire
// format. Both went away with the SHM-backed wire format; the equivalent property
// "every schema is reachable through its codegen-emitted unit marker
// and round-trips through `Default::default() → Snapshot`" — lives in
// `tests/roundtrip_test.rs` for full per-schema coverage.
//
// The remaining inline tests are structural: they exercise the unit
// marker / Snapshot / ShmMessage triple on a couple of representative
// schemas to catch outright codegen failures (e.g. a missing trait
// impl) without spinning up a publisher.
#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_core::message::ShmMessage;

    /// Sanity check: the unit marker for a fixed-layout schema implements
    /// `ShmMessage` and reports `VARIABLE_FIELD_COUNT == 0`. This catches
    /// codegen regressions where a fixed schema is mis-classified as
    /// variable (which would also break every fixed-only consumer).
    #[test]
    fn fixed_schema_marker_is_reachable() {
        // Vector3: 3 × f64 packed.
        assert_eq!(geometry_msgs::Vector3::VARIABLE_FIELD_COUNT, 0);
        assert_eq!(
            geometry_msgs::Vector3::WIRE_FIXED_SIZE,
            ::std::mem::size_of::<geometry_msgs::Vector3Shm>(),
        );

        // Default snapshot is the zero value. Round-trip through Clone.
        let snap = geometry_msgs::Vector3Snapshot::default();
        assert_eq!(snap.x, 0.0);
        assert_eq!(snap.clone(), snap);
    }

    /// Sanity check: variable-layout schema marker reports a non-zero
    /// `VARIABLE_FIELD_COUNT`, and its Snapshot is constructible at the
    /// `Default` value (codegen promises this for every schema).
    #[test]
    fn variable_schema_marker_is_reachable() {
        // sensor_msgs::Image has 7 declared fields (header + 6 fixed +
        // data); codegen emits the header field as raw bytes (Vec<u8>)
        // because it is a Nested type, plus `encoding` (String) and
        // `data` (DynamicArray<u8>) — 3 variable fields total.
        assert_eq!(sensor_msgs::Image::VARIABLE_FIELD_COUNT, 3);

        let snap = sensor_msgs::ImageSnapshot::default();
        assert!(snap.header.is_empty());
        assert!(snap.encoding.is_empty());
        assert!(snap.data.is_empty());
        assert_eq!(snap.height, 0);
        assert_eq!(snap.width, 0);
    }

    /// Every package re-exports its schemas under a stable module path.
    /// This test references one schema per package so a missing-module
    /// regression surfaces here at the crate level rather than only in
    /// the per-schema integration suite.
    #[test]
    fn all_packages_are_reachable() {
        // One reference per package — compile-time check, no runtime work.
        let _ = action_msgs::GoalStatus::SCHEMA_HASH;
        let _ = builtin_interfaces::Time::SCHEMA_HASH;
        let _ = diagnostic_msgs::KeyValue::SCHEMA_HASH;
        let _ = geometry_msgs::Vector3::SCHEMA_HASH;
        let _ = nav_msgs::Odometry::SCHEMA_HASH;
        let _ = sensor_msgs::Image::SCHEMA_HASH;
        let _ = shape_msgs::Plane::SCHEMA_HASH;
        let _ = statistics_msgs::StatisticDataPoint::SCHEMA_HASH;
        let _ = std_msgs::Header::SCHEMA_HASH;
        let _ = tf2_msgs::TFMessage::SCHEMA_HASH;
        let _ = trajectory_msgs::JointTrajectory::SCHEMA_HASH;
        let _ = visualization_msgs::Marker::SCHEMA_HASH;
    }

    /// The corpus bindings are TOTAL (one per vendored message),
    /// and each hash is the very `SCHEMA_HASH` the generated type stamps on the
    /// wire — pinned on a flat type, a type with a FIXED nested target (the
    /// `resolve_fixed_nested` fold must match the generated fold), and a
    /// variable type. A binding that disagreed with the generated constant
    /// would name the wrong type — or nothing — for every runtime topic of that
    /// type.
    #[test]
    fn builtin_hash_bindings_are_total_and_match_the_generated_wire_hashes() {
        let bindings = builtin_hash_bindings();
        assert_eq!(
            bindings.len(),
            BUILTIN_MSGS.len(),
            "one binding per vendored message — no gaps"
        );
        let hash_of = |q: &str| {
            bindings
                .iter()
                .find(|b| b.qualified == q)
                .unwrap_or_else(|| panic!("{q} must have a binding"))
                .schema_hash
        };
        assert_eq!(hash_of("std_msgs/String"), std_msgs::String::SCHEMA_HASH);
        assert_eq!(
            hash_of("geometry_msgs/Vector3"),
            geometry_msgs::Vector3::SCHEMA_HASH
        );
        // A FIXED nested target folds the SAME target hash the generated type does.
        assert_eq!(hash_of("sensor_msgs/Imu"), sensor_msgs::Imu::SCHEMA_HASH);
        assert_eq!(
            hash_of("sensor_msgs/Image"),
            sensor_msgs::Image::SCHEMA_HASH
        );
        // Distinct types never share a binding hash (the map is a real inverse).
        let mut hashes: Vec<u64> = bindings.iter().map(|b| b.schema_hash).collect();
        hashes.sort_unstable();
        hashes.dedup();
        assert_eq!(hashes.len(), bindings.len(), "no two built-ins collide");
    }
}

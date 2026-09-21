// SPDX-License-Identifier: AGPL-3.0-only
//! Shared frame-building helpers for the never-block integration tests
//! (crib: `sink_dispatch_test`). A `geometry_msgs/Twist` frame dispatches to six
//! `Scalars` `rec.log` calls, so a handful of them fills a small batcher — the
//! cheapest walkable frame that actually exercises the (blocking) log path.

#![allow(dead_code)] // each test binary uses a subset of these helpers

use cerulion_core::codegen::layout::LayoutResolver;
use cerulion_core::codegen::{parse_rosmsg, FrameWalker, MessageSchema};
use cerulion_core::message::ShmMessage;
use cerulion_core::wire::WireHeader;
use native_ros2_messages::geometry_msgs::Twist;

/// Every built-in ROS 2 schema, for the layout engine + walker.
pub fn all_schemas() -> Vec<MessageSchema> {
    let mut schemas = Vec::new();
    for (pkg, name, text) in native_ros2_messages::BUILTIN_MSGS {
        if let Ok(s) = parse_rosmsg(text, name, Some(pkg)) {
            schemas.push(s);
        }
    }
    schemas
}

/// A `FrameWalker` over every built-in schema (decodes the frames below).
pub fn builtin_walker() -> FrameWalker {
    FrameWalker::new(all_schemas()).0
}

/// Offset of a top-level fixed field within a schema's fixed section.
fn field_offset(qname: &str, field: &str) -> usize {
    let (mut resolver, _) = LayoutResolver::new(all_schemas());
    let layout = resolver.layout_of(qname).expect("schema");
    layout
        .fixed_fields
        .iter()
        .find(|f| f.name == field)
        .unwrap_or_else(|| panic!("field {field} of {qname}"))
        .offset
}

/// Build a fixed-only message wire frame from `(offset, LE bytes)` writes, using
/// the real layout engine so the walker decodes it byte-correctly.
fn build_fixed_frame(qname: &str, schema_hash: u64, writes: &[(usize, Vec<u8>)]) -> Vec<u8> {
    build_fixed_frame_at(qname, schema_hash, writes, 42_000)
}

/// [`build_fixed_frame`] with an explicit wire timestamp — for a test that feeds
/// MANY frames of one topic and needs the publisher's clock to advance between
/// them (a plot topic's rate gate keys on the wire stamp, so a fixture
/// that stamps every frame identically models a stopped publisher clock, and the
/// gate correctly plots exactly one of them).
fn build_fixed_frame_at(
    qname: &str,
    schema_hash: u64,
    writes: &[(usize, Vec<u8>)],
    timestamp_ns: u64,
) -> Vec<u8> {
    let (mut resolver, _) = LayoutResolver::new(all_schemas());
    let layout = resolver.layout_of(qname).expect("built-in schema");
    let mut payload = vec![0u8; layout.fixed_size];
    for (off, bytes) in writes {
        payload[*off..*off + bytes.len()].copy_from_slice(bytes);
    }
    let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
    let header = WireHeader {
        schema_hash,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + layout.fixed_size) as u32,
        offset_table_count: 0,
        sequence: 0,
        timestamp_ns,
    };
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(&payload);
    frame
}

/// A `geometry_msgs/Twist` frame (two nested `Vector3`s = 6 f64) — decodes to
/// `Scalars` and logs six `rec.log` samples.
pub fn build_twist(linear: [f64; 3], angular: [f64; 3]) -> Vec<u8> {
    build_twist_at(linear, angular, 42_000)
}

/// [`build_twist`] stamped with an explicit publisher wire timestamp — what a
/// test feeding a STREAM of Twist frames must use, so the frames model a running
/// publisher clock rather than a stopped one (its plot rate gate keys on
/// exactly that advancement; see [`build_fixed_frame_at`]).
#[allow(dead_code)] // used by the multi-frame wedge tests, not by every binary
pub fn build_twist_at(linear: [f64; 3], angular: [f64; 3], timestamp_ns: u64) -> Vec<u8> {
    let lin = field_offset("geometry_msgs/Twist", "linear");
    let ang = field_offset("geometry_msgs/Twist", "angular");
    let mut writes = Vec::new();
    for (i, v) in linear.iter().enumerate() {
        writes.push((lin + i * 8, v.to_le_bytes().to_vec()));
    }
    for (i, v) in angular.iter().enumerate() {
        writes.push((ang + i * 8, v.to_le_bytes().to_vec()));
    }
    build_fixed_frame_at(
        "geometry_msgs/Twist",
        <Twist as ShmMessage>::SCHEMA_HASH,
        &writes,
        timestamp_ns,
    )
}

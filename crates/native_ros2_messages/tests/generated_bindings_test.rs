// SPDX-License-Identifier: AGPL-3.0-only
//! Acceptance pins on the GENERATED bindings: FQN schema-hash
//! linkage, rosidl scalar-default parity, and bool-array wire semantics.
//!
//! These run against the real generated types (not the generator's string
//! output), so they catch drift anywhere in the parse → resolve → generate
//! pipeline.

use cerulion_core::message::ShmMessage;
use cerulion_core::wire::{fnv1a_hash, MaxSliceLen};

use std::sync::atomic::{AtomicU64, Ordering};

// Round-trips go over an isolated iceoryx2 `TestTransport`
// (same pattern as `roundtrip_test.rs`).
use cerulion_core::testing::TestTransport;
use cerulion_core::transport::publisher::CerulionPublisher;
use cerulion_core::transport::subscriber::CerulionSubscriber;

use native_ros2_messages::control_msgs::MotionPrimitiveSnapshot;
use native_ros2_messages::geometry_msgs::{Pose, PoseShm, PoseSnapshot, QuaternionShm};
use native_ros2_messages::moveit_msgs::{
    AllowedCollisionEntry, AllowedCollisionEntryShm, PlanningScene,
};

// =====================================================================
// FQN schema-hash linkage
// =====================================================================

/// The generated `SCHEMA_HASH` is the recipe-3 layout-sensitive,
/// package-qualified hash — NOT a name-only FNV. This closes the two
/// regressions that would silently revert cross-package / layout
/// isolation: a generator that hashes the bare name, or one that hashes
/// only the (qualified) name without folding in the wire layout.
#[test]
fn generated_schema_hash_is_layout_and_fqn_sensitive() {
    // Not the bare-name recipe (recipe 1).
    assert_ne!(<Pose as ShmMessage>::SCHEMA_HASH, fnv1a_hash(b"Pose"));
    // Not a qualified-NAME-only hash either: recipe 3 folds in
    // wire_fixed_size + per-field canonical_str + (for fixed nested) the
    // target's recursive hash, so the value differs from FNV over just the
    // qualified name string.
    assert_ne!(
        <Pose as ShmMessage>::SCHEMA_HASH,
        fnv1a_hash(b"geometry_msgs/Pose")
    );
    assert_ne!(
        <PlanningScene as ShmMessage>::SCHEMA_HASH,
        fnv1a_hash(b"moveit_msgs/PlanningScene")
    );
    // Distinct schemas hash distinctly (sanity floor).
    assert_ne!(
        <Pose as ShmMessage>::SCHEMA_HASH,
        <PlanningScene as ShmMessage>::SCHEMA_HASH
    );

    // CLI `topic echo` linkage (std_msgs/String, sensor_msgs/Image) is
    // pinned against the generated constants by
    // `pinned_hashes_match_generated_constants` (cerulion_cli_engine) — the
    // CLI's `STD_MSGS_STRING_SCHEMA_HASH` / `SENSOR_MSGS_IMAGE_SCHEMA_HASH`
    // literals are asserted equal to `<T>::SCHEMA_HASH` there. (Separately,
    // `schema_hash_pin_test` in this crate pins the IR `schema_hash()` ==
    // generated `SCHEMA_HASH`.) A recipe-3 layout hash cannot be reproduced
    // by `const fn fnv1a_hash` over a name, so those are literal pins, not a
    // name-only equality here.
}

// =====================================================================
// rosidl scalar-default parity
// =====================================================================

/// `geometry_msgs/Quaternion.msg` declares `float64 w 1` — the rosidl
/// default is the IDENTITY quaternion. A parser that drops the default
/// makes `default()` produce the all-zero (invalid) rotation.
#[test]
fn quaternion_default_is_identity() {
    let q = QuaternionShm::default();
    assert_eq!(q.w, 1.0, "identity quaternion: w must default to 1");
    assert_eq!((q.x, q.y, q.z), (0.0, 0.0, 0.0));

    // Defaults propagate through nested composition: Pose embeds
    // Quaternion in its fixed section.
    let p = PoseShm::default();
    assert_eq!(p.orientation.w, 1.0);
    let ps = PoseSnapshot::default();
    assert_eq!(ps.orientation.w, 1.0);
}

/// `control_msgs/MotionPrimitive.msg` declares `int8 type -1` (the
/// "undefined" sentinel, distinct from meaningful enum value 0).
#[test]
fn motion_primitive_type_defaults_to_sentinel() {
    let m = MotionPrimitiveSnapshot::default();
    assert_eq!(m.r#type, -1);
}

// =====================================================================
// bool[] wire semantics
// =====================================================================

static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

// `TestTransport` owns the iceoryx2 node and MUST be kept in scope so the
// publisher/subscriber stay valid (each caller binds it to a local). Per-call
// isolated SHM root → parallel-safe, no `#[serial]` needed.
fn make_pub_sub(label: &str) -> (TestTransport, CerulionPublisher, CerulionSubscriber) {
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    let topic = format!("test/ros2-bench/{label}/{id}");
    let tt = TestTransport::with_buffer_size(4);
    let publisher = tt.publisher(&topic, MaxSliceLen::const_new(64 * 1024), 0);
    let subscriber = tt.subscriber(&topic);
    (tt, publisher, subscriber)
}

/// End-to-end `push_<f>` for `bool[]` (the direct byte store —
/// `payload[cursor] = if item {1} else {0}` — has no other runtime
/// exercise) + canonical-byte storage assertion via the raw payload, +
/// typed reader round-trip.
#[test]
fn bool_array_push_round_trips_with_canonical_bytes() {
    let (_tt, mut publisher, mut subscriber) = make_pub_sub("bool_push");

    {
        let mut proxy = publisher
            .loan_proxy::<AllowedCollisionEntry>()
            .expect("loan");
        proxy.push_enabled(true).expect("push");
        proxy.push_enabled(false).expect("push");
        proxy.push_enabled(true).expect("push");
    }

    let recovered = subscriber
        .try_view::<AllowedCollisionEntry, _>(|view| view.enabled().to_vec())
        .expect("try_view")
        .expect("frame");
    assert_eq!(recovered, vec![true, false, true]);
}

/// The `&[bool]` reader must NOT create a reference over non-canonical
/// bytes (UB) — a corrupt/byte-skewed frame reads as empty instead.
/// Build the wire payload by hand: AllowedCollisionEntry is
/// `{ bool[] enabled }` → empty fixed section, one offset entry.
#[test]
fn bool_array_reader_rejects_non_canonical_bytes() {
    // payload = [fixed(0)][offset entry: off=8,len=3][3 bool bytes]
    let make_payload = |bytes: [u8; 3]| {
        let mut payload = vec![0u8; 8 + 3];
        payload[0..4].copy_from_slice(&8u32.to_le_bytes()); // offset
        payload[4..8].copy_from_slice(&3u32.to_le_bytes()); // length
        payload[8..11].copy_from_slice(&bytes);
        payload
    };

    // Canonical bytes: values come through.
    let ok = make_payload([1, 0, 1]);
    let view = AllowedCollisionEntryShm::from_bytes(&ok);
    assert_eq!(view.enabled(), &[true, false, true]);

    // Non-canonical byte (2): defined degradation to empty, never UB.
    let bad = make_payload([1, 2, 1]);
    let view = AllowedCollisionEntryShm::from_bytes(&bad);
    assert_eq!(
        view.enabled(),
        &[] as &[bool],
        "non-canonical bool bytes must read as empty (corrupt frame), not UB"
    );
}

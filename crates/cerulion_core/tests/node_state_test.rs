// SPDX-License-Identifier: AGPL-3.0-only
//! PR 2 — the `#[cerulion_node]` FOLD-IN, end to end.
//!
//! `state_derive_test.rs` proves `#[derive(CerulionState)]` works. This file
//! proves the thing a user actually gets: a node struct that declares NOTHING
//! is capturable, over exactly its non-port fields, through the PRODUCTION
//! attribute macro and real rustc.
//!
//! The macro crate's own unit tests can only read the TOKENS the fold-in
//! emits — they cannot see whether the emitted impl compiles against a real
//! node (whose ports are zero-sized SHM markers and whose hidden `__cer_rt`
//! field holds an `Arc<dyn Clock>`), which is precisely where a whole-struct
//! walk would explode. Every oracle here is HAND-WRITTEN; nothing is compared
//! against a second run of the macro.
//!
//! Pure: no transport, no iceoryx2, no shared-memory singleton — the nodes are
//! never built or stepped, only their state impls are driven. Parallel-safe,
//! no `--test-threads=1`.

use cerulion_core::graph::node::CerNodeRuntimeFields;
use cerulion_core::prelude::*;
use cerulion_core::state::{CerulionState, StateCursor, StateError, StateShape, VecSink};
use native_ros2_messages::geometry_msgs::Vector3;
use std::sync::atomic::{AtomicU64, Ordering};

// ---------------------------------------------------------------------------
// helpers (crib: state_derive_test.rs)
// ---------------------------------------------------------------------------

fn capture(value: &impl CerulionState) -> Vec<u8> {
    let mut sink = VecSink::new();
    value.cer_capture(&mut sink).expect("unbounded capture");
    sink.into_inner()
}

/// Read `INLINE_SAFE` THROUGH a function so it is a runtime read rather than a
/// const assertion clippy rejects and the optimiser folds away.
fn inline_safe<T: CerulionState>() -> bool {
    T::INLINE_SAFE
}

fn restore_from<T: CerulionState>(value: &mut T, bytes: &[u8]) {
    let mut cursor = StateCursor::new(bytes);
    value.cer_restore(&mut cursor).expect("restore");
    cursor.finish().expect("blob fully consumed");
}

// ---------------------------------------------------------------------------
// the ORDINARY node — the 97.5% case, declaring nothing
// ---------------------------------------------------------------------------

/// A node whose state is two primitives and one plain helper, with a port on
/// either side. Not one line of capture machinery is written by hand.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct OrdinaryNode {
    // Plain `#[input]`, NOT `#[input(trigger)]`: the macro rejects a trigger
    // input alongside `period_ms` (time-driven vs data-driven policy). The
    // fold-in excludes a port either way — trigger-ness is not what makes a
    // field a port.
    #[input]
    scan: Vector3,
    #[output]
    cmd: Vector3,
    frames: u64,
    label: String,
}

#[cerulion_node_impl]
impl OrdinaryNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.frames += 1;
        self.cmd.x = self.frames as f64;
        Ok(())
    }
}

#[test]
fn an_ordinary_node_captures_its_non_port_fields_in_declaration_order() {
    let node = OrdinaryNode {
        frames: 0x0102_0304_0506_0708,
        label: "hi".to_string(),
        ..Default::default()
    };

    // HAND ORACLE: `frames` as 8 LE bytes, then `label` as a u32 length + its
    // UTF-8 bytes. The two PORTS contribute nothing, and so does the macro's
    // hidden `__cer_rt`. If any of the three were walked this would not even
    // compile (a port type carries no impl), so the byte count below is the
    // positive half of that claim.
    let mut oracle = Vec::new();
    oracle.extend_from_slice(&0x0102_0304_0506_0708u64.to_le_bytes());
    oracle.extend_from_slice(&2u32.to_le_bytes());
    oracle.extend_from_slice(b"hi");

    assert_eq!(capture(&node), oracle, "node capture must be field order");
}

/// Restore writes the state fields and leaves every NON-state field alone.
///
/// "Every non-state field" is two different claims with two different oracles,
/// and only one of them can be a value:
///
/// - The **ports** carry no value to check. A port's DECLARED type is a
///   zero-sized SHM marker — `geometry_msgs::Vector3` expands to `pub struct
///   Vector3;`, asserted below so this reasoning fails loudly if that ever
///   changes — reached inside a tick through a per-tick local, never through
///   the struct field. So there is no such thing as a port value a restore
///   could observably reset, and an assertion on one would be vacuous. The
///   port exclusion is enforced at COMPILE time instead: walking a port does
///   not build (`Vector3` carries no `CerulionState`), which is
///   what this file's capture/shape oracles rest on.
/// - The **hidden `__cer_rt`** field is where the value-level hazard actually
///   lands, and it is real: it holds the runtime's `Arc<dyn Clock>` and the
///   shared `ShutdownSignal`, both installed by `Entry::init` and both
///   process-live. The rejected alternative design — a `cer_read` that
///   constructs `Self { <state>, ..Default::default() }` — is exactly the
///   failure this arm catches: `Default` mints an `UninitClock` (a sentinel whose
///   every method PANICS) and a fresh, un-requested signal, so a node that
///   restored mid-run would come back detached from the runtime that owns it
///   and from any shutdown already in flight.
#[test]
fn an_ordinary_node_restores_in_place_and_leaves_every_non_state_field_alone() {
    // Drift guard for the reasoning above: a port type with FIELDS would make
    // "there is no port value to check" false, and this test would owe a value
    // oracle for the ports as well.
    assert_eq!(
        std::mem::size_of::<Vector3>(),
        0,
        "a port's declared type is a zero-sized SHM marker; if that changed, \
         a port now HAS an observable value and this test must pin it"
    );

    let source = OrdinaryNode {
        frames: 77,
        label: "restored".to_string(),
        ..Default::default()
    };
    let bytes = capture(&source);

    let mut target = OrdinaryNode {
        frames: 1,
        label: "stale".to_string(),
        ..Default::default()
    };

    // Stand in for `Entry::init`: give the target the two things a LIVE node's
    // hidden runtime field holds, each distinguishable from what `Default`
    // would mint.
    let virtual_clock = VirtualClock::new();
    virtual_clock.set(4242);
    let clock: std::sync::Arc<dyn Clock> = std::sync::Arc::new(virtual_clock);
    let shutdown = ShutdownSignal::new();
    shutdown.request();
    target.__cer_rt = CerNodeRuntimeFields {
        clock: std::sync::Arc::clone(&clock),
        shutdown_signal: shutdown.clone(),
    };

    restore_from(&mut target, &bytes);

    assert_eq!(target.frames, 77);
    assert_eq!(target.label, "restored");

    // The hidden runtime field is UNTOUCHED — the same clock ALLOCATION the
    // runtime installed, still reading the time it was set to, and the same
    // shutdown signal still carrying the request that was already in flight.
    // Asserted before the clock is read, because under that rejected design the clock is
    // an `UninitClock` whose `now_ns()` panics instead of returning a wrong
    // number, and a panic is a worse failure report than a value mismatch.
    assert!(
        target.__cer_rt.shutdown_signal.is_requested(),
        "restore must not swap in a fresh ShutdownSignal — a node restored \
         mid-shutdown would keep running"
    );
    assert!(
        std::sync::Arc::ptr_eq(&target.__cer_rt.clock, &clock),
        "restore must not replace the runtime's clock handle"
    );
    assert_eq!(target.__cer_rt.clock.now_ns(), 4242);

    // ... and the state the restore DID write is ordinary owned state
    // afterwards, not something borrowed from the blob.
    target.frames += 1;
    assert_eq!(target.frames, 78);
}

#[test]
fn an_ordinary_nodes_shape_is_keyed_by_its_non_port_field_names() {
    // HAND ORACLE: the struct name, then ONLY the two state fields. A shape
    // that folded the ports (or `__cer_rt`) could not match this.
    let expected = StateShape::of("OrdinaryNode")
        .field("frames", <u64 as CerulionState>::STATE_SHAPE)
        .field("label", <String as CerulionState>::STATE_SHAPE)
        .finish();

    assert_eq!(<OrdinaryNode as CerulionState>::STATE_SHAPE, expected);
}

#[test]
fn an_ordinary_node_is_inline_safe_and_declares_its_floor() {
    assert!(
        inline_safe::<OrdinaryNode>(),
        "a lock-free node must stay inline-eligible: the Q10 five-week subset \
         is INLINE_SAFE-gated and is only reachable this way"
    );
    // 8 bytes for the u64 + a String's 4-byte length prefix. The ports
    // contribute nothing, which is the point.
    assert_eq!(<OrdinaryNode as CerulionState>::MIN_ENCODED_BYTES, 12);
}

// ---------------------------------------------------------------------------
// a node with NO state at all
// ---------------------------------------------------------------------------

#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct PortsOnlyNode {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl PortsOnlyNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = 1.0;
        Ok(())
    }
}

#[test]
fn a_node_whose_only_fields_are_ports_captures_nothing_and_still_compiles() {
    // The `let _ = out;` the emission inserts is not tidiness — every crate
    // here and every scaffolded node crate sets `unused_variables = "deny"`,
    // so a node that writes nothing would be a hard error in the USER's build
    // without it. This test compiling IS that pin.
    let node = PortsOnlyNode::default();
    assert!(capture(&node).is_empty());
    assert_eq!(<PortsOnlyNode as CerulionState>::MIN_ENCODED_BYTES, 0);
    assert!(inline_safe::<PortsOnlyNode>());
}

// ---------------------------------------------------------------------------
// the EXCEPTIONAL node — one attribute, on the one field that needs it
// ---------------------------------------------------------------------------

/// Stands in for a handle the framework does not recognise (a user's own
/// wrapper over a device). Deliberately NOT `CerulionState`.
#[derive(Debug, Default)]
struct DeviceHandle {
    fd: i32,
}

#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct EscapedNode {
    #[output]
    out: Vector3,
    #[cerulion(reconstruct)]
    device: DeviceHandle,
    ticks: u32,
}

#[cerulion_node_impl]
impl EscapedNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.ticks += 1;
        self.out.x = self.ticks as f64;
        Ok(())
    }
}

#[test]
fn an_escaped_node_field_is_neither_captured_nor_restored_but_is_in_the_shape() {
    let node = EscapedNode {
        device: DeviceHandle { fd: 42 },
        ticks: 9,
        ..Default::default()
    };

    // HAND ORACLE: the escaped field contributes NO bytes.
    assert_eq!(capture(&node), 9u32.to_le_bytes().to_vec());

    // ...and the live handle SURVIVES a restore — the decisive property, and
    // the reason `cer_restore` walks field by field instead of replacing self.
    let bytes = capture(&node);
    let mut target = EscapedNode {
        device: DeviceHandle { fd: 7 },
        ticks: 0,
        ..Default::default()
    };
    restore_from(&mut target, &bytes);
    assert_eq!(target.ticks, 9, "the captured field is restored");
    assert_eq!(
        target.device.fd, 7,
        "the escaped handle must be left UNTOUCHED, not defaulted"
    );

    // The escape KIND is in the shape, so adding the attribute to a field
    // cannot leave a pre-change bag decoding "successfully" against a node
    // that no longer captures it.
    let expected = StateShape::of("EscapedNode")
        .field("device", StateShape::of("cerulion::reconstruct").finish())
        .field("ticks", <u32 as CerulionState>::STATE_SHAPE)
        .finish();
    assert_eq!(<EscapedNode as CerulionState>::STATE_SHAPE, expected);
}

// ---------------------------------------------------------------------------
// a RESOURCE the inventory already knows — zero tag
// ---------------------------------------------------------------------------

#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct RecognisedResourceNode {
    #[output]
    out: Vector3,
    /// No attribute: `TcpStream` is in the resource inventory, and a
    /// `Box<dyn ..>` is structural. Neither needs a tag.
    conn: Option<std::net::TcpStream>,
    hook: Option<Box<dyn std::fmt::Debug + Send>>,
    counter: u64,
}

#[cerulion_node_impl]
impl RecognisedResourceNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.counter += 1;
        self.out.x = self.counter as f64;
        Ok(())
    }
}

#[test]
fn a_node_holding_a_recognised_resource_needs_no_tag_at_all() {
    let node = RecognisedResourceNode {
        counter: 5,
        ..Default::default()
    };
    // Only `counter` is on the wire; the two handles contribute nothing.
    assert_eq!(capture(&node), 5u64.to_le_bytes().to_vec());
    assert_eq!(
        <RecognisedResourceNode as CerulionState>::MIN_ENCODED_BYTES,
        8
    );
    // Read both handles here rather than in `tick`, and the reason is worth
    // stating: an inventory-recognised field is captured by NOTHING, so under
    // this workspace's `dead_code = "deny"` a handle nothing ever reads is a
    // hard error — the compiler asking "then why is it on the struct?". A
    // production node reads its handle in `tick`; a fixture whose tick is
    // never called has to read it somewhere, and the test is the right place.
    assert!(node.conn.is_none() && node.hook.is_none());
}

// ---------------------------------------------------------------------------
// `cer_read` on a node
// ---------------------------------------------------------------------------

#[test]
fn cer_read_on_a_node_reports_rather_than_inventing_ports() {
    // A node's walk excludes its ports and `__cer_rt`, so `Self { .. }` could
    // never be written — and a `..Default::default()` fallback would silently
    // mint a node with DEFAULT ports whenever anybody called this. The path
    // the runtime takes is `init() -> cer_restore -> restored()`, which never
    // does; reporting is the correct answer.
    let bytes = capture(&OrdinaryNode::default());
    let mut cursor = StateCursor::new(&bytes);
    // Matched rather than `expect_err`'d: that helper needs `T: Debug`, and a
    // node struct carries PORT fields whose types are SHM markers — requiring
    // `Debug` on the node just to phrase this assertion would be the test
    // dictating the fixture's derives.
    match <OrdinaryNode as CerulionState>::cer_read(&mut cursor) {
        Err(StateError::Unrestorable { type_name }) => assert_eq!(type_name, "OrdinaryNode"),
        Err(other) => panic!("expected Unrestorable, got {other:?}"),
        Ok(_) => panic!("a node must refuse to be CONSTRUCTED from a blob"),
    }
}

// ---------------------------------------------------------------------------
// the fold-in does not disturb the node itself
// ---------------------------------------------------------------------------

#[test]
fn the_fold_in_leaves_the_nodes_own_surface_untouched() {
    // `NodeEntry` still exists, `new()` still exists, and the declared ports
    // are still what `info()` reports — i.e. emitting a second impl beside the
    // node changed nothing about the node.
    let entry = OrdinaryNodeEntry::new();
    let info = entry.info().expect("node info");
    assert_eq!(info.input_names(), ["scan".to_string()]);
    assert_eq!(info.output_names(), ["cmd".to_string()]);
}

#[test]
fn a_node_captures_deterministically_across_runs() {
    let build = || OrdinaryNode {
        frames: 1234,
        label: "det".to_string(),
        ..Default::default()
    };
    let a = capture(&build());
    let b = capture(&build());

    let mut oracle = Vec::new();
    oracle.extend_from_slice(&1234u64.to_le_bytes());
    oracle.extend_from_slice(&3u32.to_le_bytes());
    oracle.extend_from_slice(b"det");

    assert_eq!(a, b);
    assert_eq!(a, oracle, "and both equal the hand oracle, not each other");
}

// ---------------------------------------------------------------------------
// atomics through the fold-in (decision D2)
// ---------------------------------------------------------------------------

#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct AtomicNode {
    #[output]
    out: Vector3,
    hits: AtomicU64,
}

#[cerulion_node_impl]
impl AtomicNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.hits.fetch_add(1, Ordering::Relaxed);
        self.out.x = 1.0;
        Ok(())
    }
}

#[test]
fn an_atomic_node_field_rides_the_inventory_and_stays_inline_safe() {
    // 171 root-workspace field sites are
    // `Arc<AtomicU64>`-shaped, and without the atomic rows every one of them
    // would be a compile error the instant the fold-in landed.
    let node = AtomicNode::default();
    node.hits.store(9, Ordering::Relaxed);
    assert_eq!(capture(&node), 9u64.to_le_bytes().to_vec());
    assert!(
        inline_safe::<AtomicNode>(),
        "an atomic load takes NO lock, so it cannot block the node thread"
    );
}

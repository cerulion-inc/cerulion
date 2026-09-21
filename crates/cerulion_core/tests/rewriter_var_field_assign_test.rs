// SPDX-License-Identifier: AGPL-3.0-only
//! The schema-blind `=` assignment rewriter (formerly the
//! variable-field rewriter).
//!
//! Tests that inside `#[cerulion_node_impl]`, the macro rewrites EVERY
//! simple `self.<port>.<field> = expr` on a declared OUTPUT port to the
//! uniform codegen-emitted shim `__cer_<port>.__cer_assign_<field>(&expr)?`
//! — fixed AND variable alike; the generated schema types resolve
//! fixed-vs-variable at compile time (the earlier `#[output(data, ...)]`
//! field lists are gone).
//!
//! # Coverage
//!
//! - Simple variable field (`std_msgs::String::data`): shimmed assign
//!   round-trips through a subscriber.
//! - Byte-string literal pass-through (clippy `needless_borrow` pin).
//! - Fixed + variable fields on ONE port, both through the shim,
//!   both round-tripping.
//! - Undeclared (non-port) struct fields: NOT rewritten.
//! - Nested (3+-segment) WRITE paths on a port ARE rewritten
//!   — `self.<port>.<f1>…<leaf> = expr` / `.fill_from(...)` route through the
//!   recursive `__cer_with_nested_*` closure chain (fixed AND
//!   complex-variable nested, any depth). Bare nested READS still fall
//!   through to the leaf rewrite + Deref. Mixing staged nested sugar with a
//!   whole-field write on ONE field is a loud `NestedWriteConflict`; a staged
//!   child obeys the every-variable-field publish gate.

use cerulion_core::prelude::*;
use cerulion_core::testing::TestTransport;
use cerulion_core::testing::{
    count_at_exclusively, debug_lines_expected, line_level, never_loud, trace_level_compiled_in,
};
use cerulion_core::wire::MaxSliceLen;
use native_ros2_messages::builtin_interfaces::Time;
use native_ros2_messages::sensor_msgs::{Image, JoyFeedback};
use native_ros2_messages::std_msgs::String as RosString;

// Oracle helper: a zeroed 8-byte-aligned buffer (backed by
// `Vec<u64>`; `Vec<u8>` is only 1-aligned but `from_bytes_mut` asserts
// align 8 for f64-bearing fixed sections).
struct AlignedBuf(Vec<u64>);
impl AlignedBuf {
    fn new(n_bytes: usize) -> Self {
        Self(vec![0u64; n_bytes.div_ceil(8).max(1)])
    }
    fn bytes(&mut self) -> &mut [u8] {
        let n = self.0.len() * 8;
        // SAFETY: `Vec<u64>` is 8-aligned; all `n` bytes are initialized (zeroed).
        unsafe { std::slice::from_raw_parts_mut(self.0.as_mut_ptr() as *mut u8, n) }
    }
}

/// Build the hand-oracle Header payload: a STANDALONE `HeaderShm` writer
/// (NOT the staged path) executing `ops`, returning its
/// `[fixed|offset table|variable…cursor]` prefix — exactly the byte shape an
/// Image view's `header_bytes()` returns for a flushed staged header. Never a
/// self-compare: the oracle uses the ordinary writer, the assertion targets
/// the staged/rewrite path.
fn header_payload_oracle(
    ops: impl FnOnce(&mut native_ros2_messages::std_msgs::HeaderShm<'_>),
) -> Vec<u8> {
    use cerulion_core::wire::MaxPayloadCapacity;
    use native_ros2_messages::std_msgs::Header;
    let mut ab = AlignedBuf::new(2048);
    let bytes = ab.bytes();
    let mut w = Header::build_writer(bytes, MaxPayloadCapacity::const_new(2048), "oracle".into());
    ops(&mut w);
    let end = w.cursor() as usize;
    ab.bytes()[..end].to_vec()
}

#[cerulion_node(external)]
struct StringEcho {
    #[output]
    out: RosString,
}

#[cerulion_node_impl]
impl StringEcho {
    fn tick(&mut self) -> Result<(), cerulion_core::error::NodeError> {
        // This `=` is rewritten by the macro to the uniform
        // shim `__cer_out.__cer_assign_data("hello-world")?` (str literal
        // passes through unborrowed), which delegates to `set_data`.
        self.out.data = "hello-world";
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[test]
fn rewriter_assigns_simple_variable_field_round_trips() {
    use cerulion_core::graph::node::NodeEntry;

    let tt = TestTransport::with_buffer_size(4);
    let pubr = tt.publisher("test/rewriter_str", MaxSliceLen::const_new(64 * 1024), 0);
    let sub = tt.subscriber("test/rewriter_str");

    // Build NodeContext + entry, run a tick.
    let mut publishers = indexmap::IndexMap::new();
    publishers.insert(
        "out".to_string(),
        cerulion_core::graph::node::AnyPublisher::Ipc(pubr),
    );
    let subscribers = indexmap::IndexMap::new();
    let ctx = cerulion_core::graph::node::NodeContext::for_tests(publishers, subscribers);

    let mut entry = StringEchoEntry::new();
    entry.init(ctx).expect("init");
    entry
        .tick()
        .expect("tick should succeed via macro-rewritten `=`");
    entry.shutdown().expect("shutdown");

    // Verify subscriber received "hello-world".
    let mut received: Vec<Vec<u8>> = Vec::new();
    let _ = sub
        .try_receive(|m| received.push(m.payload().to_vec()))
        .expect("recv");
    assert_eq!(received.len(), 1);
    let payload_bytes = &received[0];
    // Wire frame's variable section starts after the 8-byte offset
    // table (std_msgs/String has 1 variable field, WIRE_FIXED_SIZE=0).
    assert_eq!(&payload_bytes[8..8 + 11], b"hello-world");
}

// ============================================================
// Byte-string literal (`b"..."`) pass-through into a `uint8[]` field
// ============================================================
//
// Byte-string literals parse as
// `syn::Lit::ByteStr`, NOT `syn::Lit::Str`, and have type `&[u8; N]` —
// already a reference. Were the var-field `=` rewriter's `already_borrowed`
// classification to match only `Lit::Str`, a `b"..."` RHS would
// fall to the else-branch and be rewritten to `__cer_assign_<f>(&(b"..."))` =
// `&&[u8; N]`. Since `b"..."` alone unsize-coerces to `&[u8]`, the outer
// `&` is needless → trips `clippy::needless_borrow` (deny-level in CI)
// for any node writing `self.<bytefield> = b"..."`.
//
// `Lit::ByteStr(_)` is in the `already_borrowed` match so the
// byte-string literal passes through UNBORROWED: the rewrite emits
// `__cer_assign_data(b"...")`, not `__cer_assign_data(&(b"..."))` (the
// borrow-adaptation is shared with the uniform-shim
// rewrite; the shim's param types match the typed setters').
//
// This test pins that contract two ways at once:
//   1. BEHAVIORAL — the node compiles, ticks, and the byte payload
//      arrives intact (`b"\x01\x02\x03\x04"` round-trips through SHM).
//   2. CODEGEN-SHAPE — because this test file is compiled under the CI
//      lint gate (`cargo clippy --workspace --all-targets -- -D warnings`),
//      the macro-generated `__cer_assign_data(b"...")` call is itself
//      subjected to `clippy::needless_borrow`. If the regression returns
//      (the rewriter emits `__cer_assign_data(&(b"..."))`), clippy fails
//      deny-level on this exact
//      generated line. (A trybuild `pass/` fixture could NOT pin this:
//      trybuild pass only checks compilation and does NOT run clippy, so
//      a needless `&` would slip through. There is no token-shape harness
//      in the macro crate, so this round-trip-under-clippy is the
//      strongest available pin.)
//
// `sensor_msgs::Image::data` is `uint8[]`, so its generated setter takes
// `&[u8]` — the exact field type that exposed the bug.

#[cerulion_node(external)]
struct ByteFieldFromByteLiteral {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl ByteFieldFromByteLiteral {
    fn tick(&mut self) -> Result<(), cerulion_core::error::NodeError> {
        // The load-bearing line: a `b"..."` RHS into a `uint8[]` field.
        // The rewriter MUST emit `__cer_assign_data(b"...")` (pass-through,
        // no surrounding `&`), not `__cer_assign_data(&(b"..."))`. Both
        // correctness (round-trip below) AND clippy-cleanliness (CI lint
        // gate) pin it.
        self.image.data = b"\x01\x02\x03\x04";
        // Clear the other declared variable fields so the publish gate
        // (all_variables_written) is satisfied.
        self.image.encoding = "mono8";
        self.image.header = &[][..];
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[test]
fn rewriter_passes_byte_string_literal_through_unborrowed() {
    use cerulion_core::graph::node::NodeEntry;

    let tt = TestTransport::with_buffer_size(4);
    let pubr = tt.publisher(
        "test/rewriter_byte_literal",
        MaxSliceLen::const_new(64 * 1024),
        0,
    );
    let sub = tt.subscriber("test/rewriter_byte_literal");

    let mut publishers = indexmap::IndexMap::new();
    publishers.insert(
        "image".to_string(),
        cerulion_core::graph::node::AnyPublisher::Ipc(pubr),
    );
    let subscribers = indexmap::IndexMap::new();
    let ctx = cerulion_core::graph::node::NodeContext::for_tests(publishers, subscribers);

    let mut entry = ByteFieldFromByteLiteralEntry::new();
    entry.init(ctx).expect("init");
    entry
        .tick()
        .expect("tick should succeed via macro-rewritten byte-literal `=`");
    entry.shutdown().expect("shutdown");

    // Verify the subscriber received the exact 4 bytes — proving the
    // byte-string literal flowed through `set_data` unmodified.
    let mut received: Vec<Vec<u8>> = Vec::new();
    let _ = sub
        .try_receive(|m| received.push(m.payload().to_vec()))
        .expect("recv");
    assert_eq!(received.len(), 1, "exactly one frame published");
    // sensor_msgs/Image has fixed fields + multiple variable fields; rather
    // than hand-compute the variable-section offset, assert the 4-byte
    // payload appears verbatim somewhere in the frame (it cannot appear by
    // accident — it is the only non-zero variable content besides the
    // 5-byte "mono8" encoding).
    let frame = &received[0];
    let needle = b"\x01\x02\x03\x04";
    assert!(
        frame.windows(needle.len()).any(|w| w == needle),
        "byte-string literal payload {needle:?} must round-trip verbatim through SHM; \
         frame = {frame:?}"
    );
}

// ============================================================
// Fixed + variable fields on ONE port, both shimmed
// ============================================================
//
// The schema-blind rewrite sends EVERY simple `self.<port>.<field> = expr`
// through `__cer_assign_<field>`. Fixed fields (`height`, `width`) resolve
// to the FixedSection shim (direct assignment under the hood); variable
// fields (`data`, `encoding`, complex `header`) to the Shm shim delegating
// to `set_*`/`set_*_bytes`. Both must round-trip through a real subscriber
// against hand-known oracle values.

#[cerulion_node(external)]
struct MixedFixedVariableNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl MixedFixedVariableNode {
    fn tick(&mut self) -> Result<(), cerulion_core::error::NodeError> {
        // Fixed fields — shimmed to `__cer_assign_height(&(1080))?` etc.
        self.image.height = 1080;
        self.image.width = 1920;
        self.image.step = 5760;
        self.image.is_bigendian = 0;
        // Variable fields — shimmed to the same uniform call shape.
        self.image.data = b"\x09\x08\x07";
        // OWNED String RHS (witnesses the `&(rhs)` borrow-
        // adaptation else-branch — `&String` deref-coerces to the shim's
        // `&str` param; a literal-only test would never exercise it).
        let enc_owned = String::from("rgb8");
        self.image.encoding = enc_owned;
        self.image.header = &[][..];
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[test]
fn shim_rewrites_fixed_and_variable_fields_and_both_round_trip() {
    use cerulion_core::graph::node::NodeEntry;

    let tt = TestTransport::with_buffer_size(4);
    let pubr = tt.publisher(
        "test/rewriter_mixed_shim",
        MaxSliceLen::const_new(64 * 1024),
        0,
    );
    let mut sub = tt.subscriber("test/rewriter_mixed_shim");

    let mut publishers = indexmap::IndexMap::new();
    publishers.insert(
        "image".to_string(),
        cerulion_core::graph::node::AnyPublisher::Ipc(pubr),
    );
    let ctx =
        cerulion_core::graph::node::NodeContext::for_tests(publishers, indexmap::IndexMap::new());

    let mut entry = MixedFixedVariableNodeEntry::new();
    entry.init(ctx).expect("init");
    entry.tick().expect("tick");
    entry.shutdown().expect("shutdown");

    // Oracle read-back through the typed view (fixed via Deref, variable
    // via the reader accessors) — hand-known values, not a self-compare.
    let mut seen: Option<(u32, u32, Vec<u8>, String)> = None;
    let _ = sub
        .try_view::<Image, _>(|view| {
            seen = Some((
                view.height,
                view.width,
                view.data().to_vec(),
                view.encoding().expect("utf8").to_string(),
            ));
        })
        .expect("recv");
    let (height, width, data, encoding) = seen.expect("one frame");
    assert_eq!(height, 1080, "fixed field written through the shim");
    assert_eq!(width, 1920);
    assert_eq!(
        data, b"\x09\x08\x07",
        "variable field written through the shim"
    );
    assert_eq!(encoding, "rgb8");
}

// ============================================================
// Undeclared (non-port) struct fields are NOT rewritten
// ============================================================
//
// `self.stats.count = 7` where `stats` is a plain state field (no
// `#[output]`) must be left untouched by the rewriter — if it were
// wrongly shimmed, the emitted `__cer_stats.__cer_assign_count(...)`
// would fail to compile (no such local). The behavioral half asserts
// the plain field actually mutated.

#[derive(Default, cerulion_core::state::CerulionState)]
struct PlainStats {
    count: u32,
}

#[cerulion_node(external)]
struct NonPortFieldNode {
    #[output]
    out: RosString,
    stats: PlainStats,
}

#[cerulion_node_impl]
impl NonPortFieldNode {
    fn tick(&mut self) -> Result<(), cerulion_core::error::NodeError> {
        // NOT a port — must stay a plain struct-field assignment.
        self.stats.count = 7;
        // Port write so the publish gate clears.
        self.out.data = "np";
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[test]
fn non_port_struct_field_assignment_is_not_rewritten() {
    use cerulion_core::graph::node::NodeEntry;

    let tt = TestTransport::with_buffer_size(4);
    let pubr = tt.publisher("test/rewriter_non_port", MaxSliceLen::const_new(4096), 0);

    let mut publishers = indexmap::IndexMap::new();
    publishers.insert(
        "out".to_string(),
        cerulion_core::graph::node::AnyPublisher::Ipc(pubr),
    );
    let ctx =
        cerulion_core::graph::node::NodeContext::for_tests(publishers, indexmap::IndexMap::new());

    let mut entry = NonPortFieldNodeEntry::new();
    entry.init(ctx).expect("init");
    entry.tick().expect("tick");
    assert_eq!(
        entry.inner.stats.count, 7,
        "plain (non-port) field assignment must reach user state untouched"
    );
}

// ============================================================
// 3-segment FIXED nested paths route via the __cer_with_nested_*
// chain (never a Deref fall-through)
// ============================================================
//
// `self.imu.orientation.x = 1.5` has TWO segments past the port. A leaf-only
// rewrite would fall through to the leaf `self.imu` → `__cer_imu` rewrite + an
// INFALLIBLE Deref write onto the inline `QuaternionShm`. The
// recursive rewrite OWNS it: it lowers to
// `__cer_imu.__cer_with_nested_orientation(|__cer_v| __cer_v.__cer_assign_x(&(1.5)))?`
// — now FALLIBLE (uniform fallibility, accepted by design). The SHM oracle is
// unchanged: x=1.5 and w=1.0 must land, read back through a real subscriber.

#[cerulion_node(external)]
struct NestedPathNode {
    #[output]
    imu: native_ros2_messages::sensor_msgs::Imu,
}

#[cerulion_node_impl]
impl NestedPathNode {
    fn tick(&mut self) -> Result<(), cerulion_core::error::NodeError> {
        // 3-segment fixed nested — the rewriter routes each through
        // `__cer_with_nested_orientation` (fallible chain), NOT a Deref write.
        self.imu.orientation.x = 1.5;
        self.imu.orientation.w = 1.0;
        // Single-segment on the same port still shims (complex variable);
        // written wholesale (no staged sugar on `header` → no conflict).
        self.imu.header = &[][..];
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[test]
fn multi_segment_port_path_routes_via_with_nested_chain() {
    use cerulion_core::graph::node::NodeEntry;

    let tt = TestTransport::with_buffer_size(4);
    let pubr = tt.publisher("test/rewriter_nested_path", MaxSliceLen::const_new(4096), 0);
    let mut sub = tt.subscriber("test/rewriter_nested_path");

    let mut publishers = indexmap::IndexMap::new();
    publishers.insert(
        "imu".to_string(),
        cerulion_core::graph::node::AnyPublisher::Ipc(pubr),
    );
    let ctx =
        cerulion_core::graph::node::NodeContext::for_tests(publishers, indexmap::IndexMap::new());

    let mut entry = NestedPathNodeEntry::new();
    entry.init(ctx).expect("init");
    entry.tick().expect("tick");

    let mut seen: Option<(f64, f64)> = None;
    let _ = sub
        .try_view::<native_ros2_messages::sensor_msgs::Imu, _>(|view| {
            seen = Some((view.orientation.x, view.orientation.w));
        })
        .expect("recv");
    let (x, w) = seen.expect("one frame");
    assert_eq!(x, 1.5, "nested-path chain write must land in SHM");
    assert_eq!(w, 1.0);
}

// ============================================================
// Same-port RHS on a nested fixed path (the RHS-hoist pin)
// ============================================================
//
// `self.imu.orientation.y = self.imu.orientation.w * 2.0` reads the SAME
// port it writes. The rewrite HOISTS the RHS into a temp before the write
// chain — without that the closure would capture `__cer_imu` while it is
// mutably borrowed (E0502). Compiling AND landing y = 2*w pins the hoist.

#[cerulion_node(external)]
struct SamePortNestedRhsNode {
    #[output]
    imu: native_ros2_messages::sensor_msgs::Imu,
}

#[cerulion_node_impl]
impl SamePortNestedRhsNode {
    fn tick(&mut self) -> Result<(), cerulion_core::error::NodeError> {
        self.imu.orientation.w = 1.25;
        // Same-port RHS read on a nested fixed path — the hoist makes it
        // compile; y must land as 2 * w.
        self.imu.orientation.y = self.imu.orientation.w * 2.0;
        self.imu.header = &[][..];
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[test]
fn same_port_rhs_on_nested_fixed_path_compiles_and_lands() {
    use cerulion_core::graph::node::NodeEntry;

    let tt = TestTransport::with_buffer_size(4);
    let pubr = tt.publisher(
        "test/rewriter_same_port_rhs",
        MaxSliceLen::const_new(4096),
        0,
    );
    let mut sub = tt.subscriber("test/rewriter_same_port_rhs");

    let mut publishers = indexmap::IndexMap::new();
    publishers.insert(
        "imu".to_string(),
        cerulion_core::graph::node::AnyPublisher::Ipc(pubr),
    );
    let ctx =
        cerulion_core::graph::node::NodeContext::for_tests(publishers, indexmap::IndexMap::new());

    let mut entry = SamePortNestedRhsNodeEntry::new();
    entry.init(ctx).expect("init");
    entry.tick().expect("tick");

    let mut seen: Option<(f64, f64)> = None;
    let _ = sub
        .try_view::<native_ros2_messages::sensor_msgs::Imu, _>(|view| {
            seen = Some((view.orientation.w, view.orientation.y));
        })
        .expect("recv");
    let (w, y) = seen.expect("one frame");
    assert_eq!(w, 1.25);
    assert_eq!(y, 2.5, "y = 2 * w proves the hoisted same-port RHS read");
}

// ============================================================
// Variable-nested sugar e2e (frame_id + depth-2 stamp.sec)
// ============================================================
//
// `self.image.header.frame_id = "cam0"` (staged variable leaf) AND
// `self.image.header.stamp.sec = 5` (depth-2: header.stamp.sec, fixed leaf
// through the staged view) in ONE tick, round-tripped through a real
// subscriber. Oracle = the SAME logical Header writes through a STANDALONE
// HeaderShm (never a self-compare).

#[cerulion_node(external)]
struct VarNestedSugarNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl VarNestedSugarNode {
    fn tick(&mut self) -> Result<(), cerulion_core::error::NodeError> {
        // Staged variable leaf + depth-2 fixed leaf on the SAME header field
        // (both staged → resume, no conflict).
        self.image.header.frame_id = "cam0";
        self.image.header.stamp.sec = 5;
        // Other Image variable fields so the publish gate clears.
        self.image.encoding = "rgb8";
        self.image.data = b"\x01\x02";
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[test]
fn variable_nested_sugar_round_trips_against_standalone_header_oracle() {
    use cerulion_core::graph::node::NodeEntry;

    let tt = TestTransport::with_buffer_size(4);
    let pubr = tt.publisher("test/rewriter_var_nested", MaxSliceLen::const_new(4096), 0);
    let mut sub = tt.subscriber("test/rewriter_var_nested");

    let mut publishers = indexmap::IndexMap::new();
    publishers.insert(
        "image".to_string(),
        cerulion_core::graph::node::AnyPublisher::Ipc(pubr),
    );
    let ctx =
        cerulion_core::graph::node::NodeContext::for_tests(publishers, indexmap::IndexMap::new());

    let mut entry = VarNestedSugarNodeEntry::new();
    entry.init(ctx).expect("init");
    entry.tick().expect("tick");

    // Hand-built oracle: a STANDALONE HeaderShm executing the SAME logical
    // writes (frame_id + stamp.sec) — byte equality proves the staged/rewrite
    // path encodes identically.
    let oracle_bytes = header_payload_oracle(|w| {
        w.set_frame_id("cam0").expect("oracle frame_id");
        w.stamp.sec = 5;
    });

    let mut seen: Option<Vec<u8>> = None;
    let _ = sub
        .try_view::<Image, _>(|view| {
            seen = Some(view.header_bytes().to_vec());
        })
        .expect("recv");
    let header_bytes = seen.expect("one frame");
    assert_eq!(
        header_bytes, oracle_bytes,
        "staged nested sugar must serialize the header byte-identically to the standalone oracle"
    );
}

// ============================================================
// `header.stamp = Time::from_ns(..)` — the WHOLE fixed-nested write
// through the staged Header, sourced from the built-in stamp helper
// ============================================================
//
// The headline one-liner: `self.image.header.stamp = Time::from_ns(NS)` is a
// WHOLE fixed-nested-field write (leaf = the `stamp` substruct, not `.sec`)
// through the staged variable Header. It rewrites to
// `__cer_with_nested_header(|v| v.__cer_assign_stamp(&(Time::from_ns(NS))))`,
// resolving `__cer_assign_stamp(&TimeShm)` on `HeaderFixedSection` via the
// `HeaderShm` DerefMut chain. Oracle = a STANDALONE HeaderShm whose stamp is
// set from HAND-COMPUTED sec/nanosec (never `from_ns` on both sides), so byte
// equality pins BOTH the whole-nested write path AND the helper's arithmetic.

/// 2024-01-01T00:00:00.5Z in unix ns → sec 1_704_067_200, nanosec 500_000_000.
const STAMP_NS: u64 = 1_704_067_200_500_000_000;
const STAMP_SEC: i32 = 1_704_067_200;
const STAMP_NANOSEC: u32 = 500_000_000;

#[cerulion_node(external)]
struct WholeStampWriteNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl WholeStampWriteNode {
    fn tick(&mut self) -> Result<(), cerulion_core::error::NodeError> {
        // WHOLE fixed-nested write through the staged header, from the helper.
        self.image.header.stamp = Time::from_ns(STAMP_NS);
        // frame_id keeps the header a legit staged write; encoding + data clear
        // the every-variable-field publish gate.
        self.image.header.frame_id = "cam0";
        self.image.encoding = "rgb8";
        self.image.data = b"\x01\x02";
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[test]
fn whole_stamp_write_from_ns_round_trips_against_standalone_header_oracle() {
    use cerulion_core::graph::node::NodeEntry;

    let tt = TestTransport::with_buffer_size(4);
    let pubr = tt.publisher("test/rewriter_whole_stamp", MaxSliceLen::const_new(4096), 0);
    let mut sub = tt.subscriber("test/rewriter_whole_stamp");

    let mut publishers = indexmap::IndexMap::new();
    publishers.insert(
        "image".to_string(),
        cerulion_core::graph::node::AnyPublisher::Ipc(pubr),
    );
    let ctx =
        cerulion_core::graph::node::NodeContext::for_tests(publishers, indexmap::IndexMap::new());

    let mut entry = WholeStampWriteNodeEntry::new();
    entry.init(ctx).expect("init");
    entry.tick().expect("tick");

    // Oracle: the SAME logical header writes through a standalone HeaderShm, but
    // stamp is set from HAND-COMPUTED sec/nanosec — NOT `from_ns` — so the
    // helper's split is genuinely cross-checked, not self-compared.
    let oracle_bytes = header_payload_oracle(|w| {
        w.stamp.sec = STAMP_SEC;
        w.stamp.nanosec = STAMP_NANOSEC;
        w.set_frame_id("cam0").expect("oracle frame_id");
    });

    let mut seen: Option<Vec<u8>> = None;
    let _ = sub
        .try_view::<Image, _>(|view| {
            seen = Some(view.header_bytes().to_vec());
        })
        .expect("recv");
    let header_bytes = seen.expect("one frame");
    assert_eq!(
        header_bytes, oracle_bytes,
        "`header.stamp = Time::from_ns(..)` must serialize byte-identically to the \
         hand-computed standalone header oracle"
    );
}

// ============================================================
// Mixed staged sugar + with_header() in one tick (resume)
// ============================================================
//
// `self.image.header.frame_id = "cam0"` (sugar) AND
// `self.image.with_header(|h| { h.stamp.sec = 9; Ok(()) })?` (closure API)
// both write DIFFERENT parts of the SAME header field. Both land — the
// second call RESUMES the first's staged state (resume-not-reset).

#[cerulion_node(external)]
struct MixedSugarWithHeaderNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl MixedSugarWithHeaderNode {
    fn tick(&mut self) -> Result<(), cerulion_core::error::NodeError> {
        self.image.header.frame_id = "cam0";
        self.image.with_header(|h| {
            h.stamp.sec = 9;
            Ok(())
        })?;
        self.image.encoding = "rgb8";
        self.image.data = b"\x03";
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[test]
fn mixed_sugar_and_with_header_both_land_via_resume() {
    use cerulion_core::graph::node::NodeEntry;

    let tt = TestTransport::with_buffer_size(4);
    let pubr = tt.publisher("test/rewriter_mixed_sugar", MaxSliceLen::const_new(4096), 0);
    let mut sub = tt.subscriber("test/rewriter_mixed_sugar");

    let mut publishers = indexmap::IndexMap::new();
    publishers.insert(
        "image".to_string(),
        cerulion_core::graph::node::AnyPublisher::Ipc(pubr),
    );
    let ctx =
        cerulion_core::graph::node::NodeContext::for_tests(publishers, indexmap::IndexMap::new());

    let mut entry = MixedSugarWithHeaderNodeEntry::new();
    entry.init(ctx).expect("init");
    entry.tick().expect("tick");

    let oracle_bytes = header_payload_oracle(|w| {
        w.set_frame_id("cam0").expect("oracle frame_id");
        w.stamp.sec = 9;
    });

    let mut seen: Option<Vec<u8>> = None;
    let _ = sub
        .try_view::<Image, _>(|view| {
            seen = Some(view.header_bytes().to_vec());
        })
        .expect("recv");
    assert_eq!(
        seen.expect("one frame"),
        oracle_bytes,
        "sugar + with_header must both land (resume) and match the standalone oracle"
    );
}

// ============================================================
// Nested fill_from composes
// ============================================================
//
// `self.image.header.frame_id.fill_from(producer)` lowers to
// `__cer_image.__cer_with_nested_header(|__cer_v| __cer_v.__cer_fill_from_frame_id(producer))`
// (no trailing `?` — the user's own `?` wraps it). The producer writes
// "lidar" into frame_id; round-trips against a standalone oracle.

#[cerulion_node(external)]
struct NestedFillFromNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl NestedFillFromNode {
    fn tick(&mut self) -> Result<(), cerulion_core::error::NodeError> {
        self.image.header.frame_id.fill_from(|buf: &mut [u8]| {
            buf[..5].copy_from_slice(b"lidar");
            Ok(5)
        })?;
        self.image.encoding = "rgb8";
        self.image.data = b"\x04";
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[test]
fn nested_fill_from_composes_and_round_trips() {
    use cerulion_core::graph::node::NodeEntry;

    let tt = TestTransport::with_buffer_size(4);
    let pubr = tt.publisher("test/rewriter_nested_fill", MaxSliceLen::const_new(4096), 0);
    let mut sub = tt.subscriber("test/rewriter_nested_fill");

    let mut publishers = indexmap::IndexMap::new();
    publishers.insert(
        "image".to_string(),
        cerulion_core::graph::node::AnyPublisher::Ipc(pubr),
    );
    let ctx =
        cerulion_core::graph::node::NodeContext::for_tests(publishers, indexmap::IndexMap::new());

    let mut entry = NestedFillFromNodeEntry::new();
    entry.init(ctx).expect("init");
    entry.tick().expect("tick");

    let oracle_bytes = header_payload_oracle(|w| {
        w.set_frame_id("lidar").expect("oracle frame_id");
    });

    let mut seen: Option<Vec<u8>> = None;
    let _ = sub
        .try_view::<Image, _>(|view| {
            seen = Some(view.header_bytes().to_vec());
        })
        .expect("recv");
    assert_eq!(
        seen.expect("one frame"),
        oracle_bytes,
        "nested fill_from must write frame_id byte-identically to the standalone oracle"
    );
}

// ============================================================
// Grandchild sugar (complex-inside-complex) e2e
// ============================================================
//
// Native chain: RobotTrajectory.joint_trajectory (JointTrajectory — variable,
// so complex-nested) → .header (complex-nested Header) → .frame_id. The
// grandchild leaf stages into the transient VIEW's own slot; the staging persists
// it across views (`StagedNested::children`) and flushes it recursively.

/// Hand-oracle JointTrajectory payload via a STANDALONE writer (never the
/// staged path).
fn joint_trajectory_payload_oracle(
    ops: impl FnOnce(&mut native_ros2_messages::trajectory_msgs::JointTrajectoryShm<'_>),
) -> Vec<u8> {
    use cerulion_core::wire::MaxPayloadCapacity;
    use native_ros2_messages::trajectory_msgs::JointTrajectory;
    let mut ab = AlignedBuf::new(4096);
    let bytes = ab.bytes();
    let mut w =
        JointTrajectory::build_writer(bytes, MaxPayloadCapacity::const_new(4096), "oracle".into());
    ops(&mut w);
    let end = w.cursor() as usize;
    ab.bytes()[..end].to_vec()
}

#[cerulion_node(external)]
struct GrandchildSugarNode {
    #[output]
    traj: native_ros2_messages::moveit_msgs::RobotTrajectory,
}

#[cerulion_node_impl]
impl GrandchildSugarNode {
    fn tick(&mut self) -> Result<(), cerulion_core::error::NodeError> {
        // THE regression pin: a grandchild leaf through TWO
        // complex-variable levels. Were this staging to die with the
        // transient JointTrajectory view, the Drop flush would report
        // NestedChildIncomplete (joint_trajectory.header) and DISCARD the
        // frame even though the user wrote everything — this test would FAIL
        // with "one frame — a dropped staging discards the frame" (got 0 frames).
        self.traj.joint_trajectory.header.frame_id = "base";
        // The staged child's other variable fields (3-segment paths through
        // the SAME staged child — each statement re-enters the accessor, so
        // children persistence is exercised on every one).
        self.traj.joint_trajectory.joint_names = &[][..];
        self.traj.joint_trajectory.points = &[][..];
        // The parent's sibling variable field (whole-field bytes, unstaged).
        self.traj.multi_dof_joint_trajectory = &[][..];
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[test]
fn grandchild_sugar_publishes_and_round_trips_to_standalone_oracle() {
    use cerulion_core::graph::node::NodeEntry;
    use native_ros2_messages::moveit_msgs::RobotTrajectory;

    let tt = TestTransport::with_buffer_size(4);
    let pubr = tt.publisher(
        "test/rewriter_grandchild",
        MaxSliceLen::const_new(64 * 1024),
        0,
    );
    let mut sub = tt.subscriber("test/rewriter_grandchild");

    let mut publishers = indexmap::IndexMap::new();
    publishers.insert(
        "traj".to_string(),
        cerulion_core::graph::node::AnyPublisher::Ipc(pubr),
    );
    let ctx =
        cerulion_core::graph::node::NodeContext::for_tests(publishers, indexmap::IndexMap::new());

    let mut entry = GrandchildSugarNodeEntry::new();
    entry.init(ctx).expect("init");
    entry.tick().expect("tick");
    entry.shutdown().expect("shutdown");

    // Hand oracle: standalone JointTrajectory writer applying the same
    // logical writes in staged-path order (joint_names + points in tick
    // order; the staged header flushes LAST at the cursor tail).
    let header_bytes = header_payload_oracle(|h| {
        h.set_frame_id("base").expect("oracle frame_id");
    });
    let jt_oracle = joint_trajectory_payload_oracle(|o| {
        o.set_joint_names_bytes(&[]).expect("oracle names");
        o.set_points_bytes(&[]).expect("oracle points");
        o.set_header_bytes(&header_bytes).expect("oracle header");
    });

    let mut seen: Option<Vec<u8>> = None;
    let _ = sub
        .try_view::<RobotTrajectory, _>(|view| {
            seen = Some(view.joint_trajectory_bytes().to_vec());
        })
        .expect("recv");
    assert_eq!(
        seen.expect("one frame — a dropped staging discards the frame"),
        jt_oracle,
        "grandchild sugar must flush recursively and round-trip byte-identically"
    );
}

#[cerulion_node(external)]
struct GrandchildTwoChainsNode {
    #[output]
    traj: native_ros2_messages::moveit_msgs::RobotTrajectory,
}

#[cerulion_node_impl]
impl GrandchildTwoChainsNode {
    fn tick(&mut self) -> Result<(), cerulion_core::error::NodeError> {
        // TWO separate grandchild chains — the second re-enters the
        // joint_trajectory accessor, so chain 1's header staging must be
        // RESTORED into the fresh view (resume-of-children pin: without the
        // restore the second chain lazily allocates FRESH Header staging and
        // frame_id is silently lost).
        self.traj.joint_trajectory.header.frame_id = "base";
        self.traj.joint_trajectory.header.stamp.sec = 7;
        self.traj.joint_trajectory.joint_names = &[][..];
        self.traj.joint_trajectory.points = &[][..];
        self.traj.multi_dof_joint_trajectory = &[][..];
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[test]
fn grandchild_multi_leaf_across_two_chains_resumes_children() {
    use cerulion_core::graph::node::NodeEntry;
    use native_ros2_messages::moveit_msgs::RobotTrajectory;

    let tt = TestTransport::with_buffer_size(4);
    let pubr = tt.publisher(
        "test/rewriter_grandchild_two",
        MaxSliceLen::const_new(64 * 1024),
        0,
    );
    let mut sub = tt.subscriber("test/rewriter_grandchild_two");

    let mut publishers = indexmap::IndexMap::new();
    publishers.insert(
        "traj".to_string(),
        cerulion_core::graph::node::AnyPublisher::Ipc(pubr),
    );
    let ctx =
        cerulion_core::graph::node::NodeContext::for_tests(publishers, indexmap::IndexMap::new());

    let mut entry = GrandchildTwoChainsNodeEntry::new();
    entry.init(ctx).expect("init");
    entry.tick().expect("tick");
    entry.shutdown().expect("shutdown");

    let header_bytes = header_payload_oracle(|h| {
        h.set_frame_id("base").expect("oracle frame_id");
        h.stamp.sec = 7;
    });
    let jt_oracle = joint_trajectory_payload_oracle(|o| {
        o.set_joint_names_bytes(&[]).expect("oracle names");
        o.set_points_bytes(&[]).expect("oracle points");
        o.set_header_bytes(&header_bytes).expect("oracle header");
    });

    let mut seen: Option<Vec<u8>> = None;
    let _ = sub
        .try_view::<RobotTrajectory, _>(|view| {
            seen = Some(view.joint_trajectory_bytes().to_vec());
        })
        .expect("recv");
    assert_eq!(
        seen.expect("one frame"),
        jt_oracle,
        "both grandchild leaves (across two chains) must land — children resume"
    );
}

// ============================================================
// The OutputProxy::Drop step-0 flush-error branch e2e
// ============================================================
//
// A tick that stages a PARTIAL child (Header's fixed `stamp.sec` only —
// `frame_id`, its lone variable field, unwritten; a depth-1 partial,
// unaffected by the grandchild persistence) and returns Ok(()). At tick end the OutputProxy
// Drops: step 0's `flush_staged_nested` fails the child gate
// (`NestedChildIncomplete`), the loud discard error fires, and NO frame is
// published.

#[cerulion_node(external)]
struct PartialChildDropNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl PartialChildDropNode {
    fn tick(&mut self) -> Result<(), cerulion_core::error::NodeError> {
        // Stage ONLY the fixed grand-leaf; header.frame_id stays unwritten.
        self.image.header.stamp.sec = 5;
        // Satisfy the TOP-LEVEL variable fields so the failure is isolated
        // to the step-0 staged-flush branch (not the all-variables gate).
        self.image.encoding = "rgb8";
        self.image.data = b"\x01";
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[test]
#[tracing_test::traced_test]
fn partial_staged_child_discards_at_drop_loudly_and_publishes_nothing() {
    use cerulion_core::graph::node::NodeEntry;

    let tt = TestTransport::with_buffer_size(4);
    let pubr = tt.publisher(
        "test/rewriter_partial_drop",
        MaxSliceLen::const_new(4096),
        0,
    );
    let sub = tt.subscriber("test/rewriter_partial_drop");

    let mut publishers = indexmap::IndexMap::new();
    publishers.insert(
        "image".to_string(),
        cerulion_core::graph::node::AnyPublisher::Ipc(pubr),
    );
    let ctx =
        cerulion_core::graph::node::NodeContext::for_tests(publishers, indexmap::IndexMap::new());

    let mut entry = PartialChildDropNodeEntry::new();
    entry.init(ctx).expect("init");
    // The tick itself SUCCEEDS — the failure surfaces at proxy Drop (step 0).
    entry
        .tick()
        .expect("tick returns Ok; the discard happens at Drop");
    entry.shutdown().expect("shutdown");

    // No frame published.
    let mut count = 0usize;
    let _ = sub.try_receive(|_| count += 1);
    assert_eq!(count, 0, "a failed staged flush must not publish a frame");

    // The loud discard fired (OutputProxy::Drop step 0) and the gate error
    // (rendered via the `error` field) names the dotted child location.
    // (`tracing-test` here has `no-env-filter`, so the cerulion_core-emitted
    // event is captured — same pattern as overflow_redirect_e2e_test.)
    // "Loud" is a LEVEL claim, so the level token is matched as well as the
    // marker: a flush failure demoted to `warn!`/`info!` would still satisfy a
    // bare `logs_contain`.
    logs_assert(|lines: &[&str]| {
        if lines.iter().any(|l| {
            line_level(l) == Some("ERROR") && l.contains("staged nested-field flush failed")
        }) {
            Ok(())
        } else {
            Err("the Drop-path flush error must be loud — an ERROR line".to_string())
        }
    });
    assert!(
        logs_contain("header.frame_id"),
        "the discard must name the unwritten child field"
    );

    // POSITIVE CONTROL: the SAME node/harness with a
    // COMPLETE child publishes exactly one frame — proving count==0 above
    // is the discard, not a vacuously-broken pipeline (wrong topic wiring /
    // detached subscriber would pass the count==0 leg silently).
    let pubr2 = tt.publisher(
        "test/rewriter_partial_drop_ctl",
        MaxSliceLen::const_new(4096),
        0,
    );
    let sub2 = tt.subscriber("test/rewriter_partial_drop_ctl");
    let mut publishers2 = indexmap::IndexMap::new();
    publishers2.insert(
        "image".to_string(),
        cerulion_core::graph::node::AnyPublisher::Ipc(pubr2),
    );
    let ctx2 =
        cerulion_core::graph::node::NodeContext::for_tests(publishers2, indexmap::IndexMap::new());
    let mut entry2 = CompleteChildDropControlNodeEntry::new();
    entry2.init(ctx2).expect("init control");
    entry2.tick().expect("control tick");
    entry2.shutdown().expect("control shutdown");
    let mut count2 = 0usize;
    let _ = sub2.try_receive(|_| count2 += 1);
    assert_eq!(
        count2, 1,
        "the complete-child control must publish exactly one frame"
    );
}

/// Positive-control twin of [`PartialChildDropNode`]: identical shape, but
/// the child is COMPLETE (`frame_id` written) — the frame must publish.
#[cerulion_node(external)]
struct CompleteChildDropControlNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl CompleteChildDropControlNode {
    fn tick(&mut self) -> Result<(), cerulion_core::error::NodeError> {
        self.image.header.stamp.sec = 5;
        self.image.header.frame_id = "cam0";
        self.image.encoding = "rgb8";
        self.image.data = b"\x01";
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

// ============================================================
// Mixing staged sugar + whole-field write is a LOUD tick error
// ============================================================
//
// A complex-nested field written by BOTH the staged leaf sugar AND a
// whole-field write (either order) within one tick is a `NestedWriteConflict`.
// Because every port write carries `?`, the conflict propagates out of `tick`
// and NO frame is published (loud path, not a silent partial frame).

#[cerulion_node(external)]
struct StagedThenWholeNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl StagedThenWholeNode {
    fn tick(&mut self) -> Result<(), cerulion_core::error::NodeError> {
        self.image.header.frame_id = "cam0"; // staged leaf sugar
        self.image.header = b"\x01\x02\x03"; // whole-field write → conflict via `?`
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[test]
fn staged_then_whole_field_conflict_fails_tick_and_does_not_publish() {
    use cerulion_core::graph::node::NodeEntry;

    let tt = TestTransport::with_buffer_size(4);
    let pubr = tt.publisher("test/rewriter_conflict_sw", MaxSliceLen::const_new(4096), 0);
    let sub = tt.subscriber("test/rewriter_conflict_sw");

    let mut publishers = indexmap::IndexMap::new();
    publishers.insert(
        "image".to_string(),
        cerulion_core::graph::node::AnyPublisher::Ipc(pubr),
    );
    let ctx =
        cerulion_core::graph::node::NodeContext::for_tests(publishers, indexmap::IndexMap::new());

    let mut entry = StagedThenWholeNodeEntry::new();
    entry.init(ctx).expect("init");
    let err = entry
        .tick()
        .expect_err("a staged-then-whole conflict must surface as a tick error");
    assert!(
        err.to_string()
            .contains("Conflicting writes to nested field 'header'"),
        "got: {err}"
    );

    let mut count = 0usize;
    let _ = sub.try_receive(|_| count += 1);
    assert_eq!(count, 0, "a conflicting tick must not publish a frame");
}

#[cerulion_node(external)]
struct WholeThenStagedNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl WholeThenStagedNode {
    fn tick(&mut self) -> Result<(), cerulion_core::error::NodeError> {
        self.image.header = b"\x01\x02\x03"; // whole-field write
        self.image.header.frame_id = "cam0"; // staged sugar → conflict at first touch
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[test]
fn whole_field_then_staged_conflict_fails_tick_and_does_not_publish() {
    use cerulion_core::graph::node::NodeEntry;

    let tt = TestTransport::with_buffer_size(4);
    let pubr = tt.publisher("test/rewriter_conflict_ws", MaxSliceLen::const_new(4096), 0);
    let sub = tt.subscriber("test/rewriter_conflict_ws");

    let mut publishers = indexmap::IndexMap::new();
    publishers.insert(
        "image".to_string(),
        cerulion_core::graph::node::AnyPublisher::Ipc(pubr),
    );
    let ctx =
        cerulion_core::graph::node::NodeContext::for_tests(publishers, indexmap::IndexMap::new());

    let mut entry = WholeThenStagedNodeEntry::new();
    entry.init(ctx).expect("init");
    let err = entry
        .tick()
        .expect_err("a whole-then-staged conflict must surface as a tick error");
    assert!(
        err.to_string()
            .contains("Conflicting writes to nested field 'header'"),
        "got: {err}"
    );

    let mut count = 0usize;
    let _ = sub.try_receive(|_| count += 1);
    assert_eq!(count, 0, "a conflicting tick must not publish a frame");
}

// ============================================================
// Two output ports of the SAME schema
// ============================================================
//
// The schema-blind rewrite keys ONLY on the declared port ident. Two
// ports of the SAME schema type must route each assignment to its own
// `__cer_<port>` local (kills any port-vs-schema keying confusion —
// distinct oracles per port prove no cross-talk).

#[cerulion_node(external)]
struct DualImageNode {
    #[output]
    left: Image,
    #[output]
    right: Image,
}

#[cerulion_node_impl]
impl DualImageNode {
    fn tick(&mut self) -> Result<(), cerulion_core::error::NodeError> {
        self.left.height = 1;
        self.left.data = b"L";
        self.left.encoding = "l8";
        self.left.header = &[][..];
        self.right.height = 2;
        self.right.data = b"R";
        self.right.encoding = "r8";
        self.right.header = &[][..];
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[test]
fn two_same_schema_output_ports_route_to_their_own_shims() {
    use cerulion_core::graph::node::NodeEntry;

    let tt = TestTransport::with_buffer_size(4);
    let pub_l = tt.publisher("test/rewriter_dual_left", MaxSliceLen::const_new(4096), 0);
    let pub_r = tt.publisher("test/rewriter_dual_right", MaxSliceLen::const_new(4096), 0);
    let mut sub_l = tt.subscriber("test/rewriter_dual_left");
    let mut sub_r = tt.subscriber("test/rewriter_dual_right");

    let mut publishers = indexmap::IndexMap::new();
    publishers.insert(
        "left".to_string(),
        cerulion_core::graph::node::AnyPublisher::Ipc(pub_l),
    );
    publishers.insert(
        "right".to_string(),
        cerulion_core::graph::node::AnyPublisher::Ipc(pub_r),
    );
    let ctx =
        cerulion_core::graph::node::NodeContext::for_tests(publishers, indexmap::IndexMap::new());

    let mut entry = DualImageNodeEntry::new();
    entry.init(ctx).expect("init");
    entry.tick().expect("tick");
    entry.shutdown().expect("shutdown");

    let mut seen_l: Option<(u32, Vec<u8>, String)> = None;
    let _ = sub_l
        .try_view::<Image, _>(|view| {
            seen_l = Some((
                view.height,
                view.data().to_vec(),
                view.encoding().expect("utf8").to_string(),
            ));
        })
        .expect("recv left");
    let mut seen_r: Option<(u32, Vec<u8>, String)> = None;
    let _ = sub_r
        .try_view::<Image, _>(|view| {
            seen_r = Some((
                view.height,
                view.data().to_vec(),
                view.encoding().expect("utf8").to_string(),
            ));
        })
        .expect("recv right");

    let (hl, dl, el) = seen_l.expect("left frame");
    let (hr, dr, er) = seen_r.expect("right frame");
    assert_eq!((hl, dl.as_slice(), el.as_str()), (1, b"L".as_slice(), "l8"));
    assert_eq!((hr, dr.as_slice(), er.as_str()), (2, b"R".as_slice(), "r8"));
}

// ============================================================
// Keyword-named FIXED field via the shim
// ============================================================
//
// `sensor_msgs/JoyFeedback.type` is a keyword-named fixed field — the
// standard ROS2 pattern (Marker.type, SolidPrimitive.type, Shape.type).
// The rewriter's matched field name renders as `r#type`; it must strip
// the `r#` when building the shim ident (`__cer_assign_type` — codegen
// names shims with the RAW schema name). Without the strip the macro PANICS at
// expansion ("__cer_assign_r#type" is not a valid Ident), so this node
// COMPILING is the regression pin; the round-trip proves the write
// lands. (The fill_from rewrite shares the identical strip — see
// impl_macro.rs; no vendored schema has a keyword-named VARIABLE field
// to drive that arm end-to-end.)

#[cerulion_node(external)]
struct KeywordFieldNode {
    #[output]
    fb: JoyFeedback,
}

#[cerulion_node_impl]
impl KeywordFieldNode {
    fn tick(&mut self) -> Result<(), cerulion_core::error::NodeError> {
        self.fb.r#type = 1;
        self.fb.id = 7;
        self.fb.intensity = 0.5;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[test]
fn keyword_named_fixed_field_writes_through_shim() {
    use cerulion_core::graph::node::NodeEntry;

    let tt = TestTransport::with_buffer_size(4);
    let pubr = tt.publisher("test/rewriter_kw_field", MaxSliceLen::const_new(4096), 0);
    let mut sub = tt.subscriber("test/rewriter_kw_field");

    let mut publishers = indexmap::IndexMap::new();
    publishers.insert(
        "fb".to_string(),
        cerulion_core::graph::node::AnyPublisher::Ipc(pubr),
    );
    let ctx =
        cerulion_core::graph::node::NodeContext::for_tests(publishers, indexmap::IndexMap::new());

    let mut entry = KeywordFieldNodeEntry::new();
    entry.init(ctx).expect("init");
    entry.tick().expect("tick");
    entry.shutdown().expect("shutdown");

    let mut seen: Option<(u8, u8, f32)> = None;
    let _ = sub
        .try_view::<JoyFeedback, _>(|view| {
            seen = Some((view.r#type, view.id, view.intensity));
        })
        .expect("recv");
    let (ty, id, intensity) = seen.expect("one frame");
    assert_eq!(ty, 1, "keyword-named fixed field written through the shim");
    assert_eq!(id, 7);
    assert_eq!(intensity, 0.5);
}

// ============================================================
// A failed tick publishes NOTHING (fixed-only hole closed)
// ============================================================
//
// Without the inversion, on user-body Err the output proxies drop normally and
// `OutputProxy::Drop` SENDS unless an unwritten VARIABLE field trips the
// loud-discard — a fixed-only schema (no variable fields) ships a
// zero-init frame with valid headers/sequences downstream on EVERY failed
// tick. The publish-on-success inversion: each output is deferred the
// moment its FIRST write loans it (`LazyOutput::__cer_loan` →
// `__cer_defer_publish`, the lazy loan), only the tail ARMS publishing on
// a fully-Ok outcome (`__cer_arm_if_loaned`), and Drop releases an un-armed
// loan without sending (checked before the staged flush and the all-variables
// gate). The tick error itself stays the loud signal. A LOANED-but-discarded
// output (a partial/complete write THEN Err — the quiet-discard path)
// carries a DEBUG breadcrumb; an output the tick NEVER wrote never loans, so
// there is nothing to discard — it emits the per-skip TRACE non-event
// instead (`output not written this tick`).

/// Pure-Err tick on a FIXED-ONLY output (Vector3 — the exact hole): no
/// port write at all, tick just fails. Lazy-loan: because the tick writes `out`
/// NOWHERE, `out` is never loaned → nothing to discard → the per-skip TRACE
/// non-event fires (NOT the DEBUG discard breadcrumb; that still owns
/// loaned-but-unarmed proxies — see the partial/complete-write tests below).
#[cerulion_node(external)]
struct ErrTickFixedOnlyNode {
    #[output]
    out: native_ros2_messages::geometry_msgs::Vector3,
}

#[cerulion_node_impl]
impl ErrTickFixedOnlyNode {
    fn tick(&mut self) -> Result<(), cerulion_core::error::NodeError> {
        Err(cerulion_core::error::NodeError::Logic(
            "pure err".to_string(),
        ))
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[test]
#[tracing_test::traced_test]
fn err_tick_on_fixed_only_output_publishes_nothing_across_n_ticks() {
    use cerulion_core::graph::node::NodeEntry;

    let tt = TestTransport::with_buffer_size(4);
    let pubr = tt.publisher("test/rewriter_pure", MaxSliceLen::const_new(256), 0);
    let sub = tt.subscriber("test/rewriter_pure");

    let mut publishers = indexmap::IndexMap::new();
    publishers.insert(
        "out".to_string(),
        cerulion_core::graph::node::AnyPublisher::Ipc(pubr),
    );
    let ctx =
        cerulion_core::graph::node::NodeContext::for_tests(publishers, indexmap::IndexMap::new());

    let mut entry = ErrTickFixedOnlyNodeEntry::new();
    entry.init(ctx).expect("init");
    // N ticks, each erring — and each error must remain the USER error
    // ("pure err"), which doubles as the loan-release pin: if the
    // discard path leaked the iceoryx2 loan instead of returning the slot,
    // a later tick's `loan_proxy()?` would fail with a transport/loan
    // error instead.
    for i in 0..8 {
        let err = entry.tick().expect_err("tick must surface the user Err");
        assert!(
            err.to_string().contains("pure err"),
            "tick {i}: error must stay the USER error (a loan-exhaustion error here \
             would mean the discard path leaks slots); got: {err}"
        );
    }
    entry.shutdown().expect("shutdown");

    let mut count = 0usize;
    let _ = sub.try_receive(|_| count += 1);
    assert_eq!(
        count, 0,
        "a failed tick must publish NOTHING — without the inversion this receives 8 \
         zero-init Vector3 frames"
    );
    // The tick writes `out` NOWHERE, so it never loans — there is no
    // discard, hence no discard breadcrumb. The per-skip TRACE non-event fires
    // instead (captured here via `no-env-filter` traced_test).
    // The non-event is `trace!`: it exists only where `trace!` is compiled in.
    // Level-free twin: the per-skip TRACE non-event must never be LOUD — the half of the contract
    // that survives `release_max_level_info`, where the gated count below reads 0.
    logs_assert(|lines: &[&str]| {
        never_loud(lines, "output not written this tick")?;
        // Checked AT TRACE, not by text: the non-event re-emitted at `debug!`
        // still satisfies a `logs_contain`, and the loud sweep permits DEBUG.
        let skips = count_at_exclusively(lines, "TRACE", &["output not written this tick"])?;
        // EXACTLY one non-event per skipped output per tick: eight
        // erring ticks × one output = 8 where `trace!` is
        // compiled in, 0 where it is not — `>= 1` passed after a single event.
        let want = if trace_level_compiled_in() { 8 } else { 0 };
        if skips == want {
            Ok(())
        } else {
            Err(format!(
                "lazy-loan: a never-written output emits the per-skip TRACE \
                 non-event at TRACE once per tick (no loan → no discard), want {want} got \
                 {skips}"
            ))
        }
    });
    assert!(
        !logs_contain("output discarded, loan released without publish"),
        "lazy-loan: a never-written output never loans, so the DEBUG discard \
         breadcrumb must NOT fire (that path is for loaned-but-unarmed proxies)"
    );
}

/// Partial-write-then-Err: one fixed field written, then the tick fails.
#[cerulion_node(external)]
struct PartialWriteThenErrNode {
    #[output]
    out: native_ros2_messages::geometry_msgs::Vector3,
}

#[cerulion_node_impl]
impl PartialWriteThenErrNode {
    fn tick(&mut self) -> Result<(), cerulion_core::error::NodeError> {
        self.out.x = 1.5;
        Err(cerulion_core::error::NodeError::Logic(
            "partial err".to_string(),
        ))
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[test]
fn partial_write_then_err_publishes_nothing() {
    use cerulion_core::graph::node::NodeEntry;

    let tt = TestTransport::with_buffer_size(4);
    let pubr = tt.publisher("test/rewriter_partial", MaxSliceLen::const_new(256), 0);
    let sub = tt.subscriber("test/rewriter_partial");

    let mut publishers = indexmap::IndexMap::new();
    publishers.insert(
        "out".to_string(),
        cerulion_core::graph::node::AnyPublisher::Ipc(pubr),
    );
    let ctx =
        cerulion_core::graph::node::NodeContext::for_tests(publishers, indexmap::IndexMap::new());

    let mut entry = PartialWriteThenErrNodeEntry::new();
    entry.init(ctx).expect("init");
    entry.tick().expect_err("tick errs after the partial write");
    entry.shutdown().expect("shutdown");

    let mut count = 0usize;
    let _ = sub.try_receive(|_| count += 1);
    assert_eq!(
        count, 0,
        "a partial fixed write before the Err must not publish"
    );
}

/// Complete-write-then-Err: EVERY variable field written (including a
/// STAGED nested child) before the tick fails. Without the inversion this publishes (all
/// fields written → nothing trips the gate), and the Drop-step-0 flush
/// would even flush the staging first. The discard must preempt
/// BOTH — quietly (no flush-failure error, no unwritten-field error).
#[cerulion_node(external)]
struct CompleteWriteThenErrNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl CompleteWriteThenErrNode {
    fn tick(&mut self) -> Result<(), cerulion_core::error::NodeError> {
        self.image.encoding = "rgb8";
        self.image.data = b"\x01\x02";
        self.image.header.frame_id = "cam0"; // staged nested child
        Err(cerulion_core::error::NodeError::Logic(
            "complete err".to_string(),
        ))
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[test]
#[tracing_test::traced_test]
fn complete_staged_write_then_err_publishes_nothing_quietly() {
    use cerulion_core::graph::node::NodeEntry;

    let tt = TestTransport::with_buffer_size(4);
    let pubr = tt.publisher("test/rewriter_complete", MaxSliceLen::const_new(4096), 0);
    let sub = tt.subscriber("test/rewriter_complete");

    let mut publishers = indexmap::IndexMap::new();
    publishers.insert(
        "image".to_string(),
        cerulion_core::graph::node::AnyPublisher::Ipc(pubr),
    );
    let ctx =
        cerulion_core::graph::node::NodeContext::for_tests(publishers, indexmap::IndexMap::new());

    let mut entry = CompleteWriteThenErrNodeEntry::new();
    entry.init(ctx).expect("init");
    entry
        .tick()
        .expect_err("tick errs after the complete write");
    entry.shutdown().expect("shutdown");

    let mut count = 0usize;
    let _ = sub.try_receive(|_| count += 1);
    assert_eq!(
        count, 0,
        "complete-write-then-Err must not publish (the discard preempts the flush)"
    );
    // QUIET-discard pin: the staging is dropped WITHOUT flushing and
    // WITHOUT the child-gate — no flush-failure error, no unwritten-field
    // error. Only the DEBUG breadcrumb fires.
    assert!(
        !logs_contain("staged nested-field flush failed"),
        "the discard must preempt the staged flush (no flush error noise)"
    );
    assert!(
        !logs_contain("dropped without writing all declared variable fields"),
        "the discard must preempt the all-variables gate"
    );
    // The breadcrumb is `debug!`: it exists only where `debug!` is compiled in.
    // Level-free twin: the discard breadcrumb must never be LOUD — the half of the contract
    // that survives `release_max_level_info`, where the gated count below reads 0.
    logs_assert(|lines: &[&str]| {
        never_loud(lines, "output discarded, loan released without publish")?;
        // Checked AT DEBUG, not by text: the breadcrumb re-emitted at `trace!`
        // still satisfies a `logs_contain`, and the loud sweep permits TRACE.
        let breadcrumbs = count_at_exclusively(
            lines,
            "DEBUG",
            &["output discarded, loan released without publish"],
        )?;
        if breadcrumbs == debug_lines_expected(1) {
            Ok(())
        } else {
            Err(format!(
                "the DEBUG discard breadcrumb must fire at DEBUG EXACTLY once, got {breadcrumbs}"
            ))
        }
    });
}

/// Ok-path control: the SAME shape as the complete-write node but
/// returning Ok — publishes exactly one correct frame (the discard tail
/// must be inert on the Ok path).
#[cerulion_node(external)]
struct OkPathControlNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl OkPathControlNode {
    fn tick(&mut self) -> Result<(), cerulion_core::error::NodeError> {
        self.image.encoding = "rgb8";
        self.image.data = b"\x01\x02";
        self.image.header.frame_id = "cam0";
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[test]
fn ok_path_still_publishes_one_frame() {
    use cerulion_core::graph::node::NodeEntry;

    let tt = TestTransport::with_buffer_size(4);
    let pubr = tt.publisher("test/rewriter_ok", MaxSliceLen::const_new(4096), 0);
    let sub = tt.subscriber("test/rewriter_ok");

    let mut publishers = indexmap::IndexMap::new();
    publishers.insert(
        "image".to_string(),
        cerulion_core::graph::node::AnyPublisher::Ipc(pubr),
    );
    let ctx =
        cerulion_core::graph::node::NodeContext::for_tests(publishers, indexmap::IndexMap::new());

    let mut entry = OkPathControlNodeEntry::new();
    entry.init(ctx).expect("init");
    entry.tick().expect("tick");
    entry.shutdown().expect("shutdown");

    let mut count = 0usize;
    let _ = sub.try_receive(|_| count += 1);
    assert_eq!(
        count, 1,
        "the Ok path must keep publishing exactly one frame"
    );
}

// ============================================================
// The transport-Err leg: a TRANSPORT-errored tick publishes nothing
// ============================================================
//
// A try_view input chain whose `?` early-returned the TransportError
// OUT of the generated tick while the output proxies were live would drop
// them undiscarded (the same zero-init publish hole, transport-flavored;
// treated as the same class). The whole chain therefore runs inside the
// capture closure, so a SchemaMismatch from a wrong-schema frame on a
// data-trigger input aborts the tick BODY before it ever writes an output.
// Because the body never runs, no output is ever loaned — so there
// is nothing to discard; the per-skip TRACE non-event fires in place of the
// discard breadcrumb (the quiet-discard path still owns any output a
// running body DID loan before a later Err).

#[cerulion_node]
struct WrongSchemaInputNode {
    #[input(trigger)]
    inp: native_ros2_messages::geometry_msgs::Vector3,
    #[output]
    out: native_ros2_messages::geometry_msgs::Vector3,
}

#[cerulion_node_impl]
impl WrongSchemaInputNode {
    fn tick(&mut self) -> Result<(), cerulion_core::error::NodeError> {
        // Would publish a real frame if the input view ever materialized —
        // this body must NOT run when try_view rejects the frame.
        self.out.x = self.inp.x + 1.0;
        self.out.y = 2.0;
        self.out.z = 3.0;
        Ok(())
    }
}

#[test]
#[tracing_test::traced_test]
fn wrong_schema_input_frame_fails_tick_and_publishes_nothing() {
    use cerulion_core::graph::node::NodeEntry;

    let tt = TestTransport::with_buffer_size(4);
    // The node reads this topic as Vector3; a RAW publisher will put a
    // std_msgs/String frame (different schema hash) on it.
    let mut rogue_pub = tt.publisher(
        "test/rewriter_wrong_schema_in",
        MaxSliceLen::const_new(4096),
        0,
    );
    let node_in_sub = tt.subscriber("test/rewriter_wrong_schema_in");
    let node_out_pub = tt.publisher(
        "test/rewriter_wrong_schema_out",
        MaxSliceLen::const_new(256),
        0,
    );
    let out_sub = tt.subscriber("test/rewriter_wrong_schema_out");

    let mut publishers = indexmap::IndexMap::new();
    publishers.insert(
        "out".to_string(),
        cerulion_core::graph::node::AnyPublisher::Ipc(node_out_pub),
    );
    let mut subscribers = indexmap::IndexMap::new();
    subscribers.insert(
        "inp".to_string(),
        cerulion_core::graph::node::AnySubscriber::Ipc(node_in_sub),
    );
    let ctx = cerulion_core::graph::node::NodeContext::for_tests(publishers, subscribers);

    let mut entry = WrongSchemaInputNodeEntry::new();
    entry.init(ctx).expect("init");

    // Publish the WRONG-schema frame (String's hash != Vector3's) onto the
    // node's input topic AFTER its subscriber exists.
    {
        let mut proxy = rogue_pub.loan_proxy::<RosString>().expect("rogue loan");
        proxy.set_data("not-a-vector3").expect("rogue write");
        // Drop publishes.
    }

    // try_view::<Vector3> rejects the frame → the tick fails with the
    // TRANSPORT error (identity preserved — not wrapped as a user error).
    let err = entry
        .tick()
        .expect_err("wrong-schema input must fail the tick");
    assert!(
        err.to_string().contains("Schema mismatch"),
        "the SchemaMismatch transport error must surface unchanged; got: {err}"
    );
    entry.shutdown().expect("shutdown");

    // The body never ran (the input was rejected), so `out` was never loaned:
    // nothing to publish (without the inversion: one zero-init Vector3 frame here).
    let mut count = 0usize;
    let _ = out_sub.try_receive(|_| count += 1);
    assert_eq!(
        count, 0,
        "a transport-errored tick must publish NOTHING (transport leg)"
    );
    // The rejected input aborts the tick before the body writes `out`,
    // so `out` never loans — no discard, hence no discard breadcrumb. The
    // per-skip TRACE non-event fires instead (traced BEFORE the `?` re-raises
    // the transport error, so a skipped port is still surfaced on an escape).
    // Level-free twin: the per-skip TRACE non-event must never be LOUD — the half of the contract
    // that survives `release_max_level_info`, where the gated count below reads 0.
    logs_assert(|lines: &[&str]| {
        never_loud(lines, "output not written this tick")?;
        // Checked AT TRACE, not by text (see the sibling arm above).
        let skips = count_at_exclusively(lines, "TRACE", &["output not written this tick"])?;
        // EXACTLY one: a single aborted tick, one output.
        let want = if trace_level_compiled_in() { 1 } else { 0 };
        if skips == want {
            Ok(())
        } else {
            Err(format!(
                "lazy-loan: an output the aborted body never wrote emits the \
                 per-skip TRACE non-event at TRACE (no loan → no discard), got {skips}"
            ))
        }
    });
    assert!(
        !logs_contain("output discarded, loan released without publish"),
        "lazy-loan: `out` never loaned on the transport-Err leg, so the DEBUG \
         discard breadcrumb must NOT fire"
    );
}

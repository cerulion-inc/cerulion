// SPDX-License-Identifier: AGPL-3.0-only
//! Integration tests for `#[cerulion_node_impl]` AST-rewriting macro
//! (tick rewriting plus the helper rewriter).
//!
//! Verifies that the impl-block macro:
//!   1. Compiles a `tick` method that uses `self.<port>` for both inputs
//!      and outputs.
//!   2. Generates a `__cer_zero_copy_tick(ctx)` method on the inner struct.
//!   3. The generated method writes outputs directly into the
//!      iceoryx2-loaned SHM (proven by round-tripping through the full
//!      graph and observing the published value).
//!   4. The struct macro's `<Name>Entry::tick` correctly dispatches to
//!      `__cer_zero_copy_tick` when `zero_copy` is set on `#[cerulion_node]`.
//!
//! Coverage also includes:
//!   - **A1:** variable-schema outputs (`set_<field>` / `loan_<field>` /
//!     `push_<field>` / fixed-field-on-variable-schema setters).
//!   - **A2:** helper methods on the impl block — `self.<port>` references
//!     inside helpers are rewritten and the helper signature gains the
//!     necessary `&mut LazyOutput<'_, T>` (the lazy-loan slot) /
//!     `&InputView<'_, T>` params.
//!   - **A3:** multi-input tick — two or more `#[input]` ports with all
//!     views in scope simultaneously inside the user body.
//!
//! These tests run over the iceoryx2 `TestTransport` helper,
//! using an isolated per-test transport instance.

use cerulion_core::wire::MaxSliceLen;

use cerulion_core::graph::node::{AnyPublisher, AnySubscriber, NodeContext, NodeEntry};
use cerulion_core::prelude::*;
use cerulion_core::testing::TestTransport;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use native_ros2_messages::sensor_msgs::Image;

// ===========================================================================
// Baseline: source node with no inputs, fixed-schema output.
// ===========================================================================

#[cerulion_node(period_ms = 10)]
struct ZcSourceNode {
    // Marker field only: the impl macro rewrites `self.vec_out.<f>` to
    // `__cer_vec_out.<f>` (the per-tick OutputProxy<Vector3>), so the
    // unit-struct field is never read directly by user code.
    #[output]
    vec_out: Vector3,
    tick_count: u32,
}

#[cerulion_node_impl]
impl ZcSourceNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.tick_count += 1;
        // After the impl-macro rewrite these become direct stores into
        // the OutputProxy<Vector3>'s `Vector3Shm` writer (which is the
        // iceoryx2-loaned SHM region). No snapshot, no memcpy.
        self.vec_out.x = self.tick_count as f64;
        self.vec_out.y = self.tick_count as f64 * 2.0;
        self.vec_out.z = self.tick_count as f64 * 3.0;
        Ok(())
    }
}

#[test]
fn zero_copy_source_info_round_trips() {
    let entry = ZcSourceNodeEntry::new();
    let info = entry.info().expect("info should parse");
    assert_eq!(info.output_names(), vec!["vec_out"]);
    assert!(info.input_names().is_empty());
}

#[test]
fn zero_copy_source_publishes_vector3_through_shm() {
    let topic = "test/zc/vec_out";
    let tt = TestTransport::with_buffer_size(8);
    let pubr = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let mut sub = tt.subscriber(topic);

    let mut publishers: IndexMap<String, AnyPublisher> = IndexMap::new();
    publishers.insert("vec_out".to_string(), AnyPublisher::Ipc(pubr));
    let context = NodeContext::for_tests(publishers, IndexMap::new());

    let mut entry = ZcSourceNodeEntry::new();
    entry.init(context).expect("init");

    entry.tick().expect("tick 1");
    entry.tick().expect("tick 2");
    entry.tick().expect("tick 3");

    // Drain the subscriber and read the *last* Vector3 view.
    let mut last_x: Option<f64> = None;
    let mut last_y: Option<f64> = None;
    let mut last_z: Option<f64> = None;
    while let Ok(Some(())) = sub.try_view::<Vector3, _>(|view| {
        last_x = Some(view.x);
        last_y = Some(view.y);
        last_z = Some(view.z);
    }) {}

    // After 3 ticks: x=3, y=6, z=9.
    assert_eq!(last_x, Some(3.0), "last_x should be tick_count = 3");
    assert_eq!(last_y, Some(6.0), "last_y should be tick_count * 2 = 6");
    assert_eq!(last_z, Some(9.0), "last_z should be tick_count * 3 = 9");
}

// ===========================================================================
// A1 — Variable-schema outputs.
// ===========================================================================
//
// Image is a variable schema with fixed fields (height, width, step,
// is_bigendian) and three variable fields (header, encoding, data).
// Direct `self.image.<field> = ...` doesn't apply for variable schemas
// because codegen exposes `set_<field>(...)` accessors rather than
// `pub` fields. The tick rewriter `self.<port> -> __cer_<port>`
// passes through unchanged for `set_<field>`/`loan_<field>`/`push_<field>`
// method calls because they resolve via `OutputProxy::DerefMut` to
// `<Image as ShmMessage>::Writer<'_>`.

#[cerulion_node(period_ms = 33)]
struct ZcImageSource {
    #[output]
    image: Image,
    tick_count: u32,
}

#[cerulion_node_impl]
impl ZcImageSource {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.tick_count += 1;

        // Fixed-section direct field writes on a variable schema (they go
        // through Deref<Target = ImageFixedSection>: single-mov SHM stores).
        self.image.height = 2 + self.tick_count;
        self.image.width = 8 + self.tick_count;
        self.image.step = 16 + self.tick_count;
        self.image.is_bigendian = 0;

        // header_bytes is a variable bytes field — must be written for
        // all_variables_written() to be true. Use set_header_bytes to
        // exercise the boundary memcpy path.
        self.image.set_header_bytes(&[0u8; 0])?;

        // encoding is a variable string field — set_<field> path.
        self.image.set_encoding("rgb8")?;

        // data is a variable bytes field — exercise the loan_<field>
        // path (the true zero-copy primitive). Write a small payload
        // straight into the loaned SHM slice.
        let payload_len = 4usize;
        let dst: &mut [u8] = self.image.loan_data(payload_len)?;
        for (i, b) in dst.iter_mut().enumerate() {
            *b = (self.tick_count as u8).wrapping_add(i as u8);
        }

        Ok(())
    }
}

#[test]
fn variable_schema_zero_copy_publish() {
    // Image needs a max_slice_len large enough for header + fixed +
    // offset table + variable payload (header_bytes + encoding + data).
    // 256 bytes is plenty for the small payload we write.
    let topic = "test/zc/image";
    let tt = TestTransport::with_buffer_size(8);
    let pubr = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let mut sub = tt.subscriber(topic);

    let mut publishers: IndexMap<String, AnyPublisher> = IndexMap::new();
    publishers.insert("image".to_string(), AnyPublisher::Ipc(pubr));
    let context = NodeContext::for_tests(publishers, IndexMap::new());

    let mut entry = ZcImageSourceEntry::new();
    entry.init(context).expect("init");

    entry.tick().expect("tick 1");
    entry.tick().expect("tick 2");

    // Drain the subscriber and read the last view's fields.
    let mut last_height: Option<u32> = None;
    let mut last_width: Option<u32> = None;
    let mut last_step: Option<u32> = None;
    let mut last_encoding: Option<String> = None;
    let mut last_data: Option<Vec<u8>> = None;
    while let Ok(Some(())) = sub.try_view::<Image, _>(|view| {
        last_height = Some(view.height);
        last_width = Some(view.width);
        last_step = Some(view.step);
        last_encoding = Some(view.encoding().expect("encoding utf-8").to_string());
        last_data = Some(view.data().to_vec());
    }) {}

    // After 2 ticks: tick_count=2, so height=4, width=10, step=18.
    assert_eq!(last_height, Some(4), "height should be 2 + tick_count = 4");
    assert_eq!(last_width, Some(10), "width should be 8 + tick_count = 10");
    assert_eq!(last_step, Some(18), "step should be 16 + tick_count = 18");
    assert_eq!(last_encoding.as_deref(), Some("rgb8"));
    // data was written via loan_data: 4 bytes starting at tick_count=2,
    // then [2,3,4,5].
    assert_eq!(last_data.as_deref(), Some(&[2u8, 3, 4, 5][..]));
}

// ===========================================================================
// A2 — Helper-method rewriting.
// ===========================================================================
//
// Helpers reference ports through `self.<port>`. The helper rewriter
// detects each helper's port references, appends the per-port references
// (`&mut LazyOutput<'_, T>` for outputs — the lazy-loan slot, loaned
// on first write — `&InputView<'_, T>` for inputs) to its parameter list, and
// rewrites every `self.<helper>(args)` call in `tick` (and in other helpers)
// to pass the per-tick locals.
//
// This source node uses a helper that fills the entire Vector3 from a
// scalar, plus a second helper that itself calls the first helper —
// proving the rewriter handles arbitrary helper-helper chains.

#[cerulion_node(period_ms = 10)]
struct ZcHelperSource {
    #[output]
    vec_out: Vector3,
    tick_count: u32,
}

#[cerulion_node_impl]
impl ZcHelperSource {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.tick_count += 1;
        let scalar = self.tick_count as f64;
        // Helper-of-helper chain: fill_pose calls fill_xyz which writes
        // through the per-tick OutputProxy<Vector3>.
        self.fill_pose(scalar)?;
        Ok(())
    }

    /// Helper that itself only delegates to another helper — exercises
    /// the case where a method does NOT reference ports directly but
    /// must still pass through the per-tick locals to a callee that does.
    /// Port-writing helper chains are FALLIBLE: every rewritten
    /// assignment appends `?`, so helpers that (transitively) write ports
    /// must return `Result`.
    fn fill_pose(&mut self, scalar: f64) -> Result<(), NodeError> {
        self.fill_xyz(scalar)
    }

    /// Leaf helper that writes through the port directly. The macro
    /// detects the `self.vec_out.<field>` references, rewrites the
    /// assignments to `__cer_vec_out.__cer_loan()?.__cer_assign_<field>(&value)?`
    /// (the write loans the slot), and appends
    /// `__cer_vec_out: &mut LazyOutput<'_, Vector3>` to the parameter list.
    fn fill_xyz(&mut self, scalar: f64) -> Result<(), NodeError> {
        self.vec_out.x = scalar;
        self.vec_out.y = scalar * 2.0;
        self.vec_out.z = scalar * 3.0;
        Ok(())
    }
}

#[test]
fn helper_method_rewrites_port_access() {
    let topic = "test/zc/helper_vec_out";
    let tt = TestTransport::with_buffer_size(8);
    let pubr = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let mut sub = tt.subscriber(topic);

    let mut publishers: IndexMap<String, AnyPublisher> = IndexMap::new();
    publishers.insert("vec_out".to_string(), AnyPublisher::Ipc(pubr));
    let context = NodeContext::for_tests(publishers, IndexMap::new());

    let mut entry = ZcHelperSourceEntry::new();
    entry.init(context).expect("init");

    entry.tick().expect("tick 1");
    entry.tick().expect("tick 2");

    let mut last_x: Option<f64> = None;
    let mut last_y: Option<f64> = None;
    let mut last_z: Option<f64> = None;
    while let Ok(Some(())) = sub.try_view::<Vector3, _>(|view| {
        last_x = Some(view.x);
        last_y = Some(view.y);
        last_z = Some(view.z);
    }) {}

    // After 2 ticks: scalar=2, so x=2, y=4, z=6.
    assert_eq!(last_x, Some(2.0));
    assert_eq!(last_y, Some(4.0));
    assert_eq!(last_z, Some(6.0));
}

// ===========================================================================
// A3 — Multi-input zero-copy tick.
// ===========================================================================
//
// Two `#[input]` ports + one `#[output]` port. The helper rewriter
// splits ctx into disjoint publishers/subscribers borrows, takes both
// subscribers off the subscribers map via `IndexMap::get_disjoint_mut`,
// then nests `try_view` closures so both views are in scope
// simultaneously inside the user body.

#[cerulion_node]
struct ZcTwoInOneOut {
    #[input(trigger)]
    a: Vector3,
    #[input]
    b: Vector3,
    #[output]
    sum: Vector3,
}

#[cerulion_node_impl]
impl ZcTwoInOneOut {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Both `self.a` and `self.b` are rewritten to `__cer_a` and
        // `__cer_b` — the per-input InputView<'_, Vector3> handles
        // populated by the nested try_view chain.
        self.sum.x = self.a.x + self.b.x;
        self.sum.y = self.a.y + self.b.y;
        self.sum.z = self.a.z + self.b.z;
        Ok(())
    }
}

#[test]
fn multi_input_zero_copy_tick() {
    // Build two upstream publishers and a downstream subscriber for
    // the sum output. We feed each input one Vector3, then tick the
    // node, then check the sum.
    let tt = TestTransport::with_buffer_size(4);
    let topic_a = "test/zc/multi/a";
    let topic_b = "test/zc/multi/b";
    let topic_sum = "test/zc/multi/sum";
    let mut pub_a = tt.publisher(topic_a, MaxSliceLen::const_new(256), 0);
    let mut pub_b = tt.publisher(topic_b, MaxSliceLen::const_new(256), 0);
    let pub_sum = tt.publisher(topic_sum, MaxSliceLen::const_new(256), 0);
    let sub_a = tt.subscriber(topic_a);
    let sub_b = tt.subscriber(topic_b);
    let mut sub_sum = tt.subscriber(topic_sum);

    // Publish a = (1,2,3) and b = (10,20,30).
    {
        let mut p = pub_a.loan_proxy::<Vector3>().expect("loan a");
        p.x = 1.0;
        p.y = 2.0;
        p.z = 3.0;
    }
    {
        let mut p = pub_b.loan_proxy::<Vector3>().expect("loan b");
        p.x = 10.0;
        p.y = 20.0;
        p.z = 30.0;
    }

    // Build the node context: sum output is a publisher; a and b are
    // subscribers (we only need the sub side; the upstream pubs are
    // already detached and feeding the queues).
    let mut publishers: IndexMap<String, AnyPublisher> = IndexMap::new();
    publishers.insert("sum".to_string(), AnyPublisher::Ipc(pub_sum));
    let mut subscribers: IndexMap<String, AnySubscriber> = IndexMap::new();
    subscribers.insert("a".to_string(), AnySubscriber::Ipc(sub_a));
    subscribers.insert("b".to_string(), AnySubscriber::Ipc(sub_b));
    let context = NodeContext::for_tests(publishers, subscribers);

    let mut entry = ZcTwoInOneOutEntry::new();
    entry.init(context).expect("init");
    entry.tick().expect("tick");

    let mut last_sum: Option<(f64, f64, f64)> = None;
    while let Ok(Some(())) = sub_sum.try_view::<Vector3, _>(|view| {
        last_sum = Some((view.x, view.y, view.z));
    }) {}

    assert_eq!(
        last_sum,
        Some((11.0, 22.0, 33.0)),
        "sum should be a + b = (11, 22, 33)"
    );
}

// ===========================================================================
// Random-order writes for variable-schema fields.
// ===========================================================================
//
// The wire format uses an offset-table preamble in the variable section, so
// writing variable fields in a different order than declaration order MUST
// still round-trip correctly: each `set_<field>`/`loan_<field>` updates its
// own offset-table slot independently, and the reader resolves each field
// through its own offset entry.
//
// This regression test pins the structural property even if a future
// rewrite of the writer would otherwise let "declaration order" bleed into
// the on-wire layout (e.g. by appending payload sequentially based on call
// order rather than per-field cursor bookkeeping).

#[cerulion_node(period_ms = 33)]
struct ZcReverseOrderImage {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl ZcReverseOrderImage {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Image declares variable fields in this order:
        //   header_bytes, encoding, data
        //
        // Write them in REVERSE: data first, then encoding, then
        // header_bytes. Mix in the fixed-field assignments in between so the
        // test also exercises interleaved fixed/variable writes.
        self.image.height = 7;

        // Variable field #3 first (data via loan + direct write).
        let dst: &mut [u8] = self.image.loan_data(4)?;
        dst.copy_from_slice(&[0xAA, 0xBB, 0xCC, 0xDD]);

        self.image.width = 8;

        // Variable field #2 next (encoding string).
        self.image.set_encoding("rgb8")?;

        self.image.step = 9;

        // Variable field #1 last (header_bytes).
        self.image.set_header_bytes(&[0x01, 0x02, 0x03])?;

        self.image.is_bigendian = 0;

        Ok(())
    }
}

#[test]
fn variable_schema_random_order_writes() {
    let topic = "test/zc/reverse_image";
    let tt = TestTransport::with_buffer_size(8);
    let pubr = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let mut sub = tt.subscriber(topic);

    let mut publishers: IndexMap<String, AnyPublisher> = IndexMap::new();
    publishers.insert("image".to_string(), AnyPublisher::Ipc(pubr));
    let context = NodeContext::for_tests(publishers, IndexMap::new());

    let mut entry = ZcReverseOrderImageEntry::new();
    entry.init(context).expect("init");
    entry.tick().expect("tick");

    let mut last_height: Option<u32> = None;
    let mut last_width: Option<u32> = None;
    let mut last_step: Option<u32> = None;
    let mut last_encoding: Option<String> = None;
    let mut last_data: Option<Vec<u8>> = None;
    let mut last_header: Option<Vec<u8>> = None;
    while let Ok(Some(())) = sub.try_view::<Image, _>(|view| {
        last_height = Some(view.height);
        last_width = Some(view.width);
        last_step = Some(view.step);
        last_encoding = Some(view.encoding().expect("encoding utf-8").to_string());
        last_data = Some(view.data().to_vec());
        last_header = Some(view.header_bytes().to_vec());
    }) {}

    // Every field round-trips correctly even though variable fields were
    // written in reverse declaration order with fixed setters interleaved.
    assert_eq!(last_height, Some(7));
    assert_eq!(last_width, Some(8));
    assert_eq!(last_step, Some(9));
    assert_eq!(last_encoding.as_deref(), Some("rgb8"));
    assert_eq!(last_data.as_deref(), Some(&[0xAAu8, 0xBB, 0xCC, 0xDD][..]));
    assert_eq!(last_header.as_deref(), Some(&[0x01u8, 0x02, 0x03][..]));
}

// Last-write-wins for the same variable field: writing `encoding` twice
// must surface only the second value to the subscriber. This ensures the
// offset-table slot is overwritten in-place rather than appended-to.

#[cerulion_node(period_ms = 33)]
struct ZcDoubleWriteImage {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl ZcDoubleWriteImage {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.image.height = 1;
        self.image.width = 1;
        self.image.step = 1;
        self.image.is_bigendian = 0;

        self.image.set_header_bytes(&[])?;

        // First write — should be overwritten by the second.
        self.image.set_encoding("first_value")?;
        // Second write — must be the value the subscriber observes.
        self.image.set_encoding("second_value")?;

        // Same pattern for `data` via loan_data: write a sentinel, then
        // re-loan and write a different value. Last write wins.
        {
            let dst1: &mut [u8] = self.image.loan_data(2)?;
            dst1.copy_from_slice(&[0x11, 0x22]);
        }
        {
            let dst2: &mut [u8] = self.image.loan_data(2)?;
            dst2.copy_from_slice(&[0x77, 0x88]);
        }
        Ok(())
    }
}

#[test]
fn variable_schema_last_write_wins() {
    let topic = "test/zc/double_write_image";
    let tt = TestTransport::with_buffer_size(8);
    let pubr = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let mut sub = tt.subscriber(topic);

    let mut publishers: IndexMap<String, AnyPublisher> = IndexMap::new();
    publishers.insert("image".to_string(), AnyPublisher::Ipc(pubr));
    let context = NodeContext::for_tests(publishers, IndexMap::new());

    let mut entry = ZcDoubleWriteImageEntry::new();
    entry.init(context).expect("init");
    entry.tick().expect("tick");

    let mut last_encoding: Option<String> = None;
    let mut last_data: Option<Vec<u8>> = None;
    while let Ok(Some(())) = sub.try_view::<Image, _>(|view| {
        last_encoding = Some(view.encoding().expect("encoding utf-8").to_string());
        last_data = Some(view.data().to_vec());
    }) {}

    assert_eq!(
        last_encoding.as_deref(),
        Some("second_value"),
        "second set_encoding call must overwrite the first",
    );
    assert_eq!(
        last_data.as_deref(),
        Some(&[0x77u8, 0x88][..]),
        "second loan_data write must overwrite the first",
    );
}

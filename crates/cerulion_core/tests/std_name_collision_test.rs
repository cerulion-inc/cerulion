// SPDX-License-Identifier: AGPL-3.0-only
//! Std-name collision regression test for the macro emission paths
//! (items E and F below).
//!
//! # Item E — `String`/`Empty` resolve through the user's `use`
//!
//! Schemas whose names shadow std types — `std_msgs::String`,
//! `std_msgs::Empty` (which doesn't shadow but is short and lives next
//! door), etc. — used to require path-qualified field types because
//! the earlier snapshot path derived marker names by stripping
//! `Snapshot` from the user's field type via `format_ident!`. Now
//! the macro consumes the user's `syn::Type` directly, and
//! the user's `use` statement shadows the std prelude entry inside
//! the field's hygiene span — so unqualified `String` resolves to the
//! schema marker without any macro-side trickery.
//!
//! This test pins that contract: a node defined with unqualified
//! `String` and `Empty` field types compiles cleanly and round-trips
//! through the in-process transport.
//!
//! # Item F — marker imports survive `#[deny(unused_imports)]`
//!
//! The crate-level `#![deny(unused_imports)]` below proves the imports of
//! `String` and `Empty` are *consumed* by the macro emission (which uses
//! them as type arguments to `loan_proxy::<T>()`, `try_view::<T, _>(...)`,
//! `OutputProxy<'_, T>`, and `InputView<'_, T>`). Type-argument position
//! counts as a use for `unused_imports`, so the imports never get flagged
//! even though no user-visible code references them by name.

#![deny(unused_imports)]

use cerulion_core::wire::MaxSliceLen;

// Bring the schemas into scope without renaming. `String` here shadows
// `std::string::String` in this module's namespace, which is exactly
// the situation users have when they `use native_ros2_messages::std_msgs::*`.
use native_ros2_messages::std_msgs::Empty;
use native_ros2_messages::std_msgs::String;

use cerulion_core::graph::node::{AnyPublisher, AnySubscriber, NodeContext, NodeEntry, NodeInfo};
use cerulion_core::prelude::*;
use cerulion_core::testing::TestTransport;
use indexmap::IndexMap;

#[cerulion_node]
struct StdCollisionNode {
    #[input(trigger)]
    name_in: String,

    #[output]
    echo: String,

    #[output]
    pulse: Empty,
}

#[cerulion_node_impl]
impl StdCollisionNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // 1. Echo the input string back out unchanged.
        //    `data()` returns `Result<&str, WireError>`; map the wire error
        //    to a NodeError on the boundary.
        let received = self
            .name_in
            .data()
            .map_err(|e| NodeError::Logic(format!("name_in utf-8: {e}")))?;
        self.echo.set_data(received)?;
        // 2. `pulse` is an Empty (fixed, zero-field) output — a heartbeat. An
        //    `Empty` schema has NO field to write, so under lazy-loan
        //    there is no field-write to trigger the loan; the explicit
        //    `emit()` gesture (Task A) is how a fieldless output publishes. The
        //    macro rewrites `self.pulse.emit()?` to loan the proxy (the publish
        //    intent), and the tick tail arms it on this fully-Ok tick → `pulse`
        //    publishes an Empty frame. Only outputs you `emit()` are published;
        //    an un-emitted fieldless output would be a zero-traffic non-event.
        self.pulse.emit()?;
        Ok(())
    }
}

/// Helper: assert generated `NodeInfo` carries both ports as expected.
fn nominal_info() -> NodeInfo {
    let entry = StdCollisionNodeEntry::new();
    entry.info().expect("info should parse")
}

#[test]
fn std_name_collision_macro_compiles_and_emits_info() {
    let info = nominal_info();
    assert_eq!(info.input_names(), vec!["name_in".to_string()]);
    assert_eq!(
        info.output_names(),
        vec!["echo".to_string(), "pulse".to_string()]
    );
}

#[test]
fn std_name_collision_round_trips_through_in_process() {
    let tt = TestTransport::with_buffer_size(8);

    // Source publisher feeds `name_in`; sink subscribers read `echo` + `pulse`.
    let mut source_pub = tt.publisher("topic/name", MaxSliceLen::const_new(256), 0);
    let echo_pub = tt.publisher("topic/echo", MaxSliceLen::const_new(256), 0);
    let pulse_pub = tt.publisher("topic/pulse", MaxSliceLen::const_new(256), 0);

    let in_sub = tt.subscriber("topic/name");
    let mut echo_sink = tt.subscriber("topic/echo");
    let mut pulse_sink = tt.subscriber("topic/pulse");

    // Wire the node: echo + pulse publishers from above (NOT the source —
    // that's the upstream feed for the input) and `name_in` subscriber.
    let mut pubs = IndexMap::new();
    pubs.insert("echo".to_string(), AnyPublisher::Ipc(echo_pub));
    pubs.insert("pulse".to_string(), AnyPublisher::Ipc(pulse_pub));
    let mut subs = IndexMap::new();
    subs.insert("name_in".to_string(), AnySubscriber::Ipc(in_sub));

    let mut entry = StdCollisionNodeEntry::new();
    entry
        .init(NodeContext::for_tests(pubs, subs))
        .expect("init should succeed");

    // Send a message into name_in.
    {
        let mut p = source_pub
            .loan_proxy::<String>()
            .expect("loan source proxy");
        p.set_data("hello collision").expect("set_data");
    }

    // Tick the node — should consume the input, emit echo + pulse.
    entry.tick().expect("tick should succeed");

    // Verify echo carries the round-tripped string.
    let echoed = echo_sink
        .try_view::<String, _>(|view| view.data().expect("utf-8").to_owned())
        .expect("try_view should not error")
        .expect("echo subscriber should have a sample");
    assert_eq!(echoed, "hello collision");

    // `pulse` (an Empty, zero-field output) publishes because
    // the tick calls `self.pulse.emit()?` — the explicit gesture that loans +
    // arms a fieldless output. So the pulse subscriber DOES receive an Empty
    // frame. (A review flipped this to `is_none()` when a fieldless output was
    // un-emittable. The `emit()` gesture restores emittability, and with it this
    // original collision-shape delivery coverage.)
    let pulse_observed = pulse_sink
        .try_view::<Empty, _>(|_view| ())
        .expect("try_view on Empty should not error");
    assert!(
        pulse_observed.is_some(),
        "lazy-loan: an `emit()`ed Empty output must publish — pulse subscriber \
         should have received a sample"
    );
}

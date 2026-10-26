// SPDX-License-Identifier: AGPL-3.0-only
//! Two instances of one node library, loaded into one process, tick at the
//! same time.
//!
//! A library's generated code keeps its instances in one process-wide table.
//! Each instance has its own lock, and the table's lock is held only to look an
//! instance up, so a tick that is still running on one instance does not hold a
//! tick of another instance of the same library. Two executors in one process
//! may host one node type twice; this is what keeps them independent.
//!
//! The fixture's tick waits while a hold file exists. Instance A ticks on a
//! thread with its hold file present; instance B, with no hold file, must
//! finish its own tick while A is still inside its tick.
//!
//! # Running
//!
//! ```bash
//! cargo build -p test_node_macro_gate_cdylib
//! cargo test -p cerulion_core --test cdylib_instance_lock_test
//! ```

use std::sync::mpsc;
use std::time::{Duration, Instant};

use cerulion_core::graph::node::{
    AnyPublisher, AnySubscriber, DylibNodeEntry, NodeContext, NodeEntry,
};
use cerulion_core::graph::params::{ParamValue, ParamValues};
use cerulion_core::testing::TestTransport;
use cerulion_core::transport::subscriber::CerulionSubscriber;
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

const GATE_FIXTURE: &str = "test_node_macro_gate_cdylib";

/// How long instance B's tick may take while A holds its own tick open.
/// B's tick does no waiting, so anything near this bound is a block.
const B_TICK_BOUND: Duration = Duration::from_secs(2);

fn wire(tt: &TestTransport, tag: &str) -> (NodeContext, CerulionSubscriber) {
    let topic = format!("test/cdylib_instance_lock/{tag}/cmd");
    let publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 0);
    let subscriber = tt.subscriber(&topic);
    let mut publishers: IndexMap<String, AnyPublisher> = IndexMap::new();
    publishers.insert("cmd".to_string(), AnyPublisher::Ipc(publisher));
    let subscribers: IndexMap<String, AnySubscriber> = IndexMap::new();
    (NodeContext::for_tests(publishers, subscribers), subscriber)
}

fn instance(
    tt: &TestTransport,
    tag: &str,
    hold: &str,
    x: f64,
) -> (DylibNodeEntry, CerulionSubscriber) {
    let path = cerulion_core::testing::find_fixture_cdylib(GATE_FIXTURE);
    let mut node = DylibNodeEntry::load(&path).expect("load gate fixture");
    let params: ParamValues = [
        ("hold_path".to_string(), ParamValue::Str(hold.to_string())),
        ("tag".to_string(), ParamValue::Float(x)),
    ]
    .into_iter()
    .collect();
    node.set_params(&params).expect("set_params");
    let (ctx, sub) = wire(tt, tag);
    node.init(ctx).expect("init");
    (node, sub)
}

fn read_x(sub: &mut CerulionSubscriber) -> Option<f64> {
    sub.try_view::<Vector3, _>(|v| v.x).expect("try_view cmd")
}

#[test]
fn a_held_tick_of_one_instance_does_not_hold_another_instance_of_the_same_library() {
    let tt = TestTransport::with_buffer_size(4);
    let dir = std::path::Path::new(env!("CARGO_TARGET_TMPDIR"))
        .join(format!("cdylib_instance_lock_{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("create hold dir");
    let hold = dir.join("hold_a");
    std::fs::write(&hold, b"held").expect("create hold file");

    let (mut a, mut sub_a) = instance(&tt, "a", hold.to_str().expect("utf-8 path"), 1.0);
    let (mut b, mut sub_b) = instance(&tt, "b", "", 2.0);

    // A ticks on its own thread and stays inside the tick while the file exists.
    let (a_done_tx, a_done_rx) = mpsc::channel();
    let a_thread = std::thread::spawn(move || {
        a.tick().expect("tick a");
        a_done_tx.send(()).expect("report a");
        a
    });
    // Give A's tick time to enter and take whatever it takes.
    std::thread::sleep(Duration::from_millis(100));
    assert!(
        a_done_rx.try_recv().is_err(),
        "instance A's tick returned while its hold file exists: the fixture does not hold, \
         so this run proves nothing"
    );

    // B's tick on another thread, with a bound.
    let (b_done_tx, b_done_rx) = mpsc::channel();
    let b_thread = std::thread::spawn(move || {
        b.tick().expect("tick b");
        b_done_tx.send(()).expect("report b");
        b
    });
    let started = Instant::now();
    let b_finished = b_done_rx.recv_timeout(B_TICK_BOUND).is_ok();
    let b_elapsed = started.elapsed();
    let a_still_held = a_done_rx.try_recv().is_err();

    // Release A whatever happened, so neither thread outlives the test.
    std::fs::remove_file(&hold).expect("remove hold file");
    let mut a = a_thread.join().expect("join a");
    let mut b = b_thread.join().expect("join b");

    assert!(
        b_finished,
        "instance B's tick did not finish within {B_TICK_BOUND:?} while instance A of the same \
         library was inside its tick: the two instances share one lock across the tick"
    );
    assert!(
        a_still_held,
        "instance A finished before B was measured, so B's tick was not concurrent with A's"
    );
    assert!(
        b_elapsed < Duration::from_millis(500),
        "instance B's tick took {b_elapsed:?}; it does no waiting of its own"
    );

    // Both instances published their own value: B's frame exists before A's
    // release, A's after it.
    let deadline = Instant::now() + Duration::from_secs(2);
    let (mut got_a, mut got_b) = (None, None);
    while (got_a.is_none() || got_b.is_none()) && Instant::now() < deadline {
        got_a = got_a.or_else(|| read_x(&mut sub_a));
        got_b = got_b.or_else(|| read_x(&mut sub_b));
        std::thread::sleep(Duration::from_millis(5));
    }
    assert_eq!(got_a, Some(1.0), "instance A's frame");
    assert_eq!(got_b, Some(2.0), "instance B's frame");

    a.shutdown().expect("shutdown a");
    b.shutdown().expect("shutdown b");
}

#[test]
fn a_shut_down_handle_is_not_found_and_its_twin_keeps_ticking() {
    let tt = TestTransport::with_buffer_size(4);
    let (mut a, _sub_a) = instance(&tt, "sd_a", "", 3.0);
    let (mut b, mut sub_b) = instance(&tt, "sd_b", "", 4.0);
    a.shutdown().expect("shutdown a");
    assert!(
        a.tick().is_err(),
        "a tick on a shut-down instance must be refused, not run"
    );
    b.tick().expect("tick b after its twin's shutdown");
    let deadline = Instant::now() + Duration::from_secs(2);
    let mut got = None;
    while got.is_none() && Instant::now() < deadline {
        got = read_x(&mut sub_b);
        std::thread::sleep(Duration::from_millis(5));
    }
    assert_eq!(got, Some(4.0));
    b.shutdown().expect("shutdown b");
}

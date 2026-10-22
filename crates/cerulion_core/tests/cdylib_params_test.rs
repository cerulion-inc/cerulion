// SPDX-License-Identifier: AGPL-3.0-only
//! `#[param]` across the cdylib FFI: the declared specs ride the info JSON,
//! and the graph's values reach the node through the OPTIONAL
//! `cerulion_node_init_with_params` symbol before its first tick. A cdylib
//! that lacks the symbol loads unchanged and refuses a handoff by name; one
//! that lacks it yet declares params is refused when its info is read. A
//! param is not node state: a restore leaves the graph's value in force.
//!
//! Hand oracles from the fixture body (`test_node_macro_param_cdylib`:
//! `cmd.x = gain`, `cmd.y = offset`, `cmd.z = frame_id.len()` or `-1.0` when
//! disabled; defaults `1.0`, `0`, `"base_link"`, `true`).
//!
//! Prerequisite: `cargo build -p test_node_macro_param_cdylib -p test_node_cdylib
//! -p test_node_params_no_symbol_cdylib` (the test panics with that
//! instruction when a fixture is missing).
//!
//! SERIAL: run with `-- --test-threads=1`. A loaded cdylib carries its own
//! statically linked iceoryx2, and two copies minting node ids at once in one
//! process collide. The four transport scenarios share ONE test on purpose:
//! on macOS a process that has run a plugin node cannot create any further
//! iceoryx2 resource (upstream iceoryx2 0.10.0 defect 2034), so every
//! transport is created up front, before the first library is loaded, and
//! the scenarios then run in sequence. The same shape runs on Linux.
//!
//! ```bash
//! cargo test -p cerulion_core --test cdylib_params_test -- --test-threads=1
//! ```

use std::time::{Duration, Instant};

use cerulion_core::graph::node::{
    AnyPublisher, AnySubscriber, DylibNodeEntry, NodeContext, NodeEntry,
};
use cerulion_core::graph::params::{ParamSpec, ParamType, ParamValue, ParamValues};
use cerulion_core::state::VecSink;
use cerulion_core::testing::TestTransport;
use cerulion_core::transport::subscriber::CerulionSubscriber;
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

const PARAM_FIXTURE: &str = "test_node_macro_param_cdylib";
/// A raw-FFI fixture that exports no params symbol and declares no params.
const PLAIN_FIXTURE: &str = "test_node_cdylib";
/// A raw-FFI fixture whose info JSON declares a param (`gain`) but which
/// exports no params symbol: no value could ever reach it.
const DECLARES_WITHOUT_SYMBOL_FIXTURE: &str = "test_node_params_no_symbol_cdylib";

fn load(crate_name: &str) -> DylibNodeEntry {
    let path = cerulion_core::testing::find_fixture_cdylib(crate_name);
    DylibNodeEntry::load(&path).expect("load fixture cdylib")
}

/// One isolated transport with the node's `cmd` publisher wired into a
/// context and a host-side subscriber on the same topic.
fn wire(tag: &str) -> (TestTransport, NodeContext, CerulionSubscriber) {
    let tt = TestTransport::with_buffer_size(4);
    let topic = format!("test/cdylib_params/{tag}/cmd");
    let publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 0);
    let subscriber = tt.subscriber(&topic);
    let mut publishers: IndexMap<String, AnyPublisher> = IndexMap::new();
    publishers.insert("cmd".to_string(), AnyPublisher::Ipc(publisher));
    let subscribers: IndexMap<String, AnySubscriber> = IndexMap::new();
    (
        tt,
        NodeContext::for_tests(publishers, subscribers),
        subscriber,
    )
}

fn read_cmd(sub: &mut CerulionSubscriber) -> (f64, f64, f64) {
    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        if let Some(frame) = sub
            .try_view::<Vector3, _>(|v| (v.x, v.y, v.z))
            .expect("try_view cmd")
        {
            return frame;
        }
        assert!(
            Instant::now() < deadline,
            "no cmd frame delivered within 2s"
        );
        std::thread::sleep(Duration::from_millis(5));
    }
}

fn values(pairs: &[(&str, ParamValue)]) -> ParamValues {
    pairs
        .iter()
        .map(|(k, v)| (k.to_string(), v.clone()))
        .collect()
}

#[test]
fn the_info_json_carries_every_declared_spec_in_order() {
    let node = load(PARAM_FIXTURE);
    let info = node.info().expect("info");
    let expected = vec![
        ParamSpec {
            name: "gain".into(),
            ty: ParamType::F64,
            default: ParamValue::Float(1.0),
            min: Some(ParamValue::Float(0.0)),
            max: Some(ParamValue::Float(10.0)),
            description: Some("multiplier written to cmd.x".into()),
        },
        ParamSpec {
            name: "offset".into(),
            ty: ParamType::I32,
            default: ParamValue::Int(0),
            min: Some(ParamValue::Int(-100)),
            max: Some(ParamValue::Int(100)),
            description: Some("written to cmd.y".into()),
        },
        ParamSpec {
            name: "frame_id".into(),
            ty: ParamType::String,
            default: ParamValue::Str("base_link".into()),
            min: None,
            max: None,
            description: Some("its byte length is written to cmd.z".into()),
        },
        ParamSpec {
            name: "enabled".into(),
            ty: ParamType::Bool,
            default: ParamValue::Bool(true),
            min: None,
            max: None,
            description: Some("false writes -1.0 to cmd.z".into()),
        },
    ];
    assert_eq!(info.params(), expected.as_slice());
}

#[test]
fn a_cdylib_without_the_symbol_loads_and_refuses_a_handoff_by_name() {
    let mut node = load(PLAIN_FIXTURE);
    assert!(
        node.info().expect("info").params().is_empty(),
        "the plain fixture declares no params"
    );
    node.set_params(&ParamValues::new())
        .expect("an empty handoff is the no-params case");
    let err = node
        .set_params(&values(&[("gain", ParamValue::Float(1.0))]))
        .expect_err("a value for a cdylib with no params symbol must be refused");
    let text = err.to_string();
    assert!(
        text.contains("cerulion_node_init_with_params") && text.contains("gain"),
        "{text}"
    );
}

/// The graph build hands every node its FULL resolved map, defaults
/// included, so a library that declares params it cannot receive would be
/// refused on every run, with or without `params:` in the graph. It is
/// refused once, where the library's info is read, so `graph validate`,
/// `graph run` and `node info` all say the same thing: the load succeeds,
/// `info()` names the missing symbol and the declared params.
#[test]
fn a_library_that_declares_params_without_the_symbol_is_refused_when_its_info_is_read() {
    let node = load(DECLARES_WITHOUT_SYMBOL_FIXTURE);
    let err = node
        .info()
        .expect_err("a declared param with no symbol to deliver it must refuse the library");
    let text = err.to_string();
    assert!(
        text.contains("declares params (gain)")
            && text.contains("exports no cerulion_node_init_with_params symbol")
            && text.contains("cerulion node build"),
        "{text}"
    );
    // Not cached: a second read refuses again rather than serving a stale Ok.
    assert!(node.info().is_err());
}

/// The four transport scenarios, in one process, transports first (see the
/// module docs for why):
///
/// 1. no handoff: the first tick publishes the defaults;
/// 2. a full handoff through the optional symbol: the first tick publishes
///    the graph's values;
/// 3. two instances of one library hold different values;
/// 4. a name the library does not declare is refused at `init`.
#[test]
fn values_cross_the_optional_symbol_and_instances_hold_their_own() {
    // Every iceoryx2 resource BEFORE the first library load.
    let (_tt1, ctx_defaults, mut sub_defaults) = wire("defaults");
    let (_tt2, ctx_values, mut sub_values) = wire("values");
    let (_tt3, ctx_twin_a, mut sub_twin_a) = wire("twin_a");
    let (_tt4, ctx_twin_b, mut sub_twin_b) = wire("twin_b");
    let (_tt5, ctx_unknown, _sub_unknown) = wire("unknown");
    let (_tt6, ctx_anchor, mut sub_anchor) = wire("anchor");
    let (_tt7, ctx_resumed, mut sub_resumed) = wire("resumed");

    // 1. Defaults.
    let mut defaults = load(PARAM_FIXTURE);
    defaults.init(ctx_defaults).expect("init defaults");
    defaults.tick().expect("tick defaults");
    assert_eq!(read_cmd(&mut sub_defaults), (1.0, 0.0, 9.0));

    // 2. A full handoff: disabled writes -1.0 to z regardless of the
    //    eight-byte frame id.
    let mut with_values = load(PARAM_FIXTURE);
    with_values
        .set_params(&values(&[
            ("gain", ParamValue::Float(2.5)),
            ("offset", ParamValue::Int(-7)),
            ("frame_id", ParamValue::Str("left_cam".into())),
            ("enabled", ParamValue::Bool(false)),
        ]))
        .expect("set_params");
    with_values.init(ctx_values).expect("init values");
    with_values.tick().expect("tick values");
    assert_eq!(read_cmd(&mut sub_values), (2.5, -7.0, -1.0));

    // 3. Two instances of one library. `b` also overrides the string while
    //    `enabled` keeps its default true, so z is the eight-byte "left_cam"
    //    that crossed the FFI, not the nine-byte default.
    let mut twin_a = load(PARAM_FIXTURE);
    let mut twin_b = load(PARAM_FIXTURE);
    twin_a
        .set_params(&values(&[("gain", ParamValue::Float(2.5))]))
        .expect("set_params a");
    twin_b
        .set_params(&values(&[
            ("gain", ParamValue::Float(0.5)),
            ("frame_id", ParamValue::Str("left_cam".into())),
        ]))
        .expect("set_params b");
    twin_a.init(ctx_twin_a).expect("init a");
    twin_b.init(ctx_twin_b).expect("init b");
    twin_a.tick().expect("tick a");
    twin_b.tick().expect("tick b");
    assert_eq!(read_cmd(&mut sub_twin_a), (2.5, 0.0, 9.0));
    assert_eq!(read_cmd(&mut sub_twin_b), (0.5, 0.0, 8.0));

    // 4. An unknown name is held by `set_params` and refused at `init`.
    let mut unknown = load(PARAM_FIXTURE);
    unknown
        .set_params(&values(&[("bogus", ParamValue::Float(1.0))]))
        .expect("the handoff is held until init");
    let err = unknown
        .init(ctx_unknown)
        .expect_err("init must refuse an unknown name");
    assert!(err.to_string().contains("unknown param 'bogus'"), "{err}");

    // 5. A restore keeps the graph's values. The fixture's only non-port
    //    fields are params, and a param is not node state, so the anchor
    //    captured at gain 2.0 is EMPTY, and a node the graph built with
    //    gain 3.0 still reads 3.0 on its first tick after the restore.
    let mut anchor = load(PARAM_FIXTURE);
    anchor
        .set_params(&values(&[("gain", ParamValue::Float(2.0))]))
        .expect("set_params anchor");
    anchor.init(ctx_anchor).expect("init anchor");
    anchor.tick().expect("tick anchor");
    assert_eq!(read_cmd(&mut sub_anchor), (2.0, 0.0, 9.0));
    let mut payload = VecSink::new();
    anchor.capture_state(&mut payload).expect("capture anchor");
    assert!(
        payload.is_empty(),
        "no param may be captured: {:?}",
        payload.as_slice()
    );
    let mut resumed = load(PARAM_FIXTURE);
    resumed
        .set_params(&values(&[("gain", ParamValue::Float(3.0))]))
        .expect("set_params resumed");
    resumed.init(ctx_resumed).expect("init resumed");
    resumed
        .restore_state(payload.as_slice())
        .expect("restore resumed");
    resumed.tick().expect("tick resumed");
    assert_eq!(read_cmd(&mut sub_resumed), (3.0, 0.0, 9.0));
}

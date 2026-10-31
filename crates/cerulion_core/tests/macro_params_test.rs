// SPDX-License-Identifier: AGPL-3.0-only
//! `#[param]` on an IN-PROCESS macro node: the declared specs reach
//! `info()`, the defaults land at construction, the graph's values handed
//! over through `NodeEntry::set_params` reach the user's `init` and the first
//! tick, and a value for a name the node does not declare is refused rather
//! than dropped.
//!
//! Every expected frame is a HAND ORACLE from the node body below
//! (`cmd.x = gain`, `cmd.y = offset`, `cmd.z = frame_id.len()` or `-1.0` when
//! disabled), never a second run of the same code.
//!
//! Parallel-safe: each test owns a [`TestTransport`] (a per-test SHM root).
//!
//! ```bash
//! cargo test -p cerulion_core --test macro_params_test
//! ```

use std::sync::Mutex;
use std::time::{Duration, Instant};

use cerulion_core::graph::node::{AnyPublisher, AnySubscriber, NodeContext, NodeEntry, NodeInfo};
use cerulion_core::graph::params::{ParamSpec, ParamType, ParamValue, ParamValues};
use cerulion_core::prelude::*;
use cerulion_core::testing::TestTransport;
use cerulion_core::transport::subscriber::CerulionSubscriber;
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

/// What the user's `init` saw in `self.gain`, per test (keyed by the topic
/// the test minted, so parallel tests cannot read each other's value).
static INIT_SAW_GAIN: Mutex<Vec<(String, f64)>> = Mutex::new(Vec::new());

#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct ParamNode {
    #[output]
    cmd: Vector3,
    #[param(
        default = 1.0,
        min = 0.0,
        max = 10.0,
        description = "multiplier written to cmd.x"
    )]
    gain: f64,
    #[param(default = 0, min = -100, max = 100)]
    offset: i32,
    #[param(default = "base_link")]
    frame_id: String,
    #[param(default = true)]
    enabled: bool,
    /// Set by the test before `init` so `init` can file what it saw.
    probe_tag: String,
}

#[cerulion_node_impl]
impl ParamNode {
    fn init(&mut self, _ctx: &mut NodeContext) -> Result<(), NodeError> {
        INIT_SAW_GAIN
            .lock()
            .expect("probe lock")
            .push((self.probe_tag.clone(), self.gain));
        Ok(())
    }

    fn tick(&mut self) -> Result<(), NodeError> {
        self.cmd.x = self.gain;
        self.cmd.y = f64::from(self.offset);
        self.cmd.z = if self.enabled {
            self.frame_id.len() as f64
        } else {
            -1.0
        };
        Ok(())
    }
}

/// A node that declares NO params: the macro still accepts the handoff and
/// refuses any value by name at `init`.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct PlainNode {
    #[output]
    cmd: Vector3,
}

#[cerulion_node_impl]
impl PlainNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.cmd.x = 0.0;
        Ok(())
    }
}

/// A param on a RAW-IDENTIFIER field: `r#type` is the param `type` in the
/// spec and in the graph's `params:` keys, never `r#type`.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct RawNameNode {
    #[output]
    cmd: Vector3,
    #[param(default = "camera")]
    r#type: String,
}

#[cerulion_node_impl]
impl RawNameNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.cmd.x = self.r#type.len() as f64;
        Ok(())
    }
}

/// One isolated transport with the node's `cmd` publisher wired into a
/// context and a host-side subscriber on the same topic.
fn wire(tag: &str) -> (TestTransport, NodeContext, CerulionSubscriber) {
    let tt = TestTransport::with_buffer_size(4);
    let topic = format!("test/macro_params/{tag}/cmd");
    let publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 0);
    let subscriber = tt.subscriber(&topic);
    let mut publishers: IndexMap<String, AnyPublisher> = IndexMap::new();
    publishers.insert("cmd".to_string(), AnyPublisher::Ipc(publisher));
    let subscribers: IndexMap<String, AnySubscriber> = IndexMap::new();
    let ctx = NodeContext::for_tests(publishers, subscribers);
    (tt, ctx, subscriber)
}

/// The next `cmd` frame as `(x, y, z)`, waiting briefly for delivery.
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
fn info_publishes_the_declared_specs_in_declaration_order() {
    let entry = ParamNodeEntry::new();
    let info: NodeInfo = entry.info().expect("info");
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
            description: None,
        },
        ParamSpec {
            name: "frame_id".into(),
            ty: ParamType::String,
            default: ParamValue::Str("base_link".into()),
            min: None,
            max: None,
            description: None,
        },
        ParamSpec {
            name: "enabled".into(),
            ty: ParamType::Bool,
            default: ParamValue::Bool(true),
            min: None,
            max: None,
            description: None,
        },
    ];
    assert_eq!(info.params(), expected.as_slice());
    assert!(
        PlainNodeEntry::new()
            .info()
            .expect("info")
            .params()
            .is_empty(),
        "a node without #[param] fields declares no params"
    );
}

#[test]
fn a_node_run_with_no_params_publishes_its_defaults() {
    let (_tt, ctx, mut sub) = wire("defaults");
    let mut entry = ParamNodeEntry::new();
    entry.init(ctx).expect("init");
    entry.tick().expect("tick");
    // gain 1.0, offset 0, "base_link" is nine bytes.
    assert_eq!(read_cmd(&mut sub), (1.0, 0.0, 9.0));
}

#[test]
fn graph_values_handed_over_before_init_reach_init_and_the_first_tick() {
    let (_tt, ctx, mut sub) = wire("values");
    let mut entry = ParamNodeEntry::with_state(ParamNode {
        probe_tag: "values".into(),
        ..Default::default()
    });
    entry
        .set_params(&values(&[
            ("gain", ParamValue::Float(2.5)),
            ("offset", ParamValue::Int(-7)),
            ("frame_id", ParamValue::Str("left_cam".into())),
            ("enabled", ParamValue::Bool(false)),
        ]))
        .expect("set_params");
    entry.init(ctx).expect("init");
    entry.tick().expect("tick");
    // Disabled writes -1.0 to z regardless of the eight-byte frame id.
    assert_eq!(read_cmd(&mut sub), (2.5, -7.0, -1.0));
    let seen = INIT_SAW_GAIN
        .lock()
        .expect("probe lock")
        .iter()
        .find(|(tag, _)| tag == "values")
        .map(|(_, gain)| *gain);
    assert_eq!(
        seen,
        Some(2.5),
        "the user's init must already see the graph's value, not the default"
    );
}

#[test]
fn a_partial_handoff_keeps_the_defaults_for_the_names_it_omits() {
    let (_tt, ctx, mut sub) = wire("partial");
    let mut entry = ParamNodeEntry::new();
    entry
        .set_params(&values(&[("offset", ParamValue::Int(42))]))
        .expect("set_params");
    entry.init(ctx).expect("init");
    entry.tick().expect("tick");
    assert_eq!(read_cmd(&mut sub), (1.0, 42.0, 9.0));
}

#[test]
fn with_state_keeps_the_given_values_while_new_writes_the_defaults() {
    let kept = ParamNodeEntry::with_state(ParamNode {
        gain: 7.0,
        ..Default::default()
    });
    assert_eq!(kept.inner.gain, 7.0, "with_state takes the state as given");
    assert_eq!(
        kept.inner.frame_id, "",
        "no default is written over given state"
    );
    let fresh = ParamNodeEntry::new();
    assert_eq!(fresh.inner.gain, 1.0);
    assert_eq!(fresh.inner.offset, 0);
    assert_eq!(fresh.inner.frame_id, "base_link");
    assert!(fresh.inner.enabled);
}

#[test]
fn a_name_the_node_does_not_declare_is_refused_at_init() {
    let (_tt, ctx, _sub) = wire("unknown");
    let mut entry = ParamNodeEntry::new();
    entry
        .set_params(&values(&[("bogus", ParamValue::Float(1.0))]))
        .expect("the handoff itself is accepted; the check runs at init");
    let err = entry
        .init(ctx)
        .expect_err("init must refuse an unknown name");
    let text = err.to_string();
    assert!(text.contains("unknown param 'bogus'"), "{text}");
}

#[test]
fn a_value_of_the_wrong_type_is_refused_at_init_naming_the_param() {
    let (_tt, ctx, _sub) = wire("wrong_type");
    let mut entry = ParamNodeEntry::new();
    entry
        .set_params(&values(&[("gain", ParamValue::Str("fast".into()))]))
        .expect("set_params");
    let err = entry
        .init(ctx)
        .expect_err("init must refuse a string for an f64");
    let text = err.to_string();
    assert!(
        text.contains("param 'gain' expects f64, got \"fast\""),
        "{text}"
    );
}

#[test]
fn a_node_without_params_refuses_any_value_by_name() {
    let (_tt, ctx, _sub) = wire("plain");
    let mut entry = PlainNodeEntry::new();
    entry
        .set_params(&values(&[("gain", ParamValue::Float(1.0))]))
        .expect("set_params");
    let err = entry
        .init(ctx)
        .expect_err("a node with no params has nowhere to put a value");
    assert!(err.to_string().contains("unknown param 'gain'"), "{err}");
    // An empty handoff is the no-params case and is fine.
    let (_tt2, ctx2, _sub2) = wire("plain_empty");
    let mut entry = PlainNodeEntry::new();
    entry.set_params(&ParamValues::new()).expect("set_params");
    entry.init(ctx2).expect("init");
}

#[test]
fn a_raw_identifier_field_is_the_param_named_without_its_prefix() {
    let info = RawNameNodeEntry::new().info().expect("info");
    let names: Vec<&str> = info.params().iter().map(|p| p.name.as_str()).collect();
    assert_eq!(names, ["type"]);

    let (_tt, ctx, mut sub) = wire("raw_name");
    let mut entry = RawNameNodeEntry::new();
    entry
        .set_params(&values(&[("type", ParamValue::Str("lidar_front".into()))]))
        .expect("set_params");
    entry.init(ctx).expect("init");
    entry.tick().expect("tick");
    // `"lidar_front"` is eleven bytes; the default `"camera"` would be six.
    assert_eq!(read_cmd(&mut sub), (11.0, 0.0, 0.0));
}

/// The trait DEFAULT (a hand-written entry that never overrides
/// `set_params`) refuses a non-empty map and accepts an empty one.
#[test]
fn the_trait_default_refuses_a_non_empty_handoff() {
    struct Bare;
    impl NodeEntry for Bare {
        fn info(&self) -> cerulion_core::error::TransportResult<NodeInfo> {
            Ok(NodeInfo::from_names(vec![], vec![]))
        }
        fn init(&mut self, _context: NodeContext) -> cerulion_core::error::TransportResult<()> {
            Ok(())
        }
        fn tick(&mut self) -> cerulion_core::error::TransportResult<()> {
            Ok(())
        }
    }
    let mut bare = Bare;
    bare.set_params(&ParamValues::new()).expect("empty is fine");
    let err = bare
        .set_params(&values(&[
            ("gain", ParamValue::Float(1.0)),
            ("offset", ParamValue::Int(2)),
        ]))
        .expect_err("non-empty must be refused");
    assert_eq!(
        err.to_string(),
        "Graph validation error: node accepts no params, but 2 were supplied (gain, offset)"
    );
}

#[test]
fn two_nodes_given_the_same_params_publish_identical_frames() {
    let handoff = values(&[
        ("gain", ParamValue::Float(3.25)),
        ("frame_id", ParamValue::Str("odom".into())),
    ]);
    let mut frames = Vec::new();
    for tag in ["det_a", "det_b"] {
        let (_tt, ctx, mut sub) = wire(tag);
        let mut entry = ParamNodeEntry::new();
        entry.set_params(&handoff).expect("set_params");
        entry.init(ctx).expect("init");
        entry.tick().expect("tick");
        frames.push(read_cmd(&mut sub));
    }
    // "odom" is four bytes.
    assert_eq!(frames, vec![(3.25, 0.0, 4.0), (3.25, 0.0, 4.0)]);
}

/// A param is not node state: the fold-in leaves every `#[param]` field out
/// of the shape, so the shape holds the struct name and `probe_tag` alone.
/// Changing a param's type or name therefore never refuses an anchor.
#[test]
fn a_nodes_shape_leaves_every_param_out() {
    use cerulion_core::state::{CerulionState, StateShape};
    // HAND ORACLE: the struct name, then ONLY the one non-param, non-port
    // field. A shape that folded `gain`, `offset`, `frame_id` or `enabled`
    // could not match this.
    let expected = StateShape::of("ParamNode")
        .field("probe_tag", <String as CerulionState>::STATE_SHAPE)
        .finish();
    assert_eq!(<ParamNode as CerulionState>::STATE_SHAPE, expected);
    // A node whose only non-port fields are params has an empty shape.
    assert_eq!(
        <RawNameNode as CerulionState>::STATE_SHAPE,
        StateShape::of("RawNameNode").finish()
    );
}

/// The resume rule at the node seam: an anchor captured while `gain` was 2.0,
/// restored into a node the graph built with `gain` 3.0, keeps 3.0 for
/// `init` and the first tick, while the node's own state (`probe_tag`) is
/// restored from the anchor.
#[test]
fn a_restore_keeps_the_graphs_param_values_and_restores_the_rest() {
    use cerulion_core::state::VecSink;
    let anchored = ParamNodeEntry::with_state(ParamNode {
        gain: 2.0,
        offset: 5,
        frame_id: "odom".into(),
        enabled: true,
        probe_tag: "anchored".into(),
        ..Default::default()
    });
    let mut payload = VecSink::new();
    anchored.capture_state(&mut payload).expect("capture");

    let (_tt, ctx, mut sub) = wire("restore");
    let mut resumed = ParamNodeEntry::new();
    resumed
        .set_params(&values(&[("gain", ParamValue::Float(3.0))]))
        .expect("set_params");
    resumed.init(ctx).expect("init");
    resumed.restore_state(payload.as_slice()).expect("restore");
    assert_eq!(
        resumed.inner.probe_tag, "anchored",
        "the node's own state comes from the anchor"
    );
    resumed.tick().expect("tick");
    // gain 3.0 from the graph, offset 0 and "base_link" (nine bytes) from the
    // defaults: no param value comes from the anchor.
    assert_eq!(read_cmd(&mut sub), (3.0, 0.0, 9.0));
}

/// A node that WRITES its params: `tick` flips the sign of `gain` (so `0.0`
/// becomes `-0.0`, equal under `==` but not the same value) and `init`
/// renames `frame_id` when asked.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct WritesParamsNode {
    #[output]
    cmd: Vector3,
    #[param(default = 0.0)]
    gain: f64,
    #[param(default = "base_link")]
    frame_id: String,
    rename_in_init: bool,
}

#[cerulion_node_impl]
impl WritesParamsNode {
    fn init(&mut self, _ctx: &mut NodeContext) -> Result<(), NodeError> {
        if self.rename_in_init {
            self.frame_id.push_str("_moved");
        }
        Ok(())
    }

    fn tick(&mut self) -> Result<(), NodeError> {
        self.gain = -self.gain;
        self.cmd.x = self.gain;
        Ok(())
    }
}

/// A param is fixed for the run and an anchor does not carry it, so a node
/// that writes one after the graph set it could not resume as it ran. The
/// capture refuses, naming the param, instead of recording that anchor.
#[test]
fn a_capture_refuses_once_the_node_wrote_a_param_in_tick() {
    use cerulion_core::state::VecSink;
    let (_tt, ctx, _sub) = wire("writes_tick");
    let mut entry = WritesParamsNodeEntry::new();
    entry
        .set_params(&values(&[("gain", ParamValue::Float(0.0))]))
        .expect("set_params");
    entry.init(ctx).expect("init");
    // ANTI-TAUTOLOGY: before the write the same node captures.
    entry
        .capture_state(&mut VecSink::new())
        .expect("an unwritten param captures");
    entry.tick().expect("tick");
    assert_eq!(entry.inner.gain.to_bits(), (-0.0_f64).to_bits());
    let err = entry
        .capture_state(&mut VecSink::new())
        .expect_err("a written param refuses the capture")
        .to_string();
    assert!(
        err.contains("param 'gain' changed after the graph set it"),
        "{err}"
    );
}

/// A write in the user's `init` is a change too: the values a param holds
/// for the run are the ones the graph handed over, taken before `init`.
#[test]
fn a_capture_refuses_a_param_the_users_init_wrote() {
    use cerulion_core::state::VecSink;
    let (_tt, ctx, _sub) = wire("writes_init");
    let mut entry = WritesParamsNodeEntry::new();
    entry.inner.rename_in_init = true;
    entry.init(ctx).expect("init");
    assert_eq!(entry.inner.frame_id, "base_link_moved");
    let err = entry
        .capture_state(&mut VecSink::new())
        .expect_err("a param written in init refuses the capture")
        .to_string();
    assert!(
        err.contains("param 'frame_id' changed after the graph set it"),
        "{err}"
    );
}

// SPDX-License-Identifier: AGPL-3.0-only
//! /tf + /tf_static ride the zenoh network link.
//!
//! Transforms are the payload that makes rviz-style visualization work, and
//! `tf2_msgs/TFMessage` is the WORST-CASE wire shape in the vendored set — a
//! variable array-of-nested-messages-with-strings
//! (`geometry_msgs/TransformStamped[] transforms`, each carrying a
//! `std_msgs/Header` + two strings + a `Transform`). This file pins that a
//! TFMessage frame crosses the robot-computer-egress → Mac-ingress hop (pure shipped
//! config) BYTE-IDENTICAL, and that the pipe is enabled
//! by `network:` blocks alone (no code).
//!
//! Two arms, both HAND oracles (never a two-run self-compare — Principle #7).
//! Coverage ownership:
//!
//! 1. **Wire-format arm — owns the /tf WIRE pin**
//!    (`tf_frames_cross_hop_byte_identical`, `tf_pipe_is_deterministic`): the
//!    CROSS-SESSION two-manager path (cribbed from
//!    `network_ingress_e2e_test.rs`) — machine A (egress) and machine B
//!    (ingress) as two `TransportManager::init_for_test` managers on DISTINCT
//!    per-test SHM roots, zenoh sessions linked over a REAL 127.0.0.1 TCP hop
//!    (bind-probed ephemeral port, scouting off). A publishes REAL
//!    `TFMessage` frames via `loan_proxy` + `set_transforms_bytes` in
//!    PER-FRAME LOCKSTEP (publish one → bounded-wait until B's local
//!    iceoryx2 subscriber yields THAT frame, via the one-sample-per-call
//!    `try_receive_one` — never the drain-all `try_receive` → publish the
//!    next), and each FULL received frame (32-byte header rebuilt via
//!    `write_to_buf` + the payload slice, offset-table bytes included) is
//!    compared against a HAND-BUILT oracle frame. Covers the multi-transform broadcast, a single-transform frame,
//!    AND the empty-array edge (`set_transforms_bytes(&[])` → an offset-table
//!    entry with length 0), plus determinism (two runs == each other == the
//!    oracle).
//!
//! 2. **Config-only arm — owns the /tf_static coverage AND the two-topic docs
//!    shape** (`tf_and_tf_static_config_only_yaml_byte_identical`, cribbed
//!    from `network_yaml_e2e_test.rs`): two graphs built PURELY from YAML
//!    strings — the robot-computer side OWNS both `/tf` and `/tf_static` (ONE
//!    broadcaster node with two egress-listed `TFMessage` outputs, the
//!    `docs/networking.md` transforms shape exercised as a unit), the Mac
//!    side INGRESSes BOTH with in-graph consumers whose `TFMessage`
//!    `#[input(trigger)]`s resolve the ingress schema hashes. Asserts
//!    FULL-frame byte identity for BOTH topics (the same oracle discipline as
//!    arm 1 — header via `write_to_buf` + payload + offset-table bytes, with
//!    hand-predicted sequence 0 and loan timestamp from a one-shot fire —
//!    read via raw harness taps on B's re-injected topics) PLUS the typed
//!    `transforms_bytes()` read through each graph consumer. /tf_static has
//!    NO separate wire-arm test: its publisher code path and wire shape are
//!    IDENTICAL to /tf (same schema, same `set_transforms_bytes` path), so
//!    its byte-identity coverage lives HERE, where the two-topic config shape
//!    is also pinned.
//!
//! ## The TFMessage wire layout the oracle rebuilds
//!
//! TFMessage is a 0-fixed / 1-variable schema (drift-guarded below), so the
//! publisher lays the post-header payload out as
//! `[OffsetEntry { offset, length }][transforms bytes]`: the writer's cursor
//! starts at `WIRE_FIXED_SIZE + 8 * VARIABLE_FIELD_COUNT` (= 8, just past the
//! single 8-byte offset-table slot), so the stored `offset` is 8 and the
//! `length` is `transforms.len()`. The `transforms` bytes themselves are
//! opaque to the transport (raw-bytes accessor family), so the pipe carries
//! them VERBATIM — exactly the property this file pins.
//!
//! Port-reuse race + bounded-retry conventions cribbed verbatim from
//! `network_ingress_e2e_test.rs`. NOT `#[serial]`: distinct per-test SHM
//! roots, per-run freshly probed ports, unique topics.

use std::net::TcpListener;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime};

use cerulion_core::clock::{Clock, VirtualClock};
use cerulion_core::graph::node::{NodeEntry, NodeInfo};
use cerulion_core::graph::{compute_gateway_plan, parse_graph_raw, GraphRuntime, GraphTopology};
use cerulion_core::message::ShmMessage;
use cerulion_core::prelude::*;
use cerulion_core::transport::gateway::{GatewayEgressPolicy, GatewayPlan, GatewayRuntime};
use cerulion_core::transport::network::NetworkConfig;
use cerulion_core::transport::{NetworkPosture, TransportConfig, TransportManager};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use indexmap::IndexMap;
use native_ros2_messages::tf2_msgs::TFMessage;

static COUNTER: AtomicU64 = AtomicU64::new(0);

/// SHM slot capacity for the TF topics — comfortably above the largest
/// hand-built TFMessage frame in this file (~214 bytes) with room to spare.
const CAP: u32 = 4096;

/// Arm-1 (wire-format) machine-A setup bundle: producer manager, its virtual
/// clock, the producer publisher, A's egress gateway, and the probed port.
type WireArmASetup = (
    Arc<TransportManager>,
    Arc<VirtualClock>,
    cerulion_core::CerulionPublisher,
    GatewayRuntime,
    u16,
);

/// Arm-2 (config-only) graph-A setup bundle: the graph runtime, A's egress
/// gateway, the manager, its virtual clock, and the probed port.
type YamlArmASetup = (
    GraphRuntime,
    GatewayRuntime,
    Arc<TransportManager>,
    Arc<VirtualClock>,
    u16,
);

fn unique_id() -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("{nanos}_{id}")
}

/// Bind-probe an ephemeral TCP port (crib: `network_ingress_e2e_test.rs` —
/// the probe→rebind window is absorbed by the caller's bounded retry).
fn probe_ephemeral_port() -> u16 {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind ephemeral probe");
    let port = listener.local_addr().expect("probe local_addr").port();
    drop(listener);
    port
}

// ===========================================================================
// Hand-built TFMessage `transforms` payloads
//
// A Cerulion TEST encoding of a transform (NOT ROS2 CDR): this file pins only
// that the pipe carries the `transforms` bytes VERBATIM, so the exact element
// encoding is irrelevant as long as it is DISTINCTIVE and NON-SYMMETRIC
// enough that a byte-swap / truncation regression is caught. Every field is
// little-endian.
// ===========================================================================

/// One transform record:
/// `sec:i32 | nanosec:u32 | frame_id_len:u32 | frame_id | child_len:u32 |
///  child | translation x,y,z:f64 | rotation x,y,z,w:f64`.
fn transform_record(
    sec: i32,
    nanosec: u32,
    frame_id: &str,
    child_frame_id: &str,
    translation: [f64; 3],
    rotation: [f64; 4],
) -> Vec<u8> {
    let mut v = Vec::new();
    v.extend_from_slice(&sec.to_le_bytes());
    v.extend_from_slice(&nanosec.to_le_bytes());
    v.extend_from_slice(&(frame_id.len() as u32).to_le_bytes());
    v.extend_from_slice(frame_id.as_bytes());
    v.extend_from_slice(&(child_frame_id.len() as u32).to_le_bytes());
    v.extend_from_slice(child_frame_id.as_bytes());
    for c in translation {
        v.extend_from_slice(&c.to_le_bytes());
    }
    for c in rotation {
        v.extend_from_slice(&c.to_le_bytes());
    }
    v
}

/// A TFMessage `transforms` array: a `u32` count prefix then that many
/// [`transform_record`]s concatenated.
fn tf_transforms(records: &[Vec<u8>]) -> Vec<u8> {
    let mut v = Vec::new();
    v.extend_from_slice(&(records.len() as u32).to_le_bytes());
    for r in records {
        v.extend_from_slice(r);
    }
    v
}

/// A 2-transform broadcast (`odom→base_link`, `base_link→camera_link`) — the
/// dynamic-TF-broadcaster shape. Distinctive per-field values so a swapped or
/// truncated leg is caught.
fn two_transform_payload() -> Vec<u8> {
    tf_transforms(&[
        transform_record(
            100,
            250_000_000,
            "odom",
            "base_link",
            [1.5, -2.5, 3.25],
            [0.0, 0.0, 0.25, 0.96875],
        ),
        transform_record(
            100,
            250_000_001,
            "base_link",
            "camera_link",
            [-0.125, 0.0625, -4.75],
            [0.5, -0.5, 0.5, 0.5],
        ),
    ])
}

/// A single dynamic transform (`map→odom`).
fn one_transform_payload() -> Vec<u8> {
    tf_transforms(&[transform_record(
        200,
        500_000_000,
        "map",
        "odom",
        [10.0, -20.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    )])
}

/// A single LATCHED static transform (`base_link→lidar`) — the /tf_static
/// shape (sensor mounting, published once).
fn static_transform_payload() -> Vec<u8> {
    tf_transforms(&[transform_record(
        0,
        0,
        "base_link",
        "lidar",
        [0.2, 0.0, 0.35],
        [0.0, 0.0, 0.0, 1.0],
    )])
}

// ===========================================================================
// The full-frame hand oracle
// ===========================================================================

/// Hand-build the FULL wire frame a `loan_proxy::<TFMessage>` publish commits
/// for `transforms_bytes`: 32-byte header + `[OffsetEntry][transforms bytes]`.
/// See the module doc for the layout derivation. Asserts the 0-fixed /
/// 1-variable shape it depends on, so a codegen change to TFMessage's layout
/// fails HERE (loudly) rather than silently skewing the oracle.
fn tfmessage_oracle_frame(seq: u32, ts: u64, transforms_bytes: &[u8]) -> Vec<u8> {
    assert_eq!(
        TFMessage::WIRE_FIXED_SIZE,
        0,
        "oracle assumes TFMessage has an empty fixed section"
    );
    assert_eq!(
        TFMessage::VARIABLE_FIELD_COUNT,
        1,
        "oracle assumes TFMessage has exactly one (variable) field"
    );

    // Cursor starts past the single 8-byte offset-table slot → offset == 8.
    let offset = (TFMessage::WIRE_FIXED_SIZE + 8 * TFMessage::VARIABLE_FIELD_COUNT) as u32;
    let length = transforms_bytes.len() as u32;

    // Payload: [OffsetEntry { offset, length }][transforms bytes]. The fixed
    // section is empty (WIRE_FIXED_SIZE == 0), so the offset table leads.
    let mut payload = Vec::with_capacity(offset as usize + transforms_bytes.len());
    payload.extend_from_slice(&offset.to_le_bytes());
    payload.extend_from_slice(&length.to_le_bytes());
    payload.extend_from_slice(transforms_bytes);

    let header = WireHeader {
        schema_hash: TFMessage::SCHEMA_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + TFMessage::WIRE_FIXED_SIZE) as u32,
        offset_table_count: TFMessage::VARIABLE_FIELD_COUNT as u32,
        sequence: seq,
        timestamp_ns: ts,
    };
    let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(&payload);
    frame
}

/// One received frame: (schema_hash, sequence, timestamp_ns, total_size,
/// payload, FULL frame bytes) — the same tuple shape as
/// `network_ingress_e2e_test.rs`, so the FULL-slice pin covers the header AND
/// the offset-table bytes.
type Received = (u64, u32, u64, u32, Vec<u8>, Vec<u8>);

/// The hand oracle B's received frames must equal: sequence `i` (commit
/// consumed from 0), the plan's timestamp, and the full hand-built frame.
fn tf_oracle(plan: &[(u64, Vec<u8>)]) -> Vec<Received> {
    plan.iter()
        .enumerate()
        .map(|(i, (ts, tb))| {
            let full = tfmessage_oracle_frame(i as u32, *ts, tb);
            let payload = full[WireHeader::SIZE..].to_vec();
            (
                TFMessage::SCHEMA_HASH,
                i as u32,
                *ts,
                full.len() as u32,
                payload,
                full,
            )
        })
        .collect()
}

/// Non-blocking: take at most ONE frame off `sub`, rebuilt as a [`Received`]
/// — the FULL frame slice reconstructed from `header()` (via `write_to_buf`,
/// `read_from_buf`'s total inverse — the six fields cover all 32 header
/// bytes) + `payload()` (everything after the header: offset table +
/// transforms bytes).
///
/// MUST route through `try_receive_one` (the one-sample-per-call receive),
/// NEVER the drain-everything `try_receive`: that one invokes the callback
/// for EVERY queued sample in a single call, so a `got = Some(..)` capture
/// keeps only the NEWEST frame and silently discards the rest of the queue
/// (drain-to-latest: a
/// 3-frame plan collapses to one received tuple, the last frame).
fn take_one(sub: &CerulionSubscriber) -> Option<Received> {
    let mut got: Option<Received> = None;
    sub.try_receive_one(|msg| {
        let h = msg.header();
        let payload = msg.payload().to_vec();
        let mut full = vec![0u8; WireHeader::SIZE + payload.len()];
        h.write_to_buf(&mut full[..WireHeader::SIZE]);
        full[WireHeader::SIZE..].copy_from_slice(&payload);
        got = Some((
            h.schema_hash,
            h.sequence,
            h.timestamp_ns,
            h.total_size,
            payload,
            full,
        ));
    })
    .expect("try_receive_one");
    got
}

/// Run the full cross-manager TF flow once, in PER-FRAME LOCKSTEP: A
/// publishes ONE `(ts, transforms_bytes)` plan entry via
/// `loan_proxy::<TFMessage>`, then bounded-waits until B's LOCAL subscriber
/// yields THAT frame before publishing the next — so at most one frame is
/// ever in flight and the collection never depends on B's local queue depth.
/// Panics (bounded) on any wedge — no hangs; a missing frame panics naming
/// its plan index.
fn run_tf_cross_manager_flow(tag: &str, plan: &[(u64, Vec<u8>)]) -> Vec<Received> {
    let id = unique_id();
    let topic = format!("/tf/{tag}/{id}");

    // ---- Machine A (egress): a producer publisher + a GATEWAY on ONE
    // network manager, listen on a probed ephemeral port. Bounded retry absorbs
    // the probe→rebind port-steal race (crib: network_ingress_e2e_test.rs).
    let mut a_setup: Option<WireArmASetup> = None;
    // Every attempt's error is kept and surfaced in the final panic.
    let mut attempt_errors: Vec<String> = Vec::new();
    for attempt in 0..3 {
        let port = probe_ephemeral_port();
        let clock = Arc::new(VirtualClock::new());
        let clock_dyn: Arc<dyn Clock> = clock.clone();
        let a = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("tf_a_{tag}_{id}_{attempt}"),
                clock: clock_dyn,
                network: Some(NetworkConfig {
                    listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                    // Announce keys carry the robot chunk.
                    robot_identity: Some("tf-a".to_string()),
                    ..Default::default()
                }),
                ..Default::default()
            },
            cerulion_core::testing::iceoryx_test_config(),
        )
        .expect("init manager A");
        // The producer publisher creates the topic's SHM service (the gateway
        // taps it). Created BEFORE the gateway boots.
        let a_pub = a
            .create_publisher_simple(&topic, MaxSliceLen::const_new(CAP))
            .expect("A publisher");
        // The gateway announces + taps `topic`. Its watch start binds A's listen
        // endpoint — a port steal fails HERE.
        let plan = GatewayPlan {
            egress_policy: GatewayEgressPolicy::AllowAll,
            announce: vec![topic.clone()],
            ingress: vec![],
        };
        match GatewayRuntime::new(Arc::clone(&a), plan) {
            Ok(gateway) => {
                a_setup = Some((a, clock, a_pub, gateway, port));
                break;
            }
            Err(e) => {
                let msg = format!("attempt {attempt} (probed port {port}): {e}");
                eprintln!("A's gateway boot failed — {msg}");
                attempt_errors.push(msg);
            }
        }
    }
    let Some((a_transport, a_clock, mut a_pub, mut gateway_a, port)) = a_setup else {
        panic!(
            "could not boot A's gateway listener in 3 attempts. Distinct ports \
             failing at the zenoh bind = the probe->rebind port-steal race; 3 \
             similar non-port errors = a deterministic failure. All attempts:\n{}",
            attempt_errors.join("\n")
        );
    };

    // ---- Machine B (ingress): distinct SHM root, connects to A's port.
    let b_transport = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("tf_b_{tag}_{id}"),
            network: Some(NetworkConfig {
                connect_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                ..Default::default()
            }),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init manager B");

    // B's local subscriber first (connected before the ingress publisher's
    // first re-injected send), then the one-call ingress wiring keyed on the
    // TFMessage schema hash.
    let b_sub = b_transport.create_subscriber(&topic).expect("B subscriber");
    b_transport
        .register_ingress_topic(&topic, TFMessage::SCHEMA_HASH, MaxSliceLen::const_new(CAP))
        .expect("B register_ingress_topic");

    // ---- Liveliness handshake: B's TopicToken must flip A's gateway egress
    // flag. `register_topic` is idempotent and returns the SAME Arc<AtomicBool>
    // A's gateway reads. Bounded: a silent port steal or liveliness loss panics
    // here instead of hanging.
    let flag = a_transport
        .bridge_manager()
        .register_topic(&topic)
        .expect("A bridge flag handle");
    let deadline = Instant::now() + Duration::from_secs(10);
    while !flag.load(Ordering::Relaxed) {
        assert!(
            Instant::now() < deadline,
            "liveliness handshake did not flip A's bridge flag within 10s — \
             B's TopicToken never reached A's gateway watcher (TCP link down, \
             port stolen silently, or liveliness propagation failure)"
        );
        std::thread::sleep(Duration::from_millis(20));
    }
    // Attach A's egress tap BEFORE the first publish (the tap has no history).
    let attach_deadline = Instant::now() + Duration::from_secs(5);
    while gateway_a.active_tap_topics().is_empty() {
        assert!(
            Instant::now() < attach_deadline,
            "A's gateway tap did not attach"
        );
        gateway_a.drive_once().expect("drive A's gateway attach");
        std::thread::sleep(Duration::from_millis(10));
    }

    // ---- Publish the plan on A in PER-FRAME LOCKSTEP: hand-set the virtual
    // clock, publish ONE entry (explicit drop commits: sequence consumed,
    // iceoryx2 send), drive A's gateway (tap → zenoh forward), then bounded-wait
    // until B's local subscriber yields THAT frame before publishing the next.
    // With `take_one` capped at one sample per call and the lockstep keeping at
    // most one frame in flight, no frame can ever be displaced by a drain.
    let mut got: Vec<Received> = Vec::new();
    for (i, (ts, transforms_bytes)) in plan.iter().enumerate() {
        a_clock.set(*ts);
        let mut proxy = a_pub.loan_proxy::<TFMessage>().expect("A loan_proxy");
        proxy
            .set_transforms_bytes(transforms_bytes)
            .expect("A set_transforms_bytes");
        drop(proxy);
        gateway_a.drive_once().expect("drive A's gateway forward");

        // Bounded wait for exactly this frame.
        let deadline = Instant::now() + Duration::from_secs(10);
        loop {
            if let Some(r) = take_one(&b_sub) {
                got.push(r);
                break;
            }
            // Keep forwarding in case the frame is still in A's SHM tap queue.
            gateway_a.drive_once().expect("drive A's gateway forward");
            assert!(
                Instant::now() < deadline,
                "plan frame {i} (of {}) never reached B's local subscriber \
                 within 10s — {} frame(s) delivered before it",
                plan.len(),
                got.len()
            );
            std::thread::sleep(Duration::from_millis(20));
        }
    }
    got
}

// ===========================================================================
// Arm 1: wire-format byte identity
// ===========================================================================

/// The headline /tf pin: a multi-transform broadcast, a single-transform
/// frame, AND the empty-array edge (`set_transforms_bytes(&[])`) all cross
/// the TCP hop BYTE-IDENTICAL — schema hash, commit sequence 0..3,
/// hand-stamped timestamps, total_size, and the FULL frame slice (offset-table
/// bytes included), against a hand oracle (Principle #7).
#[test]
fn tf_frames_cross_hop_byte_identical() {
    // Drift-guard the oracle's layout assumptions loudly (also asserted
    // inside tfmessage_oracle_frame, surfaced here for readability).
    assert_eq!(TFMessage::WIRE_FIXED_SIZE, 0);
    assert_eq!(TFMessage::VARIABLE_FIELD_COUNT, 1);

    let plan: Vec<(u64, Vec<u8>)> = vec![
        (1_000_000, two_transform_payload()),
        (2_000_000, one_transform_payload()),
        // The empty-array edge: zero transforms → an offset-table entry with
        // length 0 and NO variable data (payload is exactly the 8-byte table).
        (3_000_000, Vec::new()),
    ];
    let got = run_tf_cross_manager_flow("headline", &plan);
    assert_eq!(
        got,
        tf_oracle(&plan),
        "B must receive A's TFMessage frames verbatim: schema hash, commit \
         sequence 0..3, hand-stamped timestamps, total_size, payload bytes, \
         and the FULL frame slice (offset-table + transforms bytes included)"
    );
}

/// Determinism (Principle #7): two full cross-manager runs (fresh ports,
/// fresh SHM roots, same plan) deliver IDENTICAL frame vectors, and both
/// equal the hand oracle (not merely each other).
#[test]
fn tf_pipe_is_deterministic() {
    let plan: Vec<(u64, Vec<u8>)> = vec![
        (1_000_000, two_transform_payload()),
        (2_000_000, Vec::new()),
    ];
    let a = run_tf_cross_manager_flow("det_a", &plan);
    let b = run_tf_cross_manager_flow("det_b", &plan);
    let oracle = tf_oracle(&plan);
    assert_eq!(a, oracle, "run A must equal the hand oracle");
    assert_eq!(b, oracle, "run B must equal the hand oracle");
    assert_eq!(a, b, "two identical runs must deliver byte-identically");
}

// ===========================================================================
// Arm 2: config-only delivery — /tf AND /tf_static from two YAML strings
// ===========================================================================

/// Graph A's producer — the `docs/networking.md` broadcaster shape: ONE node
/// owning BOTH transform topics. External (HostDriven) so the harness fires
/// it deterministically; each fire publishes the dynamic broadcast on `tf`
/// and the static mounting transform on `tf_static` (the config arm fires it
/// exactly ONCE, so each topic carries exactly one frame with commit
/// sequence 0).
#[cerulion_node(external)]
#[derive(Default)]
struct TfSource {
    #[output]
    tf: TFMessage,
    #[output]
    tf_static: TFMessage,
}

#[cerulion_node_impl]
impl TfSource {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.tf
            .set_transforms_bytes(&two_transform_payload())
            .map_err(|e| NodeError::Logic(e.to_string()))?;
        self.tf_static
            .set_transforms_bytes(&static_transform_payload())
            .map_err(|e| NodeError::Logic(e.to_string()))?;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// Graph B's consumer: fires on each network-delivered TFMessage frame and
/// records the observed `transforms_bytes()` — the typed reader parsing the
/// re-injected frame — into a shared buffer. Instantiated TWICE (once per
/// transform topic), each instance wired to its own `source:`.
#[cerulion_node]
#[derive(Default)]
struct TfSink {
    #[input(trigger)]
    tf: TFMessage,
    seen: Arc<Mutex<Vec<u8>>>,
}

#[cerulion_node_impl]
impl TfSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let bytes = self.tf.transforms_bytes().to_vec();
        *self.seen.lock().expect("seen mutex") = bytes;
        Ok(())
    }
}

/// Config-only cross-machine transforms — /tf AND /tf_static as a unit: two
/// YAML strings, two `GraphRuntime::build`s, FULL-frame byte identity for
/// BOTH topics plus the typed `transforms_bytes()` read at each graph
/// consumer. The `network:` blocks are the ONLY network inputs — each side's
/// transport `NetworkConfig` is minted FROM its parsed block via
/// `GraphConfig::network_transport_config()` (exactly the CLI's
/// `resolve_run_network`).
///
/// Full-frame oracle discipline (same as arm 1): A fires EXACTLY ONCE, so
/// each topic carries exactly one frame with hand-predicted commit sequence 0
/// and loan timestamp. The timestamp is predictable because `step(delta)`
/// advances the gating clock ONCE by `delta` BEFORE any fire
/// (`Scheduler::begin_step`), and `loan_proxy` stamps `timestamp_ns` from the
/// manager's clock (the SAME `VirtualClock` Arc handed to both the transport
/// and the build) at loan time — so a fire launched from
/// `clock.set(TS_BASE)` + `step(1ms)` loans at exactly `TS_BASE + 1ms`. Both
/// ports loan within the one tick, so both frames carry the same stamp. The
/// one-shot publish is safe: the zenoh TCP link is reliable, and the
/// per-topic liveliness handshake below confirms B's subscriber declarations
/// reached A (declarations propagate in order per session, before the token)
/// BEFORE the single fanned-out publish.
#[test]
fn tf_and_tf_static_config_only_yaml_byte_identical() {
    let id = unique_id();
    let topic_tf = format!("/tf/{id}");
    let topic_static = format!("/tf_static/{id}");

    // Hand-predicted stamps for the one-shot fire (see doc comment above).
    const TS_BASE: u64 = 41_000_000;
    const STEP: Duration = Duration::from_millis(1);
    const TS_EXPECT: u64 = TS_BASE + 1_000_000;

    // ---- Graph A (egress, listens): ONE broadcaster node owning BOTH
    // transform topics — the docs/networking.md shape. The graph build is
    // network-free; A's egress is a GATEWAY computed from A's parsed
    // `network:` block. Bounded retry: a stolen probed port fails the GATEWAY's
    // zenoh bind (its watch start opens the listen session).
    let mut a_setup: Option<YamlArmASetup> = None;
    let mut attempt_errors: Vec<String> = Vec::new();
    for attempt in 0..3 {
        let port = probe_ephemeral_port();
        let yaml_a = format!(
            r#"
name: tf_robot
prefix: tfa
nodes:
  - id: broadcaster
    type: tf_source
    outputs:
      - name: tf
        schema: tf2_msgs/TFMessage
        topic: {topic_tf}
      - name: tf_static
        schema: tf2_msgs/TFMessage
        topic: {topic_static}
network:
  mode: peer
  listen:
    - tcp/127.0.0.1:{port}
  egress:
    - {topic_tf}
    - {topic_static}
"#
        );
        let config_a = parse_graph_raw(&yaml_a).expect("graph A YAML parses");
        let net_a = config_a
            .network_transport_config()
            .expect("graph A declares an enabled network block");
        let clock_a = Arc::new(VirtualClock::new());
        let mgr_a = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("tf_yaml_a_{id}_{attempt}"),
                clock: clock_a.clone(),
                subscriber_buffer_size: 16,
                network: Some(net_a),
            },
            cerulion_core::testing::iceoryx_test_config(),
        )
        .expect("init manager A");
        let mut factories_a: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
        factories_a.insert("broadcaster".to_string(), Box::new(TfSourceEntry::new()));
        // Compute A's gateway plan from the parsed YAML block BEFORE the build
        // moves config/factories (the CLI-analog wiring).
        let infos_a: IndexMap<String, NodeInfo> = factories_a
            .iter()
            .map(|(k, e)| (k.clone(), e.info().expect("node info")))
            .collect();
        let topology_a = GraphTopology::build(&config_a, &infos_a).expect("topology A");
        let plan_a = compute_gateway_plan(&config_a, &topology_a, &infos_a, NetworkPosture::Strict)
            .expect("plan A computes")
            .expect("graph A declares an enabled network block");
        let rt = GraphRuntime::build(config_a, factories_a, &mgr_a, clock_a.clone())
            .expect("graph A build (network-free)");
        match GatewayRuntime::new(Arc::clone(&mgr_a), plan_a) {
            Ok(gateway) => {
                a_setup = Some((rt, gateway, mgr_a, clock_a, port));
                break;
            }
            Err(e) => {
                let msg = format!("attempt {attempt} (probed port {port}): {e}");
                eprintln!("graph A gateway boot failed — {msg}");
                attempt_errors.push(msg);
            }
        }
    }
    let Some((mut rt_a, mut gateway_a, mgr_a, clock_a, port)) = a_setup else {
        panic!(
            "could not boot graph A's gateway listener in 3 attempts. Distinct \
             ports failing at the zenoh bind = the probe->rebind port-steal \
             race; 3 similar non-port errors = a deterministic failure. \
             All attempts:\n{}",
            attempt_errors.join("\n")
        );
    };

    // ---- Graph B (ingress, connects) — distinct SHM root, its network
    // minted from ITS yaml block; TWO single-input data-trigger consumers
    // (one per transform topic — each resolves its topic's ingress hash).
    let yaml_b = format!(
        r#"
name: tf_workstation
prefix: tfb
nodes:
  - id: viz_tf
    type: tf_sink
    inputs:
      - name: tf
        source: {topic_tf}
  - id: viz_static
    type: tf_sink
    inputs:
      - name: tf
        source: {topic_static}
network:
  mode: peer
  connect:
    - tcp/127.0.0.1:{port}
  ingress:
    - {topic_tf}
    - {topic_static}
"#
    );
    let config_b = parse_graph_raw(&yaml_b).expect("graph B YAML parses");
    let net_b = config_b
        .network_transport_config()
        .expect("graph B declares an enabled network block");
    let clock_b = Arc::new(VirtualClock::new());
    let mgr_b = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("tf_yaml_b_{id}"),
            clock: clock_b.clone(),
            subscriber_buffer_size: 16,
            network: Some(net_b),
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init manager B");
    let seen_tf = Arc::new(Mutex::new(Vec::<u8>::new()));
    let seen_static = Arc::new(Mutex::new(Vec::<u8>::new()));
    let mut factories_b: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories_b.insert(
        "viz_tf".to_string(),
        Box::new(TfSinkEntry::with_state(TfSink {
            seen: Arc::clone(&seen_tf),
            ..Default::default()
        })),
    );
    factories_b.insert(
        "viz_static".to_string(),
        Box::new(TfSinkEntry::with_state(TfSink {
            seen: Arc::clone(&seen_static),
            ..Default::default()
        })),
    );
    // Compute B's gateway plan from ITS parsed block BEFORE the build moves
    // config/factories (the CLI-analog wiring): both `ingress:` entries carry
    // the TFMessage schema hash resolved from the sinks' macro InputMeta.
    let infos_b: IndexMap<String, NodeInfo> = factories_b
        .iter()
        .map(|(k, e)| (k.clone(), e.info().expect("node info")))
        .collect();
    let topology_b = GraphTopology::build(&config_b, &infos_b).expect("topology B");
    let plan_b = compute_gateway_plan(&config_b, &topology_b, &infos_b, NetworkPosture::Strict)
        .expect("plan B computes")
        .expect("graph B declares an enabled network block");
    // The graph build is network-free — B's ingress lives in ITS
    // gateway, booted below.
    let mut rt_b = GraphRuntime::build(config_b, factories_b, &mgr_b, clock_b)
        .expect("graph B build (network-free)");
    // Boot B's GATEWAY after the build (the sinks' subscribers have opened both
    // topics' services, so each ingress re-injection publisher attaches to the
    // existing service): registers BOTH network→local ingress bridges AND
    // declares the two DEMAND TopicTokens that flip A's egress flags —
    // `register_ingress` declares each data subscriber BEFORE its token (inside
    // `register_ingress_topic`), so A's first forwarded frames route here (no
    // first-frame race). Ingress needs no drive loop (re-injection rides the
    // zenoh callback), but the handle must stay alive for the test's duration.
    let _gateway_b = GatewayRuntime::new(Arc::clone(&mgr_b), plan_b)
        .expect("graph B gateway boot (its ingress bridges connect to A's live listener)");

    // Raw harness taps on B's re-injected topics — the FULL-frame pins.
    // Created AFTER the graph build (the build's provisioning owns the
    // services; the taps ride the introspection headroom slots) and BEFORE
    // the fire, so both are connected when the single re-injected send lands.
    let tap_tf = mgr_b.create_subscriber(&topic_tf).expect("tap /tf");
    let tap_static = mgr_b
        .create_subscriber(&topic_static)
        .expect("tap /tf_static");

    // ---- Liveliness handshake, PER TOPIC: each of B's ingress tokens must
    // flip its egress flag on A (through the allow-list — both topics ARE
    // declared under A's `egress:`). `register_topic` is idempotent: each
    // handle IS the flag A's gateway reads.
    for topic in [&topic_tf, &topic_static] {
        let flag = mgr_a
            .bridge_manager()
            .register_topic(topic)
            .expect("A bridge flag handle");
        let deadline = Instant::now() + Duration::from_secs(10);
        while !flag.load(Ordering::Relaxed) {
            assert!(
                Instant::now() < deadline,
                "liveliness handshake did not flip A's bridge flag for {topic} \
                 within 10s — B's ingress TopicToken never reached A's gateway watcher"
            );
            std::thread::sleep(Duration::from_millis(20));
        }
    }
    // Attach A's egress taps for BOTH topics BEFORE the one-shot fire (the taps
    // have no history — they only forward frames published after they attach).
    let attach_deadline = Instant::now() + Duration::from_secs(5);
    while gateway_a.active_tap_topics().len() < 2 {
        assert!(
            Instant::now() < attach_deadline,
            "A's gateway taps did not attach for both topics (have {:?})",
            gateway_a.active_tap_topics()
        );
        gateway_a.drive_once().expect("drive A's gateway attach");
        std::thread::sleep(Duration::from_millis(10));
    }

    // ---- The one-shot fire: set the gating clock, trigger, ONE step. Both
    // output ports publish once (sequence 0 each) with the hand-predicted
    // TS_EXPECT stamp; A's gateway taps + forwards both frames to zenoh.
    clock_a.set(TS_BASE);
    rt_a.trigger_external("broadcaster")
        .expect("trigger A's broadcaster");
    rt_a.step(STEP);
    gateway_a.drive_once().expect("drive A's gateway forward");

    // ---- Collect (bounded): step B (fires the data-trigger sinks on the
    // re-injected frames) while draining both raw taps.
    let oracle_tf_bytes = two_transform_payload();
    let oracle_static_bytes = static_transform_payload();
    let mut tap_tf_frame: Option<Received> = None;
    let mut tap_static_frame: Option<Received> = None;
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        gateway_a.drive_once().expect("drive A's gateway forward");
        rt_b.step(STEP);
        if tap_tf_frame.is_none() {
            tap_tf_frame = take_one(&tap_tf);
        }
        if tap_static_frame.is_none() {
            tap_static_frame = take_one(&tap_static);
        }
        let sinks_fired = !seen_tf.lock().expect("seen_tf mutex").is_empty()
            && !seen_static.lock().expect("seen_static mutex").is_empty();
        if tap_tf_frame.is_some() && tap_static_frame.is_some() && sinks_fired {
            break;
        }
        assert!(
            Instant::now() < deadline,
            "config-only /tf + /tf_static delivery did not complete within 10s \
             (tap_tf: {}, tap_static: {}, sinks fired: {sinks_fired})",
            tap_tf_frame.is_some(),
            tap_static_frame.is_some()
        );
        std::thread::sleep(Duration::from_millis(20));
    }

    // FULL-frame byte identity for BOTH topics (arm-1 oracle discipline:
    // header via write_to_buf + payload + offset-table bytes, hand-predicted
    // sequence 0 + TS_EXPECT). The tuples are std types (Debug), so expect()
    // is fine; the loop above breaks only when both are Some.
    let got_tf = tap_tf_frame.expect("loop invariant: /tf tap frame present");
    let got_static = tap_static_frame.expect("loop invariant: /tf_static tap frame present");
    assert_eq!(
        got_tf,
        tf_oracle(&[(TS_EXPECT, oracle_tf_bytes.clone())]).remove(0),
        "/tf must arrive FULL-frame byte-identical through the config-only path"
    );
    assert_eq!(
        got_static,
        tf_oracle(&[(TS_EXPECT, oracle_static_bytes.clone())]).remove(0),
        "/tf_static must arrive FULL-frame byte-identical through the \
         config-only path"
    );

    // Typed reads at the graph consumers: each sink's transforms_bytes()
    // equals its topic's hand-built payload BIT-FOR-BIT.
    assert_eq!(
        *seen_tf.lock().expect("seen_tf mutex"),
        oracle_tf_bytes,
        "viz_tf's typed transforms_bytes() read must match /tf's payload"
    );
    assert_eq!(
        *seen_static.lock().expect("seen_static mutex"),
        oracle_static_bytes,
        "viz_static's typed transforms_bytes() read must match /tf_static's \
         payload"
    );
}

// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end: the DdsBridge queue→tick→publish path over a REAL
//! graph runtime + iceoryx2, driven with INJECTED decoded samples (no DDS, no
//! robot — the camera_jpeg `node_publish_test` seam).
//!
//! An `#[cerulion_node(external)]` node fires on the polled path via
//! `GraphRuntime::trigger_external` + `step()`, and the polled path NEVER
//! queries `external_source` — so no DDS participant, config file,
//! or env var is touched here. We hand the node a SHARED `SampleQueue`
//! (`DdsBridge::with_shared_queue`), inject decoded `cerulion_go2_dds` samples exactly as
//! the pump would, fire, and observe each output port on a raw subscriber.
//!
//! Pinned:
//! - Cloud pass-through: fixed geometry verbatim, `data` bytes verbatim, the
//!   packed `fields` blob equal to an INDEPENDENT hand-built byte oracle (the
//!   cerulion_viz `parse_point_fields` contract), header stamp = the DDS sample's.
//! - Odometry projection: position/velocity values, the Unitree→ROS
//!   quaternion REORDER ([w,x,y,z] → xyzw fields), stamp = the state's.
//! - TwistStamped: values verbatim + the node-clock stamp schedule
//!   (`t0 + k*STEP` — the camera stamp-determinism contract).
//! - Request → String: the parameter JSON verbatim.
//! - The PARTIAL-FIRE GATE: a fire that drained only a cloud publishes
//!   NOTHING on the other three ports (the output gate discards untouched
//!   ports — the reason `twist` is TwistStamped, see lib.rs docs).
//! - Latest-wins eviction accounting through the tick.
//! - An empty-queue fire publishes nothing anywhere.
//! - Drain determinism (Principle #7): the same injected input script run
//!   twice on isolated transports yields byte-identical observed sequences,
//!   each anchored to the hand oracles above (never a bare self-compare).
//!
//! Isolated per-test SHM root (`init_for_test`) — parallel-safe, no
//! `#[serial]`.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::{Clock, VirtualClock};
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::CerulionSubscriber;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::TwistStamped;
use native_ros2_messages::nav_msgs::Odometry;
use native_ros2_messages::sensor_msgs::PointCloud2;
use native_ros2_messages::std_msgs::String as RosString;

use cerulion_go2_dds::messages;
use dds_bridge::queue::SampleQueue;
use dds_bridge::registry::{BridgeSample, RosType};
use dds_bridge::{DdsBridge, DdsBridgeEntry, TickStats};

const STEP: Duration = Duration::from_millis(20);
const STEP_NS: u64 = 20_000_000;

// ---------------------------------------------------------------------------
// Hand fixtures + independent byte oracles
// ---------------------------------------------------------------------------

/// The known 2-point cloud (points (1,2,3)/(4,5,6), LE f32, x/y/z at 0/4/8).
fn sample_cloud() -> messages::PointCloud2 {
    messages::PointCloud2 {
        header: messages::Header {
            stamp: messages::Time {
                sec: 12,
                nanosec: 34,
            },
            frame_id: "lidar".to_string(),
        },
        height: 1,
        width: 2,
        fields: [("x", 0u32), ("y", 4), ("z", 8)]
            .into_iter()
            .map(|(n, off)| messages::PointField {
                name: n.to_string(),
                offset: off,
                datatype: 7,
                count: 1,
            })
            .collect(),
        is_bigendian: false,
        point_step: 12,
        row_step: 24,
        data: vec![
            0, 0, 128, 63, 0, 0, 0, 64, 0, 0, 64, 64, // (1.0, 2.0, 3.0)
            0, 0, 128, 64, 0, 0, 160, 64, 0, 0, 192, 64, // (4.0, 5.0, 6.0)
        ],
        is_dense: true,
    }
}

/// INDEPENDENT hand-built packed `fields` blob for x/y/z float32 at 0/4/8 —
/// the byte layout `cerulion_viz::pointcloud::parse_point_fields` documents
/// (`[name_len u32][name][offset u32][datatype u8][count u32]` per record).
/// NOT built via `mapping::encode_point_fields` — this is the oracle it is
/// checked against.
fn oracle_fields_blob() -> Vec<u8> {
    let mut v = Vec::new();
    for (name, off) in [("x", 0u32), ("y", 4), ("z", 8)] {
        v.extend_from_slice(&(name.len() as u32).to_le_bytes());
        v.extend_from_slice(name.as_bytes());
        v.extend_from_slice(&off.to_le_bytes());
        v.push(7u8);
        v.extend_from_slice(&1u32.to_le_bytes());
    }
    v
}

fn sample_sport() -> messages::SportModeState {
    messages::SportModeState {
        stamp: messages::Time {
            sec: 100,
            nanosec: 250_000_000,
        },
        imu_state: messages::ImuState {
            // Unitree [w,x,y,z] — DISTINCT components so a wrong reorder is a
            // wrong VALUE downstream.
            quaternion: [0.5, 0.1, 0.2, 0.3],
            ..Default::default()
        },
        position: [1.5, -2.5, 0.25],
        velocity: [0.5, 0.0, 0.0],
        yaw_speed: 0.25,
        ..Default::default()
    }
}

fn sample_twist() -> messages::Twist {
    messages::Twist {
        linear: messages::Vector3 {
            x: 0.5,
            y: 0.0,
            z: 0.0,
        },
        angular: messages::Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.25,
        },
    }
}

fn sample_request() -> messages::Request {
    let mut r = messages::Request::default();
    r.header.identity.id = 7;
    r.header.identity.api_id = 1008;
    r.parameter = r#"{"x":0.1,"y":0.0,"z":0.0}"#.to_string();
    r
}

// ---------------------------------------------------------------------------
// Rig
// ---------------------------------------------------------------------------

struct Rig {
    rt: GraphRuntime,
    queue: Arc<Mutex<SampleQueue>>,
    stats: Arc<TickStats>,
    clock: Arc<VirtualClock>,
    obs_cloud: CerulionSubscriber,
    obs_odom: CerulionSubscriber,
    obs_twist: CerulionSubscriber,
    obs_request: CerulionSubscriber,
}

fn build_rig(prefix: &str) -> Rig {
    build_rig_capped(prefix, Some(64 * 1024))
}

/// Build the rig with a configurable `cloud` output `max_slice_len`. The boundary
/// tests use a small cap so an oversized fields or data write fails. Both
/// paths must discard the whole cloud while allowing sibling ports to publish.
fn build_rig_capped(prefix: &str, cloud_max_slice_len: Option<usize>) -> Rig {
    let clock = Arc::new(VirtualClock::new());
    let ix_config = cerulion_core::testing::iceoryx_test_config();
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("dds_bridge_e2e_{prefix}"),
            clock: clock.clone(),
            subscriber_buffer_size: 16,
            network: None,
        },
        ix_config,
    )
    .expect("init isolated e2e transport");

    let queue: Arc<Mutex<SampleQueue>> = Arc::new(Mutex::new(SampleQueue::default()));
    let stats: Arc<TickStats> = Arc::new(TickStats::default());

    let outputs = [
        ("cloud", "sensor_msgs/PointCloud2", cloud_max_slice_len),
        ("odom", "nav_msgs/Odometry", None),
        ("twist", "geometry_msgs/TwistStamped", None),
        ("request_json", "std_msgs/String", None),
    ]
    .into_iter()
    .map(|(name, schema, max_slice_len)| OutputDef {
        name: name.to_string(),
        schema: schema.to_string(),
        max_slice_len,
        history_size: 0,
        topic: None,
    })
    .collect();

    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: format!("dds_bridge_e2e_{prefix}"),
        prefix: prefix.to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "bridge".to_string(),
            node_type: "dds_bridge".to_string(),
            inputs: vec![],
            outputs,
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "bridge".to_string(),
        Box::new(DdsBridgeEntry::with_state(
            DdsBridge::with_shared_queue_and_stats(Arc::clone(&queue), Arc::clone(&stats)),
        )),
    );

    let rt = GraphRuntime::build(config, factories, &mgr, clock.clone())
        .expect("the bridge graph must build (external node, four outputs)");

    let sub = |port: &str| {
        mgr.create_subscriber(&format!("/{prefix}/bridge/{port}"))
            .expect("observer subscriber")
    };

    Rig {
        rt,
        queue,
        stats,
        clock,
        obs_cloud: sub("cloud"),
        obs_odom: sub("odom"),
        obs_twist: sub("twist"),
        obs_request: sub("request_json"),
    }
}

fn push(rig: &Rig, sample: BridgeSample) {
    rig.queue
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
        .push(sample);
}

fn lifetime_dropped(rig: &Rig, t: RosType) -> u64 {
    rig.queue
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
        .lifetime_dropped(t)
}

fn fire(rig: &mut Rig) {
    rig.rt
        .trigger_external("bridge")
        .expect("trigger_external bridge");
    rig.rt.step(STEP);
}

/// Parse the 8-byte fixed stamp at the head of a nested Header payload (the
/// camera_jpeg observation pattern — `stamp` is the Header schema's first
/// fixed field).
fn parse_stamp(header_bytes: &[u8]) -> (i32, u32) {
    assert!(header_bytes.len() >= 8, "header carries the 8-byte stamp");
    (
        i32::from_le_bytes(header_bytes[0..4].try_into().unwrap()),
        u32::from_le_bytes(header_bytes[4..8].try_into().unwrap()),
    )
}

/// Parse the nested `Header.frame_id` string out of the raw header payload:
/// a `std_msgs/Header`'s WIRE_FIXED_SIZE is 8 (the stamp), so its single
/// variable field (`frame_id`) is offset-table entry 0 at bytes 8..16 of the
/// nested payload. Reading the AUTHORITATIVE offset/length (not assuming where
/// the data starts) via the same helper the generated readers use.
fn parse_header_frame_id(header_bytes: &[u8]) -> String {
    let (off, len) = cerulion_core::shm_runtime::read_offset_entry(header_bytes, 8, 0);
    let (off, len) = (off as usize, len as usize);
    assert!(
        off + len <= header_bytes.len(),
        "frame_id slice {off}+{len} in bounds of {}",
        header_bytes.len()
    );
    std::str::from_utf8(&header_bytes[off..off + len])
        .expect("frame_id is valid UTF-8")
        .to_string()
}

/// One observed cloud frame (the fields the oracle pins), Debug-comparable.
/// The ROS bools are u8 on the generated wire schema.
#[derive(Debug, PartialEq)]
struct CloudObs {
    height: u32,
    width: u32,
    point_step: u32,
    row_step: u32,
    is_bigendian: u8,
    is_dense: u8,
    data: Vec<u8>,
    fields_blob: Vec<u8>,
    stamp: (i32, u32),
    frame_id: String,
}

fn observe_cloud(rig: &mut Rig) -> Option<CloudObs> {
    rig.obs_cloud
        .try_view::<PointCloud2, _>(|v| CloudObs {
            height: v.height,
            width: v.width,
            point_step: v.point_step,
            row_step: v.row_step,
            is_bigendian: v.is_bigendian,
            is_dense: v.is_dense,
            data: v.data().to_vec(),
            fields_blob: v.fields_bytes().to_vec(),
            stamp: parse_stamp(v.header_bytes()),
            frame_id: parse_header_frame_id(v.header_bytes()),
        })
        .expect("try_view cloud")
}

#[derive(Debug, PartialEq)]
struct OdomObs {
    position: [f64; 3],
    orientation_xyzw: [f64; 4],
    linear_x: f64,
    angular_z: f64,
    frame_id: String,
    child_frame_id: String,
    stamp: (i32, u32),
}

fn observe_odom(rig: &mut Rig) -> Option<OdomObs> {
    rig.obs_odom
        .try_view::<Odometry, _>(|v| OdomObs {
            position: [
                v.pose.pose.position.x,
                v.pose.pose.position.y,
                v.pose.pose.position.z,
            ],
            orientation_xyzw: [
                v.pose.pose.orientation.x,
                v.pose.pose.orientation.y,
                v.pose.pose.orientation.z,
                v.pose.pose.orientation.w,
            ],
            linear_x: v.twist.twist.linear.x,
            angular_z: v.twist.twist.angular.z,
            frame_id: parse_header_frame_id(v.header_bytes()),
            child_frame_id: v.child_frame_id().map(str::to_string).unwrap_or_default(),
            stamp: parse_stamp(v.header_bytes()),
        })
        .expect("try_view odom")
}

#[derive(Debug, PartialEq)]
struct TwistObs {
    linear: [f64; 3],
    angular: [f64; 3],
    stamp: (i32, u32),
    frame_id: String,
}

fn observe_twist(rig: &mut Rig) -> Option<TwistObs> {
    rig.obs_twist
        .try_view::<TwistStamped, _>(|v| TwistObs {
            linear: [v.twist.linear.x, v.twist.linear.y, v.twist.linear.z],
            angular: [v.twist.angular.x, v.twist.angular.y, v.twist.angular.z],
            stamp: parse_stamp(v.header_bytes()),
            frame_id: parse_header_frame_id(v.header_bytes()),
        })
        .expect("try_view twist")
}

fn observe_request(rig: &mut Rig) -> Option<String> {
    rig.obs_request
        .try_view::<RosString, _>(|v| v.data().map(str::to_string).unwrap_or_default())
        .expect("try_view request")
}

/// Bounded warm-up: fire all-four-port beats until every observer has seen a
/// frame (first same-process iceoryx2 delivery can lag one cycle). Warm-up
/// frames are drained + discarded.
fn warmup(rig: &mut Rig) {
    let (mut c, mut o, mut t, mut r) = (false, false, false, false);
    for _ in 0..200 {
        push(rig, BridgeSample::Cloud(sample_cloud()));
        push(rig, BridgeSample::Sport(sample_sport()));
        push(rig, BridgeSample::Twist(sample_twist()));
        push(rig, BridgeSample::Request(sample_request()));
        fire(rig);
        c |= observe_cloud(rig).is_some();
        o |= observe_odom(rig).is_some();
        t |= observe_twist(rig).is_some();
        r |= observe_request(rig).is_some();
        if c && o && t && r {
            return;
        }
    }
    panic!("bridge outputs never all established subscriber connections in 200 warm-up fires");
}

// ---------------------------------------------------------------------------
// The script (each beat asserts its hand oracle inline; the collected Debug
// record is the two-run determinism artifact)
// ---------------------------------------------------------------------------

fn run_script(prefix: &str) -> Vec<String> {
    let mut rig = build_rig(prefix);
    warmup(&mut rig);
    let t0 = rig.clock.now_ns();
    let evicted_at_t0 = lifetime_dropped(&rig, RosType::PointCloud2);
    let mut record: Vec<String> = Vec::new();
    let mut beat: u64 = 0;

    // Beat 1: cloud only — pass-through oracle + the PARTIAL-FIRE GATE.
    push(&rig, BridgeSample::Cloud(sample_cloud()));
    fire(&mut rig);
    beat += 1;
    let cloud = observe_cloud(&mut rig).expect("cloud published");
    assert_eq!(
        cloud,
        CloudObs {
            height: 1,
            width: 2,
            point_step: 12,
            row_step: 24,
            is_bigendian: 0,
            is_dense: 1,
            data: sample_cloud().data,
            fields_blob: oracle_fields_blob(),
            stamp: (12, 34),
            // The DDS cloud's frame passes through verbatim (hand oracle).
            frame_id: "lidar".to_string(),
        },
        "cloud pass-through must match the hand oracle exactly (bools as wire u8)"
    );
    assert_eq!(observe_odom(&mut rig), None, "odom port must stay silent");
    assert_eq!(observe_twist(&mut rig), None, "twist port must stay silent");
    assert_eq!(
        observe_request(&mut rig),
        None,
        "request port must stay silent"
    );
    record.push(format!("cloud:{cloud:?}"));

    // Beat 2: sport state only — the Odometry projection oracle.
    push(&rig, BridgeSample::Sport(sample_sport()));
    fire(&mut rig);
    beat += 1;
    let odom = observe_odom(&mut rig).expect("odom published");
    assert_eq!(
        odom,
        OdomObs {
            position: [1.5, -2.5, 0.25],
            // The Unitree [w,x,y,z] = [0.5,0.1,0.2,0.3] REORDERED to xyzw.
            orientation_xyzw: [0.1f32 as f64, 0.2f32 as f64, 0.3f32 as f64, 0.5f32 as f64],
            linear_x: 0.5,
            angular_z: 0.25,
            // header.frame_id = odom, child_frame_id = base (hand oracles;
            // a swapped/dropped frame_id write fails here).
            frame_id: "odom".to_string(),
            child_frame_id: "base".to_string(),
            stamp: (100, 250_000_000),
        },
        "odometry projection must match the hand oracle (incl. the quaternion reorder)"
    );
    assert_eq!(observe_cloud(&mut rig), None, "cloud port must stay silent");
    record.push(format!("odom:{odom:?}"));

    // Beat 3: twist only — node-clock stamp schedule (t0 + beat*STEP).
    push(&rig, BridgeSample::Twist(sample_twist()));
    fire(&mut rig);
    beat += 1;
    let twist = observe_twist(&mut rig).expect("twist published");
    let expect_ns = t0 + beat * STEP_NS;
    assert_eq!(
        twist,
        TwistObs {
            linear: [0.5, 0.0, 0.0],
            angular: [0.0, 0.0, 0.25],
            stamp: (
                (expect_ns / 1_000_000_000) as i32,
                (expect_ns % 1_000_000_000) as u32
            ),
            // TwistStamped's frame is the robot base (hand oracle).
            frame_id: "base".to_string(),
        },
        "twist values + the deterministic node-clock stamp schedule"
    );
    record.push(format!("twist:{twist:?}"));

    // Beat 4: request only — parameter JSON verbatim.
    push(&rig, BridgeSample::Request(sample_request()));
    fire(&mut rig);
    beat += 1;
    let req = observe_request(&mut rig).expect("request published");
    assert_eq!(req, r#"{"x":0.1,"y":0.0,"z":0.0}"#);
    record.push(format!("request:{req:?}"));

    // Beat 5: ALL FOUR in one fire — every port publishes.
    push(&rig, BridgeSample::Cloud(sample_cloud()));
    push(&rig, BridgeSample::Sport(sample_sport()));
    push(&rig, BridgeSample::Twist(sample_twist()));
    push(&rig, BridgeSample::Request(sample_request()));
    fire(&mut rig);
    beat += 1;
    assert!(
        observe_cloud(&mut rig).is_some(),
        "cloud in the multi-port fire"
    );
    assert!(
        observe_odom(&mut rig).is_some(),
        "odom in the multi-port fire"
    );
    assert!(
        observe_twist(&mut rig).is_some(),
        "twist in the multi-port fire"
    );
    assert!(
        observe_request(&mut rig).is_some(),
        "request in the multi-port fire"
    );
    record.push("multi:all-four".to_string());

    // Beat 6: latest-wins — two clouds pushed, newest published, EXACT
    // eviction accounting.
    let mut newer = sample_cloud();
    newer.header.stamp.sec = 99;
    push(&rig, BridgeSample::Cloud(sample_cloud()));
    push(&rig, BridgeSample::Cloud(newer));
    fire(&mut rig);
    beat += 1;
    let winner = observe_cloud(&mut rig).expect("newest cloud published");
    assert_eq!(winner.stamp, (99, 34), "the NEWER cloud wins");
    assert_eq!(
        lifetime_dropped(&rig, RosType::PointCloud2) - evicted_at_t0,
        1,
        "exactly one eviction since t0"
    );
    record.push(format!("latest:{:?}", winner.stamp));

    // Beat 7: empty fire — nothing anywhere.
    fire(&mut rig);
    let _ = beat;
    assert_eq!(observe_cloud(&mut rig), None);
    assert_eq!(observe_odom(&mut rig), None);
    assert_eq!(observe_twist(&mut rig), None);
    assert_eq!(observe_request(&mut rig), None);
    record.push("empty:none".to_string());

    record
}

#[test]
fn bridge_publishes_mapped_ports_with_hand_oracles() {
    // The full script: every beat's inline assert IS the oracle anchoring.
    run_script("dbr1");
}

#[test]
fn two_isolated_runs_are_byte_identical() {
    // Drain determinism (Principle #7): the same injected script on two
    // isolated transports yields identical observed sequences. Each run's
    // beats are ALSO hand-oracle-anchored inside run_script, so this is not a
    // bare self-compare.
    let a = run_script("dbr2");
    let b = run_script("dbr3");
    assert_eq!(a, b, "two runs must observe byte-identical sequences");
}

// ---------------------------------------------------------------------------
// Per-port write-error ISOLATION (a hostile sample must not drop siblings)
// ---------------------------------------------------------------------------

/// A cloud whose re-encoded `fields` blob (300 records ≈ 4.2 KiB) far exceeds a
/// small cloud output slot → `set_fields_bytes` fails the loan → `write_cloud`
/// returns Err without publishing a partial frame.
fn hostile_fields_cloud() -> messages::PointCloud2 {
    let mut c = sample_cloud();
    c.fields = (0..300u32)
        .map(|i| messages::PointField {
            name: "f".to_string(),
            offset: 0,
            datatype: 7,
            count: i,
        })
        .collect();
    c
}

#[test]
fn hostile_cloud_write_error_isolates_siblings_counts_and_recovers() {
    // A small cloud slot so the hostile fields blob overflows set_fields_bytes.
    let mut rig = build_rig_capped("dbf1", Some(1024));
    warmup(&mut rig);
    const N: u64 = 5;
    for _ in 0..N {
        // Cloud (slot 0, drained FIRST) errs; the healthy sport (slot 1) is
        // already drained. If write_cloud's `?` aborted the whole tick, the
        // odom sibling would be LOST; instead the error is counted+latched and
        // the tick continues.
        push(&rig, BridgeSample::Cloud(hostile_fields_cloud()));
        push(&rig, BridgeSample::Sport(sample_sport()));
        fire(&mut rig);
        let odom = observe_odom(&mut rig).expect("odom sibling SURVIVES the cloud write error");
        assert_eq!(odom.position, [1.5, -2.5, 0.25]);
        assert_eq!(odom.child_frame_id, "base");
        assert_eq!(
            observe_cloud(&mut rig),
            None,
            "the errored cloud publishes nothing"
        );
    }
    // The write-error hop is real + queryable (Principle #3).
    assert_eq!(rig.stats.write_errors_total(RosType::PointCloud2), N);
    assert_eq!(
        rig.stats.write_errors_total(RosType::SportModeState),
        0,
        "the sibling port never errored"
    );
    // Recovery: a HEALTHY cloud writes cleanly and publishes (latch re-arms).
    push(&rig, BridgeSample::Cloud(sample_cloud()));
    fire(&mut rig);
    let healthy = observe_cloud(&mut rig).expect("healthy cloud publishes after the error regime");
    assert_eq!(healthy.stamp, (12, 34));
    assert_eq!(healthy.frame_id, "lidar");
    assert_eq!(healthy.data, sample_cloud().data);
    assert_eq!(
        rig.stats.write_errors_total(RosType::PointCloud2),
        N,
        "recovery adds no error"
    );
}

// ---------------------------------------------------------------------------
// Cloud payload boundaries: complete publish or counted drop.
// ---------------------------------------------------------------------------

/// 1024 xyz points: repeat the hand-authored two-point byte oracle. Geometry
/// and payload length agree, so this exercises capacity rather than bad input.
fn large_data_cloud() -> messages::PointCloud2 {
    let mut c = sample_cloud();
    c.width = 1024;
    c.row_step = 12 * 1024;
    c.data = c.data.repeat(512);
    c
}

#[test]
fn oversized_cloud_data_drops_whole_frame_counts_and_recovers() {
    let mut rig = build_rig_capped("db_cloud_ceiling", Some(1024));
    warmup(&mut rig);
    const N: u64 = 4;
    for _ in 0..N {
        push(&rig, BridgeSample::Cloud(large_data_cloud()));
        push(&rig, BridgeSample::Sport(sample_sport()));
        fire(&mut rig);
        assert_eq!(observe_cloud(&mut rig), None, "no torn cloud may publish");
        let odom = observe_odom(&mut rig).expect("a cloud drop does not drop its sibling");
        assert_eq!(odom.position, [1.5, -2.5, 0.25]);
    }
    assert_eq!(rig.stats.write_errors_total(RosType::PointCloud2), N);
    assert_eq!(rig.stats.write_errors_total(RosType::SportModeState), 0);

    push(&rig, BridgeSample::Cloud(sample_cloud()));
    fire(&mut rig);
    let full = observe_cloud(&mut rig).expect("healthy cloud publishes after oversized frames");
    assert_eq!(full.data, sample_cloud().data);
    assert_eq!(full.frame_id, "lidar");
    assert_eq!(full.stamp, (12, 34));
    assert_eq!(full.fields_blob, oracle_fields_blob());
    assert_eq!(rig.stats.write_errors_total(RosType::PointCloud2), N);

    push(&rig, BridgeSample::Cloud(large_data_cloud()));
    fire(&mut rig);
    assert_eq!(observe_cloud(&mut rig), None);
    assert_eq!(rig.stats.write_errors_total(RosType::PointCloud2), N + 1);
}

#[test]
fn larger_cloud_grows_adaptive_loan_and_preserves_every_byte() {
    let mut rig = build_rig_capped("db_cloud_growth", Some(64 * 1024));
    warmup(&mut rig);
    // Warm the size window with small complete frames. The next cloud is
    // within max_slice_len but much larger than the current adaptive loan.
    for _ in 0..cerulion_core::transport::adaptive_sizer::WINDOW_SIZE {
        push(&rig, BridgeSample::Cloud(sample_cloud()));
        fire(&mut rig);
        assert_eq!(observe_cloud(&mut rig).unwrap().data, sample_cloud().data);
    }
    let large = large_data_cloud();
    push(&rig, BridgeSample::Cloud(large.clone()));
    fire(&mut rig);
    let full = observe_cloud(&mut rig).expect("larger frame within the ceiling must publish");
    assert_eq!(full.width, 1024);
    assert_eq!(full.height, 1);
    assert_eq!(full.point_step, 12);
    assert_eq!(full.row_step, 12 * 1024);
    assert_eq!(full.data, large.data);
    assert_eq!(full.data.len(), (full.height * full.row_step) as usize);
    assert_eq!(full.frame_id, "lidar");
    assert_eq!(full.stamp, (12, 34));
    assert_eq!(full.fields_blob, oracle_fields_blob());
    assert_eq!(rig.stats.write_errors_total(RosType::PointCloud2), 0);
}

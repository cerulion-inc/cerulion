// SPDX-License-Identifier: AGPL-3.0-only
//! Oracle-vector tests for `cerulion ros2 attach`'s PURE half
//! (`cerulion_cli_engine::ros_cmd`). No DDS peer — a hand-built endpoint list
//! is injected through the `cerulion_dds::DdsDiscovery` seam (`FakeDiscovery`)
//! and every assertion is against a HAND-WRITTEN oracle (never a self-compare):
//! discovery-report text, the resolvable/unresolvable partition + remediation,
//! byte-exact config + graph YAML (also cross-parsed for structural validity),
//! and the full consent ladder. Parallel-safe (tempdirs; FakeDiscovery never
//! touches the DDS one-per-process slot), so NO `#[serial]`.

use std::net::IpAddr;
use std::path::Path;
use std::time::Duration;

use cerulion_cli_engine::error::{CliError, CliResult};
use cerulion_cli_engine::local_harvest::LocalAmentAcquirer;
use cerulion_cli_engine::ros_cmd::{
    self, aggregate_topics, assign_typed_ports, build_mappings, exclude_malformed_topics,
    generate_bridge_config_with_store, generate_bridge_graph, partition_topics_with,
    render_discovery_report, AttachOutcome, AttachSchemaChain, BridgeRoute, DroppedDuplicate,
    DroppedTopicConflict, ExcludedMalformedTopic, RawRoutedLoser, RosAttachOptions,
};
use cerulion_dds::{
    AcquiredMsg, AcquiredSchema, AcquisitionOutcome, AcquisitionRung, DdsDiscovery, DdsError,
    DiscoveredEndpoint, DiscoveredNode, DiscoveredQos, DiscoveryParams, DiscoveryResult,
    EndpointKind, NoopAcquirer, QosDurability, QosReliability, RungSkip, SchemaAcquirer,
    TypeAcquisition,
};

// ─────────────────────────── Fixtures / seam ───────────────────────────────

/// A `DdsDiscovery` that replays a hand-built endpoint list (or a canned
/// error) — the DDS-free injection seam for the pure command logic.
struct FakeDiscovery {
    endpoints: Vec<DiscoveredEndpoint>,
    own_endpoints_hidden: usize,
    /// The `ros_discovery_info` node table the migration section
    /// attributes endpoints with. Empty everywhere but the migration tests
    /// (the wire-rung fields are otherwise unused by the pure command logic).
    nodes: Vec<DiscoveredNode>,
    fail: Option<DdsError>,
}

impl FakeDiscovery {
    fn ok(endpoints: Vec<DiscoveredEndpoint>) -> Self {
        Self {
            endpoints,
            own_endpoints_hidden: 0,
            nodes: Vec::new(),
            fail: None,
        }
    }
    /// Own-endpoint fixture: a run where the backend hid `hidden` of our own
    /// discovery endpoints (the report must footer-account for them).
    fn ok_with_hidden(endpoints: Vec<DiscoveredEndpoint>, hidden: usize) -> Self {
        Self {
            endpoints,
            own_endpoints_hidden: hidden,
            nodes: Vec::new(),
            fail: None,
        }
    }
    /// A run whose harvest carried a node table.
    fn ok_with_nodes(endpoints: Vec<DiscoveredEndpoint>, nodes: Vec<DiscoveredNode>) -> Self {
        Self {
            endpoints,
            own_endpoints_hidden: 0,
            nodes,
            fail: None,
        }
    }
    fn failing(e: DdsError) -> Self {
        Self {
            endpoints: Vec::new(),
            own_endpoints_hidden: 0,
            nodes: Vec::new(),
            fail: Some(e),
        }
    }
}

impl DdsDiscovery for FakeDiscovery {
    fn discover(&self, _params: &DiscoveryParams) -> Result<DiscoveryResult, DdsError> {
        match &self.fail {
            Some(e) => Err(e.clone()),
            None => Ok(DiscoveryResult {
                endpoints: self.endpoints.clone(),
                own_endpoints_hidden: self.own_endpoints_hidden,
                nodes: self.nodes.clone(),
                participants: Vec::new(),
            }),
        }
    }
}

fn ep(
    dds_topic: &str,
    type_name: &str,
    kind: EndpointKind,
    qos: DiscoveredQos,
) -> DiscoveredEndpoint {
    DiscoveredEndpoint {
        dds_topic: dds_topic.to_string(),
        type_name: type_name.to_string(),
        qos,
        kind,
        // The pure engine logic never reads the wire-rung
        // discovery fields (the wire rung is a live acquirer, box-tested).
        type_hash: None,
        writer_guid: None,
    }
}

const BE_VOL: DiscoveredQos = DiscoveredQos {
    reliability: QosReliability::BestEffort,
    durability: QosDurability::Volatile,
};
const REL_VOL: DiscoveredQos = DiscoveredQos {
    reliability: QosReliability::Reliable,
    durability: QosDurability::Volatile,
};
/// A LATCHED publisher — the near-static-map QoS shape (the Go2's
/// `/uslam/cloud_map` advertises TRANSIENT_LOCAL so late joiners get the last
/// map). Distinguishes a latched map from a live VOLATILE sensor stream.
const BE_TL: DiscoveredQos = DiscoveredQos {
    reliability: QosReliability::BestEffort,
    durability: QosDurability::TransientLocal,
};
/// An OMITTED SEDP durability param (by design): what CycloneDDS
/// (the Go2 itself) emits for ordinary default-QoS volatile writers, since
/// RTPS ParameterList encoding omits default-valued params. Ranks EQUAL to
/// Volatile (the DDS-spec default the absence means).
const BE_UNK: DiscoveredQos = DiscoveredQos {
    reliability: QosReliability::BestEffort,
    durability: QosDurability::Unknown,
};

fn iface() -> IpAddr {
    "192.168.123.18".parse().unwrap()
}

fn params() -> DiscoveryParams {
    DiscoveryParams {
        only_networks: vec![iface()],
        domain_id: 0,
        window: Duration::from_secs(5),
    }
}

fn opts(dry_run: bool, assume_yes: bool) -> RosAttachOptions {
    RosAttachOptions {
        iface: iface(),
        domain_id: 0,
        window: Duration::from_secs(5),
        dry_run,
        assume_yes,
        graph_name: "attach".to_string(),
        topic_prefix: None,
        robot_name: None,
    }
}

/// A confirm provider that must never be reached (proves the arm bypasses it).
fn panic_confirm(_preview: &str) -> CliResult<bool> {
    panic!("the confirm provider must not be invoked on this path")
}

/// Interactive confirm that ACCEPTS (the operator answered "yes").
fn accept_confirm(_preview: &str) -> CliResult<bool> {
    Ok(true)
}

/// Interactive confirm that DECLINES (the operator answered "no").
fn decline_confirm(_preview: &str) -> CliResult<bool> {
    Ok(false)
}

// ─────────────────────────── Report rendering ──────────────────────────────

/// The headline report oracle, with the raw route in place:
/// Imu — outside the 4-type registry but a built-in ROS 2 message —
/// resolves RAW and is marked "(bridged via the generic codec)"; the typed
/// PointCloud2 line carries "(bridged natively)"; a garbage type lands
/// UNRESOLVABLE with BOTH remediations (registry set + schema chain).
#[test]
fn discovery_report_exact_text_oracle() {
    let endpoints = vec![
        ep(
            "rt/utlidar/cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/imu_data",
            "sensor_msgs::msg::dds_::Imu_",
            EndpointKind::Writer,
            REL_VOL,
        ),
        ep(
            "rt/widget",
            "acme_msgs::msg::dds_::Widget_",
            EndpointKind::Writer,
            BE_VOL,
        ),
    ];
    let topics = aggregate_topics(&endpoints);
    let part = partition_topics_with(&AttachSchemaChain::builtins_only(), &topics);
    let (mappings, topic_conflicts) = build_mappings(&part.resolvable, None);
    assert!(topic_conflicts.is_empty());
    assert_eq!(mappings.len(), 2, "typed cloud + raw imu");
    let report = render_discovery_report(
        &params(),
        &topics,
        &part,
        &[],
        &[],
        &topic_conflicts,
        &[],
        0,
    );

    // HAND oracle (explicit literal, NOT derived from the renderer).
    let expected = concat!(
        "DISCOVERED DDS TOPICS (interface 192.168.123.18, domain 0)\n",
        "\n",
        "RESOLVABLE (2) — bridged onto the Cerulion wire:\n",
        "  /imu_data  (sensor_msgs/Imu)  writers=1 readers=0  qos=reliable/volatile  (bridged via the generic codec)\n",
        "  /utlidar/cloud  (sensor_msgs/PointCloud2)  writers=1 readers=0  qos=best_effort/volatile  (bridged natively)\n",
        "\n",
        "UNRESOLVABLE (1) — no Cerulion schema; NOT bridged:\n",
        "  /widget  (acme_msgs/Widget)  — 'acme_msgs/Widget' is neither a typed bridge mapping (sensor_msgs/PointCloud2, unitree_go/SportModeState, geometry_msgs/Twist, unitree_api/Request) nor a built-in ROS 2 message; add a schema for it or exclude the topic (raw DDS type: acme_msgs::msg::dds_::Widget_)\n",
        "\n",
        "Summary: 3 topic(s) discovered, 2 resolvable, 1 unresolvable.\n",
    );
    assert_eq!(report, expected);
}

#[test]
fn empty_discovery_report_is_a_loud_actionable_hint() {
    let topics = aggregate_topics(&[]);
    let part = partition_topics_with(&AttachSchemaChain::builtins_only(), &topics);
    let report = render_discovery_report(&params(), &topics, &part, &[], &[], &[], &[], 0);
    assert!(report.contains("no DDS topics discovered"), "{report}");
    assert!(report.contains("--iface"), "{report}");
    assert!(report.contains("ROS_DOMAIN_ID"), "{report}");
}

/// Two topics of ONE registry type (`geometry_msgs/Twist`, a BUILT-IN
/// type). The winner keeps the typed port; the loser RIDES THE GENERIC CODEC
/// (without the rescue it would be dropped, NOT bridged). Both durability and writer count
/// tie here, so the lexicographic tie-break picks `/cmd_vel` for the typed port —
/// and `/joy_vel` is RESCUED onto the generic codec instead of vanishing. Pinned
/// BYTE-EXACT: no PORT CONFLICTS block (nothing unbridgeable), the loser on a
/// RESOLVABLE line with the "riding the generic codec" marker, and the
/// `, N same-type sibling(s) on the generic codec` summary suffix.
#[test]
fn report_typed_port_loser_rides_generic_codec_byte_exact_oracle() {
    let endpoints = vec![
        ep(
            "rt/cmd_vel",
            "geometry_msgs::msg::dds_::Twist_",
            EndpointKind::Reader,
            BE_VOL,
        ),
        ep(
            "rt/joy_vel",
            "geometry_msgs::msg::dds_::Twist_",
            EndpointKind::Reader,
            BE_VOL,
        ),
    ];
    let topics = aggregate_topics(&endpoints);
    let mut part = partition_topics_with(&AttachSchemaChain::builtins_only(), &topics);
    // Assign the ONE typed port, then re-route the loser.
    let assignment = assign_typed_ports(&AttachSchemaChain::builtins_only(), part.resolvable);
    part.resolvable = assignment.resolvable;
    let (mappings, topic_conflicts) = build_mappings(&part.resolvable, None);
    // BOTH topics bridge: /cmd_vel typed, /joy_vel raw (the rescue).
    assert_eq!(mappings.len(), 2);
    assert_eq!(mappings[0].dds_topic, "/cmd_vel");
    assert!(matches!(mappings[0].route, BridgeRoute::Typed(_)));
    assert_eq!(mappings[1].dds_topic, "/joy_vel");
    assert!(matches!(mappings[1].route, BridgeRoute::Raw));
    assert!(
        assignment.unbridgeable.is_empty(),
        "nothing is unbridgeable"
    );
    assert_eq!(
        assignment.raw_routed,
        vec![RawRoutedLoser {
            ros_topic: "/joy_vel".to_string(),
            ros_type: "geometry_msgs/Twist".to_string(),
            typed_port_kept_by: "/cmd_vel".to_string(),
        }]
    );
    assert!(topic_conflicts.is_empty());

    let report = render_discovery_report(
        &params(),
        &topics,
        &part,
        &assignment.unbridgeable,
        &assignment.raw_routed,
        &topic_conflicts,
        &[],
        0,
    );
    // HAND oracle (explicit literal, NOT derived from the renderer).
    let expected = concat!(
        "DISCOVERED DDS TOPICS (interface 192.168.123.18, domain 0)\n",
        "\n",
        "RESOLVABLE (2) — bridged onto the Cerulion wire:\n",
        "  /cmd_vel  (geometry_msgs/Twist)  writers=0 readers=1  qos=best_effort/volatile  (bridged natively)\n",
        "  /joy_vel  (geometry_msgs/Twist)  writers=0 readers=1  qos=best_effort/volatile  (bridged via the generic codec; shares the geometry_msgs/Twist typed port with /cmd_vel — riding the generic codec until per-instance typed ports land)\n",
        "\n",
        "UNRESOLVABLE (0) — no Cerulion schema; NOT bridged:\n",
        "  (none)\n",
        "\n",
        "Summary: 2 topic(s) discovered, 2 resolvable, 0 unresolvable, 1 same-type sibling(s) on the generic codec.\n",
    );
    assert_eq!(report, expected);
}

// ───────────────────── Typed-port assignment (Go2) ──────────────────

/// The Go2 SLAM shape: with SLAM up, `/uslam/cloud_map` (a latched
/// near-static map — TRANSIENT_LOCAL) and `/utlidar/cloud` (the live ~20Hz lidar
/// — VOLATILE) both discover as `sensor_msgs/PointCloud2`. An alphabetical
/// assignment would give `/uslam/cloud_map` the ONE typed port and the live lidar
/// would be DROPPED — zero pointclouds on the wire. The STREAMING preference
/// (VOLATILE beats latched TRANSIENT_LOCAL) keeps the live lidar on the typed
/// port and the map RIDES THE GENERIC CODEC. Both bridged; byte-exact report.
#[test]
fn go2_slam_shape_streaming_preference_keeps_the_live_lidar() {
    let endpoints = vec![
        ep(
            "rt/uslam/cloud_map",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_TL,
        ),
        ep(
            "rt/utlidar/cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        ),
    ];
    let topics = aggregate_topics(&endpoints);
    let mut part = partition_topics_with(&AttachSchemaChain::builtins_only(), &topics);
    let assignment = assign_typed_ports(&AttachSchemaChain::builtins_only(), part.resolvable);
    // The LIVE lidar keeps the typed port; the latched map is raw-routed (NOT
    // dropped) — the exact inversion of the live failure.
    assert_eq!(
        assignment.raw_routed,
        vec![RawRoutedLoser {
            ros_topic: "/uslam/cloud_map".to_string(),
            ros_type: "sensor_msgs/PointCloud2".to_string(),
            typed_port_kept_by: "/utlidar/cloud".to_string(),
        }]
    );
    assert!(assignment.unbridgeable.is_empty());
    part.resolvable = assignment.resolvable;
    let (mappings, topic_conflicts) = build_mappings(&part.resolvable, None);
    assert!(topic_conflicts.is_empty());
    // sorted by ros_topic: /uslam/cloud_map (raw), /utlidar/cloud (typed).
    assert_eq!(mappings.len(), 2);
    assert_eq!(mappings[0].dds_topic, "/uslam/cloud_map");
    assert!(matches!(mappings[0].route, BridgeRoute::Raw));
    assert_eq!(mappings[1].dds_topic, "/utlidar/cloud");
    assert!(matches!(mappings[1].route, BridgeRoute::Typed(_)));

    let report = render_discovery_report(
        &params(),
        &topics,
        &part,
        &assignment.unbridgeable,
        &assignment.raw_routed,
        &topic_conflicts,
        &[],
        0,
    );
    let expected = concat!(
        "DISCOVERED DDS TOPICS (interface 192.168.123.18, domain 0)\n",
        "\n",
        "RESOLVABLE (2) — bridged onto the Cerulion wire:\n",
        "  /uslam/cloud_map  (sensor_msgs/PointCloud2)  writers=1 readers=0  qos=best_effort/transient_local  (bridged via the generic codec; shares the sensor_msgs/PointCloud2 typed port with /utlidar/cloud — riding the generic codec until per-instance typed ports land)\n",
        "  /utlidar/cloud  (sensor_msgs/PointCloud2)  writers=1 readers=0  qos=best_effort/volatile  (bridged natively)\n",
        "\n",
        "UNRESOLVABLE (0) — no Cerulion schema; NOT bridged:\n",
        "  (none)\n",
        "\n",
        "Summary: 2 topic(s) discovered, 2 resolvable, 0 unresolvable, 1 same-type sibling(s) on the generic codec.\n",
    );
    assert_eq!(report, expected);
}

/// The discriminator pin: if BOTH pointclouds were VOLATILE (durability
/// no longer discriminates), the lexicographic tie-break ALONE keeps
/// `/uslam/cloud_map` (it sorts first) on the typed port — the WRONG outcome (the
/// static map, not the live lidar). This proves the DURABILITY key is
/// LOAD-BEARING: it is what flips the Go2 case, since writer counts tie (1 each),
/// and the tie-break alone cannot save it. What WOULD save it without durability
/// is a per-topic observed-rate/liveliness signal — but
/// `DiscoveredEndpoint`/`DiscoveryResult` carry none (only writer/reader counts +
/// QoS), so durability is the strongest live-vs-latched signal available.
#[test]
fn tie_break_alone_cannot_save_go2_without_the_durability_key() {
    // BOTH volatile: durability ties, writers tie, so lexicographic decides.
    let endpoints = vec![
        ep(
            "rt/uslam/cloud_map",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/utlidar/cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        ),
    ];
    let topics = aggregate_topics(&endpoints);
    let part = partition_topics_with(&AttachSchemaChain::builtins_only(), &topics);
    let assignment = assign_typed_ports(&AttachSchemaChain::builtins_only(), part.resolvable);
    // Lexicographic tie-break keeps /uslam/cloud_map (sorts first) — the WRONG
    // topic when the real signal (the durability the live case carries) is gone.
    assert_eq!(
        assignment.raw_routed,
        vec![RawRoutedLoser {
            ros_topic: "/utlidar/cloud".to_string(),
            ros_type: "sensor_msgs/PointCloud2".to_string(),
            typed_port_kept_by: "/uslam/cloud_map".to_string(),
        }]
    );
    // Both still BRIDGED — the rescue holds regardless of who wins.
    assert!(assignment.unbridgeable.is_empty());
    assert_eq!(assignment.resolvable.len(), 2);
}

/// Typed-port determinism (a genuinely discriminating pin, never a
/// self-compare): a MIXED-DURABILITY
/// MULTI-WRITER topic must rank order-independently, and `assign_typed_ports`
/// must not depend on its input Vec's order either. Shape (by design):
/// `/uslam/cloud_map` has TWO writers — the TRANSIENT_LOCAL map server AND a
/// volatile relay republisher; the writer-set MIN rank makes it rank 0 (any
/// volatile writer ⇒ live stream) REGARDLESS of which writer's SEDP record
/// enumerates first (a first-writer-wins representative QoS would flip
/// the winner with endpoint order). Ranks then TIE with the 1-writer volatile
/// `/utlidar/cloud`, so WRITER COUNT decides: cloud_map keeps the typed port —
/// the HAND oracle every permutation is checked against (never a
/// self-compare). Every leg also runs on a REVERSED `resolvable` Vec
/// (aggregate-BYPASSING — the upstream sort in `aggregate_topics` otherwise
/// feeds both legs identical input, which would turn the leg into a self-compare).
#[test]
fn typed_port_assignment_is_order_independent_incl_mixed_durability_writers() {
    let tl_map = ep(
        "rt/uslam/cloud_map",
        "sensor_msgs::msg::dds_::PointCloud2_",
        EndpointKind::Writer,
        BE_TL,
    );
    let vol_relay = ep(
        "rt/uslam/cloud_map",
        "sensor_msgs::msg::dds_::PointCloud2_",
        EndpointKind::Writer,
        BE_VOL,
    );
    let lidar = ep(
        "rt/utlidar/cloud",
        "sensor_msgs::msg::dds_::PointCloud2_",
        EndpointKind::Writer,
        BE_VOL,
    );
    // All 6 endpoint permutations BEFORE aggregation — the min-fold must be
    // commutative (kills the first-writer-wins representative-QoS artifact).
    let perms: Vec<Vec<DiscoveredEndpoint>> = vec![
        vec![tl_map.clone(), vol_relay.clone(), lidar.clone()],
        vec![tl_map.clone(), lidar.clone(), vol_relay.clone()],
        vec![vol_relay.clone(), tl_map.clone(), lidar.clone()],
        vec![vol_relay.clone(), lidar.clone(), tl_map.clone()],
        vec![lidar.clone(), tl_map.clone(), vol_relay.clone()],
        vec![lidar, vol_relay, tl_map],
    ];
    let oracle_losers = vec![RawRoutedLoser {
        ros_topic: "/utlidar/cloud".to_string(),
        ros_type: "sensor_msgs/PointCloud2".to_string(),
        typed_port_kept_by: "/uslam/cloud_map".to_string(),
    }];
    for (i, endpoints) in perms.iter().enumerate() {
        let part = partition_topics_with(
            &AttachSchemaChain::builtins_only(),
            &aggregate_topics(endpoints),
        );
        // Aggregate-BYPASSING input permutations: the sorted Vec AND its
        // reverse must yield the identical assignment.
        let mut reversed_input = part.resolvable.clone();
        reversed_input.reverse();
        for (leg, input) in [part.resolvable.clone(), reversed_input]
            .into_iter()
            .enumerate()
        {
            let a = assign_typed_ports(&AttachSchemaChain::builtins_only(), input);
            assert_eq!(
                a.raw_routed, oracle_losers,
                "perm {i} leg {leg}: the 2-writer mixed-durability cloud_map must keep the \
                 typed port (min-rank 0 ties, writer count decides)"
            );
            assert!(a.unbridgeable.is_empty(), "perm {i} leg {leg}");
            assert_eq!(a.resolvable.len(), 2, "perm {i} leg {leg}");
            assert_eq!(a.resolvable[0].topic.ros_topic, "/uslam/cloud_map");
            assert!(
                matches!(a.resolvable[0].route, BridgeRoute::Typed(_)),
                "perm {i} leg {leg}: winner keeps the typed route"
            );
            assert_eq!(a.resolvable[1].topic.ros_topic, "/utlidar/cloud");
            assert!(
                matches!(a.resolvable[1].route, BridgeRoute::Raw),
                "perm {i} leg {leg}: loser flipped to the generic codec"
            );
        }
    }
}

/// A WRITER-LESS sibling must never
/// steal the typed port from the type's only data-bearing topic.
/// `/utlidar/cloud_deskewed` has 0 writers + 1 VOLATILE-REQUESTING reader (a
/// viz node subscribed to a producer that never started); `/uslam/cloud_map`
/// has the type's ONLY writer (TRANSIENT_LOCAL). Ranked on durability alone the reader's
/// REQUESTED durability would out-rank the writer's offered one and the typed
/// cloud port would bind to a topic that will NEVER carry data. Writer-presence
/// FIRST: the data-bearing topic wins before durability is consulted; the
/// reader-only sibling still bridges (an idle raw mapping, harmless).
#[test]
fn writer_less_sibling_never_steals_the_typed_port() {
    let endpoints = vec![
        ep(
            "rt/utlidar/cloud_deskewed",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Reader,
            BE_VOL,
        ),
        ep(
            "rt/uslam/cloud_map",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_TL,
        ),
    ];
    let topics = aggregate_topics(&endpoints);
    let part = partition_topics_with(&AttachSchemaChain::builtins_only(), &topics);
    let assignment = assign_typed_ports(&AttachSchemaChain::builtins_only(), part.resolvable);
    assert_eq!(
        assignment.raw_routed,
        vec![RawRoutedLoser {
            ros_topic: "/utlidar/cloud_deskewed".to_string(),
            ros_type: "sensor_msgs/PointCloud2".to_string(),
            typed_port_kept_by: "/uslam/cloud_map".to_string(),
        }],
        "the type's only data-bearing topic keeps the typed port"
    );
    assert!(assignment.unbridgeable.is_empty());
}

/// By design, `Unknown` durability ranks EQUAL to
/// `Volatile`. RTPS ParameterList encoding OMITS default-valued QoS params and
/// the DDS-spec default durability IS VOLATILE — CycloneDDS (the Go2 itself)
/// surfaces its ordinary live writers as `Unknown` while an explicitly-
/// serializing stack surfaces `Volatile`. Ranking Unknown below Volatile would
/// hand the typed port to an idle explicit-volatile sibling on vendor
/// serialization habit; equal ranks let WRITER COUNT decide — the 2-writer
/// Unknown-durability topic wins despite sorting last.
#[test]
fn unknown_durability_ranks_equal_to_volatile_the_dds_default() {
    let endpoints = vec![
        ep(
            "rt/a_vol_cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/z_unk_cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_UNK,
        ),
        ep(
            "rt/z_unk_cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_UNK,
        ),
    ];
    let topics = aggregate_topics(&endpoints);
    let part = partition_topics_with(&AttachSchemaChain::builtins_only(), &topics);
    let assignment = assign_typed_ports(&AttachSchemaChain::builtins_only(), part.resolvable);
    // Unknown == Volatile (rank 0 both) ⇒ the durability key TIES and writer
    // count decides. A rank split would hand the port to /a_vol_cloud on key 2
    // before writer count was ever consulted.
    assert_eq!(
        assignment.raw_routed,
        vec![RawRoutedLoser {
            ros_topic: "/a_vol_cloud".to_string(),
            ros_type: "sensor_msgs/PointCloud2".to_string(),
            typed_port_kept_by: "/z_unk_cloud".to_string(),
        }]
    );
    assert!(assignment.unbridgeable.is_empty());
}

/// With durability tied, MORE writers win the typed port (key #2 —
/// writer count beats the lexicographic tie-break). `/z_cloud` (2 writers) keeps
/// the port over the lexicographically-smaller single-writer `/a_cloud`.
#[test]
fn more_writers_win_the_typed_port_when_durability_ties() {
    let endpoints = vec![
        ep(
            "rt/a_cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/z_cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/z_cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        ),
    ];
    let topics = aggregate_topics(&endpoints);
    let part = partition_topics_with(&AttachSchemaChain::builtins_only(), &topics);
    let assignment = assign_typed_ports(&AttachSchemaChain::builtins_only(), part.resolvable);
    // /z_cloud (2 writers) keeps the port DESPITE sorting last; /a_cloud raw-routed.
    assert_eq!(
        assignment.raw_routed,
        vec![RawRoutedLoser {
            ros_topic: "/a_cloud".to_string(),
            ros_type: "sensor_msgs/PointCloud2".to_string(),
            typed_port_kept_by: "/z_cloud".to_string(),
        }]
    );
    assert!(assignment.unbridgeable.is_empty());
}

/// `unitree_go/SportModeState`
/// and `unitree_api/Request` are NOT built-ins and (here) have no workspace
/// store — but the BRIDGE's generic codec carries them (its embedded
/// `UNITREE_MSGS` set), so typed-port losers of these types RAW-ROUTE like any
/// other. A `chain.resolves()` gate alone would drop them with the
/// factually false "the generic codec has nothing to decode it with" — on a
/// Go2 that is ~19 `unitree_api/Request` topics per attach, silently
/// killing the sibling rescue for 2 of the 4 registry types.
#[test]
fn unitree_registry_losers_raw_route_via_the_bridges_own_codec() {
    let endpoints = vec![
        ep(
            "rt/a_state",
            "unitree_go::msg::dds_::SportModeState_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/b_state",
            "unitree_go::msg::dds_::SportModeState_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/api/robot_state/request",
            "unitree_api::msg::dds_::Request_",
            EndpointKind::Writer,
            REL_VOL,
        ),
        ep(
            "rt/api/sport/request",
            "unitree_api::msg::dds_::Request_",
            EndpointKind::Writer,
            REL_VOL,
        ),
    ];
    let topics = aggregate_topics(&endpoints);
    let mut part = partition_topics_with(&AttachSchemaChain::builtins_only(), &topics);
    let assignment = assign_typed_ports(&AttachSchemaChain::builtins_only(), part.resolvable);
    // NOTHING is unbridgeable — the bridge codec decodes all four registry
    // types unconditionally (the pin this test provides).
    assert!(
        assignment.unbridgeable.is_empty(),
        "unitree registry losers must raw-route, not drop: {:?}",
        assignment.unbridgeable
    );
    // Lexicographic winners (all-volatile single-writer ties): the losers,
    // sorted by ros_topic.
    assert_eq!(
        assignment.raw_routed,
        vec![
            RawRoutedLoser {
                ros_topic: "/api/sport/request".to_string(),
                ros_type: "unitree_api/Request".to_string(),
                typed_port_kept_by: "/api/robot_state/request".to_string(),
            },
            RawRoutedLoser {
                ros_topic: "/b_state".to_string(),
                ros_type: "unitree_go/SportModeState".to_string(),
                typed_port_kept_by: "/a_state".to_string(),
            },
        ]
    );
    part.resolvable = assignment.resolvable;
    // ALL FOUR topics bridge; the two losers carry `route: raw` in the config
    // (registry types on the raw path need the bridge override).
    let (mappings, topic_conflicts) = build_mappings(&part.resolvable, None);
    assert!(topic_conflicts.is_empty());
    assert_eq!(mappings.len(), 4);
    let config = generate_bridge_config_with_store(0, &[iface()], &mappings, &[]);
    assert!(
        config.contains(
            "  - dds_topic: /api/sport/request\n    ros_type: unitree_api/Request\n    \
             cerulion_topic: /api/sport/request\n    qos: best_effort\n    route: raw\n"
        ),
        "{config}"
    );
    assert!(
        config.contains(
            "  - dds_topic: /b_state\n    ros_type: unitree_go/SportModeState\n    \
             cerulion_topic: /b_state\n    qos: best_effort\n    route: raw\n"
        ),
        "{config}"
    );
}

/// The ONE genuine unbridgeable arm — a workspace store schema that
/// SHADOWS the bridge's own entry (store wins at the bridge) with an
/// INCOMPLETE nested closure. The loser stays NOT bridged, loudly, and the
/// PORT CONFLICTS remediation names the MISSING nested member (a fixed
/// "drop the type's .msg" suffix would tell the operator to re-drop
/// a file already in the store). Byte-exact report pin.
#[test]
fn incomplete_store_shadow_keeps_loser_not_bridged_with_precise_remediation() {
    let tmp = tempfile::tempdir().unwrap();
    // A store SportModeState whose nested dep resolves NOWHERE (not in store,
    // builtins, or the bridge's UNITREE set).
    write_store_msg(
        tmp.path(),
        "unitree_go",
        "SportModeState",
        "unitree_go/GhostDep dep\n",
    );
    let chain = AttachSchemaChain::from_workspace(tmp.path());
    let endpoints = vec![
        ep(
            "rt/a_state",
            "unitree_go::msg::dds_::SportModeState_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/b_state",
            "unitree_go::msg::dds_::SportModeState_",
            EndpointKind::Writer,
            BE_VOL,
        ),
    ];
    let topics = aggregate_topics(&endpoints);
    let mut part = partition_topics_with(&chain, &topics);
    assert_eq!(
        part.resolvable.len(),
        2,
        "typed routes bypass the store gap"
    );
    let assignment = assign_typed_ports(&chain, part.resolvable);
    assert!(assignment.raw_routed.is_empty());
    let expected_reason = "the workspace .msg store schema for unitree_go/SportModeState \
                           SHADOWS the bridge's own schema but its nested closure is INCOMPLETE \
                           (missing unitree_go/GhostDep) — the bridge would accept the mapping \
                           and then fail every frame at CDR decode. Add the missing .msg(s) to \
                           schemas/<pkg>/msg/, or remove the incomplete store schema so the \
                           bridge's embedded schema serves the type";
    assert_eq!(
        assignment.unbridgeable,
        vec![DroppedDuplicate {
            dds_topic: "/b_state".to_string(),
            ros_type: "unitree_go/SportModeState".to_string(),
            kept_dds_topic: "/a_state".to_string(),
            raw_fallback_reason: expected_reason.to_string(),
        }]
    );
    part.resolvable = assignment.resolvable;
    let (mappings, topic_conflicts) = build_mappings(&part.resolvable, None);
    assert_eq!(mappings.len(), 1);
    assert_eq!(mappings[0].dds_topic, "/a_state");
    assert!(topic_conflicts.is_empty());

    let report = render_discovery_report(
        &params(),
        &topics,
        &part,
        &assignment.unbridgeable,
        &assignment.raw_routed,
        &topic_conflicts,
        &[],
        0,
    );
    let expected = concat!(
        "DISCOVERED DDS TOPICS (interface 192.168.123.18, domain 0)\n",
        "\n",
        "RESOLVABLE (1) — bridged onto the Cerulion wire:\n",
        "  /a_state  (unitree_go/SportModeState)  writers=1 readers=0  qos=best_effort/volatile  (bridged natively)\n",
        "\n",
        "UNRESOLVABLE (0) — no Cerulion schema; NOT bridged:\n",
        "  (none)\n",
        "\n",
        "PORT CONFLICTS (1) — bridge v1 binds each type to ONE port; these share a type with a kept topic and could not be raw-routed, so they are NOT bridged:\n",
        "  /b_state  (unitree_go/SportModeState)  — typed port taken by /a_state; the workspace .msg store schema for unitree_go/SportModeState SHADOWS the bridge's own schema but its nested closure is INCOMPLETE (missing unitree_go/GhostDep) — the bridge would accept the mapping and then fail every frame at CDR decode. Add the missing .msg(s) to schemas/<pkg>/msg/, or remove the incomplete store schema so the bridge's embedded schema serves the type (per-instance typed ports are the eventual home)\n",
        "\n",
        "Summary: 2 topic(s) discovered, 1 resolvable, 0 unresolvable, 1 port-conflict(s) dropped.\n",
    );
    assert_eq!(report, expected);
}

/// A store shadow whose only "missing" nested dep is BRIDGE-EMBEDDED
/// (`unitree_go/IMUState` — absent from the store AND the builtins but shipped
/// in the bridge's `UNITREE_MSGS`) is COMPLETE for the bridge: the gate unions
/// those names into the closure walk, so the loser still raw-routes (no false
/// drop — the CLI-side mirror of the bridge's actual nested-ref universe).
#[test]
fn store_shadow_with_bridge_embedded_nested_dep_still_raw_routes() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(
        tmp.path(),
        "unitree_go",
        "SportModeState",
        "unitree_go/IMUState imu\n",
    );
    let chain = AttachSchemaChain::from_workspace(tmp.path());
    let endpoints = vec![
        ep(
            "rt/a_state",
            "unitree_go::msg::dds_::SportModeState_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/b_state",
            "unitree_go::msg::dds_::SportModeState_",
            EndpointKind::Writer,
            BE_VOL,
        ),
    ];
    let topics = aggregate_topics(&endpoints);
    let part = partition_topics_with(&chain, &topics);
    let assignment = assign_typed_ports(&chain, part.resolvable);
    assert!(
        assignment.unbridgeable.is_empty(),
        "a bridge-embedded nested dep is not a closure gap: {:?}",
        assignment.unbridgeable
    );
    assert_eq!(
        assignment.raw_routed,
        vec![RawRoutedLoser {
            ros_topic: "/b_state".to_string(),
            ros_type: "unitree_go/SportModeState".to_string(),
            typed_port_kept_by: "/a_state".to_string(),
        }]
    );
}

/// The reader-QoS discriminator: a fold that hoisted the
/// reader's durability out of the WRITER-only arm would flip the winner. Topic
/// A = 1 TransientLocal WRITER + 1 Volatile-REQUESTING reader; topic B = 2
/// TransientLocal WRITERS. The CORRECT writer-only fold ranks both at 1 (TL) so
/// the durability key TIES and writer count decides → B (2 writers) keeps the
/// typed port, A raw-routes. A CONTAMINATED fold folds A's Volatile READER into
/// A's rank (→ 0), so A out-ranks B on durability and WRONGLY wins. Order is
/// permuted to prove the fold is order-independent.
#[test]
fn reader_durability_never_ranks_only_writers_decide_the_typed_port() {
    let a_writer = ep(
        "rt/a_cloud",
        "sensor_msgs::msg::dds_::PointCloud2_",
        EndpointKind::Writer,
        BE_TL,
    );
    let a_reader = ep(
        "rt/a_cloud",
        "sensor_msgs::msg::dds_::PointCloud2_",
        EndpointKind::Reader,
        BE_VOL,
    );
    let b_writer1 = ep(
        "rt/b_cloud",
        "sensor_msgs::msg::dds_::PointCloud2_",
        EndpointKind::Writer,
        BE_TL,
    );
    let b_writer2 = ep(
        "rt/b_cloud",
        "sensor_msgs::msg::dds_::PointCloud2_",
        EndpointKind::Writer,
        BE_TL,
    );
    // HAND oracle: B (2 TL writers) keeps the typed port; A (1 TL writer, ignore
    // the Volatile reader) raw-routes.
    let oracle = vec![RawRoutedLoser {
        ros_topic: "/a_cloud".to_string(),
        ros_type: "sensor_msgs/PointCloud2".to_string(),
        typed_port_kept_by: "/b_cloud".to_string(),
    }];
    for order in [
        vec![
            a_writer.clone(),
            a_reader.clone(),
            b_writer1.clone(),
            b_writer2.clone(),
        ],
        vec![
            b_writer2.clone(),
            a_reader.clone(),
            b_writer1.clone(),
            a_writer.clone(),
        ],
        vec![
            a_reader.clone(),
            b_writer1.clone(),
            a_writer.clone(),
            b_writer2.clone(),
        ],
    ] {
        let topics = aggregate_topics(&order);
        let part = partition_topics_with(&AttachSchemaChain::builtins_only(), &topics);
        let assignment = assign_typed_ports(&AttachSchemaChain::builtins_only(), part.resolvable);
        assert_eq!(
            assignment.raw_routed, oracle,
            "B (2 TL writers) must keep the typed port and A raw-route — the reader's Volatile \
             request must NOT lower A's rank below B (order {order:?})"
        );
        assert!(assignment.unbridgeable.is_empty());
    }
}

/// The mirror of
/// [`unknown_durability_ranks_equal_to_volatile_the_dds_default`]: a 2-writer
/// VOLATILE topic that sorts LAST vs a 1-writer UNKNOWN topic that sorts FIRST.
/// Correct EQUAL ranks (Volatile == Unknown == 0) ⇒ the durability key TIES and
/// writer count decides ⇒ the Volatile topic (2 writers) keeps the typed port
/// DESPITE sorting last; the Unknown topic raw-routes. A rank swap that made
/// Volatile rank 1 (latched) while Unknown stayed 0 would hand the port to the
/// single-writer Unknown topic on rank — this oracle catches it. Complements the
/// one-direction pin where the UNKNOWN topic carried the extra writer.
#[test]
fn volatile_equals_unknown_from_the_other_direction_writer_count_decides() {
    let endpoints = vec![
        ep(
            "rt/a_unk_cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_UNK,
        ),
        ep(
            "rt/z_vol_cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/z_vol_cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        ),
    ];
    let topics = aggregate_topics(&endpoints);
    let part = partition_topics_with(&AttachSchemaChain::builtins_only(), &topics);
    let assignment = assign_typed_ports(&AttachSchemaChain::builtins_only(), part.resolvable);
    // Vol == Unknown (rank 0 both) ⇒ writer count decides ⇒ the 2-writer Volatile
    // /z_vol_cloud keeps the port DESPITE sorting last; the Unknown sibling raw-routes.
    assert_eq!(
        assignment.raw_routed,
        vec![RawRoutedLoser {
            ros_topic: "/a_unk_cloud".to_string(),
            ros_type: "sensor_msgs/PointCloud2".to_string(),
            typed_port_kept_by: "/z_vol_cloud".to_string(),
        }]
    );
    assert!(assignment.unbridgeable.is_empty());
}

/// ALL same-type siblings are writer-LESS (readers only).
/// Every topic has writer_count 0 (writer-presence ties) and
/// writer_durability_rank `u8::MAX` (the durability key ties — `u8::MAX` is
/// present in the key but never FLIPS a decision), so the lexicographic
/// tie-break ALONE decides: the smallest ros_topic keeps the typed port, the
/// rest raw-route. Pins the "`u8::MAX` never consulted" claim.
#[test]
fn all_writer_less_siblings_pick_the_lexicographic_winner() {
    let endpoints = vec![
        ep(
            "rt/z_cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Reader,
            BE_VOL,
        ),
        ep(
            "rt/a_cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Reader,
            BE_VOL,
        ),
        ep(
            "rt/m_cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Reader,
            BE_VOL,
        ),
    ];
    let topics = aggregate_topics(&endpoints);
    let part = partition_topics_with(&AttachSchemaChain::builtins_only(), &topics);
    let assignment = assign_typed_ports(&AttachSchemaChain::builtins_only(), part.resolvable);
    // /a_cloud (lexicographically first) keeps the port; /m_cloud, /z_cloud raw-route.
    assert_eq!(
        assignment.raw_routed,
        vec![
            RawRoutedLoser {
                ros_topic: "/m_cloud".to_string(),
                ros_type: "sensor_msgs/PointCloud2".to_string(),
                typed_port_kept_by: "/a_cloud".to_string(),
            },
            RawRoutedLoser {
                ros_topic: "/z_cloud".to_string(),
                ros_type: "sensor_msgs/PointCloud2".to_string(),
                typed_port_kept_by: "/a_cloud".to_string(),
            },
        ]
    );
    assert!(assignment.unbridgeable.is_empty());
}

/// Transitively-INCOMPLETE store shadow: Root → Mid (in
/// store) → Leaf (MISSING nowhere). A direct-only walk would see Root's only
/// dep (Mid) resolve and declare it COMPLETE (route: raw would be written and
/// the bridge would fail every frame at CDR decode); the transitive walk
/// descends into Mid and NAMES the missing grandchild in the remediation.
#[test]
#[tracing_test::traced_test]
fn transitively_incomplete_store_shadow_is_blocked_and_names_the_grandchild() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "acme_msgs", "Root", "acme_msgs/Mid m\n");
    write_store_msg(tmp.path(), "acme_msgs", "Mid", "acme_msgs/Leaf l\n");
    let chain = AttachSchemaChain::from_workspace(tmp.path());
    assert!(chain.resolves("acme_msgs/Root"));
    // Direct walk: the transitive missing grandchild is NAMED (hand oracle).
    assert_eq!(
        chain.store_closure_missing("acme_msgs/Root"),
        Some(vec!["acme_msgs/Leaf".to_string()]),
        "the transitive walk must descend into the store-resolved dep and name the gap"
    );

    // The partition flips it UNRESOLVABLE, loudly.
    let topics = aggregate_topics(&[ep(
        "rt/root",
        "acme_msgs::msg::dds_::Root_",
        EndpointKind::Writer,
        BE_VOL,
    )]);
    let part = partition_topics_with(&chain, &topics);
    assert!(part.resolvable.is_empty(), "{part:?}");
    assert_eq!(part.unresolvable.len(), 1);
    assert!(
        logs_contain("nested closure is INCOMPLETE"),
        "the transitive flip must be loud"
    );

    // e2e: the attach report names the transitive missing member under UNRESOLVABLE.
    let mut confirm = panic_confirm;
    let report = ros_cmd::ros_attach(
        &FakeDiscovery::ok(vec![ep(
            "rt/root",
            "acme_msgs::msg::dds_::Root_",
            EndpointKind::Writer,
            BE_VOL,
        )]),
        tmp.path(),
        &opts(true, false),
        true,
        &mut confirm,
    )
    .expect("dry run");
    assert!(report.mappings.is_empty(), "{}", report.report);
    assert!(report.report.contains("INCOMPLETE"), "{}", report.report);
    assert!(
        report.report.contains("acme_msgs/Leaf"),
        "the transitive grandchild must be named:\n{}",
        report.report
    );
}

/// Transitively-COMPLETE two-level store shadow: Root →
/// Mid → Leaf, ALL in store, Leaf primitive-only ⇒ the whole closure resolves
/// and the type raw-routes fine (the anti-tautology control for the transitive
/// blocker: a two-level closure is not falsely flagged).
#[test]
fn transitively_complete_two_level_store_shadow_raw_routes() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "acme_msgs", "Root", "acme_msgs/Mid m\n");
    write_store_msg(tmp.path(), "acme_msgs", "Mid", "acme_msgs/Leaf l\n");
    write_store_msg(tmp.path(), "acme_msgs", "Leaf", "int32 v\n");
    let chain = AttachSchemaChain::from_workspace(tmp.path());
    assert_eq!(
        chain.store_closure_missing("acme_msgs/Root"),
        None,
        "a fully-resolvable two-level closure is COMPLETE"
    );
    let topics = aggregate_topics(&[ep(
        "rt/root",
        "acme_msgs::msg::dds_::Root_",
        EndpointKind::Writer,
        BE_VOL,
    )]);
    let part = partition_topics_with(&chain, &topics);
    assert_eq!(part.resolvable.len(), 1, "{part:?}");
    assert!(matches!(part.resolvable[0].route, BridgeRoute::Raw));
}

/// CYCLE guard: A refs B, B refs A, both in store. The
/// `visited` set breaks the cycle so the walk TERMINATES; every ref resolves ⇒
/// COMPLETE (no infinite loop, no false gap).
#[test]
fn cyclic_store_shadow_terminates_and_is_complete() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "acme_msgs", "A", "acme_msgs/B b\n");
    write_store_msg(tmp.path(), "acme_msgs", "B", "acme_msgs/A a\n");
    let chain = AttachSchemaChain::from_workspace(tmp.path());
    assert_eq!(
        chain.store_closure_missing("acme_msgs/A"),
        None,
        "a cycle terminates as COMPLETE"
    );
    assert_eq!(chain.store_closure_missing("acme_msgs/B"), None);
}

/// The generated bridge CONFIG carries BOTH the typed winner AND the
/// raw-routed loser — the loser as a `route: raw` mapping so the bridge sends it
/// through the SAME generic codec (never contending for the fixed typed port).
/// A registry type routed raw is the ONLY case that needs the override; a
/// non-registry raw mapping (Imu) emits none (the mapping carries no `route:` key).
#[test]
fn config_emits_route_raw_only_for_the_registry_type_loser() {
    let endpoints = vec![
        ep(
            "rt/uslam/cloud_map",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_TL,
        ),
        ep(
            "rt/utlidar/cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        // A non-registry raw sibling — proves it needs NO override.
        ep(
            "rt/imu_data",
            "sensor_msgs::msg::dds_::Imu_",
            EndpointKind::Writer,
            REL_VOL,
        ),
    ];
    let topics = aggregate_topics(&endpoints);
    let mut part = partition_topics_with(&AttachSchemaChain::builtins_only(), &topics);
    let assignment = assign_typed_ports(&AttachSchemaChain::builtins_only(), part.resolvable);
    part.resolvable = assignment.resolvable;
    let (mappings, _) = build_mappings(&part.resolvable, None);
    let config = generate_bridge_config_with_store(0, &[iface()], &mappings, &[]);
    // Registry-type loser: a raw mapping WITH the override (rides the generic codec).
    assert!(
        config.contains(
            "  - dds_topic: /uslam/cloud_map\n    ros_type: sensor_msgs/PointCloud2\n    \
             cerulion_topic: /uslam/cloud_map\n    qos: best_effort\n    route: raw\n"
        ),
        "the registry-type loser must carry `route: raw`:\n{config}"
    );
    // Non-registry raw (Imu): NO override — its qos line is immediately followed
    // by the NEXT mapping (proves no `route:` line for it; sorted order is
    // /imu_data, /uslam/cloud_map, /utlidar/cloud).
    assert!(
        config.contains(
            "    cerulion_topic: /imu_data\n    qos: best_effort\n  - dds_topic: /uslam/cloud_map\n"
        ),
        "a non-registry raw mapping must have no route override:\n{config}"
    );
    // Typed winner: NO override — it is the LAST mapping and ends the file with
    // its qos line (no trailing `route:`).
    assert!(
        config.ends_with("    cerulion_topic: /utlidar/cloud\n    qos: best_effort\n"),
        "the typed winner must not be forced raw:\n{config}"
    );
}

// ──────────── Typed-port production wiring (e2e through ros_attach) ────────────

/// The no-inert-shipping pin: the
/// typed-port race through the PRODUCTION entry point (`ros_cmd::ros_attach` →
/// `ros_attach_with_acquirer`), not a hand-composed pure-fn pipeline. Two
/// same-type topics (the Go2 SLAM shape) → the WRITTEN bridge config
/// carries the typed winner WITHOUT `route:` and the loser WITH `route: raw`
/// (byte-exact file), the graph gives the typed cloud port to the winner, and
/// the report carries the rescue wording. A passthrough mutation at the
/// `assign_typed_ports` call site (returning `clean_resolvable` unassigned)
/// writes TWO Auto PointCloud2 mappings — this test's byte oracle catches it.
/// The workspace has NO vendored dds_bridge: the RouteMode probe's ABSENT-file
/// arm must default to SUPPORTS (the operator copies the CURRENT node type
/// from the repo, which speaks `route:`).
#[test]
fn attach_e2e_same_type_siblings_write_typed_winner_plus_route_raw_loser() {
    let tmp = tempfile::tempdir().unwrap();
    let disc = FakeDiscovery::ok(vec![
        ep(
            "rt/uslam/cloud_map",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_TL,
        ),
        ep(
            "rt/utlidar/cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        ),
    ]);
    let mut confirm = panic_confirm; // --yes never consults confirm
    let report = ros_cmd::ros_attach(&disc, tmp.path(), &opts(false, true), false, &mut confirm)
        .expect("--yes attach writes");
    assert!(matches!(report.outcome, AttachOutcome::Written { .. }));

    // The WRITTEN config, byte-exact: loser first (sorted), `route: raw`;
    // typed winner second, no route line.
    let config = read(&tmp.path().join("graphs").join("attach.bridge.yaml"));
    let expected_config = concat!(
        "# Generated by `cerulion ros2 attach`.\n",
        "# DDS->Cerulion mapping config for the `dds_bridge` node, consumed via the\n",
        "# DDS_BRIDGE_CONFIG env var. Typed mappings ride the bridge's fixed ports\n",
        "# (cerulion_topic is documentation; the graph's `topic:` override is\n",
        "# authoritative). Raw mappings ride the generic codec: cerulion_topic\n",
        "# is AUTHORITATIVE, and max_slice_len defaults to 1 MiB (set it per mapping\n",
        "# for larger frames). qos is best_effort (matches both reliable and\n",
        "# best-effort publishers). Edit freely.\n",
        "domain_id: 0\n",
        "only_networks:\n",
        "  - 192.168.123.18\n",
        "mappings:\n",
        "  - dds_topic: /uslam/cloud_map\n",
        "    ros_type: sensor_msgs/PointCloud2\n",
        "    cerulion_topic: /uslam/cloud_map\n",
        "    qos: best_effort\n",
        "    route: raw\n",
        "  - dds_topic: /utlidar/cloud\n",
        "    ros_type: sensor_msgs/PointCloud2\n",
        "    cerulion_topic: /utlidar/cloud\n",
        "    qos: best_effort\n",
    );
    assert_eq!(config, expected_config);

    // The WRITTEN graph: the typed cloud port belongs to the WINNER; the
    // raw-routed loser has NO graph port (its cerulion_topic is authoritative
    // in the config).
    let graph = read(&tmp.path().join("graphs").join("attach.yaml"));
    assert!(graph.contains("        topic: /utlidar/cloud\n"), "{graph}");
    assert!(
        !graph.contains("/uslam/cloud_map"),
        "the raw-routed loser must not claim a graph port:\n{graph}"
    );

    // The report: the rescue wording (never port-taken/NOT-bridged) + summary.
    assert!(
        report.report.contains(
            "  /uslam/cloud_map  (sensor_msgs/PointCloud2)  writers=1 readers=0  \
             qos=best_effort/transient_local  (bridged via the generic codec; shares the \
             sensor_msgs/PointCloud2 typed port with /utlidar/cloud — riding the generic \
             codec until per-instance typed ports land)\n"
        ),
        "{}",
        report.report
    );
    assert!(
        report.report.contains(
            "Summary: 2 topic(s) discovered, 2 resolvable, 0 unresolvable, 1 same-type \
             sibling(s) on the generic codec.\n"
        ),
        "{}",
        report.report
    );
    assert!(
        !report.report.contains("PORT CONFLICTS"),
        "nothing is unbridgeable here:\n{}",
        report.report
    );
}

/// Helper: plant a VENDORED `dds_bridge` node source at
/// `<root>/nodes/dds_bridge/src/config.rs` with the given content — the
/// RouteMode probe's target file.
fn write_vendored_bridge_config_rs(root: &Path, content: &str) {
    let dir = root.join("nodes").join("dds_bridge").join("src");
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join("config.rs"), content).unwrap();
}

/// Arm 1 of 2: an UPGRADED workspace — the vendored
/// `dds_bridge` source declares `RouteMode` — gets `route: raw` in the written
/// config (the probe's positive arm; the absent-file default is pinned by
/// `attach_e2e_same_type_siblings_write_typed_winner_plus_route_raw_loser`).
#[test]
fn attach_e2e_route_mode_bearing_vendored_bridge_gets_route_raw() {
    let tmp = tempfile::tempdir().unwrap();
    write_vendored_bridge_config_rs(
        tmp.path(),
        "// newer vendored copy: declares RouteMode\npub enum RouteMode { Auto, Raw }\n",
    );
    let disc = FakeDiscovery::ok(vec![
        ep(
            "rt/uslam/cloud_map",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_TL,
        ),
        ep(
            "rt/utlidar/cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        ),
    ]);
    let mut confirm = panic_confirm;
    let report = ros_cmd::ros_attach(&disc, tmp.path(), &opts(false, true), false, &mut confirm)
        .expect("--yes attach writes");
    assert!(matches!(report.outcome, AttachOutcome::Written { .. }));
    let config = read(&tmp.path().join("graphs").join("attach.bridge.yaml"));
    assert!(
        config.contains("    route: raw\n"),
        "a RouteMode-bearing vendored bridge takes route: raw:\n{config}"
    );
    assert!(
        !report.report.contains("does not support `route: raw`"),
        "no degrade note on an upgraded workspace:\n{}",
        report.report
    );
}

/// Arm 2 of 2, the doomed-run guard: an OLDER vendored `dds_bridge`
/// (its `config.rs` has no
/// `RouteMode`) would reject the WHOLE generated config on the unknown `route`
/// key (`deny_unknown_fields`) at graph run — ZERO topics bridged, strictly
/// worse than before. The attach seam must instead OMIT `route: raw` and fall
/// back to the pre-rework drop: the winner still bridges (byte-exact config, no
/// `route:` anywhere), the loser lands in PORT CONFLICTS with the
/// vendored-bridge reason, and the report carries the preflight note naming
/// the incompatibility + the re-copy remediation.
#[test]
fn attach_e2e_old_vendored_bridge_degrades_loudly_never_writes_route_raw() {
    let tmp = tempfile::tempdir().unwrap();
    write_vendored_bridge_config_rs(
        tmp.path(),
        "// older vendored copy: deny_unknown_fields TopicMapping, no route field\n\
         pub struct TopicMapping { pub dds_topic: String }\n",
    );
    let disc = FakeDiscovery::ok(vec![
        ep(
            "rt/uslam/cloud_map",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_TL,
        ),
        ep(
            "rt/utlidar/cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        ),
    ]);
    let mut confirm = panic_confirm;
    let report = ros_cmd::ros_attach(&disc, tmp.path(), &opts(false, true), false, &mut confirm)
        .expect("--yes attach writes");
    assert!(matches!(report.outcome, AttachOutcome::Written { .. }));

    // The WRITTEN config: ONLY the typed winner, no `route:` key anywhere —
    // byte-exact (the old bridge parses this fine; the pre-rework shape).
    let config = read(&tmp.path().join("graphs").join("attach.bridge.yaml"));
    let expected_config = concat!(
        "# Generated by `cerulion ros2 attach`.\n",
        "# DDS->Cerulion mapping config for the `dds_bridge` node, consumed via the\n",
        "# DDS_BRIDGE_CONFIG env var. Typed mappings ride the bridge's fixed ports\n",
        "# (cerulion_topic is documentation; the graph's `topic:` override is\n",
        "# authoritative). Raw mappings ride the generic codec: cerulion_topic\n",
        "# is AUTHORITATIVE, and max_slice_len defaults to 1 MiB (set it per mapping\n",
        "# for larger frames). qos is best_effort (matches both reliable and\n",
        "# best-effort publishers). Edit freely.\n",
        "domain_id: 0\n",
        "only_networks:\n",
        "  - 192.168.123.18\n",
        "mappings:\n",
        "  - dds_topic: /utlidar/cloud\n",
        "    ros_type: sensor_msgs/PointCloud2\n",
        "    cerulion_topic: /utlidar/cloud\n",
        "    qos: best_effort\n",
    );
    assert_eq!(config, expected_config);
    assert!(!config.contains("route:"), "{config}");

    // The report: the loser in PORT CONFLICTS with the vendored-bridge reason
    // (the exact line), the preflight note, and NO rescue wording.
    assert!(
        report.report.contains(
            "  /uslam/cloud_map  (sensor_msgs/PointCloud2)  — typed port taken by \
             /utlidar/cloud; this workspace's vendored dds_bridge node does not support \
             `route: raw` and would reject the whole generated config; re-copy nodes/dds_bridge \
             from the Cerulion repo (examples/go2/nodes/dds_bridge/), then re-run attach to \
             bridge it via the generic codec (per-instance typed ports are the eventual home)\n"
        ),
        "{}",
        report.report
    );
    assert!(
        report
            .report
            .contains("note: this workspace's vendored `dds_bridge` node does not support"),
        "{}",
        report.report
    );
    assert!(
        report.report.contains(
            "Summary: 2 topic(s) discovered, 1 resolvable, 0 unresolvable, 1 port-conflict(s) \
             dropped.\n"
        ),
        "{}",
        report.report
    );
    assert!(
        !report.report.contains("riding the generic codec"),
        "no rescue wording when the loser was degraded:\n{}",
        report.report
    );
}

/// The degrade genuine-raw pass-through: an OLD vendored
/// bridge (no RouteMode) degrades the PointCloud2 typed-port LOSER out of the
/// config, but a GENUINE raw topic (sensor_msgs/Imu — non-registry, always
/// raw-bridged, never a typed-port contender) MUST survive the degrade
/// untouched, alongside the typed winner. Pins that
/// `degrade_raw_routed_for_old_bridge` strips ONLY the flipped registry-type
/// losers, never a genuine raw mapping.
#[test]
fn attach_e2e_old_bridge_degrade_leaves_genuine_raw_imu_untouched() {
    let tmp = tempfile::tempdir().unwrap();
    write_vendored_bridge_config_rs(
        tmp.path(),
        "// older vendored copy: deny_unknown_fields TopicMapping, no route field\n\
         pub struct TopicMapping { pub dds_topic: String }\n",
    );
    let disc = FakeDiscovery::ok(vec![
        ep(
            "rt/imu_data",
            "sensor_msgs::msg::dds_::Imu_",
            EndpointKind::Writer,
            REL_VOL,
        ),
        ep(
            "rt/uslam/cloud_map",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_TL,
        ),
        ep(
            "rt/utlidar/cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        ),
    ]);
    let mut confirm = panic_confirm;
    let report = ros_cmd::ros_attach(&disc, tmp.path(), &opts(false, true), false, &mut confirm)
        .expect("--yes attach writes");
    assert!(matches!(report.outcome, AttachOutcome::Written { .. }));

    // Byte-exact config: the genuine-raw Imu (no route line) + the typed
    // PointCloud2 winner (no route line); the PointCloud2 LOSER is degraded out;
    // NO `route:` key anywhere (the old bridge parses this fine).
    let config = read(&tmp.path().join("graphs").join("attach.bridge.yaml"));
    let expected_config = concat!(
        "# Generated by `cerulion ros2 attach`.\n",
        "# DDS->Cerulion mapping config for the `dds_bridge` node, consumed via the\n",
        "# DDS_BRIDGE_CONFIG env var. Typed mappings ride the bridge's fixed ports\n",
        "# (cerulion_topic is documentation; the graph's `topic:` override is\n",
        "# authoritative). Raw mappings ride the generic codec: cerulion_topic\n",
        "# is AUTHORITATIVE, and max_slice_len defaults to 1 MiB (set it per mapping\n",
        "# for larger frames). qos is best_effort (matches both reliable and\n",
        "# best-effort publishers). Edit freely.\n",
        "domain_id: 0\n",
        "only_networks:\n",
        "  - 192.168.123.18\n",
        "mappings:\n",
        "  - dds_topic: /imu_data\n",
        "    ros_type: sensor_msgs/Imu\n",
        "    cerulion_topic: /imu_data\n",
        "    qos: best_effort\n",
        "  - dds_topic: /utlidar/cloud\n",
        "    ros_type: sensor_msgs/PointCloud2\n",
        "    cerulion_topic: /utlidar/cloud\n",
        "    qos: best_effort\n",
    );
    assert_eq!(config, expected_config);
    assert!(
        !config.contains("route:"),
        "old bridge must get no route key:\n{config}"
    );

    // The degraded LOSER lands in PORT CONFLICTS with the vendored-bridge reason.
    assert!(
        report.report.contains(
            "  /uslam/cloud_map  (sensor_msgs/PointCloud2)  — typed port taken by \
             /utlidar/cloud; this workspace's vendored dds_bridge node does not support \
             `route: raw` and would reject the whole generated config; re-copy nodes/dds_bridge \
             from the Cerulion repo (examples/go2/nodes/dds_bridge/), then re-run attach to \
             bridge it via the generic codec (per-instance typed ports are the eventual home)\n"
        ),
        "{}",
        report.report
    );
    assert!(
        report
            .report
            .contains("note: this workspace's vendored `dds_bridge` node does not support"),
        "{}",
        report.report
    );
    // The genuine-raw Imu survived into RESOLVABLE (bridged via the generic codec).
    assert!(
        report.report.contains("/imu_data  (sensor_msgs/Imu)"),
        "the genuine raw Imu must still be bridged:\n{}",
        report.report
    );
}

/// 3+ same-type siblings through the PRODUCTION attach
/// path → ONE typed winner (no route line) + exactly TWO `route: raw` losers in
/// the written config. The N-way (>2) e2e sibling of
/// [`attach_e2e_same_type_siblings_write_typed_winner_plus_route_raw_loser`].
#[test]
fn attach_e2e_three_same_type_siblings_write_one_winner_two_route_raw_losers() {
    let tmp = tempfile::tempdir().unwrap();
    let disc = FakeDiscovery::ok(vec![
        ep(
            "rt/a_cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/m_cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/z_cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        ),
    ]);
    let mut confirm = panic_confirm;
    let report = ros_cmd::ros_attach(&disc, tmp.path(), &opts(false, true), false, &mut confirm)
        .expect("--yes attach writes");
    assert!(matches!(report.outcome, AttachOutcome::Written { .. }));

    // Byte-exact config: /a_cloud (lexicographic winner, no route) + two
    // route: raw losers (/m_cloud, /z_cloud), sorted by ros_topic.
    let config = read(&tmp.path().join("graphs").join("attach.bridge.yaml"));
    let expected_config = concat!(
        "# Generated by `cerulion ros2 attach`.\n",
        "# DDS->Cerulion mapping config for the `dds_bridge` node, consumed via the\n",
        "# DDS_BRIDGE_CONFIG env var. Typed mappings ride the bridge's fixed ports\n",
        "# (cerulion_topic is documentation; the graph's `topic:` override is\n",
        "# authoritative). Raw mappings ride the generic codec: cerulion_topic\n",
        "# is AUTHORITATIVE, and max_slice_len defaults to 1 MiB (set it per mapping\n",
        "# for larger frames). qos is best_effort (matches both reliable and\n",
        "# best-effort publishers). Edit freely.\n",
        "domain_id: 0\n",
        "only_networks:\n",
        "  - 192.168.123.18\n",
        "mappings:\n",
        "  - dds_topic: /a_cloud\n",
        "    ros_type: sensor_msgs/PointCloud2\n",
        "    cerulion_topic: /a_cloud\n",
        "    qos: best_effort\n",
        "  - dds_topic: /m_cloud\n",
        "    ros_type: sensor_msgs/PointCloud2\n",
        "    cerulion_topic: /m_cloud\n",
        "    qos: best_effort\n",
        "    route: raw\n",
        "  - dds_topic: /z_cloud\n",
        "    ros_type: sensor_msgs/PointCloud2\n",
        "    cerulion_topic: /z_cloud\n",
        "    qos: best_effort\n",
        "    route: raw\n",
    );
    assert_eq!(config, expected_config);
    assert_eq!(
        config.matches("    route: raw\n").count(),
        2,
        "exactly two losers carry route: raw:\n{config}"
    );
    assert!(
        report.report.contains(
            "Summary: 3 topic(s) discovered, 3 resolvable, 0 unresolvable, 2 same-type \
             sibling(s) on the generic codec.\n"
        ),
        "{}",
        report.report
    );
}

/// An OLD vendored bridge (probe would say NOT-supported)
/// with NO typed-port losers produces output BYTE-IDENTICAL to the SAME run in a
/// no-vendored-bridge (SUPPORTS) workspace — same written config, no preflight
/// degrade note, no PORT CONFLICTS. Pins the degrade path INERT without losers
/// via OUTPUT EQUALITY. (Scope: the oracle proves output equality only
/// — it cannot distinguish "probe consulted and harmless" from "probe never
/// consulted", so it makes NO claim about the degrade gate's `&&`
/// short-circuit / probe-consultation order.)
#[test]
fn attach_e2e_old_bridge_without_losers_is_byte_identical_to_supports() {
    let disc = || {
        FakeDiscovery::ok(vec![ep(
            "rt/utlidar/cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        )])
    };

    // OLD vendored bridge (no RouteMode), single PointCloud2 ⇒ no losers.
    let tmp_old = tempfile::tempdir().unwrap();
    write_vendored_bridge_config_rs(
        tmp_old.path(),
        // NOTE: this content must NOT contain the token the probe greps for
        // (`vendored_bridge_supports_route_mode` does a raw `contains` over
        // the whole file, comments included), or the fixture silently models
        // a NEW bridge and every assertion below becomes vacuous.
        "// older vendored copy: declares no route enum\n\
         pub struct TopicMapping { pub dds_topic: String }\n",
    );
    let mut c1 = panic_confirm;
    let old = ros_cmd::ros_attach(&disc(), tmp_old.path(), &opts(false, true), false, &mut c1)
        .expect("--yes attach writes");

    // SUPPORTS (no vendored bridge at all).
    let tmp_new = tempfile::tempdir().unwrap();
    let mut c2 = panic_confirm;
    ros_cmd::ros_attach(&disc(), tmp_new.path(), &opts(false, true), false, &mut c2)
        .expect("--yes attach writes");

    let old_config = read(&tmp_old.path().join("graphs").join("attach.bridge.yaml"));
    let new_config = read(&tmp_new.path().join("graphs").join("attach.bridge.yaml"));
    assert_eq!(
        old_config, new_config,
        "no losers ⇒ old-bridge config == supports config"
    );

    // No preflight note, no PORT CONFLICTS in the output — byte-identical to
    // the supports case (the oracle proves output equality, not that the probe
    // was consulted; see the docstring above).
    assert!(
        !old.report.contains("does not support `route: raw`"),
        "no degrade note without losers:\n{}",
        old.report
    );
    assert!(
        !old.report.contains("PORT CONFLICTS"),
        "no port conflicts without losers:\n{}",
        old.report
    );
}

/// Raw DDS legally allows ONE topic name to
/// carry TWO types. If both survived `build_mappings` the generated
/// graph would declare two ports publishing the SAME `topic:` — a single-writer
/// collision at run. Exactly one is kept (the deterministic rule:
/// the input is sorted by `(ros_topic, ros_type)`, first wins ⇒ the
/// lexicographically-smallest type keeps the topic) and the drop is reported
/// LOUDLY with the dropped type + remediation — pinned byte-exact.
#[test]
fn same_topic_two_types_keeps_one_deterministically_and_reports_loudly() {
    let endpoints = vec![
        ep(
            "rt/cmd",
            "unitree_api::msg::dds_::Request_",
            EndpointKind::Writer,
            REL_VOL,
        ),
        ep(
            "rt/cmd",
            "geometry_msgs::msg::dds_::Twist_",
            EndpointKind::Writer,
            BE_VOL,
        ),
    ];
    let topics = aggregate_topics(&endpoints);
    let part = partition_topics_with(&AttachSchemaChain::builtins_only(), &topics);
    assert_eq!(part.resolvable.len(), 2, "both types resolve");

    // Two DIFFERENT registry types on ONE topic — no same-type typed-port race,
    // so the cerulion_topic dedup (pass 2) still keeps exactly one.
    let (mappings, topic_conflicts) = build_mappings(&part.resolvable, None);
    // Exactly ONE mapping survives — the lexicographically-smallest type.
    assert_eq!(mappings.len(), 1, "exactly one survives: {mappings:?}");
    assert_eq!(mappings[0].ros_type, "geometry_msgs/Twist");
    assert_eq!(mappings[0].cerulion_topic, "/cmd");
    assert_eq!(
        topic_conflicts,
        vec![DroppedTopicConflict {
            ros_topic: "/cmd".to_string(),
            ros_type: "unitree_api/Request".to_string(),
            kept_ros_type: "geometry_msgs/Twist".to_string(),
        }]
    );

    // The report names the drop loudly — byte-exact.
    let report = render_discovery_report(
        &params(),
        &topics,
        &part,
        &[],
        &[],
        &topic_conflicts,
        &[],
        0,
    );
    let expected = concat!(
        "DISCOVERED DDS TOPICS (interface 192.168.123.18, domain 0)\n",
        "\n",
        "RESOLVABLE (2) — bridged onto the Cerulion wire:\n",
        "  /cmd  (geometry_msgs/Twist)  writers=1 readers=0  qos=best_effort/volatile  (bridged natively)\n",
        "  /cmd  (unitree_api/Request)  writers=1 readers=0  qos=reliable/volatile  (bridged natively)\n",
        "\n",
        "UNRESOLVABLE (0) — no Cerulion schema; NOT bridged:\n",
        "  (none)\n",
        "\n",
        "TOPIC CONFLICTS (1) — one DDS topic name carries TWO types (legal in DDS); Cerulion topics are single-writer, so only the first type (lexicographically smallest) is bridged:\n",
        "  /cmd  — dropped type unitree_api/Request (topic kept by geometry_msgs/Twist); to bridge both, edit the generated config to give one mapping a distinct cerulion_topic\n",
        "\n",
        "Summary: 2 topic(s) discovered, 2 resolvable, 0 unresolvable, 1 topic-conflict(s) dropped.\n",
    );
    assert_eq!(report, expected);

    // The generated graph carries NO duplicate `topic:` value (without the
    // dedup the twist AND request_json ports would both declare `/cmd`).
    let graph = generate_bridge_graph("attach", "attach", &mappings);
    let topic_lines: Vec<&str> = graph
        .lines()
        .filter(|l| l.trim_start().starts_with("topic: "))
        .collect();
    let mut deduped = topic_lines.clone();
    deduped.sort();
    deduped.dedup();
    assert_eq!(
        topic_lines.len(),
        deduped.len(),
        "duplicate topic: values in the generated graph:\n{graph}"
    );
}

// ─────────────────────────── Aggregation ───────────────────────────────────

#[test]
fn aggregation_dedups_and_counts_and_prefers_writer_qos() {
    // Two readers (reliable) + one writer (best_effort) on ONE topic: folded to
    // one topic, counts 1 writer / 2 readers, QoS from the WRITER.
    let endpoints = vec![
        ep(
            "rt/state",
            "unitree_go::msg::dds_::SportModeState_",
            EndpointKind::Reader,
            REL_VOL,
        ),
        ep(
            "rt/state",
            "unitree_go::msg::dds_::SportModeState_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/state",
            "unitree_go::msg::dds_::SportModeState_",
            EndpointKind::Reader,
            REL_VOL,
        ),
    ];
    let topics = aggregate_topics(&endpoints);
    assert_eq!(topics.len(), 1);
    let t = &topics[0];
    assert_eq!(t.ros_topic, "/state");
    assert_eq!(t.ros_type, "unitree_go/SportModeState");
    assert_eq!(t.writer_count, 1);
    assert_eq!(t.reader_count, 2);
    assert_eq!(t.qos.reliability, QosReliability::BestEffort); // writer wins
}

#[test]
fn aggregation_is_sorted_deterministically() {
    let endpoints = vec![
        ep(
            "rt/zeta",
            "geometry_msgs::msg::dds_::Twist_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/alpha",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        ),
    ];
    let topics = aggregate_topics(&endpoints);
    assert_eq!(topics[0].ros_topic, "/alpha");
    assert_eq!(topics[1].ros_topic, "/zeta");
}

// ─────────────────────────── Partition predicate ───────────────────────────

/// Splits by the resolvability seam AND pins the ORDERING CONTRACT: the
/// partition preserves the aggregate's `(ros_topic, ros_type)` sort — stable
/// input order, never a re-sort by type or anything else. Sorted TOPIC order
/// here is `/cloud` < `/cmd` < `/imu` < `/tf` (byte-wise: `'l'` 0x6C < `'m'`
/// 0x6D puts `/cloud` before `/cmd`), so the resolvable list is
/// PointCloud2-first. (The final gate caught the original oracle asserting a
/// hand-mis-sorted `/cmd`-first order; the topic-order contract is what the
/// downstream first-wins dedup rules and the report rendering key off.)
#[test]
fn partition_splits_resolvable_from_unresolvable() {
    let endpoints = vec![
        ep(
            "rt/cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/imu",
            "sensor_msgs::msg::dds_::Imu_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/tf",
            "tf2_msgs::msg::dds_::TFMessage_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/cmd",
            "geometry_msgs::msg::dds_::Twist_",
            EndpointKind::Reader,
            BE_VOL,
        ),
        ep(
            "rt/widget",
            "acme_msgs::msg::dds_::Widget_",
            EndpointKind::Writer,
            BE_VOL,
        ),
    ];
    let topics = aggregate_topics(&endpoints);
    let part = partition_topics_with(&AttachSchemaChain::builtins_only(), &topics);
    // The full (topic, type, route-kind) triples — pins the split, the stable
    // sorted order (the contract), AND the raw-route flip: Imu and TFMessage
    // (built-in ROS 2 messages outside the 4-type registry) resolve RAW
    // instead of landing unresolvable; only a schema-less type is left out.
    let resolvable: Vec<(&str, &str, bool)> = part
        .resolvable
        .iter()
        .map(|r| {
            (
                r.topic.ros_topic.as_str(),
                r.topic.ros_type.as_str(),
                matches!(r.route, BridgeRoute::Typed(_)),
            )
        })
        .collect();
    let unresolvable: Vec<(&str, &str)> = part
        .unresolvable
        .iter()
        .map(|t| (t.ros_topic.as_str(), t.ros_type.as_str()))
        .collect();
    assert_eq!(
        resolvable,
        [
            ("/cloud", "sensor_msgs/PointCloud2", true), // typed
            ("/cmd", "geometry_msgs/Twist", true),       // typed
            ("/imu", "sensor_msgs/Imu", false),          // raw
            ("/tf", "tf2_msgs/TFMessage", false),        // raw
        ]
    );
    assert_eq!(
        unresolvable,
        [("/widget", "acme_msgs/Widget")],
        "only the schema-less type stays unresolvable"
    );
}

// ─────────────────────────── Config gen (byte oracle) ──────────────────────

/// A local mirror of `examples/go2/nodes/dds_bridge/src/config.rs::BridgeConfig`
/// (same field names + `deny_unknown_fields`) — proves the generated YAML is
/// structurally valid for the REAL bridge without depending on the isolated
/// demo crate.
#[derive(serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct BridgeConfigMirror {
    #[serde(default)]
    domain_id: u16,
    #[serde(default)]
    only_networks: Vec<IpAddr>,
    // Mirrors the bridge's new optional `.msg` store key so a
    // generated config carrying `msg_dirs:` still round-trips under
    // `deny_unknown_fields` (absent ⇒ None ⇒ a config without the key still parses).
    #[serde(default)]
    msg_dirs: Option<Vec<String>>,
    mappings: Vec<MappingMirror>,
}
#[derive(serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct MappingMirror {
    dds_topic: String,
    ros_type: String,
    cerulion_topic: String,
    #[serde(default)]
    qos: Option<String>,
    // Mirror the REAL TopicMapping's `route` +
    // `max_slice_len` so a `route: raw` config still round-trips under
    // deny_unknown_fields. `route` is Option<String> (absent ⇒ None ⇒ the
    // bridge's Auto default; `raw` ⇒ Some("raw")); `max_slice_len` is
    // Option<u32> — the EXACT type of the real bridge's TopicMapping field
    // (examples/go2/nodes/dds_bridge/src/config.rs), so a generated
    // config carrying an out-of-u32-range value fails THIS mirror parse the way
    // the real bridge would (absent ⇒ None ⇒ the 1 MiB raw default). Both fields
    // are read by `mapping_mirror_round_trips_route_and_max_slice_len` (an
    // unread mirror field is dead_code AND an unpinned
    // contract).
    #[serde(default)]
    route: Option<String>,
    #[serde(default)]
    max_slice_len: Option<u32>,
}

#[test]
fn config_yaml_byte_oracle_and_parses() {
    let endpoints = vec![ep(
        "rt/utlidar/cloud",
        "sensor_msgs::msg::dds_::PointCloud2_",
        EndpointKind::Writer,
        BE_VOL,
    )];
    let topics = aggregate_topics(&endpoints);
    let part = partition_topics_with(&AttachSchemaChain::builtins_only(), &topics);
    let (mappings, _) = build_mappings(&part.resolvable, None);
    let yaml = generate_bridge_config_with_store(0, &[iface()], &mappings, &[]);

    let expected = concat!(
        "# Generated by `cerulion ros2 attach`.\n",
        "# DDS->Cerulion mapping config for the `dds_bridge` node, consumed via the\n",
        "# DDS_BRIDGE_CONFIG env var. Typed mappings ride the bridge's fixed ports\n",
        "# (cerulion_topic is documentation; the graph's `topic:` override is\n",
        "# authoritative). Raw mappings ride the generic codec: cerulion_topic\n",
        "# is AUTHORITATIVE, and max_slice_len defaults to 1 MiB (set it per mapping\n",
        "# for larger frames). qos is best_effort (matches both reliable and\n",
        "# best-effort publishers). Edit freely.\n",
        "domain_id: 0\n",
        "only_networks:\n",
        "  - 192.168.123.18\n",
        "mappings:\n",
        "  - dds_topic: /utlidar/cloud\n",
        "    ros_type: sensor_msgs/PointCloud2\n",
        "    cerulion_topic: /utlidar/cloud\n",
        "    qos: best_effort\n",
    );
    assert_eq!(yaml, expected);

    // Structural cross-check: parses into the bridge's serde shape, EVERY
    // mirror field round-tripped (unread mirror fields
    // are dead_code AND an unpinned contract).
    let cfg: BridgeConfigMirror = serde_yaml::from_str(&yaml).expect("generated config parses");
    assert_eq!(cfg.domain_id, 0);
    assert_eq!(cfg.only_networks, vec![iface()]);
    assert_eq!(cfg.mappings.len(), 1);
    assert_eq!(cfg.mappings[0].dds_topic, "/utlidar/cloud");
    assert_eq!(cfg.mappings[0].ros_type, "sensor_msgs/PointCloud2");
    assert_eq!(cfg.mappings[0].cerulion_topic, "/utlidar/cloud");
    assert_eq!(cfg.mappings[0].qos.as_deref(), Some("best_effort"));
    // A typed mapping carries neither `route` nor `max_slice_len` (⇒ Auto / 1 MiB).
    assert_eq!(cfg.mappings[0].route, None);
    assert_eq!(cfg.mappings[0].max_slice_len, None);
}

/// The [`MappingMirror`] carries `route` + `max_slice_len`
/// in lockstep with the REAL bridge TopicMapping (both under
/// deny_unknown_fields). Feed the same-type-siblings config through the mirror:
/// the raw-routed LOSER parses `route: Some("raw")`, the typed WINNER
/// `route: None` (absent ⇒ the bridge's Auto default); neither sets
/// `max_slice_len` (⇒ the 1 MiB default). A pre-refresh mirror could not even
/// PARSE a `route: raw` config (deny_unknown_fields rejects the unknown key).
#[test]
fn mapping_mirror_round_trips_route_and_max_slice_len() {
    let endpoints = vec![
        ep(
            "rt/uslam/cloud_map",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_TL,
        ),
        ep(
            "rt/utlidar/cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        ),
    ];
    let topics = aggregate_topics(&endpoints);
    let mut part = partition_topics_with(&AttachSchemaChain::builtins_only(), &topics);
    let assignment = assign_typed_ports(&AttachSchemaChain::builtins_only(), part.resolvable);
    part.resolvable = assignment.resolvable;
    let (mappings, _) = build_mappings(&part.resolvable, None);
    let yaml = generate_bridge_config_with_store(0, &[iface()], &mappings, &[]);
    let cfg: BridgeConfigMirror =
        serde_yaml::from_str(&yaml).expect("a route-bearing config must parse under the mirror");
    assert_eq!(cfg.mappings.len(), 2);
    // Sorted by ros_topic: /uslam/cloud_map (raw loser) then /utlidar/cloud (typed winner).
    assert_eq!(cfg.mappings[0].dds_topic, "/uslam/cloud_map");
    assert_eq!(
        cfg.mappings[0].route.as_deref(),
        Some("raw"),
        "the raw-routed loser carries route: raw"
    );
    assert_eq!(
        cfg.mappings[0].max_slice_len, None,
        "no max_slice_len ⇒ the 1 MiB default"
    );
    assert_eq!(cfg.mappings[1].dds_topic, "/utlidar/cloud");
    assert_eq!(
        cfg.mappings[1].route, None,
        "the typed winner has no route key"
    );
    assert_eq!(cfg.mappings[1].max_slice_len, None);
}

#[test]
fn config_empty_only_networks_renders_inline_list() {
    let endpoints = vec![ep(
        "rt/cmd_vel",
        "geometry_msgs::msg::dds_::Twist_",
        EndpointKind::Reader,
        BE_VOL,
    )];
    let topics = aggregate_topics(&endpoints);
    let part = partition_topics_with(&AttachSchemaChain::builtins_only(), &topics);
    let (mappings, _) = build_mappings(&part.resolvable, None);
    let yaml = generate_bridge_config_with_store(7, &[], &mappings, &[]);
    assert!(yaml.contains("domain_id: 7\n"), "{yaml}");
    assert!(yaml.contains("only_networks: []\n"), "{yaml}");
    // Still valid for the bridge.
    let _cfg: BridgeConfigMirror = serde_yaml::from_str(&yaml).expect("parses");
}

#[test]
fn config_topic_prefix_namespaces_cerulion_topic() {
    let endpoints = vec![ep(
        "rt/utlidar/cloud",
        "sensor_msgs::msg::dds_::PointCloud2_",
        EndpointKind::Writer,
        BE_VOL,
    )];
    let topics = aggregate_topics(&endpoints);
    let part = partition_topics_with(&AttachSchemaChain::builtins_only(), &topics);
    let (mappings, _) = build_mappings(&part.resolvable, Some("/go2"));
    assert_eq!(mappings[0].cerulion_topic, "/go2/utlidar/cloud");
    let yaml = generate_bridge_config_with_store(0, &[iface()], &mappings, &[]);
    assert!(
        yaml.contains("cerulion_topic: /go2/utlidar/cloud\n"),
        "{yaml}"
    );
}

#[test]
fn config_with_store_emits_msg_dirs_block_byte_oracle() {
    // A NON-EMPTY store-dir slice emits the top-level
    // `msg_dirs:` block (comment + list) between only_networks and mappings.
    let endpoints = vec![ep(
        "rt/utlidar/cloud",
        "sensor_msgs::msg::dds_::PointCloud2_",
        EndpointKind::Writer,
        BE_VOL,
    )];
    let topics = aggregate_topics(&endpoints);
    let part = partition_topics_with(&AttachSchemaChain::builtins_only(), &topics);
    let (mappings, _) = build_mappings(&part.resolvable, None);
    // Pass the PRODUCTION const so the oracle tracks the value attach emits
    // (supplement A: `../schemas` — config-file-relative, the config lives in
    // graphs/ beside the workspace store).
    let yaml = generate_bridge_config_with_store(
        0,
        &[iface()],
        &mappings,
        &[cerulion_cli_engine::ros_cmd::BRIDGE_STORE_MSG_DIR],
    );

    let expected = concat!(
        "# Generated by `cerulion ros2 attach`.\n",
        "# DDS->Cerulion mapping config for the `dds_bridge` node, consumed via the\n",
        "# DDS_BRIDGE_CONFIG env var. Typed mappings ride the bridge's fixed ports\n",
        "# (cerulion_topic is documentation; the graph's `topic:` override is\n",
        "# authoritative). Raw mappings ride the generic codec: cerulion_topic\n",
        "# is AUTHORITATIVE, and max_slice_len defaults to 1 MiB (set it per mapping\n",
        "# for larger frames). qos is best_effort (matches both reliable and\n",
        "# best-effort publishers). Edit freely.\n",
        "domain_id: 0\n",
        "only_networks:\n",
        "  - 192.168.123.18\n",
        "# msg_dirs: workspace .msg schema store(s) the bridge's generic codec loads at\n",
        "# startup — each an ament-mirror `<dir>/<pkg>/msg/<Type>.msg` tree.\n",
        "# RELATIVE paths resolve against THIS FILE's directory (the bridge joins them to\n",
        "# the config path at load), so `../schemas` is the workspace store beside graphs/\n",
        "# and the graph runs from any working directory. A store schema wins over a\n",
        "# built-in of the same name (a store schema shadows a built-in).\n",
        "msg_dirs:\n",
        "  - ../schemas\n",
        "mappings:\n",
        "  - dds_topic: /utlidar/cloud\n",
        "    ros_type: sensor_msgs/PointCloud2\n",
        "    cerulion_topic: /utlidar/cloud\n",
        "    qos: best_effort\n",
    );
    assert_eq!(yaml, expected);

    // Round-trips into the bridge serde shape with msg_dirs present.
    let cfg: BridgeConfigMirror = serde_yaml::from_str(&yaml).expect("parses");
    assert_eq!(cfg.msg_dirs, Some(vec!["../schemas".to_string()]));
}

#[test]
fn config_without_store_omits_msg_dirs() {
    // Back-compat: an EMPTY store-dir slice emits NO msg_dirs key.
    let endpoints = vec![ep(
        "rt/utlidar/cloud",
        "sensor_msgs::msg::dds_::PointCloud2_",
        EndpointKind::Writer,
        BE_VOL,
    )];
    let topics = aggregate_topics(&endpoints);
    let part = partition_topics_with(&AttachSchemaChain::builtins_only(), &topics);
    let (mappings, _) = build_mappings(&part.resolvable, None);

    let with_empty = generate_bridge_config_with_store(0, &[iface()], &mappings, &[]);
    assert!(
        !with_empty.contains("msg_dirs"),
        "no msg_dirs key when the store slice is empty: {with_empty}"
    );
    let cfg: BridgeConfigMirror = serde_yaml::from_str(&with_empty).expect("parses");
    assert!(cfg.msg_dirs.is_none());
}

// ─────────────────────────── Graph gen (byte oracle) ───────────────────────

#[test]
fn graph_yaml_byte_oracle_and_parses_as_graphconfig() {
    // Only PointCloud2 mapped ⇒ cloud gets the real topic + max_slice_len; the
    // other three FIXED ports stay declared with silent default topics.
    let endpoints = vec![ep(
        "rt/utlidar/cloud",
        "sensor_msgs::msg::dds_::PointCloud2_",
        EndpointKind::Writer,
        BE_VOL,
    )];
    let topics = aggregate_topics(&endpoints);
    let part = partition_topics_with(&AttachSchemaChain::builtins_only(), &topics);
    let (mappings, _) = build_mappings(&part.resolvable, None);
    let yaml = generate_bridge_graph("attach", "attach", &mappings);

    let expected = concat!(
        "# Generated by `cerulion ros2 attach` — the lean one-node bridge graph.\n",
        "# Runs the `dds_bridge` node; its DDS mappings come from graphs/attach.bridge.yaml\n",
        "# (set DDS_BRIDGE_CONFIG to it). The four outputs are the bridge's FIXED port set —\n",
        "# unmapped ports stay silent. The `dds_bridge` node type must exist in this workspace.\n",
        "# To SEE this robot's data, run `cerulion viz --robot <name>` (or Cerulion Studio) on\n",
        "# YOUR machine — visualization is desk-side; the robot only ships raw frames.\n",
        "prefix: attach\n",
        "nodes:\n",
        "  - id: bridge\n",
        "    type: dds_bridge\n",
        "    outputs:\n",
        "      - name: cloud\n",
        "        schema: sensor_msgs/PointCloud2\n",
        "        topic: /utlidar/cloud\n",
        "        max_slice_len: 1048576\n",
        "      - name: odom\n",
        "        schema: nav_msgs/Odometry\n",
        "        topic: /dds_bridge/odom\n",
        "      - name: twist\n",
        "        schema: geometry_msgs/TwistStamped\n",
        "        topic: /dds_bridge/twist\n",
        "      - name: request_json\n",
        "        schema: std_msgs/String\n",
        "        topic: /dds_bridge/request_json\n",
    );
    assert_eq!(yaml, expected);

    // By design, the robot stages no visualization node:
    // the header names one node type and the body declares exactly that one.
    assert!(
        !yaml.contains("rerun_sink"),
        "the robot graph must stage no visualization node:\n{yaml}"
    );
    assert!(
        !yaml.contains("id: viz"),
        "the robot graph must declare no viz node:\n{yaml}"
    );

    // Structural cross-check against the REAL graph loader.
    let cfg: cerulion_core::graph::config::GraphConfig =
        serde_yaml::from_str(&yaml).expect("generated graph parses as GraphConfig");
    // A generated attach graph is named by its FILE
    // (`graphs/attach.yaml`), so it emits no `name:` key.
    assert!(
        cfg.name.is_none(),
        "`ros2 attach` must not emit a deprecated `name:` key"
    );
    assert_eq!(cfg.nodes.len(), 1);
    assert_eq!(cfg.nodes[0].node_type, "dds_bridge");
    assert_eq!(cfg.nodes[0].outputs.len(), 4);
    let cloud = &cfg.nodes[0].outputs[0];
    assert_eq!(cloud.name, "cloud");
    assert_eq!(cloud.topic.as_deref(), Some("/utlidar/cloud"));
    assert_eq!(cloud.max_slice_len, Some(1_048_576));
}

// ─────────────────────────────── `--viz` ───────────────────────────────────

/// A typed + two raw topics — the mixed set the viz node wires over. Sorted by
/// `(ros_topic, ros_type)` the mapping order is `/imu`, `/tf_static`,
/// `/utlidar/cloud`.
fn viz_multi_endpoints() -> Vec<DiscoveredEndpoint> {
    vec![
        ep(
            "rt/utlidar/cloud",
            "sensor_msgs::msg::dds_::PointCloud2_", // typed
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/imu",
            "sensor_msgs::msg::dds_::Imu_", // built-in → raw
            EndpointKind::Writer,
            REL_VOL,
        ),
        ep(
            "rt/tf_static",
            "tf2_msgs::msg::dds_::TFMessage_", // built-in → raw
            EndpointKind::Writer,
            BE_VOL,
        ),
    ]
}

/// THE inverted e2e pin: no emitted `rerun_sink` node, no
/// input-name convention for it, and no viz-inclusive write.
///
/// The workspace deliberately HAS `nodes/rerun_sink/`: that is the exact arm the
/// deleted loud-degrade guard treated as "sink available ⇒ stage it", so a
/// regression restoring robot-side staging would show up HERE first. The
/// discovery fixture is multi-topic for the same reason — the old path emitted
/// the node only when there was >= 1 mapping to visualize.
///
/// Asserts the ABSENCE structurally (no `rerun_sink` type, no `id: viz`, exactly
/// ONE node parsed back), that the written file equals the pure generator over
/// the same mappings, that the bridge ports are still really wired
/// (anti-tautology — this is a REAL bridge graph, not a trivially-empty one),
/// and that two runs are byte-identical (determinism, Principle #7).
#[test]
fn default_yes_writes_no_visualization_node_and_is_deterministic() {
    let run = || -> String {
        let tmp = tempfile::tempdir().unwrap();
        // The sink type IS present — staging must STILL not happen.
        add_rerun_sink_node_type(tmp.path());
        let disc = FakeDiscovery::ok(viz_multi_endpoints());
        let mut confirm = panic_confirm; // --yes never consults confirm
        let report = ros_cmd::ros_attach(
            &disc,
            tmp.path(),
            &opts(false, true), // write, non-interactive
            false,              // non-TTY (--yes governs)
            &mut confirm,
        )
        .expect("--yes writes");
        assert!(matches!(report.outcome, AttachOutcome::Written { .. }));
        let graph = read(&tmp.path().join("graphs").join("attach.yaml"));

        assert!(
            !graph.contains("rerun_sink"),
            "the robot graph must stage no rerun_sink even when the node type \
             exists in the workspace:\n{graph}"
        );
        assert!(
            !graph.contains("id: viz"),
            "the robot graph must declare no viz node:\n{graph}"
        );
        // Byte-identical to the pure generator over the SAME mappings.
        assert_eq!(
            graph,
            generate_bridge_graph("attach", &report.robot_identity, &report.mappings)
        );

        let cfg: cerulion_core::graph::config::GraphConfig =
            serde_yaml::from_str(&graph).expect("the written graph parses as GraphConfig");
        assert_eq!(
            cfg.nodes.len(),
            1,
            "an attach graph declares exactly one node (the bridge):\n{graph}"
        );
        assert_eq!(cfg.nodes[0].node_type, "dds_bridge");
        // Anti-tautology: the bridged topics really ARE wired, so the absence
        // above is not the absence of a graph.
        assert!(
            graph.contains("        topic: /utlidar/cloud\n"),
            "the typed mapping must still drive its port:\n{graph}"
        );
        // The committed shape must pass the validator `graph run` applies — the
        // doomed-run class the old loud-degrade guard existed to prevent.
        cerulion_core::graph::validation::validate_graph(&cfg)
            .expect("the generated graph must pass validate_graph");

        graph
    };
    assert_eq!(run(), run(), "two runs must be byte-identical");
}

// ─────────────────────── Robot identity (e2e report) ───────────────────────

/// The DERIVED `report.robot_identity` is real and correct across the
/// three derivation paths — a non-tautological hand-oracle check (the field was
/// added for exactly this). Drives the full attach flow (`--yes` into a
/// tempdir workspace; `--yes` never consults `confirm`) and asserts the
/// resulting identity — NOT the graph name "attach".
#[test]
fn report_robot_identity_reflects_derivation_across_paths() {
    let mut confirm = panic_confirm; // --yes path: confirm is never reached

    // (i) Every bridged ROS topic under ONE namespace ⇒ that namespace IS the
    //     robot identity. `/spot/cloud` (typed) + `/spot/imu` (raw) both bridge.
    let shared_ns = vec![
        ep(
            "rt/spot/cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/spot/imu",
            "sensor_msgs::msg::dds_::Imu_",
            EndpointKind::Writer,
            REL_VOL,
        ),
    ];
    let tmp = tempfile::tempdir().unwrap();
    let report = ros_cmd::ros_attach(
        &FakeDiscovery::ok(shared_ns),
        tmp.path(),
        &opts(false, true),
        false,
        &mut confirm,
    )
    .expect("shared-namespace attach writes");
    assert!(matches!(report.outcome, AttachOutcome::Written { .. }));
    assert_eq!(
        report.robot_identity, "spot",
        "shared /spot namespace ⇒ 'spot'"
    );

    // (ii) NO shared namespace (the Go2 scatter) ⇒ the hostname fallback — which
    //      is EXACTLY `default_prefix("")` (the same call the code uses, so the
    //      oracle is not machine-brittle) and NEVER the graph name "attach".
    let scattered = vec![
        ep(
            "rt/utlidar/cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/lf/imu",
            "sensor_msgs::msg::dds_::Imu_",
            EndpointKind::Writer,
            REL_VOL,
        ),
    ];
    let tmp2 = tempfile::tempdir().unwrap();
    let report = ros_cmd::ros_attach(
        &FakeDiscovery::ok(scattered),
        tmp2.path(),
        &opts(false, true),
        false,
        &mut confirm,
    )
    .expect("scattered attach writes");
    let hostname_fallback = cerulion_core::graph::default_prefix("");
    assert_eq!(
        report.robot_identity, hostname_fallback,
        "no shared namespace ⇒ the hostname fallback (default_prefix(\"\"))"
    );
    assert_ne!(
        report.robot_identity, "attach",
        "the identity is never the graph name"
    );

    // (iii) An explicit `--robot-name` override flows through to the report —
    //       overriding even a derivable namespace.
    let overridable = vec![ep(
        "rt/spot/cloud",
        "sensor_msgs::msg::dds_::PointCloud2_",
        EndpointKind::Writer,
        BE_VOL,
    )];
    let tmp3 = tempfile::tempdir().unwrap();
    let overridden = RosAttachOptions {
        robot_name: Some("go2-override".to_string()),
        ..opts(false, true)
    };
    let report = ros_cmd::ros_attach(
        &FakeDiscovery::ok(overridable),
        tmp3.path(),
        &overridden,
        false,
        &mut confirm,
    )
    .expect("override attach writes");
    assert_eq!(
        report.robot_identity, "go2-override",
        "--robot-name flows through to the report identity"
    );
}

/// The inverted viz-staging + loud-degrade-guard pins. The guard existed only
/// because viz was staged by default and could name a node type the workspace
/// lacked; with nothing staged there is nothing to degrade, so the contract is
/// now UNIFORM and SILENT: the consent preview names exactly one node type, and
/// the workspace's `nodes/rerun_sink/` presence changes NOTHING.
///
/// Both arms run the same discovery through the same write path and differ ONLY
/// in whether the sink type exists on disk, so a regression that reintroduces
/// any sink-presence-dependent behaviour (staging, a note, or a warn) fails
/// here regardless of which direction it leans.
#[test]
#[tracing_test::traced_test]
fn preview_names_only_dds_bridge_and_the_sink_type_changes_nothing() {
    let single = "(the `dds_bridge` node type must exist in this workspace.)";
    let mut confirm = panic_confirm; // --yes path; preview built, not shown

    // Sink type PRESENT (the shape the deleted guard staged the node on).
    let tmp = tempfile::tempdir().unwrap();
    add_rerun_sink_node_type(tmp.path());
    let present = ros_cmd::ros_attach(
        &FakeDiscovery::ok(viz_multi_endpoints()),
        tmp.path(),
        &opts(false, true),
        false,
        &mut confirm,
    )
    .expect("write with the sink type present");
    assert!(matches!(present.outcome, AttachOutcome::Written { .. }));

    // Sink type ABSENT (the shape the deleted guard fired the loud degrade on).
    let tmp2 = tempfile::tempdir().unwrap();
    let absent = ros_cmd::ros_attach(
        &FakeDiscovery::ok(viz_multi_endpoints()),
        tmp2.path(),
        &opts(false, true),
        false,
        &mut confirm,
    )
    .expect("write with the sink type absent");
    assert!(matches!(absent.outcome, AttachOutcome::Written { .. }));

    for r in [&present, &absent] {
        assert!(r.preview.contains(single), "{}", r.preview);
        assert!(
            !r.preview.contains("rerun_sink"),
            "the preview must never mention rerun_sink: {}",
            r.preview
        );
        assert!(
            !r.report.contains(VIZ_DEGRADE_MARKER),
            "there is nothing to degrade: {}",
            r.report
        );
        assert!(
            !r.preview.contains(VIZ_DEGRADE_MARKER),
            "there is nothing to degrade: {}",
            r.preview
        );
    }

    // The reports are byte-identical apart from nothing — sink presence is not
    // an input to attach at all (the sharpest form of the contract).
    assert_eq!(
        present.preview, absent.preview,
        "the workspace's rerun_sink node type must not influence attach"
    );

    // Both written graphs are the lean bridge graph.
    for (t, r) in [(&tmp, &present), (&tmp2, &absent)] {
        let graph = read(&t.path().join("graphs").join("attach.yaml"));
        assert!(!graph.contains("rerun_sink"), "{graph}");
        assert_eq!(
            graph,
            generate_bridge_graph("attach", &r.robot_identity, &r.mappings)
        );
    }

    // No warn on EITHER arm — the deleted guard's `warn!` must be gone, not
    // merely relocated.
    assert!(
        !logs_contain("has no `rerun_sink` node type"),
        "no sink-presence warn may fire on any arm"
    );

    // The dry-run "what would I get" view is equally silent.
    let tmp3 = tempfile::tempdir().unwrap();
    let dry = ros_cmd::ros_attach(
        &FakeDiscovery::ok(viz_multi_endpoints()),
        tmp3.path(),
        &opts(true, false),
        true,
        &mut confirm,
    )
    .expect("dry run");
    assert!(!dry.report.contains(VIZ_DEGRADE_MARKER), "{}", dry.report);
    assert!(!dry.report.contains("rerun_sink"), "{}", dry.report);
}

/// The inverted dry-run pins. `--dry-run` is the
/// "what would I get" view an operator decides on, so it must never advertise a
/// visualization node the write will not produce. Covers the three shapes the
/// old affirmative/degrade note pair split between: sink type present, sink type
/// absent, and nothing resolvable at all.
#[test]
fn dry_run_report_never_advertises_a_visualization_node() {
    let mut confirm = panic_confirm;

    let with_sink = tempfile::tempdir().unwrap();
    add_rerun_sink_node_type(with_sink.path());
    let without_sink = tempfile::tempdir().unwrap();

    for (label, dir, disc) in [
        (
            "sink type present",
            &with_sink,
            FakeDiscovery::ok(viz_multi_endpoints()),
        ),
        (
            "sink type absent",
            &without_sink,
            FakeDiscovery::ok(viz_multi_endpoints()),
        ),
    ] {
        let report = ros_cmd::ros_attach(&disc, dir.path(), &opts(true, false), true, &mut confirm)
            .expect("dry run");
        assert!(matches!(report.outcome, AttachOutcome::DryRun));
        // Anti-tautology: the run really did resolve topics, so the absence
        // below is not the absence of a report.
        assert!(
            report.report.contains("/utlidar/cloud"),
            "[{label}] the dry-run report must list the bridged topics: {}",
            report.report
        );
        assert!(
            !report.report.contains("rerun_sink"),
            "[{label}] the dry-run report must not mention rerun_sink: {}",
            report.report
        );
        assert!(
            !report.report.contains(VIZ_DEGRADE_MARKER),
            "[{label}] there is nothing to degrade: {}",
            report.report
        );
    }

    // Nothing resolvable: still note-free.
    let bare = tempfile::tempdir().unwrap();
    let report = ros_cmd::ros_attach(
        &FakeDiscovery::ok(vec![]),
        bare.path(),
        &opts(false, true),
        false,
        &mut confirm,
    )
    .expect("nothing-resolvable run");
    assert!(matches!(report.outcome, AttachOutcome::NothingResolvable));
    assert!(!report.report.contains("rerun_sink"), "{}", report.report);
    assert!(
        !report.report.contains(VIZ_DEGRADE_MARKER),
        "{}",
        report.report
    );
}

/// Supplement D: an INCOMPLETE store schema (a hand-dropped `.msg` whose nested
/// dependency is missing) is UNRESOLVABLE — not silently bridged into a mapping
/// that validates at bridge load and fails per-frame at CDR decode — and the
/// report names the missing dep. A COMPLETE store schema stays resolvable (the
/// control).
#[test]
#[tracing_test::traced_test]
fn incomplete_store_schema_is_unresolvable_with_missing_dep_named() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(
        tmp.path(),
        "acme_msgs",
        "Widget",
        "int32 id\ndep_pkg/DepType dep\n", // dep_pkg/DepType exists NOWHERE
    );
    let chain = AttachSchemaChain::from_workspace(tmp.path());
    // Name-presence still resolves; the completeness walk reports the gap.
    assert!(chain.resolves("acme_msgs/Widget"));
    let missing = chain
        .store_closure_missing("acme_msgs/Widget")
        .expect("incomplete store schema must report its gap");
    assert_eq!(missing, vec!["dep_pkg/DepType".to_string()]);

    // The partition flips it to UNRESOLVABLE, loudly.
    let topics = aggregate_topics(&[ep(
        "rt/widget",
        "acme_msgs::msg::dds_::Widget_",
        EndpointKind::Writer,
        BE_VOL,
    )]);
    let part = partition_topics_with(&chain, &topics);
    assert!(part.resolvable.is_empty(), "{part:?}");
    assert_eq!(part.unresolvable.len(), 1);
    assert!(
        logs_contain("nested closure is INCOMPLETE"),
        "the partition flip must be loud"
    );

    // e2e: the attach report names the missing dep under UNRESOLVABLE.
    let mut confirm = panic_confirm;
    let report = ros_cmd::ros_attach(
        &FakeDiscovery::ok(vec![ep(
            "rt/widget",
            "acme_msgs::msg::dds_::Widget_",
            EndpointKind::Writer,
            BE_VOL,
        )]),
        tmp.path(),
        &opts(true, false),
        true,
        &mut confirm,
    )
    .expect("dry run");
    assert!(report.mappings.is_empty(), "{}", report.report);
    assert!(report.report.contains("INCOMPLETE"), "{}", report.report);
    assert!(
        report.report.contains("dep_pkg/DepType"),
        "{}",
        report.report
    );

    // Control: the SAME schema with its dep present is resolvable again.
    let ok_ws = tempfile::tempdir().unwrap();
    write_store_msg(
        ok_ws.path(),
        "acme_msgs",
        "Widget",
        "int32 id\ndep_pkg/DepType dep\n",
    );
    write_store_msg(ok_ws.path(), "dep_pkg", "DepType", "int32 v\n");
    let ok_chain = AttachSchemaChain::from_workspace(ok_ws.path());
    assert!(ok_chain.store_closure_missing("acme_msgs/Widget").is_none());
    let ok_part = partition_topics_with(&ok_chain, &topics);
    assert_eq!(ok_part.resolvable.len(), 1, "{ok_part:?}");
}

// ─────────────────────────── Consent ladder ────────────────────────────────

fn read(path: &Path) -> String {
    std::fs::read_to_string(path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
}

fn cloud_only() -> FakeDiscovery {
    FakeDiscovery::ok(vec![ep(
        "rt/utlidar/cloud",
        "sensor_msgs::msg::dds_::PointCloud2_",
        EndpointKind::Writer,
        BE_VOL,
    )])
}

/// Materialize the `rerun_sink` node TYPE in a tempdir workspace. Removing the
/// robot-side staging made this INERT for `ros2 attach` (the robot stages no
/// visualization node whether or not the type exists), and it is kept precisely
/// so the staging pins can prove that: the sink-PRESENT arm is the shape the
/// deleted guard staged the node on.
///
/// The name is a pure STAND-IN: no real `cerulion_viz/nodes/rerun_sink` crate
/// exists (with nothing staging it, it would have no caller), so do not go
/// looking for it in-tree. What the pins need is only that
/// SOME node type by that name is discoverable in the workspace, because that is
/// the condition the deleted "sink available ⇒ stage it" guard keyed on.
fn add_rerun_sink_node_type(ws: &Path) {
    std::fs::create_dir_all(ws.join("nodes").join("rerun_sink")).unwrap();
}

/// The loud-degrade note's stable marker (the report/preview line asserted by
/// the guard tests and asserted ABSENT on the silent arms).
const VIZ_DEGRADE_MARKER: &str = "note: visualization skipped";

#[test]
fn dry_run_writes_nothing_and_never_confirms() {
    let tmp = tempfile::tempdir().unwrap();
    let mut confirm = panic_confirm;
    let report = ros_cmd::ros_attach(
        &cloud_only(),
        tmp.path(),
        &opts(true, true),
        true,
        &mut confirm,
    )
    .expect("dry run");
    assert_eq!(report.outcome, AttachOutcome::DryRun);
    assert!(report.graph_to_run.is_none());
    assert!(!tmp.path().join("graphs").join("attach.yaml").exists());
    assert!(!tmp
        .path()
        .join("graphs")
        .join("attach.bridge.yaml")
        .exists());
}

#[test]
fn assume_yes_writes_both_files_and_hands_off_to_run() {
    let tmp = tempfile::tempdir().unwrap();
    // The sink type is present but is never staged — kept so this
    // test exercises the same workspace shape as the staging pins.
    add_rerun_sink_node_type(tmp.path());
    let mut confirm = panic_confirm; // --yes never consults confirm
    let report = ros_cmd::ros_attach(
        &cloud_only(),
        tmp.path(),
        &opts(false, true),
        false,
        &mut confirm,
    )
    .expect("assume-yes write");
    assert!(matches!(report.outcome, AttachOutcome::Written { .. }));
    assert_eq!(report.graph_to_run.as_deref(), Some("attach"));
    let graph = tmp.path().join("graphs").join("attach.yaml");
    let config = tmp.path().join("graphs").join("attach.bridge.yaml");
    // Files exist AND their bytes equal the generator's output (the report
    // carries the same mappings the files were written from).
    assert_eq!(
        read(&graph),
        generate_bridge_graph("attach", &report.robot_identity, &report.mappings)
    );
    assert_eq!(
        read(&config),
        generate_bridge_config_with_store(0, &[iface()], &report.mappings, &[])
    );
}

/// The confirm-yes arm ALSO content-checks the preview
/// — the EXACT string the user approves must carry the discovery report, BOTH
/// generated file bodies verbatim, and the `graph run` hand-off line (without
/// this, nothing pins the preview's content, and an empty/garbled preview passes).
#[test]
fn interactive_confirm_yes_writes() {
    let tmp = tempfile::tempdir().unwrap();
    // The sink type is present but is never staged — kept so this
    // test exercises the same workspace shape as the staging pins.
    add_rerun_sink_node_type(tmp.path());
    let mut captured = String::new();
    let mut confirm = |p: &str| -> CliResult<bool> {
        captured.push_str(p);
        Ok(true)
    };
    let report = ros_cmd::ros_attach(
        &cloud_only(),
        tmp.path(),
        &opts(false, false),
        true,
        &mut confirm,
    )
    .expect("confirmed write");
    assert!(matches!(report.outcome, AttachOutcome::Written { .. }));
    assert!(report.preview_shown);
    assert!(tmp.path().join("graphs").join("attach.yaml").exists());

    // The captured preview is what the report exposes (one string, verbatim).
    assert_eq!(captured, report.preview);
    // ... and it carries the full consent contract:
    assert!(
        captured.contains("DISCOVERED DDS TOPICS"),
        "preview must embed the discovery report:\n{captured}"
    );
    let graph_body = generate_bridge_graph("attach", &report.robot_identity, &report.mappings);
    let config_body = generate_bridge_config_with_store(0, &[iface()], &report.mappings, &[]);
    assert!(
        captured.contains(&graph_body),
        "preview must embed the generated graph body verbatim:\n{captured}"
    );
    assert!(
        captured.contains(&config_body),
        "preview must embed the generated config body verbatim:\n{captured}"
    );
    assert!(
        captured.contains("cerulion graph run attach --single-process"),
        "preview must name the run hand-off:\n{captured}"
    );
}

#[test]
fn interactive_confirm_no_declines_and_writes_nothing() {
    let tmp = tempfile::tempdir().unwrap();
    let mut confirm = |_p: &str| -> CliResult<bool> { Ok(false) };
    let report = ros_cmd::ros_attach(
        &cloud_only(),
        tmp.path(),
        &opts(false, false),
        true,
        &mut confirm,
    )
    .expect("declined");
    assert_eq!(report.outcome, AttachOutcome::Declined);
    assert!(report.graph_to_run.is_none());
    assert!(!tmp.path().join("graphs").join("attach.yaml").exists());
}

/// The never-mutate floor vs PRE-EXISTING files — the other
/// decline/refusal arms only prove "no file appears"; this pins that files
/// ALREADY at the generated paths (hand edits included) survive BOTH no-write
/// arms byte-identical, with no `.bak` churn either.
#[test]
fn decline_and_non_tty_refusal_never_mutate_preexisting_files() {
    let tmp = tempfile::tempdir().unwrap();
    let graphs = tmp.path().join("graphs");
    std::fs::create_dir_all(&graphs).unwrap();
    let graph = graphs.join("attach.yaml");
    let config = graphs.join("attach.bridge.yaml");
    const G_SENTINEL: &str = "# sentinel graph — hand edits live here\nname: keep_me\n";
    const C_SENTINEL: &str = "# sentinel config — hand edits live here\nmappings: []\n";
    std::fs::write(&graph, G_SENTINEL).unwrap();
    std::fs::write(&config, C_SENTINEL).unwrap();

    // Decline arm: confirm answers "no".
    let mut deny = |_p: &str| -> CliResult<bool> { Ok(false) };
    let report = ros_cmd::ros_attach(
        &cloud_only(),
        tmp.path(),
        &opts(false, false),
        true,
        &mut deny,
    )
    .expect("declined");
    assert_eq!(report.outcome, AttachOutcome::Declined);
    assert_eq!(read(&graph), G_SENTINEL, "decline must not touch the graph");
    assert_eq!(
        read(&config),
        C_SENTINEL,
        "decline must not touch the config"
    );

    // Non-TTY refusal arm: no TTY, no --yes.
    let mut confirm = panic_confirm;
    ros_cmd::ros_attach(
        &cloud_only(),
        tmp.path(),
        &opts(false, false),
        false,
        &mut confirm,
    )
    .expect_err("non-TTY without --yes must refuse");
    assert_eq!(read(&graph), G_SENTINEL, "refusal must not touch the graph");
    assert_eq!(
        read(&config),
        C_SENTINEL,
        "refusal must not touch the config"
    );

    // No backup churn on either arm — a .bak would imply the write path ran.
    assert!(!graphs.join("attach.yaml.bak").exists());
    assert!(!graphs.join("attach.bridge.yaml.bak").exists());
}

#[test]
fn non_tty_without_yes_refuses_naming_the_escape_hatches() {
    let tmp = tempfile::tempdir().unwrap();
    let mut confirm = panic_confirm;
    let err = ros_cmd::ros_attach(
        &cloud_only(),
        tmp.path(),
        &opts(false, false),
        false,
        &mut confirm,
    )
    .unwrap_err();
    let msg = err.to_string();
    assert!(msg.contains("--yes"), "{msg}");
    assert!(msg.contains("--dry-run"), "{msg}");
    assert!(msg.contains("not a TTY"), "{msg}");
    assert!(!tmp.path().join("graphs").join("attach.yaml").exists());
}

#[test]
fn nothing_resolvable_writes_nothing_even_with_yes() {
    // Only unresolvable types discovered.
    // Garbage packages: Imu/LaserScan (built-in ROS 2
    // messages) resolve RAW — a truly unresolvable type is one with NO schema.
    let disc = FakeDiscovery::ok(vec![
        ep(
            "rt/widget",
            "acme_msgs::msg::dds_::Widget_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/gadget",
            "acme_msgs::msg::dds_::Gadget_",
            EndpointKind::Writer,
            BE_VOL,
        ),
    ]);
    let tmp = tempfile::tempdir().unwrap();
    let mut confirm = panic_confirm;
    let report = ros_cmd::ros_attach(&disc, tmp.path(), &opts(false, true), false, &mut confirm)
        .expect("nothing resolvable");
    assert_eq!(report.outcome, AttachOutcome::NothingResolvable);
    assert!(report.graph_to_run.is_none());
    assert!(!tmp.path().join("graphs").join("attach.yaml").exists());
    assert!(
        report.report.contains("UNRESOLVABLE (2)"),
        "{}",
        report.report
    );
}

#[test]
fn discovery_failure_is_a_loud_error() {
    let disc = FakeDiscovery::failing(DdsError::ParticipantAlreadyExists);
    let tmp = tempfile::tempdir().unwrap();
    let mut confirm = panic_confirm;
    let err =
        ros_cmd::ros_attach(&disc, tmp.path(), &opts(true, false), true, &mut confirm).unwrap_err();
    let msg = err.to_string();
    assert!(msg.contains("DDS discovery failed"), "{msg}");
    assert!(msg.contains("--iface"), "{msg}");
    assert!(msg.contains("ROS_DOMAIN_ID"), "{msg}");
}

#[test]
fn invalid_graph_name_is_rejected_before_any_write() {
    let tmp = tempfile::tempdir().unwrap();
    let mut confirm = panic_confirm;
    let mut o = opts(false, true);
    o.graph_name = "../escape".to_string();
    let err = ros_cmd::ros_attach(&cloud_only(), tmp.path(), &o, false, &mut confirm).unwrap_err();
    assert!(matches!(err, CliError::Validation(_)));
    assert!(err.to_string().contains("invalid --graph-name"), "{err}");
}

#[test]
fn second_run_backs_up_the_prior_files() {
    let tmp = tempfile::tempdir().unwrap();
    let mut confirm = panic_confirm;
    // First write.
    ros_cmd::ros_attach(
        &cloud_only(),
        tmp.path(),
        &opts(false, true),
        false,
        &mut confirm,
    )
    .expect("first write");
    // Second write ⇒ .bak of both files.
    let report = ros_cmd::ros_attach(
        &cloud_only(),
        tmp.path(),
        &opts(false, true),
        false,
        &mut confirm,
    )
    .expect("second write");
    match report.outcome {
        AttachOutcome::Written {
            graph_backup,
            config_backup,
            ..
        } => {
            assert!(graph_backup.is_some(), "graph .bak on overwrite");
            assert!(config_backup.is_some(), "config .bak on overwrite");
        }
        other => panic!("expected Written, got {other:?}"),
    }
    assert!(tmp.path().join("graphs").join("attach.yaml.bak").exists());
    assert!(tmp
        .path()
        .join("graphs")
        .join("attach.bridge.yaml.bak")
        .exists());
}

// ─────────────────── --topic-prefix at the entry ──────────────

/// A prefix without a leading '/' is
/// normalized AT THE ENTRY, so every generated `cerulion_topic` / graph
/// `topic:` is absolute and both files pass their real validators. Without it
/// `--topic-prefix go2` produces the RELATIVE `go2/utlidar/cloud` (rejected by
/// `BridgeConfig::validate` AND `validate_graph` — but only after the consent
/// ladder has already written both files).
#[test]
fn relative_topic_prefix_is_normalized_before_generation() {
    let tmp = tempfile::tempdir().unwrap();
    let mut confirm = panic_confirm;
    let mut o = opts(false, true);
    o.topic_prefix = Some("go2".to_string()); // no leading '/'
    let report = ros_cmd::ros_attach(&cloud_only(), tmp.path(), &o, false, &mut confirm)
        .expect("normalized prefix must write");
    assert!(matches!(report.outcome, AttachOutcome::Written { .. }));

    // The mapping carries the ABSOLUTE topic (never "go2/utlidar/cloud").
    assert_eq!(report.mappings[0].cerulion_topic, "/go2/utlidar/cloud");

    // The written config parses AND its topics are absolute (the bridge's
    // leading-'/' validation shape).
    let config = read(&tmp.path().join("graphs").join("attach.bridge.yaml"));
    let cfg: BridgeConfigMirror = serde_yaml::from_str(&config).expect("config parses");
    assert_eq!(cfg.mappings[0].cerulion_topic, "/go2/utlidar/cloud");
    assert!(
        cfg.mappings
            .iter()
            .all(|m| m.cerulion_topic.starts_with('/')),
        "every generated cerulion_topic must be absolute:\n{config}"
    );

    // The written graph parses and the mapped port publishes the prefixed
    // absolute topic.
    let graph = read(&tmp.path().join("graphs").join("attach.yaml"));
    let g: cerulion_core::graph::config::GraphConfig =
        serde_yaml::from_str(&graph).expect("graph parses");
    assert_eq!(
        g.nodes[0].outputs[0].topic.as_deref(),
        Some("/go2/utlidar/cloud")
    );
}

/// The reject arms: prefixes that stay invalid after
/// normalization — trailing '/', embedded whitespace, empty-after-normalize —
/// are rejected with the exact remediation BEFORE anything is generated or
/// written, even under `--yes`. (Otherwise `go2/` writes both files and the run
/// then fails validation — a mutated workspace for a run that can never
/// succeed.) The set also covers the shapes the local edge
/// rules alone would miss but the GRAPH layer's own predicate rejects — interior `//`
/// (`/a//b`) and zenoh-reserved chars (`/cam*`): left to the graph layer, both write both
/// files under `--yes` and then die at graph load; the normalizer probes
/// `cerulion_core::graph::malformed_absolute_name` (single source of truth),
/// so they are refused here with nothing written.
#[test]
fn invalid_topic_prefixes_are_rejected_before_any_write() {
    for bad in [
        "go2/", "/go2/", "a b", "/a b", "", "/", "\t", "/a//b", "/cam*",
    ] {
        let tmp = tempfile::tempdir().unwrap();
        let mut confirm = panic_confirm;
        let mut o = opts(false, true); // --yes: only the reject stops the write
        o.topic_prefix = Some(bad.to_string());
        let err = ros_cmd::ros_attach(&cloud_only(), tmp.path(), &o, false, &mut confirm)
            .expect_err(&format!("--topic-prefix {bad:?} must be rejected"));
        assert!(
            err.to_string().contains("--topic-prefix"),
            "the error must name the flag for {bad:?}: {err}"
        );
        assert!(
            !tmp.path().join("graphs").join("attach.yaml").exists(),
            "{bad:?}: nothing may be written on the reject arm"
        );
        assert!(
            !tmp.path()
                .join("graphs")
                .join("attach.bridge.yaml")
                .exists(),
            "{bad:?}: nothing may be written on the reject arm"
        );
    }
}

// ────────── Malformed discovered topic names ────────────────────────

/// The pure half: a hostile/nonconforming raw DDS
/// topic name (`//`, `*`) flows through normalization into a candidate
/// `cerulion_topic` that graph load would reject. The exclusion gate must
/// (a) move each one into the loudly-reported bucket with the graph
/// predicate's reason VERBATIM, (b) never let it claim a bridge port a valid
/// sibling should keep — otherwise `/a//b` (sorts first) takes the PointCloud2
/// port and the VALID `/utlidar/cloud` is dropped as a duplicate — and
/// (c) render its own report section, pinned byte-exact.
#[test]
fn malformed_discovered_topics_are_excluded_loudly() {
    let endpoints = vec![
        ep(
            "rt/a//b",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/cmd*",
            "geometry_msgs::msg::dds_::Twist_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/utlidar/cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        ),
    ];
    let topics = aggregate_topics(&endpoints);
    let mut part = partition_topics_with(&AttachSchemaChain::builtins_only(), &topics);
    assert_eq!(part.resolvable.len(), 3, "all three types resolve");

    let (clean, malformed) = exclude_malformed_topics(std::mem::take(&mut part.resolvable), None);
    part.resolvable = clean;

    // The exclusion bucket: both hostile names, graph-predicate reasons
    // verbatim, in sorted (deterministic) order.
    assert_eq!(
        malformed,
        vec![
            ExcludedMalformedTopic {
                ros_topic: "/a//b".to_string(),
                ros_type: "sensor_msgs/PointCloud2".to_string(),
                cerulion_topic: "/a//b".to_string(),
                reason: "empty segment ('//')",
            },
            ExcludedMalformedTopic {
                ros_topic: "/cmd*".to_string(),
                ros_type: "geometry_msgs/Twist".to_string(),
                cerulion_topic: "/cmd*".to_string(),
                reason: "zenoh-reserved character ('*', '?', '#', '$', '@') — topic \
                         names must be valid zenoh key chunks for network bridging \
                         ('@' marks a verbatim chunk with special matching semantics)",
            },
        ]
    );

    // The VALID sibling keeps the PointCloud2 port (without the exclusion `/a//b`
    // would claim it and `/utlidar/cloud` be dropped). After malformed exclusion only ONE
    // PointCloud2 remains, so the typed-port race has no loser: nothing
    // is raw-routed onto the generic codec, nothing is unbridgeable.
    let assignment = assign_typed_ports(&AttachSchemaChain::builtins_only(), part.resolvable);
    assert!(
        assignment.raw_routed.is_empty() && assignment.unbridgeable.is_empty(),
        "the lone valid sibling keeps the typed port: raw_routed={:?} unbridgeable={:?}",
        assignment.raw_routed,
        assignment.unbridgeable
    );
    part.resolvable = assignment.resolvable;
    let (mappings, topic_conflicts) = build_mappings(&part.resolvable, None);
    assert_eq!(mappings.len(), 1);
    assert_eq!(mappings[0].cerulion_topic, "/utlidar/cloud");
    assert!(topic_conflicts.is_empty());

    // The report: byte-exact, with the EXCLUDED section + summary suffix.
    let report = render_discovery_report(
        &params(),
        &topics,
        &part,
        &assignment.unbridgeable,
        &assignment.raw_routed,
        &topic_conflicts,
        &malformed,
        0,
    );
    let expected = concat!(
        "DISCOVERED DDS TOPICS (interface 192.168.123.18, domain 0)\n",
        "\n",
        "RESOLVABLE (1) — bridged onto the Cerulion wire:\n",
        "  /utlidar/cloud  (sensor_msgs/PointCloud2)  writers=1 readers=0  qos=best_effort/volatile  (bridged natively)\n",
        "\n",
        "UNRESOLVABLE (0) — no Cerulion schema; NOT bridged:\n",
        "  (none)\n",
        "\n",
        "EXCLUDED — MALFORMED TOPIC NAMES (2): these would be rejected at graph load; NOT bridged (fix the publisher's topic name, or bridge it manually with a valid `topic:` override):\n",
        "  /a//b  (sensor_msgs/PointCloud2)  — excluded: empty segment ('//')\n",
        "  /cmd*  (geometry_msgs/Twist)  — excluded: zenoh-reserved character ('*', '?', '#', '$', '@') — topic names must be valid zenoh key chunks for network bridging ('@' marks a verbatim chunk with special matching semantics)\n",
        "\n",
        "Summary: 3 topic(s) discovered, 1 resolvable, 0 unresolvable, 2 malformed-topic(s) excluded.\n",
    );
    assert_eq!(report, expected);

    // Every topic the generators would emit passes the graph predicate — the
    // single-source-of-truth loop closed at the output side too.
    let graph = generate_bridge_graph("attach", "attach", &mappings);
    for line in graph
        .lines()
        .filter(|l| l.trim_start().starts_with("topic: "))
    {
        let topic = line.trim_start().trim_start_matches("topic: ");
        assert!(
            cerulion_core::graph::malformed_absolute_name(topic).is_none(),
            "generated graph carries a topic graph load would reject: {topic}"
        );
    }
}

/// The e2e half: through the full `ros_attach`
/// consent write under `--yes`, the hostile names land in the report's
/// EXCLUDED section and in NEITHER written file, while the valid sibling
/// still generates — otherwise both files would be written carrying `/a//b` (which
/// would also steal the cloud port) and the run would then die at graph load.
#[test]
fn malformed_discovered_topics_never_reach_the_written_files() {
    let disc = FakeDiscovery::ok(vec![
        ep(
            "rt/a//b",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/cmd*",
            "geometry_msgs::msg::dds_::Twist_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/utlidar/cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        ),
    ]);
    let tmp = tempfile::tempdir().unwrap();
    let mut confirm = panic_confirm;
    let report = ros_cmd::ros_attach(&disc, tmp.path(), &opts(false, true), false, &mut confirm)
        .expect("valid sibling must still write");
    assert!(matches!(report.outcome, AttachOutcome::Written { .. }));

    // The report names both exclusions with reasons.
    assert!(
        report
            .report
            .contains("EXCLUDED — MALFORMED TOPIC NAMES (2)"),
        "{}",
        report.report
    );
    assert!(
        report
            .report
            .contains("  /a//b  (sensor_msgs/PointCloud2)  — excluded: empty segment ('//')"),
        "{}",
        report.report
    );
    assert!(
        report
            .report
            .contains("  /cmd*  (geometry_msgs/Twist)  — excluded: zenoh-reserved"),
        "{}",
        report.report
    );

    // Neither hostile name reaches either written file; the valid sibling does.
    let graph = read(&tmp.path().join("graphs").join("attach.yaml"));
    let config = read(&tmp.path().join("graphs").join("attach.bridge.yaml"));
    for hostile in ["/a//b", "/cmd*"] {
        assert!(
            !graph.contains(hostile),
            "graph must not carry {hostile}:\n{graph}"
        );
        assert!(
            !config.contains(hostile),
            "config must not carry {hostile}:\n{config}"
        );
    }
    assert!(graph.contains("topic: /utlidar/cloud\n"), "{graph}");
    assert!(
        config.contains("cerulion_topic: /utlidar/cloud\n"),
        "{config}"
    );

    // The valid sibling owns the cloud port (not a silent default) — the
    // port-claim half of the exclusion contract.
    assert_eq!(report.mappings.len(), 1);
    assert_eq!(report.mappings[0].ros_type, "sensor_msgs/PointCloud2");
    assert_eq!(report.mappings[0].cerulion_topic, "/utlidar/cloud");
}

// ───────── The raw route, end to end ─────────────────

/// A built-in ROS 2 message outside the 4-type registry (Imu, which would
/// otherwise land UNRESOLVABLE on a real robot) resolves as a
/// RAW mapping, emitted in the generated config alongside the typed one
/// (byte-exact oracle, parsing under the bridge's `TopicMapping` shape) while the
/// LEAN one-node graph stays byte-identical to the typed-only case (raw
/// routes involve NO graph port).
#[test]
fn imu_resolves_as_raw_with_byte_oracle_config_and_untouched_graph() {
    let endpoints = vec![
        ep(
            "rt/utlidar/cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/imu_data",
            "sensor_msgs::msg::dds_::Imu_",
            EndpointKind::Writer,
            REL_VOL,
        ),
    ];
    let topics = aggregate_topics(&endpoints);
    let part = partition_topics_with(&AttachSchemaChain::builtins_only(), &topics);
    let (mappings, topic_conflicts) = build_mappings(&part.resolvable, None);
    assert!(topic_conflicts.is_empty());
    assert_eq!(mappings.len(), 2);
    assert_eq!(mappings[0].ros_type, "sensor_msgs/Imu");
    assert!(matches!(mappings[0].route, BridgeRoute::Raw));
    assert!(matches!(mappings[1].route, BridgeRoute::Typed(_)));

    // Config byte oracle: raw mapping emitted alongside the typed one (same
    // 4-field shape; the 726 bridge routes them internally by type).
    let yaml = generate_bridge_config_with_store(0, &[iface()], &mappings, &[]);
    let expected = concat!(
        "# Generated by `cerulion ros2 attach`.\n",
        "# DDS->Cerulion mapping config for the `dds_bridge` node, consumed via the\n",
        "# DDS_BRIDGE_CONFIG env var. Typed mappings ride the bridge's fixed ports\n",
        "# (cerulion_topic is documentation; the graph's `topic:` override is\n",
        "# authoritative). Raw mappings ride the generic codec: cerulion_topic\n",
        "# is AUTHORITATIVE, and max_slice_len defaults to 1 MiB (set it per mapping\n",
        "# for larger frames). qos is best_effort (matches both reliable and\n",
        "# best-effort publishers). Edit freely.\n",
        "domain_id: 0\n",
        "only_networks:\n",
        "  - 192.168.123.18\n",
        "mappings:\n",
        "  - dds_topic: /imu_data\n",
        "    ros_type: sensor_msgs/Imu\n",
        "    cerulion_topic: /imu_data\n",
        "    qos: best_effort\n",
        "  - dds_topic: /utlidar/cloud\n",
        "    ros_type: sensor_msgs/PointCloud2\n",
        "    cerulion_topic: /utlidar/cloud\n",
        "    qos: best_effort\n",
    );
    assert_eq!(yaml, expected);
    // Parses under the bridge's serde shape (the 726 TopicMapping accepts the
    // same fields; max_slice_len omitted = the 1 MiB raw default).
    let cfg: BridgeConfigMirror = serde_yaml::from_str(&yaml).expect("raw+typed config parses");
    assert_eq!(cfg.mappings.len(), 2);
    assert_eq!(cfg.mappings[0].ros_type, "sensor_msgs/Imu");

    // The LEAN graph is UNTOUCHED by the raw mapping: byte-identical to the
    // graph generated from the typed mapping alone.
    let typed_only: Vec<_> = mappings
        .iter()
        .filter(|m| matches!(m.route, BridgeRoute::Typed(_)))
        .cloned()
        .collect();
    assert_eq!(
        generate_bridge_graph("attach", "attach", &mappings),
        generate_bridge_graph("attach", "attach", &typed_only),
        "raw mappings must not change the generated graph"
    );
    let graph = generate_bridge_graph("attach", "attach", &mappings);
    assert!(
        !graph.contains("/imu_data"),
        "no graph port for a raw mapping:\n{graph}"
    );
}

// ───────── Own-endpoint accounting in the report ─────────────

/// The backend's own-endpoint hidden count surfaces as the
/// one-line report footer — on a populated report AND on the empty arm (a
/// LAN where everything discovered was our own participant) — and rides
/// through `ros_attach` end to end. Zero stays silent (pinned by every other
/// byte-exact oracle in this file carrying no footer).
#[test]
fn own_hidden_endpoints_are_footer_accounted_never_silently_dropped() {
    // Render-level: populated report carries the footer verbatim.
    let endpoints = vec![ep(
        "rt/utlidar/cloud",
        "sensor_msgs::msg::dds_::PointCloud2_",
        EndpointKind::Writer,
        BE_VOL,
    )];
    let topics = aggregate_topics(&endpoints);
    let part = partition_topics_with(&AttachSchemaChain::builtins_only(), &topics);
    let (_, topic_conflicts) = build_mappings(&part.resolvable, None);
    let report = render_discovery_report(
        &params(),
        &topics,
        &part,
        &[],
        &[],
        &topic_conflicts,
        &[],
        6,
    );
    assert!(
        report.ends_with("(6 of our own discovery endpoints hidden)\n"),
        "{report}"
    );

    // Empty arm: everything on the LAN was ours — the footer explains why
    // the list is empty instead of implying a dead robot.
    let empty = aggregate_topics(&[]);
    let empty_part = partition_topics_with(&AttachSchemaChain::builtins_only(), &empty);
    let empty_report =
        render_discovery_report(&params(), &empty, &empty_part, &[], &[], &[], &[], 4);
    assert!(
        empty_report.contains("no DDS topics discovered"),
        "{empty_report}"
    );
    assert!(
        empty_report.ends_with("(4 of our own discovery endpoints hidden)\n"),
        "{empty_report}"
    );

    // End to end through ros_attach (dry-run): the backend's count reaches
    // the printed report.
    let disc = FakeDiscovery::ok_with_hidden(
        vec![ep(
            "rt/utlidar/cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        )],
        3,
    );
    let tmp = tempfile::tempdir().unwrap();
    // The footer unconditionally ends the DISCOVERY report: no viz note
    // (affirmative or degrade) is appended after it. The automatic
    // MIGRATION section (printed on every run) FOLLOWS the discovery
    // report, so "last line" means last line of the
    // discovery half — split at the migration header and pin the footer as
    // that half's final line.
    let mut confirm = panic_confirm;
    let out = ros_cmd::ros_attach(&disc, tmp.path(), &opts(true, false), true, &mut confirm)
        .expect("dry run");
    let (discovery_half, _) = out
        .report
        .split_once("\nMIGRATION — what could run natively on rmw_cerulion:")
        .expect("the migration section follows the discovery report");
    assert!(
        discovery_half.ends_with("(3 of our own discovery endpoints hidden)\n"),
        "{}",
        out.report
    );
}

// ─────────────── The store→builtins resolvability chain ────

/// Helper: write a `.msg` file into `<root>/schemas/<pkg>/msg/<Type>.msg`.
fn write_store_msg(root: &Path, pkg: &str, type_name: &str, text: &str) {
    let dir = root.join("schemas").join(pkg).join("msg");
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join(format!("{type_name}.msg")), text).unwrap();
}

/// The store WIDENS the resolvability seam: a type the built-in corpus does
/// not carry (`acme_msgs/Widget`) is UNRESOLVABLE on the builtins-only chain
/// but flips to a RAW route once a `schemas/acme_msgs/msg/Widget.msg` file
/// exists — toggling ONLY the store flips the decision (the load-bearing pin).
#[test]
fn store_only_type_flips_unresolvable_to_raw() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(
        tmp.path(),
        "acme_msgs",
        "Widget",
        "int32 id\nfloat64 value\n",
    );

    let endpoints = vec![ep(
        "rt/widget",
        "acme_msgs::msg::dds_::Widget_",
        EndpointKind::Writer,
        BE_VOL,
    )];
    let topics = aggregate_topics(&endpoints);

    // Builtins-only (the legacy entry point): acme_msgs/Widget stays out.
    let builtins = partition_topics_with(&AttachSchemaChain::builtins_only(), &topics);
    assert!(
        builtins.resolvable.is_empty(),
        "no built-in for acme_msgs/Widget"
    );
    assert_eq!(builtins.unresolvable.len(), 1);

    // Store-backed chain from the workspace root: RESOLVABLE (raw).
    let chain = AttachSchemaChain::from_workspace(tmp.path());
    let with_store = partition_topics_with(&chain, &topics);
    assert!(
        with_store.unresolvable.is_empty(),
        "the store type flips resolvable"
    );
    assert_eq!(with_store.resolvable.len(), 1);
    assert_eq!(with_store.resolvable[0].topic.ros_type, "acme_msgs/Widget");
    assert!(
        matches!(with_store.resolvable[0].route, BridgeRoute::Raw),
        "a store-resolved type routes through the generic (raw) codec"
    );
}

/// Anti-tautology control: a BUILT-IN type still resolves through an EMPTY
/// store (the chain's builtins arm is intact — the store is additive, never
/// a gate). The workspace has no `schemas/` at all here.
#[test]
fn builtin_type_resolves_through_empty_store_chain() {
    let tmp = tempfile::tempdir().unwrap();
    let endpoints = vec![ep(
        "rt/imu",
        "sensor_msgs::msg::dds_::Imu_",
        EndpointKind::Writer,
        BE_VOL,
    )];
    let topics = aggregate_topics(&endpoints);

    let chain = AttachSchemaChain::from_workspace(tmp.path());
    let part = partition_topics_with(&chain, &topics);
    assert_eq!(part.resolvable.len(), 1);
    assert_eq!(part.resolvable[0].topic.ros_type, "sensor_msgs/Imu");
    assert!(matches!(part.resolvable[0].route, BridgeRoute::Raw));
    assert!(part.unresolvable.is_empty());
}

/// A store entry whose qualified name equals a built-in is a LOUD fact,
/// surfaced ONCE at chain construction (the boundary where the store-wins
/// inference happens) — never silent.
#[tracing_test::traced_test]
#[test]
fn store_shadowing_builtin_warns_at_chain_build() {
    let tmp = tempfile::tempdir().unwrap();
    // sensor_msgs/Imu is a built-in; the store copy shadows it.
    write_store_msg(tmp.path(), "sensor_msgs", "Imu", "int32 marker\n");

    let _chain = AttachSchemaChain::from_workspace(tmp.path());
    assert!(
        logs_contain("shadows a built-in ROS 2 message"),
        "the store→builtin shadow must be loud"
    );
    assert!(
        logs_contain("sensor_msgs/Imu"),
        "the warn must name the shadowed qualified type"
    );
}

/// The builtins-only chain never warns about shadows (a workspace with a
/// non-colliding store entry) — the shadow warn fires only for a genuine
/// built-in collision, not on every attach.
#[tracing_test::traced_test]
#[test]
fn non_colliding_store_entry_does_not_warn() {
    let tmp = tempfile::tempdir().unwrap();
    write_store_msg(tmp.path(), "acme_msgs", "Widget", "int32 id\n");
    let _chain = AttachSchemaChain::from_workspace(tmp.path());
    assert!(
        !logs_contain("shadows a built-in"),
        "a non-colliding store type must NOT warn about a shadow"
    );
}

// ───────── The schema-acquisition ladder + consent ─────────
//
// Every acquirer here is a CANNED `cerulion_dds::SchemaAcquirer` (hermetic — no
// DDS, no wire) and every assertion is against a HAND-WRITTEN oracle: the
// report markers, the resolvable/unresolvable flip, byte-verbatim `.msg`
// materialization, the consent ladder (dry-run/--yes/non-TTY), partial-closure
// reporting, dedupe, and determinism. Parallel-safe (tempdirs).

use std::cell::{Cell, RefCell};
use std::collections::BTreeMap;

/// A canned [`SchemaAcquirer`] returning a hand-built outcome per type, and
/// RECORDING every type it was handed (to pin dedupe + store-exclusion).
struct CannedAcquirer {
    responses: BTreeMap<String, AcquisitionOutcome>,
    /// Every type name passed to `acquire`, in order, across all calls.
    seen: RefCell<Vec<String>>,
    /// Number of `acquire` invocations.
    calls: Cell<usize>,
}

impl CannedAcquirer {
    fn new(responses: BTreeMap<String, AcquisitionOutcome>) -> Self {
        Self {
            responses,
            seen: RefCell::new(Vec::new()),
            calls: Cell::new(0),
        }
    }
    fn seen(&self) -> Vec<String> {
        self.seen.borrow().clone()
    }
    fn call_count(&self) -> usize {
        self.calls.get()
    }
}

impl SchemaAcquirer for CannedAcquirer {
    fn acquire(&self, types: &[String], _discovery: &DiscoveryResult) -> Vec<TypeAcquisition> {
        self.calls.set(self.calls.get() + 1);
        self.seen.borrow_mut().extend(types.iter().cloned());
        types
            .iter()
            .filter_map(|t| {
                self.responses.get(t).map(|o| TypeAcquisition {
                    requested: t.clone(),
                    outcome: o.clone(),
                })
            })
            .collect()
    }
}

/// A [`SchemaAcquirer`] that PANICS if invoked — proves a code path never
/// reaches the acquirer (empty unresolvable set; a store-resolved type).
struct PanicAcquirer;
impl SchemaAcquirer for PanicAcquirer {
    fn acquire(&self, types: &[String], _discovery: &DiscoveryResult) -> Vec<TypeAcquisition> {
        panic!("the acquirer must not be invoked (handed: {types:?})");
    }
}

/// Build an `Acquired` outcome from a rung + a closure of `(pkg, type, text)`.
fn acquired(rung: AcquisitionRung, closure: &[(&str, &str, &str)]) -> AcquisitionOutcome {
    AcquisitionOutcome::Acquired(AcquiredSchema {
        rung,
        closure: closure
            .iter()
            .map(|(pkg, ty, text)| AcquiredMsg {
                package: pkg.to_string(),
                type_name: ty.to_string(),
                msg_text: text.to_string(),
            })
            .collect(),
    })
}

/// Build a `Skipped` outcome from a list of `(rung, reason)`.
fn skipped(reasons: &[(AcquisitionRung, &str)]) -> AcquisitionOutcome {
    AcquisitionOutcome::Skipped(
        reasons
            .iter()
            .map(|(rung, reason)| RungSkip {
                rung: *rung,
                reason: reason.to_string(),
            })
            .collect(),
    )
}

fn canned(responses: &[(&str, AcquisitionOutcome)]) -> CannedAcquirer {
    CannedAcquirer::new(
        responses
            .iter()
            .map(|(t, o)| (t.to_string(), o.clone()))
            .collect(),
    )
}

/// Discovery of ONE writer on `rt/widget` publishing `acme_msgs/Widget` (a type
/// no built-in / typed registry resolves — the acquisition candidate).
fn widget_only() -> FakeDiscovery {
    FakeDiscovery::ok(vec![ep(
        "rt/widget",
        "acme_msgs::msg::dds_::Widget_",
        EndpointKind::Writer,
        BE_VOL,
    )])
}

const WIDGET_MSG: &str = "int32 id\nfloat64 value\n";

/// TEST 1 — happy path: an unresolvable type acquired with its full closure
/// flips to RESOLVABLE (raw) carrying the acquired marker, and its bridge route
/// is SHAPE-IDENTICAL to a store-resolved twin (the acquisition path produces
/// exactly the mapping/config a hand-seeded store would).
#[test]
fn test_acquired_type_flips_to_resolvable_with_marker_and_shape_identical_route() {
    let tmp = tempfile::tempdir().unwrap();
    let acq = canned(&[(
        "acme_msgs/Widget",
        acquired(
            AcquisitionRung::WireService,
            &[("acme_msgs", "Widget", WIDGET_MSG)],
        ),
    )]);
    let disc = widget_only();
    let mut confirm = panic_confirm;
    let report = ros_cmd::ros_attach_with_acquirer(
        &disc,
        &acq,
        tmp.path(),
        &opts(true, false), // dry-run: inspect without writing
        true,
        &mut confirm,
    )
    .expect("dry run");

    // The type flipped to RESOLVABLE (raw route).
    assert_eq!(report.mappings.len(), 1);
    assert_eq!(report.mappings[0].ros_type, "acme_msgs/Widget");
    assert!(matches!(report.mappings[0].route, BridgeRoute::Raw));

    // The report AUGMENTS the raw route marker with the acquired note (both
    // the HOW and the not-yet-written state) + names the ACQUIRED SCHEMAS
    // file (its header's tense agrees with the marker).
    assert!(
        report.report.contains(
            "(bridged via the generic codec; schema acquired — writes \
             schemas/acme_msgs/msg/Widget.msg on consent)"
        ),
        "{}",
        report.report
    );
    assert!(
        report.report.contains("ACQUIRED SCHEMAS (1)"),
        "{}",
        report.report
    );
    assert!(
        report.report.contains(
            "  schemas/acme_msgs/msg/Widget.msg  (acme_msgs/Widget, wire service via \
             get_type_description)"
        ),
        "{}",
        report.report
    );

    // Shape-identical to a STORE-resolved twin: seed the SAME `.msg` into the
    // store, run with NO acquirer, and the mappings + config are byte-identical.
    let tmp_store = tempfile::tempdir().unwrap();
    write_store_msg(tmp_store.path(), "acme_msgs", "Widget", WIDGET_MSG);
    let store_report = ros_cmd::ros_attach(
        &widget_only(),
        tmp_store.path(),
        &opts(true, false),
        true,
        &mut confirm,
    )
    .expect("store dry run");
    assert_eq!(
        report.mappings, store_report.mappings,
        "acquired route mappings must equal the store-resolved twin's"
    );
    assert_eq!(
        generate_bridge_config_with_store(0, &[iface()], &report.mappings, &[]),
        generate_bridge_config_with_store(0, &[iface()], &store_report.mappings, &[]),
        "acquired route config must be shape-identical to the store-resolved twin"
    );
}

/// TEST 2 — dry-run PREVIEWS the would-be file but writes NOTHING (not even a
/// `schemas/` dir): the never-mutate-before-consent floor.
#[test]
fn test_dry_run_previews_schema_file_but_writes_nothing() {
    let tmp = tempfile::tempdir().unwrap();
    let acq = canned(&[(
        "acme_msgs/Widget",
        acquired(
            AcquisitionRung::WireService,
            &[("acme_msgs", "Widget", WIDGET_MSG)],
        ),
    )]);
    let mut confirm = panic_confirm;
    let report = ros_cmd::ros_attach_with_acquirer(
        &widget_only(),
        &acq,
        tmp.path(),
        &opts(true, false),
        true,
        &mut confirm,
    )
    .expect("dry run");
    assert_eq!(report.outcome, AttachOutcome::DryRun);
    assert!(
        report.report.contains("schemas/acme_msgs/msg/Widget.msg"),
        "dry-run report must NAME the would-be file:\n{}",
        report.report
    );
    // ZERO filesystem writes.
    assert!(
        !tmp.path().join("schemas").exists(),
        "dry-run must not create schemas/"
    );
    assert!(
        !tmp.path().join("graphs").exists(),
        "dry-run must not create graphs/"
    );
}

/// TEST 3 — non-TTY without `--yes` refuses with the existing refusal-text
/// pattern; NO schema file is materialized.
#[test]
fn test_non_tty_without_yes_refuses_and_writes_no_schema() {
    let tmp = tempfile::tempdir().unwrap();
    let acq = canned(&[(
        "acme_msgs/Widget",
        acquired(
            AcquisitionRung::WireService,
            &[("acme_msgs", "Widget", WIDGET_MSG)],
        ),
    )]);
    let mut confirm = panic_confirm;
    let err = ros_cmd::ros_attach_with_acquirer(
        &widget_only(),
        &acq,
        tmp.path(),
        &opts(false, false),
        false, // not a TTY
        &mut confirm,
    )
    .expect_err("non-TTY without --yes must refuse");
    let msg = err.to_string();
    assert!(msg.contains("--yes"), "{msg}");
    assert!(msg.contains("--dry-run"), "{msg}");
    assert!(msg.contains("not a TTY"), "{msg}");
    // The refusal names the THIRD write class (the acquired .msg
    // schemas), so a CI operator consents to exactly what --yes writes.
    assert!(
        msg.contains("acquired schema file(s)") && msg.contains("schemas/acme_msgs/msg/Widget.msg"),
        "refusal must name the pending schema writes: {msg}"
    );
    assert!(
        !tmp.path().join("schemas").exists(),
        "refusal must materialize no schema"
    );
}

/// TEST 4 — consent (`--yes`) materializes the `.msg` BYTE-VERBATIM into the
/// store, records it in the outcome, and the written store is immediately
/// consistent with the in-memory re-resolve (a later no-acquirer run resolves
/// the same type).
#[test]
fn test_assume_yes_materializes_schema_byte_verbatim_and_store_is_consistent() {
    let tmp = tempfile::tempdir().unwrap();
    let acq = canned(&[(
        "acme_msgs/Widget",
        acquired(
            AcquisitionRung::WireService,
            &[("acme_msgs", "Widget", WIDGET_MSG)],
        ),
    )]);
    let mut confirm = panic_confirm;
    let report = ros_cmd::ros_attach_with_acquirer(
        &widget_only(),
        &acq,
        tmp.path(),
        &opts(false, true), // --yes
        false,
        &mut confirm,
    )
    .expect("assume-yes write");
    assert!(matches!(report.outcome, AttachOutcome::Written { .. }));

    let dest = tmp
        .path()
        .join("schemas")
        .join("acme_msgs")
        .join("msg")
        .join("Widget.msg");
    // BYTE-VERBATIM (hand oracle).
    assert_eq!(read(&dest), WIDGET_MSG);

    // The outcome records the write (no .bak for a fresh file).
    match &report.outcome {
        AttachOutcome::Written { schema_writes, .. } => {
            assert_eq!(schema_writes.len(), 1);
            assert_eq!(schema_writes[0].path, dest);
            assert!(schema_writes[0].backup.is_none());
        }
        other => panic!("expected Written, got {other:?}"),
    }
    // The graph + config were still written.
    assert!(tmp.path().join("graphs").join("attach.yaml").exists());
    assert!(tmp
        .path()
        .join("graphs")
        .join("attach.bridge.yaml")
        .exists());

    // The written store is immediately consistent: a fresh chain resolves the
    // acquired type WITHOUT any acquirer.
    let chain = AttachSchemaChain::from_workspace(tmp.path());
    assert!(
        chain.resolves("acme_msgs/Widget"),
        "the materialized store must resolve the acquired type"
    );
}

/// TEST 5 — partial closure: the acquirer returns a type whose nested closure
/// references a type it does NOT include (and no store/builtin serves) — the
/// topic stays UNRESOLVABLE with a LOUD, precise reason naming the missing
/// nested type, and NOTHING is staged.
#[tracing_test::traced_test]
#[test]
fn test_partial_closure_stays_unresolvable_with_loud_missing_dep_reason() {
    let tmp = tempfile::tempdir().unwrap();
    // Widget references acme_msgs/Sub, which is absent from the closure.
    let acq = canned(&[(
        "acme_msgs/Widget",
        acquired(
            AcquisitionRung::WireService,
            &[("acme_msgs", "Widget", "acme_msgs/Sub sub\nint32 id\n")],
        ),
    )]);
    let mut confirm = panic_confirm;
    let report = ros_cmd::ros_attach_with_acquirer(
        &widget_only(),
        &acq,
        tmp.path(),
        &opts(true, false),
        true,
        &mut confirm,
    )
    .expect("dry run");

    // NOT resolved — no half-resolved claim.
    assert!(
        report.mappings.is_empty(),
        "an incomplete closure must not resolve: {:?}",
        report.mappings
    );
    assert!(
        report.report.contains("UNRESOLVABLE (1)"),
        "{}",
        report.report
    );
    // The reason is loud + precise: names INCOMPLETE and the missing nested type.
    assert!(report.report.contains("INCOMPLETE"), "{}", report.report);
    assert!(
        report.report.contains("acme_msgs/Sub"),
        "the missing nested type must be named: {}",
        report.report
    );
    // Nothing staged to write.
    assert!(
        !report.report.contains("ACQUIRED SCHEMAS"),
        "an incomplete closure stages nothing: {}",
        report.report
    );
    // The warn fired (loud path).
    assert!(
        logs_contain("INCOMPLETE nested closure"),
        "the incomplete-closure warn must fire"
    );
}

/// TEST 6 — dedupe by type: two topics of the SAME unresolvable type acquire
/// ONCE (the acquirer sees the type exactly once), and both topics flip
/// resolvable off the single acquisition.
#[test]
fn test_two_topics_same_type_acquire_the_type_once() {
    let tmp = tempfile::tempdir().unwrap();
    let acq = canned(&[(
        "acme_msgs/Widget",
        acquired(
            AcquisitionRung::WireService,
            &[("acme_msgs", "Widget", WIDGET_MSG)],
        ),
    )]);
    let disc = FakeDiscovery::ok(vec![
        ep(
            "rt/w1",
            "acme_msgs::msg::dds_::Widget_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/w2",
            "acme_msgs::msg::dds_::Widget_",
            EndpointKind::Writer,
            BE_VOL,
        ),
    ]);
    let mut confirm = panic_confirm;
    let report = ros_cmd::ros_attach_with_acquirer(
        &disc,
        &acq,
        tmp.path(),
        &opts(true, false),
        true,
        &mut confirm,
    )
    .expect("dry run");

    // The acquirer saw the type EXACTLY once (deduped), in one call.
    assert_eq!(acq.seen(), vec!["acme_msgs/Widget".to_string()]);
    assert_eq!(acq.call_count(), 1);
    // Both topics resolved off the single acquisition.
    assert_eq!(report.mappings.len(), 2);
    assert!(report
        .mappings
        .iter()
        .all(|m| m.ros_type == "acme_msgs/Widget"));
    // Exactly one `.msg` staged.
    assert!(
        report.report.contains("ACQUIRED SCHEMAS (1)"),
        "{}",
        report.report
    );
}

/// TEST 7 — no-acquirer default is BYTE-IDENTICAL to the legacy attach: the
/// `NoopAcquirer` path and `ros_attach` produce the same report + preview +
/// mappings, and an unresolvable type carries the legacy line (no
/// acquisition markers, no SCHEMAS section, no skip lines).
#[test]
fn test_noop_acquirer_is_byte_identical_to_legacy_attach() {
    let tmp_a = tempfile::tempdir().unwrap();
    let tmp_b = tempfile::tempdir().unwrap();
    let mut confirm = panic_confirm;

    // A discovery with a resolvable AND an unresolvable type.
    let mk = || {
        FakeDiscovery::ok(vec![
            ep(
                "rt/imu",
                "sensor_msgs::msg::dds_::Imu_",
                EndpointKind::Writer,
                BE_VOL,
            ),
            ep(
                "rt/widget",
                "acme_msgs::msg::dds_::Widget_",
                EndpointKind::Writer,
                BE_VOL,
            ),
        ])
    };
    let legacy = ros_cmd::ros_attach(&mk(), tmp_a.path(), &opts(true, false), true, &mut confirm)
        .expect("legacy dry run");
    let noop = ros_cmd::ros_attach_with_acquirer(
        &mk(),
        &NoopAcquirer,
        tmp_b.path(),
        &opts(true, false),
        true,
        &mut confirm,
    )
    .expect("noop dry run");

    assert_eq!(legacy.report, noop.report, "reports must be byte-identical");
    assert_eq!(
        legacy.preview, noop.preview,
        "previews must be byte-identical"
    );
    assert_eq!(legacy.mappings, noop.mappings);
    // And the report carries NONE of the acquisition surface.
    for marker in ["schema acquired", "ACQUIRED SCHEMAS", "· acquisition:"] {
        assert!(
            !noop.report.contains(marker),
            "no-acquirer report must not carry {marker:?}:\n{}",
            noop.report
        );
    }
    // The unresolvable type still renders its standard line.
    assert!(noop.report.contains("UNRESOLVABLE (1)"), "{}", noop.report);
}

/// TEST 8 — an acquirer SKIP reason lands in the report (under the UNRESOLVABLE
/// line) AND fires a loud warn.
#[tracing_test::traced_test]
#[test]
fn test_acquirer_skip_reason_lands_in_report_and_warns() {
    let tmp = tempfile::tempdir().unwrap();
    let acq = canned(&[(
        "acme_msgs/Widget",
        skipped(&[(
            AcquisitionRung::WireService,
            "no type hash in discovery (pre-Iron distro or non-ROS DDS publisher)",
        )]),
    )]);
    let mut confirm = panic_confirm;
    let report = ros_cmd::ros_attach_with_acquirer(
        &widget_only(),
        &acq,
        tmp.path(),
        &opts(true, false),
        true,
        &mut confirm,
    )
    .expect("dry run");

    assert!(
        report.mappings.is_empty(),
        "a skipped type stays unresolvable"
    );
    assert!(
        report.report.contains(
            "· acquisition: wire service via get_type_description: no type hash in discovery"
        ),
        "the skip reason must surface under the unresolvable line:\n{}",
        report.report
    );
    assert!(
        logs_contain("schema-acquisition rung skipped"),
        "the skip must fire a loud warn"
    );
}

/// TEST 9 — determinism: two identical runs (fresh tempdirs, fresh acquirers)
/// with a mix of acquired + skipped + already-resolvable types produce
/// BYTE-IDENTICAL reports and previews.
#[test]
fn test_two_identical_runs_produce_byte_identical_reports() {
    let mut confirm = panic_confirm;
    let responses: &[(&str, AcquisitionOutcome)] = &[
        (
            "acme_msgs/Widget",
            acquired(
                AcquisitionRung::WireService,
                &[("acme_msgs", "Widget", WIDGET_MSG)],
            ),
        ),
        (
            "acme_msgs/Gadget",
            skipped(&[(AcquisitionRung::BridgeScavenge, "no bridge on :8765")]),
        ),
    ];
    let mk_disc = || {
        FakeDiscovery::ok(vec![
            ep(
                "rt/imu",
                "sensor_msgs::msg::dds_::Imu_",
                EndpointKind::Writer,
                BE_VOL,
            ),
            ep(
                "rt/widget",
                "acme_msgs::msg::dds_::Widget_",
                EndpointKind::Writer,
                BE_VOL,
            ),
            ep(
                "rt/gadget",
                "acme_msgs::msg::dds_::Gadget_",
                EndpointKind::Writer,
                BE_VOL,
            ),
        ])
    };
    let mut run = || {
        let tmp = tempfile::tempdir().unwrap();
        let acq = canned(responses);
        ros_cmd::ros_attach_with_acquirer(
            &mk_disc(),
            &acq,
            tmp.path(),
            &opts(true, false),
            true,
            &mut confirm,
        )
        .expect("dry run")
    };
    let r1 = run();
    let r2 = run();
    assert_eq!(r1.report, r2.report, "reports must be deterministic");
    assert_eq!(r1.preview, r2.preview, "previews must be deterministic");
}

/// TEST 10a — empty unresolvable set: the acquirer is NEVER invoked (a
/// `PanicAcquirer` proves it — no panic ⇒ never called).
#[test]
fn test_empty_unresolvable_set_never_invokes_the_acquirer() {
    let tmp = tempfile::tempdir().unwrap();
    let mut confirm = panic_confirm;
    // Only a built-in type discovered ⇒ everything resolves ⇒ nothing to acquire.
    let disc = FakeDiscovery::ok(vec![ep(
        "rt/imu",
        "sensor_msgs::msg::dds_::Imu_",
        EndpointKind::Writer,
        BE_VOL,
    )]);
    let report = ros_cmd::ros_attach_with_acquirer(
        &disc,
        &PanicAcquirer,
        tmp.path(),
        &opts(true, false),
        true,
        &mut confirm,
    )
    .expect("dry run (acquirer must not be reached)");
    assert_eq!(report.mappings.len(), 1);
    assert_eq!(report.mappings[0].ros_type, "sensor_msgs/Imu");
}

/// TEST 10b — a type already resolvable via the store is EXCLUDED from the
/// acquirer's input: the acquirer sees ONLY the genuinely-unresolvable type.
#[test]
fn test_store_resolved_type_is_excluded_from_acquirer_input() {
    let tmp = tempfile::tempdir().unwrap();
    // Widget is pre-seeded in the store (resolvable); Gadget is not.
    write_store_msg(tmp.path(), "acme_msgs", "Widget", "int32 id\n");
    let acq = canned(&[(
        "acme_msgs/Gadget",
        acquired(
            AcquisitionRung::WireService,
            &[("acme_msgs", "Gadget", "int32 g\n")],
        ),
    )]);
    let disc = FakeDiscovery::ok(vec![
        ep(
            "rt/widget",
            "acme_msgs::msg::dds_::Widget_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/gadget",
            "acme_msgs::msg::dds_::Gadget_",
            EndpointKind::Writer,
            BE_VOL,
        ),
    ]);
    let mut confirm = panic_confirm;
    let report = ros_cmd::ros_attach_with_acquirer(
        &disc,
        &acq,
        tmp.path(),
        &opts(true, false),
        true,
        &mut confirm,
    )
    .expect("dry run");
    // The acquirer NEVER saw the store-resolved Widget.
    assert_eq!(
        acq.seen(),
        vec!["acme_msgs/Gadget".to_string()],
        "the store-resolved type must be excluded from the acquirer input"
    );
    // Both resolve — Widget via the store, Gadget via acquisition.
    assert_eq!(report.mappings.len(), 2);
}

/// TEST 11 — closure COMPLETED by a built-in nested dep resolves, and the
/// built-in is NOT duplicated into the store (only the genuinely-new type is
/// written). Also covers the array/nested walk (a nested `geometry_msgs/Point`).
#[test]
fn test_closure_completed_by_builtin_dep_resolves_without_duplicating_builtin() {
    let tmp = tempfile::tempdir().unwrap();
    let acq = canned(&[(
        "acme_msgs/Widget",
        acquired(
            AcquisitionRung::WireService,
            &[(
                "acme_msgs",
                "Widget",
                "geometry_msgs/Point origin\nint32 id\n",
            )],
        ),
    )]);
    let mut confirm = panic_confirm;
    let report = ros_cmd::ros_attach_with_acquirer(
        &widget_only(),
        &acq,
        tmp.path(),
        &opts(false, true), // --yes
        false,
        &mut confirm,
    )
    .expect("assume-yes write");
    assert!(matches!(report.outcome, AttachOutcome::Written { .. }));
    // Widget resolves (its nested geometry_msgs/Point is a built-in).
    assert_eq!(report.mappings.len(), 1);
    assert_eq!(report.mappings[0].ros_type, "acme_msgs/Widget");

    // ONLY Widget.msg is written — the built-in Point is served from the corpus,
    // never duplicated (and never shadow-warned later).
    match &report.outcome {
        AttachOutcome::Written { schema_writes, .. } => {
            assert_eq!(schema_writes.len(), 1);
            assert!(
                schema_writes[0].path.ends_with("acme_msgs/msg/Widget.msg"),
                "{:?}",
                schema_writes[0].path
            );
        }
        other => panic!("expected Written, got {other:?}"),
    }
    assert!(
        !tmp.path().join("schemas").join("geometry_msgs").exists(),
        "a built-in nested dep must not be materialized into the store"
    );
}

// ───── Acquisition hardening ──────────────

/// Drift-guard: feed cases through the codec's SHARED nested-ref ladder
/// (`resolve_nested_qname_ladder` — the exact fn the CDR decoder AND the attach
/// completeness walk both call) and assert every documented rung. Because the
/// codec delegates to this fn, this is the anti-drift pin: the walk can never
/// resolve a nested reference differently from the decoder.
#[test]
fn ladder_resolves_every_documented_rung() {
    fn ladder(name: &str, pkg: Option<&str>, parent: &str, universe: &[&str]) -> Option<String> {
        let u: Vec<&str> = universe.to_vec();
        cerulion_core::codegen::resolve_nested_qname_ladder(
            name,
            pkg,
            parent,
            |k| u.contains(&k),
            u.iter().copied(),
        )
    }
    // Rung 1 — qualified present / absent.
    assert_eq!(
        ladder(
            "Point",
            Some("geometry_msgs"),
            "acme/W",
            &["geometry_msgs/Point"]
        ),
        Some("geometry_msgs/Point".to_string())
    );
    assert_eq!(
        ladder("Point", Some("foo"), "acme/W", &["geometry_msgs/Point"]),
        None
    );
    // Rung 2 — same-package.
    assert_eq!(
        ladder("Sub", None, "acme/Widget", &["acme/Sub"]),
        Some("acme/Sub".to_string())
    );
    // Rung 3 — bare Header → std_msgs/Header.
    assert_eq!(
        ladder("Header", None, "acme/Widget", &["std_msgs/Header"]),
        Some("std_msgs/Header".to_string())
    );
    // Rung 2 WINS over rung 3 when same-package Header exists (codec order).
    assert_eq!(
        ladder(
            "Header",
            None,
            "acme/Widget",
            &["acme/Header", "std_msgs/Header"]
        ),
        Some("acme/Header".to_string())
    );
    // Rung 4 — unambiguous bare suffix.
    assert_eq!(
        ladder("Thing", None, "zzz/Widget", &["foo/Thing"]),
        Some("foo/Thing".to_string())
    );
    // Rung 4 — ambiguous (two suffix matches) → None.
    assert_eq!(
        ladder("Thing", None, "zzz/Widget", &["foo/Thing", "bar/Thing"]),
        None
    );
    // No rung resolves → None.
    assert_eq!(ladder("Ghost", None, "acme/Widget", &["acme/Other"]), None);
}

/// A COMPLETE closure using ROS 2's legal BARE `Header` reference
/// (the exact style in the vendored corpus) completes via the built-in
/// `std_msgs/Header` and flips RESOLVABLE — an eager `<pkg>/Header`
/// resolution would WRONGLY reject it as INCOMPLETE.
#[test]
fn test_bare_header_closure_completes_via_builtins() {
    let tmp = tempfile::tempdir().unwrap();
    let acq = canned(&[(
        "acme_msgs/Widget",
        acquired(
            AcquisitionRung::WireService,
            &[("acme_msgs", "Widget", "Header header\nfloat64 value\n")],
        ),
    )]);
    let mut confirm = panic_confirm;
    let report = ros_cmd::ros_attach_with_acquirer(
        &widget_only(),
        &acq,
        tmp.path(),
        &opts(true, false),
        true,
        &mut confirm,
    )
    .expect("dry run");
    assert_eq!(
        report.mappings.len(),
        1,
        "bare Header must not block resolution"
    );
    assert_eq!(report.mappings[0].ros_type, "acme_msgs/Widget");
    // Only Widget is staged — the built-in std_msgs/Header is served from the
    // corpus, never duplicated.
    assert!(
        report.report.contains("ACQUIRED SCHEMAS (1)"),
        "{}",
        report.report
    );
}

/// A bare SAME-PACKAGE reference present in the closure resolves.
#[test]
fn test_bare_same_package_ref_resolves() {
    let tmp = tempfile::tempdir().unwrap();
    let acq = canned(&[(
        "acme_msgs/Widget",
        acquired(
            AcquisitionRung::WireService,
            &[
                ("acme_msgs", "Widget", "Sub sub\nint32 id\n"),
                ("acme_msgs", "Sub", "float64 v\n"),
            ],
        ),
    )]);
    let mut confirm = panic_confirm;
    let report = ros_cmd::ros_attach_with_acquirer(
        &widget_only(),
        &acq,
        tmp.path(),
        &opts(true, false),
        true,
        &mut confirm,
    )
    .expect("dry run");
    assert_eq!(report.mappings.len(), 1);
    // Both Widget + its bare same-package Sub are staged.
    assert!(
        report.report.contains("ACQUIRED SCHEMAS (2)"),
        "{}",
        report.report
    );
}

/// An AMBIGUOUS bare suffix (two same-suffix universe members, no
/// same-package match) stays REJECTED with a reason naming the candidates.
#[test]
fn test_ambiguous_bare_suffix_rejected_naming_candidates() {
    let tmp = tempfile::tempdir().unwrap();
    let acq = canned(&[(
        "acme_msgs/Widget",
        acquired(
            AcquisitionRung::WireService,
            &[
                ("acme_msgs", "Widget", "Thing t\nint32 id\n"),
                ("foo_msgs", "Thing", "int32 v\n"),
                ("bar_msgs", "Thing", "int32 v\n"),
            ],
        ),
    )]);
    let mut confirm = panic_confirm;
    let report = ros_cmd::ros_attach_with_acquirer(
        &widget_only(),
        &acq,
        tmp.path(),
        &opts(true, false),
        true,
        &mut confirm,
    )
    .expect("dry run");
    assert!(
        report.mappings.is_empty(),
        "an ambiguous bare ref must not resolve"
    );
    assert!(report.report.contains("AMBIGUOUS"), "{}", report.report);
    assert!(
        report.report.contains("foo_msgs/Thing"),
        "{}",
        report.report
    );
    assert!(
        report.report.contains("bar_msgs/Thing"),
        "{}",
        report.report
    );
}

/// Acquirer-supplied path-traversal / empty identifiers NEVER
/// reach the file writer — validated before staging — so a hostile bundle
/// member creates NO file even under `--yes`, and the type stays UNRESOLVABLE.
#[tracing_test::traced_test]
#[test]
fn test_path_traversal_package_and_type_never_write_any_file() {
    let tmp = tempfile::tempdir().unwrap();
    // A legit root + a hostile sibling (absolute package + `..` type name).
    let acq = canned(&[(
        "acme_msgs/Widget",
        acquired(
            AcquisitionRung::WireService,
            &[
                ("acme_msgs", "Widget", WIDGET_MSG),
                ("/tmp/pwn", "../../escape", "int32 x\n"),
            ],
        ),
    )]);
    let mut confirm = panic_confirm;
    let report = ros_cmd::ros_attach_with_acquirer(
        &widget_only(),
        &acq,
        tmp.path(),
        &opts(false, true), // --yes: the silent path with no preview
        false,
        &mut confirm,
    )
    .expect("assume-yes over a hostile bundle must not error, just refuse to stage");
    // The whole bundle is untrusted → nothing resolves → nothing written.
    assert!(
        report.mappings.is_empty(),
        "a hostile bundle must not resolve"
    );
    assert!(
        !tmp.path().join("schemas").exists(),
        "no schemas/ dir may be created from a hostile bundle"
    );
    assert!(
        !tmp.path().join("graphs").exists(),
        "nothing resolvable ⇒ no graph"
    );
    assert!(
        logs_contain("failed validation"),
        "the invalid member must fire a loud warn"
    );
}

/// An empty `.msg` body is rejected (an empty schema would bridge a real
/// type as a silent zero-field message).
#[test]
fn test_empty_msg_text_rejected() {
    let tmp = tempfile::tempdir().unwrap();
    let acq = canned(&[(
        "acme_msgs/Widget",
        acquired(
            AcquisitionRung::WireService,
            &[("acme_msgs", "Widget", "   \n")],
        ),
    )]);
    let mut confirm = panic_confirm;
    let report = ros_cmd::ros_attach_with_acquirer(
        &widget_only(),
        &acq,
        tmp.path(),
        &opts(true, false),
        true,
        &mut confirm,
    )
    .expect("dry run");
    assert!(report.mappings.is_empty(), "an empty .msg must not resolve");
    assert!(report.report.contains("is empty"), "{}", report.report);
}

/// An UNSOLICITED acquisition (a type not requested) is rejected
/// loudly and NEVER staged; the requested type still resolves.
#[tracing_test::traced_test]
#[test]
fn test_unsolicited_acquisition_is_rejected() {
    /// Returns the requested Widget PLUS an unsolicited `evil_msgs/Ghost`.
    struct UnsolicitedAcquirer;
    impl SchemaAcquirer for UnsolicitedAcquirer {
        fn acquire(&self, types: &[String], _discovery: &DiscoveryResult) -> Vec<TypeAcquisition> {
            let mut out: Vec<TypeAcquisition> = types
                .iter()
                .filter(|t| *t == "acme_msgs/Widget")
                .map(|t| TypeAcquisition {
                    requested: t.clone(),
                    outcome: acquired(
                        AcquisitionRung::WireService,
                        &[("acme_msgs", "Widget", WIDGET_MSG)],
                    ),
                })
                .collect();
            out.push(TypeAcquisition {
                requested: "evil_msgs/Ghost".to_string(),
                outcome: acquired(
                    AcquisitionRung::PublicLookup,
                    &[("evil_msgs", "Ghost", "int32 g\n")],
                ),
            });
            out
        }
    }
    let tmp = tempfile::tempdir().unwrap();
    let mut confirm = panic_confirm;
    let report = ros_cmd::ros_attach_with_acquirer(
        &widget_only(),
        &UnsolicitedAcquirer,
        tmp.path(),
        &opts(true, false),
        true,
        &mut confirm,
    )
    .expect("dry run");
    // Widget resolves; Ghost never staged (no evil_msgs in the section).
    assert_eq!(report.mappings.len(), 1);
    assert!(
        report.report.contains("ACQUIRED SCHEMAS (1)"),
        "{}",
        report.report
    );
    assert!(!report.report.contains("evil_msgs"), "{}", report.report);
    assert!(
        logs_contain("not requested"),
        "the unsolicited type must fire a loud warn"
    );
}

/// A DUPLICATE requested key keeps the first result and warns
/// on the discard (never a silent first-win).
#[tracing_test::traced_test]
#[test]
fn test_duplicate_requested_first_wins_with_warn() {
    /// Emits TWO results for the SAME requested type (first must win).
    struct DupRequestedAcquirer;
    impl SchemaAcquirer for DupRequestedAcquirer {
        fn acquire(&self, types: &[String], _discovery: &DiscoveryResult) -> Vec<TypeAcquisition> {
            let mut out = Vec::new();
            for t in types {
                if t == "acme_msgs/Widget" {
                    out.push(TypeAcquisition {
                        requested: t.clone(),
                        outcome: acquired(
                            AcquisitionRung::WireService,
                            &[("acme_msgs", "Widget", WIDGET_MSG)],
                        ),
                    });
                    out.push(TypeAcquisition {
                        requested: t.clone(),
                        outcome: acquired(
                            AcquisitionRung::PublicLookup,
                            &[("acme_msgs", "Widget", "int32 SHOULD_NOT_WIN\n")],
                        ),
                    });
                }
            }
            out
        }
    }
    let tmp = tempfile::tempdir().unwrap();
    let mut confirm = panic_confirm;
    let report = ros_cmd::ros_attach_with_acquirer(
        &widget_only(),
        &DupRequestedAcquirer,
        tmp.path(),
        &opts(false, true), // --yes to inspect the materialized bytes
        false,
        &mut confirm,
    )
    .expect("assume-yes write");
    assert!(matches!(report.outcome, AttachOutcome::Written { .. }));
    let dest = tmp
        .path()
        .join("schemas")
        .join("acme_msgs")
        .join("msg")
        .join("Widget.msg");
    // The FIRST result won byte-verbatim.
    assert_eq!(read(&dest), WIDGET_MSG);
    assert!(
        logs_contain("duplicate results"),
        "the discarded duplicate must fire a loud warn"
    );
}

/// Two acquired bundles carrying the SAME nested type with
/// BYTE-DIFFERENT text REFUSE both requested types loudly — neither divergent
/// copy ships (deterministic, never first-wins-silent).
#[tracing_test::traced_test]
#[test]
fn test_conflicting_texts_refuse_both_types() {
    let tmp = tempfile::tempdir().unwrap();
    let acq = canned(&[
        (
            "acme_msgs/A",
            acquired(
                AcquisitionRung::WireService,
                &[
                    ("acme_msgs", "A", "shared_msgs/Dep d\nint32 a\n"),
                    ("shared_msgs", "Dep", "float64 x\n"),
                ],
            ),
        ),
        (
            "acme_msgs/B",
            acquired(
                AcquisitionRung::PublicLookup,
                &[
                    ("acme_msgs", "B", "shared_msgs/Dep d\nint32 b\n"),
                    ("shared_msgs", "Dep", "float64 y\n"), // BYTE-DIFFERENT
                ],
            ),
        ),
    ]);
    let disc = FakeDiscovery::ok(vec![
        ep(
            "rt/a",
            "acme_msgs::msg::dds_::A_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/b",
            "acme_msgs::msg::dds_::B_",
            EndpointKind::Writer,
            BE_VOL,
        ),
    ]);
    let mut confirm = panic_confirm;
    let report = ros_cmd::ros_attach_with_acquirer(
        &disc,
        &acq,
        tmp.path(),
        &opts(false, true), // --yes — must still write NOTHING for the conflict
        false,
        &mut confirm,
    )
    .expect("assume-yes over a conflicting pair refuses to stage, not error");
    // Both types refused ⇒ nothing resolvable ⇒ nothing written.
    assert!(
        report.mappings.is_empty(),
        "both conflicting types must be refused"
    );
    assert!(
        !tmp.path().join("schemas").exists(),
        "a byte-different conflict must materialize no schema"
    );
    assert!(report.report.contains("REFUSED"), "{}", report.report);
    assert!(
        report.report.contains("shared_msgs/Dep"),
        "{}",
        report.report
    );
    assert!(
        logs_contain("byte-different schema conflict"),
        "the conflict must fire a loud warn"
    );
}

/// Byte-IDENTICAL duplicate members across bundles dedupe QUIETLY — both
/// types resolve, the shared dep is written ONCE, no conflict.
#[test]
fn test_identical_duplicate_texts_dedupe_quietly() {
    let tmp = tempfile::tempdir().unwrap();
    let acq = canned(&[
        (
            "acme_msgs/A",
            acquired(
                AcquisitionRung::WireService,
                &[
                    ("acme_msgs", "A", "shared_msgs/Dep d\nint32 a\n"),
                    ("shared_msgs", "Dep", "float64 x\n"),
                ],
            ),
        ),
        (
            "acme_msgs/B",
            acquired(
                AcquisitionRung::WireService,
                &[
                    ("acme_msgs", "B", "shared_msgs/Dep d\nint32 b\n"),
                    ("shared_msgs", "Dep", "float64 x\n"), // IDENTICAL
                ],
            ),
        ),
    ]);
    let disc = FakeDiscovery::ok(vec![
        ep(
            "rt/a",
            "acme_msgs::msg::dds_::A_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/b",
            "acme_msgs::msg::dds_::B_",
            EndpointKind::Writer,
            BE_VOL,
        ),
    ]);
    let mut confirm = panic_confirm;
    let report = ros_cmd::ros_attach_with_acquirer(
        &disc,
        &acq,
        tmp.path(),
        &opts(true, false),
        true,
        &mut confirm,
    )
    .expect("dry run");
    assert_eq!(report.mappings.len(), 2, "both types resolve");
    assert!(!report.report.contains("REFUSED"), "{}", report.report);
    // A + B + ONE shared Dep = 3 files (Dep deduped).
    assert!(
        report.report.contains("ACQUIRED SCHEMAS (3)"),
        "{}",
        report.report
    );
}

/// An acquired closure member that COLLIDES with a built-in by name
/// but carries DIFFERENT bytes warns loudly — the authoritative built-in wins
/// (the divergent variant is discarded, never materialized).
#[tracing_test::traced_test]
#[test]
fn test_builtin_divergent_member_warns_and_uses_builtin() {
    let tmp = tempfile::tempdir().unwrap();
    let acq = canned(&[(
        "acme_msgs/Widget",
        acquired(
            AcquisitionRung::WireService,
            &[
                ("acme_msgs", "Widget", "std_msgs/Header header\nint32 id\n"),
                ("std_msgs", "Header", "int32 WRONG_LAYOUT\n"), // divergent builtin
            ],
        ),
    )]);
    let mut confirm = panic_confirm;
    let report = ros_cmd::ros_attach_with_acquirer(
        &widget_only(),
        &acq,
        tmp.path(),
        &opts(false, true), // --yes
        false,
        &mut confirm,
    )
    .expect("assume-yes write");
    // Widget resolves (Header served from the built-in corpus).
    assert_eq!(report.mappings.len(), 1);
    // The divergent built-in Header is NEVER materialized.
    assert!(
        !tmp.path().join("schemas").join("std_msgs").exists(),
        "the authoritative built-in must not be overwritten by the acquired variant"
    );
    assert!(
        logs_contain("differs from the vendored ROS 2 corpus"),
        "the built-in divergence must warn loudly"
    );
}

/// Schemas are written BEFORE the graph/config; a mid-batch
/// schema-write failure names the exact path, states nothing was written yet,
/// and leaves the graph/config ABSENT (the order pin).
#[test]
fn test_mid_batch_write_failure_names_path_and_leaves_graph_absent() {
    let tmp = tempfile::tempdir().unwrap();
    // Make schemas/acme_msgs a FILE so create_dir_all for the .msg parent fails.
    std::fs::create_dir_all(tmp.path().join("schemas")).unwrap();
    std::fs::write(tmp.path().join("schemas").join("acme_msgs"), b"not a dir").unwrap();
    let acq = canned(&[(
        "acme_msgs/Widget",
        acquired(
            AcquisitionRung::WireService,
            &[("acme_msgs", "Widget", WIDGET_MSG)],
        ),
    )]);
    let mut confirm = panic_confirm;
    let err = ros_cmd::ros_attach_with_acquirer(
        &widget_only(),
        &acq,
        tmp.path(),
        &opts(false, true), // --yes
        false,
        &mut confirm,
    )
    .expect_err("a schema-write failure must surface a loud, accountable error");
    let msg = err.to_string();
    assert!(
        msg.contains("Widget.msg"),
        "error must name the failing path: {msg}"
    );
    assert!(
        msg.contains("no files were written before this failure"),
        "schemas are written first, so nothing landed: {msg}"
    );
    // Order pin: the graph/config are written AFTER schemas, so a failed schema
    // write leaves neither behind.
    assert!(
        !tmp.path().join("graphs").join("attach.yaml").exists(),
        "the graph must NOT be written when the schema write fails first"
    );
    assert!(
        !tmp.path()
            .join("graphs")
            .join("attach.bridge.yaml")
            .exists(),
        "the config must NOT be written when the schema write fails first"
    );
}

/// The INTERACTIVE consent path — confirm(yes) materializes ALL THREE
/// write classes and the preview NAMES the `.msg` file + shows its verbatim body.
#[test]
fn test_interactive_confirm_yes_writes_all_three_classes() {
    let tmp = tempfile::tempdir().unwrap();
    let acq = canned(&[(
        "acme_msgs/Widget",
        acquired(
            AcquisitionRung::WireService,
            &[("acme_msgs", "Widget", WIDGET_MSG)],
        ),
    )]);
    let mut confirm = accept_confirm;
    let report = ros_cmd::ros_attach_with_acquirer(
        &widget_only(),
        &acq,
        tmp.path(),
        &opts(false, false), // interactive
        true,                // TTY
        &mut confirm,
    )
    .expect("interactive accept");
    assert!(report.preview_shown, "the preview must have been shown");
    assert!(matches!(report.outcome, AttachOutcome::Written { .. }));
    // The preview names the schema file AND shows its verbatim body.
    assert!(
        report.preview.contains(
            "── schemas/acme_msgs/msg/Widget.msg (acquired schema, wire service via \
             get_type_description)"
        ),
        "{}",
        report.preview
    );
    assert!(report.preview.contains(WIDGET_MSG), "{}", report.preview);
    // All three write classes landed.
    assert!(tmp
        .path()
        .join("schemas")
        .join("acme_msgs")
        .join("msg")
        .join("Widget.msg")
        .exists());
    assert!(tmp.path().join("graphs").join("attach.yaml").exists());
    assert!(tmp
        .path()
        .join("graphs")
        .join("attach.bridge.yaml")
        .exists());
}

/// The INTERACTIVE consent path — confirm(no) writes NOTHING, including
/// no schema.
#[test]
fn test_interactive_confirm_no_writes_nothing() {
    let tmp = tempfile::tempdir().unwrap();
    let acq = canned(&[(
        "acme_msgs/Widget",
        acquired(
            AcquisitionRung::WireService,
            &[("acme_msgs", "Widget", WIDGET_MSG)],
        ),
    )]);
    let mut confirm = decline_confirm;
    let report = ros_cmd::ros_attach_with_acquirer(
        &widget_only(),
        &acq,
        tmp.path(),
        &opts(false, false),
        true,
        &mut confirm,
    )
    .expect("interactive decline");
    assert_eq!(report.outcome, AttachOutcome::Declined);
    assert!(
        !tmp.path().join("schemas").exists(),
        "decline writes no schema"
    );
    assert!(
        !tmp.path().join("graphs").exists(),
        "decline writes no graph/config"
    );
}

/// A pre-existing (UNPARSEABLE) store file at the dest is backed up to
/// `.msg.bak` before the acquired schema overwrites it (the `.bak` arm).
#[test]
fn test_preexisting_store_file_is_backed_up_on_overwrite() {
    let tmp = tempfile::tempdir().unwrap();
    // An UNPARSEABLE store file: the chain does NOT resolve it, so the type is
    // still unresolvable → acquired → staged → overwrites + backs up the file.
    const STALE: &str = "int32\n"; // single token = hard parse error
    write_store_msg(tmp.path(), "acme_msgs", "Widget", STALE);
    let acq = canned(&[(
        "acme_msgs/Widget",
        acquired(
            AcquisitionRung::WireService,
            &[("acme_msgs", "Widget", WIDGET_MSG)],
        ),
    )]);
    let mut confirm = panic_confirm;
    let report = ros_cmd::ros_attach_with_acquirer(
        &widget_only(),
        &acq,
        tmp.path(),
        &opts(false, true), // --yes
        false,
        &mut confirm,
    )
    .expect("assume-yes write");
    let dest = tmp
        .path()
        .join("schemas")
        .join("acme_msgs")
        .join("msg")
        .join("Widget.msg");
    // New bytes landed; the old (unparseable) bytes were backed up.
    assert_eq!(read(&dest), WIDGET_MSG);
    match &report.outcome {
        AttachOutcome::Written { schema_writes, .. } => {
            assert_eq!(schema_writes.len(), 1);
            let backup = schema_writes[0]
                .backup
                .as_ref()
                .expect("a pre-existing store file must be backed up");
            assert_eq!(read(backup), STALE, "the .bak must hold the old bytes");
        }
        other => panic!("expected Written, got {other:?}"),
    }
}

// ───── The wire rung PREPENDS the local rung (ladder order) ──

/// Write `<prefix>/share/<pkg>/msg/<ty>.msg` + append the pkg's ament index
/// marker — a minimal fake ROS install for the local rung.
fn write_ament_msg(prefix: &Path, pkg: &str, ty: &str, text: &str) {
    let msg_dir = prefix.join("share").join(pkg).join("msg");
    std::fs::create_dir_all(&msg_dir).unwrap();
    std::fs::write(msg_dir.join(format!("{ty}.msg")), text).unwrap();
    let idx_dir = prefix
        .join("share")
        .join("ament_index")
        .join("resource_index")
        .join("rosidl_interfaces");
    std::fs::create_dir_all(&idx_dir).unwrap();
    let marker = idx_dir.join(pkg);
    let mut content = std::fs::read_to_string(&marker).unwrap_or_default();
    content.push_str(&format!("msg/{ty}.msg\n"));
    std::fs::write(&marker, content).unwrap();
}

/// Through the REAL [`ros_cmd::AttachAcquirers`] the wire rung
/// is tried BEFORE the local ament rung. Two pins in one flow: (1) a type BOTH
/// could serve (`acme_msgs/Widget`) is acquired by the WIRE rung — the written
/// `.msg` is the wire text, not the local install's different text (batch
/// semantics remove it before local ever runs); (2) a type NEITHER can serve
/// (`other_msgs/Gadget`) accumulates BOTH skip reasons in ladder order — the
/// wire skip precedes the local skip.
#[test]
fn test_wire_rung_prepends_local_ament_in_the_ladder() {
    const WIRE_WIDGET: &str = "# from the wire\nint32 id\nfloat64 value\n";
    const LOCAL_WIDGET: &str = "# from the local install (must NOT win)\nint32 stale\n";

    let ws = tempfile::tempdir().unwrap();
    let install = tempfile::tempdir().unwrap();
    // The local install ALSO carries Widget — so the WIN is genuine, not a
    // walkover (the wire rung must preempt it).
    write_ament_msg(install.path(), "acme_msgs", "Widget", LOCAL_WIDGET);

    // Wire rung (canned): acquires Widget with ITS text, SKIPS Gadget (no hash).
    let wire = canned(&[
        (
            "acme_msgs/Widget",
            acquired(
                AcquisitionRung::WireService,
                &[("acme_msgs", "Widget", WIRE_WIDGET)],
            ),
        ),
        (
            "other_msgs/Gadget",
            skipped(&[(
                AcquisitionRung::WireService,
                "no type hash in discovery (pre-Iron distro or non-ROS DDS publisher)",
            )]),
        ),
    ]);
    let local = LocalAmentAcquirer::new(vec![install.path().to_path_buf()]);
    // Struct-literal composition (the hermetic seam — `with_wire` was deleted
    // so that production has exactly one wire-bearing
    // constructor, `production()`).
    let acquirers = ros_cmd::AttachAcquirers {
        wire: Some(Box::new(wire)),
        local_ament: local,
    };
    let chain = acquirers.chain();

    let disc = FakeDiscovery::ok(vec![
        ep(
            "rt/widget",
            "acme_msgs::msg::dds_::Widget_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/gadget",
            "other_msgs::msg::dds_::Gadget_",
            EndpointKind::Writer,
            BE_VOL,
        ),
    ]);
    let mut confirm = accept_confirm;
    let report = ros_cmd::ros_attach_with_acquirer(
        &disc,
        &chain,
        ws.path(),
        &opts(false, true), // --yes: write so we can read the winning .msg text
        false,
        &mut confirm,
    )
    .expect("assume-yes write");

    // (1) Widget resolved, and the written .msg is the WIRE text — wire beat the
    // local install (which never got to serve it).
    match &report.outcome {
        AttachOutcome::Written { schema_writes, .. } => {
            let widget = schema_writes
                .iter()
                .find(|w| w.path.ends_with("acme_msgs/msg/Widget.msg"))
                .expect("Widget.msg must be written");
            assert_eq!(
                std::fs::read_to_string(&widget.path).unwrap(),
                WIRE_WIDGET,
                "the wire rung's text must win over the local install's"
            );
        }
        other => panic!("expected Written, got {other:?}"),
    }

    // (2) Gadget unresolved: the report accumulates BOTH skip reasons, wire first.
    let r = &report.report;
    let wire_skip = r
        .find("wire service via get_type_description: no type hash in discovery")
        .expect("wire skip line present");
    let local_skip = r
        .find("local ROS install via ament index: other_msgs not found in any ament prefix")
        .expect("local skip line present");
    assert!(
        wire_skip < local_skip,
        "the wire skip reason must precede the local one (ladder order)\n{r}"
    );
}

/// The acceptance one-byte diff: rcl's REP-2011
/// `type_sources[].raw_file_contents` embeds the `.msg` text WITHOUT the final
/// newline, so a wire payload materialized literally produced
/// no-newline-at-EOF store files. E2E pin over the real write path: a
/// newline-LESS canned wire payload materializes as the payload plus EXACTLY
/// one restored `'\n'` — content-identical, single-final-newline form (note:
/// a robot original ending in EXTRA blank lines is normalized — rcl strips
/// exactly one final newline, so that distinction never survives the wire). The
/// no-doubling twin is `test_wire_rung_prepends_local_ament_in_the_ladder`,
/// whose already-newlined `WIRE_WIDGET` is asserted byte-UNCHANGED on disk;
/// the rule's full oracle vector is
/// `ros_cmd::tests::restore_trailing_newline_oracle`.
#[test]
fn test_wire_payload_without_trailing_newline_materializes_posix_form() {
    // The live shape: the payload ends `...string label` BARE.
    const BARE_WIRE: &str = "# probe schema\nuint32 seq\nstring label";

    let ws = tempfile::tempdir().unwrap();
    let wire = canned(&[(
        "wire_msgs/Bare",
        acquired(
            AcquisitionRung::WireService,
            &[("wire_msgs", "Bare", BARE_WIRE)],
        ),
    )]);
    let acquirers = ros_cmd::AttachAcquirers {
        wire: Some(Box::new(wire)),
        local_ament: LocalAmentAcquirer::new(vec![]),
    };
    let chain = acquirers.chain();

    let disc = FakeDiscovery::ok(vec![ep(
        "rt/bare",
        "wire_msgs::msg::dds_::Bare_",
        EndpointKind::Writer,
        BE_VOL,
    )]);
    let mut confirm = accept_confirm;
    let report = ros_cmd::ros_attach_with_acquirer(
        &disc,
        &chain,
        ws.path(),
        &opts(false, true), // --yes: write so the on-disk bytes are assertable
        false,
        &mut confirm,
    )
    .expect("assume-yes write");

    match &report.outcome {
        AttachOutcome::Written { schema_writes, .. } => {
            let bare = schema_writes
                .iter()
                .find(|w| w.path.ends_with("wire_msgs/msg/Bare.msg"))
                .expect("Bare.msg must be written");
            let on_disk = std::fs::read_to_string(&bare.path).unwrap();
            assert_eq!(
                on_disk,
                format!("{BARE_WIRE}\n"),
                "the materialized file is the wire payload + exactly ONE restored newline"
            );
            assert!(
                !on_disk.ends_with("\n\n"),
                "never a doubled trailing newline"
            );
        }
        other => panic!("expected Written, got {other:?}"),
    }
}

/// The seam-placement discriminator: a
/// MIXED-RUNG attach where a wire bundle and a local-ament bundle share nested
/// members must dedupe QUIETLY, never refuse. Two shared deps pin two arms:
/// `DepOne` — wire payload bare (`float64 x`) vs ament `float64 x\n` — the
/// restore-at-STAGING seam makes them byte-equal before the duplicate compare
/// (restore at the fs-write instead and this arm REFUSES on a phantom
/// newline conflict); `DepTwo` — ament BLANK-LINE-terminated `float64 y\n\n`
/// (the `std_msgs/Header`-style original rcl's one-newline strip can never
/// round-trip) vs wire-restored `float64 y\n` — bytes still differ post-restore,
/// so only the newline-form-insensitive compare
/// (`eq_ignoring_trailing_newlines`) dedupes it. A GENUINELY content-different
/// shared member still refuses — pinned by the untouched
/// `test_conflicting_texts_refuse_both_types` (`float64 x` vs `float64 y`).
#[test]
fn test_mixed_rung_shared_member_newline_deltas_dedupe_quietly() {
    let ws = tempfile::tempdir().unwrap();
    let install = tempfile::tempdir().unwrap();
    // The local install serves B + both deps (DepTwo blank-line-terminated).
    write_ament_msg(
        install.path(),
        "acme_msgs",
        "B",
        "shared_msgs/DepOne one\nshared_msgs/DepTwo two\nint32 b\n",
    );
    write_ament_msg(install.path(), "shared_msgs", "DepOne", "float64 x\n");
    write_ament_msg(install.path(), "shared_msgs", "DepTwo", "float64 y\n\n");

    // The wire rung serves A + both deps, every payload newline-LESS (the
    // live rcl shape); it does not attempt B.
    let wire = canned(&[(
        "acme_msgs/A",
        acquired(
            AcquisitionRung::WireService,
            &[
                (
                    "acme_msgs",
                    "A",
                    "shared_msgs/DepOne one\nshared_msgs/DepTwo two\nint32 a",
                ),
                ("shared_msgs", "DepOne", "float64 x"),
                ("shared_msgs", "DepTwo", "float64 y"),
            ],
        ),
    )]);
    let acquirers = ros_cmd::AttachAcquirers {
        wire: Some(Box::new(wire)),
        local_ament: LocalAmentAcquirer::new(vec![install.path().to_path_buf()]),
    };
    let chain = acquirers.chain();

    let disc = FakeDiscovery::ok(vec![
        ep(
            "rt/a",
            "acme_msgs::msg::dds_::A_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/b",
            "acme_msgs::msg::dds_::B_",
            EndpointKind::Writer,
            BE_VOL,
        ),
    ]);
    let mut confirm = accept_confirm;
    let report = ros_cmd::ros_attach_with_acquirer(
        &disc,
        &chain,
        ws.path(),
        &opts(false, true), // --yes: write so the deduped store is assertable
        false,
        &mut confirm,
    )
    .expect("mixed-rung newline deltas must attach cleanly");

    // No refusal anywhere: both roots resolvable, nothing REFUSED.
    assert!(
        !report.report.contains("REFUSED"),
        "newline-form deltas must not refuse:\n{}",
        report.report
    );
    match &report.outcome {
        AttachOutcome::Written { schema_writes, .. } => {
            // Each shared dep written EXACTLY once…
            for dep in ["DepOne", "DepTwo"] {
                let hits = schema_writes
                    .iter()
                    .filter(|w| {
                        w.path
                            .ends_with(format!("shared_msgs/msg/{dep}.msg").as_str())
                    })
                    .count();
                assert_eq!(hits, 1, "{dep} must be staged/written exactly once");
            }
            // …with the agreed CONTENT (newline form = whichever bundle staged
            // first — first-seen wins as ever; content is what's pinned).
            let read = |ty: &str| {
                std::fs::read_to_string(
                    ws.path()
                        .join("schemas")
                        .join("shared_msgs")
                        .join("msg")
                        .join(format!("{ty}.msg")),
                )
                .unwrap()
            };
            assert_eq!(read("DepOne").trim_end_matches('\n'), "float64 x");
            assert_eq!(read("DepTwo").trim_end_matches('\n'), "float64 y");
        }
        other => panic!("expected Written, got {other:?}"),
    }
}

/// `AttachAcquirers::rungs` prepends the wire rung when
/// installed, and is the local rung alone when not (pure composition pin).
#[test]
fn test_attach_acquirers_rungs_prepends_wire_when_installed() {
    let base = ros_cmd::AttachAcquirers::with_local_ament(LocalAmentAcquirer::new(vec![]));
    assert_eq!(base.rungs().len(), 1, "no wire rung ⇒ local rung alone");

    // A marker wire rung that acquires a distinctive type — proves it is FIRST.
    let wire = canned(&[(
        "acme/Marker",
        acquired(
            AcquisitionRung::WireService,
            &[("acme", "Marker", "int32 x\n")],
        ),
    )]);
    let withwire = ros_cmd::AttachAcquirers {
        wire: Some(Box::new(wire)),
        local_ament: LocalAmentAcquirer::new(vec![]),
    };
    assert_eq!(withwire.rungs().len(), 2, "wire rung ⇒ two rungs");
    // The first rung is the wire rung: it acquires acme/Marker, and the chain
    // returns it as WireService (the local rung has no prefixes → would skip).
    let out = withwire
        .chain()
        .acquire(&["acme/Marker".to_string()], &DiscoveryResult::empty());
    assert_eq!(out.len(), 1);
    assert!(matches!(
        &out[0].outcome,
        AcquisitionOutcome::Acquired(s) if s.rung == AcquisitionRung::WireService
    ));
}

/// The PRODUCTION composition root
/// (`AttachAcquirers::production`, the constructor `cerulion_cli`'s `main.rs`
/// calls) PREPENDS the required wire rung to the env-derived local ament rung.
/// Removing the wire rung from a `production()` call is a compile error at the
/// required argument, and no wire-less named constructor (`from_env` /
/// `with_wire`) exists for a one-line main.rs edit to reach — the only
/// wire-less paths are the documented hermetic test seams (`with_local_ament` /
/// a struct literal). (`cerulion_cli` has no lib target so `main.rs` is not
/// unit-testable directly; this is the strongest shape short of a binary e2e.)
#[test]
fn test_production_acquirers_prepend_wire_rung() {
    // A marker wire rung that acquires a distinctive type — proves it is FIRST.
    let wire = canned(&[(
        "acme/Marker",
        acquired(
            AcquisitionRung::WireService,
            &[("acme", "Marker", "int32 x\n")],
        ),
    )]);
    let acquirers = ros_cmd::AttachAcquirers::production(Box::new(wire));
    // Two rungs: the wire rung FIRST, then the (env-derived) local ament rung.
    assert_eq!(
        acquirers.rungs().len(),
        2,
        "production must carry the wire rung + the local ament rung"
    );
    // The wire rung is first: it acquires acme/Marker via WireService (the local
    // ament rung would have no such marker type to serve).
    let out = acquirers
        .chain()
        .acquire(&["acme/Marker".to_string()], &DiscoveryResult::empty());
    assert_eq!(out.len(), 1);
    assert!(
        matches!(
            &out[0].outcome,
            AcquisitionOutcome::Acquired(s) if s.rung == AcquisitionRung::WireService
        ),
        "the wire rung must be tried FIRST in the production ladder"
    );
}

// ──────────────────────── The lockstep pin ────────────────────────

/// Extract every Rust string-literal CONTENT (escape-aware) from `src`, in
/// order. Captures the bytes between unescaped `"` delimiters; a backslash
/// escapes the following char (so `\n` / `\"` never terminate a literal). Used
/// by the T1 lockstep pin to read the demo bridge's `UNITREE_MSGS` tuples out of
/// its source without depending on the isolated demo crate.
fn string_literals(src: &str) -> Vec<String> {
    let chars: Vec<char> = src.chars().collect();
    let mut out = Vec::new();
    let mut i = 0;
    while i < chars.len() {
        if chars[i] != '"' {
            i += 1;
            continue;
        }
        i += 1; // consume the opening quote
        let mut lit = String::new();
        while i < chars.len() {
            let nc = chars[i];
            i += 1;
            if nc == '\\' {
                // Escaped char: keep it verbatim (we only read pkg/name, which
                // carry no escapes; this just prevents `\"` from closing early).
                if i < chars.len() {
                    lit.push('\\');
                    lit.push(chars[i]);
                    i += 1;
                }
                continue;
            }
            if nc == '"' {
                break;
            }
            lit.push(nc);
        }
        out.push(lit);
    }
    out
}

/// The lockstep pin (against mirror drift and a stale real
/// artifact): read the REAL demo bridge sources and pin the engine's mirrors to
/// them, so a bridge-side change fails the ENGINE suite naming both files.
///
/// (a) Parse `examples/go2/nodes/dds_bridge/src/generic.rs`'s `UNITREE_MSGS` const
/// (each tuple is `(pkg, name, text)`) into a `pkg/name` set and assert SET-EQUAL
/// to [`ros_cmd::bridge_unitree_msg_names`] — an add / retire / rename in the
/// bridge fails here.
///
/// (b) Assert `examples/go2/nodes/dds_bridge/src/config.rs` declares `RouteMode`
/// (the probe marker) AND that a byte-copy of the REAL config.rs in a temp
/// workspace probes as route-mode-SUPPORTING via
/// [`ros_cmd::vendored_bridge_supports_route_mode`] — a marker rename in the demo
/// crate fails loudly.
#[test]
fn bridge_mirrors_are_lockstep_with_the_demo_crate_sources() {
    let manifest = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let bridge_src = manifest.join("../../examples/go2/nodes/dds_bridge/src");

    // (a) UNITREE_MSGS set-equality.
    let generic = std::fs::read_to_string(bridge_src.join("generic.rs"))
        .expect("read examples/go2/nodes/dds_bridge/src/generic.rs");
    let start = generic
        .find("const UNITREE_MSGS")
        .expect("generic.rs must declare `const UNITREE_MSGS`");
    let rest = &generic[start..];
    // The array is `= &[ ... ];`; no tuple text contains `];`, so the first `];`
    // after the const terminates the block.
    let end = rest
        .find("];")
        .expect("the UNITREE_MSGS array must terminate with `];`")
        + 2;
    let lits = string_literals(&rest[..end]);
    assert_eq!(
        lits.len() % 3,
        0,
        "each UNITREE_MSGS entry is a (pkg, name, text) tuple — got {} literals: {lits:?}",
        lits.len()
    );
    assert!(!lits.is_empty(), "UNITREE_MSGS must not be empty");
    let parsed: std::collections::BTreeSet<String> = lits
        .chunks(3)
        .map(|c| format!("{}/{}", c[0], c[1]))
        .collect();
    let engine: std::collections::BTreeSet<String> = ros_cmd::bridge_unitree_msg_names()
        .iter()
        .map(|s| s.to_string())
        .collect();
    assert_eq!(
        parsed, engine,
        "BRIDGE_UNITREE_MSG_NAMES (cerulion_cli_engine/src/ros_cmd.rs) drifted from the bridge's \
         UNITREE_MSGS (examples/go2/nodes/dds_bridge/src/generic.rs) — an add / retire / rename in \
         one requires the same in the other"
    );

    // (b) The real config.rs declares the RouteMode marker, and a byte-copy of
    // it probes as SUPPORTS.
    let config_src = std::fs::read_to_string(bridge_src.join("config.rs"))
        .expect("read examples/go2/nodes/dds_bridge/src/config.rs");
    assert!(
        config_src.contains("RouteMode"),
        "the REAL demo config.rs must declare `RouteMode` (the probe marker)"
    );
    let tmp = tempfile::tempdir().unwrap();
    write_vendored_bridge_config_rs(tmp.path(), &config_src);
    assert!(
        ros_cmd::vendored_bridge_supports_route_mode(tmp.path()),
        "a byte-copy of the REAL config.rs must probe as route-mode-supporting"
    );
}

/// The config.rs-UNREADABLE fail-CLOSED arm of
/// [`ros_cmd::vendored_bridge_supports_route_mode`]. The vendored `dds_bridge`
/// dir EXISTS but its `src/config.rs` is missing (a half-copied bridge) — the
/// probe must fail CLOSED (`false`, loudly) so an older cdylib never gets a
/// `route: raw` it would reject wholesale. This is the arm a fail-OPEN
/// probe (`Err(_) => true`) would silently break — the rest of the suite
/// passes even with that regression, so this is the ONLY guard on it. The positive-
/// control twin writes a RouteMode-bearing config.rs into the SAME dir ⇒ `true`,
/// proving the `false` came from the missing FILE, not the dir shape.
#[tracing_test::traced_test]
#[test]
fn vendored_bridge_dir_present_but_config_unreadable_fails_closed() {
    let tmp = tempfile::tempdir().unwrap();
    // nodes/dds_bridge/src exists, but NO config.rs inside it.
    std::fs::create_dir_all(tmp.path().join("nodes").join("dds_bridge").join("src")).unwrap();
    assert!(
        !ros_cmd::vendored_bridge_supports_route_mode(tmp.path()),
        "a vendored bridge dir with a missing/unreadable config.rs must fail CLOSED"
    );
    assert!(
        logs_contain("could not be read"),
        "the fail-closed arm must warn LOUDLY that config.rs could not be read"
    );

    // Positive control: the SAME dir shape, WITH a RouteMode-bearing
    // config.rs ⇒ true (the false above came from the missing file, not the dir).
    write_vendored_bridge_config_rs(
        tmp.path(),
        "// newer vendored copy: declares RouteMode\npub enum RouteMode { Auto, Raw }\n",
    );
    assert!(
        ros_cmd::vendored_bridge_supports_route_mode(tmp.path()),
        "the same dir with a RouteMode-bearing config.rs must probe as SUPPORTS"
    );
}

/// The non-dir FILE arm of
/// [`ros_cmd::vendored_bridge_supports_route_mode`]. `nodes/dds_bridge` EXISTS
/// but is a stray plain FILE (not a directory) — a weird half-state — so the
/// probe must fail CLOSED (`false`, loudly) rather than risk a `route: raw` an
/// older cdylib would reject wholesale. A separate `#[traced_test]` fn (not an
/// arm of the config-unreadable test) keeps the log capture isolated to this one
/// warn, and a fresh tempdir avoids the dir-shape planted by that test.
///
/// The unique warn substring `"is NOT a directory"` is what makes this arm
/// mutation-proof: replacing `Ok(_) => { warn!(..); return false }` with
/// `Ok(_) => {}` would STILL return false (fall-through: `config.rs` read →
/// `NotFound`), but with the WRONG warn ("could not be read"), so asserting the
/// specific substring — not just the `false` — kills that swap.
///
/// (The EACCES/stat-`Err` branch stays untested — it needs root games to force a
/// non-`NotFound` metadata error — matching the config.rs arm's precedent.)
#[tracing_test::traced_test]
#[test]
fn vendored_bridge_present_as_a_file_not_a_dir_fails_closed() {
    let tmp = tempfile::tempdir().unwrap();
    // Plant `nodes/dds_bridge` as a plain FILE (nodes/ dir first).
    std::fs::create_dir_all(tmp.path().join("nodes")).unwrap();
    std::fs::write(
        tmp.path().join("nodes").join("dds_bridge"),
        b"not a directory\n",
    )
    .unwrap();
    assert!(
        !ros_cmd::vendored_bridge_supports_route_mode(tmp.path()),
        "a stray FILE named nodes/dds_bridge must fail CLOSED"
    );
    assert!(
        logs_contain("is NOT a directory"),
        "the non-dir arm must warn LOUDLY that nodes/dds_bridge is NOT a directory"
    );
}

// ───────────────── Migration report (pure + seam) ─────────────────
//
// The MIGRATION section prints AUTOMATICALLY on every attach run (by
// design, no flag): pure rendering over the node table + endpoints + the two
// resolvability sets the flow already holds. Every test below asserts against
// a HAND-WRITTEN oracle (never a self-compare); the full-text oracle is the
// mutation kill for each group arm (deleting an arm fails it byte-for-byte).

use cerulion_cli_engine::ros_cmd::{render_migration_report, MigrationReportInputs};
use std::collections::BTreeSet;

/// An endpoint carrying its 16-byte GUID (prefix + entity id) — the join key
/// the migration attribution uses.
fn ep_owned(
    dds_topic: &str,
    type_name: &str,
    kind: EndpointKind,
    prefix: [u8; 12],
    entity: u8,
) -> DiscoveredEndpoint {
    let mut guid = [0u8; 16];
    guid[..12].copy_from_slice(&prefix);
    guid[15] = entity;
    DiscoveredEndpoint {
        dds_topic: dds_topic.to_string(),
        type_name: type_name.to_string(),
        qos: BE_VOL,
        kind,
        type_hash: None,
        writer_guid: Some(guid),
    }
}

fn node(namespace: &str, name: &str, prefix: [u8; 12]) -> DiscoveredNode {
    DiscoveredNode {
        namespace: namespace.to_string(),
        name: name.to_string(),
        participant_prefix: prefix,
    }
}

fn set(items: &[&str]) -> BTreeSet<String> {
    items.iter().map(|s| s.to_string()).collect()
}

const PART_A: [u8; 12] = [1; 12];
const PART_B: [u8; 12] = [2; 12];
const PART_C: [u8; 12] = [3; 12];
const PART_D: [u8; 12] = [4; 12];
/// A participant prefix that matches NO node-table entry (a vendor process
/// whose endpoints carry GUIDs but that never published ros_discovery_info).
const PART_VENDOR: [u8; 12] = [9; 12];

/// The all-groups fixture: one restartable-today process, one
/// restartable-after-materialization, one blocked by an unresolvable type,
/// one with no message endpoints, plus two orphan (no-owning-node) topics —
/// with ROS plumbing endpoints (rosout + parameter services) mixed in to
/// prove the infra filter (without it /talker would be blocked by
/// rcl_interfaces/Log and the oracle would change shape).
fn migration_fixture() -> (Vec<DiscoveredNode>, Vec<DiscoveredEndpoint>) {
    let nodes = vec![
        node("/", "talker", PART_A),
        node("/sensors", "imu_filter", PART_B),
        node("/", "widget_node", PART_C),
        node("/", "param_server", PART_D),
    ];
    let endpoints = vec![
        ep_owned(
            "rt/chatter",
            "std_msgs::msg::dds_::String_",
            EndpointKind::Writer,
            PART_A,
            1,
        ),
        // ROS plumbing on the SAME process — must not affect its verdict.
        ep_owned(
            "rt/rosout",
            "rcl_interfaces::msg::dds_::Log_",
            EndpointKind::Writer,
            PART_A,
            2,
        ),
        ep_owned(
            "rq/talker/get_parametersRequest",
            "rcl_interfaces::srv::dds_::GetParameters_Request_",
            EndpointKind::Reader,
            PART_A,
            3,
        ),
        ep_owned(
            "rt/imu_pkt",
            "vendor_msgs::msg::dds_::ImuPacket_",
            EndpointKind::Writer,
            PART_B,
            1,
        ),
        ep_owned(
            "rt/widget",
            "acme_msgs::msg::dds_::Widget_",
            EndpointKind::Writer,
            PART_C,
            1,
        ),
        // param_server exposes ONLY plumbing — the no-message-endpoints arm.
        ep_owned(
            "rq/param_server/set_parametersRequest",
            "rcl_interfaces::srv::dds_::SetParameters_Request_",
            EndpointKind::Reader,
            PART_D,
            1,
        ),
        // Orphans: a GUID-less endpoint and one whose prefix matches no node.
        ep(
            "rt/utlidar/cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep_owned(
            "rt/lowstate",
            "unitree_go::msg::dds_::LowState_",
            EndpointKind::Writer,
            PART_VENDOR,
            1,
        ),
    ];
    (nodes, endpoints)
}

/// The headline byte-exact oracle: every group present, in order, with the
/// commands block naming the restartable nodes, the cost line, and the
/// closing `cerulion ros2 migrate` pointer. Deleting any group's render arm,
/// the infra filter, the classification precedence, or a whole part of the
/// report fails THIS test against the hand oracle.
#[test]
fn migration_report_all_groups_exact_text_oracle() {
    let (nodes, endpoints) = migration_fixture();
    let locally = set(&["std_msgs/String", "sensor_msgs/PointCloud2"]);
    let resolvable = set(&[
        "std_msgs/String",
        "sensor_msgs/PointCloud2",
        "vendor_msgs/ImuPacket",
    ]);
    let out = render_migration_report(&MigrationReportInputs {
        nodes: &nodes,
        endpoints: &endpoints,
        locally_resolvable: &locally,
        resolvable: &resolvable,
    });
    let expected = concat!(
        "\nMIGRATION — what could run natively on rmw_cerulion:\n",
        "\nRESTARTABLE TODAY (1 process(es)) — every message type these nodes use ",
        "resolves locally; restart each process under rmw_cerulion ",
        "(RMW_IMPLEMENTATION=rmw_cerulion) and its topics ride the Cerulion wire ",
        "natively, no bridge hop:\n",
        "  /talker  — std_msgs/String\n",
        "\nRESTARTABLE AFTER THIS ATTACH WRITES ITS SCHEMAS (1 process(es)) — these ",
        "nodes use custom types resolved during this attach; consent writes the ",
        ".msg files, then the same restart applies:\n",
        "  /sensors/imu_filter  — vendor_msgs/ImuPacket\n",
        "\nSTAYS BRIDGED (2 topic(s), 1 process(es)) — the dds_bridge keeps carrying these:\n",
        "  /lowstate  (unitree_go/LowState)  — no ROS 2 node record seen (vendor/raw DDS, ",
        "or the node table was not observed this window)\n",
        "  /utlidar/cloud  (sensor_msgs/PointCloud2)  — no ROS 2 node record seen ",
        "(vendor/raw DDS, or the node table was not observed this window); the type ",
        "resolves locally — if this is one of your nodes, restarting it under ",
        "rmw_cerulion works\n",
        "  /widget_node  — blocked by unresolvable or excluded type(s): acme_msgs/Widget ",
        "(see the report above)\n",
        "\nNO MESSAGE ENDPOINTS SEEN (1 process(es)) — these nodes exposed no message ",
        "endpoints during the window; nothing to judge, nothing bridged:\n",
        "  /param_server\n",
        "\nRestart your own bringup natively — one word in front of the launch you ",
        "already own:\n",
        "  cerulion ros2 launch <your-bringup>.launch.py\n",
        "Or declare the restartable nodes as ros2: entries in a Cerulion graph and let ",
        "`cerulion graph run` bring them up beside native nodes (fill in each ",
        "package/executable — DDS discovery sees endpoints, not launch metadata):\n",
        "\n",
        "nodes:\n",
        "  - id: \"talker\"\n",
        "    ros2:\n",
        "      package: <package>       # the package that ships /talker\n",
        "      executable: <executable>\n",
        "  - id: \"sensors_imu_filter\"\n",
        "    ros2:\n",
        "      package: <package>       # the package that ships /sensors/imu_filter\n",
        "      executable: <executable>\n",
        "\nBridged vs native: a bridged topic pays a per-message CDR decode in the ",
        "dds_bridge; a native topic is published once on the Cerulion wire — no decode ",
        "hop, zero-copy eligible. The cost of native is one process restart.\n",
        "\nTo adopt the loaned zero-copy publish API inside your own nodes, see ",
        "`cerulion ros2 migrate`.\n",
    );
    assert_eq!(out, expected);
}

/// The Go2 shape: endpoints but NO node table at all — the
/// "everything stays bridged" line names the missing ros_discovery_info, the
/// orphan topics are listed, and the commands + cost blocks (which would name
/// restartable nodes that do not exist) are ABSENT while the closing migrate
/// pointer stays.
#[test]
fn migration_report_no_node_table_prints_honest_everything_bridged() {
    let endpoints = vec![
        ep(
            "rt/utlidar/cloud",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/lowstate",
            "unitree_go::msg::dds_::LowState_",
            EndpointKind::Writer,
            BE_VOL,
        ),
    ];
    let locally = set(&["sensor_msgs/PointCloud2"]);
    let out = render_migration_report(&MigrationReportInputs {
        nodes: &[],
        endpoints: &endpoints,
        locally_resolvable: &locally,
        resolvable: &locally,
    });
    let expected = concat!(
        "\nMIGRATION — what could run natively on rmw_cerulion:\n",
        "\nNo restartable process can be named: no ROS 2 node table was seen ",
        "(nothing published ros_discovery_info during the window — vendor/raw-DDS ",
        "processes, or the table was simply not observed). Every discovered topic ",
        "stays on the dds_bridge; where a topic's type already resolves, its line ",
        "below says what a restart would buy.\n",
        "\nSTAYS BRIDGED (2 topic(s)) — the dds_bridge keeps carrying these:\n",
        "  /lowstate  (unitree_go/LowState)  — no ROS 2 node record seen (vendor/raw DDS, ",
        "or the node table was not observed this window)\n",
        "  /utlidar/cloud  (sensor_msgs/PointCloud2)  — no ROS 2 node record seen ",
        "(vendor/raw DDS, or the node table was not observed this window); the type ",
        "resolves locally — if this is one of your nodes, restarting it under ",
        "rmw_cerulion works\n",
        "\nTo adopt the loaned zero-copy publish API inside your own nodes, see ",
        "`cerulion ros2 migrate`.\n",
    );
    assert_eq!(out, expected);
    assert!(
        !out.contains("cerulion ros2 launch") && !out.contains("Bridged vs native"),
        "zero restartable nodes must not render the restart commands or cost blocks: {out}"
    );
}

/// A node table IS present but every process is blocked — that line
/// takes the with-node-table wording ("yet", pointing at the reasons below)
/// instead of claiming the table was missing.
#[test]
fn migration_report_all_blocked_with_node_table_says_yet() {
    let nodes = vec![node("/", "widget_node", PART_C)];
    let endpoints = vec![ep_owned(
        "rt/widget",
        "acme_msgs::msg::dds_::Widget_",
        EndpointKind::Writer,
        PART_C,
        1,
    )];
    let empty = BTreeSet::new();
    let out = render_migration_report(&MigrationReportInputs {
        nodes: &nodes,
        endpoints: &endpoints,
        locally_resolvable: &empty,
        resolvable: &empty,
    });
    assert!(
        out.contains(
            "No process here is restartable under rmw_cerulion yet — every attributed \
             process stays on the dds_bridge (see the reasons below)."
        ),
        "{out}"
    );
    assert!(
        out.contains("blocked by unresolvable or excluded type(s): acme_msgs/Widget"),
        "{out}"
    );
    assert!(
        !out.contains("no ROS 2 node table was seen"),
        "the node table WAS seen — the missing-table wording would be a lie: {out}"
    );
}

/// Remote-supplied strings (node names, topic names, type names) are
/// terminal-escape-sanitized: every C0/C1 control lands as U+FFFD, never on
/// the operator's terminal.
#[test]
fn migration_report_sanitizes_remote_supplied_names() {
    let nodes = vec![node("/", "evil\u{1b}[2Jnode", PART_A)];
    let endpoints = vec![
        ep_owned(
            "rt/bad\rtopic",
            "acme_msgs::msg::dds_::We\u{7}ird_",
            EndpointKind::Writer,
            PART_A,
            1,
        ),
        ep(
            "rt/orphan\u{1b}]0;pwn\u{7}",
            "acme_msgs::msg::dds_::Orphan_",
            EndpointKind::Writer,
            BE_VOL,
        ),
    ];
    let empty = BTreeSet::new();
    let out = render_migration_report(&MigrationReportInputs {
        nodes: &nodes,
        endpoints: &endpoints,
        locally_resolvable: &empty,
        resolvable: &empty,
    });
    assert!(
        !out.chars().any(|c| matches!(
            c,
            '\u{0000}'..='\u{0009}'
                | '\u{000B}'..='\u{001F}'
                | '\u{007F}'
                | '\u{0080}'..='\u{009F}'
        )),
        "no control character may survive into the report: {out:?}"
    );
    assert!(
        out.contains('\u{FFFD}'),
        "sanitized names must carry the replacement char, not silently vanish: {out:?}"
    );
}

/// The infra filter, pinned directly: a process exposing ONLY ROS plumbing
/// (rosout, parameter_events, ros_discovery_info, parameter services) has
/// nothing to judge — and none of those plumbing names leak into the report.
#[test]
fn migration_report_excludes_ros_infra_endpoints_from_the_judgment() {
    let nodes = vec![node("/", "plumbing_only", PART_A)];
    let endpoints = vec![
        ep_owned(
            "rt/rosout",
            "rcl_interfaces::msg::dds_::Log_",
            EndpointKind::Writer,
            PART_A,
            1,
        ),
        ep_owned(
            "rt/parameter_events",
            "rcl_interfaces::msg::dds_::ParameterEvent_",
            EndpointKind::Writer,
            PART_A,
            2,
        ),
        ep_owned(
            "ros_discovery_info",
            "rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_",
            EndpointKind::Writer,
            PART_A,
            3,
        ),
        ep_owned(
            "rq/plumbing_only/get_parametersRequest",
            "rcl_interfaces::srv::dds_::GetParameters_Request_",
            EndpointKind::Reader,
            PART_A,
            4,
        ),
        ep_owned(
            "rr/plumbing_only/get_parametersReply",
            "rcl_interfaces::srv::dds_::GetParameters_Response_",
            EndpointKind::Writer,
            PART_A,
            5,
        ),
    ];
    let empty = BTreeSet::new();
    let out = render_migration_report(&MigrationReportInputs {
        nodes: &nodes,
        endpoints: &endpoints,
        locally_resolvable: &empty,
        resolvable: &empty,
    });
    assert!(
        out.contains("NO MESSAGE ENDPOINTS SEEN (1 process(es))"),
        "plumbing-only processes must land in the nothing-to-judge arm: {out}"
    );
    assert!(out.contains("  /plumbing_only\n"), "{out}");
    for leaked in [
        "rosout",
        "parameter_events",
        "rcl_interfaces",
        "rmw_dds_common",
    ] {
        assert!(
            !out.contains(leaked),
            "plumbing name {leaked:?} must not leak into the migration report: {out}"
        );
    }
}

/// Nothing discovered at all ⇒ the EMPTY form of the section — still
/// rendered ("every attach report ends with MIGRATION" is a contract, and a
/// silent absence is indistinguishable from the feature not running), with
/// nothing to judge said plainly and the closing pointer kept.
#[test]
fn migration_report_empty_discovery_renders_the_honest_empty_section() {
    let empty = BTreeSet::new();
    let out = render_migration_report(&MigrationReportInputs {
        nodes: &[],
        endpoints: &[],
        locally_resolvable: &empty,
        resolvable: &empty,
    });
    let expected = concat!(
        "\nMIGRATION — what could run natively on rmw_cerulion:\n",
        "\nNothing was discovered this window, so there is nothing to judge — see the ",
        "discovery hints above.\n",
        "\nTo adopt the loaned zero-copy publish API inside your own nodes, see ",
        "`cerulion ros2 migrate`.\n",
    );
    assert_eq!(out, expected);
}

/// An orphan (no node record) is rendered in
/// THREE shapes by its TYPE, mirroring the process-classification precedence:
/// a locally-resolvable type gets the today-works hint; a type resolvable
/// only AFTER this attach writes its schemas gets the after-write hint; an
/// unknown type gets the plain absence line. All three on one report, byte
/// oracle.
#[test]
fn migration_report_renders_all_three_orphan_shapes() {
    // No node table at all ⇒ every endpoint is an orphan.
    let endpoints = vec![
        ep(
            "rt/known_now",
            "sensor_msgs::msg::dds_::PointCloud2_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/acquired",
            "vendor_msgs::msg::dds_::ImuPacket_",
            EndpointKind::Writer,
            BE_VOL,
        ),
        ep(
            "rt/unknown",
            "mystery_msgs::msg::dds_::Blob_",
            EndpointKind::Writer,
            BE_VOL,
        ),
    ];
    let locally = set(&["sensor_msgs/PointCloud2"]);
    // ImuPacket resolves ONLY in the final set (acquired this attach), not
    // locally; Blob resolves in neither.
    let resolvable = set(&["sensor_msgs/PointCloud2", "vendor_msgs/ImuPacket"]);
    let out = render_migration_report(&MigrationReportInputs {
        nodes: &[],
        endpoints: &endpoints,
        locally_resolvable: &locally,
        resolvable: &resolvable,
    });
    let expected = concat!(
        "\nMIGRATION — what could run natively on rmw_cerulion:\n",
        "\nNo restartable process can be named: no ROS 2 node table was seen ",
        "(nothing published ros_discovery_info during the window — vendor/raw-DDS ",
        "processes, or the table was simply not observed). Every discovered topic ",
        "stays on the dds_bridge; where a topic's type already resolves, its line ",
        "below says what a restart would buy.\n",
        "\nSTAYS BRIDGED (3 topic(s)) — the dds_bridge keeps carrying these:\n",
        // Sorted by (topic, type): /acquired, /known_now, /unknown.
        "  /acquired  (vendor_msgs/ImuPacket)  — no ROS 2 node record seen (vendor/raw DDS, ",
        "or the node table was not observed this window); the type resolved during this ",
        "attach — if this is one of your nodes, it is restartable after the schemas are ",
        "written\n",
        "  /known_now  (sensor_msgs/PointCloud2)  — no ROS 2 node record seen (vendor/raw ",
        "DDS, or the node table was not observed this window); the type resolves locally — ",
        "if this is one of your nodes, restarting it under rmw_cerulion works\n",
        "  /unknown  (mystery_msgs/Blob)  — no ROS 2 node record seen (vendor/raw DDS, or ",
        "the node table was not observed this window)\n",
        "\nTo adopt the loaned zero-copy publish API inside your own nodes, see ",
        "`cerulion ros2 migrate`.\n",
    );
    assert_eq!(out, expected);
}

/// The classifier checks `locally_resolvable` FIRST: a type whose schema
/// exists locally keeps its restartable-today claim even when it is absent
/// from the post-exclusion `resolvable` set (its only topics were
/// malformed-excluded from the bridge) — restartable-today is a
/// schema-exists claim, not a bridge-mapping one. Reordering the classifier
/// back to missing-first demotes this process to blocked and fails here.
#[test]
fn migration_report_locally_resolvable_type_keeps_today_even_when_bridge_excluded() {
    let nodes = vec![node("/", "talker", PART_A)];
    let endpoints = vec![ep_owned(
        "rt/chatter",
        "std_msgs::msg::dds_::String_",
        EndpointKind::Writer,
        PART_A,
        1,
    )];
    let locally = set(&["std_msgs/String"]);
    // The bridge carries nothing for it (malformed-only topics ⇒ absent from
    // the post-exclusion set).
    let resolvable = BTreeSet::new();
    let out = render_migration_report(&MigrationReportInputs {
        nodes: &nodes,
        endpoints: &endpoints,
        locally_resolvable: &locally,
        resolvable: &resolvable,
    });
    assert!(
        out.contains("RESTARTABLE TODAY (1 process(es))")
            && out.contains("  /talker  — std_msgs/String\n"),
        "a locally-resolvable type is restartable today regardless of bridge exclusion: {out}"
    );
    assert!(
        !out.contains("blocked by unresolvable or excluded type(s)"),
        "{out}"
    );
}

/// Collect every `- id: "<x>"` value the YAML block emitted, in order (the
/// emit site double-quotes ids — remote-derived values; the quotes are part
/// of the injection defense, so this helper REQUIRES them).
fn yaml_ids(out: &str) -> Vec<String> {
    out.lines()
        .filter_map(|l| {
            l.trim()
                .strip_prefix("- id: \"")
                .and_then(|r| r.strip_suffix('"'))
                .map(str::to_string)
        })
        .collect()
}

/// Render a migration report for a set of `(namespace, name, participant,
/// type)` restartable nodes and return the emitted YAML ids. `ty` is a
/// canonical `pkg/Type` that must be in `locally` so every node classifies
/// restartable-today (so it reaches the YAML block).
fn ids_for(nodes_spec: &[(&str, &str, [u8; 12])]) -> Vec<String> {
    let nodes: Vec<DiscoveredNode> = nodes_spec
        .iter()
        .map(|(ns, name, p)| node(ns, name, *p))
        .collect();
    let endpoints: Vec<DiscoveredEndpoint> = nodes_spec
        .iter()
        .enumerate()
        .map(|(i, (_, _, p))| {
            ep_owned(
                &format!("rt/t{i}"),
                "std_msgs::msg::dds_::String_",
                EndpointKind::Writer,
                *p,
                1,
            )
        })
        .collect();
    let locally = set(&["std_msgs/String"]);
    let out = render_migration_report(&MigrationReportInputs {
        nodes: &nodes,
        endpoints: &endpoints,
        locally_resolvable: &locally,
        resolvable: &locally,
    });
    yaml_ids(&out)
}

/// The plain slash→underscore fold is LOSSY (`/a/b` and `/a_b` both fold to
/// `a_b`) and a `nodes:` list with duplicate ids is rejected by the real
/// graph validator. Colliding bases get a participant-keyed FNV suffix on ALL
/// colliding members; a non-colliding sibling stays bare. Exact hand oracle
/// (FNV-1a 64 low-32 of `participant_prefix ++ fqn`): PART_A(=[1;12]) ++
/// "/a/b" ⇒ f4694148, PART_B(=[2;12]) ++ "/a_b" ⇒ ff9c0f3c.
#[test]
fn migration_yaml_ids_disambiguate_colliding_node_names() {
    let ids = ids_for(&[
        ("/a", "b", PART_A),
        ("/", "a_b", PART_B),
        ("/", "solo", PART_C),
    ]);
    assert_eq!(ids, vec!["a_b_f4694148", "a_b_ff9c0f3c", "solo"], "{ids:?}");
}

/// The used-set safety pass (step 2), pinned by the only case that forces it
/// deterministically: a node table listing the SAME (namespace, name,
/// participant) twice ⇒ two YAML entries with IDENTICAL `(prefix, fqn)`, so
/// their step-1 candidates are equal and step 2 must break the tie. Without
/// the pass the two ids alias (a duplicate `nodes:` id the real validator
/// rejects). Two nodes in one process, so `ids_for`'s per-node endpoint gives
/// the process its std_msgs/String type and it classifies restartable-today.
#[test]
fn migration_yaml_ids_identical_entries_are_broken_apart_by_the_used_set_pass() {
    let ids = ids_for(&[("/", "dup", PART_A), ("/", "dup", PART_A)]);
    assert_eq!(ids.len(), 2, "{ids:?}");
    assert_ne!(
        ids[0], ids[1],
        "identical (prefix, fqn) entries must not alias: {ids:?}"
    );
    // Base `dup` is claimed twice ⇒ both get the (identical) participant
    // suffix, then step 2 appends `_2` to the second.
    assert_eq!(ids[0], "dup_0339bb67", "{ids:?}");
    assert_eq!(ids[1], "dup_0339bb67_2", "{ids:?}");
}

/// YAML injection: remote-derived node names reach
/// the id position, and `sanitize_display` neuters only terminal controls —
/// a name like `pwn: - #owned` carries YAML-meaningful bytes. The id
/// alphabet is RESTRICTED to `[A-Za-z0-9_]` (every other char folds to
/// `_`) AND the emit site double-quotes it, so the paste-ready block stays
/// structurally inert. Oracle: the folded quoted id byte-exactly, the
/// extracted `nodes:` block PARSES as YAML with the id round-tripping as a
/// STRING, and the hostile bytes never appear in the id position. The
/// comment position is covered too: newlines are U+FFFD'd by the sanitizer,
/// so a name cannot escape the `# … ships <fqn>` comment to a new line.
#[test]
fn migration_yaml_block_is_injection_proof_against_hostile_node_names() {
    let ids = ids_for(&[("/", "pwn: - #owned", PART_A)]);
    // p w n : SP - SP # o w n e d — the 5 non-alphanumerics each fold to _.
    assert_eq!(ids, vec!["pwn_____owned"], "{ids:?}");

    // Re-render to inspect the full block (ids_for only returns ids).
    let nodes = vec![node("/", "pwn: - #owned", PART_A)];
    let endpoints = vec![ep_owned(
        "rt/t0",
        "std_msgs::msg::dds_::String_",
        EndpointKind::Writer,
        PART_A,
        1,
    )];
    let locally = set(&["std_msgs/String"]);
    let out = render_migration_report(&MigrationReportInputs {
        nodes: &nodes,
        endpoints: &endpoints,
        locally_resolvable: &locally,
        resolvable: &locally,
    });
    assert!(
        out.contains("  - id: \"pwn_____owned\"\n"),
        "the id must be alphabet-folded AND quoted: {out}"
    );
    // The whole emitted YAML fragment (from `nodes:` to the blank line before
    // the cost paragraph) must PARSE, with the id a plain string.
    let start = out.find("nodes:\n").expect("the YAML block renders");
    let yaml = &out[start..];
    let end = yaml.find("\n\n").unwrap_or(yaml.len());
    let parsed: serde_yaml::Value =
        serde_yaml::from_str(&yaml[..end]).expect("the paste-ready block must parse as YAML");
    let id = parsed["nodes"][0]["id"]
        .as_str()
        .expect("id must round-trip as a YAML string")
        .to_string();
    assert_eq!(id, "pwn_____owned");
}

/// A suffix-on-collision scheme keyed on the FQN alone does
/// not CONVERGE — a suffixed id could equal ANOTHER entry's bare base. The
/// exact adversarial set `/a/b`, `/a_b`, `/a_b_b38ee0cc` (b38ee0cc = the
/// fqn-only FNV of `/a/b`, so such a scheme mints `a_b_b38ee0cc`
/// for `/a/b` — colliding with the third's literal base) must yield THREE
/// distinct ids. The participant-keyed suffix moves `/a/b` off b38ee0cc, and
/// the third's unique base stays bare; the used-set pass is the backstop.
#[test]
fn migration_yaml_ids_the_round2_aliasing_adversarial_set_is_now_three_distinct() {
    let ids = ids_for(&[
        ("/a", "b", PART_A),
        ("/", "a_b", PART_B),
        ("/", "a_b_b38ee0cc", PART_C),
    ]);
    assert_eq!(ids.len(), 3, "exactly one id per entry: {ids:?}");
    let distinct: BTreeSet<&String> = ids.iter().collect();
    assert_eq!(
        distinct.len(),
        3,
        "three FQNs must mint three distinct ids: {ids:?}"
    );
    // The third's base is unique, so it stays bare; the first two are
    // suffixed off b38ee0cc, so no member aliases it.
    assert!(ids.contains(&"a_b_b38ee0cc".to_string()), "{ids:?}");
    assert!(
        !ids.iter().any(|id| id == "a_b"),
        "both a_b claimants must be suffixed: {ids:?}"
    );
}

/// The SAME class ACROSS participants — the
/// discovery layer keeps two same-named nodes on distinct participants, and
/// they must not mint the same id. Keying the suffix on participant+FQN makes
/// both `/talker` rows distinct without an order-dependent tiebreak. Exact
/// hand oracle: PART_A_10(=[10;12]) ++ "/talker" ⇒ b3596a1f, PART_B_20(=[20;
/// 12]) ++ "/talker" ⇒ 97d9479f.
#[test]
fn migration_yaml_ids_same_fqn_on_two_participants_stay_distinct() {
    const PART_A_10: [u8; 12] = [10; 12];
    const PART_B_20: [u8; 12] = [20; 12];
    let ids = ids_for(&[("/", "talker", PART_A_10), ("/", "talker", PART_B_20)]);
    assert_eq!(ids, vec!["talker_b3596a1f", "talker_97d9479f"], "{ids:?}");
    assert_ne!(
        ids[0], ids[1],
        "same-FQN-different-participant must differ: {ids:?}"
    );
}

/// The attach-flow seam: a dry-run's report carries the MIGRATION section
/// AFTER the discovery report's Summary line, the outcome stays DryRun (the
/// section never changes the outcome/exit contract), and the node-table
/// attribution flows end-to-end from the injected DiscoveryResult through the
/// REAL resolvability chain (std_msgs/String resolves via builtins ⇒
/// restartable today).
#[test]
fn migration_section_rides_the_dry_run_report_after_the_summary() {
    let tmp = tempfile::tempdir().unwrap();
    let discovery = FakeDiscovery::ok_with_nodes(
        vec![
            ep_owned(
                "rt/chatter",
                "std_msgs::msg::dds_::String_",
                EndpointKind::Writer,
                PART_A,
                1,
            ),
            ep(
                "rt/lowstate",
                "unitree_go::msg::dds_::LowState_",
                EndpointKind::Writer,
                BE_VOL,
            ),
        ],
        vec![node("/", "talker", PART_A)],
    );
    let mut confirm = panic_confirm;
    let report = ros_cmd::ros_attach(
        &discovery,
        tmp.path(),
        &opts(true, true),
        true,
        &mut confirm,
    )
    .expect("dry run");
    assert_eq!(report.outcome, AttachOutcome::DryRun);
    let summary_at = report
        .report
        .find("Summary:")
        .expect("the discovery report keeps its Summary line");
    let migration_at = report
        .report
        .find("MIGRATION — what could run natively on rmw_cerulion:")
        .expect("the migration section prints on every run — no flag");
    assert!(
        summary_at < migration_at,
        "the migration section slots AFTER the discovery/partition report: {}",
        report.report
    );
    assert!(
        report.report.contains("RESTARTABLE TODAY (1 process(es))"),
        "the builtin-resolvable /talker must be restartable today: {}",
        report.report
    );
    assert!(
        report
            .report
            .contains("  /lowstate  (unitree_go/LowState)  — no ROS 2 node record seen"),
        "{}",
        report.report
    );
}

/// The capture-point pin: an acquired custom type whose ONLY topic
/// is malformed-excluded must NOT be labeled after-write restartable — its
/// topic is excluded from the mapping set, and with nothing else bridgeable
/// the run refuses the consent write entirely, so nothing would be
/// materialized. The post-exclusion capture classifies its process as
/// blocked (naming the type) instead. Reverting `resolvable_types` to the
/// pre-exclusion partition fails this test with the process in the
/// after-write group.
#[test]
fn migration_acquired_type_on_a_malformed_only_topic_is_not_after_write_restartable() {
    let tmp = tempfile::tempdir().unwrap();
    // The DDS topic normalizes to `/bad//pkt` — an empty segment, rejected by
    // the graph layer's own absolute-name predicate (malformed-excluded).
    let discovery = FakeDiscovery::ok_with_nodes(
        vec![ep_owned(
            "rt/bad//pkt",
            "vendor_msgs::msg::dds_::ImuPacket_",
            EndpointKind::Writer,
            PART_B,
            1,
        )],
        vec![node("/sensors", "imu_filter", PART_B)],
    );
    let canned = CannedAcquirer::new(BTreeMap::from([(
        "vendor_msgs/ImuPacket".to_string(),
        acquired(
            AcquisitionRung::LocalAment,
            &[("vendor_msgs", "ImuPacket", "float64 x\n")],
        ),
    )]));
    let mut confirm = panic_confirm;
    let report = ros_cmd::ros_attach_with_acquirer(
        &discovery,
        &canned,
        tmp.path(),
        &opts(true, true),
        true,
        &mut confirm,
    )
    .expect("dry run");
    assert_eq!(report.outcome, AttachOutcome::DryRun);
    // The topic really was malformed-excluded (the premise assert — without
    // it a fixture typo would make the after-write absence vacuous).
    assert!(
        report
            .report
            .contains("EXCLUDED — MALFORMED TOPIC NAMES (1)"),
        "{}",
        report.report
    );
    assert!(
        !report
            .report
            .contains("RESTARTABLE AFTER THIS ATTACH WRITES ITS SCHEMAS"),
        "an acquired type bridging NOTHING must not be claimed after-write restartable: {}",
        report.report
    );
    assert!(
        report.report.contains(
            "  /sensors/imu_filter  — blocked by unresolvable or excluded type(s): \
             vendor_msgs/ImuPacket (see the report above)"
        ),
        "{}",
        report.report
    );
}

/// The consent side of the seam: on a --yes write the preview EMBEDS the
/// migration section BEFORE the "Will write" consent block (the report slots
/// before the consent gate), and the outcome/write behavior is byte-for-byte
/// what it was without the section (Written + hand-off, files on disk).
#[test]
fn migration_section_precedes_the_consent_block_and_leaves_the_outcome_alone() {
    let tmp = tempfile::tempdir().unwrap();
    let discovery = FakeDiscovery::ok_with_nodes(
        vec![ep_owned(
            "rt/chatter",
            "std_msgs::msg::dds_::String_",
            EndpointKind::Writer,
            PART_A,
            1,
        )],
        vec![node("/", "talker", PART_A)],
    );
    let mut confirm = panic_confirm; // --yes never consults confirm
    let report = ros_cmd::ros_attach(
        &discovery,
        tmp.path(),
        &opts(false, true),
        false,
        &mut confirm,
    )
    .expect("assume-yes write");
    assert!(matches!(report.outcome, AttachOutcome::Written { .. }));
    assert_eq!(report.graph_to_run.as_deref(), Some("attach"));
    assert!(tmp.path().join("graphs").join("attach.yaml").exists());
    let migration_at = report
        .preview
        .find("MIGRATION — what could run natively on rmw_cerulion:")
        .expect("the consent preview embeds the migration section");
    let will_write_at = report
        .preview
        .find("Will write")
        .expect("the consent preview keeps its Will write block");
    assert!(
        migration_at < will_write_at,
        "the migration section slots BEFORE the consent gate: {}",
        report.preview
    );
}

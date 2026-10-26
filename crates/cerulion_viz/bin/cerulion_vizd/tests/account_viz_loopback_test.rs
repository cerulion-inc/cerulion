// SPDX-License-Identifier: AGPL-3.0-only
//! Real account HTTP -> CLI snapshot -> account controller -> robot owner pair
//! -> catalog/schema -> vizd UDS attach -> SHM delivery -> exact Rerun scalars.
//! Relay-disabled loopback only. No hosted service, robot, or viewer process.
//! This is a separate serial binary because it owns daemons and process env.

#![cfg(unix)]

#[path = "account_viz_loopback/account.rs"]
mod account;
#[path = "account_viz_loopback/robot.rs"]
mod robot;
#[path = "account_viz_loopback/support.rs"]
mod support;

use std::collections::{BTreeMap, HashMap};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::transport::demand_authorizer::AllowAllAuthorizer;
use cerulion_core::wire::MaxSliceLen;
use cerulion_link::RelayConfig;
use cerulion_netd::account_controller::{AccountControllerConfig, AccountWanController};
use cerulion_netd::wan::WanRegistry;
use cerulion_netd::NetdConfig;
use cerulion_viz::schema_registry::builtin_walker;
use cerulion_viz::sink::SinkState;
use cerulion_viz::worker::VizLogWorker;
use cerulion_vizd::{start_with_demand_plane, NetdDemandPlane, DEFAULT_POLL_INTERVAL};
use re_log_types::{LogMsg, StoreKind};
use serde_json::json;
use serial_test::serial;

use support::{BOUND, DESK_SEED, TOPIC, TYPE};

// Literal little-endian payload oracles: (1.5, -2.5), (3.5, 4.5), (-5.5, 6.5).
const PAYLOADS: [[u8; 16]; 3] = [
    [0, 0, 0, 0, 0, 0, 0xf8, 0x3f, 0, 0, 0, 0, 0, 0, 0x04, 0xc0],
    [0, 0, 0, 0, 0, 0, 0x0c, 0x40, 0, 0, 0, 0, 0, 0, 0x12, 0x40],
    [0, 0, 0, 0, 0, 0, 0x16, 0xc0, 0, 0, 0, 0, 0, 0, 0x1a, 0x40],
];

fn frame(hash: u64, index: usize) -> [u8; 48] {
    let mut bytes = [0_u8; 48];
    bytes[..8].copy_from_slice(&hash.to_le_bytes());
    bytes[8] = 48; // total_size
    bytes[12] = 48; // offset table follows the fixed fields, zero entries
    bytes[20] = index as u8; // sequence
    bytes[24..32].copy_from_slice(&((index as u64 + 1) * 1_000_000_000).to_le_bytes());
    bytes[32..].copy_from_slice(&PAYLOADS[index]);
    bytes
}

fn take_scalars(messages: Vec<LogMsg>, values: &mut BTreeMap<String, Vec<f64>>) {
    for message in messages {
        let LogMsg::ArrowMsg(store, arrow) = message else {
            continue;
        };
        if store.kind() != StoreKind::Recording {
            continue;
        }
        let chunk = re_chunk::Chunk::from_arrow_msg(&arrow).unwrap();
        let entity = chunk
            .entity_path()
            .to_string()
            .trim_start_matches('/')
            .to_owned();
        for descriptor in chunk.components().component_descriptors() {
            if descriptor.component.as_str() == "Scalars:scalars" {
                for samples in chunk.iter_slices::<f64>(descriptor.component) {
                    values
                        .entry(entity.clone())
                        .or_default()
                        .extend_from_slice(samples);
                }
            }
        }
    }
}

#[test]
#[serial]
fn account_directory_to_vizd_renders_exact_loopback_samples_and_detaches() {
    // These are every filesystem root this fixture owns. Directory removes the
    // complete tree last, including on panic, after the daemon Drop guards.
    let directory = support::Directory::new();
    let home = directory.child("auth");
    let state = directory.child("robot-state");
    let robot_shm = directory.child("robot-shm");
    let desk_shm = directory.child("desk-shm");
    let account = account::Account::start(&home);
    let robot = robot::Robot::start(&account, &state, &robot_shm, directory.0.join("robot.sock"));
    let mut publisher = robot
        .manager
        .create_publisher_simple(TOPIC, MaxSliceLen::const_new(48))
        .unwrap();

    let desk = support::manager(&desk_shm, "account_viz_desk");
    let controller = Arc::new(
        AccountWanController::new(
            desk.clone(),
            WanRegistry::new(HashMap::new(), DESK_SEED, RelayConfig::Disabled),
            Arc::new(AllowAllAuthorizer),
            AccountControllerConfig {
                config_home: Some(home.clone()),
                network_enabled: true,
                serving_gateway: false,
                epoch_dir: None,
                trusted_direct: HashMap::from([(
                    (account.robot.0, robot.public_key),
                    robot.direct.clone(),
                )]),
            },
        )
        .unwrap(),
    );
    let desk_socket = directory.0.join("desk.sock");
    let mut netd = cerulion_netd::start(
        desk_socket.clone(),
        controller,
        NetdConfig {
            account_identity_watch: true,
            ..Default::default()
        },
    )
    .unwrap();
    let _home = support::EnvGuard::set("CERULION_HOME", &home);
    let _service = support::EnvGuard::set("CERULION_ACCOUNT_SERVICE", &account.url);
    let _socket = support::EnvGuard::set("CERULION_NETD_SOCK", &desk_socket);
    // An unexpected daemon loss must refuse instead of launching an ambient netd.
    let _no_spawn = support::EnvGuard::set("CERULION_NETD_BIN", directory.0.join("absent-netd"));
    let route = cerulion_netd::account_access::robot_route(&account.robot.0);
    // Actual CLI entry performs owner-only HTTP lookup and installs the snapshot.
    // Retaining this value also tests the daemon hold while vizd starts.
    let target =
        cerulion_cli_engine::account_robot_access::prepare_viz_target(&route, &[], &[]).unwrap();
    assert_eq!(target.route, route);
    assert!(target.diagnostics.is_empty());
    assert_eq!(netd.active_demand_count(), 0);

    let (recording, storage) = rerun::RecordingStreamBuilder::new("account_viz_loopback")
        .recording_id("account-viz-test-recording")
        .memory()
        .unwrap();
    let flush = recording.clone();
    let worker = VizLogWorker::spawn(recording, builtin_walker(), SinkState::new()).unwrap();
    let viz_socket = directory.0.join("viz.sock");
    let mut vizd = start_with_demand_plane(
        viz_socket.clone(),
        DEFAULT_POLL_INTERVAL,
        desk.clone(),
        worker,
        builtin_walker(),
        None,
        Arc::new(NetdDemandPlane::with_socket(desk_socket)),
    )
    .unwrap();
    let mut client = support::Client::connect(&viz_socket);
    let attached =
        client.request(json!({"id":1,"method":"attach","topic":TOPIC,"robot":target.route}));
    assert_eq!(attached["ok"], true, "{attached}");
    assert_eq!(
        attached["schema"], TYPE,
        "custom schema must cross both adapters"
    );
    assert_eq!(attached["entity"], "world/account_loopback/telemetry");
    assert_eq!(netd.active_demand_count(), 1);
    assert_eq!(desk.topic_publisher_count_checked(TOPIC), Some(1));
    let provenance = desk.gather_mirror_provenance_checked(BOUND).unwrap();
    assert!(provenance.completeness.is_settled());
    assert_eq!(provenance.records.len(), 1);
    assert_eq!(provenance.records[0].topic, TOPIC);
    assert_eq!(provenance.records[0].origin_robot, route);
    robot.assert_one_owner_pair(&account);

    let mut probe = desk.create_data_only_subscriber(TOPIC).unwrap();
    let mut scratch = Vec::new();
    let mut values = BTreeMap::new();
    for i in 0..PAYLOADS.len() {
        let oracle = frame(robot.hash, i);
        publisher.publish_raw(&oracle).unwrap();
        let deadline = Instant::now() + BOUND;
        let mut delivered = false;
        loop {
            scratch.clear();
            let count = probe.drain_owned(1, &mut scratch).unwrap();
            for sample in scratch.iter().take(count) {
                assert!(!delivered, "one authored sample must arrive once");
                assert_eq!(
                    sample.payload(),
                    oracle,
                    "complete frame must survive transport"
                );
                delivered = true;
            }
            flush.flush_blocking().unwrap();
            take_scalars(storage.take(), &mut values);
            if delivered
                && values.values().all(|samples| samples.len() == i + 1)
                && values.len() == 2
            {
                break;
            }
            assert!(
                Instant::now() < deadline,
                "sample {i} delivery={delivered}, rendered={values:?}"
            );
            std::thread::sleep(Duration::from_millis(10));
        }
    }
    assert_eq!(
        values,
        BTreeMap::from([
            (
                "world/account_loopback/telemetry/x".into(),
                vec![1.5, 3.5, -5.5]
            ),
            (
                "world/account_loopback/telemetry/y".into(),
                vec![-2.5, 4.5, 6.5]
            ),
        ])
    );
    let status = client.request(json!({"id":2,"method":"status"}));
    let row = status["topics"]
        .as_array()
        .unwrap()
        .iter()
        .find(|row| row["topic"] == TOPIC)
        .unwrap();
    assert_eq!(row["schema"], TYPE);
    assert_eq!(row["frames"], 3);
    drop(probe);
    let detached = client.request(json!({"id":3,"method":"detach","topic":TOPIC}));
    assert_eq!(detached["ok"], true, "{detached}");
    assert_eq!(netd.active_demand_count(), 0);
    let deadline = Instant::now() + BOUND;
    loop {
        let provenance = desk
            .gather_mirror_provenance_checked(Duration::from_millis(200))
            .unwrap();
        if netd.mirror_count() == 0
            && netd.lingering_count() == 0
            && desk.topic_publisher_count_checked(TOPIC) == Some(0)
            && provenance.completeness.is_settled()
            && provenance.records.is_empty()
        {
            break;
        }
        assert!(
            Instant::now() < deadline,
            "detach must retire the actual mirror publisher and settled provenance"
        );
    }
    robot.assert_one_owner_pair(&account);
    drop(client);
    vizd.shutdown();
    drop(target);
    netd.shutdown();
}

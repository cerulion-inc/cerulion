// SPDX-License-Identifier: AGPL-3.0-only
//! `service call` against a real service server over iceoryx2: the server is
//! `cerulion_core`'s `ServiceServer`, the one `rmw_cerulion` wraps for every
//! ROS 2 service, so the call's topic names, envelope and body framing go
//! through the server's own take and send paths. The server's hashes and its
//! request decode come from the same `.srv` resolution as the client's, so
//! this pins the transport protocol, not the hash recipe (the placeholder
//! member unit test pins that against a hand-written schema).
//!
//! The service types are the ros2_control Jazzy `SetHardwareComponentState`
//! and `ListControllers` definitions (comments stripped), with the
//! `lifecycle_msgs/State` and `controller_manager_msgs` messages they use.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_cli_engine::message_text::{body_to_json, message_frame, EXIT_UNAVAILABLE};
use cerulion_cli_engine::service_cmd::{resolve_service_types, service_call, CallRequest};
use cerulion_core::wire::{MaxSliceLen, WireHeader};

const STORE: &[(&str, &str)] = &[
    (
        "lifecycle_msgs/msg/State.msg",
        "uint8 PRIMARY_STATE_UNKNOWN = 0\nuint8 PRIMARY_STATE_UNCONFIGURED = 1\n\
         uint8 PRIMARY_STATE_INACTIVE = 2\nuint8 PRIMARY_STATE_ACTIVE = 3\n\
         uint8 PRIMARY_STATE_FINALIZED = 4\nuint8 id\nstring label\n",
    ),
    (
        "controller_manager_msgs/msg/ChainConnection.msg",
        "string name\nstring[] reference_interfaces\n",
    ),
    (
        "controller_manager_msgs/msg/ControllerState.msg",
        "string name\nstring state\nstring type\nbool is_async\nuint16 update_rate\n\
         string[] claimed_interfaces\nstring[] required_command_interfaces\n\
         string[] required_state_interfaces\nbool is_chainable\nbool is_chained\n\
         string[] exported_state_interfaces\nstring[] reference_interfaces\n\
         ChainConnection[] chain_connections\n",
    ),
    (
        "controller_manager_msgs/srv/SetHardwareComponentState.srv",
        "string name\nlifecycle_msgs/State target_state\n---\nbool ok\nlifecycle_msgs/State state\n",
    ),
    (
        "controller_manager_msgs/srv/ListControllers.srv",
        "---\ncontroller_manager_msgs/ControllerState[] controller\n",
    ),
];

fn store() -> tempfile::TempDir {
    let dir = tempfile::tempdir().expect("tempdir");
    for (rel, text) in STORE {
        let path = dir.path().join(rel);
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(path, text).unwrap();
    }
    dir
}

/// Serve `service` until `stop`: every request is decoded to JSON, recorded,
/// and answered with `response` (a JSON mapping of the response type).
fn serve(
    service: &'static str,
    srv_type: &'static str,
    response: &'static str,
    schemas: std::path::PathBuf,
    ready: Arc<AtomicBool>,
    stop: Arc<AtomicBool>,
) -> std::thread::JoinHandle<Vec<String>> {
    std::thread::spawn(move || {
        let types = resolve_service_types(srv_type, Some(&schemas)).expect("types");
        let request_hash = types.walker.schema_hash_for(&types.request).unwrap();
        let response_hash = types.walker.schema_hash_for(&types.response).unwrap();
        let transport = cerulion_core::TransportManager::get_or_init().expect("transport");
        let mut server = transport
            .create_service_server(
                service,
                MaxSliceLen::const_new(4 * 1024 * 1024),
                request_hash,
                response_hash,
            )
            .expect("server");
        let reply = message_frame(&types.walker, &types.response, response, 0).unwrap();
        ready.store(true, Ordering::SeqCst);
        let mut seen = Vec::new();
        while !stop.load(Ordering::SeqCst) {
            let mut got = None;
            server
                .try_take_one_request(|envelope, payload| {
                    got = Some((
                        envelope,
                        body_to_json(&types.walker, request_hash, payload).unwrap(),
                    ));
                })
                .expect("take");
            match got {
                Some((envelope, json)) => {
                    seen.push(json);
                    server
                        .send_response(&envelope, &reply[WireHeader::SIZE..])
                        .expect("respond");
                }
                None => std::thread::sleep(Duration::from_millis(2)),
            }
        }
        seen
    })
}

fn wait_ready(ready: &AtomicBool) {
    for _ in 0..2500 {
        if ready.load(Ordering::SeqCst) {
            return;
        }
        std::thread::sleep(Duration::from_millis(2));
    }
    panic!("server never became ready");
}

#[test]
fn a_call_reaches_a_core_service_server_and_prints_its_response() {
    let dir = store();
    let ready = Arc::new(AtomicBool::new(false));
    let stop = Arc::new(AtomicBool::new(false));
    let server = serve(
        "/c1_test/controller_manager/set_hardware_component_state",
        "controller_manager_msgs/srv/SetHardwareComponentState",
        r#"{"ok": true, "state": {"id": 3, "label": "active"}}"#,
        dir.path().to_path_buf(),
        ready.clone(),
        stop.clone(),
    );
    wait_ready(&ready);
    let running = AtomicBool::new(true);
    let req = CallRequest {
        service: "/c1_test/controller_manager/set_hardware_component_state",
        type_name: "controller_manager_msgs/srv/SetHardwareComponentState",
        request: "name: arm\ntarget_state: {id: 3, label: active}\n",
        timeout: Duration::from_secs(10),
    };
    let first = service_call(&req, Some(dir.path()), &running).expect("first call");
    let second = service_call(&req, Some(dir.path()), &running).expect("second call");
    stop.store(true, Ordering::SeqCst);
    let seen = server.join().expect("server thread");
    let want = r#"{"ok":true,"state":{"id":3,"label":"active"}}"#;
    assert_eq!(first, want);
    assert_eq!(
        second, want,
        "a second call from one process is answered too"
    );
    assert_eq!(
        seen,
        vec![r#"{"name":"arm","target_state":{"id":3,"label":"active"}}"#.to_string(); 2],
        "the server decoded the request the client encoded"
    );
}

#[test]
fn an_empty_request_reaches_a_server_with_a_nested_array_response() {
    let dir = store();
    let ready = Arc::new(AtomicBool::new(false));
    let stop = Arc::new(AtomicBool::new(false));
    let server = serve(
        "/c1_test/controller_manager/list_controllers",
        "controller_manager_msgs/ListControllers",
        r#"{"controller": [{"name": "arm_controller", "state": "active",
            "type": "joint_trajectory_controller/JointTrajectoryController",
            "update_rate": 100, "claimed_interfaces": ["j1/position"],
            "chain_connections": [{"name": "x", "reference_interfaces": ["a", "b"]}]}]}"#,
        dir.path().to_path_buf(),
        ready.clone(),
        stop.clone(),
    );
    wait_ready(&ready);
    let running = AtomicBool::new(true);
    let req = CallRequest {
        service: "/c1_test/controller_manager/list_controllers",
        type_name: "controller_manager_msgs/ListControllers",
        request: "{}",
        timeout: Duration::from_secs(10),
    };
    let out = service_call(&req, Some(dir.path()), &running).expect("call");
    stop.store(true, Ordering::SeqCst);
    assert_eq!(
        server.join().unwrap(),
        vec![r#"{"structure_needs_at_least_one_member":0}"#.to_string()],
        "a field-less request carries the member ROS 2 gives it"
    );
    assert_eq!(
        out,
        r#"{"controller":[{"name":"arm_controller","state":"active","type":"joint_trajectory_controller/JointTrajectoryController","is_async":false,"update_rate":100,"claimed_interfaces":["j1/position"],"required_command_interfaces":[],"required_state_interfaces":[],"is_chainable":false,"is_chained":false,"exported_state_interfaces":[],"reference_interfaces":[],"chain_connections":[{"name":"x","reference_interfaces":["a","b"]}]}]}"#
    );
}

#[test]
fn a_call_with_no_server_is_unavailable_after_the_timeout() {
    let dir = store();
    let running = AtomicBool::new(true);
    let req = CallRequest {
        service: "/c1_test/nobody_serves_this",
        type_name: "controller_manager_msgs/srv/ListControllers",
        request: "{}",
        timeout: Duration::from_millis(300),
    };
    let started = std::time::Instant::now();
    let err = service_call(&req, Some(dir.path()), &running).unwrap_err();
    assert_eq!(err.exit_code(), EXIT_UNAVAILABLE, "{err}");
    assert!(
        err.to_string()
            .contains("svc/c1_test/nobody_serves_this/request"),
        "{err}"
    );
    assert!(started.elapsed() >= Duration::from_millis(300));
}

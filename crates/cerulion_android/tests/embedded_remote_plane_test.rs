// SPDX-License-Identifier: AGPL-3.0-only
//! A start's `remote` block drives the remote plane host the app library
//! registers.
//!
//! The host here is a fake that records each call in one ordered log and
//! answers `bind` as the step tells it to. The real host, which serves the
//! plane over iroh, has its own test in `cerulion_android_remote`. The host
//! registry and the transport are process state, so this file is its own
//! binary and holds ONE test that runs its steps in order.

use std::net::SocketAddr;
use std::sync::{Arc, Mutex};

use cerulion_android::remote::{
    register_host, BoundRemote, RemoteExit, RemoteHost, RemoteStart, RemoteStartError,
};
use cerulion_android::{codes, EmbeddedRuntime, RemoteRelay, RemoteStatus, RuntimeState};
use serde_json::json;
use tempfile::tempdir;

cerulion_android::embed_nodes! {
    "test_node_macro_relay_rlib" => test_node_macro_relay_rlib::RelayNodeEntry,
    "test_node_macro_period_cdylib" => test_node_macro_period_cdylib::PeriodNodeEntry,
}

const JNI_VERSION_1_6: i32 = 0x0001_0006;
const VECTOR3: &str = "geometry_msgs/Vector3";
const ROBOT: &str = "remote_plane_test";
const ENDPOINT: &str = "fake-endpoint";
const SOCKET: &str = "127.0.0.1:4433";

/// How the fake host answers the next `bind`.
#[derive(Clone, Copy)]
enum Answer {
    Bind,
    /// Bind, then refuse to serve.
    BindServeFails,
    /// Bind and serve, then fail to stop.
    BindStopFails,
    NotProvisioned,
    Fail,
}

/// What a `bind` was handed, apart from the transport.
#[derive(Debug, PartialEq)]
struct Handed {
    state_root: std::path::PathBuf,
    robot: String,
    relay: RemoteRelay,
    served_topics: Vec<String>,
    topic_schemas: Vec<(String, String)>,
}

#[derive(Default)]
struct Shared {
    log: Vec<String>,
    answer: Option<Answer>,
    handed: Vec<Handed>,
    exit: Option<RemoteExit>,
}

type Fake = Arc<Mutex<Shared>>;

struct FakeHost(Fake);

impl RemoteHost for FakeHost {
    fn bind(&self, start: RemoteStart) -> Result<Box<dyn BoundRemote>, RemoteStartError> {
        let mut shared = self.0.lock().unwrap();
        shared.log.push("remote.bind".into());
        shared.handed.push(Handed {
            state_root: start.state_root,
            robot: start.robot,
            relay: start.relay,
            served_topics: start.serving.served_topics,
            topic_schemas: start
                .serving
                .topic_schemas
                .into_iter()
                .map(|binding| (binding.topic, binding.schema_name))
                .collect(),
        });
        match shared.answer.expect("each step sets the answer") {
            answer @ (Answer::Bind | Answer::BindServeFails | Answer::BindStopFails) => {
                Ok(Box::new(FakeBound(self.0.clone(), answer)))
            }
            Answer::NotProvisioned => Err(RemoteStartError::NotProvisioned(
                "no device key under the state directory".into(),
            )),
            Answer::Fail => Err(RemoteStartError::Failed(
                "the endpoint could not bind".into(),
            )),
        }
    }
}

struct FakeBound(Fake, Answer);

impl BoundRemote for FakeBound {
    fn endpoint_id(&self) -> String {
        ENDPOINT.into()
    }
    fn bound_sockets(&self) -> Vec<SocketAddr> {
        vec![SOCKET.parse().unwrap()]
    }
    fn serve(&mut self, exit: RemoteExit) -> Result<(), String> {
        let mut shared = self.0.lock().unwrap();
        shared.log.push("remote.serve".into());
        if matches!(self.1, Answer::BindServeFails) {
            return Err("the plane could not open its receipt log".into());
        }
        shared.exit = Some(exit);
        Ok(())
    }
    fn stop(self: Box<Self>) -> Result<(), String> {
        self.0.lock().unwrap().log.push("remote.stop".into());
        if matches!(self.1, Answer::BindStopFails) {
            return Err("the endpoint did not close".into());
        }
        Ok(())
    }
}

struct Topics {
    cmd: String,
    echo: String,
    graph: String,
}

fn topics(pid: u32) -> Topics {
    let prefix = format!("remote_graph_{pid}");
    let cmd = format!("/remote_{pid}/cmd");
    let graph = format!(
        "prefix: {prefix}
nodes:
  - id: logic
    type: test_node_macro_relay_rlib
    inputs:
      - name: cmd
        source: {cmd}
    outputs:
      - name: echo
        schema: {VECTOR3}
"
    );
    Topics {
        echo: format!("/{prefix}/logic/echo"),
        cmd,
        graph,
    }
}

fn config(topics: &Topics, remote: serde_json::Value) -> String {
    json!({
        "graph_yaml": topics.graph,
        "publish": [{"topic": topics.cmd, "schema": VECTOR3}],
        "remote": remote,
    })
    .to_string()
}

fn answer(fake: &Fake, answer: Answer) {
    let mut shared = fake.lock().unwrap();
    shared.answer = Some(answer);
    shared.log.clear();
    shared.handed.clear();
}

fn log(fake: &Fake) -> Vec<String> {
    fake.lock().unwrap().log.clone()
}

fn status(runtime: &EmbeddedRuntime) -> cerulion_android::Status {
    runtime.status_handle().lock().unwrap().clone()
}

fn publish_refused(runtime: &mut EmbeddedRuntime, topic: &str) -> i32 {
    runtime
        .publish(topic, r#"{"x":1.0,"y":0.0,"z":0.0}"#)
        .expect_err("a runtime that did not start publishes nothing")
        .code()
}

#[test]
fn the_remote_block_binds_before_the_run_serves_at_commit_and_stops_first() {
    assert_eq!(
        JNI_OnLoad(std::ptr::null_mut(), std::ptr::null_mut()),
        JNI_VERSION_1_6
    );
    let files = tempdir().unwrap();
    let topics = topics(std::process::id());
    let mut runtime = EmbeddedRuntime::new(files.path().to_path_buf(), ROBOT.into()).unwrap();
    let enabled = json!({"enabled": true, "relay": "disabled"});

    // No host linked: the start fails with the remote code and names the
    // crate that provides one.
    let error = runtime
        .start(&config(&topics, enabled.clone()))
        .expect_err("no remote plane host is linked");
    assert_eq!(error.code(), codes::REMOTE);
    assert!(
        error.to_string().contains("cerulion_android_remote"),
        "error: {error}"
    );
    assert_eq!(status(&runtime).state, RuntimeState::Failed);

    let fake: Fake = Arc::default();
    register_host(Box::new(FakeHost(fake.clone()))).expect("the first host registers");
    let again =
        register_host(Box::new(FakeHost(fake.clone()))).expect_err("a second host is refused");
    assert_eq!(again.code(), codes::BAD_STATE);

    // `enabled: false` binds nothing.
    answer(&fake, Answer::Bind);
    runtime
        .start(&config(&topics, json!({"enabled": false})))
        .expect("a start with the plane off");
    assert_eq!(log(&fake), Vec::<String>::new());
    assert_eq!(status(&runtime).remote, None);
    runtime.stop().unwrap();

    // A fresh install has no keys: the start fails closed with the named
    // code, the host never serves and nothing of the run is left.
    answer(&fake, Answer::NotProvisioned);
    let error = runtime
        .start(&config(&topics, enabled.clone()))
        .expect_err("an unprovisioned plane fails the start");
    assert_eq!(error.code(), codes::REMOTE_NOT_PROVISIONED);
    assert!(
        error
            .to_string()
            .contains("no device key under the state directory"),
        "error: {error}"
    );
    assert_eq!(log(&fake), ["remote.bind"]);
    let failed = status(&runtime);
    assert_eq!(failed.state, RuntimeState::Failed);
    assert_eq!(failed.remote, None);
    assert_eq!(
        failed.last_error.map(|last| last.code),
        Some(codes::REMOTE_NOT_PROVISIONED)
    );
    assert_eq!(publish_refused(&mut runtime, &topics.cmd), codes::BAD_STATE);

    // Any other bind failure is the remote code.
    answer(&fake, Answer::Fail);
    let error = runtime
        .start(&config(&topics, enabled.clone()))
        .expect_err("a failed bind fails the start");
    assert_eq!(error.code(), codes::REMOTE);
    assert_eq!(log(&fake), ["remote.bind"]);

    // A bound plane serves at the commit and reports its endpoint. The host
    // gets the state root under the files directory, the relay, the robot
    // and the run's schema serving: graph outputs, then publish topics.
    answer(&fake, Answer::Bind);
    runtime
        .start(&config(
            &topics,
            json!({"enabled": true, "relay": "disabled", "state_dir": "plane/one"}),
        ))
        .expect("a provisioned plane starts");
    assert_eq!(log(&fake), ["remote.bind", "remote.serve"]);
    assert_eq!(
        fake.lock().unwrap().handed,
        [Handed {
            state_root: files.path().join("plane").join("one"),
            robot: ROBOT.into(),
            relay: RemoteRelay::Disabled,
            served_topics: vec![topics.echo.clone(), topics.cmd.clone()],
            topic_schemas: vec![
                (topics.echo.clone(), VECTOR3.into()),
                (topics.cmd.clone(), VECTOR3.into()),
            ],
        }]
    );
    let running = status(&runtime);
    assert_eq!(running.state, RuntimeState::Running);
    assert_eq!(
        running.remote,
        Some(RemoteStatus {
            endpoint_id: ENDPOINT.into(),
            bound_sockets: vec![SOCKET.into()],
        })
    );
    // The plane takes no gateway: a start with no listen endpoints and no
    // connect locators leaves the LAN gateway free.
    assert!(!cerulion_android::runtime::gateway_owned_for_test());

    // Serving that ends on its own fails the run with the remote code and
    // withdraws the endpoint from status.
    let exit = fake
        .lock()
        .unwrap()
        .exit
        .take()
        .expect("serve kept the exit");
    exit("the endpoint closed".into());
    let ended = status(&runtime);
    assert_eq!(ended.state, RuntimeState::Failed);
    assert_eq!(ended.remote, None);
    let last = ended.last_error.expect("the exit is recorded");
    assert_eq!(last.code, codes::REMOTE);
    assert!(last.message.contains("the endpoint closed"), "{last:?}");
    runtime.stop().ok();

    // A config the plan refuses never reaches the host.
    answer(&fake, Answer::Bind);
    let unlinked = json!({
        "graph_yaml": "prefix: unlinked\nnodes:\n  - id: a\n    type: not_linked\n",
        "remote": enabled.clone(),
    });
    let error = runtime
        .start(&unlinked.to_string())
        .expect_err("an unlinked node type fails the start");
    assert_eq!(error.code(), codes::GRAPH, "{error}");
    assert_eq!(log(&fake), Vec::<String>::new());

    // A component that fails after the bind stops the bound plane: the
    // start fails with that component's code and leaves no plane behind.
    // A recording file that already exists fails the recorder's build step.
    answer(&fake, Answer::Bind);
    std::fs::create_dir_all(files.path().join("bags")).unwrap();
    std::fs::write(files.path().join("bags").join("taken.mcap"), b"").unwrap();
    let recorded = json!({
        "publish": [{"topic": topics.cmd, "schema": VECTOR3}],
        "record": {"file": "taken.mcap"},
        "remote": enabled.clone(),
    });
    let error = runtime
        .start(&recorded.to_string())
        .expect_err("an existing recording file fails the start");
    assert_eq!(error.code(), codes::RECORDER, "{error}");
    assert_eq!(log(&fake), ["remote.bind", "remote.stop"]);
    assert_eq!(status(&runtime).remote, None);

    // A plane that refuses to serve fails the commit with the remote code
    // and is stopped.
    answer(&fake, Answer::BindServeFails);
    let error = runtime
        .start(&config(&topics, enabled.clone()))
        .expect_err("a plane that cannot serve fails the start");
    assert_eq!(error.code(), codes::REMOTE);
    assert!(error.to_string().contains("receipt log"), "{error}");
    assert_eq!(log(&fake), ["remote.bind", "remote.serve", "remote.stop"]);
    let failed = status(&runtime);
    assert_eq!((failed.state, failed.remote), (RuntimeState::Failed, None));

    // A plane that does not stop cleanly fails the stop: the state is not
    // `stopped` while an endpoint may still be open.
    answer(&fake, Answer::BindStopFails);
    runtime
        .start(&config(&topics, enabled.clone()))
        .expect("the plane serves");
    let error = runtime.stop().expect_err("the plane did not stop");
    assert_eq!(error.code(), codes::REMOTE);
    let failed = status(&runtime);
    assert_eq!((failed.state, failed.remote), (RuntimeState::Failed, None));

    // A graph that finished on its own keeps its plane serving until the
    // next start; a refused restart still stops it and withdraws it.
    answer(&fake, Answer::Bind);
    std::env::set_var("CER_PERIOD_REQUEST_SHUTDOWN", "1");
    let finishing = json!({
        "graph_yaml": "prefix: finishing\nnodes:\n  - id: ticker\n    type: test_node_macro_period_cdylib\n    outputs:\n      - name: cmd\n        schema: geometry_msgs/Vector3\n",
        "remote": enabled.clone(),
    });
    runtime
        .start(&finishing.to_string())
        .expect("a graph that finishes starts");
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
    while status(&runtime).state != RuntimeState::Finished {
        assert!(std::time::Instant::now() < deadline, "the graph finishes");
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
    std::env::remove_var("CER_PERIOD_REQUEST_SHUTDOWN");
    assert!(status(&runtime).remote.is_some(), "the plane still serves");
    let refused = runtime
        .start(&config(
            &topics,
            json!({"enabled": true, "state_dir": "../outside"}),
        ))
        .expect_err("a state_dir outside the files directory is refused");
    assert_eq!(refused.code(), codes::INVALID_ARGUMENT);
    assert_eq!(log(&fake), ["remote.bind", "remote.serve", "remote.stop"]);
    assert_eq!(status(&runtime).remote, None);

    // Stop stops the plane before the graph: no desk reads a run that is
    // going down. The default state directory is `remote`.
    answer(&fake, Answer::Bind);
    let events = fake.clone();
    runtime.set_graph_exit_hook_for_test(move |_trace| {
        events.lock().unwrap().log.push("graph.exit".into());
    });
    runtime
        .start(&config(&topics, enabled))
        .expect("a provisioned plane starts again");
    assert_eq!(
        fake.lock().unwrap().handed[0].state_root,
        files.path().join("remote")
    );
    runtime.stop().expect("stop");
    assert_eq!(
        log(&fake),
        ["remote.bind", "remote.serve", "remote.stop", "graph.exit"]
    );
    let stopped = status(&runtime);
    assert_eq!(stopped.state, RuntimeState::Stopped);
    assert_eq!(stopped.remote, None);
}

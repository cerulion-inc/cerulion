// SPDX-License-Identifier: AGPL-3.0-only
//! A graph the app runs is described to a desk: the `runs` verb lists it, with
//! its graph, its endpoints and its topology, and the `fires` verb serves when
//! each of its nodes ticked.
//!
//! The desk runs in a re-executed child process: one zenoh session per
//! process. It dials the app's loopback listener and answers each request
//! file the parent writes (`req.<n>`: `runs`, or `fires <run_id> <after>`)
//! with the payloads of the replies it received (`resp.<n>`). The parent
//! starts, stops and restarts the runtime, and decodes and asserts on every
//! reply.
//!
//! The node registry is process state, so this file holds ONE test, which
//! runs its steps in order. It runs under both transports: the default
//! cross-process one, and the in-process one an app uses
//! (`--features local-transport`).

use std::fs::{self, File};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use cerulion_android::{EmbeddedRuntime, RuntimeState, Status};
use cerulion_core::fire_tap::{FireCause, FireFlags, FireTapStats};
use cerulion_core::graph::{parse_graph, GraphConfig};
use cerulion_core::transport::cerulion_q::{
    decode_fires_reply, decode_runs_reply, fires_selector, parse_run_id, runs_selector,
    FiresOutcome, FiresReply, RunEntry, RunEntryState, RunsCompleteness, RunsReply,
};
use serde_json::{json, Value};
use tempfile::tempdir;
use zenoh::Wait;

cerulion_android::embed_nodes! {
    "test_node_macro_period_cdylib" => test_node_macro_period_cdylib::PeriodNodeEntry,
    "test_node_macro_relay_rlib" => test_node_macro_relay_rlib::RelayNodeEntry,
}

const JNI_VERSION_1_6: i32 = 0x0001_0006;
const ROBOT: &str = "embedded_hosted_run_test";
const VECTOR3: &str = "geometry_msgs/Vector3";
const APP_IN: &str = "/hosted/app/in";
const SOURCE_CMD: &str = "/hosted/source/cmd";
const RELAY_ECHO: &str = "/hosted/relay/echo";

const DESK_DIR_ENV: &str = "CERULION_HOSTED_RUN_DESK_DIR";
const DESK_PORT_ENV: &str = "CERULION_HOSTED_RUN_DESK_PORT";
const DESK_ROBOT_ENV: &str = "CERULION_HOSTED_RUN_DESK_ROBOT";
/// How long the parent asks again before it gives up on a reply.
const WIRE_CEILING: Duration = Duration::from_secs(30);

/// A period source (every 50 ms) and a relay that the source's output
/// triggers, both linked into this binary; `name` heads the YAML when given.
fn graph_yaml(name: Option<&str>) -> String {
    let name = name.map_or_else(String::new, |name| format!("name: {name}\n"));
    format!(
        "{name}prefix: hosted
nodes:
  - id: source
    type: test_node_macro_period_cdylib
    outputs:
      - name: cmd
        schema: {VECTOR3}
  - id: relay
    type: test_node_macro_relay_rlib
    inputs:
      - name: cmd
        source: source/cmd
    outputs:
      - name: echo
        schema: {VECTOR3}
"
    )
}

fn config(listen: &str, lan_access: &str, graph: Option<String>) -> String {
    let mut config = json!({
        "listen": [listen],
        "lan_access": lan_access,
        "publish": [{"topic": APP_IN, "schema": "std_msgs/String"}],
    });
    if let Some(graph) = graph {
        config["graph_yaml"] = json!(graph);
        config["subscribe"] = json!([{"topic": RELAY_ECHO, "schema": VECTOR3}]);
    }
    config.to_string()
}

fn free_port() -> u16 {
    let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("bind ephemeral port");
    listener.local_addr().expect("local_addr").port()
}

fn wall_clock_ns() -> u64 {
    let since_epoch = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("the wall clock is after the Unix epoch");
    u64::try_from(since_epoch.as_nanos()).expect("wall clock ns fit in u64")
}

/// Write `contents` to `dir/name` in one step, so a reader polling for the
/// name never reads half of it.
fn publish_file(dir: &Path, name: &str, contents: &[u8]) {
    let staging = dir.join(format!("{name}.part"));
    fs::write(&staging, contents).expect("write staging file");
    fs::rename(&staging, dir.join(name)).expect("publish file");
}

/// DESK ROLE: a client session that dials the app's listener. For each
/// `req.<n>`, in order, it sends the GET the request names to the robot the
/// parent named and writes the payload of every reply to `resp.<n>`, as a
/// JSON array of strings. `quit` ends the role.
#[test]
#[ignore = "desk role; spawned by the parent test"]
fn desk_role() {
    let dir = PathBuf::from(std::env::var_os(DESK_DIR_ENV).expect(DESK_DIR_ENV));
    let port: u16 = std::env::var(DESK_PORT_ENV)
        .expect(DESK_PORT_ENV)
        .parse()
        .expect("port");
    let robot = std::env::var(DESK_ROBOT_ENV).expect(DESK_ROBOT_ENV);
    let mut config = zenoh::Config::default();
    config.insert_json5("mode", r#""client""#).unwrap();
    config
        .insert_json5("connect/endpoints", &format!(r#"["tcp/127.0.0.1:{port}"]"#))
        .unwrap();
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .unwrap();
    let session = zenoh::open(config).wait().expect("open desk session");
    for n in 0.. {
        let request_path = dir.join(format!("req.{n}"));
        let deadline = Instant::now() + Duration::from_secs(120);
        while !request_path.exists() {
            assert!(Instant::now() < deadline, "request {n} never arrived");
            std::thread::sleep(Duration::from_millis(5));
        }
        let request = fs::read_to_string(&request_path).expect("read request");
        let selector = match request.split_whitespace().collect::<Vec<_>>().as_slice() {
            ["quit"] => return,
            ["runs"] => runs_selector(&robot),
            ["fires", run_id, "-"] => fires_selector(&robot, run_id, None),
            ["fires", run_id, after] => {
                fires_selector(&robot, run_id, Some(after.parse().expect("after")))
            }
            other => panic!("unknown request {other:?}"),
        };
        let replies = session
            .get(&selector)
            .timeout(Duration::from_secs(1))
            .wait()
            .expect("send GET");
        let mut payloads = Vec::new();
        while let Ok(reply) = replies.recv() {
            if let Ok(sample) = reply.into_result() {
                payloads.push(String::from_utf8_lossy(&sample.payload().to_bytes()).into_owned());
            }
        }
        let response = serde_json::to_vec(&payloads).expect("encode response");
        publish_file(&dir, &format!("resp.{n}"), &response);
    }
}

/// The parent's handle on the desk process. Dropping it stops the process
/// if it is still running.
struct Desk {
    child: Child,
    dir: PathBuf,
    log: PathBuf,
    next: u64,
}

impl Drop for Desk {
    fn drop(&mut self) {
        if matches!(self.child.try_wait(), Ok(None)) {
            let _ = self.child.kill();
            let _ = self.child.wait();
        }
    }
}

impl Desk {
    fn spawn(dir: &Path, port: u16) -> Self {
        let log = dir.join("desk.log");
        let stdout = File::create(&log).expect("create desk log");
        let stderr = stdout.try_clone().expect("clone desk log");
        let child = Command::new(std::env::current_exe().expect("current test executable"))
            .args([
                "--exact",
                "desk_role",
                "--include-ignored",
                "--nocapture",
                "--test-threads=1",
            ])
            .env(DESK_DIR_ENV, dir)
            .env(DESK_PORT_ENV, port.to_string())
            .env(DESK_ROBOT_ENV, ROBOT)
            .stdout(Stdio::from(stdout))
            .stderr(Stdio::from(stderr))
            .spawn()
            .expect("spawn desk process");
        Self {
            child,
            dir: dir.to_path_buf(),
            log,
            next: 0,
        }
    }

    fn log_text(&self) -> String {
        fs::read_to_string(&self.log).unwrap_or_default()
    }

    /// Send one request and return the payload of every reply it got.
    fn ask(&mut self, request: &str) -> Vec<String> {
        let n = self.next;
        self.next += 1;
        publish_file(&self.dir, &format!("req.{n}"), request.as_bytes());
        let response = self.dir.join(format!("resp.{n}"));
        let deadline = Instant::now() + Duration::from_secs(30);
        while !response.exists() {
            if let Some(status) = self.child.try_wait().expect("desk try_wait") {
                panic!("desk exited with {status}:\n{}", self.log_text());
            }
            assert!(
                Instant::now() < deadline,
                "desk did not answer '{request}':\n{}",
                self.log_text()
            );
            std::thread::sleep(Duration::from_millis(5));
        }
        serde_json::from_slice(&fs::read(&response).expect("read response"))
            .expect("decode response")
    }

    /// GET `runs` until the decoded reply satisfies `done`.
    fn runs_until(&mut self, done: impl Fn(&RunsReply) -> bool) -> RunsReply {
        let deadline = Instant::now() + WIRE_CEILING;
        let mut last = String::from("no reply");
        while Instant::now() < deadline {
            match self.ask("runs").as_slice() {
                [payload] => match decode_runs_reply(payload.as_bytes()) {
                    Ok(reply) if done(&reply) => return reply,
                    other => last = format!("{other:?}"),
                },
                other => last = format!("{} replies", other.len()),
            }
            std::thread::sleep(Duration::from_millis(50));
        }
        panic!("no matching runs reply within {WIRE_CEILING:?}; last {last}");
    }

    /// One `fires` GET that the robot answers.
    fn fires(&mut self, run_id: &str, after: Option<u64>) -> FiresReply {
        let after = after.map_or_else(|| "-".to_string(), |after| after.to_string());
        let deadline = Instant::now() + WIRE_CEILING;
        loop {
            match self.ask(&format!("fires {run_id} {after}")).as_slice() {
                [payload] => {
                    return decode_fires_reply(payload.as_bytes()).expect("a decodable fires reply")
                }
                [] if Instant::now() < deadline => std::thread::sleep(Duration::from_millis(50)),
                other => panic!("expected one fires reply, got {other:?}"),
            }
        }
    }

    fn quit(mut self) {
        let n = self.next;
        publish_file(&self.dir, &format!("req.{n}"), b"quit");
        let status = self.child.wait().expect("reap desk");
        assert!(
            status.success(),
            "desk failed with {status}:\n{}",
            self.log_text()
        );
    }
}

/// The status as the app's `status` call reads it: a clone taken under the
/// status lock.
fn read_status(runtime: &EmbeddedRuntime) -> Status {
    runtime.status_handle().lock().unwrap().clone()
}

fn fire_stats(runtime: &EmbeddedRuntime) -> Option<FireTapStats> {
    read_status(runtime).fire_telemetry.stats()
}

fn status_json(runtime: &EmbeddedRuntime) -> Value {
    serde_json::to_value(read_status(runtime)).expect("status JSON")
}

/// Each node of `graph` with its type, its inputs as `(name, source)` and
/// its outputs as `(name, schema)`.
type NodeWiring<'a> = (
    &'a str,
    &'a str,
    Vec<(&'a str, &'a str)>,
    Vec<(&'a str, &'a str)>,
);

fn wiring(graph: &GraphConfig) -> Vec<NodeWiring<'_>> {
    graph
        .nodes
        .iter()
        .map(|node| {
            (
                node.id.as_str(),
                node.node_type.as_str(),
                node.inputs
                    .iter()
                    .map(|input| (input.name.as_str(), input.source.as_str()))
                    .collect(),
                node.outputs
                    .iter()
                    .map(|output| (output.name.as_str(), output.schema.as_str()))
                    .collect(),
            )
        })
        .collect()
}

/// The only run of a reply that serves exactly one.
fn only_run(reply: &RunsReply) -> &RunEntry {
    match reply.runs.as_slice() {
        [entry] => entry,
        other => panic!("expected one run, got {other:?}"),
    }
}

/// The `run.json` a start of [`graph_yaml`] with `config`'s endpoints hosts,
/// with the values that change per run taken from `entry`.
fn expected_run_json(entry: &RunEntry, graph_name: &str, lan_access: &str) -> Value {
    json!({
        "version": 1,
        "run_id": entry.run_id,
        "graph_name": graph_name,
        "graph_file": graph_name,
        "supervisor_pid": std::process::id(),
        "run_started_at_ns": entry.run_started_at_ns,
        "host": "embedded",
        "lan_access": lan_access,
        "endpoints": [{
            "id": "app",
            "publishes": [{"topic": APP_IN, "schema": "std_msgs/String"}],
            "subscribes": [{"topic": RELAY_ECHO, "schema": VECTOR3}],
        }],
        "topology": {
            "version": 1,
            "nodes": [
                {
                    "id": "source",
                    "type": "test_node_macro_period_cdylib",
                    "trigger": {"kind": "period", "interval_ns": 50_000_000},
                    "inputs": [],
                    "outputs": [{"name": "cmd", "topic": SOURCE_CMD}],
                    "reports_writes": true,
                },
                {
                    "id": "relay",
                    "type": "test_node_macro_relay_rlib",
                    "trigger": {"kind": "data", "topics": [SOURCE_CMD], "ambiguous": false},
                    "inputs": [{"name": "cmd", "topic": SOURCE_CMD}],
                    "outputs": [{"name": "echo", "topic": RELAY_ECHO}],
                    "reports_writes": true,
                },
            ],
        },
        "telemetry": {"fires": {"version": 1, "capacity": 4096, "lease_ms": 5000}},
    })
}

/// The `next` and records of a served fires reply.
fn served(reply: &FiresReply) -> (u64, u64, &[cerulion_core::fire_tap::FireRecord]) {
    match &reply.outcome {
        FiresOutcome::Served {
            next, lost, fires, ..
        } => (*next, *lost, fires.as_slice()),
        other => panic!("expected served fires, got {other:?}"),
    }
}

#[test]
fn an_embedded_graph_is_hosted_as_a_run_with_its_topology_and_node_fires() {
    assert_eq!(
        JNI_OnLoad(std::ptr::null_mut(), std::ptr::null_mut()),
        JNI_VERSION_1_6
    );
    let temp = tempdir().unwrap();
    let desk_dir = tempdir().unwrap();
    let port = free_port();
    let listen = format!("tcp/127.0.0.1:{port}");
    let mut runtime = EmbeddedRuntime::new(temp.path().to_path_buf(), ROBOT.into()).unwrap();
    assert_eq!(
        fire_stats(&runtime),
        None,
        "no run is hosted before a start"
    );
    assert_eq!(status_json(&runtime)["fire_telemetry"], Value::Null);

    // 1. An open start with a named graph hosts one live run.
    let before = wall_clock_ns();
    runtime
        .start(&config(
            &listen,
            "open",
            Some(graph_yaml(Some("hosted_graph"))),
        ))
        .expect("start the named graph");
    let after = wall_clock_ns();
    assert_eq!(
        fire_stats(&runtime),
        Some(FireTapStats {
            armed: false,
            recorded: 0,
            capacity: 4096
        }),
        "the tap records nothing until a desk asks"
    );
    assert_eq!(
        status_json(&runtime)["fire_telemetry"],
        json!({"armed": false, "recorded": 0, "capacity": 4096})
    );
    let mut desk = Desk::spawn(desk_dir.path(), port);
    let reply = desk.runs_until(|reply| reply.error.is_none() && !reply.runs.is_empty());
    assert_eq!(reply.robot, ROBOT);
    assert_eq!(reply.completeness, RunsCompleteness::Settled);
    assert!(reply.undescribable.is_empty(), "{reply:?}");
    let first = only_run(&reply).clone();
    assert!(parse_run_id(&first.run_id).is_some(), "{}", first.run_id);
    assert_eq!(first.graph_name, "hosted_graph");
    assert_eq!(first.state, RunEntryState::Live);
    assert!(
        (before..=after).contains(&first.run_started_at_ns),
        "started at {} ns, outside the start call [{before}, {after}]",
        first.run_started_at_ns
    );
    let hosted_graph = parse_graph(&first.graph_yaml).expect("the hosted graph.yaml parses");
    assert_eq!(hosted_graph.identity(), "hosted_graph");
    assert_eq!(hosted_graph.prefix, "hosted");
    assert_eq!(
        wiring(&hosted_graph),
        [
            (
                "source",
                "test_node_macro_period_cdylib",
                vec![],
                vec![("cmd", VECTOR3)]
            ),
            (
                "relay",
                "test_node_macro_relay_rlib",
                vec![("cmd", "source/cmd")],
                vec![("echo", VECTOR3)]
            ),
        ]
    );
    let started_graph =
        parse_graph(&graph_yaml(Some("hosted_graph"))).expect("the started graph parses");
    assert_eq!(
        serde_json::to_value(&hosted_graph).expect("hosted graph JSON"),
        serde_json::to_value(&started_graph).expect("started graph JSON"),
        "the hosted graph.yaml parses to the graph the app started:\n{}",
        first.graph_yaml
    );
    assert_eq!(
        runtime.hosted_run_id_for_test().as_deref(),
        Some(first.run_id.as_str())
    );
    let run_json: Value = serde_json::from_str(&first.run_json).expect("run.json is JSON");
    assert_eq!(
        run_json,
        expected_run_json(&first, "hosted_graph", "open"),
        "{}",
        first.run_json
    );

    // 2. Two fires GETs 300 ms apart: the first arms the tap and reads from
    //    now, the second reads every tick of both nodes since.
    let from_now = desk.fires(&first.run_id, None);
    let (start, _, none) = served(&from_now);
    assert!(none.is_empty(), "a read from now holds no fire: {none:?}");
    assert!(fire_stats(&runtime).is_some_and(|stats| stats.armed));
    std::thread::sleep(Duration::from_millis(300));
    let page = desk.fires(&first.run_id, Some(start));
    let asked = Instant::now();
    assert_eq!(page.robot, ROBOT);
    assert_eq!(page.run_id, first.run_id);
    let (next, lost, fires) = served(&page);
    assert_eq!(lost, 0, "{fires:?}");
    assert_eq!(fires.len() as u64, next - start, "{fires:?}");
    for (index, record) in fires.iter().enumerate() {
        assert_eq!(record.seq, start + 1 + index as u64, "{fires:?}");
        let cause = match record.node {
            0 => FireCause::Period,
            1 => FireCause::Data,
            other => panic!("node {other} is not in the topology: {record:?}"),
        };
        assert_eq!(record.cause, cause, "{record:?}");
        assert_eq!(record.flags, FireFlags(0), "{record:?}");
        assert_eq!(
            record.wrote, 0b1,
            "each tick wrote its one output: {record:?}"
        );
        assert!(record.dur_ns > 0, "{record:?}");
    }
    for node in [0, 1] {
        assert!(
            fires.iter().any(|record| record.node == node),
            "node {node} did not fire in 300 ms: {fires:?}"
        );
    }

    // 3. The lease: armed while a desk asks, disarmed 5 s after the last GET.
    let stats = fire_stats(&runtime).expect("a hosted run reports its tap");
    assert!(stats.armed, "{stats:?}");
    assert!(stats.recorded >= next, "{stats:?}");
    while fire_stats(&runtime).is_some_and(|stats| stats.armed) {
        assert!(
            asked.elapsed() < Duration::from_secs(15),
            "the tap stayed armed long after the lease"
        );
        std::thread::sleep(Duration::from_millis(50));
    }
    assert!(
        asked.elapsed() >= Duration::from_secs(4),
        "the tap disarmed {:?} after the last GET, before its 5 s lease",
        asked.elapsed()
    );

    // 4. Stop withdraws the run: an open start without a graph serves the
    //    settled empty answer, and the stopped run is unknown.
    runtime.stop().expect("stop the named graph");
    assert_eq!(fire_stats(&runtime), None, "a stopped run is not hosted");
    assert_eq!(runtime.hosted_run_id_for_test(), None);
    runtime
        .start(&config(&listen, "open", None))
        .expect("start without a graph");
    assert_eq!(
        fire_stats(&runtime),
        None,
        "a start without a graph hosts no run"
    );
    let empty = desk.runs_until(|reply| reply.error.is_none() && reply.runs.is_empty());
    assert_eq!(empty.completeness, RunsCompleteness::Settled);
    assert!(empty.undescribable.is_empty(), "{empty:?}");
    assert_eq!(
        desk.fires(&first.run_id, Some(next)).outcome,
        FiresOutcome::UnknownRun
    );
    runtime.stop().expect("stop the start without a graph");

    // 5. A graph whose thread exits is served as ending, and an unnamed graph
    //    is named `unnamed`. The source asks for shutdown on its first tick;
    //    it reads the variable from the environment the graph was built in.
    std::env::set_var("CER_PERIOD_REQUEST_SHUTDOWN", "1");
    let started = runtime.start(&config(&listen, "open", Some(graph_yaml(None))));
    std::env::remove_var("CER_PERIOD_REQUEST_SHUTDOWN");
    started.expect("start the unnamed graph");
    let deadline = Instant::now() + Duration::from_secs(10);
    while runtime.status_handle().lock().unwrap().state != RuntimeState::Finished {
        assert!(Instant::now() < deadline, "the graph did not finish");
        std::thread::sleep(Duration::from_millis(10));
    }
    let ending = desk.runs_until(|reply| {
        reply
            .runs
            .iter()
            .any(|entry| entry.state == RunEntryState::Ending)
    });
    let second = only_run(&ending);
    assert_eq!(second.graph_name, "unnamed");
    assert_ne!(second.run_id, first.run_id, "a restart mints a new run id");
    let run_json: Value = serde_json::from_str(&second.run_json).expect("run.json is JSON");
    assert_eq!(run_json["graph_name"], "unnamed");
    assert_eq!(run_json["graph_file"], "unnamed");
    assert_eq!(run_json["run_id"], second.run_id.as_str());
    runtime.stop().expect("stop the finished graph");

    // 6. Paired: the run is hosted, and both verbs refuse the desk; a fires
    //    GET that names the hosted run does not arm its tap.
    runtime
        .start(&config(
            &listen,
            "paired",
            Some(graph_yaml(Some("hosted_graph"))),
        ))
        .expect("start the paired graph");
    let paired = runtime
        .hosted_run_id_for_test()
        .expect("a paired start hosts its run");
    assert_ne!(paired, first.run_id);
    let refused = desk.runs_until(|reply| reply.error.is_some());
    assert!(refused.runs.is_empty(), "{refused:?}");
    let reason = refused.error.as_deref().unwrap_or_default();
    assert!(reason.contains("paired"), "{reason}");
    for after in [None, Some(0)] {
        let outcome = desk.fires(&paired, after).outcome;
        assert!(
            matches!(&outcome, FiresOutcome::Refused { reason } if reason.contains("paired")),
            "a paired robot refuses fires from {after:?}: {outcome:?}"
        );
    }
    assert_eq!(
        fire_stats(&runtime),
        Some(FireTapStats {
            armed: false,
            recorded: 0,
            capacity: 4096
        }),
        "a refused GET does not arm the tap"
    );
    runtime.stop().expect("stop the paired graph");
    desk.quit();
}

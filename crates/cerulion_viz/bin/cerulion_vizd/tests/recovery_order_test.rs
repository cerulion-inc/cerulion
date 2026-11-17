// SPDX-License-Identifier: AGPL-3.0-only
//! Serialized netd recovery must let a reachable source recover before an offline one.

#![cfg(unix)]

use std::collections::BTreeMap;
use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::{UnixListener, UnixStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cerulion_core::transport::network::{IngressInjector, NetworkConfig, ReinjectOutcome};
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::{CatalogReply, SchemaReply, TransportConfig, TransportManager};
use cerulion_viz::schema_registry::builtin_walker;
use cerulion_viz::sink::SinkState;
use cerulion_viz::worker::VizLogWorker;
use cerulion_vizd::{start_with_demand_plane, DemandPlane, NetdDemandPlane, DEFAULT_POLL_INTERVAL};
use serde_json::{json, Value};

type Held = Arc<Mutex<BTreeMap<usize, (String, Arc<IngressInjector>)>>>;
type Trace = Arc<Mutex<BTreeMap<String, Vec<(usize, String)>>>>;

struct Directory(std::path::PathBuf);
impl Directory {
    fn new(run: usize) -> Self {
        use std::os::unix::fs::DirBuilderExt;
        let path =
            std::path::PathBuf::from(format!("/tmp/cer-recovery-{}-{run}", std::process::id()));
        std::fs::DirBuilder::new()
            .mode(0o700)
            .create(&path)
            .unwrap();
        Self(path)
    }
    fn path(&self) -> &std::path::Path {
        &self.0
    }
}
impl Drop for Directory {
    fn drop(&mut self) {
        let result = std::fs::remove_dir_all(&self.0);
        if !std::thread::panicking() {
            result.unwrap();
        }
    }
}

struct Client {
    writer: UnixStream,
    reader: BufReader<UnixStream>,
    id: u64,
}
impl Client {
    fn new(path: &std::path::Path) -> Self {
        let writer = UnixStream::connect(path).unwrap();
        writer
            .set_read_timeout(Some(Duration::from_secs(5)))
            .unwrap();
        let mut reader = BufReader::new(writer.try_clone().unwrap());
        let mut hello = String::new();
        reader.read_line(&mut hello).unwrap();
        Self {
            writer,
            reader,
            id: 0,
        }
    }
    fn request(&mut self, mut request: Value) -> Value {
        self.id += 1;
        request["id"] = json!(self.id);
        writeln!(self.writer, "{request}").unwrap();
        let mut line = String::new();
        assert_ne!(self.reader.read_line(&mut line).unwrap(), 0);
        let reply: Value = serde_json::from_str(&line).unwrap();
        assert_eq!(reply["id"], self.id);
        reply
    }
}

// Drive each real recovery pass explicitly so its ordering is observable.
// The actual vizd still attaches, drains, decodes and serves samples normally.
struct ManualHealth(Arc<NetdDemandPlane>);
impl DemandPlane for ManualHealth {
    fn demand(&self, robot: &str, topic: &str, hash: u64) -> Result<(), String> {
        self.0.demand(robot, topic, hash)
    }
    fn release(&self, robot: &str, topic: &str) -> Result<(), String> {
        self.0.release(robot, topic)
    }
    fn local_channel(&self, robot: &str, topic: &str) -> Result<String, String> {
        self.0.local_channel(robot, topic)
    }
    fn query_catalog(&self, robot: &str) -> Result<Vec<CatalogReply>, String> {
        self.0.query_catalog(robot)
    }
    fn query_catalog_all(&self) -> Result<Vec<CatalogReply>, String> {
        self.0.query_catalog_all()
    }
    fn query_schema(&self, robot: &str, name: &str) -> Result<Vec<SchemaReply>, String> {
        self.0.query_schema(robot, name)
    }
}

struct Spy {
    running: Arc<AtomicBool>,
    lose: Arc<AtomicBool>,
    busy: Arc<AtomicBool>,
    held: Held,
    trace: Trace,
    thread: Option<std::thread::JoinHandle<()>>,
}
impl Spy {
    fn start(path: &std::path::Path, manager: Arc<TransportManager>, hash: u64) -> Self {
        let listener = UnixListener::bind(path).unwrap();
        listener.set_nonblocking(true).unwrap();
        let running = Arc::new(AtomicBool::new(true));
        let lose = Arc::new(AtomicBool::new(false));
        let busy = Arc::new(AtomicBool::new(false));
        let held: Held = Arc::new(Mutex::new(BTreeMap::new()));
        let trace: Trace = Arc::new(Mutex::new(BTreeMap::new()));
        let gate = Arc::new(Mutex::new(()));
        let (r, l, b, h, t) = (
            running.clone(),
            lose.clone(),
            busy.clone(),
            held.clone(),
            trace.clone(),
        );
        let thread = std::thread::spawn(move || {
            let mut workers = Vec::new();
            let mut connection = 0;
            while r.load(Ordering::Acquire) {
                let (mut stream, _) = match listener.accept() {
                    Ok(pair) => pair,
                    Err(error) if error.kind() == std::io::ErrorKind::WouldBlock => {
                        std::thread::sleep(Duration::from_millis(2));
                        continue;
                    }
                    Err(error) => panic!("spy accept: {error}"),
                };
                connection += 1;
                let id = connection;
                let (r, l, b, h, t, gate, manager) = (
                    r.clone(),
                    l.clone(),
                    b.clone(),
                    h.clone(),
                    t.clone(),
                    gate.clone(),
                    manager.clone(),
                );
                workers.push(std::thread::spawn(move || {
                    stream.set_nonblocking(false).unwrap();
                    stream.set_read_timeout(Some(Duration::from_millis(50))).unwrap();
                    writeln!(stream, "{}", cerulion_netd::Hello::new().to_json_line()).unwrap();
                    let mut reader = BufReader::new(stream.try_clone().unwrap());
                    let mut source: Option<String> = None;
                    while r.load(Ordering::Acquire) {
                        if l.load(Ordering::Acquire) && source.as_deref().is_some_and(|s| s == "B" || s == "C") {
                            let robot = source.as_ref().unwrap();
                            let topic = format!("/{}", robot.to_lowercase());
                            h.lock().unwrap().remove(&id);
                            t.lock().unwrap().entry(robot.clone()).or_default().push((id, "lost".into()));
                            let _ = writeln!(stream, "{}", json!({"event":"mirror_lost","robot":robot,"topic":topic}));
                            break;
                        }
                        let mut line = String::new();
                        match reader.read_line(&mut line) {
                            Ok(0) => break,
                            Ok(_) => {}
                            Err(error) if matches!(error.kind(), std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut) => continue,
                            Err(error) => panic!("spy request: {error}"),
                        }
                        let request: Value = serde_json::from_str(&line).unwrap();
                        let request_id = request["id"].as_u64().unwrap();
                        let reply = match request["method"].as_str().unwrap() {
                            "observe" => {
                                // The same registry lock serializes observe and status.
                                let _registry = gate.lock().unwrap();
                                let robot = request["robot"].as_str().unwrap().to_string();
                                if robot == "B" && l.load(Ordering::Acquire) {
                                    b.store(true, Ordering::Release);
                                    t.lock().unwrap().entry(robot.clone()).or_default().push((id, "offline".into()));
                                    std::thread::sleep(Duration::from_secs(12));
                                    b.store(false, Ordering::Release);
                                    json!({"id":request_id,"error":"B is offline","robot":robot,"topic":"/b"})
                                } else {
                                    let topic = request["topic"].as_str().unwrap();
                                    let injector = manager.create_remote_ingress_injector(topic, hash, MaxSliceLen::const_new(48)).unwrap();
                                    h.lock().unwrap().insert(id, (robot.clone(), Arc::new(injector)));
                                    t.lock().unwrap().entry(robot.clone()).or_default().push((id, "observed".into()));
                                    // Only original holders receive the simulated restart loss.
                                    if !l.load(Ordering::Acquire) { source = Some(robot.clone()); }
                                    json!({"id":request_id,"robot":robot,"topic":topic,"refcount":1,"mirror_created":true,"local_topic":topic})
                                }
                            }
                            "status" => {
                                let _registry = gate.lock().unwrap();
                                let rows: Vec<_> = h.lock().unwrap().values().map(|(robot, _)| json!({"robot":robot,"topic":format!("/{}",robot.to_lowercase()),"refcount":1,"schema_hash":hash,"local_topic":format!("/{}",robot.to_lowercase())})).collect();
                                json!({"id":request_id,"demands":rows,"active_connections":3,"idle":false})
                            }
                            "unobserve" => {
                                let robot = request["robot"].as_str().unwrap();
                                h.lock().unwrap().remove(&id);
                                t.lock().unwrap().entry(robot.into()).or_default().push((id, "released".into()));
                                json!({"id":request_id,"robot":robot,"topic":request["topic"],"refcount":0,"last_release":true})
                            }
                            other => panic!("unexpected spy request: {other}"),
                        };
                        if writeln!(stream, "{reply}").is_err() { break; }
                    }
                    h.lock().unwrap().remove(&id);
                }));
            }
            for worker in workers {
                worker.join().unwrap();
            }
        });
        Self {
            running,
            lose,
            busy,
            held,
            trace,
            thread: Some(thread),
        }
    }
    fn holder(&self, robot: &str) -> Option<usize> {
        let ids: Vec<_> = self
            .held
            .lock()
            .unwrap()
            .iter()
            .filter(|(_, (r, _))| r == robot)
            .map(|(id, _)| *id)
            .collect();
        assert!(ids.len() <= 1, "source owns multiple mirrors");
        ids.first().copied()
    }
    fn send(&self, robot: &str, frame: &[u8]) {
        let injector = self
            .held
            .lock()
            .unwrap()
            .values()
            .find(|(r, _)| r == robot)
            .map(|(_, injector)| injector.clone())
            .expect("source must have a real mirror before delivery");
        assert!(matches!(
            injector.reinject_raw(frame),
            ReinjectOutcome::Injected { .. }
        ));
    }
}
impl Drop for Spy {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Release);
        if let Some(thread) = self.thread.take() {
            thread.join().unwrap();
        }
    }
}

fn wait(mut condition: impl FnMut() -> bool, budget: Duration) -> bool {
    let deadline = Instant::now() + budget;
    while !condition() {
        if Instant::now() >= deadline {
            return false;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    true
}

fn frame(hash: u64, seq: u32, payload: [u8; 8]) -> [u8; 48] {
    let mut frame = [0; 48];
    frame[..8].copy_from_slice(&hash.to_le_bytes());
    frame[8] = 48;
    frame[12] = 48;
    frame[20..24].copy_from_slice(&seq.to_le_bytes());
    frame[40..].copy_from_slice(&payload);
    frame
}

fn delivered(client: &mut Client, topic: &str, seq: u32, data: f64) {
    assert!(
        wait(
            || {
                let reply = client.request(json!({"method":"sample","topic":topic,"n":1}));
                if reply["rows"][0]["seq"] != seq {
                    return false;
                }
                assert_eq!(reply["rows"][0]["fields"], json!({"data":data}));
                true
            },
            Duration::from_secs(2)
        ),
        "vizd did not deliver {topic} sequence {seq}"
    );
}

#[test]
fn reachable_lost_source_recovers_before_offline_retry_and_vizd_delivers_oracle_frames() {
    let mut traces = Vec::new();
    for run in 0..2 {
        let root = Directory::new(run);
        let mut config =
            serde_json::to_value(cerulion_core::testing::iceoryx_test_config()).unwrap();
        config["global"]["root-path"] = json!(root.path().join("shm"));
        let manager = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("viz-recovery-{run}"),
                network: Some(NetworkConfig {
                    listen_endpoints: vec!["tcp/127.0.0.1:0".into()],
                    multicast_scouting: false,
                    gossip_scouting: false,
                    ..Default::default()
                }),
                ..Default::default()
            },
            serde_json::from_value(config).unwrap(),
        )
        .unwrap();
        let hash = builtin_walker()
            .schema_hash_for("std_msgs/Float64")
            .unwrap();
        let socket = root.path().join("netd.sock");
        let spy = Spy::start(&socket, manager.clone(), hash);
        let plane = Arc::new(NetdDemandPlane::with_socket(socket));
        let (recording, _storage) = rerun::RecordingStreamBuilder::new("recovery_order")
            .memory()
            .unwrap();
        let worker = VizLogWorker::spawn(recording, builtin_walker(), SinkState::new()).unwrap();
        let viz_socket = root.path().join("viz.sock");
        let mut daemon = start_with_demand_plane(
            viz_socket.clone(),
            DEFAULT_POLL_INTERVAL,
            manager.clone(),
            worker,
            builtin_walker(),
            None,
            Arc::new(ManualHealth(plane.clone())),
        )
        .unwrap();
        let mut client = Client::new(&viz_socket);
        for (robot, topic) in [("A", "/a"), ("B", "/b"), ("C", "/c")] {
            let reply = client.request(
                json!({"method":"attach","robot":robot,"topic":topic,"schema":"std_msgs/Float64"}),
            );
            assert_eq!(reply["ok"], true, "{reply}");
            assert_eq!(
                client.request(json!({"method":"sample","topic":topic}))["ok"],
                true
            );
        }
        let a = spy.holder("A").unwrap();
        let c = spy.holder("C").unwrap();
        spy.lose.store(true, Ordering::Release);
        assert!(wait(
            || spy.holder("B").is_none() && spy.holder("C").is_none(),
            Duration::from_secs(2)
        ));
        let first_plane = plane.clone();
        let first = std::thread::spawn(move || first_plane.check_remote_readers());
        assert!(wait(
            || spy.busy.load(Ordering::Acquire),
            Duration::from_secs(2)
        ));
        assert_eq!(
            client.request(json!({"method":"sample","topic":"/a"}))["ok"],
            true
        );
        spy.send("A", &frame(hash, 0, [0, 0, 0, 0, 0, 0, 0xf8, 0x3f]));
        delivered(&mut client, "/a", 0, 1.5);
        assert_eq!(spy.holder("A"), Some(a));
        let (sent, received) = std::sync::mpsc::channel();
        let local_plane = plane.clone();
        let lookup =
            std::thread::spawn(move || sent.send(local_plane.local_channel("A", "/a")).unwrap());
        let unlocked = received.recv_timeout(Duration::from_millis(250)).is_ok();
        assert!(first.join().unwrap().is_err());
        lookup.join().unwrap();
        assert!(wait(
            || !spy.busy.load(Ordering::Acquire),
            Duration::from_secs(15)
        ));
        let second_plane = plane.clone();
        let second = std::thread::spawn(move || second_plane.check_remote_readers());
        let recovered = wait(|| spy.holder("C").is_some(), Duration::from_secs(2));
        if recovered {
            assert_ne!(
                spy.holder("C"),
                Some(c),
                "C must own a fresh holding connection"
            );
            assert_eq!(
                client.request(json!({"method":"sample","topic":"/c"}))["ok"],
                true
            );
            spy.send("C", &frame(hash, 0, [0, 0, 0, 0, 0, 0, 0x04, 0xc0]));
            delivered(&mut client, "/c", 0, -2.5);
        }
        assert!(wait(
            || spy.busy.load(Ordering::Acquire),
            Duration::from_secs(2)
        ));
        assert_eq!(
            client.request(json!({"method":"sample","topic":"/a"}))["ok"],
            true
        );
        spy.send("A", &frame(hash, 1, [0, 0, 0, 0, 0, 0, 0x0c, 0x40]));
        delivered(&mut client, "/a", 1, 3.5);
        assert_eq!(spy.holder("A"), Some(a));
        assert!(second.join().unwrap().is_err());
        assert!(wait(
            || !spy.busy.load(Ordering::Acquire),
            Duration::from_secs(15)
        ));
        daemon.shutdown();
        drop(client);
        drop(plane);
        let trace = spy.trace.lock().unwrap().clone();
        drop(spy);
        assert!(
            recovered,
            "C stayed dark in the first non-busy pass while B timed out again"
        );
        assert!(
            unlocked,
            "source lookup held the map lock behind B's observe"
        );
        // Compare real per-source connection/outcome history, normalized by each
        // source's first-seen connection ids, rather than comparing payload oracles.
        let normalized: BTreeMap<_, _> = trace
            .into_iter()
            .map(|(robot, entries)| {
                let mut ids = Vec::new();
                let events: Vec<_> = entries
                    .into_iter()
                    .map(|(id, event)| {
                        let position = match ids.iter().position(|old| *old == id) {
                            Some(n) => n,
                            None => {
                                ids.push(id);
                                ids.len() - 1
                            }
                        };
                        (position, event)
                    })
                    .collect();
                (robot, events)
            })
            .collect();
        assert_eq!(
            normalized["B"]
                .iter()
                .filter(|(_, event)| event == "offline")
                .count(),
            2
        );
        assert_eq!(
            normalized["C"]
                .iter()
                .filter(|(_, event)| event == "observed")
                .count(),
            2
        );
        traces.push(normalized);
    }
    assert_eq!(traces[0], traces[1]);
}

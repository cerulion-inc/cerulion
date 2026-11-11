// SPDX-License-Identifier: AGPL-3.0-only
//! A stalled source recovery must preserve other sources' held readers and delivery.

#![cfg(unix)]

#[path = "support/idle_spy.rs"]
mod idle_spy;

use std::collections::{BTreeMap, BTreeSet};
use std::io::{BufReader, Write};
use std::os::unix::net::UnixListener;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cerulion_core::wire::MaxSliceLen;
use cerulion_core::{TransportConfig, TransportManager};
use cerulion_vizd::{DemandPlane, NetdDemandPlane};
use serde_json::{json, Value};

struct Directory(std::path::PathBuf);
impl Directory {
    fn new(run: usize) -> Self {
        use std::os::unix::fs::DirBuilderExt;
        let path =
            std::path::PathBuf::from(format!("/tmp/cer-source-{}-{run}", std::process::id()));
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

struct Spy {
    running: Arc<AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
}
impl Spy {
    fn finish(mut self) {
        self.running.store(false, Ordering::Release);
        self.thread.take().unwrap().join().unwrap();
    }
}
impl Drop for Spy {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Release);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

#[test]
fn offline_source_reobserve_preserves_healthy_source_refcount_and_oracle_delivery() {
    let mut results = Vec::new();
    for run in 0..2 {
        let root = Directory::new(run);
        let socket = root.path().join("netd.sock");
        let listener = UnixListener::bind(&socket).unwrap();
        listener.set_nonblocking(true).unwrap();
        let running = Arc::new(AtomicBool::new(true));
        let busy = Arc::new(AtomicBool::new(true));
        let attempts = Arc::new(AtomicUsize::new(0));
        let held = Arc::new(Mutex::new(BTreeMap::<usize, BTreeSet<String>>::new()));
        let server_running = running.clone();
        let server_busy = busy.clone();
        let server_attempts = attempts.clone();
        let server_held = held.clone();
        let server = std::thread::spawn(move || {
            let mut workers = Vec::new();
            let mut next = 0;
            while server_running.load(Ordering::Acquire) {
                let (mut stream, _) = match listener.accept() {
                    Ok(pair) => pair,
                    Err(error) if error.kind() == std::io::ErrorKind::WouldBlock => {
                        std::thread::sleep(Duration::from_millis(2));
                        continue;
                    }
                    Err(error) => panic!("spy accept failed: {error}"),
                };
                next += 1;
                let id = next;
                let held = server_held.clone();
                let busy = server_busy.clone();
                let attempts = server_attempts.clone();
                let worker_running = server_running.clone();
                workers.push(std::thread::spawn(move || {
                    stream.set_nonblocking(false).unwrap();
                    stream.set_read_timeout(Some(Duration::from_millis(200))).unwrap();
                    writeln!(stream, "{}", cerulion_netd::Hello::new().to_json_line()).unwrap();
                    let mut reader = BufReader::new(stream.try_clone().unwrap());
                    while let Some(line) = idle_spy::read_request(&mut reader, &worker_running) {
                        let request: Value = serde_json::from_str(&line).unwrap();
                        let request_id = request["id"].as_u64().unwrap();
                        let reply = match request["method"].as_str().unwrap() {
                            "observe" => {
                                let robot = request["robot"].as_str().unwrap();
                                if robot == "B" && attempts.fetch_add(1, Ordering::AcqRel) > 0 {
                                    std::thread::sleep(Duration::from_secs(6));
                                    json!({"id":request_id,"error":"B is offline","robot":"B","topic":"/b"})
                                } else {
                                    held.lock().unwrap().entry(id).or_default().insert(robot.into());
                                    let reply = json!({"id":request_id,"robot":robot,"topic":request["topic"],
                                        "refcount":1,"mirror_created":true,"local_topic":request["topic"]});
                                    writeln!(stream, "{reply}").unwrap();
                                    if robot == "B" {
                                        writeln!(stream, "{{\"event\":\"mirror_lost\",\"robot\":\"B\",\"topic\":\"/b\"}}").unwrap();
                                    }
                                    continue;
                                }
                            }
                            "status" => {
                                if busy.load(Ordering::Acquire) {
                                    std::thread::sleep(Duration::from_millis(300));
                                }
                                json!({"id":request_id,"demands":[
                                    {"robot":"A","topic":"/a","refcount":1,"schema_hash":7,"local_topic":"/a"},
                                    {"robot":"B","topic":"/b","refcount":1,"schema_hash":7,"local_topic":"/b"}],
                                    "active_connections":2,"idle":false})
                            }
                            "unobserve" => {
                                let robot = request["robot"].as_str().unwrap();
                                held.lock().unwrap().entry(id).or_default().remove(robot);
                                json!({"id":request_id,"robot":robot,"topic":request["topic"],"refcount":0,"last_release":true})
                            }
                            other => panic!("unexpected spy method {other}"),
                        };
                        if writeln!(stream, "{reply}").is_err() {
                            break;
                        }
                    }
                    held.lock().unwrap().remove(&id);
                }));
            }
            for worker in workers {
                worker.join().unwrap();
            }
        });
        let spy = Spy {
            running: running.clone(),
            thread: Some(server),
        };
        let plane = Arc::new(NetdDemandPlane::with_socket(socket));
        plane.demand("A", "/a", 7).unwrap();
        plane.demand("B", "/b", 7).unwrap();
        // A busy probe must not start the known-lost B's blocking first demand.
        let busy_result = plane.check_remote_readers();
        let busy_attempts = attempts.load(Ordering::Acquire);
        busy.store(false, Ordering::Release);
        let recovery_plane = plane.clone();
        let recovery = std::thread::spawn(move || recovery_plane.check_remote_readers());
        let mut config =
            serde_json::to_value(cerulion_core::testing::iceoryx_test_config()).unwrap();
        config["global"]["root-path"] = json!(root.path().join("shm"));
        let manager = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("viz-source-{run}"),
                ..Default::default()
            },
            serde_json::from_value(config).unwrap(),
        )
        .unwrap();
        let mut publisher = manager
            .create_publisher_simple("/a", MaxSliceLen::const_new(42))
            .unwrap();
        let mut tap = manager.create_data_only_subscriber("/a").unwrap();
        let mut observed = Vec::new();
        let started = Instant::now();
        let mut kept = true;
        for sequence in 0..70_u32 {
            let refcount = held
                .lock()
                .unwrap()
                .values()
                .filter(|sources| sources.contains("A"))
                .count();
            kept &= refcount == 1;
            if refcount == 1 {
                // Literal v2 header and hand-written two-byte payload oracle.
                let mut frame = [0_u8; 42];
                frame[0] = 7;
                frame[8] = 42;
                frame[12] = 42;
                frame[20..24].copy_from_slice(&sequence.to_le_bytes());
                frame[40..].copy_from_slice(&[0x31, 0x72]);
                publisher.publish_raw(&frame).unwrap();
                let deadline = Instant::now() + Duration::from_secs(1);
                let mut batch = Vec::new();
                loop {
                    tap.drain_owned(1, &mut batch).unwrap();
                    if let Some(sample) = batch.pop() {
                        assert_eq!(sample.payload(), frame);
                        observed.push(sequence);
                        break;
                    }
                    assert!(
                        Instant::now() < deadline,
                        "A stopped delivering its oracle frame"
                    );
                    std::thread::sleep(Duration::from_millis(2));
                }
            }
            std::thread::sleep(Duration::from_millis(100));
        }
        let recovered = recovery.join().unwrap();
        let final_count = held
            .lock()
            .unwrap()
            .values()
            .filter(|sources| sources.contains("A"))
            .count();
        drop(plane);
        spy.finish();
        assert!(started.elapsed() >= Duration::from_secs(6));
        assert!(kept, "B's timeout released A's holding connection");
        assert_eq!(final_count, 1);
        assert!(
            busy_result.is_ok(),
            "busy probe started recovery: {busy_result:?}"
        );
        assert_eq!(busy_attempts, 1, "busy pass must not re-observe B");
        assert!(
            recovered.is_err(),
            "offline B must report its recovery failure"
        );
        assert_eq!(observed, (0..70).collect::<Vec<_>>());
        results.push(observed);
    }
    assert_eq!(results[0], results[1]);
}

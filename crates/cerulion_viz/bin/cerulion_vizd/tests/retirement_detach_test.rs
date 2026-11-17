// SPDX-License-Identifier: AGPL-3.0-only
//! A retired observer must never be re-demanded after detach, even if release fails.

#![cfg(unix)]

#[path = "support/idle_spy.rs"]
mod idle_spy;

use std::io::{BufReader, Write};
use std::os::unix::net::UnixListener;
use std::time::Duration;

struct Directory(std::path::PathBuf);
impl Directory {
    fn new() -> Self {
        use std::os::unix::fs::DirBuilderExt;
        use std::sync::atomic::{AtomicUsize, Ordering};
        static NEXT: AtomicUsize = AtomicUsize::new(0);
        let path = std::path::PathBuf::from(format!(
            "/tmp/cer-viz-retire-{}-{}",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        ));
        std::fs::DirBuilder::new()
            .mode(0o700)
            .create(&path)
            .unwrap();
        Self(path)
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

use cerulion_vizd::{DemandPlane, NetdDemandPlane};
use serde_json::{json, Value};

#[test]
fn retirement_during_detach_removes_recovery_before_release_io() {
    cancellation_keeps_no_recovery_intent(false);
}

#[test]
fn changed_channel_cancellation_removes_recovery_before_release_io() {
    cancellation_keeps_no_recovery_intent(true);
}

#[test]
fn detach_during_reobserve_removes_intent_before_the_pending_reply() {
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
    use std::sync::Arc;
    use std::time::Instant;

    for _ in 0..2 {
        let root = Directory::new();
        let socket = root.0.join("control.sock");
        let listener = UnixListener::bind(&socket).unwrap();
        let started = Arc::new(AtomicBool::new(false));
        let finish = Arc::new(AtomicBool::new(false));
        let (server_started, server_finish) = (started.clone(), finish.clone());
        let observes = AtomicUsize::new(0);
        let (server, methods) = spy(listener, move |request, stream| {
            let id = request["id"].as_u64().unwrap();
            match request["method"].as_str().unwrap() {
                "observe" => {
                    if observes.fetch_add(1, Ordering::AcqRel) == 0 {
                        writeln!(
                            stream,
                            "{{\"event\":\"mirror_lost\",\"robot\":\"arm\",\"topic\":\"/vector\"}}"
                        )
                        .unwrap();
                    } else {
                        server_started.store(true, Ordering::Release);
                        let deadline = Instant::now() + Duration::from_secs(3);
                        while !server_finish.load(Ordering::Acquire) && Instant::now() < deadline {
                            std::thread::sleep(Duration::from_millis(2));
                        }
                    }
                    json!({"id":id,"robot":"arm","topic":"/vector","refcount":1,
                        "mirror_created":true,"local_topic":"/vector"})
                }
                "unobserve" => json!({"id":id,"robot":"arm","topic":"/vector",
                    "refcount":0,"last_release":true}),
                "status" => json!({"id":id,"demands":[],"active_connections":1,"idle":false}),
                other => panic!("unexpected source request {other}"),
            }
        });
        let plane = Arc::new(NetdDemandPlane::with_socket(socket));
        plane.demand("arm", "/vector", 7).unwrap();
        let recovery_plane = plane.clone();
        let recovery = std::thread::spawn(move || recovery_plane.check_remote_readers());
        let deadline = Instant::now() + Duration::from_secs(2);
        while !started.load(Ordering::Acquire) {
            assert!(
                Instant::now() < deadline,
                "re-observe never reached the spy"
            );
            std::thread::sleep(Duration::from_millis(2));
        }
        let release_plane = plane.clone();
        let release = std::thread::spawn(move || release_plane.release("arm", "/vector"));
        let (sent, received) = std::sync::mpsc::channel();
        let lookup_plane = plane.clone();
        let lookup = std::thread::spawn(move || {
            let deadline = Instant::now() + Duration::from_secs(1);
            while lookup_plane.local_channel("arm", "/vector").is_ok() {
                if Instant::now() >= deadline {
                    sent.send(false).unwrap();
                    return;
                }
                std::thread::sleep(Duration::from_millis(2));
            }
            sent.send(true).unwrap();
        });
        let removed = received
            .recv_timeout(Duration::from_millis(250))
            .unwrap_or(false);
        finish.store(true, Ordering::Release);
        recovery.join().unwrap().unwrap();
        release.join().unwrap().unwrap();
        lookup.join().unwrap();
        plane.check_remote_readers().unwrap();
        plane.check_remote_readers().unwrap();
        drop(plane);
        server.finish();
        assert!(
            removed,
            "detach waited for observe before removing recovery intent"
        );
        let calls: Vec<_> = methods
            .lock()
            .unwrap()
            .iter()
            .map(|(method, _)| method.clone())
            .collect();
        assert_eq!(calls, ["observe", "observe", "unobserve"]);
    }
}

#[test]
fn demand_refusal_rolls_back_only_an_uninitialized_source() {
    use std::sync::atomic::{AtomicUsize, Ordering};
    for initial_refusal in [true, false] {
        for _ in 0..2 {
            let root = Directory::new();
            let socket = root.0.join("control.sock");
            let listener = UnixListener::bind(&socket).unwrap();
            let observes = AtomicUsize::new(0);
            let (server, methods) = spy(listener, move |request, _| {
                let id = request["id"].as_u64().unwrap();
                match request["method"].as_str().unwrap() {
                    "observe" => {
                        let first = observes.fetch_add(1, Ordering::AcqRel) == 0;
                        if first == initial_refusal {
                            json!({"id":id,"error":"source policy refused"})
                        } else {
                            json!({"id":id,"robot":"arm","topic":"/vector","refcount":1,
                                "mirror_created":true,"local_topic":"/vector"})
                        }
                    }
                    "unobserve" => json!({"id":id,"robot":"arm","topic":"/vector",
                        "refcount":0,"last_release":true}),
                    other => panic!("unexpected source request {other}"),
                }
            });
            let plane = NetdDemandPlane::with_socket(socket);
            let first = plane.demand("arm", "/vector", 7);
            if initial_refusal {
                assert!(first.unwrap_err().contains("source policy refused"));
                assert!(plane.local_channel("arm", "/vector").is_err());
                plane.check_remote_readers().unwrap();
                plane.demand("arm", "/vector", 7).unwrap();
            } else {
                first.unwrap();
                assert!(plane
                    .demand("arm", "/vector", 7)
                    .unwrap_err()
                    .contains("source policy refused"));
            }
            assert_eq!(plane.local_channel("arm", "/vector").unwrap(), "/vector");
            plane.release("arm", "/vector").unwrap();
            drop(plane);
            server.finish();
            let calls: Vec<_> = methods
                .lock()
                .unwrap()
                .iter()
                .map(|(method, _)| method.clone())
                .collect();
            assert_eq!(calls, ["observe", "observe", "unobserve"]);
        }
    }
}

#[test]
fn loss_during_fresh_observe_preserves_both_committed_demands() {
    let root = Directory::new();
    let socket = root.0.join("control.sock");
    let listener = UnixListener::bind(&socket).unwrap();
    let (server, methods) = spy(listener, move |request, stream| {
        let method = request["method"].as_str().unwrap();
        let topic = request["topic"].as_str().unwrap();
        let id = request["id"].as_u64().unwrap();
        match method {
            "observe" => {
                if topic == "/extra" {
                    writeln!(
                        stream,
                        "{{\"event\":\"mirror_lost\",\"robot\":\"arm\",\"topic\":\"/vector\"}}"
                    )
                    .unwrap();
                }
                json!({"id":id,"robot":"arm","topic":topic,"refcount":1,
                    "mirror_created":true,"local_topic":topic})
            }
            "unobserve" => json!({"id":id,"robot":"arm","topic":topic,
                "refcount":0,"last_release":true}),
            other => panic!("unexpected control method {other}"),
        }
    });
    let plane = NetdDemandPlane::with_socket(socket);
    plane.demand("arm", "/vector", 7).unwrap();
    plane.demand("arm", "/extra", 7).unwrap();
    plane.release("arm", "/extra").unwrap();
    plane.release("arm", "/vector").unwrap();
    drop(plane);
    assert_eq!(
        {
            server.finish();
            methods.lock().unwrap().clone()
        },
        [
            ("observe".into(), "/vector".into()),
            ("observe".into(), "/extra".into()),
            ("unobserve".into(), "/extra".into()),
            ("unobserve".into(), "/vector".into()),
        ]
    );
}

fn cancellation_keeps_no_recovery_intent(changed_channel: bool) {
    for lost in [true, false] {
        for _ in 0..2 {
            let root = Directory::new();
            let socket = root.0.join("control.sock");
            let listener = UnixListener::bind(&socket).unwrap();
            let observes = std::sync::atomic::AtomicUsize::new(0);
            let (server, methods) = spy(listener, move |request, stream| {
                let method = request["method"].as_str().unwrap();
                let id = request["id"].as_u64().unwrap();
                match method {
                    "observe" => {
                        let observes =
                            observes.fetch_add(1, std::sync::atomic::Ordering::AcqRel) + 1;
                        if changed_channel && observes == 1 {
                            writeln!(stream, "{{\"event\":\"mirror_lost\",\"robot\":\"arm\",\"topic\":\"/vector\"}}").unwrap();
                        }
                        assert_eq!(request["robot"], "arm");
                        assert_eq!(request["topic"], "/vector");
                        assert_eq!(request["schema_hash"], 7);
                        let channel = if changed_channel && observes > 1 {
                            "/__cerulion/observers/e756c0190570ccb5/70c0eb8e1f85cdd5"
                        } else {
                            "/vector"
                        };
                        json!({"id":id,"robot":"arm","topic":"/vector",
                                "refcount":1,"mirror_created":true,"local_topic":channel})
                    }
                    "unobserve" if lost => {
                        writeln!(
                            stream,
                            "{{\"event\":\"mirror_lost\",\"robot\":\"arm\",\"topic\":\"/vector\"}}"
                        )
                        .unwrap();
                        json!({"id":id,"robot":"arm","topic":"/vector","refcount":0,"last_release":true})
                    }
                    "unobserve" => json!({"id":id,"error":"release unavailable",
                            "robot":"arm","topic":"/vector"}),
                    "status" => {
                        json!({"id":id,"demands":[],"active_connections":1,"idle":false})
                    }
                    other => panic!("unexpected control method {other}"),
                }
            });
            let plane = NetdDemandPlane::with_socket(socket);
            plane.demand("arm", "/vector", 7).unwrap();
            let error = if changed_channel {
                plane.check_remote_readers().unwrap_err()
            } else {
                match plane.release("arm", "/vector") {
                    Ok(()) => "released arm /vector".into(),
                    Err(error) => error,
                }
            };
            assert!(
                error.contains("arm") && error.contains("/vector"),
                "{error}"
            );
            // Drive the production health entry point twice after the failed release.
            plane.check_remote_readers().unwrap();
            plane.check_remote_readers().unwrap();
            drop(plane);
            let expected = if changed_channel {
                vec!["observe", "observe", "unobserve"]
            } else {
                vec!["observe", "unobserve"]
            };
            server.finish();
            let actual: Vec<_> = methods
                .lock()
                .unwrap()
                .iter()
                .map(|(method, _)| method.clone())
                .collect();
            assert_eq!(actual, expected);
        }
    }
}

struct Spy {
    running: std::sync::Arc<std::sync::atomic::AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
}
impl Spy {
    fn finish(mut self) {
        self.running
            .store(false, std::sync::atomic::Ordering::Release);
        self.thread.take().unwrap().join().unwrap();
    }
}
impl Drop for Spy {
    fn drop(&mut self) {
        self.running
            .store(false, std::sync::atomic::Ordering::Release);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

type Methods = std::sync::Arc<std::sync::Mutex<Vec<(String, String)>>>;
fn spy(
    listener: UnixListener,
    reply: impl Fn(&Value, &mut std::os::unix::net::UnixStream) -> Value + Send + Sync + 'static,
) -> (Spy, Methods) {
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::{Arc, Mutex};
    listener.set_nonblocking(true).unwrap();
    let running = Arc::new(AtomicBool::new(true));
    let methods = Arc::new(Mutex::new(Vec::new()));
    let thread_running = running.clone();
    let thread_methods = methods.clone();
    let reply = Arc::new(reply);
    let thread = std::thread::spawn(move || {
        let mut workers = Vec::new();
        while thread_running.load(Ordering::Acquire) {
            let (mut stream, _) = match listener.accept() {
                Ok(pair) => pair,
                Err(error) if error.kind() == std::io::ErrorKind::WouldBlock => {
                    std::thread::sleep(Duration::from_millis(2));
                    continue;
                }
                Err(error) => panic!("spy accept failed: {error}"),
            };
            let methods = thread_methods.clone();
            let reply = reply.clone();
            let worker_running = thread_running.clone();
            workers.push(std::thread::spawn(move || {
                stream.set_nonblocking(false).unwrap();
                stream
                    .set_read_timeout(Some(Duration::from_millis(200)))
                    .unwrap();
                writeln!(stream, "{}", cerulion_netd::Hello::new().to_json_line()).unwrap();
                let mut reader = BufReader::new(stream.try_clone().unwrap());
                while let Some(line) = idle_spy::read_request(&mut reader, &worker_running) {
                    let request: Value = serde_json::from_str(&line).unwrap();
                    let method = request["method"].as_str().unwrap();
                    if method != "status" {
                        methods
                            .lock()
                            .unwrap()
                            .push((method.into(), request["topic"].as_str().unwrap().into()));
                    }
                    let response = reply(&request, &mut stream);
                    writeln!(stream, "{response}").unwrap();
                }
            }));
        }
        for worker in workers {
            worker.join().unwrap();
        }
    });
    (
        Spy {
            running,
            thread: Some(thread),
        },
        methods,
    )
}

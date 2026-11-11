// SPDX-License-Identifier: AGPL-3.0-only
//! A retired observer must never be re-demanded after detach, even if release fails.

#![cfg(unix)]

use std::io::{BufRead, BufReader, Write};
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
            workers.push(std::thread::spawn(move || {
                stream.set_nonblocking(false).unwrap();
                stream
                    .set_read_timeout(Some(Duration::from_secs(5)))
                    .unwrap();
                writeln!(stream, "{}", cerulion_netd::Hello::new().to_json_line()).unwrap();
                let mut reader = BufReader::new(stream.try_clone().unwrap());
                loop {
                    let mut line = String::new();
                    if reader.read_line(&mut line).unwrap() == 0 {
                        break;
                    }
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

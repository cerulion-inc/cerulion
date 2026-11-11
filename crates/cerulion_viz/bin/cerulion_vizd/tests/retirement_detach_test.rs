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
    let server = std::thread::spawn(move || {
        let (mut stream, _) = listener.accept().unwrap();
        stream
            .set_read_timeout(Some(Duration::from_secs(5)))
            .unwrap();
        writeln!(stream, "{}", cerulion_netd::Hello::new().to_json_line()).unwrap();
        let mut reader = BufReader::new(stream.try_clone().unwrap());
        let mut methods = Vec::new();
        loop {
            let mut line = String::new();
            if reader.read_line(&mut line).unwrap() == 0 {
                return methods;
            }
            let request: Value = serde_json::from_str(&line).unwrap();
            let method = request["method"].as_str().unwrap();
            let topic = request["topic"].as_str().unwrap();
            methods.push((method.to_owned(), topic.to_owned()));
            let id = request["id"].as_u64().unwrap();
            let reply = match method {
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
            };
            writeln!(stream, "{reply}").unwrap();
        }
    });
    let plane = NetdDemandPlane::with_socket(socket);
    plane.demand("arm", "/vector", 7).unwrap();
    plane.demand("arm", "/extra", 7).unwrap();
    plane.release("arm", "/extra").unwrap();
    plane.release("arm", "/vector").unwrap();
    drop(plane);
    assert_eq!(
        server.join().unwrap(),
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
            let server = std::thread::spawn(move || {
                let (mut stream, _) = listener.accept().unwrap();
                stream
                    .set_read_timeout(Some(Duration::from_secs(5)))
                    .unwrap();
                writeln!(stream, "{}", cerulion_netd::Hello::new().to_json_line()).unwrap();
                let mut reader = BufReader::new(stream.try_clone().unwrap());
                let mut methods = Vec::new();
                let mut observes = 0;
                loop {
                    let mut line = String::new();
                    if reader.read_line(&mut line).unwrap() == 0 {
                        return methods;
                    }
                    let request: Value = serde_json::from_str(&line).unwrap();
                    let method = request["method"].as_str().unwrap();
                    methods.push(method.to_owned());
                    let id = request["id"].as_u64().unwrap();
                    let reply = match method {
                        "observe" => {
                            observes += 1;
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
                            writeln!(stream, "{{\"event\":\"mirror_lost\",\"robot\":\"arm\",\"topic\":\"/vector\"}}").unwrap();
                            json!({"id":id,"robot":"arm","topic":"/vector","refcount":0,"last_release":true})
                        }
                        "unobserve" => json!({"id":id,"error":"release unavailable",
                            "robot":"arm","topic":"/vector"}),
                        "status" => {
                            json!({"id":id,"demands":[],"active_connections":1,"idle":false})
                        }
                        other => panic!("unexpected control method {other}"),
                    };
                    writeln!(stream, "{reply}").unwrap();
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
            assert_eq!(server.join().unwrap(), expected);
        }
    }
}

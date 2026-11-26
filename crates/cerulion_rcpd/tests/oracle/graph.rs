// SPDX-License-Identifier: AGPL-3.0-only
//! The stand-in graph the vectors run against.
//!
//! It reads every frame on the calls topic, oldest first, and records it
//! before it answers. It answers on the results topic with
//! `{"call_id", "tool"}`, between two decoy answers to no call
//! (`"call_id": 0`). It never answers `arm.mute`, and holds its answer to
//! `arm.slow` until [`Graph::release`].

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Duration;

use cerulion_core::message::ShmMessage;
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::{CerulionPublisher, CerulionSubscriber, TransportManager};
use native_ros2_messages::std_msgs::String as RosString;
use serde_json::{json, Value};
use tokio::sync::watch;

use super::publish_json;

/// How long a test waits for a frame before it fails.
const FRAME_DEADLINE: Duration = Duration::from_secs(30);

/// The stand-in graph's thread.
pub struct Graph {
    stop: Arc<AtomicBool>,
    release: Arc<AtomicBool>,
    frames: watch::Receiver<Vec<Value>>,
    handle: JoinHandle<()>,
}

/// The graph's ports and what it has received.
struct Ports {
    calls: CerulionSubscriber,
    results: CerulionPublisher,
    frames: watch::Sender<Vec<Value>>,
    /// `arm.slow` frames not yet answered.
    held: Vec<Value>,
}

impl Graph {
    /// Open the graph's ports on `/{prefix}/calls` and `/{prefix}/results`
    /// and start answering; returns once the ports exist.
    pub fn spawn(mgr: Arc<TransportManager>, prefix: &str) -> Self {
        let topic = |name: &str| format!("/{prefix}/{name}");
        let (calls_topic, results_topic) = (topic("calls"), topic("results"));
        let stop = Arc::new(AtomicBool::new(false));
        let release = Arc::new(AtomicBool::new(false));
        let (frames_tx, frames) = watch::channel(Vec::new());
        let (stopping, released) = (Arc::clone(&stop), Arc::clone(&release));
        let (ready_tx, ready_rx) = std::sync::mpsc::channel();
        let handle = std::thread::spawn(move || {
            let mut ports = Ports {
                calls: mgr
                    .create_subscriber(&calls_topic)
                    .expect("calls subscriber"),
                results: mgr
                    .create_publisher_simple(&results_topic, MaxSliceLen::const_new(4096))
                    .expect("results publisher"),
                frames: frames_tx,
                held: Vec::new(),
            };
            ready_tx.send(()).expect("ready");
            while !stopping.load(Ordering::Relaxed) {
                if !ports.pass(released.load(Ordering::Relaxed)) {
                    std::thread::sleep(Duration::from_millis(1));
                }
            }
            // A frame published just before the stop is still recorded.
            ports.pass(released.load(Ordering::Relaxed));
        });
        ready_rx.recv().expect("graph thread ready");
        Self {
            stop,
            release,
            frames,
            handle,
        }
    }

    /// Answer every held `arm.slow` frame, and later ones at once.
    pub fn release(&self) {
        self.release.store(true, Ordering::Relaxed);
    }

    /// Wait until the graph has received a frame for `tool`.
    pub async fn wait_for(&self, tool: &str) {
        let mut frames = self.frames.clone();
        let seen = frames.wait_for(|frames| frames.iter().any(|f| f["tool"] == tool));
        tokio::time::timeout(FRAME_DEADLINE, seen)
            .await
            .unwrap_or_else(|_| panic!("no {tool} frame reached the graph"))
            .expect("graph thread");
    }

    /// Stop the graph; every frame it received, in order.
    pub fn stop(self) -> Vec<Value> {
        self.stop.store(true, Ordering::Relaxed);
        self.handle.join().expect("graph thread");
        self.frames.borrow().clone()
    }
}

impl Ports {
    /// Record and answer every frame queued on the calls topic, and every
    /// held one once `released`; whether there was any.
    fn pass(&mut self, released: bool) -> bool {
        let mut batch = Vec::new();
        self.calls
            .try_receive(|m| {
                let text = RosString::build_reader(m.payload())
                    .data()
                    .expect("utf-8")
                    .to_owned();
                batch.push(serde_json::from_str::<Value>(&text).expect("call is JSON"));
            })
            .expect("receive");
        let mut any = !batch.is_empty();
        for call in batch {
            self.frames.send_modify(|frames| frames.push(call.clone()));
            if call["tool"] == "arm.mute" {
                continue;
            }
            if call["tool"] == "arm.slow" && !released {
                self.held.push(call);
                continue;
            }
            self.answer(&call);
        }
        if released && !self.held.is_empty() {
            any = true;
            for call in std::mem::take(&mut self.held) {
                self.answer(&call);
            }
        }
        any
    }

    /// Answer `call` between two decoys.
    fn answer(&mut self, call: &Value) {
        let decoy = json!({ "call_id": 0, "decoy": true });
        publish_json(&mut self.results, &decoy);
        publish_json(&mut self.results, &answer(call));
        publish_json(&mut self.results, &decoy);
    }
}

/// The graph's answer to the frame `call`.
pub fn answer(call: &Value) -> Value {
    json!({ "call_id": call["call_id"], "tool": call["tool"] })
}

// SPDX-License-Identifier: AGPL-3.0-only
//! The stand-in graph the vectors run against.
//!
//! It reads every frame on the calls topic, oldest first, and records it
//! before it answers. It answers on the results topic with
//! `{"call_id", "tool"}`, between two decoy answers to no call
//! (`"call_id": 0`). It never answers `arm.mute`.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Duration;

use cerulion_core::message::ShmMessage;
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::{CerulionPublisher, CerulionSubscriber, TransportManager};
use native_ros2_messages::std_msgs::String as RosString;
use serde_json::{json, Value};

use super::publish_json;

/// The stand-in graph's thread.
pub struct Graph {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<Vec<Value>>,
}

impl Graph {
    /// Open the graph's ports on `/{prefix}/calls` and `/{prefix}/results`
    /// and start answering; returns once the ports exist.
    pub fn spawn(mgr: Arc<TransportManager>, prefix: &str) -> Self {
        let calls_topic = format!("/{prefix}/calls");
        let results_topic = format!("/{prefix}/results");
        let stop = Arc::new(AtomicBool::new(false));
        let stopping = Arc::clone(&stop);
        let (ready_tx, ready_rx) = std::sync::mpsc::channel();
        let handle = std::thread::spawn(move || {
            let calls = mgr
                .create_subscriber(&calls_topic)
                .expect("calls subscriber");
            let mut results = mgr
                .create_publisher_simple(&results_topic, MaxSliceLen::const_new(4096))
                .expect("results publisher");
            ready_tx.send(()).expect("ready");
            let mut seen = Vec::new();
            while !stopping.load(Ordering::Relaxed) {
                if !answer_batch(&calls, &mut results, &mut seen) {
                    std::thread::sleep(Duration::from_millis(1));
                }
            }
            // A frame published just before the stop is still recorded.
            answer_batch(&calls, &mut results, &mut seen);
            seen
        });
        ready_rx.recv().expect("graph thread ready");
        Self { stop, handle }
    }

    /// Stop the graph; every frame it received, in order.
    pub fn stop(self) -> Vec<Value> {
        self.stop.store(true, Ordering::Relaxed);
        self.handle.join().expect("graph thread")
    }
}

/// Record and answer every frame queued on `calls`; whether there was one.
fn answer_batch(
    calls: &CerulionSubscriber,
    results: &mut CerulionPublisher,
    seen: &mut Vec<Value>,
) -> bool {
    let mut batch = Vec::new();
    calls
        .try_receive(|m| {
            let text = RosString::build_reader(m.payload())
                .data()
                .expect("utf-8")
                .to_owned();
            batch.push(serde_json::from_str::<Value>(&text).expect("call is JSON"));
        })
        .expect("receive");
    let any = !batch.is_empty();
    for call in batch {
        seen.push(call.clone());
        if call["tool"] == "arm.mute" {
            continue;
        }
        let decoy = json!({ "call_id": 0, "decoy": true });
        publish_json(results, &decoy);
        publish_json(results, &answer(&call));
        publish_json(results, &decoy);
    }
    any
}

/// The graph's answer to the frame `call`.
pub fn answer(call: &Value) -> Value {
    json!({ "call_id": call["call_id"], "tool": call["tool"] })
}

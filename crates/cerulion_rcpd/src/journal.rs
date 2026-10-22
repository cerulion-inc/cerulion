// SPDX-License-Identifier: AGPL-3.0-only
//! The journal: the one place tool calls enter the graph.
//!
//! A std thread owns the transport ports (they stay on the thread that made
//! them): one publisher on the manifest's calls topic, one subscriber on its
//! results topic, and one subscriber per `robot://state/{name}` topic. Every
//! frame is a `std_msgs/String` holding JSON.
//!
//! A call is published as `{"call_id", "tool", "args"}`. The graph answers on
//! the results topic with a JSON object carrying the same `call_id`. The
//! thread never blocks on an answer: it keeps every published call in a
//! pending table, reads EVERY frame on the results topic in arrival order, and
//! completes the call whose `call_id` a frame carries. A frame that matches no
//! pending call is dropped.
//!
//! Requests arrive on two queues. The urgent queue carries the e-stop and is
//! read before the normal queue on every pass of the loop, so an e-stop frame
//! is published within one pass (about a millisecond) whatever else is
//! pending. A latch-gated call (every mutating tool) is published only while
//! the shared e-stop latch is clear; the check runs immediately before the
//! publish.
//!
//! Because the calls topic is an input the graph consumes, a `--record` bag
//! holds every published call, and `cerulion bag play --resim all --verify`
//! re-injects them and recomputes the answers.

use std::collections::BTreeMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::sync::{Arc, Mutex, PoisonError};
use std::time::{Duration, Instant};

use cerulion_core::message::ShmMessage;
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::{CerulionPublisher, CerulionSubscriber, TransportManager};
use native_ros2_messages::std_msgs::String as RosString;
use serde_json::Value;
use tokio::sync::oneshot;

use crate::manifest::{JournalConfig, StateResource};
use crate::RcpdError;

/// The largest call frame the publisher can carry, bytes.
pub const MAX_CALL_BYTES: u32 = 64 * 1024;

/// The largest serialized call (`{"call_id", "tool", "args"}`) accepted for
/// publishing, bytes: [`MAX_CALL_BYTES`] less room for the wire header and
/// the string's offset table.
pub const MAX_CALL_JSON_BYTES: usize = MAX_CALL_BYTES as usize - 1024;

/// How long the loop sleeps when nothing is queued or pending.
const IDLE: Duration = Duration::from_millis(1);

/// Why a journaled call produced no answer.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum JournalError {
    /// The graph did not answer within the tool's timeout. The call WAS
    /// published, so the graph may still act on it.
    #[error("no answer on {topic} within {timeout_ms} ms")]
    Timeout {
        /// The results topic.
        topic: String,
        /// The timeout that lapsed.
        timeout_ms: u64,
    },
    /// The call could not be published.
    #[error("publish on {topic} failed: {reason}")]
    Publish {
        /// The calls topic.
        topic: String,
        /// The transport's explanation.
        reason: String,
    },
    /// The e-stop latch closed before the call was published; it was not.
    #[error("the e-stop engaged before the call was published")]
    Estopped,
    /// The journal thread is gone.
    #[error("the journal thread stopped")]
    Stopped,
}

struct CallRequest {
    call_id: u64,
    frame: String,
    timeout: Duration,
    /// Publish even while the e-stop latch is closed (the e-stop itself, and
    /// read-only tools).
    bypass_latch: bool,
    reply: oneshot::Sender<Result<Value, JournalError>>,
}

/// The newest frame per state resource name.
type NewestState = Arc<Mutex<BTreeMap<String, Option<String>>>>;

/// The handle async code uses to reach the journal thread.
#[derive(Debug, Clone)]
pub struct Journal {
    normal: mpsc::Sender<CallRequest>,
    urgent: mpsc::Sender<CallRequest>,
    state: NewestState,
}

impl Journal {
    /// Open the ports on a new thread; returns once they exist. Normal calls
    /// are refused while `latch` is set.
    ///
    /// # Errors
    ///
    /// [`RcpdError::Transport`] when a port cannot be created,
    /// [`RcpdError::ObserverSpawn`] when the thread cannot start.
    pub fn spawn(
        manager: Arc<TransportManager>,
        config: JournalConfig,
        state: Vec<StateResource>,
        latch: Arc<AtomicBool>,
    ) -> Result<Self, RcpdError> {
        let (normal, normal_rx) = mpsc::channel();
        let (urgent, urgent_rx) = mpsc::channel();
        let newest: NewestState = Arc::new(Mutex::new(
            state.iter().map(|s| (s.name.clone(), None)).collect(),
        ));
        let thread_newest = Arc::clone(&newest);
        let (ready_tx, ready_rx) = mpsc::sync_channel(1);
        std::thread::Builder::new()
            .name(String::from("rcp-journal"))
            .spawn(move || {
                let ports = match Ports::open(&manager, &config, &state, latch, thread_newest) {
                    Ok(ports) => {
                        let _ = ready_tx.send(Ok(()));
                        ports
                    }
                    Err(e) => {
                        let _ = ready_tx.send(Err(e));
                        return;
                    }
                };
                ports.run(&urgent_rx, &normal_rx);
            })
            .map_err(RcpdError::ObserverSpawn)?;
        match ready_rx.recv() {
            Ok(Ok(())) => Ok(Self {
                normal,
                urgent,
                state: newest,
            }),
            Ok(Err(e)) => Err(e),
            Err(_) => Err(RcpdError::Session {
                reason: String::from("the journal thread exited before opening its ports"),
            }),
        }
    }

    /// The serialized frame for one call, as it will be published.
    pub fn frame(call_id: u64, tool: &str, args: &Value) -> String {
        serde_json::json!({ "call_id": call_id, "tool": tool, "args": args }).to_string()
    }

    /// Publish one call and wait for the graph's answer to it. A
    /// `latch_gated` call is not published while the e-stop latch is closed.
    ///
    /// # Errors
    ///
    /// [`JournalError`] when the call is not published or not answered.
    pub async fn call(
        &self,
        call_id: u64,
        frame: String,
        timeout: Duration,
        latch_gated: bool,
    ) -> Result<Value, JournalError> {
        Self::submit(&self.normal, call_id, frame, timeout, !latch_gated).await
    }

    /// Publish the e-stop call ahead of everything queued, latch or not, and
    /// wait for the graph's answer.
    ///
    /// # Errors
    ///
    /// [`JournalError`] when the call is not published or not answered.
    pub async fn urgent(
        &self,
        call_id: u64,
        frame: String,
        timeout: Duration,
    ) -> Result<Value, JournalError> {
        Self::submit(&self.urgent, call_id, frame, timeout, true).await
    }

    async fn submit(
        queue: &mpsc::Sender<CallRequest>,
        call_id: u64,
        frame: String,
        timeout: Duration,
        bypass_latch: bool,
    ) -> Result<Value, JournalError> {
        let (reply, answer) = oneshot::channel();
        queue
            .send(CallRequest {
                call_id,
                frame,
                timeout,
                bypass_latch,
                reply,
            })
            .map_err(|_| JournalError::Stopped)?;
        answer.await.map_err(|_| JournalError::Stopped)?
    }

    /// The newest frame seen on the state topic named `name`; `None` when the
    /// name is unknown or nothing has arrived yet.
    pub fn state(&self, name: &str) -> Option<String> {
        self.state
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .get(name)
            .cloned()
            .flatten()
    }
}

struct Pending {
    deadline: Instant,
    timeout: Duration,
    reply: oneshot::Sender<Result<Value, JournalError>>,
}

struct Ports {
    calls_topic: String,
    results_topic: String,
    calls: CerulionPublisher,
    results: CerulionSubscriber,
    state: Vec<(String, CerulionSubscriber)>,
    newest: NewestState,
    latch: Arc<AtomicBool>,
    pending: BTreeMap<u64, Pending>,
}

/// The smallest `std_msgs/String` payload a reader can be built over: the
/// fixed section plus one 8-byte offset table entry per variable field.
const MIN_STRING_PAYLOAD: usize = RosString::WIRE_FIXED_SIZE + 8 * RosString::VARIABLE_FIELD_COUNT;

/// The text of one `std_msgs/String` payload, or why it was dropped.
///
/// The generated reader asserts on a payload too short for its offset table
/// or not 8-byte aligned, so both are refused first; anything else the reader
/// panics on is caught, so a malformed frame from any local publisher costs
/// that frame, never the journal thread.
fn decode_string(payload: &[u8]) -> Result<String, String> {
    if payload.len() < MIN_STRING_PAYLOAD {
        return Err(format!(
            "payload of {} bytes is shorter than the {MIN_STRING_PAYLOAD} a String needs",
            payload.len()
        ));
    }
    if payload.as_ptr().align_offset(8) != 0 {
        return Err(String::from("payload is not 8-byte aligned"));
    }
    std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        RosString::build_reader(payload)
            .data()
            .map(str::to_owned)
            .map_err(|e| e.to_string())
    }))
    .unwrap_or_else(|_| Err(String::from("the reader refused the payload")))
}

/// Decode every frame queued on `subscriber`, oldest first.
fn drain_strings(subscriber: &CerulionSubscriber, topic: &str) -> Vec<String> {
    let mut out = Vec::new();
    let read = subscriber.try_receive(|message| {
        if message.header().schema_hash != RosString::SCHEMA_HASH {
            tracing::warn!(topic, "dropped a frame that is not std_msgs/String");
            return;
        }
        match decode_string(message.payload()) {
            Ok(text) => out.push(text),
            Err(reason) => tracing::warn!(topic, %reason, "dropped a malformed frame"),
        }
    });
    if let Err(e) = read {
        tracing::warn!(topic, error = %e, "receive failed");
    }
    out
}

impl Ports {
    fn open(
        manager: &TransportManager,
        config: &JournalConfig,
        state: &[StateResource],
        latch: Arc<AtomicBool>,
        newest: NewestState,
    ) -> Result<Self, RcpdError> {
        let calls = manager
            .create_publisher_simple(&config.calls_topic, MaxSliceLen::const_new(MAX_CALL_BYTES))?;
        let results = manager.create_subscriber(&config.results_topic)?;
        let mut state_ports = Vec::new();
        for entry in state {
            state_ports.push((entry.name.clone(), manager.create_subscriber(&entry.topic)?));
        }
        let ports = Self {
            calls_topic: config.calls_topic.clone(),
            results_topic: config.results_topic.clone(),
            calls,
            results,
            state: state_ports,
            newest,
            latch,
            pending: BTreeMap::new(),
        };
        // Answers queued before this daemon started belong to no call of it.
        let stale = drain_strings(&ports.results, &ports.results_topic).len();
        if stale > 0 {
            tracing::info!(stale, "dropped answers queued before start");
        }
        Ok(ports)
    }

    fn run(mut self, urgent: &mpsc::Receiver<CallRequest>, normal: &mpsc::Receiver<CallRequest>) {
        loop {
            let mut busy = false;
            match urgent.try_recv() {
                Ok(request) => {
                    self.publish(request);
                    busy = true;
                }
                Err(mpsc::TryRecvError::Empty) => {}
                Err(mpsc::TryRecvError::Disconnected) => return,
            }
            if !busy {
                match normal.try_recv() {
                    Ok(request) => {
                        self.publish(request);
                        busy = true;
                    }
                    Err(mpsc::TryRecvError::Empty) => {}
                    Err(mpsc::TryRecvError::Disconnected) => return,
                }
            }
            busy |= self.collect_answers();
            self.expire();
            self.refresh_state();
            if !busy {
                std::thread::sleep(IDLE);
            }
        }
    }

    fn publish(&mut self, request: CallRequest) {
        if !request.bypass_latch && self.latch.load(Ordering::SeqCst) {
            let _ = request.reply.send(Err(JournalError::Estopped));
            return;
        }
        let topic = self.calls_topic.clone();
        let fail = |reason: String| JournalError::Publish { topic, reason };
        let published = match self.calls.loan_proxy::<RosString>() {
            // The frame is published when the proxy drops.
            Ok(mut proxy) => proxy
                .set_data(&request.frame)
                .map_err(|e| fail(e.to_string())),
            Err(e) => Err(fail(e.to_string())),
        };
        match published {
            Ok(()) => {
                self.pending.insert(
                    request.call_id,
                    Pending {
                        deadline: Instant::now() + request.timeout,
                        timeout: request.timeout,
                        reply: request.reply,
                    },
                );
            }
            Err(e) => {
                let _ = request.reply.send(Err(e));
            }
        }
    }

    /// Complete every pending call answered on the results topic. Returns
    /// whether any frame arrived.
    fn collect_answers(&mut self) -> bool {
        let frames = drain_strings(&self.results, &self.results_topic);
        let any = !frames.is_empty();
        for text in frames {
            let answer: Value = match serde_json::from_str(&text) {
                Ok(answer) => answer,
                Err(e) => {
                    tracing::warn!(error = %e, "dropped a non-JSON answer");
                    continue;
                }
            };
            let Some(call_id) = answer.get("call_id").and_then(Value::as_u64) else {
                tracing::warn!("dropped an answer without a call_id");
                continue;
            };
            match self.pending.remove(&call_id) {
                Some(pending) => {
                    let _ = pending.reply.send(Ok(answer));
                }
                None => tracing::debug!(call_id, "dropped an answer to no pending call"),
            }
        }
        any
    }

    fn expire(&mut self) {
        let now = Instant::now();
        let lapsed: Vec<u64> = self
            .pending
            .iter()
            .filter(|(_, p)| p.deadline <= now)
            .map(|(id, _)| *id)
            .collect();
        for call_id in lapsed {
            if let Some(pending) = self.pending.remove(&call_id) {
                let _ = pending.reply.send(Err(JournalError::Timeout {
                    topic: self.results_topic.clone(),
                    timeout_ms: u64::try_from(pending.timeout.as_millis()).unwrap_or(u64::MAX),
                }));
            }
        }
    }

    fn refresh_state(&mut self) {
        for (name, subscriber) in &self.state {
            if let Some(text) = drain_strings(subscriber, name).pop() {
                self.newest
                    .lock()
                    .unwrap_or_else(PoisonError::into_inner)
                    .insert(name.clone(), Some(text));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::decode_string;

    /// A 16-byte window of a zeroed buffer starting `skew` bytes past an
    /// 8-byte boundary.
    fn window(buf: &mut [u8], skew: usize) -> &mut [u8] {
        let start = buf.as_ptr().align_offset(8) + skew;
        &mut buf[start..start + 16]
    }

    #[test]
    fn short_and_misaligned_payloads_are_refused_not_panicked_on() {
        assert!(decode_string(&[]).is_err());
        let mut buf = vec![0u8; 64];
        assert!(decode_string(&window(&mut buf, 0)[..4]).is_err());
        // An offset table entry pointing far past the payload.
        let entry = window(&mut buf, 0);
        entry[..4].copy_from_slice(&1_000_000u32.to_le_bytes());
        entry[4..8].copy_from_slice(&1_000u32.to_le_bytes());
        assert!(decode_string(entry).is_err());
        let mut buf = vec![0u8; 64];
        assert!(decode_string(window(&mut buf, 1)).is_err());
    }
}

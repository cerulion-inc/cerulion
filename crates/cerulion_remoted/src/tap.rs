// SPDX-License-Identifier: AGPL-3.0-only
//! — the demand-driven SHM `TapManager`.
//!
//! `remoted`'s wire plane is a listener-less **tap set**, NOT the LAN gateway's
//! `AtomicBool` egress-flag machinery (which gates zenoh dual-publish within ONE
//! gateway process). A remote demand attaches `remoted`'s OWN
//! [`DataOnlySubscriber`] on its OWN `TransportManager`; the tap's EXISTENCE is
//! the gate. Un-demanding drops the tap — an un-demanded topic costs nothing
//! (mirrors the zenoh demand-token Delete). This is deliberate: the tap is
//! the only per-topic state the robot holds.
//!
//! Each demanded topic runs two cooperating halves, bridged by a bounded
//! drop-to-live [`ForwardChannel`] ([`crate::forward`]):
//!
//! - a **drain thread** (`std::thread`) owns the tap and POLLS SHM
//!   ([`DataOnlySubscriber::drain_owned`] is a non-blocking receive), copying
//!   each frame OUT of the SHM slot (releasing the zero-copy borrow at once — the
//!   taps must stay observation-only) into the channel;
//! - a **writer task** (`tokio`) pops frames and `write_frame`s them on the
//!   topic's own uni stream — it ALONE blocks on QUIC flow control.
//!
//! Dropping the tap (drain thread exit) releases the SHM subscriber slot; the
//! demand lifecycle IS the stream lifecycle.

use std::collections::HashMap;
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use cerulion_core::transport::subscriber::{DataOnlySubscriber, OwnedInboundSample};
use cerulion_core::TransportManager;
use cerulion_link::{open_uni_frame_stream, write_frame, Connection};
use tokio::task;

use crate::forward::{ForwardChannel, PushOutcome, WanDropLevel};
use crate::wire::{StreamPreamble, TopicStatus, WireError};

/// How long the drain thread sleeps between polls when the tap yields no frame —
/// bounds the per-hop tap latency at ~1 ms (crib `bagd`'s 1 ms drain-loop pace).
const DRAIN_IDLE_SLEEP: Duration = Duration::from_millis(1);

/// A belt-and-suspenders wake for the writer task so a (theoretical) lost
/// `notify_one` never wedges a demanded stream.
const WRITER_BACKSTOP: Duration = Duration::from_millis(50);

/// Grace at teardown for the writer task to finish a HEALTHY in-flight frame
/// before it is ABORTED. A healthy writer, woken by `channel.stop()`'s
/// `notify_waiters`, returns almost immediately; a writer BLOCKED inside a
/// flow-control-stalled `write_frame` (the slow-WAN steady state) can never
/// observe `stop()` — it re-checks only AFTER the write completes — so it MUST be
/// aborted, dropping its `SendStream` (resetting the QUIC stream, unblocking the
/// stalled `write_all`). Short so `undemand`/`shutdown` never wedge; drop-to-live
/// already tolerates losing the in-flight frame.
const WRITER_STOP_GRACE: Duration = Duration::from_millis(250);

/// The best-effort per-topic budget the `catalog` schema-hash PEEK spends
/// waiting for one live frame before giving up (`schema_hash = None`). Bounded so
/// a silent topic never stalls a catalog reply. A live producer's next publish
/// lands within a poll or two.
const CATALOG_PEEK_BUDGET: Duration = Duration::from_millis(750);

/// One demanded topic's live handle: the shared channel plus the two half-lives
/// (the drain thread + the writer task). Dropping it does NOT stop the halves —
/// call [`Self::stop`] to tear down cleanly (releasing the SHM slot).
struct DemandedTopic {
    channel: Arc<ForwardChannel>,
    drain: Option<JoinHandle<()>>,
    writer: task::JoinHandle<()>,
}

impl DemandedTopic {
    /// Whether the topic's forward stream is still LIVE (its writer has not
    /// self-terminated). The writer sets `channel.stop()` on death, so
    /// a still-mapped topic whose channel is stopped is a DEAD stream — an
    /// intentional teardown removes the entry from `demanded` before the writer's
    /// stop is ever observed here.
    fn is_alive(&self) -> bool {
        !self.channel.is_stopped()
    }

    /// Tear the topic down: signal both halves, JOIN the drain thread (so its
    /// tap is definitively dropped → the SHM subscriber slot is RELEASED before
    /// this returns — the undemand-releases-the-slot contract), then reap the
    /// writer under a BOUNDED grace.
    async fn stop(mut self) {
        self.channel.stop();
        if let Some(handle) = self.drain.take() {
            // Join off the async runtime — the drain loop exits within one poll
            // (DRAIN_IDLE_SLEEP) of the stop flag; dropping the tap frees the slot.
            let _ = task::spawn_blocking(move || handle.join()).await;
        }
        // Bound the writer join: a flow-control-stalled `write_frame`
        // cannot observe `stop()`, so give a healthy in-flight write a short grace
        // then ABORT — dropping the task drops its `SendStream`, resetting the QUIC
        // stream and unblocking the stalled `write_all`. An unbounded await here
        // would wedge `undemand` (freezing the whole control loop) and `shutdown`
        // (leaking every remaining topic's drain thread + SHM slot).
        match tokio::time::timeout(WRITER_STOP_GRACE, &mut self.writer).await {
            Ok(_) => {} // healthy writer finished
            Err(_) => {
                self.writer.abort();
                let _ = self.writer.await; // reap the aborted (Cancelled) task
            }
        }
    }
}

/// The per-connection demand-driven tap set. Owns one `DemandedTopic` per
/// currently-demanded topic and a cache of the last-seen wire `schema_hash` per
/// topic (fed to the `catalog` reply). One `TapManager` per wire connection.
#[derive(Default)]
pub struct TapManager {
    demanded: HashMap<String, DemandedTopic>,
    /// Last-seen wire `schema_hash` per topic this connection has tapped — the
    /// zero-cost source for the `catalog` reply (a live demand already reads it).
    schema_hashes: HashMap<String, u64>,
}

impl TapManager {
    /// A fresh, empty tap set.
    pub fn new() -> Self {
        Self::default()
    }

    /// Every `(topic, schema_hash)` this connection has recorded (from a live
    /// demand OR a catalog peek). The `catalog` handler feeds this into its known
    /// set so a hash observed ONCE is never re-peeked (a cache that
    /// read only DEMANDED topics would re-peek every peeked hash on every
    /// catalog).
    pub fn all_cached_schema_hashes(&self) -> std::collections::BTreeMap<String, u64> {
        self.schema_hashes
            .iter()
            .map(|(t, h)| (t.clone(), *h))
            .collect()
    }

    /// Attach a demand-driven tap for `topic` and open its robot→desk uni data
    /// stream. On success frames flow VERBATIM until [`Self::undemand`] (or the
    /// connection drops). A re-demand of an already-demanded topic is idempotent
    /// (no second tap, no second stream).
    ///
    /// Errors (topic does not exist / no free introspection slot / stream open
    /// failure) are returned so the caller surfaces them over the control stream
    /// — NEVER silent.
    pub async fn demand(
        &mut self,
        manager: &Arc<TransportManager>,
        connection: &Connection,
        topic: &str,
    ) -> Result<(), WireError> {
        if let Some(existing) = self.demanded.get(topic) {
            if existing.is_alive() {
                return Ok(()); // idempotent: already tapped + streaming
            }
            // The topic's forward stream DIED (writer error). Reap the
            // dead handle (its drain already exited + released the SHM slot when
            // the writer set stop) and fall through to RE-ATTACH a fresh tap +
            // stream, instead of idempotently returning DemandAccepted with no
            // live stream (a silent dead-and-unrecoverable state).
            if let Some(dead) = self.demanded.remove(topic) {
                tracing::info!(
                    topic = %topic,
                    "cerulion_remoted wire: re-demand of a died stream — reaping + re-attaching"
                );
                dead.stop().await;
            }
        }

        // Attach the listener-less tap (opens ONLY the data service; NEVER
        // creates it). A missing topic / exhausted introspection slot surfaces
        // here with an actionable message naming the topic.
        let tap =
            manager
                .create_data_only_subscriber(topic)
                .map_err(|e| WireError::DemandFailed {
                    topic: topic.to_string(),
                    reason: e.to_string(),
                })?;

        // Open the per-topic uni data stream (robot → desk) and lead it with a
        // JSON preamble naming the topic, so the desk correlates stream ↔ topic.
        let mut send =
            open_uni_frame_stream(connection)
                .await
                .map_err(|e| WireError::DemandFailed {
                    topic: topic.to_string(),
                    reason: format!("open_uni failed: {e}"),
                })?;
        let preamble = serde_json::to_vec(&StreamPreamble {
            topic: topic.to_string(),
        })
        .map_err(|e| WireError::Encode(e.to_string()))?;
        write_frame(&mut send, &preamble)
            .await
            .map_err(|e| WireError::DemandFailed {
                topic: topic.to_string(),
                reason: format!("stream preamble write failed: {e}"),
            })?;

        // Spawn the two halves bridged by the bounded drop-to-live channel.
        let channel = Arc::new(ForwardChannel::new());
        let drain_channel = channel.clone();
        let drain_topic = topic.to_string();
        let drain = std::thread::Builder::new()
            .name(format!("remoted-drain-{topic}"))
            .spawn(move || run_drain(tap, drain_channel, drain_topic))
            .map_err(|e| WireError::DemandFailed {
                topic: topic.to_string(),
                reason: format!("could not spawn drain thread: {e}"),
            })?;
        let writer_channel = channel.clone();
        let writer_topic = topic.to_string();
        let writer = task::spawn(run_writer(send, writer_channel, writer_topic));

        self.demanded.insert(
            topic.to_string(),
            DemandedTopic {
                channel,
                drain: Some(drain),
                writer,
            },
        );
        Ok(())
    }

    /// Drop the tap for `topic`, releasing its SHM subscriber slot. Returns
    /// `true` if the topic was demanded (torn down here), `false` if it was not
    /// demanded. Awaits the teardown so the slot is RELEASED before returning.
    pub async fn undemand(&mut self, topic: &str) -> bool {
        match self.demanded.remove(topic) {
            Some(demanded) => {
                demanded.stop().await;
                true
            }
            None => false,
        }
    }

    /// The `status` snapshot: one row per demanded topic (sorted), each carrying
    /// the WAN counters + the free wire-timestamp Hz + the live/dead stream state
    /// (a topic whose writer died is surfaced `stream_alive: false`,
    /// never misreported as healthy).
    pub fn status(&self) -> Vec<TopicStatus> {
        let mut rows: Vec<TopicStatus> = self
            .demanded
            .iter()
            .map(|(topic, d)| {
                let c = d.channel.counters();
                TopicStatus {
                    topic: topic.clone(),
                    wan_forwarded: c.forwarded(),
                    wan_dropped: c.dropped(),
                    frames_seen: c.frames_seen(),
                    hz: c.hz(),
                    stream_alive: d.is_alive(),
                }
            })
            .collect();
        rows.sort_by(|a, b| a.topic.cmp(&b.topic));
        rows
    }

    /// Tear down EVERY demanded topic (connection close). After this the tap set
    /// is empty and all SHM slots are released.
    pub async fn shutdown(&mut self) {
        let topics: Vec<String> = self.demanded.keys().cloned().collect();
        for topic in topics {
            if let Some(demanded) = self.demanded.remove(&topic) {
                demanded.stop().await;
            }
        }
    }

    /// Cache a freshly-observed wire `schema_hash` for `topic`. The `catalog`
    /// handler records what its PEEK found so a later catalog on the same
    /// connection is a zero-cost cache hit (the live drain thread has no
    /// `&mut self`, so it does not feed this cache — the peek is the source).
    pub fn record_schema_hash(&mut self, topic: &str, hash: u64) {
        self.schema_hashes.insert(topic.to_string(), hash);
    }
}

/// The drain thread body: OWN the tap and poll SHM forever, pushing each frame's
/// VERBATIM bytes into the bounded channel, until teardown is signalled. Dropping
/// the tap on return releases the SHM subscriber slot.
fn run_drain(mut tap: DataOnlySubscriber, channel: Arc<ForwardChannel>, topic: String) {
    // Reused batch buffer — bounded by the tap's per-service borrow budget so a
    // `drain_owned` never trips `ExceedsMaxBorrows`.
    let budget = tap.max_borrowed_samples().max(1);
    let mut batch: Vec<OwnedInboundSample> = Vec::with_capacity(budget);
    // Once-per-regime latch for the drain-error log (mirrors the WAN
    // flood-latch discipline): a sustained recoverable drain-error regime must
    // NOT log every ~1ms iteration. First error of a regime logs; a successful
    // drain (the error cleared) re-arms so the next regime logs again.
    let mut error_regime_open = false;
    while !channel.is_stopped() {
        batch.clear();
        match tap.drain_owned(budget, &mut batch) {
            Ok(0) => {
                error_regime_open = false; // a clean (empty) receive ends any regime
                std::thread::sleep(DRAIN_IDLE_SLEEP);
            }
            Ok(_n) => {
                error_regime_open = false; // a successful drain ends any regime
                for sample in batch.drain(..) {
                    // Read the wire timestamp (free Hz), then COPY the full frame
                    // out of SHM and DROP the sample — releasing the borrow before
                    // the frame ever waits on QUIC (taps stay observation-only).
                    let wire_ts = sample.wire_header().map(|h| h.timestamp_ns).unwrap_or(0);
                    let frame = sample.payload().to_vec();
                    drop(sample);
                    let outcome = channel.push(frame, wire_ts);
                    log_push_outcome(&topic, outcome, channel.counters().dropped());
                }
            }
            Err(e) => {
                // A drain error is recoverable (the tap may reconnect). Log ONCE
                // per regime at debug (not every ~1ms iteration — the pre-f10
                // flood) + back off, never busy-spin.
                if !error_regime_open {
                    error_regime_open = true;
                    tracing::debug!(topic = %topic, error = %e, "cerulion_remoted: tap drain error (regime open)");
                }
                std::thread::sleep(DRAIN_IDLE_SLEEP);
            }
        }
    }
    tracing::debug!(topic = %topic, "cerulion_remoted: tap drain thread stopped (slot released)");
}

/// The writer task body: pop queued frames and `write_frame` them on the topic's
/// uni stream. It ALONE blocks on QUIC flow control — a stall fills the channel,
/// which drop-to-lives. Each `write_frame` runs to COMPLETION (never inside a
/// `select!`) so the stream is never left mid-frame (framing cancel-safety).
async fn run_writer(
    mut send: cerulion_link::SendStream,
    channel: Arc<ForwardChannel>,
    topic: String,
) {
    let mut pending: Vec<Vec<u8>> = Vec::new();
    loop {
        channel.wait_writable(WRITER_BACKSTOP).await;
        pending.clear();
        channel.take_into(&mut pending);
        for frame in pending.drain(..) {
            if let Err(e) = write_frame(&mut send, &frame).await {
                // The uni stream died while the CONTROL connection may still be
                // live (a QUIC stream reset / desk STOP_SENDING). Signal teardown
                // so the drain thread exits + RELEASES its SHM slot
                // promptly instead of spinning forever with the slot pinned, and
                // so a re-demand observes the dead stream + re-attaches (rather
                // than idempotently short-circuiting). A demanded stream dying is
                // operator-visible (Principle #12/#3) → warn!, not debug!.
                channel.stop();
                tracing::warn!(
                    topic = %topic,
                    error = %e,
                    "cerulion_remoted wire: demanded stream's uni write failed — tearing the tap \
                     down (SHM slot released); a re-demand will re-attach"
                );
                return; // stream broken; drop-to-live means nothing to recover
            }
            channel.counters().record_forwarded();
        }
        if channel.is_stopped() && channel.is_empty() {
            let _ = send.finish();
            return;
        }
    }
}

/// Map one [`PushOutcome`] onto the WAN-drop tracing surface (extracted from the
/// drain loop so the once-per-regime `warn!` emission is deterministically
/// pinnable without a real QUIC stall). The FIRST drop of a regime is a loud
/// `warn!`; sustained drops downgrade to `debug!`; a room push that ends a regime
/// is a recovery `info!`; a healthy room push is silent.
fn log_push_outcome(topic: &str, outcome: PushOutcome, wan_dropped: u64) {
    match outcome {
        PushOutcome::DroppedOldest {
            level: WanDropLevel::Warn,
        } => tracing::warn!(
            topic = %topic,
            wan_dropped,
            "cerulion_remoted: WAN slower than topic — dropping the OLDEST frame to stay live \
             (bounded queue full); freshest wins. Repeats downgrade to debug until the link recovers."
        ),
        PushOutcome::DroppedOldest {
            level: WanDropLevel::Debug,
        } => tracing::debug!(
            topic = %topic,
            wan_dropped,
            "cerulion_remoted: sustained WAN drop (regime open)"
        ),
        PushOutcome::Queued { recovered: true } => tracing::info!(
            topic = %topic,
            wan_dropped,
            "cerulion_remoted: WAN forward recovered — queue draining again"
        ),
        PushOutcome::Queued { recovered: false } => {}
    }
}

/// The per-topic `catalog` schema-hash PEEK budget — the caller passes
/// `min(now + CATALOG_PEEK_BUDGET, total_catalog_deadline)` so a single silent
/// topic can never overrun the whole-catalog cap.
pub fn catalog_peek_budget() -> Duration {
    CATALOG_PEEK_BUDGET
}

/// Best-effort PEEK of a live topic's wire `schema_hash` for the `catalog` reply:
/// attach a fresh listener-less tap, poll for ONE frame until `deadline`, read
/// its header hash, then drop the tap. Returns `None` if the topic is silent
/// through the budget (a plain "no hash yet", never fabricated). `deadline` is
/// the caller's hard stop (the min of the per-topic budget and the whole-catalog
/// cap), so N silent topics can never freeze the control stream for
/// N×750ms. Blocking (poll + sleep) — the caller runs it under `spawn_blocking`.
pub fn peek_schema_hash(manager: &TransportManager, topic: &str, deadline: Instant) -> Option<u64> {
    let mut tap = manager.create_data_only_subscriber(topic).ok()?;
    let mut batch: Vec<OwnedInboundSample> = Vec::with_capacity(1);
    while Instant::now() < deadline {
        batch.clear();
        match tap.drain_owned(1, &mut batch) {
            Ok(0) => std::thread::sleep(DRAIN_IDLE_SLEEP),
            Ok(_) => {
                if let Some(hash) = batch
                    .first()
                    .and_then(|s| s.wire_header())
                    .map(|h| h.schema_hash)
                {
                    return Some(hash);
                }
                std::thread::sleep(DRAIN_IDLE_SLEEP);
            }
            Err(_) => return None,
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::forward::WanDropLevel;
    use tracing_test::traced_test;

    /// The once-per-regime WAN-drop warn EMISSION (the flood-latch pin at the
    /// tracing surface). A sustained-drop regime (Warn then N Debug) followed by a
    /// recovery + a new regime must emit EXACTLY 2 `warn!` lines (one per regime),
    /// the sustained drops as `debug!`, and 1 recovery `info!`. Deterministic — no
    /// real QUIC stall (that path's flow window makes the emission count flaky);
    /// the drop-to-live SEMANTICS are pinned by `forward.rs`'s oracle vectors.
    #[traced_test]
    #[test]
    fn wan_drop_warn_is_once_per_regime_at_the_tracing_surface() {
        let topic = "/wan/regime";
        // Regime 1: Warn, then 4 sustained Debug.
        log_push_outcome(
            topic,
            PushOutcome::DroppedOldest {
                level: WanDropLevel::Warn,
            },
            1,
        );
        for n in 2..=5 {
            log_push_outcome(
                topic,
                PushOutcome::DroppedOldest {
                    level: WanDropLevel::Debug,
                },
                n,
            );
        }
        // Recovery ends regime 1.
        log_push_outcome(topic, PushOutcome::Queued { recovered: true }, 5);
        // A healthy room push is silent.
        log_push_outcome(topic, PushOutcome::Queued { recovered: false }, 5);
        // Regime 2: a fresh Warn.
        log_push_outcome(
            topic,
            PushOutcome::DroppedOldest {
                level: WanDropLevel::Warn,
            },
            6,
        );

        // EXACTLY 2 warn lines (one per regime) — the WARN-unique substring.
        logs_assert(|lines: &[&str]| {
            let warns = lines
                .iter()
                .filter(|l| l.contains("WARN") && l.contains("dropping the OLDEST frame"))
                .count();
            if warns == 2 {
                Ok(())
            } else {
                Err(format!("expected exactly 2 WAN-drop warns, got {warns}"))
            }
        });
        // The recovery info fired once.
        logs_assert(|lines: &[&str]| {
            let recos = lines
                .iter()
                .filter(|l| l.contains("INFO") && l.contains("WAN forward recovered"))
                .count();
            if recos == 1 {
                Ok(())
            } else {
                Err(format!("expected exactly 1 recovery info, got {recos}"))
            }
        });
    }
}

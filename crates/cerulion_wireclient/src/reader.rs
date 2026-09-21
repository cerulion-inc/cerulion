// SPDX-License-Identifier: AGPL-3.0-only
//! The desk-side per-topic re-inject reader.
//!
//! One dedicated task per demanded topic OWNS its uni [`RecvStream`] and drives
//! [`read_frame`] to COMPLETION in a loop (never inside a `select!` — the framing
//! is not cancel-safe). Each frame is VALIDATED (`total_size` + `schema_hash`,
//! the ingress rules) and re-injected via the zenoh-free [`IngressInjector`] on the desk's
//! `network:None` [`TransportManager`]. No unbounded buffering: each reader
//! re-injects inline into the bounded SHM queue.
//!
//! # The shared client substrate
//!
//! Extracted from `cerulion_connectd::worker` into this neutral crate so
//! `cerulion_netd` can reuse the wire vocabulary + dial-config parsers WITHOUT a
//! `netd → connectd` cyclic package edge. This `run_topic_reader` loop is the CONNECTD
//! session driver's reader (`cerulion connect`'s worker calls it via a re-export); the
//! session driver (`cerulion_connectd::worker::run_connect` — the dial / demand /
//! catalog phase) stays in `cerulion_connectd`.
//!
//! `cerulion_netd`'s iroh WAN plane does NOT call `run_topic_reader` — it drives its
//! OWN supervised re-inject loop (`iroh_plane::run_reinject_reader`, with the
//! reader-death teardown netd needs). The element the LAN and WAN planes genuinely
//! SHARE is the underlying [`IngressInjector::reinject_raw`] PRIMITIVE (in
//! `cerulion_core`), not this loop. The two reader loops are not unified onto
//! `run_topic_reader`.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use cerulion_core::transport::network::{IngressInjector, ReinjectOutcome};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::TransportManager;
use cerulion_link::{read_frame, RecvStream, DEFAULT_MAX_FRAME_LEN};

/// The desk re-injection publisher's SHM slot cap. Sized to the wire read cap
/// ([`DEFAULT_MAX_FRAME_LEN`], 16 MiB) — the true upper bound on a receivable
/// frame (a larger frame is refused by [`read_frame`] before it ever reaches the
/// injector), so no re-injectable frame ever overflows the slot. iceoryx2 Static
/// pools are lazy/demand-paged, so an oversized cap costs nothing until used.
const INGRESS_MAX_SLICE_LEN: MaxSliceLen = MaxSliceLen::const_new(DEFAULT_MAX_FRAME_LEN as u32);

/// Per-topic re-injection counters (Principle #3). Shared (behind `Arc`) between a
/// reader task and the worker that snapshots them into the summary.
#[derive(Debug, Default)]
pub struct TopicReinjectCounters {
    /// Frames validated + re-injected into desk-local SHM.
    reinjected: AtomicU64,
    /// Frames REFUSED by ingress validation (schema/size/decode mismatch).
    rejected: AtomicU64,
    /// Frames validated but the local `publish_raw` failed (pool exhaustion / a
    /// poisoned injector). Investigate the desk.
    reinject_failed: AtomicU64,
}

impl TopicReinjectCounters {
    /// Frames validated + re-injected into desk-local SHM.
    pub fn reinjected(&self) -> u64 {
        self.reinjected.load(Ordering::Relaxed)
    }

    /// Frames refused by ingress validation.
    pub fn rejected(&self) -> u64 {
        self.rejected.load(Ordering::Relaxed)
    }

    /// Frames validated but not publishable locally.
    pub fn reinject_failed(&self) -> u64 {
        self.reinject_failed.load(Ordering::Relaxed)
    }

    /// Snapshot into a plain-data row.
    pub fn snapshot(&self, topic: &str) -> TopicReinjectStats {
        TopicReinjectStats {
            topic: topic.to_string(),
            reinjected: self.reinjected.load(Ordering::Relaxed),
            rejected: self.rejected.load(Ordering::Relaxed),
            reinject_failed: self.reinject_failed.load(Ordering::Relaxed),
        }
    }
}

/// A per-topic re-injection tally (the snapshot form for the connect summary).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TopicReinjectStats {
    /// The topic.
    pub topic: String,
    /// Frames re-injected into desk-local SHM.
    pub reinjected: u64,
    /// Frames refused by ingress validation.
    pub rejected: u64,
    /// Frames validated but not publishable locally.
    pub reinject_failed: u64,
}

/// Register this desk mirror's PROVENANCE so `cerulion topic
/// list` folds `topic` into the REMOTE section attributed to `robot` (the
/// "one data source = one topic" decision) instead of surfacing the
/// re-injected SHM service as a phantom LOCAL topic. Best-effort — a failure logs
/// a warn but never fails the re-inject (frames still flow; the topic just would
/// not fold). Called ONCE per topic, right after the desk-local ingress publisher
/// is created.
fn register_mirror_provenance(manager: &TransportManager, topic: &str, robot: &str) {
    if let Err(e) = manager.register_mirror_provenance(topic, robot) {
        tracing::warn!(
            topic = %topic, robot = %robot, error = %e,
            "cerulion connect: could not register mirror provenance — the mirror still streams, \
             but `topic list` will show it as LOCAL rather than REMOTE from this robot"
        );
    }
}

/// The dedicated per-topic reader task: OWN the uni [`RecvStream`], drive
/// [`read_frame`] to completion, and re-inject each validated frame into
/// desk-local SHM. Returns when the stream ends (undemand / connection drop) or an
/// abort drops it. Exposed `pub` for the deterministic reader/rejection e2e (a
/// hand-fed uni stream of good + schema-mismatched frames pins the validate +
/// re-inject + count contract through this exact code path).
pub async fn run_topic_reader(
    mut recv: RecvStream,
    topic: String,
    robot: String,
    manager: Arc<TransportManager>,
    counters: Arc<TopicReinjectCounters>,
    expected_hash: Option<u64>,
) {
    // Create the injector EAGERLY when the catalog declared the topic's hash (so
    // the desk SHM service exists before the first frame — a subscriber can attach
    // immediately); else derive it lazily from the first frame's header below.
    let mut injector: Option<IngressInjector> = match expected_hash
        .map(|h| manager.create_ingress_injector(&topic, h, INGRESS_MAX_SLICE_LEN))
    {
        Some(Ok(inj)) => {
            register_mirror_provenance(&manager, &topic, &robot);
            Some(inj)
        }
        Some(Err(e)) => {
            tracing::error!(
                topic = %topic, error = %e,
                "cerulion connect: cannot create desk-local ingress publisher — dropping this \
                 topic's stream (is it already produced locally?)"
            );
            return;
        }
        None => None,
    };

    loop {
        let frame = match read_frame(&mut recv, DEFAULT_MAX_FRAME_LEN).await {
            Ok(f) => f,
            Err(e) => {
                tracing::debug!(topic = %topic, error = %e, "cerulion connect: uni stream ended");
                break;
            }
        };

        // Lazily create the injector from the first frame's schema_hash (only when
        // the catalog gave no hash).
        if injector.is_none() {
            let header = match WireHeader::read_from_buf(&frame) {
                Some(h) => h,
                None => {
                    counters.rejected.fetch_add(1, Ordering::Relaxed);
                    tracing::debug!(topic = %topic, "cerulion connect: undecodable first frame (too short for a header); skipping");
                    continue;
                }
            };
            match manager.create_ingress_injector(&topic, header.schema_hash, INGRESS_MAX_SLICE_LEN)
            {
                Ok(inj) => {
                    tracing::info!(
                        topic = %topic, schema_hash = header.schema_hash,
                        "cerulion connect: desk-local ingress ready (remote = local)"
                    );
                    register_mirror_provenance(&manager, &topic, &robot);
                    injector = Some(inj);
                }
                Err(e) => {
                    tracing::error!(
                        topic = %topic, error = %e,
                        "cerulion connect: cannot create desk-local ingress publisher — dropping \
                         this topic's stream"
                    );
                    break;
                }
            }
        }

        let inj = match injector.as_ref() {
            Some(i) => i,
            None => continue, // header decode failed on the first frame; retry next
        };
        match inj.reinject_raw(&frame) {
            ReinjectOutcome::Injected { .. } => {
                counters.reinjected.fetch_add(1, Ordering::Relaxed);
            }
            ReinjectOutcome::Rejected(_) => {
                counters.rejected.fetch_add(1, Ordering::Relaxed);
            }
            ReinjectOutcome::ReinjectFailed => {
                counters.reinject_failed.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}

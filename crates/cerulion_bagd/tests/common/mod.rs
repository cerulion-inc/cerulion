// SPDX-License-Identifier: AGPL-3.0-only
//! Shared test helpers for the `cerulion_bagd` integration tests.
//!
//! Every test builds its own ISOLATED iceoryx2 transport
//! ([`cerulion_core::testing::iceoryx_test_config`] → a unique per-instance SHM
//! prefix via `TransportManager::init_for_test`), so tests are parallel-safe on
//! distinct topic namespaces (the `#[serial]` on the iceoryx2-touching tests is
//! belt-and-suspenders against the SHM singleton, not a correctness dependency
//! of these per-instance managers).

#![allow(dead_code)] // helpers are shared; not every test uses every one.

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime};

use cerulion_bagd::{BagdError, BagdSummary};
use cerulion_core::testing::iceoryx_test_config;
use cerulion_core::transport::publisher::CerulionPublisher;
use cerulion_core::transport::{
    PublisherProvisioning, MULTI_PUBLISHER_LOOSE_MAX, MULTI_SUBSCRIBER_LOOSE_MAX,
};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::{TransportConfig, TransportManager, VirtualClock};

static COUNTER: AtomicU64 = AtomicU64::new(0);

fn nanos() -> u128 {
    SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos()
}

/// A process-unique, canonical (leading-`/`) topic name.
pub fn unique_topic(base: &str) -> String {
    let id = COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("/bagd_test/{base}/{}/{}", nanos(), id)
}

/// A process-unique temp bag path (`<tmp>/bagd_<base>_<nanos>_<id>.mcap`).
pub fn unique_out(base: &str) -> PathBuf {
    let id = COUNTER.fetch_add(1, Ordering::Relaxed);
    std::env::temp_dir().join(format!("bagd_{base}_{}_{}.mcap", nanos(), id))
}

/// A process-unique trace-ring tag.
pub fn unique_ring_tag(base: &str) -> String {
    let id = COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("bagd_{base}_{}_{}", nanos(), id)
}

/// Build an ISOLATED transport manager, returning the manager AND the serialized
/// iceoryx2 `Config` JSON (a subprocess child joins the SAME namespace by
/// deserializing it via the hidden `--test-iox2-config` flag).
pub fn make_manager_with_json(buffer: usize) -> (Arc<TransportManager>, String) {
    let ix = iceoryx_test_config();
    let json = serde_json::to_string(&ix).expect("serialize iceoryx2 config");
    let config = TransportConfig {
        node_name: "cerulion_bagd_test".into(),
        clock: Arc::new(VirtualClock::new()),
        subscriber_buffer_size: buffer,
        network: None,
    };
    let mgr = TransportManager::init_for_test(config, ix).expect("init_for_test");
    (mgr, json)
}

/// An isolated transport manager (discards the serialized JSON).
pub fn make_manager(buffer: usize) -> Arc<TransportManager> {
    make_manager_with_json(buffer).0
}

/// Create a publisher provisioning the service with an explicit borrow budget +
/// queue depth (so an open-only tap on this topic inherits them). Created FIRST
/// (before the tap) so IT creates the service.
pub fn publisher_with_provisioning(
    mgr: &TransportManager,
    topic: &str,
    borrow: usize,
    buffer: usize,
    max_slice_len: u32,
) -> CerulionPublisher {
    let mut tc = mgr.default_topic_config();
    tc.subscriber_max_borrowed_samples = Some(borrow);
    tc.subscriber_max_buffer_size = buffer;
    mgr.create_publisher_with_topic_config(topic, MaxSliceLen::const_new(max_slice_len), 0, tc)
        .expect("create_publisher_with_topic_config")
}

/// Create a publisher on a topic provisioned MULTI-PUBLISHER, so SEVERAL
/// `CerulionPublisher`s can coexist on ONE iceoryx2 service (the
/// `multi_publisher_topics:` opt-in shape — `/tf` and friends).
///
/// # Why this cannot reuse [`publisher_with_provisioning`]
///
/// [`TransportManager::create_publisher_with_topic_config`] runs an
/// active-publisher PRE-CHECK keyed on `TopicServiceConfig::publisher_provisioning
/// == SingleWriter`, and `default_topic_config()` provisions
/// `PublisherProvisioning::External` with `max_publishers: None` (iceoryx2's
/// create-default of 2). The declared INTENT is what the enforcement site
/// reads, so a second writer needs the `Multi` arm, not a hand-mutated port
/// count.
///
/// The three port/ceiling caps are the SHARED loose constants rather than
/// per-caller numbers, for the reason the `Multi` variant's own doc gives: a
/// `Some(n)` requirement is at-least verified on OPEN, so both publishers (and
/// the recorder's tap) must ask for the SAME values or the second open is
/// refused. `buffer` is the one knob a caller picks — the tap inherits it as
/// its receive-queue depth, i.e. the loss boundary.
///
/// Created FIRST (before the tap) so IT creates the service; the SECOND writer
/// then opens the same service by equality.
pub fn multi_publisher_with_provisioning(
    mgr: &TransportManager,
    topic: &str,
    buffer: usize,
    max_slice_len: u32,
) -> CerulionPublisher {
    let mut tc = mgr.default_topic_config();
    tc.subscriber_max_buffer_size = buffer;
    tc.max_subscribers = Some(MULTI_SUBSCRIBER_LOOSE_MAX);
    tc.max_publishers = Some(MULTI_PUBLISHER_LOOSE_MAX);
    tc.publisher_provisioning = PublisherProvisioning::Multi;
    mgr.create_publisher_with_topic_config(topic, MaxSliceLen::const_new(max_slice_len), 0, tc)
        .expect("create_publisher_with_topic_config (multi-publisher)")
}

/// Create a publisher whose service provisions exactly `slots` subscriber
/// slots. Saturating them (attach `slots` subscribers) makes any FURTHER
/// subscriber-port creation on the topic fail — the only shape left
/// that produces a genuinely un-attachable live topic for the recorder
/// to report as `attach_failed`.
pub fn publisher_with_subscriber_slots(
    mgr: &TransportManager,
    topic: &str,
    slots: usize,
    max_slice_len: u32,
) -> CerulionPublisher {
    let mut tc = mgr.default_topic_config();
    tc.max_subscribers = Some(slots);
    mgr.create_publisher_with_topic_config(topic, MaxSliceLen::const_new(max_slice_len), 0, tc)
        .expect("create_publisher_with_topic_config")
}

/// A default-provisioned publisher (default borrow budget = 2).
pub fn publisher(mgr: &TransportManager, topic: &str, max_slice_len: u32) -> CerulionPublisher {
    mgr.create_publisher(topic, MaxSliceLen::const_new(max_slice_len), 0)
        .expect("create_publisher")
}

/// Hand-build a wire frame: a valid 32-byte [`WireHeader`] (`total_size` = full
/// frame length) followed by `body`.
pub fn build_frame(schema_hash: u64, sequence: u32, timestamp_ns: u64, body: &[u8]) -> Vec<u8> {
    let total = WireHeader::SIZE + body.len();
    let mut header = WireHeader::new(schema_hash, sequence, timestamp_ns);
    header.total_size = total as u32;
    let mut frame = vec![0u8; total];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(body);
    frame
}

/// How often [`wait_for_file`] polls. Exposed because a test that MEASURES from
/// a `wait_for_file` return has an observation origin up to this late, and an
/// exact `>=` assertion against that origin must account for it.
pub const FILE_POLL_INTERVAL: Duration = Duration::from_millis(5);

/// Block until `path` exists or `timeout` elapses; returns whether it appeared.
pub fn wait_for_file(path: &std::path::Path, timeout: Duration) -> bool {
    await_condition(timeout, || path.exists())
}

/// Block until `cond` holds or `timeout` elapses; returns whether it held.
///
/// The CONDITION form of a settle, and the one to reach for by default: a
/// `sleep(n * cadence)` is a bet on how much work a loaded runner completes in a
/// window, which load can invert, while a bounded wait on the thing you actually
/// need can only be DELAYED by load. The generous ceiling is a liveness backstop,
/// never the property under test.
pub fn await_condition(timeout: Duration, mut cond: impl FnMut() -> bool) -> bool {
    let start = Instant::now();
    while start.elapsed() < timeout {
        if cond() {
            return true;
        }
        std::thread::sleep(FILE_POLL_INTERVAL);
    }
    cond()
}

/// iceoryx2 delivery is synchronous, but a fresh cross-connection needs a beat
/// to surface on a CI VM — mirrors `drain_owned_test::settle`.
pub fn settle() {
    std::thread::sleep(Duration::from_millis(40));
}

/// Generous liveness ceiling for the taps-ready handshake.
///
/// Load can only DELAY the sentinel; nothing makes it appear early, so this is a
/// backstop that turns a wedged recorder into an attributable failure — never a
/// wall stated in units of anything under test.
///
/// MEASURED at ~milliseconds on a healthy desk, so this carries roughly five
/// orders of magnitude of margin, and it is sized against a measurement rather
/// than a guess: at 30 s it was itself trippable under 2x CPU oversubscription
/// with the whole test process demoted to macOS background QoS (`taskpolicy -b`
/// against 32 spinners), which is well past anything CI does but is exactly the
/// direction a liveness backstop must not be tight in. A ceiling this loose
/// costs a healthy run nothing — the wait is a condition, not a sleep — and
/// costs a genuinely wedged one two bounded minutes before it fails LOUDLY.
pub const RECORDER_READY_DEADLINE: Duration = Duration::from_secs(120);

/// Generous liveness ceiling for a signalled recorder's own teardown. Sized on
/// the same reasoning as [`RECORDER_READY_DEADLINE`].
pub const RECORDER_JOIN_DEADLINE: Duration = Duration::from_secs(120);

/// A process-unique path for a recorder's ready-file sentinel.
pub fn unique_ready_file(base: &str) -> PathBuf {
    let id = COUNTER.fetch_add(1, Ordering::Relaxed);
    std::env::temp_dir().join(format!("bagd_ready_{base}_{}_{}", nanos(), id))
}

/// Block until the recorder has written its ready-file, or fail ATTRIBUTABLY.
///
/// THE rendezvous a test owes any recorder it spawns, and the reason is
/// structural rather than a matter of timing margins. `run_bagd` writes this
/// sentinel immediately after `Recorder::setup` returns — which is where the
/// data-only TAPS attach and where the flashback RESPONDER's control subscriber
/// is created — so before it appears:
///
///   * a frame published on a tapped topic lands in no queue at all (a
///     `DataOnlySubscriber` requests no late-joiner history, so it is not late,
///     it is GONE), and
///   * a capture request published on `/__cerulion/flashback` reaches no
///     subscriber (iceoryx2 pub/sub keeps none for a port that attaches later),
///     so the verdict a test then waits for can never come.
///
/// Both are silent, and both grow more likely exactly when a runner is loaded —
/// which is the `await_discovered_tap` lesson applied to the handshake
/// bagd already ships for this purpose.
pub fn await_bagd_ready(ready: &std::path::Path, what: &str) {
    assert!(
        await_condition(RECORDER_READY_DEADLINE, || ready.exists()),
        "{what}: bagd never wrote its ready-file ({}) within {RECORDER_READY_DEADLINE:?}. \
         `Recorder::setup` had not finished, so NO tap is attached and NO flashback responder is \
         listening — every frame and every capture request published from here would be \
         delivered to nothing. This is a FAILURE of the recorder or of this harness, not of the \
         assertions below",
        ready.display()
    );
}

/// Join a recorder thread under a BOUNDED liveness ceiling.
///
/// A bare `join()` on a recorder that never notices its shutdown flag HANGS the
/// whole binary — the suite then burns its CI job timeout with no attributable
/// red, which is strictly worse than a failing arm (the `join_self_terminating`
/// precedent in `run_lifetime_e2e_test`).
pub fn join_bagd(
    handle: std::thread::JoinHandle<Result<BagdSummary, BagdError>>,
    shutdown: &Arc<AtomicBool>,
    what: &str,
) -> Result<BagdSummary, BagdError> {
    let deadline = Instant::now() + RECORDER_JOIN_DEADLINE;
    while !handle.is_finished() {
        assert!(
            Instant::now() < deadline,
            "{what}: the recorder was signalled (shutdown={}) and must have finalized within \
             {RECORDER_JOIN_DEADLINE:?}, but it is still running",
            shutdown.load(Ordering::Relaxed)
        );
        std::thread::sleep(FILE_POLL_INTERVAL);
    }
    handle.join().expect("the recorder thread must not panic")
}

/// Best-effort cleanup of a bag file + its rotation siblings.
pub fn cleanup(base: &std::path::Path) {
    let _ = std::fs::remove_file(base);
    if let (Some(stem), Some(parent)) = (base.file_stem(), base.parent()) {
        if let Ok(rd) = std::fs::read_dir(parent) {
            let prefix = stem.to_string_lossy().to_string();
            for e in rd.flatten() {
                let name = e.file_name().to_string_lossy().to_string();
                if name.starts_with(&prefix) {
                    let _ = std::fs::remove_file(e.path());
                }
            }
        }
    }
}

// ===========================================================================
// Structural-walk helpers
// ===========================================================================
//
// SHARED rather than copied: two integration binaries now walk this crate's own
// source for wiring no behavioural arm can reach, and a second copy of a
// comment stripper is how the two would drift.

/// Strip Rust comments so a walk cannot be satisfied — or defeated — by prose.
///
/// Block comments NEST in Rust, so the depth is tracked. String literals are
/// deliberately NOT modelled (the `code_only` precedent in this repo): the
/// bodies walked below contain no literal naming an enumeration verb, and the
/// anti-tautology arm below fails loudly if this stripper ever eats real code.
pub fn code_only(src: &str) -> String {
    let bytes = src.as_bytes();
    let mut out = String::with_capacity(src.len());
    let mut i = 0usize;
    let mut block_depth = 0usize;
    while i < bytes.len() {
        if block_depth > 0 {
            if bytes[i..].starts_with(b"/*") {
                block_depth += 1;
                i += 2;
            } else if bytes[i..].starts_with(b"*/") {
                block_depth -= 1;
                i += 2;
            } else {
                if bytes[i] == b'\n' {
                    out.push('\n');
                }
                i += 1;
            }
            continue;
        }
        if bytes[i..].starts_with(b"/*") {
            block_depth += 1;
            i += 2;
        } else if bytes[i..].starts_with(b"//") {
            while i < bytes.len() && bytes[i] != b'\n' {
                i += 1;
            }
        } else {
            out.push(bytes[i] as char);
            i += 1;
        }
    }
    assert_eq!(
        block_depth, 0,
        "unterminated block comment — the stripped view is truncated and every ABSENCE assertion \
         over it would be vacuous"
    );
    out
}

/// The brace-matched body of the first `fn <name>(` in `src`.
///
/// Function-SCOPED on purpose: a whole-file `contains` is true the moment ANY
/// function in an 11k-line file calls the verb, and would be blind to the one
/// that matters.
pub fn fn_body(src: &str, name: &str) -> String {
    // A GENERIC function is spelled `fn name<E>(`, so the bare `(` needle misses
    // it entirely and the helper panics as if the function had been renamed —
    // which is a confusing failure for a guard whose whole job is to notice a
    // rename. Both spellings are tried, nearest first.
    let start = src
        .find(&format!("fn {name}("))
        .or_else(|| src.find(&format!("fn {name}<")))
        .unwrap_or_else(|| panic!("`fn {name}` not found — did it move or get renamed?"));
    let open = src[start..]
        .find('{')
        .unwrap_or_else(|| panic!("`fn {name}` has no body brace"))
        + start;
    let bytes = src.as_bytes();
    let mut depth = 0usize;
    for (idx, b) in bytes.iter().enumerate().skip(open) {
        match b {
            b'{' => depth += 1,
            b'}' => {
                depth -= 1;
                if depth == 0 {
                    return src[open..=idx].to_string();
                }
            }
            _ => {}
        }
    }
    panic!("`fn {name}`'s body brace is never closed");
}

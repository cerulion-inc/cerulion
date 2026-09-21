// SPDX-License-Identifier: AGPL-3.0-only
//! Shared test helpers for the `cerulion bagd` subprocess tests (the bagd
//! fold moved the subprocess suite here — it spawns
//! the REAL `cerulion` binary with the `bagd` subcommand). A copy of
//! `cerulion_bagd/tests/common/mod.rs` (whose in-process e2e suite still uses
//! the original).
//!
//! Every test builds its own ISOLATED iceoryx2 transport
//! ([`cerulion_core::testing::iceoryx_test_config`] → a unique per-instance SHM
//! prefix via `TransportManager::init_for_test`), so tests are parallel-safe on
//! distinct topic namespaces (the `#[serial]` on the iceoryx2-touching tests is
//! belt-and-suspenders against the SHM singleton, not a correctness dependency
//! of these per-instance managers).

#![allow(dead_code)] // helpers are shared; not every test uses every one.

use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime};

use cerulion_core::testing::iceoryx_test_config;
use cerulion_core::transport::publisher::CerulionPublisher;
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

/// Block until `path` exists or `timeout` elapses; returns whether it appeared.
pub fn wait_for_file(path: &std::path::Path, timeout: Duration) -> bool {
    let start = Instant::now();
    while start.elapsed() < timeout {
        if path.exists() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    path.exists()
}

/// iceoryx2 delivery is synchronous, but a fresh cross-connection needs a beat
/// to surface on a CI VM — mirrors `drain_owned_test::settle`.
pub fn settle() {
    std::thread::sleep(Duration::from_millis(40));
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

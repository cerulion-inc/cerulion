// SPDX-License-Identifier: AGPL-3.0-only
//! The [`BagReader::completeness`] pre-loop scan is a full front-to-back
//! walk (chunk-CRC + torn-tail detection) that touches every page — one of the
//! passes that peaked replay RSS to the machine's RAM ceiling on a 200 GB-scale bag under
//! `MADV_SEQUENTIAL` alone. It now drives EXPLICIT batched advise-behind behind a
//! per-pass cursor.
//!
//! `mcap::MessageStream` exposes no byte cursor and copies each payload, so the
//! completeness watermark is a CONSERVATIVE frontier: the cumulative sum of
//! consumed message-payload lengths (a strict lower bound on the true read
//! position — framing/chunk overhead sits behind the payloads already passed).
//! These pins prove the scan really evicts, safely (every advised region ends at
//! or before that frontier), monotonically, and page-batched.

use std::path::PathBuf;

use cerulion_bag::{BagReader, BagWriter, BagWriterConfig, TopicSchema};

fn tmp() -> PathBuf {
    static C: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let n = C.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    std::env::temp_dir().join(format!(
        "cerulion_advise_completeness_{}_{}_{}.mcap",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos(),
        n
    ))
}

/// Write a finalized bag whose payloads sum to several hundred KiB, so a
/// page-scale eviction batch fires many times during the completeness scan.
fn write_multi_page_bag(path: &std::path::Path, msgs: usize, payload_len: usize) {
    let cfg = BagWriterConfig::default();
    let mut w = BagWriter::create(
        path,
        cfg,
        &[TopicSchema {
            topic: "/a".into(),
            schema_name: "std_msgs/UInt8".into(),
            schema_hash: 0x1,
            wire_fixed_size: payload_len as u32,
        }],
    )
    .unwrap();
    // Payloads must outlive the chunk scope (the writer borrows them).
    let payloads: Vec<Vec<u8>> = (0..msgs)
        .map(|i| vec![(i & 0xff) as u8; payload_len])
        .collect();
    w.write_chunk(|c| {
        for (i, p) in payloads.iter().enumerate() {
            c.write_message("/a", i as u32, 1000 + i as u64, 1000 + i as u64, &[&p[..]])?;
        }
        Ok(())
    })
    .unwrap();
    w.finalize().unwrap();
}

/// The runtime page size (all advised offsets are page-aligned).
fn page() -> usize {
    let ps = unsafe { libc::sysconf(libc::_SC_PAGESIZE) };
    if ps > 0 {
        ps as usize
    } else {
        4096
    }
}

#[test]
fn completeness_scan_advises_behind_its_own_payload_frontier() {
    let path = tmp();
    // ~4000 * 128 B = ~512 KiB of payload → dozens of page-scale batches.
    write_multi_page_bag(&path, 4000, 128);

    let reader = BagReader::open(&path).expect("open bag mapped");
    reader.enable_scoped_advise_probe();
    reader.set_advise_behind_batch_for_test(4096); // page-scale batch.

    let completeness = reader.completeness().expect("completeness scan runs");
    assert!(
        completeness.is_finalized(),
        "control: the intact bag is Finalized; got {completeness:?}"
    );

    let calls = reader.take_scoped_advise_probe();
    assert!(
        !calls.is_empty(),
        "the completeness scan must evict behind its payload frontier (mutation guard: \
         reverting the advise_evict_behind_scoped call in completeness() empties this)"
    );
    let p = page();
    let mut prev_end = 0usize;
    for (i, c) in calls.iter().enumerate() {
        // The conservative payload-sum frontier is a LOWER bound on the true
        // read position, so an evicted region ending at or before it is always
        // already-read — eviction never races the live scan.
        assert!(
            c.end <= c.watermark,
            "call {i}: advised end {} must not exceed the scan payload frontier {}",
            c.end,
            c.watermark
        );
        assert_eq!(c.start % p, 0, "call {i}: start {} page-aligned", c.start);
        assert_eq!(c.end % p, 0, "call {i}: end {} page-aligned", c.end);
        assert!(c.start < c.end, "call {i}: non-empty region");
        assert_eq!(
            c.start, prev_end,
            "call {i}: regions tile monotonically without gap or overlap"
        );
        prev_end = c.end;
    }
    // The scan really progressed far: the final payload frontier is many
    // batches in (not a frozen cursor emitting one token eviction).
    let last = calls.last().unwrap();
    assert!(
        last.watermark >= 8 * 4096,
        "the payload frontier must advance well past a single batch: {}",
        last.watermark
    );

    let _ = std::fs::remove_file(&path);
}

#[test]
fn completeness_scan_advise_is_a_noop_on_an_owned_buffer() {
    // The scoped driver no-ops on a non-mapped (owned) bag — the probe stays
    // empty even armed, so an in-memory bag pays nothing.
    let path = tmp();
    write_multi_page_bag(&path, 500, 64);
    let bytes = std::fs::read(&path).unwrap();
    let _ = std::fs::remove_file(&path);

    let reader = BagReader::from_bytes(bytes);
    reader.enable_scoped_advise_probe();
    reader.set_advise_behind_batch_for_test(4096);
    let completeness = reader
        .completeness()
        .expect("completeness runs on owned bytes");
    assert!(completeness.is_finalized());
    assert!(
        reader.take_scoped_advise_probe().is_empty(),
        "an owned (non-mapped) bag has no map to evict — zero advise calls"
    );
}

// SPDX-License-Identifier: AGPL-3.0-only
//! A cdylib node's throttle and once log state across the cdylib wall:
//! `cerulion_node_capture_state` writes it after the node's fields, and
//! `cerulion_node_restore_state` takes it from the bytes, over a real
//! `dlopen` of `test_node_macro_log_throttle_cdylib` through
//! `DylibNodeEntry`.
//!
//! The fixture's state is one field, `ticks: u32`, and its tick runs one
//! `warn_throttle!` and one `info_once!` site. Hand oracle, read off the
//! fixture source and the documented layout (`graph::log_sites`):
//!
//! - before any tick the capture is the field alone: `ticks` 0, 4 bytes;
//! - after one tick it is `ticks` 1, then tag `CLOG`, count 2, and two
//!   `(id, word)` pairs in ascending id order: the once word is 1, the
//!   throttle word is the graph time of that tick plus one. The runtime's
//!   virtual clock starts at 0 and one `step(10 ms)` advances it once to
//!   10 ms before the tick runs, so the throttle word is 10_000_001;
//! - a fresh instance restored from those bytes captures them back byte for
//!   byte: the section crossed the wall in both directions;
//! - that restored instance then runs on the restored words: its first step
//!   is `step(100 ms)`, so its first ticks read graph time 100 ms, not 10 ms
//!   (the 10 ms node catches up 10 ticks inside that one step, and each tick
//!   reads the step's one time). A reset throttle word would emit there and
//!   hold 100_000_001, which differs from the saved word. Ticks at 100 to
//!   250 ms are inside the 250 ms period of the saved 10 ms emit, so neither
//!   word moves; a tick at 260 ms emits the throttle site, whose word becomes
//!   260_000_001 (a reset word would wait until 350 ms), and the once word
//!   stays 1;
//! - an instance that ticked, restored from the field-only bytes, captures
//!   the field-only bytes: its own live log state is gone, the table is the
//!   checkpoint's.
//!
//! ```bash
//! cargo build -p test_node_macro_log_throttle_cdylib
//! cargo test -p cerulion_core --test log_sites_cdylib_state_test -- --test-threads=1
//! ```

use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::{DylibNodeEntry, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::state::VecSink;
use cerulion_core::state_restore::AnchorBlob;
use indexmap::IndexMap;
use serial_test::serial;

fn fixture() -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_macro_log_throttle_cdylib")
}

fn graph(tag: &str) -> GraphConfig {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .expect("clock")
        .as_nanos();
    GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "logcdy".to_string(),
        prefix: format!("logcdy_{tag}_{nanos}"),
        nodes: vec![NodeDef {
            fuse: None,
            params: Default::default(),
            ros2: None,
            id: "chatter".to_string(),
            node_type: "chatter".to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "cmd".to_string(),
                schema: "Vector3".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        }],
    }
}

/// A graph around the fixture, and the runtime's own entry for it.
fn built(tag: &str) -> (GraphRuntime, Arc<Mutex<Box<dyn NodeEntry>>>) {
    let entry = DylibNodeEntry::load(&fixture()).expect("the fixture cdylib loads");
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("chatter".to_string(), Box::new(entry));
    let runtime =
        GraphRuntime::build_for_test(graph(tag), factories, Arc::new(VirtualClock::new()), 16)
            .expect("graph builds");
    let node = runtime
        .node_entry_arc_for_test("chatter")
        .expect("chatter is in the node map");
    (runtime, node)
}

fn capture(node: &Arc<Mutex<Box<dyn NodeEntry>>>) -> Vec<u8> {
    let mut sink = VecSink::new();
    node.lock()
        .expect("node")
        .capture_state(&mut sink)
        .expect("a macro cdylib captures");
    sink.into_inner()
}

/// Restore `payload` through the runtime's own anchor path.
fn restore(runtime: &mut GraphRuntime, node: &Arc<Mutex<Box<dyn NodeEntry>>>, payload: &[u8]) {
    let shape = node
        .lock()
        .expect("node")
        .state_shape()
        .expect("a macro cdylib declares a shape");
    let mut sink = VecSink::new();
    AnchorBlob::write_header(shape, &mut sink).expect("framing fits a Vec");
    let mut blob = sink.into_inner();
    blob.extend_from_slice(payload);
    let mut anchors = std::collections::BTreeMap::new();
    anchors.insert("chatter".to_string(), blob);
    let report = runtime
        .restore_node_states(&anchors)
        .expect("the anchor applies");
    assert_eq!(report.restored, vec!["chatter".to_string()]);
}

/// The `(id, word)` pairs of a capture's log section, in byte order. The
/// section starts after the 4-byte field, the tag and the count.
fn log_table(capture: &[u8]) -> Vec<(u64, u64)> {
    assert_eq!(&capture[4..8], b"CLOG");
    let count = u32::from_le_bytes(capture[8..12].try_into().unwrap()) as usize;
    assert_eq!(capture.len(), 12 + count * 16, "{capture:?}");
    let word = |o: usize| u64::from_le_bytes(capture[o..o + 8].try_into().unwrap());
    (0..count)
        .map(|i| (word(12 + i * 16), word(20 + i * 16)))
        .collect()
}

/// The one id in `table` whose word is `word`.
fn site_with_word(table: &[(u64, u64)], word: u64) -> u64 {
    let hits: Vec<u64> = table
        .iter()
        .filter(|&&(_, w)| w == word)
        .map(|&(id, _)| id)
        .collect();
    assert_eq!(hits.len(), 1, "one site holds word {word}: {table:?}");
    hits[0]
}

/// The word `table` holds for `id`.
fn word_of(table: &[(u64, u64)], id: u64) -> u64 {
    table
        .iter()
        .find(|&&(i, _)| i == id)
        .map(|&(_, w)| w)
        .unwrap_or_else(|| panic!("site {id} is in {table:?}"))
}

/// The graph time of the first tick: one `step(10 ms)` from a virtual clock
/// at 0.
const TICK_NS: u64 = 10_000_000;

/// The graph time of the restored instance's first step: one `step(100 ms)`
/// from its own clock at 0. It is inside the 250 ms period of the saved emit
/// at [`TICK_NS`], and not equal to it, so a reset throttle word shows.
const FRESH_FIRST_TICK_NS: u64 = 100_000_000;

#[test]
#[serial]
fn a_cdylib_checkpoint_carries_its_log_state_and_a_restore_takes_it_from_the_bytes() {
    let (mut ticked_rt, ticked) = built("ticked");
    assert_eq!(
        capture(&ticked),
        0u32.to_le_bytes(),
        "no tick, no log section"
    );

    ticked_rt.step(Duration::from_nanos(TICK_NS));
    let after_one_tick = capture(&ticked);
    assert_eq!(
        after_one_tick.len(),
        4 + 4 + 4 + 2 * 16,
        "{after_one_tick:?}"
    );
    assert_eq!(after_one_tick[..4], 1u32.to_le_bytes(), "ticks");
    assert_eq!(&after_one_tick[4..8], b"CLOG");
    assert_eq!(after_one_tick[8..12], 2u32.to_le_bytes(), "two sites ran");
    let table = log_table(&after_one_tick);
    let ids: Vec<u64> = table.iter().map(|&(id, _)| id).collect();
    assert!(ids[0] < ids[1], "ascending ids");
    // Each word stays with its own id. The once site is the id whose word is
    // 1, the throttle site the id whose word is the tick's graph time plus one.
    let once_id = site_with_word(&table, 1);
    let throttle_id = site_with_word(&table, TICK_NS + 1);
    assert_ne!(once_id, throttle_id);

    // A fresh instance takes the section from the bytes, and gives it back.
    let (mut fresh_rt, fresh) = built("fresh");
    restore(&mut fresh_rt, &fresh, &after_one_tick);
    assert_eq!(
        capture(&fresh),
        after_one_tick,
        "the section crossed the wall"
    );

    // The restored words drive the restored instance. Its first step runs 10
    // catch-up ticks, and each reads the step's one graph time, 100 ms: a
    // time no saved word holds, so a reset throttle word would emit there
    // and hold 100_000_001. Then 15 ticks run at 110, 120, .., 250 ms. All
    // are less than 250 ms after the saved throttle emit at 10 ms: inside the
    // period, so the throttle word does not move, and the once word stays.
    fresh_rt.step(Duration::from_nanos(FRESH_FIRST_TICK_NS));
    let first = log_table(&capture(&fresh));
    assert_eq!(
        word_of(&first, throttle_id),
        TICK_NS + 1,
        "the first restored tick is inside the saved period"
    );
    for _ in 0..15 {
        fresh_rt.step(Duration::from_nanos(TICK_NS));
    }
    let inside = capture(&fresh);
    assert_eq!(
        inside[..4],
        26u32.to_le_bytes(),
        "the restored tick, then 10 catch-up ticks, then 15"
    );
    assert_eq!(
        log_table(&inside),
        vec![
            (ids[0], word_of(&table, ids[0])),
            (ids[1], word_of(&table, ids[1]))
        ],
        "both sites kept the saved word"
    );
    // One more tick at 260 ms is 250 ms after the saved emit: the throttle
    // site emits and takes 260 ms plus one, the once site stays at 1.
    fresh_rt.step(Duration::from_nanos(TICK_NS));
    let resumed = log_table(&capture(&fresh));
    assert_eq!(
        word_of(&resumed, once_id),
        1,
        "the once site stays suppressed"
    );
    assert_eq!(
        word_of(&resumed, throttle_id),
        260_000_001,
        "the throttle site resumed from the saved time"
    );

    // The ticked instance, restored from field-only bytes, drops its own
    // live log state: the restored table is the checkpoint's, empty.
    restore(&mut ticked_rt, &ticked, &5u32.to_le_bytes());
    assert_eq!(capture(&ticked), 5u32.to_le_bytes());
}

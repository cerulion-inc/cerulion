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
//!   throttle word is the graph time of that tick plus one, so it is never 0;
//! - a fresh instance restored from those bytes captures them back byte for
//!   byte: the section crossed the wall in both directions;
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
    let mut path = match std::env::var("CARGO_TARGET_DIR") {
        Ok(dir) if !dir.is_empty() => std::path::PathBuf::from(dir),
        _ => {
            let mut p = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
            p.pop();
            p.pop();
            p.push("target");
            p
        }
    };
    path.push("debug");
    let name = "test_node_macro_log_throttle_cdylib";
    if cfg!(target_os = "macos") {
        path.push(format!("lib{name}.dylib"));
    } else {
        path.push(format!("lib{name}.so"));
    }
    assert!(
        path.exists(),
        "test cdylib not found at {path:?}. Run `cargo build -p {name}` first."
    );
    path
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

#[test]
#[serial]
fn a_cdylib_checkpoint_carries_its_log_state_and_a_restore_takes_it_from_the_bytes() {
    let (mut ticked_rt, ticked) = built("ticked");
    assert_eq!(
        capture(&ticked),
        0u32.to_le_bytes(),
        "no tick, no log section"
    );

    ticked_rt.step(Duration::from_millis(10));
    let after_one_tick = capture(&ticked);
    assert_eq!(
        after_one_tick.len(),
        4 + 4 + 4 + 2 * 16,
        "{after_one_tick:?}"
    );
    assert_eq!(after_one_tick[..4], 1u32.to_le_bytes(), "ticks");
    assert_eq!(&after_one_tick[4..8], b"CLOG");
    assert_eq!(after_one_tick[8..12], 2u32.to_le_bytes(), "two sites ran");
    let pair = |at: usize| {
        let word = |o: usize| u64::from_le_bytes(after_one_tick[o..o + 8].try_into().unwrap());
        (word(at), word(at + 8))
    };
    let ((id_a, word_a), (id_b, word_b)) = (pair(12), pair(28));
    assert!(id_a < id_b, "ascending ids");
    let mut words = [word_a, word_b];
    words.sort_unstable();
    assert_eq!(words[0], 1, "the once word");
    assert!(
        words[1] >= 1,
        "the throttle word is a time plus one: {words:?}"
    );

    // A fresh instance takes the section from the bytes, and gives it back.
    let (mut fresh_rt, fresh) = built("fresh");
    restore(&mut fresh_rt, &fresh, &after_one_tick);
    assert_eq!(
        capture(&fresh),
        after_one_tick,
        "the section crossed the wall"
    );

    // The ticked instance, restored from field-only bytes, drops its own
    // live log state: the restored table is the checkpoint's, empty.
    restore(&mut ticked_rt, &ticked, &5u32.to_le_bytes());
    assert_eq!(capture(&ticked), 5u32.to_le_bytes());
}

// SPDX-License-Identifier: AGPL-3.0-only
//! Integration tests for NodeEntry implementations.
//!
//! Tests both `ClosureNodeEntry` (in-process) and `DylibNodeEntry` (cdylib loading).
//!
//! # Running
//!
//! ```bash
//! # Build the test cdylib first
//! cargo build -p test_node_cdylib
//! # Then run
//! cargo test -p cerulion_core --test node_test
//! ```

use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Arc;

use cerulion_core::graph::node::{
    ClosureNodeEntry, DylibNodeEntry, NodeContext, NodeEntry, NodeInfo,
};
use indexmap::IndexMap;

// ============================================================
// Checkpoint 5.4b: ClosureNodeEntry info
// ============================================================

#[test]
fn test_node_info_from_closure_entry() {
    let info = NodeInfo::from_names(
        vec!["trigger".to_string()],
        vec!["image".to_string(), "metadata".to_string()],
    );

    let node = ClosureNodeEntry::new(info, |_ctx| Ok(()));
    let retrieved = node.info().expect("info should parse");

    // NodeInfo no longer carries node_type — the type
    // is the folder name and is resolved at graph-load time.
    assert_eq!(retrieved.input_names(), vec!["trigger"]);
    assert_eq!(retrieved.output_names(), vec!["image", "metadata"]);
}

// ============================================================
// Checkpoint 5.5b: ClosureNodeEntry init and tick
// ============================================================

#[test]
fn test_closure_node_init_and_tick() {
    let counter = Arc::new(AtomicU32::new(0));
    let counter_clone = Arc::clone(&counter);

    let info = NodeInfo::from_names(vec![], vec![]);

    let mut node = ClosureNodeEntry::new(info, move |_ctx| {
        counter_clone.fetch_add(1, Ordering::Relaxed);
        Ok(())
    });

    // Tick without init should fail
    assert!(node.tick().is_err());

    // Init then tick
    let ctx = NodeContext::for_tests(IndexMap::new(), IndexMap::new());
    node.init(ctx).unwrap();

    node.tick().unwrap();
    node.tick().unwrap();
    node.tick().unwrap();

    assert_eq!(counter.load(Ordering::Relaxed), 3);
}

// ============================================================
// Test: ClosureNodeEntry with init callback
// ============================================================

#[test]
fn test_closure_node_with_init_callback() {
    let init_called = Arc::new(AtomicU32::new(0));
    let init_clone = Arc::clone(&init_called);

    let info = NodeInfo::from_names(vec![], vec![]);

    let mut node = ClosureNodeEntry::new(info, |_ctx| Ok(())).with_init(move |_ctx| {
        init_clone.fetch_add(1, Ordering::Relaxed);
        Ok(())
    });

    let ctx = NodeContext::for_tests(IndexMap::new(), IndexMap::new());
    node.init(ctx).unwrap();

    assert_eq!(init_called.load(Ordering::Relaxed), 1);
}

// ============================================================
// Test: ClosureNodeEntry shutdown
// ============================================================

#[test]
fn test_closure_node_shutdown() {
    let shutdown_called = Arc::new(AtomicU32::new(0));
    let shutdown_clone = Arc::clone(&shutdown_called);

    let info = NodeInfo::from_names(vec![], vec![]);

    let mut node = ClosureNodeEntry::new(info, |_ctx| Ok(())).with_shutdown(move || {
        shutdown_clone.fetch_add(1, Ordering::Relaxed);
        Ok(())
    });

    let ctx = NodeContext::for_tests(IndexMap::new(), IndexMap::new());
    node.init(ctx).unwrap();
    node.shutdown().unwrap();

    assert_eq!(shutdown_called.load(Ordering::Relaxed), 1);

    // After shutdown, tick should fail (context dropped)
    assert!(node.tick().is_err());
}

// ============================================================
// Checkpoint 5.4: Load node info from dylib
// ============================================================

#[test]
fn test_load_node_info_from_dylib() {
    let dylib_path = find_test_cdylib();

    let node = DylibNodeEntry::load(&dylib_path).expect("should load test cdylib");
    let info = node.info().expect("info should parse");

    assert!(info.input_names().is_empty());
    assert!(info.output_names().is_empty());
}

// ============================================================
// Checkpoint 5.5: Load and call callback from dylib
// ============================================================

#[test]
fn test_load_and_call_callback() {
    let dylib_path = find_test_cdylib();

    let mut node = DylibNodeEntry::load(&dylib_path).expect("should load test cdylib");

    // Init
    let ctx = NodeContext::for_tests(IndexMap::new(), IndexMap::new());
    node.init(ctx).expect("init should succeed");

    // Tick multiple times
    for _ in 0..5 {
        node.tick().expect("tick should succeed");
    }

    // Shutdown
    node.shutdown().expect("shutdown should succeed");
}

// ============================================================
// Test: DylibNodeEntry with non-existent path
// ============================================================

#[test]
fn test_dylib_load_nonexistent_path() {
    let result = DylibNodeEntry::load(std::path::Path::new("/nonexistent/path.dylib"));
    assert!(result.is_err());
}

// ============================================================
// Test: NodeContext accessors
// ============================================================

#[test]
fn test_node_context_empty() {
    let ctx = NodeContext::for_tests(IndexMap::new(), IndexMap::new());
    assert!(ctx.publisher("missing").is_none());
    assert!(ctx.subscriber("missing").is_none());
    assert_eq!(ctx.publisher_names().count(), 0);
    assert_eq!(ctx.subscriber_names().count(), 0);
}

/// Find the test cdylib built by `cargo build -p test_node_cdylib`.
///
/// Resolution (platform file name, profile, and the target directory itself)
/// is delegated to [`cerulion_core::testing::find_fixture_cdylib`], which
/// derives the target dir from THIS test binary's own location — so an
/// isolated `CARGO_TARGET_DIR` resolves to the tree the fixture was actually
/// built into instead of a hardcoded `<repo_root>/target`.
fn find_test_cdylib() -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_cdylib")
}

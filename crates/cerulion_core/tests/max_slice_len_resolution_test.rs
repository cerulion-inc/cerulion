// SPDX-License-Identifier: AGPL-3.0-only
//! 3-tier `max_slice_len` resolution coverage.
//!
//! - **Tier 1** — explicit `max_slice_len:` in graph YAML wins.
//! - **Tier 2** — `<T as ShmMessage>::MAX_SLICE_LEN` consulted via
//!   `OutputMeta::max_slice_len_default`.
//! - **Tier 3** — `DEFAULT_MAX_SLICE_LEN` (128 MiB) — final fallback;
//!   emits `tracing::warn!` so operators see they are using the
//!   coarse default.
//!
//! These tests verify the ACTUAL resolved value: the `StubEntry`
//! captures each publisher's configured `max_slice_len()` during
//! `init()` into a shared `Arc<Mutex<HashMap>>`, so the test asserts
//! tier-1/tier-2/tier-3 selected the expected number rather than just
//! "the build didn't panic" (a build-only assertion would be
//! tautological).

use cerulion_core::wire::MaxSliceLen;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::DEFAULT_MAX_SLICE_LEN;
use cerulion_core::graph::node::{NodeContext, NodeEntry, NodeInfo, OutputMeta};
use cerulion_core::graph::{parse_graph, GraphRuntime};
use cerulion_core::TransportResult;
use indexmap::IndexMap;

/// Per-test resolved-value capture: maps "node_id/output_name" →
/// publisher's configured `max_slice_len`. The publisher
/// surface is `u32` (matches the wire format's `WireHeader::total_size`).
type ResolvedMap = Arc<Mutex<HashMap<String, u32>>>;

/// Test stub: implements `NodeEntry` with a configurable `OutputMeta`
/// list AND a shared capture map. During `init()`, walks every
/// publisher in the context and records its `max_slice_len()` so the
/// test can assert the runtime resolved to the expected tier.
struct StubEntry {
    node_id: String,
    output_meta: Vec<OutputMeta>,
    captured: ResolvedMap,
    context: Option<NodeContext>,
}

impl NodeEntry for StubEntry {
    fn info(&self) -> TransportResult<NodeInfo> {
        Ok(NodeInfo::with_input_names_and_output_meta(
            Vec::new(),
            self.output_meta.clone(),
        ))
    }

    fn init(&mut self, context: NodeContext) -> TransportResult<()> {
        let names: Vec<String> = context.publisher_names().map(|s| s.to_string()).collect();
        let mut map = self.captured.lock().expect("test mutex");
        for name in &names {
            if let Some(pub_port) = context.publisher(name) {
                map.insert(
                    format!("{}/{}", self.node_id, name),
                    pub_port.max_slice_len().get(),
                );
            }
        }
        // Hold the context so the publisher Arc / channel state survives
        // for the duration of the test even though tick is never called.
        self.context = Some(context);
        Ok(())
    }

    fn tick(&mut self) -> TransportResult<()> {
        Ok(())
    }

    fn shutdown(&mut self) -> TransportResult<()> {
        Ok(())
    }
}

fn build_runtime(
    yaml: &str,
    output_meta_per_node: &[(&str, Vec<OutputMeta>)],
) -> (GraphRuntime, ResolvedMap) {
    let config = parse_graph(yaml).unwrap();
    let captured: ResolvedMap = Arc::new(Mutex::new(HashMap::new()));
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    for (id, output_meta) in output_meta_per_node {
        factories.insert(
            (*id).to_string(),
            Box::new(StubEntry {
                node_id: (*id).to_string(),
                output_meta: output_meta.clone(),
                captured: Arc::clone(&captured),
                context: None,
            }),
        );
    }
    let clock = Arc::new(VirtualClock::new());
    let runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("graph should build");
    (runtime, captured)
}

fn resolved(captured: &ResolvedMap, key: &str) -> u32 {
    *captured
        .lock()
        .expect("test mutex")
        .get(key)
        .unwrap_or_else(|| panic!("no captured max_slice_len for key {}", key))
}

#[test]
fn test_tier1_explicit_yaml_max_slice_len_wins() {
    // YAML sets max_slice_len: 4096 explicitly. Even though the macro-
    // emitted OutputMeta has max_slice_len_default = Some(16 MiB), the
    // YAML value wins.
    let yaml = r#"
name: t1
nodes:
  - id: pub1
    type: stub
    outputs:
      - name: data
        schema: test/Data
        max_slice_len: 4096
"#;
    let output_meta = vec![OutputMeta::new(
        "data".to_string(),
        0xabcd,
        MaxSliceLen::try_new(16 * 1024 * 1024),
    )];
    let (_runtime, captured) = build_runtime(yaml, &[("pub1", output_meta)]);
    assert_eq!(resolved(&captured, "pub1/data"), 4096);
}

#[test]
fn test_tier2_schema_default_used_when_yaml_omits() {
    // YAML omits max_slice_len. The macro-emitted OutputMeta carries
    // max_slice_len_default = Some(64 KiB). Resolution lands on 64 KiB.
    let yaml = r#"
name: t2
nodes:
  - id: pub1
    type: stub
    outputs:
      - name: data
        schema: test/Data
"#;
    let output_meta = vec![OutputMeta::new(
        "data".to_string(),
        0xabcd,
        MaxSliceLen::try_new(64 * 1024),
    )];
    let (_runtime, captured) = build_runtime(yaml, &[("pub1", output_meta)]);
    assert_eq!(resolved(&captured, "pub1/data"), 64 * 1024);
}

#[test]
fn test_tier3_default_max_slice_len_used_when_neither_provided() {
    // YAML omits max_slice_len AND macro-emitted OutputMeta has
    // max_slice_len_default = None (e.g. user-written impl with no
    // MAX_SLICE_LEN const). Resolution falls back to
    // DEFAULT_MAX_SLICE_LEN (128 MiB) and emits a tracing::warn!.
    let yaml = r#"
name: t3
nodes:
  - id: pub1
    type: stub
    outputs:
      - name: data
        schema: test/Data
"#;
    let output_meta = vec![OutputMeta::new("data".to_string(), 0xabcd, None)];
    let (_runtime, captured) = build_runtime(yaml, &[("pub1", output_meta)]);
    assert_eq!(
        resolved(&captured, "pub1/data"),
        DEFAULT_MAX_SLICE_LEN as u32
    );
    assert_eq!(DEFAULT_MAX_SLICE_LEN, 128 * 1024 * 1024);
}

#[test]
fn test_tier3_default_used_when_output_meta_empty() {
    // Closure-based entries (and a cdylib that exports no output metadata) return empty OutputMeta.
    // Resolution must still fall through to tier-3 cleanly.
    let yaml = r#"
name: t3b
nodes:
  - id: pub1
    type: stub
    outputs:
      - name: data
        schema: test/Data
"#;
    let (_runtime, captured) = build_runtime(yaml, &[("pub1", Vec::new())]);
    assert_eq!(
        resolved(&captured, "pub1/data"),
        DEFAULT_MAX_SLICE_LEN as u32
    );
}

#[test]
fn test_tier1_overrides_tier2_when_user_explicitly_sets_smaller() {
    // YAML sets 8 KiB explicitly; macro-emitted OutputMeta says
    // 16 MiB. User wins — even at a smaller value.
    let yaml = r#"
name: t4
nodes:
  - id: pub1
    type: stub
    outputs:
      - name: data
        schema: test/Data
        max_slice_len: 8192
"#;
    let output_meta = vec![OutputMeta::new(
        "data".to_string(),
        0xabcd,
        MaxSliceLen::try_new(16 * 1024 * 1024),
    )];
    let (_runtime, captured) = build_runtime(yaml, &[("pub1", output_meta)]);
    assert_eq!(resolved(&captured, "pub1/data"), 8192);
}

#[test]
fn test_tier2_yaml_output_name_mismatch_falls_through_to_tier3() {
    // Adversarial: the YAML declares an output named "data" but the
    // macro-emitted OutputMeta is keyed by name "image". The lookup
    // fails to find a matching entry, so tier-2 silently no-ops and
    // tier-3 fires (DEFAULT + tracing::warn!).
    let yaml = r#"
name: t5
nodes:
  - id: pub1
    type: stub
    outputs:
      - name: data
        schema: test/Data
"#;
    let output_meta = vec![OutputMeta::new(
        "image".to_string(),
        0xabcd,
        MaxSliceLen::try_new(64 * 1024),
    )];
    let (_runtime, captured) = build_runtime(yaml, &[("pub1", output_meta)]);
    assert_eq!(
        resolved(&captured, "pub1/data"),
        DEFAULT_MAX_SLICE_LEN as u32
    );
}

#[test]
fn test_tier2_undersized_meta_value_rejected_at_newtype_construction() {
    // Adversarial: the macro-emitted MAX_SLICE_LEN const is buggy or
    // missing the wire-header offset (e.g. 8 — smaller than
    // WireHeader::SIZE = 32). The resolver needs no runtime
    // floor check: the rejection happens at
    // `MaxSliceLen::try_new` construction (compile-time-aware
    // fallible constructor) — `try_new(8)` returns `None` before the
    // resolver is even called. The resolver then sees
    // `meta.max_slice_len_default == None` and falls through to
    // tier-3 normally.
    //
    // This test pins the TYPE-SYSTEM property (try_new rejects < 32)
    // separately from the resolver behavior. If `try_new`'s lower
    // bound is ever loosened, this test will fail loud — independent
    // of whether the resolver's downstream behavior also changes.
    assert!(
        MaxSliceLen::try_new(8).is_none(),
        "MaxSliceLen::try_new must reject values < WireHeader::SIZE (32)"
    );
    assert!(
        MaxSliceLen::try_new(31).is_none(),
        "MaxSliceLen::try_new must reject 31 (just below floor)"
    );
    assert!(
        MaxSliceLen::try_new(32).is_some(),
        "MaxSliceLen::try_new must accept WireHeader::SIZE (32) — header-only payload is valid"
    );

    // Belt-and-suspenders: also verify the resolver-level fallthrough
    // when tier-2 is None (the typed `Option<MaxSliceLen>` shape).
    let yaml = r#"
name: t6
nodes:
  - id: pub1
    type: stub
    outputs:
      - name: data
        schema: test/Data
"#;
    let output_meta = vec![OutputMeta::new(
        "data".to_string(),
        0xabcd,
        None, // simulates a schema with no MAX_SLICE_LEN declared
    )];
    let (_runtime, captured) = build_runtime(yaml, &[("pub1", output_meta)]);
    assert_eq!(
        resolved(&captured, "pub1/data"),
        DEFAULT_MAX_SLICE_LEN as u32
    );
}

// There is no tier-2 oversized-value test (a clamp to `u32::MAX`):
// that contract is a type-system
// property. `OutputMeta::max_slice_len_default` is typed
// `Option<NonZeroU32>`; a tier-2 value
// above `u32::MAX` is unrepresentable at the trait surface. The
// resolver's `u32::try_from` clamp remains as defense-in-depth but
// is unreachable from compile-checked code paths — see the
// resolver doc-block in `runtime.rs::resolve_max_slice_len`.

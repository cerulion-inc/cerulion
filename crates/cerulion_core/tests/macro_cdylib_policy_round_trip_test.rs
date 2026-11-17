// SPDX-License-Identifier: AGPL-3.0-only
//! The macro→runtime
//! policy round-trip oracle.
//!
//! Each test loads a cdylib whose `#[cerulion_node]` attribute declared
//! a specific `MacroPolicy` variant, calls `info()` via the loader, and
//! asserts the parsed `NodeInfo::policy` matches the declared variant
//! exactly. Locks the contract:
//!
//!   #[cerulion_node(period_ms = 50)]
//!     → cdylib JSON: `"policy":{"period_ms":50}`
//!     → host parse_info_json: `MacroPolicy::Period { period_ms: 50 }`
//!
//! A regression to either side (macro emission OR host parsing) fires
//! the test, which exercises the property
//! end-to-end.
//!
//! Adversarial coverage: `test_node_macro_cdylib` declares
//! NO node-level policy attribute, only `#[input(trigger)]`; it must
//! round-trip as `MacroPolicy::DataTrigger`. A
//! regression that started emitting a default policy (e.g. an empty
//! `policy: {}` object misparsed as `Period { period_ms: 0 }`) would
//! fire here.

use cerulion_core::graph::node::{DylibNodeEntry, MacroPolicy, NodeEntry};
use cerulion_core::message::ShmMessage;
use native_ros2_messages::geometry_msgs::Vector3;

const HELPER_CHILD: &str = "CER_STABLE_HELPER_CHILD";
const HELPER_EVENTS: &str = "CER_STABLE_HELPER_EVENTS";
const HELPER_CONTEXT_STOP: &str = "CER_STABLE_HELPER_CONTEXT_STOP";

// The child owns its SHM singleton; parent metadata tests remain parallel-safe.
#[test]
#[ignore]
fn helper_shutdown_child() {
    if std::env::var(HELPER_CHILD).as_deref() != Ok("1") {
        return;
    }
    let path = cerulion_core::testing::find_fixture_cdylib_same_profile(
        "test_node_nondeterministic_cdylib",
    );
    let entry = DylibNodeEntry::load(&path).expect("stable helper fixture");
    let mut graph = cerulion_core::graph::parse_graph(
        "prefix: shutdownhelper\nnodes:\n  - id: helper\n    type: helper\n    outputs:\n      - name: cmd\n        schema: geometry_msgs/Vector3\n        max_slice_len: 64\n",
    )
    .expect("helper graph");
    graph.prefix = format!("shutdownhelper{}", std::process::id());
    let mut entries: indexmap::IndexMap<String, Box<dyn NodeEntry>> = indexmap::IndexMap::new();
    entries.insert("helper".into(), Box::new(entry));
    let mut runtime = cerulion_core::graph::GraphRuntime::build_for_test(
        graph,
        entries,
        std::sync::Arc::new(cerulion_core::clock::VirtualClock::new()),
        8,
    )
    .expect("live helper context");
    runtime.step(std::time::Duration::from_millis(50));
    let events = std::path::PathBuf::from(std::env::var(HELPER_EVENTS).expect("event path"));
    let context_stop = std::env::var(HELPER_CONTEXT_STOP).as_deref() == Ok("1");
    loop {
        let observed = std::fs::read(&events).unwrap_or_default();
        if (context_stop && observed == [1, 2]) || (!context_stop && observed == [1]) {
            break;
        }
        std::thread::yield_now();
    }
    if context_stop {
        assert!(
            runtime.shutdown_requested(),
            "context request reaches host too"
        );
    } else {
        runtime.shutdown_signal().request();
    }
    runtime.shutdown_all_nodes();
    assert_eq!(
        std::fs::read(&events).expect("joined helper events"),
        [1, 2, 3]
    );
    std::fs::write(events, [1, 2, 3, 4]).expect("graph shutdown returned");
}

#[test]
fn stable_helper_shutdown_joins_after_host_and_context_requests() {
    use std::process::{Command, Stdio};
    use std::time::{Duration, Instant};
    const BOUND: Duration = Duration::from_secs(30);
    for context_stop in [false, true] {
        for _ in 0..2 {
            let directory = tempfile::tempdir().expect("isolated helper events");
            let events = directory.path().join("events.bin");
            let mut child = Command::new(std::env::current_exe().expect("child executable"))
                .args([
                    "--exact",
                    "helper_shutdown_child",
                    "--ignored",
                    "--nocapture",
                ])
                .env(HELPER_CHILD, "1")
                .env(HELPER_EVENTS, &events)
                .env(HELPER_CONTEXT_STOP, if context_stop { "1" } else { "0" })
                .env("RUST_LOG", "off")
                .stdout(Stdio::null())
                .stderr(Stdio::piped())
                .spawn()
                .expect("helper child");
            let start = Instant::now();
            while child.try_wait().expect("child status").is_none() {
                if start.elapsed() >= BOUND {
                    child.kill().expect("kill only this spawned child");
                    let output = child.wait_with_output().expect("reap helper child");
                    panic!(
                        "helper shutdown exceeded bound: {}",
                        String::from_utf8_lossy(&output.stderr)
                    );
                }
                std::thread::sleep(Duration::from_millis(5));
            }
            let output = child.wait_with_output().expect("helper child output");
            assert!(
                output.status.success(),
                "{}",
                String::from_utf8_lossy(&output.stderr)
            );
            assert_eq!(
                std::fs::read(events).expect("helper exit observed"),
                [1, 2, 3, 4]
            );
        }
    }
}

/// Locate a cdylib produced by the workspace build. The fixture crates
/// are workspace members, so `cargo test` triggers their build
/// transitively.
fn find_cdylib(crate_name: &str) -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib(crate_name)
}

#[test]
fn cdylib_info_carries_macro_declared_period_ms() {
    let path = find_cdylib("test_node_macro_period_cdylib");
    let node = DylibNodeEntry::load(&path).expect("load period cdylib");
    let info = node.info().expect("info should parse");
    assert_eq!(
        info.policy(),
        Some(MacroPolicy::Period { period_ms: 50 }),
        "macro-declared period_ms = 50 must round-trip through cdylib JSON \
         and host parse_info_json; got policy={:?}",
        info.policy()
    );
}

#[test]
fn cdylib_info_carries_macro_declared_sync_window_ms() {
    let path = find_cdylib("test_node_macro_sync_cdylib");
    let node = DylibNodeEntry::load(&path).expect("load sync cdylib");
    let info = node.info().expect("info should parse");
    assert_eq!(
        info.policy(),
        Some(MacroPolicy::Sync { window_ms: 25 }),
        "macro-declared sync_window_ms = 25 must round-trip; got policy={:?}",
        info.policy()
    );
    // The sync fixture declares two trigger inputs (cam, imu), which the
    // runtime threads through `macro_policy_to_trigger`.
    // Here we only need to lock the policy round-trip; assert input
    // names AND ORDER (insertion order must be preserved for the downstream
    // trigger-policy synthesis to be deterministic).
    assert_eq!(
        info.input_names(),
        vec!["cam".to_string(), "imu".to_string()],
        "input names must preserve declaration order; got: {:?}",
        info.input_names()
    );
    // ABI v9: the per-input `#[input(trigger)]` mark rides the
    // cdylib FFI. Both `cam` and `imu` are declared `#[input(trigger, ...)]`,
    // so the full macro-emission → FFI → host-parse round-trip must land
    // `trigger == true` on each. A host that hardcodes `false` here fails this.
    let meta = info.input_meta();
    assert_eq!(meta.len(), 2);
    assert_eq!(meta[0].name, "cam");
    assert!(
        meta[0].trigger,
        "cam is `#[input(trigger)]` — must round-trip"
    );
    assert_eq!(meta[1].name, "imu");
    assert!(
        meta[1].trigger,
        "imu is `#[input(trigger)]` — must round-trip"
    );
}

#[test]
fn cdylib_info_carries_macro_declared_external() {
    let path = find_cdylib("test_node_macro_external_cdylib");
    let node = DylibNodeEntry::load(&path).expect("load external cdylib");
    let info = node.info().expect("info should parse");
    assert_eq!(
        info.policy(),
        Some(MacroPolicy::External),
        "macro-declared `external` must round-trip; got policy={:?}",
        info.policy()
    );
}

/// The `unbounded_sync` node-level attr must round-trip across the
/// cdylib FFI. Without an `unbounded_sync` arm in `gen_cdylib`'s `policy_json`
/// chain the emitted `cerulion_node_info()` JSON carries no
/// `"policy"` key and `info().policy()` is `None`. The runtime then defaults
/// the node to `TriggerPolicy::Data` — fires on ANY single input arrival
/// (`runtime.rs` build loop, `macro_policy: None` arm) — instead of the
/// all-inputs UnboundedSync contract, so a fusion cdylib silently stops
/// waiting for all its inputs. This assertion FAILS without that arm (policy `None`).
///
/// The fixture declares two `#[input(trigger)]` fields (a, b) — `unbounded_sync`
/// requires ≥2 trigger inputs (macro-enforced by `validate_trigger_inference`).
#[test]
fn cdylib_info_carries_macro_declared_unbounded_sync() {
    let path = find_cdylib("test_node_macro_unbounded_sync_cdylib");
    let node = DylibNodeEntry::load(&path).expect("load unbounded_sync cdylib");
    let info = node.info().expect("info should parse");
    assert_eq!(
        info.policy(),
        Some(MacroPolicy::UnboundedSync),
        "macro-declared `unbounded_sync` must round-trip through cdylib JSON \
         and host parse_info_json; got policy={:?} (None when the gen_cdylib \
         arm is missing — it degrades the node to TriggerPolicy::Data)",
        info.policy()
    );
    // Input names AND ORDER preserved (declaration order a, b) — downstream
    // `macro_policy_to_trigger` maps ALL inputs in order for the synthesized
    // UnboundedSync trigger, so order must be deterministic.
    assert_eq!(
        info.input_names(),
        vec!["a".to_string(), "b".to_string()],
        "input names must preserve declaration order; got: {:?}",
        info.input_names()
    );
}

/// Round-trip for the canonical
/// `#[input(trigger)]`-without-node-level-attr pattern. A
/// macro that silently dropped the trigger field on the FFI floor would leave
/// `info().policy` `None` (a silent never-fire, which the default-policy
/// warn makes loud). The macro emits
/// `"policy":{"data_trigger":{"input_name":"velocity_in"}}` and the
/// host parses it into `MacroPolicy::DataTrigger { input_name }`.
///
/// `test_node_macro_cdylib` declares `#[input(trigger, depth = 1)]
/// velocity_in: Vector3` plus an output, with no node-level policy
/// attribute. The trigger flag survives the macro's input-attr
/// parser, gets filtered by the `single_trigger_input_name`
/// computation in `gen_node_entry`, and lands in `policy_json`.
#[test]
fn cdylib_input_trigger_only_yields_macro_data_trigger_policy() {
    let path = find_cdylib("test_node_macro_cdylib");
    let node = DylibNodeEntry::load(&path).expect("load input-trigger-only cdylib");
    let info = node.info().expect("info should parse");
    assert_eq!(
        info.policy(),
        Some(MacroPolicy::DataTrigger {
            input_name: "velocity_in".to_string(),
        }),
        "the canonical `#[input(trigger)]` shape must \
         round-trip as MacroPolicy::DataTrigger; got policy={:?}",
        info.policy()
    );
}

/// Dedicated round-trip fixture. Mirrors the
/// `cdylib_input_trigger_only_…` test above against a separate
/// fixture (`test_node_macro_data_trigger_cdylib`) whose ONLY input
/// attribute is `#[input(trigger)]` (no depth modifier). A
/// regression in the macro's filter logic (e.g. the
/// `single_trigger_input_name` predicate accidentally requiring
/// extra attrs, or excluding the case where the field has none)
/// would fire here even if the `test_node_macro_cdylib` test passed.
#[test]
fn cdylib_dedicated_data_trigger_fixture_round_trips() {
    let path = find_cdylib("test_node_macro_data_trigger_cdylib");
    let node = DylibNodeEntry::load(&path).expect("load data-trigger fixture");
    let info = node.info().expect("info should parse");
    assert_eq!(
        info.policy(),
        Some(MacroPolicy::DataTrigger {
            input_name: "trigger_in".to_string(),
        }),
        "dedicated data-trigger fixture must round-trip; got policy={:?}",
        info.policy()
    );
    // ABI v9: the `#[input(trigger)]` mark on `trigger_in` rides the
    // FFI too — the parsed `InputMeta` carries `trigger == true`.
    let meta = info.input_meta();
    assert_eq!(meta.len(), 1);
    assert_eq!(meta[0].name, "trigger_in");
    assert!(
        meta[0].trigger,
        "`trigger_in` is `#[input(trigger)]` — the mark must round-trip via ABI v9"
    );
}

/// The per-input `schema_hash` rides the cdylib info JSON. Were the
/// macro to emit NO input schema_hash, a `DylibNodeEntry`-loaded consumer
/// would land `InputMeta.schema_hash == 0` and the network ingress-hash
/// resolver (`resolve_ingress_schema_hash`) would refuse EVERY cdylib consumer with
/// "no consumer with a declared schema" — the resolver would only work for
/// in-process macro nodes (test-only).
///
/// The `test_node_macro_data_trigger_cdylib` fixture declares
/// `#[input(trigger)] trigger_in: Vector3`. Assert its parsed hash is (a)
/// non-zero and (b) EXACTLY the in-process twin type's const —
/// `<Vector3 as ShmMessage>::SCHEMA_HASH` — a real parity cross-check, NOT a
/// self-compare. A regression on either side (macro emission dropping the key,
/// or the host parser hardcoding `0`) fires here.
#[test]
fn cdylib_input_meta_carries_schema_hash_matching_in_process_twin() {
    let path = find_cdylib("test_node_macro_data_trigger_cdylib");
    let node = DylibNodeEntry::load(&path).expect("load data-trigger fixture");
    let info = node.info().expect("info should parse");
    let meta = info.input_meta();
    assert_eq!(meta.len(), 1);
    assert_eq!(meta[0].name, "trigger_in");
    assert_ne!(
        meta[0].schema_hash, 0,
        "a cdylib input must carry a real schema_hash, not the 0 \
         sentinel that broke the ingress-hash resolver"
    );
    assert_eq!(
        meta[0].schema_hash,
        <Vector3 as ShmMessage>::SCHEMA_HASH,
        "the cdylib input schema_hash must equal the in-process Vector3 \
         SCHEMA_HASH const (macro-emission ↔ FFI ↔ host-parse parity)"
    );
}

/// A `#[cerulion_node]` cdylib's info JSON carries
/// `"publisher_for_every_output":true`, and the host parse keeps it. Without
/// the key the host reads `false`, so the graph build does not refuse an
/// output the node declares and its graph does not list, and the node then
/// fails every tick. This test fails if `gen_cdylib` drops the key.
#[test]
fn cdylib_info_marks_a_macro_node_as_needing_a_publisher_for_every_output() {
    let path = find_cdylib("test_node_macro_cdylib");
    let node = DylibNodeEntry::load(&path).expect("load macro cdylib");
    let info = node.info().expect("info should parse");
    assert!(
        info.publisher_for_every_output(),
        "the macro cdylib info JSON must carry \"publisher_for_every_output\":true"
    );
}

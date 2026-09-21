// SPDX-License-Identifier: AGPL-3.0-only
//! `tracing-test` warn-emission coverage for the 3-tier `max_slice_len`
//! resolution ladder.
//!
//! The sibling `max_slice_len_resolution_test.rs` pins the resolved
//! *value* per tier via a publisher-capture map. These tests pin the
//! observable *side effect*: tier-3 fall-through emits a
//! `tracing::warn!` carrying `topic` / `schema` / `default_bytes`
//! structured fields, and tier-2 resolution is SILENT (no warn for the
//! resolved topic). Both drive the real `GraphRuntime::build_for_test`
//! path — the warn fires from `cerulion_core::graph::runtime`, not the
//! test crate, so the `tracing-test` dev-dep is configured with
//! `no-env-filter` (see `cerulion_core/Cargo.toml`) to capture it.
//!
//! Each test is `#[traced_test]` (per-test log capture) so the asserted
//! warn state is isolated to that test's build.
//! `build_for_test` provisions the graph over the real iceoryx2
//! transport, which shares the
//! process-global `TransportManager` singleton — so each test is also
//! `#[serial]` (serial_test dev-dep) to serialize against the shared SHM.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::DEFAULT_MAX_SLICE_LEN;
use cerulion_core::graph::node::{NodeContext, NodeEntry, NodeInfo, OutputMeta};
use cerulion_core::graph::{parse_graph, GraphRuntime};
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::TransportResult;
use indexmap::IndexMap;
use serial_test::serial;
use tracing_test::traced_test;

/// Capture for the publisher's resolved `max_slice_len` (output name →
/// bytes), recorded during `init()`. Lets the tier-2 test assert the
/// schema default was actually USED, not merely that the tier-3 warn was
/// absent.
type ResolvedCapture = Arc<Mutex<Option<u32>>>;

/// Minimal `NodeEntry` stub: declares a configurable `OutputMeta` list
/// so the runtime's tier-2 lookup sees exactly the schema-default state
/// each test wants (present vs absent). During `init()` it records its
/// lone publisher's resolved `max_slice_len` into `captured`, and holds
/// the `NodeContext` so the publisher state survives for the test body;
/// it never ticks.
struct StubEntry {
    output_meta: Vec<OutputMeta>,
    captured: ResolvedCapture,
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
        // Record the resolved max_slice_len for the (single) publisher so
        // a test can assert the tier that fired by VALUE, not just by the
        // presence/absence of a warn.
        if let Some(name) = context.publisher_names().next() {
            if let Some(pub_port) = context.publisher(name) {
                *self.captured.lock().expect("test mutex") = Some(pub_port.max_slice_len().get());
            }
        }
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

/// Build a single-publisher graph (over the real iceoryx2 transport via
/// `build_for_test`) with the given output meta. Returns the runtime (so
/// its publishers — and their resolved `max_slice_len` — stay alive for
/// the assertion) AND the capture of the resolved `max_slice_len`. The
/// warn-emission side effect happens DURING this `build_for_test` call,
/// while the `#[traced_test]` subscriber is installed.
fn build_single_output(
    yaml: &str,
    output_meta: Vec<OutputMeta>,
) -> (GraphRuntime, ResolvedCapture) {
    let config = parse_graph(yaml).expect("graph YAML must parse");
    let captured: ResolvedCapture = Arc::new(Mutex::new(None));
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "pub1".to_string(),
        Box::new(StubEntry {
            output_meta,
            captured: Arc::clone(&captured),
            context: None,
        }),
    );
    let clock = Arc::new(VirtualClock::new());
    let runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("graph should build");
    (runtime, captured)
}

/// Build a single-publisher graph over the real iceoryx2 transport with
/// the given output meta AND an optional schema-name → recipe-3 hash map
/// for the divergence warn. Routes through `build_for_test_with_schema_hashes` so the
/// YAML-`schema:`-vs-macro-output divergence warn can be exercised
/// end-to-end. Returns the runtime so its publishers stay alive for the
/// assertion.
fn build_single_output_with_schema_hashes(
    yaml: &str,
    output_meta: Vec<OutputMeta>,
    schema_hashes: Option<&HashMap<String, u64>>,
) -> GraphRuntime {
    let config = parse_graph(yaml).expect("graph YAML must parse");
    let captured: ResolvedCapture = Arc::new(Mutex::new(None));
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "pub1".to_string(),
        Box::new(StubEntry {
            output_meta,
            captured,
            context: None,
        }),
    );
    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test_with_schema_hashes(config, factories, clock, 8, schema_hashes)
        .expect("graph should build")
}

/// YAML for a single publisher whose lone output OMITS `max_slice_len:`
/// — so tier-1 is absent and resolution is decided by the `OutputMeta`
/// the stub declares (tier-2 present → silent; tier-2 absent → tier-3
/// warn).
const YAML_NO_TIER1: &str = r#"
name: warn_emission
nodes:
  - id: pub1
    type: stub
    outputs:
      - name: data
        schema: sensor_msgs/LaserScan
"#;

/// The exact YAML schema string the single-output stub graph carries — the
/// map key the runtime looks up for the divergence check.
const STUB_SCHEMA: &str = "sensor_msgs/LaserScan";

/// The macro-side `OutputMeta::schema_hash` the stub declares for its lone
/// output. The divergence warn fires iff the schema-hash map carries
/// `STUB_SCHEMA` with a DIFFERENT value.
const MACRO_OUTPUT_HASH: u64 = 0xAAAA_BBBB_CCCC_DDDD;

/// Unique substring of the divergence warn (`runtime.rs`
/// `warn_on_schema_hash_divergence`). Pinned here so a future reword fails
/// these tests loudly.
const DIVERGENCE_WARN_PHRASE: &str =
    "graph YAML `schema:` declares a layout the macro output does not produce";

/// Tier-3: YAML omits `max_slice_len` AND the schema has no
/// `MAX_SLICE_LEN` const (`OutputMeta::max_slice_len_default = None`,
/// e.g. a hand-written `NodeEntry` impl). Resolution falls through to
/// `DEFAULT_MAX_SLICE_LEN` and MUST emit a `tracing::warn!` carrying the
/// `topic`, `schema`, and `default_bytes` structured fields.
#[traced_test]
#[test]
#[serial]
fn tier3_emits_warn_with_structured_fields() {
    // tier-2 ABSENT: no schema default → tier-3 fires.
    let output_meta = vec![OutputMeta::new("data".to_string(), 0xabcd, None)];
    let (_runtime, captured) = build_single_output(YAML_NO_TIER1, output_meta);

    // The resolver landed on the tier-3 universal default (16 MiB).
    assert_eq!(
        *captured.lock().expect("test mutex"),
        Some(DEFAULT_MAX_SLICE_LEN as u32),
        "tier-3 must resolve the publisher to DEFAULT_MAX_SLICE_LEN"
    );

    // The warn message body.
    assert!(
        logs_contain("no max_slice_len configured for topic"),
        "tier-3 fall-through must emit its warn message"
    );

    // Structured fields. `tracing-test` captures the formatted event,
    // and the warn site renders `topic` / `schema` via `%` (Display), so
    // each is an UNQUOTED `key=value` pair; `default_bytes` is a numeric.
    // We assert each field independently so a future field rename or drop
    // fails loud and specifically.
    //
    // `schema` is the exact YAML schema string. `topic` is
    // `resolve_topic(prefix, node_id, output_name)` — its value carries a
    // machine-derived prefix, so we assert the field KEY plus the
    // wiring-stable `pub1/data` suffix rather than the full hostname.
    assert!(
        logs_contain("schema=sensor_msgs/LaserScan"),
        "warn must carry the `schema` structured field with the schema string"
    );
    assert!(
        logs_contain("topic=") && logs_contain("pub1/data"),
        "warn must carry the `topic` structured field (node_id/output_name suffix)"
    );
    assert!(
        logs_contain(&format!("default_bytes={DEFAULT_MAX_SLICE_LEN}")),
        "warn must carry the `default_bytes` structured field = DEFAULT_MAX_SLICE_LEN"
    );
}

/// Tier-2: YAML omits `max_slice_len` but the schema HAS a populated
/// `MAX_SLICE_LEN` const (`OutputMeta::max_slice_len_default =
/// Some(64 KiB)`). Tier-2 resolves silently — NO tier-3 warn for this
/// topic.
#[traced_test]
#[test]
#[serial]
fn tier2_does_not_emit_warn_when_resolved() {
    // tier-2 PRESENT: schema default carries 64 KiB.
    let output_meta = vec![OutputMeta::new(
        "data".to_string(),
        0xabcd,
        MaxSliceLen::try_new(64 * 1024),
    )];
    let (_runtime, captured) = build_single_output(YAML_NO_TIER1, output_meta);

    // POSITIVE: the schema default (64 KiB) was actually USED — this
    // discriminates tier-2 from any future silent tier inserted between 2
    // and 3, which the warn-absence assertion alone would not catch.
    assert_eq!(
        *captured.lock().expect("test mutex"),
        Some(64 * 1024),
        "tier-2 must resolve the publisher to the 64 KiB schema default"
    );

    // NEGATIVE: tier-2 resolution is silent — the tier-3 fall-through warn
    // must NOT appear. The message string is unique to tier-3
    // (`tier3_default`), so its absence confirms tier-3 was not taken.
    assert!(
        !logs_contain("no max_slice_len configured"),
        "tier-2 resolution must NOT emit the tier-3 fall-through warn"
    );
}

// ============================================================
// YAML-`schema:`-vs-macro-output schema-hash divergence warn.
//
// When the CLI supplies a (schema name → recipe-3 hash) map and a graph
// YAML output's `schema:` resolves to a hash DIFFERENT from the macro
// output field's `OutputMeta::schema_hash`, the runtime keeps the macro's
// max_slice_len default but warns loudly so the no-op `schema:` override is
// visible. These oracle cases pin (a) divergence → warn, (b) match → no
// warn, (c) map absent (`None`) → no warn (back-compat), (d) name absent
// from the map → no warn.
// ============================================================

/// (a) Divergence: the map carries the YAML schema name with a hash that
/// DIFFERS from the macro output's `OutputMeta::schema_hash` → the warn
/// fires with the schema and both hashes as structured fields.
#[traced_test]
#[test]
#[serial]
fn schema_hash_divergence_emits_warn() {
    let yaml_hash: u64 = 0x1111_2222_3333_4444; // ≠ MACRO_OUTPUT_HASH
    let output_meta = vec![OutputMeta::new(
        "data".to_string(),
        MACRO_OUTPUT_HASH,
        MaxSliceLen::try_new(64 * 1024),
    )];
    let mut map = HashMap::new();
    map.insert(STUB_SCHEMA.to_string(), yaml_hash);

    let _runtime = build_single_output_with_schema_hashes(YAML_NO_TIER1, output_meta, Some(&map));

    assert!(
        logs_contain(DIVERGENCE_WARN_PHRASE),
        "a YAML-vs-macro schema-hash mismatch must emit the divergence warn"
    );
    // Structured fields: the schema string + both hashes (rendered as
    // numeric `key=value` pairs). Assert each independently so a future
    // field rename/drop fails specifically.
    assert!(
        logs_contain(&format!("schema={STUB_SCHEMA}")),
        "warn must carry the `schema` structured field"
    );
    assert!(
        logs_contain(&format!("yaml_schema_hash={yaml_hash}")),
        "warn must carry the YAML-side `yaml_schema_hash` field"
    );
    assert!(
        logs_contain(&format!("macro_schema_hash={MACRO_OUTPUT_HASH}")),
        "warn must carry the macro-side `macro_schema_hash` field"
    );
}

/// (b) Match: the map carries the YAML schema name with a hash EQUAL to the
/// macro output's → NO divergence warn (the healthy common case).
#[traced_test]
#[test]
#[serial]
fn schema_hash_match_does_not_emit_warn() {
    let output_meta = vec![OutputMeta::new(
        "data".to_string(),
        MACRO_OUTPUT_HASH,
        MaxSliceLen::try_new(64 * 1024),
    )];
    let mut map = HashMap::new();
    map.insert(STUB_SCHEMA.to_string(), MACRO_OUTPUT_HASH); // identical

    let _runtime = build_single_output_with_schema_hashes(YAML_NO_TIER1, output_meta, Some(&map));

    assert!(
        !logs_contain(DIVERGENCE_WARN_PHRASE),
        "matching YAML and macro schema hashes must NOT emit the divergence warn"
    );
}

/// (c) Map absent (`None`): the back-compat path every non-CLI caller uses
/// → NO divergence warn even though the macro hash would differ from any
/// hypothetical YAML hash. Pins the purely-additive contract.
#[traced_test]
#[test]
#[serial]
fn schema_hashes_none_does_not_emit_warn() {
    let output_meta = vec![OutputMeta::new(
        "data".to_string(),
        MACRO_OUTPUT_HASH,
        MaxSliceLen::try_new(64 * 1024),
    )];

    let _runtime = build_single_output_with_schema_hashes(YAML_NO_TIER1, output_meta, None);

    assert!(
        !logs_contain(DIVERGENCE_WARN_PHRASE),
        "a None schema-hash map must NOT emit the divergence warn (back-compat)"
    );
}

/// (d) Name absent from the map: the map is `Some` but does NOT carry the
/// YAML schema name (e.g. a ROS2 package-qualified schema not defined in
/// the workspace `schemas/` dir) → NO warn. We only speak when we can
/// compare two REAL hashes.
#[traced_test]
#[test]
#[serial]
fn schema_name_absent_from_map_does_not_emit_warn() {
    let output_meta = vec![OutputMeta::new(
        "data".to_string(),
        MACRO_OUTPUT_HASH,
        MaxSliceLen::try_new(64 * 1024),
    )];
    let mut map = HashMap::new();
    // A DIFFERENT schema name — STUB_SCHEMA ("sensor_msgs/LaserScan") is
    // absent, so the lookup misses and the check is skipped.
    map.insert("other_pkg/OtherSchema".to_string(), 0xDEAD_BEEF);

    let _runtime = build_single_output_with_schema_hashes(YAML_NO_TIER1, output_meta, Some(&map));

    assert!(
        !logs_contain(DIVERGENCE_WARN_PHRASE),
        "a schema name absent from the map must NOT emit the divergence warn"
    );
}

// ---- (e) macro `schema_hash == 0` sentinel: never warn ---------------

/// (e) The closure-entry / no-type sentinel: when the macro side's
/// `OutputMeta::schema_hash == 0` (closure-based nodes carry no real
/// per-port hash), comparing against ANY workspace hash would
/// false-positive. The check is skipped even when the map carries the YAML
/// schema name with a real, DIFFERENT non-zero hash — proves the explicit
/// zero-sentinel guard, not merely "the hashes happen to match".
#[traced_test]
#[test]
#[serial]
fn macro_schema_hash_zero_sentinel_does_not_emit_warn() {
    // Macro hash 0 (the sentinel) but the map carries a real non-zero hash
    // for the SAME schema name — without the guard the check would compare
    // 0 vs 0x9999… and warn spuriously.
    let output_meta = vec![OutputMeta::new(
        "data".to_string(),
        0, // closure-entry / no-type sentinel
        MaxSliceLen::try_new(64 * 1024),
    )];
    let mut map = HashMap::new();
    map.insert(STUB_SCHEMA.to_string(), 0x9999_8888_7777_6666);

    let _runtime = build_single_output_with_schema_hashes(YAML_NO_TIER1, output_meta, Some(&map));

    assert!(
        !logs_contain(DIVERGENCE_WARN_PHRASE),
        "a macro schema_hash of 0 (the closure/no-type sentinel) must NOT emit \
         the divergence warn even against a real non-zero workspace hash"
    );
}

// ---- (f) multi-output keying: warn carries the DIVERGING port only ----

/// A second YAML schema string for the multi-output keying test. Distinct
/// from `STUB_SCHEMA` so the two outputs carry different `schema:` values,
/// and the assertions can confirm which one the warn names.
const STUB_SCHEMA_B: &str = "sensor_msgs/Image";

/// The macro hash the matching output declares for its `data_b` port. The
/// map carries `STUB_SCHEMA_B` with this SAME value, so `data_b` does NOT
/// diverge; only `data` (whose YAML hash differs from `MACRO_OUTPUT_HASH`)
/// does.
const MACRO_OUTPUT_HASH_B: u64 = 0x5555_6666_7777_8888;

/// Two-output stub YAML. The first output (`data`) carries `STUB_SCHEMA`
/// and DIVERGES from the map; the second (`data_b`) carries `STUB_SCHEMA_B`
/// and MATCHES. Distinct output names → distinct resolved topics.
const YAML_TWO_OUTPUTS: &str = r#"
name: warn_emission_multi
nodes:
  - id: pub1
    type: stub
    outputs:
      - name: data
        schema: sensor_msgs/LaserScan
      - name: data_b
        schema: sensor_msgs/Image
"#;

/// (f) Two outputs, ONE diverging: the warn must carry the DIVERGING
/// port's topic + schema and NOT the matching one's. `logs_contain` is
/// monotonic over the whole captured log, so the single-output cases above
/// cannot prove keying — a variant that warns for every output would pass
/// them but fail here (it would warn for `data_b` too, naming `STUB_SCHEMA_B` and
/// the `pub1/data_b` topic).
#[traced_test]
#[test]
#[serial]
fn multi_output_warn_names_only_the_diverging_port() {
    let yaml_hash_diverging: u64 = 0x1111_2222_3333_4444; // ≠ MACRO_OUTPUT_HASH
    let output_meta = vec![
        // `data`: macro hash MACRO_OUTPUT_HASH; map will carry a DIFFERENT
        // hash → diverges.
        OutputMeta::new(
            "data".to_string(),
            MACRO_OUTPUT_HASH,
            MaxSliceLen::try_new(64 * 1024),
        ),
        // `data_b`: macro hash MACRO_OUTPUT_HASH_B; map carries the SAME
        // value → matches (no warn for this port).
        OutputMeta::new(
            "data_b".to_string(),
            MACRO_OUTPUT_HASH_B,
            MaxSliceLen::try_new(64 * 1024),
        ),
    ];
    let mut map = HashMap::new();
    map.insert(STUB_SCHEMA.to_string(), yaml_hash_diverging); // data → diverge
    map.insert(STUB_SCHEMA_B.to_string(), MACRO_OUTPUT_HASH_B); // data_b → match

    let _runtime =
        build_single_output_with_schema_hashes(YAML_TWO_OUTPUTS, output_meta, Some(&map));

    // Inspect ONLY the divergence-warn lines (scoped — the build also emits
    // unrelated `publisher created topic=…/pub1/data_b` DEBUG lines, so a
    // raw `logs_contain` over the whole capture cannot prove keying). There
    // must be EXACTLY ONE divergence warn, naming the DIVERGING port's
    // schema/topic/yaml-hash and NOT the matching port's.
    logs_assert(|lines: &[&str]| {
        let warns: Vec<&&str> = lines
            .iter()
            .filter(|l| l.contains(DIVERGENCE_WARN_PHRASE))
            .collect();
        if warns.len() != 1 {
            return Err(format!(
                "expected exactly ONE divergence warn (the diverging `data` output), got {} \
                 — a 'warns for every output' mutant would emit two",
                warns.len()
            ));
        }
        let warn = warns[0];
        // The single warn names the DIVERGING port.
        if !warn.contains(&format!("schema={STUB_SCHEMA}")) {
            return Err(format!(
                "warn must carry the diverging schema {STUB_SCHEMA}: {warn}"
            ));
        }
        if !warn.contains("/pub1/data ") && !warn.ends_with("/pub1/data") {
            // The topic field renders as `topic=<prefix>/pub1/data` followed
            // by a space and the next field — assert the `/pub1/data`
            // segment is the diverging port (a trailing space rules out the
            // `/pub1/data_b` segment matching).
            return Err(format!(
                "warn must carry the diverging topic suffix /pub1/data: {warn}"
            ));
        }
        if !warn.contains(&format!("yaml_schema_hash={yaml_hash_diverging}")) {
            return Err(format!("warn must carry the diverging yaml hash: {warn}"));
        }
        // The single warn must NOT name the MATCHING port.
        if warn.contains(&format!("schema={STUB_SCHEMA_B}")) {
            return Err(format!(
                "warn must NOT name the matching schema {STUB_SCHEMA_B}: {warn}"
            ));
        }
        if warn.contains("/pub1/data_b") {
            return Err(format!(
                "warn must NOT name the matching topic /pub1/data_b: {warn}"
            ));
        }
        Ok(())
    });
}

// ---- (g) determinism: two builds, identical warn count + content ------

/// (g) Determinism (Principle #7): building the SAME divergent graph twice
/// yields the SAME divergence warn each time — same count, same content.
/// `tracing-test` accumulates per-test, so the second build's warn appears
/// in the same captured buffer; we assert the phrase appears EXACTLY twice
/// and both carry identical schema + hash fields.
#[traced_test]
#[test]
#[serial]
fn schema_hash_divergence_is_deterministic_across_builds() {
    let yaml_hash: u64 = 0x1111_2222_3333_4444; // ≠ MACRO_OUTPUT_HASH
    let mut map = HashMap::new();
    map.insert(STUB_SCHEMA.to_string(), yaml_hash);

    // Build #1.
    let _r1 = build_single_output_with_schema_hashes(
        YAML_NO_TIER1,
        vec![OutputMeta::new(
            "data".to_string(),
            MACRO_OUTPUT_HASH,
            MaxSliceLen::try_new(64 * 1024),
        )],
        Some(&map),
    );
    // Build #2 (independent runtime, same inputs).
    let _r2 = build_single_output_with_schema_hashes(
        YAML_NO_TIER1,
        vec![OutputMeta::new(
            "data".to_string(),
            MACRO_OUTPUT_HASH,
            MaxSliceLen::try_new(64 * 1024),
        )],
        Some(&map),
    );

    // The warn phrase appears once PER build → exactly twice in the
    // accumulated capture. `logs_assert` receives every captured line.
    logs_assert(|lines: &[&str]| {
        let phrase_hits = lines
            .iter()
            .filter(|l| l.contains(DIVERGENCE_WARN_PHRASE))
            .count();
        if phrase_hits != 2 {
            return Err(format!(
                "expected the divergence warn exactly twice (one per build), got {phrase_hits}"
            ));
        }
        // Both must carry identical schema + hash fields (deterministic
        // content, not just count).
        let field_hits = lines
            .iter()
            .filter(|l| {
                l.contains(DIVERGENCE_WARN_PHRASE)
                    && l.contains(&format!("schema={STUB_SCHEMA}"))
                    && l.contains(&format!("yaml_schema_hash={yaml_hash}"))
                    && l.contains(&format!("macro_schema_hash={MACRO_OUTPUT_HASH}"))
            })
            .count();
        if field_hits != 2 {
            return Err(format!(
                "expected both warns to carry identical schema + hash fields, got {field_hits}"
            ));
        }
        Ok(())
    });
}

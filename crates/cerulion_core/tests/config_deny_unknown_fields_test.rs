// SPDX-License-Identifier: AGPL-3.0-only
//! CI hardening: **a misspelled key in a user-authored config
//! file must be a loud parse error, never a silently dropped setting.**
//!
//! The macro side has enforced this for some time: `#[cerulion_node(periodms
//! = 16)]` is a `compile_error!` naming the closed attribute set. The YAML
//! side did not. `serde`'s default is to IGNORE an unknown key, so
//! `proces_groups:`, `dpeth: 32` or a resurrected legacy `policy:` block
//! deserialized happily while the graph ran with the intended setting absent
//! — the exact "user surface dead on arrival" class the project's
//! loud-over-silent rule exists to prevent, on the single most user-authored
//! file format in the product.
//!
//! Three gates live here, each pinning a different half of that contract:
//!
//! 1. [`every_user_authored_config_type_denies_unknown_fields`] — the source
//!    walk. Over a DECLARED inventory of config and interchange modules PLUS
//!    two module trees swept WHOLE (`cerulion_core/src/graph/` and, added in a
//!    later sweep, `cerulion_cli_engine/src/`, the crate owning the CLI's
//!    user-authored document surface and the manifests a bag carries), every
//!    `Deserialize`-deriving struct/enum must carry
//!    `#[serde(deny_unknown_fields)]` or be CLASSIFIED, with a stated reason,
//!    into one of two other states: `WarnInsteadOfDeny` (serde forbids the
//!    attribute on its shape, so its module reports unknown keys through a
//!    `warn!` instead — VERIFIED, not taken on trust: the walk finds that warn
//!    or fails, and each such classification NAMES the warn that covers it, so
//!    a module reporting at two levels — `node.rs` reports both a top-level
//!    info-JSON key and a key inside a port entry — cannot have one warn answer
//!    for the other's types) or `ExemptWithReason` (nothing hand-authors this,
//!    or it has no named fields to deny). Checked both ways: an un-denied new
//!    type fails
//!    until classified, a STALE classification (one whose type grew the
//!    attribute, or vanished) fails too, and a classification that does not
//!    match what the module and the type can actually do fails as well — a
//!    field-bearing type in a warn-reporting module cannot claim to be exempt.
//! 2. [`a_misspelled_graph_yaml_key_is_refused_naming_the_key_and_the_fix`] —
//!    the behavioural oracle. The walk proves the attribute is *present*;
//!    this proves what a user actually SEES when they typo a key.
//! 3. [`every_graph_yaml_in_the_repo_still_parses`] — the consequence
//!    check. Turning the attribute on is a hard compatibility change, so every
//!    graph-shaped YAML in the tree (fixtures, examples, demos, benches — 24
//!    files across 5 separate workspaces at the time of writing) is re-parsed
//!    under the strict types. It walks the FILESYSTEM, not `git ls-files`, so
//!    a fixture written but not yet `git add`ed is checked too.
//!
//! What arm 3 CANNOT see: graph YAML embedded in a Rust string literal.
//! MEASURED at 29 such files across the tree, and this gate sees NONE of them.
//! The blind spot is not nearly empty: it is most of the graph YAML the
//! test suite exercises.
//!
//! Of those 29, zero carry a legacy `policy:` block. A stale block in an
//! embedded document is found by a test RUNNING it, not by this gate. The
//! two-node pipeline is the workspace `examples/basic_timer/`,
//! whose graph is a real file that arm 3 DOES walk, and the in-code programs
//! are test code in `in_code_pipeline_test.rs`, which executes their embedded
//! YAML on every run.
//! That is the exact description of the coverage: an embedded document is
//! checked when some test executes it, and not otherwise.
//! Extending the walk to raw-string literals is deliberately NOT done: several
//! test files embed DELIBERATELY invalid graph YAML as their fixture (the
//! adversarial arm in `tutorial_yaml_test.rs` splices a `policy:` block back
//! in on purpose), so a literal-scanning gate would fail on exactly the tests
//! that prove the refusal works. Embedded YAML is covered by whatever test
//! embeds it.
//!
//! Why a source walk and not just the behavioural test: the behavioural test
//! can only reach the types it names. The walk is what makes a config type
//! added TOMORROW fail until somebody decides which side of the line it is on.
//!
//! # What the `cerulion_cli_engine` sweep found, and did not
//!
//! Before it, the walk named two FILES in that crate (`graph_cmd.rs`,
//! `tolerance.rs`) — which is the same mistake the gate exists to catch one
//! level up, and the tell was that two of the four declared modules already
//! lived there: the crate was in scope by intent and out of scope by
//! construction. The sweep brought in 34 further `Deserialize` types, and ALL
//! 34 classify `ExemptWithReason` in five families — remote-service response
//! bodies, the never-bricks local stores, process-handoff plans, bag manifests
//! and report JSON, and deliberately PARTIAL views of documents whose other
//! keys belong to other readers.
//!
//! Zero of them are documents a human types, so nothing was weakened to make
//! the sweep pass — but the ratio is the real headline: the gate's value here
//! is that the 35th fails until somebody decides, not that it found strictness
//! debt today.
//!
//! One REAL gap surfaced and is recorded rather than papered over:
//! `node_metadata.rs`'s unknown-key walk scans the info-JSON ENVELOPE
//! only, so an unknown key inside a port entry is reported by the RUNTIME
//! reader (`graph/node.rs`) and by nothing in the CLI's own reader. The
//! classification for `RawInput` says so.

use std::collections::BTreeSet;
use std::fs;
use std::path::{Path, PathBuf};

/// Shared with `config_deny_unknown_fields_test.rs`: ONE literal-aware
/// comment stripper, so the two walks cannot drift apart again.
mod common;
use common::code_only;

/// The repo root — this crate's manifest dir with the crate name popped.
fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("cerulion_core lives one level under the repo root")
        .to_path_buf()
}

// ─────────────────────────────────────────────────────────────────────────
// The declared inventories
// ─────────────────────────────────────────────────────────────────────────

/// One module this gate walks.
#[derive(Clone)]
struct WalkedModule {
    /// Repo-relative path.
    path: &'static str,
    /// WHY it is on the list — which document these types parse.
    why: &'static str,
    /// `Some(field)` declares: this is an INTERCHANGE format whose types
    /// cannot carry `deny_unknown_fields`, and which therefore reports an
    /// unknown key through a `tracing::warn!` carrying the structured field
    /// named here.
    ///
    /// The gate VERIFIES that warn exists (see
    /// [`warn_carries_field`]) rather than taking the declaration on trust,
    /// and only a module carrying one may classify a type
    /// [`Classification::WarnInsteadOfDeny`]. So a module that declares a
    /// warn and loses it FAILS, and a module that declares none cannot use
    /// the classification at all.
    unknown_key_warn_field: Option<&'static str>,
    /// `false` for a module the walk DISCOVERED rather than one a human
    /// listed — a discovered module may legitimately hold no config type.
    declared: bool,
}

/// The config / interchange modules this gate walks, each with WHY.
///
/// Two FAMILIES, and the distinction is recorded rather than collapsed:
///
/// * **user-authored** — a human types the file, so an unknown key is a typo
///   and the whole point is a HARD parse error.
/// * **machine-emitted interchange** (`unknown_key_warn_field: Some(..)`) — the
///   cdylib info JSON, written by `cerulion_macros::codegen::gen_cdylib` and
///   read by `DylibNodeEntry`. Leniency there is a FORWARD-COMPAT requirement
///   (a newer cdylib on an older host must still load), so `deny` is the wrong
///   answer and SILENCE is the defect. What this gate holds is that an unknown
///   key is REPORTED. Key PARITY between the emitter and the parser is a
///   different property with its own gate — `info_json_key_parity_test.rs`.
///
/// Deliberately NOT on the list, and why:
///
/// * **schema YAML** (`schemas/*.yaml`, read by `cerulion_cli_engine`'s
///   `schema_cmd`) — parsed as a `serde_yaml::Value` and hand-walked, so there
///   is no `Deserialize` derive for the attribute to sit on. It needs its own
///   unknown-key gate, which it does not have; a `deny_unknown_fields` walk over
///   it would silently find nothing and read as coverage.
/// * **`cerulion_cli_engine::auth`, `peers.json`, `robots.toml`** — the
///   never-bricks stores (see the classification inventory below).
const WALKED_CONFIG_MODULES: &[WalkedModule] = &[
    WalkedModule {
        path: "cerulion_core/src/graph/config.rs",
        why: "graph YAML (`graphs/*.yaml`) — the graph is the source of truth (Principle #5)",
        unknown_key_warn_field: None,
        declared: true,
    },
    WalkedModule {
        path: "cerulion_cli_engine/src/graph_cmd.rs",
        why: "the cost-snapshot artifact (`graphs/<name>.costs.yaml`), documented as user-editable",
        unknown_key_warn_field: None,
        declared: true,
    },
    WalkedModule {
        path: "cerulion_cli_engine/src/tolerance.rs",
        why: "the replay tolerance document (`cerulion replay --tolerance <file>`)",
        unknown_key_warn_field: None,
        declared: true,
    },
    WalkedModule {
        path: "cerulion_core/src/graph/node.rs",
        why: "the cdylib info JSON — machine-emitted interchange across the FFI, read by \
              `DylibNodeEntry::parse_info_json_labeled`",
        unknown_key_warn_field: Some("unknown_key"),
        declared: true,
    },
];

/// Which `tracing::warn!` in the module covers ONE classified type.
///
/// A module may report unknown keys at more than one LEVEL — `node.rs` has
/// two such warns, one for the info-JSON envelope and one for a port entry —
/// and a per-module check cannot tell them apart, so deleting either would
/// leave the other answering for both. This is the discriminator: the covering
/// warn is identified by the structured fields its argument list does and does
/// not carry, so each classification stands or falls on its OWN warn.
///
/// Fields, never message text: `code_only` strips comments but deliberately
/// does not model string literals, so a warn's own prose must never be able to
/// satisfy a structural claim (see [`warn_span_matches`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct WarnScope {
    /// Structured fields the covering warn MUST assign (`field = ...`).
    requires: &'static [&'static str],
    /// Structured fields it must NOT assign — what separates this warn from
    /// its siblings in the same module.
    forbids: &'static [&'static str],
    /// Prose for the failure message: which scan this is, in the user's terms.
    what: &'static str,
}

/// Every module the walk covers: the DECLARED inventory, plus every `.rs`
/// under `cerulion_core/src/graph/` discovered on disk.
///
/// The declared list alone was a four-FILE allow-list, which is the same
/// mistake the gate exists to catch one level up: a graph-YAML type added in
/// a NEW file — or a sub-block type that lives in a sibling module —
/// deserializes user-authored YAML while being invisible to the walk that is
/// supposed to govern it. `graph/` is where `GraphConfig`'s field types live,
/// so it is swept WHOLE and a new file is covered the moment it exists.
///
/// A discovered module carries no `unknown_key_warn_field`: warn-instead-of-
/// deny is a claim only a DECLARED entry may make, because the claim has to
/// be checked against a specific warn.
fn walked_modules(root: &Path) -> Vec<WalkedModule> {
    let mut out: Vec<WalkedModule> = WALKED_CONFIG_MODULES.to_vec();
    let declared: BTreeSet<&str> = WALKED_CONFIG_MODULES.iter().map(|m| m.path).collect();

    for (dir, why) in SWEPT_DIRS {
        let mut found: Vec<PathBuf> = Vec::new();
        collect_rs(&root.join(dir), &mut found);
        found.sort();
        // FAIL CLOSED. `collect_rs` cannot propagate a `read_dir` failure — a
        // missing, renamed or unreadable tree yields an EMPTY list — and an
        // empty sweep is indistinguishable from a clean one: the walk would run
        // over the declared modules alone and PASS while covering none of this
        // tree. A swept root that contributes nothing is a broken gate, not a
        // clean one, and the count is the only signal available here.
        assert!(
            !found.is_empty(),
            "swept directory `{dir}` contributed NO modules — it was renamed, moved or is \
             unreadable, and this gate would silently stop covering it. Fix the path in \
             SWEPT_DIRS (this test is in cerulion_core/tests/config_deny_unknown_fields_test.rs)."
        );
        for abs in found {
            let Ok(rel) = abs.strip_prefix(root) else {
                continue;
            };
            let rel = rel.to_string_lossy().replace('\\', "/");
            if declared.contains(rel.as_str()) || out.iter().any(|m| m.path == rel) {
                continue;
            }
            out.push(WalkedModule {
                path: Box::leak(rel.into_boxed_str()),
                why,
                unknown_key_warn_field: None,
                declared: false,
            });
        }
    }
    out
}

/// The module trees swept WHOLE, each with WHY.
///
/// A swept directory is what makes a config type added TOMORROW fail until
/// somebody classifies it. A hand list of FILES is the same mistake this gate
/// exists to catch one level up — a new type in a new file deserializes a
/// user-authored document while being invisible to the walk that governs it.
const SWEPT_DIRS: &[(&str, &str)] = &[
    (
        "cerulion_core/src/graph",
        "discovered under cerulion_core/src/graph/ — the module tree GraphConfig's field types \
         resolve in",
    ),
    // Before this sweep, the walk named two `cerulion_cli_engine` FILES
    // (`graph_cmd.rs`, `tolerance.rs`) and was therefore blind to every
    // `Deserialize` type in the other ~60 modules of the crate that owns the
    // CLI's whole document surface — the tolerance document, the cost snapshot,
    // the peer/robot stores, the bag manifests a bag carries and `bag info`
    // renders. Two of the four DECLARED modules already lived here, which is
    // the tell: the crate was in scope by intent and out of scope by
    // construction. Swept whole, on the same rule as `graph/`.
    (
        "cerulion_cli_engine/src",
        "discovered under cerulion_cli_engine/src/ — the crate owning the CLI's user-authored \
         document surface and the manifests a bag carries",
    ),
];

/// Every `.rs` under `dir` (recursive), skipping build output and any nested
/// checkout.
///
/// A `read_dir` failure on a NESTED directory is reported LOUDLY rather than
/// skipped: the walk's whole value is that a type it cannot see fails the gate,
/// so a subtree that silently contributes nothing is the one outcome that must
/// never look like coverage. The ROOT's own readability is enforced by its
/// caller's non-empty assertion, which also catches a rename.
fn collect_rs(dir: &Path, out: &mut Vec<PathBuf>) {
    let entries = match fs::read_dir(dir) {
        Ok(e) => e,
        Err(e) => panic!(
            "cannot read `{}` while sweeping for config types ({e}) — a directory this gate \
             cannot walk is a directory it cannot govern, so this is a failure rather than a \
             skip",
            dir.display()
        ),
    };
    for entry in entries.flatten() {
        let path = entry.path();
        let name = entry.file_name();
        let name = name.to_string_lossy();
        if path.is_dir() {
            if name == "target" || name == ".git" || common::is_other_checkout(&path) {
                continue;
            }
            collect_rs(&path, out);
        } else if name.ends_with(".rs") {
            out.push(path);
        }
    }
}

/// How a `Deserialize`-deriving type that does NOT carry
/// `#[serde(deny_unknown_fields)]` is classified.
///
/// Three states in total: DENY is the default and needs no entry; the other
/// two are declared below, each with a reason, and each CHECKED rather than
/// taken on trust.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Classification {
    /// An unknown key is silently ignored, and that is CORRECT here — the type
    /// is not a document anybody hand-authors, or has no named fields for the
    /// attribute to govern.
    ExemptWithReason,
    /// The type CANNOT deny (serde forbids the attribute on its shape), so its
    /// module reports an unknown key through a `warn!` instead. Legal only in
    /// a module declaring `unknown_key_warn_field`, and only for a type that
    /// HAS named fields — a fieldless one has nothing to report. The `scope`
    /// names WHICH warn does the reporting, so the claim is verified for this
    /// type rather than for the module as a whole.
    WarnInsteadOfDeny { scope: WarnScope },
}

/// The info-JSON ENVELOPE scan: the warn that fires for a top-level key, which
/// belongs to no port and therefore carries no `port` field. That absence is
/// the discriminator — it is what the port-entry warn below cannot satisfy.
const ENVELOPE_SCAN: WarnScope = WarnScope {
    requires: &["unknown_key"],
    forbids: &["port"],
    what: "the ENVELOPE scan — the warn that reports a TOP-LEVEL info-JSON key. It carries \
           no `port` field, because a top-level key belongs to no port",
};

/// The PORT-ENTRY scan: the warn that fires for an unknown key inside an
/// `inputs`/`outputs` entry, and names the port it was found on.
const PORT_ENTRY_SCAN: WarnScope = WarnScope {
    requires: &["unknown_key", "port"],
    forbids: &[],
    what: "the PORT-ENTRY scan — the warn that reports an unknown key inside an \
           `inputs`/`outputs` entry and names the `port` it sat on",
};

/// Every `Deserialize`-deriving type inside [`WALKED_CONFIG_MODULES`] that
/// does NOT deny unknown fields, with its classification and reason.
///
/// Checked BOTH ways: an entry naming a type that now denies (or no longer
/// exists) fails as STALE, so this list cannot quietly become a blanket
/// pre-authorisation for future drift.
const UNDENIED_TYPES: &[(&str, &str, Classification, &str)] = &[
    (
        "cerulion_cli_engine/src/graph_cmd.rs",
        "GatewayZenohMode",
        Classification::ExemptWithReason,
        "not user-authored: a serde mirror of `cerulion_core::ZenohMode` on the \
         CLI→gateway-child process handoff (`graph run-gateway --handoff`), written \
         and read by the same build",
    ),
    (
        "cerulion_cli_engine/src/graph_cmd.rs",
        "GatewayNetworkParams",
        Classification::ExemptWithReason,
        "not user-authored: the `NetworkConfig` half of the same process handoff",
    ),
    (
        "cerulion_cli_engine/src/graph_cmd.rs",
        "GatewayPostureCarrier",
        Classification::ExemptWithReason,
        "not user-authored: the `NetworkPosture` half of the same process handoff",
    ),
    (
        "cerulion_cli_engine/src/graph_cmd.rs",
        "GatewayHandoff",
        Classification::ExemptWithReason,
        "not user-authored: the process-handoff envelope itself",
    ),
    (
        "cerulion_cli_engine/src/graph_cmd.rs",
        "VersionProbe",
        Classification::ExemptWithReason,
        "DELIBERATELY lenient: the `parse_profile_artifact` pre-flight reads ONLY \
         `version` out of a possibly-newer artifact so a too-new file gets the \
         actionable version message instead of a raw unknown-field error from the \
         strict `ProfileArtifact` parse that follows",
    ),
    (
        "cerulion_core/src/graph/node.rs",
        "NodeInfoJson",
        Classification::WarnInsteadOfDeny {
            scope: ENVELOPE_SCAN,
        },
        "the cdylib info-JSON envelope. Every field of this struct is \
         `#[serde(default)] Option<..>`, so a misspelled TOP-LEVEL key took its \
         default in silence and the node ran with the intended setting absent; \
         the module now re-walks the raw JSON post-parse and warns \
         once per unknown envelope key. It stays a WARN rather than a deny \
         because this JSON crosses the cdylib ABI: a NEWER cdylib emitting a key \
         an OLDER host has not learned must still load, which is what the \
         `#[serde(default)]` on every field promises",
    ),
    (
        "cerulion_core/src/graph/node.rs",
        "InputJson",
        Classification::WarnInsteadOfDeny {
            scope: PORT_ENTRY_SCAN,
        },
        "`#[serde(untagged)]`, which CANNOT carry `deny_unknown_fields` (an \
         unknown key makes the `Full` variant unmatchable and fails the WHOLE \
         entry as `no variant matched` — worse than ignoring), so its unknown \
         keys are reported by the module's `LEGAL_INPUT_KEYS` walk",
    ),
    (
        "cerulion_core/src/graph/node.rs",
        "OutputJson",
        Classification::WarnInsteadOfDeny {
            scope: PORT_ENTRY_SCAN,
        },
        "`#[serde(untagged)]` — cannot deny (see `InputJson`); its unknown keys \
         are reported against `LEGAL_OUTPUT_KEYS` by the same walk",
    ),
    (
        "cerulion_core/src/graph/node.rs",
        "BackpressureJson",
        Classification::ExemptWithReason,
        "NO NAMED FIELDS to deny: a unit/newtype-variant enum, so serde's \
         variant matcher already rejects an unknown VALUE loudly — the same \
         shape as `NetworkMode`. Verified structurally, not taken on trust: \
         the gate refuses this classification for a field-bearing type in a \
         warn-reporting module",
    ),

    // ── The deferred `cerulion_cli_engine/src/` sweep ───────────────────
    //
    // Five families, and every one of them is machine-written or
    // machine-served rather than hand-authored — which is why they are
    // exempt and NOT a weakening: the gate exists for the file a human types.
    (
        "cerulion_cli_engine/src/account_cmd.rs",
        "DeviceEntry",
        Classification::ExemptWithReason,
        "a REMOTE SERVICE's response body, not a document anybody authors. Denying an unknown key here is a self-inflicted outage: the account service adding a field would break every CLI already in the field, and the CLI reads only the keys it names. Forward-compat is the requirement, silence the correct behaviour",
    ),
    (
        "cerulion_cli_engine/src/account_cmd.rs",
        "ListDevicesResponse",
        Classification::ExemptWithReason,
        "a REMOTE SERVICE's response body, not a document anybody authors. Denying an unknown key here is a self-inflicted outage: the account service adding a field would break every CLI already in the field, and the CLI reads only the keys it names. Forward-compat is the requirement, silence the correct behaviour",
    ),
    (
        "cerulion_cli_engine/src/account_cmd.rs",
        "RevokeDeviceResult",
        Classification::ExemptWithReason,
        "a REMOTE SERVICE's response body, not a document anybody authors. Denying an unknown key here is a self-inflicted outage: the account service adding a field would break every CLI already in the field, and the CLI reads only the keys it names. Forward-compat is the requirement, silence the correct behaviour",
    ),
    (
        "cerulion_cli_engine/src/account_cmd.rs",
        "RobotAccess",
        Classification::ExemptWithReason,
        "a REMOTE SERVICE's response body, not a document anybody authors. Denying an unknown key here is a self-inflicted outage: the account service adding a field would break every CLI already in the field, and the CLI reads only the keys it names. Forward-compat is the requirement, silence the correct behaviour",
    ),
    (
        "cerulion_cli_engine/src/account_cmd.rs",
        "ErrorBody",
        Classification::ExemptWithReason,
        "a REMOTE SERVICE's response body, not a document anybody authors. Denying an unknown key here is a self-inflicted outage: the account service adding a field would break every CLI already in the field, and the CLI reads only the keys it names. Forward-compat is the requirement, silence the correct behaviour",
    ),
    (
        "cerulion_cli_engine/src/login_cmd.rs",
        "DeviceStart",
        Classification::ExemptWithReason,
        "a REMOTE SERVICE's response body, not a document anybody authors. Denying an unknown key here is a self-inflicted outage: the account service adding a field would break every CLI already in the field, and the CLI reads only the keys it names. Forward-compat is the requirement, silence the correct behaviour",
    ),
    (
        "cerulion_cli_engine/src/login_cmd.rs",
        "TokenResponse",
        Classification::ExemptWithReason,
        "a REMOTE SERVICE's response body, not a document anybody authors. Denying an unknown key here is a self-inflicted outage: the account service adding a field would break every CLI already in the field, and the CLI reads only the keys it names. Forward-compat is the requirement, silence the correct behaviour",
    ),
    (
        "cerulion_cli_engine/src/login_cmd.rs",
        "ErrorBody",
        Classification::ExemptWithReason,
        "a REMOTE SERVICE's response body, not a document anybody authors. Denying an unknown key here is a self-inflicted outage: the account service adding a field would break every CLI already in the field, and the CLI reads only the keys it names. Forward-compat is the requirement, silence the correct behaviour",
    ),
    (
        "cerulion_cli_engine/src/login_cmd.rs",
        "RegisterDeviceResponse",
        Classification::ExemptWithReason,
        "a REMOTE SERVICE's response body, not a document anybody authors. Denying an unknown key here is a self-inflicted outage: the account service adding a field would break every CLI already in the field, and the CLI reads only the keys it names. Forward-compat is the requirement, silence the correct behaviour",
    ),
    (
        "cerulion_cli_engine/src/login_cmd.rs",
        "ChallengeResponse",
        Classification::ExemptWithReason,
        "a REMOTE SERVICE's response body, not a document anybody authors. Denying an unknown key here is a self-inflicted outage: the account service adding a field would break every CLI already in the field, and the CLI reads only the keys it names. Forward-compat is the requirement, silence the correct behaviour",
    ),
    (
        "cerulion_cli_engine/src/login_cmd.rs",
        "Me",
        Classification::ExemptWithReason,
        "a REMOTE SERVICE's response body, not a document anybody authors. Denying an unknown key here is a self-inflicted outage: the account service adding a field would break every CLI already in the field, and the CLI reads only the keys it names. Forward-compat is the requirement, silence the correct behaviour",
    ),
    (
        "cerulion_cli_engine/src/robot_cmd.rs",
        "RegisterRobotResponse",
        Classification::ExemptWithReason,
        "a REMOTE SERVICE's response body, not a document anybody authors. Denying an unknown key here is a self-inflicted outage: the account service adding a field would break every CLI already in the field, and the CLI reads only the keys it names. Forward-compat is the requirement, silence the correct behaviour",
    ),
    (
        "cerulion_cli_engine/src/robot_cmd.rs",
        "ErrorBody",
        Classification::ExemptWithReason,
        "a REMOTE SERVICE's response body, not a document anybody authors. Denying an unknown key here is a self-inflicted outage: the account service adding a field would break every CLI already in the field, and the CLI reads only the keys it names. Forward-compat is the requirement, silence the correct behaviour",
    ),
    (
        "cerulion_cli_engine/src/auth.rs",
        "MachineRole",
        Classification::ExemptWithReason,
        "NO NAMED FIELDS to deny: a unit/newtype-variant enum, so serde's variant matcher already rejects an unknown VALUE loudly",
    ),
    (
        "cerulion_cli_engine/src/auth.rs",
        "AuthState",
        Classification::ExemptWithReason,
        "a NEVER-BRICKS local store this build writes and reads (the header's own carve-out). A record written by a NEWER CLI must not stop an older one from connecting, and the reader takes only the keys it names — this is the credential store itself, where a refusal means the operator cannot log in at all",
    ),
    (
        "cerulion_cli_engine/src/connect_cmd.rs",
        "RobotsDoc",
        Classification::ExemptWithReason,
        "a NEVER-BRICKS local store this build writes and reads (the header's own carve-out). A record written by a NEWER CLI must not stop an older one from connecting, and the reader takes only the keys it names (`~/.cerulion/robots.toml`, written by `cerulion pair`)",
    ),
    (
        "cerulion_cli_engine/src/hostname_peers.rs",
        "ConfigPeersDoc",
        Classification::ExemptWithReason,
        "a deliberately PARTIAL view: it reads ONE key (`peers`) out of `~/.cerulion/config.toml`, a shared document whose other keys belong to other readers. Denying would make this reader refuse every config file that carries anything else — the opposite of the contract",
    ),
    (
        "cerulion_cli_engine/src/multiprocess.rs",
        "WedgePagePlan",
        Classification::ExemptWithReason,
        "not user-authored: a process-handoff plan, serialized by the supervisor and read by the worker it just spawned — same build, same run, never edited in between",
    ),
    (
        "cerulion_cli_engine/src/multiprocess.rs",
        "WorkerPlan",
        Classification::ExemptWithReason,
        "not user-authored: a process-handoff plan, serialized by the supervisor and read by the worker it just spawned — same build, same run, never edited in between",
    ),
    (
        "cerulion_cli_engine/src/multiprocess.rs",
        "DeploymentPlan",
        Classification::ExemptWithReason,
        "not user-authored: a process-handoff plan, serialized by the supervisor and read by the worker it just spawned — same build, same run, never edited in between",
    ),
    (
        "cerulion_cli_engine/src/multiprocess.rs",
        "ExecutionMode",
        Classification::ExemptWithReason,
        "a unit-variant enum with NO named fields (`lockstep` / `free_run`), so there is no key to drop: it deserializes from its snake_case NAME, an unknown VALUE is refused by serde (pinned by `the_worker_plans_execution_mode_is_additive_in_both_directions`), and it rides only the `WorkerPlan` process handoff above and the bag's `coordination` stamp",
    ),
    (
        "cerulion_cli_engine/src/node_metadata.rs",
        "RawBackpressure",
        Classification::ExemptWithReason,
        "NO NAMED FIELDS to deny: a unit/newtype-variant enum, so serde's variant matcher already rejects an unknown VALUE loudly",
    ),
    (
        "cerulion_cli_engine/src/node_metadata.rs",
        "RawInput",
        Classification::ExemptWithReason,
        "`#[serde(untagged)]`, which CANNOT carry `deny_unknown_fields` (an unknown key makes the `Full` variant unmatchable and fails the WHOLE entry). NOT classified `WarnInsteadOfDeny`, and the reason is a real gap rather than a formality: this module's unknown-key walk scans the ENVELOPE only (`section = \"<top-level>\"`), so an unknown key INSIDE a port entry is reported by `graph/node.rs`'s PORT-ENTRY scan on the runtime path and by nothing here. This reader feeds `node info`/`node list` display; the runtime reader is the authoritative one",
    ),
    (
        "cerulion_cli_engine/src/node_metadata.rs",
        "InfoJson",
        Classification::ExemptWithReason,
        "the cdylib info-JSON envelope again — machine-emitted across the cdylib ABI, where a NEWER cdylib's key must not stop an OLDER host loading it, so deny is the wrong answer. This module DOES report an unknown envelope key (under the same `unknown_key=` field `graph/node.rs` uses), but the classification is not `WarnInsteadOfDeny`: that state is legal only in a module DECLARING a warn, and declaring one here would also claim a covering warn for `RawInput`, which has none",
    ),
    (
        "cerulion_cli_engine/src/bag_migrate.rs",
        "StrippedKey",
        Classification::ExemptWithReason,
        "not user-authored: a manifest/report this build WRITES into a bag (or beside one) and reads back. Leniency is deliberate and already load-bearing elsewhere — `read_record_coverage` WARNS and READS a manifest from a newer recorder rather than refusing — so denying here would make an old build refuse a new bag (`__cerulion/migration.json`)",
    ),
    (
        "cerulion_cli_engine/src/bag_migrate.rs",
        "MigrationRecord",
        Classification::ExemptWithReason,
        "not user-authored: a manifest/report this build WRITES into a bag (or beside one) and reads back. Leniency is deliberate and already load-bearing elsewhere — `read_record_coverage` WARNS and READS a manifest from a newer recorder rather than refusing — so denying here would make an old build refuse a new bag (`__cerulion/migration.json`). Read back by this verb's own already-migrated refusal, which reports an unparseable record as unparseable rather than failing on it",
    ),
    (
        "cerulion_cli_engine/src/replay_cmd.rs",
        "TraceManifest",
        Classification::ExemptWithReason,
        "not user-authored: a manifest/report this build WRITES into a bag (or beside one) and reads back. Leniency is deliberate and already load-bearing elsewhere — `read_record_coverage` WARNS and READS a manifest from a newer recorder rather than refusing — so denying here would make an old build refuse a new bag",
    ),
    (
        "cerulion_cli_engine/src/replay_engine.rs",
        "RecorderInfo",
        Classification::ExemptWithReason,
        "not user-authored: a manifest/report this build WRITES into a bag (or beside one) and reads back. Leniency is deliberate and already load-bearing elsewhere — `read_record_coverage` WARNS and READS a manifest from a newer recorder rather than refusing — so denying here would make an old build refuse a new bag (`__cerulion/recorder.json`)",
    ),
    (
        "cerulion_cli_engine/src/replay_engine.rs",
        "CoordinationMode",
        Classification::ExemptWithReason,
        "NO NAMED FIELDS to deny: a unit/newtype-variant enum, so serde's variant matcher already rejects an unknown VALUE loudly",
    ),
    (
        "cerulion_cli_engine/src/replay_engine.rs",
        "CoveredRangeReport",
        Classification::ExemptWithReason,
        "not user-authored: a manifest/report this build WRITES into a bag (or beside one) and reads back. Leniency is deliberate and already load-bearing elsewhere — `read_record_coverage` WARNS and READS a manifest from a newer recorder rather than refusing — so denying here would make an old build refuse a new bag (the `--report` JSON)",
    ),
    (
        "cerulion_cli_engine/src/replay_engine.rs",
        "ResumeReport",
        Classification::ExemptWithReason,
        "not user-authored: a manifest/report this build WRITES into a bag (or beside one) and reads back. Leniency is deliberate and already load-bearing elsewhere — `read_record_coverage` WARNS and READS a manifest from a newer recorder rather than refusing — so denying here would make an old build refuse a new bag (the `--report` JSON)",
    ),
    (
        "cerulion_cli_engine/src/replay_engine.rs",
        "ResumeSeedReport",
        Classification::ExemptWithReason,
        "not user-authored: a manifest/report this build WRITES into a bag (or beside one) and reads back. Leniency is deliberate and already load-bearing elsewhere — `read_record_coverage` WARNS and READS a manifest from a newer recorder rather than refusing — so denying here would make an old build refuse a new bag (the `--report` JSON)",
    ),
    (
        "cerulion_cli_engine/src/schema_serve.rs",
        "BridgeYamlLite",
        Classification::ExemptWithReason,
        "a deliberately PARTIAL view of the bridge config YAML — the two fields the catalog fold needs, out of a document that also carries `domain_id`/`only_networks`/`qos`/`route`/`max_slice_len`. The module's own doc states the leniency as the design: denying would make this reader reject every real bridge config. The keys it DOES need are non-`Option`, so a mapping missing one still fails the parse loudly",
    ),
    (
        "cerulion_cli_engine/src/schema_serve.rs",
        "BridgeMappingLite",
        Classification::ExemptWithReason,
        "the mapping half of the same partial view — extra keys are ignored by design, while a mapping missing `cerulion_topic`/`ros_type` still fails the parse",
    ),
    (
        "cerulion_cli_engine/src/ros2_migrate.rs",
        "CompileCommandEntry",
        Classification::ExemptWithReason,
        "a FOREIGN schema: `compile_commands.json` is owned by CMake/clang, and a real entry legitimately carries keys this reader ignores (`output`, `arguments`, `command`). Denying an unknown key would refuse every genuine compile database, and this reader takes only the `directory`/`file` it names. The `ToolOutput`/`ToolRewrite`/`ToolEdit`/`ToolCandidate` types in the SAME module DO deny — those parse the Cerulion migration engine's OWN output, deployed from the same checkout, where an unknown key means a stale/garbled edit and must fail loudly",
    ),
];

// ─────────────────────────────────────────────────────────────────────────
// Comment stripper (house pattern — oracle-tested below)
// ─────────────────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────────────────
// The walk
// ─────────────────────────────────────────────────────────────────────────

/// One `Deserialize`-deriving item found by the walk.
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord)]
struct DeserializeItem {
    /// Repo-relative path of the module it was declared in.
    file: String,
    /// The type name.
    name: String,
    /// 1-based line of the item declaration (for the failure message).
    line: usize,
    /// Whether its attribute block carries `deny_unknown_fields`.
    denies: bool,
    /// Whether its body declares any NAMED field (`ident: Type`), at any
    /// depth — so a struct-variant enum like `InputJson::Full { name, .. }`
    /// counts, while a unit/newtype-variant enum does not.
    ///
    /// This is what turns [`Classification`] from a LABEL into a CHECKED
    /// claim: a field-bearing type in a warn-reporting module is exactly the
    /// shape whose unknown keys can be silently dropped, so `ExemptWithReason`
    /// is refused for it.
    has_named_fields: bool,
}

/// Net `[` … `]` depth of a source fragment.
fn bracket_balance(s: &str) -> i32 {
    s.chars().fold(0i32, |acc, c| match c {
        '[' => acc + 1,
        ']' => acc - 1,
        _ => acc,
    })
}

/// Collect every `struct`/`enum` in `src` whose contiguous attribute block
/// derives `Deserialize`, recording whether that same block also carries
/// `deny_unknown_fields`.
///
/// Attributes accumulate across lines (a multi-line `#[derive(…)]` is joined
/// by bracket balance) and blank lines do NOT break the block — `code_only`
/// turns every `///` doc line into a blank one, so a doc-commented type would
/// otherwise lose its attributes and vanish from the walk entirely.
fn deserialize_items(file: &str, src: &str) -> Vec<DeserializeItem> {
    let stripped = code_only(src);
    let lines: Vec<&str> = stripped.lines().collect();
    let mut found = Vec::new();
    let mut pending = String::new();
    let mut i = 0usize;
    while i < lines.len() {
        let trimmed = lines[i].trim();
        if trimmed.starts_with("#[") || trimmed.starts_with("#![") {
            let mut buf = trimmed.to_string();
            while bracket_balance(&buf) > 0 && i + 1 < lines.len() {
                i += 1;
                buf.push(' ');
                buf.push_str(lines[i].trim());
            }
            // The attribute may be followed ON THE SAME LINE by the item it
            // governs (`#[derive(Deserialize)] struct S { .. }`). Splitting the
            // buffer where the brackets close is what keeps such a type
            // VISIBLE: consuming the whole line as an attribute and `continue`
            // -ing meant `item_name` never saw it, so a same-line config type
            // could omit `deny_unknown_fields` and pass this gate.
            let (attrs, rest) = split_after_attributes(&buf);
            pending.push(' ');
            pending.push_str(attrs);
            let rest = rest.trim();
            if rest.is_empty() {
                i += 1;
                continue;
            }
            if let Some(name) = item_name(rest) {
                if pending.contains("Deserialize") {
                    found.push(DeserializeItem {
                        file: file.to_string(),
                        name,
                        line: i + 1,
                        denies: pending.contains("deny_unknown_fields"),
                        has_named_fields: body_has_named_fields(&lines, i),
                    });
                }
            }
            pending.clear();
            i += 1;
            continue;
        }
        if trimmed.is_empty() {
            i += 1;
            continue;
        }
        if let Some(name) = item_name(trimmed) {
            if pending.contains("Deserialize") {
                found.push(DeserializeItem {
                    file: file.to_string(),
                    name,
                    line: i + 1,
                    denies: pending.contains("deny_unknown_fields"),
                    has_named_fields: body_has_named_fields(&lines, i),
                });
            }
        }
        pending.clear();
        i += 1;
    }
    found
}

/// Split a line that begins with attributes into `(attributes, remainder)` at
/// the point the bracket nesting first returns to zero.
///
/// `#[derive(Deserialize)] struct S {` -> `("#[derive(Deserialize)]", " struct S {")`.
/// A line that is attributes only yields an empty remainder.
fn split_after_attributes(buf: &str) -> (&str, &str) {
    let mut depth = 0i32;
    for (idx, ch) in buf.char_indices() {
        match ch {
            '[' => depth += 1,
            ']' => {
                depth -= 1;
                if depth == 0 {
                    let after = idx + ch.len_utf8();
                    // Another attribute may follow immediately; keep consuming.
                    if buf[after..].trim_start().starts_with("#[") {
                        continue;
                    }
                    return (&buf[..after], &buf[after..]);
                }
            }
            _ => {}
        }
    }
    (buf, "")
}

#[test]
fn the_attribute_splitter_finds_an_item_sharing_the_attribute_s_line() {
    assert_eq!(
        split_after_attributes("#[derive(Deserialize)] struct S {"),
        ("#[derive(Deserialize)]", " struct S {")
    );
    assert_eq!(
        split_after_attributes("#[serde(deny_unknown_fields)]"),
        ("#[serde(deny_unknown_fields)]", "")
    );
    // Two attributes then the item: BOTH must stay on the attribute side, or
    // the `deny_unknown_fields` half would be dropped and the type misread as
    // un-denied.
    assert_eq!(
        split_after_attributes("#[derive(Deserialize)] #[serde(deny_unknown_fields)] enum E {"),
        (
            "#[derive(Deserialize)] #[serde(deny_unknown_fields)]",
            " enum E {"
        )
    );
}

#[test]
fn a_type_declared_on_its_attribute_s_line_is_still_seen() {
    // The gap this closes: a scanner that swallows the whole line as an
    // attribute lets this type reach no assertion at all.
    let src = "#[derive(Deserialize)] struct SameLine { a: u8 }\n\
               #[derive(Deserialize)] #[serde(deny_unknown_fields)] struct SameLineDenies { b: u8 }\n";
    let items = deserialize_items("x.rs", src);
    let names: Vec<&str> = items.iter().map(|i| i.name.as_str()).collect();
    assert_eq!(names, vec!["SameLine", "SameLineDenies"]);
    assert!(
        !items[0].denies,
        "the un-denied same-line type must be caught"
    );
    assert!(
        items[1].denies,
        "its denying sibling must be read correctly"
    );
}

/// The type name if `line` opens a `struct` or `enum` declaration (with or
/// without a visibility qualifier), else `None`.
fn item_name(line: &str) -> Option<String> {
    let mut rest = line;
    for vis in ["pub(crate) ", "pub(super) ", "pub(self) ", "pub "] {
        if let Some(r) = rest.strip_prefix(vis) {
            rest = r.trim_start();
            break;
        }
    }
    for kw in ["struct ", "enum "] {
        if let Some(r) = rest.strip_prefix(kw) {
            let name: String = r
                .trim_start()
                .chars()
                .take_while(|c| c.is_alphanumeric() || *c == '_')
                .collect();
            if !name.is_empty() {
                return Some(name);
            }
        }
    }
    None
}

#[test]
fn the_item_name_reader_reads_declarations_and_nothing_else() {
    assert_eq!(
        item_name("pub struct GraphConfig {"),
        Some("GraphConfig".into())
    );
    assert_eq!(item_name("enum NetworkMode {"), Some("NetworkMode".into()));
    assert_eq!(
        item_name("pub(crate) struct Foo<T> {"),
        Some("Foo".to_string())
    );
    // Not declarations.
    assert_eq!(item_name("let s = struct_like;"), None);
    assert_eq!(item_name("impl GraphConfig {"), None);
    assert_eq!(item_name("fn structure() {}"), None);
}

/// Does the item declared at `decl_line` have any NAMED field in its body?
///
/// Brace-matched from the declaration, so a struct-variant enum counts and a
/// tuple/unit struct (which reaches a `;` before any `{`) does not.
fn body_has_named_fields(lines: &[&str], decl_line: usize) -> bool {
    // A SAME-LINE declaration carries its body on the declaration line
    // (`struct S { a: u8 }`). The scan below only inspects lines strictly
    // INSIDE the body, so it never saw those fields and reported the type as
    // fieldless — which would let a field-bearing same-line type in a
    // warn-reporting module be classified `ExemptWithReason`, the one
    // combination the walk exists to refuse.
    if let Some(open) = lines[decl_line].find('{') {
        // Segment on every brace and comma, so a struct VARIANT nested on the
        // same line (`enum E { V { a: u8 } }`) is reached too — splitting only
        // to the first `}` stops at the variant's own opening brace.
        if lines[decl_line][open + 1..]
            .split([',', '{', '}'])
            .any(is_named_field_line)
        {
            return true;
        }
    }
    let mut depth = 0usize;
    let mut entered = false;
    for line in lines.iter().skip(decl_line) {
        // Field detection runs on lines strictly INSIDE the body, so the
        // declaration's own generic bounds (`struct Foo<T: Bar>`) are never
        // mistaken for a field.
        if entered && depth > 0 && is_named_field_line(line) {
            return true;
        }
        for ch in line.chars() {
            match ch {
                '{' => {
                    depth += 1;
                    entered = true;
                }
                '}' => {
                    depth = depth.saturating_sub(1);
                    if entered && depth == 0 {
                        return false;
                    }
                }
                // A `;` before any `{` ends a tuple or unit struct; without
                // this the walk would run on into the NEXT item's body.
                ';' if !entered => return false,
                _ => {}
            }
        }
    }
    false
}

/// `true` when `line` declares a named field — `ident:` but not `ident::`.
fn is_named_field_line(line: &str) -> bool {
    let mut rest = line.trim();
    if rest.is_empty() || rest.starts_with('#') {
        return false;
    }
    for vis in ["pub(crate) ", "pub(super) ", "pub(self) ", "pub "] {
        if let Some(r) = rest.strip_prefix(vis) {
            rest = r.trim_start();
            break;
        }
    }
    let name: String = rest
        .chars()
        .take_while(|c| c.is_alphanumeric() || *c == '_')
        .collect();
    if name.is_empty() {
        return false;
    }
    let after = rest[name.len()..].trim_start();
    after.starts_with(':') && !after.starts_with("::")
}

#[test]
fn the_field_reader_sees_struct_and_struct_variant_fields_and_nothing_else() {
    let field_bearing = ["struct A {", "    pub name: String,", "}"];
    assert!(body_has_named_fields(&field_bearing, 0));

    // A struct-variant enum: the field is at depth 2, which still counts.
    let struct_variant = [
        "enum B {",
        "    Name(String),",
        "    Full {",
        "        name: String,",
        "    },",
        "}",
    ];
    assert!(body_has_named_fields(&struct_variant, 0));

    // Unit + newtype variants only — nothing to deny.
    let fieldless = [
        "enum C {",
        "    DropOldest,",
        "    Block,",
        "    Sample(u64),",
        "}",
    ];
    assert!(!body_has_named_fields(&fieldless, 0));

    // A tuple struct must not run on into the NEXT item's body.
    let tuple_then_struct = ["struct D(u32);", "struct E {", "    f: u8,", "}"];
    assert!(!body_has_named_fields(&tuple_then_struct, 0));

    // An attribute line carrying a colon is not a field.
    let attr_only = [
        "struct F {",
        "    #[serde(rename = \"x\")]",
        "    g: u8,",
        "}",
    ];
    assert!(body_has_named_fields(&attr_only, 0));
    assert!(!is_named_field_line("    #[serde(default)]"));
    // A path is not a field.
    assert!(!is_named_field_line("    Self::Variant => 1,"));
    assert!(is_named_field_line("    inputs: Option<Vec<InputJson>>,"));
}

/// Does `src` contain a `tracing::warn!` carrying the structured field
/// `field` (i.e. `field = ...`) inside its argument list?
///
/// The argument list is PAREN-MATCHED, so a `warn!` and a distant mention of
/// the field name cannot satisfy each other. Matching `field =` rather than a
/// bare `field` is what keeps the warn's own MESSAGE from answering for it:
/// `node.rs`'s line reads "unknown key in ..." with a SPACE, which is not the
/// `unknown_key` token, and `code_only` deliberately does not model string
/// literals.
///
/// SCOPE: this answers "does this module report unknown keys AT ALL" and is
/// the module-level anchor. It cannot say WHICH level is covered — `node.rs`
/// has two such warns and either one satisfies it. Per-TYPE coverage is
/// [`warn_span_matches`]'s job, driven by the [`WarnScope`] on each
/// `WarnInsteadOfDeny` classification.
fn warn_carries_field(src: &str, field: &str) -> bool {
    warn_span_matches(src, &[field], &[])
}

/// Does `src` contain a `tracing::warn!` whose argument list assigns EVERY
/// field in `requires` and NONE of the fields in `forbids`?
///
/// The argument list is PAREN-MATCHED, so a `warn!` and a distant mention of a
/// field name cannot satisfy each other. Matching `field =` rather than a bare
/// `field` is what keeps the warn's own MESSAGE from answering for it:
/// `node.rs`'s line reads "unknown key in ..." with a SPACE, which is not the
/// `unknown_key` token, and `code_only` deliberately does not model string
/// literals.
///
/// `forbids` is what makes a per-type claim possible in a module carrying more
/// than one such warn: the two must be separated by a field one has and the
/// other cannot, or deleting either leaves the survivor answering for both.
fn warn_span_matches(src: &str, requires: &[&str], forbids: &[&str]) -> bool {
    let mut from = 0usize;
    while let Some(hit) = src[from..].find("tracing::warn!(") {
        let open = from + hit + "tracing::warn!(".len();
        let mut depth = 1usize;
        let mut end = open;
        for (i, ch) in src[open..].char_indices() {
            match ch {
                '(' => depth += 1,
                ')' => {
                    depth -= 1;
                    if depth == 0 {
                        end = open + i;
                        break;
                    }
                }
                _ => {}
            }
        }
        if depth == 0 {
            let span = &src[open..end];
            if requires.iter().all(|f| field_is_assigned_in(span, f))
                && !forbids.iter().any(|f| field_is_assigned_in(span, f))
            {
                return true;
            }
        }
        from = open;
    }
    false
}

/// `field` appears in `span` as a WHOLE token immediately followed by `=`.
fn field_is_assigned_in(span: &str, field: &str) -> bool {
    let mut from = 0usize;
    while let Some(hit) = span[from..].find(field) {
        let at = from + hit;
        let before_ok = at == 0
            || !span[..at]
                .chars()
                .next_back()
                .is_some_and(|c| c.is_alphanumeric() || c == '_');
        let after = span[at + field.len()..].trim_start();
        if before_ok && after.starts_with('=') && !after.starts_with("==") {
            return true;
        }
        from = at + field.len();
    }
    false
}

#[test]
fn the_warn_reader_requires_a_structured_field_inside_the_macro_call() {
    let real =
        "tracing::warn!(\n    port = %port,\n    unknown_key = %key,\n    \"unknown key in x\"\n);";
    assert!(warn_carries_field(real, "unknown_key"));

    // The warn's own MESSAGE must not answer for the field.
    let message_only = "tracing::warn!(\"unknown_key was ignored\");";
    assert!(!warn_carries_field(message_only, "unknown_key"));

    // A warn without the field, and the field without a warn, each fail.
    assert!(!warn_carries_field(
        "tracing::warn!(port = %port, \"x\");",
        "unknown_key"
    ));
    assert!(!warn_carries_field("let unknown_key = 1;", "unknown_key"));

    // A nested paren inside the argument list must not end the span early.
    let nested = "tracing::warn!(a = f(1), unknown_key = %k, \"m\");";
    assert!(warn_carries_field(nested, "unknown_key"));

    // A longer identifier that merely CONTAINS the field name is not it.
    assert!(!warn_carries_field(
        "tracing::warn!(the_unknown_key = %k, \"m\");",
        "unknown_key"
    ));
}

#[test]
fn the_scope_discriminator_separates_two_warns_in_one_module() {
    // The real shape: `node.rs` carries both warns, distinguished only by
    // whether the line names a `port`. A module-level "has an unknown_key
    // warn" check is satisfied by either; a scoped one must not be.
    let both = concat!(
        "tracing::warn!(section = \"<top-level>\", unknown_key = %key, \"unknown key in x\");\n",
        "tracing::warn!(section, port = %port, unknown_key = %key, \"unknown key in x\");\n",
    );
    let envelope_only = "tracing::warn!(section = \"<top-level>\", unknown_key = %key, \"m\");";
    let port_only = "tracing::warn!(section, port = %port, unknown_key = %key, \"m\");";

    // With both present, each scope finds its own.
    assert!(warn_span_matches(
        both,
        ENVELOPE_SCAN.requires,
        ENVELOPE_SCAN.forbids
    ));
    assert!(warn_span_matches(
        both,
        PORT_ENTRY_SCAN.requires,
        PORT_ENTRY_SCAN.forbids
    ));

    // THE POINT: delete one and the survivor must not answer for it.
    assert!(
        !warn_span_matches(port_only, ENVELOPE_SCAN.requires, ENVELOPE_SCAN.forbids),
        "the port-entry warn carries `port`, so it cannot stand in for the envelope scan"
    );
    assert!(
        !warn_span_matches(
            envelope_only,
            PORT_ENTRY_SCAN.requires,
            PORT_ENTRY_SCAN.forbids
        ),
        "the envelope warn names no port, so it cannot stand in for the port-entry scan"
    );

    // The module-level anchor is deliberately blind to the difference — which
    // is exactly why the scoped check exists. (Anti-tautology for the two
    // assertions above: without this, they could be passing because
    // `warn_carries_field` itself broke.)
    assert!(warn_carries_field(port_only, "unknown_key"));
    assert!(warn_carries_field(envelope_only, "unknown_key"));

    // `forbids` matches the same whole-token rule as `requires`: a field whose
    // name merely CONTAINS a forbidden one does not trip it.
    assert!(warn_span_matches(
        "tracing::warn!(report = %r, unknown_key = %k, \"m\");",
        &["unknown_key"],
        &["port"]
    ));
}

#[test]
fn every_user_authored_config_type_denies_unknown_fields() {
    let root = repo_root();
    let mut offenders: Vec<String> = Vec::new();
    // (file, type) pairs that are genuinely un-denied — used to prove every
    // declared exemption is still describing a real divergence.
    let mut undenied: BTreeSet<(String, String)> = BTreeSet::new();
    let classified: std::collections::BTreeMap<(&str, &str), Classification> = UNDENIED_TYPES
        .iter()
        .map(|(f, t, c, _)| ((*f, *t), *c))
        .collect();

    // The fourth tuple element is the REASON, and every message this gate
    // prints tells the reader an exemption needs one. Nothing checked it: the
    // map above destructures it as `_`, so an entry could carry `""` and the
    // gate would still pass while the inventory silently became a bare
    // allow-list. A reason is the whole difference between a classification
    // and a waiver.
    let reasonless: Vec<String> = UNDENIED_TYPES
        .iter()
        .filter(|(_, _, _, why)| why.trim().len() < 20)
        .map(|(f, t, c, why)| format!("  {f} `{t}` (classified {c:?}) — reason: {why:?}"))
        .collect();
    assert!(
        reasonless.is_empty(),
        "every entry in `UNDENIED_TYPES` must carry a REASON saying why this type does not \
         deny unknown fields — these do not:\n{}\n\nFIX: write the reason, or delete the \
         entry and add `#[serde(deny_unknown_fields)]` to the type.",
        reasonless.join("\n")
    );
    let mut total = 0usize;

    let modules = walked_modules(&root);
    for module in &modules {
        let path = root.join(module.path);
        let src = fs::read_to_string(&path).unwrap_or_else(|e| {
            panic!("declared config module {} is unreadable: {e}", module.path)
        });
        let code = code_only(&src);

        // A module that DECLARES it reports unknown keys must still do so.
        // This is what makes `WarnInsteadOfDeny` a checked claim rather than a
        // label: delete the warn and the classification stops being true.
        if let Some(field) = module.unknown_key_warn_field {
            assert!(
                warn_carries_field(&code, field),
                "{} is declared as reporting unknown keys through a `tracing::warn!` carrying \
                 `{field} = ...`, and no such warn exists in it. Every type below classified \
                 `WarnInsteadOfDeny` is now SILENTLY dropping unknown keys — the exact defect \
                 that classification exists to rule out.\n\nFIX: restore the warn, or reclassify \
                 those types (and say what reports their unknown keys instead) in \
                 `UNDENIED_TYPES` in cerulion_core/tests/config_deny_unknown_fields_test.rs.",
                module.path
            );
        }

        let items = deserialize_items(module.path, &src);
        assert!(
            !items.is_empty() || !module.declared,
            "declared config module {} ({}) has NO `Deserialize`-deriving type — \
             either the walk broke or the module moved; fix the walk or the inventory, \
             never delete the entry silently",
            module.path,
            module.why
        );
        total += items.len();
        for item in items {
            if item.denies {
                continue;
            }
            undenied.insert((item.file.clone(), item.name.clone()));
            let Some(class) = classified.get(&(module.path, item.name.as_str())) else {
                offenders.push(format!(
                    "  {}:{} `{}` derives Deserialize with NO `#[serde(deny_unknown_fields)]` \
                     and NO classification",
                    item.file, item.line, item.name
                ));
                continue;
            };
            match class {
                Classification::WarnInsteadOfDeny { .. }
                    if module.unknown_key_warn_field.is_none() =>
                {
                    offenders.push(format!(
                        "  {}:{} `{}` is classified `WarnInsteadOfDeny`, but {} declares no \
                         unknown-key warn — so nothing reports its unknown keys and the \
                         classification claims a report that does not exist",
                        item.file, item.line, item.name, module.path
                    ));
                }
                Classification::WarnInsteadOfDeny { .. } if !item.has_named_fields => {
                    offenders.push(format!(
                        "  {}:{} `{}` is classified `WarnInsteadOfDeny` but declares no named \
                         field — there is no unknown key to report. Classify it \
                         `ExemptWithReason` (nothing to deny) instead",
                        item.file, item.line, item.name
                    ));
                }
                // The per-TYPE half. The module-level check above proves SOME
                // warn reports unknown keys; this proves the one that covers
                // THIS type is still there. A module reporting at two levels
                // (`node.rs`: the info-JSON envelope and a port entry) would
                // otherwise let either warn answer for both, so deleting one
                // would cost nothing.
                Classification::WarnInsteadOfDeny { scope }
                    if !warn_span_matches(&code, scope.requires, scope.forbids) =>
                {
                    offenders.push(format!(
                        "  {}:{} `{}` is classified `WarnInsteadOfDeny`, covered by {} — and \
                         {} contains no `tracing::warn!` assigning [{}] while assigning none \
                         of [{}]. Unknown keys of this type are now dropped in SILENCE, which \
                         is the one thing the classification rules out. Restore that warn, or \
                         reclassify the type and say what reports its keys instead",
                        item.file,
                        item.line,
                        item.name,
                        scope.what,
                        module.path,
                        scope.requires.join(", "),
                        scope.forbids.join(", "),
                    ));
                }
                Classification::ExemptWithReason
                    if module.unknown_key_warn_field.is_some() && item.has_named_fields =>
                {
                    offenders.push(format!(
                        "  {}:{} `{}` is classified `ExemptWithReason` — i.e. silently dropping \
                         an unknown key is fine — but it HAS named fields and {} is a module \
                         that reports unknown keys. The correct classification is \
                         `WarnInsteadOfDeny`",
                        item.file, item.line, item.name, module.path
                    ));
                }
                _ => {}
            }
        }
    }

    assert!(
        total >= 6,
        "the walk found only {total} Deserialize-deriving types across {} declared config \
         modules — an implausibly small number means the walk stopped seeing items, which \
         would make every assertion below vacuous",
        WALKED_CONFIG_MODULES.len()
    );

    assert!(
        offenders.is_empty(),
        "a config type must never DROP a misspelled key in silence (the YAML half of the \
         macro contract):\n{}\n\nFIX, in order of preference: add \
         `#[serde(deny_unknown_fields)]` to the type; or, if serde forbids the attribute on \
         its shape, report the unknown key with a `warn!` and classify it \
         `WarnInsteadOfDeny`; or, if the type is genuinely not a document anybody authors \
         (a process handoff, a deliberately-lenient version probe, a type with no named \
         fields), classify it `ExemptWithReason`. Classifications live in `UNDENIED_TYPES` \
         in cerulion_core/tests/config_deny_unknown_fields_test.rs and each needs a REASON.",
        offenders.join("\n")
    );

    // The other direction: a declared classification that no longer describes
    // a real divergence is stale and must be deleted, or it silently
    // pre-authorises the next type that reuses the name.
    let stale: Vec<String> = UNDENIED_TYPES
        .iter()
        .filter(|(f, t, _, _)| !undenied.contains(&((*f).to_string(), (*t).to_string())))
        .map(|(f, t, c, _)| format!("  {f} `{t}` (classified {c:?})"))
        .collect();
    assert!(
        stale.is_empty(),
        "STALE classification(s) — the named type now denies unknown fields, was renamed, or \
         no longer exists:\n{}\n\nFIX: delete the entry from `UNDENIED_TYPES`.",
        stale.join("\n")
    );
}

// ─────────────────────────────────────────────────────────────────────────
// What a user actually sees
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn a_misspelled_graph_yaml_key_is_refused_naming_the_key_and_the_fix() {
    // `depht` for `depth`-shaped intent on an output: the exact silent-drop
    // shape. (`max_slice_len` is the real key; the point is the typo, not
    // which key was meant.)
    let yaml = r#"
name: typo_demo
nodes:
  - id: cam
    type: camera
    outputs:
      - name: image
        schema: sensor_msgs/Image
        depht: 32
"#;
    let err = cerulion_core::graph::parse_graph_raw(yaml)
        .expect_err("a misspelled output key must be REFUSED, not silently dropped");
    let reason = err.to_string();
    assert!(
        reason.contains("depht"),
        "the error must NAME the offending key so the user can find it; got: {reason}"
    );
    // The accepted set is the fix — the user needs to know what to write.
    // Matched in serde's BACKTICKED form (`` `name` ``, not a bare `name`),
    // because a bare substring check is satisfied by the boilerplate of the
    // message itself ("unknown field" contains "name" nowhere useful, but
    // several accepted keys are substrings of one another).
    for accepted in ["name", "schema", "max_slice_len", "topic", "history_size"] {
        assert!(
            reason.contains(&format!("`{accepted}`")),
            "the error must list the accepted output keys (the fix); `{accepted}` missing \
             from: {reason}"
        );
    }

    // Anti-tautology: the SAME document with the typo corrected parses. Without
    // this the assertions above would pass against a parser that refuses
    // everything.
    let fixed = yaml.replace("depht: 32", "max_slice_len: 32");
    let ok =
        cerulion_core::graph::parse_graph_raw(&fixed).expect("the corrected document must parse");
    assert_eq!(ok.nodes[0].outputs[0].max_slice_len, Some(32));
}

#[test]
fn a_misspelled_top_level_graph_key_is_refused_naming_the_key_and_the_fix() {
    let yaml = r#"
name: typo_demo
proces_groups:
  p0: [cam]
nodes:
  - id: cam
    type: camera
"#;
    let err = cerulion_core::graph::parse_graph_raw(yaml)
        .expect_err("a misspelled top-level key must be REFUSED, not silently dropped");
    let reason = err.to_string();
    assert!(
        reason.contains("`proces_groups`") && reason.contains("`process_groups`"),
        "the error must name the typo AND the real key it was meant to be; got: {reason}"
    );

    let fixed = yaml.replace("proces_groups", "process_groups");
    let ok =
        cerulion_core::graph::parse_graph_raw(&fixed).expect("the corrected document must parse");
    assert!(ok.has_process_groups());
}

/// The one place this gate's attribute and the file-stem graph-name rule
/// meet: `deny_unknown_fields` and a DEPRECATED-but-still-accepted key.
///
/// The FILE STEM is the graph name and `name:` is "optional and ignored".
/// That rule keeps `GraphConfig::name` as a `#[serde(default)]
/// Option<String>` — deliberately, so an old YAML still parses and still
/// ROUND-TRIPS through `node stage` / `graph partition`. Had it instead
/// DELETED the field and leaned on serde's default ignore-unknown, this
/// attribute would have converted every graph still carrying `name:` — which
/// is most graphs written before that decision — into a hard parse error. A
/// graph that ran yesterday would not load.
///
/// Nothing pinned that interaction from either side: the stem rule's own tests predate
/// the attribute, and this file's other arms never carry a deprecated key. The
/// field is marked DEPRECATED in its doc comment, so "tidy it away" is a
/// plausible future edit, and the cost of getting it wrong is silent until
/// somebody's robot will not start.
///
/// So: a legacy `name:` PARSES (and does not become the identity), while a key
/// that is merely misspelled is still REFUSED. Both halves in one body — the
/// acceptance alone would be satisfied by dropping the attribute entirely.
#[test]
fn a_deprecated_but_accepted_key_still_parses_while_a_misspelled_one_is_refused() {
    let with_legacy_name = r#"
name: legacy_stem
nodes:
  - id: cam
    type: camera
"#;
    let ok = cerulion_core::graph::parse_graph_raw(with_legacy_name)
        .expect("a legacy `name:` must still parse; by design it is IGNORED, not REJECTED");
    assert_eq!(
        ok.name.as_deref(),
        Some("legacy_stem"),
        "the key is retained on the struct (it must round-trip, not be deleted \
         out from under the author)"
    );

    // ANTI-TAUTOLOGY: the same document with the key MISSPELLED must still be
    // refused, naming it. Without this, an arm asserting only the acceptance
    // above passes just as happily against a `GraphConfig` that carries no
    // `deny_unknown_fields` at all.
    let typo = with_legacy_name.replace("name:", "nmae:");
    let err = cerulion_core::graph::parse_graph_raw(&typo)
        .expect_err("a MISSPELLED top-level key must still be refused");
    let reason = err.to_string();
    assert!(
        reason.contains("`nmae`"),
        "the error must NAME the offending key; got: {reason}"
    );
    assert!(
        reason.contains("`name`"),
        "the accepted set must still list the deprecated-but-accepted key, or the \
         user is told to delete the line rather than fix the spelling; got: {reason}"
    );
}

// ─────────────────────────────────────────────────────────────────────────
// Consequence check: everything this repo ships still loads
// ─────────────────────────────────────────────────────────────────────────

/// True when a parsed YAML document has the SHAPE of a graph: a top-level
/// `nodes:` sequence whose entries carry `id` and `type`.
///
/// Shape, not directory: `examples/go2/graphs/go2.bridge.yaml` sits in a
/// `graphs/` directory but is the `dds_bridge` node's own mapping config
/// (`DDS_BRIDGE_CONFIG`), carries `mappings:`/`domain_id:` and is not a
/// `GraphConfig` at all. Keying on the path would have failed this gate on a
/// file that is not its business.
fn looks_like_a_graph(doc: &serde_yaml::Value) -> bool {
    let Some(nodes) = doc.get("nodes").and_then(|n| n.as_sequence()) else {
        return false;
    };
    // An EMPTY `nodes:` sequence is still a graph document, and skipping it
    // meant a strict-parse regression in such a file left this gate green.
    // `.all()` over an empty sequence is vacuously true, which is the right
    // answer here: the shape test is "has a `nodes:` sequence whose entries,
    // if any, look like nodes".
    nodes
        .iter()
        .all(|n| n.get("id").is_some() && n.get("type").is_some())
}

/// Every `.yaml`/`.yml` file under `dir`, relative to `root`, skipping build
/// output and VCS state.
///
/// A FILESYSTEM walk, deliberately not `git ls-files`: a graph fixture that
/// has been written but not yet `git add`ed is exactly the one a contributor
/// wants checked, and an untracked file is invisible to git.
fn collect_yaml(root: &Path, dir: &Path, out: &mut Vec<PathBuf>) {
    // FAIL CLOSED. A walk that swallows its errors — an unreadable
    // directory returns, a bad entry is `flatten`ed away — lets a permission
    // or I/O failure hide whatever graph YAML lives below it, and the gate
    // reports a clean sheet for files it never opened. A walk that cannot
    // see a file must SAY so, not pass.
    let entries = fs::read_dir(dir).unwrap_or_else(|e| {
        panic!(
            "cannot read {} while walking for graph YAML: {e}",
            dir.display()
        )
    });
    for entry in entries {
        let entry = entry.unwrap_or_else(|e| {
            panic!(
                "cannot read an entry of {} while walking for graph YAML: {e}",
                dir.display()
            )
        });
        let path = entry.path();
        let name = entry.file_name();
        let name = name.to_string_lossy();
        if path.is_dir() {
            if name == "target" || name == ".git" || name == "node_modules" {
                continue;
            }
            // Another checkout's tree is not this branch's business.
            if common::is_other_checkout(&path) {
                continue;
            }
            collect_yaml(root, &path, out);
        } else if name.ends_with(".yaml") || name.ends_with(".yml") {
            if let Ok(rel) = path.strip_prefix(root) {
                out.push(rel.to_path_buf());
            }
        }
    }
}

#[test]
fn the_field_reader_sees_a_body_declared_on_the_declaration_line() {
    // The same-line shape: the per-line scan only inspects lines strictly
    // INSIDE the body, so it read this as fieldless — which would let a
    // field-bearing type be classified `ExemptWithReason` in a module the
    // walk requires to report its keys.
    let inline = ["struct S { a: u8 }"];
    assert!(body_has_named_fields(&inline, 0));

    // A same-line body with NO named field is still fieldless.
    let tuple_inline = ["struct T(u32);"];
    assert!(!body_has_named_fields(&tuple_inline, 0));
    let empty_inline = ["struct U {}"];
    assert!(!body_has_named_fields(&empty_inline, 0));

    // A same-line ENUM with a struct variant carries named fields.
    let variant_inline = ["enum E { V { a: u8 } }"];
    assert!(body_has_named_fields(&variant_inline, 0));

    // The multi-line path is unchanged.
    let multi = ["struct M {", "    a: u8,", "}"];
    assert!(body_has_named_fields(&multi, 0));
}

#[test]
fn a_graph_with_an_empty_node_list_is_still_a_graph() {
    // `nodes: []` parses as a graph and must be CHECKED. Skipping it meant a
    // strict-parse regression in such a file left the consequence gate green.
    let empty: serde_yaml::Value = serde_yaml::from_str("nodes: []\n").unwrap();
    assert!(looks_like_a_graph(&empty));

    // The shape test still refuses a document that is not a graph.
    let not_a_graph: serde_yaml::Value = serde_yaml::from_str("mappings:\n  - from: /a\n").unwrap();
    assert!(!looks_like_a_graph(&not_a_graph));
    // ...and one whose entries are not nodes.
    let wrong_entries: serde_yaml::Value = serde_yaml::from_str("nodes:\n  - name: x\n").unwrap();
    assert!(!looks_like_a_graph(&wrong_entries));
}

#[test]
fn every_graph_yaml_in_the_repo_still_parses() {
    // Every OTHER arm in this file joins a CRATE-relative path onto
    // `repo_root()`, which is `crates/` after the move and therefore correct.
    // This arm is the one that sweeps the WHOLE repo, so it ascends once more.
    let root = repo_root()
        .parent()
        .expect("crates/ has a parent (the repo root)")
        .to_path_buf();
    let mut all_yaml: Vec<PathBuf> = Vec::new();
    collect_yaml(&root, &root, &mut all_yaml);
    all_yaml.sort();

    let mut checked: Vec<String> = Vec::new();
    let mut failures: Vec<String> = Vec::new();
    for path in &all_yaml {
        let rel = path.to_string_lossy().to_string();
        // Also fail CLOSED: a file the walk FOUND but cannot read is a file
        // this gate has no verdict on, and skipping it silently is how an
        // unparseable graph would ride through green.
        let text = fs::read_to_string(root.join(path)).unwrap_or_else(|e| {
            panic!(
                "cannot read {} — this gate has no verdict on it: {e}",
                path.display()
            )
        });
        let Ok(doc) = serde_yaml::from_str::<serde_yaml::Value>(&text) else {
            continue; // not YAML we can classify; other gates own malformed files
        };
        if !looks_like_a_graph(&doc) {
            continue;
        }
        checked.push(rel.clone());
        if let Err(e) = cerulion_core::graph::parse_graph_raw(&text) {
            failures.push(format!("  {rel}: {e}"));
        }
    }

    assert!(
        checked.len() >= 11,
        "only {} graph YAML file(s) were classified — the shape detector has stopped \
         recognising graphs, which would make this gate vacuous (found: {:?})",
        checked.len(),
        checked
    );
    assert!(
        failures.is_empty(),
        "graph YAML shipped in this repo no longer parses under \
         `#[serde(deny_unknown_fields)]`:\n{}\n\nFIX: correct the key in the FILE. Do not \
         loosen the type — the whole point of the attribute is that the file and the \
         schema agree.",
        failures.join("\n")
    );
}

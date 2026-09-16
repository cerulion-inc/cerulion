// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion bag play --resim` — re-execute a recorded bag deterministically
//! and, under `--verify`, check it against the recording.
//!
//! The verb this module was written for, `cerulion replay`, has been REMOVED;
//! the engine below is unchanged and is reached through
//! `cerulion_cli_engine::resim_cmd`.
//!
//! This module owns the verb's **entry gates** and the stable **exit-code
//! contract**; [`crate::replay_engine`] owns the deterministic replay itself.
//! [`run_replay`] runs the gates, then hands the gated artifacts to
//! the engine and maps its [`ReplayOutcome`]
//! to the pass/violation exit codes. Tolerance YAML validation
//! reuses the [`ReplayError`] variants defined here.
//!
//! # Exit-code contract (stable surface, 0–6)
//!
//! | Code | Class (the operator-facing phrase) | Meaning |
//! |---|---|---|
//! | 0 | — | pass ([`EXIT_PASS`]) — `outcome.passed` |
//! | 1 | **frame-content divergence** | data violation ([`EXIT_VIOLATION`]) — `!outcome.passed` |
//! | 2 | — | bag I/O or not-replay-grade (incl. bag/graph mismatch) |
//! | 3 | — | node failure: cdylib LOAD error, or panic-class EXECUTION failure during the replay (a later widening) |
//! | 4 | — | tolerance-YAML validation error |
//! | 5 | — | internal error (panic, transport, scheduler) |
//! | 6 | **fire-schedule divergence** | structural trace divergence |
//!
//! Vocabulary rule: the two
//! COMPARISON codes are named by descriptive phrase wherever an operator sees
//! them — the verdict block header, the `--report` JSON's `divergence_classes`
//! array, and the verifier's own slotting — never by a bare exit-code label.
//! The third class,
//! [`DivergenceClass::EdgeRead`](crate::replay_engine::DivergenceClass::EdgeRead)
//! (**edge-read divergence**), has NO exit code at all: the read-log verifier
//! is report-only until its promotion window closes. The phrases render
//! from ONE function
//! ([`divergence_class_phrase`](crate::replay_engine::divergence_class_phrase)).
//! The table above, though, is MARKDOWN — a literal, which no renderer can
//! keep in step — so what holds it to that function is this module's own
//! `tests::the_exit_code_tables_phrases_are_the_rendered_vocabulary`, which
//! reads this file's source and requires each of the three phrases to be
//! exactly what `divergence_class_phrase` returns. The rename is WORDS ONLY:
//! the numbers in this table, `outcome.passed`, and the precedence below are
//! byte-unchanged.
//!
//! Code **7** ([`crate::login_cmd::EXIT_AUTH_REQUIRED`]) is NOT a replay
//! outcome — it is the CLI-wide "authentication required" refusal `main`'s login
//! gate returns before a command runs. It is defined in `login_cmd` (the gate
//! that produces it), NOT here, because this module is `#[cfg(unix)]` while the
//! login gate and its exit code are platform-independent; it is listed in this
//! contract so the whole exit-code SPACE stays visible in one place. Sitting
//! above the replay range keeps it from ever colliding with [`EXIT_VIOLATION`]
//! (1).
//!
//! Codes 2–5 are typed [`ReplayError`]s — with one widening: exit 3 ALSO
//! fires from the engine's outcome when
//! [`ReplayOutcome::node_failures`](crate::replay_engine::ReplayOutcome::node_failures)
//! is non-empty (a candidate node PANICKED mid-replay — panic-class only,
//! detected structurally; a deterministic tick `Err` is normal execution and
//! replays to exit 0). 0/1/3(execution)/6 are the engine's [`ReplayOutcome`]
//! verdict, with precedence 3 > 6 > 1 (root cause over symptoms — a crashed
//! candidate explains both a diverged schedule and missing frames; the
//! verdict still renders every block). Exit 6 fires when
//! [`ReplayOutcome::trace_divergence`](crate::replay_engine::ReplayOutcome::trace_divergence)
//! is populated — it takes precedence over exit 1. The typed 2–5 error
//! mapping is pinned by an oracle-vector test so the contract cannot drift
//! silently; the outcome-driven exit-6 mapping is pinned by the subprocess CLI
//! test (`crates/cerulion_cli/tests/replay_cli_test.rs`), which also pins the exit-3
//! execution arm end-to-end (a panicking twin cdylib).
//!
//! # Unix-only
//!
//! The module reads bags via `cerulion_bag`, which is `#![cfg(unix)]` (it reuses
//! `cerulion_core`'s Unix-only `trace_ring::TraceRingRecord`). The module is
//! therefore gated at its declaration in `lib.rs`; a non-Unix
//! `cerulion bag play --resim` returns a loud "unsupported on this platform"
//! error from the CLI dispatcher.

use std::path::{Path, PathBuf};

use cerulion_bag::{BagAttachment, BagCompleteness, BagError, BagReader};
// Only DEPARTURE is named here: the FIRE / STEP_BOUNDARY / READ-OUTCOME arms
// live in `trace_ring::classify_trace_record`, which is the point — this file
// carries no copy of its own of the record-type rules.
// IMPORTED, never re-implemented: see the same
// note in `bag_migrate`. This refusal and the migration must agree about
// whether a bag is repairable, or one sends the user to the other.
use cerulion_core::graph::unknown_field_key;
use cerulion_core::trace_ring::RECORD_TYPE_DEPARTURE;

use cerulion_bagd::{RecordHealth, RECORD_HEALTH_ATTACHMENT};

use crate::replay_engine::{self, CompiledTolerance, ReplayInputs, ReplayNodes};
pub use crate::replay_engine::{
    render_verdict, NodeFailure, RecordHealthReport, RecorderInfo, ReplayOutcome,
};

/// Exit code 0 — replay passed.
pub const EXIT_PASS: u8 = 0;
/// Exit code 1 — a data violation (missing topic vs. missing messages are two
/// distinct violation classes). Returned when the engine's
/// [`ReplayOutcome`] reports one or more
/// [`Violation`](crate::replay_engine::Violation)s (`!outcome.passed`) — a
/// non-error verification failure, distinct from the typed [`ReplayError`]s.
pub const EXIT_VIOLATION: u8 = 1;

/// Exit code 2 — bag I/O or not-replay-grade.
const EXIT_NOT_REPLAY_GRADE: u8 = 2;
/// Exit code 3 — node failure: cdylib LOAD error (the typed
/// [`ReplayError::NodeLoad`]) OR a panic-class EXECUTION failure during the
/// replay (a later widening — the engine's
/// [`ReplayOutcome::node_failures`](crate::replay_engine::ReplayOutcome::node_failures)
/// is non-empty; the CLI maps it from the outcome, PREEMPTING exits 6 and 1:
/// a crashed candidate is the root cause, the diverged schedule / missing
/// frames are its symptoms). Panic-class ONLY (a caught panic or the cdylib
/// panic/poisoned FFI codes) — a deterministic tick `Err` is normal
/// execution and never maps here.
pub const EXIT_NODE_FAILURE: u8 = 3;
/// Exit code 4 — tolerance-YAML validation error.
const EXIT_TOLERANCE_INVALID: u8 = 4;
/// Exit code 5 — internal error.
const EXIT_INTERNAL: u8 = 5;
/// Exit code 6 — structural trace divergence. Returned when a successfully
/// replayed bag's [`ReplayOutcome::trace_divergence`](crate::replay_engine::ReplayOutcome::trace_divergence)
/// is populated (the replayed fire schedule diverged from the recording) — it
/// TAKES PRECEDENCE over a data violation (exit 1). A non-error verification
/// failure like [`EXIT_VIOLATION`], distinct from the typed [`ReplayError`]s;
/// the CLI maps it from the outcome, not from an `Err`.
pub const EXIT_TRACE_DIVERGENCE: u8 = 6;

// Exit code 7 (authentication required) is NOT defined here — it lives in
// `crate::login_cmd::EXIT_AUTH_REQUIRED`, alongside the login gate that produces
// it, because that gate is platform-independent while THIS module is
// `#[cfg(unix)]` (see the module-level exit-code contract). The `main` login
// gate references it directly; the replay verb never returns it.

/// The recorded graph-source attachment name (bare, no `__cerulion/` prefix —
/// see `cerulion_bagd`'s `--attach graph.yaml:<path>`).
pub(crate) const GRAPH_ATTACHMENT: &str = "graph.yaml";
/// The recorded environment-snapshot attachment name.
const ENV_ATTACHMENT: &str = "env.json";
/// Prefix of a per-rank trace-manifest attachment
/// (`__cerulion/trace_manifest_rank{N}.json`, written by `cerulion_bagd`).
/// `pub(crate)`: [`crate::replay_engine`] re-derives the per-rank tables from
/// the same names (multi-rank).
pub(crate) const TRACE_MANIFEST_PREFIX: &str = "__cerulion/trace_manifest_rank";
/// The recorder-host-identity attachment (written by
/// `graph run --record`). OPTIONAL: absent = a pre-recorder.json bag
/// (back-compat, silent); malformed = loud WARN; a host arch/os mismatch =
/// loud WARN. NEVER exit-affecting.
const RECORDER_ATTACHMENT: &str = "__cerulion/recorder.json";
/// Suffix of a trace-manifest attachment name. (`pub(crate)`: see
/// [`TRACE_MANIFEST_PREFIX`].)
pub(crate) const TRACE_MANIFEST_SUFFIX: &str = ".json";

// ===========================================================================
// What NAMES a resim
// ===========================================================================

/// How a bag's file name gets to name the run that re-executes it —
/// [`resolve_resim_identity`]'s verdict.
///
/// # The defect this exists for
///
/// The file stem is the graph's identity by design: `name:` became
/// optional-and-IGNORED, `graph create` stopped writing it, and
/// [`adopt_file_stem_identity`](cerulion_core::graph::adopt_file_stem_identity)
/// stamps `graphs/<stem>.yaml`'s stem onto every config the CLI resolves.
///
/// A bag's embedded `graph.yaml` has NO file to stem from. It is the EFFECTIVE
/// config, serialized by `graph_cmd::render_effective_graph_yaml`,
/// and `GraphConfig::identity` is `#[serde(skip)]` — so the only identity that
/// survives into the embed is the deprecated `name:` key, which
/// [`parse_graph_raw`](cerulion_core::graph::parse_graph_raw) seeds from
/// precisely because it is "the only identity such a config can have".
///
/// Under the file-stem rule there is no such key on a fresh graph. So every bag
/// recorded from a graph created after that change resimmed as
/// [`UNNAMED_GRAPH`](cerulion_core::graph::UNNAMED_GRAPH) — cosmetically loud
/// (`graph=unnamed` in the logs) and, worse, indistinguishable: two concurrent
/// resims of two DIFFERENT bags both mint the iceoryx2 node
/// `cerulion_replay_unnamed`.
///
/// # Why the BAG's stem is the right answer, not a guess
///
/// The recorder already stems the bag BY the identity —
/// `graph_cmd::resolve_recording_paths` writes
/// `{identity}_{utc_stamp}.mcap` — so the identity was never actually lost;
/// it was written on a channel nothing read back. Reading it is the same rule
/// the file-stem rule applies to `graphs/<stem>.yaml`, pointed at the artifact a resim
/// is actually re-executing.
///
/// The whole stem is adopted, `_{utc_stamp}` suffix included: stripping it
/// would be an INFERENCE from a file-name convention a renamed bag does not
/// follow (and this repo prefers a loud literal to a silent smart guess). The
/// consequence is that a resim's `graph=` field reads
/// `perception_20260101T000000Z` where the live run logged `perception` —
/// which is also what makes two concurrent resims tell themselves apart.
#[derive(Debug, Clone, PartialEq, Eq)]
enum ResimIdentity {
    /// The embed named itself (a bag predating the file-stem rule, or a later graph whose
    /// author kept the deprecated key), so nothing is adopted.
    ///
    /// FALLBACK-ONLY is deliberate: adopting unconditionally would rename the
    /// resim of every bag that names itself, where the file-stem fallback
    /// exists only for the ones that do not. See the residual in
    /// [`resolve_resim_identity`].
    Declared,
    /// Nothing named the embed, and the bag's file stem is adopted as the
    /// identity.
    Adopted(String),
    /// Nothing named the embed and the bag file could not supply a name either
    /// — the resim stays `unnamed`, for a stated reason.
    Declined(DeclineCause),
}

/// Why a bag's file name could not name the resim.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum DeclineCause {
    /// The path has no file stem at all (`/`, `..`, an empty stem).
    NoStem,
    /// The stem is not valid UTF-8 (an identity crosses into log fields and an
    /// iceoryx2 node name, both of which are `&str`).
    NotUtf8,
    /// The stem would overflow the iceoryx2 node-name cap once
    /// [`REPLAY_NODE_NAME_PREFIX`](crate::replay_engine::REPLAY_NODE_NAME_PREFIX)
    /// is prepended.
    ///
    /// DECLINED rather than TRUNCATED: truncation is a second inference, and
    /// two bags sharing a long prefix would silently collide on one node name.
    TooLong {
        /// The stem's length in BYTES (the cap is a byte capacity).
        len: usize,
        /// The longest stem this resim could have carried.
        max: usize,
    },
    /// iceoryx2 refused the fully-prefixed candidate as a node name.
    ///
    /// UTF-8 is NOT sufficient: `NodeName` is a `StaticString`, whose
    /// `insert_bytes` rejects any byte `>= 128` or `== 0` — "Only unicode
    /// points less than 128 (U+0080) are supported" — so a perfectly valid
    /// stem like `naïve` is refused. The `TransportManager::init` conversion
    /// is fallible, so such a bag would FAIL the resim rather than panic it;
    /// this arm earns its place by declining the stem BEFORE adopting it,
    /// which keeps a resim running under
    /// `unnamed` instead of refusing it over a name it never needed.
    ///
    /// The verdict is iceoryx2's OWN (`NodeName::new` on the candidate the
    /// engine will actually mint), never a charset re-encoded here — a
    /// hand-written rule would be a second copy of a constraint that lives in
    /// a dependency, free to drift from it silently. See
    /// [`resim_identity_max_len`] for why the length arm above is kept anyway.
    NotRepresentable,
}

impl DeclineCause {
    /// The operator-facing reason, naming the cause.
    fn reason(self) -> String {
        match self {
            Self::NoStem => "the bag path has no file stem".to_string(),
            Self::NotUtf8 => "the bag's file stem is not valid UTF-8".to_string(),
            Self::TooLong { len, max } => format!(
                "the bag's file stem is {len} bytes, past the {max} an iceoryx2 node name \
                 leaves once `{}` is prepended",
                crate::replay_engine::REPLAY_NODE_NAME_PREFIX
            ),
            Self::NotRepresentable => format!(
                "iceoryx2 will not accept `{}<stem>` as a node name — a node name admits \
                 only ASCII below U+0080, so a stem with an accent, an emoji or any other \
                 non-ASCII character is refused even though it is valid UTF-8",
                crate::replay_engine::REPLAY_NODE_NAME_PREFIX
            ),
        }
    }
}

/// The longest identity a resim may carry.
///
/// DERIVED from iceoryx2's own cap rather than copied: the engine mints
/// `{REPLAY_NODE_NAME_PREFIX}{identity}`, and an identity that overruns the cap
/// costs the resim its run. This check exists for the message: without it the
/// `NodeName` conversion inside `TransportManager::init` refuses the identity
/// with a transport error instead of this verb's diagnostic. A bag from a
/// stem-identified graph always carries the fixed-length `unnamed`, so nothing
/// user-controlled reaches that conversion on that path; adopting a FILE NAME
/// widens it.
///
/// It is NOT the whole constraint, only the half worth a good message:
/// `NodeName` also rejects any byte at or above U+0080, so
/// [`resolve_resim_identity`] asks `NodeName::new` itself as the authority.
/// This arm runs first purely so an over-long stem reports its two numbers
/// instead of the generic charset refusal.
///
/// The pre-existing exposure through a bag's deprecated `name:` key is
/// untouched by either check (an embed predating the file-stem rule can declare anything)
/// — see the residual in [`resolve_resim_identity`].
fn resim_identity_max_len() -> usize {
    iceoryx2::prelude::NodeName::max_len()
        .saturating_sub(crate::replay_engine::REPLAY_NODE_NAME_PREFIX.len())
}

/// Decide what names a resim: the embed if it named itself, else the BAG's file
/// stem, else nothing.
///
/// PURE (no I/O — `bag` is inspected as a path, never opened), so every arm is
/// oracle-tested including the ones a healthy bag never reaches. The one call
/// into iceoryx2 is a string validity check that opens nothing; it may emit
/// iceoryx2's own diagnostic line when it REFUSES, which is a rare and
/// explanatory event rather than a cost on the healthy path.
///
/// # Residuals
///
/// * An embed predating the file-stem rule whose `name:` diverges from the stem the run actually
///   executed under still wins here — the ignored key naming the resim is a
///   (cosmetic) instance of exactly what the file-stem rule removed. Both repairs
///   carry a cost of their own: preferring the stem outright changes the identity
///   of bags that replay correctly, and preferring it only "when the stem
///   starts with the declared name" is the silent inference this repo rejects.
/// * Both checks cover only what THIS function adopts. A bag DECLARING a
///   `name:` that is over-long, or that carries a single non-ASCII character,
///   never reaches them. That case is handled where it belongs:
///   the `cerulion_core` conversion is fallible, and the resim maps its
///   refusal onto the exit-2 not-replay-grade arm
///   (`replay_engine::replay_transport_init_error`). So the declared route
///   REFUSES loudly rather than aborting, while this fallback still DECLINES
///   (the run continues under `unnamed`) — a stem is a guess, a declaration is
///   not, and only the second is worth failing a run over.
fn resolve_resim_identity(embedded: &str, bag: &Path) -> ResimIdentity {
    if !embedded.is_empty() {
        return ResimIdentity::Declared;
    }
    let Some(stem) = bag.file_stem() else {
        return ResimIdentity::Declined(DeclineCause::NoStem);
    };
    let Some(stem) = stem.to_str() else {
        return ResimIdentity::Declined(DeclineCause::NotUtf8);
    };
    if stem.is_empty() {
        return ResimIdentity::Declined(DeclineCause::NoStem);
    }
    let max = resim_identity_max_len();
    if stem.len() > max {
        return ResimIdentity::Declined(DeclineCause::TooLong {
            len: stem.len(),
            max,
        });
    }
    // THE AUTHORITY, asked LAST and about the candidate the engine will
    // actually mint. `NodeName`'s charset is NARROWER than UTF-8 (ASCII below
    // U+0080, no NUL), so the length arm above cannot stand in for it — and
    // `TransportManager::init` would REFUSE this very conversion, so
    // anything it rejects would cost the resim
    // its run rather than merely its name.
    //
    // Asking iceoryx2 rather than re-encoding its rule is the whole point: the
    // constraint lives in a dependency, so a copy here would be free to drift
    // from it silently. The length arm is kept ABOVE it only because it
    // produces the better message (the two numbers an operator can act on) —
    // this call would refuse an over-long candidate too.
    let candidate = format!("{}{stem}", crate::replay_engine::REPLAY_NODE_NAME_PREFIX);
    if iceoryx2::prelude::NodeName::new(&candidate).is_err() {
        return ResimIdentity::Declined(DeclineCause::NotRepresentable);
    }
    ResimIdentity::Adopted(stem.to_string())
}

/// One recorded topic whose recorded schema layout drifted from
/// the current workspace's. Carried by [`ReplayError::SchemaDrift`]; the
/// operator-facing table is rendered by `render_schema_drift`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SchemaDrift {
    /// The graph-produced topic whose schema drifted.
    pub topic: String,
    /// The `schema_hash` stamped in the recording's frames on `topic`.
    pub recorded_hash: u64,
    /// The current workspace's recipe-3 `schema_hash` for `topic`'s producing
    /// output — what the candidate would stamp today.
    pub current_hash: u64,
}

/// Render the [`ReplayError::SchemaDrift`] refusal: a per-topic table of the
/// recorded vs current schema hashes plus the remediation pair. The message is
/// deterministic (drifts are collected in recorded-topic file order).
fn render_schema_drift(drifts: &[SchemaDrift]) -> String {
    let mut s = String::new();
    let n = drifts.len();
    let plural = if n == 1 { "topic" } else { "topics" };
    s.push_str(&format!(
        "this bag is not replay-grade FOR THIS WORKSPACE: {n} produced {plural} \
         changed schema since it was recorded (the recorded frames carry a \
         different layout hash than the current build produces):"
    ));
    for d in drifts {
        s.push_str(&format!(
            "\n  - {}: recorded schema_hash 0x{:016x}, current 0x{:016x}",
            d.topic, d.recorded_hash, d.current_hash
        ));
    }
    s.push_str(
        "\nA byte-exact replay would report a mismatch on every frame of these \
         topics with no meaningful verdict. Re-record the bag with the current \
         workspace, or check out the recording-era schemas before replaying.",
    );
    s
}

/// Typed failure classes of `cerulion bag play --resim`, each mapping to a stable exit
/// code (2–5) via [`exit_code`](Self::exit_code).
///
/// The [`ToleranceInvalid`](Self::ToleranceInvalid) variant fires from the
/// tolerance gate and is defined here so the exit-code contract is
/// complete and testable in one place. There is deliberately NO
/// `TraceDivergence` variant: a structural trace divergence (exit 6) is an
/// OUTCOME (`ReplayOutcome::trace_divergence`), not an error — the engine
/// COMPLETES the data diff even when the schedule diverged (the fuller report)
/// and the CLI picks the exit from the outcome. An `Err`-based exit 6 would
/// short-circuit that diff and silently drop the both-blocks report.
#[derive(Debug, thiserror::Error)]
pub enum ReplayError {
    /// The bag file could not be read as a valid MCAP recording (missing,
    /// unreadable, or corrupt). Exit 2.
    #[error("failed to read bag {path:?}: {source}")]
    BagOpen {
        /// The bag path.
        path: PathBuf,
        /// The underlying bag error (I/O, bad magic, chunk-CRC, torn record).
        #[source]
        source: BagError,
    },

    /// The file is not an MCAP bag at all — it does not begin with the MCAP
    /// magic. Exit 2. (Distinguishes "wrong file" from a real-but-incomplete
    /// bag: `recover_messages` folds a bad start-magic into a `TornTail`
    /// completeness, which would otherwise mislabel a non-bag as "not
    /// finalized".)
    #[error(
        "file {path:?} is not an MCAP bag — it does not begin with the MCAP magic bytes. \
         `cerulion bag play --resim` operates on `.mcap` bags recorded by \
         `cerulion graph run --record`"
    )]
    NotMcapBag {
        /// The path that was not an MCAP bag.
        path: PathBuf,
    },

    /// The bag did not close cleanly — its recording is not replay-grade. Exit 2.
    #[error(
        "bag is not finalized (completeness: {completeness}); strict replay requires a cleanly \
         finalized recording. The recorder crashed mid-write or was killed uncleanly — re-record \
         with `cerulion graph run --record` and stop it with Ctrl+C so it finalizes the bag"
    )]
    BagNotFinalized {
        /// The `BagCompleteness` state that failed the finalized check.
        completeness: String,
    },

    /// The bag carries no scheduler trace, so it cannot be strictly replayed.
    /// Exit 2.
    ///
    /// A `graph run --record` bag carries a trace, and a Flashback
    /// CAPTURE carries one too.
    ///
    /// # The cause and the remedy
    ///
    /// The message does not say a plain `graph run` holds no trace ring to drain,
    /// and does not name `--record` as the remediation:
    /// a multi-process `graph run` provisions per-rank trace
    /// rings whether or not it records, so `--record` is not what puts a
    /// trace in a bag, and the project's rule forbids an absence message naming another
    /// verb's flag in any case.
    ///
    /// What reaches this arm is a bag whose RUN had no trace to give: a
    /// wall-gated run shape (single-process, `ros2 attach`, `node run`, a
    /// virtual/external clock), a run that declined or was refused its rings, or
    /// a build older than the rings. The run records which, and the bag carries
    /// that record — so the message names the cause and points at the evidence
    /// inside the bag the reader is already holding.
    #[error(
        "strict replay requires the recorded scheduler trace, but this bag's \
         `__cerulion/scheduler_trace` channel is empty. If this bag carries \
         `__cerulion/run.json`, the run states why under `trace_rings` — it declined its rings \
         at launch, it was refused them, or it is a wall-gated shape (a single-process run, \
         `ros2 attach`, `node run`, or an external time source) that mints none by design; and \
         a RECORDING also carries `__cerulion/record_coverage.json`, whose `rings_unavailable` \
         names a declared ring the recorder could not open. If it carries no `run.json`, nothing \
         in the bag can say — the shapes that lack one are a standalone `cerulion bag record` \
         bag (taps topics, never carries a trace), a Flashback capture (whose \
         `__cerulion/flashback.json` `handoff.trace` field states whether its recorder was \
         handed trace rings and why this capture carries none), or a run recorded before \
         run directories existed"
    )]
    BagNoSchedulerTrace,

    /// A required replay-grade attachment is absent. Exit 2.
    #[error(
        "bag is missing the required `{name}` attachment — it is not a replay-grade recording \
         produced by `cerulion graph run --record`"
    )]
    BagMissingAttachment {
        /// The missing attachment name.
        name: String,
    },

    /// A required attachment is present but unparseable or fails validation.
    /// Exit 2.
    #[error("bag attachment `{name}` is present but invalid: {reason}")]
    BagInvalidAttachment {
        /// The offending attachment name.
        name: String,
        /// What was wrong (parse error, out-of-bounds index, bad name segment).
        reason: String,
    },

    /// The multi-process trace manifests are non-contiguous — the worker ranks
    /// are not `0..=k` with no gaps (e.g. ranks {0, 2} with rank 1 absent). A
    /// well-formed multi-process recording stamps one manifest per worker rank
    /// starting at 0; a hole means the bag is corrupt or hand-edited. Exit 2.
    /// (The `u32::MAX` supervisor departure-ring manifest is NOT a worker rank
    /// and is excluded from this contiguity contract.)
    #[error(
        "corrupt multi-process bag: trace manifests are non-contiguous — rank {missing} is \
         missing (found worker ranks {present:?}). A multi-process recording must carry a \
         manifest for every rank 0..={max} with no gaps"
    )]
    MultiRankManifestGap {
        /// The contiguous worker ranks actually present (sorted, sentinel excluded).
        present: Vec<u32>,
        /// The first missing rank in the `0..=max` sequence.
        missing: u32,
        /// The highest worker rank present (the sequence upper bound).
        max: u32,
    },

    /// The bag is a DEGRADED recording: the scheduler trace carries a departure
    /// (fault) boundary — a `RECORD_TYPE_DEPARTURE` record, or a FIRE/BOUNDARY
    /// record stamped with the `u32::MAX` supervisor departure-ring sentinel
    /// rank (bagd stamps the ring's rank into `reserved`). A departure boundary
    /// means a peer worker was lost mid-run; replaying a fault-degraded
    /// recording is not supported yet. Exit 2 — refused PRE-canonicalization.
    #[error(
        "degraded recording: scheduler-trace record {index} is a departure/fault boundary \
         ({detail}) — this bag captured a peer departure, and fault replay lands post-launch. \
         Re-record from a clean run (no peer loss) to produce a replay-grade bag"
    )]
    DegradedRecordingDeparture {
        /// The offending record's index in the trace channel (write order).
        index: usize,
        /// Which signal flagged the degradation (departure record vs sentinel rank).
        detail: String,
    },

    /// A scheduler-trace record carries a `record_type` replay cannot honor — an
    /// invalid/zeroed record, or a reserved kind from a newer/foreign writer.
    /// Exit 2. The fault is in the trace CHANNEL, not the manifest. (A departure
    /// boundary is refused earlier — see
    /// [`DegradedRecordingDeparture`](ReplayError::DegradedRecordingDeparture).)
    #[error("scheduler-trace record {index} has unsupported record_type {record_type}: {reason}")]
    TraceRecordUnsupported {
        /// The offending record's index in the trace channel (write order).
        index: usize,
        /// The record's raw `record_type` value.
        record_type: u32,
        /// Why v1 replay refuses it (per record-type class).
        reason: String,
    },

    /// A recorded topic is neither produced nor consumed by the bag's embedded
    /// graph — the recording and its graph.yaml disagree (corrupt or
    /// hand-edited). Exit 2. (Fires at classification time.)
    #[error("bag/graph mismatch on topic `{topic}`: {detail}")]
    BagGraphMismatch {
        /// The recorded topic the graph does not account for.
        topic: String,
        /// Why the bag and its embedded graph disagree.
        detail: String,
    },

    /// The scheduler trace carries FIRE records but no STEP_BOUNDARY (kind 3)
    /// records — without the per-step clock targets, the recorded gating-clock
    /// trajectory cannot be re-advanced (a `Period` fire's `fire_time_ns` is the
    /// period BOUNDARY, which under wall-advance recording undershoots the
    /// step's advanced clock). Exit 2. (Fires at target derivation.)
    ///
    /// **An absence message names no other verb's flag:** the remediation does not read
    /// "re-record with `cerulion graph run --record`". That names another verb's
    /// flag, which that rule forbids for an absence, and it is not even
    /// the distinguishing act, since every multi-process run carries a trace
    /// whether or not it records. A trace WITH fires but WITHOUT boundaries is a
    /// version fact about the recorder that wrote it, so the message says that.
    #[error(
        "strict replay requires per-step STEP_BOUNDARY (record_type 3) scheduler-trace records, \
         but this bag's trace carries none — the recorded clock trajectory cannot be re-advanced. \
         Two bags reach this and the trace cannot tell them apart: one written by a Cerulion \
         older than step boundaries, and one whose boundary records were stripped or corrupted \
         after recording. Either way nothing can add them back to THIS bag; a recording made on \
         this version carries them from the start"
    )]
    BagNoStepBoundaries,

    /// The bag failed an internal cross-consistency check between its
    /// STEP_BOUNDARY records, FIRE records, and recorded frames — the recording
    /// is corrupt or hand-edited. Exit 2. (Fires before the replay
    /// loop.)
    #[error(
        "bag internal consistency check failed: {detail} — the recording is corrupt or \
         hand-edited; re-record with `cerulion graph run --record`"
    )]
    RecordingInconsistent {
        /// What disagreed (record index / topic / frame + the two values).
        detail: String,
    },

    /// One or more recorded graph-produced topics were recorded
    /// under a DIFFERENT schema layout than the current workspace produces — the
    /// recorded frame's `WireHeader.schema_hash` disagrees with the workspace's
    /// current recipe-3 hash for that topic's producing output. Refused as a
    /// PRE-FLIGHT (exit 2, not replay-grade FOR THIS WORKSPACE) so the operator
    /// sees the actionable drift table up front, instead of an unexplained
    /// per-frame byte mismatch (exit 1) on every affected topic. Fires only for
    /// topics with recorded frames AND a resolvable current hash AND a channel
    /// recorded under the current hash recipe (legacy-recipe bags are skipped).
    #[error("{}", render_schema_drift(.drifts))]
    SchemaDrift {
        /// Every drifted topic, with its recorded and current schema hashes.
        drifts: Vec<SchemaDrift>,
    },

    /// A node's cdylib failed to load during replay. Exit 3.
    #[error("failed to load node cdylib for type `{node_type}`: {reason}")]
    NodeLoad {
        /// The node type whose cdylib failed to load.
        node_type: String,
        /// The load failure reason.
        reason: String,
    },

    /// The `--tolerance` YAML failed validation. Exit 4.
    #[error("tolerance YAML is invalid: {reason}")]
    ToleranceInvalid {
        /// What was wrong with the tolerance document.
        reason: String,
    },

    /// This recording begins MID-RUN and its checkpoint
    /// cannot be applied. Exit 2 (the not-replay-grade class) because the fault
    /// is in what the RECORDING can support, not in the candidate: every arm —
    /// an incomplete anchor, a lossy re-executed topic, drifted state shape, an
    /// unseedable sequence, an ambiguous ring or run, `--strict-state` — is a
    /// statement about the bag, and the remedies all name re-recording or
    /// narrowing the replay rather than changing the node.
    ///
    /// The message is composed by
    /// [`cerulion_core::state_restore::RestoreRefusal`], which owns the
    /// vocabulary and names the offender and the fix in every arm; this variant
    /// carries it to the exit-code contract without paraphrasing it.
    #[error("{reason}")]
    StateRestore {
        /// The refusal, rendered by its own `Display`.
        reason: String,
    },

    /// A `coordination: free_run` bag whose recording begins
    /// MID-RUN (its first recorded STEP_BOUNDARY is not step 0) and holds MORE
    /// THAN ONE worker rank. Exit 2.
    ///
    /// Resume anchors on the ONE first recorded boundary, places the
    /// clock off that single value and splits each topic's pre-anchor prefix
    /// against it: three assumptions that hold for exactly one worker rank
    /// (rank 0's first boundary IS the first boundary, one clock, one stamp
    /// domain) and that a free-run recording with several ranks abolishes.
    /// A one-rank free-run bag takes the ordinary resume; the multi-rank
    /// shape is refused rather than silently mis-anchored, and per-rank
    /// anchors are not implemented.
    #[error(
        "this bag is stamped `coordination: free_run`, its recording begins MID-RUN (its \
         first recorded STEP_BOUNDARY is step {first_step}, not step 0), and it holds \
         {ranks} worker ranks. A mid-run resume anchors on the ONE first recorded boundary \
         and places the clock off that single value, which holds for exactly one worker \
         rank, and a free-run recording with several ranks abolishes it: each rank keeps \
         its own boundary stream and its own clock, so there is no single first boundary, \
         {ranks} clocks cannot be placed off one value, and a frame stamp compared against \
         it crosses clock domains. Per-rank resume anchors are not supported yet; until \
         they land, re-execute this bag from its start, or capture a deployment whose \
         graph runs in ONE process group (its mid-run captures resume)"
    )]
    FreeRunResumeUnsupported {
        /// The recording's first recorded (rank-0) STEP_BOUNDARY step.
        first_step: u64,
        /// The worker trace-manifest ranks the recording holds (always > 1
        /// here; a one-rank recording is admitted).
        ranks: usize,
    },

    /// An unexpected internal failure (panic, transport, scheduler). Exit 5.
    #[error("internal replay error: {reason}")]
    Internal {
        /// The internal failure detail.
        reason: String,
    },
}

impl ReplayError {
    /// The stable exit code for this failure class (see the module docs).
    pub fn exit_code(&self) -> u8 {
        match self {
            ReplayError::BagOpen { .. }
            | ReplayError::NotMcapBag { .. }
            | ReplayError::BagNotFinalized { .. }
            | ReplayError::BagNoSchedulerTrace
            | ReplayError::BagMissingAttachment { .. }
            | ReplayError::BagInvalidAttachment { .. }
            | ReplayError::MultiRankManifestGap { .. }
            | ReplayError::DegradedRecordingDeparture { .. }
            | ReplayError::TraceRecordUnsupported { .. }
            | ReplayError::BagGraphMismatch { .. }
            | ReplayError::BagNoStepBoundaries
            | ReplayError::RecordingInconsistent { .. }
            | ReplayError::SchemaDrift { .. }
            | ReplayError::StateRestore { .. }
            // Both free-run refusals are "this bag is not
            // replay-grade FOR THIS BINARY" — the same class as the
            // trace-format version gate they sit beside, never a candidate
            // divergence (1/6) and never a harness bug (5).
            | ReplayError::FreeRunResumeUnsupported { .. } => EXIT_NOT_REPLAY_GRADE,
            ReplayError::NodeLoad { .. } => EXIT_NODE_FAILURE,
            ReplayError::ToleranceInvalid { .. } => EXIT_TOLERANCE_INVALID,
            ReplayError::Internal { .. } => EXIT_INTERNAL,
        }
    }
}

/// The `node_ids` table decoded from a trace-manifest attachment. Extra
/// manifest keys (`rank`, `generation`) are intentionally ignored — the rank is
/// authoritative from the attachment NAME, and only the id table is consumed
/// here (for the trace `node_idx` bounds check). `pub(crate)`:
/// [`crate::replay_engine`] parses the same shape when re-deriving the
/// per-rank tables (multi-rank).
#[derive(serde::Deserialize)]
pub(crate) struct TraceManifest {
    pub(crate) node_ids: Vec<String>,
    /// ADDITIVE: each node's ordered input-name list, keyed by
    /// node id — the offline resolver for a kind-6 READ-OUTCOME record's
    /// `input_idx` (bagd stamps it from the ring manifest's input section).
    /// Absent on every manifest written before this key existed (`None`, the
    /// serde default), which the read-log verifier treats as "no input table":
    /// when kind-6 records are nevertheless present it warns loudly and
    /// disables itself for that bag, never guesses a name and never refuses
    /// the replay.
    ///
    /// The parse is LENIENT — a present-but-MALFORMED value
    /// (e.g. `"inputs": 42` in a hand-edited or corrupted bag) folds to
    /// `None` instead of failing the whole `TraceManifest` deserialization.
    /// This struct is ALSO what the PRIMARY replay gate parses for
    /// `node_ids`, so a strict parse here would turn a corrupt
    /// diagnostic-plane section into an exit-2 refusal of a bag the primary
    /// gates fully accept — the read log must never cost a replay its
    /// verdict. The verifier's disable warn names both possible causes
    /// (absent OR malformed).
    #[serde(default, deserialize_with = "lenient_inputs")]
    pub(crate) inputs: Option<std::collections::BTreeMap<String, Vec<String>>>,
    /// ADDITIVE: the read-log staging capacity the
    /// RECORDING binary declared (bagd stamps it beside `inputs`).
    /// The capacity is a format parameter — the overflow truncation shape
    /// depends on it — so the read-log verifier stands down loudly when it
    /// differs from the replaying binary's own.
    ///
    /// This key is a VERSION SENTINEL, not a capacity — the
    /// per-input `read_log_capacities` table above is authoritative.
    ///
    /// A per-input recorder stamps `0`, which exists so a binary that predates the
    /// per-input table stays correct:
    /// it parses `Some(0)`, compares against its own linked `320`, and stands
    /// the whole read log down loudly rather than verifying at a rim it cannot
    /// read. A global-rim bag carries a real global capacity (64/160/320) which this
    /// binary adopts for every stage, and a bag whose recorder derived per-input
    /// rims but had no table to write them into carries
    /// `u64::MAX`, which is above the ceiling and stands the log down. The full
    /// matrix lives in `replay_engine::classify_recorded_staging`.
    ///
    /// Absent on every pre-stamp bag; LENIENT for the same reason as `inputs`
    /// (this struct is also the primary gate's `node_ids` parse — a corrupt
    /// diagnostic-plane key must never cost a replay its verdict), but NOT
    /// folded: see [`RecordedCapacityStamp`], the one key whose absent arm
    /// trusts.
    #[serde(default, deserialize_with = "lenient_capacity")]
    pub(crate) read_log_capacity: RecordedCapacityStamp,
    /// ADDITIVE: the ring's PUBLISHER table (32-hex
    /// `UniquePublisherId` → `[node, output]`), which is what turns a bagged
    /// `READ_OUTCOME_PRODUCER` annotation's 64-bit token back into a name
    /// offline (the read log's `(producer, seq)` extension on
    /// `multi_publisher_topics` edges). Absent on every manifest that predates the table, and
    /// on any manifest that degraded past the 64 KiB budget (`None`), which
    /// the read-log verifier treats as "no publisher table": a token it
    /// cannot resolve renders as `foreign(...)` and stands that EDGE down
    /// loudly, never guesses a producer and never refuses the replay.
    ///
    /// LENIENT for the same reason as `inputs` / `read_log_capacity` — this
    /// struct is ALSO what the PRIMARY replay gate parses for `node_ids`, so
    /// a corrupt diagnostic-plane key must never cost a replay its verdict.
    #[serde(default, deserialize_with = "lenient_publishers")]
    pub(crate) publishers: Option<std::collections::BTreeMap<String, Vec<String>>>,
    /// ADDITIVE: each node's per-stage staging RIMS as
    /// `[input_idx, role, capacity]` triples — the AUTHORITATIVE table; the
    /// scalar `read_log_capacity` is only a version sentinel.
    ///
    /// The replay adopts these rims so both sides truncate
    /// identically. Keyed on `(input_idx, role)` and never on position: an input
    /// under the Separate or legacy-`Sync` discipline carries TWO stages that
    /// share an index.
    ///
    /// LENIENT for the same reason as `inputs` / `publishers` — this struct is
    /// also the PRIMARY gate's `node_ids` parse. The fold is SAFE here because
    /// the absent arm is the conservative one: no table means "this bag
    /// declares no rims", which stands the read-log verifier down rather than
    /// adopting a rim nobody stated (see `RecordedCapacityStamp` for the one key
    /// where the absent arm trusts).
    #[serde(default, deserialize_with = "lenient_stage_capacities")]
    pub(crate) read_log_capacities: RecordedCapacityTable,
}

/// Deserialize the manifest's `inputs` value
/// SOFTLY — any shape that is not a `{node: [input, ...]}` map folds to
/// `None` (the verifier's loud-disabled state), never a parse error.
/// The fold is not fully silent — a
/// present-but-unparsable value leaves a `debug!` naming the actual
/// malformed shape (the verifier's later disable WARN says "absent OR
/// malformed"; this breadcrumb is what tells a debugging operator WHICH,
/// and what the value actually looked like).
///
/// Folding present-but-malformed into the absent
/// arm is safe HERE and is not a second instance of the `read_log_capacity`
/// bug, because for this key the absent arm is the CONSERVATIVE one — it stands
/// the verifier (or the edge) DOWN. The discriminator for the whole class is
/// "does the absent arm trust or distrust?"; see [`RecordedCapacityStamp`],
/// which is the only key whose absent arm trusts.
fn lenient_inputs<'de, D>(
    deserializer: D,
) -> Result<Option<std::collections::BTreeMap<String, Vec<String>>>, D::Error>
where
    D: serde::Deserializer<'de>,
{
    use serde::Deserialize as _;
    let value = serde_json::Value::deserialize(deserializer)?;
    match serde_json::from_value(value.clone()) {
        Ok(parsed) => Ok(parsed),
        Err(e) => {
            tracing::debug!(
                error = %e,
                shape = %truncate_json_for_log(&value),
                "trace manifest `inputs` key is PRESENT but not a {{node: [input, ...]}} \
                 map — folding to None (the read-log verifier will disable loudly)"
            );
            Ok(None)
        }
    }
}

/// The lenient twin for `publishers`: any shape that is not a
/// `{hex-id: [node, output]}` map folds to `None` (the verifier's
/// no-publisher-table state, where every producer token resolves to
/// `foreign`), with the same debug breadcrumb posture as [`lenient_inputs`].
///
/// Safe for the same reason as [`lenient_inputs`]:
/// this key's absent arm is the CONSERVATIVE one (an unresolvable token stands
/// its EDGE down loudly), so a malformed value that folds into it distrusts
/// rather than trusts. See [`RecordedCapacityStamp`] for the one key where that
/// is not true.
fn lenient_publishers<'de, D>(
    deserializer: D,
) -> Result<Option<std::collections::BTreeMap<String, Vec<String>>>, D::Error>
where
    D: serde::Deserializer<'de>,
{
    use serde::Deserialize as _;
    let value = serde_json::Value::deserialize(deserializer)?;
    match serde_json::from_value(value.clone()) {
        Ok(parsed) => Ok(parsed),
        Err(e) => {
            tracing::debug!(
                error = %e,
                shape = %truncate_json_for_log(&value),
                "trace manifest `publishers` key is PRESENT but not a \
                 {{hex-id: [node, output]}} map — folding to None (every producer token \
                 in this bag will resolve to `foreign` and stand its edge down)"
            );
            Ok(None)
        }
    }
}

/// One node's per-stage staging rims, as the manifest carries
/// them — `(input_idx, role, capacity)` rows keyed on the PAIR, never on
/// position (an input under the Separate or legacy-`Sync` discipline carries two
/// stages that share an index).
pub(crate) type StageCapacityRows = Vec<(u16, u8, u32)>;

/// The whole manifest's staging-rim table, node id to its rows.
pub(crate) type StageCapacityTable = std::collections::BTreeMap<String, StageCapacityRows>;

/// The lenient twin for `read_log_capacities` — any shape that is
/// not a `{node: [[input_idx, role, capacity], ...]}` map folds to `None` (the
/// verifier's no-capacity-table state), with the same debug breadcrumb posture
/// as [`lenient_inputs`].
///
/// Safe to fold, by the class discriminator: this key's absent arm is the
/// CONSERVATIVE one — a bag that declares no rims stands the verifier down
/// rather than having a rim guessed for it.
/// The staging table in three states, for the same
/// reason the scalar stamp has three ([`RecordedCapacityStamp`]).
///
/// Folding a MALFORMED table into `Absent` let a legacy scalar sitting beside it
/// be adopted — the bag says "here are my per-edge rims", the rims are
/// unreadable, and the verifier quietly falls back to a global that describes a
/// different plane. Malformed must stand down BEFORE any legacy adoption is
/// considered.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub(crate) enum RecordedCapacityTable {
    /// No key at all. Either a bag that predates this table or a recorder that
    /// had no rows to stamp and truthfully said so rather than emitting `{}`.
    #[default]
    Absent,
    /// Present but unreadable: not a `{node: [[idx, role, cap], ..]}` map, or
    /// carrying DUPLICATE `(input_idx, role)` keys. Carries the raw shape so the
    /// stand-down can name what it found.
    Malformed(String),
    /// A usable table.
    Present(StageCapacityTable),
}

fn lenient_stage_capacities<'de, D>(deserializer: D) -> Result<RecordedCapacityTable, D::Error>
where
    D: serde::Deserializer<'de>,
{
    use serde::Deserialize as _;
    let value = serde_json::Value::deserialize(deserializer)?;
    if value.is_null() {
        return Ok(RecordedCapacityTable::Absent);
    }
    let shape = truncate_json_for_log(&value);
    match serde_json::from_value::<StageCapacityTable>(value) {
        Ok(parsed) => {
            // Duplicate `(input_idx, role)` rows are
            // MALFORMED, not "last one wins". The replay keys its adoption map
            // on that pair, so a duplicate silently picks whichever row happened
            // to land last — a rim the recording may never have used.
            for (node, rows) in &parsed {
                let mut seen = std::collections::HashSet::new();
                for (idx, role, _) in rows {
                    if !seen.insert((*idx, *role)) {
                        tracing::debug!(
                            node_id = %node,
                            input_idx = *idx,
                            role_byte = *role,
                            "trace manifest `read_log_capacities` declares the SAME \
                             (input_idx, role) twice for one node — the pair is the key, \
                             so a duplicate makes the table unreadable"
                        );
                        return Ok(RecordedCapacityTable::Malformed(format!(
                            "duplicate (input_idx {idx}, role {role}) rows for node `{node}`"
                        )));
                    }
                }
            }
            Ok(RecordedCapacityTable::Present(parsed))
        }
        Err(e) => {
            tracing::debug!(
                error = %e,
                shape = %shape,
                "trace manifest `read_log_capacities` key is PRESENT but not a \
                 node-to-[[input_idx, role, capacity], ..] map — the read-log verifier \
                 stands down rather than falling back to a rim this bag did not state"
            );
            Ok(RecordedCapacityTable::Malformed(shape))
        }
    }
}

/// The read-log staging stamp in three states, because two of
/// them must not be confused.
///
/// The other lenient keys (`inputs`, `publishers`) may fold a
/// present-but-malformed value into their absent arm safely, because for those
/// the absent arm is the CONSERVATIVE one: no input table stands the whole
/// verifier down, an unresolvable publisher token stands its edge down. This
/// key is the one place where the absent arm is the TRUSTING one — absent means
/// "a pre-stamp bag, compare normally" — so folding a corrupt stamp into it
/// makes the verifier compare truncation shapes it has just been told it cannot
/// interpret, and report the difference as node divergence. That is a wrong
/// answer about the candidate, not a missing one.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub(crate) enum RecordedCapacityStamp {
    /// No key at all: every pre-stamp bag. Compare normally (back-compat).
    #[default]
    Absent,
    /// The key is PRESENT but is not a `u64`. Carries the raw shape (truncated)
    /// so the stand-down can name what it actually found.
    Malformed(String),
    /// A well-formed stamp.
    Present(u64),
}

/// The lenient twin for `read_log_capacity`.
///
/// Still LENIENT in the sense that matters — a malformed value never fails the
/// whole `TraceManifest` deserialization, because this struct is also the
/// PRIMARY gate's `node_ids` parse and a corrupt diagnostic key must never cost
/// a replay its verdict. The malformed case is
/// DISTINGUISHABLE from the absent one: it folds to
/// [`RecordedCapacityStamp::Malformed`], which the read-log verifier stands
/// down on rather than treating as "no check".
fn lenient_capacity<'de, D>(deserializer: D) -> Result<RecordedCapacityStamp, D::Error>
where
    D: serde::Deserializer<'de>,
{
    use serde::Deserialize as _;
    let value = serde_json::Value::deserialize(deserializer)?;
    // An explicit JSON `null` is the absent arm: serde reaches this function
    // only when the KEY is present, and `"read_log_capacity": null` is how a
    // hand-edited bag spells "no stamp" rather than a corrupt one.
    if value.is_null() {
        return Ok(RecordedCapacityStamp::Absent);
    }
    match serde_json::from_value::<u64>(value.clone()) {
        Ok(parsed) => Ok(RecordedCapacityStamp::Present(parsed)),
        Err(e) => {
            let shape = truncate_json_for_log(&value);
            tracing::debug!(
                error = %e,
                shape = %shape,
                "trace manifest `read_log_capacity` key is PRESENT but not an integer — \
                 the read-log verifier will stand down rather than compare truncation \
                 shapes it cannot interpret"
            );
            Ok(RecordedCapacityStamp::Malformed(shape))
        }
    }
}

/// Turn a `graph.yaml` attachment parse failure into something a user can act
/// on.
///
/// Graph YAML denies unknown fields (deliberately),
/// and a legacy bag can embed the ON-DISK
/// `graphs/<name>.yaml` rather than the EFFECTIVE config the run executed. A
/// graph file of that era could still carry a legacy `policy:` block,
/// dead configuration, so such a bag refuses
/// to resim with a raw serde string:
///
/// ```text
/// graph YAML did not parse: unknown field `policy`, expected one of `prefix`, …
/// ```
///
/// which names no bag, no remedy, and reads like the user's fault. The refusal
/// stands (a bag whose embedded graph does not parse is not replay-grade, and
/// the exit code is unchanged at 2) but it has to SAY what happened.
///
/// Only the unknown-field shape is rewritten. Any other parse error is real
/// YAML damage and keeps its original text — this is a diagnosis, not a
/// blanket excuse.
/// Keys that are EVIDENCE of a legacy graph shape rather than of a typo.
///
/// `policy:` is the legacy trigger block. It was deleted from the format,
/// it is the key a graph file of that era actually carries, and no current
/// tool writes it — so finding it in an embed says something about WHEN the
/// bag was recorded. A misspelling says nothing of the sort.
const LEGACY_GRAPH_KEYS: &[&str] = &["policy"];

fn describe_graph_attachment_parse_failure(err: &str) -> String {
    let Some(key) = unknown_field_key(err) else {
        return format!("graph YAML did not parse: {err}");
    };
    let head = format!(
        "graph YAML did not parse: unknown key `{key}` in the bag's embedded \
         `{GRAPH_ATTACHMENT}` attachment ({err})."
    );
    // The remedy is the same either way — only the DIAGNOSIS is conditional.
    //
    // This names a command to RUN.
    // It deliberately does not say "in place": `bag migrate` writes a
    // NEW bag and never touches the recording, which is the property that makes
    // it safe to suggest at all.
    let remedy = "FIX: run `cerulion bag migrate <bag>`. It writes a NEW bag whose \
                  embedded graph has the undefined keys removed — listing every one of them \
                  first, and asking before it writes anything — and leaves this recording \
                  untouched.\n\nOR: re-record from the current graph, which is the better \
                  answer whenever the run can simply be done again.";

    if LEGACY_GRAPH_KEYS.contains(&key.as_str()) {
        // The obsolete key identifies the embedded graph compatibility problem;
        // it does not establish when this bag was recorded.
        format!(
            "{head}\n\nThat key is the legacy `policy:` block. Older recorders embedded the on-disk \
             `graphs/<name>.yaml` rather than the EFFECTIVE config the run executed, so a key \
             the format no longer defines travels into the bag and is refused now that graph \
             YAML checks every key. The recording itself is fine; its embedded graph is \
             stale.\n\n{remedy}"
        )
    } else {
        // NO era evidence. Saying "this is an old-format bag" here would be a
        // fabricated diagnosis: the same message is produced by a CURRENT bag
        // whose attachment was hand-edited or simply typo'd, and telling that
        // user their recording is stale sends them to re-record a bag that was
        // never the problem. State what is known, offer the era as one
        // POSSIBILITY, and stop.
        format!(
            "{head}\n\nThe key is not one the graph format defines. That MAY mean the bag \
             embeds the original on-disk `graphs/<name>.yaml` rather than the effective \
             config, allowing a since-removed setting to travel into it, or it may simply \
             be a typo or a hand-edited attachment — this refusal \
             cannot tell which, so it does not guess. Compare the key against the accepted \
             list above.\n\n{remedy}"
        )
    }
}

#[cfg(test)]
mod graph_attachment_refusal_tests {
    use super::*;

    #[test]
    fn a_legacy_key_refusal_names_the_key_the_shape_and_the_remedy() {
        let err = "unknown field `policy`, expected one of `prefix`, `nodes` at line 7 column 5";
        let msg = describe_graph_attachment_parse_failure(err);

        // The KEY, as a whole backticked token — the thing the user has to find.
        assert!(
            msg.contains("`policy`"),
            "must name the offending key: {msg}"
        );
        // The LOCATION: which artifact inside the bag, not just "graph YAML".
        assert!(
            msg.contains("graph.yaml") && msg.contains("attachment"),
            "must say WHERE in the bag: {msg}"
        );
        // The SHAPE sentence explains how an obsolete key can enter a bag.
        assert!(
            msg.contains("embedded the on-disk") && msg.contains("EFFECTIVE config"),
            "must explain the embedded-graph shape: {msg}"
        );
        // The diagnosis names the obsolete key and the graph compatibility issue.
        assert!(
            msg.contains("legacy `policy:` block") && msg.contains("embedded graph is stale"),
            "must identify the legacy key and stale embedded graph: {msg}"
        );
        // The REMEDY names the verb and the issue.
        assert!(
            msg.contains("cerulion bag migrate") && msg.contains("writes a NEW bag"),
            "must name the remedy and its issue: {msg}"
        );
        // `bag migrate` is a real verb, so the message must read as a command to
        // RUN — not as an issue to watch. Two tokens that would mark it
        // unavailable are forbidden: a user who reads "PLANNED" will not try the
        // one thing that works.
        assert!(
            !msg.contains("PLANNED") && !msg.contains("NOT a subcommand yet"),
            "`bag migrate` is a real verb — the message must not call it planned: {msg}"
        );
        // ...and the second path stays named, because for a graph that has
        // moved on it is the better answer.
        assert!(
            msg.contains("re-record"),
            "must keep the re-record path: {msg}"
        );
        // It must NOT claim the rewrite happens in place: `bag migrate` writes
        // a NEW bag, and a user told otherwise would reasonably fear for their
        // recording.
        assert!(
            !msg.contains("in place"),
            "migration never rewrites a bag in place: {msg}"
        );
        // The original serde text survives, so the detail is not lost.
        assert!(
            msg.contains("line 7"),
            "must keep the underlying error: {msg}"
        );
    }

    #[test]
    fn a_typod_key_is_not_diagnosed_as_a_stale_recording() {
        // The false-diagnosis case: a CURRENT-format bag whose attachment was
        // hand-edited or simply misspelled produces the same serde error shape
        // as a legacy one. Claiming "this is an old-format bag" there is a
        // fabricated story that sends the user to re-record a recording that
        // was never the problem.
        let err = "unknown field `dpeth`, expected one of `prefix`, `nodes` at line 4 column 7";
        let msg = describe_graph_attachment_parse_failure(err);

        assert!(msg.contains("`dpeth`"), "must still name the key: {msg}");
        // NO era CLAIM. The word may appear inside the hedged sentence, so the
        // pin is on the asserting forms, not on the token.
        assert!(
            !msg.contains("Older recorders embedded the on-disk"),
            "must not assert the on-disk-graph era without evidence: {msg}"
        );
        assert!(
            !msg.contains("embedded graph is stale"),
            "must not claim the recording is stale without evidence: {msg}"
        );
        assert!(
            msg.contains("MAY mean") && msg.contains("typo"),
            "must offer the era as a possibility and name the alternative: {msg}"
        );
        // The remedy is unconditional — it is the same either way.
        assert!(msg.contains("re-record") && msg.contains("cerulion bag migrate"));
    }

    #[test]
    fn an_ordinary_yaml_error_is_left_alone() {
        // ANTI-TAUTOLOGY: without this, a function that returned the legacy-shape
        // paragraph unconditionally would pass the arm above. Real YAML damage
        // is not a legacy-shape bag and must not be told to run `bag migrate`.
        let err = "invalid type: string \"x\", expected a sequence at line 3 column 1";
        let msg = describe_graph_attachment_parse_failure(err);
        assert_eq!(msg, format!("graph YAML did not parse: {err}"));
        assert!(
            !msg.contains("bag migrate"),
            "must not offer the wrong remedy: {msg}"
        );
    }

    #[test]
    fn the_key_reader_reads_the_key_and_nothing_else() {
        assert_eq!(
            unknown_field_key("unknown field `dpeth`, expected").as_deref(),
            Some("dpeth")
        );
        assert_eq!(unknown_field_key("some other failure"), None);
        // An empty pair of backticks is not a key.
        assert_eq!(unknown_field_key("unknown field ``, expected"), None);
    }
}

/// A bounded rendering of a JSON value for the lenient-parse breadcrumbs — a
/// corrupt bag could carry megabytes under the key, and a diagnostic
/// breadcrumb must never balloon the log.
fn truncate_json_for_log(value: &serde_json::Value) -> String {
    let mut rendered = value.to_string();
    const MAX: usize = 120;
    if rendered.len() > MAX {
        let cut = (0..=MAX).rev().find(|i| rendered.is_char_boundary(*i));
        rendered.truncate(cut.unwrap_or(0));
        rendered.push_str("… (truncated)");
    }
    rendered
}

/// Knobs for [`run_replay`] (the `--duration` / `--report` flags +
/// the `--tolerance` flag).
#[derive(Debug, Default, Clone)]
pub struct ReplayOptions {
    /// `--duration D`, in NANOSECONDS of BAG TIME. Stop
    /// re-executing once a recorded boundary's clock target passes the run's
    /// bag-time ORIGIN plus this. `None` re-executes the whole recorded trace.
    ///
    /// It replaces the deleted `--max-ticks` step cap. The origin is the
    /// MINIMUM first-boundary target across the ranks, so they share an ORIGIN
    /// rather than an epoch VALUE: one bound names one wall interval for all of
    /// them, and a later-booting rank loses more of its own tail. A step count
    /// has no shared axis at all — each rank numbers its own steps — which is
    /// what the bound in TIME buys.
    pub duration_bound_ns: Option<u64>,
    /// Write the [`ReplayOutcome`] JSON to this path (`--report`).
    pub report_path: Option<PathBuf>,
    /// Path to the tolerance YAML (`--tolerance`). Parsed +
    /// validated as a PRE-FLIGHT gate (exit 4) before the engine constructs the
    /// runtime. `None` = the default byte-exact comparison. A validated document
    /// relaxes each nominated per-field override by its metric while the rest of
    /// the frame stays byte-exact; a tolerance that only relaxes to `bit_exact`
    /// (or targets no field) leaves the replay byte-identical.
    pub tolerance_path: Option<PathBuf>,
    /// `--strict-state`. On a replay that RESUMES from a
    /// recorded checkpoint, refuse if any executed node declares no restorable
    /// state rather than running it from its constructor.
    ///
    /// It governs COVERAGE, not drift: every state-shape difference is already
    /// terminal under the state encoding, so a flag that governed drift would
    /// be a no-op. What it refuses is the reading a default replay takes
    /// silently — "this node has no `CerulionState` yet, so it starts fresh" —
    /// which is correct for a stateless node and a staged-rollout gap for one
    /// that simply has no derive yet, and only the operator can tell which.
    ///
    /// Inert on a from-start replay (nothing is restored there by design).
    pub strict_state: bool,
}

/// Run `cerulion bag play --resim` end-to-end against `bag`.
///
/// First the replay-grade **entry gates** in order (open, completeness,
/// scheduler-trace presence, graph attachment, trace manifest, env.json) — any
/// failure is a typed [`ReplayError`] (exit 2–5). Then the gated artifacts are
/// handed to [`crate::replay_engine::run_engine`], which loads the CURRENT
/// workspace's node cdylibs (a missing type → [`ReplayError::NodeLoad`], exit
/// 3), re-executes the graph on the recorded-clock re-advance, injects recorded
/// external frames, captures graph-produced frames, and diffs them bit-exact.
///
/// Returns the engine's [`ReplayOutcome`] on any successfully-executed replay —
/// `outcome.passed` distinguishes a clean run (exit 0) from one with data
/// violations (exit 1). The CLI maps this via `outcome.passed`; the human
/// verdict is [`crate::replay_engine::render_verdict`].
pub fn run_replay(bag: &Path, opts: ReplayOptions) -> Result<ReplayOutcome, ReplayError> {
    tracing::debug!(bag = ?bag, "replay: opening bag");

    // 1. Open — read the whole file. Missing/unreadable → BagOpen (exit 2).
    let reader = BagReader::open(bag).map_err(|source| ReplayError::BagOpen {
        path: bag.to_path_buf(),
        source,
    })?;

    // 1b. Fail fast on a file that is not an MCAP bag at all, with a clean
    //     "wrong file" message — otherwise `recover_messages` would fold the bad
    //     start-magic into a `TornTail` completeness and mislabel it "not
    //     finalized" below.
    if !reader.bytes().starts_with(&cerulion_bag::record::MAGIC) {
        return Err(ReplayError::NotMcapBag {
            path: bag.to_path_buf(),
        });
    }

    // 2. Completeness — stream every complete record and inspect how the bag
    //    ENDED, WITHOUT collecting any payload (the memory-lean gate: peak is one
    //    message, not the whole file, where a `recover_messages` call would
    //    hold every payload at once). A non-Finalized bag is
    //    refused (exit 2) naming the state. A hard error (not even MCAP) is
    //    corrupt-bag → BagOpen.
    let completeness = reader
        .completeness()
        .map_err(|source| ReplayError::BagOpen {
            path: bag.to_path_buf(),
            source,
        })?;
    if !completeness.is_finalized() {
        return Err(ReplayError::BagNotFinalized {
            completeness: describe_completeness(&completeness),
        });
    }

    // 3. Scheduler-trace presence — strict replay is driven by the recorded
    //    trace. An empty channel is refused with the precise re-record message.
    //    A wrong-sized trace payload is data corruption (exit 2 via BagOpen).
    //
    //    Memory scaler: the channel is STREAMED (`trace_records`), never
    //    collected — a 300 GB-scale bag's trace is ~40 B × hundreds of millions
    //    of records, so a collected `scheduler_trace()` Vec would dominate replay's
    //    heap. This walk counts + decode-validates every record; the
    //    record-TYPE gate below and the engine's passes each re-walk the mapped
    //    channel (sequential re-walks are the intended pattern — the framing
    //    walk is cheap and the map is page-cache-hot after the first).
    let bag_open = |source| ReplayError::BagOpen {
        path: bag.to_path_buf(),
        source,
    };
    let mut trace_record_count = 0usize;
    {
        // A full data-section walk (trace records interleave the whole
        // section) — a pre-loop pass that peaked RSS at scale. Advise behind
        // this pass's own frontier (per-pass cursor, never the replay loop's
        // shared cursor).
        let mut it = reader.trace_records().map_err(bag_open)?;
        let mut advise = cerulion_bag::AdviseCursor::new();
        while let Some(item) = it.next() {
            item.map_err(bag_open)?;
            trace_record_count += 1;
            reader.advise_evict_behind_scoped(&mut advise, it.file_frontier());
        }
    }
    if trace_record_count == 0 {
        return Err(ReplayError::BagNoSchedulerTrace);
    }

    // 4. Graph attachment — must exist, be UTF-8, parse, and validate.
    let graph_att = require_attachment(&reader, bag, GRAPH_ATTACHMENT)?;
    let graph_yaml =
        std::str::from_utf8(&graph_att.data).map_err(|e| ReplayError::BagInvalidAttachment {
            name: GRAPH_ATTACHMENT.to_string(),
            reason: format!("not valid UTF-8: {e}"),
        })?;
    // NOTE: `parse_graph` fills a missing `prefix:` with the REPLAY host's
    // hostname — and this parse is NOT validation-only: THIS `config` is the one
    // moved into `ReplayInputs`, so `replay_engine::classify_topics` resolves
    // produced/consumed topic names under that prefix and a name that matches no
    // recorded channel is a `BagGraphMismatch` (exit 2). The fill is NOT
    // "benign HERE" on the strength of the parse being
    // validation-only; it is not, and that reading hides a real cross-machine hazard.
    //
    // What makes it safe is upstream, not here: the bag embeds the
    // EFFECTIVE config (`graph_cmd::render_effective_graph_yaml`), whose prefix
    // is always the RESOLVED one the recording named its channels under, so the
    // fill has nothing to fill. It still fires for an older bag, recorded
    // from a graph file with no `prefix:` line, which replays only on a host
    // whose hostname matches the recorder's — re-record to fix.
    let mut config = cerulion_core::graph::parse_graph(graph_yaml).map_err(|e| {
        ReplayError::BagInvalidAttachment {
            name: GRAPH_ATTACHMENT.to_string(),
            reason: describe_graph_attachment_parse_failure(&e.to_string()),
        }
    })?;
    cerulion_core::graph::validate_graph(&config).map_err(|e| {
        ReplayError::BagInvalidAttachment {
            name: GRAPH_ATTACHMENT.to_string(),
            reason: format!("graph failed validation: {e}"),
        }
    })?;

    // `ros2:` graph entries: the bag embeds a mixed graph's
    // ROS 2 entries VERBATIM — they are part of the run the bag describes —
    // and a resim re-executes NATIVE nodes only. Lift them out here (the
    // same split `graph run` performs) with ONE warn naming them: never a
    // BagGraphMismatch, never a respawn. Their recorded topics classify like
    // any other unmodelled channel.
    let ros2_entries_skipped: Vec<String> =
        config.take_ros2_nodes().into_iter().map(|n| n.id).collect();
    if !ros2_entries_skipped.is_empty() {
        tracing::warn!(
            entries = %ros2_entries_skipped.join(", "),
            "resim: ros2 entries in the recorded graph are not re-executed — their topics are \
             recorded inputs (run the live graph with `cerulion graph run` to spawn them)"
        );
    }

    // An embed has no file to stem from, so under the file-stem identity rule
    // a bag recorded from a newer graph arrives nameless and every resim
    // of it called itself `unnamed`. The identity was never lost — the recorder
    // stems the bag BY it — so read it back off the bag's own file name. See
    // `resolve_resim_identity` for why this is a FALLBACK and what it declines.
    let resolution = resolve_resim_identity(&config.identity, bag);
    if let ResimIdentity::Adopted(stem) = &resolution {
        config.identity = stem.clone();
    }
    // ONE line, on every arm, and it reads `config.identity()` — the identity
    // ACTUALLY IN FORCE — rather than the candidate the resolver returned.
    //
    // That is load-bearing, not a style choice: were the line to log
    // the candidate from inside the adopt arm, code that LOGGED the stem
    // and never assigned it would print the right line while every resim still ran
    // as `unnamed`, and the gate arm that exists to catch exactly that would pass.
    // Reading the field back is what makes the log an observable of the
    // ASSIGNMENT (count where the change is APPLIED, not where it
    // is detected).
    //
    // `debug!` on every arm: a resim that resolved its own name is not an
    // operator problem, and the declined arm is cosmetic too (the run works,
    // it is just called `unnamed`) — a `warn!` there would be cry-wolf.
    match &resolution {
        ResimIdentity::Declared => tracing::debug!(
            graph = %config.identity(),
            bag = %bag.display(),
            "resim: the bag's embedded `graph.yaml` declares a name, so this run keeps it"
        ),
        ResimIdentity::Adopted(_) => tracing::debug!(
            graph = %config.identity(),
            bag = %bag.display(),
            "resim: the bag's embedded `graph.yaml` names no graph (a created graph carries \
             no `name:` key), so this run is named by the BAG FILE"
        ),
        ResimIdentity::Declined(cause) => tracing::debug!(
            graph = %config.identity(),
            bag = %bag.display(),
            reason = %cause.reason(),
            "resim: the bag's embedded `graph.yaml` names no graph and the bag file \
             cannot supply one either, so this run stays unnamed"
        ),
    }

    // The CURRENT workspace build is the candidate — resolve from the CWD (the
    // whole point of the self-contained model). A non-workspace CWD falls back
    // to the CWD so `find_cdylib` produces the precise per-node NodeLoad. Hoisted
    // BEFORE the tolerance gate (which resolves field paths against the
    // workspace's `schemas/` set) and reused for the cdylib candidate below.
    let workspace_root = std::env::current_dir()
        .ok()
        .and_then(|cwd| {
            crate::workspace::CerulionWorkspace::discover(&cwd)
                .ok()
                .map(|ws| ws.root)
        })
        .or_else(|| std::env::current_dir().ok())
        .unwrap_or_else(|| PathBuf::from("."));

    // 4b. The `--tolerance` PRE-FLIGHT gate (exit 4). Parse +
    //     validate the tolerance YAML NOW — after the bag's graph attachment
    //     has loaded (the topic/field registry is derived from it) but BEFORE
    //     the engine constructs the runtime (`run_engine` → `build_runtime`),
    //     so a bad tolerance document fails FAST without loading a node cdylib
    //     or touching the transport. Any failure is `ToleranceInvalid` (exit 4).
    let tolerance = validate_tolerance(opts.tolerance_path.as_deref(), &config, &workspace_root)?;

    // 5. Trace manifests — EVERY worker rank's node-id table (index == rank,
    //    contiguity-validated); STRICT record gate on the SAME single pass:
    //    replay accepts FIRE records (whose node_idx must index the record's
    //    OWN rank's manifest table — the rank is bagd-stamped in `reserved`)
    //    and STEP_BOUNDARY records (the per-step clock targets replay
    //    re-advances to — their node_idx is 0/meaningless, so no bounds check;
    //    their SEMANTIC validation — one per step per rank, non-decreasing — is
    //    the engine's cross-check) and READ_OUTCOME records
    //    (trace_format 3, informational; node_idx = the consumer, bounds-checked
    //    like a FIRE's). A DEPARTURE (fault) boundary or a
    //    departure-ring-sentinel-stamped record marks a DEGRADED recording and
    //    is refused PRE-canonicalization; a rank stamp with no manifest, and
    //    invalid/reserved record types, are refused too. A well-formed
    //    multi-rank bag PROCEEDS to the engine (per-rank demux
    //    + k-way per-step merge in `replay_engine`).
    let rank_tables = load_trace_manifests(&reader, bag)?;
    // Re-walk the streamed trace channel (never collected — see step 3); `index`
    // counts trace-channel records in file order, exactly a collected Vec's index.
    // Another full data-section walk — advise behind its own per-pass
    // frontier (never the replay loop's shared cursor).
    // Node counts by rank — the classifier's view of `rank_tables`, built once
    // rather than per record.
    let rank_counts: Vec<usize> = rank_tables.iter().map(|(_, ids)| ids.len()).collect();
    let mut trace_it = reader.trace_records().map_err(bag_open)?;
    let mut gate_advise = cerulion_bag::AdviseCursor::new();
    let mut index = 0usize;
    while let Some(item) = trace_it.next() {
        let rec = item.map_err(bag_open)?;
        // DEGRADED-recording gate, hoisted PRE-canonicalization: a
        // departure/fault boundary — either a RECORD_TYPE_DEPARTURE record OR
        // any record bagd stamped with the supervisor departure-ring sentinel
        // rank (`reserved == u32::MAX`; bagd stamps the ring's rank into
        // `reserved`) — means a peer worker was lost mid-run. Fault replay
        // lands post-launch, so this is refused up front. Runs on the SAME
        // single pass as the record-type gate below (no extra walk).
        //
        // The CONDITION is `TraceRingRecord::is_departure_boundary`,
        // shared with the Flashback recorder rather than written twice. A
        // capture decides whether it may claim `resimmable` by asking the same
        // question this gate asks, and two copies of one refusal rule is how a
        // capture comes to be stamped resimmable and then refused here.
        if rec.is_departure_boundary() {
            let detail = if rec.record_type == RECORD_TYPE_DEPARTURE {
                format!("a RECORD_TYPE_DEPARTURE (record_type {RECORD_TYPE_DEPARTURE}) record")
            } else {
                format!(
                    "a record stamped with the supervisor departure-ring sentinel rank \
                     (reserved == {DEPARTURE_RING_RANK})"
                )
            };
            return Err(ReplayError::DegradedRecordingDeparture { index, detail });
        }
        // Rank-stamp gate: every surviving record's
        // bagd-stamped rank (`reserved`) must name a worker manifest — a stamp
        // with no manifest is corruption (the engine's per-rank demux would
        // have no table/cursor for it). Checked BEFORE the record-type match so
        // a foreign-rank record is diagnosed as the rank fault it is.
        // Read `reserved` as a RANK via `.rank()` (masks off the
        // discard bit). A discard-marked FIRE record carries `rank | DISCARD_BIT
        // >= 2^31`; an UNMASKED read would spuriously exceed `rank_tables.len()`
        // and refuse a valid new-format bag. The DEPARTURE gate above stays RAW
        // (a real departure record can never be a discard-marked FIRE). An
        // out-of-range rank AFTER masking is still rejected (kills a marker on a
        // truly foreign-rank record).
        // The four record-level refusals are one SHARED condition
        // (`cerulion_core::trace_ring::classify_trace_record`), asked here in
        // this gate's own order and mapped back onto this gate's own errors —
        // which carry the record INDEX and the manifest NAME the pure classifier
        // deliberately does not. A Flashback capture asks the identical question
        // before it may claim `resimmable`, so a refusal added here cannot drift
        // away from the judge: the same move `is_departure_boundary` made for
        // the gate above.
        //
        // `.rank()` masking, the rank-before-type order and the record-type
        // arms all live in the classifier; see its docs for why each is
        // where it is.
        if let Some(fault) = cerulion_core::trace_ring::classify_trace_record(&rec, &rank_counts) {
            use cerulion_core::trace_ring::TraceRecordFault;
            return Err(match fault {
                TraceRecordFault::ForeignRank { rank, ranks_known } => {
                    ReplayError::RecordingInconsistent {
                        detail: format!(
                            "trace record {index} is stamped with rank {rank} (reserved) but the \
                             bag carries worker trace manifests only for ranks 0..={}",
                            ranks_known - 1
                        ),
                    }
                }
                TraceRecordFault::NodeIdxOutOfRange {
                    rank,
                    node_idx,
                    nodes,
                    record_type,
                } => ReplayError::BagInvalidAttachment {
                    name: rank_tables[rank as usize].0.clone(),
                    reason: format!(
                        "a scheduler-trace {} record references node_idx {node_idx} but the \
                         manifest lists only {nodes} node id(s)",
                        cerulion_core::trace_ring::trace_record_noun(record_type)
                    ),
                },
                TraceRecordFault::ZeroedRecord => ReplayError::TraceRecordUnsupported {
                    index,
                    record_type: 0,
                    reason: "record_type 0 is an invalid (zeroed) trace record — the \
                             recording is corrupt"
                        .to_string(),
                },
                TraceRecordFault::UnsupportedRecordType { record_type } => {
                    ReplayError::TraceRecordUnsupported {
                        index,
                        record_type,
                        reason: format!(
                            "record_type {record_type} is a reserved kind unknown to this \
                             Cerulion version — the bag was written by a newer or foreign \
                             writer; upgrade cerulion to replay it"
                        ),
                    }
                }
            });
        }
        reader.advise_evict_behind_scoped(&mut gate_advise, trace_it.file_frontier());
        index += 1;
    }

    // The engine consumes rank-0's table via `ReplayInputs::node_ids` and
    // self-derives the higher-rank tables from the same mapped reader
    // (`replay_engine::load_rank_tables`) — every table was gate-validated
    // above, so the engine-side re-read trusts this pass.
    let node_ids = rank_tables
        .into_iter()
        .next()
        .expect("load_trace_manifests guarantees rank 0 exists")
        .1;

    // 6. env.json — OPTIONAL, but if present it must be a JSON object. Retain
    //    the bytes so the engine can WARN on env divergence (names always;
    //    values for plain entries; fnv64 for redacted ones). The manifest's
    //    `node_ids` table (already bounds-checked by the gate loop above) is
    //    threaded into the engine below so the trace-divergence check can
    //    resolve each recorded FIRE record's `node_idx` → node id.
    let env_json: Option<Vec<u8>> = if let Some(env_att) = reader
        .attachment(ENV_ATTACHMENT)
        .map_err(|source| ReplayError::BagOpen {
            path: bag.to_path_buf(),
            source,
        })? {
        match serde_json::from_slice::<serde_json::Value>(&env_att.data) {
            Ok(serde_json::Value::Object(_)) => Some(env_att.data),
            Ok(_) => {
                return Err(ReplayError::BagInvalidAttachment {
                    name: ENV_ATTACHMENT.to_string(),
                    reason: "expected a JSON object at the top level".to_string(),
                })
            }
            Err(e) => {
                return Err(ReplayError::BagInvalidAttachment {
                    name: ENV_ATTACHMENT.to_string(),
                    reason: format!("did not parse as JSON: {e}"),
                })
            }
        }
    } else {
        None
    };

    // 6b. `__cerulion/recorder.json` — OPTIONAL recorder-host identity
    //     attachment. Absent = silent (pre-recorder.json bags);
    //     malformed = loud WARN; arch/os mismatch vs THIS host = loud WARN
    //     (cross-arch float/libm ULP advisory). NEVER exit-affecting.
    let RecorderRead {
        info: recorder,
        unknown_coordination,
    } = read_recorder_info(&reader, bag)?;

    // 6b'. Trace-format version gate: a bag stamped with a `trace_format`
    //      GREATER than this binary supports carries a scheduler-trace layout
    //      this Cerulion cannot decode (e.g. a future `reserved`-bit assignment).
    //      REFUSE it up front with a clear "recorded by a newer Cerulion —
    //      upgrade to replay" message (exit 2, not replay-grade) rather than
    //      silently mis-replaying. ABSENT `trace_format` decodes to version 1
    //      (full back-compat: every unstamped bag). A recorder.json this
    //      binary cannot read AT ALL never reaches here: an unparseable
    //      CONTRACT CARRIER is a refusal inside
    //      `read_recorder_info` rather than a silent treated-as-absent.
    if let Some(info) = &recorder {
        if info.trace_format > replay_engine::SUPPORTED_TRACE_FORMAT {
            return Err(ReplayError::RecordingInconsistent {
                detail: format!(
                    "this bag's scheduler-trace format is version {} but this Cerulion \
                     supports up to version {} — it was recorded by a NEWER Cerulion; \
                     upgrade to replay it",
                    info.trace_format,
                    replay_engine::SUPPORTED_TRACE_FORMAT
                ),
            });
        }
        // 6b''. There is no free-run refusal here: `run_engine`
        //       replays a free-run bag through
        //       the SEQUENTIAL PER-RANK executor,
        //       so a bag this binary can decode is a bag it can
        //       re-execute. What this gate refuses is the UNKNOWN-value
        //       case below, which is about a coordination contract this
        //       binary has no variant for at all.
        //
        //       ORDER: the UNKNOWN-value refusal sits BELOW the version gate on
        //       purpose. Both can be true at once — a bag from a newer Cerulion
        //       stamping a format PAST `SUPPORTED_TRACE_FORMAT` and a
        //       coordination mode this binary
        //       has no variant for is ONE fact, and the version message states
        //       its CAUSE ("recorded by a NEWER Cerulion; upgrade") while the
        //       unknown-value message states only a SYMPTOM. The more
        //       informative refusal therefore wins whenever both apply; the
        //       unknown-value arm is what catches the same skew on a bag whose
        //       format version this binary DOES support.
        if let Some(value) = &unknown_coordination {
            return Err(ReplayError::RecordingInconsistent {
                detail: format!(
                    "this bag's `coordination` stamp is {value}, which is not a coordination \
                     contract this Cerulion knows — it was recorded by a NEWER Cerulion; \
                     upgrade to replay it. (It is REFUSED rather than assumed to be lockstep: \
                     an unknown stamp is positive evidence the recording was taken under a \
                     contract this binary cannot apply, so replaying it under the lockstep one \
                     would either refuse a correct bag as corrupt or mis-anchor it silently.)"
                ),
            });
        }
    }

    // 6c. `__cerulion/record_health.json` — OPTIONAL per-recording health stamp
    //     (written by bagd for every finalized bag). Absent = an older
    //     bag (silent back-compat); malformed = loud WARN then treated-as-absent
    //     (the recorder.json precedent). A LOSSY stamp drives the engine's
    //     up-front warn block + per-topic record-side-loss attribution. NEVER
    //     exit-affecting at the gate (only a bag READ error is a real `Err`).
    let record_health = read_record_health(&reader, bag)?;

    // 7. All gates passed — the bag IS replay-grade. Build the zero-copy
    //    recorded-message index (FILE order, `__cerulion/*` excluded) OVER the
    //    memory-mapped bag, so the engine serves recorded frames as borrowed
    //    page-cache slices — no owned per-frame `Vec`. Every
    //    reader-borrowing gate above has already run.
    //
    //    An index failure here means the BAG is corrupt (bad footer
    //    summary_start, torn/residue framing, a compressed or foreign chunk) —
    //    the same class as every other bag read failure, so it maps to `BagOpen`
    //    (exit 2: "your bag is not replay-grade"), NEVER exit 5 (`Internal` = a
    //    replay-harness bug). The strictest-coherent line: anything wrong WITH
    //    the bag's bytes = 2; exit 5 is reserved for post-gate harness state
    //    (transport, scheduler, tap loss).
    //    The reader is SHARED (`Arc`) between the two zero-copy views of the
    //    one map: `RecordedMessages` (user-frame spans) and `RecordedTrace`
    //    (the streamed scheduler-trace channel the engine re-walks per pass —
    //    never a materialized `Vec<TraceRingRecord>`, the memory
    //    scaler).
    let reader = std::sync::Arc::new(reader);
    let recorded_messages =
        replay_engine::RecordedMessages::from_reader(reader.clone()).map_err(|source| {
            ReplayError::BagOpen {
                path: bag.to_path_buf(),
                source,
            }
        })?;
    // 7b. The schema-drift PRE-FLIGHT (exit 2). Now that the
    //     per-topic recorded aggregates are built (first-frame `schema_hash` per
    //     topic — already parsed by the `RecordedMessages` fold, no extra walk),
    //     compare each recorded produced topic's schema hash against the CURRENT
    //     workspace's. Any drift refuses up front with an actionable table —
    //     BEFORE the engine loads a cdylib — so a since-recording schema change
    //     surfaces as "not replay-grade for this workspace" instead of an
    //     unexplained per-frame byte mismatch (exit 1). Fires only for topics
    //     with recorded frames + a resolvable current hash; a no-drift bag
    //     replays exactly as if the check were absent. Runs AFTER the tolerance
    //     preflight (exit 4) BY DESIGN: the user's own input is validated
    //     before the bag's replay-grade is judged, so a run with both
    //     problems reports the tolerance error first.
    detect_schema_drift(&reader, &recorded_messages, &config, &workspace_root)?;

    let trace = replay_engine::RecordedTrace::new(reader);
    tracing::info!(
        bag = ?bag,
        nodes = config.nodes.len(),
        trace_records = trace_record_count,
        user_topics = recorded_messages.topics().count(),
        "replay: bag is replay-grade; handing to the deterministic replay engine"
    );

    let inputs = ReplayInputs {
        config,
        trace,
        node_ids,
        recorded_messages,
        env_json,
        recorder,
        record_health,
        duration_bound_ns: opts.duration_bound_ns,
        report_path: opts.report_path,
        fire_tap: None,
        tolerance,
        strict_state: opts.strict_state,
        ros2_entries_skipped,
    };
    let nodes = ReplayNodes::Cdylib {
        workspace_root,
        prefer_release: false,
    };
    replay_engine::run_engine(inputs, nodes)
}

/// The schema-drift PRE-FLIGHT (exit 2). Compares each recorded
/// graph-produced topic's recorded schema hash (the first recorded frame's
/// `WireHeader.schema_hash`, already parsed by the `RecordedMessages` fold)
/// against the CURRENT workspace's recipe-3 hash for that topic (resolved via
/// [`crate::replay_field_registry::FieldRegistry`], the same machinery the
/// tolerance gate uses). Any topic whose recorded and current hashes disagree is
/// a [`SchemaDrift`]; a non-empty set is refused with
/// [`ReplayError::SchemaDrift`] (exit 2). Contract:
///
/// * Fires ONLY for topics that HAVE recorded frames (`recorded.topics()`),
///   never for a registered-but-empty channel.
/// * Fires ONLY for topics with a resolvable CURRENT produced-schema hash —
///   external/consumed-only topics (whose schema is not in the graph YAML) and
///   unresolvable schemas are skipped.
/// * Skips topics whose bag channel was NOT recorded under the current hash
///   recipe (a legacy bag would carry a recipe-1 frame hash that is
///   not comparable to the recipe-3 registry hash — those surface as an
///   unexplained byte mismatch, out of this preflight's scope).
/// * A `channels()` read failure conservatively DISABLES the preflight (the
///   replay proceeds) rather than misclassifying a bag read error as schema drift.
///
/// A no-drift bag returns `Ok(())` and the replay proceeds exactly as if this
/// preflight were absent.
fn detect_schema_drift(
    reader: &BagReader,
    recorded: &replay_engine::RecordedMessages,
    config: &cerulion_core::graph::GraphConfig,
    workspace_root: &Path,
) -> Result<(), ReplayError> {
    use cerulion_core::trace::bag::HASH_RECIPE;

    // Current per-topic produced-schema hashes (workspace `schemas/*.yaml` +
    // built-in ROS 2 registry), computed codegen-faithfully (recipe 3).
    let registry = crate::replay_field_registry::FieldRegistry::from_graph(config, workspace_root);

    // Recipe guard: map each recorded topic → whether its channel descriptor was
    // stamped with the CURRENT hash recipe. A `channels()` failure (broken
    // summary) leaves the map empty → every topic is skipped (preflight
    // disabled, the replay proceeds) rather than misattributed as drift.
    let recipe_current: std::collections::HashMap<String, bool> = match reader.channels() {
        Ok(chans) => chans
            .into_iter()
            .filter_map(|c| {
                c.descriptor
                    .map(|d| (c.topic, d.hash_recipe == HASH_RECIPE))
            })
            .collect(),
        Err(e) => {
            tracing::debug!(error = ?e, "replay: schema-drift preflight skipped (channels unreadable)");
            std::collections::HashMap::new()
        }
    };

    let mut drifts: Vec<SchemaDrift> = Vec::new();
    for topic in recorded.topics() {
        // Only topics whose CURRENT produced-schema hash resolves.
        let Some(current_hash) = registry.expected_schema_hash(topic) else {
            continue;
        };
        // Only frames recorded under the current hash recipe (skip legacy bags).
        if recipe_current.get(topic).copied() != Some(true) {
            continue;
        }
        // The recorded frame's own schema hash (never the descriptor's, which
        // synthetic bags stamp 0). `None` = no parseable header → nothing to
        // compare.
        let Some(recorded_hash) = recorded.first_schema_hash(topic) else {
            continue;
        };
        if recorded_hash != current_hash {
            drifts.push(SchemaDrift {
                topic: topic.clone(),
                recorded_hash,
                current_hash,
            });
        }
    }

    if drifts.is_empty() {
        Ok(())
    } else {
        Err(ReplayError::SchemaDrift { drifts })
    }
}

/// The `--tolerance` pre-flight gate. Reads + parses +
/// validates the tolerance YAML against the bag's graph + the workspace schema
/// set (via [`crate::replay_field_registry::FieldRegistry`]); ANY failure —
/// unreadable file, unknown/misspelled key, out-of-range threshold, an
/// unresolvable topic/field name, OR a non-`bit_exact` metric on a
/// publisher-opaque field — is a [`ReplayError::ToleranceInvalid`] (exit 4). A
/// `None` path is a no-op (`Ok(None)`) — the default byte-exact replay.
///
/// This runs BEFORE [`replay_engine::run_engine`] constructs the runtime, so a
/// bad tolerance document never loads a cdylib or touches the transport. On
/// success it returns the VALIDATED [`CompiledTolerance`] (spec + the same
/// field registry validation ran against), which the engine reuses to build the
/// per-topic decoders (the metric MATH relaxes the diff — a
/// tolerance targeting a topic compares its per-field overrides by their metric
/// and the rest byte-exact; an untargeted topic stays byte-exact).
fn validate_tolerance(
    tolerance_path: Option<&Path>,
    config: &cerulion_core::graph::GraphConfig,
    workspace_root: &Path,
) -> Result<Option<CompiledTolerance>, ReplayError> {
    let path = match tolerance_path {
        Some(p) => p,
        None => return Ok(None),
    };
    // A directory ALREADY surfaces loudly: the `read_to_string` fall-through
    // below wraps its "Is a directory (os error 21)" io error into a
    // `ToleranceInvalid` (exit 4) naming the path. This guard only SHARPENS that
    // message — it names the directory as the structural cause and adds the
    // `--tolerance <file.yaml>` hint. Only `is_dir()` is special-cased; other
    // non-file kinds intentionally fall through to the still-loud read error.
    if path.is_dir() {
        return Err(ReplayError::ToleranceInvalid {
            reason: format!(
                "tolerance path {} is a directory, not a tolerance YAML file — pass \
                 `--tolerance <file.yaml>`",
                path.display()
            ),
        });
    }
    let yaml = std::fs::read_to_string(path).map_err(|e| ReplayError::ToleranceInvalid {
        reason: format!("could not read tolerance file {}: {e}", path.display()),
    })?;
    let spec = crate::tolerance::ToleranceSpec::parse(&yaml).map_err(|reason| {
        ReplayError::ToleranceInvalid {
            reason: format!("tolerance file {}: {reason}", path.display()),
        }
    })?;
    let registry = crate::replay_field_registry::FieldRegistry::from_graph(config, workspace_root);
    spec.validate(&registry)
        .map_err(|reason| ReplayError::ToleranceInvalid { reason })?;
    // A `default_metric` and topic-wide `metric:` are FUNCTIONAL (they
    // expand over the topic's schema fields at the diff seam). The only remaining
    // no-op is a document whose EVERY resolution is `bit_exact` — warn LOUDLY so
    // the user is never left believing a relaxation applied when the whole file is
    // byte-exact.
    warn_on_inert_tolerance(&spec);
    tracing::info!(
        tolerance = %path.display(),
        topics = spec.topics.len(),
        "replay: tolerance document validated (per-field metrics relax the diff)"
    );
    Ok(Some(CompiledTolerance { spec, registry }))
}

/// Warn when a tolerance document has NO effect — i.e. EVERY metric in it
/// (the `default_metric`, every topic-wide `metric:`, and every `fields:`
/// override) resolves to `bit_exact`, so replay stays byte-exact everywhere,
/// identical to running with no `--tolerance`. Loud-over-silent: the user asked
/// for a relaxation and got none. (A non-`bit_exact` `default_metric`
/// or topic-wide `metric:` IS functional — it expands over the topic's schema
/// fields — so it is not inert; only an all-`bit_exact` file is.)
fn warn_on_inert_tolerance(spec: &crate::tolerance::ToleranceSpec) {
    let any_effect = !spec.default_metric.is_bit_exact()
        || spec.topics.values().any(|t| {
            t.metric.as_ref().is_some_and(|m| !m.is_bit_exact())
                || t.fields.values().any(|m| !m.is_bit_exact())
        });
    if !any_effect {
        tracing::warn!(
            "replay tolerance: every metric in this document resolves to bit_exact — the document \
             has NO effect (replay stays byte-exact everywhere, identical to running with no \
             `--tolerance`)"
        );
    }
}

/// What reading `__cerulion/recorder.json` produced.
///
/// A struct rather than `Option<RecorderInfo>`, because a
/// bare `Option` cannot express the one distinction the two CONTRACT gates
/// need — "this bag says nothing" versus "this bag says something this binary
/// does not understand".
struct RecorderRead {
    /// The parsed info, or `None` when the attachment is ABSENT (a
    /// pre-recorder.json bag).
    info: Option<RecorderInfo>,
    /// A `coordination` value present on the wire that this binary has no
    /// variant for — refused at the gate, NEVER inferred to lockstep. Carried
    /// out rather than refused here so the caller can order it AFTER the
    /// trace-format gate (see the gate block in [`run_replay`]).
    unknown_coordination: Option<String>,
}

/// Read the OPTIONAL
/// `__cerulion/recorder.json` attachment.
///
/// # This attachment is a CONTRACT CARRIER, not only an advisory
///
/// Its host identity (arch/os/version) is pure advisory, and for that half ANY parse
/// failure may fold to "absent" with the bag staying replayable. But it also carries
/// `trace_format` and `coordination`, and BOTH gates
/// live behind "did this parse?": were a malformed document to skip the
/// version refusal AND the free-run refusal, the verdict would print
/// `(inferred: no coordination stamp)`, a positive claim about a bag
/// whose stamp is right there and unreadable. The most dangerous shape is the
/// one that reads most reassuring: a bag written by a NEWER Cerulion, whose
/// unknown `coordination` string makes serde fail the whole document, replaying
/// under the lockstep contract while announcing it carries no stamp.
///
/// So the read is FIELD-LEVEL LENIENT (the manifest-key precedent), and the
/// two halves have DIFFERENT failure policies:
/// - the ADVISORY half (arch/os/version/recorded_at_ns) still degrades — an
///   unreadable identity costs the cross-arch warn and nothing else, and it
///   degrades PER FIELD: a document whose `arch` is a number keeps its `os`,
///   its version and its timestamp (see the advisory block below — a
///   whole-object parse would lose all four to the first bad member, which
///   contradicts `RecorderInfo`'s own "a PARTIALLY readable document keeps the
///   halves that decoded");
/// - the CONTRACT half (`trace_format`, `coordination`) REFUSES (exit 2).
///
/// Five arms:
/// - ABSENT → `info: None`, silently (every pre-recorder.json bag — back-compat,
///   no warn spam). This is the ONLY inferred-lockstep path left, and it is the
///   one where the inference is true by construction.
/// - PRESENT but not parseable as a JSON object → **REFUSE** (exit 2) naming
///   what was lost. Never "the bag is still replayable": this document is what
///   says which contract to replay it under.
/// - `trace_format` present but not a `u32` → **REFUSE** (exit 2). It is the
///   version gate's whole input; guessing a version is the hazard the gate
///   exists for.
/// - `coordination` present but not a value this binary knows → carried out as
///   `unknown_coordination` for the caller to refuse (exit 2) — see the gate.
/// - otherwise → parsed; warn on an arch/os mismatch vs THIS host (see
///   [`warn_on_recorder_host_mismatch`]), and carry the info into the
///   outcome/`--report`.
///
/// A bag READ error (corrupt summary) stays a real `Err` (exit 2), same as
/// every other attachment fetch.
fn read_recorder_info(reader: &BagReader, bag: &Path) -> Result<RecorderRead, ReplayError> {
    let att =
        match reader
            .attachment(RECORDER_ATTACHMENT)
            .map_err(|source| ReplayError::BagOpen {
                path: bag.to_path_buf(),
                source,
            })? {
            Some(att) => att,
            None => {
                return Ok(RecorderRead {
                    info: None,
                    unknown_coordination: None,
                })
            }
        };
    let invalid = |reason: String| ReplayError::BagInvalidAttachment {
        name: RECORDER_ATTACHMENT.to_string(),
        reason,
    };
    let doc: serde_json::Value = serde_json::from_slice(&att.data).map_err(|e| {
        invalid(format!(
            "did not parse as JSON: {e}. This attachment carries the bag's SCHEDULER-TRACE \
             FORMAT (`trace_format`) and its COORDINATION contract (`coordination`), so an \
             unreadable one means replay cannot tell which contract this recording was taken \
             under — it is refused rather than replayed under a guessed one. Re-record the \
             bag, or repair the attachment"
        ))
    })?;
    let Some(obj) = doc.as_object() else {
        return Err(invalid(
            "expected a JSON object at the top level — this attachment carries the bag's \
             `trace_format` and `coordination` contract keys, which cannot be read from any \
             other shape"
                .to_string(),
        ));
    };

    // CONTRACT key 1 — the trace-format version. ABSENT decodes to v1 (every
    // unstamped bag, full back-compat); PRESENT-but-unreadable refuses.
    let trace_format: u32 = match obj.get("trace_format") {
        None => 1,
        Some(v) => match v.as_u64().filter(|n| *n <= u32::MAX as u64) {
            Some(n) => n as u32,
            None => {
                return Err(invalid(format!(
                    "`trace_format` is {v}, which is not a scheduler-trace format version — it \
                     is the key this binary gates on to decide whether it can decode this bag's \
                     trace at all, so an unreadable value is refused rather than assumed"
                )))
            }
        },
    };

    // CONTRACT key 2 — the coordination contract. ABSENT is the sound
    // lockstep inference; an UNKNOWN value is NOT (a value this binary has
    // no variant for was written by a NEWER one, so "assume lockstep" is
    // exactly the wrong guess). Carried out; refused by the caller.
    let (coordination, unknown_coordination) = match obj.get("coordination") {
        None => (None, None),
        Some(v) => match serde_json::from_value::<replay_engine::CoordinationMode>(v.clone()) {
            Ok(mode) => (Some(mode), None),
            Err(_) => (None, Some(v.to_string())),
        },
    };

    // ADVISORY half — lenient PER FIELD, and judged apart from the contract.
    //
    // One whole-object `from_value::<RecorderInfo>` would make
    // the four identity fields all-or-nothing: serde fails the WHOLE document
    // on the FIRST unreadable member, so `{"arch": 42, "os": "linux", ..}` loses
    // a perfectly good `os`, `cerulion_version` and `recorded_at_ns` along with
    // the one field that is wrong — while `RecorderInfo`'s own doc promises
    // that "a PARTIALLY readable document keeps the halves that decoded". Under a
    // whole-object parse that promise is true of an ABSENT field (serde reads a
    // missing `Option` as `None`) and false of a MALFORMED one, which is the shape a
    // hand-edited or half-written attachment actually takes. Each field is
    // read on its own, so one bad member costs exactly itself.
    //
    // The CONTRACT keys are NOT read here — they were extracted above and their
    // policy is the opposite (refuse, never degrade). Picking the four advisory
    // keys by name is also what keeps an unknown `coordination` value (which
    // serde cannot map to a variant) from failing the identity: reading it is
    // simply not this parse's business.
    let mut unreadable: Vec<&'static str> = Vec::new();
    let info = RecorderInfo {
        arch: advisory_string(obj, "arch", &mut unreadable),
        os: advisory_string(obj, "os", &mut unreadable),
        cerulion_version: advisory_string(obj, "cerulion_version", &mut unreadable),
        recorded_at_ns: advisory_u64(obj, "recorded_at_ns", &mut unreadable),
        trace_format,
        coordination,
    };
    if !unreadable.is_empty() {
        tracing::warn!(
            attachment = RECORDER_ATTACHMENT,
            fields = %unreadable.join(", "),
            "replay: recorder.json's HOST IDENTITY half did not decode for these fields — \
             the `--report` JSON OMITS them rather than filling them in, while the SIBLING \
             identity fields that DID read are kept. Its CONTRACT keys (`trace_format`, \
             `coordination`) are read separately and ARE applied."
        );
    }
    // An unreadable identity field is ABSENT, never
    // fabricated. Filling all four with `""`/`""`/`""`/`0` would make
    // the `--report` JSON carry them as if they were read off the bag — an
    // empty `arch` reads as an answer and `recorded_at_ns: 0` reads as
    // 1970-01-01, so the one durable artifact a CI consumer parses would carry
    // values nobody measured while the only warning that they were unavailable
    // is an ephemeral log line.
    //
    // The mismatch advisory is asked UNCONDITIONALLY, because it can only
    // speak where it has a value: `warn_on_recorder_host_mismatch` claims a
    // mismatch on a field it actually read, so a bag whose `arch` is unreadable
    // and whose `os` is not still gets the `os` half of the advisory instead of
    // losing both.
    warn_on_recorder_host_mismatch(&info, std::env::consts::ARCH, std::env::consts::OS);
    Ok(RecorderRead {
        info: Some(info),
        unknown_coordination,
    })
}

/// Read ONE advisory string field field-leniently: absent (or an explicit
/// `null`, which serde also reads as `None` on an `Option`) yields `None`
/// silently; a present-but-non-string value yields `None` and NAMES itself in
/// `unreadable`, so the warn can say which field was lost rather than
/// condemning the whole identity.
fn advisory_string(
    obj: &serde_json::Map<String, serde_json::Value>,
    key: &'static str,
    unreadable: &mut Vec<&'static str>,
) -> Option<String> {
    match obj.get(key) {
        None | Some(serde_json::Value::Null) => None,
        Some(serde_json::Value::String(s)) => Some(s.clone()),
        Some(_) => {
            unreadable.push(key);
            None
        }
    }
}

/// [`advisory_string`] for the one numeric identity field. The accepted shape
/// is exactly serde's for an `Option<u64>` — a negative, fractional or
/// oversized number is unreadable, not silently rounded or clamped.
fn advisory_u64(
    obj: &serde_json::Map<String, serde_json::Value>,
    key: &'static str,
    unreadable: &mut Vec<&'static str>,
) -> Option<u64> {
    match obj.get(key) {
        None | Some(serde_json::Value::Null) => None,
        Some(v) => match v.as_u64() {
            Some(n) => Some(n),
            None => {
                unreadable.push(key);
                None
            }
        },
    }
}

/// Read the OPTIONAL `__cerulion/record_health.json` attachment (the
/// per-recording health stamp bagd writes into every cleanly-finalized bag).
/// Three arms map to a [`RecordHealthReport`], NONE exit-affecting (the
/// recorder.json precedent — warn-never-refuse):
/// - ABSENT → [`RecordHealthReport::Absent`] silently (every older bag
///   lacks it — back-compat, no warn spam; the replay itself is unaffected);
/// - MALFORMED JSON → loud `warn!`, [`RecordHealthReport::Malformed`] (the bag
///   is still replayable — only the record-side attribution is unavailable);
/// - PARSED → [`RecordHealthReport::Present`] (the engine emits the up-front
///   lossy warn block + attributes affected topics' diffs to record-side loss).
///
/// Only a bag READ error (corrupt summary) is a real `Err` (exit 2), same as
/// every other attachment fetch.
fn read_record_health(reader: &BagReader, bag: &Path) -> Result<RecordHealthReport, ReplayError> {
    let att = match reader
        .attachment(RECORD_HEALTH_ATTACHMENT)
        .map_err(|source| ReplayError::BagOpen {
            path: bag.to_path_buf(),
            source,
        })? {
        Some(att) => att,
        None => return Ok(RecordHealthReport::Absent),
    };
    match serde_json::from_slice::<RecordHealth>(&att.data) {
        Ok(health) => Ok(RecordHealthReport::Present(health)),
        Err(e) => {
            tracing::warn!(
                attachment = RECORD_HEALTH_ATTACHMENT,
                error = %e,
                "replay: malformed record_health.json attachment — ignoring it (the bag is still \
                 replayable; record-side-loss attribution is unavailable for this bag)"
            );
            Ok(RecordHealthReport::Malformed)
        }
    }
}

/// WARN (never refuse) when the bag was recorded on a
/// different architecture or OS than the replaying host. `host_arch` /
/// `host_os` are parameters (not read inline) so the `#[traced_test]` pins
/// below can inject a mismatch on any CI host.
///
/// Each half is compared only where there IS a value.
/// The identity fields are `Option` (absence rides the `Option`, never a
/// sentinel — see [`RecorderInfo`]), and a field the document did not carry is
/// not evidence of a mismatch: warning on it would turn "we could not read the
/// recording host's arch" into "you are replaying cross-arch", which is a
/// positive claim about a bag nobody measured. An absent value renders as
/// `<unknown>` in the structured fields for the same reason.
fn warn_on_recorder_host_mismatch(info: &RecorderInfo, host_arch: &str, host_os: &str) {
    let arch_mismatch = info.arch.as_deref().is_some_and(|a| a != host_arch);
    let os_mismatch = info.os.as_deref().is_some_and(|o| o != host_os);
    if arch_mismatch || os_mismatch {
        let unknown = "<unknown>";
        tracing::warn!(
            recorded_arch = %info.arch.as_deref().unwrap_or(unknown),
            host_arch = %host_arch,
            recorded_os = %info.os.as_deref().unwrap_or(unknown),
            host_os = %host_os,
            recorded_cerulion_version = %info.cerulion_version.as_deref().unwrap_or(unknown),
            "cross-architecture/OS replay: float/libm results MAY differ in ULPs — a \
             byte-mismatch on transcendental-heavy nodes may be arch skew, not a regression; \
             a tolerance file is the remedy"
        );
    }
}

/// Fetch a required attachment by name, or fail with
/// [`ReplayError::BagMissingAttachment`].
/// A read error (corrupt summary) surfaces as [`BagOpen`](ReplayError::BagOpen).
fn require_attachment(
    reader: &BagReader,
    bag: &Path,
    name: &str,
) -> Result<BagAttachment, ReplayError> {
    reader
        .attachment(name)
        .map_err(|source| ReplayError::BagOpen {
            path: bag.to_path_buf(),
            source,
        })?
        .ok_or_else(|| ReplayError::BagMissingAttachment {
            name: name.to_string(),
        })
}

/// The `u32::MAX` sentinel a multi-process supervisor stamps as the rank of its
/// own DEPARTURE ring — NOT a worker rank. Its manifest
/// (`trace_manifest_rank4294967295.json`) is a supervisor artifact and is
/// EXCLUDED from the worker-rank contiguity contract.
///
/// This is an ALIAS of
/// [`cerulion_core::trace_ring::DEPARTURE_RING_RANK`], not a third declaration
/// of `u32::MAX`. Keeping it local "to avoid cross-module coupling"
/// would trade a dependency for a drift hazard on a value that is written into
/// the wire form by one module and read back by two others, plus Flashback, a
/// fourth reader (a Flashback capture judging its own resimmability). A sentinel
/// that must MATCH is a shared constant, not a local convention.
///
/// `pub(crate)`: [`crate::replay_engine`] uses it when re-deriving the per-rank
/// tables (multi-rank).
pub(crate) const DEPARTURE_RING_RANK: u32 = cerulion_core::trace_ring::DEPARTURE_RING_RANK;

/// Load EVERY worker rank's trace manifest (`(attachment name, node_ids
/// table)`, indexed by rank), enforcing the replay-grade manifest contract.
///
/// Contract:
/// - zero manifests → [`BagMissingAttachment`](ReplayError::BagMissingAttachment);
/// - a manifest name whose rank segment is not an integer, or a present-but-
///   unparseable worker manifest → [`BagInvalidAttachment`](ReplayError::BagInvalidAttachment);
/// - duplicate manifests for the SAME rank (corruption, not multi-process) →
///   [`BagInvalidAttachment`](ReplayError::BagInvalidAttachment) — KEPT exactly;
/// - worker ranks that are not contiguous `0..=k` (a hole) →
///   [`MultiRankManifestGap`](ReplayError::MultiRankManifestGap) naming the
///   missing rank.
///
/// The `u32::MAX` supervisor departure-ring manifest ([`DEPARTURE_RING_RANK`])
/// is TOLERATED — it is a supervisor artifact, excluded from the worker-rank
/// contiguity check and NOT decoded (its node table is empty by contract).
/// Multi-rank bags are ACCEPTED (the engine demuxes per-rank
/// streams by `reserved` and k-way-merges each step's fires); every worker
/// rank's table is decoded here so the gate can bounds-check EACH rank's FIRE
/// `node_idx` against its OWN manifest.
///
/// Returns the per-rank `(manifest name, node_ids)` pairs; index == rank
/// (contiguity-validated), so `.len()` is the worker rank count `k`.
fn load_trace_manifests(
    reader: &BagReader,
    bag: &Path,
) -> Result<Vec<(String, Vec<String>)>, ReplayError> {
    let attachments = reader
        .attachments()
        .map_err(|source| ReplayError::BagOpen {
            path: bag.to_path_buf(),
            source,
        })?;
    let manifests: Vec<&BagAttachment> = attachments
        .iter()
        .filter(|a| {
            a.name.starts_with(TRACE_MANIFEST_PREFIX) && a.name.ends_with(TRACE_MANIFEST_SUFFIX)
        })
        .collect();

    if manifests.is_empty() {
        // Name the canonical rank-0 manifest in the "missing" message.
        return Err(ReplayError::BagMissingAttachment {
            name: format!("{TRACE_MANIFEST_PREFIX}0{TRACE_MANIFEST_SUFFIX}"),
        });
    }

    // Parse the rank segment out of each matching attachment name, keeping the
    // REAL attachment reference alongside its parsed rank (the data is decoded
    // below for rank 0). Leading zeros mean two distinct attachments (`rank0`
    // and `rank00`) parse to the same rank, so a duplicate-rank error must name
    // the actual offending attachments — a synthesized canonical name could
    // match neither.
    let mut ranked: Vec<(u32, &BagAttachment)> = Vec::with_capacity(manifests.len());
    for m in &manifests {
        let rank_segment =
            &m.name[TRACE_MANIFEST_PREFIX.len()..m.name.len() - TRACE_MANIFEST_SUFFIX.len()];
        let rank = rank_segment
            .parse::<u32>()
            .map_err(|_| ReplayError::BagInvalidAttachment {
                name: m.name.clone(),
                reason: format!(
                    "expected an integer rank between `{TRACE_MANIFEST_PREFIX}` and \
                     `{TRACE_MANIFEST_SUFFIX}`, found `{rank_segment}`"
                ),
            })?;
        ranked.push((rank, m));
    }
    // Sort by rank; tie-break by name so a same-rank pair reports its two
    // attachments in a deterministic, byte-stable order across runs.
    ranked.sort_unstable_by(|a, b| a.0.cmp(&b.0).then_with(|| a.1.name.cmp(&b.1.name)));

    // TWO manifests for the SAME rank is corruption (a rank cannot legitimately
    // be recorded twice) — checked FIRST so a duplicate never mislabels as a
    // gap. Report the FIRST real offending attachment as `name` and BOTH real
    // names in the reason (leading-zero variants of one rank are distinct
    // attachments a synthesized canonical name would fail to identify). Covers
    // duplicate sentinel manifests too (two `u32::MAX` rings is still corrupt).
    if let Some(pair) = ranked.windows(2).find(|w| w[0].0 == w[1].0) {
        let (rank, first_name) = (pair[0].0, pair[0].1.name.as_str());
        let second_name = pair[1].1.name.as_str();
        return Err(ReplayError::BagInvalidAttachment {
            name: first_name.to_string(),
            reason: format!(
                "duplicate trace manifest for rank {rank} (attachments `{first_name}` and \
                 `{second_name}`) — corrupt recording"
            ),
        });
    }

    // Split the supervisor departure-ring sentinel (u32::MAX) from the WORKER
    // ranks — the sentinel is a supervisor artifact, excluded from contiguity.
    let worker_ranks: Vec<u32> = ranked
        .iter()
        .map(|(r, _)| *r)
        .filter(|r| *r != DEPARTURE_RING_RANK)
        .collect();

    // A bag with ONLY the sentinel manifest (no worker rank) is missing its
    // rank-0 recording — the same "no worker manifest" class as zero manifests.
    if worker_ranks.is_empty() {
        return Err(ReplayError::BagMissingAttachment {
            name: format!("{TRACE_MANIFEST_PREFIX}0{TRACE_MANIFEST_SUFFIX}"),
        });
    }

    // Worker ranks must be contiguous 0..=max with no hole (deduped above, so a
    // gap is the only remaining non-contiguity). Naming the first missing rank
    // is the precise corruption diagnosis.
    //
    // The RULE is `trace_ring::first_rank_manifest_gap`, shared with
    // the Flashback capture's `resimmable` judge, so a bag this gate refuses can
    // never be stamped resimmable — the same move `is_departure_boundary` and
    // `classify_trace_record` already made for their refusals. The ERROR stays
    // here, because it carries the roster and the max this gate reports and the
    // pure predicate deliberately does not.
    let max = *worker_ranks.last().expect("non-empty checked above");
    if let Some(missing) = cerulion_core::trace_ring::first_rank_manifest_gap(&worker_ranks) {
        return Err(ReplayError::MultiRankManifestGap {
            present: worker_ranks.clone(),
            missing,
            max,
        });
    }

    // Contiguity holds — decode EVERY worker rank's node-id table (index ==
    // rank, since worker ranks are exactly 0..=max after the check above). The
    // sentinel manifest is skipped (supervisor artifact, empty table by
    // contract — never indexed by a FIRE record that survives the degraded
    // gate).
    let mut tables: Vec<(String, Vec<String>)> = Vec::with_capacity(worker_ranks.len());
    for (rank, m) in &ranked {
        if *rank == DEPARTURE_RING_RANK {
            continue;
        }
        let parsed: TraceManifest =
            serde_json::from_slice(&m.data).map_err(|e| ReplayError::BagInvalidAttachment {
                name: m.name.clone(),
                reason: format!("manifest JSON did not parse: {e}"),
            })?;
        tables.push((m.name.clone(), parsed.node_ids));
    }
    Ok(tables)
}

/// A short, human-readable label for a non-finalized completeness state (used
/// only in the [`BagNotFinalized`](ReplayError::BagNotFinalized) message).
fn describe_completeness(completeness: &BagCompleteness) -> String {
    match completeness {
        BagCompleteness::TruncatedAtChunkBoundary => "TruncatedAtChunkBoundary".to_string(),
        BagCompleteness::TornTail(e) => format!("TornTail: {e}"),
        // `Finalized` never reaches here (the caller gates on `is_finalized`);
        // `#[non_exhaustive]` requires a wildcard for future variants.
        other => format!("{other:?}"),
    }
}

#[cfg(test)]
mod tests {
    /// This module's exit-code TABLE is markdown, and a
    /// markdown table is a literal.
    ///
    /// The divergence vocabulary renders from ONE
    /// function so no surface can drift, and every CODE surface does. The doc
    /// table cannot — it spells the phrases out — so a rename of
    /// `divergence_class_phrase`'s output would leave the operator-facing
    /// contract in this file quietly describing the old words. This reads the
    /// source and requires each phrase to be EXACTLY what the function returns.
    ///
    /// Read from the file rather than from `module_path!` docs because Rust
    /// exposes no runtime access to `//!` text; the path is resolved from
    /// `CARGO_MANIFEST_DIR`, so it moves with the crate.
    #[test]
    fn the_exit_code_tables_phrases_are_the_rendered_vocabulary() {
        use crate::replay_engine::DivergenceClass;
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("src")
            .join("replay_cmd.rs");
        let src = std::fs::read_to_string(&path)
            .unwrap_or_else(|e| panic!("reading {} for the doc-table pin: {e}", path.display()));
        // Only the module doc block — the `//!` prefix is what makes this the
        // CONTRACT rather than any prose further down the file.
        let doc: String = src
            .lines()
            .take_while(|l| l.starts_with("//!") || l.starts_with("// SPDX"))
            .collect::<Vec<_>>()
            .join("\n");
        assert!(
            doc.contains("Exit-code contract"),
            "the module-doc extraction must reach the exit-code table — it read {} bytes",
            doc.len()
        );
        for class in DivergenceClass::ALL {
            let phrase = class.phrase();
            assert!(
                doc.contains(phrase),
                "the exit-code contract must name the class by its RENDERED phrase \
                 ({phrase:?}) — `divergence_class_phrase` is the one vocabulary and this \
                 table is a literal copy of it"
            );
        }
        // ANTI-TAUTOLOGY: a phrase the vocabulary does NOT produce must be
        // absent, or `contains` over a large doc block proves nothing.
        assert!(
            !doc.contains("read-log divergence"),
            "the module doc must not carry a SECOND spelling of the edge-read class — that \
             internal name is exactly the drift this pin exists to catch"
        );
    }

    use super::*;

    /// Oracle-vector pin for the typed (2–5) half of the stable 0–6 exit-code
    /// contract: every [`ReplayError`] variant asserted against its
    /// HAND-WRITTEN expected code. A remap of any variant flips exactly one row
    /// here. The outcome-driven exits (0/1/6) have no `ReplayError` variant —
    /// exit 6's pin is the subprocess CLI test
    /// (`crates/cerulion_cli/tests/replay_cli_test.rs`).
    #[test]
    fn exit_code_contract_is_stable() {
        let io = || BagError::from(std::io::Error::other("boom"));
        let cases: Vec<(ReplayError, u8)> = vec![
            (
                ReplayError::BagOpen {
                    path: PathBuf::from("/x.mcap"),
                    source: io(),
                },
                2,
            ),
            (
                ReplayError::NotMcapBag {
                    path: PathBuf::from("/x.mcap"),
                },
                2,
            ),
            (
                ReplayError::BagNotFinalized {
                    completeness: "TruncatedAtChunkBoundary".to_string(),
                },
                2,
            ),
            (ReplayError::BagNoSchedulerTrace, 2),
            (
                ReplayError::BagMissingAttachment {
                    name: "graph.yaml".to_string(),
                },
                2,
            ),
            (
                ReplayError::BagInvalidAttachment {
                    name: "graph.yaml".to_string(),
                    reason: "bad".to_string(),
                },
                2,
            ),
            (
                ReplayError::MultiRankManifestGap {
                    present: vec![0, 2],
                    missing: 1,
                    max: 2,
                },
                2,
            ),
            (
                ReplayError::DegradedRecordingDeparture {
                    index: 3,
                    detail: "a RECORD_TYPE_DEPARTURE (record_type 2) record".to_string(),
                },
                2,
            ),
            (
                ReplayError::TraceRecordUnsupported {
                    index: 7,
                    record_type: 2,
                    reason: "departure".to_string(),
                },
                2,
            ),
            (
                ReplayError::BagGraphMismatch {
                    topic: "/x/n/out".to_string(),
                    detail: "not in graph".to_string(),
                },
                2,
            ),
            (ReplayError::BagNoStepBoundaries, 2),
            (
                ReplayError::RecordingInconsistent {
                    detail: "frame 0 timestamp not in boundary set".to_string(),
                },
                2,
            ),
            (
                ReplayError::NodeLoad {
                    node_type: "camera".to_string(),
                    reason: "missing".to_string(),
                },
                3,
            ),
            // Both free-run refusals are exit 2 (the bag is not
            // replay-grade FOR THIS BINARY) — never 1 (a data violation) and
            // never 6 (a schedule divergence), which is what keeps the
            // 3 > 6 > 1 precedence untouched.
            (
                ReplayError::FreeRunResumeUnsupported {
                    first_step: 12,
                    ranks: 2,
                },
                2,
            ),
            (
                ReplayError::ToleranceInvalid {
                    reason: "bad".to_string(),
                },
                4,
            ),
            (
                ReplayError::Internal {
                    reason: "panic".to_string(),
                },
                5,
            ),
        ];
        for (err, expected) in cases {
            assert_eq!(
                err.exit_code(),
                expected,
                "exit code drifted for {err:?} (Display: {err})"
            );
        }
    }

    /// The pass/violation sentinels are their contracted numeric values.
    #[test]
    fn pass_and_violation_sentinels() {
        assert_eq!(EXIT_PASS, 0);
        assert_eq!(EXIT_VIOLATION, 1);
    }

    /// Every error message names something actionable (a path, attachment,
    /// count, or the re-record instruction) — a smoke pin that Display strings
    /// are non-empty and mention the subject.
    #[test]
    fn messages_are_actionable() {
        assert!(ReplayError::BagNoSchedulerTrace
            .to_string()
            .contains("scheduler_trace"));
        assert!(ReplayError::BagMissingAttachment {
            name: "graph.yaml".to_string(),
        }
        .to_string()
        .contains("graph.yaml"));
        // The contiguity-gap error names the missing rank.
        let gap = ReplayError::MultiRankManifestGap {
            present: vec![0, 2],
            missing: 1,
            max: 2,
        }
        .to_string();
        assert!(
            gap.contains("rank 1") && gap.contains("non-contiguous"),
            "got: {gap}"
        );
        // The degraded-recording error carries the fault-replay wording.
        let degraded = ReplayError::DegradedRecordingDeparture {
            index: 3,
            detail: "a RECORD_TYPE_DEPARTURE (record_type 2) record".to_string(),
        }
        .to_string();
        assert!(
            degraded.contains("degraded recording")
                && degraded.contains("fault replay")
                && degraded.contains("post-launch"),
            "got: {degraded}"
        );
        assert!(ReplayError::BagGraphMismatch {
            topic: "/x/n/out".to_string(),
            detail: "neither produced nor consumed".to_string(),
        }
        .to_string()
        .contains("/x/n/out"));
        // By design the missing-boundary error names the
        // record kind and the CAUSE (a recorder older than step boundaries), and
        // never the `--record` re-record remedy. The pin is INVERTED rather
        // than dropped — asserting only that `--record` is gone would be
        // satisfied by a message that explains nothing — and the negative half
        // is kept beside it so the flag cannot creep back.
        let msg = ReplayError::BagNoStepBoundaries.to_string();
        assert!(
            msg.contains("STEP_BOUNDARY") && msg.contains("older than step boundaries"),
            "got: {msg}"
        );
        // …and it must not attribute EVERY such bag to an old binary. A trace
        // whose boundary records were stripped or corrupted after recording
        // reaches the identical error, and the bag cannot tell the two apart —
        // so naming only the version is false provenance on the other one.
        assert!(
            msg.contains("stripped or corrupted"),
            "the message must name BOTH ways a trace ends up boundary-less: {msg}"
        );
        assert!(
            !msg.contains("--record") && !msg.contains("--no-rings"),
            "an absence names the CAUSE, never another verb's flag: {msg}"
        );
        // …and the same rule on its sibling: a plain run HAS a trace, so the
        // absence is the run's own state, recorded in `trace_rings`.
        let msg = ReplayError::BagNoSchedulerTrace.to_string();
        assert!(
            msg.contains("scheduler_trace")
                && msg.contains("trace_rings")
                && msg.contains("wall-gated"),
            "got: {msg}"
        );
        assert!(
            !msg.contains("--record") && !msg.contains("--no-rings"),
            "an absence names the CAUSE, never another verb's flag: {msg}"
        );
        // The consistency error carries the caller's detail + the corrupt hint.
        let msg = ReplayError::RecordingInconsistent {
            detail: "boundary step 3 skipped".to_string(),
        }
        .to_string();
        assert!(
            msg.contains("boundary step 3 skipped") && msg.contains("corrupt"),
            "got: {msg}"
        );
        let trace_err = ReplayError::TraceRecordUnsupported {
            index: 7,
            record_type: 9,
            reason: "reserved kind".to_string(),
        };
        let msg = trace_err.to_string();
        assert!(
            msg.contains("record 7") && msg.contains("record_type 9"),
            "got: {msg}"
        );
    }

    // =======================================================================
    // `warn_on_recorder_host_mismatch` pins: the
    // warn fires on an arch OR os mismatch (with the ULP advisory + all four
    // structured fields), and stays SILENT on a matching host. The
    // absent/malformed attachment arms + the never-exit-affecting contract
    // are pinned end-to-end by the subprocess test
    // (`crates/cerulion_cli/tests/replay_cli_test.rs::recorder_json_mismatch_warns_absent_silent_malformed_loud`).
    // =======================================================================

    fn recorder(arch: &str, os: &str) -> RecorderInfo {
        RecorderInfo {
            arch: Some(arch.to_string()),
            os: Some(os.to_string()),
            cerulion_version: Some("0.1.0".to_string()),
            recorded_at_ns: Some(42),
            trace_format: crate::replay_engine::SUPPORTED_TRACE_FORMAT,
            // An explicitly LOCKSTEP bag: what every bag this
            // binary writes today carries.
            coordination: Some(crate::replay_engine::CoordinationMode::Lockstep),
        }
    }

    #[test]
    #[tracing_test::traced_test]
    fn recorder_arch_mismatch_warns_with_ulp_advisory() {
        warn_on_recorder_host_mismatch(&recorder("x86_64", "linux"), "aarch64", "linux");
        assert!(
            logs_contain("cross-architecture/OS replay")
                && logs_contain("MAY differ in ULPs")
                && logs_contain("a tolerance file is the remedy"),
            "the arch-mismatch warn carries the human advisory"
        );
        assert!(
            logs_contain("x86_64") && logs_contain("aarch64"),
            "both the recorded and host arch appear as structured fields"
        );
    }

    #[test]
    #[tracing_test::traced_test]
    fn recorder_os_only_mismatch_also_warns() {
        warn_on_recorder_host_mismatch(&recorder("aarch64", "macos"), "aarch64", "linux");
        assert!(
            logs_contain("cross-architecture/OS replay")
                && logs_contain("macos")
                && logs_contain("linux"),
            "an OS-only mismatch warns too, naming both OSes"
        );
    }

    #[test]
    #[tracing_test::traced_test]
    fn recorder_matching_host_is_silent() {
        warn_on_recorder_host_mismatch(&recorder("aarch64", "macos"), "aarch64", "macos");
        assert!(
            !logs_contain("cross-architecture/OS replay"),
            "a same-host bag must not warn"
        );
    }

    // =======================================================================
    // `read_record_health` — the gate-side read of the bag's
    // `__cerulion/record_health.json` stamp (the recorder.json precedent).
    // Three arms, none exit-affecting: absent (silent), malformed (loud warn +
    // treated-as-absent), present (parsed). Built over a REAL minimal MCAP bag
    // so the attachment NAME const + the JSON shape are pinned end-to-end.
    // =======================================================================

    use cerulion_bag::{BagWriter, BagWriterConfig};
    use cerulion_bagd::{TopicHealth, RECORD_HEALTH_VERSION};
    use std::collections::BTreeMap;

    /// Write a minimal finalized bag at `path`, optionally carrying a
    /// `record_health` attachment with the given bytes.
    fn write_min_bag(path: &Path, health: Option<&[u8]>) {
        let mut w = BagWriter::create(path, BagWriterConfig::default(), &[]).unwrap();
        w.write_chunk(|_c| Ok(())).unwrap();
        if let Some(bytes) = health {
            w.write_attachment(RECORD_HEALTH_ATTACHMENT, "application/json", 0, 0, bytes)
                .unwrap();
        }
        w.finalize().unwrap();
    }

    #[test]
    #[tracing_test::traced_test]
    fn read_record_health_absent_is_silent() {
        let dir = tempfile::tempdir().unwrap();
        let bag = dir.path().join("no_health.mcap");
        write_min_bag(&bag, None);
        let reader = BagReader::open(&bag).unwrap();
        let report = read_record_health(&reader, &bag).unwrap();
        assert!(
            matches!(report, RecordHealthReport::Absent),
            "a bag with no health document reads as Absent"
        );
        assert!(
            !logs_contain("record_health"),
            "the absent arm is silent (back-compat, no warn spam)"
        );
    }

    #[test]
    #[tracing_test::traced_test]
    fn read_record_health_malformed_warns_and_is_malformed() {
        let dir = tempfile::tempdir().unwrap();
        let bag = dir.path().join("bad_health.mcap");
        write_min_bag(&bag, Some(b"{ this is not valid json"));
        let reader = BagReader::open(&bag).unwrap();
        let report = read_record_health(&reader, &bag).unwrap();
        assert!(
            matches!(report, RecordHealthReport::Malformed),
            "unparseable JSON -> Malformed (treated as absent)"
        );
        assert!(
            logs_contain("malformed record_health.json attachment"),
            "the malformed arm warns LOUDLY"
        );
    }

    #[test]
    fn read_record_health_present_round_trips_bagd_built_json() {
        // Build the stamp with bagd's OWN types (single source of truth — the
        // `cerulion_bagd` lib edge already exists) and serialize it with bagd's
        // serde shape. Reading it back proves the replay side and the bagd write
        // side agree on the JSON format + the attachment name (format-lockstep).
        let mut topics = BTreeMap::new();
        let declared = TopicHealth {
            frames_recorded: 10,
            frames_lost: 4,
            gap_events: 2,
            multi_publisher: true,
            sequence_anomaly: false,
            first_seq: None,
            last_seq: None,
            baseline_resets_after_first: 0,
            gap_detection_disabled_reason: None,
            classify_calls: 0,
            defer_count: 0,
            staging_full_passes: 0,
            // Additive producer-label fields, vacant here.
            producer_labels: 0,
            label_catch_up: false,
            // Additive Options, absent here.
            tap_buffer_depth: None,
            shm_pinned_bytes: None,
            // Present for the same round-trip reason — the
            // enum's wire token has to cross too.
            loss_counting_basis: Some(cerulion_bagd::LossCountingBasis::PrefixProven),
            // PRESENT on purpose, for the same reason
            // the cadence fields above are non-zero — this test asserts a
            // byte-for-byte round trip, and the absent shape would prove
            // nothing about reading the new nested object back.
            absorbance: Some(cerulion_bagd::TopicAbsorbance {
                tap_buffer_depth: 4,
                rate_mhz: Some(300_000),
                rate_is_floor: true,
                absorbance_us: Some(13_333),
                measured_tail_us: Some(50_000),
                required_depth: Some(15),
                shortfall_at_least_us: Some(36_667),
                verdict: cerulion_bagd::AbsorbanceVerdict::Short,
                short_evaluations: 7,
                // FALSE beside a `Some(over_budget)`: the two are mutually
                // exclusive by construction (`TopicAbsorbance::inconsistency`
                // rejects the pair), so the round trip covers the shape a
                // producer can actually emit.
                budget_unpriced: false,
                budget_bytes: Some(4 * 1024 * 1024),
                over_budget: Some(cerulion_bagd::OverBudget {
                    budget_bytes: 4 * 1024 * 1024,
                    capacity_bytes: 4 * (1024 * 1024 + 40),
                    widest_slot_bytes: 1024 * 1024 + 40,
                }),
            }),
        };
        // A second row, discovered, so the document-wide basis below
        // is one a recorder can actually write. `floor_loss_counting_basis` is
        // the FLOOR over every tap's own, so a one-row document whose only row
        // reads `prefix_proven` cannot carry a `prefix_invisible` floor — and a
        // `prefix_proven` ROW already implies `armed_before_producers`, which is
        // the floor's other term. Two rows keep BOTH wire tokens exercised while
        // every value here stays producible. Its `absorbance` is absent, which
        // covers the skipped shape of the nested object beside `/tf`'s present
        // one.
        let discovered = TopicHealth {
            frames_recorded: 3,
            frames_lost: 0,
            gap_events: 0,
            multi_publisher: false,
            loss_counting_basis: Some(cerulion_bagd::LossCountingBasis::PrefixInvisible),
            absorbance: None,
            ..declared.clone()
        };
        topics.insert("/tf".to_string(), declared);
        topics.insert("/scan".to_string(), discovered);
        let expected = RecordHealth {
            version: RECORD_HEALTH_VERSION,
            dropped_unwritten: 5,
            // The drive-loop observables. Non-zero on purpose — this
            // test asserts a BYTE-FOR-BYTE round trip, so leaving them at their
            // skipped default would exercise the one shape where the new keys
            // are absent from the JSON and prove nothing about reading them.
            drive_passes: 4_211,
            max_pass_duration_us: 117_500,
            // Same reasoning — non-zero span and a populated
            // histogram, so the round trip exercises the PRESENT shape of both
            // new keys rather than their skipped default.
            drive_span_us: 73_900_000,
            drain_gaps: {
                let mut h = cerulion_bagd::DrainGapHistogram::new();
                h.record(std::time::Duration::from_micros(29_500));
                h
            },
            // Likewise PRESENT, so the round trip covers the
            // token rather than its skipped default.
            loss_counting_basis: Some(cerulion_bagd::LossCountingBasis::PrefixInvisible),
            topics,
        };
        let bytes = serde_json::to_vec(&expected).unwrap();

        let dir = tempfile::tempdir().unwrap();
        let bag = dir.path().join("present_health.mcap");
        write_min_bag(&bag, Some(&bytes));
        let reader = BagReader::open(&bag).unwrap();
        match read_record_health(&reader, &bag).unwrap() {
            RecordHealthReport::Present(health) => assert_eq!(
                health, expected,
                "the parsed stamp round-trips bagd's JSON byte-for-byte"
            ),
            other => panic!("expected Present, got {other:?}"),
        }
    }

    // =======================================================================
    // The recorder.json ADVISORY half degrades to
    // ABSENCE, never to a fabricated identity. Built over a REAL minimal MCAP
    // bag so the attachment NAME const, the split parse and the SERIALIZED
    // (`--report`) shape are pinned end-to-end.
    // =======================================================================

    /// Write a minimal finalized bag at `path` carrying a
    /// `__cerulion/recorder.json` attachment with the given bytes.
    fn write_min_bag_with_recorder(path: &Path, recorder: &[u8]) {
        let mut w = BagWriter::create(path, BagWriterConfig::default(), &[]).unwrap();
        w.write_chunk(|_c| Ok(())).unwrap();
        w.write_attachment(RECORDER_ATTACHMENT, "application/json", 0, 0, recorder)
            .unwrap();
        w.finalize().unwrap();
    }

    /// A recorder.json whose CONTRACT keys read perfectly and whose HOST
    /// IDENTITY does not must leave the UNREADABLE FIELD absent from the
    /// outcome — and therefore from `--report` — rather than filling it in, and
    /// must NOT take its readable siblings with it.
    ///
    /// A degraded arm building `RecorderInfo { arch: "", os: "",
    /// cerulion_version: "", recorded_at_ns: 0, .. }` is the defect this pins. Those are not absence
    /// markers on a machine-readable artifact: `"recorded_at_ns": 0` is a
    /// readable instant (1970-01-01) and an empty `arch` reads as an answer, so
    /// the one DURABLE thing a CI consumer parses would carry four values nobody
    /// measured while the only signal that they were unavailable is an
    /// ephemeral stderr warn. Absence rides the `Option`, never a sentinel.
    ///
    /// **Why the oracle reads per field.** Asserting `os == None` on a
    /// document whose `os` is well-formed, under the rule "the half is judged
    /// AS A WHOLE", describes a single whole-object
    /// `from_value::<RecorderInfo>`, since serde fails the document on its
    /// first bad member. But `RecorderInfo`'s own doc promises that a PARTIALLY
    /// readable document keeps the halves that decoded, and under a whole-object
    /// parse that promise holds
    /// only for an ABSENT field, not for the MALFORMED one a hand-edited or
    /// half-written attachment actually produces. The read is per-field, so
    /// one bad member costs exactly itself.
    ///
    /// Three parts in ONE body on purpose. The two degraded parts pin DIFFERENT
    /// fields (a string and the numeric one) so the warn cannot be satisfied by
    /// a hardcoded name and both extraction helpers are covered; the control is
    /// what makes "absent" mean something, and it discriminates on VALUES
    /// (`arch == "aarch64"`) rather than on a log, so it cannot be satisfied by
    /// a reader that took the degraded arm quietly.
    #[test]
    #[tracing_test::traced_test]
    fn a_malformed_host_identity_is_absent_from_the_report_not_fabricated() {
        let dir = tempfile::tempdir().unwrap();
        // The recording host's OS is spelled as THIS host's so the cross-arch
        // advisory has nothing to say — which lets the arm below pin that an
        // UNREADABLE `arch` never manufactures a mismatch, on every platform.
        let host_os = std::env::consts::OS;

        // ---- degraded: contract keys readable, ONE identity field is not ----
        // `"arch": 42` is the shape under test — a well-formed document
        // whose ADVISORY half cannot fully decode. Every CONTRACT key is valid,
        // so an arm that refused here would be refusing on something it can read.
        let bag = dir.path().join("bad_identity.mcap");
        write_min_bag_with_recorder(
            &bag,
            format!(
                r#"{{"arch":42,"os":"{host_os}","cerulion_version":"0.0.1","recorded_at_ns":7,"trace_format":3,"coordination":"lockstep"}}"#
            )
            .as_bytes(),
        );
        let reader = BagReader::open(&bag).unwrap();
        let read = read_recorder_info(&reader, &bag).unwrap();
        let info = read
            .info
            .expect("the CONTRACT keys survive a degraded advisory half");

        // The CONTRACT half is READ and APPLIED — which is exactly why this arm
        // yields a `RecorderInfo` rather than `None` (a `None` would discard the
        // version gate's and the coordination gate's only input).
        assert_eq!(
            info.trace_format, 3,
            "the contract key is applied: {info:?}"
        );
        assert_eq!(
            info.coordination,
            Some(crate::replay_engine::CoordinationMode::Lockstep),
            "…and so is the coordination stamp: {info:?}"
        );
        assert_eq!(
            read.unknown_coordination, None,
            "the stamp was known, so nothing is carried out for the caller to refuse"
        );

        // The UNREADABLE field is ABSENT — never a stand-in…
        assert_eq!(info.arch, None, "no fabricated arch: {info:?}");
        // …and its READABLE siblings SURVIVE. This is the point: a
        // whole-object parse loses all four to the first bad member, so a
        // `--report` consumer would lose metadata that was right there and perfectly
        // well-formed.
        assert_eq!(
            info.os.as_deref(),
            Some(host_os),
            "a sibling field that decoded must survive: {info:?}"
        );
        assert_eq!(
            info.cerulion_version.as_deref(),
            Some("0.0.1"),
            "…so must the version: {info:?}"
        );
        assert_eq!(info.recorded_at_ns, Some(7), "…and the timestamp: {info:?}");

        // THE `--report` CLAIM: the unreadable key is OMITTED from the
        // serialized form (the struct field could be `None` and still serialize
        // as a `null` a consumer reads as data) while the readable ones appear.
        let json = serde_json::to_value(&info).expect("recorder info serializes");
        assert!(
            json.get("arch").is_none(),
            "`--report` must OMIT `arch` rather than carry a stand-in: {json}"
        );
        for (key, expected) in [
            ("os", serde_json::json!(host_os)),
            ("cerulion_version", serde_json::json!("0.0.1")),
            ("recorded_at_ns", serde_json::json!(7)),
        ] {
            assert_eq!(
                json[key], expected,
                "`--report` keeps the identity it COULD read: {json}"
            );
        }
        assert_eq!(
            json["trace_format"], 3,
            "…while the contract keys it DID read are reported: {json}"
        );
        assert!(
            logs_contain("HOST IDENTITY half did not decode"),
            "the degradation is LOUD as well as reported"
        );
        assert!(
            logs_contain("fields=arch"),
            "…and NAMES the field that was lost, so the warn is actionable"
        );
        assert!(
            !logs_contain("cross-architecture/OS replay"),
            "an UNREADABLE arch is not a mismatch — the advisory claims a \
             difference only where it has a value to compare"
        );

        // ---- degraded, the NUMERIC field: the name is not hardcoded ----
        // `recorded_at_ns` goes through the other extraction helper, and a
        // second bad-field shape is what stops `fields=arch` above from being
        // satisfied by a warn that always says "arch".
        let host_arch = std::env::consts::ARCH;
        let bag = dir.path().join("bad_timestamp.mcap");
        write_min_bag_with_recorder(
            &bag,
            format!(
                r#"{{"arch":"{host_arch}","os":"{host_os}","cerulion_version":"0.0.1","recorded_at_ns":"soon","trace_format":3}}"#
            )
            .as_bytes(),
        );
        let reader = BagReader::open(&bag).unwrap();
        let info = read_recorder_info(&reader, &bag)
            .unwrap()
            .info
            .expect("the contract half still yields an info");
        assert_eq!(
            info.recorded_at_ns, None,
            "a non-numeric timestamp is absent, never 0 (1970-01-01 reads as an \
             answer): {info:?}"
        );
        assert_eq!(
            info.arch.as_deref(),
            Some(host_arch),
            "and the three readable fields survive it: {info:?}"
        );
        assert!(
            logs_contain("fields=recorded_at_ns"),
            "the warn names THIS field — the naming is per-field, not a constant"
        );

        // ---- ANTI-TAUTOLOGY control: a well-formed identity is carried ----
        let bag = dir.path().join("good_identity.mcap");
        write_min_bag_with_recorder(
            &bag,
            br#"{"arch":"aarch64","os":"macos","cerulion_version":"0.1.0","recorded_at_ns":7,"trace_format":3,"coordination":"lockstep"}"#,
        );
        let reader = BagReader::open(&bag).unwrap();
        let info = read_recorder_info(&reader, &bag)
            .unwrap()
            .info
            .expect("a well-formed recorder.json parses");
        assert_eq!(info.arch.as_deref(), Some("aarch64"), "{info:?}");
        assert_eq!(info.os.as_deref(), Some("macos"), "{info:?}");
        assert_eq!(info.cerulion_version.as_deref(), Some("0.1.0"), "{info:?}");
        assert_eq!(info.recorded_at_ns, Some(7), "{info:?}");
        let json = serde_json::to_value(&info).expect("recorder info serializes");
        assert_eq!(
            json["arch"], "aarch64",
            "a healthy bag's report is unchanged: {json}"
        );
        assert_eq!(json["recorded_at_ns"], 7, "{json}");
    }

    // ── The inert-tolerance warn ───────────────────────────

    #[test]
    #[tracing_test::traced_test]
    fn inert_warn_fires_only_for_an_all_bit_exact_document() {
        // An all-`bit_exact` document (here: an explicit bit_exact default with a
        // topic listing only a bit_exact field) has NO effect — so the
        // warn fires.
        let spec = crate::tolerance::ToleranceSpec::parse(
            "default_metric:\n  kind: bit_exact\ntopics:\n  /a:\n    fields:\n      x:\n        kind: bit_exact",
        )
        .unwrap();
        warn_on_inert_tolerance(&spec);
        assert!(
            logs_contain("every metric in this document resolves to bit_exact"),
            "an all-bit_exact document must warn it is a no-op"
        );
    }

    #[test]
    #[tracing_test::traced_test]
    fn inert_warn_is_silent_for_a_functional_topic_wide_metric() {
        // A bare topic-wide non-bit_exact metric is functional (it
        // expands over the topic's fields), so it is NOT inert — no warn (warning
        // here would be a false claim).
        let spec = crate::tolerance::ToleranceSpec::parse(
            "topics:\n  /a:\n    metric:\n      kind: set_equal",
        )
        .unwrap();
        warn_on_inert_tolerance(&spec);
        assert!(
            !logs_contain("has NO effect"),
            "a functional topic-wide metric must NOT be warned as inert"
        );
    }

    #[test]
    #[tracing_test::traced_test]
    fn inert_warn_is_silent_for_a_functional_default_metric() {
        // A non-bit_exact default_metric is functional (covers every
        // field of every topic) — not inert.
        let spec = crate::tolerance::ToleranceSpec::parse(
            "default_metric:\n  kind: max_abs\n  threshold: 0.1",
        )
        .unwrap();
        warn_on_inert_tolerance(&spec);
        assert!(
            !logs_contain("has NO effect"),
            "a functional default_metric must NOT be warned as inert"
        );
    }
}

#[cfg(test)]
mod resim_identity_tests {
    use super::{resim_identity_max_len, resolve_resim_identity, DeclineCause, ResimIdentity};
    use crate::replay_engine::REPLAY_NODE_NAME_PREFIX;
    use std::path::{Path, PathBuf};

    /// The nameless embed and its file-stem identity, in one arm.
    ///
    /// `graph create` writes no `name:`, so a bag recorded from such a
    /// graph embeds a `graph.yaml` with nothing to name it — and
    /// `parse_graph`'s seed leaves `identity` empty, which `identity()` serves
    /// as `unnamed`. The bag's own file name still carries it, because the
    /// recorder stemmed the file BY it.
    ///
    /// The oracle is the WHOLE stem, `_{utc_stamp}` included: keeping it is the
    /// no-inference rule, and asserting it is what fails a "helpfully" stripped
    /// implementation.
    #[test]
    fn a_nameless_embed_is_named_by_the_bags_own_file_stem() {
        assert_eq!(
            resolve_resim_identity(
                "",
                Path::new("/var/cerulion/recordings/perception_20260101T000000Z.mcap"),
            ),
            ResimIdentity::Adopted("perception_20260101T000000Z".to_string()),
        );
        // A bag the operator renamed carries the name they chose, verbatim.
        assert_eq!(
            resolve_resim_identity("", Path::new("./yesterdays run.mcap")),
            ResimIdentity::Adopted("yesterdays run".to_string()),
        );
    }

    /// BACK-COMPAT / anti-tautology: a bag that DID name itself keeps its name.
    ///
    /// Without this the file-stem fallback is indistinguishable from "always rename the
    /// resim after the file", which would change the identity of every bag that
    /// names itself, where the fallback exists only for the ones that do not.
    #[test]
    fn an_embed_that_named_itself_is_left_alone() {
        assert_eq!(
            resolve_resim_identity("recorded_run", Path::new("/rec/something_else.mcap")),
            ResimIdentity::Declared,
        );
        // …even when the bag file agrees, so the arm cannot pass on equality.
        assert_eq!(
            resolve_resim_identity("recorded_run", Path::new("/rec/recorded_run.mcap")),
            ResimIdentity::Declared,
        );
    }

    /// A path with no stem to take declines and names the cause,
    /// rather than adopting an empty identity that would render as nothing.
    #[test]
    fn a_path_with_no_file_stem_declines_and_says_why() {
        for p in ["/", "..", "."] {
            assert_eq!(
                resolve_resim_identity("", Path::new(p)),
                ResimIdentity::Declined(DeclineCause::NoStem),
                "`{p}` has no adoptable stem"
            );
        }
    }

    /// An identity crosses into a `&str` log field and a `&str` node name, so a
    /// non-UTF-8 file name declines rather than being lossily coerced.
    ///
    /// No `#[cfg(unix)]` needed: the whole module is Unix-gated (`lib.rs`), so
    /// `OsStrExt` is always in scope here.
    #[test]
    fn a_non_utf8_file_stem_declines() {
        use std::os::unix::ffi::OsStrExt;
        let raw = std::ffi::OsStr::from_bytes(b"bad\xffname.mcap");
        assert_eq!(
            resolve_resim_identity("", Path::new(raw)),
            ResimIdentity::Declined(DeclineCause::NotUtf8),
        );
    }

    /// The length bound, pinned on BOTH sides — and DECLINED, never truncated.
    ///
    /// Truncation would be a second inference AND a collision: two bags sharing
    /// a long prefix would mint one node name between them.
    #[test]
    fn an_over_long_stem_declines_instead_of_truncating() {
        let max = resim_identity_max_len();
        let at = "b".repeat(max);
        let over = "b".repeat(max + 1);
        assert_eq!(
            resolve_resim_identity("", &PathBuf::from(format!("/rec/{at}.mcap"))),
            ResimIdentity::Adopted(at.clone()),
            "exactly at the bound is adoptable"
        );
        assert_eq!(
            resolve_resim_identity("", &PathBuf::from(format!("/rec/{over}.mcap"))),
            ResimIdentity::Declined(DeclineCause::TooLong { len: max + 1, max }),
        );
    }

    /// The bound is the REAL one — driven through iceoryx2 itself, not asserted
    /// against arithmetic this module also wrote.
    ///
    /// An identity one byte past the bound costs a resim its run
    /// (`TransportManager::init` refuses it). A drift in
    /// `MAX_NODE_NAME_LENGTH` or in the prefix must fail
    /// HERE, not on a robot.
    #[test]
    fn the_bound_is_exactly_what_an_iceoryx2_node_name_leaves_for_an_identity() {
        let max = resim_identity_max_len();
        assert_eq!(
            REPLAY_NODE_NAME_PREFIX.len() + max,
            iceoryx2::prelude::NodeName::max_len(),
            "the bound must SPEND the whole cap, or resims are named more \
             narrowly than they could be"
        );
        let longest = format!("{REPLAY_NODE_NAME_PREFIX}{}", "b".repeat(max));
        assert!(
            iceoryx2::prelude::NodeName::new(&longest).is_ok(),
            "the longest adoptable identity must still mint a node name"
        );
        let one_more = format!("{REPLAY_NODE_NAME_PREFIX}{}", "b".repeat(max + 1));
        assert!(
            iceoryx2::prelude::NodeName::new(&one_more).is_err(),
            "…and one byte more must not — otherwise the bound is not the real one"
        );
    }

    /// A VALID-UTF-8 but non-ASCII stem DECLINES — it does not panic.
    ///
    /// UTF-8 is not sufficient for an iceoryx2 node name: `StaticString`'s
    /// `insert_bytes` refuses any byte `>= 128`, so `naïve` is rejected while
    /// being perfectly good Rust. A fallback that adopted it would hand
    /// `TransportManager::init` a name it refuses, and the resim
    /// would fail with a transport error — a worse outcome than the
    /// `unnamed` this fallback exists to avoid. (The conversion
    /// refuses rather than aborts, so the fallback is not the only thing
    /// standing between a stem and a panic — but declining still beats refusing
    /// a run over a name it was only guessing at.)
    ///
    /// Every shape here is one a real bag file can carry, and the ASCII
    /// CONTROL in the same body is what stops "declines" from being satisfied
    /// by a fallback that adopts nothing at all.
    #[test]
    fn a_non_ascii_stem_declines_rather_than_panicking_the_resim() {
        for stem in ["naïve", "ré-run", "日本語", "bag→2", "robot_café", "run✅"] {
            assert_eq!(
                resolve_resim_identity("", &PathBuf::from(format!("/rec/{stem}.mcap"))),
                ResimIdentity::Declined(DeclineCause::NotRepresentable),
                "`{stem}` is valid UTF-8 but not a valid node name"
            );
        }
        // CONTROL: the ASCII sibling of the first shape is still adopted, so
        // the arm cannot pass on a resolver that declines everything.
        assert_eq!(
            resolve_resim_identity("", Path::new("/rec/naive.mcap")),
            ResimIdentity::Adopted("naive".to_string()),
        );
    }

    /// The charset boundary, driven through iceoryx2 itself on BOTH sides.
    ///
    /// U+007F is the last code point a node name admits and U+0080 the first it
    /// refuses (`insert_bytes`: `128 <= byte`), so this pins the exact edge
    /// rather than "some accented word is rejected" — and it pins it against
    /// the authority, so a widened charset in a future iceoryx2 shows up here
    /// as a failure rather than as silently narrower behaviour.
    #[test]
    fn the_charset_boundary_is_pinned_on_both_sides_against_iceoryx2_itself() {
        let admitted = format!("ok{}", '\u{7f}');
        let refused = format!("ok{}", '\u{80}');
        assert!(
            iceoryx2::prelude::NodeName::new(&format!("{REPLAY_NODE_NAME_PREFIX}{admitted}"))
                .is_ok(),
            "U+007F must still be a valid node name, or the boundary moved"
        );
        assert_eq!(
            resolve_resim_identity("", &PathBuf::from(format!("/rec/{admitted}.mcap"))),
            ResimIdentity::Adopted(admitted.clone()),
        );
        assert_eq!(
            resolve_resim_identity("", &PathBuf::from(format!("/rec/{refused}.mcap"))),
            ResimIdentity::Declined(DeclineCause::NotRepresentable),
        );
    }

    /// The length arm keeps its own reason: an over-long ASCII stem reports its
    /// two NUMBERS, not the generic charset refusal.
    ///
    /// Both checks would refuse it (`NodeName::new` fails on capacity too), so
    /// this is what pins the ORDER — the arm exists to give an operator the
    /// figures they can act on.
    #[test]
    fn an_over_long_ascii_stem_reports_its_length_not_the_charset() {
        let max = resim_identity_max_len();
        let over = "b".repeat(max + 1);
        assert_eq!(
            resolve_resim_identity("", &PathBuf::from(format!("/rec/{over}.mcap"))),
            ResimIdentity::Declined(DeclineCause::TooLong { len: max + 1, max }),
            "the length arm must win, or the operator loses the two numbers"
        );
    }

    /// Every decline reason NAMES its cause and is distinct, so a `debug!` line
    /// tells an operator which of the four happened.
    #[test]
    fn each_decline_reason_names_its_own_cause() {
        let no_stem = DeclineCause::NoStem.reason();
        let not_utf8 = DeclineCause::NotUtf8.reason();
        let too_long = DeclineCause::TooLong { len: 200, max: 112 }.reason();
        let charset = DeclineCause::NotRepresentable.reason();
        assert!(no_stem.contains("no file stem"), "{no_stem}");
        assert!(not_utf8.contains("UTF-8"), "{not_utf8}");
        assert!(
            too_long.contains("200") && too_long.contains("112"),
            "the over-long reason must carry BOTH numbers, got: {too_long}"
        );
        assert!(
            too_long.contains(REPLAY_NODE_NAME_PREFIX),
            "…and name the prefix that spends the budget, got: {too_long}"
        );
        // The charset refusal must say WHAT is admitted, or an operator staring
        // at a perfectly valid file name has nothing to act on. It must ALSO
        // not read as a UTF-8 complaint — the stem IS valid UTF-8, and the
        // `NotUtf8` arm is a different condition with a different remedy.
        assert!(
            charset.contains("ASCII") && charset.contains("U+0080"),
            "the charset reason must name the admitted set, got: {charset}"
        );
        assert!(
            charset.contains("valid UTF-8"),
            "…and say the stem IS valid UTF-8, so it is not read as that arm: {charset}"
        );
        for (a, b) in [
            (&no_stem, &not_utf8),
            (&not_utf8, &too_long),
            (&too_long, &charset),
            (&charset, &no_stem),
        ] {
            assert_ne!(a, b, "every decline reason must be distinguishable");
        }
    }
}

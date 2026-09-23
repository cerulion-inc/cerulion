// SPDX-License-Identifier: AGPL-3.0-only
//! The PURE half of live-topic discovery — "record what EXISTS, not
//! what was declared".
//!
//! # The defect this closes
//!
//! A recorder whose topic universe is only the STATIC graph
//! declaration (`graph run --record` resolves `config.nodes[].outputs[]` into a
//! `--topics-json` tap list and bagd taps exactly those) misses any producer
//! registered DYNAMICALLY — created at runtime rather than declared in the
//! graph YAML — silently.
//!
//! The flagship instance is `cerulion ros2 attach`: its generated graph declares
//! the `dds_bridge` node with FOUR typed output ports, while the bridge opens
//! one `create_ingress_publisher` per DISCOVERED DDS topic (~71 on a real robot —
//! camera H.264, `/lowstate`, `/tf`, lidar, every `api/*` route). Those
//! publishers are real, live, single-writer SHM producers on the recorder's own
//! SHM root; they are simply absent from the YAML. Without discovery a `--record` run captures
//! ~1813 messages while `/lowstate` alone streams at 499 Hz — and bagd
//! finalizes with `frames_lost=0` and "recording complete", saying nothing.
//!
//! The class is general (any dynamically-registered producer); the attach
//! bridge is only its most visible instance.
//!
//! # What this module owns
//!
//! The DECISION, kept pure so it is oracle-testable without a transport:
//! [`plan_discovery`] takes a live topic snapshot plus what the recorder already
//! knows and returns which topics to ATTACH and which to SKIP **with a reason**.
//! It performs no I/O, opens no port, and reads no clock.
//!
//! Enumeration itself (`TransportManager::list_topics`, an iceoryx2
//! `Service::list` over the recorder's own SHM root) and tap attachment live in
//! `lib.rs`; the reporting of the result lives in
//! [`crate::RecordCoverage`].

use std::collections::{BTreeMap, BTreeSet};
use std::time::Duration;

use cerulion_bag::RESERVED_PREFIX;

/// The enumeration CADENCE quantum.
///
/// Two roles, and the first is the one that matters on a settled machine:
///
/// * It is the quantum the settle window is built from
///   (`DISCOVERY_SETTLE_MIN` = this times `DISCOVERY_SETTLE_QUIET_SCANS`), so
///   lowering it shortens that window unless the quiet threshold rises to
///   compensate.
/// * It is the cadence the recorder falls back to when it cannot WATCH the
///   iceoryx2 service directory for changes. It is NOT the shipping discovery
///   mechanism: enumeration is driven by filesystem events (see
///   `crate::service_dir_watch`), because a `Service::list` walk costs 117.5 ms
///   at 86 live topics and running one four times a second forever, on a
///   machine whose topic set never changes, is a permanent CPU cost for an
///   answer nobody asked for.
///
/// Discovery is a SNAPSHOT of a moving world: the recorder is armed BEFORE the
/// graph is released to step 0 (the taps-ready → GO handshake), so at arm time
/// a dynamically-registered producer does not exist yet. One scan at arm would
/// therefore find nothing on exactly the path this feature exists to fix -
/// re-enumerating IS the mechanism, not a refinement of it, and the only
/// question is what triggers it.
///
/// The window a topic must be discovered INSIDE — to get a channel, since MCAP
/// channels are registered at bag creation and immutable afterwards — is the
/// SETTLE HOLD: at least `DISCOVERY_SETTLE_MIN`, re-extended by every
/// enumeration that finds something, capped at `DEFAULT_DISCOVERY_SETTLE_MS`.
/// It is NOT the `--schema-wait-timeout-ms` grace: that is a force-create
/// DEADLINE governing schema learning on already-tapped topics, and on the
/// `graph run --record` path every declared tap is exact-mode, so the bag would
/// be created on the first drive-loop pass however large it is set.
///
/// An enumeration is a `Service::list` walk over the SHM service directory; it
/// holds no port and touches no data plane.
pub const DISCOVERY_RESCAN_INTERVAL: Duration = Duration::from_millis(DISCOVERY_RESCAN_INTERVAL_MS);

/// [`DISCOVERY_RESCAN_INTERVAL`] in milliseconds — the scalar
/// [`crate::DISCOVERY_SETTLE_MIN`] is derived from, so the cadence and the
/// settle floor cannot drift apart.
pub const DISCOVERY_RESCAN_INTERVAL_MS: u64 = 250;

/// Ceiling on DISCOVERED taps (declared taps are never counted against it and
/// are never refused — an explicit request outranks an inference).
///
/// Each tap costs one iceoryx2 subscriber slot on that topic's service plus its
/// held-sample borrows, so an unbounded discovery on a machine with thousands of
/// topics would exhaust slots that belong to the running graph. 256 sits an
/// order of magnitude above the motivating deployment (~90 routes on a live
/// Go2) while still being a bound.
///
/// Refusal is never silent: a topic past the budget is reported in the coverage
/// manifest with [`UntappedReason::BudgetExhausted`].
pub const DISCOVERY_MAX_TAPS: usize = 256;

/// Topic-name prefixes discovery NEVER auto-taps.
///
/// - `__cerulion/` (both spellings) is the bag format's OWN reserved namespace.
///   `cerulion_bag`'s writer rejects a USER topic under it
///   (`BagError::ReservedTopicPrefix`), so tapping one could not produce a
///   channel. The reserved channels the writer registers itself are exempt from
///   that rejection and are not live SHM topics in any case.
/// - `/bagd/` is the recorder's own `/bagd/status` channel: auto-recording it
///   means a recorder recording itself, which grows without bound and tells the
///   operator nothing.
///
/// This is the same PREFIX list `cerulion bag record`'s `--all` / `--regex`
/// auto-select applies (`cerulion_cli_engine::bag_cmd::AUTO_SELECT_EXCLUDED_PREFIXES`).
/// It is only HALF of that verb's policy: the other half is the netd-MIRROR
/// fold, which this module applies separately in [`plan_discovery`] via the
/// shared `mirror_registry` predicate (see [`UntappedReason::RemoteMirror`]).
/// Two auto-selection paths must not disagree about what "everything" means —
/// and originally they did, because the prefix list was copied
/// across and the mirror fold was not.
///
/// Note what needs no entry here: the framework's control services
/// (`/__cerulion/gateway_topics`, `/__cerulion/mirrors`) and the WaitSet
/// external doorbells (`cer_ext_doorbell/{node}/event`) are iceoryx2 EVENT
/// services with no `/data` counterpart, and `TransportManager::list_topics`
/// derives a topic name only from a `/data` service — so the enumerator never
/// yields them. Trace/recording rings and cross-process barriers are POSIX SHM
/// objects, not iceoryx2 services, so they cannot appear either.
pub const EXCLUDED_TOPIC_PREFIXES: &[&str] = &["__cerulion/", "/__cerulion/", "/bagd/"];

// The reserved prefix is owned by `cerulion_bag`; pin our copy against it so a
// change there cannot silently leave this list stale (the same guard
// `bag_cmd::AUTO_SELECT_EXCLUDED_PREFIXES` carries).
const _: () = assert!(
    matches!(RESERVED_PREFIX.as_bytes(), b"__cerulion/"),
    "EXCLUDED_TOPIC_PREFIXES hardcodes the reserved prefix — update both together"
);

/// Why a LIVE producer is not being recorded.
///
/// Every variant is reported by name in the coverage manifest and in the
/// terminal log. A recorder must either tap what exists or SAY what it is not
/// tapping; this enum is the vocabulary of the second half.
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(tag = "reason", rename_all = "snake_case")]
pub enum UntappedReason {
    /// A framework-internal / recorder-owned name (see
    /// [`EXCLUDED_TOPIC_PREFIXES`]).
    ExcludedInternal,
    /// The topic is a `cerulion-netd` MIRROR of a REMOTE robot's stream, not a
    /// producer on this machine.
    ///
    /// A mirror is a real local `{topic}/data` service, so the enumerator yields
    /// it like any other — but recording it would claim a local capture of
    /// another robot's data (the "one data source = one topic" decision,
    /// which `cerulion bag record --all` already honours by
    /// folding mirrors out of auto-selection).
    RemoteMirror {
        /// The origin robot the mirror is attributed to.
        robot: String,
    },
    /// The discovered-tap ceiling ([`DISCOVERY_MAX_TAPS`]) was already full.
    BudgetExhausted {
        /// The ceiling that was hit.
        budget: usize,
    },
    /// The topic's producer appeared AFTER the bag was created.
    ///
    /// MCAP channels are registered when the bag is created and are immutable
    /// afterwards (`BagWriter::create` takes the complete topic list up front,
    /// and rotation reuses that set verbatim), so a topic discovered later can
    /// be given no channel to write into.
    ///
    /// The commonest cause of this verdict (a `ros2 attach` bridge opening its
    /// raw routes on a COLD DDS bus) is largely self-correcting: the recorder
    /// also holds bag creation open while a process on the machine is
    /// demonstrably still CREATING runtime ingress routes, so a burst-and-gap
    /// build is carried across its gaps with no operator intervention. This
    /// verdict survives for what that hold cannot see: a producer appearing long
    /// after its plane settled, a build that outran the hold's ceiling, and any
    /// late producer that is not an ingress route at all.
    ///
    /// The manual remedy is the discovery SETTLE window, which holds bag creation open:
    /// `CERULION_RECORD_DISCOVERY_SETTLE_MS`, which reaches EVERY path —
    /// neither verb that reaches this verdict by default accepts a settle flag
    /// of its own, `cerulion graph run --record` because it builds the
    /// recorder's command line itself and `cerulion bag record --run`
    /// because it assembles the config IN-PROCESS and spawns no
    /// recorder at all — or `--discovery-settle-ms` when running
    /// `cerulion bagd` by hand. It is
    /// NOT `--schema-wait-timeout-ms`: that grace is a force-create DEADLINE
    /// governing schema learning on already-tapped topics, and on the
    /// `graph run --record` path every declared tap is exact-mode, so the bag
    /// would be created on the first drive-loop pass however large it is set.
    AppearedAfterBagCreation,
    /// The data-only tap could not be opened (slot exhaustion, a borrow budget
    /// below the recorder's floor, a service that vanished between the scan and
    /// the open). Carries the transport's own message.
    AttachFailed {
        /// The transport error, verbatim.
        error: String,
    },
    /// The CALLER declared this topic and no live service backed
    /// it when the recording armed.
    ///
    /// The odd one out, deliberately: every other variant describes a LIVE
    /// producer this recorder chose not to tap, while this one describes a topic
    /// that was NAMED and had no producer at all. It exists because
    /// `bag record --run` derives its set from a run's DECLARATION
    /// (`config.nodes[].outputs[]`), and a graph legitimately holds outputs that
    /// have not published yet or belong to nodes that never fire in this
    /// configuration — so refusing the whole recording over one quiet output
    /// would make `--run` unusable on precisely the graphs it exists for, while
    /// dropping it in silence would leave the bag claiming, by omission, that
    /// the run declared only what the bag contains.
    ///
    /// Only the CALLER can mint it (`BagdConfig::declared_untapped`): the
    /// recorder is handed a tap vector and cannot know which names were DERIVED
    /// from a declaration versus chosen by an operator — the same
    /// provenance-lives-one-level-up rule as `discover_live` and
    /// `armed_before_producers`.
    DeclaredNotLive,
    /// The OPERATOR took this topic out
    /// (`cerulion bag record --exclude <pattern>`), and it is live.
    ///
    /// It exists because turning discovery ON for `--run`
    /// would otherwise silently DEFEAT `--exclude`, which is
    /// meaningful on the declaration side only: [`plan_discovery`] filters on the
    /// internal-prefix list, the netd-mirror fold and `known` — and `known` is
    /// seeded from the TAP set, which the exclusion has already removed the
    /// topic from. So the rescan re-finds it as an undeclared live producer and
    /// RECORDS it. MEASURED without this variant: stdout announces one topic
    /// and the bag holds the excluded one with 291 frames, `untapped = {}`.
    ///
    /// Why it is ACCOUNTED FOR rather than silent, when `derive_run_topics`'
    /// rule 3 says an excluded topic is "neither recorded nor reported as
    /// missing": rule 3's subject is the MISSING report — the
    /// [`DeclaredNotLive`](Self::DeclaredNotLive) row that tells an operator a
    /// declared output never produced — and this is not that. A row here is not
    /// a report of absence; it is the recorder's account of a producer it SAW
    /// and deliberately did not tap, which is the same thing
    /// [`ExcludedInternal`](Self::ExcludedInternal) and
    /// [`RemoteMirror`](Self::RemoteMirror) already are, and it is
    /// [not a coverage gap](Self::is_coverage_gap) exactly as they are not.
    /// Dropping it in silence would break the one claim `enumerated: true`
    /// makes — *`untapped` names every live producer this recorder did not
    /// record*. That is the defect this module closes, verbatim: a manifest that
    /// looks clean while a live producer went unrecorded.
    ///
    /// Carries the PATTERN that matched, on the
    /// [`RemoteMirror`](Self::RemoteMirror) / [`AttachFailed`](Self::AttachFailed)
    /// precedent: with several `--exclude` patterns in play, the tag alone
    /// withholds the one fact an operator can act on — WHICH of their patterns
    /// caught this topic.
    ///
    /// Only the CALLER can mint it (`BagdConfig::discovery_excludes`): bagd's
    /// own CLI has no `--exclude`, so the patterns exist one level up, in the
    /// verb that owns the selection — the same provenance rule as
    /// [`DeclaredNotLive`](Self::DeclaredNotLive).
    ExcludedByRequest {
        /// The `--exclude` pattern that matched, verbatim.
        pattern: String,
    },
}

impl UntappedReason {
    /// A stable, greppable one-word tag for the terminal log.
    pub fn tag(&self) -> &'static str {
        match self {
            Self::ExcludedInternal => "excluded_internal",
            Self::RemoteMirror { .. } => "remote_mirror",
            Self::BudgetExhausted { .. } => "budget_exhausted",
            Self::AppearedAfterBagCreation => "appeared_after_bag_creation",
            Self::AttachFailed { .. } => "attach_failed",
            Self::DeclaredNotLive => "declared_not_live",
            Self::ExcludedByRequest { .. } => "excluded_by_request",
        }
    }

    /// Whether the verdict is FINAL for this run.
    ///
    /// A terminal verdict is remembered so the topic is never re-considered (and
    /// never re-logged); a non-terminal one is retried on the next scan, because
    /// the condition can genuinely clear — a subscriber slot freed by a
    /// departing `topic echo` makes a previously-refused attach succeed.
    pub fn is_terminal(&self) -> bool {
        match self {
            // Nothing about a name changes mid-run.
            Self::ExcludedInternal => true,
            // Channels are immutable once the bag exists; this cannot heal.
            Self::AppearedAfterBagCreation => true,
            // The budget is measured against the count of SUCCESSFULLY-ATTACHED
            // discovered taps, and a tap is never removed for the life of the
            // recorder — so that count is monotonically non-decreasing and a
            // budget refusal provably cannot clear. (This once
            // returned `false` under a comment claiming the budget "can clear
            // because the set is recomputed each scan". Recomputing it cannot
            // LOWER it, so the topic was re-planned on every scan and, once the
            // bag existed, RE-LABELLED `AppearedAfterBagCreation` — replacing a
            // true reason with a false one carrying an unusable remedy.)
            Self::BudgetExhausted { .. } => true,
            // A mirror's origin does not change within one recording. Stated
            // residual: if netd RETIRES a mirror mid-run and a genuine local
            // producer then claims that exact name, this recorder will not pick
            // it up — it stays reported as `remote_mirror` rather than being
            // silently recorded as local, which is the safer of the two errors.
            Self::RemoteMirror { .. } => true,
            // Genuinely transient: a subscriber slot freed by a departing
            // `topic echo` makes a previously-refused attach succeed.
            Self::AttachFailed { .. } => false,
            // TERMINAL, and the reason is the LEDGER PRUNE rather than the
            // condition: `rescan_discovery` retains a non-terminal verdict only
            // while its topic is LIVE, and this verdict's whole subject is a
            // topic that is NOT live — so a non-terminal spelling would be
            // swept off the ledger by the first rescan and the manifest would
            // lose the one fact it was seeded to carry.
            //
            // Being terminal costs nothing on the heal path, because terminal
            // here is NOT the same as never-reconsidered: the recorder seeds
            // the ledger WITHOUT seeding `discovery_known`, so if the producer
            // does appear the ordinary discovery attach records it and CLEARS
            // this entry (`discovery_ledger.remove`) — the manifest describes
            // the run's outcome, not its history.
            Self::DeclaredNotLive => true,
            // Nothing about a name — or about the pattern set, which is fixed
            // for the run — changes mid-recording. Same shape as
            // `ExcludedInternal`, and terminal for the same reason: a
            // re-decided exclusion would re-log every 250 ms.
            Self::ExcludedByRequest { .. } => true,
        }
    }

    /// Whether this verdict is a COVERAGE GAP — a live producer whose absence
    /// from the bag the operator should act on.
    ///
    /// Exclusions by RULE are not gaps: the recorder's own status channel and a
    /// remote robot's mirror are deliberately not this recording's data, and
    /// counting them would train an operator to skim the number that matters.
    ///
    /// [`DeclaredNotLive`](Self::DeclaredNotLive) is not a gap
    /// either, and the reason is this method's own subject line — a gap is *a
    /// LIVE producer whose absence the operator should act on*, and that topic
    /// had no producer. Counting it would escalate `is_incomplete()` and the
    /// terminal WARN on every ordinary `bag record --run` whose graph holds one
    /// output that has not fired, which is the normal shape of a real graph and
    /// exactly the case `TopicSelection::Run` exists to tolerate. It is still
    /// ACCOUNTED FOR — it rides `untapped` with its own reason tag, so
    /// `bag info` and any manifest reader see it.
    ///
    /// [`ExcludedByRequest`](Self::ExcludedByRequest)
    /// is not a gap either — the operator asked for it to be out, which is the
    /// same kind of fact as the two rule-based exclusions above, and escalating
    /// on it would WARN on every recording that used `--exclude` for its
    /// intended purpose.
    pub fn is_coverage_gap(&self) -> bool {
        matches!(self.class(), UntappedClass::Gap)
    }

    /// Whether the topic was taken OUT by a rule — the recorder's own
    /// namespaces, another robot's mirror, or the operator's `--exclude`.
    ///
    /// The complement within the not-a-gap set is
    /// [`DeclaredNotLive`](Self::DeclaredNotLive), which is not an exclusion at
    /// all: nothing took that topic out, it simply had no producer. Both
    /// surfaces that report a not-a-gap COUNT (the recorder's terminal line and
    /// `bag info`) split on this predicate, so neither can file a
    /// never-produced topic under a heading that says a rule excluded it.
    pub fn is_excluded_by_rule(&self) -> bool {
        matches!(self.class(), UntappedClass::ExcludedByRule)
    }

    /// The ONE place a reason is sorted into the three
    /// buckets every reporting surface prints.
    ///
    /// # Why this exists rather than two independent predicates
    ///
    /// The recorder's terminal line and `bag info`'s renderer both split
    /// `untapped` three ways, and doing it with DIFFERENT shapes is fragile:
    /// `bag info` partitioning `is_coverage_gap` / `is_excluded_by_rule` / an
    /// `else` CATCH-ALL, while the terminal counts `matches!(r,
    /// DeclaredNotLive)` explicitly. Both are right for today's seven variants
    /// and neither is right by construction — an eighth not-a-gap,
    /// not-by-rule reason would be MISLABELLED "declared by the run but never
    /// live" by the renderer's catch-all and counted in NEITHER bucket by the
    /// terminal, so the "the two surfaces cannot disagree" claim both sites make
    /// would be scoped to a variant list rather than enforced.
    ///
    /// This match is EXHAUSTIVE with no wildcard, and both surfaces classify
    /// through it, so a new variant is a COMPILE ERROR here (and, if it needs a
    /// fourth bucket, at both renderers) instead of a silently wrong label.
    /// `is_coverage_gap` / `is_excluded_by_rule` are kept as the names callers
    /// already read and are now DERIVED from this, so there is exactly one
    /// classification in the crate (the refuse-a-second-copy rule).
    pub fn class(&self) -> UntappedClass {
        match self {
            // Taken OUT by a rule: the recorder's own namespaces, another
            // robot's mirror, the operator's own `--exclude`.
            Self::ExcludedInternal | Self::RemoteMirror { .. } | Self::ExcludedByRequest { .. } => {
                UntappedClass::ExcludedByRule
            }
            // Nothing took it out — it was ASKED FOR and had no producer.
            Self::DeclaredNotLive => UntappedClass::DeclaredNotLive,
            // A LIVE producer whose absence from the bag the operator acts on.
            Self::BudgetExhausted { .. }
            | Self::AppearedAfterBagCreation
            | Self::AttachFailed { .. } => UntappedClass::Gap,
        }
    }
}

/// Which of the three reporting buckets an
/// [`UntappedReason`] belongs to.
///
/// The three are genuinely different facts with different next steps, which is
/// why they are three lines rather than one count — see
/// [`UntappedReason::class`] for why they are derived in ONE place.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UntappedClass {
    /// A live producer that is NOT in the bag. Escalates the run's verdict.
    Gap,
    /// A rule took it out (internal namespace, remote mirror, `--exclude`).
    /// Accounted for, never a gap.
    ExcludedByRule,
    /// The caller DECLARED it and nothing was producing it. Accounted for,
    /// never a gap, and not an exclusion either.
    DeclaredNotLive,
}

/// The operator's `--exclude` patterns, compiled.
///
/// # Why the PATTERNS travel and not a name set
///
/// The obvious cheap fix — resolve `--exclude` to a set of topic NAMES at arm
/// time and hand that down — is wrong on the one path that matters. Discovery
/// finds topics OVER TIME (the rescan is the load-bearing half of this feature,
/// precisely because a dynamically-registered producer does not exist when the
/// recorder arms), so a name set computed at arm time is blind to exactly the
/// producers this feature exists to record — and therefore blind to the ones an
/// operator most wants to exclude, since `--exclude` is how a camera or a lidar
/// is kept out of a bag. The predicate has to be evaluated at every scan, so the
/// patterns are what travel.
///
/// # Semantics
///
/// UNANCHORED `regex::Regex::is_match`, identical to
/// `cerulion_cli_engine::bag_cmd::compile_patterns` — the same call, on purpose.
/// The two halves of one `--exclude` (the DECLARED set, filtered by
/// `derive_run_topics`, and the DISCOVERED set, filtered here) must agree about
/// what a pattern means, or one flag would take a topic out of one half and
/// leave it in the other. That agreement is pinned by a cross-crate oracle in
/// `bag_cmd`, not merely asserted here.
///
/// Compilation is FALLIBLE and the error is the caller's to surface: the only
/// minter today (`cerulion bag record`) has already compiled the same patterns
/// for its own selection, so a failure here is an internal inconsistency, and
/// arming with a pattern the recorder could not compile would record exactly
/// what the operator excluded.
#[derive(Debug, Clone, Default)]
pub struct ExcludePatterns {
    /// `(source pattern, compiled)` — the source is kept so a verdict can name
    /// WHICH pattern matched (`regex::Regex::as_str` would do, but keeping the
    /// caller's own bytes means the row echoes what they typed).
    compiled: Vec<(String, regex::Regex)>,
}

impl ExcludePatterns {
    /// Compile `patterns`, or report the FIRST one that is not a valid regex.
    ///
    /// The message names the flag the reaching verb accepts (`--exclude`), per
    /// the per-verb remedy rule: bagd's own CLI has none, so a bagd-flag
    /// spelling here would name something no caller can type.
    pub fn compile(patterns: &[String]) -> Result<Self, String> {
        let mut compiled = Vec::with_capacity(patterns.len());
        for p in patterns {
            let re = regex::Regex::new(p)
                .map_err(|e| format!("--exclude pattern '{p}' is not a valid regex: {e}"))?;
            compiled.push((p.clone(), re));
        }
        Ok(Self { compiled })
    }

    /// No exclusions — the shape every path but `cerulion bag record` is in.
    pub fn none() -> Self {
        Self::default()
    }

    /// Whether any pattern is held.
    pub fn is_empty(&self) -> bool {
        self.compiled.is_empty()
    }

    /// The FIRST pattern that matches `topic`, or `None`.
    ///
    /// First-match rather than all-matches: the verdict carries ONE pattern, and
    /// the first in the operator's own argument order is the one they will look
    /// for. Order is therefore the caller's, never sorted.
    pub fn matched(&self, topic: &str) -> Option<&str> {
        self.compiled
            .iter()
            .find(|(_, re)| re.is_match(topic))
            .map(|(src, _)| src.as_str())
    }
}

/// One scan's decision.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct DiscoveryDecision {
    /// Topics to open a data-only tap on, in canonical (sorted) order.
    pub attach: Vec<String>,
    /// Live topics NOT being tapped, each with its reason, in canonical order.
    pub skipped: Vec<(String, UntappedReason)>,
}

/// Decide what to do with a live-topic snapshot. PURE — no I/O, no clock.
///
/// - `live` — the enumerated topic names (any order; duplicates tolerated).
/// - `known` — topics the recorder is already tapping OR has already given a
///   TERMINAL verdict. Neither is re-considered, so a decision is never
///   re-logged and an incumbent tap is never disturbed.
/// - `mirrors` — the `/__cerulion/mirrors` provenance snapshot (`topic → origin
///   robot`). A mirrored topic is another robot's stream re-injected locally;
///   recording it would claim a local capture of remote data, so it is folded
///   out and REPORTED, exactly as `cerulion bag record --all` does.
/// - `excludes` — the operator's `--exclude` patterns.
///   A live topic matching one is folded out and REPORTED as
///   [`UntappedReason::ExcludedByRequest`]. Without this input, turning
///   discovery ON for `--run` silently re-recorded every topic `--exclude`
///   removed, because `known` is seeded from the TAP set the exclusion had
///   already emptied of them.
/// - `discovered_taps` — how many DISCOVERED taps are already open (declared
///   taps do not count against the budget).
/// - `bag_created` — whether the bag's channels have been registered. Once true,
///   no further topic can be recorded (channel immutability), so every unknown
///   live topic is reported rather than tapped.
///
/// The walk is SORTED, so which topics win a contended budget is deterministic
/// and reproducible rather than an artifact of service-directory iteration
/// order.
///
/// VERDICT PRECEDENCE is deliberate: a name that is excluded by rule, or that
/// belongs to another robot, is reported as such EVEN AFTER the bag exists.
/// Reporting `/bagd/status` or a remote mirror as "appeared after bag creation"
/// would send the reader looking for a producer to blame and a remedy that
/// cannot apply.
///
/// The `--exclude` check sits BELOW the internal-prefix and mirror folds and
/// ABOVE the bag-created and budget verdicts, and both halves of that placement
/// follow the rule above. The first two describe what the topic IS — the
/// recorder's own channel, or another robot's stream — which is true whatever
/// the operator asked and carries a remedy the operator's flag cannot change;
/// so a `/bagd/status` matched by a broad pattern still reads
/// `excluded_internal`, and a mirror still names its robot. The last two
/// describe a MECHANISM (channel immutability) and a CAPACITY (the tap ceiling),
/// and both of their remedies — widen the settle window, free up taps — are
/// meaningless for a topic nobody wanted, which is the same reason the two folds
/// already outrank them.
pub fn plan_discovery(
    live: &[String],
    known: &BTreeSet<String>,
    mirrors: &BTreeMap<String, String>,
    excludes: &ExcludePatterns,
    discovered_taps: usize,
    bag_created: bool,
) -> DiscoveryDecision {
    let mut decision = DiscoveryDecision::default();
    // Sort + dedup: the enumerator's order is an SHM-directory artifact, and a
    // contended budget must not resolve differently run to run.
    let candidates: BTreeSet<&String> = live.iter().collect();

    for topic in candidates {
        if known.contains(topic) {
            continue;
        }
        if EXCLUDED_TOPIC_PREFIXES.iter().any(|p| topic.starts_with(p)) {
            decision
                .skipped
                .push((topic.clone(), UntappedReason::ExcludedInternal));
            continue;
        }
        // The netd-mirror fold, through the SHARED predicate rather than a
        // second copy of it (`attribute_local_topic` was hoisted into
        // cerulion_core for exactly this reuse).
        if let Some(robot) =
            cerulion_core::transport::mirror_registry::attribute_local_topic(topic, mirrors, None)
        {
            decision.skipped.push((
                topic.clone(),
                UntappedReason::RemoteMirror {
                    robot: robot.to_string(),
                },
            ));
            continue;
        }
        // The operator's own fold. Evaluated HERE, on
        // every scan, so a producer that registers mid-recording is held out
        // too — a name set resolved at arm time could not see it.
        if let Some(pattern) = excludes.matched(topic) {
            decision.skipped.push((
                topic.clone(),
                UntappedReason::ExcludedByRequest {
                    pattern: pattern.to_string(),
                },
            ));
            continue;
        }
        if bag_created {
            decision
                .skipped
                .push((topic.clone(), UntappedReason::AppearedAfterBagCreation));
            continue;
        }
        if discovered_taps + decision.attach.len() >= DISCOVERY_MAX_TAPS {
            decision.skipped.push((
                topic.clone(),
                UntappedReason::BudgetExhausted {
                    budget: DISCOVERY_MAX_TAPS,
                },
            ));
            continue;
        }
        decision.attach.push(topic.clone());
    }
    decision
}

/// How many times the arm-time mirror-provenance gather is attempted
/// before the recorder declares the picture UNESTABLISHED.
///
/// # Why retrying is the right response, and why here
///
/// The registry gather is a windowed LISTEN over SHM, so it can expire having
/// heard nothing from a writer that is live and republishing — MEASURED twice on
/// macOS CI, where the recorder paid the full 600 ms window and then recorded
/// another robot's mirrored stream as this machine's data. There is no
/// request/response on that service; listening again is the only way to get more
/// evidence.
///
/// Arm time is the one moment a retry is nearly free: bagd is armed BEFORE the
/// graph is released to step 0, so no tapped topic is streaming yet and a longer
/// wait costs no frames — the same argument that put the gather here at all.
///
/// # Two residuals this budget MOVES, stated because they are not obvious
///
/// (a) The arm-time listening WINDOW is SHORTER with the presence frame. A
/// zero-mirror desk without it spends the whole ~1.8 s budget timing out (unioning
/// records across attempts); with the presence frame its writer is heard on the
/// first republish and the snapshot closes in ~200 ms MEASURED. So the window
/// in which a CONCURRENTLY-registering netd mirror happens to be caught is ~9x
/// narrower. That MOVES the boundary of the long-documented "a mirror netd
/// creates AFTER the snapshot is not folded out" residual; it does not create a
/// new class, and the same early exit has always applied to any writer holding
/// at least one record.
///
/// (b) The completeness signal is DIRECTIONAL: a recorder against a writer
/// (an already-running netd that has not been restarted) still gets
/// `Incomplete` forever when that writer's record map is empty — the old writer
/// sends no presence frame, so there is nothing to hear. The verdict is then
/// conservative rather than wrong (`mirrors_established: false`, and the recorder
/// says so), but the remedy is restarting netd, and nothing in the recorder can
/// detect that case to say so.
///
/// # What it costs, stated rather than implied
///
/// Bounded at `MIRROR_GATHER_ATTEMPTS × MIRROR_GATHER_WINDOW` (~1.8 s), and paid
/// ONLY when a gather comes back incomplete. Two shapes never pay it: a desk with
/// no re-injector at all (the registry reports zero publishers and the gather
/// returns instantly), and a healthy netd (heard on its first republish, ~150 ms).
/// The shape that DOES pay the full budget every run is a machine carrying a
/// CRASHED writer's un-swept iceoryx2 port, where the live count permanently
/// exceeds the writers heard — there the first attempt's records were already
/// correct and the retries add nothing but latency. That is accepted: the
/// recorder cannot tell that case apart from a live writer it has not yet heard,
/// and guessing wrong writes another robot's data into a bag.
pub const MIRROR_GATHER_ATTEMPTS: usize = 3;

// A budget of 1 IS the earlier behaviour — one gather, believed whatever it
// says — and every oracle below would stay green under it, because each passes
// its own `attempts`. Guarded at COMPILE time rather than by a test so the
// regression cannot be introduced at all (the `RESERVED_PREFIX` precedent
// above).
const _: () = assert!(
    MIRROR_GATHER_ATTEMPTS > 1,
    "MIRROR_GATHER_ATTEMPTS must leave room for a retry — a budget of 1 believes a \
     single windowed listen"
);

/// The arm-time mirror-snapshot decision, kept PURE so the retry policy
/// is oracle-testable without a transport, a clock, or a lost race.
///
/// Runs `gather` until it SETTLES or the attempt budget is spent, and returns the
/// snapshot together with whether it is ESTABLISHED — i.e. whether an absent
/// topic may be read as "not a mirror". That second value is the whole point:
/// without it, the recorder reads an empty snapshot as fact, so a lost race is
/// indistinguishable from a desk with no mirrors.
///
/// Records UNION across attempts (last attribution wins, matching the gather's
/// own per-topic dedup). An incomplete attempt's records are REAL — a record
/// exists only because a writer published it — so keeping them can only ever add
/// a true mirror, never invent one, and the union errs toward NOT recording
/// another robot's stream.
///
/// # What the union CANNOT do, stated at its real strength
///
/// It is one-way: a later attempt's ABSENCE never retracts an earlier one's
/// hearing, and that holds even when the later attempt SETTLED — which is the
/// case where the absence is genuine evidence (a settled gather heard every live
/// writer, so a topic missing from it is a topic nobody is mirroring any more).
/// A writer that retires a mirror between attempt 1 and attempt 2 therefore
/// leaves a stale record standing for the rest of the run, and its effect is
/// SILENT: `plan_discovery` folds that topic out as `RemoteMirror`, which is not
/// a coverage gap, so `gap_count()` stays 0 and nothing in the manifest says a
/// genuinely-local topic went unrecorded.
///
/// Accepted rather than fixed, because the window is one arm-time budget
/// (< 2 s, before the graph is released) and the failure direction is
/// conservative — it declines to record a topic, never mis-attributes one. The
/// clean fix is to let a SETTLED attempt REPLACE rather than union; it is
/// deliberately not taken here because a settled attempt's record set is a
/// snapshot of that instant and replacing on it would discard a mirror an
/// earlier attempt legitimately heard from a writer that has since gone quiet.
///
/// An `Err` attempt (SHM/config failure opening the control service) is spent
/// like any other and reported through `on_error` so the caller can log it; it
/// never settles the question.
/// # Cancellation
///
/// `cancelled` is consulted BEFORE each attempt, so a shutdown request does not
/// have to wait out the remaining budget. Without it the retry loop is the
/// longest uninterruptible stretch of a recorder's startup: on a machine
/// carrying a crashed writer's un-swept port every attempt runs the full window,
/// and a Ctrl-C landing on attempt 1 would be ignored for ~1.2 s more. A
/// cancelled resolve reports UNESTABLISHED, which is what actually happened —
/// the run stopped asking before it knew.
pub fn resolve_mirror_snapshot<E>(
    attempts: usize,
    mut gather: impl FnMut() -> Result<cerulion_core::transport::mirror_registry::MirrorGather, E>,
    mut on_error: impl FnMut(E),
    mut cancelled: impl FnMut() -> bool,
) -> (BTreeMap<String, String>, bool) {
    let mut snapshot: BTreeMap<String, String> = BTreeMap::new();
    for _ in 0..attempts {
        if cancelled() {
            return (snapshot, false);
        }
        match gather() {
            Ok(g) => {
                let settled = g.completeness.is_settled();
                for r in g.records {
                    snapshot.insert(r.topic, r.origin_robot);
                }
                if settled {
                    return (snapshot, true);
                }
            }
            Err(e) => on_error(e),
        }
    }
    (snapshot, false)
}

/// PURE: is bag creation still being held open for the live topic set to
/// SETTLE?
///
/// ONE rule, stated in the unit the guarantee is about: the channel set stays
/// open until `quiet_window` has passed with NOTHING NEW DISCOVERED, measured
/// from the later of the drive loop's start and the last discovered tap, and
/// capped by `cap` (which `--discovery-settle-ms 0` sets to zero to disable the
/// hold entirely). `quiet_window` is clamped to `cap` at the call site, so a cap
/// below the window still means what it says.
///
/// # Why a wall rather than a count of quiet enumerations
///
/// Enumeration is driven by filesystem events, so a settled machine produces no
/// enumerations at all. A rule phrased as "N consecutive scans found nothing"
/// could never be satisfied there, and every plain recording would pay the whole
/// cap instead of the window. The wall is also directly measurable by a test,
/// rather than being an emergent property of which enumerations happen to be
/// counted.
///
/// `since_last_find` is `None` when discovery has never added a tap on this
/// run, which is the common case: a graph whose live set was complete before the
/// recorder armed pays exactly the window and nothing more.
pub fn discovery_hold_open(
    discover_live: bool,
    elapsed: Duration,
    cap: Duration,
    quiet_window: Duration,
    since_last_find: Option<Duration>,
) -> bool {
    if !discover_live || elapsed >= cap {
        return false;
    }
    if elapsed < quiet_window {
        return true;
    }
    since_last_find.is_some_and(|since| since < quiet_window)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn known(items: &[&str]) -> BTreeSet<String> {
        items.iter().map(|s| s.to_string()).collect()
    }

    fn live(items: &[&str]) -> Vec<String> {
        items.iter().map(|s| s.to_string()).collect()
    }

    /// A `/__cerulion/mirrors` snapshot: `topic → origin robot`.
    fn mirrors(items: &[(&str, &str)]) -> BTreeMap<String, String> {
        items
            .iter()
            .map(|(t, r)| (t.to_string(), r.to_string()))
            .collect()
    }

    /// No mirrors present (the single-machine case).
    fn no_mirrors() -> BTreeMap<String, String> {
        BTreeMap::new()
    }

    /// No operator exclusions — the shape every path but
    /// `cerulion bag record --exclude` is in.
    fn no_excludes() -> ExcludePatterns {
        ExcludePatterns::none()
    }

    /// The operator's `--exclude` patterns, compiled.
    fn excludes(patterns: &[&str]) -> ExcludePatterns {
        ExcludePatterns::compile(&patterns.iter().map(|s| s.to_string()).collect::<Vec<_>>())
            .expect("the fixture patterns must compile")
    }

    #[test]
    fn an_unknown_live_topic_is_attached() {
        let d = plan_discovery(
            &live(["/a", "/b"].as_ref()),
            &known(&[]),
            &no_mirrors(),
            &no_excludes(),
            0,
            false,
        );
        assert_eq!(d.attach, vec!["/a".to_string(), "/b".to_string()]);
        assert!(d.skipped.is_empty());
    }

    #[test]
    fn an_already_known_topic_is_neither_attached_nor_reported() {
        // The declared taps + terminal verdicts: re-deciding them would
        // double-tap a topic and re-log a verdict on every scan.
        let d = plan_discovery(
            &live(&["/a", "/b"]),
            &known(&["/a"]),
            &no_mirrors(),
            &no_excludes(),
            1,
            false,
        );
        assert_eq!(d.attach, vec!["/b".to_string()]);
        assert!(d.skipped.is_empty(), "a known topic yields NO verdict");
    }

    #[test]
    fn the_internal_namespaces_are_excluded_with_a_reason() {
        let d = plan_discovery(
            &live(&[
                "/bagd/status",
                "/__cerulion/mirrors",
                "__cerulion/scheduler_trace",
                "/real",
            ]),
            &known(&[]),
            &no_mirrors(),
            &no_excludes(),
            0,
            false,
        );
        assert_eq!(d.attach, vec!["/real".to_string()]);
        assert_eq!(
            d.skipped,
            vec![
                (
                    "/__cerulion/mirrors".to_string(),
                    UntappedReason::ExcludedInternal
                ),
                ("/bagd/status".to_string(), UntappedReason::ExcludedInternal),
                (
                    "__cerulion/scheduler_trace".to_string(),
                    UntappedReason::ExcludedInternal
                ),
            ]
        );
    }

    #[test]
    fn a_topic_appearing_after_bag_creation_is_reported_not_tapped() {
        // The headline residual: MCAP channels are immutable once written, so
        // this topic CANNOT be recorded — but it must never be silent.
        let d = plan_discovery(
            &live(&["/late"]),
            &known(&[]),
            &no_mirrors(),
            &no_excludes(),
            0,
            true,
        );
        assert!(d.attach.is_empty());
        assert_eq!(
            d.skipped,
            vec![(
                "/late".to_string(),
                UntappedReason::AppearedAfterBagCreation
            )]
        );
    }

    #[test]
    fn an_excluded_name_outranks_the_after_creation_verdict() {
        // Ordering matters for the operator: `/bagd/status` appearing late is
        // not a coverage gap, it is bagd's own channel. Reporting it as
        // "appeared after bag creation" would send the reader looking for a
        // producer to blame.
        let d = plan_discovery(
            &live(&["/bagd/status"]),
            &known(&[]),
            &no_mirrors(),
            &no_excludes(),
            0,
            true,
        );
        assert_eq!(
            d.skipped,
            vec![("/bagd/status".to_string(), UntappedReason::ExcludedInternal)]
        );
    }

    #[test]
    fn the_budget_is_a_ceiling_on_discovered_taps_and_is_reported() {
        // One slot left: the canonically-FIRST candidate wins it, the rest are
        // reported. Hand oracle over an unsorted input, so a decision that
        // followed enumeration order would fail.
        let d = plan_discovery(
            &live(&["/z", "/m", "/a"]),
            &known(&[]),
            &no_mirrors(),
            &no_excludes(),
            DISCOVERY_MAX_TAPS - 1,
            false,
        );
        assert_eq!(d.attach, vec!["/a".to_string()]);
        assert_eq!(
            d.skipped,
            vec![
                (
                    "/m".to_string(),
                    UntappedReason::BudgetExhausted {
                        budget: DISCOVERY_MAX_TAPS
                    }
                ),
                (
                    "/z".to_string(),
                    UntappedReason::BudgetExhausted {
                        budget: DISCOVERY_MAX_TAPS
                    }
                ),
            ]
        );
    }

    #[test]
    fn a_full_budget_attaches_nothing() {
        let d = plan_discovery(
            &live(&["/a"]),
            &known(&[]),
            &no_mirrors(),
            &no_excludes(),
            DISCOVERY_MAX_TAPS,
            false,
        );
        assert!(d.attach.is_empty());
        assert_eq!(d.skipped.len(), 1);
    }

    #[test]
    fn the_walk_is_sorted_and_deduplicated() {
        // Duplicates cannot double-count against the budget, and the attach
        // order is canonical whatever the enumerator handed us.
        let d = plan_discovery(
            &live(&["/c", "/a", "/c", "/b"]),
            &known(&[]),
            &no_mirrors(),
            &no_excludes(),
            0,
            false,
        );
        assert_eq!(
            d.attach,
            vec!["/a".to_string(), "/b".to_string(), "/c".to_string()]
        );
    }

    #[test]
    fn an_empty_live_set_decides_nothing() {
        let d = plan_discovery(
            &[],
            &known(&["/a"]),
            &no_mirrors(),
            &no_excludes(),
            1,
            false,
        );
        assert_eq!(d, DiscoveryDecision::default());
    }

    #[test]
    fn a_netd_mirror_of_a_remote_robot_is_folded_out_and_attributed() {
        // A mirror is a real local `{topic}/data` service, so the enumerator
        // yields it exactly like a local producer. Recording it would claim a
        // local capture of ANOTHER robot's stream, against the "one data source
        // = one topic" decision `bag record --all` already honours.
        let d = plan_discovery(
            &live(&["/local", "/remote/lowstate"]),
            &known(&[]),
            &mirrors(&[("/remote/lowstate", "go2")]),
            &no_excludes(),
            0,
            false,
        );
        assert_eq!(d.attach, vec!["/local".to_string()]);
        assert_eq!(
            d.skipped,
            vec![(
                "/remote/lowstate".to_string(),
                UntappedReason::RemoteMirror {
                    robot: "go2".to_string()
                }
            )]
        );
    }

    #[test]
    fn a_mirror_outranks_the_after_creation_verdict() {
        // Same argument as the excluded-name precedence: reporting another
        // robot's mirror as "appeared after bag creation" points the reader at
        // a local producer to blame and a remedy that cannot apply.
        let d = plan_discovery(
            &live(&["/remote/x"]),
            &known(&[]),
            &mirrors(&[("/remote/x", "go2")]),
            &no_excludes(),
            0,
            true,
        );
        assert_eq!(
            d.skipped,
            vec![(
                "/remote/x".to_string(),
                UntappedReason::RemoteMirror {
                    robot: "go2".to_string()
                }
            )]
        );
    }

    #[test]
    fn an_exclusion_by_rule_is_not_a_coverage_gap_but_every_other_reason_is() {
        // `gap_count()` is the number an operator reads to decide whether the
        // recording missed something actionable. A mirror and the recorder's
        // own status channel are deliberately not this recording's data.
        assert!(!UntappedReason::ExcludedInternal.is_coverage_gap());
        assert!(!UntappedReason::RemoteMirror {
            robot: "go2".into()
        }
        .is_coverage_gap());
        assert!(UntappedReason::AppearedAfterBagCreation.is_coverage_gap());
        assert!(UntappedReason::BudgetExhausted { budget: 1 }.is_coverage_gap());
        assert!(UntappedReason::AttachFailed { error: "e".into() }.is_coverage_gap());
        // The operator asked for it to be out, which is
        // the same kind of fact as the two above — not a gap.
        assert!(!UntappedReason::ExcludedByRequest {
            pattern: "^/cam".into()
        }
        .is_coverage_gap());
    }

    /// The not-a-gap set splits in TWO, and the
    /// split is what keeps both surfaces' labels accurate.
    ///
    /// `excluded_by_rule` is what the recorder's terminal line counts and what
    /// `bag info` files under "by rule"; `declared_not_live` is neither — the
    /// topic was ASKED FOR and had no producer, so no rule excluded it. Folding
    /// them together made the label a lie for the second, and a reader chasing
    /// "which rule took it out?" was chasing a rule that does not exist.
    ///
    /// EXHAUSTIVE over the six variants on purpose: a seventh reason must decide
    /// which side it falls on rather than inheriting one silently.
    #[test]
    fn the_not_a_gap_set_splits_into_excluded_by_rule_and_never_live() {
        // Excluded by a rule — something took the topic out.
        assert!(UntappedReason::ExcludedInternal.is_excluded_by_rule());
        assert!(UntappedReason::RemoteMirror {
            robot: "go2".into()
        }
        .is_excluded_by_rule());
        assert!(UntappedReason::ExcludedByRequest {
            pattern: "^/cam".into()
        }
        .is_excluded_by_rule());

        // NOT an exclusion: nothing took this topic out, it had no producer.
        assert!(!UntappedReason::DeclaredNotLive.is_excluded_by_rule());

        // And a genuine GAP is not "by rule" either — the two predicates
        // partition the ledger three ways, which is what the two surfaces
        // render.
        assert!(!UntappedReason::AppearedAfterBagCreation.is_excluded_by_rule());
        assert!(!UntappedReason::BudgetExhausted { budget: 1 }.is_excluded_by_rule());
        assert!(!UntappedReason::AttachFailed { error: "e".into() }.is_excluded_by_rule());
    }

    // ---------------------------------------------------------------------
    // `--exclude` reaches DISCOVERY.
    //
    // `--exclude` was made meaningful on the DECLARED half
    // and discovery was turned ON for `--run`. Composed, the second
    // silently undid the first: `plan_discovery`'s only name filter is `known`,
    // which is seeded from the TAP set the exclusion had already removed the
    // topic from, so the rescan re-found it as an undeclared live producer and
    // gave it a channel. Measured on the head before this fix: stdout announced ONE
    // topic and the bag held the excluded one with 291 frames, `untapped = {}`.
    // ---------------------------------------------------------------------

    #[test]
    fn an_excluded_live_topic_is_folded_out_and_names_the_pattern_that_matched() {
        let d = plan_discovery(
            &live(&["/camera/front", "/camera/rear", "/odom"]),
            &known(&[]),
            &no_mirrors(),
            &excludes(&["^/camera"]),
            0,
            false,
        );
        // The un-excluded topic is still recorded — the exclusion is a filter,
        // not a kill switch.
        assert_eq!(d.attach, vec!["/odom".to_string()]);
        // Both matches are ACCOUNTED FOR, each naming the pattern. Silence here
        // is the defect: `enumerated: true` claims `untapped` names
        // every live producer this recorder did not record.
        assert_eq!(
            d.skipped,
            vec![
                (
                    "/camera/front".to_string(),
                    UntappedReason::ExcludedByRequest {
                        pattern: "^/camera".to_string()
                    }
                ),
                (
                    "/camera/rear".to_string(),
                    UntappedReason::ExcludedByRequest {
                        pattern: "^/camera".to_string()
                    }
                ),
            ]
        );
    }

    /// The case a name set resolved at ARM TIME cannot serve, and therefore the
    /// reason `BagdConfig::discovery_excludes` carries PATTERNS.
    ///
    /// Discovery finds producers over TIME — the rescan is the load-bearing half
    /// of this feature precisely because a dynamically-registered producer does not
    /// exist when the recorder arms — so a topic can first appear long after any
    /// arm-time set was computed. `bag_created: true` is that world: the topic
    /// is new to this scan, and the verdict must still be the operator's
    /// exclusion rather than the channel-immutability boundary.
    #[test]
    fn a_topic_first_seen_mid_recording_is_still_excluded_by_the_operators_pattern() {
        let d = plan_discovery(
            &live(&["/lidar/points"]),
            &known(&[]),
            &no_mirrors(),
            &excludes(&["lidar"]),
            0,
            /* bag_created */ true,
        );
        assert!(d.attach.is_empty());
        assert_eq!(
            d.skipped,
            vec![(
                "/lidar/points".to_string(),
                UntappedReason::ExcludedByRequest {
                    pattern: "lidar".to_string()
                }
            )],
            "an excluded topic must read `excluded_by_request`, never \
             `appeared_after_bag_creation` — the settle-window remedy that \
             verdict prints cannot help a topic nobody wanted"
        );
    }

    /// PRECEDENCE, both directions, in one body.
    ///
    /// BELOW the two folds that describe what a topic IS (the recorder's own
    /// channel, another robot's stream) — those are true whatever the operator
    /// asked, and carry remedies `--exclude` cannot change. ABOVE the two that
    /// describe a MECHANISM and a CAPACITY, whose remedies (widen the settle
    /// window, free up taps) are meaningless for a topic nobody wanted.
    #[test]
    fn the_operators_exclusion_sits_below_the_two_folds_and_above_the_two_limits() {
        // Below: an internal name and a mirror keep their own verdicts even
        // when a broad pattern also matches them.
        let d = plan_discovery(
            &live(&["/bagd/status", "/go2/lidar", "/odom"]),
            &known(&[]),
            &mirrors(&[("/go2/lidar", "go2")]),
            &excludes(&[".*"]),
            0,
            false,
        );
        assert!(d.attach.is_empty());
        assert_eq!(
            d.skipped,
            vec![
                ("/bagd/status".to_string(), UntappedReason::ExcludedInternal),
                (
                    "/go2/lidar".to_string(),
                    UntappedReason::RemoteMirror {
                        robot: "go2".to_string()
                    }
                ),
                (
                    "/odom".to_string(),
                    UntappedReason::ExcludedByRequest {
                        pattern: ".*".to_string()
                    }
                ),
            ]
        );

        // Above: with the discovered-tap budget ALREADY FULL, an excluded topic
        // still reads as excluded rather than `budget_exhausted`.
        let d = plan_discovery(
            &live(&["/camera/front"]),
            &known(&[]),
            &no_mirrors(),
            &excludes(&["^/camera"]),
            DISCOVERY_MAX_TAPS,
            false,
        );
        assert_eq!(
            d.skipped,
            vec![(
                "/camera/front".to_string(),
                UntappedReason::ExcludedByRequest {
                    pattern: "^/camera".to_string()
                }
            )]
        );
    }

    /// The ANTI-TAUTOLOGY control for every arm above: with NO patterns the
    /// same live set is recorded in full.
    ///
    /// Without it, "the excluded topic is not attached" is satisfied by a
    /// `plan_discovery` that attaches nothing at all.
    #[test]
    fn with_no_patterns_the_same_topics_are_all_attached() {
        let d = plan_discovery(
            &live(&["/camera/front", "/camera/rear", "/odom"]),
            &known(&[]),
            &no_mirrors(),
            &no_excludes(),
            0,
            false,
        );
        assert_eq!(
            d.attach,
            vec![
                "/camera/front".to_string(),
                "/camera/rear".to_string(),
                "/odom".to_string()
            ]
        );
        assert!(d.skipped.is_empty());
    }

    #[test]
    fn exclude_patterns_report_the_first_match_in_the_operators_own_order() {
        let p = excludes(&["camera", "^/camera/front$"]);
        // BOTH patterns match, and the row must name the one the operator
        // wrote FIRST — that is the one they will look for.
        assert_eq!(p.matched("/camera/front"), Some("camera"));
        assert_eq!(p.matched("/camera/rear"), Some("camera"));
        assert_eq!(p.matched("/odom"), None);
        assert!(!p.is_empty());

        let none = ExcludePatterns::none();
        assert!(none.is_empty());
        assert_eq!(none.matched("/anything"), None);
    }

    #[test]
    fn an_uncompilable_pattern_is_reported_by_name_with_the_flag_that_carries_it() {
        let err = ExcludePatterns::compile(&["ok".to_string(), "[".to_string()])
            .expect_err("`[` is not a valid regex");
        assert!(
            err.contains("--exclude") && err.contains('['),
            "the message must name the FLAG the reaching verb accepts and the \
             offending pattern: {err}"
        );
    }

    #[test]
    fn terminal_verdicts_are_exactly_the_ones_that_cannot_heal() {
        // A retryable verdict is re-decided next scan; a terminal one is
        // remembered. Getting this backwards either floods the log (re-logging
        // an excluded name every 250 ms) or freezes a transient refusal into a
        // permanent one.
        assert!(UntappedReason::ExcludedInternal.is_terminal());
        assert!(UntappedReason::AppearedAfterBagCreation.is_terminal());
        assert!(UntappedReason::RemoteMirror {
            robot: "go2".into()
        }
        .is_terminal());
        // The budget is measured against SUCCESSFULLY-ATTACHED discovered taps,
        // which are never removed — so the count is monotonically
        // non-decreasing and a refusal provably cannot clear. Marking it
        // retryable is what let a budget verdict be re-planned every scan and
        // then RE-LABELLED `appeared_after_bag_creation`.
        assert!(UntappedReason::BudgetExhausted { budget: 1 }.is_terminal());
        // The pattern set is fixed for the run, so this
        // verdict cannot change either — and a re-decided exclusion re-logs
        // every 250 ms, the same flood `ExcludedInternal` is terminal to avoid.
        assert!(UntappedReason::ExcludedByRequest {
            pattern: "^/cam".into()
        }
        .is_terminal());
        // Genuinely transient — a freed subscriber slot heals it.
        assert!(!UntappedReason::AttachFailed {
            error: "slots".into()
        }
        .is_terminal());
    }

    #[test]
    fn reason_tags_are_stable_and_distinct() {
        let all = [
            UntappedReason::ExcludedInternal,
            UntappedReason::BudgetExhausted { budget: 1 },
            UntappedReason::AppearedAfterBagCreation,
            UntappedReason::AttachFailed { error: "e".into() },
            UntappedReason::RemoteMirror {
                robot: "go2".into(),
            },
            UntappedReason::DeclaredNotLive,
            UntappedReason::ExcludedByRequest {
                pattern: "^/cam".into(),
            },
        ];
        let tags: Vec<&str> = all.iter().map(UntappedReason::tag).collect();
        assert_eq!(
            tags,
            vec![
                "excluded_internal",
                "budget_exhausted",
                "appeared_after_bag_creation",
                "attach_failed",
                "remote_mirror",
                "declared_not_live",
                "excluded_by_request"
            ]
        );
        let unique: BTreeSet<&&str> = tags.iter().collect();
        assert_eq!(unique.len(), tags.len(), "tags must discriminate");
    }

    #[test]
    fn reasons_round_trip_through_json_with_their_tag() {
        // The manifest is the machine-readable half; a reader must be able to
        // discriminate on `reason` and read the payload fields.
        let r = UntappedReason::BudgetExhausted { budget: 256 };
        let s = serde_json::to_string(&r).expect("serialize");
        assert_eq!(s, r#"{"reason":"budget_exhausted","budget":256}"#);
        assert_eq!(
            serde_json::from_str::<UntappedReason>(&s).expect("deserialize"),
            r
        );

        let a = UntappedReason::AttachFailed {
            error: "no slots".into(),
        };
        assert_eq!(
            serde_json::to_string(&a).expect("serialize"),
            r#"{"reason":"attach_failed","error":"no slots"}"#
        );

        // The pattern is a PAYLOAD FIELD, so a manifest
        // reader gets the same fact `bag info` renders rather than a bare tag.
        let e = UntappedReason::ExcludedByRequest {
            pattern: "^/camera".into(),
        };
        let s = serde_json::to_string(&e).expect("serialize");
        assert_eq!(
            s,
            r#"{"reason":"excluded_by_request","pattern":"^/camera"}"#
        );
        assert_eq!(
            serde_json::from_str::<UntappedReason>(&s).expect("deserialize"),
            e
        );
    }

    // ---------------------------------------------------------------------
    // The arm-time mirror-snapshot retry policy.
    //
    // Driven by SCRIPTED gather outcomes, which is the only way to reach these
    // states deterministically — the production trigger is a lost SHM race that
    // fired twice on macOS CI and reproduces on no desk on demand. The oracle
    // in every arm is the pair (snapshot, established) PLUS the exact number of
    // gathers run: the call count is what a pure test can see that the return
    // value cannot, and it is what separates "settled on the first answer" from
    // "spent the whole budget and happened to agree".
    // ---------------------------------------------------------------------

    use cerulion_core::transport::mirror_registry::{
        GatherCompleteness, MirrorGather, MirrorRecord,
    };

    /// A scripted gather outcome carrying records and a verdict.
    fn gather(records: &[(&str, &str)], completeness: GatherCompleteness) -> MirrorGather {
        MirrorGather {
            records: records
                .iter()
                .map(|(t, r)| MirrorRecord {
                    topic: (*t).to_string(),
                    origin_robot: (*r).to_string(),
                })
                .collect(),
            completeness,
        }
    }

    /// The INCOMPLETE verdict, with counts that do not matter to this policy
    /// (it keys on `is_settled()`, never on how far short the gather fell).
    const UNHEARD: GatherCompleteness = GatherCompleteness::Incomplete {
        live_writers: 1,
        writers_heard: 0,
    };

    /// Drive `resolve_mirror_snapshot` over a script, returning the outcome plus
    /// how many gathers it actually ran.
    fn drive(
        attempts: usize,
        script: Vec<Result<MirrorGather, String>>,
    ) -> (BTreeMap<String, String>, bool, usize, Vec<String>) {
        let mut calls = 0usize;
        let mut errors: Vec<String> = Vec::new();
        let mut it = script.into_iter();
        let (snapshot, established) = resolve_mirror_snapshot(
            attempts,
            || {
                calls += 1;
                it.next()
                    .expect("the script must cover every attempt the policy makes")
            },
            |e| errors.push(e),
            || false,
        );
        (snapshot, established, calls, errors)
    }

    /// A settled FIRST answer is taken as-is: no retry, and the picture is
    /// established. The majority shipping path (a desk with no re-injector
    /// returns instantly), so a policy that retried unconditionally would add
    /// ~1.2 s of dead wait to every recording that starts.
    #[test]
    fn a_settled_first_gather_is_believed_and_never_retried() {
        let (snapshot, established, calls, errors) =
            drive(3, vec![Ok(gather(&[], GatherCompleteness::Settled))]);

        assert!(snapshot.is_empty());
        assert!(
            established,
            "a settled empty answer IS evidence: this desk has no mirrors"
        );
        assert_eq!(calls, 1, "a settled answer must not buy a second gather");
        assert!(errors.is_empty());
    }

    /// THE fix, as a decision: an incomplete gather is retried, and a later
    /// settled answer establishes the picture. Without the retry, the first empty
    /// answer is the answer, which is how another robot's stream gets recorded.
    #[test]
    fn an_incomplete_gather_is_retried_until_one_settles() {
        let (snapshot, established, calls, errors) = drive(
            3,
            vec![
                Ok(gather(&[], UNHEARD)),
                Ok(gather(
                    &[("/utlidar/cloud", "go2")],
                    GatherCompleteness::Settled,
                )),
            ],
        );

        assert_eq!(
            snapshot,
            BTreeMap::from([("/utlidar/cloud".to_string(), "go2".to_string())]),
            "the retry found the mirror the first gather missed"
        );
        assert!(established);
        assert_eq!(
            calls, 2,
            "it stops at the first settled answer, not at the budget"
        );
        assert!(errors.is_empty());
    }

    /// The budget is REAL and bounded, and an exhausted one is reported as
    /// UNESTABLISHED rather than as an absence. Without the second half the
    /// retry would merely make the silent lie less frequent.
    #[test]
    fn a_budget_of_incomplete_gathers_is_spent_then_reported_unestablished() {
        let (snapshot, established, calls, errors) = drive(
            3,
            vec![
                Ok(gather(&[], UNHEARD)),
                Ok(gather(&[], UNHEARD)),
                Ok(gather(&[], UNHEARD)),
            ],
        );

        assert!(snapshot.is_empty());
        assert!(
            !established,
            "nothing was ever heard from a live writer — this empty is an UNKNOWN"
        );
        assert_eq!(calls, 3, "exactly the budget: never fewer, never unbounded");
        assert!(errors.is_empty());
    }

    /// Records from an INCOMPLETE attempt are real and are kept, unioned across
    /// attempts. The stale-port shape: attempt 1 hears a genuine mirror but
    /// cannot speak for a second publisher, attempt 2 hears a different one.
    /// Losing either would record that robot's stream as local.
    #[test]
    fn records_union_across_attempts_and_the_last_attribution_wins() {
        let (snapshot, established, calls, _) = drive(
            3,
            vec![
                Ok(gather(&[("/a", "go2"), ("/b", "spot")], UNHEARD)),
                Ok(gather(&[("/c", "go2"), ("/b", "arm")], UNHEARD)),
                Ok(gather(&[], UNHEARD)),
            ],
        );

        assert_eq!(
            snapshot,
            BTreeMap::from([
                ("/a".to_string(), "go2".to_string()),
                // Last attribution wins, matching the gather's own per-topic
                // dedup — one rule, not two.
                ("/b".to_string(), "arm".to_string()),
                ("/c".to_string(), "go2".to_string()),
            ]),
            "every record heard is kept: a partial answer still names real mirrors"
        );
        assert!(
            !established,
            "holding real records is not the same as having settled the question"
        );
        assert_eq!(calls, 3);
    }

    /// An `Err` attempt (an SHM/config failure opening the control service) is
    /// SPENT like any other, REPORTED so the caller can log it, and never
    /// settles the question — while a later Ok still can.
    #[test]
    fn an_erroring_attempt_is_reported_spends_its_turn_and_settles_nothing() {
        let (snapshot, established, calls, errors) = drive(
            3,
            vec![
                Err("shm open failed".to_string()),
                Ok(gather(&[("/x", "go2")], GatherCompleteness::Settled)),
            ],
        );

        assert_eq!(
            snapshot,
            BTreeMap::from([("/x".to_string(), "go2".to_string())])
        );
        assert!(established, "the surviving attempt settled it");
        assert_eq!(calls, 2);
        assert_eq!(
            errors,
            vec!["shm open failed".to_string()],
            "the failure must reach the caller — degrading silently is the defect, not the fix"
        );
    }

    /// All-error: the caller hears about every one, and the verdict is
    /// UNESTABLISHED — never a confident empty.
    #[test]
    fn an_all_error_budget_reports_every_failure_and_establishes_nothing() {
        let (snapshot, established, calls, errors) =
            drive(2, vec![Err("e1".to_string()), Err("e2".to_string())]);

        assert!(snapshot.is_empty());
        assert!(!established);
        assert_eq!(calls, 2);
        assert_eq!(errors, vec!["e1".to_string(), "e2".to_string()]);
    }

    /// A shutdown STOPS the retry budget, and the result says
    /// UNESTABLISHED — because that is what happened: the run stopped asking
    /// before it knew.
    ///
    /// The budget is the longest uninterruptible stretch of a recorder's
    /// startup. Asserting the CALL COUNT is what makes this real: the verdict
    /// alone is `false` under a plain exhausted budget too, so a loop that
    /// ignored the flag entirely would pass on the verdict.
    #[test]
    fn a_cancelled_resolve_stops_asking_and_claims_nothing() {
        let mut calls = 0usize;
        let (snapshot, established) = resolve_mirror_snapshot(
            3,
            || -> Result<MirrorGather, String> {
                calls += 1;
                Ok(gather(&[("/a", "go2")], UNHEARD))
            },
            |_e: String| {},
            // Cancelled from the outset — the shape a Ctrl-C during startup
            // produces on the second attempt onward.
            || true,
        );

        assert!(snapshot.is_empty());
        assert!(
            !established,
            "a cancelled resolve knows nothing — reporting it settled would claim a picture \
             nobody finished gathering"
        );
        assert_eq!(
            calls, 0,
            "the flag is checked BEFORE each attempt, so a cancelled run pays no window"
        );

        // And the anti-tautology half: the same script with the flag CLEAR runs
        // the whole budget, so the assertion above cannot pass on a policy that
        // simply never gathers.
        let mut calls2 = 0usize;
        let (_, established2) = resolve_mirror_snapshot(
            3,
            || -> Result<MirrorGather, String> {
                calls2 += 1;
                Ok(gather(&[("/a", "go2")], UNHEARD))
            },
            |_e: String| {},
            || false,
        );
        assert_eq!(calls2, 3);
        assert!(!established2);
    }

    /// A ZERO budget runs NO gather and establishes nothing. The degenerate
    /// guard: a policy that treated "no attempts" as success would ship the
    /// whole feature inert while every other arm here stayed green.
    #[test]
    fn a_zero_attempt_budget_gathers_nothing_and_claims_nothing() {
        let (snapshot, established, calls, errors) = drive(0, vec![]);

        assert!(snapshot.is_empty());
        assert!(
            !established,
            "having asked nothing, the recorder knows nothing"
        );
        assert_eq!(calls, 0);
        assert!(errors.is_empty());
    }

    // =======================================================================
    // The settle hold: one rule, in wall time.
    // =======================================================================

    const CAP: Duration = Duration::from_millis(2000);
    const WINDOW: Duration = Duration::from_millis(500);

    /// The whole rule against a HAND-WRITTEN verdict vector.
    ///
    /// Written as a table rather than separate assertions because the property
    /// is the COMBINATION: a rule that only honoured the window, and one that
    /// only honoured the last find, each satisfy several rows on their own.
    #[test]
    fn the_settle_hold_is_the_window_since_the_later_of_start_and_the_last_find() {
        // (discovery on, elapsed ms, since-last-find ms, expected hold)
        let oracle = [
            // Discovery off: nothing settles, ever.
            (false, 0u64, None, false),
            (false, 100, Some(0u64), false),
            // No find yet: the window alone holds, and releases at it.
            (true, 0, None, true),
            (true, 499, None, true),
            (true, 500, None, false),
            (true, 1999, None, false),
            // A find RESTARTS the window from the find, not from the start.
            (true, 600, Some(100), true),
            (true, 900, Some(400), true),
            (true, 1000, Some(499), true),
            (true, 1000, Some(500), false),
            (true, 1500, Some(1400), true),
            // The cap outranks a stream of finds: a machine that never stops
            // producing topics must not hold the channel set open forever.
            (true, 2000, Some(0), false),
            (true, 2500, Some(0), false),
            // An OLD find on a settled machine is not a reason to hold.
            (true, 1800, Some(1700), false),
        ];
        for (i, (on, elapsed, since, want)) in oracle.into_iter().enumerate() {
            let got = discovery_hold_open(
                on,
                Duration::from_millis(elapsed),
                CAP,
                WINDOW,
                since.map(Duration::from_millis),
            );
            assert_eq!(
                got, want,
                "row {i}: discovery={on} elapsed={elapsed}ms since_find={since:?} must hold={want}"
            );
        }
    }

    /// A zero cap disables the hold entirely, whatever else is true.
    ///
    /// `--discovery-settle-ms 0` is documented as restoring the pre-hold timing
    /// exactly, and an operator who turns a knob to zero must get zero.
    #[test]
    fn a_zero_cap_never_holds() {
        for since in [None, Some(Duration::ZERO), Some(Duration::from_millis(10))] {
            assert!(
                !discovery_hold_open(true, Duration::ZERO, Duration::ZERO, Duration::ZERO, since),
                "a zero cap must release immediately (since_find {since:?})"
            );
        }
    }

    /// A cap BELOW the window is honoured as the cap, not silently widened.
    ///
    /// The clamp lives at the call site; this pins that the rule respects it
    /// rather than treating the window as a floor that outranks the operator.
    #[test]
    fn a_cap_below_the_window_releases_at_the_cap() {
        let cap = Duration::from_millis(200);
        // The call site clamps the window to the cap, which is what is passed.
        assert!(discovery_hold_open(
            true,
            Duration::from_millis(199),
            cap,
            cap,
            None
        ));
        assert!(!discovery_hold_open(
            true,
            Duration::from_millis(200),
            cap,
            cap,
            None
        ));
    }
}

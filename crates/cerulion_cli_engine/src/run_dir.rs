// SPDX-License-Identifier: AGPL-3.0-only
//! The RUN DIRECTORY — what a live `graph run` is, on disk.
//!
//! # Why every run, not just `--record`
//!
//! Today the artifacts that describe a run — the effective graph, the env
//! snapshot, the recorder/host descriptor — are prepared ONLY under `--record`,
//! as bagd inputs (`graph_cmd::prepare_recording_inputs`). So a run you did not
//! decide to record in advance is, from the outside, undescribed: `cerulion bag
//! record` attaching to it mid-flight has no graph, no env, and no identity to
//! bind its lifetime to, and the bag it produces cannot carry the same content
//! a `graph run --record` bag does.
//!
//! Writing the directory on EVERY run is what makes those two verbs able to
//! produce the same thing, because it makes them read the same thing. The
//! registry record ([`cerulion_core::transport::run_registry`]) is the pointer
//! to it.
//!
//! # Platform scope
//!
//! UNIX ONLY, and a non-Unix `graph run` says so once, loudly (`graph_cmd`'s
//! non-Unix arm), rather than skipping silently — an undescribed run that
//! announces nothing is the condition this module exists to close, so it must
//! never be indistinguishable from a described one.
//!
//! The gate is structural rather than incidental: the owner-only permissions
//! (the run dir at `0700`, its artifacts at `0600`) are a `std::os::unix`
//! implementation,
//! and the three artifact renderers this module is fed are themselves
//! Unix-gated. It also matches the repo's platform posture — running
//! multi-process is Unix-only (non-Unix takes the monolith fallback, with its
//! own loud notice) and CI runs Linux + macOS.
//!
//! # Lifecycle
//!
//! **The directory is valid only while the run is LIVE.** That is a contract on
//! READERS, and it is stated here because there is no lock and deliberately no
//! synchronisation: a reader that resolved `run_dir` from the registry and is
//! part-way through reading it can be overtaken by the run ending, and see
//! ENOENT. Nothing can prevent that — a run can end at any instant, a SIGKILLed
//! one leaves the directory with no announcement at all, and `Ending` is a
//! best-effort last word (MEASURED 0/8 at a 250 ms poll), not a barrier. So the
//! rule is: **a consumer must treat a mid-read disappearance as "the run
//! ended", never as corruption**, and must not cache the path across the run's
//! lifetime. The attaching recorder is the first consumer and the one that has
//! to enforce it; this module itself has none, so the race is unreachable here.
//!
//! Created at run start, REMOVED when the returned [`RunDescriptor`] drops —
//! the RAII shape `graph_cmd::ScratchGuard` already uses for bagd's inputs, so
//! every exit path (early `?` returns included) cleans up. A directory left
//! behind by a SIGKILLed run is inert: its run has no registry writer, so a
//! gather never points anyone at it.
//!
//! # Failure policy: a run is never failed by its own description
//!
//! `graph_cmd` calls [`begin_run_descriptor`], which maps every failure to ONE
//! loud `warn!` naming what is lost and returns `None`. A read-only home, a
//! full disk, or an SHM refusal must not stop a robot from running its graph —
//! the cost is that this run is undiscoverable, which the warn says.
//!
//! # What it does NOT carry yet, and why
//!
//! `topics.json` (the exact-mode tap set) and `schemas.json` (the schema
//! catalog) are deliberately NOT written here, and their absence is a scope
//! decision rather than an oversight:
//!
//! * `topics.json` is built by `resolve_recorded_topics`, which needs the
//!   LOADED `NodeEntry` infos for their authoritative wire schema hashes. Those
//!   exist only AFTER the node factories are loaded — BELOW this seam on the
//!   monolith path, and inside `graph_run_supervisor` on the multi-process one.
//!   Writing it here would mean either moving the seam below the build (losing
//!   the coverage of the supervisor's early return that its placement exists
//!   for) or adding a second write site per path, which is exactly the drift
//!   hazard `RecordingInputSpec` was bundled to remove.
//!   (The supervisor DOES load node cdylibs — it builds the full graph once for
//!   planning. The obstacle is ORDERING, not absence.)
//! * `schemas.json` re-parses the whole built-in message corpus. A run that
//!   nobody records should not pay that on the chance that somebody might.
//!
//! Both belong with the ATTACHING recorder, which has the run's
//! effective graph from this directory and can resolve them itself.

use std::io::Write;
use std::path::{Path, PathBuf};

use cerulion_core::transport::run_registry::{RunHandle, RunRecord, RunState};

/// Minting a run's IDENTITY and PERSISTING its description are two
/// acts, and only the second can fail.
///
/// Re-exported because the caller must be able to do the first on its own. The
/// Flashback capture plane is NAMED from the run id, and this module deliberately
/// makes descriptor creation never-fatal — so a plane gated on the PERSISTED
/// descriptor silently disappears on exactly the hosts (read-only home, full
/// disk) where losing the black box is least affordable. See
/// [`RunDescriptorSpec::run_id`].
pub use cerulion_core::transport::run_registry::mint_run_id;

use crate::error::{CliError, CliResult};

/// The `run.json` format version. Bumped only on an INCOMPATIBLE change; new
/// fields are additive and readers must ignore what they do not know.
pub const RUN_MANIFEST_VERSION: u32 = 1;

/// The manifest file name inside the run directory.
pub const RUN_MANIFEST_FILE: &str = "run.json";

/// The effective graph config the run executes (see
/// `graph_cmd::render_effective_graph_yaml` for why this is NOT
/// `graphs/<file>.yaml`).
pub const RUN_GRAPH_FILE: &str = "graph.yaml";

/// The env snapshot, byte-identical to the bag's `env.json` attachment.
pub const RUN_ENV_FILE: &str = "env.json";

/// The recording host's identity, byte-identical to the bag's
/// `__cerulion/recorder.json` attachment.
pub const RUN_RECORDER_FILE: &str = "recorder.json";

/// How this run's `process_groups:` came to be — the partition
/// PROVENANCE, which the file alone cannot tell you.
///
/// The auto-partitioner's no-TTY floor derives a partition and runs it multi-process with
/// the graph file deliberately untouched, so "does the file declare groups?"
/// and "did this run execute groups?" are different questions with different
/// answers on exactly the headless/robot shape.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PartitionProvenance {
    /// The run uses whatever the graph file declares — including nothing.
    Declared,
    /// A partition was DERIVED and adopted in memory; the file is untouched.
    DerivedInMemory,
    /// A partition was DERIVED and written to the graph file.
    DerivedPersisted,
    /// `--auto-partition` was declined: the file's EXISTING block is used.
    KeptExisting,
    /// The file already carried exactly the derived partition.
    AlreadyCurrent,
}

impl PartitionProvenance {
    /// The stable wire label. Written into `run.json`, so these strings are a
    /// format surface: rename one and you break a reader.
    #[must_use]
    pub fn label(self) -> &'static str {
        match self {
            PartitionProvenance::Declared => "declared",
            PartitionProvenance::DerivedInMemory => "derived-in-memory",
            PartitionProvenance::DerivedPersisted => "derived-persisted",
            PartitionProvenance::KeptExisting => "kept-existing",
            PartitionProvenance::AlreadyCurrent => "already-current",
        }
    }
}

/// The run's network POSTURE, as decided by
/// `graph_cmd::resolve_run_network`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NetworkPostureLabel {
    /// `--network off` / `CERULION_NETWORK=off`: local-only.
    Off,
    /// A replay-class clock: the network plane is inert.
    Inert,
    /// An explicit enabled `network:` block.
    Strict,
    /// The default: every produced topic announced + egressable.
    Permissive,
}

impl NetworkPostureLabel {
    /// The stable wire label (a `run.json` format surface — see
    /// [`PartitionProvenance::label`]).
    #[must_use]
    pub fn label(self) -> &'static str {
        match self {
            NetworkPostureLabel::Off => "off",
            NetworkPostureLabel::Inert => "inert",
            NetworkPostureLabel::Strict => "strict",
            NetworkPostureLabel::Permissive => "permissive",
        }
    }
}

/// The inputs [`start_run_descriptor`] needs.
///
/// The three artifact bodies arrive ALREADY RENDERED, by the callers' own
/// renderers (`render_effective_graph_yaml` / `render_env_json` /
/// `render_recorder_json`). That is the anti-drift property, stated precisely:
/// the run directory and a `--record` bag carry the same bytes because they are
/// produced by the same pure function, not because two writers are believed to
/// agree.
pub struct RunDescriptorSpec<'a> {
    /// This run's IDENTITY, minted by the CALLER before it asks for any of this
    /// to be written.
    ///
    /// Minting it inside [`start_run_descriptor`] would make the
    /// identity a by-product of a successful write. That is backwards, and it
    /// would cost the always-on capture plane: `graph_run` derives the Flashback arm
    /// tag from the run id, so on a host where the description cannot be
    /// written — the read-only home and full-disk cases this module degrades over
    /// on purpose — there would be no id, no tag, and therefore no plane and no
    /// ring. The black box would switch off because a bookkeeping file could not be
    /// created.
    ///
    /// Minting is infallible ([`mint_run_id`]) and the identity is what names
    /// the plane, so the caller mints it FIRST and hands it in. The run is then
    /// identified whether or not it turns out to be describable.
    pub run_id: u128,
    /// The run's identity — the FILE STEM the CLI resolved the graph by
    /// (the graph name), and the same value that stems a bag filename.
    ///
    /// The config's internal `name:` is optional-and-ignored, so there is ONE
    /// name; `run.json` renders BOTH keys (`graph_name` and `graph_file`)
    /// from this field for reader compatibility.
    pub graph_name: &'a str,
    /// The graph YAML that was actually LOADED (the CLI argument). Differs from
    /// `graph_name` whenever a graph's file name is not its internal `name:`.

    /// Wall-clock ns since the Unix epoch at run start.
    pub run_started_at_ns: u64,
    /// SHM the run declares BEFORE it dispatches — the classes
    /// whose tags are known this early.
    ///
    /// Trace and departure rings are declared LATER, by
    /// [`declare_run_rings`], because they do not exist yet at this seam. The
    /// checkpoint ARM word is the opposite case: its tag is a pure function of
    /// the run id (or of `CERULION_STATE_ARM_TAG`), both known before anything
    /// is created — so declaring it here is what makes a run that CRASHES
    /// DURING BRING-UP sweepable at all. That is exactly the run a sweeper
    /// exists for, and it is the one a declare-after-create rule cannot cover.
    ///
    /// It records the NAME SPACE this run may occupy, not a claim that it
    /// occupied it: a run whose plane the kill switch declined leaves an entry
    /// naming objects that never existed. Harmless to a sweeper (`shm_unlink` of
    /// an absent name is a silent ENOENT) and the opposite direction — a created
    /// object nothing names — is the leak this exists to prevent.
    pub shm: Vec<ShmDecl>,
    /// This run's network posture.
    pub network: NetworkPostureLabel,
    /// How this run's partition (if any) came to be.
    pub partition: PartitionProvenance,
    /// Whether the run actually executes `process_groups:`.
    pub process_groups: bool,
    /// How this run's GATING clock advances.
    ///
    /// Recorded because it is known HERE and nowhere else, and because a resim
    /// judge cannot recover it from a bag: a boundary produced by a clock the
    /// scheduler ADVANCES and one produced by a clock it merely READS are
    /// byte-identical records with opposite meanings. See [`GatingClock`].
    pub gating: GatingClock,
    /// The EFFECTIVE graph config, serialized.
    pub graph_yaml: String,
    /// The env snapshot, serialized.
    pub env_json: Vec<u8>,
    /// The recording host descriptor, serialized.
    pub recorder_json: Vec<u8>,
}

/// A live run's on-disk description PLUS its registry writer.
///
/// Dropping it removes the directory and stops the registry republish, so the
/// run stops being discoverable — which is exactly what a crash looks like, and
/// why its `Drop` announces a graceful end BEFORE releasing the writer (see
/// the `Drop` impl for why that announcement lives there and what it is worth).
pub struct RunDescriptor {
    dir: PathBuf,
    run_id: u128,
    /// `None` only when the registry could not be opened — the directory is
    /// still written (a human can read it), but nothing will point at it.
    handle: Option<RunHandle>,
    /// This run's HELD `run.lock`.
    ///
    /// Never read — holding it IS its purpose. It is what tells the next run's
    /// sweeper that this directory belongs to a live process, and the kernel
    /// releases it however this one ends, SIGKILL included. Dropped with the
    /// descriptor, i.e. at the same instant the directory is removed.
    #[cfg(unix)]
    _lock: crate::run_lock::RunLock,
}

/// One trace ring a run has CREATED, as `run.json` declares it.
///
/// The `tag` is the supervisor's own spelling (`cer_rec_<graph>_<pid>_r<N>`),
/// NOT the resolved POSIX SHM name — the two are different strings and a
/// consumer resolves the second from the first via
/// `cerulion_core::shm_ring::ring_shm_name`. Declaring the tag keeps the
/// manifest readable (`/dev/shm` breadcrumbs and log lines carry tags) and keeps
/// exactly one hash recipe in the system.
///
/// `rank` is the ring's header rank — the WORKER rank, or
/// `cerulion_core::trace_ring::DEPARTURE_RING_RANK` for the supervisor's
/// departure ring — carried because a recorder's head-step policy is chosen per
/// ring by rank, and because the one-bag-one-run check needs it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RingDecl {
    /// The ring TAG (not the resolved SHM name).
    pub tag: String,
    /// The ring's header rank.
    pub rank: u32,
}

// ===========================================================================
// The run.json TRACE vocabulary
// ===========================================================================

/// What a run says about its SCHEDULER-TRACE rings, as
/// `run.json`'s `trace_rings` key.
///
/// # Why a key at all, when `rings` already exists
///
/// `rings` is a LIST, and a list has exactly one degraded value: empty. Empty
/// today means at least four different things — the run declined rings, the run
/// wanted them and was refused, the run is an older binary that never had them,
/// or the reader could not parse the document — and a reader that renders one
/// sentence for all four states a fact about the RUN that nobody observed. That
/// is the positive-claim-from-an-absence class this crate splits everywhere
/// else (`RecordCoverage::mirrors_established`'s three-state `Option<bool>`,
/// `enumerated`-vs-`discovery_requested`).
///
/// So the run says, in its own words, WHICH of those happened — and the reader
/// keeps `absent ⇒ unknown`. Only [`Declined`](Self::Declined) and
/// [`Unavailable`](Self::Unavailable) license a positive claim; an absent key is
/// an older binary or a run that carries no declaration, and is
/// rendered as the legacy cause.
///
/// # The wire form
///
/// A single STRING, not an object, because the two degraded arms carry exactly
/// one datum (a reason) and a `{"state": …, "reason": …}` object would be a
/// second spelling of `"<state>: <reason>"`. Round-trips through
/// [`label`](Self::label) / [`parse`](Self::parse).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TraceRingsDecl {
    /// The run CREATED scheduler-trace rings; `rings` names them.
    ///
    /// Note what this does NOT promise: that every declared rank's ring exists.
    /// A rank whose worker failed to create its ring is reported separately, in
    /// `declared_unavailable` — because `rings` is stamped BEFORE creation (a
    /// supervisor spells the tags when it makes the plan) and a reader must be
    /// able to tell "declared and created" from "declared and absent".
    Declared,
    /// The run DECLINED rings at launch — BY CHOICE.
    ///
    /// The reason is the run's own words. It deliberately does NOT have to name
    /// a flag: the project's rule is that an absence explanation names the cause,
    /// not another verb's flag, and the reader of this value is holding a bag or
    /// a capture rather than a `graph run` command line.
    Declined { reason: String },
    /// The run wanted rings and could not have them — BY REFUSAL (e.g. a
    /// `/dev/shm` free-space gate, or a run shape whose clock cannot produce a
    /// resimmable trace).
    Unavailable { reason: String },
}

impl TraceRingsDecl {
    /// The `run.json` string form.
    #[must_use]
    pub fn label(&self) -> String {
        match self {
            TraceRingsDecl::Declared => "declared".to_string(),
            TraceRingsDecl::Declined { reason } => format!("declined: {reason}"),
            TraceRingsDecl::Unavailable { reason } => format!("unavailable: {reason}"),
        }
    }

    /// PURE: parse the `run.json` string form.
    ///
    /// Tolerant in ONE direction only, like `bag_cmd`'s own private
    /// `run_manifest_ring_tags`: an
    /// unrecognised value yields `None` (⇒ the reader's `unknown` arm) rather
    /// than an error, because the manifest is written by a different process and
    /// possibly a different version, and a recording must never be refused over
    /// a field the reader can simply not use. A recognised state with an EMPTY
    /// reason is still recognised — the state is the load-bearing half.
    #[must_use]
    pub fn parse(raw: &str) -> Option<Self> {
        let raw = raw.trim();
        if raw == "declared" {
            return Some(TraceRingsDecl::Declared);
        }
        if let Some(rest) = raw.strip_prefix("declined:") {
            return Some(TraceRingsDecl::Declined {
                reason: rest.trim().to_string(),
            });
        }
        if let Some(rest) = raw.strip_prefix("unavailable:") {
            return Some(TraceRingsDecl::Unavailable {
                reason: rest.trim().to_string(),
            });
        }
        None
    }
}

/// One rank that DECLARED a scheduler-trace ring which was
/// never created, and why.
///
/// This exists because the declaration is stamped BEFORE creation and cannot be
/// un-stamped afterwards: a supervisor spells every worker's ring tag when it
/// builds the plan, then each worker creates its own ring and reports the
/// outcome. Over-declaration is harmless to a SWEEPER (an unlink of a name that
/// was never created is a silent `ENOENT`) and is NOT harmless to a READER — it
/// would open the rings that exist, fail on the one that does not, and have no
/// way to say which fact it is reporting. So the failure is recorded by rank.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RankUnavailable {
    /// The ring's header rank (a worker rank, or
    /// `cerulion_core::trace_ring::DEPARTURE_RING_RANK`).
    pub rank: u32,
    /// Why creation failed, in the creator's own words.
    pub reason: String,
}

/// What CLASS of shared-memory object a ledger entry names.
///
/// The classes are not interchangeable to a sweeper: they are unlinked by
/// different owners on different exit paths, and a trace ring's name is a HASH
/// (`/cer_rg_<fnv1a64(tag)>`) while a state ring's is derived from the run id.
/// Recording the class is what lets a later sweep resolve a name from a tag at
/// all — a `/dev/shm` listing of hashed names is unattributable on its own,
/// which is the whole reason the ledger lives in `run.json`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShmClass {
    /// A per-rank scheduler-trace ring.
    Trace,
    /// The supervisor's departure ring.
    Departure,
    /// A per-rank checkpoint (state) ring.
    State,
    /// The run's checkpoint ARM word.
    Arm,
}

impl ShmClass {
    /// The `run.json` string form.
    #[must_use]
    pub fn label(self) -> &'static str {
        match self {
            ShmClass::Trace => "trace",
            ShmClass::Departure => "departure",
            ShmClass::State => "state",
            ShmClass::Arm => "arm",
        }
    }

    /// PURE: parse the `run.json` string form; unrecognised ⇒ `None`.
    #[must_use]
    pub fn parse(raw: &str) -> Option<Self> {
        match raw.trim() {
            "trace" => Some(ShmClass::Trace),
            "departure" => Some(ShmClass::Departure),
            "state" => Some(ShmClass::State),
            "arm" => Some(ShmClass::Arm),
            _ => None,
        }
    }
}

/// Where a ledger entry's TAG came from.
///
/// A DERIVED tag is a function of the run id and is therefore unique to this
/// run; an EXPLICIT one was handed in by an operator (`CERULION_STATE_ARM_TAG`)
/// and may be SHARED with a live run. A sweeper must treat the two differently
/// — an explicit name present in a live run's ledger must never be unlinked —
/// so the ledger records which it is rather than leaving a sweeper to guess from
/// the string.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TagSource {
    /// Derived from this run's `run_id` — unique to this run.
    Derived,
    /// Handed in by an operator; may be shared with another run.
    Explicit,
}

impl TagSource {
    /// The `run.json` string form.
    #[must_use]
    pub fn label(self) -> &'static str {
        match self {
            TagSource::Derived => "derived",
            TagSource::Explicit => "explicit",
        }
    }

    /// PURE: parse the `run.json` string form; unrecognised ⇒ `None`.
    #[must_use]
    pub fn parse(raw: &str) -> Option<Self> {
        match raw.trim() {
            "derived" => Some(TagSource::Derived),
            "explicit" => Some(TagSource::Explicit),
            _ => None,
        }
    }
}

/// One shared-memory object this run has DECLARED, for the
/// tag ledger a later sweep reads.
///
/// # What an entry means, and what its ABSENCE does not
///
/// The array is what the run has declared SO FAR, by class. A class that does
/// not appear is UNDECLARED — never "this run minted none of those". The
/// distinction is load-bearing for a sweeper: acting on absence here would make
/// it skip exactly the objects a partially-written ledger failed to record,
/// which are the ones a crashed run is most likely to have leaked.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ShmDecl {
    /// The object's TAG — the minter's own spelling, NOT the resolved POSIX SHM
    /// name. The two are different strings and the resolution is a hash fold
    /// (`cerulion_core::shm_ring::ring_shm_name`); declaring the tag keeps
    /// exactly one hash recipe in the system.
    pub tag: String,
    /// What kind of object it is.
    pub class: ShmClass,
    /// The object's rank, where it has one (`None` for the arm word).
    pub rank: Option<u32>,
    /// Whether the tag is derived from the run id or was handed in.
    pub source: TagSource,
}

/// How this run's GATING clock advances — the fact a
/// resim judge needs and cannot recover from a bag.
///
/// # Why a run has to say it
///
/// `bag play --resim` is driven by recorded step BOUNDARIES, and a boundary is
/// only re-advanceable if the gating clock was ADVANCED by the scheduler rather
/// than merely READ from it. Those two shapes produce byte-identical trace
/// records, so nothing downstream can tell them apart — which is why a ring
/// minted on the read-only arm would yield a confident-false `resimmable: true`
/// that resim then refuses at exit 2. The arm is known at launch and nowhere
/// else, so the run records it.
///
/// The variants are the `GraphRuntime::live_step` gating arms, one for one, plus
/// the polled shape that never enters that loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GatingClock {
    /// The scheduler advances a controlled clock by a FIXED LOGICAL QUANTUM
    /// (`live_gating_quantum = Some(q)`, deterministic-live). Run-independent,
    /// and the shape γ's rings are minted on.
    Quantum,
    /// The scheduler advances a CONTROLLED clock by the MEASURED WALL elapsed
    /// (`live_gating_quantum = Some(_)` with `gating_follows_wall`) — today's
    /// `--single-process --record` discipline, and every rank of
    /// a FREE-RUN `--record` deployment, from a shared epoch. Jitter is
    /// preserved, but every boundary is still recorded and re-advanceable, so
    /// it is resim-grade.
    RecordedWall,
    /// The scheduler does NOT advance the gating clock; every boundary is a
    /// CLOCK READING (`live_gating_quantum = None`, `ClockInner::Real`). Covers
    /// both `RealClock` (`--single-process`, `ros2 attach`, `node run`, and
    /// every rank of a non-record FREE-RUN deployment) and
    /// `ExternalClock`, which is read-only in the same sense — the scheduler
    /// never advances it either.
    Wall,
    /// The run never enters the live loop at all: a polled `step()` monolith
    /// under `--time-source virtual`.
    Polled,
}

impl GatingClock {
    /// The `run.json` string form.
    #[must_use]
    pub fn label(self) -> &'static str {
        match self {
            GatingClock::Quantum => "quantum",
            GatingClock::RecordedWall => "recorded_wall",
            GatingClock::Wall => "wall",
            GatingClock::Polled => "polled",
        }
    }

    /// PURE: parse the `run.json` string form; unrecognised ⇒ `None` (a reader
    /// keeps `absent ⇒ unknown`).
    #[must_use]
    pub fn parse(raw: &str) -> Option<Self> {
        match raw.trim() {
            "quantum" => Some(GatingClock::Quantum),
            "recorded_wall" => Some(GatingClock::RecordedWall),
            "wall" => Some(GatingClock::Wall),
            "polled" => Some(GatingClock::Polled),
            _ => None,
        }
    }

    /// PURE: classify a run's gating arm from the three deployment facts known
    /// at launch plus the resolved execution mode.
    ///
    /// One function so the CLI cannot spell the classification twice and drift:
    /// the same three facts already decide the deployment (the mode is resolved
    /// from that decision plus the opt-in), and the mapping is exactly the
    /// `live_step` match plus the polled shape.
    ///
    /// * a SUPERVISOR run under the default LOCKSTEP execution mode builds
    ///   every worker through `build_live_deterministic_with_manager_and_barrier`,
    ///   which hands a quantum ⇒ [`Quantum`](Self::Quantum); under the
    ///   `CERULION_EXECUTION_MODE=free_run` opt-in every rank is
    ///   on its own wall-faithful clock — a RECORDING free-run rank follows the
    ///   wall on a controlled clock from a shared epoch ⇒
    ///   [`RecordedWall`](Self::RecordedWall), a live one is on the read-only
    ///   `RealClock` ⇒ [`Wall`](Self::Wall) (the mode is threaded from the ONE
    ///   resolution `graph run` makes, so this label and the run's
    ///   `coordination` stamp cannot disagree);
    /// * a `--time-source virtual` monolith runs the polled loop and never
    ///   reaches `live_step` ⇒ [`Polled`](Self::Polled);
    /// * a RECORDING monolith is configured `gating_follows_wall` on a
    ///   controlled clock ⇒ [`RecordedWall`](Self::RecordedWall);
    /// * everything else is the read-only arm ⇒ [`Wall`](Self::Wall).
    ///
    /// `--record` is refused under virtual and external clocks, so the
    /// `RecordedWall` arm cannot be reached with a non-real clock and the order
    /// of the last two tests is not load-bearing — it is written virtual-first
    /// anyway, so a future relaxation of that refusal does not silently relabel
    /// a polled run.
    #[must_use]
    pub fn classify(
        supervisor: bool,
        virtual_time_source: bool,
        records: bool,
        execution_mode: crate::multiprocess::ExecutionMode,
    ) -> Self {
        if supervisor {
            match execution_mode {
                crate::multiprocess::ExecutionMode::Lockstep => GatingClock::Quantum,
                crate::multiprocess::ExecutionMode::FreeRun if records => GatingClock::RecordedWall,
                crate::multiprocess::ExecutionMode::FreeRun => GatingClock::Wall,
            }
        } else if virtual_time_source {
            GatingClock::Polled
        } else if records {
            GatingClock::RecordedWall
        } else {
            GatingClock::Wall
        }
    }
}

/// Everything a run declares about its scheduler trace, in
/// ONE write.
///
/// The four keys are written together because they are one statement and a
/// reader takes them together: `trace_rings` says which of the states the run is
/// in, `rings` names what a recorder may open, `declared_unavailable` names the
/// declared ranks it may NOT, and `shm` is the sweep ledger. Splitting them
/// across writes would let a reader see a `declared` state with no ring list, or
/// a ring list with no statement about the ranks missing from it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RunTraceDecl {
    /// The trace rings a recorder may open, in declaration order.
    pub rings: Vec<RingDecl>,
    /// Which of the three states this run is in.
    pub state: TraceRingsDecl,
    /// Declared ranks whose ring was never created.
    pub declared_unavailable: Vec<RankUnavailable>,
    /// The SHM tag ledger — what this run has declared, by class.
    pub shm: Vec<ShmDecl>,
}

impl RunTraceDecl {
    /// The ordinary shape: rings were created, nothing failed, and the ledger is
    /// derived from the rings themselves.
    ///
    /// The class comes from the RANK, through
    /// `cerulion_core::trace_ring::DEPARTURE_RING_RANK`, so the one place that
    /// knows what a departure ring is stays the one place that knows.
    #[must_use]
    pub fn declared(rings: Vec<RingDecl>) -> Self {
        let shm = rings
            .iter()
            .map(|r| ShmDecl {
                tag: r.tag.clone(),
                class: if r.rank == cerulion_core::trace_ring::DEPARTURE_RING_RANK {
                    ShmClass::Departure
                } else {
                    ShmClass::Trace
                },
                rank: Some(r.rank),
                // Every ring tag is spelled from the run's own identity by the
                // supervisor; none of them is operator-supplied.
                source: TagSource::Derived,
            })
            .collect();
        Self {
            rings,
            state: TraceRingsDecl::Declared,
            declared_unavailable: Vec::new(),
            shm,
        }
    }
}

/// Declare the trace rings this run created, into the `run.json`
/// it already wrote.
///
/// # Why this is a SECOND write rather than a field on [`RunDescriptorSpec`]
///
/// It is not a convenience — it is the only correct ORDER. The rings do not
/// exist when the descriptor is written: `begin_run_descriptor` runs before the
/// deployment dispatch (deliberately, so it covers every exit path), while the
/// rings are created inside the record bring-up, after the plan is made and the
/// workers are READY. A manifest that named them earlier would advertise SHM
/// objects `shm_open` cannot find, and an attaching recorder would refuse or
/// degrade over rings that were merely not created yet. Declaring them once they
/// exist means the manifest's `rings` array is a statement about reality at the
/// moment it is written.
///
/// It also keeps the WRITER and the MINTER the same code: the tags are spelled
/// by `graph_cmd`'s own `worker_recording_ring_tag`/`departure_ring_tag`, and
/// nothing here re-derives them from a naming convention.
///
/// # Errors
///
/// Returns the underlying I/O or JSON error. Callers treat it as a DEGRADE, not
/// a failure: a run that cannot describe its rings still records perfectly well
/// through the bag its own `--record` is writing; what is lost is a LATER
/// recorder's ability to pick up the scheduler trace.
pub fn declare_run_rings(run_dir: &Path, rings: &[RingDecl]) -> CliResult<()> {
    declare_run_trace(run_dir, &RunTraceDecl::declared(rings.to_vec()))
}

/// Declare EVERYTHING this run has to say about its
/// scheduler trace, into the `run.json` it already wrote.
///
/// The general form of [`declare_run_rings`], which is now the "rings were
/// created and nothing failed" shortcut over it. The ordering argument in that
/// function's docs applies unchanged and is the reason this is a SECOND write:
/// the rings do not exist when the descriptor is written.
///
/// # Why the four keys are one write
///
/// A reader takes them TOGETHER — `trace_rings` says which state the run is in,
/// `rings` names what may be opened, `declared_unavailable` names the declared
/// ranks that may not, and `shm` is the sweep ledger. Written separately, a
/// reader could observe a `declared` state with no ring list, or a ring list
/// with no statement about the ranks missing from it, and would have to invent
/// a rule for the inconsistency. One write, one statement.
///
/// # Errors
///
/// Returns the underlying I/O or JSON error. Callers treat it as a DEGRADE, not
/// a failure: a run that cannot describe its trace still runs, and still records
/// through whatever bag its own `--record` is writing; what is lost is a LATER
/// recorder's ability to pick that trace up, and a later sweep's ability to
/// reclaim these names.
pub fn declare_run_trace(run_dir: &Path, decl: &RunTraceDecl) -> CliResult<()> {
    edit_run_manifest(run_dir, "trace rings", |obj| {
        let rings = &decl.rings[..];
        // Entries this writer does not own, read BEFORE anything is replaced. It
        // owns TRACE and DEPARTURE; every other class was declared by somebody else
        // (today: the ARM word, at descriptor write) and must survive this rewrite.
        let preserved: Vec<serde_json::Value> = obj
            .get("shm")
            .and_then(|v| v.as_array())
            .map(|entries| {
                entries
                    .iter()
                    .filter(|e| {
                        !matches!(
                            e.get("class")
                                .and_then(|c| c.as_str())
                                .and_then(ShmClass::parse),
                            Some(ShmClass::Trace) | Some(ShmClass::Departure)
                        )
                    })
                    .cloned()
                    .collect()
            })
            .unwrap_or_default();
        obj.insert(
            "rings".to_string(),
            serde_json::Value::Array(
                rings
                    .iter()
                    .map(|r| serde_json::json!({ "tag": r.tag, "rank": r.rank }))
                    .collect(),
            ),
        );
        // The run's own statement about WHICH of the trace states it
        // is in. `rings` alone cannot carry it — an empty list is four different
        // facts (see `TraceRingsDecl`) — and a reader keeps `absent ⇒ unknown`, so
        // this key is what licenses any positive claim about a trace-less run.
        obj.insert(
            "trace_rings".to_string(),
            serde_json::Value::String(decl.state.label()),
        );
        // Declared ranks whose ring was never created. Written even when EMPTY, and
        // that is deliberate: written only when non-empty, its absence would be
        // ambiguous between "no rank failed" and "this writer does not report rank
        // failures", and a reader deciding whether it may render a from-attach
        // verdict needs the first.
        obj.insert(
            "declared_unavailable".to_string(),
            serde_json::Value::Array(
                decl.declared_unavailable
                    .iter()
                    .map(|u| serde_json::json!({ "rank": u.rank, "reason": u.reason }))
                    .collect(),
            ),
        );
        // The SHM tag ledger a later sweep reads. `/dev/shm` carries
        // HASHED names (`/cer_rg_<fnv1a64(tag)>`), so a listing is unattributable
        // and this file is the only place a name can be resolved back to the run
        // that minted it. What is here is what has been DECLARED — a class that does
        // not appear is UNDECLARED, never "this run minted none".
        obj.insert(
            "shm".to_string(),
            // MERGED, not replaced. This writer owns the TRACE and DEPARTURE
            // classes; the ARM word is declared earlier, at descriptor write, when
            // its tag first becomes known. A wholesale `insert` of this writer's
            // vector DELETED that entry — and deleting it does not merely lose a
            // line, it makes the ledger claim the arm class is UNDECLARED, which
            // `ShmDecl`'s own contract says means "nothing can be concluded". A
            // sweeper reading that skips exactly the objects the run really did
            // create. So entries of classes this writer does not own are carried
            // through verbatim.
            serde_json::Value::Array(
                preserved
                    .into_iter()
                    .chain(
                        render_shm_entries(&decl.shm)
                            .as_array()
                            .expect("render_shm_entries yields an array")
                            .iter()
                            .cloned(),
                    )
                    .collect(),
            ),
        );
    })
}

/// PURE-ish: read `run.json`, hand its object to `edit`, and write it back.
///
/// Extracted so the two writers that AMEND a run manifest in place — the
/// trace declaration and the state-ring-consumer declaration — share
/// ONE read/rewrite shell rather than two copies of it (the no-second-copy rule). What
/// is genuinely shared is not convenience: it is the 0600 RE-ASSERT below, which
/// `write_artifact`'s `OpenOptions` mode cannot supply on a rewrite, and which a
/// second copy would be free to forget.
///
/// `what` names the thing being declared, so a failure says which declaration
/// was lost rather than "a manifest edit failed".
///
/// # Errors
///
/// Returns the underlying I/O or JSON error. Every caller treats it as a
/// DEGRADE, not a failure — see [`declare_run_trace`].
fn edit_run_manifest(
    run_dir: &Path,
    what: &str,
    edit: impl FnOnce(&mut serde_json::Map<String, serde_json::Value>),
) -> CliResult<()> {
    let path = run_dir.join(RUN_MANIFEST_FILE);
    let bytes = std::fs::read(&path).map_err(|e| {
        CliError::Validation(format!(
            "cannot read `{}` to declare this run's {what}: {e}",
            path.display()
        ))
    })?;
    let mut doc: serde_json::Value = serde_json::from_slice(&bytes).map_err(|e| {
        CliError::Validation(format!(
            "`{}` is not readable JSON, so this run's {what} cannot be declared: {e}",
            path.display()
        ))
    })?;
    let obj = doc.as_object_mut().ok_or_else(|| {
        CliError::Validation(format!(
            "`{}` is not a JSON object, so this run's {what} cannot be declared",
            path.display()
        ))
    })?;
    edit(obj);
    let mut out = serde_json::to_vec_pretty(&doc)
        .map_err(|e| CliError::Validation(format!("cannot re-render `{}`: {e}", path.display())))?;
    out.push(b'\n');
    // ATOMIC: write a sibling and RENAME over the manifest.
    //
    // `write_artifact`'s `OpenOptions` carries `.truncate(true)`, so an in-place
    // rewrite leaves `run.json` at ZERO BYTES between `open` and `write_all` —
    // and a run is registered, and therefore attachable, long before either
    // amend runs. A `cerulion bag record --run` landing in that window reads
    // bytes that parse as nothing, loses the run's ring list AND its
    // state-ring-consumer statement, and reports both as a fact about the
    // ARTIFACT ("could not be parsed") rather than about the writer that
    // truncated it — every visible signal pointing at a corrupt run directory.
    //
    // Flashback is what forces the atomic rewrite rather than merely inviting it:
    // the trace amend runs BEFORE the GO sentinel, while the state-ring-consumer
    // declaration runs AFTER it, squarely inside the attachable window. `rename`
    // on the same filesystem is atomic, so a reader sees either the old document
    // or the new one and never a partial.
    let tmp_name = format!("{RUN_MANIFEST_FILE}.tmp");
    let tmp_path = run_dir.join(&tmp_name);
    // A failed sibling write is cleaned up and reported under the name the
    // OPERATOR knows. `write_artifact`'s message would name `run.json.tmp`,
    // which is an implementation detail of this function and appears in no other
    // diagnostic — and leaving the partial behind is the residue the rename arm
    // below argues against.
    if let Err(e) = write_artifact(run_dir, &tmp_name, &out) {
        let _ = std::fs::remove_file(&tmp_path);
        return Err(CliError::Validation(format!(
            "cannot write this run's {what} into `{}`: {e}",
            path.display()
        )));
    }
    std::fs::rename(&tmp_path, &path).map_err(|e| {
        // Best effort: leaving the sibling behind would make the next amend's
        // `create(true)` reuse it, which is harmless, but a stray `.tmp` in a
        // run directory reads as a crash that did not happen.
        let _ = std::fs::remove_file(&tmp_path);
        CliError::Validation(format!(
            "cannot replace `{}` with its rewritten form: {e}",
            path.display()
        ))
    })?;
    // …and then set them EXPLICITLY, because `write_artifact`'s `OpenOptions`
    // mode applies only at CREATION and this is by construction a rewrite of a
    // file that already exists. Relying on the first write's bits would be an
    // assumption about a file this call did not create — and the manifest sits
    // beside `env.json`, whose whole rule is that a run's description is
    // owner-only on a shared machine. (Found by this function's own pin, which
    // seeded a 0644 manifest and read 0644 back out.)
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        // A UMASK BACKSTOP, not the primary mechanism — and the distinction
        // rests on the atomic rewrite above. A rewrite that truncated
        // `run.json` IN PLACE would keep whatever bits the file already had
        // (a real 0644 manifest is the shape). `rename` installs a file `write_artifact` just created
        // under `OpenOptions::mode(RUN_FILE_MODE)`, so the bits are already
        // right on every ordinary path. What remains is a umask that strips
        // owner-write, and a stale `.tmp` this call re-opened rather than
        // created.
        //
        // NOT an `Err`: the declaration was written. Returning here would report a
        // successful write as a lost one, and every caller of this shell treats
        // an `Err` as "the later attach cannot read what this run decided" —
        // which would be false. The permission miss is its own, lesser fact, so
        // it gets its own line.
        if let Err(e) =
            std::fs::set_permissions(&path, std::fs::Permissions::from_mode(RUN_FILE_MODE))
        {
            tracing::warn!(
                path = %path.display(),
                error = %e,
                "this run's manifest was rewritten but its owner-only permissions \
                 could not be restored — the declaration itself landed; on a shared machine \
                 the run's description may be readable by other users"
            );
        }
    }
    Ok(())
}

/// What a READER found when it looked for `trace_rings` —
/// THREE shapes, because two of them are different facts about different
/// subjects.
///
/// # Why absent and unrecognised cannot be one value
///
/// They are claims about different parties. An ABSENT key is a fact about the
/// RUN: it said nothing, because it is an older build or its declaration never
/// landed, and the reader may correctly serve the legacy by-omission cause. An
/// UNRECOGNISED value is a fact about THIS BUILD: the run DID say something and
/// this reader cannot read it — almost always because a newer `cerulion` wrote
/// a state that did not exist when this binary was compiled.
///
/// Collapsing the second onto the first turns "I cannot read this" into "the
/// run declared no trace rings", which is a confident FALSE claim about a
/// manifest a newer build wrote, on evidence that says the opposite. That is
/// precisely the positive-claim-from-an-absence rule the trace vocabulary
/// exists to enforce — `absent ⇒ unknown`, never `absent ⇒ a wrong answer` —
/// and it is the rule this type makes unskippable rather than remembered.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum TraceRingsReport {
    /// The manifest carries no `trace_rings` key at all: an older build,
    /// or a run that carries no declaration. A fact about the RUN.
    ///
    /// Also what an UNPARSEABLE document yields, which is safe because every
    /// caller must establish [`RunArtifacts::manifest_parsed`] before it may
    /// make any claim at all — see [`run_manifest_trace_rings`].
    ///
    /// [`RunArtifacts::manifest_parsed`]: crate::bag_cmd::RunArtifacts::manifest_parsed
    #[default]
    Absent,
    /// A value this build understands.
    Known(TraceRingsDecl),
    /// A value this build does NOT understand. A fact about this BUILD.
    ///
    /// The raw token is carried so the reader can NAME what it could not read:
    /// an operator holding a bag that says "this build cannot read `<token>`"
    /// can act on it, while one told "no trace rings were declared" has been
    /// sent somewhere else entirely.
    Unrecognised {
        /// The value verbatim, as the manifest carried it (trimmed).
        raw: String,
    },
}

/// PURE — the trace-ring state a run manifest declares.
///
/// Lives here, beside the writer that produces it, so the key name and its
/// grammar are spelled ONCE.
///
/// # What each answer means
///
/// * key absent, or the bytes do not parse ⇒ [`TraceRingsReport::Absent`];
/// * a value this build knows ⇒ [`TraceRingsReport::Known`];
/// * a value it does not ⇒ [`TraceRingsReport::Unrecognised`], carrying the
///   token — NOT `Absent`, see that type's docs for why the distinction is
///   load-bearing rather than tidy.
///
/// Folding UNPARSEABLE bytes onto `Absent` is safe HERE and only here, for the
/// same reason `run_manifest_declared_unavailable` may return an empty vector
/// for them: every caller must first establish that the manifest PARSED
/// (`RunArtifacts::manifest_parsed`) before it may turn any of this into a
/// claim, and that check outranks this one.
///
/// Tolerant of SHAPE in one direction only, exactly as
/// `bag_cmd::run_manifest_ring_tags` is — a non-string value is not an error,
/// because the manifest is written by a different process and possibly a
/// different version, and a recording must never be refused over a field the
/// reader can simply not use. A non-string is reported `Absent` rather than
/// `Unrecognised`: `Unrecognised` promises a token a human can act on, and a
/// JSON array or number is not one.
#[must_use]
pub fn run_manifest_trace_rings(run_json: &[u8]) -> TraceRingsReport {
    let Ok(doc) = serde_json::from_slice::<serde_json::Value>(run_json) else {
        return TraceRingsReport::Absent;
    };
    let Some(raw) = doc.get("trace_rings").and_then(|v| v.as_str()) else {
        return TraceRingsReport::Absent;
    };
    match TraceRingsDecl::parse(raw) {
        Some(decl) => TraceRingsReport::Known(decl),
        None => TraceRingsReport::Unrecognised {
            raw: raw.trim().to_string(),
        },
    }
}

/// PURE — the declared ranks whose ring was never created.
///
/// An EMPTY vector is returned for "the key says none", for an absent key, and
/// for bytes that do not parse. That collapse is safe HERE and only here,
/// because every caller has already had to establish that the manifest parsed
/// before it may make any claim at all (`RunArtifacts::manifest_parsed`) — the
/// same discipline `run_manifest_ring_tags` relies on. An entry missing a usable
/// `rank` is DROPPED rather than defaulted: rank 0 is a real worker, so a
/// defaulted rank would accuse an innocent one.
#[must_use]
pub fn run_manifest_declared_unavailable(run_json: &[u8]) -> Vec<RankUnavailable> {
    let Ok(doc) = serde_json::from_slice::<serde_json::Value>(run_json) else {
        return Vec::new();
    };
    let Some(entries) = doc.get("declared_unavailable").and_then(|v| v.as_array()) else {
        return Vec::new();
    };
    entries
        .iter()
        .filter_map(|e| {
            let rank = u32::try_from(e.get("rank")?.as_u64()?).ok()?;
            Some(RankUnavailable {
                rank,
                // A missing reason is not a missing FAILURE: the rank is the
                // load-bearing half and the reason is what the creator managed
                // to say. Rendering "unstated" beats dropping the rank.
                reason: e
                    .get("reason")
                    .and_then(|r| r.as_str())
                    .unwrap_or("reason unstated")
                    .to_string(),
            })
        })
        .collect()
}

/// PURE: render `shm` ledger entries, in the ONE shape
/// [`run_manifest_shm`] parses.
///
/// Shared by the descriptor write and [`declare_run_rings`] so the two writers
/// cannot spell one ledger two ways — the reader is a single parser, and a
/// second spelling is how a class silently stops being readable.
fn render_shm_entries(decls: &[ShmDecl]) -> serde_json::Value {
    serde_json::Value::Array(
        decls
            .iter()
            .map(|d| {
                serde_json::json!({
                    "tag": d.tag,
                    "class": d.class.label(),
                    "rank": d.rank,
                    "source": d.source.label(),
                })
            })
            .collect(),
    )
}

/// `run.json`'s `state_ring_consumer` key.
///
/// # The question it answers, and why nothing else could
///
/// A run's per-rank node-STATE rings are `OverrunPolicy::Backpressure`: every
/// consumer stores its cursor into ONE shared header slot and the producer
/// trusts whoever published last, so two consumers of one state ring lap each
/// other and the slower one is retired with an `Overrun` it never asked for.
/// By default every ordinary `graph run` starts a standing
/// Flashback window recorder, and that recorder is handed the run's
/// capture-plane tag — so it IS a state-ring consumer, on the DEFAULT run shape.
///
/// A `cerulion bag record --run` attaching to that run would be the second one.
/// The attach can see the run directory, so the run WRITES DOWN
/// what it decided; without this key that decision would exist only as a log line,
/// which a later process cannot read.
///
/// # Why the state ring needs its own key rather than riding `trace_rings`
///
/// Different plane, different lifecycle, different answer. Trace rings are
/// created by the run and drained by any number of readers (by design:
/// `FailLoud`, LOCAL cursors, N independent readers is sound); the state ring is
/// SPSC-by-cursor and admits exactly one. A run can perfectly well declare trace
/// rings while starting no window recorder (`CERULION_FLASHBACK=off` is exactly
/// that shape), so one key cannot answer both questions.
///
/// # What `Standing` claims, exactly
///
/// That a recorder this run started was handed this run's capture-plane tag, so
/// it sweeps the per-rank state rings. It does NOT claim the recorder is still
/// alive — nothing in a manifest can — which is why the attach's refusal names
/// the standing recorder as the place anchors come from rather than promising
/// that any particular anchor exists.
///
/// # The wire form
///
/// A single STRING, exactly as [`TraceRingsDecl`] is and for the same reason:
/// the degraded arm carries one datum (a reason) and `{"state": …, "reason": …}`
/// would be a second spelling of `"none: <reason>"`. Round-trips through
/// [`label`](Self::label) / [`parse`](Self::parse).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StateRingConsumerDecl {
    /// A recorder this run started holds this run's capture-plane tag, so it is
    /// draining the per-rank state rings.
    Standing,
    /// Nothing this run started is draining them, and why — the run's own words.
    ///
    /// It deliberately does not have to name a flag: the project's rule is that an
    /// absence explanation names the CAUSE, not another verb's flag, and the
    /// reader of this value is attaching a recorder rather than typing a
    /// `graph run` command line.
    None {
        /// Why no standing consumer exists, in the run's own words.
        reason: String,
    },
}

impl StateRingConsumerDecl {
    /// The `run.json` string form.
    #[must_use]
    pub fn label(&self) -> String {
        match self {
            StateRingConsumerDecl::Standing => "standing".to_string(),
            StateRingConsumerDecl::None { reason } => format!("none: {reason}"),
        }
    }

    /// PURE: parse the `run.json` string form.
    ///
    /// Tolerant in ONE direction only, like [`TraceRingsDecl::parse`]: an
    /// unrecognised value yields `None` (⇒ the reader's UNKNOWN arm) rather than
    /// an error, because the manifest is written by a different process and
    /// possibly a different version, and an attach must never be refused over a
    /// field the reader can simply not use. A recognised state with an EMPTY
    /// reason is still recognised — the state is the load-bearing half.
    #[must_use]
    pub fn parse(raw: &str) -> Option<Self> {
        let raw = raw.trim();
        if raw == "standing" {
            return Some(StateRingConsumerDecl::Standing);
        }
        if let Some(rest) = raw.strip_prefix("none:") {
            return Some(StateRingConsumerDecl::None {
                reason: rest.trim().to_string(),
            });
        }
        None
    }
}

/// What a READER found when it looked for
/// `state_ring_consumer` — THREE shapes, for exactly the reasons
/// [`TraceRingsReport`] has three.
///
/// An ABSENT key is a fact about the RUN (a build older than the key, or a
/// run with no declaration); an UNRECOGNISED value is a fact about THIS
/// BUILD. Neither licenses a claim, and they are kept apart so a reader can say
/// WHICH it is rather than blaming the run for its own inability to read.
///
/// Only [`Known`](Self::Known) licenses a positive claim in either direction —
/// and note the asymmetry that makes the distinction matter here: `Absent` must
/// behave as "proceed, and say the manifest predates the key", never as "no
/// standing consumer", because the second is exactly the confident-false answer
/// that laps a state ring.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum StateRingConsumerReport {
    /// The manifest carries no `state_ring_consumer` key at all: an older
    /// build, or a run that carries no declaration. A fact about the RUN.
    ///
    /// Also what an UNPARSEABLE document yields, which is safe for the same
    /// reason [`TraceRingsReport::Absent`] is: every caller must establish that
    /// the manifest parsed before it may make any claim.
    #[default]
    Absent,
    /// A value this build understands.
    Known(StateRingConsumerDecl),
    /// A value this build does NOT understand. A fact about this BUILD.
    Unrecognised {
        /// The value verbatim, as the manifest carried it (trimmed).
        raw: String,
    },
}

/// PURE — the state-ring-consumer state a run manifest
/// declares.
///
/// Lives beside its writer so the key name and its grammar are spelled ONCE.
/// Shape tolerance and the unparseable-⇒-`Absent` fold are exactly
/// [`run_manifest_trace_rings`]'s, for the reasons stated there.
#[must_use]
pub fn run_manifest_state_ring_consumer(run_json: &[u8]) -> StateRingConsumerReport {
    let Ok(doc) = serde_json::from_slice::<serde_json::Value>(run_json) else {
        return StateRingConsumerReport::Absent;
    };
    let Some(value) = doc.get("state_ring_consumer") else {
        return StateRingConsumerReport::Absent;
    };
    // A present key holding a NON-STRING is `Unrecognised`, NOT `Absent`, and
    // this is where this reader deliberately parts company with its trace
    // sibling. `run_manifest_trace_rings` folds a non-string onto `Absent` on
    // the grounds that `Unrecognised` promises a token a human can act on; there
    // the cost of being wrong is a thinner sentence. Here the cost is a
    // DECISION: `Absent` renders "this run's manifest carries no
    // state-ring-consumer statement" for a manifest whose statement is right
    // there, and sends the attach down the proceed arm on a run that may have
    // said `standing` in a shape this build cannot read. An object form
    // (`{"state": "standing", …}`) is the single most likely way a newer
    // `cerulion` extends this key, so it is exactly the case the third state
    // exists for. The value is rendered compactly so the operator sees WHAT it
    // could not read.
    let Some(raw) = value.as_str() else {
        return StateRingConsumerReport::Unrecognised {
            raw: value.to_string(),
        };
    };
    match StateRingConsumerDecl::parse(raw) {
        Some(decl) => StateRingConsumerReport::Known(decl),
        None => StateRingConsumerReport::Unrecognised {
            raw: raw.trim().to_string(),
        },
    }
}

/// Declare whether a standing Flashback recorder is draining
/// this run's state rings, into the `run.json` it already wrote.
///
/// # Why this is a THIRD write and cannot be a field on `RunDescriptorSpec`
///
/// The same ORDER argument [`declare_run_trace`] makes, one step further out.
/// The descriptor is written before the deployment dispatch; the window-recorder
/// decision is taken after the GO sentinel on the supervisor path and after the
/// runtime is built on the monolith one — and it is not merely a policy read,
/// because a best-effort spawn can FAIL (no capture directory, no `current_exe`,
/// a refused `spawn`). Predicting it at descriptor time would claim `standing`
/// for a run whose recorder never started, which is the
/// positive-claim-from-an-absence class this crate refuses everywhere else.
///
/// # Errors
///
/// Returns the underlying I/O or JSON error. Callers treat it as a DEGRADE: the
/// run and its recorder are unaffected, and what is lost is a later attach's
/// ability to tell whether it would be a second state-ring consumer.
pub fn declare_state_ring_consumer(run_dir: &Path, decl: &StateRingConsumerDecl) -> CliResult<()> {
    edit_run_manifest(run_dir, "state-ring consumer", |obj| {
        obj.insert(
            "state_ring_consumer".to_string(),
            serde_json::Value::String(decl.label()),
        );
    })
}

/// PURE — the SHM tag ledger a run manifest declares.
///
/// Entries with an unusable `tag`, an unknown `class`, or an unknown `source`
/// are DROPPED. That is the strict direction on purpose: this vector feeds a
/// sweeper that UNLINKS names, and an entry it cannot fully understand is one it
/// must not act on. Absence of a class means UNDECLARED, never "none minted" —
/// see [`ShmDecl`].
#[must_use]
pub fn run_manifest_shm(run_json: &[u8]) -> Vec<ShmDecl> {
    let Ok(doc) = serde_json::from_slice::<serde_json::Value>(run_json) else {
        return Vec::new();
    };
    let Some(entries) = doc.get("shm").and_then(|v| v.as_array()) else {
        return Vec::new();
    };
    entries
        .iter()
        .filter_map(|e| {
            let tag = e.get("tag")?.as_str()?.trim();
            if tag.is_empty() {
                return None;
            }
            Some(ShmDecl {
                tag: tag.to_string(),
                class: ShmClass::parse(e.get("class")?.as_str()?)?,
                rank: e
                    .get("rank")
                    .and_then(|r| r.as_u64())
                    .and_then(|r| u32::try_from(r).ok()),
                source: TagSource::parse(e.get("source")?.as_str()?)?,
            })
        })
        .collect()
}

/// PURE — the gating-clock arm a run manifest
/// declares. `None` ⇒ the run did not say (an older build, unparseable, or a
/// spelling this reader does not know).
#[must_use]
pub fn run_manifest_gating(run_json: &[u8]) -> Option<GatingClock> {
    let doc: serde_json::Value = serde_json::from_slice(run_json).ok()?;
    GatingClock::parse(doc.get("gating")?.as_str()?)
}

impl RunDescriptor {
    /// The run directory.
    #[must_use]
    pub fn path(&self) -> &Path {
        &self.dir
    }

    /// This run's 128-bit identity.
    #[must_use]
    pub fn run_id(&self) -> u128 {
        self.run_id
    }

    /// Whether this run is actually DISCOVERABLE — i.e. a registry writer is
    /// live and publishing the pointer to [`RunDescriptor::path`]. `false`
    /// means the directory exists but nothing announces it (Principle #3: the
    /// degraded state is observable rather than assumed).
    #[must_use]
    pub fn is_discoverable(&self) -> bool {
        self.handle.is_some()
    }
}

impl Drop for RunDescriptor {
    /// Announce a GRACEFUL end, then remove the directory.
    ///
    /// The `Ending` announcement lives HERE rather than at an explicit call
    /// site because Drop is exactly the discriminator a consumer needs: a
    /// controlled exit — a clean return, an error return, an unwind — runs it,
    /// and a SIGKILL does not. So "the record said Ending" means the run
    /// stopped deliberately, and "the record simply stopped" means the process
    /// vanished, with no restructuring of `graph_run`'s several return paths
    /// to keep the two accurate.
    ///
    /// STATED AT ITS MEASURED STRENGTH, which is weaker than "a consumer that
    /// polls will see it". `set_state` commits the frame to SHM synchronously,
    /// but the publisher is released microseconds later and the registry
    /// service requests NO history — so the ONLY consumer that can observe
    /// `Ending` is one whose subscriber was already attached inside that
    /// window, and the window is the ~2 ms this `Drop` takes.
    ///
    /// MEASURED with a gatherer racing a graceful exit: **7 of 8**
    /// at a hot spin (a gather loop with no sleep at all), **0 of 8** at a
    /// natural 250 ms cadence. A polling consumer therefore sees the run
    /// DISAPPEAR, which is exactly what it sees for a crash.
    ///
    /// So a recorder cannot get the graceful/vanished discrimination by polling. Its
    /// options are a tight spin (wasteful) or a LONG-LIVED subscriber on the
    /// registry that never misses a frame — which this crate does not expose
    /// (`gather_runs_on_config` opens a fresh reader per call).
    ///
    /// What Drop DOES buy unconditionally is the ORDERING: a controlled exit
    /// announces before it goes, a SIGKILL cannot, so the announcement is never
    /// WRONG — it is only, often, unheard.
    fn drop(&mut self) {
        if let Some(handle) = &self.handle {
            handle.set_ending();
        }
        // Best-effort, like `ScratchGuard`: a run that cannot clean up its own
        // description must not fail on the way out.
        if let Err(e) = std::fs::remove_dir_all(&self.dir) {
            if e.kind() != std::io::ErrorKind::NotFound {
                tracing::debug!(
                    dir = %self.dir.display(),
                    error = %e,
                    "could not remove the run directory (it is inert — nothing \
                     republishes a pointer to it)"
                );
            }
        }
    }
}

/// `~/.cerulion/runs` (honoring `CERULION_HOME`, the established isolation
/// knob — see [`crate::auth::cerulion_config_dir`]).
///
/// The derivation itself lives in
/// [`cerulion_core::transport::run_registry::run_dir_root`] and this DELEGATES to
/// it. A gathered `RunRecord::run_dir` is a path the `runs` verb's serve side has
/// to bound, that serve lives in `cerulion_core`, and core cannot depend on this
/// crate — so one root, defined in the crate the registry lives in, rather than
/// two derivations free to disagree (the duplicated-rule mistake). This function keeps
/// its `CliResult` shape, so every caller here is unchanged.
///
/// # Errors
///
/// [`CliError::Validation`] when neither `CERULION_HOME` nor a home directory
/// resolves.
pub fn run_dir_root() -> CliResult<PathBuf> {
    cerulion_core::transport::run_registry::run_dir_root().ok_or_else(|| {
        CliError::Validation(
            "cannot resolve a run directory — neither CERULION_HOME nor a home \
                 directory is set"
                .to_string(),
        )
    })
}

/// Make `name` safe as ONE path component: everything outside
/// `[A-Za-z0-9._-]` becomes `_`, and a LEADING dot becomes `_` as well.
///
/// A graph name reaches here from a YAML `name:` field, so it is user-supplied
/// text being spliced into a path. Collapsing the character set is a
/// containment rule, not cosmetics: the run id already makes the directory
/// unique, so the name is purely a human label and has nothing to lose by being
/// conservative.
///
/// # Why a leading dot is treated as unsafe
///
/// A dot is harmless in the MIDDLE of a name (`v1.2-rc` is a legitimate graph
/// name and stays intact) but not at the FRONT, where it makes the run
/// directory a DOTFILE: if `...` survived verbatim, `~/.cerulion/runs`
/// would gain a `...-<run_id>` entry that a plain `ls` does not show. An
/// undiscoverable run directory is the condition this module exists to close —
/// an operator who cannot see the directory cannot point `cerulion bag record`
/// at it, and cannot sweep it after a SIGKILLed run leaves it behind.
///
/// **Traversal is NOT the hazard here and never was**, which is why this is a
/// visibility rule rather than a containment one: separators are already
/// transliterated (`../../etc/passwd` cannot keep a `/`), and the caller
/// appends `-{run_id:032x}` unconditionally, so the component can never be
/// exactly `.` or `..` no matter what the name was.
///
/// Leading dots transliterate to `_` rather than falling back to `"graph"`,
/// matching what every other unsafe character already does (`///` → `___`):
/// the `"graph"` fallback is for the case with NOTHING to preserve, and
/// `.hidden` still has a name worth keeping as `_hidden`.
fn sanitize_component(name: &str) -> String {
    let cleaned: String = name
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '.' || c == '-' || c == '_' {
                c
            } else {
                '_'
            }
        })
        .collect();
    if cleaned.is_empty() {
        return "graph".to_string();
    }
    let rest = cleaned.trim_start_matches('.');
    // `.` is one UTF-8 byte, so the byte delta IS the leading-dot count
    // whatever the remainder holds.
    let leading_dots = cleaned.len() - rest.len();
    if leading_dots == 0 {
        cleaned
    } else {
        format!("{}{rest}", "_".repeat(leading_dots))
    }
}

/// Render `run.json`.
///
/// Pure so its exact shape is oracle-testable: this is a format surface a
/// future `cerulion bag record --run` reads, and a silent field rename is a
/// broken reader.
fn render_run_manifest(spec: &RunDescriptorSpec<'_>, run_id: u128, dir: &Path) -> Vec<u8> {
    let mut bytes = serde_json::to_vec_pretty(&serde_json::json!({
        "version": RUN_MANIFEST_VERSION,
        "run_id": format!("0x{run_id:032x}"),
        "graph_name": spec.graph_name,
        // There is no graph_name/graph_file distinction:
        // the FILE STEM is the graph name, so these two keys always carry
        // the SAME value. `graph_file` is KEPT rather than dropped because
        // `run.json` is a documented format surface a reader may already key
        // on, and a silent field removal is a broken reader.
        "graph_file": spec.graph_name,
        "supervisor_pid": std::process::id(),
        "run_started_at_ns": spec.run_started_at_ns,
        "run_dir": dir.display().to_string(),
        "network": spec.network.label(),
        "partition": {
            "provenance": spec.partition.label(),
            "process_groups": spec.process_groups,
        },
        // Written on EVERY run, at the one moment the arm is
        // known. A reader keeps `absent ⇒ unknown` (an older manifest).
        "gating": spec.gating.label(),
        // The SHM tag ledger, in ONE vocabulary with
        // `declare_run_rings`' later write (which PRESERVES what is here).
        "shm": render_shm_entries(&spec.shm),
    }))
    .expect("run.json is a static-shape object; serialization cannot fail");
    bytes.push(b'\n');
    bytes
}

/// The run directory's permission bits: OWNER ONLY (`rwx------`).
///
/// Matching the repo's config-dir rule rather than the process umask, because
/// `env.json` is a snapshot of the process ENVIRONMENT — the same content
/// `--record` deliberately redacts by default (`RecordEnvMode::Allowlist`
/// replaces non-allowlisted values with a hash). A 0755 directory would make
/// every run's environment world-readable on a shared machine, on every run,
/// whether or not anyone asked to record.
#[cfg(unix)]
const RUN_DIR_MODE: u32 = 0o700;

/// The run-artifact permission bits: OWNER ONLY (`rw-------`), for the same
/// reason as [`RUN_DIR_MODE`]. Belt and braces — the 0700 directory already
/// gates access, but a file that leaks out of it (a copy, a backup sweep, a
/// future move) carries its own restriction.
#[cfg(unix)]
const RUN_FILE_MODE: u32 = 0o600;

/// Create the run directory OWNER-ONLY, through the crate's ONE `~/.cerulion`
/// directory seam ([`crate::auth::create_secret_dir`]) rather than a second
/// copy of the same `DirBuilder::mode` call — every other writer under that
/// tree already goes through it.
///
/// The mode is set at CREATE time rather than chmod-ed afterwards, so a
/// FRESHLY-created path is never briefly world-readable.
///
/// Stated because the guarantee is narrower than "0700 always": an
/// ALREADY-EXISTING directory keeps its own mode, and `~/.cerulion/runs` on a
/// machine upgraded from an older build may exist at 0755. Hence
/// [`tighten_existing_run_root`], which normalizes it once per run.
fn create_run_dir(dir: &Path) -> std::io::Result<()> {
    crate::auth::create_secret_dir(dir)
}

/// Bring an ALREADY-EXISTING `~/.cerulion/runs` down to owner-only.
///
/// `create_secret_dir` sets the mode only on directories it CREATES, so a
/// machine that ran an older build (or any other `~/.cerulion` writer that
/// created the tree at the umask default) keeps a 0755 `runs/` forever.
/// MEASURED: the run directories inside are still 0700 and their files 0600, so
/// no CONTENT leaks — what a wider mode exposes is the directory LISTING, i.e.
/// the graph names and run ids of every run on the machine.
///
/// Best-effort and silent-at-debug: a `runs/` this process cannot chmod (a foreign owner)
/// is not a reason to fail a run, and the per-run directory's own 0700 is what
/// actually protects the artifacts.
fn tighten_existing_run_root(root: &Path) {
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        let Ok(meta) = std::fs::metadata(root) else {
            return;
        };
        let mode = meta.permissions().mode() & 0o777;
        if mode == RUN_DIR_MODE {
            return;
        }
        let mut perms = meta.permissions();
        perms.set_mode(RUN_DIR_MODE);
        if let Err(e) = std::fs::set_permissions(root, perms) {
            tracing::debug!(
                root = %root.display(),
                error = %e,
                from = format!("{mode:o}"),
                "could not tighten the run root's mode — each run's own directory \
                 is still 0700, so only the LISTING of run names stays readable"
            );
        }
    }
    #[cfg(not(unix))]
    {
        let _ = root;
    }
}

/// Write one run-directory file OWNER-ONLY, naming the file in any error (a
/// bare io error on a multi-file write says nothing about which write failed).
///
/// The mode rides `OpenOptions` rather than a post-write `set_permissions`, so
/// the bytes are never briefly readable by anyone else.
fn write_artifact(dir: &Path, name: &str, bytes: &[u8]) -> CliResult<()> {
    let path = dir.join(name);
    // create+truncate rather than `auth::write_new_secret_file`'s `create_new`:
    // the enclosing directory was created by THIS call, fresh and 0700, so
    // there is no pre-existing file and no symlink an attacker could have
    // planted — the `O_EXCL` belt those secret files need buys nothing here.
    // (It is deliberately not `create_new`: the partial-write cleanup path may
    // re-enter, and a retry landing on its own leftovers would fail.)
    let mut opts = std::fs::OpenOptions::new();
    opts.write(true).create(true).truncate(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        opts.mode(RUN_FILE_MODE);
    }
    let mut f = opts.open(&path).map_err(|e| {
        CliError::Validation(format!(
            "cannot create `{}` in the run directory: {e}",
            path.display()
        ))
    })?;
    f.write_all(bytes)
        .map_err(|e| CliError::Validation(format!("cannot write `{}`: {e}", path.display())))
}

/// Test-only fault seam: when armed, the LAST artifact write fails, leaving the
/// directory and its first three files behind — the exact partial state
/// [`start_run_descriptor`]'s cleanup exists for, and the one no ordinary input
/// can produce (every artifact is rendered in memory before any write).
#[cfg(test)]
static FAULT_MANIFEST_WRITE: std::sync::atomic::AtomicBool =
    std::sync::atomic::AtomicBool::new(false);

/// Arm [`FAULT_MANIFEST_WRITE`] for the lifetime of the returned guard.
#[cfg(test)]
fn fault_inject_manifest_write() -> impl Drop {
    struct Guard;
    impl Drop for Guard {
        fn drop(&mut self) {
            FAULT_MANIFEST_WRITE.store(false, std::sync::atomic::Ordering::SeqCst);
        }
    }
    FAULT_MANIFEST_WRITE.store(true, std::sync::atomic::Ordering::SeqCst);
    Guard
}

/// Write the four artifacts. Split out so [`start_run_descriptor`] can wrap
/// every one of their failures in ONE cleanup arm.
fn write_run_artifacts(dir: &Path, spec: &RunDescriptorSpec<'_>, run_id: u128) -> CliResult<()> {
    write_artifact(dir, RUN_GRAPH_FILE, spec.graph_yaml.as_bytes())?;
    write_artifact(dir, RUN_ENV_FILE, &spec.env_json)?;
    write_artifact(dir, RUN_RECORDER_FILE, &spec.recorder_json)?;
    #[cfg(test)]
    if FAULT_MANIFEST_WRITE.load(std::sync::atomic::Ordering::SeqCst) {
        return Err(CliError::Validation(format!(
            "cannot write `{}`: injected fault",
            dir.join(RUN_MANIFEST_FILE).display()
        )));
    }
    write_artifact(
        dir,
        RUN_MANIFEST_FILE,
        &render_run_manifest(spec, run_id, dir),
    )
}

/// Best-effort removal of a run directory whose write failed part way.
///
/// Best-effort because the ORIGINAL error is what the caller must see: a
/// cleanup failure on top of a write failure is a second symptom of the same
/// cause (a full disk, a read-only mount), and reporting it instead would hide
/// the diagnosis. It is logged, never returned.
fn remove_partial_run_dir(dir: &Path) {
    if let Err(e) = std::fs::remove_dir_all(dir) {
        if e.kind() != std::io::ErrorKind::NotFound {
            tracing::debug!(
                dir = %dir.display(),
                error = %e,
                "could not remove a partially-written run directory — it is inert \
                 (nothing announces it), but it will not be reaped"
            );
        }
    }
}

/// Create this run's directory and publish its registry record.
///
/// The directory is written FIRST and the record SECOND, deliberately: the
/// record is a pointer, and publishing a pointer to a directory that does not
/// yet exist would let a fast gatherer read a half-written run.
///
/// A registry failure is DEGRADED, not fatal — the directory still lands, so a
/// human can read it, and [`RunDescriptor::is_discoverable`] reports that
/// nothing announces it.
///
/// # Errors
///
/// [`CliError::Validation`] if the run root cannot be resolved or the directory
/// / its files cannot be written. Callers on the run path should use
/// [`begin_run_descriptor`], which never fails a run over its own description.
pub fn start_run_descriptor(spec: RunDescriptorSpec<'_>) -> CliResult<RunDescriptor> {
    // HANDED IN, not minted here — see `RunDescriptorSpec::run_id`. The identity
    // has to outlive a failure of this function, because the capture plane is
    // named from it.
    let run_id = spec.run_id;
    // Resolved ONCE and reused: a second `run_dir_root()` could in principle
    // answer differently (it reads the environment every call), which would
    // tighten one directory while creating the run in another — and its `Err`
    // arm would have to be either swallowed or reported a second time. Binding
    // it makes both questions structurally unreachable rather than handled.
    let root = run_dir_root()?;
    let dir = root.join(format!(
        "{}-{run_id:032x}",
        sanitize_component(spec.graph_name)
    ));
    // An `~/.cerulion/runs` from an older build (or another writer) may exist
    // at the umask default; normalize it before adding this run to it.
    tighten_existing_run_root(&root);
    create_run_dir(&dir).map_err(|e| {
        CliError::Validation(format!(
            "cannot create the run directory `{}`: {e}",
            dir.display()
        ))
    })?;
    // TAKE THE LOCK BEFORE RENDERING THE LEDGER.
    //
    // This ordering is the whole safety argument of the run sweeper, not a
    // detail of it. The sweeper reclaims a directory only when it can SEE a
    // ledger and ACQUIRE every lock, so acquiring first means the two facts can
    // never be observed in the dangerous order: at the instant `run.json`
    // becomes visible, `run.lock` is already held, and a sweeper racing this
    // very function either sees no ledger (and skips) or sees a held lock (and
    // skips). Rendering the manifest first would open a window in which a live
    // run looks exactly like a dead one, and that is the mutation
    // `run_sweep`'s start-race test kills.
    //
    // A lock that cannot be taken FAILS the descriptor rather than degrading to an
    // unlocked directory. An unlocked directory carrying a ledger is precisely
    // the shape the sweeper must never meet — it would be `Unknown` today and
    // leak forever — and `begin_run_descriptor` turns this into one warn and a
    // run that executes normally, so the cost is discoverability, never the run.
    #[cfg(unix)]
    let lock = match crate::run_lock::RunLock::acquire(&dir.join(crate::run_lock::RUN_LOCK_FILE)) {
        Ok(l) => l,
        Err(e) => {
            remove_partial_run_dir(&dir);
            return Err(CliError::Validation(format!(
                "cannot lock the run directory `{}`: {e}",
                dir.display()
            )));
        }
    };
    // Every failure PAST the create must take the directory with it. Nothing
    // else ever will: `begin_run_descriptor` swallows the error, no
    // `RunDescriptor` was built so no `Drop` runs, and there is no reaper — so
    // a partially-written directory would sit in `~/.cerulion/runs` forever,
    // indistinguishable from a SIGKILLed run's leftovers while being neither.
    if let Err(e) = write_run_artifacts(&dir, &spec, run_id) {
        remove_partial_run_dir(&dir);
        return Err(e);
    }

    let handle = match RunHandle::publish(RunRecord {
        run_id,
        supervisor_pid: std::process::id(),
        run_started_at_ns: spec.run_started_at_ns,
        state: RunState::Live,
        graph_name: spec.graph_name.to_string(),
        run_dir: dir.display().to_string(),
    }) {
        Ok(h) => Some(h),
        Err(e) => {
            tracing::warn!(
                error = %e,
                run_dir = %dir.display(),
                "this run's description was written but could NOT be announced on \
                 /__cerulion/runs — the run executes normally, but `cerulion bag record` cannot \
                 discover it and must be pointed at it by hand"
            );
            None
        }
    };
    tracing::debug!(
        run_dir = %dir.display(),
        run_id = %format!("0x{run_id:032x}"),
        discoverable = handle.is_some(),
        "run directory written"
    );
    // Reclaim what earlier, KILLED runs left in shared memory.
    //
    // Here, and not earlier, for two reasons. (1) This run's own `run.lock` is
    // already held, so its directory probes HELD and the sweep is structurally
    // unable to delete the run that is starting — a kernel property rather than
    // a path comparison. (2) Its `run.json` is already written, so this run's
    // own explicit `CERULION_STATE_ARM_TAG`, if it has one, is in the live set
    // that protects a shared tag from being swept out from under it.
    //
    // Bounded and non-blocking (every probe is `LOCK_NB`), and it never fails a
    // run: `sweep_stale_runs` returns a report, not a `Result`.
    #[cfg(unix)]
    {
        let report = crate::run_sweep::sweep_stale_runs(&root);
        if report.reclaimed_anything() {
            tracing::warn!(
                runs = report.runs_reclaimed,
                names = report.names_unlinked,
                bytes = report.bytes_reclaimed,
                refused = report.names_refused,
                "reclaimed shared memory left by earlier runs that were killed \
                 (up to the apparent size quoted — a ring's resident pages are demand-faulted)"
            );
        }
    }
    Ok(RunDescriptor {
        dir,
        run_id,
        handle,
        #[cfg(unix)]
        _lock: lock,
    })
}

/// [`start_run_descriptor`] on the RUN path — every failure becomes
/// ONE loud `warn!` and `None`.
///
/// A robot must not be stopped from running its graph because its home is
/// read-only or its disk is full. What is lost is discoverability, and the warn
/// says exactly that rather than leaving an operator to infer it from a
/// `bag record` that finds nothing.
#[must_use]
pub fn begin_run_descriptor(spec: RunDescriptorSpec<'_>) -> Option<RunDescriptor> {
    let graph = spec.graph_name.to_string();
    match start_run_descriptor(spec) {
        Ok(d) => Some(d),
        Err(e) => {
            tracing::warn!(
                graph = %graph,
                error = %e,
                "could not write this run's description — the graph runs normally, but \
                 the run is UNDISCOVERABLE: `cerulion bag record` will not find it and any bag \
                 recorded of it will carry no graph, env or run identity"
            );
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `CERULION_HOME` is PROCESS-global, so every arm that redirects it takes
    /// the CRATE-WIDE env lock — not a private one.
    ///
    /// A file-local mutex only serialises this module against ITSELF, which is
    /// not the hazard: `auth`, `connect_cmd`, `login_cmd`, `robot_cmd` and
    /// `graph_cmd` all read or write the same process environment, and a
    /// private lock lets one of them observe (or clobber) a redirected
    /// `CERULION_HOME` mid-test. The crate documents `test_env::env_lock` as
    /// mandatory for exactly this reason.
    fn home_lock() -> std::sync::MutexGuard<'static, ()> {
        crate::test_env::env_lock()
    }

    /// RAII redirect of `CERULION_HOME`, restored on drop (panic included).
    struct HomeGuard(Option<std::ffi::OsString>);

    impl HomeGuard {
        fn set(path: &Path) -> Self {
            let prev = std::env::var_os("CERULION_HOME");
            std::env::set_var("CERULION_HOME", path);
            Self(prev)
        }
    }

    impl Drop for HomeGuard {
        fn drop(&mut self) {
            match self.0.take() {
                Some(v) => std::env::set_var("CERULION_HOME", v),
                None => std::env::remove_var("CERULION_HOME"),
            }
        }
    }

    fn spec<'a>(graph_yaml: &'a str, groups: bool) -> RunDescriptorSpec<'a> {
        RunDescriptorSpec {
            run_id: mint_run_id(),
            graph_name: "go2 attach",
            run_started_at_ns: 1_753_000_000_000_000_000,
            network: NetworkPostureLabel::Permissive,
            partition: PartitionProvenance::DerivedInMemory,
            process_groups: groups,
            shm: Vec::new(),
            // A supervisor run hands its workers a quantum; a monolith does
            // not. The helper mirrors the production classification so the
            // fixture is a run shape rather than an arbitrary pairing.
            gating: GatingClock::classify(
                groups,
                false,
                false,
                crate::multiprocess::ExecutionMode::Lockstep,
            ),
            graph_yaml: graph_yaml.to_string(),
            env_json: br#"{"PATH":"/usr/bin"}"#.to_vec(),
            recorder_json: br#"{"arch":"aarch64"}"#.to_vec(),
        }
    }

    /// THE run-directory oracle: the four artifacts land, each byte-identical
    /// to what the caller rendered, and `run.json` reads back as a hand-written
    /// object.
    ///
    /// The manifest is a FORMAT surface a future `bag record --run` parses, so
    /// it is asserted field by field rather than by round-tripping it through
    /// its own writer.
    #[test]
    fn a_run_directory_carries_its_artifacts_and_a_manifest_that_reads_back() {
        let _lock = home_lock();
        let tmp = tempfile::tempdir().expect("tempdir");
        let _home = HomeGuard::set(tmp.path());

        let descriptor =
            start_run_descriptor(spec("name: solo\nprefix: p\n", true)).expect("run dir");
        let dir = descriptor.path().to_path_buf();

        // The directory lives under `~/.cerulion/runs` and its name is the
        // SANITIZED graph name plus the run id (a space must never reach a path).
        assert_eq!(dir.parent(), Some(tmp.path().join("runs").as_path()));
        let name = dir.file_name().expect("name").to_string_lossy().to_string();
        assert_eq!(
            name,
            format!("go2_attach-{:032x}", descriptor.run_id()),
            "the directory is <sanitized graph name>-<run id>"
        );

        assert_eq!(
            std::fs::read_to_string(dir.join(RUN_GRAPH_FILE)).expect("graph.yaml"),
            "name: solo\nprefix: p\n",
            "graph.yaml is the caller's rendered EFFECTIVE config, verbatim"
        );
        assert_eq!(
            std::fs::read(dir.join(RUN_ENV_FILE)).expect("env.json"),
            br#"{"PATH":"/usr/bin"}"#.to_vec()
        );
        assert_eq!(
            std::fs::read(dir.join(RUN_RECORDER_FILE)).expect("recorder.json"),
            br#"{"arch":"aarch64"}"#.to_vec()
        );

        let manifest: serde_json::Value =
            serde_json::from_slice(&std::fs::read(dir.join(RUN_MANIFEST_FILE)).expect("run.json"))
                .expect("run.json parses");
        assert_eq!(manifest["version"], serde_json::json!(RUN_MANIFEST_VERSION));
        assert_eq!(
            manifest["run_id"],
            serde_json::json!(format!("0x{:032x}", descriptor.run_id())),
            "the manifest's id must be THIS run's id, not a second mint"
        );
        assert_eq!(manifest["graph_name"], serde_json::json!("go2 attach"));
        // There is no logical-name/loaded-file split, so the
        // two keys carry the SAME value. `graph_file` is KEPT rather than
        // dropped because `run.json` is a documented format surface a reader
        // may already key on, and a silent field removal is a broken reader —
        // asserted as an EQUALITY against `graph_name` rather than against a
        // literal, so the two can never drift apart.
        assert_eq!(
            manifest["graph_file"], manifest["graph_name"],
            "one name: `graph_file` must mirror `graph_name`"
        );
        assert_eq!(manifest["graph_file"], serde_json::json!("go2 attach"));
        assert_eq!(
            manifest["supervisor_pid"],
            serde_json::json!(std::process::id())
        );
        assert_eq!(
            manifest["run_started_at_ns"],
            serde_json::json!(1_753_000_000_000_000_000u64)
        );
        assert_eq!(
            manifest["run_dir"],
            serde_json::json!(dir.display().to_string()),
            "the manifest names its own directory, so a reader handed only the \
             file can still resolve the run"
        );
        assert_eq!(manifest["network"], serde_json::json!("permissive"));
        assert_eq!(
            manifest["partition"],
            serde_json::json!({"provenance": "derived-in-memory", "process_groups": true})
        );
    }

    /// The directory is REMOVED when the descriptor drops — every exit path,
    /// the `ScratchGuard` shape.
    #[test]
    fn dropping_the_descriptor_removes_the_run_directory() {
        let _lock = home_lock();
        let tmp = tempfile::tempdir().expect("tempdir");
        let _home = HomeGuard::set(tmp.path());

        let descriptor = start_run_descriptor(spec("name: solo\n", false)).expect("run dir");
        let dir = descriptor.path().to_path_buf();
        assert!(dir.join(RUN_MANIFEST_FILE).exists());
        drop(descriptor);
        assert!(
            !dir.exists(),
            "a run's description must not outlive the run — a stale directory with \
             no registry writer is exactly what a crash leaves behind, and one \
             left by a CLEAN exit would be indistinguishable from it"
        );
    }

    /// THE ALWAYS-ON pin: a run's directory is ANNOUNCED, and the announcement
    /// points at that exact directory.
    ///
    /// This is what makes the run dir more than a file nobody can find: an
    /// attaching recorder learns a run exists only from `/__cerulion/runs`. The
    /// gather runs on the PROCESS-GLOBAL namespace — the one production
    /// publishes on, so this arm also proves the production entry point
    /// (`RunHandle::publish`) is wired, not just `publish_on_config` — and is
    /// filtered by THIS run's id, so a co-tenant run on the developer's desk
    /// cannot make it pass or fail.
    #[test]
    fn a_run_is_announced_on_the_registry_pointing_at_its_own_directory() {
        let _lock = home_lock();
        let tmp = tempfile::tempdir().expect("tempdir");
        let _home = HomeGuard::set(tmp.path());

        let descriptor = start_run_descriptor(spec("name: solo\n", false)).expect("run dir");
        assert!(
            descriptor.is_discoverable(),
            "the registry writer must be live — without it the directory exists \
             but nothing can find it"
        );

        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
        loop {
            let gather =
                cerulion_core::transport::run_registry::gather_current_runs().expect("gather runs");
            if let Some(mine) = gather
                .records
                .iter()
                .find(|r| r.run_id == descriptor.run_id())
            {
                assert_eq!(
                    mine.run_dir,
                    descriptor.path().display().to_string(),
                    "the announcement must point at THIS run's directory"
                );
                assert_eq!(mine.graph_name, "go2 attach");
                assert_eq!(mine.supervisor_pid, std::process::id());
                assert_eq!(mine.state, RunState::Live);
                return;
            }
            assert!(
                std::time::Instant::now() < deadline,
                "this run was never announced on /__cerulion/runs within 10s; the \
                 gather saw {} other run(s)",
                gather.records.len()
            );
        }
    }

    /// The RUN-PATH entry point itself does the work, on inputs that name
    /// no recording at all.
    ///
    /// `begin_run_descriptor` has exactly ONE caller, so a `record.is_some()`
    /// guard (or an env gate) added INSIDE it would ship the run directory inert with
    /// every other arm green — the sibling arms all drive
    /// `start_run_descriptor` and would not notice. This one asserts the whole
    /// contract through the entry `graph_run` actually calls: a descriptor is
    /// built, the directory carries its four artifacts, and the run is
    /// ANNOUNCED.
    #[test]
    fn the_run_path_entry_point_writes_and_announces_with_no_recording_in_sight() {
        let _lock = home_lock();
        let tmp = tempfile::tempdir().expect("tempdir");
        let _home = HomeGuard::set(tmp.path());

        let descriptor = begin_run_descriptor(spec("name: plain\n", false))
            .expect("a plain run must still be described");
        let dir = descriptor.path().to_path_buf();
        for name in [
            RUN_MANIFEST_FILE,
            RUN_GRAPH_FILE,
            RUN_ENV_FILE,
            RUN_RECORDER_FILE,
        ] {
            assert!(dir.join(name).exists(), "`{name}` must be written");
        }
        assert!(
            descriptor.is_discoverable(),
            "a run nobody asked to record is still announced — that is the whole \
             point of writing the description on EVERY run"
        );

        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
        loop {
            let gather =
                cerulion_core::transport::run_registry::gather_current_runs().expect("gather");
            if gather
                .records
                .iter()
                .any(|r| r.run_id == descriptor.run_id())
            {
                return;
            }
            assert!(
                std::time::Instant::now() < deadline,
                "a run reached through `begin_run_descriptor` must be discoverable"
            );
        }
    }

    /// The run directory and its artifacts are OWNER-ONLY.
    ///
    /// `env.json` is a snapshot of the process environment — the content
    /// `--record` redacts by default — so a world-readable file here would leak
    /// on EVERY run of every graph, recorded or not, on any shared machine.
    /// Asserted on the real mode bits rather than on the umask, because the
    /// umask is the operator's and this guarantee is not.
    #[cfg(unix)]
    #[test]
    fn the_run_directory_and_its_artifacts_are_owner_only() {
        use std::os::unix::fs::PermissionsExt;

        let _lock = home_lock();
        let tmp = tempfile::tempdir().expect("tempdir");
        let _home = HomeGuard::set(tmp.path());

        let descriptor = start_run_descriptor(spec("name: solo\n", false)).expect("run dir");
        let dir = descriptor.path();

        let dir_mode = std::fs::metadata(dir)
            .expect("stat dir")
            .permissions()
            .mode()
            & 0o777;
        assert_eq!(
            dir_mode, RUN_DIR_MODE,
            "the run directory must be rwx------, not the umask's default"
        );
        for name in [
            RUN_MANIFEST_FILE,
            RUN_GRAPH_FILE,
            RUN_ENV_FILE,
            RUN_RECORDER_FILE,
        ] {
            let mode = std::fs::metadata(dir.join(name))
                .expect("stat artifact")
                .permissions()
                .mode()
                & 0o777;
            assert_eq!(
                mode, RUN_FILE_MODE,
                "`{name}` must be rw------- — it is inside a 0700 directory, but a file \
                 that leaves it (a copy, a backup sweep) must carry its own restriction"
            );
        }
    }

    /// A failure PART WAY through writing takes the directory with it.
    ///
    /// Nothing else ever would: `begin_run_descriptor` swallows the error, no
    /// `RunDescriptor` exists so no `Drop` runs, and there is no reaper — so
    /// the leftovers would be indistinguishable from a SIGKILLed run's while
    /// being neither.
    ///
    /// This arm covers the EARLIEST failure — a `runs/` root that cannot hold a
    /// directory at all, so `create_run_dir` itself fails and the property is
    /// that nothing was left behind on the way out. The LATER, genuinely
    /// partial state (directory created, three artifacts written, the fourth
    /// failing) is the sibling arm
    /// `a_failure_after_the_directory_exists_still_removes_it`, which reaches
    /// it through the `fault_inject_manifest_write` seam.
    ///
    /// The poison is a `runs` path that is a FILE. A read-only MODE would not
    /// do: `tighten_existing_run_root` normalizes an existing root's mode
    /// before every run, so it would REPAIR the poison and the arm would test
    /// nothing (a 0500 chmod is repaired the same way).
    #[test]
    fn a_write_that_fails_part_way_leaves_no_partial_run_directory() {
        let _lock = home_lock();
        let tmp = tempfile::tempdir().expect("tempdir");
        let _home = HomeGuard::set(tmp.path());

        // The run directory's name embeds a run id minted INSIDE the call, so it
        // cannot be poisoned by name from out here. Poison the ROOT instead.
        let runs_root = run_dir_root().expect("root");
        std::fs::write(&runs_root, b"not a directory").expect("plant the poison");

        let before: Vec<_> = std::fs::read_dir(tmp.path())
            .expect("read home")
            .filter_map(Result::ok)
            .map(|e| e.file_name())
            .collect();
        assert!(
            start_run_descriptor(spec("name: solo\n", false)).is_err(),
            "a runs root that cannot hold a directory must fail loudly"
        );
        let after: Vec<_> = std::fs::read_dir(tmp.path())
            .expect("read home")
            .filter_map(Result::ok)
            .map(|e| e.file_name())
            .collect();
        assert_eq!(before, after, "a failed run must leave NOTHING behind");
        assert!(
            runs_root.is_file(),
            "and must not have replaced the poisoned path"
        );
    }

    /// The arm the read-only-root case cannot reach: the directory IS
    /// created and a LATER artifact write fails.
    ///
    /// Injected at the LAST artifact (`fault_inject_manifest_write`), so three
    /// files land and the fourth fails — the run id is minted inside the call,
    /// so the directory's name cannot be poisoned from outside, and every
    /// artifact is rendered in memory before any write, so no ordinary input
    /// reaches this state either.
    #[test]
    fn a_failure_after_the_directory_exists_still_removes_it() {
        let _lock = home_lock();
        let tmp = tempfile::tempdir().expect("tempdir");
        let _home = HomeGuard::set(tmp.path());

        let poison = fault_inject_manifest_write();
        let err = match start_run_descriptor(spec("name: solo\n", false)) {
            Err(e) => e,
            Ok(_) => panic!("the poisoned manifest write must fail"),
        };
        drop(poison);
        assert!(
            format!("{err}").contains(RUN_MANIFEST_FILE),
            "the error must name the artifact that failed; got: {err}"
        );

        let runs_root = run_dir_root().expect("root");
        let leftovers: Vec<_> = std::fs::read_dir(&runs_root)
            .map(|d| d.filter_map(Result::ok).map(|e| e.file_name()).collect())
            .unwrap_or_default();
        assert!(
            leftovers.is_empty(),
            "the partially-written run directory must be removed; found {leftovers:?}"
        );
    }

    /// A run with NO home to write to still RUNS: the error is returned to
    /// `begin_run_descriptor`, which is the arm that keeps a robot working on a
    /// read-only filesystem.
    #[test]
    fn a_run_that_cannot_write_its_description_degrades_rather_than_failing() {
        let _lock = home_lock();
        let tmp = tempfile::tempdir().expect("tempdir");
        // A FILE where the run root's parent must be a directory: every
        // `create_dir_all` under it fails, on every platform.
        let blocker = tmp.path().join("not_a_dir");
        std::fs::write(&blocker, b"x").expect("write blocker");
        let _home = HomeGuard::set(&blocker);

        assert!(
            start_run_descriptor(spec("name: solo\n", false)).is_err(),
            "the strict entry point reports the failure"
        );
        assert!(
            begin_run_descriptor(spec("name: solo\n", false)).is_none(),
            "and the RUN entry point degrades to None instead of failing the run"
        );
    }

    /// A run DECLARES the rings it created,
    /// and the recorder's own reader finds them.
    ///
    /// The trace-attach path goes inert without this: `render_run_manifest` is a fixed
    /// nine-key object with no `rings`, `RunArtifacts::rings` reads exclusively
    /// from that key, so `config.rings` would be empty on EVERY path and no
    /// bag could ever take the `TRACE_FROM_ATTACH` arm.
    ///
    /// The round trip is asserted through `bag_cmd::read_run_artifacts`, the
    /// PRODUCTION reader, rather than by re-parsing the JSON here: the two halves
    /// agreeing is the whole property, and a test that parsed the file itself
    /// would pass against a writer whose shape that reader cannot use.
    ///
    /// `#[cfg(unix)]` because that production reader is: `bag_cmd` is
    /// `#[cfg(unix)]` at `lib.rs` (it reads bags via `cerulion_bag`, itself
    /// `#![cfg(unix)]`), while THIS module is declared unconditionally — so the
    /// reference crosses a gate the rest of the file does not.
    /// `declare_run_rings` MERGES the `shm` ledger — it owns the
    /// TRACE and DEPARTURE classes and must not delete anybody else's.
    ///
    /// The ARM word is declared EARLIER, at descriptor write, because its tag is
    /// known before anything is created and a run that crashes during bring-up
    /// must still be sweepable. A wholesale `insert` of this writer's vector
    /// destroyed that entry — and destroying it does not merely lose a line: it
    /// makes the ledger claim the arm class is UNDECLARED, which `ShmDecl`'s own
    /// contract defines as "nothing can be concluded", so a sweeper reads it and
    /// skips exactly the objects the run really did create.
    #[test]
    fn declaring_rings_preserves_ledger_entries_of_classes_it_does_not_own() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let arm = ShmDecl {
            tag: "cer_run_002a".to_string(),
            class: ShmClass::Arm,
            rank: None,
            source: TagSource::Explicit,
        };
        let spec = RunDescriptorSpec {
            run_id: mint_run_id(),
            graph_name: "merge",
            run_started_at_ns: 7,
            network: NetworkPostureLabel::Off,
            partition: PartitionProvenance::Declared,
            process_groups: false,
            shm: vec![arm.clone()],
            gating: GatingClock::Wall,
            graph_yaml: "nodes: {}\n".to_string(),
            env_json: b"{}".to_vec(),
            recorder_json: b"{}".to_vec(),
        };
        let bytes = render_run_manifest(&spec, spec.run_id, tmp.path());
        std::fs::write(tmp.path().join(RUN_MANIFEST_FILE), &bytes).expect("seed");
        assert_eq!(
            run_manifest_shm(&bytes),
            vec![arm.clone()],
            "the descriptor write must declare the arm entry in the first place"
        );

        declare_run_rings(
            tmp.path(),
            &[RingDecl {
                tag: "cer_rec_merge_991_r0".to_string(),
                rank: 0,
            }],
        )
        .expect("declare");

        let after = std::fs::read(tmp.path().join(RUN_MANIFEST_FILE)).expect("re-read");
        let ledger = run_manifest_shm(&after);
        assert!(
            ledger.contains(&arm),
            "the ARM entry must SURVIVE a trace-ring declaration — it is a class \
             this writer does not own. Got: {ledger:?}"
        );
        assert!(
            ledger.iter().any(|d| d.class == ShmClass::Trace),
            "…and the trace ring must be declared alongside it"
        );
        assert_eq!(
            ledger.len(),
            2,
            "exactly the two, no duplication: {ledger:?}"
        );

        // A SECOND declaration must not accumulate: this writer REPLACES its own
        // classes while still preserving the arm.
        declare_run_rings(
            tmp.path(),
            &[RingDecl {
                tag: "cer_rec_merge_991_r1".to_string(),
                rank: 1,
            }],
        )
        .expect("re-declare");
        let again = run_manifest_shm(&std::fs::read(tmp.path().join(RUN_MANIFEST_FILE)).unwrap());
        assert!(again.contains(&arm));
        assert_eq!(
            again.iter().filter(|d| d.class == ShmClass::Trace).count(),
            1,
            "a re-declaration replaces this writer's own classes: {again:?}"
        );
    }

    #[cfg(unix)]
    #[test]
    fn a_declared_ring_reaches_the_recorders_own_reader_and_the_manifest_survives() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let spec = RunDescriptorSpec {
            run_id: mint_run_id(),
            graph_name: "percept",
            run_started_at_ns: 7,
            network: NetworkPostureLabel::Off,
            partition: PartitionProvenance::Declared,
            process_groups: false,
            shm: Vec::new(),
            gating: GatingClock::Wall,
            graph_yaml: "name: percept\n".to_string(),
            env_json: b"{}".to_vec(),
            recorder_json: b"{}".to_vec(),
        };
        std::fs::write(
            tmp.path().join(RUN_MANIFEST_FILE),
            render_run_manifest(&spec, 0xabc, tmp.path()),
        )
        .expect("seed the manifest");

        // A run with no rings declares none — the ordinary `graph run`, and the
        // ANTI-TAUTOLOGY half: without it, "the reader finds two rings" is
        // satisfied by a reader that invents them.
        assert!(
            crate::bag_cmd::read_run_artifacts(tmp.path())
                .rings
                .is_empty(),
            "a manifest with no `rings` key declares no rings"
        );

        declare_run_rings(
            tmp.path(),
            &[
                RingDecl {
                    tag: "cer_rec_percept_9_r0".to_string(),
                    rank: 0,
                },
                RingDecl {
                    tag: "cer_rec_percept_9_dep".to_string(),
                    rank: u32::MAX,
                },
            ],
        )
        .expect("declaring rings must succeed on a manifest this process wrote");

        let got = crate::bag_cmd::read_run_artifacts(tmp.path());
        assert_eq!(
            got.rings,
            vec![
                "cer_rec_percept_9_r0".to_string(),
                "cer_rec_percept_9_dep".to_string()
            ],
            "the recorder must read back exactly the tags the run declared, in \
             declaration order"
        );

        // The REWRITE is additive: every key the first write put there survives,
        // so declaring rings cannot cost a reader the run id or the partition
        // provenance. `run_id` is the field the whole attachment exists for.
        let doc: serde_json::Value = serde_json::from_slice(
            &std::fs::read(tmp.path().join(RUN_MANIFEST_FILE)).expect("read"),
        )
        .expect("still valid JSON");
        assert_eq!(doc["run_id"], "0x00000000000000000000000000000abc");
        assert_eq!(doc["graph_name"], "percept");
        assert_eq!(doc["partition"]["provenance"], "declared");
        assert_eq!(doc["rings"][1]["rank"], serde_json::json!(u32::MAX));

        // And the bits stay OWNER-ONLY: a rewrite must not widen what the first
        // write deliberately narrowed (`env.json`'s sibling rule).
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            let mode = std::fs::metadata(tmp.path().join(RUN_MANIFEST_FILE))
                .expect("stat")
                .permissions()
                .mode()
                & 0o777;
            assert_eq!(mode, RUN_FILE_MODE, "the rewrite must stay 0600");
        }
    }

    // =======================================================================
    // The run.json TRACE vocabulary
    // =======================================================================

    /// The four label sets are a `run.json` FORMAT surface, so each is pinned
    /// against a HAND-WRITTEN string and each parses back to what it came from.
    ///
    /// Round-tripping ALONE would be satisfied by a writer and a reader that
    /// agree on the wrong spelling — which is exactly the failure mode a format
    /// surface has — so every arm is asserted against a literal FIRST.
    #[test]
    fn the_trace_vocabulary_labels_are_pinned_and_round_trip() {
        // TraceRingsDecl — a state, and for two arms a reason.
        let cases = [
            (TraceRingsDecl::Declared, "declared"),
            (
                TraceRingsDecl::Declined {
                    reason: "--no-rings".to_string(),
                },
                "declined: --no-rings",
            ),
            (
                TraceRingsDecl::Unavailable {
                    reason: "/dev/shm free 12 MiB < 40 MiB".to_string(),
                },
                "unavailable: /dev/shm free 12 MiB < 40 MiB",
            ),
        ];
        for (decl, literal) in &cases {
            assert_eq!(&decl.label(), literal, "the wire spelling is pinned");
            assert_eq!(
                TraceRingsDecl::parse(literal).as_ref(),
                Some(decl),
                "and parses back to the state it came from"
            );
        }

        for (class, literal) in [
            (ShmClass::Trace, "trace"),
            (ShmClass::Departure, "departure"),
            (ShmClass::State, "state"),
            (ShmClass::Arm, "arm"),
        ] {
            assert_eq!(class.label(), literal);
            assert_eq!(ShmClass::parse(literal), Some(class));
        }
        for (src, literal) in [
            (TagSource::Derived, "derived"),
            (TagSource::Explicit, "explicit"),
        ] {
            assert_eq!(src.label(), literal);
            assert_eq!(TagSource::parse(literal), Some(src));
        }
        for (gate, literal) in [
            (GatingClock::Quantum, "quantum"),
            (GatingClock::RecordedWall, "recorded_wall"),
            (GatingClock::Wall, "wall"),
            (GatingClock::Polled, "polled"),
        ] {
            assert_eq!(gate.label(), literal);
            assert_eq!(GatingClock::parse(literal), Some(gate));
        }

        // UNRECOGNISED is `None` on every one of them, never a defaulted state:
        // a reader keeps `absent ⇒ unknown`, and silently mapping a spelling
        // from a newer writer onto one of ours would manufacture a claim.
        for unknown in [
            "",
            "  ",
            "DECLARED",
            "declined",
            "none",
            "wall_clock",
            "traces",
        ] {
            assert_eq!(TraceRingsDecl::parse(unknown), None, "{unknown:?}");
            assert_eq!(ShmClass::parse(unknown), None, "{unknown:?}");
            assert_eq!(TagSource::parse(unknown), None, "{unknown:?}");
            assert_eq!(GatingClock::parse(unknown), None, "{unknown:?}");
        }

        // A recognised STATE with an empty reason is still recognised — the
        // state is the load-bearing half, and a run that could not say why still
        // said WHAT.
        assert_eq!(
            TraceRingsDecl::parse("declined:"),
            Some(TraceRingsDecl::Declined {
                reason: String::new()
            })
        );
    }

    /// `GatingClock::classify` against a HAND-WRITTEN table of the run shapes.
    ///
    /// The classification is the fact a resim judge cannot recover from a bag
    /// (two clock arms produce byte-identical boundary records), so it is worth
    /// enumerating rather than trusting to read.
    #[test]
    fn gating_classification_covers_every_run_shape() {
        use crate::multiprocess::ExecutionMode::{FreeRun, Lockstep};
        // (supervisor, virtual, records, execution_mode) -> arm
        let table = [
            // A LOCKSTEP supervisor run hands every worker a quantum, whatever
            // else is true — including under `--record`, which is the
            // multi-process recording shape.
            ((true, false, false, Lockstep), GatingClock::Quantum),
            ((true, false, true, Lockstep), GatingClock::Quantum),
            // A FREE-RUN supervisor run puts every rank on its own
            // wall-faithful clock — recording ranks follow the wall on a
            // controlled clock from a shared epoch, live ranks read the
            // RealClock. A classifier that took no mode would label BOTH `quantum`
            // while the bag said `free_run`.
            ((true, false, true, FreeRun), GatingClock::RecordedWall),
            ((true, false, false, FreeRun), GatingClock::Wall),
            // A virtual monolith never enters the live loop at all.
            ((false, true, false, Lockstep), GatingClock::Polled),
            // A recording monolith runs the controlled-clock-follows-wall
            // discipline: still recorded, still re-advanceable.
            ((false, false, true, Lockstep), GatingClock::RecordedWall),
            // Everything else is the read-only arm — the plain `graph run
            // --single-process`, `ros2 attach` and `node run` shapes, and the
            // external clock, which the scheduler does not advance either.
            ((false, false, false, Lockstep), GatingClock::Wall),
            // The mode is a SUPERVISOR fact: `resolve_run_execution_mode` never
            // yields FreeRun on a monolith, and the classifier ignores it there
            // rather than inventing a fifth shape.
            ((false, false, true, FreeRun), GatingClock::RecordedWall),
            ((false, false, false, FreeRun), GatingClock::Wall),
        ];
        for ((sup, virt, rec, mode), want) in table {
            assert_eq!(
                GatingClock::classify(sup, virt, rec, mode),
                want,
                "supervisor={sup} virtual={virt} records={rec} mode={mode:?}"
            );
        }
        // The supervisor arm OUTRANKS both others — a partitioned LOCKSTEP
        // recording run is a quantum run, never `recorded_wall`. Asserted
        // separately because it is the one precedence a reordering would
        // silently invert.
        assert_eq!(
            GatingClock::classify(true, true, true, Lockstep),
            GatingClock::Quantum
        );
    }

    /// `RunTraceDecl::declared` classes each ring by its RANK, so the one place
    /// that knows what a departure ring is stays the one place that knows.
    #[test]
    fn the_declared_shortcut_classes_the_departure_ring_by_its_rank() {
        let decl = RunTraceDecl::declared(vec![
            RingDecl {
                tag: "cer_rec_g_7_r0".to_string(),
                rank: 0,
            },
            RingDecl {
                tag: "cer_rec_g_7_r1".to_string(),
                rank: 1,
            },
            RingDecl {
                tag: "cer_rec_g_7_dep".to_string(),
                rank: cerulion_core::trace_ring::DEPARTURE_RING_RANK,
            },
        ]);
        assert_eq!(decl.state, TraceRingsDecl::Declared);
        assert!(decl.declared_unavailable.is_empty());
        assert_eq!(
            decl.shm
                .iter()
                .map(|d| (d.tag.as_str(), d.class, d.rank, d.source))
                .collect::<Vec<_>>(),
            vec![
                (
                    "cer_rec_g_7_r0",
                    ShmClass::Trace,
                    Some(0),
                    TagSource::Derived
                ),
                (
                    "cer_rec_g_7_r1",
                    ShmClass::Trace,
                    Some(1),
                    TagSource::Derived
                ),
                (
                    "cer_rec_g_7_dep",
                    ShmClass::Departure,
                    Some(cerulion_core::trace_ring::DEPARTURE_RING_RANK),
                    TagSource::Derived
                ),
            ],
            "rank u32::MAX is the DEPARTURE ring; every other rank is a worker's trace ring"
        );
    }

    /// `declare_run_trace` writes all four keys in ONE rewrite, and a reader
    /// reads back exactly what was written — including the two degraded states,
    /// which is the whole reason the key exists.
    #[test]
    fn declaring_a_declined_run_writes_a_state_a_reader_can_act_on() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let spec = RunDescriptorSpec {
            run_id: mint_run_id(),
            graph_name: "percept",
            run_started_at_ns: 7,
            network: NetworkPostureLabel::Off,
            partition: PartitionProvenance::Declared,
            process_groups: false,
            shm: Vec::new(),
            gating: GatingClock::Wall,
            graph_yaml: "prefix: /p\n".to_string(),
            env_json: b"{}".to_vec(),
            recorder_json: b"{}".to_vec(),
        };
        let manifest = render_run_manifest(&spec, 0xabc, tmp.path());
        std::fs::write(tmp.path().join(RUN_MANIFEST_FILE), &manifest).expect("seed");

        // ANTI-TAUTOLOGY: before the declaration the run says NOTHING about its
        // trace rings, so every `Some` below is a real read rather than a
        // reader that answers the same way regardless.
        assert_eq!(
            run_manifest_trace_rings(&manifest),
            TraceRingsReport::Absent
        );
        assert!(run_manifest_declared_unavailable(&manifest).is_empty());
        assert!(run_manifest_shm(&manifest).is_empty());
        // …but `gating` IS written by the first write, on every run.
        assert_eq!(run_manifest_gating(&manifest), Some(GatingClock::Wall));

        declare_run_trace(
            tmp.path(),
            &RunTraceDecl {
                rings: Vec::new(),
                state: TraceRingsDecl::Declined {
                    reason: "the operator declined them at launch".to_string(),
                },
                declared_unavailable: Vec::new(),
                shm: Vec::new(),
            },
        )
        .expect("declaring must succeed on a manifest this process wrote");

        let back = std::fs::read(tmp.path().join(RUN_MANIFEST_FILE)).expect("read");
        assert_eq!(
            run_manifest_trace_rings(&back),
            TraceRingsReport::Known(TraceRingsDecl::Declined {
                reason: "the operator declined them at launch".to_string()
            }),
            "a DECLINED run must be distinguishable from one that simply has no rings"
        );
        // The rewrite is additive: the first write's keys all survive.
        let doc: serde_json::Value = serde_json::from_slice(&back).expect("valid JSON");
        assert_eq!(doc["run_id"], "0x00000000000000000000000000000abc");
        assert_eq!(doc["gating"], "wall");
        assert_eq!(doc["rings"], serde_json::json!([]));
        assert_eq!(doc["declared_unavailable"], serde_json::json!([]));
    }

    /// A run that declared rings and lost one to a failed rank: the ledger, the
    /// per-rank failure and the ring list are all readable, and the ring list
    /// still OVER-declares (tags are stamped before creation).
    #[test]
    fn a_rank_that_could_not_create_its_ring_is_recorded_by_number() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let spec = RunDescriptorSpec {
            run_id: mint_run_id(),
            graph_name: "percept",
            run_started_at_ns: 7,
            network: NetworkPostureLabel::Off,
            partition: PartitionProvenance::Declared,
            process_groups: true,
            shm: Vec::new(),
            gating: GatingClock::Quantum,
            graph_yaml: "prefix: /p\n".to_string(),
            env_json: b"{}".to_vec(),
            recorder_json: b"{}".to_vec(),
        };
        std::fs::write(
            tmp.path().join(RUN_MANIFEST_FILE),
            render_run_manifest(&spec, 0xfeed, tmp.path()),
        )
        .expect("seed");

        let rings = vec![
            RingDecl {
                tag: "cer_rec_percept_9_r0".to_string(),
                rank: 0,
            },
            RingDecl {
                tag: "cer_rec_percept_9_r1".to_string(),
                rank: 1,
            },
        ];
        let mut decl = RunTraceDecl::declared(rings);
        decl.declared_unavailable = vec![RankUnavailable {
            rank: 1,
            reason: "shm_open: No space left on device".to_string(),
        }];
        decl.shm.push(ShmDecl {
            tag: "cer_run_000000000000000000000000000feed".to_string(),
            class: ShmClass::Arm,
            rank: None,
            source: TagSource::Derived,
        });
        declare_run_trace(tmp.path(), &decl).expect("declare");

        let back = std::fs::read(tmp.path().join(RUN_MANIFEST_FILE)).expect("read");
        assert_eq!(
            run_manifest_trace_rings(&back),
            TraceRingsReport::Known(TraceRingsDecl::Declared)
        );
        assert_eq!(
            run_manifest_declared_unavailable(&back),
            vec![RankUnavailable {
                rank: 1,
                reason: "shm_open: No space left on device".to_string()
            }],
            "the FAILED rank is named by number, with the reason the creator gave"
        );
        assert_eq!(
            crate::bag_cmd::read_run_artifacts(tmp.path()).rings.len(),
            2,
            "the ring list still names rank 1 — tags are stamped BEFORE creation, which is \
             exactly why the failure needs its own key"
        );
        let ledger = run_manifest_shm(&back);
        assert_eq!(
            ledger.iter().map(|d| (d.class, d.rank)).collect::<Vec<_>>(),
            vec![
                (ShmClass::Trace, Some(0)),
                (ShmClass::Trace, Some(1)),
                (ShmClass::Arm, None),
            ],
            "the sweep ledger carries every declared object with its class, and the arm word \
             has no rank"
        );
        assert_eq!(run_manifest_gating(&back), Some(GatingClock::Quantum));
    }

    /// The parsers are TOLERANT in one direction and STRICT in the other, and
    /// which is which is the point.
    ///
    /// Tolerant: unparseable bytes, an absent key, a non-array, a value shape
    /// this reader does not know — all yield "nothing", because the manifest is
    /// written by another process and possibly another version and a recording
    /// must never be refused over a field a reader can simply not use.
    ///
    /// Strict: a ledger entry it cannot FULLY understand is DROPPED, because
    /// that vector feeds a sweeper that unlinks names. And a per-rank failure
    /// with no usable `rank` is dropped rather than defaulted — rank 0 is a real
    /// worker, so a defaulted rank would accuse an innocent one.
    #[test]
    fn the_manifest_parsers_are_tolerant_of_shape_and_strict_about_meaning() {
        for junk in [
            &b"not json"[..],
            b"[1,2]",
            b"{}",
            br#"{"trace_rings": 7, "declared_unavailable": {}, "shm": "x", "gating": null}"#,
        ] {
            // ABSENT, not `Unrecognised`: an unparseable document and a
            // non-string value carry no token a human could act on, and this
            // arm's caller has already refused to make a claim over an
            // unparseable manifest (`manifest_parsed`).
            assert_eq!(
                run_manifest_trace_rings(junk),
                TraceRingsReport::Absent,
                "{junk:?}"
            );
            assert!(
                run_manifest_declared_unavailable(junk).is_empty(),
                "{junk:?}"
            );
            assert!(run_manifest_shm(junk).is_empty(), "{junk:?}");
            assert_eq!(run_manifest_gating(junk), None, "{junk:?}");
        }

        // A ledger with one GOOD entry and four unusable ones keeps exactly the
        // good one — dropping the rest, never defaulting them.
        let doc = br#"{
          "shm": [
            {"tag": "good", "class": "trace", "rank": 3, "source": "derived"},
            {"tag": "", "class": "trace", "rank": 0, "source": "derived"},
            {"tag": "no_class", "rank": 0, "source": "derived"},
            {"tag": "bad_class", "class": "wedge", "rank": 0, "source": "derived"},
            {"tag": "bad_source", "class": "trace", "rank": 0, "source": "guessed"}
          ],
          "declared_unavailable": [
            {"rank": 2},
            {"reason": "no rank at all"},
            {"rank": -1, "reason": "negative"},
            {"rank": 4, "reason": "real"}
          ]
        }"#;
        assert_eq!(
            run_manifest_shm(doc),
            vec![ShmDecl {
                tag: "good".to_string(),
                class: ShmClass::Trace,
                rank: Some(3),
                source: TagSource::Derived,
            }],
            "a sweeper acts on names — an entry it cannot fully understand is not one of them"
        );
        assert_eq!(
            run_manifest_declared_unavailable(doc),
            vec![
                RankUnavailable {
                    rank: 2,
                    reason: "reason unstated".to_string()
                },
                RankUnavailable {
                    rank: 4,
                    reason: "real".to_string()
                },
            ],
            "a missing REASON keeps the rank (the rank is the load-bearing half); a missing or \
             unusable RANK drops the entry"
        );
    }

    /// A manifest that cannot be read or parsed is an `Err`, never a silent
    /// success — the caller degrades LOUDLY (`declare_run_rings_best_effort`),
    /// and it can only do that if this returns something to degrade on.
    #[test]
    fn declaring_rings_over_an_unreadable_manifest_is_an_error_not_a_silent_no_op() {
        let tmp = tempfile::tempdir().expect("tempdir");
        assert!(
            declare_run_rings(tmp.path(), &[]).is_err(),
            "no manifest at all must be an error"
        );
        std::fs::write(tmp.path().join(RUN_MANIFEST_FILE), b"not json").expect("write");
        assert!(
            declare_run_rings(tmp.path(), &[]).is_err(),
            "an unparseable manifest must be an error"
        );
        std::fs::write(tmp.path().join(RUN_MANIFEST_FILE), b"[1,2]").expect("write");
        assert!(
            declare_run_rings(tmp.path(), &[]).is_err(),
            "a manifest that is not an OBJECT must be an error"
        );
    }

    /// The two label sets are a `run.json` FORMAT surface — a reader keys on
    /// these exact strings, so a rename must fail here rather than silently in
    /// somebody else's parser.
    #[test]
    fn the_manifest_labels_are_the_pinned_wire_strings() {
        assert_eq!(PartitionProvenance::Declared.label(), "declared");
        assert_eq!(
            PartitionProvenance::DerivedInMemory.label(),
            "derived-in-memory"
        );
        assert_eq!(
            PartitionProvenance::DerivedPersisted.label(),
            "derived-persisted"
        );
        assert_eq!(PartitionProvenance::KeptExisting.label(), "kept-existing");
        assert_eq!(
            PartitionProvenance::AlreadyCurrent.label(),
            "already-current"
        );
        assert_eq!(NetworkPostureLabel::Off.label(), "off");
        assert_eq!(NetworkPostureLabel::Inert.label(), "inert");
        assert_eq!(NetworkPostureLabel::Strict.label(), "strict");
        assert_eq!(NetworkPostureLabel::Permissive.label(), "permissive");
    }

    /// A graph name is user-supplied YAML text spliced into a path, so it is
    /// collapsed to ONE safe component — traversal, separators and NULs
    /// included. Hand oracles.
    #[test]
    fn a_graph_name_is_collapsed_to_one_safe_path_component() {
        assert_eq!(sanitize_component("go2_attach"), "go2_attach");
        assert_eq!(sanitize_component("v1.2-rc"), "v1.2-rc");
        assert_eq!(sanitize_component("../../etc/passwd"), "___.._etc_passwd");
        assert_eq!(sanitize_component("a/b"), "a_b");
        assert_eq!(sanitize_component("a b\tc"), "a_b_c");
        assert_eq!(sanitize_component("naïve"), "na_ve");
        assert_eq!(sanitize_component(""), "graph");
        assert_eq!(sanitize_component("///"), "___");
        for name in ["../../etc/passwd", "a/b", "", "///"] {
            let got = sanitize_component(name);
            assert!(
                !got.contains(std::path::MAIN_SEPARATOR),
                "'{name}' -> '{got}' must be a single component"
            );
        }
    }

    /// A sanitized name never LEADS with a dot, so the run directory it names
    /// is never a dotfile a plain `ls` hides.
    ///
    /// `.` survives the character filter, so before this rule a graph named
    /// `...` produced `~/.cerulion/runs/...-<run_id>` — invisible to an
    /// operator trying to point `cerulion bag record` at a live run, or to
    /// sweep what a SIGKILLed one left behind. The traversal shape above was
    /// the same defect: `../../etc/passwd` sanitized to `.._.._etc_passwd`,
    /// which is ALSO dot-leading, so this file's own oracle was pinning it.
    ///
    /// Interior dots are the anti-tautology half: a rule that simply banned
    /// `.` would pass every assertion below while mangling `v1.2-rc`, so each
    /// arm names the exact surviving text rather than only the first byte.
    #[test]
    fn a_sanitized_name_never_leads_with_a_dot_so_a_run_dir_is_never_hidden() {
        // Dot-ONLY names: nothing to keep, every dot transliterates.
        assert_eq!(sanitize_component("."), "_");
        assert_eq!(sanitize_component(".."), "__");
        assert_eq!(sanitize_component("..."), "___");
        // Dot-LEADING names keep the name they still have.
        assert_eq!(sanitize_component(".hidden"), "_hidden");
        assert_eq!(sanitize_component("..v2"), "__v2");
        assert_eq!(sanitize_component(".a.b"), "_a.b");
        // Interior and trailing dots are untouched — the rule is positional.
        assert_eq!(sanitize_component("v1.2-rc"), "v1.2-rc");
        assert_eq!(sanitize_component("a..b"), "a..b");
        assert_eq!(sanitize_component("trailing."), "trailing.");

        for name in [".", "..", "...", ".hidden", "../../etc/passwd", ".a.b"] {
            let got = sanitize_component(name);
            assert!(
                !got.starts_with('.'),
                "'{name}' -> '{got}' would be a HIDDEN run directory"
            );
        }

        // The suffix the caller appends is what makes traversal structurally
        // impossible — stated here so the rule above is never mistaken for the
        // thing standing between a graph name and `..`.
        for name in [".", "..", "../.."] {
            let component = format!("{}-{:032x}", sanitize_component(name), 1u128);
            assert!(component != "." && component != "..");
            assert_eq!(std::path::Path::new(&component).components().count(), 1);
        }
    }

    /// The `state_ring_consumer` grammar round-trips, and
    /// its tolerance is the ONE direction it claims.
    ///
    /// Hand oracle over the wire form, never a self-compare: a `label`/`parse`
    /// pair that drifted TOGETHER would satisfy a round-trip alone, so the
    /// literal string a `run.json` must hold is written out here.
    #[test]
    fn the_state_ring_consumer_grammar_round_trips() {
        let cases: &[(StateRingConsumerDecl, &str)] = &[
            (StateRingConsumerDecl::Standing, "standing"),
            (
                StateRingConsumerDecl::None {
                    reason: "the Flashback capture plane is switched off for this run".to_string(),
                },
                "none: the Flashback capture plane is switched off for this run",
            ),
            // A recognised state with an EMPTY reason is still recognised — the
            // STATE is the load-bearing half, and refusing here would send a
            // reader to the UNKNOWN arm over a missing sentence.
            (
                StateRingConsumerDecl::None {
                    reason: String::new(),
                },
                // The `format!("none: {reason}")` shape leaves the separating
                // SPACE, so the empty-reason wire form is `"none: "`. Written
                // out rather than trimmed in the oracle: this is the byte
                // sequence a `run.json` really holds, and `parse` trims it back.
                "none: ",
            ),
        ];
        for (decl, wire) in cases {
            assert_eq!(&decl.label(), wire, "{decl:?}");
            assert_eq!(
                StateRingConsumerDecl::parse(wire).as_ref(),
                Some(decl),
                "{wire:?}"
            );
        }
        // Whitespace tolerance, in both places a hand-edited manifest shows it.
        assert_eq!(
            StateRingConsumerDecl::parse("  standing  "),
            Some(StateRingConsumerDecl::Standing)
        );
        assert_eq!(
            StateRingConsumerDecl::parse("none:   switched off  "),
            Some(StateRingConsumerDecl::None {
                reason: "switched off".to_string()
            })
        );
        // …and NOTHING else parses. Case is significant on purpose: a state this
        // build does not know must reach the reader's UNKNOWN arm rather than be
        // guessed at.
        for bogus in [
            "",
            "STANDING",
            "Standing",
            "none",
            "declined: x",
            "sweeping",
        ] {
            assert_eq!(
                StateRingConsumerDecl::parse(bogus),
                None,
                "{bogus:?} must not parse"
            );
        }
    }

    /// The reader keeps THREE states apart, and a present
    /// key is never reported as an absent one.
    ///
    /// The asymmetry that makes this matter: `Absent` licenses the attach to
    /// proceed and say "this run declared nothing", while a present value it
    /// cannot read is a fact about THIS BUILD. Collapsing the second onto the
    /// first states something false about a manifest whose statement is right
    /// there — and does it on the branch that decides whether to lap a newer
    /// run's black box.
    #[test]
    fn the_state_ring_consumer_reader_separates_absent_from_unreadable() {
        let of = |body: &str| run_manifest_state_ring_consumer(body.as_bytes());

        assert_eq!(
            of(r#"{"state_ring_consumer":"standing"}"#),
            StateRingConsumerReport::Known(StateRingConsumerDecl::Standing)
        );
        assert_eq!(
            of(r#"{"state_ring_consumer":"none: switched off"}"#),
            StateRingConsumerReport::Known(StateRingConsumerDecl::None {
                reason: "switched off".to_string()
            })
        );
        // Absent key, and an unparseable document — both are facts about the
        // RUN (or about bytes nobody could read), and both fold to `Absent`
        // because every caller must establish `manifest_parsed()` first.
        assert_eq!(of(r#"{"version":1}"#), StateRingConsumerReport::Absent);
        assert_eq!(of("{ not json"), StateRingConsumerReport::Absent);

        // A value this build does not know: named, so the operator sees WHAT it
        // could not read.
        assert_eq!(
            of(r#"{"state_ring_consumer":"pending"}"#),
            StateRingConsumerReport::Unrecognised {
                raw: "pending".to_string()
            }
        );
        // …INCLUDING a non-string, which is where this reader deliberately
        // parts company with its `trace_rings` sibling. An object form is the
        // most likely way a newer `cerulion` extends this key, so folding it
        // onto `Absent` would render "carries no statement" for a manifest that
        // carries one — and send the attach down the proceed arm on a run that
        // may have said `standing`.
        let obj = of(r#"{"state_ring_consumer":{"state":"standing"}}"#);
        match obj {
            StateRingConsumerReport::Unrecognised { ref raw } => {
                assert!(raw.contains("standing"), "the value must be NAMED: {raw}");
            }
            other => panic!("a present non-string is UNRECOGNISED, not absent: {other:?}"),
        }
    }

    /// The two in-place manifest writers COMPOSE.
    ///
    /// They share one shell (`edit_run_manifest`) and run in sequence on the
    /// supervisor path — the trace declaration before GO, the state-ring one
    /// after it. An extraction shared by two writers is exactly where one
    /// caller's key gets clobbered by the other's rewrite, and nothing else
    /// asserts that the second leaves the first's work standing.
    ///
    /// Also pins the 0600 re-assert on the SECOND writer: `write_artifact`'s
    /// mode applies at creation only, and this path renames a fresh sibling
    /// over an existing file. (The comment on that re-assert records that the
    /// pin was earned by a real 0644 regression on the other caller.)
    #[test]
    fn the_two_manifest_writers_compose_without_clobbering_each_other() {
        // RAII, like every other temp-dir test in this file: a hand-rolled
        // directory with a trailing `remove_dir_all` leaks a 0600 manifest into
        // `$TMPDIR` on any failing assertion, which is exactly when someone is
        // running it.
        let dir = tempfile::tempdir().expect("run dir");
        let tmp = dir.path().to_path_buf();
        std::fs::write(
            tmp.join(RUN_MANIFEST_FILE),
            br#"{"version":1,"run_id":"0x1","shm":[{"class":"arm","tag":"cer_run_1","rank":null,"source":"derived"}]}"#,
        )
        .expect("seed the manifest");

        declare_run_trace(
            &tmp,
            &RunTraceDecl::declared(vec![RingDecl {
                tag: "cer_rec_g_1_r0".to_string(),
                rank: 0,
            }]),
        )
        .expect("the trace declaration must land");
        declare_state_ring_consumer(&tmp, &StateRingConsumerDecl::Standing)
            .expect("the state-ring declaration must land");

        let bytes = std::fs::read(tmp.join(RUN_MANIFEST_FILE)).expect("read back");
        let doc: serde_json::Value = serde_json::from_slice(&bytes).expect("still valid JSON");

        // BOTH writers' keys survive, and so does the entry NEITHER owns.
        assert_eq!(doc["state_ring_consumer"], serde_json::json!("standing"));
        assert_eq!(doc["trace_rings"], serde_json::json!("declared"));
        assert_eq!(doc["rings"][0]["tag"], serde_json::json!("cer_rec_g_1_r0"));
        assert_eq!(doc["run_id"], serde_json::json!("0x1"));
        let shm = doc["shm"].as_array().expect("the ledger survives");
        assert!(
            shm.iter().any(|e| e["class"] == "arm"),
            "the ARM entry belongs to NEITHER writer and must outlive both: {shm:?}"
        );

        // …and the second rewrite left the file owner-only.
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            let mode = std::fs::metadata(tmp.join(RUN_MANIFEST_FILE))
                .expect("stat")
                .permissions()
                .mode()
                & 0o777;
            assert_eq!(mode, RUN_FILE_MODE, "a rewrite must not widen the bits");
        }
        // No sibling left behind by the atomic write.
        assert!(
            !tmp.join(format!("{RUN_MANIFEST_FILE}.tmp")).exists(),
            "the temp file must be renamed away, not left in the run directory"
        );
    }
}

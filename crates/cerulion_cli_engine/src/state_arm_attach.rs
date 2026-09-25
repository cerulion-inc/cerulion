// SPDX-License-Identifier: AGPL-3.0-only
//! The ONE always-on capture plane:
//! who creates it, who opens it, and what refuses to create it.
//!
//! # Why the plane is always on
//!
//! An OPT-IN node-state capture needs its two halves paired
//! by hand BEFORE the graph starts: a recorder creates the arm word,
//! and the graph is told the same name
//! through `CERULION_STATE_ARM_TAG`. Nothing on the `graph run --record` path
//! mints such a tag, so in practice nothing would ever be armed and no recording
//! would be resimmable. `cerulion bagd` has no `--state-arm` flag for that reason.
//!
//! So the GRAPH owns the plane. Every serving `graph run` creates the arm word,
//! arms it at the Flashback cadence, and publishes a state ring per rank —
//! whether or not anybody is recording. The anchors feed two consumers:
//!
//! 1. **Flashback retention** — the rolling window (not implemented yet), so a robot
//!    always has the seconds before an incident.
//! 2. **A recorder, when one is attached** — `graph run --record` and
//!    `cerulion bag record --run` DISCOVER the rings by tag and drain them into
//!    the bag, which is what makes every recording resimmable with NO FLAG.
//!
//! **SCOPE.** An UNRECORDED run's anchors go
//! into the ring with nothing draining them. That is harmless rather than merely
//! tolerable, and the mechanism is worth stating because "an undrained
//! backpressure ring blocks its writer" is the natural assumption: the anchor
//! boundary asks `StateRingProducer::free_records()` BEFORE it walks anything and
//! DECLINES (`SkipCause::RecorderBehind`, published with the non-blocking
//! `try_push_skip`) rather than waiting. So such a run fills its ring once with
//! the run's OLDEST anchors and then declines cheaply forever — one relaxed load
//! and a failed try-push per cadence, on the node thread, with no fork, no smear
//! and no wait. What it does NOT do is retain the NEWEST anchors, which is
//! exactly what a black box needs and exactly what Flashback retention adds.
//!
//! # Who creates, and who opens
//!
//! Exactly one process per run CREATES the word, because
//! [`MappedStateArm::create_owned`](cerulion_core::state_arm::MappedStateArm::create_owned)
//! unlinks the name first: two creators under one tag would end up mapping two
//! DIFFERENT objects and each believing it had paired with the other.
//!
//! ```text
//! monolith `graph run` (incl. --record)   create + arm, then attach as rank 0
//! multi-process SUPERVISOR                create + arm, hold for the run's life
//! `graph run-worker`                      OPEN the word its plan named, rank N
//! ```
//!
//! # The tag
//!
//! Explicit `CERULION_STATE_ARM_TAG` if somebody set one, else DERIVED from this
//! run's `run_id` — the token a recorder can independently re-derive from
//! `run.json`, which is what removes the launch-time coordination. The explicit
//! variable is the manual override under the always-on plane, and it is not a legacy
//! escape hatch: a state-bearing process that is not a `graph run` (an
//! `rmw_cerulion`-hosted ROS 2 process) mints no run and has no `run_id`, so
//! naming its plane by hand is the only way it can have one.
//!
//! # Failing to open is NEVER fatal
//!
//! A missing or unreadable arm word must not stop a robot running its graph: the
//! cost of a failed attach is that THIS process runs unclamped (its `Period`
//! catch-up bursts stay unbounded) and captures nothing, which is exactly the
//! earlier behaviour. So the failure is a log line and the run continues —
//! and because the outcome is also observable through `GraphRuntime::state_arm()`,
//! a caller can tell "un-armed" from "the attach failed" rather than inferring it.
//!
//! RESIDUAL, stated because it is real: on a multi-process run a rank whose open
//! failed does not clamp while its peers do, so for that run the ranks disagree
//! about the burst bound. The bound is on OBSERVER EFFECT, not on data, so the
//! cost is a smeared anchor window on one rank rather than a divergence the
//! barrier must repair — and the log is what makes it visible.
//!
//! RESIDUAL 2 (unchanged by the always-on switch): the read is STARTUP-ONLY. A plane the
//! graph did not create at launch — because the RAM gate refused it, or because
//! `CERULION_FLASHBACK=off` — stays off for that run's life; nothing polls for a
//! change of mind. Turning it back on means restarting the graph.

#[cfg(unix)]
use cerulion_core::state_arm::MappedStateArm;

/// The environment variable a recorder hands the arm's SHM tag through.
pub const STATE_ARM_TAG_ENV: &str = "CERULION_STATE_ARM_TAG";

/// PURE: normalise a raw tag value into "attach to this" or "nothing to attach".
///
/// An UNSET variable and an empty/whitespace-only one are the same instruction —
/// do not attach — because an env var set to `""` is what a shell script that
/// computed no tag leaves behind, and treating it as a name would send every
/// process chasing a word called nothing.
pub fn resolve_state_arm_tag(raw: Option<&str>) -> Option<String> {
    let tag = raw?.trim();
    if tag.is_empty() {
        None
    } else {
        Some(tag.to_string())
    }
}

/// Where a resolved arm tag came from.
///
/// The two are not interchangeable and the difference decides how LOUD a failed
/// attach is — see [`resolve_arm_tag`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArmTagSource {
    /// Somebody set `CERULION_STATE_ARM_TAG`. An explicit REQUEST.
    Explicit,
    /// Derived from this run's `run_id`. Nobody asked for anything.
    DerivedFromRun,
}

/// PURE: resolve the arm tag for a process, and say where it came from.
///
/// Precedence is EXPLICIT over DERIVED, and it has to be: the explicit variable
/// is the only way to name the checkpoint plane of a process that has no run
/// (the `rmw_cerulion` shape — see
/// [`state_arm_tag_for_run`](cerulion_core::state_arm::state_arm_tag_for_run)),
/// and an operator who set it against a `graph run` is naming a plane a
/// recorder already created. Deriving over their answer would send the two
/// halves to two different words.
///
/// `None` means this process has NO checkpoint plane at all: no explicit tag and
/// no run to derive one from. Nothing is armed and nothing is refused — there is
/// no request to refuse. A caller that must arm (a recorder asked for a
/// checkpoint) turns this `None` into its OWN loud refusal, because only the
/// caller knows whether one was asked for.
pub fn resolve_arm_tag(env: Option<&str>, run_id: Option<u128>) -> Option<(String, ArmTagSource)> {
    if let Some(tag) = resolve_state_arm_tag(env) {
        return Some((tag, ArmTagSource::Explicit));
    }
    run_id.map(|id| {
        (
            cerulion_core::state_arm::state_arm_tag_for_run(id),
            ArmTagSource::DerivedFromRun,
        )
    })
}

/// This process's arm tag: the explicit variable, else derived from `run_id`.
pub fn state_arm_tag_for_process(run_id: Option<u128>) -> Option<(String, ArmTagSource)> {
    resolve_arm_tag(std::env::var(STATE_ARM_TAG_ENV).ok().as_deref(), run_id)
}

/// Open the arm word named `tag` and ARM the runtime, reporting a failure at the
/// volume the tag's SOURCE justifies.
///
/// # Why the source decides the level, and why that is not a style choice
///
/// An EXPLICIT tag is a request: somebody pointed this run at a word, so its
/// absence is a broken pairing and gets a `warn!`. A DERIVED tag is not a
/// request: the graph derives one on EVERY run so that a recorder can find
/// this run without coordination, and on the overwhelming majority of runs there
/// is no recorder and no word. Warning there would put a scary line in the log of
/// every ordinary `graph run` for a feature nobody asked for, which is how a
/// warning stops meaning anything.
///
/// The outcome is identical either way, and it is observable through
/// `GraphRuntime::state_arm()` rather than through the log (Principle #3).
#[cfg(unix)]
pub fn attach_state_arm_by_tag_from(
    runtime: &mut cerulion_core::GraphRuntime,
    tag: &str,
    source: ArmTagSource,
) -> bool {
    match cerulion_core::state_arm::MappedStateArm::open_unowned(tag) {
        Ok(arm) => {
            runtime.attach_state_arm(std::sync::Arc::new(arm));
            tracing::info!(
                tag = %tag,
                source = ?source,
                "state arm attached — Period catch-up bursts are clamped while a recorder is armed"
            );
            true
        }
        Err(e) => {
            match source {
                ArmTagSource::Explicit => tracing::warn!(
                    tag = %tag,
                    error = %e,
                    "could NOT open the state arm word this run was pointed at; the run continues \
                     UNCLAMPED (its Period catch-up bursts stay unbounded, as on any un-armed run) \
                     and any checkpoint anchor it takes may smear the fire set of a stalled step"
                ),
                // Nobody asked. The overwhelmingly common case is "no recorder is
                // checkpointing this run", which is not news.
                ArmTagSource::DerivedFromRun => tracing::debug!(
                    tag = %tag,
                    error = %e,
                    "no checkpoint arm word exists for this run's derived tag — nothing is \
                     checkpointing it, so it runs exactly as an un-armed run always has"
                ),
            }
            false
        }
    }
}

// There is deliberately no `attach_state_arm_from_env` / `attach_state_arm_by_tag`
// pair here. An opt-in graph half — open the
// word an operator named in `CERULION_STATE_ARM_TAG`, and nothing else — is not a
// seam: every caller creates or opens through the two entry points below.
// The variable itself is still honoured (see the module docs): it names the plane, and
// `resolve_arm_tag` is where it is read.

/// CREATE and ARM this run's capture plane: the always-on
/// half, and the only place a plane is ever born.
///
/// Returns the OWNER of the arm word, which the caller must bind to a local (or
/// hand to the runtime) that outlives the run: dropping it unlinks the SHM name,
/// and a recorder that has not yet found it would then never find it. `None` means
/// this run has NO capture plane, for one of three reasons, each of which says so
/// in the log:
///
/// * the operator switched it off (`CERULION_FLASHBACK=off`);
/// * the arm-time projection REFUSED it — see [`cerulion_core::flashback`]; or
/// * the word could not be created (a full `/dev/shm`, a name collision).
///
/// # The gate is the price of "always on"
///
/// An opt-in feature can let its user decide whether the cost is worth paying. An
/// always-on one cannot ask, so it has to decline on the user's behalf and TELL
/// them — with the number, the threshold and the override, because a refusal an
/// operator cannot act on is only marginally better than a silent one.
///
/// # A failure here is never fatal
///
/// Every arm returns `None` and the graph runs exactly as it always has. Refusing
/// to run a robot's graph because its black box could not be armed would turn an
/// observability feature into an outage.
#[cfg(unix)]
pub fn arm_capture_plane(
    role: PlaneRole,
    tag: &str,
    tightest_timing_ns: Option<u64>,
) -> Option<std::sync::Arc<MappedStateArm>> {
    let projection = admit_capture_plane(role, tag, tightest_timing_ns)?;
    let arm = match MappedStateArm::create_owned(tag) {
        Ok(a) => a,
        Err(e) => {
            tracing::error!(
                tag = %tag,
                error = %e,
                "could not create this run's capture plane, so the run has NO black box \
                 and any recording of it will not be resimmable. The graph continues running"
            );
            return None;
        }
    };
    // This run now owns `tag` — `create_owned` is
    // unlink-first, so it just destroyed whatever arm word was there. The per-rank
    // STATE RING names under the same tag are the rest of that namespace and the
    // half that carries DATA, so assert ownership of them too, HERE: before any
    // rank creates its own ring, and in the ONE process that creates the word
    // (a worker never reaches this function — it `admit`s and opens).
    //
    // Without it, a REUSED explicit `CERULION_STATE_ARM_TAG` lets a crashed
    // earlier run's rings stand at every rank this run does not create one for —
    // a worker the arm-time RAM gate refused, or a rank this deployment does not have
    // (the earlier run had more workers, or was multi-process where this one is a
    // monolith). `state_ring_run_id` derives from the TAG, so those records carry
    // this run's own `run_id` and nothing downstream can reject them: a reported
    // HOLE becomes a dead run's node state, silently.
    //
    // Best-effort by construction (see `unlink_stale_state_rings`): a leftover
    // name is somebody else's crash and must not stop this graph from running.
    let sweep = cerulion_core::state_ring::unlink_stale_state_rings(tag);
    if !sweep.removed.is_empty() {
        tracing::warn!(
            tag = %tag,
            ranks = ?sweep.removed,
            "removed state ring(s) left over under this capture plane's tag by an \
             earlier run — this only happens under a REUSED explicit tag, and leaving them \
             would have let a recorder adopt a dead run's node state as this run's"
        );
    }
    if !sweep.refused.is_empty() {
        tracing::error!(
            tag = %tag,
            refused = ?sweep.refused,
            "state ring(s) left over under this capture plane's tag could NOT be \
             removed, so a recorder armed with this tag may adopt an earlier run's node state \
             at those ranks. Remove them by hand, or use a fresh tag"
        );
    }
    // Step 0 is a free and perfect anchor (recording and resim both build
    // every node through `with_state(Default::default())`), so the FIRST due step
    // is 0 and the cadence carries it from there.
    arm.arm(projection.cadence_steps, 0);
    // The STANDING COST is stated per role, because it is not the same statement.
    // A monolith anchors itself, so the cost is ITS memory and the numbers sampled
    // above describe it. A supervisor holds the word and anchors NOTHING, so quoting
    // its own resident memory as the price of an anchor would be describing a copy
    // that never happens — the same misleading-surface class as gating the wrong
    // process, one layer up in the log.
    match role {
        PlaneRole::Supervisor => tracing::info!(
            tag = %tag,
            cadence_steps = projection.cadence_steps,
            cadence_ms = projection.cadence_ms,
            "Flashback capture plane ARMED for this deployment — every rank anchors \
             node state every {} step(s). This process holds the word and anchors nothing; each \
             WORKER pays one front-loaded copy of ITS OWN private memory per anchor and reserves \
             its own state ring, and each decides for itself whether it can afford to. \
             `CERULION_FLASHBACK=off` disables it",
            projection.cadence_steps
        ),
        PlaneRole::Monolith | PlaneRole::Worker { .. } => tracing::info!(
            tag = %tag,
            cadence_steps = projection.cadence_steps,
            cadence_ms = projection.cadence_ms,
            state_bytes = ?projection.state_bytes,
            projected_stall_ms = ?projection.stall_ms(),
            ring_bytes = projection.ring_bytes,
            "Flashback capture plane ARMED — this run anchors node state every \
             {} step(s). Standing cost: one front-loaded copy of this process's private memory \
             per anchor, plus the state ring reserved above. `CERULION_FLASHBACK=off` disables it",
            projection.cadence_steps
        ),
    }
    Some(std::sync::Arc::new(arm))
}

/// WHICH process is asking, and therefore both whose refusal this is and whether
/// the memory gate applies to it at all.
#[cfg(unix)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlaneRole {
    /// A monolith `graph run` (including `--record`): it CREATES the plane and it
    /// is also the process that anchors, so it both holds the word and pays the
    /// fork. Its refusal means the RUN has no plane at all.
    Monolith,
    /// The multi-process SUPERVISOR: it creates and holds the arm word for the
    /// deployment and NEVER anchors — see [`PlaneRole::pays_the_fork_cost`]. Its
    /// refusal would silence every worker, so only run-level policy (the kill
    /// switch) may refuse it.
    Supervisor,
    /// One `graph run-worker`, opening a plane its supervisor created and doing the
    /// forking. Its refusal means THIS RANK captures nothing while its peers carry
    /// on.
    Worker { rank: u32 },
}

#[cfg(unix)]
impl PlaneRole {
    /// PURE: does this process ever `fork(2)` for a checkpoint?
    ///
    /// THE question the arm-time memory gate is asking, made explicit because getting
    /// it wrong is silent in BOTH directions:
    ///
    /// * gate a process that does NOT fork and a fat one disables capture for
    ///   processes that would have been fine (the supervisor);
    /// * skip one that DOES and the ceiling is unenforced on exactly the process it
    ///   was set for (the worker).
    ///
    /// A SUPERVISOR answers `false`, and structurally rather than by convention:
    /// the fork lives in `GraphRuntime::take_anchor`, which returns at its first
    /// line unless a state-ring producer is installed, and a producer is installed
    /// only by `attach_state_capture_*`. The supervisor calls neither — it never
    /// builds a live loop at all, it spawns workers and joins them — so it holds
    /// the word, allocates no ring, and copies nothing. Pricing its resident memory
    /// prices a fork that cannot happen.
    pub fn pays_the_fork_cost(self) -> bool {
        match self {
            Self::Monolith | Self::Worker { .. } => true,
            Self::Supervisor => false,
        }
    }
}

/// The admission decision (the kill switch, the knobs, and the arm-time
/// projection), asked by EVERY process that would fork.
///
/// Returns the projection when this process is admitted; `None` (having said why,
/// out loud, in this process's own log) when it is not.
///
/// # Why every process, and not just the one that arms
///
/// The gate prices a `fork(2)` of THE CALLING PROCESS, and under a
/// multi-process split the process that forks is each WORKER — while the process
/// that ARMS is the supervisor, whose own resident memory is a few megabytes of
/// spawn bookkeeping and says nothing about any worker's state.
///
/// Evaluating it only at the supervisor would therefore measure the wrong process
/// entirely: a 1 MiB supervisor admits the plane and a worker of any size opens
/// it, so the ceiling an operator set is silently unenforced on exactly the
/// processes it was set for. The always-on plane makes this gate the AUTOMATIC safety for the
/// process paying the cost, so the decision belongs where the cost lands.
///
/// # A worker's refusal is its OWN, and its peers keep running
///
/// A refused rank creates no state ring, so a graph-wide anchor is partial
/// and the recorder reports the hole through `missing_state_ring_ranks` — which is
/// the accurate outcome and already has machinery. The alternative, tearing down the
/// whole run's plane because one rank is fat, would throw away every other rank's
/// anchors for no gain; and the ranks cannot vote, since each learns its own size
/// only once it is built.
#[cfg(unix)]
pub fn admit_capture_plane(
    role: PlaneRole,
    tag: &str,
    tightest_timing_ns: Option<u64>,
) -> Option<cerulion_core::flashback::ArmProjection> {
    use cerulion_core::flashback;
    use cerulion_core::state_carrier::fork::{
        sample_anon_rss, sample_mem_available, CAPTURE_MEM_FLOOR_BYTES,
    };

    match flashback::parse_plane_switch(std::env::var(flashback::FLASHBACK_ENV).ok().as_deref()) {
        flashback::PlaneSwitch::Off => {
            // LOUD, and at `info!` rather than `debug!`: an operator debugging a
            // run that produced no anchors must be able to see, in that run's own
            // log, that they are the reason.
            tracing::info!(
                env = flashback::FLASHBACK_ENV,
                "Flashback is OFF for this run — no capture plane, no state ring, no anchors, \
                 and any recording of this run will not be resimmable. Unset \
                 CERULION_FLASHBACK (or set it to `on`) to restore the default"
            );
            return None;
        }
        flashback::PlaneSwitch::Unrecognized(value) => {
            // The plane stays ON: an unreadable instruction must not silently
            // disable a safety feature. But the operator's belief and the
            // machine's behaviour have diverged, and nothing else will say so.
            tracing::warn!(
                env = flashback::FLASHBACK_ENV,
                value = %value,
                "unrecognised value — Flashback stays ON. The only value that turns it off is \
                 `off`"
            );
        }
        flashback::PlaneSwitch::On => {}
    }

    let (cadence_ms, complaint) = flashback::resolve_cadence_ms(
        std::env::var(flashback::FLASHBACK_CADENCE_MS_ENV)
            .ok()
            .as_deref(),
    );
    if let Some(reason) = complaint {
        tracing::warn!(env = flashback::FLASHBACK_CADENCE_MS_ENV, %reason, "ignoring");
    }
    let (max_state_bytes, complaint) = flashback::resolve_max_state_bytes(
        std::env::var(flashback::FLASHBACK_MAX_STATE_MB_ENV)
            .ok()
            .as_deref(),
    );
    if let Some(reason) = complaint {
        tracing::warn!(env = flashback::FLASHBACK_MAX_STATE_MB_ENV, %reason, "ignoring");
    }

    let projection = flashback::ArmProjection {
        state_bytes: sample_anon_rss(),
        mem_available: sample_mem_available(),
        max_state_bytes,
        // The size REALLY in force, not the built-in default —
        // an operator who raised the ring must see the raised figure in the
        // projection they are being priced against.
        ring_bytes: cerulion_core::flashback::resolve_state_ring_bytes(
            std::env::var(cerulion_core::flashback::FLASHBACK_STATE_RING_MB_ENV)
                .ok()
                .as_deref(),
        )
        .0,
        // The GATING QUANTUM, derived exactly as `build_live_deterministic` does:
        // the graph's tightest declared timing, 1 ms when it declares none (a
        // purely data-driven graph), floored at 1 ms either way. Re-deriving it
        // the same way is what keeps the cadence a real number of that graph's
        // steps — a cadence computed against a quantum the scheduler does not use
        // would anchor at the wrong logical rate on exactly the graphs whose
        // timing is unusual.
        cadence_steps: flashback::cadence_steps(
            cadence_ms,
            tightest_timing_ns.unwrap_or(1_000_000).max(1_000_000),
        ),
        cadence_ms,
    };
    // THE MEMORY GATE — asked ONLY of a process that will actually fork.
    //
    // A supervisor holds the word and anchors nothing (see
    // `PlaneRole::pays_the_fork_cost`), so its resident memory is a few megabytes of
    // spawn bookkeeping that describes no capture child anywhere. Judging it was the
    // exact mirror of the worker bug: a fat supervisor with workers that all fit
    // returned `None`, stamped no tag, and disabled capture for the WHOLE
    // deployment. Its workers each ask this question for themselves, later and about
    // the right process.
    //
    // The HEADROOM arm goes with the size arm rather than staying behind, for the
    // same reason: it asks whether the FIRST ANCHOR could afford to run, every
    // anchor happens in a worker, and each worker re-reads `MemAvailable` at its own
    // attach. Leaving it here would let one transient low-memory moment at
    // supervisor launch silence a whole run's capture for its lifetime.
    if role.pays_the_fork_cost() {
        if let Some(refusal) = flashback::arm_verdict(&projection, CAPTURE_MEM_FLOOR_BYTES) {
            report_arm_refusal(role, tag, &projection, refusal, max_state_bytes);
            return None;
        }
    }
    Some(projection)
}

/// The arm-time report, on the path where it REFUSES.
///
/// Its whole job is to be actionable, so it names four things: what was measured,
/// what it would have cost, what the ceiling is, and which variable moves the
/// ceiling. A refusal that states only the verdict leaves an operator with a robot
/// that has no black box and no idea why.
#[cfg(unix)]
fn report_arm_refusal(
    role: PlaneRole,
    tag: &str,
    projection: &cerulion_core::flashback::ArmProjection,
    refusal: cerulion_core::flashback::ArmRefusal,
    max_state_bytes: u64,
) {
    use cerulion_core::flashback::{
        ArmRefusal, FLASHBACK_MAX_STATE_MB_ENV, FLASHBACK_STALL_BUDGET_MS,
    };

    let mb = |b: u64| b / (1024 * 1024);
    // WHAT IS LOST differs by role, and an operator cannot infer it from the
    // refusal alone: an owner's refusal means the run has no plane, while a
    // worker's means ONE rank is missing from anchors the other ranks still take —
    // which is what the recorder will report as a rank hole.
    let scope = match role {
        PlaneRole::Monolith => "this run has NO capture plane at all, so any bag recorded of \
             it carries NO node-state anchors and is NOT resimmable"
            .to_string(),
        PlaneRole::Worker { rank } => format!(
            "rank {rank} captures NOTHING while its peers carry on, so every anchor of this \
             run will be reported PARTIAL"
        ),
        // UNREACHABLE today: a supervisor never reaches the memory gate
        // (`pays_the_fork_cost` is false for it), so nothing calls this with that
        // role. Stated rather than `unreachable!()`ed — a panic on a robot's launch
        // path is never the right answer to a checkpoint-plane problem, and the
        // match must stay total if a future role is added.
        PlaneRole::Supervisor => "this deployment has NO capture plane, so no rank anchors \
             and no bag of this run is resimmable"
            .to_string(),
    };
    let rank = match role {
        PlaneRole::Monolith | PlaneRole::Supervisor => None,
        PlaneRole::Worker { rank } => Some(rank),
    };
    match refusal {
        ArmRefusal::StateTooLarge => tracing::warn!(
            tag = %tag,
            rank = ?rank,
            state_mb = projection.state_bytes.map(mb),
            projected_stall_ms = ?projection.stall_ms(),
            stall_budget_ms = FLASHBACK_STALL_BUDGET_MS,
            max_state_mb = mb(max_state_bytes),
            env = FLASHBACK_MAX_STATE_MB_ENV,
            "Flashback STATE capture is OFF for this process — its {} MiB of private \
             memory would stall a node thread for about {} ms on every anchor, past the {} ms \
             budget. The graph runs normally and nothing else is affected; what is lost is that \
             {}. Raise the ceiling with CERULION_FLASHBACK_MAX_STATE_MB (currently {} MiB) if the \
             duty cycle can afford it, or lengthen CERULION_FLASHBACK_CADENCE_MS to pay it less \
             often",
            projection.state_bytes.map(mb).unwrap_or(0),
            projection.stall_ms().unwrap_or(0),
            FLASHBACK_STALL_BUDGET_MS,
            scope,
            mb(max_state_bytes)
        ),
        ArmRefusal::NoHeadroom => tracing::warn!(
            tag = %tag,
            rank = ?rank,
            state_mb = projection.state_bytes.map(mb),
            mem_available_mb = projection.mem_available.map(mb),
            "Flashback STATE capture is OFF for this process — forking a capture child \
             would leave the machine under the memory floor the live graph needs, so every anchor \
             would be declined anyway and arming would reserve a ring for nothing. The graph runs \
             normally; {}",
            scope
        ),
    }
}

/// Arm this process's half of the CHECKPOINT DATA PLANE — attach the
/// arm word AND create the per-rank state ring the boundary pushes anchor parts
/// into.
///
/// Returns the ring OWNER, which the caller must bind to a local that outlives the
/// live loop: dropping it `shm_unlink`s the ring name, and a recorder that has not
/// yet found the ring would then never find it. `None` means this process is not
/// capturing — either nothing armed it, or the ring could not be created (loudly,
/// see below).
///
/// # The ring is created only once the arm attach SUCCEEDED
///
/// A state ring is a 64 MiB SHM reservation by default. Creating one on every
/// graph run — the overwhelming majority of which nobody is checkpointing — would
/// charge every robot for a feature it did not ask for. A SUCCESSFUL attach is the
/// proof that a recorder created an arm word for this tag, i.e. that something is
/// on the other end.
///
/// The consequent ordering is deliberate and is what the recorder's DISCOVERY
/// exists to absorb: the ring appears when the graph arms, which on a
/// `graph run --record` deployment is after the recorder was spawned. A recorder
/// cannot therefore open per-rank rings from its argv, so it probes for them
/// (`(tag, rank)` is a name both sides derive) for the whole run.
///
/// # A failure here is LOUD, and it is not fatal
///
/// A rank that cannot create its ring contributes no anchor parts, and a
/// graph-wide anchor is all-or-nothing across ranks — so this rank alone
/// voids every anchor of the run. That is worth an `error!`, not a `warn!`, and it
/// is deliberately NOT fatal: refusing to run the graph because a checkpoint ring
/// could not be created would turn an observability feature into an outage. The
/// recorder half of the same statement is `missing_state_ring_ranks`, which turns
/// a HOLE in the discovered ranks into its own loud report — between them a
/// silently-absent rank is unreachable from both ends.
///
/// `source` carries where the tag came from, so a failed attach is reported at the
/// volume that source justifies (see [`attach_state_arm_by_tag_from`]).
#[cfg(unix)]
pub fn attach_state_capture_from(
    runtime: &mut cerulion_core::GraphRuntime,
    tag: &str,
    source: ArmTagSource,
    rank: u32,
    node_ids: &[String],
) -> Option<cerulion_core::state_ring::StateRingOwner> {
    if !attach_state_arm_by_tag_from(runtime, tag, source) {
        return None;
    }
    create_state_ring(runtime, tag, rank, node_ids)
}

/// Attach a plane THIS process just created.
///
/// The owner half of [`attach_state_capture_from`], and the reason it is a
/// separate entry point rather than a re-open: the creator already holds a mapping
/// of the word, so opening it a second time would add a mapping, a second failure
/// mode, and a question about which handle owns the NAME. Handing the `Arc`
/// straight to the runtime answers all three — the runtime holds the OWNER, so the
/// plane's name lives exactly as long as the run does.
#[cfg(unix)]
pub fn attach_state_capture_owned(
    runtime: &mut cerulion_core::GraphRuntime,
    arm: std::sync::Arc<MappedStateArm>,
    tag: &str,
    rank: u32,
    node_ids: &[String],
) -> Option<cerulion_core::state_ring::StateRingOwner> {
    runtime.attach_state_arm(arm);
    create_state_ring(runtime, tag, rank, node_ids)
}

/// The RING half, shared by both attach paths so the creator and the opener cannot
/// drift in what they publish or in how loudly they fail.
#[cfg(unix)]
fn create_state_ring(
    runtime: &mut cerulion_core::GraphRuntime,
    tag: &str,
    rank: u32,
    node_ids: &[String],
) -> Option<cerulion_core::state_ring::StateRingOwner> {
    use cerulion_core::state_ring::{state_ring_run_id, state_ring_tag, StateRingOwner};

    let ring_tag = match state_ring_tag(tag, rank) {
        Ok(t) => t,
        Err(e) => {
            tracing::error!(
                tag = %tag,
                rank,
                error = %e,
                "this run is ARMED for checkpoints but its state-ring name cannot be \
                 derived, so this rank captures NOTHING — and because a graph-wide anchor is \
                 all-or-nothing across ranks, every anchor of this run will be reported partial. \
                 The graph continues running"
            );
            return None;
        }
    };
    let refs: Vec<&str> = node_ids.iter().map(String::as_str).collect();
    // The per-rank ring's SIZE comes from an operator knob rather than a fixed
    // constant. A capture's pre-window needs `window / cadence + 1` anchors held,
    // and an anchor is bounded only by the arm-time gate, so this is a property
    // of the robot rather than of the framework.
    let (ring_bytes, complaint) = cerulion_core::flashback::resolve_state_ring_bytes(
        std::env::var(cerulion_core::flashback::FLASHBACK_STATE_RING_MB_ENV)
            .ok()
            .as_deref(),
    );
    if let Some(reason) = complaint {
        tracing::warn!(env = cerulion_core::flashback::FLASHBACK_STATE_RING_MB_ENV, %reason, "ignoring");
    }
    let mut owner = match StateRingOwner::create(
        &ring_tag,
        cerulion_core::state_ring::capacity_records_for_bytes(ring_bytes as usize),
        rank,
        state_ring_run_id(tag),
        &refs,
    ) {
        Ok(o) => o,
        Err(e) => {
            tracing::error!(
                tag = %tag,
                rank,
                ring_tag = %ring_tag,
                error = %e,
                "this run is ARMED for checkpoints but its state ring could not be \
                 created, so this rank captures NOTHING — and because a graph-wide anchor is \
                 all-or-nothing across ranks, every anchor of this run will be reported partial. \
                 The graph continues running"
            );
            return None;
        }
    };
    let Some(producer) = owner.producer() else {
        // Unreachable: a freshly created owner has minted no producer. Reported
        // rather than asserted, on the same terms as everything else here — a
        // panic on the launch path of a robot's graph is never the right answer
        // to a checkpoint-plane problem.
        tracing::error!(
            tag = %tag,
            rank,
            ring_tag = %ring_tag,
            "the state ring yielded no producer (internal invariant violation — a \
             freshly created ring must yield its single producer); this rank captures NOTHING"
        );
        return None;
    };
    runtime.set_state_ring_producer(producer, node_ids);
    tracing::info!(
        tag = %tag,
        rank,
        ring = %owner.name(),
        nodes = node_ids.len(),
        "checkpoint state ring created — this rank publishes its anchor parts here, \
         and a recorder armed with the same tag discovers it by name"
    );
    Some(owner)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The normalisation table, hand-written. The three "nothing to attach"
    /// rows are the load-bearing ones: an env var that exists but says nothing
    /// is the shape a script leaves behind when it computed no tag, and reading
    /// it as a NAME sends the process off to open a word that cannot exist.
    #[test]
    fn only_a_non_blank_tag_is_an_instruction_to_attach() {
        assert_eq!(resolve_state_arm_tag(None), None, "unset");
        assert_eq!(resolve_state_arm_tag(Some("")), None, "empty");
        assert_eq!(resolve_state_arm_tag(Some("   ")), None, "whitespace only");
        assert_eq!(resolve_state_arm_tag(Some("\t\n")), None, "whitespace only");
        assert_eq!(
            resolve_state_arm_tag(Some("cer_run_42")),
            Some("cer_run_42".to_string())
        );
        // Surrounding whitespace is TRIMMED rather than rejected: a tag that
        // round-trips through a shell variable routinely picks it up, and the
        // SHM name is derived by hashing this string, so an untrimmed copy would
        // silently name a DIFFERENT word than the recorder created.
        assert_eq!(
            resolve_state_arm_tag(Some("  cer_run_42\n")),
            Some("cer_run_42".to_string())
        );
        // Interior whitespace is part of the name — only the ENDS are trimmed.
        assert_eq!(
            resolve_state_arm_tag(Some(" a b ")),
            Some("a b".to_string())
        );
    }

    /// Precedence, and the shape that resolves to NOTHING.
    ///
    /// The `None` row is the one that matters and it is the `rmw_cerulion` rider:
    /// a state-bearing process that is not a `graph run` mints no run and has no
    /// `run_id`, so nothing can be derived for it. It must be given an EXPLICIT
    /// tag — and if it is given neither, this resolves to `None` and NOTHING is
    /// armed, rather than a name being invented for a plane no recorder will
    /// ever find.
    #[test]
    fn explicit_beats_derived_and_neither_resolves_to_nothing() {
        let derived = cerulion_core::state_arm::state_arm_tag_for_run(0x2a);

        // A run, no explicit tag: derive.
        assert_eq!(
            resolve_arm_tag(None, Some(0x2a)),
            Some((derived.clone(), ArmTagSource::DerivedFromRun))
        );
        // An explicit tag WINS over a run's own — an operator naming a plane is
        // naming one a recorder already created, and deriving over that would
        // send the two halves to two different words.
        assert_eq!(
            resolve_arm_tag(Some("hand_picked"), Some(0x2a)),
            Some(("hand_picked".to_string(), ArmTagSource::Explicit))
        );
        // A blank explicit tag says NOTHING (a script that computed no tag), so
        // the run's derivation still applies — the blank must not shadow it.
        for blank in ["", "   "] {
            assert_eq!(
                resolve_arm_tag(Some(blank), Some(0x2a)),
                Some((derived.clone(), ArmTagSource::DerivedFromRun)),
                "a blank tag is not an answer"
            );
        }
        // THE RIDER: no run and no explicit tag — the rmw shape. Nothing.
        assert_eq!(resolve_arm_tag(None, None), None);
        assert_eq!(resolve_arm_tag(Some("  "), None), None);
        // …and an explicit tag with no run IS the supported rmw path.
        assert_eq!(
            resolve_arm_tag(Some("rmw_plane"), None),
            Some(("rmw_plane".to_string(), ArmTagSource::Explicit))
        );
    }

    /// Two runs derive two tags; one run derives one, from anywhere.
    ///
    /// Both halves are load-bearing: the first is why the tag is keyed on
    /// `run_id` and not on the graph NAME (two runs of one graph would collide
    /// into one checkpoint plane), and the second is what lets a recorder that
    /// only read `run.json` reach the same word the graph opened.
    #[test]
    fn a_runs_derived_tag_is_a_function_of_its_id_alone() {
        use cerulion_core::state_arm::state_arm_tag_for_run;
        assert_eq!(state_arm_tag_for_run(1), state_arm_tag_for_run(1));
        assert_ne!(state_arm_tag_for_run(1), state_arm_tag_for_run(2));
        // Zero-padded to the full 128 bits, so no two ids can render alike and
        // the name is a fixed length whatever the id.
        assert_eq!(
            state_arm_tag_for_run(0x2a),
            "cer_run_0000000000000000000000000000002a"
        );
        assert_eq!(state_arm_tag_for_run(u128::MAX).len(), 8 + 32);
    }

    /// A FAILED attach creates NO state ring.
    ///
    /// # Why this needs its own arm rather than following from the others
    ///
    /// EVERY `graph run` derives an arm tag, so this path is taken on
    /// every run on every robot — and on the overwhelming majority there is no
    /// recorder and no arm word. A state ring is a 64 MiB SHM reservation by
    /// default, so a carrier that created one anyway would charge every robot
    /// that much, permanently, for a feature nobody asked for.
    ///
    /// NOTHING else catches it. Every functional arm in the suite drives the
    /// path where the attach SUCCEEDS, and a ring created on the failing path is
    /// invisible to all of them: the run behaves identically, the coverage
    /// manifest is unaffected, and the recorder that would notice does not
    /// exist. Dropping the early return leaves every other state test green,
    /// which is why the oracle here is the SHM OBJECT
    /// — the only place the cost is observable.
    #[cfg(unix)]
    #[test]
    fn a_failed_attach_creates_no_state_ring() {
        use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
        use cerulion_core::graph::node::{ClosureNodeEntry, NodeEntry, NodeInfo};
        use cerulion_core::MacroPolicy;
        use indexmap::IndexMap;
        use std::sync::Arc;

        // A tag nothing has armed — no recorder ever created this word.
        let tag = format!(
            "c5_noarm_{}_{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .expect("clock")
                .subsec_nanos()
        );
        let ring_name = cerulion_core::state_ring::state_ring_shm_name(&tag, 0)
            .expect("an ordinary rank names a ring");
        assert!(
            !cerulion_core::shm_ring::shm_object_exists(&ring_name),
            "precondition: this run's ring name must start unused"
        );

        let config = GraphConfig {
            execution: None,
            level_assignments: None,
            network: None,
            process_groups: Default::default(),
            process_group_order: Default::default(),
            multi_publisher_topics: Vec::new(),
            name: None,
            identity: "noarm".to_string(),
            prefix: "c53na".to_string(),
            nodes: vec![NodeDef {
                fuse: None,
                ros2: None,
                id: "n".to_string(),
                node_type: "n".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "u8".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            }],
        };
        let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
        factories.insert(
            "n".to_string(),
            Box::new(ClosureNodeEntry::new(
                NodeInfo::from_names(vec![], vec!["out".to_string()])
                    .with_policy(MacroPolicy::Period { period_ms: 100 }),
                |_ctx| Ok(()),
            )),
        );
        let clock = Arc::new(cerulion_core::VirtualClock::new());
        let mut runtime =
            cerulion_core::GraphRuntime::build_for_test_barrier(config, factories, clock, 8)
                .expect("the pin graph builds");

        let owner = attach_state_capture_from(
            &mut runtime,
            &tag,
            ArmTagSource::DerivedFromRun,
            0,
            &["n".to_string()],
        );
        assert!(
            owner.is_none(),
            "no word exists, so nothing may be armed and nothing captured"
        );
        assert!(
            runtime.state_arm().is_none(),
            "…and the runtime must not report itself armed"
        );
        // THE ORACLE: no segment was created. This is the assertion a
        // dropped early return fails.
        assert!(
            !cerulion_core::shm_ring::shm_object_exists(&ring_name),
            "a run nobody is checkpointing must not reserve a state ring — `{ring_name}` \
             exists, which is 64 MiB per graph run charged to every robot"
        );
    }

    /// The env var's NAME is a cross-process contract: the recorder sets it and
    /// the graph process reads it, in two different binaries, so a rename in one
    /// place is a silently un-armed robot.
    #[test]
    fn the_tag_env_var_is_the_documented_name() {
        assert_eq!(STATE_ARM_TAG_ENV, "CERULION_STATE_ARM_TAG");
    }

    /// The crate-wide env lock, not a private one (the
    /// rule `lib.rs`'s `test_env` module states outright).
    ///
    /// `#[serial_test::serial]` is NOT a substitute:
    /// libtest runs every `#[cfg(test)] mod tests` in this library in ONE process,
    /// so `account_cmd`, `connect_cmd`, `login_cmd`, `robot_cmd`, `run_dir`,
    /// `schema_serve` and `viz_client` all mutate the SAME environment — and
    /// `#[serial]` takes serial_test's own mutex, putting a `#[serial]` env test
    /// and a `test_env` env test in two INDEPENDENT domains that run concurrently.
    /// The arm below would then race a sibling's env snapshot, and the failure it
    /// produces (a `CERULION_FLASHBACK` restored to somebody else's value, so
    /// `arm_capture_plane` returns `None` and the `expect` fires) looks like a
    /// flake rather than a lock bug.
    #[cfg(unix)]
    fn env_lock() -> std::sync::MutexGuard<'static, ()> {
        crate::test_env::env_lock()
    }

    /// RAII: set a process-global env var for the body and restore it after.
    ///
    /// The kill switch is read from the environment inside `admit_capture_plane`,
    /// and a developer (or a sibling test) with `CERULION_FLASHBACK=off` set would
    /// otherwise turn the arm below into a vacuous `None`.
    #[cfg(unix)]
    struct EnvVarGuard {
        key: &'static str,
        prior: Option<String>,
    }

    #[cfg(unix)]
    impl EnvVarGuard {
        fn set(key: &'static str, value: &str) -> Self {
            let prior = std::env::var(key).ok();
            std::env::set_var(key, value);
            Self { key, prior }
        }
    }

    #[cfg(unix)]
    impl Drop for EnvVarGuard {
        fn drop(&mut self) {
            match self.prior.take() {
                Some(v) => std::env::set_var(self.key, v),
                None => std::env::remove_var(self.key),
            }
        }
    }

    /// Plant a state ring the way a CRASH leaves one: create it, then leak the
    /// owner so its `Drop` never `shm_unlink`s the name.
    #[cfg(unix)]
    fn plant_orphan_ring(arm_tag: &str, rank: u32) {
        let ring_tag = cerulion_core::state_ring::state_ring_tag(arm_tag, rank).expect("nameable");
        let owner = cerulion_core::state_ring::StateRingOwner::create(
            &ring_tag,
            8,
            rank,
            cerulion_core::state_ring::state_ring_run_id(arm_tag),
            &["n0"],
        )
        .expect("plant the orphan ring");
        std::mem::forget(owner);
    }

    /// The ranks a recorder handed `arm_tag` could ADOPT — bagd's own probe
    /// (`shm_object_exists` over `state_ring_shm_name`, walked by
    /// `scan_state_ring_ranks`), which is what `Recorder::discover_state_rings`
    /// runs at arm time and again every 250 ms for the whole run.
    #[cfg(unix)]
    fn adoptable_ranks(arm_tag: &str) -> Vec<u32> {
        cerulion_core::state_ring::scan_state_ring_ranks(|rank| {
            cerulion_core::state_ring::state_ring_shm_name(arm_tag, rank)
                .map(|n| cerulion_core::shm_ring::shm_object_exists(&n))
                .unwrap_or(false)
        })
    }

    /// **Arming a tag must leave a recorder handed that tag
    /// nothing of an EARLIER run to adopt — and must still leave the HOLE visible.**
    ///
    /// # The defect
    ///
    /// The monolith seam gates its `--state-tag` on having really created its own
    /// rank-0 ring, but a SUPERVISOR cannot: its ranks' rings are created by other
    /// processes, AFTER the GO sentinel, i.e. after the point at which the
    /// always-on recorder is spawned. There is no moment at which the supervisor
    /// could hand over "confirmed ring identities", and withholding the whole
    /// deployment's tag because one rank refused would throw away every other
    /// rank's anchors — the opposite of the partial-capture design.
    ///
    /// So the contamination is removed at its SOURCE instead. `create_owned` is
    /// unlink-first, so arming under a tag already asserts ownership of it; the
    /// per-rank ring names under the same tag are the rest of that namespace and
    /// the half that carries DATA. Clearing them here closes the class for BOTH
    /// seams, and for two shapes neither gate could ever have covered: a rank this
    /// deployment does not HAVE (the earlier run was wider), and a monolith run
    /// under a tag whose earlier run was multi-process.
    ///
    /// # What each half of this arm is for
    ///
    /// `PlaneRole::Supervisor` because it is the role the defect above concerns, and
    /// because it is the one role the arm-time memory gate does not judge — so the arm
    /// is decided by the code under test rather than by the size of the test
    /// process (`pays_the_fork_cost`).
    ///
    /// The rank-2 half pins that partial-capture reporting is
    /// preserved: after the sweep and a legitimate rank-0 create, a
    /// recorder sees rank 0 and NOT rank 2 — which is exactly the input
    /// `bagd`'s `a_rank_that_published_no_ring_is_reported_and_makes_the_recording_incomplete`
    /// turns into `ranks_missing` + `is_incomplete()`. A leaked ring at rank 2
    /// would silently invert that arm, and this is the seam that stops it.
    /// No `#[serial]`: the only process-global state this arm touches besides the
    /// environment is the POSIX SHM namespace, and every name it uses is derived
    /// from a pid+nanos tag, so it cannot collide with a sibling. The environment
    /// is what needs the lock, and `env_lock` is what serialises it.
    #[test]
    #[cfg(unix)]
    fn arming_a_tag_clears_the_state_rings_an_earlier_run_left_under_it() {
        // Held for the WHOLE body, taken BEFORE the env is touched. Declared
        // first, so reverse drop order restores `CERULION_FLASHBACK` (the guard
        // below) while this lock is still held.
        let _env = env_lock();
        let tag = format!(
            "stale_{}_{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        );
        // Rank 0: a rank the next run HAS. Rank 2: one whose worker refuses the
        // arm-time memory gate, or that the next run does not have at all.
        plant_orphan_ring(&tag, 0);
        plant_orphan_ring(&tag, 2);
        // ANTI-TAUTOLOGY: without this, "nothing is adoptable" is satisfied by a
        // planting helper that planted nothing.
        assert_eq!(
            adoptable_ranks(&tag),
            vec![0, 2],
            "precondition: a crashed earlier run really does leave rings a recorder can adopt"
        );

        let _switch = EnvVarGuard::set(cerulion_core::flashback::FLASHBACK_ENV, "on");
        let arm = arm_capture_plane(PlaneRole::Supervisor, &tag, Some(1_000_000)).expect(
            "the supervisor role skips the arm-time memory gate, so only the kill switch — which \
             this test sets to `on` — can refuse it",
        );

        // THE ORACLE: a recorder handed this tag can adopt NOTHING.
        assert!(
            adoptable_ranks(&tag).is_empty(),
            "arming `{tag}` must leave no ring this run did not create — a leaked one carries a \
             TAG-derived `run_id` identical to this run's, so nothing downstream can reject it"
        );

        // …and the HOLE survives: rank 0 creates its ring for real, rank 2 does
        // not, and that is exactly what the recorder must see.
        let ring_tag = cerulion_core::state_ring::state_ring_tag(&tag, 0).expect("nameable");
        let rank0 = cerulion_core::state_ring::StateRingOwner::create(
            &ring_tag,
            8,
            0,
            cerulion_core::state_ring::state_ring_run_id(&tag),
            &["n0"],
        )
        .expect("rank 0 creates its ring");
        assert_eq!(
            adoptable_ranks(&tag),
            vec![0],
            "rank 2 must read as an ABSENCE the recorder reports, never as a dead run's records"
        );
        drop(rank0);
        drop(arm);
    }
}

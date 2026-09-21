// SPDX-License-Identifier: AGPL-3.0-only
//! The query-plane seam — the boundary between netd's UDS
//! control server ([`crate::daemon`]) and the machine's ONE zenoh session for the
//! CATALOG / SCHEMA query surface.
//!
//! Without this plane, every desk consumer that wants a remote robot's topic catalog or a
//! type's `.msg`/YAML closure opens its OWN transient zenoh discovery session
//! (`topic echo`/`info`/`hz` schema-hash resolution, `cerulion schema info`'s
//! network resolution, vizd's `attach_remote`). That is N sessions + N network
//! copies and it violates the "one network gateway per computer" rule
//! on the QUERY plane too. So those queries fold onto netd: a consumer sends
//! `query_catalog` / `query_schema` over the UDS control seam, netd executes the
//! GET over its ONE zenoh session (the SAME session the mirror + egress planes
//! own — Principle #8), and returns the decoded replies over UDS.
//!
//! # Stateless one-shots — NOT refcounted
//!
//! Unlike a `demand` (which refcounts a shared mirror), a query creates NOTHING and
//! is NOT tracked: netd issues the GET, returns the replies, and forgets. The query
//! CONNECTION counts as live only while it is open (a consumer connects, queries,
//! reads the answer, and disconnects), which is exactly the idle-lifecycle contract
//! — a query keeps netd alive for the duration of the query, never longer.
//!
//! # The production plane reuses the ONE session
//!
//! [`GatewayQueryPlane`] wraps the SAME [`TransportManager`] the mirror plane owns.
//! Its GETs go over `manager.network().session()` — the lazy zenoh session that
//! opens on the first network op — so a netd that only ever serves queries opens
//! ONE session and reuses it across every consumer's query. A local-only
//! (`CERULION_NETD_NETWORK=off`) daemon has no `NetworkManager`, so a query returns
//! an explicit [`QueryError::NoNetwork`] — the consumer then LOUDLY degrades to its
//! own transient session (the fallback discipline; a kill-switched netd never
//! silently swallows a query).
//!
//! # Authorization rides the SERVE side (unchanged)
//!
//! A queried robot's own [`DemandAuthorizer`](cerulion_core::transport::demand_authorizer::DemandAuthorizer)
//! gate may REFUSE to serve its catalog/schema — which arrives as a decodable
//! [`CatalogReply`] / [`SchemaReply`]
//! carrying an `error` (a docs-empty refusal, `CatalogReply::refused` /
//! `SchemaReply::refused`). netd passes those refusal replies through VERBATIM in its
//! query response, so the desk surfaces the explicit refusal (never a silent empty).
//! Nothing to gate on the DESK side today (AllowAll default); the refusal is the
//! robot's, carried faithfully to the consumer.

use std::collections::BTreeSet;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::{Duration, Instant};

use cerulion_core::transport::cerulion_q::UnusableRunsAnswer;
use cerulion_core::transport::discovery::{
    query_announce_entries, query_robot_catalog, query_robot_catalogs, query_robot_runs,
    query_robot_runs_all, query_robot_schema, query_robot_schemas, RunsHarvest,
};
use cerulion_core::transport::TransportManager;
use cerulion_core::{CatalogReply, RunsReply, SchemaReply, TransportError};

pub use crate::protocol::DiscoveryState;

/// The bounded window netd's announce-space harvest waits for liveliness replies —
/// the same value the desk's transient path used (`topic_cmd::REMOTE_QUERY_GATHER_WINDOW`).
/// 500 ms comfortably covers a same-LAN round-trip. Kept a netd-local const so the
/// query plane does not depend on `cerulion_cli_engine` (the dependency runs the
/// other way).
const QUERY_HARVEST_WINDOW_MS: u64 = 500;

/// See [`QUERY_HARVEST_WINDOW_MS`] — the same value as a [`Duration`].
///
/// `pub(crate)` so the convergence-wait constants guard can DERIVE the daemon's worst-case first
/// answer from the terms that produce it instead of hard-coding 1250 ms beside a
/// comment claiming it tracks them.
pub(crate) const QUERY_HARVEST_WINDOW: Duration = Duration::from_millis(QUERY_HARVEST_WINDOW_MS);

/// How many harvest WINDOWS the cold-start budget spans. It sizes the
/// WINDOW, not the loop — the number of harvests actually run depends on how long
/// each harvest itself takes (see [`COLD_START_DISCOVERY_BUDGET`]'s
/// "Attempt COUNT" note).
const COLD_START_HARVEST_ATTEMPTS: u64 = 5;

/// The bounded window a plane that has NEVER gathered a non-empty answer
/// keeps re-running its harvest before it will answer a query empty. Spent ONCE per
/// DAEMON, never once per query.
///
/// # Derivation
///
/// netd's zenoh session is LAZY — the FIRST query OPENS it — and a just-opened
/// scouting session has not yet completed the multicast scout → TCP connect →
/// session establishment → liveliness/queryable exchange that makes a robot's
/// announce tokens and its `catalog` queryable visible. ONE harvest is only
/// `QUERY_HARVEST_WINDOW` (500 ms) and `query_announce_entries` breaks EARLY when
/// its reply channel closes, so on a cold session the first harvest routinely
/// observes nothing on an otherwise-healthy LAN and returns almost immediately —
/// the cold-start false "not found", measured repeatedly against a live robot
/// and healing on the very next invocation against the still-warm daemon.
///
/// `COLD_START_HARVEST_ATTEMPTS` × `QUERY_HARVEST_WINDOW` = **2.5 s**: five harvest
/// windows for that handshake to land in, which comfortably covers the measured LAN
/// convergence (a healing invocation was measured against a
/// daemon warm for under a second of network use), while keeping a genuinely
/// robot-less answer inside the interactive "a few seconds" bar — the same order as
/// the `topic list` discovery ladder's own 1.5 s gather ceiling plus its 1 s connect
/// bound.
///
/// # It is scoped to the DAEMON, not to the query
///
/// The window this bridges is the SESSION-ESTABLISHMENT window, which happens ONCE
/// per daemon lifetime — so the grace is scoped to the daemon. The first query that
/// spends the whole budget without reading anything LATCHES `grace_spent` on the
/// plane ([`GatewayQueryPlane::is_cold_start_grace_spent`]), and every later query
/// answers after ONE harvest.
///
/// Scoping it is not an optimisation, it is a correctness requirement. The `ever_settled`
/// bit alone latches only on a NON-EMPTY gather, so on a desk whose robots never
/// answer (robot powered off, on another VLAN, still booting) the plane never
/// settles — and unscoped, that means re-paying the FULL budget on EVERY query,
/// forever. `cerulion-vizd` holds ONE `NetdClient` behind a mutex across the whole
/// round trip, so a Studio sidebar refresh on a robot-less desk would block for seconds
/// AND stall every concurrent attach/detach behind the same lock.
///
/// A robot that appears LATER still settles the plane: every query — graced or not —
/// runs one FRESH harvest, and the first one that gathers something marks the plane
/// settled, from which point an empty answer is immediate AND authoritative.
///
/// # Worst-case wall (the guard test's own arithmetic)
///
/// The budget is checked AFTER each attempt, and a retry is PACED to `attempt_floor`
/// (= `QUERY_HARVEST_WINDOW`). So the worst case is an attempt that classifies at
/// `budget − ε`, plus that retry's pacing (one `QUERY_HARVEST_WINDOW`), plus one FULL
/// final attempt — a `QUERY_HARVEST_WINDOW` announce harvest followed by a
/// `QUERY_GATHER_WINDOW` per-robot GET (the per-robot GETs run CONCURRENTLY, so the
/// robot count does not enter). That is
/// `2500 + 500 + 500 + 250 = 3750 ms`, comfortably inside the client's 5 s
/// `ROUNDTRIP_TIMEOUT` (`crate::client`, deliberately not linked — it is
/// `pub(crate)` and a `pub` item may not link to it) — asserted by
/// `the_shipped_cold_start_budget_fits_inside_the_client_round_trip_timeout`.
/// Nothing here is unbounded.
///
/// # Attempt COUNT is not `COLD_START_HARVEST_ATTEMPTS`
///
/// On the paced grid an INSTANT harvest (the cold peerless shape —
/// `query_announce_entries` breaks out as soon as its reply channel closes) starts
/// one attempt every `QUERY_HARVEST_WINDOW`, so attempts land at 0, 0.5 … 2.5 s:
/// SIX harvests, five of them followed by a retry. A harvest that consumes its full
/// window runs FIVE. The accurate statement is "at least five harvests spanning
/// 2.5 s", not "exactly five attempts".
pub const COLD_START_DISCOVERY_BUDGET: Duration =
    Duration::from_millis(QUERY_HARVEST_WINDOW_MS * COLD_START_HARVEST_ATTEMPTS);

/// The bounded window ONE per-robot `catalog`/`schema` GET waits for its reply —
/// materially shorter than [`QUERY_HARVEST_WINDOW`] (matches the desk's
/// `topic_cmd::CATALOG_GATHER_WINDOW`). The per-robot GETs run CONCURRENTLY, so the
/// whole gather stays ~one harvest + one gather window regardless of robot count.
pub(crate) const QUERY_GATHER_WINDOW: Duration = Duration::from_millis(250);

/// What the plane should do after ONE harvest attempt. PURE — the whole
/// cold-start decision, oracle-tested, with no clock and no network.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GatherVerdict {
    /// Answer NOW, and the answer is AUTHORITATIVE: either this attempt gathered
    /// something, or a previous query on this plane already did (so discovery
    /// demonstrably works here and an empty answer means genuine absence).
    AnswerSettled,
    /// Answer NOW with the explicit "nothing has ever been discovered" marker — the
    /// plane has never gathered anything and the cold-start grace is unavailable
    /// (spent by an earlier query, or exhausted by this one). The answer is empty
    /// but it is NOT evidence of absence.
    AnswerColdStart,
    /// Re-run the harvest: this attempt found nothing, no earlier query ever found
    /// anything, the daemon's one cold-start grace is not yet spent, and budget
    /// remains within it.
    Retry,
}

/// THE cold-start decision. A plane answers immediately whenever it has
/// something to say (`found_any`) or has EVER had something to say (`ever_settled`);
/// only a plane that has never seen the network answer keeps trying, and only while
/// its ONE cold-start grace is unspent AND `elapsed < budget`. PURE — oracle-tested.
///
/// The `ever_settled` arm is load-bearing in BOTH directions: without it a warm desk
/// would re-pay the grace on every genuinely-absent topic; with it inverted a plane
/// would never retry at all.
///
/// The `grace_spent` arm is what bounds the cost to ONCE PER DAEMON. `ever_settled`
/// alone cannot: it latches only on a NON-EMPTY gather, so a desk whose robots never
/// answer never settles and would re-pay the full budget on EVERY query, forever (see
/// [`COLD_START_DISCOVERY_BUDGET`]). A `grace_spent` plane still runs ONE fresh harvest
/// per query, so a robot that appears later settles it on the next query.
///
/// The boundary is `elapsed < budget` (strictly less), so `elapsed == budget`
/// answers rather than starting an attempt that could only overshoot it.
pub fn classify_gather(
    found_any: bool,
    ever_settled: bool,
    grace_spent: bool,
    elapsed: Duration,
    budget: Duration,
) -> GatherVerdict {
    if found_any || ever_settled {
        GatherVerdict::AnswerSettled
    } else if !grace_spent && elapsed < budget {
        GatherVerdict::Retry
    } else {
        GatherVerdict::AnswerColdStart
    }
}

/// The plane's memory of the network — the two MONOTONE bits the
/// [`classify_gather`] decision turns on. Both only ever latch ON.
///
/// - `settled`: the network has ANSWERED this plane at least once (some query
///   gathered a non-empty result). A robot that later goes away does not un-settle
///   it, which is correct — discovery provably works on this machine, so a
///   subsequent empty answer really is absence.
/// - `grace_spent`: the daemon's ONE cold-start grace has been spent — some
///   query re-harvested for the whole budget and still read nothing. The grace
///   bridges the session-establishment window, which happens once per daemon, so it
///   is not re-paid per query.
#[derive(Debug, Default)]
struct DiscoveryMaturity {
    settled: AtomicBool,
    grace_spent: AtomicBool,
    /// When this plane FIRST tried to reach the network (its first gather).
    ///
    /// A plane that has never settled reports `now - first_attempt_at` to the
    /// consumer, which is what makes the client's wait a FIRST-CONTACT wait rather
    /// than a per-command tax: a desk whose robots never answer never settles, so a
    /// client keyed only on its own elapsed would spend the ceiling on every command
    /// forever — the exact shape the `grace_spent` latch removes one layer down.
    ///
    /// `OnceLock` because it is written once, on the first gather, and read on every
    /// query thereafter.
    first_attempt_at: std::sync::OnceLock<Instant>,
    /// The flood latch for the cold-start `warn!`.
    ///
    /// That line is emitted once per query and the client now issues up to ~13 of
    /// them per command, so an un-latched warn multiplied its rate by the poll count.
    /// It is a persistent-REGIME line (a robot-less desk emits it forever), which is
    /// exactly what `FailureRegimeLatch` — the repo's ONE shared flood-suppression
    /// machine — exists for: loud head, `debug!` repeats carrying the
    /// running count, a loud re-announcement at each decade, and a recovery `info!`
    /// when the plane finally settles. The latch is a DIAGNOSTIC, so a poisoned mutex
    /// must never wedge the query path (`lock_regime_latch` absorbs poison).
    cold_start_warn_latch:
        std::sync::Mutex<cerulion_core::transport::failure_regime_latch::FailureRegimeLatch>,
    /// How many harvests this plane has run, ever. Observable so the
    /// PRODUCTION-path test can prove that a `grace_spent` plane still HARVESTS —
    /// the claim that keeps a robot appearing after the grace discoverable. Without
    /// it that claim is only checkable against a scripted closure, and a "fix" that
    /// short-circuited a spent-grace plane into answering with no harvest at all
    /// would look identical from outside (same empty answer, same verdict, same
    /// wall) while permanently blinding the desk.
    harvests: AtomicU64,
}

impl DiscoveryMaturity {
    fn is_settled(&self) -> bool {
        self.settled.load(Ordering::Relaxed)
    }

    fn mark_settled(&self) {
        self.settled.store(true, Ordering::Relaxed);
    }

    fn is_grace_spent(&self) -> bool {
        self.grace_spent.load(Ordering::Relaxed)
    }

    fn mark_grace_spent(&self) {
        self.grace_spent.store(true, Ordering::Relaxed);
    }

    /// Record that this plane has begun trying (idempotent — the FIRST
    /// gather wins). Called at the top of every gather so the age is anchored to real
    /// network effort, not to plane construction (the session is lazy).
    fn note_attempt(&self) {
        let _ = self.first_attempt_at.set(Instant::now());
    }

    /// How long this plane has been running WITHOUT ever settling, or `None`
    /// once it HAS settled (a settled plane's age says nothing a consumer needs) or
    /// before its first attempt.
    fn unsettled_for(&self) -> Option<Duration> {
        if self.is_settled() {
            return None;
        }
        self.first_attempt_at.get().map(|t| t.elapsed())
    }

    fn note_harvest(&self) {
        self.harvests.fetch_add(1, Ordering::Relaxed);
    }

    fn harvests_run(&self) -> u64 {
        self.harvests.load(Ordering::Relaxed)
    }
}

/// What ONE harvest attempt saw. The gathered answer is what the consumer
/// gets; `robots_announced` and `error` exist ONLY to make the cold-start `warn!`
/// diagnostic — they are deliberately NOT consumer-visible, because the consumer's
/// conclusion is identical for every cause (nothing was read ⇒ no absence claim) and
/// an operator's is not.
struct HarvestAttempt<T> {
    /// The replies this attempt gathered (empty ⇒ nothing was read).
    gathered: Vec<T>,
    /// Did ANY robot answer the announce space this attempt?
    ///
    /// `Some(true)` with an empty `gathered` is the "a robot is there but its
    /// catalog/schema GET did not come back in time" shape — reachable, because
    /// `query_robot_catalogs` drops every robot that misses its window.
    ///
    /// `None` means this attempt NEVER OBSERVED the announce space, so it can say
    /// nothing either way. A SINGLE-ROBOT query is exactly that shape: it is one
    /// explicit-key GET with no announce harvest at all, so reporting `false` there
    /// would assert the alarming "no robot was discovered" discriminator on a query
    /// that structurally cannot know it.
    robots_announced: Option<bool>,
    /// The harvest's own failure, if it failed. Folded into "empty" for the RETRY
    /// decision (a transient failure on a just-opened session is exactly what the
    /// grace is for) but surfaced LOUDLY if the grace expires with it still failing,
    /// so a desk-local session fault is never reported as a quiet "nothing found".
    error: Option<String>,
    /// A peer ANSWERED and its answer could not be used, a wire skew
    /// or corruption. TERMINAL for this query: the grace exists to bridge a session
    /// that has not converged, and a peer that already replied has converged. It will
    /// re-serve the same bytes on every retry, so the loop stops rather than paying
    /// its serve cost again to re-read them.
    ///
    /// It does NOT make the answer authoritative — nothing was READ, so the reported
    /// state is still `NotConverged`. What it changes is only whether we keep asking.
    ///
    /// Deliberately a BOOL on this generic struct rather than the typed list of
    /// offenders: the reason text is verb-specific (only `runs` carries one on the
    /// wire), and the caller accumulates it in its own closure, where its type is
    /// known.
    terminal: bool,
    /// This attempt could not ask everyone it meant to — a fan-out GET
    /// worker PANICKED, so `gathered` is a PARTIAL account of what the network
    /// would have said.
    ///
    /// It demotes the RESPONSE MARKER to [`DiscoveryState::NotConverged`] and
    /// withholds the durable settled latch, and it changes NOTHING else: the
    /// [`classify_gather`] decision, and therefore the retry loop's wall
    /// behaviour, is byte-identical. That split is the seam this reuses rather
    /// than invents — the `AnswerSettled` arm already draws it, letting the
    /// daemon-wide latch and the per-response marker disagree.
    ///
    /// Why the marker and not the retry decision: retrying would re-run a harvest
    /// whose worker panicked deterministically, paying a fresh window per attempt
    /// to reproduce the same panic. What the pass must not do is license an
    /// absence, and the marker is exactly what licenses one — `Settled` means
    /// "an empty answer here is real absence", and a partial gather's emptiness
    /// about the robot it never asked is evidence of nothing.
    ///
    /// A BOOL for the same reason `terminal` is one: WHICH robots were lost is a
    /// verb-specific list the caller already logs at the site that knows it.
    incomplete: bool,
}

impl<T> HarvestAttempt<T> {
    /// A successful attempt that gathered `gathered`. `robots_announced` is `None`
    /// when this query shape does not observe the announce space at all.
    fn ok(gathered: Vec<T>, robots_announced: Option<bool>) -> Self {
        Self {
            gathered,
            robots_announced,
            error: None,
            terminal: false,
            incomplete: false,
        }
    }

    /// Mark this attempt INCOMPLETE — a fan-out GET worker PANICKED, so
    /// `gathered` is a partial account. See [`HarvestAttempt::incomplete`].
    fn incomplete_if(mut self, incomplete: bool) -> Self {
        self.incomplete = incomplete;
        self
    }

    /// Mark this attempt TERMINAL: a peer answered UNUSABLY, so a
    /// retry would re-read the same bytes at the same cost. See
    /// [`HarvestAttempt::terminal`].
    fn terminal_if(mut self, terminal: bool) -> Self {
        self.terminal = terminal;
        self
    }

    /// A FAILED attempt — nothing gathered, and the cause carried for the diagnostic.
    /// The announce space was NOT observed (the harvest is what would have observed
    /// it, and it failed), so the discriminator stays unknown.
    ///
    /// NOT terminal: a harvest that could not RUN is exactly the transient the grace
    /// exists for — the opposite of a peer that answered.
    fn failed(error: String) -> Self {
        Self {
            gathered: Vec::new(),
            robots_announced: None,
            error: Some(error),
            terminal: false,
            incomplete: false,
        }
    }
}

/// How the cold-start `warn!` renders the announce-space discriminator.
/// A single-robot query runs NO announce harvest, so it must report `not-observed`
/// rather than asserting the alarming "no robot was discovered". PURE —
/// oracle-tested.
///
/// The `None` text says no announce harvest COMPLETED, not that none RAN. Both
/// paths that produce `None` reach it differently — a single-robot GET never runs
/// one, while a fan-out whose harvest FAILED ran one that yielded nothing — and
/// "ran no announce harvest" would be false on the second. What is true on
/// both, and is the whole point of the label, is that no announce observation
/// completed, so the discriminator is unknown.
fn announce_observation_label(saw_a_robot: Option<bool>) -> &'static str {
    match saw_a_robot {
        Some(true) => "yes",
        Some(false) => "no",
        None => "not-observed (no announce harvest completed for this query)",
    }
}

/// Run `harvest` under the cold-start grace and report BOTH the gathered
/// answer and the [`DiscoveryState`] it was produced under.
///
/// Returns as soon as `harvest` yields a non-empty answer (latching `maturity`
/// settled), immediately if the plane was ALREADY settled OR its one cold-start
/// grace is already spent, or when `budget` is spent — whichever comes first. Every
/// retry is paced to occupy at least `attempt_floor`, so the loop is bounded in
/// ITERATIONS as well as wall time: the real `query_announce_entries` breaks early
/// when its reply channel closes (a cold session with no peers answers in
/// microseconds), and without the floor this loop would busy-spin for the whole
/// budget.
///
/// Exhausting the budget LATCHES `grace_spent`, so the retries are paid at most
/// once per daemon. A `grace_spent` plane still runs exactly ONE fresh harvest per
/// query, so a robot that shows up later is discovered on the next query and settles
/// the plane normally. See [`COLD_START_DISCOVERY_BUDGET`].
///
/// A cold-start answer is LOUD (`warn!`), not quiet, and it NAMES what the grace
/// saw — whether any robot was announced (or that this query shape never looked),
/// and the last harvest error if every attempt failed. Those are the two
/// discriminators an operator needs and the consumer must not have (see
/// [`HarvestAttempt`]).
///
/// Generic over the harvest so the whole grace is exercised by hermetic oracle
/// tests with no zenoh — the production planes pass a closure that runs the real
/// GET.
fn gather_with_cold_start_grace<T>(
    maturity: &DiscoveryMaturity,
    budget: Duration,
    attempt_floor: Duration,
    mut harvest: impl FnMut() -> HarvestAttempt<T>,
) -> (Vec<T>, DiscoveryState) {
    let start = Instant::now();
    // Anchor the plane's first-contact clock on its FIRST real gather.
    maturity.note_attempt();
    // Diagnostics accumulated ACROSS attempts — whether a robot was announced on any
    // attempt, and the most recent failure, are both facts about the whole grace, not
    // one attempt. `None` = no attempt ever OBSERVED the announce space.
    let mut saw_a_robot: Option<bool> = None;
    let mut last_error: Option<String> = None;
    loop {
        let attempt_start = Instant::now();
        let attempt = harvest();
        maturity.note_harvest();
        if let Some(seen) = attempt.robots_announced {
            saw_a_robot = Some(saw_a_robot.unwrap_or(false) || seen);
        }
        if attempt.error.is_some() {
            last_error = attempt.error;
        }
        let terminal = attempt.terminal;
        // A pass that lost a fan-out worker to a PANIC asked fewer robots
        // than it meant to, so what it gathered is a partial account. Carried per
        // ATTEMPT and never accumulated across the grace: a later attempt that ran
        // clean really did ask everyone, and its answer is the one returned.
        let incomplete = attempt.incomplete;
        let gathered = attempt.gathered;
        let found_any = !gathered.is_empty();
        match classify_gather(
            found_any,
            maturity.is_settled(),
            maturity.is_grace_spent(),
            start.elapsed(),
            budget,
        ) {
            GatherVerdict::AnswerSettled => {
                // The latch means "LAN DISCOVERY produced evidence on
                // this daemon", so ONLY announce-plane evidence may set it.
                //
                // A SINGLE-ROBOT query is one explicit-key GET with no announce
                // harvest at all (`robots_announced: None` — the same fact the
                // cold-start warn refuses to draw a conclusion from). Its answer
                // proves the zenoh SESSION works; it proves nothing about the
                // announce plane. Letting it settle the daemon-wide latch made a
                // later fan-out report `Settled` beside an empty list even when its
                // announce harvest found NOBODY — a confident "the LAN was searched
                // and nothing is there" backed by a GET to one robot somebody named.
                //
                // This is NOT specific to the runs verb: `query_catalog`
                // and `query_schema` have the identical single-robot shape;
                // the runs verb merely adds a third door.
                // The rule lives at the SHARED helper for that reason: keying it to one
                // verb would cover one door and leave the other two open.
                //
                // The asymmetry is real and deliberate. Announce evidence is STRICTLY
                // STRONGER: a fan-out that read something proves both planes work, so
                // it settles every verb including the single-robot ones (whose empty
                // means "that robot did not answer", a fact about the robot rather
                // than about discovery). Nothing is lost in practice — every
                // consumer's LAN gather (`topic echo`'s ladder, vizd's `discover`)
                // fans out — while a desk that ONLY ever addresses named robots now
                // reports `NotConverged` instead of vouching for a plane it
                // never exercised.
                //
                // WHAT THE RETURNED MARKER MEANS HERE, stated because the latch and
                // the marker are now deliberately allowed to disagree. The latch is
                // the DURABLE, daemon-wide claim ("discovery works on this machine,
                // so a future empty answer is real absence"); the marker describes
                // THIS RESPONSE. Only the latch carries forward, which is why only it
                // is gated on announce evidence.
                //
                // The safety property that gating buys is INTACT, and it is structural
                // rather than argued: `classify_gather` reaches this arm only on
                // `found_any || ever_settled`, so an EMPTY answer here IMPLIES
                // `ever_settled`, which only announce evidence can set. An empty
                // answer therefore never reports `Settled` without announce evidence —
                // the `debug_assert!` below executes that claim, and
                // `an_empty_answer_never_reports_settled_without_announce_evidence`
                // pins it. Since `DiscoveryState::Settled` licenses exactly one thing,
                // reading an EMPTY answer as absence, nothing a direct GET can do
                // licenses an absence claim.
                //
                // Reporting `NotConverged` on a NON-empty direct-addressed answer was
                // considered and REFUSED. It is inert where the marker is designed to
                // be read — `ConvergenceWait::decide` proceeds on
                // `Settled || !answer_empty`, and this arm's answer is non-empty by
                // construction — and on the ONE seam where it is not inert it is a
                // regression: `cerulion-vizd` forwards this marker into its attach
                // `RetryHint` and the client passes `answer_empty: true` unconditionally
                // (the hint's PRESENCE is that seam's emptiness test), so a robot that
                // ANSWERED and does not serve the requested topic would stop being an
                // instant, correct "not there" and become the full convergence-wait ceiling
                // followed by UNKNOWN — on every mistyped topic against a named robot.
                //
                // A pass whose FAN-OUT lost a worker to a PANIC did not ask
                // everyone, so it may neither report `Settled` NOR set the daemon-wide
                // latch.
                //
                // That is NOT the refusal directly above running backwards, and the
                // difference is the TRIGGER rather than the shape. The refused change
                // fires on every direct-addressed non-empty answer — the routine path
                // — so it would charge the ceiling to every mistyped topic
                // against a named robot. This one fires only when a GET worker
                // UNWOUND, which on a healthy binary never happens at all. The same
                // vizd `RetryHint` cost is then paid on a pass that has already proved
                // this process is broken, where waiting is the right answer to "we
                // could not ask everyone" and a confident absence is not.
                //
                // The marker first: `Settled` licenses exactly one reading — "this
                // empty answer is absence" — and the consumer's absence claim is
                // about a SPECIFIC thing missing from the list, which is exactly
                // what a robot nobody asked would be missing from. It is NOT inert
                // there, which is the whole reason it is worth demoting: the
                // schema fan-out's `classify_unserved_schema` turns a `Settled`
                // marker straight into "no robot serves this type", so a panicked
                // worker would convert this binary's own bug into a confident
                // claim about the network. `NotConverged` makes it the explicit
                // UNKNOWN that seam already has a name for.
                //
                // The latch second, and for a different reason: it is the DURABLE
                // "LAN discovery produced evidence on this daemon" claim, and a
                // pass that dropped a thread mid-fan-out is not the clean
                // demonstration that claim is made of. Withholding it costs at
                // most one more grace on the next query, which is the safe
                // direction.
                let complete = !incomplete;
                let announce_evidence = saw_a_robot == Some(true);
                if !complete {
                    tracing::warn!(
                        "cerulion-netd query plane: a fan-out GET worker PANICKED, so this \
                         pass asked fewer robots than it meant to — answering with the \
                         explicit 'not converged' marker (this is NOT a claim that the \
                         requested thing is absent). The panicked robots are named at \
                         ERROR by the gather itself."
                    );
                    return (gathered, DiscoveryState::NotConverged);
                }
                if found_any && announce_evidence {
                    maturity.mark_settled();
                    // The regime is over — report it ONCE, carrying what the
                    // operator missed while it was open.
                    if let Some(suppressed) =
                        cerulion_core::transport::failure_regime_latch::lock_regime_latch(
                            &maturity.cold_start_warn_latch,
                        )
                        .on_success()
                    {
                        tracing::info!(
                            suppressed_count = suppressed,
                            "cerulion-netd query plane: discovery CONVERGED — a robot \
                             answered, so empty answers are now authoritative"
                        );
                    }
                }
                debug_assert!(
                    !gathered.is_empty() || maturity.is_settled(),
                    "an EMPTY answer must never report Settled unless the plane's \
                     announce-evidence latch is set — Settled licenses exactly one \
                     reading, and that reading is 'this empty answer is absence'"
                );
                return (gathered, DiscoveryState::Settled);
            }
            GatherVerdict::AnswerColdStart => {
                // The grace bridges the daemon's session-establishment window, so
                // it is spent ONCE. Latch it here — every later query then answers
                // after ONE fresh harvest instead of re-paying the whole budget.
                maturity.mark_grace_spent();
                // LOUD, and it names the cause an operator can act on. `saw_a_robot`
                // separates "nothing is out there" from "a robot is there but never
                // served" — and from "this query never looked" (a single-robot GET
                // runs no announce harvest, so it must not assert either); `last_error`
                // separates all of those from a desk-local session fault, which would
                // otherwise steer the user at the robot's power switch.
                // Behind the shared flood latch. A robot-less desk emits
                // this on EVERY query forever, and the client now issues ~13 queries
                // per command — an un-latched line multiplied by the poll count is
                // the disk-fill class. Loud head, `debug!` repeats, a loud
                // re-announcement at each decade, recovery on settle.
                let decision = cerulion_core::transport::failure_regime_latch::lock_regime_latch(
                    &maturity.cold_start_warn_latch,
                )
                .on_failure();
                let budget_ms = budget.as_millis() as u64;
                let robots_announced = announce_observation_label(saw_a_robot);
                let last_error_s = last_error.as_deref().unwrap_or("");
                use cerulion_core::transport::failure_regime_latch::RegimeDecision;
                match decision {
                    RegimeDecision::Loud => tracing::warn!(
                        budget_ms,
                        robots_announced,
                        last_error = last_error_s,
                        "cerulion-netd query plane: discovery did NOT converge within the \
                         cold-start grace — answering with the explicit 'not converged' marker \
                         (this is NOT a claim that the requested thing is absent)"
                    ),
                    RegimeDecision::Suppressed { suppressed } => tracing::debug!(
                        budget_ms,
                        robots_announced,
                        last_error = last_error_s,
                        suppressed,
                        "cerulion-netd query plane: discovery still not converged (suppressed)"
                    ),
                    RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
                        budget_ms,
                        robots_announced,
                        last_error = last_error_s,
                        total_failures = total,
                        suppressed,
                        "cerulion-netd query plane: discovery STILL has not converged — \
                         nothing on the network has answered this daemon yet"
                    ),
                }
                return (gathered, DiscoveryState::NotConverged);
            }
            GatherVerdict::Retry => {
                // A peer ANSWERED and its answer was unusable. Retrying
                // re-issues the GET and reads the same bytes, which on this verb costs
                // the ROBOT a fresh iceoryx2 reader node each time (the ~620 ms serve)
                // — so the grace is spent on a condition it cannot bridge.
                //
                // The verdict is unchanged (`NotConverged`: nothing was READ, so no
                // absence is licensed); what stops is the ASKING. The remedy is a
                // redeploy, and it is named LOUDLY at the decode site, once per robot.
                //
                // Deliberately NOT `mark_grace_spent()`: the daemon's one cold-start
                // grace bridges SESSION ESTABLISHMENT, and a skewed peer says nothing
                // about that — burning it here would leave the next genuinely-cold
                // query answering after a single harvest.
                if terminal {
                    return (gathered, DiscoveryState::NotConverged);
                }
                // Pace the retry so a harvest that returns early (the cold-session
                // shape: the liveliness reply channel closes at once) cannot spin.
                let spent = attempt_start.elapsed();
                if let Some(rest) = attempt_floor.checked_sub(spent) {
                    std::thread::sleep(rest);
                }
            }
        }
    }
}

/// The outcome of a catalog query — the gathered replies PLUS whether netd
/// had completed a discovery pass when it answered. An empty `catalogs` is a claim
/// of ABSENCE only under [`DiscoveryState::Settled`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CatalogGather {
    /// Every decoded catalog netd gathered (one per answering robot).
    pub catalogs: Vec<CatalogReply>,
    /// Whether netd had completed a discovery pass when it answered.
    pub discovery: DiscoveryState,
    /// How long the ANSWERING daemon's query plane has been running without
    /// ever settling, or `None` if it has settled / cannot report (an older daemon).
    /// `None` is UNKNOWN, never a positive claim — see
    /// [`ConvergenceWait::decide`](crate::convergence::ConvergenceWait::decide).
    pub unsettled_for: Option<Duration>,
}

/// The outcome of a runs query; see [`CatalogGather`].
///
/// The two verdicts it carries answer DIFFERENT questions and neither substitutes
/// for the other. [`Self::discovery`] is about THIS DAEMON's LAN session ("has
/// anything on the network ever answered me?"); each reply's own
/// `RunsCompleteness` is about THAT ROBOT's registry gather ("did I hear from every
/// live run writer on my machine?"). A settled plane can carry an incomplete reply,
/// and an unconverged plane says nothing about any robot at all.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RunsGather {
    /// Every decoded runs reply netd gathered — one per ANSWERING robot, which is a
    /// possibly-STRICT SUBSET of the robots asked. A robot that contributed no reply
    /// is UNKNOWN, never idle; nothing here is synthesized on its behalf.
    pub replies: Vec<RunsReply>,
    /// Robots that ANSWERED with something this binary could not use.
    ///
    /// A different kind of absent from a robot that simply did not answer, and the
    /// difference is the REMEDY: this one will re-serve the same unusable bytes
    /// forever, so a consumer renders it as "redeploy that robot", never as "still
    /// looking". netd stops retrying the moment one appears.
    pub unusable: Vec<UnusableRunsAnswer>,
    /// Robots that were ASKED and did not answer at all.
    ///
    /// **This is what makes [`Self::replies`] readable as COVERAGE rather than
    /// merely as content.** A consumer holding replies and unusable answers alone
    /// cannot tell a LAN where everyone answered from one where half the robots
    /// stayed silent — both look like a complete list — so folding either into
    /// "these are the runs" produces a SETTLED ABSENCE about machines nobody
    /// heard from. That is the confident-empty class already killed at the
    /// discovery latch, arriving one layer up through the coverage door.
    ///
    /// Non-empty ⇒ the answer is a strict subset of the question, whatever the
    /// discovery latch says. Its remedy is neither of its siblings': such a robot
    /// may answer later, or may never (a binary predating the verb, a Strict
    /// ingress-only gateway with no query surface) — so a consumer waits or
    /// upgrades, and either way does not claim to know what it is running.
    pub silent: Vec<String>,
    /// Whether netd had completed a discovery pass when it answered.
    pub discovery: DiscoveryState,
    /// See [`CatalogGather::unsettled_for`].
    pub unsettled_for: Option<Duration>,
}

/// The ANSWER half of a [`RunsGather`]: every reply netd got, with the
/// gather's own metadata ([`RunsGather::discovery`], [`RunsGather::unsettled_for`])
/// left out. What [`NetdClient::query_runs`](crate::client::NetdClient::query_runs)
/// and its `_no_respawn` sibling return.
///
/// # Why a type rather than a bare `Vec`
///
/// The catalog and schema verbs' bare forms return a plain `Vec` because the only
/// thing they drop IS metadata: a robot whose reply could not be decoded is simply
/// absent from their list, so there is nothing else to carry. The runs verb
/// deliberately refuses to collapse that case — a robot that ANSWERED UNUSABLY has a
/// different REMEDY (redeploy it) from one that never answered (it may yet answer, or
/// it cannot be asked at all) — so returning a bare `Vec<RunsReply>` would destroy the
/// one distinction this verb was extended to draw, handing the caller `Ok([])` for a
/// wire-skewed robot that it could then only report as "nothing answered".
///
/// So the split is by KIND, not by convenience: metadata is what the bare verb hides,
/// and [`Self::unusable`] is an answer.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RunsAnswer {
    /// See [`RunsGather::replies`].
    pub replies: Vec<RunsReply>,
    /// See [`RunsGather::unusable`].
    pub unusable: Vec<UnusableRunsAnswer>,
    /// See [`RunsGather::silent`].
    ///
    /// On the ANSWER half for the same reason `unusable` is: it is not metadata
    /// about the gather, it is part of what the gather LEARNED. Dropping it here
    /// would hand a caller `replies` with no way to tell a complete answer from a
    /// third of one — the coverage half of the confident-empty class.
    pub silent: Vec<String>,
}

impl From<RunsGather> for RunsAnswer {
    fn from(gather: RunsGather) -> Self {
        Self {
            replies: gather.replies,
            unusable: gather.unusable,
            silent: gather.silent,
        }
    }
}

/// The outcome of a schema query — see [`CatalogGather`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SchemaGather {
    /// Every decoded schema reply netd gathered (one per answering robot).
    pub replies: Vec<SchemaReply>,
    /// Whether netd had completed a discovery pass when it answered.
    pub discovery: DiscoveryState,
    /// See [`CatalogGather::unsettled_for`].
    pub unsettled_for: Option<Duration>,
}

/// An error from a [`QueryPlane`] operation — the daemon maps it to a structured
/// protocol [`Response::error`](crate::protocol::Response::error), which the
/// consumer treats as "netd could not run the query" and LOUDLY degrades to its own
/// transient session (the fallback discipline). Every arm is LOUD.
#[derive(Debug)]
pub enum QueryError {
    /// The daemon is not network-configured (`CERULION_NETD_NETWORK=off`, or a
    /// local-only transport) — it has no `NetworkManager`, so it cannot reach any
    /// robot. The consumer degrades to its own transient discovery session.
    NoNetwork,
    /// The shared zenoh session could not be opened for the query. Carries the
    /// underlying transport cause (BOXED — `TransportError` is a large enum, and a
    /// bare `Result<_, QueryError>` would trip clippy's `result_large_err`).
    Session {
        /// The underlying transport error opening the session.
        source: Box<TransportError>,
    },
}

impl std::fmt::Display for QueryError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            QueryError::NoNetwork => write!(
                f,
                // VERB-AGNOSTIC: this error is returned by every arm of the plane,
                // and enumerating them (it read "a catalog/schema query") goes stale
                // the moment one is added — which the `runs` verb did,
                // leaving the message quietly wrong about which query had failed.
                // WHICH verb the caller asked for is named by the caller's own
                // wrapper (`daemon::query_runs` prefixes "runs query failed:"), so
                // saying it here is a second place to keep correct.
                "this cerulion-netd instance is not network-configured (CERULION_NETD_NETWORK=off \
                 or a local-only transport) — it cannot run any network query"
            ),
            QueryError::Session { source } => {
                write!(
                    f,
                    "failed to open the shared zenoh session for the query: {source}"
                )
            }
        }
    }
}

impl std::error::Error for QueryError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            QueryError::NoNetwork => None,
            QueryError::Session { source } => Some(source.as_ref()),
        }
    }
}

/// The seam between netd's UDS control server and the machine's ONE zenoh session's
/// query surface. The daemon calls [`Self::query_catalog`] for a `query_catalog`
/// verb and [`Self::query_schema`] for a `query_schema` verb. `Send + Sync`: shared
/// across the daemon's per-connection threads. A query is a STATELESS one-shot — the
/// plane holds no per-query state (unlike the mirror/egress planes).
pub trait QueryPlane: Send + Sync {
    /// Query the LAN topic catalog. `robot: Some` scopes it to ONE robot's catalog;
    /// `None` harvests every announcing robot and gathers all their catalogs.
    /// Returns every decoded [`CatalogReply`] plus the [`DiscoveryState`] it was
    /// gathered under — an EMPTY vec means "no robot answered", and it is a claim of
    /// ABSENCE only under [`DiscoveryState::Settled`]. Either way the desk
    /// does NOT fall back; only a [`QueryError`] (netd could not run the query at
    /// all) triggers the desk's transient-session fallback. A refusal reply
    /// (`CatalogReply.error`) is passed through verbatim inside the vec.
    fn query_catalog(&self, robot: Option<&str>) -> Result<CatalogGather, QueryError>;

    /// Fetch a remote type's `.msg`/YAML closure. `robot: Some` scopes the GET to
    /// ONE robot; `None` asks every announcing robot. `requested` is a qualified
    /// `pkg/Type` OR a package-less bare `Name`. Returns every decoded
    /// [`SchemaReply`] (found / not-found / refused) — the desk picks the first with
    /// non-empty `docs` — plus the [`DiscoveryState`] it was gathered under. An
    /// EMPTY vec means no robot answered; a [`QueryError`] is a netd-transport
    /// failure (the desk falls back).
    fn query_schema(
        &self,
        robot: Option<&str>,
        requested: &str,
    ) -> Result<SchemaGather, QueryError>;

    /// Ask which runs are LIVE on a robot right now. `robot: Some`
    /// scopes the GET to ONE robot; `None` asks every announcing robot. Returns every
    /// DECODABLE reply plus the [`DiscoveryState`] it was gathered under — a robot
    /// that did not answer contributes NOTHING (see [`RunsGather::replies`]), and a
    /// [`QueryError`] means netd could not run the query at all.
    ///
    /// **This verb's serve is expensive on the ROBOT** (a fresh iceoryx2 reader node
    /// per call, plus a gather window when a live registry writer must be heard), so
    /// a consumer fetches once per `run_id` and must never fold it into a poll —
    /// see [`crate::protocol::Request::QueryRuns`].
    fn query_runs(&self, robot: Option<&str>) -> Result<RunsGather, QueryError>;
}

/// A no-op query plane for a mirror-ONLY daemon (no network query surface) —
/// [`crate::daemon::start`] injects it so a mirror-only daemon still answers the
/// query verbs (with an explicit [`QueryError::NoNetwork`] refusal) rather than
/// crashing. The production [`GatewayQueryPlane`] is what `main.rs` builds via
/// [`crate::daemon::start_with_planes`].
pub struct NoopQueryPlane;

impl QueryPlane for NoopQueryPlane {
    fn query_catalog(&self, _robot: Option<&str>) -> Result<CatalogGather, QueryError> {
        Err(QueryError::NoNetwork)
    }
    fn query_schema(
        &self,
        _robot: Option<&str>,
        _requested: &str,
    ) -> Result<SchemaGather, QueryError> {
        Err(QueryError::NoNetwork)
    }
    fn query_runs(&self, _robot: Option<&str>) -> Result<RunsGather, QueryError> {
        Err(QueryError::NoNetwork)
    }
}

/// The production query plane: runs the catalog/schema GETs over the SAME
/// network-configured [`TransportManager`] the mirror + egress planes own (Principle
/// #8 — one zenoh session for the whole machine). `main.rs` builds this from the
/// SAME manager. The session opens LAZILY inside the first GET, so a netd that never
/// serves a query opens no session.
pub struct GatewayQueryPlane {
    manager: std::sync::Arc<TransportManager>,
    /// Has the network EVER answered this plane? Shared across every query
    /// (catalog and schema, fan-out and single-robot) — one daemon, one session, one
    /// "discovery works here" fact.
    maturity: DiscoveryMaturity,
    /// The cold-start grace this plane applies. Production is
    /// [`COLD_START_DISCOVERY_BUDGET`]; the injectable constructor exists so the
    /// production-path tests can drive both the graced and the settled arms without
    /// spending the shipped ceiling on every assertion.
    cold_start_budget: Duration,
}

impl GatewayQueryPlane {
    /// Wrap the shared (network-configured) transport manager, with the shipped
    /// cold-start grace ([`COLD_START_DISCOVERY_BUDGET`]).
    pub fn new(manager: std::sync::Arc<TransportManager>) -> Self {
        Self::with_cold_start_budget(manager, COLD_START_DISCOVERY_BUDGET)
    }

    /// [`Self::new`] with an explicit cold-start grace. The production entry
    /// point is [`Self::new`]; this exists so a test can exercise the real plane
    /// (real session, real GETs) without paying the shipped ceiling on every arm.
    /// A ZERO budget still runs exactly one harvest — the grace only ever adds
    /// RETRIES, never removes the first attempt.
    pub fn with_cold_start_budget(
        manager: std::sync::Arc<TransportManager>,
        cold_start_budget: Duration,
    ) -> Self {
        Self {
            manager,
            maturity: DiscoveryMaturity::default(),
            cold_start_budget,
        }
    }

    /// The shared transport manager (diagnostics / the daemon's shutdown drop).
    pub fn manager(&self) -> &std::sync::Arc<TransportManager> {
        &self.manager
    }

    /// Principle #3: has this plane ever gathered a non-empty answer from
    /// the network? Observable so a test — and a future `status` field — can tell a
    /// cold plane from a settled one without inferring it from timings.
    pub fn is_discovery_settled(&self) -> bool {
        self.maturity.is_settled()
    }

    /// Principle #3: has this plane already spent its ONE cold-start
    /// grace? Observable for the same reason as [`Self::is_discovery_settled`] — a
    /// test (and an operator, via a future `status` field) must be able to tell a
    /// plane that will still retry from one that answers after a single harvest,
    /// without inferring it from wall-clock timings. See
    /// [`COLD_START_DISCOVERY_BUDGET`].
    pub fn is_cold_start_grace_spent(&self) -> bool {
        self.maturity.is_grace_spent()
    }

    /// Principle #3: how many harvests this plane has run, ever.
    ///
    /// The observable that makes "a plane whose grace is spent still runs ONE FRESH
    /// harvest per query" checkable on the PRODUCTION path — that is what keeps a
    /// robot appearing after the grace discoverable, and from outside a plane that
    /// short-circuited into answering with no harvest is indistinguishable (identical
    /// empty answer, identical verdict, identical wall) from one that harvested and
    /// found nothing.
    pub fn harvests_run(&self) -> u64 {
        self.maturity.harvests_run()
    }

    /// Principle #3: how long this plane has been running WITHOUT ever
    /// settling — the quantity that rides out as `plane_unsettled_ms` and caps the
    /// consumer's first-contact wait. `None` before the first gather, and `None`
    /// once the plane has SETTLED (a settled plane's age says nothing a consumer
    /// needs).
    ///
    /// Observable for the same reason as [`Self::harvests_run`], and the
    /// need is concrete: `first_attempt_at` is a `OnceLock` that is only ever `.get()`,
    /// so deleting its single `set` call raises no lint, returns `None` forever, makes
    /// the consumer's cap vacuously true, and restores the per-command tax the gate
    /// exists to prevent — with every other test green, because they
    /// hand-write the number through a scripted fake daemon.
    pub fn plane_unsettled_for(&self) -> Option<Duration> {
        self.maturity.unsettled_for()
    }
}

/// Dedup + sort the DISTINCT announcing-robot identities from the raw announce
/// entries (the second `Option<topic>` half is dropped — we key on the identity).
/// Deterministic (a `BTreeSet`). Pure — oracle-tested.
fn robots_from_entries(entries: Vec<(String, Option<String>)>) -> Vec<String> {
    entries
        .into_iter()
        .map(|(robot, _topic)| robot)
        .collect::<BTreeSet<String>>()
        .into_iter()
        .collect()
}

impl QueryPlane for GatewayQueryPlane {
    fn query_catalog(&self, robot: Option<&str>) -> Result<CatalogGather, QueryError> {
        // The shared session — its type (`zenoh::Session`) stays INFERRED (never
        // named), so `zenoh` need not be a direct netd dependency. `net.session()`
        // returns the ONE cached session (opens lazily on the first network op).
        let net = self.manager.network().ok_or(QueryError::NoNetwork)?;
        let session = net.session().map_err(|e| QueryError::Session {
            source: Box::new(e),
        })?;
        // EVERY attempt runs under the cold-start grace, so a plane whose
        // session has not yet converged re-tries instead of reporting a confident
        // empty. An announce-harvest ERROR folds into the same empty-and-retry arm:
        // a transient failure on a just-opened session is exactly the shape the
        // grace exists for, and it stays bounded by the same budget.
        let (catalogs, discovery) = gather_with_cold_start_grace(
            &self.maturity,
            self.cold_start_budget,
            QUERY_HARVEST_WINDOW,
            || match robot {
                // A single-robot query: one explicit-key GET (vizd's resolve). There
                // is no announce step here, so a reply IS the only evidence — and the
                // announce discriminator is UNOBSERVED (`None`), never a claimed "no
                // robot was discovered".
                Some(robot) => HarvestAttempt::ok(
                    query_robot_catalog(session, robot, QUERY_GATHER_WINDOW)
                        .into_iter()
                        .collect(),
                    None,
                ),
                // The LAN gather: harvest the announcing robots, then GET each catalog.
                None => {
                    let entries = match query_announce_entries(session, QUERY_HARVEST_WINDOW) {
                        Ok(entries) => entries,
                        Err(e) => return HarvestAttempt::failed(e.to_string()),
                    };
                    let robots = robots_from_entries(entries);
                    if robots.is_empty() {
                        return HarvestAttempt::ok(Vec::new(), Some(false));
                    }
                    // A robot ANNOUNCED. Its catalog GET may still miss its window —
                    // `query_robot_catalogs` drops non-answering robots — so the
                    // result can be empty with `robots_announced: Some(true)`. That
                    // stays NOT-converged for the consumer (nothing was read, so no
                    // absence claim is licensed) while the operator's log says a robot
                    // WAS seen, which is a completely different remedy.
                    let gathered = query_robot_catalogs(session, &robots, QUERY_GATHER_WINDOW);
                    let incomplete = !gathered.is_complete();
                    HarvestAttempt::ok(gathered.replies, Some(true)).incomplete_if(incomplete)
                }
            },
        );
        Ok(CatalogGather {
            catalogs,
            discovery,
            unsettled_for: self.maturity.unsettled_for(),
        })
    }

    fn query_schema(
        &self,
        robot: Option<&str>,
        requested: &str,
    ) -> Result<SchemaGather, QueryError> {
        let net = self.manager.network().ok_or(QueryError::NoNetwork)?;
        let session = net.session().map_err(|e| QueryError::Session {
            source: Box::new(e),
        })?;
        let (replies, discovery) = gather_with_cold_start_grace(
            &self.maturity,
            self.cold_start_budget,
            QUERY_HARVEST_WINDOW,
            || match robot {
                // Single-robot: one explicit-key GET, no announce harvest ⇒ the
                // announce discriminator is UNOBSERVED.
                Some(robot) => HarvestAttempt::ok(
                    query_robot_schema(session, robot, requested, QUERY_GATHER_WINDOW)
                        .into_iter()
                        .collect(),
                    None,
                ),
                None => {
                    let entries = match query_announce_entries(session, QUERY_HARVEST_WINDOW) {
                        Ok(entries) => entries,
                        Err(e) => return HarvestAttempt::failed(e.to_string()),
                    };
                    let robots = robots_from_entries(entries);
                    if robots.is_empty() {
                        return HarvestAttempt::ok(Vec::new(), Some(false));
                    }
                    let gathered =
                        query_robot_schemas(session, &robots, requested, QUERY_GATHER_WINDOW);
                    let incomplete = !gathered.is_complete();
                    HarvestAttempt::ok(gathered.replies, Some(true)).incomplete_if(incomplete)
                }
            },
        );
        Ok(SchemaGather {
            replies,
            discovery,
            unsettled_for: self.maturity.unsettled_for(),
        })
    }

    fn query_runs(&self, robot: Option<&str>) -> Result<RunsGather, QueryError> {
        let net = self.manager.network().ok_or(QueryError::NoNetwork)?;
        let session = net.session().map_err(|e| QueryError::Session {
            source: Box::new(e),
        })?;
        // The SAME `self.maturity` the catalog and schema verbs use, on
        // purpose. What it remembers is a fact about THIS DAEMON's session — has the
        // network ever answered it — which is a property of the LAN and the zenoh
        // plane, not of a verb. Giving `runs` its own would make a desk that has been
        // talking to robots for an hour re-pay the whole cold-start grace on its
        // first runs query, and would let two verbs on one daemon disagree about
        // whether discovery works here. Sharing also runs the other way: a runs
        // query that gathers something SETTLES the plane for the catalog, which is
        // the correct conclusion since it is the same session that answered.
        //
        // The retry loop costs the ROBOT nothing extra, which is what makes it
        // affordable on a verb this expensive to serve: `classify_gather` returns
        // `AnswerSettled` the moment anything is gathered, so an ANSWERING robot is
        // GET exactly once. Retries happen only when NOTHING came back — i.e. when
        // there is no serving robot to burden.
        // The robots that ANSWERED UNUSABLY on the LAST attempt. The
        // closure ASSIGNS rather than appends, so a retry cannot double-report; in
        // practice at most one attempt produces any (an unusable answer is terminal,
        // and a usable one from another robot ends the loop by finding something).
        let mut unusable: Vec<UnusableRunsAnswer> = Vec::new();
        // The robots that were ASKED and stayed silent. ASSIGNED per
        // attempt for the same reason `unusable` is — a retry re-asks the same set,
        // so appending would report one silent robot N times and turn a retry
        // budget into a coverage claim.
        let mut silent: Vec<String> = Vec::new();
        let (replies, discovery) = gather_with_cold_start_grace(
            &self.maturity,
            self.cold_start_budget,
            QUERY_HARVEST_WINDOW,
            || match robot {
                // Single-robot: one explicit-key GET, no announce harvest ⇒ the
                // announce discriminator is UNOBSERVED (never a claimed "no robot
                // was discovered" from a query that never looked).
                Some(robot) => {
                    // The SAME fold the fan-out uses — one judgement about what an
                    // unusable answer is, in one place (`RunsHarvest::absorb`).
                    let mut harvest = RunsHarvest::default();
                    harvest.absorb(robot, query_robot_runs(session, robot, QUERY_GATHER_WINDOW));
                    let terminal = harvest.is_terminal();
                    unusable = harvest.unusable;
                    // A named robot that did not answer is silent COVERAGE, even
                    // though this arm consulted no announce plane: the desk asked
                    // for exactly one machine and did not hear from it, which is a
                    // fact about the answer regardless of how the name was chosen.
                    silent = harvest.silent;
                    HarvestAttempt::ok(harvest.replies, None).terminal_if(terminal)
                }
                None => {
                    let entries = match query_announce_entries(session, QUERY_HARVEST_WINDOW) {
                        Ok(entries) => entries,
                        Err(e) => return HarvestAttempt::failed(e.to_string()),
                    };
                    let robots = robots_from_entries(entries);
                    if robots.is_empty() {
                        unusable.clear();
                        // Nothing was ANNOUNCED, so nothing was asked and nothing
                        // is missing — an empty `silent` here is a true statement
                        // about coverage, not an omission.
                        silent.clear();
                        return HarvestAttempt::ok(Vec::new(), Some(false));
                    }
                    let harvest = query_robot_runs_all(session, &robots, QUERY_GATHER_WINDOW);
                    let terminal = harvest.is_terminal();
                    unusable = harvest.unusable;
                    silent = harvest.silent;
                    HarvestAttempt::ok(harvest.replies, Some(true)).terminal_if(terminal)
                }
            },
        );
        Ok(RunsGather {
            replies,
            unusable,
            silent,
            discovery,
            unsettled_for: self.maturity.unsettled_for(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::sync::atomic::AtomicUsize;

    /// The cold-start decision against a HAND-WRITTEN verdict table. Every
    /// row states the situation in words first, so the table is an oracle rather
    /// than a transcription of the implementation.
    #[test]
    fn classify_gather_answers_when_it_can_and_only_retries_a_never_settled_plane() {
        let budget = Duration::from_millis(1000);
        // (found_any, ever_settled, grace_spent, elapsed_ms, expected)
        let oracle = [
            // Found something on this attempt ⇒ answer, whatever the clock or the
            // spent grace says.
            (true, false, false, 0, GatherVerdict::AnswerSettled),
            (true, false, false, 999, GatherVerdict::AnswerSettled),
            (true, false, false, 5_000, GatherVerdict::AnswerSettled),
            (true, true, false, 0, GatherVerdict::AnswerSettled),
            // A plane whose grace is SPENT still reports what it just found — the
            // latch suppresses RETRIES, never an answer.
            (true, false, true, 0, GatherVerdict::AnswerSettled),
            // Found nothing, but an EARLIER query on this plane did ⇒ the empty
            // answer is authoritative and IMMEDIATE (a warm desk must not re-pay the
            // grace for every genuinely-absent topic).
            (false, true, false, 0, GatherVerdict::AnswerSettled),
            (false, true, false, 5_000, GatherVerdict::AnswerSettled),
            // Never settled, nothing found, grace unspent, budget left ⇒ keep trying.
            (false, false, false, 0, GatherVerdict::Retry),
            (false, false, false, 1, GatherVerdict::Retry),
            (false, false, false, 999, GatherVerdict::Retry),
            // THE LATCH: the SAME rows with the daemon's one grace already spent must
            // NOT retry. This is the never-settling plane on its second query — without
            // the latch it re-pays the whole budget on every query, forever.
            (false, false, true, 0, GatherVerdict::AnswerColdStart),
            (false, false, true, 1, GatherVerdict::AnswerColdStart),
            (false, false, true, 999, GatherVerdict::AnswerColdStart),
            // The boundary is `elapsed < budget`: AT the budget we answer.
            (false, false, false, 1_000, GatherVerdict::AnswerColdStart),
            (false, false, false, 1_001, GatherVerdict::AnswerColdStart),
            (false, false, false, 60_000, GatherVerdict::AnswerColdStart),
        ];
        for (found_any, ever_settled, grace_spent, elapsed_ms, expected) in oracle {
            assert_eq!(
                classify_gather(
                    found_any,
                    ever_settled,
                    grace_spent,
                    Duration::from_millis(elapsed_ms),
                    budget
                ),
                expected,
                "found_any={found_any} ever_settled={ever_settled} grace_spent={grace_spent} \
                 elapsed={elapsed_ms}ms"
            );
        }
    }

    #[test]
    fn a_zero_budget_never_retries_but_still_answers_cold_start() {
        // The degenerate budget: no retries are possible, yet the plane must still
        // report EXPLICITLY that it has discovered nothing (a zero budget must not
        // silently become a confident empty).
        assert_eq!(
            classify_gather(false, false, false, Duration::ZERO, Duration::ZERO),
            GatherVerdict::AnswerColdStart
        );
        assert_eq!(
            classify_gather(true, false, false, Duration::ZERO, Duration::ZERO),
            GatherVerdict::AnswerSettled
        );
    }

    /// The cold-start `warn!`'s announce discriminator must only CLAIM
    /// what the query shape could observe. A single-robot GET runs no announce
    /// harvest, so `None` must render as an explicit "not-observed" — asserting
    /// "no robot was discovered" there would send an operator to a power switch on
    /// the strength of a question that was never asked. Hand oracle.
    #[test]
    fn announce_observation_label_never_claims_what_was_not_observed() {
        assert_eq!(announce_observation_label(Some(true)), "yes");
        assert_eq!(announce_observation_label(Some(false)), "no");
        let unknown = announce_observation_label(None);
        assert!(
            unknown.starts_with("not-observed"),
            "an unobserved announce space must render as such, got {unknown:?}"
        );
        // …and must not be confusable with either verdict by a log grep.
        assert_ne!(unknown, "no");
        assert_ne!(unknown, "yes");
    }

    #[test]
    fn the_shipped_cold_start_budget_is_a_multiple_of_the_harvest_window_and_interactive() {
        // The shipped constant's own bounds — a drift guard, so a future edit cannot
        // quietly make the grace useless (below one harvest window ⇒ no retry ever)
        // or un-interactive (a CLI verb that stares for ten seconds).
        assert_eq!(
            COLD_START_DISCOVERY_BUDGET,
            QUERY_HARVEST_WINDOW * COLD_START_HARVEST_ATTEMPTS as u32,
            "the budget IS the attempt count times the harvest window"
        );
        assert!(
            COLD_START_DISCOVERY_BUDGET >= QUERY_HARVEST_WINDOW * 2,
            "a grace under two harvest windows cannot retry meaningfully"
        );
        assert!(
            COLD_START_DISCOVERY_BUDGET <= Duration::from_secs(5),
            "a genuinely robot-less network must still answer within a few seconds"
        );
        // The per-GET window must fit inside the grace, else one attempt overshoots.
        assert!(QUERY_GATHER_WINDOW < COLD_START_DISCOVERY_BUDGET);
    }

    /// The grace must fit inside the CLIENT's round-trip read timeout, with
    /// room for the attempt that CROSSES the budget plus that attempt's per-robot GET.
    ///
    /// This is the guard against silently un-fixing the cold-start grace: the daemon answers a
    /// query only after spending the grace, so a budget raised toward
    /// [`crate::client::ROUNDTRIP_TIMEOUT`] would make the client give up FIRST — and
    /// the user would get an opaque IO timeout instead of the explicit "no robots
    /// discovered" verdict the grace exists to deliver. The whole fix would be
    /// inert, with every arm of this suite still green (they all drive the plane
    /// directly, below the socket).
    #[test]
    fn the_shipped_cold_start_budget_fits_inside_the_client_round_trip_timeout() {
        // Worst-case daemon-side wall for ONE query, spelled out term by term (the
        // RETRY PACING term is easy to miss, and leaving it out understates the wall):
        //
        //   1. an attempt that classifies at `budget - ε`               → budget
        //   2. that retry's pacing to `attempt_floor` (= the harvest window)
        //   3. the final attempt's announce harvest                     → harvest window
        //   4. that attempt's per-robot GET (concurrent across robots)  → gather window
        //
        // = 2500 + 500 + 500 + 250 = 3750 ms. Keep this in lockstep with
        // `COLD_START_DISCOVERY_BUDGET`'s "Worst-case wall" section.
        let retry_pacing = QUERY_HARVEST_WINDOW; // the `attempt_floor` the plane passes
        let worst_case =
            COLD_START_DISCOVERY_BUDGET + retry_pacing + QUERY_HARVEST_WINDOW + QUERY_GATHER_WINDOW;
        assert_eq!(
            worst_case,
            Duration::from_millis(3_750),
            "the documented worst case must be the one this test computes"
        );
        assert!(
            worst_case < crate::client::ROUNDTRIP_TIMEOUT,
            "the cold-start grace's worst-case daemon wall ({worst_case:?}) must stay under the \
             client's round-trip timeout ({:?}), else a cold query times out at the socket \
             instead of returning the NotConverged verdict",
            crate::client::ROUNDTRIP_TIMEOUT
        );
    }

    /// The grace loop itself, driven by a hand-scripted harvest — no zenoh,
    /// no daemon. `empty_attempts` attempts yield nothing, then every later attempt
    /// yields one item. Returns the gathered answer, its state, and the attempt count.
    fn drive_grace(
        maturity: &DiscoveryMaturity,
        empty_attempts: usize,
        budget: Duration,
    ) -> (Vec<u8>, DiscoveryState, usize) {
        let attempts = AtomicUsize::new(0);
        let (out, state) =
            gather_with_cold_start_grace(maturity, budget, Duration::from_millis(10), || {
                let n = attempts.fetch_add(1, Ordering::SeqCst);
                if n < empty_attempts {
                    HarvestAttempt::ok(Vec::new(), Some(false))
                } else {
                    HarvestAttempt::ok(vec![7u8], Some(true))
                }
            });
        (out, state, attempts.load(Ordering::SeqCst))
    }

    // ─── The cold-start warn's FLOOD LATCH ─────────────────────────
    //
    // The client issues up to ~13 queries per command, and this `warn!` would fire
    // once per query with no latch — so it goes through
    // `cerulion_core`'s shared `FailureRegimeLatch`. Every other adoption of that
    // machine in this repo ships `#[traced_test]` pins with LEVEL-TOKEN-matching
    // predicates and exact counts, because an unpinned
    // `StillFailing` arm is free to become a `debug!` and a text-only predicate
    // would pass that regression. These are those pins.

    /// Match a captured line's LEVEL as a whole whitespace token.
    ///
    /// A bare `contains("WARN")` would also match the span name (`tracing-test`
    /// renders the test function's own name into every line) or a field value, so an
    /// "exactly N" oracle could silently invert on a rename. Same discipline as
    /// `rmw_publish_reject_test`'s `line_level`.
    #[cfg(test)]
    fn line_level(line: &str) -> Option<&str> {
        line.split_whitespace()
            .find(|t| matches!(*t, "TRACE" | "DEBUG" | "INFO" | "WARN" | "ERROR"))
    }

    #[cfg(test)]
    fn count_at(logs: &str, level: &str, needle: &str) -> usize {
        logs.lines()
            .filter(|l| line_level(l) == Some(level) && l.contains(needle))
            .count()
    }

    /// The regime opens LOUD once, suppresses its repeats at `debug!`, re-announces
    /// LOUDLY at the decade, and closes with exactly one recovery `info!`.
    ///
    /// Drives the production `gather_with_cold_start_grace` past the first decade
    /// against a stub harvester, then lets it settle. What this test catches:
    /// the `Loud` arm demoted
    /// (no head at all), the `Suppressed` arm promoted (the flood fully restored —
    /// the very thing the latch prevents), the decade arm demoted, and `on_success` never
    /// reached (the regime never closes, so the next outage gets no fresh head).
    #[test]
    #[tracing_test::traced_test]
    fn the_cold_start_warn_opens_loud_suppresses_re_announces_and_recovers() {
        let maturity = DiscoveryMaturity::default();
        // A ZERO budget = exactly one harvest per call, so each call is one FAILURE
        // and the count is the loop counter — no timing in the oracle at all.
        const FAILURES: usize = 12;
        for _ in 0..FAILURES {
            let (out, state, _) = drive_grace(&maturity, usize::MAX, Duration::ZERO);
            assert!(out.is_empty());
            assert_eq!(state, DiscoveryState::NotConverged);
        }

        logs_assert(|lines: &[&str]| {
            let logs = lines.join("\n");
            let head = count_at(&logs, "WARN", "did NOT converge within the");
            let suppressed = count_at(&logs, "DEBUG", "still not converged (suppressed)");
            let decade = count_at(&logs, "WARN", "STILL has not converged");
            if head != 1 {
                return Err(format!(
                    "expected exactly 1 loud WARN head, got {head}\n{logs}"
                ));
            }
            // Failures 2..=12 are repeats; the 10th crosses the decade and is
            // RE-ANNOUNCED loudly instead of suppressed, so 12 failures = 1 head +
            // 1 decade + 10 suppressed.
            if decade != 1 {
                return Err(format!(
                    "expected exactly 1 WARN decade re-announcement at the 10th \
                     failure, got {decade}\n{logs}"
                ));
            }
            if suppressed != FAILURES - 2 {
                return Err(format!(
                    "expected {} DEBUG suppressed repeats, got {suppressed}\n{logs}",
                    FAILURES - 2
                ));
            }
            // The re-announcement must carry the running total, or an operator who
            // missed the head learns nothing from it.
            if !logs
                .lines()
                .any(|l| l.contains("STILL has not converged") && l.contains("total_failures=10"))
            {
                return Err(format!(
                    "the decade re-announcement must carry total_failures=10\n{logs}"
                ));
            }
            Ok(())
        });

        // Now the network answers: exactly ONE recovery `info!`, carrying what the
        // operator missed.
        let (out, state, _) = drive_grace(&maturity, 0, Duration::ZERO);
        assert_eq!(out, vec![7u8]);
        assert_eq!(state, DiscoveryState::Settled);
        logs_assert(|lines: &[&str]| {
            let logs = lines.join("\n");
            let recovery = count_at(&logs, "INFO", "discovery CONVERGED");
            if recovery != 1 {
                return Err(format!(
                    "expected exactly 1 INFO recovery, got {recovery}\n{logs}"
                ));
            }
            if !logs
                .lines()
                .any(|l| l.contains("discovery CONVERGED") && l.contains("suppressed_count=10"))
            {
                return Err(format!(
                    "the recovery must carry the SUPPRESSED count (10), not the total\n{logs}"
                ));
            }
            Ok(())
        });
    }

    /// ANTI-TAUTOLOGY: a plane whose very first harvest answers logs NOTHING.
    ///
    /// Without this, a reporter that fired on every gather — including successful
    /// ones — would still satisfy the "exactly N" arm above, because that arm drives
    /// only failures.
    #[test]
    #[tracing_test::traced_test]
    fn a_plane_that_converges_immediately_logs_no_cold_start_line() {
        let maturity = DiscoveryMaturity::default();
        let (out, state, _) = drive_grace(&maturity, 0, Duration::ZERO);
        assert_eq!(out, vec![7u8]);
        assert_eq!(state, DiscoveryState::Settled);
        logs_assert(|lines: &[&str]| {
            let logs = lines.join("\n");
            for needle in [
                "did NOT converge within the",
                "still not converged (suppressed)",
                "STILL has not converged",
                // No regime was ever open, so there is nothing to recover FROM —
                // `on_success` returns None and must log nothing.
                "discovery CONVERGED",
            ] {
                if logs.contains(needle) {
                    return Err(format!(
                        "a healthy plane must log nothing about the cold start, found \
                         {needle:?}\n{logs}"
                    ));
                }
            }
            Ok(())
        });
    }

    /// **A peer that ANSWERED UNUSABLY ends the query: it is not a
    /// wait condition.**
    ///
    /// The grace bridges a session that has not converged. A peer that replied HAS
    /// converged; its reply is unusable because of a wire skew or corruption, so
    /// every retry re-issues the GET and reads the same bytes — and on the `runs`
    /// verb each of those costs the ROBOT a fresh iceoryx2 reader node (the ~620 ms
    /// serve B2 measured and forbade folding into a poll).
    ///
    /// The oracle is the ATTEMPT COUNT, not a wall: it is load-independent, and it
    /// is the only thing that separates "stopped asking" from "asked again and got
    /// the same answer" — which are indistinguishable from the returned value, both
    /// being an empty list under `NotConverged`.
    ///
    /// Deliberately paired IN BODY with the non-terminal control over the same
    /// budget and floor, so the count cannot be satisfied by a loop that stopped
    /// retrying altogether.
    #[test]
    fn an_unusable_answer_stops_the_query_instead_of_re_paying_the_serve() {
        let budget = Duration::from_millis(150);
        let floor = Duration::from_millis(10);

        // TERMINAL: a peer answered, and its answer could not be used.
        let maturity = DiscoveryMaturity::default();
        let attempts = AtomicUsize::new(0);
        let (out, state) = gather_with_cold_start_grace(&maturity, budget, floor, || {
            attempts.fetch_add(1, Ordering::SeqCst);
            HarvestAttempt::ok(Vec::<u8>::new(), None).terminal_if(true)
        });
        assert!(out.is_empty(), "nothing was READ");
        assert_eq!(
            state,
            DiscoveryState::NotConverged,
            "terminal stops the ASKING, never licenses an absence claim — nothing \
             was read, so the marker must still forbid one"
        );
        assert_eq!(
            attempts.load(Ordering::SeqCst),
            1,
            "a skewed peer must be GET exactly once; retrying re-reads the same \
             bytes at the robot's full serve cost"
        );
        assert!(
            !maturity.is_grace_spent(),
            "the daemon's ONE cold-start grace bridges session establishment, and a \
             skewed peer says nothing about that — burning it here would make the \
             next genuinely-cold query answer after a single harvest"
        );

        // CONTROL, same budget and floor: a NON-terminal empty really does retry, so
        // the count above is measuring the terminal arm and not a dead loop.
        let control = DiscoveryMaturity::default();
        let control_attempts = AtomicUsize::new(0);
        let (_, state) = gather_with_cold_start_grace(&control, budget, floor, || {
            control_attempts.fetch_add(1, Ordering::SeqCst);
            HarvestAttempt::ok(Vec::<u8>::new(), Some(false))
        });
        assert_eq!(state, DiscoveryState::NotConverged);
        assert!(
            control_attempts.load(Ordering::SeqCst) > 1,
            "precondition: this budget/floor pair really does retry (got {})",
            control_attempts.load(Ordering::SeqCst)
        );

        // And a terminal attempt that ALSO gathered something is answered, not
        // discarded: one robot's skew must not suppress another's good reply.
        let mixed = DiscoveryMaturity::default();
        let (out, state) = gather_with_cold_start_grace(&mixed, budget, floor, || {
            HarvestAttempt::ok(vec![7u8], Some(true)).terminal_if(true)
        });
        assert_eq!(out, vec![7u8]);
        assert_eq!(state, DiscoveryState::Settled);
    }

    /// **A DIRECT-ADDRESSED answer never vouches for the announce
    /// plane.** The exact scenario, driven end to end on one shared maturity:
    /// `query_runs(Some(robot))` reaches a robot that is dialable but announces
    /// nothing, gathers a real answer — and a later fan-out `query_catalog(None)`
    /// whose announce harvest finds NOBODY must still report `NotConverged`.
    ///
    /// A fan-out that reported `Settled` beside an empty list there would be a confident
    /// "the LAN was searched and nothing is there", backed by a GET to one robot
    /// somebody typed the name of.
    ///
    /// The single-robot shape is `robots_announced: None` (no announce harvest
    /// ran), which is exactly what the production planes pass on their `Some(robot)`
    /// arms.
    #[test]
    fn a_direct_addressed_answer_does_not_settle_the_announce_plane() {
        let maturity = DiscoveryMaturity::default();
        let budget = Duration::from_millis(120);

        // A SINGLE-ROBOT query (the runs verb's `Some(robot)` arm) that ANSWERS.
        let (out, state) =
            gather_with_cold_start_grace(&maturity, budget, Duration::from_millis(10), || {
                HarvestAttempt::ok(vec![7u8], None)
            });
        assert_eq!(out, vec![7u8], "the direct GET really did answer");
        assert_eq!(
            state,
            DiscoveryState::Settled,
            "its OWN answer is non-empty, so the marker on THIS reply is moot — what \
             matters is what it does to the DAEMON"
        );
        assert!(
            !maturity.is_settled(),
            "a direct-addressed GET exercised no announce harvest, so it must not \
             vouch for LAN discovery on this daemon"
        );

        // …and the LATER fan-out, whose announce harvest finds nobody, stays `NotConverged`.
        let (out, state) =
            gather_with_cold_start_grace(&maturity, budget, Duration::from_millis(10), || {
                HarvestAttempt::ok(Vec::<u8>::new(), Some(false))
            });
        assert!(out.is_empty());
        assert_eq!(
            state,
            DiscoveryState::NotConverged,
            "an empty fan-out must NOT be read as absence on the strength of an \
             unrelated direct GET"
        );

        // ANTI-TAUTOLOGY: real announce evidence DOES settle the plane, and once it
        // has, a direct-addressed query inherits it (a fan-out that read something
        // proves both planes work — the asymmetry runs one way only).
        let warm = DiscoveryMaturity::default();
        let (_, state) =
            gather_with_cold_start_grace(&warm, budget, Duration::from_millis(10), || {
                HarvestAttempt::ok(vec![7u8], Some(true))
            });
        assert_eq!(state, DiscoveryState::Settled);
        assert!(
            warm.is_settled(),
            "announce evidence is what settles a plane"
        );
        let (out, state) =
            gather_with_cold_start_grace(&warm, budget, Duration::from_millis(10), || {
                HarvestAttempt::ok(Vec::<u8>::new(), None)
            });
        assert!(out.is_empty());
        assert_eq!(
            state,
            DiscoveryState::Settled,
            "a settled plane's direct GET reports an authoritative 'that robot did \
             not answer'"
        );
    }

    /// **The direct GET's `Settled` marker cannot license ANY absence
    /// claim, on this response or a later one.**
    ///
    /// The sibling above pins that a direct-addressed answer does not set the LATCH.
    /// This pins the consequence a consumer actually depends on, and it is the answer
    /// to "then why does that response say `Settled` at all?": the marker licenses
    /// exactly one reading — an EMPTY answer is absence — and an empty answer can
    /// never carry it without announce evidence, because `classify_gather` reaches
    /// `AnswerSettled` only via `found_any || ever_settled` and only announce evidence
    /// sets `ever_settled`.
    ///
    /// So the marker on a NON-empty direct reply is not a LAN-convergence claim
    /// anybody can act on; it is moot, and it is left alone because reporting
    /// `NotConverged` there regresses `cerulion-vizd`'s attach hint (see the
    /// `AnswerSettled` arm).
    #[test]
    fn an_empty_answer_never_reports_settled_without_announce_evidence() {
        let budget = Duration::from_millis(120);
        let floor = Duration::from_millis(10);

        // THE arm: a plane that answered a direct GET (so it reported `Settled` once)
        // must still refuse to call a LATER empty answer absence — nothing carried
        // forward, because the latch is what carries and the latch was not set.
        let maturity = DiscoveryMaturity::default();
        let (out, state) = gather_with_cold_start_grace(&maturity, budget, floor, || {
            HarvestAttempt::ok(vec![7u8], None)
        });
        assert_eq!(out, vec![7u8]);
        assert_eq!(
            state,
            DiscoveryState::Settled,
            "its own answer is non-empty"
        );
        let (out, state) = gather_with_cold_start_grace(&maturity, budget, floor, || {
            HarvestAttempt::ok(Vec::<u8>::new(), None)
        });
        assert!(out.is_empty());
        assert_eq!(
            state,
            DiscoveryState::NotConverged,
            "an earlier direct GET's `Settled` must not become a later EMPTY answer's \
             licence to claim absence"
        );

        // The same claim from the fan-out side, on a plane that never answered at all.
        let cold = DiscoveryMaturity::default();
        let (out, state) = gather_with_cold_start_grace(&cold, budget, floor, || {
            HarvestAttempt::ok(Vec::<u8>::new(), Some(false))
        });
        assert!(out.is_empty());
        assert_eq!(state, DiscoveryState::NotConverged);
        assert!(!cold.is_settled());

        // ANTI-TAUTOLOGY: an empty answer DOES report `Settled` once announce evidence
        // has been seen — otherwise "never Settled when empty" would be satisfied by a
        // plane that can never make an absence claim at all, which is the cold-start
        // failure running the other way (a warm desk re-paying the grace forever).
        let warm = DiscoveryMaturity::default();
        let _ = gather_with_cold_start_grace(&warm, budget, floor, || {
            HarvestAttempt::ok(vec![7u8], Some(true))
        });
        assert!(warm.is_settled(), "announce evidence settles the plane");
        let (out, state) = gather_with_cold_start_grace(&warm, budget, floor, || {
            HarvestAttempt::ok(Vec::<u8>::new(), Some(false))
        });
        assert!(out.is_empty());
        assert_eq!(
            state,
            DiscoveryState::Settled,
            "a plane the LAN has answered may report a genuine absence"
        );
    }

    /// The two FAN-OUT call sites really mark their attempt incomplete.
    ///
    /// STRUCTURAL, and it is the anti-inert pin rather than a style check. The
    /// behavioural arms below drive [`gather_with_cold_start_grace`] DIRECTLY, so
    /// they stay green if `query_catalog` / `query_schema` revert to a plain
    /// `HarvestAttempt::ok(gathered.replies, ..)` that throws the completeness
    /// away — the demotion would then be perfectly implemented and reachable from
    /// nothing. Driving it for real needs a live zenoh session AND a GET worker
    /// that unwinds, which is exactly the combination B4 could not reach either.
    ///
    /// Scoped to each function's own BODY by brace matching: a whole-file
    /// `contains` is satisfied the moment ONE site marks its attempt, and cannot
    /// see the other site losing it.
    #[test]
    fn both_fan_out_call_sites_mark_an_incomplete_attempt() {
        let src = include_str!("query.rs");
        // Scoped to the PRODUCTION plane. `query_catalog` / `query_schema` are trait
        // methods, so the same signatures also appear on the trait declaration and on
        // `NoopQueryPlane`; searching from the production impl is what keeps this
        // reading the body that actually fans out.
        let plane = src
            .find("impl QueryPlane for GatewayQueryPlane")
            .expect("the production plane must be findable");
        for (name, gather) in [
            ("fn query_catalog(", "query_robot_catalogs("),
            ("fn query_schema(", "query_robot_schemas("),
        ] {
            let body = fn_body(&src[plane..], name)
                .unwrap_or_else(|| panic!("`{name}` must be findable on the production plane"));
            // ANTI-TAUTOLOGY: the body really is the one that fans out. Without it a
            // mis-resolved slice would make the assertion below vacuously true.
            assert!(
                body.contains(gather),
                "`{name}` must be the body that calls `{gather}` — the extractor \
                 resolved the wrong span"
            );
            assert!(
                body.contains(".incomplete_if("),
                "`{name}` fans out over the robots, so it must carry the gather's \
                 completeness onto its `HarvestAttempt`. Dropping it leaves the \
                 fan-out demotion implemented and unreachable: a panicked worker \
                 would be logged and then answered over with `Settled`."
            );
        }
    }

    /// The brace-matched body of the first `fn` whose signature starts with
    /// `signature`, or `None`.
    ///
    /// Deliberately naive about strings and comments: it is used only on this
    /// file's own two query functions, and the anti-tautology assertion above is
    /// what catches a span that resolved somewhere unexpected.
    fn fn_body(src: &str, signature: &str) -> Option<String> {
        let start = src.find(signature)?;
        let open = src[start..].find('{')? + start;
        let mut depth = 0usize;
        for (i, c) in src[open..].char_indices() {
            match c {
                '{' => depth += 1,
                '}' => {
                    depth -= 1;
                    if depth == 0 {
                        return Some(src[open..=open + i].to_string());
                    }
                }
                _ => {}
            }
        }
        None
    }

    /// `fn_body` answers its hand-written vectors.
    ///
    /// The extractor is what the adoption guard asserts THROUGH, so one that
    /// returns too much (running to the end of the file) would make that guard
    /// pass on a body that carries nothing.
    #[test]
    fn fn_body_answers_its_hand_written_vectors() {
        let src = "fn a() { one(); }\nfn b() { if x { two(); } three(); }\nfn c();\n";
        assert_eq!(fn_body(src, "fn a(").as_deref(), Some("{ one(); }"));
        assert_eq!(
            fn_body(src, "fn b(").as_deref(),
            Some("{ if x { two(); } three(); }"),
            "a NESTED block must not end the body early"
        );
        assert!(
            !fn_body(src, "fn a(").unwrap().contains("two()"),
            "nor may a body run on into the NEXT function — that is how a scoped \
             guard silently becomes a whole-file one"
        );
        assert!(fn_body(src, "fn missing(").is_none());
        assert!(
            fn_body("fn d() { unclosed();", "fn d(").is_none(),
            "an unbalanced body fails CLOSED rather than returning a prefix"
        );
    }

    /// A pass whose fan-out lost a worker to a PANIC may not report
    /// `Settled`, and may not settle the daemon-wide latch.
    ///
    /// ONE VARIABLE. Both legs gather the SAME non-empty payload with the SAME
    /// announce evidence on a FRESH plane; the only difference is
    /// `incomplete_if`. Without that pairing "reports NotConverged" is satisfied
    /// by any plane that cannot settle at all, which is the cold-start failure
    /// running the other way.
    ///
    /// The non-EMPTY payload is the point. An incomplete pass that gathered
    /// nothing already reported `NotConverged` (`found_any == false` takes the
    /// cold-start arm), so a test over an empty gather would pass with the whole
    /// demotion deleted. The hole was exactly the mixed case: some robots
    /// answered, one was never asked, and the answer went out stamped
    /// authoritative.
    #[test]
    fn a_pass_that_lost_a_fan_out_worker_reports_not_converged_and_does_not_settle() {
        let budget = Duration::from_millis(120);
        let floor = Duration::from_millis(10);

        let lost = DiscoveryMaturity::default();
        let (out, state) = gather_with_cold_start_grace(&lost, budget, floor, || {
            HarvestAttempt::ok(vec![7u8], Some(true)).incomplete_if(true)
        });
        assert_eq!(
            out,
            vec![7u8],
            "what the robots that DID answer said is still returned — the demotion is \
             about the CLAIM, never about withholding data"
        );
        assert_eq!(
            state,
            DiscoveryState::NotConverged,
            "a pass that never asked one of its robots must not stamp its answer \
             authoritative: `Settled` licenses reading an absence from this list, and \
             the unasked robot is exactly what is absent from it"
        );
        assert!(
            !lost.is_settled(),
            "nor may it set the DURABLE latch — that latch is the claim `discovery \
             works on this machine`, and a pass that dropped a thread mid-fan-out is \
             not the demonstration it is made of"
        );

        // ANTI-TAUTOLOGY: the identical attempt, complete, is authoritative.
        let clean = DiscoveryMaturity::default();
        let (out, state) = gather_with_cold_start_grace(&clean, budget, floor, || {
            HarvestAttempt::ok(vec![7u8], Some(true))
        });
        assert_eq!(out, vec![7u8]);
        assert_eq!(state, DiscoveryState::Settled);
        assert!(clean.is_settled());
    }

    /// `incomplete` is scoped to ONE ATTEMPT, and a later clean attempt
    /// answers authoritatively.
    ///
    /// The alternative — accumulating it across the grace like `saw_a_robot` — is
    /// wrong for the same reason `saw_a_robot` accumulating is right: that one is a
    /// fact about the WHOLE grace ("was a robot ever announced?"), while this is a
    /// fact about the attempt whose payload is being returned. An attempt that ran
    /// clean really did ask everyone, and its answer is the one the caller gets.
    ///
    /// Also pins that the demotion costs no extra harvest: an incomplete attempt
    /// ANSWERS, it does not retry. Retrying would re-run a harvest whose worker
    /// panicked deterministically, paying a fresh window per attempt to reproduce
    /// the same panic.
    #[test]
    fn the_incomplete_marker_is_per_attempt_and_never_costs_a_retry() {
        let budget = Duration::from_millis(120);
        let floor = Duration::from_millis(10);

        let maturity = DiscoveryMaturity::default();
        let attempts = std::cell::Cell::new(0u32);
        let (out, state) = gather_with_cold_start_grace(&maturity, budget, floor, || {
            attempts.set(attempts.get() + 1);
            HarvestAttempt::ok(vec![1u8], Some(true)).incomplete_if(true)
        });
        assert_eq!(out, vec![1u8]);
        assert_eq!(state, DiscoveryState::NotConverged);
        assert_eq!(
            attempts.get(),
            1,
            "an incomplete pass ANSWERS — it must not spend the grace re-running a \
             harvest whose worker panics deterministically"
        );

        // The SAME plane, now running clean, answers authoritatively: the marker did
        // not stick to the plane.
        let (out, state) = gather_with_cold_start_grace(&maturity, budget, floor, || {
            HarvestAttempt::ok(vec![2u8], Some(true))
        });
        assert_eq!(out, vec![2u8]);
        assert_eq!(state, DiscoveryState::Settled);
        assert!(maturity.is_settled());
    }

    #[test]
    fn a_cold_plane_retries_until_the_network_answers_then_stays_settled() {
        // THE headline: a plane whose first three harvests see nothing keeps trying
        // and reports the answer that finally arrives — the cold start.
        let maturity = DiscoveryMaturity::default();
        assert!(
            !maturity.is_settled(),
            "a fresh plane has discovered nothing"
        );
        let (out, state, attempts) = drive_grace(&maturity, 3, Duration::from_millis(500));
        assert_eq!(
            out,
            vec![7u8],
            "the answer that finally arrived is returned"
        );
        assert_eq!(state, DiscoveryState::Settled);
        assert_eq!(attempts, 4, "3 empty attempts, then the one that answered");
        assert!(maturity.is_settled(), "the plane latched settled");

        // And once settled, an EMPTY harvest answers on the FIRST attempt — a warm
        // desk never re-pays the grace for a genuinely-absent topic.
        let started = Instant::now();
        let attempts = AtomicUsize::new(0);
        let (out, state) = gather_with_cold_start_grace(
            &maturity,
            Duration::from_secs(30),
            Duration::from_millis(10),
            || {
                attempts.fetch_add(1, Ordering::SeqCst);
                HarvestAttempt::ok(Vec::<u8>::new(), Some(false))
            },
        );
        assert!(out.is_empty());
        assert_eq!(
            state,
            DiscoveryState::Settled,
            "a settled plane's empty answer is AUTHORITATIVE"
        );
        assert_eq!(attempts.load(Ordering::SeqCst), 1, "exactly one attempt");
        assert!(
            started.elapsed() < Duration::from_secs(1),
            "a settled plane answers immediately, it does not spend the 30s budget"
        );
    }

    #[test]
    fn a_plane_that_never_sees_anything_answers_cold_start_within_the_budget() {
        // The robot-less network: the grace is spent, the answer is empty, and it is
        // marked NotConverged — NEVER a confident empty. Bounded: the whole call
        // stays inside a small multiple of its budget.
        let maturity = DiscoveryMaturity::default();
        let budget = Duration::from_millis(60);
        let started = Instant::now();
        let (out, state, attempts) = drive_grace(&maturity, usize::MAX, budget);
        let elapsed = started.elapsed();
        assert!(out.is_empty());
        assert_eq!(state, DiscoveryState::NotConverged);
        assert!(
            !maturity.is_settled(),
            "an empty gather must NOT latch the plane settled"
        );
        assert!(
            attempts >= 2,
            "the grace really retried (got {attempts} attempts)"
        );
        assert!(
            elapsed >= budget,
            "it spent the budget before giving up ({elapsed:?} < {budget:?})"
        );
        assert!(
            elapsed < budget + Duration::from_secs(2),
            "and it is BOUNDED — it did not hang ({elapsed:?})"
        );
        assert!(
            maturity.is_grace_spent(),
            "exhausting the budget latches the daemon's one cold-start grace"
        );
    }

    /// THE latch pin: a plane that NEVER settles must pay the cold-start grace
    /// ONCE, not on every query.
    ///
    /// `ever_settled` latches only on a NON-EMPTY gather, so on a desk whose robots
    /// never answer (powered off / another VLAN / still booting) the plane never
    /// settles — and without the latch EVERY `query_catalog` / `query_schema`
    /// re-pays the full budget, forever. `cerulion-vizd` holds one `NetdClient` behind
    /// a mutex across the whole round trip, so a Studio sidebar refresh on a robot-less
    /// desk would block for seconds AND stall every concurrent attach/detach.
    ///
    /// Hand oracles on BOTH the attempt count and the wall, plus the "still one fresh
    /// harvest" arm that keeps a robot appearing LATER discoverable.
    #[test]
    fn a_never_settling_plane_pays_the_cold_start_grace_once_not_per_query() {
        let maturity = DiscoveryMaturity::default();
        let budget = Duration::from_millis(200);

        // The pins are HARVEST COUNTS, not walls. A wall assertion tight enough to
        // separate "one harvest" from "a full re-pay" is also tight enough for a loaded
        // runner to trip (the loaded-runner class), and the counts say the same thing
        // exactly. `first_wall >= budget` is kept because load only makes it MORE true.

        // QUERY 1 — the cold start: retries across the whole budget, then answers
        // `NotConverged`. (The pre-existing first-query pin, restated here so the second
        // query's contrast is anchored in the same body.)
        let started = Instant::now();
        let (out, state, attempts) = drive_grace(&maturity, usize::MAX, budget);
        let first_wall = started.elapsed();
        assert!(out.is_empty());
        assert_eq!(state, DiscoveryState::NotConverged);
        assert!(attempts >= 2, "the first query really retried ({attempts})");
        assert!(
            first_wall >= budget,
            "the first query spent the grace ({first_wall:?}) — load only makes this \
             more true, so it is safe to assert"
        );
        assert!(maturity.is_grace_spent(), "and latched it");
        assert!(!maturity.is_settled(), "an empty gather never settles");

        // QUERY 2 on the SAME plane — the daemon's grace is spent, so this answers
        // after EXACTLY ONE harvest. THIS is the latch pin: reverting the `grace_spent`
        // arm makes it retry for the whole budget again. The oracle is the COUNT (1 vs
        // the ~16 a re-pay produces at this budget/floor), which is load-independent.
        let (out, state, attempts) = drive_grace(&maturity, usize::MAX, budget);
        assert!(out.is_empty());
        assert_eq!(
            state,
            DiscoveryState::NotConverged,
            "still NotConverged — the grace is spent, the verdict is unchanged"
        );
        assert_eq!(
            attempts, 1,
            "the grace is a per-DAEMON bridge, not a per-query tax (got {attempts} harvests)"
        );

        // …and the ONE fresh harvest is a REAL one: a robot that appears later is
        // discovered on the next query and SETTLES the plane (the anti-tautology arm —
        // a "fix" that stopped harvesting entirely would pass everything above).
        let (out, state, attempts) = drive_grace(&maturity, 0, budget);
        assert_eq!(out, vec![7u8], "the late-appearing robot's answer");
        assert_eq!(state, DiscoveryState::Settled);
        assert_eq!(attempts, 1, "one harvest, and it found something");
        assert!(maturity.is_settled(), "which settles the plane for good");
    }

    /// A robot that ANNOUNCES but whose catalog GET never comes
    /// back inside its window yields an EMPTY gather — `query_robot_catalogs` drops
    /// every robot that misses the window. Nothing was read, so the verdict must
    /// still be `NotConverged` (no absence claim), and the plane must NOT latch
    /// settled off an announce alone.
    ///
    /// This is the shape that would otherwise let the desk print "no robot was
    /// discovered on the network — check the robot is powered on" about a robot that
    /// is powered on and announcing. The two causes are separated in netd's LOG
    /// (`robots_announced`), never in the consumer's verdict.
    #[test]
    fn an_announced_but_silent_robot_is_not_converged_and_does_not_latch_settled() {
        let maturity = DiscoveryMaturity::default();
        let attempts = AtomicUsize::new(0);
        let (out, state) = gather_with_cold_start_grace(
            &maturity,
            Duration::from_millis(60),
            Duration::from_millis(10),
            || {
                attempts.fetch_add(1, Ordering::SeqCst);
                // A robot IS announced every attempt, but no catalog is gathered.
                HarvestAttempt::ok(Vec::<u8>::new(), Some(true))
            },
        );
        assert!(out.is_empty(), "nothing was read");
        assert_eq!(
            state,
            DiscoveryState::NotConverged,
            "an announce with no readable catalog licenses NO absence claim"
        );
        assert!(
            !maturity.is_settled(),
            "seeing an announce is not the same as reading a catalog — the plane must \
             not latch settled on it, or the next empty gather becomes a false absence"
        );
        assert!(attempts.load(Ordering::SeqCst) >= 2, "it kept trying");
    }

    /// A harvest that FAILS every attempt still answers within
    /// the budget, still refuses to claim absence, and does not latch settled. (The
    /// error text itself rides the cold-start `warn!` — asserted at the daemon level;
    /// what is pinned here is that a failing harvest cannot become a confident empty
    /// or an unbounded loop.)
    #[test]
    fn a_harvest_that_fails_every_attempt_is_not_converged_and_stays_bounded() {
        let maturity = DiscoveryMaturity::default();
        let budget = Duration::from_millis(60);
        let started = Instant::now();
        let attempts = AtomicUsize::new(0);
        let (out, state) =
            gather_with_cold_start_grace(&maturity, budget, Duration::from_millis(10), || {
                attempts.fetch_add(1, Ordering::SeqCst);
                HarvestAttempt::<u8>::failed("liveliness get refused".to_string())
            });
        assert!(out.is_empty());
        assert_eq!(
            state,
            DiscoveryState::NotConverged,
            "a session fault must never be reported as a settled absence"
        );
        assert!(!maturity.is_settled());
        assert!(attempts.load(Ordering::SeqCst) >= 2);
        assert!(
            started.elapsed() < budget + Duration::from_secs(2),
            "a persistently failing harvest stays bounded by the budget"
        );
    }

    #[test]
    fn the_retry_floor_bounds_the_attempt_count_on_an_instantly_returning_harvest() {
        // The busy-spin guard. The real `query_announce_entries` breaks out the
        // instant its reply channel closes, so on a cold peerless session a harvest
        // costs microseconds; without the per-attempt floor this loop would spin
        // millions of times through the budget. With a 10 ms floor and a 100 ms
        // budget the attempt count must land near 100/10, not in the thousands.
        let maturity = DiscoveryMaturity::default();
        let (_, state, attempts) = drive_grace(&maturity, usize::MAX, Duration::from_millis(100));
        assert_eq!(state, DiscoveryState::NotConverged);
        assert!(
            (2..=40).contains(&attempts),
            "the 10ms floor must pace the retries; got {attempts} attempts in a 100ms budget"
        );
    }

    #[test]
    fn noop_query_plane_refuses_loudly_with_no_network() {
        // The mirror-only daemon's plane refuses every query EXPLICITLY (never an empty
        // vec that the consumer cannot distinguish from "nobody answered") — the
        // consumer maps NoNetwork to its transient-session fallback.
        let plane = NoopQueryPlane;
        let cat = plane
            .query_catalog(None)
            .expect_err("no-op plane refuses a catalog query");
        assert!(matches!(cat, QueryError::NoNetwork));
        assert!(
            cat.to_string().contains("not network-configured"),
            "the refusal is explicit + loud: {cat}"
        );
        let sch = plane
            .query_schema(Some("go2"), "pkg/Type")
            .expect_err("no-op plane refuses a schema query");
        assert!(matches!(sch, QueryError::NoNetwork));
    }

    #[test]
    fn robots_from_entries_dedups_sorts_and_drops_topics() {
        // Two robots, one repeated (a two-topic robot announces twice), out of order
        // ⇒ distinct + sorted identities, the topic half dropped. Hand oracle.
        let entries = vec![
            ("ubuntu".to_string(), Some("/tf".to_string())),
            ("go2".to_string(), Some("/scan".to_string())),
            ("ubuntu".to_string(), Some("/odom".to_string())),
            ("go2".to_string(), None),
        ];
        assert_eq!(
            robots_from_entries(entries),
            vec!["go2".to_string(), "ubuntu".to_string()]
        );
        // Empty in → empty out (netd reached the LAN, nobody announced).
        assert!(robots_from_entries(vec![]).is_empty());
    }

    #[test]
    fn query_error_display_is_loud_and_actionable() {
        let no_net = QueryError::NoNetwork;
        assert!(no_net.to_string().contains("CERULION_NETD_NETWORK=off"));
        let sess = QueryError::Session {
            source: Box::new(TransportError::InvalidTransportConfig {
                reason: "no listener".to_string(),
            }),
        };
        assert!(sess
            .to_string()
            .contains("failed to open the shared zenoh session"));
        assert!(
            sess.to_string().contains("no listener"),
            "carries the cause"
        );
        assert!(std::error::Error::source(&sess).is_some());
    }
}

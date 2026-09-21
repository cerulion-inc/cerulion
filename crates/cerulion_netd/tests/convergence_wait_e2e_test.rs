// SPDX-License-Identifier: AGPL-3.0-only
//! The first-contact convergence WAIT, driven end to end against a
//! SCRIPTED fake daemon over a real UDS.
//!
//! The pure policy ([`cerulion_netd::ConvergenceWait::decide`]) is oracle-tested in
//! its own module. What this file pins is the half a pure test structurally cannot
//! see: that [`NetdClient`]'s query verbs actually RUN the loop — that a
//! not-converged answer is re-asked, that a settled one is not, that the loop SLEEPS
//! between polls, that a cancellation flag ends it, and that both query verbs route
//! through the same policy.
//!
//! # Why a fake daemon rather than the real one
//!
//! The behaviour under test is a SEQUENCE of daemon answers (not-converged, then
//! converged), which a real `cerulion-netd` produces only when a real robot appears
//! on a real LAN partway through the wait. A scripted `UnixListener` daemon serves
//! that sequence deterministically, in milliseconds, with no network — and, crucially,
//! COUNTS the queries it served, which is the oracle for "the loop is real", for "a
//! genuine absence never loops", and (as an UPPER bound) for "the loop actually
//! sleeps". Same pattern (and same reason) as `client.rs`'s trust-gate test.
//!
//! # No tight wall assertions (the loaded-runner class)
//!
//! Every count oracle is exact or a generous band, because the fake daemon answers
//! instantly and deterministically; every WALL assertion is a LOWER bound or a
//! generous ceiling, never a tight band a loaded runner could invert. The
//! query-count UPPER bounds are in the load-SAFE direction: contention can only
//! REDUCE how many polls fit in a fixed budget, never inflate it.
#![cfg(unix)]

use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixListener;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_netd::client::DISCOVERY_MIN_DAEMON_VERSION;
use cerulion_netd::protocol::{
    CatalogQueryResponse, DiscoveryState, Hello, Response, SchemaQueryResponse, HELLO_MARKER,
    PROTOCOL_VERSION,
};
use cerulion_netd::{ConvergenceWait, FirstContactWait, NetdClient, WaitOutcome};

/// Shrunk wait policy: the loop's SHAPE is what is under test, so the tests run at
/// milliseconds instead of the shipped 10 s / 750 ms. Never test-only sleeps against
/// production constants — the injectable policy exists for exactly this.
const TEST_CEILING: Duration = Duration::from_millis(300);
const TEST_POLL: Duration = Duration::from_millis(20);

fn policy() -> ConvergenceWait {
    ConvergenceWait::new(TEST_CEILING, TEST_POLL)
}

/// The most polls the shipped cadence can fit in `TEST_CEILING`, plus generous slack.
///
/// This is the SLEEP oracle: with the sleep deleted the fake
/// daemon answers in ~100 µs, so the loop would run thousands of round trips inside
/// the same budget while every other assertion in the arm stayed green. The bound is
/// load-SAFE — a slow runner fits FEWER polls, never more — so it cannot be inverted
/// by contention (the loaded-runner lesson).
const EXPECTED_MAX_POLLS: usize = (TEST_CEILING.as_millis() / TEST_POLL.as_millis()) as usize * 3;

/// A ceiling generous enough that a loaded runner cannot make a BOUNDED loop look
/// unbounded. Used only as an upper sanity bound, never as a tight band.
const GENEROUS_WALL_CEILING: Duration = Duration::from_secs(20);

/// One scripted answer the fake daemon serves for one query.
#[derive(Clone, Copy, Debug)]
struct Answer {
    discovery: DiscoveryState,
    /// Whether the answer carries a reply (a catalog / a schema doc) at all.
    non_empty: bool,
    /// What the daemon reports for its plane's un-settled age (the
    /// first-contact gate). `None` models an older daemon.
    plane_unsettled_ms: Option<u64>,
}

/// A YOUNG plane — the shape a genuinely cold daemon reports, and the one that must
/// NOT suppress the wait.
const NOT_CONVERGED_EMPTY: Answer = Answer {
    discovery: DiscoveryState::NotConverged,
    non_empty: false,
    plane_unsettled_ms: Some(0),
};
/// The same, from a daemon whose plane has ALREADY been trying longer than the
/// ceiling — the robot-less-desk shape after the first command.
const NOT_CONVERGED_OLD_PLANE: Answer = Answer {
    discovery: DiscoveryState::NotConverged,
    non_empty: false,
    plane_unsettled_ms: Some(TEST_CEILING.as_millis() as u64 + 1),
};
const SETTLED_EMPTY: Answer = Answer {
    discovery: DiscoveryState::Settled,
    non_empty: false,
    plane_unsettled_ms: None,
};
const SETTLED_FOUND: Answer = Answer {
    discovery: DiscoveryState::Settled,
    non_empty: true,
    plane_unsettled_ms: None,
};
/// A NON-EMPTY answer a stale (pre-discovery-marker) daemon serves: the trust gate downgrades its
/// `Settled` to `NotConverged`, so the wait must exit on emptiness alone.
const STALE_FOUND: Answer = SETTLED_FOUND;

/// A scripted netd stand-in over a real UDS: serves a `Hello` at `protocol`, then
/// answers each request line from `script` (the LAST entry repeats forever), counting
/// every query it served.
struct FakeDaemon {
    socket: PathBuf,
    dir: PathBuf,
    queries: Arc<AtomicUsize>,
}

impl FakeDaemon {
    fn queries_served(&self) -> usize {
        self.queries.load(Ordering::SeqCst)
    }
}

impl Drop for FakeDaemon {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.dir);
    }
}

/// A FIXED-WIDTH temp dir for a UDS.
///
/// A UDS path must fit `sockaddr_un::sun_path` (104 bytes on macOS), and the system
/// temp dir already eats ~half of that, so an unbounded `{tag}_{pid}_{nanos}` name
/// makes the bind fail for some test names and not others — an order-dependent
/// harness failure masquerading as a product one. Width is constant regardless of
/// `tag` (truncated to 8).
fn socket_dir(tag: &str) -> PathBuf {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64;
    let short_tag: String = tag.chars().take(8).collect();
    std::env::temp_dir().join(format!(
        "conv{:06}{:08x}_{short_tag}",
        std::process::id() % 1_000_000,
        nanos & 0xffff_ffff,
    ))
}

fn bind_socket(dir: &PathBuf) -> (PathBuf, UnixListener) {
    std::fs::create_dir_all(dir).expect("temp dir");
    let socket = dir.join("netd.sock");
    assert!(
        socket.as_os_str().len() < 100,
        "the fake daemon's socket path must fit sun_path; got {} bytes: {}",
        socket.as_os_str().len(),
        socket.display()
    );
    let listener = UnixListener::bind(&socket).expect("bind the fake daemon socket");
    (socket, listener)
}

fn spawn_fake_daemon(tag: &str, protocol: u32, script: Vec<Answer>) -> FakeDaemon {
    assert!(!script.is_empty(), "a script needs at least one answer");
    let dir = socket_dir(tag);
    let (socket, listener) = bind_socket(&dir);
    let queries = Arc::new(AtomicUsize::new(0));
    let served = Arc::clone(&queries);

    std::thread::spawn(move || {
        let Ok((stream, _)) = listener.accept() else {
            return;
        };
        let mut w = stream.try_clone().expect("clone the stream");
        let banner = Hello {
            hello: HELLO_MARKER.to_string(),
            protocol,
        }
        .to_json_line();
        let _ = writeln!(w, "{banner}");
        let _ = w.flush();

        let reader = BufReader::new(stream);
        for line in reader.lines().map_while(Result::ok) {
            let Ok(v) = serde_json::from_str::<serde_json::Value>(&line) else {
                continue;
            };
            let id = v["id"].as_u64().unwrap_or(0);
            let idx = served.fetch_add(1, Ordering::SeqCst);
            let answer = script[idx.min(script.len() - 1)];
            let resp = if v["method"] == "query_catalog" {
                Response::CatalogQuery(CatalogQueryResponse {
                    id,
                    catalogs: if answer.non_empty {
                        vec![catalog_reply()]
                    } else {
                        vec![]
                    },
                    discovery: answer.discovery,
                    plane_unsettled_ms: answer.plane_unsettled_ms,
                })
            } else {
                Response::SchemaQuery(SchemaQueryResponse {
                    id,
                    replies: if answer.non_empty {
                        vec![schema_reply()]
                    } else {
                        vec![]
                    },
                    discovery: answer.discovery,
                    plane_unsettled_ms: answer.plane_unsettled_ms,
                })
            };
            let _ = writeln!(w, "{}", resp.to_json_line());
            let _ = w.flush();
        }
    });

    FakeDaemon {
        socket,
        dir,
        queries,
    }
}

/// The ONE catalog a converged fake daemon serves — the hand oracle every
/// "the wait resolved" assertion compares against.
fn catalog_reply() -> cerulion_core::CatalogReply {
    cerulion_core::CatalogReply {
        version: 1,
        robot: "robot".to_string(),
        entries: Vec::new(),
        error: None,
    }
}

/// The ONE schema reply a converged fake daemon serves.
fn schema_reply() -> cerulion_core::SchemaReply {
    cerulion_core::SchemaReply {
        version: 1,
        robot: "robot".to_string(),
        requested: "msgs/Probe".to_string(),
        docs: Vec::new(),
        error: Some("served by the convergence-wait fake daemon".to_string()),
    }
}

fn connect(daemon: &FakeDaemon) -> NetdClient {
    NetdClient::connect_or_spawn_at(daemon.socket.clone()).expect("connect to the fake daemon")
}

/// A never-cancelled flag, for the arms that are not about cancellation.
fn live() -> AtomicBool {
    AtomicBool::new(true)
}

// ─── (a) the loop is REAL: a not-converged answer is re-asked ────────────────

/// THE headline: a daemon that reports `NotConverged` for its first two queries and
/// then converges must be re-asked until it does — the shape a slow-converging
/// daemon produces, where a retry ~11 s later resolved what the first
/// invocation could not.
///
/// The oracle is the fake daemon's own SERVED-QUERY COUNT (exactly 3: two
/// not-converged + the converging one), which a pure test cannot see and which
/// deleting the loop drives to 1.
#[test]
fn a_not_converged_daemon_is_re_asked_until_it_converges() {
    let daemon = spawn_fake_daemon(
        "resolve",
        PROTOCOL_VERSION,
        vec![NOT_CONVERGED_EMPTY, NOT_CONVERGED_EMPTY, SETTLED_FOUND],
    );
    let mut client = connect(&daemon);
    let mut progress_calls = 0usize;
    let running = live();

    let started = Instant::now();
    let out = {
        let mut sink = |_: Duration| progress_calls += 1;
        let mut wait = FirstContactWait::new(policy(), &mut sink, Some(&running));
        client
            .query_catalog_converged(None, &mut wait)
            .expect("the catalog query runs")
    };
    let wall = started.elapsed();

    assert_eq!(
        daemon.queries_served(),
        3,
        "the loop must re-ask a not-converged daemon: 2 not-converged answers + the \
         converging one. 1 means the wait is gone"
    );
    assert_eq!(
        out.outcome,
        WaitOutcome::Answered,
        "the wait SUCCEEDED — the consumer must never render a give-up line from it"
    );
    assert_eq!(
        out.answer.discovery,
        DiscoveryState::Settled,
        "the answer served is the CONVERGED one, not the first not-converged answer"
    );
    assert_eq!(
        out.answer.catalogs,
        vec![catalog_reply()],
        "and it carries the robot's catalog (hand oracle)"
    );
    // One announcement BEFORE the first round trip (the longest silent stretch of the
    // wait), then one per KeepWaiting decision.
    assert_eq!(
        progress_calls, 3,
        "1 pre-flight announcement + 2 wait decisions; the answering round trip \
         prints nothing"
    );
    assert_eq!(
        out.progress_lines, progress_calls as u32,
        "the reported line count is what the sink was actually asked to print"
    );
    assert!(
        wall < GENEROUS_WALL_CEILING,
        "bounded regardless of runner load: {wall:?}"
    );
}

// ─── (b) a genuine absence NEVER loops ───────────────────────────────────────

/// A daemon that VOUCHES for its discovery is believed on the first round trip, so a
/// `cerulion topic echo <typo>` on a warm desk is as fast as it was before the convergence wait.
///
/// The exactly-ONE-query oracle is what makes this a real pin: an implementation
/// that looped on emptiness rather than on the marker would still return the same
/// empty answer, just seconds later.
#[test]
fn a_settled_empty_answer_is_returned_without_a_single_extra_query() {
    let daemon = spawn_fake_daemon("absence", PROTOCOL_VERSION, vec![SETTLED_EMPTY]);
    let mut client = connect(&daemon);
    let mut progress_calls = 0usize;
    let running = live();

    let out = {
        let mut sink = |_: Duration| progress_calls += 1;
        let mut wait = FirstContactWait::new(policy(), &mut sink, Some(&running));
        client
            .query_catalog_converged(None, &mut wait)
            .expect("the catalog query runs")
    };

    assert_eq!(
        daemon.queries_served(),
        1,
        "a settled answer is AUTHORITATIVE — looping on it would make every typo on a \
         warm desk pay the full ceiling"
    );
    assert_eq!(out.outcome, WaitOutcome::Answered);
    assert_eq!(out.answer.discovery, DiscoveryState::Settled);
    assert!(out.answer.catalogs.is_empty(), "and it really is empty");
    assert_eq!(
        progress_calls, 1,
        "only the pre-flight announcement — no wait decision was ever taken"
    );
    assert!(
        out.waited < TEST_CEILING,
        "the immediate path costs one UDS round trip, not a poll: {:?}",
        out.waited
    );
}

// ─── (c) the ceiling BOUNDS a daemon that never converges ────────────────────

/// A daemon that never converges must still ANSWER — with the explicit UNKNOWN,
/// after a bounded number of round trips, HAVING SLEPT between them.
///
/// The upper query bound is the sleep oracle: with `sleep` deleted this arm's other
/// assertions all still hold while the client hammers the shared daemon thousands of
/// times inside the same budget.
#[test]
fn a_never_converging_daemon_gives_the_honest_unknown_after_a_bounded_wait() {
    let daemon = spawn_fake_daemon("ceiling", PROTOCOL_VERSION, vec![NOT_CONVERGED_EMPTY]);
    let mut client = connect(&daemon);
    let mut progress_calls = 0usize;
    let running = live();

    let started = Instant::now();
    let out = {
        let mut sink = |_: Duration| progress_calls += 1;
        let mut wait = FirstContactWait::new(policy(), &mut sink, Some(&running));
        client
            .query_catalog_converged(None, &mut wait)
            .expect("the catalog query runs")
    };
    let wall = started.elapsed();

    assert_eq!(
        out.outcome,
        WaitOutcome::GaveUp,
        "the terminal decision is what the consumer gates its epitaph on"
    );
    assert_eq!(
        out.answer.discovery,
        DiscoveryState::NotConverged,
        "the last answer's marker rides out, so the consumer renders UNKNOWN — never \
         absence"
    );
    assert!(out.answer.catalogs.is_empty());
    assert!(
        wall < GENEROUS_WALL_CEILING,
        "and it is BOUNDED, not a hang: {wall:?}"
    );
    let served = daemon.queries_served();
    assert!(
        served >= 2,
        "at least one re-ask must have happened, got {served}"
    );
    assert!(
        served <= EXPECTED_MAX_POLLS,
        "the loop must SLEEP between polls — {served} queries in a {TEST_CEILING:?} \
         budget at a {TEST_POLL:?} cadence means it is hammering the shared daemon \
         (max {EXPECTED_MAX_POLLS})"
    );
    assert_eq!(
        progress_calls, served,
        "one pre-flight announcement + one line per re-ask = one per query issued; \
         the final give-up answer prints none"
    );
}

// ─── (d) BOTH query verbs route through the wait ─────────────────────────────

/// `cerulion schema info` renders an empty answer to a human exactly as the topic
/// verbs do, so the schema verb must run the same loop. Without this arm the wait
/// could ship wired to one verb only, which is precisely the earlier defect's shape (the
/// topic path fixed, the schema path left claiming absence).
#[test]
fn the_schema_verb_runs_the_same_wait() {
    let daemon = spawn_fake_daemon(
        "schema",
        PROTOCOL_VERSION,
        vec![NOT_CONVERGED_EMPTY, SETTLED_FOUND],
    );
    let mut client = connect(&daemon);
    let mut progress_calls = 0usize;
    let running = live();

    let out = {
        let mut sink = |_: Duration| progress_calls += 1;
        let mut wait = FirstContactWait::new(policy(), &mut sink, Some(&running));
        client
            .query_schema_converged(None, "msgs/Probe", &mut wait)
            .expect("the schema query runs")
    };

    assert_eq!(
        daemon.queries_served(),
        2,
        "the schema verb re-asks a not-converged daemon too"
    );
    assert_eq!(out.outcome, WaitOutcome::Answered);
    assert_eq!(out.answer.discovery, DiscoveryState::Settled);
    assert_eq!(
        out.answer.replies,
        vec![schema_reply()],
        "and serves the converged reply (hand oracle)"
    );
    assert_eq!(progress_calls, 2, "1 pre-flight + 1 wait decision");
}

/// The schema verb's absence twin — a settled empty schema answer must not loop
/// either (a `cerulion schema info <typo>` on a warm desk).
#[test]
fn a_settled_empty_schema_answer_does_not_loop() {
    let daemon = spawn_fake_daemon("schemabs", PROTOCOL_VERSION, vec![SETTLED_EMPTY]);
    let mut client = connect(&daemon);
    let running = live();

    let out = {
        let mut sink = |_: Duration| {};
        let mut wait = FirstContactWait::new(policy(), &mut sink, Some(&running));
        client
            .query_schema_converged(None, "msgs/Probe", &mut wait)
            .expect("the schema query runs")
    };

    assert_eq!(daemon.queries_served(), 1);
    assert_eq!(out.outcome, WaitOutcome::Answered);
    assert!(out.answer.replies.is_empty());
    assert_eq!(out.answer.discovery, DiscoveryState::Settled);
}

// ─── (e) the trust gate × the wait ───────────────────────────────────

/// A daemon too OLD to report a discovery state has EVERY answer downgraded to
/// `NotConverged` by the trust gate. If the wait keyed on the marker alone,
/// a stale daemon serving a perfectly good answer would make every command wait the
/// full ceiling and then use the answer it already had at 0 ms.
///
/// The `!answer_empty` arm of the policy exists for this, and this is its
/// production-shaped pin: exactly ONE query, and the answer is the one served.
#[test]
fn a_stale_daemons_non_empty_answer_is_used_without_waiting() {
    let stale = DISCOVERY_MIN_DAEMON_VERSION - 1;
    let daemon = spawn_fake_daemon("stalefnd", stale, vec![STALE_FOUND]);
    let mut client = connect(&daemon);
    assert_eq!(
        client.daemon_protocol(),
        stale,
        "precondition: we really are talking to a pre-916 daemon"
    );
    let running = live();

    let out = {
        let mut sink = |_: Duration| {};
        let mut wait = FirstContactWait::new(policy(), &mut sink, Some(&running));
        client
            .query_catalog_converged(None, &mut wait)
            .expect("the catalog query runs")
    };

    assert_eq!(
        out.answer.discovery,
        DiscoveryState::NotConverged,
        "precondition: the trust gate DID downgrade this stale daemon's report — \
         without that this test would be vacuous"
    );
    assert_eq!(
        daemon.queries_served(),
        1,
        "and yet the non-empty answer is used immediately: a permanently-stale \
         daemon must not make every command pay the ceiling"
    );
    assert_eq!(out.outcome, WaitOutcome::Answered);
    assert_eq!(out.answer.catalogs, vec![catalog_reply()]);
}

/// THE outcome pin: a wait that polls, then FINDS
/// something, ended SUCCESSFULLY — even though its `discovery` still reads
/// `NotConverged` because the trust gate downgraded it.
///
/// A consumer that re-derives "did we give up?" from `(NotConverged, waited > 0)`
/// prints `…gave up — nothing on the network answered` immediately before
/// resolving the topic and streaming its frames. Only the WHOLE-loop terminal
/// decision can tell these apart, which is why `Converged` carries it — and this arm
/// makes BOTH proxies fire, so a revert to either one fails here.
#[test]
fn a_stale_daemon_that_answers_on_a_later_poll_did_not_give_up() {
    let stale = DISCOVERY_MIN_DAEMON_VERSION - 1;
    let daemon = spawn_fake_daemon(
        "staleltr",
        stale,
        vec![NOT_CONVERGED_EMPTY, NOT_CONVERGED_EMPTY, STALE_FOUND],
    );
    let mut client = connect(&daemon);
    let running = live();

    let out = {
        let mut sink = |_: Duration| {};
        let mut wait = FirstContactWait::new(policy(), &mut sink, Some(&running));
        client
            .query_catalog_converged(None, &mut wait)
            .expect("the catalog query runs")
    };

    assert_eq!(
        daemon.queries_served(),
        3,
        "it really did poll before finding"
    );
    assert!(
        !out.waited.is_zero(),
        "and it really did wait — so a duration-based give-up guard would fire here"
    );
    assert_eq!(
        out.answer.discovery,
        DiscoveryState::NotConverged,
        "the trust gate still downgrades this daemon's report — so a marker-based \
         guard would fire here too"
    );
    assert_eq!(
        out.outcome,
        WaitOutcome::Answered,
        "yet the wait SUCCEEDED: only the terminal decision distinguishes it, and it \
         is what the consumer's epitaph must be gated on"
    );
    assert_eq!(out.answer.catalogs, vec![catalog_reply()]);
}

/// The documented, deliberate COST of that trust gate: a stale daemon serving an
/// EMPTY answer is wire-indistinguishable from a genuine cold start, so it waits the
/// full ceiling before giving the explicit UNKNOWN.
#[test]
fn a_stale_daemons_empty_answer_waits_the_ceiling_by_design() {
    let stale = DISCOVERY_MIN_DAEMON_VERSION - 1;
    // A pre-discovery-marker daemon also predates the plane-age field, so it reports `None`:
    // UNKNOWN, which must not cap the wait.
    let stale_empty = Answer {
        plane_unsettled_ms: None,
        ..SETTLED_EMPTY
    };
    let daemon = spawn_fake_daemon("staleemp", stale, vec![stale_empty]);
    let mut client = connect(&daemon);
    let running = live();

    let started = Instant::now();
    let out = {
        let mut sink = |_: Duration| {};
        let mut wait = FirstContactWait::new(policy(), &mut sink, Some(&running));
        client
            .query_catalog_converged(None, &mut wait)
            .expect("the catalog query runs")
    };
    let wall = started.elapsed();

    assert_eq!(
        out.answer.discovery,
        DiscoveryState::NotConverged,
        "a pre-916 daemon's Settled is never believed"
    );
    assert_eq!(out.outcome, WaitOutcome::GaveUp);
    assert!(
        out.waited + TEST_POLL >= TEST_CEILING,
        "so it waits until one poll short of the ceiling (the predictive gate): {:?}",
        out.waited
    );
    assert!(wall < GENEROUS_WALL_CEILING, "bounded, as always: {wall:?}");
    let served = daemon.queries_served();
    assert!(served >= 2, "and it really did re-ask");
    assert!(
        served <= EXPECTED_MAX_POLLS,
        "sleeping between polls, not hammering: {served} > {EXPECTED_MAX_POLLS}"
    );
}

// ─── (f) FIRST contact, not every command ────────────────────────────────────

/// THE first-contact pin: a daemon whose plane has ALREADY been
/// trying longer than the ceiling gets NO additional wait.
///
/// `ever_settled` latches only on a non-empty gather, so a robot-less desk's daemon
/// reports `NotConverged` for its whole lifetime. Without this cap the wait is not a
/// first-contact wait at all — it is a ten-second tax on every command, forever,
/// which is the exact regression the cap one layer down prevents.
#[test]
fn a_daemon_whose_plane_already_out_waited_the_ceiling_is_not_re_asked() {
    let daemon = spawn_fake_daemon("oldplane", PROTOCOL_VERSION, vec![NOT_CONVERGED_OLD_PLANE]);
    let mut client = connect(&daemon);
    let mut progress_calls = 0usize;
    let running = live();

    let started = Instant::now();
    let out = {
        let mut sink = |_: Duration| progress_calls += 1;
        let mut wait = FirstContactWait::new(policy(), &mut sink, Some(&running));
        client
            .query_catalog_converged(None, &mut wait)
            .expect("the catalog query runs")
    };
    let wall = started.elapsed();

    assert_eq!(
        daemon.queries_served(),
        1,
        "the plane has been trying longer than we are willing to wait — adding our \
         own ceiling on top is the per-command tax this gate exists to prevent"
    );
    assert_eq!(out.outcome, WaitOutcome::GaveUp);
    assert_eq!(out.answer.discovery, DiscoveryState::NotConverged);
    assert!(
        out.waited < TEST_CEILING,
        "and it answered in one round trip, not a ceiling: {:?}",
        out.waited
    );
    assert!(wall < GENEROUS_WALL_CEILING);
    assert_eq!(
        progress_calls, 1,
        "the pre-flight announcement fires (a wait was possible) but nothing follows"
    );
}

/// ANTI-TAUTOLOGY for the cap: the SAME daemon, the SAME answers, with a YOUNG plane
/// — the wait must run. Without this, a cap that fired unconditionally would pass the
/// arm above.
#[test]
fn a_young_plane_still_gets_the_full_wait() {
    let daemon = spawn_fake_daemon("younplan", PROTOCOL_VERSION, vec![NOT_CONVERGED_EMPTY]);
    let mut client = connect(&daemon);
    let running = live();

    let out = {
        let mut sink = |_: Duration| {};
        let mut wait = FirstContactWait::new(policy(), &mut sink, Some(&running));
        client
            .query_catalog_converged(None, &mut wait)
            .expect("the catalog query runs")
    };

    assert!(
        daemon.queries_served() >= 2,
        "a young plane must still be re-asked — otherwise the cap has swallowed the \
         whole feature"
    );
    assert_eq!(out.outcome, WaitOutcome::GaveUp);
}

// ─── (g) cancellation ────────────────────────────────────────────────────────

/// THE Ctrl-C pin: `cerulion_cli` replaces the default
/// SIGINT/SIGTERM/SIGHUP disposition with a flag-flip handler BEFORE `topic echo` /
/// `topic hz` run, so during an un-sliced wait those signals are all no-ops and only
/// SIGKILL ends the command — up to ~15 s of a command that cannot be interrupted, on
/// the two verbs users interrupt most.
///
/// A watchdog thread clears the flag mid-wait; the loop must return promptly with
/// `Cancelled` (never `GaveUp` — nothing was concluded about the network).
#[test]
fn a_cleared_running_flag_ends_the_wait_promptly() {
    let daemon = spawn_fake_daemon("cancel", PROTOCOL_VERSION, vec![NOT_CONVERGED_EMPTY]);
    let mut client = connect(&daemon);
    // A ceiling far longer than the test should take, so finishing early can ONLY be
    // cancellation — not the budget quietly expiring.
    let long = ConvergenceWait::new(Duration::from_secs(30), TEST_POLL);
    let running = Arc::new(AtomicBool::new(true));
    let flipper = Arc::clone(&running);
    std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(120));
        flipper.store(false, Ordering::Relaxed);
    });

    let started = Instant::now();
    let out = {
        let mut sink = |_: Duration| {};
        let mut wait = FirstContactWait::new(long, &mut sink, Some(&running));
        client
            .query_catalog_converged(None, &mut wait)
            .expect("the catalog query runs")
    };
    let wall = started.elapsed();

    assert_eq!(
        out.outcome,
        WaitOutcome::Cancelled,
        "a cancelled wait concluded NOTHING about the network — it must not be \
         reported as a give-up"
    );
    assert!(
        wall < Duration::from_secs(5),
        "it must end promptly after the flag flips, not at the 30 s ceiling: {wall:?}"
    );
}

/// The control: with the flag left SET, the same shape runs its wait to the ceiling.
/// Without it, a loop that treated every flag as cancelled would pass the arm above.
#[test]
fn a_set_running_flag_never_cancels() {
    let daemon = spawn_fake_daemon("nocancel", PROTOCOL_VERSION, vec![NOT_CONVERGED_EMPTY]);
    let mut client = connect(&daemon);
    let running = live();

    let out = {
        let mut sink = |_: Duration| {};
        let mut wait = FirstContactWait::new(policy(), &mut sink, Some(&running));
        client
            .query_catalog_converged(None, &mut wait)
            .expect("the catalog query runs")
    };

    assert_eq!(out.outcome, WaitOutcome::GaveUp);
    assert!(daemon.queries_served() >= 2);
}

/// A caller with NO cancellation source (`topic info`, `schema info`) is never
/// cancelled — `None` must not read as "already cancelled".
#[test]
fn no_cancellation_source_is_not_cancellation() {
    let daemon = spawn_fake_daemon("nosource", PROTOCOL_VERSION, vec![NOT_CONVERGED_EMPTY]);
    let mut client = connect(&daemon);

    let out = {
        let mut sink = |_: Duration| {};
        let mut wait = FirstContactWait::new(policy(), &mut sink, None);
        client
            .query_catalog_converged(None, &mut wait)
            .expect("the catalog query runs")
    };

    assert_eq!(out.outcome, WaitOutcome::GaveUp);
    assert!(daemon.queries_served() >= 2);
}

// ─── (h) the off switch ──────────────────────────────────────────────────────

/// [`ConvergenceWait::off`] reproduces the earlier behaviour exactly — one query,
/// the explicit UNKNOWN, no sleep, and NO progress line at all.
///
/// The anti-tautology partner to every arm above: it proves the waiting in those arms
/// comes from the POLICY and not from something unconditional in the loop. It is also
/// the posture the CLI's local-walker fallback ships with, so the silence is a
/// contract, not a detail.
#[test]
fn the_off_policy_answers_on_the_first_round_trip_and_prints_nothing() {
    let daemon = spawn_fake_daemon("off", PROTOCOL_VERSION, vec![NOT_CONVERGED_EMPTY]);
    let mut client = connect(&daemon);
    let mut progress_calls = 0usize;
    let running = live();

    let out = {
        let mut sink = |_: Duration| progress_calls += 1;
        let mut wait = FirstContactWait::new(ConvergenceWait::off(), &mut sink, Some(&running));
        client
            .query_catalog_converged(None, &mut wait)
            .expect("the catalog query runs")
    };

    assert_eq!(daemon.queries_served(), 1);
    assert_eq!(out.outcome, WaitOutcome::GaveUp);
    assert_eq!(out.answer.discovery, DiscoveryState::NotConverged);
    assert_eq!(
        progress_calls, 0,
        "a caller that will not wait must not announce a wait"
    );
    assert_eq!(out.progress_lines, 0);
}

// ─── (i) a mid-poll transport error carries its elapsed out ──────────────────

/// A wait abandoned by a transport error must hand the elapsed and
/// the line count back so the consumer can CLOSE the progress lines the user has been
/// watching. A bare `?` would discard both and leave the counter never closed.
///
/// The fake daemon serves one not-converged answer and then hangs up, so the SECOND
/// poll fails mid-wait.
#[test]
fn a_mid_poll_transport_error_reports_how_long_the_wait_had_run() {
    let dir = socket_dir("abortmid");
    let (socket, listener) = bind_socket(&dir);
    std::thread::spawn(move || {
        let Ok((stream, _)) = listener.accept() else {
            return;
        };
        let mut w = stream.try_clone().expect("clone");
        let _ = writeln!(
            w,
            "{}",
            Hello {
                hello: HELLO_MARKER.to_string(),
                protocol: PROTOCOL_VERSION,
            }
            .to_json_line()
        );
        let _ = w.flush();
        let reader = BufReader::new(stream);
        for line in reader.lines().map_while(Result::ok) {
            let Ok(v) = serde_json::from_str::<serde_json::Value>(&line) else {
                continue;
            };
            let _ = writeln!(
                w,
                "{}",
                Response::CatalogQuery(CatalogQueryResponse {
                    id: v["id"].as_u64().unwrap_or(0),
                    catalogs: vec![],
                    discovery: DiscoveryState::NotConverged,
                    plane_unsettled_ms: Some(0),
                })
                .to_json_line()
            );
            let _ = w.flush();
            // Hang up: every LATER poll must fail.
            return;
        }
    });

    let mut client = NetdClient::connect_or_spawn_at(socket).expect("connect");
    let mut progress_calls = 0usize;
    let running = live();
    let abort = {
        let mut sink = |_: Duration| progress_calls += 1;
        let mut wait = FirstContactWait::new(policy(), &mut sink, Some(&running));
        client
            .query_catalog_converged(None, &mut wait)
            .expect_err("the second poll must fail once the daemon hangs up")
    };

    assert!(
        !abort.waited.is_zero(),
        "the abandoned wait's elapsed must ride out so the consumer can close its \
         progress lines"
    );
    assert!(
        abort.progress_lines >= 2,
        "the user had already seen the pre-flight line plus at least one poll line \
         ({} reported)",
        abort.progress_lines
    );
    assert_eq!(
        abort.progress_lines as usize, progress_calls,
        "the reported count is what the sink was actually asked to print"
    );
    let _ = std::fs::remove_dir_all(&dir);
}

// ─── (j) the wait loop never RECONNECTS ──────────────────────────────────────

/// The loop's round trips must NOT carry the
/// first-request reconnect, or the documented ~15 s wall is false by up to
/// ~20 s.
///
/// `query_catalog_with_discovery` retries its FIRST request through `reconnect()`,
/// and inside this loop the first round trip always IS the first request
/// (`next_id == 1`). `reconnect()` re-runs the whole connect-or-spawn ladder — which
/// its own docs bound at ~2 × `SPAWN_READY_TIMEOUT` — so that one path could add more
/// than the entire ceiling to a wall function that models a single `ROUNDTRIP_TIMEOUT`.
///
/// The oracle is the fake daemon's ACCEPT COUNT, which is deterministic and needs no
/// wall assertion: a daemon that serves its `Hello` and then hangs up WITHOUT
/// answering makes the first round trip fail with an early close — exactly
/// `is_closed_early`'s trigger. A reconnecting loop dials the still-listening socket
/// again and the listener accepts a SECOND connection; a non-reconnecting one aborts
/// on the first.
#[test]
fn the_wait_loop_does_not_reconnect_on_an_early_close() {
    let dir = socket_dir("noreconn");
    let (socket, listener) = bind_socket(&dir);
    let accepts = Arc::new(AtomicUsize::new(0));
    let counted = Arc::clone(&accepts);

    std::thread::spawn(move || {
        // Stay listening for a SECOND dial — that is the whole point of the oracle.
        for conn in listener.incoming().take(4) {
            let Ok(stream) = conn else { return };
            counted.fetch_add(1, Ordering::SeqCst);
            let mut w = stream.try_clone().expect("clone");
            let _ = writeln!(
                w,
                "{}",
                Hello {
                    hello: HELLO_MARKER.to_string(),
                    protocol: PROTOCOL_VERSION,
                }
                .to_json_line()
            );
            let _ = w.flush();
            // Hold the stream until the client has HANDSHAKED before hanging
            // up. Dropping it at the end of the loop body raced the client's
            // `finish_handshake`: if the close landed first, macOS turns the client's
            // `set_read_timeout` into EINVAL, and since `is_closed_early` classifies
            // `InvalidInput` as an early close, `connect_or_spawn_at` at
            // the SETUP line below would retry, dial this still-bound listener again,
            // and push `accepts` to 2 — failing the oracle with a message that
            // accuses the non-reconnecting seam of a regression it did not
            // have (`query_converged` never consults `is_closed_early`). Without that
            // classification the same race panics at `.expect("connect")`, so this is a
            // misattribution, not a new failure — but a test that fails by naming the
            // wrong subsystem is worse than one that fails plainly.
            //
            // The barrier reads to the NEWLINE, not one byte, for two
            // reasons. The client sends nothing until it has parsed the Hello, so this
            // returns only once the client is past `finish_handshake` — either with its
            // complete first request line, or at EOF if it went away.
            //
            // (a) A 1-byte read could complete BETWEEN the two `write_all`s that
            //     `writeln!` issues on an unbuffered stream (body, then `\n`), so
            //     dropping the stream there gave the client an EPIPE on its second
            //     write. Nothing asserts the kind today, but the comment above is the
            //     mechanism narrative someone will debug against, and "early close on
            //     the FIRST REQUEST" must name ONE syscall site, not two. Waiting for
            //     the newline means the request is fully written before we hang up, so
            //     the client's failure is always the read-side EOF on the response.
            // (b) `read_line` -> `read_until` RETRIES `ErrorKind::Interrupted`
            //     internally. A `let _ = read(..)` would discard an Err, so an
            //     EINTR would skip the barrier and silently restore the racy
            //     one-byte shape — a "structural, not probabilistic" claim that
            //     held only on the Ok path. Unreachable in this harness (no signal
            //     handlers), but the retry is free here rather than argued.
            let mut request = String::new();
            if let Err(e) = BufReader::new(&stream).read_line(&mut request) {
                // Not swallowed: any surviving error means the barrier did not hold,
                // so say so rather than hanging up as if it had.
                eprintln!("handshake barrier read failed ({e}) — the hang-up below may race the client's handshake");
            }
            // Hang up WITHOUT answering: the client's first request sees an early
            // close, which is precisely the reconnect trigger.
        }
    });

    let mut client = NetdClient::connect_or_spawn_at(socket).expect("connect");
    let running = live();
    let abort = {
        let mut sink = |_: Duration| {};
        let mut wait = FirstContactWait::new(policy(), &mut sink, Some(&running));
        client
            .query_catalog_converged(None, &mut wait)
            .expect_err("the first round trip must fail on the early close")
    };

    assert_eq!(
        accepts.load(Ordering::SeqCst),
        1,
        "the wait loop must NOT reconnect: a second accept means the \
         first-request retry is live inside the loop, and with it a connect-or-spawn \
         ladder the documented worst-case wall does not model"
    );
    assert_eq!(
        abort.progress_lines, 1,
        "only the pre-flight announcement was printed before the abort"
    );
    let _ = std::fs::remove_dir_all(&dir);
}

// ─── (k) the slice loop is load-bearing ──────────────────────────────────────

/// The sliced sleep must be the thing that
/// notices cancellation, not the per-poll boundary check.
///
/// The cancellation arm above runs at `TEST_POLL` = 20 ms, which is BELOW
/// `CANCEL_CHECK_SLICE` (100 ms), so `remaining.min(CANCEL_CHECK_SLICE)` is always
/// `remaining` and replacing the slice loop with one `thread::sleep(total)` still
/// passes it. This arm uses a poll interval well ABOVE the slice, and a ceiling far
/// longer than the test, so only the SLICING can end it in time.
#[test]
fn cancellation_is_noticed_inside_a_long_sleep_not_only_at_poll_boundaries() {
    let daemon = spawn_fake_daemon("slice", PROTOCOL_VERSION, vec![NOT_CONVERGED_EMPTY]);
    let mut client = connect(&daemon);
    // A 5 s poll interval: 50× the cancel slice. A loop that sleeps it whole cannot
    // observe the flag until the sleep ends.
    let long_poll = ConvergenceWait::new(Duration::from_secs(60), Duration::from_secs(5));
    let running = Arc::new(AtomicBool::new(true));
    let flipper = Arc::clone(&running);
    std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(150));
        flipper.store(false, Ordering::Relaxed);
    });

    let started = Instant::now();
    let out = {
        let mut sink = |_: Duration| {};
        let mut wait = FirstContactWait::new(long_poll, &mut sink, Some(&running));
        client
            .query_catalog_converged(None, &mut wait)
            .expect("the catalog query runs")
    };
    let wall = started.elapsed();

    assert_eq!(out.outcome, WaitOutcome::Cancelled);
    assert!(
        wall < Duration::from_secs(3),
        "the flag flipped 150 ms in, inside a 5 s sleep — an unsliced sleep would not \
         notice until it ended. wall={wall:?}"
    );
}

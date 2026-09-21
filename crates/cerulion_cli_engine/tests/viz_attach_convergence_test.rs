// SPDX-License-Identifier: AGPL-3.0-only
//! The CLIENT-side first-contact wait for `cerulion viz`'s attach, driven end
//! to end against a SCRIPTED fake `cerulion-vizd` over a real Unix socket.
//!
//! # Why the loop is on this side
//!
//! Real-LAN convergence was measured at ~3 to 13 s, so a cold desk's FIRST attach
//! lands before the robot is discoverable. The obvious fix — have `cerulion-vizd` wait
//! before replying — is unavailable, and that is not a preference: `viz_client` arms
//! `CONN_IO_TIMEOUT` (5 s) as `SO_RCVTIMEO` on EVERY reply read, so a longer reply does
//! not arrive late, it never arrives, and `cerulion viz` dies on an errno with the topics
//! unattached. `the_socket_read_deadline_is_real_which_is_why_the_wait_is_client_side`
//! reproduces exactly that against a deliberately-slow daemon, so the design's premise
//! is a measured fact in this file rather than a claim in a comment.
//!
//! Everything else here is the loop: it re-asks only while the daemon says the topic is
//! not discoverable YET, under `cerulion_netd`'s shipped pure policy.
//!
//! The fake daemon's ORACLE is its own SERVED-REQUEST COUNT — the thing a pure test
//! cannot see — bounded from ABOVE as well as below, which is what makes the sleep
//! observable: a loop that skipped it would hammer the daemon while every other
//! assertion stayed green.
#![cfg(unix)]

use std::io::{BufRead, BufReader, ErrorKind, Write};
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_cli_engine::viz_client::{AttachOutcome, AttachReply, VizdConn, WaitOutcome};
use cerulion_netd::protocol::DiscoveryState;
use cerulion_netd::ConvergenceWait;

/// A shrunk policy: the POLICY is `cerulion_netd`'s own oracle-tested state machine, so
/// shrinking the numbers changes the wall clock and nothing else.
const TEST_CEILING: Duration = Duration::from_millis(900);
const TEST_POLL: Duration = Duration::from_millis(100);

fn policy() -> ConvergenceWait {
    ConvergenceWait::new(TEST_CEILING, TEST_POLL)
}

/// The most polls `policy()` can start: `decide` is PREDICTIVE (it refuses to begin a
/// poll whose sleep would cross the ceiling), so the loop cannot exceed this.
fn max_polls() -> u32 {
    (TEST_CEILING.as_millis() / TEST_POLL.as_millis()) as u32
}

/// What the fake daemon answers for one attach request.
#[derive(Clone, Copy)]
enum Script {
    /// `ok:false` + a retry hint.
    Retryable {
        discovery: DiscoveryState,
        plane_unsettled_ms: Option<u64>,
    },
    /// `ok:false` with NO hint — terminal.
    Terminal,
    /// `ok:true`.
    Success,
    /// Serve the reply only after a delay (the `SO_RCVTIMEO` probe). The delay is
    /// slept in slices against the stop flag — see [`sleep_unless_stopped`] — so a
    /// delay far past the client's deadline costs the test no wall time.
    SlowSuccess(Duration),
}

/// How long [`sleep_unless_stopped`] sleeps before re-checking the stop flag.
const STOP_CHECK_SLICE: Duration = Duration::from_millis(20);

/// Sleep `total`, re-checking `stop` every [`STOP_CHECK_SLICE`]. `false` means the
/// daemon was stopped part-way through and must NOT answer.
///
/// Load-bearing for [`Script::SlowSuccess`]: `FakeVizd::drop` sets `stop` and
/// then JOINS the serving thread, so an UN-SLICED sleep makes every test with a slow
/// reply pay the whole scripted delay in wall time. That is what forced the delay to sit
/// just past the client's 5 s deadline (5.6 s — a 100 ms discriminating margin), which
/// is exactly the load-sensitive class: a correct run whose post-timeout wake landed late
/// failed a wall assertion. With the slice, a delay no runner can reach into is free.
fn sleep_unless_stopped(total: Duration, stop: &AtomicBool) -> bool {
    let deadline = Instant::now() + total;
    loop {
        if stop.load(Ordering::Relaxed) {
            return false;
        }
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return true;
        }
        std::thread::sleep(remaining.min(STOP_CHECK_SLICE));
    }
}

/// A scripted `cerulion-vizd` over a real `UnixListener`: writes the Hello banner on
/// accept, then answers each request line from `steps` (the LAST step repeats forever)
/// while counting how many it served.
struct FakeVizd {
    socket: PathBuf,
    dir: PathBuf,
    served: Arc<AtomicU32>,
    stop: Arc<AtomicBool>,
    handle: Option<std::thread::JoinHandle<()>>,
}

impl FakeVizd {
    fn start(tag: &str, steps: Vec<Script>) -> Self {
        static N: AtomicU32 = AtomicU32::new(0);
        let n = N.fetch_add(1, Ordering::Relaxed);
        let dir = std::env::temp_dir().join(format!("{tag}_{}_{n}", std::process::id()));
        std::fs::create_dir_all(&dir).expect("mk tempdir");
        let socket = dir.join("vizd.sock");
        let listener = UnixListener::bind(&socket).expect("bind");
        listener.set_nonblocking(true).expect("nonblocking");

        let served = Arc::new(AtomicU32::new(0));
        let stop = Arc::new(AtomicBool::new(false));
        let handle = {
            let served = Arc::clone(&served);
            let stop = Arc::clone(&stop);
            std::thread::spawn(move || {
                while !stop.load(Ordering::Relaxed) {
                    match listener.accept() {
                        Ok((s, _)) => serve_conn(s, &steps, &served, &stop),
                        Err(_) => std::thread::sleep(Duration::from_millis(5)),
                    }
                }
            })
        };
        Self {
            socket,
            dir,
            served,
            stop,
            handle: Some(handle),
        }
    }

    fn served(&self) -> u32 {
        self.served.load(Ordering::Relaxed)
    }

    fn connect(&self) -> VizdConn {
        let deadline = Instant::now() + Duration::from_secs(3);
        loop {
            match VizdConn::connect(&self.socket) {
                Ok(c) => return c,
                Err(e) if Instant::now() < deadline => {
                    let _ = e;
                    std::thread::sleep(Duration::from_millis(5));
                }
                Err(e) => panic!("connect: {e}"),
            }
        }
    }
}

impl Drop for FakeVizd {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
        let _ = std::fs::remove_dir_all(&self.dir);
    }
}

fn serve_conn(stream: UnixStream, steps: &[Script], served: &AtomicU32, stop: &AtomicBool) {
    let mut writer = match stream.try_clone() {
        Ok(w) => w,
        Err(_) => return,
    };
    stream
        .set_read_timeout(Some(Duration::from_millis(100)))
        .ok();
    let mut reader = BufReader::new(stream);
    // Hello banner (the shape `VizdConn::connect` parses).
    if writeln!(writer, r#"{{"vizd":"fake","protocol":1,"rerun_url":null}}"#).is_err() {
        return;
    }
    // The read timeout above lets the thread notice `stop`, but `read_line` can return
    // `WouldBlock` having ALREADY appended a partial line — so the accumulator lives
    // OUTSIDE the loop and is only consumed once a full line has arrived. Reading into
    // a fresh `String` each iteration silently dropped those partial reads, which
    // surfaced as a client-side 5 s timeout on a request the daemon never saw.
    let mut line = String::new();
    loop {
        if stop.load(Ordering::Relaxed) {
            return;
        }
        match reader.read_line(&mut line) {
            Ok(0) => return,
            Ok(_) => {}
            Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => continue,
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
            Err(_) => return,
        }
        if !line.ends_with('\n') {
            // A partial line: keep what we have and read the rest.
            continue;
        }
        let line = std::mem::take(&mut line);
        if line.trim().is_empty() {
            continue;
        }
        let n = served.fetch_add(1, Ordering::Relaxed) as usize;
        let step = steps[n.min(steps.len() - 1)];
        let body = match step {
            Script::Success => r#"{"id":1,"ok":true,"topic":"/t","schema":"s"}"#.to_string(),
            Script::SlowSuccess(d) => {
                if !sleep_unless_stopped(d, stop) {
                    return;
                }
                r#"{"id":1,"ok":true,"topic":"/t","schema":"s"}"#.to_string()
            }
            Script::Terminal => {
                r#"{"id":1,"ok":false,"topic":"/t","error":"terminal"}"#.to_string()
            }
            Script::Retryable {
                discovery,
                plane_unsettled_ms,
            } => {
                let d = match discovery {
                    DiscoveryState::Settled => "settled",
                    DiscoveryState::NotConverged => "discovering",
                };
                let age = match plane_unsettled_ms {
                    Some(ms) => format!(r#","plane_unsettled_ms":{ms}"#),
                    None => String::new(),
                };
                format!(
                    r#"{{"id":1,"ok":false,"topic":"/t","error":"not yet","retry":{{"discovery":"{d}"{age}}}}}"#
                )
            }
        };
        if writeln!(writer, "{body}").is_err() {
            return;
        }
    }
}

/// Drive the waiting attach and hand back the REPLY (most arms assert on the daemon's
/// served-request count, not on the outcome).
fn attach(conn: &mut VizdConn, running: Option<&AtomicBool>) -> std::io::Result<AttachReply> {
    attach_outcome(conn, running).map(|o| o.reply)
}

/// Drive the waiting attach and hand back the full [`AttachOutcome`] — for the arms that
/// assert on HOW the wait ended (a cancelled wait must not be reported
/// as a discovery verdict).
fn attach_outcome(
    conn: &mut VizdConn,
    running: Option<&AtomicBool>,
) -> std::io::Result<AttachOutcome> {
    let mut sink = |_: Duration| {};
    conn.attach_waiting_for_discovery(1, "/t", None, None, None, policy(), running, &mut sink)
}

/// THE headline: a not-discoverable-YET answer is RE-ASKED, and the loop settles on the
/// answer that finally succeeded. Hand oracle: exactly 3 requests served.
#[test]
fn a_not_found_yet_attach_is_re_asked_until_the_topic_appears() {
    let daemon = FakeVizd::start(
        "reask",
        vec![
            Script::Retryable {
                discovery: DiscoveryState::NotConverged,
                plane_unsettled_ms: Some(0),
            },
            Script::Retryable {
                discovery: DiscoveryState::NotConverged,
                plane_unsettled_ms: Some(0),
            },
            Script::Success,
        ],
    );
    let mut conn = daemon.connect();
    let reply = attach(&mut conn, None).expect("no transport error");
    assert!(reply.ok, "the attach succeeded once the topic appeared");
    assert_eq!(daemon.served(), 3, "one request per scripted step");
}

/// A daemon that vouches for its discovery (`settled`) is believed on the FIRST ask —
/// the convergence-wait rule, so a typo on a warm desk costs nothing.
///
/// Paired IN BODY with the `discovering` twin over the identical script, so a loop that
/// never waited at all cannot pass: the second half must serve MORE than one request.
#[test]
fn a_settled_hint_answers_at_once_while_a_discovering_one_keeps_asking() {
    let settled = FakeVizd::start(
        "settled",
        vec![Script::Retryable {
            discovery: DiscoveryState::Settled,
            plane_unsettled_ms: None,
        }],
    );
    let mut conn = settled.connect();
    let reply = attach(&mut conn, None).expect("no transport error");
    assert!(!reply.ok);
    assert_eq!(
        settled.served(),
        1,
        "a settled LAN is authoritative about NOW — never re-ask it in a loop"
    );

    let discovering = FakeVizd::start(
        "discovering",
        vec![Script::Retryable {
            discovery: DiscoveryState::NotConverged,
            plane_unsettled_ms: Some(0),
        }],
    );
    let mut conn = discovering.connect();
    let reply = attach(&mut conn, None).expect("no transport error");
    assert!(!reply.ok, "it never converged, so it still fails");
    assert!(
        discovering.served() > 1,
        "ANTI-TAUTOLOGY: an unconverged plane really is re-asked ({} requests)",
        discovering.served()
    );
    assert!(
        discovering.served() <= max_polls(),
        "and the predictive ceiling bounds it: {} > {}",
        discovering.served(),
        max_polls()
    );
}

/// A TERMINAL failure (no hint) is never re-asked. This is what stops the loop turning
/// every unrelated attach error (a bad topic name, a tap failure, the kill switch, or
/// any older daemon) into a multi-second stall.
#[test]
fn a_failure_with_no_hint_is_never_re_asked() {
    let daemon = FakeVizd::start("terminal", vec![Script::Terminal]);
    let mut conn = daemon.connect();
    let reply = attach(&mut conn, None).expect("no transport error");
    assert!(!reply.ok);
    assert!(reply.retry.is_none());
    assert_eq!(daemon.served(), 1, "terminal means terminal");
}

/// The PLANE-AGE cap: a daemon whose query plane already out-waited the ceiling
/// buys no wait, so a robot-less desk does not pay it on every attach forever.
///
/// Both halves in one body — the same script, differing only in the age reported — so a
/// cap that suppressed EVERY wait cannot pass.
#[test]
fn a_plane_that_already_out_waited_the_ceiling_buys_no_wait_but_a_young_one_does() {
    let old = FakeVizd::start(
        "old_plane",
        vec![Script::Retryable {
            discovery: DiscoveryState::NotConverged,
            plane_unsettled_ms: Some(TEST_CEILING.as_millis() as u64 * 2),
        }],
    );
    let mut conn = old.connect();
    attach(&mut conn, None).expect("no transport error");
    assert_eq!(old.served(), 1, "an old plane buys no wait");

    let young = FakeVizd::start(
        "young_plane",
        vec![Script::Retryable {
            discovery: DiscoveryState::NotConverged,
            plane_unsettled_ms: Some(0),
        }],
    );
    let mut conn = young.connect();
    attach(&mut conn, None).expect("no transport error");
    assert!(
        young.served() > 1,
        "ANTI-TAUTOLOGY: a young plane still waits ({} requests)",
        young.served()
    );
}

/// Cancellation (the verb's Ctrl-C flag) ends the wait promptly — `cerulion` replaces
/// the default SIGINT disposition, so an unsliced sleep would make Ctrl-C a no-op.
///
/// # The claim is carried by the OUTCOME and the poll COUNT, never by a wall
///
/// "Promptly" is not `elapsed < TEST_CEILING` — 900 ms against a flag that flips
/// at 200 ms would be a 700 ms allowance on a machine that can preempt a thread for
/// longer than that. A correct run on a loaded runner would fail it, which is the
/// load-sensitive class (see the socket-deadline arm's own note).
///
/// What actually separates "stopped on the flag" from "ran to the ceiling" is not a
/// duration at all: a ceiling-exhausted wait returns `GaveUp` after `max_polls()`
/// round trips, and a cancelled one returns `Cancelled` after fewer.
///
/// # The STIMULUS is gated on a condition, not on a timer
///
/// The ASSERTIONS are load-immune — contention can only make a wait serve FEWER
/// polls, never more. The STIMULUS was not.
///
/// The watcher used to flip the flag after a fixed `TEST_POLL * 2`. That is a wait
/// nested inside the wait under test: delay the watcher past the ceiling and the
/// loop runs to `GaveUp`, inverting the verdict on correct code. It is thin in
/// exactly the direction already MEASURED on this repo's own CI — macOS background
/// QoS charges timer slack per WAKEUP, a 150 ms nominal sleep returning in
/// 1100-1696 ms — against a 900 ms ceiling.
///
/// So the flag flips once the daemon has SERVED a request, i.e. once the loop is
/// provably inside the wait. Load can delay that sighting, but the loop cannot
/// reach its ceiling without serving the very requests the watcher is counting, so
/// the two cannot race: whatever the machine does, the flag lands while the
/// wait is running.
#[test]
fn a_cancelled_wait_stops_promptly() {
    let daemon = FakeVizd::start(
        "cancel",
        vec![Script::Retryable {
            discovery: DiscoveryState::NotConverged,
            plane_unsettled_ms: Some(0),
        }],
    );
    let mut conn = daemon.connect();
    let running = Arc::new(AtomicBool::new(true));
    {
        let running = Arc::clone(&running);
        let served = Arc::clone(&daemon.served);
        std::thread::spawn(move || {
            // CONDITION, not a timer: flip once the loop has provably entered the
            // wait (one request served). A fixed sleep here is a wait nested inside
            // the wait under test, and delaying it past the ceiling inverts the
            // verdict on correct code.
            let deadline = Instant::now() + Duration::from_secs(10);
            while served.load(Ordering::Relaxed) == 0 && Instant::now() < deadline {
                std::thread::sleep(Duration::from_millis(1));
            }
            running.store(false, Ordering::Relaxed);
        });
    }
    let out = attach_outcome(&mut conn, Some(running.as_ref())).expect("no transport error");
    assert_eq!(
        out.outcome,
        WaitOutcome::Cancelled,
        "the wait ended on the FLAG — a wait that ran to its ceiling reports GaveUp"
    );
    // ANTI-VACUITY: the flag was flipped because a request was SEEN, not because
    // the watcher's own deadline expired. Without this a watcher whose condition
    // never fired would still cancel, and the arm would be timing nothing.
    assert!(
        daemon.served() >= 1,
        "the daemon must have served the request the watcher waited on"
    );
    assert!(
        daemon.served() < max_polls(),
        "and it stopped re-asking: {} requests",
        daemon.served()
    );
}

/// **An interrupted wait is reported as an INTERRUPTION, never as a
/// discovery verdict.**
///
/// The first cut returned a bare `AttachReply` from all four exits, so `cerulion viz`
/// could not tell a wait the user stopped at 2 s from one that ran its full budget: it
/// printed the per-topic UNKNOWN ABSENCE paragraph, counted a failure, and exited
/// nonzero for a question nobody finished asking. That is exactly what
/// `WaitOutcome::Cancelled` exists to prevent — "NO give-up claim may be made from it" —
/// re-introduced one surface over.
///
/// Both outcomes are asserted in ONE body against the SAME script, so a loop that
/// hardcoded either answer cannot pass: cancel ⇒ `Cancelled`, run to budget ⇒ `GaveUp`.
#[test]
fn an_interrupted_wait_is_reported_as_cancelled_not_as_a_discovery_verdict() {
    let script = vec![Script::Retryable {
        discovery: DiscoveryState::NotConverged,
        plane_unsettled_ms: Some(0),
    }];

    // ── Interrupted mid-wait.
    let daemon = FakeVizd::start("cancel_outcome", script.clone());
    let mut conn = daemon.connect();
    let running = Arc::new(AtomicBool::new(true));
    {
        let running = Arc::clone(&running);
        std::thread::spawn(move || {
            std::thread::sleep(TEST_POLL * 2);
            running.store(false, Ordering::Relaxed);
        });
    }
    let out = attach_outcome(&mut conn, Some(running.as_ref())).expect("no transport error");
    assert_eq!(
        out.outcome,
        WaitOutcome::Cancelled,
        "a wait the USER stopped licenses no claim about the topic"
    );
    assert!(
        !out.reply.ok,
        "the reply itself is still handed back (empty, not-converged) — the caller must \
         not RENDER it, which is what the outcome tells it"
    );

    // ── ANTI-TAUTOLOGY: the same script, run to its budget, is a real verdict.
    let daemon = FakeVizd::start("giveup_outcome", script);
    let mut conn = daemon.connect();
    let out = attach_outcome(&mut conn, None).expect("no transport error");
    assert_eq!(
        out.outcome,
        WaitOutcome::GaveUp,
        "an uninterrupted wait that exhausted its budget IS a verdict — otherwise \
         `Cancelled` above could be hardcoded"
    );
}

/// An interruption that lands where the loop would otherwise reach a VERDICT is still
/// `Cancelled` — the POST-ROUND-TRIP check, isolated.
///
/// The sibling arm above cannot cover it: there the flag flips during the SLEEP, where
/// `sleep_cancellable` reports the cancellation on its own, so the earlier check is dead
/// weight (mutating it to `GaveUp` passes the whole suite — found by running it, the same
/// gap the vizd loop has to guard).
///
/// Here the flag is ALREADY clear and the hint carries a plane age past the ceiling, so
/// `decide` gives up on the FIRST round trip and there is no sleep to catch anything.
/// Without the post-round-trip check the loop reports `GaveUp` — a claim about the
/// network — for a run the user had already stopped. That is a real shape, not a
/// contrived one: the first round trip against a cold daemon can take seconds, and Ctrl-C
/// during it is exactly when a user reaches for it.
#[test]
fn an_interruption_at_the_verdict_is_still_reported_as_cancelled() {
    let daemon = FakeVizd::start(
        "cancel_at_verdict",
        vec![Script::Retryable {
            discovery: DiscoveryState::NotConverged,
            // Past the ceiling ⇒ the cap gives up on the first round trip.
            plane_unsettled_ms: Some(TEST_CEILING.as_millis() as u64 * 2),
        }],
    );
    let mut conn = daemon.connect();
    // Already interrupted before the attach begins.
    let running = Arc::new(AtomicBool::new(false));
    let out = attach_outcome(&mut conn, Some(running.as_ref())).expect("no transport error");
    assert_eq!(
        out.outcome,
        WaitOutcome::Cancelled,
        "an interrupted run must not be handed a verdict the loop happened to reach"
    );
    assert_eq!(daemon.served(), 1, "and it stopped after one round trip");
}

/// Cancellation is noticed INSIDE a long sleep, not only at poll boundaries.
///
/// The sibling arm above runs at `TEST_POLL` = 100 ms, which is exactly
/// `CANCEL_CHECK_SLICE` — so `remaining.min(slice)` is always `remaining` there and one
/// un-sliced `thread::sleep(total)` passes it. That is the identical gap
/// `cerulion_netd` guards against
/// (`cancellation_is_noticed_inside_a_long_sleep_not_only_at_poll_boundaries`); copying
/// the loop without copying the guard would re-open it.
///
/// Here the poll interval is [`UNSLICED_POLL`] (120 s) — 1200× the slice — so an
/// un-sliced sleep cannot come back before the flag is honoured, and Ctrl-C on
/// `cerulion viz` would be a no-op for two minutes. (It was 5 s when this arm was
/// written; the sweep widened it to put three orders of magnitude between SLICED
/// and UN-SLICED rather than one, so no amount of coalescing can close the gap.)
#[test]
fn cancellation_is_noticed_inside_a_long_sleep_not_only_at_poll_boundaries() {
    let daemon = FakeVizd::start(
        "slice",
        vec![Script::Retryable {
            discovery: DiscoveryState::NotConverged,
            plane_unsettled_ms: Some(0),
        }],
    );
    let mut conn = daemon.connect();
    let running = Arc::new(AtomicBool::new(true));
    let started = Instant::now();
    {
        let running = Arc::clone(&running);
        std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(300));
            running.store(false, Ordering::SeqCst);
        });
    }
    let mut sink = |_: Duration| {};
    let out = conn
        .attach_waiting_for_discovery(
            1,
            "/t",
            None,
            None,
            None,
            // A 120 s poll inside a 600 s ceiling: the loop's FIRST sleep is 1200x the
            // slice, and `decide` is PREDICTIVE so a 120 s sleep still fits its budget.
            ConvergenceWait::new(UNSLICED_SLEEP_CEILING, UNSLICED_POLL),
            Some(running.as_ref()),
            &mut sink,
        )
        .expect("no transport error");
    let elapsed = started.elapsed();
    // The OUTCOME first: it is load-immune, and a wait that ended any other way is a
    // different failure wearing this test's name.
    assert_eq!(
        out.outcome,
        WaitOutcome::Cancelled,
        "the flag was honoured at all"
    );
    // …and the WALL, which is the only thing that can separate "seen inside the
    // sleep" from "seen when the sleep ended" — both report `Cancelled`. It is
    // stated with a gap no runner can close rather than with a tight margin: the
    // healthy path returns ~300 ms (the flag flips then, plus at most one
    // `CANCEL_CHECK_SLICE`), an un-sliced `thread::sleep` returns at 120 s, and the
    // bound sits 100x above the first and 4x below the second. The pre-sweep version
    // asserted `< 3 s` against a 5 s interval — a 2 s discriminating margin, which is
    // the load-sensitive class: a runner that preempts this thread for 3 s fails a correct
    // run, and nothing about the 5 s bought that risk.
    assert!(
        elapsed < UNSLICED_SLEEP_BOUND,
        "cancellation must be seen INSIDE the sleep — an un-sliced `thread::sleep` \
         would return only after the whole {UNSLICED_POLL:?} poll interval: {elapsed:?}"
    );
    // # WHAT THIS ARM DELIBERATELY DOES NOT ASSERT: promptness
    //
    // A slice that regressed from 100 ms to, say, 2 s still SLICES — it returns
    // ~2 s after the flag, far under the ceiling above — so this arm cannot see
    // it. That is a real gap, and it is closed by TWO pins, neither of them a
    // wall: a CONSTANT pin on the VALUE
    // (`viz_client::tests::the_cancellation_slice_stays_inside_its_ux_bounds`)
    // and a SOURCE walk on the EXPRESSION
    // (`the_wait_loops_sleep_is_bounded_by_the_configured_cancellation_slice`,
    // below). Neither is redundant: the constant pin alone passes a
    // `sleep_cancellable` that ignores `CANCEL_CHECK_SLICE` and hard-codes 2 s, and
    // the walk alone passes a loop that faithfully uses a constant somebody
    // widened.
    //
    // The pair covers the two ways a slice can grow, and the limit is
    // that this is a claim about VALUE and SHAPE, not a proof about elapsed time —
    // the walk asserts the reviewed expression rather than deriving a bound from
    // it, which is why a refactor there needs a human. A walk asserting only
    // that the argument NAMES the constant passes a `CANCEL_CHECK_SLICE * 20`
    // widening; see
    // `SHIPPED_SLEEP_ARGUMENT`. Closing that gap with a tight `return − cancellation`
    // wall is UNSOUND ON
    // BOTH SIDES:
    //
    // * It WORKED, which is worth recording because it is not what the shape
    //   suggests. The reading is not uniform over `(0, S]`: this arm's watcher
    //   flips at a FIXED 300 ms while the loop's first sleep starts within a
    //   millisecond of the first round trip, so it reads `S − 300 ms` every time.
    //   MEASURED under a 2 s slice: 12 of 12 kills, all at ~1.72 s.
    //
    // * What is thin is the DEFECT-side margin: 1.72 s against a 1.5 s bound is
    //   1.15x. And it is thin in the direction already measured on this repo's own
    //   CI — macOS background QoS charges timer slack per WAKEUP, a 150 ms nominal
    //   sleep coming back in 1100-1696 ms, which puts a HEALTHY 100 ms slice
    //   inside that failing band while this binary runs 16 tests at default
    //   parallelism on ~4-core runners. NOT reproduced here (20 of 20 green under
    //   `taskpolicy -b` plus concurrent build load), so: a documented mechanism,
    //   not an observed failure.
    //
    // Either way a constant is the wrong thing to measure with a clock, so the
    // slice is pinned as a CONSTANT and this wall keeps only the job a wall does
    // well.
    //
    // So the wall here is left to do the ONE job a wall can do soundly: separate
    // SLICED from UN-SLICED, where the two differ by three orders of magnitude
    // (~100 ms of slice granularity against a 120 s poll interval) and no amount
    // of coalescing or phase can close the gap.
}

/// The poll interval, ceiling and bound for
/// [`cancellation_is_noticed_inside_a_long_sleep_not_only_at_poll_boundaries`].
///
/// Named constants because the three are ONE argument: the bound is meaningful only
/// as a ratio to the other two, and a reader changing one in isolation silently
/// re-opens the margin the sweep closed. The interval is 1200x
/// `CANCEL_CHECK_SLICE`, so an un-sliced sleep cannot come back before it.
const UNSLICED_POLL: Duration = Duration::from_secs(120);
/// The ceiling the poll above sits inside — `decide` is PREDICTIVE and refuses to
/// start a sleep that would cross it, so this must exceed [`UNSLICED_POLL`].
const UNSLICED_SLEEP_CEILING: Duration = Duration::from_secs(600);
/// 100x the healthy return, a quarter of the un-sliced one.
const UNSLICED_SLEEP_BOUND: Duration = Duration::from_secs(30);

/// The sleep is REAL — a loop that skipped it would serve the same poll count in
/// microseconds while hammering the shared daemon.
#[test]
fn the_wait_really_sleeps_between_polls() {
    let daemon = FakeVizd::start(
        "sleeps",
        vec![Script::Retryable {
            discovery: DiscoveryState::NotConverged,
            plane_unsettled_ms: Some(0),
        }],
    );
    let mut conn = daemon.connect();
    let started = Instant::now();
    attach(&mut conn, None).expect("no transport error");
    let elapsed = started.elapsed();
    let served = daemon.served();
    assert!(served > 1, "it re-asked");
    // Each of the `served - 1` gaps is one poll interval.
    let floor = TEST_POLL * (served - 1);
    assert!(
        elapsed >= floor,
        "waited {elapsed:?} over {served} requests, below the {floor:?} the poll \
         interval demands — the sleep is not happening"
    );
}

/// **THE DESIGN'S PREMISE, MEASURED.** `viz_client` arms `CONN_IO_TIMEOUT` (5 s) as
/// `SO_RCVTIMEO` on every reply read, so a daemon that takes longer than that to answer
/// does not make `cerulion viz` slower — it makes the verb FAIL.
///
/// The wait is client-side because a 10 s convergence wait inside vizd's attach
/// handler is unreachable through the only control client this repo ships. This
/// arm reproduces the failure directly (a deliberately-slow
/// daemon ⇒ an `Err`, not a late success), so the premise is a fact here rather than an
/// argument in a comment.
///
/// It also pins the DESYNC guard: a failed round trip leaves the daemon's reply
/// still in flight, so the connection is poisoned and the NEXT request refuses loudly
/// rather than reading the previous request's answer and mis-attributing it.
///
/// # What carries the claim
///
/// The claim is "the verb failed BY THE DEADLINE, not by waiting for the slow answer",
/// and it used to rest entirely on a wall: the daemon answered at 5.6 s and the assertion
/// demanded `< 5.5 s`, i.e. a 500 ms load allowance against a **100 ms** discriminating
/// margin. That is the load-sensitive class, and it fired: a correct run whose post-timeout
/// wake landed late on a loaded CI runner measured 5.502669154 s and failed, with the
/// errno in its own diagnostic (`os error 11` = `EAGAIN`) proving the mechanism had
/// worked. So the load-bearing assertion is now the ERROR KIND, which load cannot fake,
/// and the wall is a generous secondary bound with a 5 s margin either side.
#[test]
fn the_socket_read_deadline_is_real_which_is_why_the_wait_is_client_side() {
    // Far past the 5 s deadline — a gap no runner can close. It costs nothing: the
    // scripted sleep is sliced against the stop flag, so this test still ends at the
    // client's deadline (~5 s) and `FakeVizd::drop` joins immediately.
    const ANSWER_AT: Duration = Duration::from_secs(20);
    // Generously ABOVE the 5 s deadline (10 s of load allowance) and generously BELOW
    // `ANSWER_AT` (5 s), so neither side is a coin flip on a contended runner.
    const GAVE_UP_BY: Duration = Duration::from_secs(15);

    let daemon = FakeVizd::start("slow", vec![Script::SlowSuccess(ANSWER_AT)]);
    let mut conn = daemon.connect();
    let started = Instant::now();
    let err = attach(&mut conn, None).expect_err("a reply past the deadline must fail");
    let elapsed = started.elapsed();
    // THE load-immune half. `SO_RCVTIMEO` surfaces as `EAGAIN`, and `std` maps errnos
    // through ONE unix table (`sys/io/error/unix.rs`: `EAGAIN | EWOULDBLOCK =>
    // WouldBlock`) shared by Linux and macOS, so both platforms report `WouldBlock`
    // here — which is also what the CI flake's own `os error 11` says. `TimedOut` is
    // accepted so the assertion stays true wherever the same deadline is reported that
    // way; anything else — `InvalidData` (a mangled reply), `BrokenPipe` (a dead
    // daemon), the desync `Other` — is a DIFFERENT failure wearing this test's name.
    assert!(
        matches!(err.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut),
        "the round trip must fail on the READ DEADLINE, not on some other error: \
         {:?} ({err})",
        err.kind()
    );
    // Anti-vacuity: the read really BLOCKED. The ceiling below cannot see an INSTANT
    // failure, and this floor can only be violated by a run that returned FASTER, which
    // load cannot cause. It sits far under the 5 s deadline on purpose, so it stays true
    // for any deadline the sibling arm admits (that one requires a 1.5 s reply to
    // SUCCEED, so the deadline can never legitimately drop below ~1.5 s).
    assert!(
        elapsed >= Duration::from_secs(1),
        "the round trip returned at once — it cannot have waited out a reply \
         deadline: {elapsed:?} ({err})"
    );
    assert!(
        elapsed < GAVE_UP_BY,
        "it gave up ON THE DEADLINE, before the daemon answered at {ANSWER_AT:?}: \
         {elapsed:?} ({err})"
    );
    assert!(
        conn.is_desynced(),
        "a timed-out round trip leaves a reply in flight — the connection is poisoned"
    );
    let again = conn
        .attach(2, "/t", None, None, None)
        .expect_err("a desynced connection must refuse");
    assert!(
        again.to_string().contains("desynced"),
        "and it refuses LOUDLY rather than pairing the stale reply with a new \
         request: {again}"
    );
}

/// The memo arms on a spent ceiling, **not on the first sight of a
/// no-age hint**.
///
/// Measured on the earlier code: an attach that saw one `NotConverged`/no-age hint and
/// then SUCCEEDED at 110 ms armed the memo anyway, so the next topic got 16 µs of wait
/// and gave up — ZERO ceilings paid all run, on the DESIGNED happy path (a cold desk
/// converging after a poll or two). The memo must only ever suppress a wait that has
/// already been proven not to pay off.
///
/// The sibling memo arm cannot see this: it scripts attach #1 as permanently retryable,
/// so a ceiling IS spent there and arm-on-sighting and arm-on-exhaustion agree. Here
/// attach #1 CONVERGES on its second round trip, which is the only shape that separates
/// them.
#[test]
fn a_no_age_hint_on_an_attach_that_then_succeeds_does_not_arm_the_memo() {
    let daemon = FakeVizd::start(
        "converged_no_age",
        vec![
            // Attach #1: one no-age hint, then the topic appears.
            Script::Retryable {
                discovery: DiscoveryState::NotConverged,
                plane_unsettled_ms: None,
            },
            Script::Success,
            // Attach #2 onward: the same daemon, still reporting no plane age.
            Script::Retryable {
                discovery: DiscoveryState::NotConverged,
                plane_unsettled_ms: None,
            },
        ],
    );
    let mut conn = daemon.connect();

    let first = attach_outcome(&mut conn, None).expect("no transport error");
    assert_eq!(
        first.outcome,
        WaitOutcome::Answered,
        "attach #1 CONVERGED — no ceiling was spent, so nothing was proven about the daemon"
    );
    assert_eq!(daemon.served(), 2, "one hint, then the success");

    attach_outcome(&mut conn, None).expect("no transport error");
    assert!(
        daemone_second_attach_polls(&daemon),
        "attach #2 must still WAIT: the memo may only fire once a ceiling has actually \
         been spent, and attach #1 succeeded. Served {} requests in total (3 means the \
         memo armed on the mere SIGHTING of a no-age hint and suppressed a wait that \
         would have paid off).",
        daemon.served()
    );
}

/// Did the SECOND attach issue more than one request? (Requests 1-2 belong to attach #1.)
fn daemone_second_attach_polls(daemon: &FakeVizd) -> bool {
    daemon.served() > 3
}

/// With the memo ARMED, Ctrl-C is still honoured.
///
/// If the memo's early return sits above the cancellation check, on the stale-daemon path
/// every remaining topic ignores the flag: absence paragraphs, counted failures, no
/// interrupted line, a NONZERO exit — contradicting the exit-0 claim in `main.rs` six
/// lines below the offending return.
///
/// Attach #1 exhausts its budget against a no-age daemon (arming the memo); attach #2
/// runs with the flag ALREADY clear and must report `Cancelled`, not `GaveUp`.
#[test]
fn an_armed_memo_still_honours_cancellation() {
    let daemon = FakeVizd::start(
        "memo_cancel",
        vec![Script::Retryable {
            discovery: DiscoveryState::NotConverged,
            plane_unsettled_ms: None,
        }],
    );
    let mut conn = daemon.connect();

    // Attach #1: no cancellation, so it spends the ceiling and ARMS the memo.
    let first = attach_outcome(&mut conn, None).expect("no transport error");
    assert_eq!(first.outcome, WaitOutcome::GaveUp, "a ceiling was spent");
    let after_first = daemon.served();
    assert!(after_first > 1, "it really waited: {after_first}");

    // Attach #2: already interrupted. The memo must not swallow that.
    let running = Arc::new(AtomicBool::new(false));
    let second = attach_outcome(&mut conn, Some(running.as_ref())).expect("no transport error");
    assert_eq!(
        second.outcome,
        WaitOutcome::Cancelled,
        "an interrupted run must be reported as an INTERRUPTION even on the \
         stale-daemon path — a `GaveUp` here makes the verb print an absence claim and \
         exit nonzero for a question the user stopped us asking"
    );
    assert_eq!(
        daemon.served(),
        after_first + 1,
        "and it still costs exactly one round trip (the memo's saving is intact)"
    );
}

/// The ADOPTION pin: `cerulion viz` really calls the waiting verb.
///
/// Every arm above drives `attach_waiting_for_discovery` DIRECTLY, so all of them stay
/// green if the verb is reverted to the bare `conn.attach(...)` — the classic
/// inert-shipping shape. The verb's
/// attach loop needs a real vizd, a real netd and a real robot, so no behavioural test
/// in this repo can reach it; a source walk can.
///
/// Reading a sibling crate's source is deliberate and precedented (the guard
/// `every_hand_written_cdylib_init_applies_the_iox2_log_level` walks the whole repo for
/// the same reason): the CALLER lives in `cerulion_cli`, which is a thin clap wrapper
/// with no lib target, while the thing being adopted lives here. It fails CLOSED — an
/// unreadable path panics rather than passing.
#[test]
fn the_viz_verb_attaches_through_the_waiting_verb_not_the_bare_one() {
    let main_rs = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("cerulion_cli")
        .join("src")
        .join("main.rs");
    let src = std::fs::read_to_string(&main_rs)
        .unwrap_or_else(|e| panic!("read {}: {e}", main_rs.display()));

    assert!(
        src.contains("attach_waiting_for_discovery"),
        "`cerulion viz` must attach through the waiting verb — otherwise the convergence wait ships \
         inert: vizd emits a retry hint nobody acts on, and the first attach against a \
         cold desk fails exactly as it did before."
    );
    assert!(
        src.contains("first_contact_attach_policy"),
        "and under the SHARED policy, so Studio and `topic hz` do not wait different \
         amounts for the same daemon"
    );
    // The cancel flag has to be WIRED, not just accepted: `cerulion` replaces the
    // default SIGINT disposition before the verb runs, so passing `None` here would
    // make Ctrl-C a no-op for the whole ceiling with every other test still green.
    assert!(
        src.contains("Some(running.as_ref())"),
        "the verb must hand the wait its Ctrl-C flag — `None` would make the wait \
         uninterruptible while every behavioural arm stayed green"
    );
    // The verb must HONOUR the outcome. Carrying `Cancelled` out of the
    // loop is worthless if the caller renders the reply anyway, printing the UNKNOWN
    // absence paragraph plus a nonzero exit for a
    // question the user stopped us asking.
    assert!(
        src.contains("WaitOutcome::Cancelled"),
        "the verb must branch on the wait's terminal outcome — rendering a cancelled \
         wait's reply prints an ABSENCE CLAIM for a topic nobody finished asking about"
    );
    assert!(
        src.contains("interrupted while discovering"),
        "and report it as an interruption that concludes NOTHING about the topic"
    );
    // ANTI-TAUTOLOGY: the file really is the viz verb's source (a wrong path that
    // happened to exist would otherwise satisfy nothing above but say nothing either).
    assert!(
        src.contains("cerulion viz: attached"),
        "{} does not look like the viz verb's source",
        main_rs.display()
    );
}

/// The HEALTHY side of the deadline: a reply that arrives INSIDE it succeeds.
///
/// Its sibling above pins the deadline only from ABOVE, and that is not enough — a
/// variant shrinking `CONN_IO_TIMEOUT` from 5 s to 1 s passes every arm in this file,
/// because nothing requires a slow-but-legitimate reply to land. A cold vizd attach
/// costs one `cerulion-netd` round trip, which spends up to netd's
/// `COLD_START_DISCOVERY_BUDGET` (2.5 s) re-harvesting before it answers, so a 1 s
/// deadline would turn every cold attach into an errno — exactly the failure this whole
/// design exists to avoid, reintroduced from the other direction.
///
/// The delay is HARDCODED rather than derived from the constant on purpose: a delay
/// computed as a fraction of the deadline shrinks with it and that shrink escapes detection again.
/// 1.5 s sits above a plausible shrink and ~3.3x under the shipped 5 s, so a loaded
/// runner has wide margin. The arithmetic half — deadline vs netd's own budget — is
/// pinned in `viz_client`'s unit tests, where the private constant is visible.
#[test]
fn a_reply_that_arrives_inside_the_deadline_still_succeeds() {
    let daemon = FakeVizd::start(
        "healthy_slow",
        vec![Script::SlowSuccess(Duration::from_millis(1_500))],
    );
    let mut conn = daemon.connect();
    let started = Instant::now();
    let reply = conn.attach(1, "/t", None, None, None).expect("no error");
    let elapsed = started.elapsed();
    assert!(
        reply.ok,
        "a reply inside the deadline must be USED, not timed out"
    );
    assert!(
        elapsed >= Duration::from_millis(1_500),
        "anti-vacuity: the reply really was slow ({elapsed:?}), so this arm exercises \
         the deadline rather than a fast path"
    );
    assert!(
        !conn.is_desynced(),
        "and a successful round trip leaves the connection in step"
    );
}

/// The STALE-DAEMON MEMO — against a netd that cannot report a plane age, the
/// per-run cost is ONE ceiling, not one per topic.
///
/// `NotConverged` with no `plane_unsettled_ms` is an older daemon's signature, and the
/// plane-age cap cannot fire against it — so every not-found topic would pay the
/// full ceiling. Without the memo the cost is not "one ceiling per `cerulion viz`
/// run" but one per not-found TOPIC (measured); the memo is what makes the
/// per-run claim true.
///
/// Both halves in one body, so a memo that suppressed EVERY wait cannot pass: attach #1
/// still WAITS (the wait works against an old daemon — only the cap is missing, and
/// the design decided that bounded lateness beats a wrong claim), attach #2 does NOT.
#[test]
fn a_daemon_that_cannot_report_a_plane_age_costs_one_ceiling_per_run_not_per_topic() {
    let daemon = FakeVizd::start(
        "stale_plane",
        vec![Script::Retryable {
            discovery: DiscoveryState::NotConverged,
            // The older daemon's shape: the field is absent from the wire entirely.
            plane_unsettled_ms: None,
        }],
    );
    let mut conn = daemon.connect();

    attach(&mut conn, None).expect("no transport error");
    let after_first = daemon.served();
    assert!(
        after_first > 1,
        "the FIRST attach still waits against an old daemon ({after_first} requests) — \
         the wait works there, only the cap is missing"
    );

    attach(&mut conn, None).expect("no transport error");
    assert_eq!(
        daemon.served(),
        after_first + 1,
        "the SECOND attach costs exactly ONE round trip: the memo remembers that this \
         daemon cannot report a plane age, so the per-run cost is one ceiling rather \
         than one per topic"
    );

    // ANTI-TAUTOLOGY: a daemon that DOES report an age keeps waiting on every attach —
    // otherwise the memo above could simply be "never wait twice".
    let reporting = FakeVizd::start(
        "reporting_plane",
        vec![Script::Retryable {
            discovery: DiscoveryState::NotConverged,
            plane_unsettled_ms: Some(0),
        }],
    );
    let mut conn = reporting.connect();
    attach(&mut conn, None).expect("no transport error");
    let first = reporting.served();
    attach(&mut conn, None).expect("no transport error");
    assert!(
        reporting.served() > first + 1,
        "a daemon that reports its plane age is re-asked on EVERY attach ({} then {}) \
         — the cap, not a memo, is what bounds it there",
        first,
        reporting.served() - first
    );
}

/// A NORMAL round trip leaves the connection usable — the anti-tautology half of the
/// desync guard, without which "it refuses after a failure" would also pass a client
/// that refused always.
#[test]
fn a_healthy_round_trip_leaves_the_connection_usable() {
    let daemon = FakeVizd::start("healthy", vec![Script::Success]);
    let mut conn = daemon.connect();
    assert!(!conn.is_desynced());
    assert!(conn.attach(1, "/t", None, None, None).expect("ok").ok);
    assert!(!conn.is_desynced(), "still in step");
    assert!(conn.attach(2, "/t", None, None, None).expect("ok").ok);
    assert_eq!(daemon.served(), 2);
}

// ===========================================================================
// The wait loop's sleep is bounded by the CONFIGURED slice.
//
// `cancellation_is_noticed_inside_a_long_sleep_not_only_at_poll_boundaries`
// separates SLICED from UN-SLICED and says so; the constant pin
// (`viz_client::tests::the_cancellation_slice_stays_inside_its_ux_bounds`)
// bounds the VALUE. Between them sits one shape neither can see, and it is the
// shape a regression actually takes: a `sleep_cancellable` that still slices,
// still reports `Cancelled`, still returns far under the 30 s un-sliced
// ceiling — but slices at a coarse delay of its OWN, ignoring the constant the
// pin is busy bounding. Ctrl-C then takes ~2 s while both existing arms stay
// green and the constant reads a perfectly healthy 100 ms.
//
// A tighter WALL is not the instrument. The measurement is `return − flag`
// against a value that IS the loop's sleep slice, i.e. a phase draw dressed as
// a bound — a class already
// measured on this repo's own CI (a 150 ms nominal sleep
// charged as 1100-1696 ms under macOS background QoS). What is being asserted
// is not a duration at all: it is that ONE named constant governs the sleep, so
// the assertion is over the SOURCE, deterministic on every machine and every
// load.
// ===========================================================================

/// The source of `viz_client.rs` — the module holding the wait loop.
///
/// It fails CLOSED: an unreadable path panics rather than passing. Reading a
/// module's own source is the precedented shape here (see
/// `every_hand_written_cdylib_init_applies_the_iox2_log_level` and
/// `convergence_adoption_test`) and is the only shape available, because the
/// thing under test is WHICH constant an expression names — invisible to any
/// behavioural arm that is not itself a wall.
fn viz_client_source() -> String {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("src")
        .join("viz_client.rs");
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
}

/// The outcome of stripping comments and literals: the code-only view, plus
/// whether the scan finished BALANCED.
struct Stripped {
    code: String,
    /// Depth of unclosed `/*` at EOF. Anything but 0 means the tail was silently
    /// dropped — see [`code_only`].
    unclosed_depth: usize,
}

/// A view of `src` with `//`-to-end-of-line and `/* … */` comments removed, plus
/// string and char literals blanked.
///
/// Every part of that is load-bearing HERE specifically, not inherited caution:
///
/// * `viz_client.rs`'s doc comments NAME `CANCEL_CHECK_SLICE` repeatedly (that is
///   how the constant explains itself), so a comment-blind view would let a
///   `sleep_cancellable` that never mentions it pass on its own docs.
/// * Block comments NEST in Rust, so the scan is depth-tracked.
/// * STRING literals are blanked: regular, byte and RAW. A sibling file's own
///   stripper oracle holds `"a /* unterminated b"`, which
///   opened a block comment that never closed and dropped 42 % of the file,
///   leaving every negative assertion enforced over a prefix.
/// * CHAR literals holding a QUOTE (`'"'`) — a lone `"` there opens the STRING arm,
///   which scans to the next `"` anywhere and deletes everything between while
///   leaving `depth == 0`, so the loud guard below cannot see it. Only short shapes
///   are consumed, so a LIFETIME (`&'a str`) is never mistaken for one.
/// * The unclosed depth is REPORTED, so a caller fails LOUDLY rather than letting a
///   truncated view masquerade as a clean one.
fn code_only(src: &str) -> Stripped {
    let bytes = src.as_bytes();
    let mut out = String::with_capacity(src.len());
    let mut i = 0usize;
    let mut depth = 0usize;
    while i < bytes.len() {
        if depth == 0 {
            // A CHAR literal — but ONLY the short shapes, so a lifetime is never
            // eaten. `'x'` is 3 bytes, `'\n'` / `'\''` are 4; a lifetime has no
            // closing quote within that window.
            if bytes[i] == b'\'' {
                let close = if bytes.get(i + 1) == Some(&b'\\') {
                    (bytes.get(i + 3) == Some(&b'\'')).then_some(i + 3)
                } else {
                    (bytes.get(i + 2) == Some(&b'\'')).then_some(i + 2)
                };
                if let Some(end) = close {
                    out.push(' ');
                    i = end + 1;
                    continue;
                }
                // No closer in range ⇒ a lifetime; fall through and keep it.
            }
            // A RAW string literal: `r`, zero or more `#`, then `"`.
            if bytes[i] == b'r' {
                let mut j = i + 1;
                while j < bytes.len() && bytes[j] == b'#' {
                    j += 1;
                }
                if j < bytes.len() && bytes[j] == b'"' {
                    let hashes = j - i - 1;
                    let mut k = j + 1;
                    'raw: while k < bytes.len() {
                        if bytes[k] == b'"' {
                            let mut h = 0usize;
                            while h < hashes && k + 1 + h < bytes.len() && bytes[k + 1 + h] == b'#'
                            {
                                h += 1;
                            }
                            if h == hashes {
                                k = k + 1 + hashes;
                                break 'raw;
                            }
                        }
                        k += 1;
                    }
                    out.push(' ');
                    i = k.min(bytes.len());
                    continue;
                }
            }
            // A regular (or byte) string literal.
            if bytes[i] == b'"' {
                let mut k = i + 1;
                while k < bytes.len() {
                    match bytes[k] {
                        b'\\' => k += 2,
                        b'"' => {
                            k += 1;
                            break;
                        }
                        _ => k += 1,
                    }
                }
                out.push(' ');
                i = k.min(bytes.len());
                continue;
            }
            if bytes[i..].starts_with(b"//") {
                while i < bytes.len() && bytes[i] != b'\n' {
                    i += 1;
                }
                continue;
            }
        }
        if bytes[i..].starts_with(b"/*") {
            depth += 1;
            i += 2;
            continue;
        }
        if depth > 0 && bytes[i..].starts_with(b"*/") {
            depth -= 1;
            i += 2;
            continue;
        }
        if depth == 0 {
            // Push the whole UTF-8 character, never a partial byte.
            let ch = src[i..].chars().next().expect("valid utf-8 boundary");
            out.push(ch);
            i += ch.len_utf8();
        } else {
            i += 1;
        }
    }
    Stripped {
        code: out,
        unclosed_depth: depth,
    }
}

/// The comment-stripped code of `viz_client.rs`, with a LOUD failure if the scan
/// did not finish balanced (a truncated view makes every assertion below vacuous).
fn viz_client_code() -> String {
    let s = code_only(&viz_client_source());
    assert_eq!(
        s.unclosed_depth, 0,
        "the comment stripper ended inside {} unclosed block comment(s), so the tail of \
         viz_client.rs was DROPPED and every assertion below covers only a prefix",
        s.unclosed_depth
    );
    s.code
}

/// The body of `fn <name>(` in `src`, brace-matched from its opening `{`.
///
/// FUNCTION-SCOPED because a whole-file check answers the wrong question: this
/// module holds a legitimate bare `thread::sleep` in `ensure_daemon`'s connect
/// retry, so "the file contains a sleep that does not name the slice" is true of
/// correct code, and "the file names `CANCEL_CHECK_SLICE` somewhere" is true of a
/// `sleep_cancellable` that ignores it.
fn fn_body<'a>(src: &'a str, signature: &str) -> &'a str {
    let at = src
        .find(signature)
        .unwrap_or_else(|| panic!("`{signature}` not found in the stripped source"));
    let open = src[at..]
        .find('{')
        .unwrap_or_else(|| panic!("no opening brace after `{signature}`"))
        + at;
    let mut depth = 0usize;
    for (off, ch) in src[open..].char_indices() {
        match ch {
            '{' => depth += 1,
            '}' => {
                depth -= 1;
                if depth == 0 {
                    return &src[open..open + off + 1];
                }
            }
            _ => {}
        }
    }
    panic!("unbalanced braces after `{signature}`");
}

/// Every `sleep(…)` call in `body`, as the text of its ARGUMENT list.
///
/// The argument is paren-MATCHED, not read to the first `)`: the shipped call is
/// `sleep(remaining.min(CANCEL_CHECK_SLICE))`, so a first-`)` reader would hand
/// back `remaining.min(CANCEL_CHECK_SLICE` — right by luck here and wrong the
/// moment an expression's nesting changes.
///
/// `sleep_cancellable(…)` is NOT a sleep call: the token must be followed (past
/// whitespace) by `(`, and must not be preceded by an identifier byte, so neither
/// the wrapper nor a `my_sleep(` helper is mistaken for one. An unbalanced call
/// panics rather than yielding a truncated argument that could satisfy the pin.
fn sleep_call_arguments(body: &str) -> Vec<String> {
    let bytes = body.as_bytes();
    let mut found = Vec::new();
    let mut from = 0usize;
    while let Some(rel) = body[from..].find("sleep") {
        let at = from + rel;
        from = at + "sleep".len();
        // Not preceded by an identifier byte (`my_sleep`, `deep_sleep`).
        if at > 0 {
            let prev = bytes[at - 1];
            if prev.is_ascii_alphanumeric() || prev == b'_' {
                continue;
            }
        }
        // Followed, past whitespace, by `(` — so `sleep_cancellable(` is skipped
        // (its next byte is `_`) and `sleep (d)` is not. The whitespace test is
        // BYTE-exact rather than `bytes[open] as char`: that cast reads a UTF-8
        // continuation byte as a Latin-1 char, and 0x85 lands on U+0085 NEL,
        // which `char::is_whitespace` ACCEPTS — so a multi-byte character after
        // the token would be walked through as if it were a space.
        let mut open = from;
        while open < bytes.len() && bytes[open].is_ascii_whitespace() {
            open += 1;
        }
        if open >= bytes.len() || bytes[open] != b'(' {
            continue;
        }
        let mut depth = 0usize;
        let mut k = open;
        let mut close = None;
        while k < bytes.len() {
            match bytes[k] {
                b'(' => depth += 1,
                b')' => {
                    depth -= 1;
                    if depth == 0 {
                        close = Some(k);
                        break;
                    }
                }
                _ => {}
            }
            k += 1;
        }
        let close = close.unwrap_or_else(|| {
            panic!("unbalanced parentheses in a `sleep(` call at byte {open} of the body")
        });
        // A TRAILING COMMA is argument-list punctuation, not part of the argument
        // expression, and rustfmt writes one whenever it WRAPS a call — so without
        // this the same expression reads differently depending only on the width
        // it was formatted at. Found by the wrapped-format vector in
        // `the_source_walk_predicates_answer_their_hand_written_vectors`, which is
        // exactly what that vector is for.
        let arg = body[open + 1..close].trim();
        found.push(arg.strip_suffix(',').unwrap_or(arg).trim_end().to_string());
        from = close + 1;
    }
    found
}

/// The REVIEWED sleep expression, whitespace-squeezed.
///
/// Pinned by EQUALITY rather than by "the argument mentions the constant", because
/// naming a constant is not the same as being GOVERNED by it and the weaker form
/// was MEASURED to pass a widening: `remaining.min(CANCEL_CHECK_SLICE * 20)`
/// mentions it, still slices, still reports `Cancelled`, returns ~2 s (far under
/// the 30 s un-sliced ceiling), and leaves the constant pin reading a perfectly
/// healthy 100 ms — while Ctrl-C granularity degrades to 750 ms, 3x the
/// `UX_CEILING` that pin exists to enforce. `…min(CANCEL_CHECK_SLICE).max(2s)` is
/// the same class one step over, and no per-token adornment rule catches it: the
/// constant there is unadorned and the CONJUNCTION is what widens the sleep.
///
/// A structural walk cannot decide "is this expression bounded above by that
/// constant?" in general, so it asserts the one form a human has read. The cost is
/// stated rather than hidden: a legitimate rename or refactor fails this and must
/// be re-blessed deliberately, which for a value whose entire contract is a UX
/// bound is the posture we want.
const SHIPPED_SLEEP_ARGUMENT: &str = "remaining.min(CANCEL_CHECK_SLICE)";

/// `s` with ALL whitespace removed — the canonical form for comparing a Rust
/// expression whose formatting is not part of its meaning (`a.min(b)` and
/// `a . min ( b )` are one expression, and rustfmt may wrap either at any width).
fn squeeze(s: &str) -> String {
    s.chars().filter(|c| !c.is_whitespace()).collect()
}

/// THE pin the promptness comment above points at: the wait loop's sleep is
/// bounded by `CANCEL_CHECK_SLICE`, the constant its sibling unit test bounds.
///
/// Swapping the argument for
/// `remaining.min(Duration::from_secs(2))` leaves `CANCEL_CHECK_SLICE`
/// referenced by nothing (its `cancel_check_slice()` reader is `#[cfg(test)]`,
/// which does not apply when this crate is a dependency), so the crate's
/// `deny(dead_code)` REFUSES to compile it. That is a real, pre-existing guard
/// on exactly this one shape — and it is not a substitute for this pin, which
/// is why both live here: it cannot see the sibling arm below (a coarse delay
/// added ALONGSIDE an intact helper keeps the constant alive), and it cannot
/// see a helper that names the constant while widening it some other way.
///
/// With `#[allow(dead_code)]` added to the constant purely to make that variant
/// compile, it fails HERE and NOWHERE else in this binary — 18 of 19 green,
/// `cancellation_is_noticed_inside_a_long_sleep_not_only_at_poll_boundaries`
/// among them, since that variant still slices, still reports `Cancelled` and
/// still returns 28 s under the un-sliced ceiling. And
/// `viz_client::tests::the_cancellation_slice_stays_inside_its_ux_bounds` was
/// RUN under it too and PASSES, reading a perfectly healthy 100 ms: a gap
/// reproduced rather than assumed.
#[test]
fn the_wait_loops_sleep_is_bounded_by_the_configured_cancellation_slice() {
    let code = viz_client_code();
    let body = fn_body(&code, "fn sleep_cancellable");

    // ANTI-TAUTOLOGY, first: this really is the helper's body. A stripper or
    // brace-matcher that handed back the wrong span (or an empty one) would make
    // every assertion below pass by having nothing to check.
    for marker in ["is_cancelled", "saturating_duration_since", "deadline"] {
        assert!(
            body.contains(marker),
            "the extracted body does not look like `sleep_cancellable` (no `{marker}`) — \
             the source walk is reading the wrong span, so its verdict means nothing.\n\
             body was:\n{body}"
        );
    }

    let args = sleep_call_arguments(body);
    assert!(
        !args.is_empty(),
        "`sleep_cancellable` must actually SLEEP — a helper that spins instead would \
         satisfy every `every sleep names the constant` assertion vacuously.\nbody was:\n{body}"
    );
    for arg in &args {
        assert_eq!(
            squeeze(arg),
            SHIPPED_SLEEP_ARGUMENT,
            "`sleep_cancellable` sleeps on `{arg}`, which is not the reviewed \
             expression `{SHIPPED_SLEEP_ARGUMENT}`. Whatever it is, no behavioural arm \
             in this file can see it: a modified slice still SLICES, still reports \
             `Cancelled`, and still returns far under the un-sliced ceiling, while the \
             constant pin in `viz_client`'s unit tests keeps reading a healthy 100 ms. \
             If this is a deliberate refactor, re-bless `SHIPPED_SLEEP_ARGUMENT` — but \
             read the new expression first and satisfy yourself that Ctrl-C is still \
             noticed within one `CANCEL_CHECK_SLICE`, because that is the whole claim."
        );
    }
}

/// Its sibling, and the other half of the rule: the loop must sleep THROUGH the
/// cancellable helper, never on a coarse delay of its own.
///
/// Bounding the helper is not enough on its own — a loop that kept
/// `sleep_cancellable` intact and added its own `thread::sleep` between polls
/// would pass the pin above while making Ctrl-C exactly as slow. It is also the
/// one shape `deny(dead_code)` cannot help with, since the constant stays alive
/// through the untouched helper.
///
/// A bare `thread::sleep(Duration::from_secs(2))` before the
/// `sleep_cancellable` call compiles cleanly and fails here, naming the offending
/// argument. `a_not_found_yet_attach_is_re_asked_until_the_topic_appears` falls
/// with it — but as COLLATERAL rather than attribution: the extra 2 s per poll
/// simply overruns that arm's 900 ms test ceiling, which says nothing about
/// cancellation and would not fire for a coarse delay chosen a little smaller.
#[test]
fn the_wait_loop_sleeps_through_the_cancellable_helper_never_a_bare_thread_sleep() {
    let code = viz_client_code();
    let body = fn_body(&code, "fn attach_waiting_for_discovery");

    // ANTI-TAUTOLOGY: this is the wait loop, not some shorter function whose body
    // trivially satisfies a negative assertion.
    for marker in ["WaitDecision::KeepWaiting", "next_poll_delay"] {
        assert!(
            body.contains(marker),
            "the extracted body does not look like the wait loop (no `{marker}`) — the \
             source walk is reading the wrong span, so its verdict means nothing"
        );
    }

    assert!(
        body.contains("sleep_cancellable("),
        "the wait loop must sleep through `sleep_cancellable` — a bare `thread::sleep` \
         resumes across `EINTR`, so Ctrl-C would be a no-op for a whole poll interval"
    );
    let bare = sleep_call_arguments(body);
    assert!(
        bare.is_empty(),
        "the wait loop calls `sleep({})` directly. A coarse delay of the loop's own \
         making is uninterruptible for its whole length and is invisible to both the \
         sliced-vs-unsliced wall and the constant pin — which is the exact gap the \
         cancellable helper exists to close.",
        bare.join("`, `sleep(")
    );
}

/// The two predicates the pins assert THROUGH, against hand-written vectors.
///
/// A predicate that answers too easily makes its guard vacuous WITHOUT failing
/// anything, which is how a sibling guard can ship with a hole in
/// it. So the shapes that matter are fixed here directly — including the
/// coarse-slice shape, so the walk is known to reject it rather than assumed to.
#[test]
fn the_source_walk_predicates_answer_their_hand_written_vectors() {
    // --- sleep_call_arguments -------------------------------------------------
    // The SHIPPED shape: nested parens matched to the OUTER `)`.
    assert_eq!(
        sleep_call_arguments("std::thread::sleep(remaining.min(CANCEL_CHECK_SLICE));"),
        vec!["remaining.min(CANCEL_CHECK_SLICE)".to_string()],
        "a first-`)` reader would truncate the shipped call's own argument"
    );
    // The SHIPPED expression must satisfy the pin's own comparison, or the pin
    // asserts something the production code does not do.
    assert_eq!(
        squeeze(&sleep_call_arguments("std::thread::sleep(remaining.min(CANCEL_CHECK_SLICE));")[0]),
        SHIPPED_SLEEP_ARGUMENT
    );
    // …and so must the SAME expression after rustfmt wraps it, since formatting is
    // not part of an expression's meaning.
    assert_eq!(
        squeeze(
            &sleep_call_arguments(
                "std::thread::sleep(\n    remaining\n        .min(CANCEL_CHECK_SLICE),\n);"
            )[0]
        ),
        SHIPPED_SLEEP_ARGUMENT
    );

    // THE THREE SHAPES, each REJECTED here so the walk is KNOWN to refuse
    // them rather than assumed to. All three still SLICE, so no behavioural arm in
    // this file can tell any of them from the shipped code.
    for mutant in [
        // (1) A coarse slice of the helper's own.
        //     (A bare `contains` rule would refuse this one too.)
        "std::thread::sleep(remaining.min(Duration::from_secs(2)));",
        // (2) The constant WIDENED in place. This one NAMES `CANCEL_CHECK_SLICE`,
        //     so the `contains` rule ACCEPTED it: 750 ms of Ctrl-C latency behind a
        //     constant pin still reading a healthy 100 ms.
        "std::thread::sleep(remaining.min(CANCEL_CHECK_SLICE * 20));",
        // (3) The constant used FAITHFULLY and then out-ranked by a conjunction.
        //     The constant is UNADORNED here, so a per-token adornment rule accepts
        //     it as well — only the whole-expression shape refuses it.
        "std::thread::sleep(remaining.min(CANCEL_CHECK_SLICE).max(Duration::from_secs(2)));",
    ] {
        let args = sleep_call_arguments(mutant);
        assert_eq!(
            args.len(),
            1,
            "the extractor must still find the call in `{mutant}`"
        );
        assert_ne!(
            squeeze(&args[0]),
            SHIPPED_SLEEP_ARGUMENT,
            "`{mutant}` must be REJECTED by the pin's own comparison — otherwise the \
             walk is inert against it"
        );
    }
    // The WRAPPER is not a sleep call, or the loop's own guard would fire on the
    // very call it requires.
    assert!(
        sleep_call_arguments("if !sleep_cancellable(next_poll_delay, running) { }").is_empty(),
        "`sleep_cancellable(` must not be read as a `sleep(` call"
    );
    // Neither is an unrelated identifier that merely ENDS in `sleep`.
    assert!(sleep_call_arguments("my_sleep(x); deep_sleep(y);").is_empty());
    // Several calls in one body are all reported, in order.
    assert_eq!(
        sleep_call_arguments("thread::sleep(a); std::thread::sleep(b);"),
        vec!["a".to_string(), "b".to_string()]
    );
    // Whitespace before the paren is still a call; a token with no call is not.
    assert_eq!(
        sleep_call_arguments("sleep (c);"),
        vec!["c".to_string()],
        "`sleep (c)` is a call rustfmt would not write but the language accepts"
    );
    assert!(sleep_call_arguments("// how long we sleep between polls").is_empty());

    // --- code_only ------------------------------------------------------------
    let s = code_only("keep1 // gone\nkeep2 /* gone */ keep3");
    assert!(s.code.contains("keep1") && s.code.contains("keep2") && s.code.contains("keep3"));
    assert!(!s.code.contains("gone"));
    assert_eq!(s.unclosed_depth, 0);
    // Block comments NEST.
    let s = code_only("a /* one /* two */ still */ b");
    assert!(!s.code.contains("still") && s.code.contains('a') && s.code.contains('b'));
    // A `/*` inside a LINE comment is not an opener.
    assert_eq!(
        code_only("// /* not an opener\nreal_code").unclosed_depth,
        0
    );
    // String literals are blanked — including a raw one holding a comment opener,
    // which is what makes an unbalanced scan impossible to trigger from data.
    let s = code_only("let x = \"CANCEL_CHECK_SLICE\"; let y = r#\"also /* here\"#; z");
    assert!(
        !s.code.contains("CANCEL_CHECK_SLICE"),
        "a literal naming the constant must not satisfy the pin"
    );
    assert_eq!(s.unclosed_depth, 0);
    assert!(s.code.contains('z'), "the tail survives a raw literal");
    // A char literal holding a quote does not open the string arm…
    assert!(code_only("let q = '\"'; marker_after")
        .code
        .contains("marker_after"));
    // …while a LIFETIME is left alone.
    assert!(code_only("fn f<'a>(x: &'a str) -> &'a str { x }")
        .code
        .contains("'a"));
    // An unterminated block comment is REPORTED, not silently swallowed.
    assert_eq!(code_only("a /* unterminated").unclosed_depth, 1);
}

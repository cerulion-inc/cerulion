// SPDX-License-Identifier: AGPL-3.0-only
//! **Ctrl-C during `cerulion viz`'s discovery wait, over the REAL
//! binary** — exit 0, an interruption line, and NO absence claim.
//!
//! # Why this exists (a disclosure that was false)
//!
//! An earlier change taught the client-side wait to carry `WaitOutcome::Cancelled` out of the loop
//! and the verb to honour it, but the only guard on the VERB half was a source walk over
//! `main.rs`, disclosed as: *"no in-repo test can reach that verb's loop (it needs a real
//! vizd, a real netd and a real robot)"*. That was wrong — every
//! ingredient is already here:
//!
//! * `CERULION_VIZD_SOCK` redirects the control socket (`viz_client`'s own unit tests
//!   drive it), so a `UnixListener` in this file IS the daemon;
//! * `ensure_daemon` connects to an existing socket and never spawns;
//! * the verb starts no viewer at all (Cerulion Studio is the viewer, and it connects
//!   to the daemon itself), so no viewer is involved;
//! * a reply carrying a `retry` hint holds the verb inside the wait, on demand;
//! * `signal_matrix_e2e_test.rs` is the established spawn-binary → signal →
//!   assert-exit-0 harness.
//!
//! No netd. No robot. No viewer. So the arm ships, and it pins the whole composed
//! surface the two source walks could only approximate — including the claim in
//! `main.rs` that the interruption message is *"pinned NEGATIVELY against the
//! absence/UNKNOWN vocabulary"*, which nothing pinned until now.
//!
//! It also catches a variant the source walk cannot: an `if false && …` around the
//! verb's `Cancelled` branch leaves the string in place, so the source walk passes — but
//! the verb then renders the reply, and this test sees the absence paragraph.
#![cfg(unix)]

use std::io::{BufRead, BufReader, Read, Write};
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::PathBuf;
use std::process::{Child, Command, ExitStatus, Stdio};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

/// The absence vocabulary the real daemon uses, served verbatim by the fake so the
/// NEGATIVE assertions below are about the text a user would actually have seen.
const ABSENCE_PROSE: &str = "'/interrupt/topic' is not a local topic, and nothing on the \
     network answered cerulion-netd — so whether it exists is UNKNOWN (this is not a claim that \
     it is missing).";

/// A scripted `cerulion-vizd`: Hello banner with `rerun_url: null` (viz disabled on the
/// daemon), then every `attach` answered `ok:false` + a `retry` hint. Counts attaches so
/// the test can wait until the verb is demonstrably INSIDE its wait loop.
struct FakeVizd {
    socket: PathBuf,
    dir: PathBuf,
    attaches: Arc<AtomicU32>,
    stop: Arc<AtomicBool>,
    handle: Option<std::thread::JoinHandle<()>>,
}

impl FakeVizd {
    /// `plane_unsettled_ms` is the ONE knob that separates the two arms: `Some(0)` is a
    /// young plane (the verb keeps waiting), a value above the shipped ceiling makes
    /// `ConvergenceWait::decide` give up on the FIRST round trip (the fast control).
    fn start(tag: &str, plane_unsettled_ms: u64) -> Self {
        static N: AtomicU32 = AtomicU32::new(0);
        let n = N.fetch_add(1, Ordering::Relaxed);
        let dir = std::env::temp_dir().join(format!("viz_{tag}_{}_{n}", std::process::id()));
        std::fs::create_dir_all(&dir).expect("mk tempdir");
        let socket = dir.join("vizd.sock");
        let listener = UnixListener::bind(&socket).expect("bind fake vizd");
        listener.set_nonblocking(true).expect("nonblocking");

        let attaches = Arc::new(AtomicU32::new(0));
        let stop = Arc::new(AtomicBool::new(false));
        let handle = {
            let attaches = Arc::clone(&attaches);
            let stop = Arc::clone(&stop);
            std::thread::spawn(move || {
                while !stop.load(Ordering::Relaxed) {
                    match listener.accept() {
                        Ok((s, _)) => serve(s, &attaches, &stop, plane_unsettled_ms),
                        Err(_) => std::thread::sleep(Duration::from_millis(5)),
                    }
                }
            })
        };
        Self {
            socket,
            dir,
            attaches,
            stop,
            handle: Some(handle),
        }
    }

    fn attaches(&self) -> u32 {
        self.attaches.load(Ordering::Relaxed)
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

fn serve(stream: UnixStream, attaches: &AtomicU32, stop: &AtomicBool, plane_unsettled_ms: u64) {
    let Ok(mut writer) = stream.try_clone() else {
        return;
    };
    stream
        .set_read_timeout(Some(Duration::from_millis(100)))
        .ok();
    let mut reader = BufReader::new(stream);
    if writeln!(writer, r#"{{"vizd":"fake","protocol":1,"rerun_url":null}}"#).is_err() {
        return;
    }
    // The accumulator lives OUTSIDE the loop: a timed-out `read_line` can already have
    // appended a partial line, and reading into a fresh String would drop it.
    let mut line = String::new();
    loop {
        if stop.load(Ordering::Relaxed) {
            return;
        }
        match reader.read_line(&mut line) {
            Ok(0) => return,
            Ok(_) => {}
            Err(ref e)
                if matches!(
                    e.kind(),
                    std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                ) =>
            {
                continue
            }
            Err(_) => return,
        }
        if !line.ends_with('\n') {
            continue;
        }
        let req = std::mem::take(&mut line);
        if req.trim().is_empty() {
            continue;
        }
        if !req.contains(r#""method":"attach""#) {
            // Any other verb (a `detach` on teardown) is acknowledged blandly.
            if writeln!(writer, r#"{{"id":0,"ok":true}}"#).is_err() {
                return;
            }
            continue;
        }
        attaches.fetch_add(1, Ordering::Relaxed);
        let body = format!(
            r#"{{"id":100,"ok":false,"topic":"/interrupt/topic","error":"{ABSENCE_PROSE}",
                 "retry":{{"discovery":"discovering","plane_unsettled_ms":{plane_unsettled_ms}}}}}"#
        )
        .replace('\n', "")
        .replace("                 ", "");
        if writeln!(writer, "{body}").is_err() {
            return;
        }
    }
}

/// Spawn `cerulion viz <topic>` against the fake daemon, capturing both streams.
fn spawn_viz(sock: &PathBuf) -> Child {
    Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["viz", "/interrupt/topic"])
        .env("CERULION_VIZD_SOCK", sock)
        // The login gate is on by default; this arm is not about the gate, and a
        // device-code prompt would hang the child.
        .env("CERULION_LOGIN_GATE", "off")
        .env("CERULION_NETWORK", "off")
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn cerulion viz")
}

fn wait_bounded(child: &mut Child, bound: Duration) -> Option<ExitStatus> {
    let deadline = Instant::now() + bound;
    loop {
        match child.try_wait() {
            Ok(Some(s)) => return Some(s),
            Ok(None) if Instant::now() < deadline => std::thread::sleep(Duration::from_millis(20)),
            Ok(None) => return None,
            Err(_) => return None,
        }
    }
}

/// stdout + stderr of a finished child, concatenated (the verb writes its placement
/// lines to stdout and its diagnostics to stderr; the assertions care about both).
fn drain(child: &mut Child) -> String {
    let mut out = String::new();
    if let Some(mut s) = child.stdout.take() {
        let _ = s.read_to_string(&mut out);
    }
    let mut err = String::new();
    if let Some(mut s) = child.stderr.take() {
        let _ = s.read_to_string(&mut err);
    }
    format!("{out}{err}")
}

/// THE composed pin: **Ctrl-C during the discovery wait exits 0 and
/// makes NO claim about the topic.**
///
/// The fake daemon reports a YOUNG plane, so the verb sits in its wait loop re-asking.
/// Once it has demonstrably asked more than once, SIGINT it. Three oracles:
///
/// * exit code EXACTLY 0 — a signal-terminated process has `code() == None`, so this is
///   the discriminator proving the handler caught the signal and the verb chose to exit
///   cleanly (the topic verbs' precedent + the repo's `signal_matrix_e2e_test` rule);
/// * the interruption line is present, and it CONCLUDES NOTHING;
/// * the absence vocabulary is ABSENT — no `could not attach`, no `is UNKNOWN`, no
///   `attached 0 of`. That is the negative pin `main.rs` claimed and nothing had.
#[test]
fn a_sigint_during_the_discovery_wait_exits_zero_and_claims_nothing() {
    let daemon = FakeVizd::start("interrupt", 0);
    let mut child = spawn_viz(&daemon.socket);

    // Wait until the verb is demonstrably INSIDE the wait loop (more than one attach),
    // so the signal cannot land before the wait began.
    let deadline = Instant::now() + Duration::from_secs(20);
    while daemon.attaches() < 2 && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(20));
    }
    assert!(
        daemon.attaches() >= 2,
        "the verb never entered the wait loop ({} attaches) — the arm would be vacuous",
        daemon.attaches()
    );

    unsafe { libc::kill(child.id() as libc::pid_t, libc::SIGINT) };

    let status = wait_bounded(&mut child, Duration::from_secs(30)).unwrap_or_else(|| {
        let _ = child.kill();
        panic!("cerulion viz did not exit after SIGINT");
    });
    let output = drain(&mut child);

    assert_eq!(
        status.code(),
        Some(0),
        "a run the USER stopped must exit CLEANLY (code 0, not signal-terminated): \
         {status:?}\n{output}"
    );
    assert!(
        output.contains("interrupted while discovering"),
        "the interruption must be REPORTED:\n{output}"
    );
    // The negative half — the whole point of carrying `Cancelled` out of the loop.
    for forbidden in [
        "could not attach",
        "is UNKNOWN",
        "not a local topic",
        "attached 0 of",
    ] {
        assert!(
            !output.contains(forbidden),
            "a cancelled wait must make NO claim about the topic, found {forbidden:?}:\n{output}"
        );
    }
}

/// The ANTI-TAUTOLOGY control: without the interruption, the SAME fake daemon and the
/// SAME prose DO produce the absence paragraph and a NONZERO exit.
///
/// Without this, every negative assertion above would pass a verb that printed nothing
/// at all. One field differs — the plane age is reported ABOVE the shipped ceiling, so
/// `ConvergenceWait::decide`'s first-contact cap gives up on the FIRST round trip and the
/// control costs milliseconds instead of the ceiling.
#[test]
fn without_an_interruption_the_same_daemon_yields_the_absence_paragraph_and_nonzero() {
    // Comfortably past `FIRST_CONTACT_CONVERGENCE_CEILING` (10 s).
    let daemon = FakeVizd::start("control", 3_600_000);
    let mut child = spawn_viz(&daemon.socket);

    let status = wait_bounded(&mut child, Duration::from_secs(60)).unwrap_or_else(|| {
        let _ = child.kill();
        panic!("cerulion viz did not exit on its own");
    });
    let output = drain(&mut child);

    assert_eq!(
        daemon.attaches(),
        1,
        "an out-waited plane buys no wait — exactly one round trip:\n{output}"
    );
    assert_ne!(
        status.code(),
        Some(0),
        "attaching nothing is a FAILED run: {status:?}\n{output}"
    );
    assert!(
        output.contains("could not attach") && output.contains("is UNKNOWN"),
        "the daemon's absence prose IS rendered when nobody interrupted us:\n{output}"
    );
    assert!(
        !output.contains("interrupted while discovering"),
        "and nothing was interrupted:\n{output}"
    );
}

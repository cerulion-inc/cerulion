// SPDX-License-Identifier: AGPL-3.0-only
//! Whole runs of `cerulion pair --start` against a temporary robot state root:
//! a scripted serving host, and a scripted environment whose every sleep lets
//! a scripted daemon act on the handoff files. No process is spawned; every
//! line the daemon writes and every line the run prints is hand-written.

use std::cell::RefCell;
use std::collections::VecDeque;
use std::io::Write as _;
use std::os::unix::fs::{DirBuilderExt as _, MetadataExt as _, OpenOptionsExt as _};
use std::path::{Path, PathBuf};
use std::rc::Rc;
use std::time::{Duration, Instant};

use cerulion_pairing::pair_code::{
    CLAIMED_FILE, REQUEST_FILE, REQUEST_STAGING_PREFIX, RESULT_FILE, WRITER_LOCK_FILE,
};

use super::super::run::{run, HeldServing, ServingHost, StartEnv};
use super::super::{HoldRefusal, NetdProbe};

const NONCE_HEX: &str = "07070707070707070707070707070707";
const OTHER_HEX: &str = "08080808080808080808080808080808";
const REQUEST: &str = "cerulion-pair-code-request/1 07070707070707070707070707070707 48213907\n";
/// 22:13:20 UTC.
const UNIX: u64 = 1_700_000_000;

fn eid() -> String {
    "e1".repeat(32)
}

fn desk() -> String {
    format!("{} lab-desk", "04".repeat(32))
}

fn line(nonce: &str, status: &str, reason: &str, failed: u8, desk: &str) -> String {
    format!(
        "cerulion-pair-code-result/1 {nonce} {status} {reason} {} 600 3 {failed} {desk}\n",
        eid()
    )
}

fn armed_line(failed: u8) -> String {
    line(NONCE_HEX, "armed", "-", failed, "- -")
}

/// The handoff as the daemon sees it.
struct Daemon {
    dir: PathBuf,
    claims: Vec<Vec<u8>>,
}

impl Daemon {
    fn request_present(&self) -> bool {
        self.dir.join(REQUEST_FILE).exists()
    }

    /// Claim the request as the daemon does: rename, read, remove.
    fn claim(&mut self) -> Option<Vec<u8>> {
        let claimed = self.dir.join(CLAIMED_FILE);
        std::fs::rename(self.dir.join(REQUEST_FILE), &claimed).ok()?;
        let bytes = std::fs::read(&claimed).unwrap();
        std::fs::remove_file(&claimed).unwrap();
        self.claims.push(bytes.clone());
        Some(bytes)
    }

    /// Write the result as the daemon does: a 0600 staging file renamed over.
    fn answer(&self, bytes: &str) {
        self.answer_mode(bytes, 0o600);
    }

    fn answer_mode(&self, bytes: &str, mode: u32) {
        let staging = self.dir.join(".pair-code.result.tmp");
        let mut file = std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .mode(mode)
            .open(&staging)
            .unwrap();
        file.write_all(bytes.as_bytes()).unwrap();
        drop(file);
        std::fs::set_permissions(&staging, std::os::unix::fs::PermissionsExt::from_mode(mode))
            .unwrap();
        std::fs::rename(&staging, self.dir.join(RESULT_FILE)).unwrap();
    }
}

type Script = Box<dyn FnMut(Duration, &mut Daemon)>;
/// One way a daemon answers.
type Answer = fn(&Daemon);

/// The environment: a clock that moves only when the run sleeps, and a
/// daemon that acts after each sleep.
struct Env {
    base: Instant,
    t: Duration,
    /// The wall-clock second at `t` zero.
    unix: u64,
    interrupt_at: Option<Duration>,
    draws: VecDeque<u32>,
    daemon: Daemon,
    script: Script,
}

impl StartEnv for Env {
    fn now(&mut self) -> Instant {
        self.base + self.t
    }

    fn unix_now_s(&mut self) -> u64 {
        self.unix + self.t.as_secs()
    }

    fn sleep(&mut self, span: Duration) {
        self.t += span;
        (self.script)(self.t, &mut self.daemon);
    }

    fn interrupted(&mut self) -> bool {
        self.interrupt_at.is_some_and(|at| self.t >= at)
    }

    fn draw_u32(&mut self) -> Result<u32, String> {
        self.draws
            .pop_front()
            .ok_or_else(|| "no entropy".to_string())
    }

    fn nonce(&mut self) -> Result<[u8; 16], String> {
        Ok([7; 16])
    }
}

/// The serving host: its authority, its hold, and its probe answers (a clear
/// status once they run out).
struct Host {
    authority: Result<(), &'static str>,
    hold: Option<Result<HeldServing, HoldRefusal>>,
    probes: VecDeque<NetdProbe>,
    probed: usize,
}

impl ServingHost for Host {
    fn authority(&mut self) -> Result<(), &'static str> {
        self.authority
    }

    fn hold(&mut self, ttl_s: u64) -> Result<HeldServing, HoldRefusal> {
        assert_eq!(ttl_s, 790);
        self.hold.take().expect("one hold per run")
    }

    fn probe(&mut self) -> NetdProbe {
        self.probed += 1;
        self.probes.pop_front().unwrap_or(NetdProbe::Status {
            startup_error: false,
        })
    }
}

struct Robot {
    _tmp: tempfile::TempDir,
    root: PathBuf,
    dir: PathBuf,
    base: Instant,
}

/// A robot state root whose remoted directory exists, mode `mode`, when
/// `mode` is given.
fn robot_with(mode: Option<u32>) -> Robot {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path().join("state");
    std::fs::create_dir(&root).unwrap();
    let dir = root.join("remoted");
    if let Some(mode) = mode {
        std::fs::DirBuilder::new().mode(0o700).create(&dir).unwrap();
        std::fs::set_permissions(&dir, std::os::unix::fs::PermissionsExt::from_mode(mode)).unwrap();
    }
    Robot {
        _tmp: tmp,
        root,
        dir,
        base: Instant::now(),
    }
}

impl Robot {
    fn host(&self) -> Host {
        self.host_until(Duration::from_secs(790))
    }

    fn host_until(&self, granted: Duration) -> Host {
        Host {
            authority: Ok(()),
            hold: Some(Ok(HeldServing {
                root: self.root.clone(),
                expires: self.base + granted,
            })),
            probes: VecDeque::new(),
            probed: 0,
        }
    }

    fn env(&self, script: Script) -> Env {
        Env {
            base: self.base,
            t: Duration::ZERO,
            unix: UNIX,
            interrupt_at: None,
            draws: VecDeque::from([48_213_907]),
            daemon: Daemon {
                dir: self.dir.clone(),
                claims: Vec::new(),
            },
            script,
        }
    }
}

struct Ran {
    exit: u8,
    stdout: String,
    stderr: String,
    claims: Vec<Vec<u8>>,
}

fn run_on(robot: &Robot, host: &mut Host, env: &mut Env) -> Ran {
    let (mut out, mut err) = (Vec::new(), Vec::new());
    let exit = run(Some(&robot.root), host, env, &mut out, &mut err);
    Ran {
        exit,
        stdout: String::from_utf8(out).unwrap(),
        stderr: String::from_utf8(err).unwrap(),
        claims: std::mem::take(&mut env.daemon.claims),
    }
}

const ASKING: &str = "Asking cerulion-netd to serve this robot for code pairing.\n";

fn armed_out(port: &str) -> String {
    format!(
        "pair-code: code=48213907 eid={} port={port} expires_in_s=600 tries=3\n",
        eid()
    )
}

/// The armed lines on stderr for a code armed at `hh_mm` plus ten minutes.
fn armed_err(expires: &str) -> String {
    format!(
        "Pairing code: 48213907\n\
         It pairs one desk and expires at {expires} UTC (in 10 minutes). All desks share 3 \
         tries.\n\
         Waiting for a desk. Ctrl-C stops waiting; the code keeps working until it is used or \
         expires.\n"
    )
}

const AGAIN: &str = "Run `cerulion pair --start` again for a new code.";

/// A daemon that claims the first request it sees, answers it armed, has a
/// try fail a second later and a desk pair a second after that.
fn pairing_daemon(checks: Rc<RefCell<Vec<String>>>) -> Script {
    let mut armed_at = None;
    Box::new(move |t, d| match armed_at {
        None if d.request_present() => {
            let request = d.dir.join(REQUEST_FILE);
            let meta = std::fs::symlink_metadata(&request).unwrap();
            checks.borrow_mut().push(format!(
                "mode={:o} nlink={} result={}",
                meta.mode() & 0o777,
                meta.nlink(),
                d.dir.join(RESULT_FILE).exists()
            ));
            d.claim().unwrap();
            d.answer(&armed_line(0));
            armed_at = Some(t);
        }
        None => {}
        Some(at) if t == at + Duration::from_secs(1) => d.answer(&armed_line(1)),
        Some(at) if t == at + Duration::from_secs(2) => {
            d.answer(&line(NONCE_HEX, "paired", "-", 1, &desk()))
        }
        Some(_) => {}
    })
}

fn the_happy_run() -> (Ran, Vec<String>, u32) {
    let robot = robot_with(Some(0o700));
    // A result an earlier run left: removed before the request is left.
    std::fs::write(
        robot.dir.join(RESULT_FILE),
        armed_line(0).replace("07", "09"),
    )
    .unwrap();
    let checks = Rc::new(RefCell::new(Vec::new()));
    let mut env = robot.env(pairing_daemon(Rc::clone(&checks)));
    let mut host = robot.host();
    let ran = run_on(&robot, &mut host, &mut env);
    let lock_mode = std::fs::metadata(robot.dir.join(WRITER_LOCK_FILE))
        .unwrap()
        .mode()
        & 0o777;
    let checks = checks.borrow().clone();
    (ran, checks, lock_mode)
}

#[test]
fn a_run_leaves_its_request_shows_the_code_and_follows_it_to_a_pairing() {
    let (ran, checks, lock_mode) = the_happy_run();
    assert_eq!(ran.exit, 0, "{}", ran.stderr);
    assert_eq!(ran.claims, vec![REQUEST.as_bytes().to_vec()]);
    assert_eq!(checks, vec!["mode=600 nlink=1 result=false".to_string()]);
    assert_eq!(lock_mode, 0o600);
    let account = "04".repeat(32);
    assert_eq!(
        ran.stdout,
        format!(
            "{}try-failed: failed=1 tries=3\npaired: desk=lab-desk account={account}\n",
            armed_out("-")
        )
    );
    assert_eq!(
        ran.stderr,
        format!(
            "{ASKING}{}A pairing try failed: 1 of 3 tries failed.\nPaired with desk lab-desk \
             (account {account}). The code is used up.\n",
            armed_err("22:23")
        )
    );
}

#[test]
fn a_run_is_byte_identical_on_two_runs() {
    let (first, checks, _) = the_happy_run();
    let (second, again, _) = the_happy_run();
    assert_eq!(
        (
            first.exit,
            &first.stdout,
            &first.stderr,
            &first.claims,
            checks
        ),
        (
            second.exit,
            &second.stdout,
            &second.stderr,
            &second.claims,
            again
        )
    );
}

#[test]
fn a_missing_directory_is_waited_for_and_never_created() {
    let robot = robot_with(None);
    let checks = Rc::new(RefCell::new(Vec::new()));
    let mut pairing = pairing_daemon(Rc::clone(&checks));
    let seen = Rc::new(RefCell::new(Vec::new()));
    let seen_by_daemon = Rc::clone(&seen);
    let script: Script = Box::new(move |t, d| {
        if t < Duration::from_millis(1000) {
            seen_by_daemon.borrow_mut().push(d.dir.exists());
            return;
        }
        if t == Duration::from_millis(1000) {
            std::fs::DirBuilder::new()
                .mode(0o700)
                .create(&d.dir)
                .unwrap();
        }
        pairing(t, d);
    });
    let mut env = robot.env(script);
    let ran = run_on(&robot, &mut robot.host(), &mut env);
    assert_eq!(ran.exit, 0, "{}", ran.stderr);
    assert_eq!(
        *seen.borrow(),
        vec![false, false, false],
        "three ticks, nothing made"
    );
    assert_eq!(ran.claims.len(), 1);
}

#[test]
fn a_request_a_starting_daemon_removed_is_left_again_after_two_polls() {
    let robot = robot_with(Some(0o700));
    let mut swept_at = None;
    let mut pairing = pairing_daemon(Rc::new(RefCell::new(Vec::new())));
    let left_again = Rc::new(RefCell::new(None));
    let noted = Rc::clone(&left_again);
    let script: Script = Box::new(move |t, d| match swept_at {
        None if d.request_present() => {
            // The start sweep: the request is removed, unanswered.
            std::fs::remove_file(d.dir.join(REQUEST_FILE)).unwrap();
            swept_at = Some(t);
        }
        None => {}
        Some(_) => {
            if d.request_present() && noted.borrow().is_none() {
                *noted.borrow_mut() = Some(t);
            }
            pairing(t, d);
        }
    });
    let mut env = robot.env(script);
    let ran = run_on(&robot, &mut robot.host(), &mut env);
    assert_eq!(ran.exit, 0, "{}", ran.stderr);
    assert_eq!(ran.claims, vec![REQUEST.as_bytes().to_vec()]);
    // Swept at 500 ms and seen gone at 500 ms: left again 2 s later, seen
    // by the daemon on its next look.
    assert_eq!(*left_again.borrow(), Some(Duration::from_millis(2750)));
}

#[test]
fn the_port_is_shown_only_from_facts_naming_this_endpoint() {
    for (facts_eid, iroh_port, port) in [
        ("ab".repeat(32), 41234, "-"),
        (eid(), 41234, "41234"),
        (eid(), 0, "-"),
    ] {
        let robot = robot_with(Some(0o700));
        std::fs::write(
            robot.dir.join("beacon_facts.json"),
            format!(
                r#"{{"version":1,"eid":"{facts_eid}","iroh_port":{iroh_port},"claimable":"0"}}"#
            ),
        )
        .unwrap();
        let mut env = robot.env(pairing_daemon(Rc::new(RefCell::new(Vec::new()))));
        let ran = run_on(&robot, &mut robot.host(), &mut env);
        assert_eq!(ran.exit, 0, "{}", ran.stderr);
        assert!(ran.stdout.starts_with(&armed_out(port)), "{}", ran.stdout);
    }
    // Facts that cannot be read: a directory at their name.
    let robot = robot_with(Some(0o700));
    std::fs::DirBuilder::new()
        .mode(0o700)
        .create(robot.dir.join("beacon_facts.json"))
        .unwrap();
    let mut env = robot.env(pairing_daemon(Rc::new(RefCell::new(Vec::new()))));
    let ran = run_on(&robot, &mut robot.host(), &mut env);
    assert_eq!(ran.exit, 0, "{}", ran.stderr);
    assert!(ran.stdout.starts_with(&armed_out("-")), "{}", ran.stdout);
}

#[test]
fn a_shorter_hold_shortens_the_wait_for_the_daemon() {
    let robot = robot_with(Some(0o700));
    let mut env = robot.env(Box::new(|_, _| {}));
    let ran = run_on(
        &robot,
        &mut robot.host_until(Duration::from_secs(60)),
        &mut env,
    );
    assert_eq!(ran.exit, 3);
    assert_eq!(
        ran.stderr,
        format!(
            "{ASKING}Waiting for cerulion-remoted to start serving. A first start on this robot \
             registers it with your account and can take up to 2 minutes.\nError: \
             cerulion-remoted did not start serving within 60 s, so no code was made. The \
             cerulion-netd log says why it could not start cerulion-remoted; run `cerulion pair \
             --start` again once it can.\n"
        )
    );
    assert!(
        !robot.dir.join(REQUEST_FILE).exists(),
        "the unclaimed request is taken back"
    );
    assert!(ran.stdout.is_empty());
}

#[test]
fn a_slow_claim_has_its_copy_left_again_taken_back_before_the_code_is_shown() {
    let robot = robot_with(Some(0o700));
    let mut claimed_at = None;
    let mut answered = false;
    let script: Script = Box::new(move |t, d| match claimed_at {
        None if d.request_present() => {
            d.claim().unwrap();
            claimed_at = Some(t);
        }
        None => {}
        // The answer comes three seconds after the claim.
        Some(at) if !answered && t >= at + Duration::from_secs(3) => {
            d.answer(&armed_line(0));
            answered = true;
        }
        Some(at) if t >= at + Duration::from_secs(5) => {
            d.answer(&line(NONCE_HEX, "expired", "-", 0, "- -"))
        }
        Some(_) => {}
    });
    let mut env = robot.env(script);
    let ran = run_on(&robot, &mut robot.host(), &mut env);
    assert_eq!(ran.exit, 2, "{}", ran.stderr);
    assert_eq!(ran.claims.len(), 1, "the daemon claimed one request");
    assert!(
        !robot.dir.join(REQUEST_FILE).exists(),
        "the copy is taken back"
    );
    assert_eq!(ran.stdout.matches("pair-code:").count(), 1);
    assert_eq!(ran.stderr.matches("Pairing code:").count(), 1);
    assert!(
        ran.stdout.ends_with("expired: failed=0 tries=3\n"),
        "{}",
        ran.stdout
    );
}

#[test]
fn a_slow_claim_whose_copy_left_again_is_answered_with_the_same_code_shows_it_once() {
    let robot = robot_with(Some(0o700));
    let mut claimed = 0;
    let mut answered_at = None;
    let script: Script = Box::new(move |t, d| {
        if claimed < 2 && d.request_present() {
            d.claim().unwrap();
            claimed += 1;
            // The first copy arms the code with no answer yet; the copy left
            // again names that code, so its state is written again.
            if claimed == 2 {
                d.answer(&armed_line(0));
                answered_at = Some(t);
            }
            return;
        }
        if answered_at.is_some_and(|at| t >= at + Duration::from_secs(1)) {
            d.answer(&line(NONCE_HEX, "expired", "-", 0, "- -"));
        }
    });
    let mut env = robot.env(script);
    // The run starts at 22:13:58 UTC; the first copy, left then, is claimed
    // at 250 ms and the answer is first read at 2.5 s, 22:14:00.
    env.unix = UNIX + 38;
    let ran = run_on(&robot, &mut robot.host(), &mut env);
    assert_eq!(ran.exit, 2, "{}", ran.stderr);
    assert_eq!(ran.claims, vec![REQUEST.as_bytes().to_vec(); 2]);
    assert!(!robot.dir.join(REQUEST_FILE).exists());
    assert_eq!(
        ran.stdout,
        format!("{}expired: failed=0 tries=3\n", armed_out("-"))
    );
    // Shown once, its window counted from the leave at 22:13:58.
    assert_eq!(
        ran.stderr.matches(&armed_err("22:23")).count(),
        1,
        "{}",
        ran.stderr
    );
    assert_eq!(ran.stderr.matches("Pairing code:").count(), 1);
}

#[test]
fn a_held_lock_keeps_the_handoff_untouched_until_it_is_released() {
    let robot = robot_with(Some(0o700));
    let stale = armed_line(0).replace("07", "09");
    std::fs::write(robot.dir.join(RESULT_FILE), &stale).unwrap();
    let other = std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)
        .truncate(false)
        .mode(0o600)
        .open(robot.dir.join(WRITER_LOCK_FILE))
        .unwrap();
    other.lock().unwrap();
    let mut other = Some(other);
    let mut pairing = pairing_daemon(Rc::new(RefCell::new(Vec::new())));
    let untouched = Rc::new(RefCell::new(0));
    let counted = Rc::clone(&untouched);
    let script: Script = Box::new(move |t, d| {
        if t < Duration::from_secs(3) {
            assert!(!d.request_present(), "no request while the lock is held");
            assert!(
                d.dir.join(RESULT_FILE).exists(),
                "the result is not removed"
            );
            *counted.borrow_mut() += 1;
            return;
        }
        // The other writer releases its lock.
        other.take();
        pairing(t, d);
    });
    let mut env = robot.env(script);
    let ran = run_on(&robot, &mut robot.host(), &mut env);
    assert_eq!(ran.exit, 0, "{}", ran.stderr);
    assert!(*untouched.borrow() >= 10);
    assert!(
        ran.stderr.starts_with(&format!(
            "{ASKING}Waiting for cerulion-remoted to start serving."
        )),
        "the note shows while the lock is held: {}",
        ran.stderr
    );
}

#[test]
fn a_result_that_is_not_the_daemons_kind_is_refused() {
    let cases: [(&str, Answer); 4] = [
        ("it is not owner-only", |d| {
            d.answer_mode(&armed_line(0), 0o644)
        }),
        (
            "pair-code file: the line has the wrong number of fields",
            |d| d.answer("garbage\n"),
        ),
        ("it is a symbolic link", |d| {
            let elsewhere = d.dir.parent().unwrap().join("elsewhere");
            std::fs::write(&elsewhere, armed_line(0)).unwrap();
            std::os::unix::fs::symlink(&elsewhere, d.dir.join(RESULT_FILE)).unwrap();
        }),
        ("it is longer than 512 bytes", |d| {
            d.answer(&"x".repeat(513))
        }),
    ];
    for (why, answer) in cases {
        let robot = robot_with(Some(0o700));
        let mut done = false;
        let script: Script = Box::new(move |_, d| {
            if !done && d.request_present() {
                d.claim().unwrap();
                answer(d);
                done = true;
            }
        });
        let mut env = robot.env(script);
        let ran = run_on(&robot, &mut robot.host(), &mut env);
        assert_eq!(ran.exit, 1, "{why}");
        assert_eq!(
            ran.stderr,
            format!(
                "{ASKING}Error: could not read the answer cerulion-remoted left in {}: {why}. Use \
                 cerulion and cerulion-remoted from the same release.\n",
                robot.dir.join(RESULT_FILE).display()
            )
        );
        assert!(ran.stdout.is_empty());
    }
}

#[test]
fn a_directory_that_is_not_private_is_refused_and_nothing_is_written() {
    let robot = robot_with(Some(0o755));
    let mut env = robot.env(Box::new(|_, _| {}));
    let ran = run_on(&robot, &mut robot.host(), &mut env);
    assert_eq!(ran.exit, 1);
    assert_eq!(
        ran.stderr,
        format!(
            "{ASKING}Error: the robot state directory {} is not a private directory of this \
             user, so no code was made: run `cerulion pair --start` as the user cerulion-netd \
             runs as, and keep the directory owner-only.\n",
            robot.dir.display()
        )
    );
    assert_eq!(std::fs::read_dir(&robot.dir).unwrap().count(), 0);
}

#[test]
fn another_nonces_result_before_the_own_answer_is_passed_over() {
    let robot = robot_with(Some(0o700));
    let mut pairing = pairing_daemon(Rc::new(RefCell::new(Vec::new())));
    let mut replaced_written = false;
    let script: Script = Box::new(move |t, d| {
        if !replaced_written && d.request_present() {
            // The replaced code's own last result, written before the arm.
            d.answer(&line(OTHER_HEX, "paired", "-", 0, &desk()));
            replaced_written = true;
            return;
        }
        pairing(t, d);
    });
    let mut env = robot.env(script);
    let ran = run_on(&robot, &mut robot.host(), &mut env);
    assert_eq!(ran.exit, 0, "{}", ran.stderr);
    assert!(ran.stdout.starts_with(&armed_out("-")));
}

#[test]
fn each_refusal_before_the_handoff_leaves_nothing_under_the_root() {
    use std::path::PathBuf as P;
    let refusals = [
        (
            HoldRefusal::WillNotServe {
                netd: cerulion_netd::egress::REMOTE_SERVING_UNAVAILABLE.into(),
            },
            1,
        ),
        (HoldRefusal::TooOld { daemon: 11 }, 1),
        (HoldRefusal::NetdProtocol, 1),
        (HoldRefusal::NotFound { exe_dir: None }, 3),
        (
            HoldRefusal::Unreachable {
                socket: P::from("/run/n.sock"),
                os: "Connection refused (os error 61)".into(),
            },
            3,
        ),
    ];
    for (refusal, exit) in refusals {
        let robot = robot_with(None);
        let mut host = robot.host();
        host.hold = Some(Err(refusal.clone()));
        let mut env = robot.env(Box::new(|_, _| {}));
        let ran = run_on(&robot, &mut host, &mut env);
        assert_eq!(ran.exit, exit, "{refusal:?}");
        assert_eq!(ran.stderr, format!("{ASKING}{}\n", refusal.text()));
        assert!(ran.stdout.is_empty());
        assert_eq!(std::fs::read_dir(&robot.root).unwrap().count(), 0);
    }

    // No serving authority: refused before netd is asked.
    let robot = robot_with(None);
    let mut host = robot.host();
    host.authority = Err(cerulion_netd::serving_login::REFUSAL);
    let ran = run_on(&robot, &mut host, &mut robot.env(Box::new(|_, _| {})));
    assert_eq!(ran.exit, 1);
    assert_eq!(
        ran.stderr,
        "Error: serving the network requires a prior login; run `cerulion login`. No code was \
         made.\n"
    );
    assert!(host.hold.is_some(), "netd is not asked");

    // netd serves another root.
    let robot = robot_with(None);
    let mut host = robot.host();
    host.hold = Some(Ok(HeldServing {
        root: P::from("/srv/robot"),
        expires: robot.base + Duration::from_secs(790),
    }));
    let ran = run_on(&robot, &mut host, &mut robot.env(Box::new(|_, _| {})));
    assert_eq!(ran.exit, 1);
    assert_eq!(
        ran.stderr,
        format!(
            "{ASKING}Error: cerulion-netd serves the robot state at /srv/robot but this command \
             uses {}; run both with the same CERULION_STATE_ROOT or CERULION_HOME. No code was \
             made.\n",
            robot.root.display()
        )
    );

    // No random source.
    let robot = robot_with(None);
    let mut env = robot.env(Box::new(|_, _| {}));
    env.draws.clear();
    let ran = run_on(&robot, &mut robot.host(), &mut env);
    assert_eq!(ran.exit, 1);
    assert_eq!(
        ran.stderr,
        "Error: could not draw a random pairing code: no entropy. No code was made.\n"
    );

    // No state root.
    let (mut out, mut err) = (Vec::new(), Vec::new());
    let robot = robot_with(None);
    let exit = run(
        None,
        &mut robot.host(),
        &mut robot.env(Box::new(|_, _| {})),
        &mut out,
        &mut err,
    );
    assert_eq!(exit, 1);
    assert_eq!(
        String::from_utf8(err).unwrap(),
        "Error: no robot state root is available for this command, so no code was made; set \
         CERULION_STATE_ROOT, CERULION_HOME or HOME.\n"
    );
}

/// A daemon that claims the first request and answers it with `first`, then
/// writes each of `later` on the following looks.
fn answering(first: String, later: Vec<String>) -> Script {
    let mut later = VecDeque::from(later);
    let mut claimed = false;
    Box::new(move |_, d| {
        if !claimed {
            if d.request_present() {
                d.claim().unwrap();
                d.answer(&first);
                claimed = true;
            }
            return;
        }
        if let Some(next) = later.pop_front() {
            d.answer(&next);
        }
    })
}

#[test]
fn the_daemons_answers_end_the_run_as_their_texts_say() {
    let stopping = line(NONCE_HEX, "refused", "stopping", 0, "- -");
    let cases: Vec<(Vec<String>, Vec<NetdProbe>, u8, String)> = vec![
        (
            vec![line("-", "refused", "malformed", 0, "- -")],
            vec![],
            1,
            "Error: cerulion-remoted refused the pairing code request: it is not a well-formed \
             request of the version this daemon reads; use cerulion and cerulion-remoted from \
             the same release. No code was made.\n"
                .into(),
        ),
        (
            vec![armed_line(0), stopping.clone()],
            vec![
                NetdProbe::Status {
                    startup_error: false,
                },
                NetdProbe::Unreachable,
            ],
            3,
            format!(
                "Error: cerulion-netd stopped while the code was armed, so the code no longer \
                 works. {AGAIN}\n"
            ),
        ),
        (
            vec![armed_line(0), stopping],
            vec![
                NetdProbe::Status {
                    startup_error: false,
                },
                NetdProbe::Status {
                    startup_error: true,
                },
            ],
            3,
            format!(
                "Error: cerulion-remoted stopped while the code was armed, so the code no longer \
                 works. {AGAIN}\n"
            ),
        ),
        (
            vec![
                armed_line(0),
                line(NONCE_HEX, "refused", "unconfirmed", 0, "- -"),
            ],
            vec![],
            2,
            format!(
                "Error: cerulion-remoted withdrew this code after it was armed, so it no longer \
                 works. {AGAIN}\n"
            ),
        ),
        (
            vec![armed_line(0), line(OTHER_HEX, "armed", "-", 0, "- -")],
            vec![],
            2,
            "Error: a newer `cerulion pair --start` replaced this code, so it no longer works.\n"
                .into(),
        ),
    ];
    for (answers, probes, exit, last) in cases {
        let robot = robot_with(Some(0o700));
        let mut answers = answers.into_iter();
        let first = answers.next().unwrap();
        let mut env = robot.env(answering(first, answers.collect()));
        let mut host = robot.host();
        host.probes = probes.into();
        let ran = run_on(&robot, &mut host, &mut env);
        assert_eq!(ran.exit, exit, "{}", ran.stderr);
        assert!(ran.stderr.ends_with(&last), "{}", ran.stderr);
    }
}

#[test]
fn a_code_with_no_end_by_its_window_and_margin_ends_the_run() {
    let robot = robot_with(Some(0o700));
    let mut env = robot.env(answering(armed_line(0), vec![]));
    let mut host = robot.host();
    let ran = run_on(&robot, &mut host, &mut env);
    assert_eq!(ran.exit, 3);
    assert!(
        ran.stderr.ends_with(&format!(
            "Error: cerulion-remoted did not say how the code ended by the end of its window, so \
             the code no longer works. {AGAIN}\n"
        )),
        "{}",
        ran.stderr
    );
    // Armed at 500 ms: ended at 630.5 s, with a probe every 2 s.
    assert_eq!(env.t, Duration::from_millis(630_500));
    assert_eq!(host.probed, 316);
}

#[test]
fn an_interrupt_before_the_claim_takes_the_request_back() {
    let robot = robot_with(Some(0o700));
    let mut env = robot.env(Box::new(|_, _| {}));
    env.interrupt_at = Some(Duration::from_secs(1));
    let ran = run_on(&robot, &mut robot.host(), &mut env);
    assert_eq!(ran.exit, 130);
    assert_eq!(
        ran.stderr,
        format!("{ASKING}Stopped waiting. No code was made.\n")
    );
    assert!(!robot.dir.join(REQUEST_FILE).exists());
    assert!(ran.stdout.is_empty());
}

#[test]
fn an_interrupt_after_the_claim_shows_the_code_that_was_armed() {
    let robot = robot_with(Some(0o700));
    let mut claimed_at = None;
    let script: Script = Box::new(move |t, d| match claimed_at {
        None if d.request_present() => {
            d.claim().unwrap();
            claimed_at = Some(t);
        }
        Some(at) if t == at + Duration::from_millis(750) => d.answer(&armed_line(0)),
        _ => {}
    });
    let mut env = robot.env(script);
    env.interrupt_at = Some(Duration::from_millis(1000));
    let ran = run_on(&robot, &mut robot.host(), &mut env);
    assert_eq!(ran.exit, 130, "{}", ran.stderr);
    assert_eq!(ran.stdout, armed_out("-"));
    assert_eq!(
        ran.stderr,
        format!(
            "{ASKING}{}Stopped waiting. The code keeps working until 22:23 UTC unless a desk \
             uses it first.\n",
            armed_err("22:23")
        )
    );
}

#[test]
fn a_result_unlinked_after_it_was_opened_is_gone_on_its_handle() {
    let tmp = tempfile::tempdir().unwrap();
    let path = tmp.path().join(RESULT_FILE);
    std::fs::write(&path, armed_line(0)).unwrap();
    let file = std::fs::File::open(&path).unwrap();
    std::fs::remove_file(&path).unwrap();
    let meta = file.metadata().unwrap();
    let facts = super::super::FileFacts {
        is_file: meta.is_file(),
        nlink: meta.nlink(),
        uid: meta.uid(),
        mode: meta.mode(),
        len: meta.len(),
    };
    assert_eq!(
        super::super::result_verdict(&facts, meta.uid()),
        super::super::Verdict::Gone
    );
}

#[test]
fn taking_back_a_request_that_is_gone_reports_it_claimed() {
    let tmp = tempfile::tempdir().unwrap();
    assert_eq!(
        super::super::handoff::pull_back(tmp.path()),
        super::super::Pulled::Claimed
    );
    std::fs::write(tmp.path().join(REQUEST_FILE), REQUEST).unwrap();
    assert_eq!(
        super::super::handoff::pull_back(tmp.path()),
        super::super::Pulled::Back
    );
    assert_eq!(std::fs::read_dir(tmp.path()).unwrap().count(), 0);
}

#[test]
fn leaving_a_request_writes_it_owner_only_through_a_staging_name() {
    let tmp = tempfile::tempdir().unwrap();
    std::fs::write(tmp.path().join(RESULT_FILE), "stale").unwrap();
    let request = cerulion_pairing::pair_code::PairCodeRequest::new(
        cerulion_pairing::pair_code::PairCodeNonce::from_bytes([7; 16]),
        "48213907",
    )
    .unwrap();
    super::super::handoff::leave(tmp.path(), &request).unwrap();
    let names: Vec<_> = std::fs::read_dir(tmp.path())
        .unwrap()
        .map(|e| e.unwrap().file_name().into_string().unwrap())
        .collect();
    assert_eq!(names, vec![REQUEST_FILE.to_string()]);
    let path: &Path = &tmp.path().join(REQUEST_FILE);
    assert_eq!(std::fs::read_to_string(path).unwrap(), REQUEST);
    assert_eq!(std::fs::metadata(path).unwrap().mode() & 0o777, 0o600);
}

/// Whether the writers' lock in `dir` is free: tried, and released at once,
/// through a handle of its own.
fn lock_free(dir: &Path) -> bool {
    let file = std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .open(dir.join(WRITER_LOCK_FILE))
        .unwrap();
    match file.try_lock() {
        Ok(()) => true,
        Err(std::fs::TryLockError::WouldBlock) => false,
        Err(std::fs::TryLockError::Error(e)) => panic!("the lock try failed: {e}"),
    }
}

#[test]
fn the_lock_is_released_once_the_code_is_shown_so_a_newer_start_replaces_the_code() {
    const NEWER: &str = "cerulion-pair-code-request/1 08080808080808080808080808080808 50284617\n";
    let robot = robot_with(Some(0o700));
    let seen = Rc::new(RefCell::new(Vec::new()));
    let noted = Rc::clone(&seen);
    let mut step = 0;
    let script: Script = Box::new(move |_, d| match step {
        0 if d.request_present() => {
            noted
                .borrow_mut()
                .push(format!("handshake lock_free={}", lock_free(&d.dir)));
            d.claim().unwrap();
            d.answer(&armed_line(0));
            step = 1;
        }
        1 => {
            // A newer start: it takes the lock, removes the result, leaves
            // its request, and the daemon arms the newer code.
            let newer = std::fs::OpenOptions::new()
                .read(true)
                .write(true)
                .open(d.dir.join(WRITER_LOCK_FILE))
                .unwrap();
            let taken = newer.try_lock().is_ok();
            noted.borrow_mut().push(format!("armed lock_free={taken}"));
            step = 2;
            if !taken {
                return;
            }
            std::fs::remove_file(d.dir.join(RESULT_FILE)).unwrap();
            let mut request = std::fs::OpenOptions::new()
                .write(true)
                .create_new(true)
                .mode(0o600)
                .open(d.dir.join(REQUEST_FILE))
                .unwrap();
            request.write_all(NEWER.as_bytes()).unwrap();
            drop(request);
            d.claim().unwrap();
            d.answer(&line(OTHER_HEX, "armed", "-", 0, "- -"));
        }
        _ => {}
    });
    let mut env = robot.env(script);
    let ran = run_on(&robot, &mut robot.host(), &mut env);
    // The same try finds the lock held through the handshake and free once
    // the code is shown.
    assert_eq!(
        *seen.borrow(),
        vec![
            "handshake lock_free=false".to_string(),
            "armed lock_free=true".to_string()
        ]
    );
    assert_eq!(ran.exit, 2, "{}", ran.stderr);
    assert_eq!(
        ran.claims,
        vec![REQUEST.as_bytes().to_vec(), NEWER.as_bytes().to_vec()]
    );
    assert_eq!(ran.stdout, armed_out("-"));
    assert_eq!(
        ran.stderr,
        format!(
            "{ASKING}{}Error: a newer `cerulion pair --start` replaced this code, so it no \
             longer works.\n",
            armed_err("22:23")
        )
    );
}

/// One way to plant a file at the lock's name in a remoted directory.
type Plant = fn(&Path);

/// A file at `path` with `mode`, set after it is made so no umask strips a
/// bit.
fn plant_file(path: &Path, mode: u32) {
    std::fs::write(path, b"").unwrap();
    std::fs::set_permissions(path, std::os::unix::fs::PermissionsExt::from_mode(mode)).unwrap();
}

#[test]
fn a_lock_file_that_is_not_an_owner_only_file_of_this_user_is_refused_and_nothing_is_left() {
    let not_owner_only = "pair-start.lock is not an owner-only file of this user".to_string();
    let symlink = std::io::Error::from_raw_os_error(libc::ELOOP).to_string();
    let directory = std::io::Error::from_raw_os_error(libc::EISDIR).to_string();
    let cases: [(Plant, String); 4] = [
        (
            |dir| plant_file(&dir.join(WRITER_LOCK_FILE), 0o640),
            not_owner_only.clone(),
        ),
        (
            |dir| plant_file(&dir.join(WRITER_LOCK_FILE), 0o602),
            not_owner_only,
        ),
        (
            |dir| {
                let elsewhere = dir.parent().unwrap().join("elsewhere.lock");
                plant_file(&elsewhere, 0o600);
                std::os::unix::fs::symlink(&elsewhere, dir.join(WRITER_LOCK_FILE)).unwrap();
            },
            symlink,
        ),
        (
            |dir| {
                std::fs::DirBuilder::new()
                    .mode(0o700)
                    .create(dir.join(WRITER_LOCK_FILE))
                    .unwrap();
            },
            directory,
        ),
    ];
    for (plant, why) in cases {
        let robot = robot_with(Some(0o700));
        plant(&robot.dir);
        let mut env = robot.env(pairing_daemon(Rc::new(RefCell::new(Vec::new()))));
        let ran = run_on(&robot, &mut robot.host(), &mut env);
        assert_eq!(ran.exit, 1, "{why}: {}", ran.stderr);
        assert_eq!(
            ran.stderr,
            format!(
                "{ASKING}Error: could not leave the pairing code request in {}: {why}. No code \
                 was made.\n",
                robot.dir.display()
            )
        );
        assert!(ran.stdout.is_empty(), "{why}");
        assert!(ran.claims.is_empty(), "{why}: no request was left");
        assert_eq!(
            std::fs::read_dir(&robot.dir).unwrap().count(),
            1,
            "{why}: only the planted lock is there"
        );
    }
}

#[test]
fn a_request_that_cannot_be_left_ends_the_run_with_the_reason_and_nothing_is_left() {
    let robot = robot_with(Some(0o700));
    // A directory at this writer's staging name refuses its removal.
    let staging = robot.dir.join(format!(
        "{REQUEST_STAGING_PREFIX}{}.tmp",
        std::process::id()
    ));
    std::fs::DirBuilder::new()
        .mode(0o700)
        .create(&staging)
        .unwrap();
    let mut env = robot.env(pairing_daemon(Rc::new(RefCell::new(Vec::new()))));
    let ran = run_on(&robot, &mut robot.host(), &mut env);
    let errno = if cfg!(target_os = "linux") {
        libc::EISDIR
    } else {
        libc::EPERM
    };
    let why = std::io::Error::from_raw_os_error(errno).to_string();
    assert_eq!(ran.exit, 1, "{}", ran.stderr);
    assert_eq!(
        ran.stderr,
        format!(
            "{ASKING}Error: could not leave the pairing code request in {}: {why}. No code was \
             made.\n",
            robot.dir.display()
        )
    );
    assert!(ran.stdout.is_empty());
    assert!(ran.claims.is_empty());
    assert!(!robot.dir.join(REQUEST_FILE).exists());
}

#[test]
fn a_staging_file_removed_before_its_rename_is_written_again_three_times_at_most() {
    let request = cerulion_pairing::pair_code::PairCodeRequest::new(
        cerulion_pairing::pair_code::PairCodeNonce::from_bytes([7; 16]),
        "48213907",
    )
    .unwrap();
    // A starting daemon removes every staging file before its rename.
    let tmp = tempfile::tempdir().unwrap();
    let mut renames = 0;
    let error = super::super::handoff::leave_through(tmp.path(), &request, |from, to| {
        renames += 1;
        assert_eq!(std::fs::read_to_string(from).unwrap(), REQUEST);
        std::fs::remove_file(from).unwrap();
        std::fs::rename(from, to)
    })
    .unwrap_err();
    assert_eq!(renames, 3);
    assert_eq!(error.kind(), std::io::ErrorKind::NotFound);
    assert_eq!(
        error.to_string(),
        "the staging file of the request was removed before it could be renamed into place, \
         three times"
    );
    assert_eq!(std::fs::read_dir(tmp.path()).unwrap().count(), 0);

    // Removed once: the second write is renamed into place.
    let tmp = tempfile::tempdir().unwrap();
    let mut renames = 0;
    super::super::handoff::leave_through(tmp.path(), &request, |from, to| {
        renames += 1;
        if renames == 1 {
            std::fs::remove_file(from).unwrap();
        }
        std::fs::rename(from, to)
    })
    .unwrap();
    assert_eq!(renames, 2);
    let names: Vec<_> = std::fs::read_dir(tmp.path())
        .unwrap()
        .map(|e| e.unwrap().file_name().into_string().unwrap())
        .collect();
    assert_eq!(names, vec![REQUEST_FILE.to_string()]);
    let path = tmp.path().join(REQUEST_FILE);
    assert_eq!(std::fs::read_to_string(&path).unwrap(), REQUEST);
    assert_eq!(std::fs::metadata(&path).unwrap().mode() & 0o777, 0o600);
}

#[test]
fn a_leave_that_fails_returns_the_error_and_leaves_no_staging_file() {
    let request = cerulion_pairing::pair_code::PairCodeRequest::new(
        cerulion_pairing::pair_code::PairCodeNonce::from_bytes([7; 16]),
        "48213907",
    )
    .unwrap();
    let names = |dir: &Path| -> Vec<String> {
        let mut names: Vec<_> = std::fs::read_dir(dir)
            .unwrap()
            .map(|e| e.unwrap().file_name().into_string().unwrap())
            .collect();
        names.sort();
        names
    };
    // A rename refused for another reason than a removed staging file: the
    // staging file it wrote is removed.
    let tmp = tempfile::tempdir().unwrap();
    let error = super::super::handoff::leave_through(tmp.path(), &request, |from, _| {
        assert_eq!(std::fs::read_to_string(from).unwrap(), REQUEST);
        Err(std::io::Error::from_raw_os_error(libc::EXDEV))
    })
    .unwrap_err();
    assert_eq!(error.raw_os_error(), Some(libc::EXDEV));
    assert_eq!(names(tmp.path()), Vec::<String>::new());

    // A result that cannot be removed, a non-empty directory at its name:
    // nothing is written.
    let tmp = tempfile::tempdir().unwrap();
    std::fs::create_dir(tmp.path().join(RESULT_FILE)).unwrap();
    std::fs::write(tmp.path().join(RESULT_FILE).join("x"), b"").unwrap();
    let mut renamed = false;
    super::super::handoff::leave_through(tmp.path(), &request, |from, to| {
        renamed = true;
        std::fs::rename(from, to)
    })
    .unwrap_err();
    assert!(!renamed, "nothing is renamed into place");
    assert_eq!(names(tmp.path()), vec![RESULT_FILE.to_string()]);
}

#[test]
fn only_an_owner_only_regular_file_of_this_user_is_a_lock() {
    use super::super::handoff::lock_is_private;
    const FILE: u32 = 0o100_000;
    const FIFO: u32 = 0o010_000;
    let cases = [
        (true, 501, FILE | 0o600, true),
        (true, 501, FILE | 0o400, true),
        // A FIFO, a character device or a socket at the name.
        (false, 501, FIFO | 0o600, false),
        // Another user's file.
        (true, 502, FILE | 0o600, false),
        (true, 0, FILE | 0o600, false),
        // A group or other bit.
        (true, 501, FILE | 0o640, false),
        (true, 501, FILE | 0o604, false),
        (true, 501, FILE | 0o610, false),
    ];
    for (is_file, uid, mode, private) in cases {
        assert_eq!(
            lock_is_private(is_file, uid, mode, 501),
            private,
            "is_file={is_file} uid={uid} mode={mode:o}"
        );
    }
}

// SPDX-License-Identifier: AGPL-3.0-only
//! Every `cerulion` verb the sweep can spawn, under the network kill switch,
//! over the real binary, on a machine that never signed in. A row the sweep
//! does not spawn names why in the table.
//!
//! The rows are `src/network_switch/cases.rs`, the table the binary's own unit
//! tests (`network_switch::tests`) hold against the command tree: a verb the
//! CLI gains is a row here, or that unit test fails.
//!
//! Every spawn points the account service, the telemetry host (with a key)
//! and the relay at one loopback listener of its own, which records every
//! connection it accepts. A run that contacted any of them leaves a record.
//! Each spawn also gets a private iceoryx2 registry: a short root under
//! `/tmp` with its own prefix. The tests whose rows publish, subscribe or
//! read the registry are `#[serial]` as well.
//!
//! The listener stops and its thread is joined when the row ends, and every
//! spawn runs within a bound from its start: one still running then is
//! killed, reaped and named in the failure.
//! The listener sees only those three: mDNS, DDS and zenoh traffic is not
//! recorded.
//!
//! A refused row runs with stdin and stderr on a pseudo-terminal, as a person
//! at a shell runs it. There the login gate, had it run, would start the
//! device-code sign-in against that listener. Under the switch the row must
//! exit with its own refusal, print no sign-in text and leave no record;
//! without the switch the same row meets the gate. A gated row runs with
//! plain pipes: under the switch it meets the gate as it does without it
//! (exit 7, the gate's refusal) and leaves no record, so nothing reached the
//! telemetry host either. The verb itself does not run: the gate stops it
//! first. A row that runs without the gate exits as the table says, with no
//! record.
//!
//! `CERULION_LOGIN_GATE` is removed on every spawn: the workspace cargo
//! configuration sets it to `off` for this repository's runs, and this file
//! must meet the gate as a user's machine does.
#![cfg(unix)]

use std::io::{BufRead, BufReader, Read};
use std::net::TcpListener;
use std::path::Path;
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use serial_test::serial;

#[allow(dead_code)]
#[path = "../src/network_switch/cases.rs"]
mod cases;

use cases::{By, Case, Expect, Home, CASES};

/// The refusal the gate prints on a machine that never signed in, without a
/// terminal: its first line, hand-written from the text a user reads.
const GATE_REFUSAL: &str =
    "Error: This machine has never signed in to a Cerulion account, and every command needs one.";

/// What one spawn did.
struct Run {
    code: Option<i32>,
    stdout: String,
    stderr: String,
    /// One entry per connection the listener accepted: the request line, or
    /// `<no request line>`.
    contacts: Vec<String>,
}

/// How long one spawn may run, from the moment it starts until it has exited
/// and closed its output. A spawn still running then is killed, reaped and
/// named in the failure.
const SPAWN_BOUND: Duration = Duration::from_secs(20);

/// How long a dropped [`Recorder`]'s thread may take to see the stop flag:
/// one accept poll plus one request-line read.
const RECORDER_STOP_BOUND: Duration = Duration::from_secs(5);

/// A loopback listener that records every connection, then closes it. The
/// listener polls a non-blocking accept and checks a stop flag between
/// polls; dropping the recorder sets the flag and joins the thread, which
/// closes the listener, so a finished row leaves no socket or thread behind.
struct Recorder {
    url: String,
    contacts: mpsc::Receiver<String>,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

fn recorder() -> Recorder {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind the recording listener");
    listener
        .set_nonblocking(true)
        .expect("a non-blocking listener");
    let url = format!(
        "http://{}",
        listener.local_addr().expect("listener address")
    );
    let (tx, contacts) = mpsc::channel();
    let stop = Arc::new(AtomicBool::new(false));
    let stopped = Arc::clone(&stop);
    let thread = std::thread::spawn(move || {
        while !stopped.load(Ordering::Relaxed) {
            let stream = match listener.accept() {
                Ok((stream, _)) => stream,
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    std::thread::sleep(Duration::from_millis(10));
                    continue;
                }
                Err(_) => continue,
            };
            let _ = stream.set_nonblocking(false);
            let _ = stream.set_read_timeout(Some(Duration::from_millis(300)));
            let mut line = String::new();
            let _ = BufReader::new(&stream).read_line(&mut line);
            let line = line.trim();
            let seen = if line.is_empty() {
                "<no request line>".to_string()
            } else {
                line.to_string()
            };
            if tx.send(seen).is_err() {
                return;
            }
        }
    });
    Recorder {
        url,
        contacts,
        stop,
        thread: Some(thread),
    }
}

impl Drop for Recorder {
    /// Stop the listener's thread and join it within [`RECORDER_STOP_BOUND`].
    /// A thread still running then fails the row, unless the row is already
    /// failing.
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        let Some(thread) = self.thread.take() else {
            return;
        };
        let deadline = Instant::now() + RECORDER_STOP_BOUND;
        while !thread.is_finished() && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(5));
        }
        if thread.is_finished() {
            let _ = thread.join();
        } else if !std::thread::panicking() {
            panic!(
                "the recorder at {} did not stop within {RECORDER_STOP_BOUND:?}",
                self.url
            );
        }
    }
}

impl Recorder {
    /// Every connection accepted by now, waiting a little for a late accept:
    /// the run has exited, so each of its connections already sits in the
    /// listener's queue.
    fn drain(&self) -> Vec<String> {
        let mut out = Vec::new();
        while let Ok(seen) = self.contacts.recv_timeout(Duration::from_millis(300)) {
            out.push(seen);
        }
        out
    }
}

/// Point a spawn at a private iceoryx2 registry: iceoryx2 reads
/// `config/iceoryx2.toml` relative to the process cwd first. The root is a
/// short directory under `/tmp`, so the event socket paths under it stay
/// below the Unix pathname limit whatever `TMPDIR` is, and the prefix is the
/// root's own name, so no two spawns share a shared-memory or socket name.
/// The returned directory is the root; it lives as long as the spawn.
fn isolate_iceoryx2(cwd: &Path) -> tempfile::TempDir {
    let dir = cwd.join("config");
    std::fs::create_dir_all(&dir).expect("create the config dir");
    let root = tempfile::Builder::new()
        .prefix("nss")
        .tempdir_in("/tmp")
        .expect("create the iceoryx2 root");
    let escaped = root
        .path()
        .display()
        .to_string()
        .replace('\\', "\\\\")
        .replace('"', "\\\"");
    let prefix = root
        .path()
        .file_name()
        .and_then(|name| name.to_str())
        .expect("a UTF-8 root name");
    std::fs::write(
        dir.join("iceoryx2.toml"),
        format!("[global]\nroot-path = \"{escaped}\"\nprefix = \"{prefix}_\"\n"),
    )
    .expect("write the iceoryx2 config");
    root
}

/// How many descriptors this process has open, read from `/dev/fd`. The
/// read itself holds one while it runs, the same one each time.
fn open_fds() -> usize {
    std::fs::read_dir("/dev/fd").expect("list /dev/fd").count()
}

/// The binary under `home`, in a fresh non-workspace cwd, with every service
/// URL at `rec`, a telemetry key, and the gate, consent and CI variables
/// cleared. `network` is the `CERULION_NETWORK` value (`None` = unset).
fn command(
    home: &Path,
    cwd: &Path,
    rec: &Recorder,
    network: Option<&str>,
    argv: &[&str],
) -> Command {
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    cmd.args(argv)
        .current_dir(cwd)
        .env_remove("CERULION_LOGIN_GATE")
        .env_remove("CERULION_NETWORK")
        .env_remove("CERULION_NETD_NETWORK")
        .env_remove("CERULION_VIZD_NETWORK")
        .env_remove("DO_NOT_TRACK")
        .env_remove("CERULION_TELEMETRY")
        .env_remove("CERULION_CONNECTD_BIN")
        .env("CERULION_HOME", home)
        .env("CERULION_ACCOUNT_SERVICE", &rec.url)
        .env("CERULION_RELAY_URL", &rec.url)
        .env("POSTHOG_API_KEY", "sweep-key")
        .env("POSTHOG_HOST", &rec.url);
    for name in cerulion_telemetry::ci::CI_ENV_VARS {
        cmd.env_remove(name);
    }
    if let Some(value) = network {
        cmd.env("CERULION_NETWORK", value);
    }
    cmd
}

/// Serializes [`open_pty`]: `ptsname` returns a static buffer.
static PTY_TURN: std::sync::Mutex<()> = std::sync::Mutex::new(());

/// A pseudo-terminal pair: the master to read from and the slave, opened
/// close-on-exec, to hand to the child.
fn open_pty() -> (std::fs::File, std::os::fd::OwnedFd) {
    use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
    let _turn = PTY_TURN.lock().unwrap_or_else(|p| p.into_inner());
    let last = || std::io::Error::last_os_error();
    // SAFETY: plain libc calls on descriptors this function owns; the
    // `ptsname` buffer is read before the lock above is released.
    unsafe {
        let master = libc::posix_openpt(libc::O_RDWR | libc::O_NOCTTY);
        assert!(master >= 0, "posix_openpt: {}", last());
        let master = OwnedFd::from_raw_fd(master);
        let fd = master.as_raw_fd();
        assert_eq!(
            libc::fcntl(fd, libc::F_SETFD, libc::FD_CLOEXEC),
            0,
            "{}",
            last()
        );
        assert_eq!(libc::grantpt(fd), 0, "grantpt: {}", last());
        assert_eq!(libc::unlockpt(fd), 0, "unlockpt: {}", last());
        let name = libc::ptsname(fd);
        assert!(!name.is_null(), "ptsname: {}", last());
        let slave = libc::open(name, libc::O_RDWR | libc::O_NOCTTY | libc::O_CLOEXEC);
        assert!(slave >= 0, "open the slave: {}", last());
        (std::fs::File::from(master), OwnedFd::from_raw_fd(slave))
    }
}

/// Spawn `argv` under `home` with plain pipes, or with stdin and stderr on a
/// pseudo-terminal when `terminal`.
fn spawn(home: &Path, network: Option<&str>, argv: &[&str], terminal: bool) -> Run {
    spawn_with(home, network, &[], argv, terminal)
}

/// Read `from` to its end on a thread of its own.
fn collect(mut from: impl Read + Send + 'static) -> mpsc::Receiver<Vec<u8>> {
    let (tx, rx) = mpsc::channel();
    std::thread::spawn(move || {
        let mut bytes = Vec::new();
        let _ = from.read_to_end(&mut bytes);
        let _ = tx.send(bytes);
    });
    rx
}

/// The exit code of `child`, polled until `deadline`. Past it the child is
/// killed and reaped, and the row fails naming `argv` and the child's pid.
fn wait_until(child: &mut Child, argv: &[&str], deadline: Instant) -> Option<i32> {
    loop {
        if let Some(status) = child.try_wait().expect("poll the child") {
            return status.code();
        }
        if Instant::now() >= deadline {
            let _ = child.kill();
            let status = child.wait().expect("reap the child");
            panic!(
                "{argv:?}: pid {} was still running at its bound; killed and reaped ({status})",
                child.id()
            );
        }
        std::thread::sleep(Duration::from_millis(10));
    }
}

/// What `rx` collected, by `deadline`: a stream a process the child started
/// still holds open fails the row naming `argv`.
fn collected(
    rx: &mpsc::Receiver<Vec<u8>>,
    argv: &[&str],
    what: &str,
    deadline: Instant,
) -> Vec<u8> {
    rx.recv_timeout(deadline.saturating_duration_since(Instant::now()))
        .unwrap_or_else(|e| panic!("{argv:?}: its {what} was still open at its bound ({e})"))
}

/// Run `cmd` (the invocation `argv`) to its end within `bound` from its
/// start: with plain pipes, or with stdin and stderr on a pseudo-terminal
/// when `terminal`. Returns the exit code, stdout and stderr.
fn run_bounded(
    mut cmd: Command,
    argv: &[&str],
    terminal: bool,
    bound: Duration,
) -> (Option<i32>, String, String) {
    if !terminal {
        cmd.stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());
        let mut child = cmd.spawn().expect("spawn the child");
        let deadline = Instant::now() + bound;
        let stdout = collect(child.stdout.take().expect("a stdout pipe"));
        let stderr = collect(child.stderr.take().expect("a stderr pipe"));
        let code = wait_until(&mut child, argv, deadline);
        return (
            code,
            String::from_utf8_lossy(&collected(&stdout, argv, "stdout", deadline)).into_owned(),
            String::from_utf8_lossy(&collected(&stderr, argv, "stderr", deadline)).into_owned(),
        );
    }
    let (mut master, slave) = open_pty();
    let stdin = slave.try_clone().expect("a second slave descriptor");
    cmd.stdin(Stdio::from(stdin))
        .stdout(Stdio::piped())
        .stderr(Stdio::from(slave));
    let (chunks, received) = mpsc::channel::<Option<Vec<u8>>>();
    std::thread::spawn(move || {
        let mut buf = [0_u8; 4096];
        // Ends on EOF or EIO, which a pty master reads once every copy of
        // its slave is closed.
        loop {
            match master.read(&mut buf) {
                Ok(0) => break,
                Ok(n) => {
                    if chunks.send(Some(buf[..n].to_vec())).is_err() {
                        return;
                    }
                }
                Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
                Err(_) => break,
            }
        }
        let _ = chunks.send(None);
    });
    let mut child = cmd.spawn().expect("spawn the child");
    let deadline = Instant::now() + bound;
    // `cmd` holds the parent's copies of the slave.
    drop(cmd);
    let stdout = collect(child.stdout.take().expect("a stdout pipe"));
    let code = wait_until(&mut child, argv, deadline);
    let stdout = collected(&stdout, argv, "stdout", deadline);
    let mut stderr = Vec::new();
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        match received.recv_timeout(left) {
            Ok(Some(chunk)) => stderr.extend(chunk),
            Ok(None) => break,
            Err(e) => panic!(
                "{argv:?}: the terminal never reached its end ({e}); read so far: {:?}",
                String::from_utf8_lossy(&stderr)
            ),
        }
    }
    (
        code,
        String::from_utf8_lossy(&stdout).into_owned(),
        String::from_utf8_lossy(&stderr).replace("\r\n", "\n"),
    )
}

/// [`spawn`] with the variables `extra` set as well.
fn spawn_with(
    home: &Path,
    network: Option<&str>,
    extra: &[(&str, &str)],
    argv: &[&str],
    terminal: bool,
) -> Run {
    let before = open_fds();
    let rec = recorder();
    let cwd = tempfile::tempdir().expect("a cwd");
    let iox2 = isolate_iceoryx2(cwd.path());
    let mut cmd = command(home, cwd.path(), &rec, network, argv);
    for (name, value) in extra {
        cmd.env(name, value);
    }
    let (code, stdout, stderr) = run_bounded(cmd, argv, terminal, SPAWN_BOUND);
    let contacts = rec.drain();
    drop(rec);
    drop(iox2);
    drop(cwd);
    eprintln!("{argv:?}: fds before={before} after={}", open_fds());
    Run {
        code,
        stdout,
        stderr,
        contacts,
    }
}

/// No text of the sign-in: the gate's refusal, the prompt it prints before a
/// device-code sign-in, or the command that signs in. (`logout`'s own "was
/// not signed in" is none of these.)
fn assert_no_sign_in(what: &str, run: &Run) {
    let lower = run.stderr.to_lowercase();
    for marker in [
        "never signed in",
        "not signed in to a cerulion account",
        "signing you in",
        "cerulion login",
        "verification",
    ] {
        assert!(
            !lower.contains(marker),
            "{what}: sign-in text ({marker:?}) under the switch; stderr={}",
            run.stderr
        );
    }
}

/// No telemetry notice: under `CERULION_NETWORK` or `--network off` a run
/// sends nothing, so it has nothing to notify, even on a terminal.
fn assert_no_telemetry_notice(what: &str, run: &Run) {
    assert!(
        !run.stderr.contains("cerulion telemetry off"),
        "{what}: the telemetry notice under the switch; stderr={}",
        run.stderr
    );
}

fn assert_no_contact(what: &str, run: &Run) {
    assert!(
        run.contacts.is_empty(),
        "{what}: the run connected to the account service, the telemetry host or the relay: \
         {:?}; stderr={}",
        run.contacts,
        run.stderr
    );
}

fn assert_no_identity_written(what: &str, home: &Path) {
    assert!(
        !home.join("auth.json").exists(),
        "{what}: the run left this machine signed in"
    );
}

/// The `CERULION_NETWORK` value a refused row runs under: `off`, or unset for
/// a row its own `--network off` refuses.
fn switch_of(by: By) -> Option<&'static str> {
    match by {
        By::Network | By::ExactOff => Some("off"),
        By::Flag => None,
    }
}

/// Run `row` on every case, catching a row that fails so one run names every
/// failing row, then fail once with the list. `row` returns `false` for a case
/// its arm does not cover. Returns how many rows the arm covered.
fn each(arm: &str, mut row: impl FnMut(&Case) -> bool) -> usize {
    let mut covered = 0;
    let mut failed = Vec::new();
    for case in CASES {
        match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| row(case))) {
            Ok(true) => covered += 1,
            Ok(false) => {}
            Err(panic) => {
                covered += 1;
                let text = panic
                    .downcast_ref::<String>()
                    .cloned()
                    .or_else(|| panic.downcast_ref::<&str>().map(|s| s.to_string()))
                    .unwrap_or_default();
                failed.push(format!("{:?}: {text}", case.argv));
            }
        }
    }
    assert!(
        failed.is_empty(),
        "{arm}: {} of {covered} rows failed:\n{}",
        failed.len(),
        failed.join("\n")
    );
    covered
}

#[test]
fn every_verb_that_reaches_the_network_refuses_before_the_login_gate() {
    let refused = each("refused rows under the switch", |case| {
        let Expect::Refused { code, says, by } = case.expect else {
            return false;
        };
        let what = format!("{:?} under {by:?}", case.argv);
        let home = tempfile::tempdir().expect("a home");
        let run = spawn(home.path(), switch_of(by), case.argv, true);
        assert_eq!(
            run.code,
            Some(i32::from(code)),
            "{what}: the verb's own refusal code; stdout={} stderr={}",
            run.stdout,
            run.stderr
        );
        for fragment in says {
            assert!(
                run.stderr.contains(fragment),
                "{what}: the refusal ({fragment:?}); stderr={}",
                run.stderr
            );
        }
        assert_no_sign_in(&what, &run);
        assert_no_telemetry_notice(&what, &run);
        assert_no_contact(&what, &run);
        assert!(run.stdout.is_empty(), "{what}: stdout={}", run.stdout);
        assert_no_identity_written(&what, home.path());
        true
    });
    assert!(refused >= 7, "the table lost its refused rows: {refused}");
}

#[test]
fn without_the_switch_every_refused_verb_meets_the_login_gate() {
    // The control for the arm above: the same rows on the same kind of home,
    // switch unset and no terminal, are refused by the gate like every gated
    // verb, so the arm above cannot pass on a verb that skipped the gate.
    let controls = each("refused rows without the switch", |case| {
        let Expect::Refused { by, .. } = case.expect else {
            return false;
        };
        if by == By::Flag {
            return false;
        }
        let what = format!("{:?} without the switch", case.argv);
        let home = tempfile::tempdir().expect("a home");
        let run = spawn(home.path(), None, case.argv, false);
        assert_eq!(run.code, Some(7), "{what}: stderr={}", run.stderr);
        assert!(
            run.stderr.contains(GATE_REFUSAL),
            "{what}: the gate's refusal; stderr={}",
            run.stderr
        );
        // The listener sees what leaves without the switch: the usage event a
        // build with telemetry sends. Every "no connection" in this file is
        // read off the same kind of listener.
        if cfg!(feature = "telemetry") {
            assert!(
                run.contacts.iter().any(|c| c.starts_with("POST /batch")),
                "{what}: the usage event reaches the listener without the switch: {:?}",
                run.contacts
            );
        }
        assert_no_identity_written(&what, home.path());
        true
    });
    assert!(controls >= 6, "the table lost its refused rows: {controls}");
}

#[test]
fn every_verb_the_switch_does_not_refuse_meets_the_login_gate_and_sends_nothing() {
    let gated = each("gated rows under the switch", |case| {
        if !matches!(case.expect, Expect::Gated) {
            return false;
        }
        let what = format!("{:?} under CERULION_NETWORK=off", case.argv);
        let home = tempfile::tempdir().expect("a home");
        let run = spawn(home.path(), Some("off"), case.argv, false);
        assert_eq!(
            run.code,
            Some(7),
            "{what}: the gate refuses as without the switch; stdout={} stderr={}",
            run.stdout,
            run.stderr
        );
        assert!(
            run.stderr.contains(GATE_REFUSAL),
            "{what}: the gate's refusal; stderr={}",
            run.stderr
        );
        assert!(
            !run.stderr.contains("disables") && !run.stderr.contains("kill-switch"),
            "{what}: a local verb is not refused by the switch; stderr={}",
            run.stderr
        );
        assert_no_contact(&what, &run);
        assert!(run.stdout.is_empty(), "{what}: stdout={}", run.stdout);
        true
    });
    assert!(gated >= 40, "the table lost its gated rows: {gated}");
}

#[test]
#[serial]
fn every_verb_without_the_gate_runs_under_the_switch_and_sends_nothing() {
    let ran = each("ungated rows under the switch", |case| {
        let Expect::Runs { code, says } = case.expect else {
            return false;
        };
        let what = format!("{:?} under CERULION_NETWORK=off", case.argv);
        let home = tempfile::tempdir().expect("a home");
        let run = spawn(home.path(), Some("off"), case.argv, false);
        assert_eq!(
            run.code,
            Some(code),
            "{what}: stdout={} stderr={}",
            run.stdout,
            run.stderr
        );
        let both = format!("{}{}", run.stdout, run.stderr);
        assert!(
            both.contains(says),
            "{what}: its own output ({says:?}); stdout={} stderr={}",
            run.stdout,
            run.stderr
        );
        assert_no_sign_in(&what, &run);
        assert_no_contact(&what, &run);
        true
    });
    assert!(ran >= 6, "the table lost its ungated rows: {ran}");
}

#[test]
fn viz_robot_without_a_topic_is_answered_before_the_switch_and_the_gate() {
    // A command line that is wrong gets its usage refusal first, as on a
    // signed-in machine, with neither the switch refusal nor a sign-in.
    let home = tempfile::tempdir().expect("a home");
    let run = spawn(
        home.path(),
        Some("off"),
        &["viz", "--robot", "bench-arm"],
        true,
    );
    let what = "viz --robot with no TOPIC under CERULION_NETWORK=off";
    assert_eq!(run.code, Some(1), "{what}: stderr={}", run.stderr);
    assert!(
        run.stderr
            .contains("Error: cerulion viz --robot needs at least one TOPIC"),
        "{what}: stderr={}",
        run.stderr
    );
    assert!(
        !run.stderr.contains("remote visualization is disabled"),
        "{what}: stderr={}",
        run.stderr
    );
    assert_no_sign_in(what, &run);
    assert_no_telemetry_notice(what, &run);
    assert_no_contact(what, &run);
}

#[test]
fn viz_robot_under_a_daemon_switch_refuses_before_the_login_gate() {
    // `viz --robot` reaches the robot through the daemons, so their own
    // switches refuse it as `CERULION_NETWORK` does, before the gate.
    for (name, label) in [
        (
            "CERULION_NETD_NETWORK",
            "disabled by CERULION_NETD_NETWORK=off; unset CERULION_NETD_NETWORK",
        ),
        (
            "CERULION_VIZD_NETWORK",
            "disabled by CERULION_VIZD_NETWORK=off; unset CERULION_VIZD_NETWORK",
        ),
    ] {
        let what = format!("viz --robot under {name}=off");
        let home = tempfile::tempdir().expect("a home");
        let run = spawn_with(
            home.path(),
            None,
            &[(name, "off")],
            &["viz", "/camera", "--robot", "bench-arm"],
            true,
        );
        assert_eq!(run.code, Some(1), "{what}: stderr={}", run.stderr);
        assert!(
            run.stderr
                .contains("Error: cerulion viz --robot bench-arm: remote visualization is"),
            "{what}: stderr={}",
            run.stderr
        );
        assert!(run.stderr.contains(label), "{what}: stderr={}", run.stderr);
        assert_no_sign_in(&what, &run);
        assert_no_contact(&what, &run);
        assert_no_identity_written(&what, home.path());
    }
}

#[test]
fn graph_live_and_fires_under_the_netd_switch_refuse_before_the_login_gate() {
    // They ask robots through `cerulion-netd`, so its own switch refuses them
    // as `CERULION_NETWORK` does, before the gate.
    // With or without `--robot` and `--connect`: every form asks a robot.
    for argv in [
        &["graph", "live"][..],
        &["graph", "live", "--robot", "bench-arm"][..],
        &["graph", "live", "--connect", "tcp/127.0.0.1:7683"][..],
        &["graph", "fires"][..],
        &["graph", "fires", "--robot", "bench-arm"][..],
        &["graph", "fires", "--connect", "tcp/127.0.0.1:7683"][..],
    ] {
        let verb = &argv[..2];
        let what = format!("{argv:?} under CERULION_NETD_NETWORK=off");
        let home = tempfile::tempdir().expect("a home");
        let run = spawn_with(
            home.path(),
            None,
            &[("CERULION_NETD_NETWORK", "off")],
            argv,
            true,
        );
        assert_eq!(run.code, Some(1), "{what}: stderr={}", run.stderr);
        assert!(
            run.stderr.contains(&format!(
                "Error: cerulion {}: asking robots is disabled by CERULION_NETD_NETWORK=off",
                verb.join(" ")
            )),
            "{what}: stderr={}",
            run.stderr
        );
        assert!(
            run.stderr
                .contains("unset CERULION_NETD_NETWORK to ask robots"),
            "{what}: stderr={}",
            run.stderr
        );
        assert_no_sign_in(&what, &run);
        assert_no_contact(&what, &run);
        assert!(run.stdout.is_empty(), "{what}: stdout={}", run.stdout);
        assert_no_identity_written(&what, home.path());
    }
}

/// A signed-in home.
fn seeded_home() -> tempfile::TempDir {
    let home = tempfile::tempdir().expect("a home");
    cerulion_cli_engine::auth::seed_logged_in_at(home.path(), "acct-network-switch-sweep")
        .expect("seed the home");
    home
}

#[test]
#[serial]
fn the_local_forms_of_the_robot_verbs_run_under_the_switch() {
    // Only `--robot` is refused: on a signed-in machine the same verbs on this
    // machine run their own path under the switch and reach nothing.
    let home = seeded_home();
    let run = spawn(
        home.path(),
        Some("off"),
        &[
            "topic",
            "pub",
            "/network_switch_sweep",
            "std_msgs/String",
            "{data: sweep}",
            "--wait-subscribers",
            "0",
        ],
        false,
    );
    assert_eq!(run.code, Some(0), "topic pub: stderr={}", run.stderr);
    assert!(
        run.stdout
            .starts_with("published: topic=/network_switch_sweep type=std_msgs/String frames=1"),
        "topic pub: stdout={} stderr={}",
        run.stdout,
        run.stderr
    );
    assert_no_contact("topic pub", &run);

    let run = spawn(
        home.path(),
        Some("off"),
        &[
            "topic",
            "snapshot",
            "/network_switch_sweep_none",
            "--timeout",
            "0",
        ],
        false,
    );
    assert_eq!(run.code, Some(3), "topic snapshot: stderr={}", run.stderr);
    assert!(
        !run.stderr.contains("disables"),
        "topic snapshot: stderr={}",
        run.stderr
    );
    assert_no_contact("topic snapshot", &run);

    let run = spawn(
        home.path(),
        Some("off"),
        &[
            "service",
            "call",
            "/network_switch_sweep_srv",
            "std_srvs/srv/Trigger",
            "--timeout",
            "0",
        ],
        false,
    );
    assert_ne!(run.code, Some(7), "service call: stderr={}", run.stderr);
    assert!(
        !run.stderr.contains("disables") && !run.stderr.contains("signed in"),
        "service call: stderr={}",
        run.stderr
    );
    assert_no_contact("service call", &run);
}

#[test]
fn a_signed_in_machine_gets_the_same_refusal_under_the_switch() {
    // The refusal does not depend on the sign-in: a signed-in machine gets the
    // text and code a never-signed-in one gets above.
    let refused = each("refused rows on a signed-in machine", |case| {
        let Expect::Refused { code, says, by } = case.expect else {
            return false;
        };
        let what = format!("{:?} under {by:?} on a signed-in machine", case.argv);
        let home = seeded_home();
        let run = spawn(home.path(), switch_of(by), case.argv, false);
        assert_eq!(
            run.code,
            Some(i32::from(code)),
            "{what}: stderr={}",
            run.stderr
        );
        for fragment in says {
            assert!(
                run.stderr.contains(fragment),
                "{what}: stderr={}",
                run.stderr
            );
        }
        assert_no_contact(&what, &run);
        true
    });
    assert!(refused >= 7, "the table lost its refused rows: {refused}");
}

/// Signing in is not covered by the switch: under `CERULION_NETWORK=off`
/// each sign-in row still reaches the account service, its first request
/// line the one the table names, and it is not refused. No usage event is
/// sent. On a machine that never signed in, a local verb at a terminal starts
/// the login gate's sign-in.
#[test]
fn account_sign_in_is_allowed_under_the_switch_and_reaches_the_account_service() {
    let signs_in = each("sign-in rows under the switch", |case| {
        let Expect::SignIn {
            home,
            terminal,
            request,
        } = case.expect
        else {
            return false;
        };
        let what = format!("{:?} under CERULION_NETWORK=off", case.argv);
        let home = match home {
            Home::Fresh => tempfile::tempdir().expect("a home"),
            Home::SignedIn => seeded_home(),
        };
        let run = spawn(home.path(), Some("off"), case.argv, terminal);
        assert!(
            run.contacts
                .first()
                .is_some_and(|line| line.starts_with(request)),
            "{what}: the first request is not {request:?}: {:?}; stderr={}",
            run.contacts,
            run.stderr
        );
        assert!(
            !run.contacts
                .iter()
                .any(|line| line.starts_with("POST /batch")),
            "{what}: a usage event was sent: {:?}",
            run.contacts
        );
        assert!(
            !run.stderr.contains(" disables;") && !run.stderr.contains("kill-switch engaged"),
            "{what}: refused: {}",
            run.stderr
        );
        true
    });
    assert!(signs_in >= 5, "the table lost its sign-in rows: {signs_in}");
}

/// A recorder closes its listener and ends its thread when it is dropped,
/// whether or not anything connected to it, so a finished row leaves no
/// socket or thread behind.
#[test]
fn a_dropped_recorder_leaves_no_listener_behind() {
    use std::io::Write;
    use std::net::TcpStream;
    for connect_first in [false, true] {
        let rec = recorder();
        let addr = rec.url.trim_start_matches("http://").to_string();
        if connect_first {
            let mut probe = TcpStream::connect(&addr).expect("the listener accepts");
            probe
                .write_all(b"GET /probe HTTP/1.1\r\n\r\n")
                .expect("send a request line");
            drop(probe);
            assert_eq!(rec.drain(), ["GET /probe HTTP/1.1"], "{addr}");
        }
        drop(rec);
        let refused = TcpStream::connect(&addr).expect_err("the listener is closed");
        assert_eq!(
            refused.kind(),
            std::io::ErrorKind::ConnectionRefused,
            "{addr} after a drop (connected first: {connect_first})"
        );
    }
}

/// A spawn still running at its bound is killed, reaped and named, on plain
/// pipes and on a pseudo-terminal: a stuck row fails instead of holding the
/// sweep.
#[test]
fn a_spawn_still_running_at_its_bound_is_killed_and_named() {
    for terminal in [false, true] {
        let mut cmd = Command::new("sleep");
        cmd.arg("30");
        let started = Instant::now();
        let failed = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            run_bounded(cmd, &["sleep", "30"], terminal, Duration::from_millis(500))
        }))
        .expect_err("a child that outlives its bound fails the row");
        let text = failed.downcast_ref::<String>().cloned().unwrap_or_default();
        assert!(
            started.elapsed() < Duration::from_secs(10),
            "terminal={terminal}: the bound did not hold: {:?}",
            started.elapsed()
        );
        assert!(
            text.starts_with(r#"["sleep", "30"]: pid "#) && text.contains("killed and reaped"),
            "terminal={terminal}: {text}"
        );
        let pid: i32 = text
            .split("pid ")
            .nth(1)
            .and_then(|rest| rest.split(' ').next())
            .and_then(|pid| pid.parse().ok())
            .unwrap_or_else(|| panic!("no pid in {text}"));
        // SAFETY: signal 0 only asks whether `pid` exists.
        let alive = unsafe { libc::kill(pid, 0) } == 0;
        assert!(!alive, "terminal={terminal}: pid {pid} survived its bound");
    }
}

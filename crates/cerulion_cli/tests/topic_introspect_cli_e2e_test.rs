// SPDX-License-Identifier: AGPL-3.0-only
//! `cli-topic-introspect` gap: `cerulion_cli_engine/tests/
//! topic_observer_iox2_test.rs` is already comprehensive at the FUNCTION
//! layer — it calls `topic_cmd::topic_echo`/`topic_cmd::topic_hz` directly
//! against a raw iceoryx2 publisher it hand-builds in-process. What it does
//! NOT prove is that the actual `cerulion topic echo`/`cerulion topic hz`
//! SUBCOMMANDS — dispatched through clap argument parsing and `main.rs` —
//! work end-to-end against a LIVE process. That is the CLI-dispatch-layer
//! gap the matrix's "CLI: none" column flags for topic
//! introspection, and this file closes it.
//!
//! Spawns the real `cerulion` binary twice per test: once as `cerulion graph
//! run demo` (a hand-built tempdir workspace whose node cdylib is a COPY of
//! the prebuilt `test_node_macro_period_cdylib` fixture — no in-test cargo
//! build, cribbed from `graph_record_e2e_test.rs`'s `build_workspace`), and
//! once as `cerulion topic echo <topic>` / `cerulion topic hz <topic>`
//! against the live publisher the first process owns. Neither `topic echo`
//! nor `topic hz` has a bounded-run flag (`cerulion_cli/src/cli.rs`'s
//! `TopicAction` has no `--count`/`--duration` — confirmed by reading), so
//! both are run until their OWN output appears (bounded overall deadline)
//! and then SIGINT'd (the same graceful `setup_ctrlc_handler` path `graph
//! run`'s Ctrl-C uses).
//!
//! The verb children are POLLED, never fixed-slept: the earlier
//! harness gave each child a FIXED 1000/1800 ms window, and on a loaded desk
//! the child attached but saw no frame inside it (empty stdout, exit 0,
//! assertion red — measured 3-of-4 echo-arm failures loaded vs 6-of-6 green
//! idle). `run_topic_verb_until` polls the growing stdout every
//! `VERB_OUTPUT_POLL` for the caller's readiness predicate under a generous
//! `VERB_OUTPUT_DEADLINE` ceiling, so a healthy run is as fast as before
//! while a slow one simply takes longer. A verb that genuinely produces
//! NOTHING still fails loudly: at the deadline the (empty) stdout is returned
//! and the caller's oracle assertion fires with the full capture.
//!
//! Every failure the harness can reach is ATTRIBUTABLE, and every diagnostic
//! carries BOTH of the verb child's streams (its stderr is captured to a file,
//! never `/dev/null` — a refusal like "topic not found" is written there, so
//! discarding it left the fast-fail panic reporting an exit status with an
//! empty capture):
//! * the verb child died before its output → its status + stdout + stderr;
//! * the `graph run` PUBLISHER died first → the panic names the publisher (with
//!   its own stderr), not the verb it starved — the observer is the victim, and
//!   without the watch a dead publisher burns the whole 30 s ceiling and then
//!   reads as a verb failure;
//! * a capture file that cannot be READ panics with the path + `io::Error`
//!   (never a silent empty string — an I/O failure must be distinguishable
//!   from "the child wrote nothing").
//!
//! Needs `cargo build -p test_node_macro_period_cdylib` first (panics with
//! that instruction if the fixture artifact is missing).
//!
//! Runs on the GLOBAL iceoryx2 namespace (no isolation seam at this layer);
//! `#[serial]` + a unique per-test prefix keep it safe.

#![cfg(unix)]

use std::path::{Path, PathBuf};
use std::process::{Child, Command, ExitStatus, Stdio};
use std::time::{Duration, Instant};

use serial_test::serial;

/// SIGKILL + reap on drop so a panicking test never leaks a child.
struct ChildGuard(Child);
impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

/// Poll `try_wait` until the child exits or `timeout` elapses.
fn wait_bounded(child: &mut Child, timeout: Duration) -> Option<ExitStatus> {
    let start = Instant::now();
    loop {
        match child.try_wait().expect("try_wait") {
            Some(status) => return Some(status),
            None if start.elapsed() > timeout => return None,
            None => std::thread::sleep(Duration::from_millis(20)),
        }
    }
}

fn send_signal(pid: u32, sig: libc::c_int) {
    // SAFETY: kill(2) with a valid pid + signal; no memory is touched.
    unsafe {
        libc::kill(pid as libc::pid_t, sig);
    }
}

/// The platform cdylib filename for a crate/node name.
fn dylib_file(name: &str) -> String {
    if cfg!(target_os = "macos") {
        format!("lib{name}.dylib")
    } else {
        format!("lib{name}.so")
    }
}

/// The prebuilt fixture cdylib (`test_node_macro_period_cdylib` — a
/// `#[cerulion_node(period_ms = 50)]` node with one `cmd: Vector3` output).
/// PANICS with the build instruction if missing (the repo's fixture pattern).
fn fixture_cdylib() -> PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_macro_period_cdylib")
}

/// Hand-build a minimal single-node graph workspace in `root`: `Cargo.toml` +
/// `graphs/demo.yaml` (one `ticker` period node producing
/// `/{prefix}/ticker/cmd`) + `nodes/ticker/src/lib.rs` (a copy of the
/// fixture source, for the metadata walkers) + `target/debug/libticker.*`
/// (a copy of the prebuilt fixture cdylib). Mirrors
/// `graph_record_e2e_test.rs`'s `build_workspace`, minus the recording bits
/// this file doesn't need.
fn build_workspace(root: &Path, prefix: &str) {
    std::fs::create_dir_all(root.join("graphs")).unwrap();
    std::fs::create_dir_all(root.join("nodes/ticker/src")).unwrap();
    std::fs::create_dir_all(root.join("target/debug")).unwrap();
    std::fs::write(
        root.join("Cargo.toml"),
        "[workspace]\nresolver = \"2\"\nmembers = []\n",
    )
    .unwrap();
    std::fs::write(
        root.join("graphs/demo.yaml"),
        format!(
            "name: demo\nprefix: {prefix}\nnodes:\n- id: ticker\n  type: ticker\n  inputs: []\n  \
             outputs:\n  - name: cmd\n    schema: geometry_msgs/Vector3\n"
        ),
    )
    .unwrap();
    let fixture_src = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("test_fixtures/test_node_macro_period_cdylib/src/lib.rs");
    std::fs::copy(&fixture_src, root.join("nodes/ticker/src/lib.rs")).expect("copy fixture src");
    std::fs::copy(
        fixture_cdylib(),
        root.join("target/debug").join(dylib_file("ticker")),
    )
    .expect("copy fixture cdylib");
}

/// Spawn `cerulion graph run demo --no-validate --single-process` (NO
/// `--record` — this file needs only a live publisher, not a bag) in `root`,
/// stdout to a file.
///
/// `--single-process`: this file pins MONOLITH
/// topic introspection — `topic echo`/`hz` are spawned separately and look on
/// the DEFAULT iceoryx2 namespace. Before the default-namespace move the flag was also load-bearing
/// for visibility: the multi-process default would derive an in-memory
/// partition on this no-TTY subprocess and run the workers in a run-scoped
/// `cer_d_{hex}` namespace the introspection processes could never see.
/// Since that move the mp data plane is ALSO the default namespace (the mp
/// visibility pin lives in `mp_default_ns_e2e_test.rs`); the flag stays so
/// THIS file keeps pinning the single-process monolith shape.
fn spawn_graph_run(root: &Path) -> (ChildGuard, PathBuf) {
    let stdout_path = root.join("run.stdout");
    let stderr_path = root.join(RUN_STDERR);
    let child = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["graph", "run", "demo", "--no-validate", "--single-process"])
        .current_dir(root)
        .env_remove("CARGO_TARGET_DIR")
        // Hermetic — no scouting session/gateway in CI (a real-clock
        // run is permissive-by-default; the kill-switch env keeps it LOCAL-ONLY).
        .env("CERULION_NETWORK", "off")
        .env("RUST_LOG", "cerulion=info,cerulion_cli_engine=info")
        .stdout(Stdio::from(std::fs::File::create(&stdout_path).unwrap()))
        .stderr(Stdio::from(std::fs::File::create(&stderr_path).unwrap()))
        .spawn()
        .expect("spawn cerulion graph run");
    // Return the STDERR path: the only thing callers read off this run is the
    // "starting graph (live)" readiness breadcrumb, a `tracing::info!` that goes
    // to STDERR (init_logging writes there so a command's stdout stays clean
    // data). `topic echo`/`hz` DATA is captured separately in
    // `run_topic_verb_until`.
    (ChildGuard(child), stderr_path)
}

/// The `graph run` child's stderr capture, relative to the workspace root.
/// Named ONCE so `spawn_graph_run` (which creates it) and the verb watcher
/// (which quotes it when the publisher dies) can never drift apart.
const RUN_STDERR: &str = "run.stderr";

/// Bounded poll for `needle` in the (growing) log file at `path`; panics at
/// the deadline (the caller's `ChildGuard` reaps on the panic unwind).
fn wait_for_log_line(path: &Path, needle: &str, timeout: Duration) -> String {
    let start = Instant::now();
    loop {
        let content = read_file(path);
        if content.contains(needle) {
            return content;
        }
        assert!(
            start.elapsed() < timeout,
            "log line {needle:?} not seen within {timeout:?}; log so far:\n{content}"
        );
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// Read a child's capture file.
///
/// An I/O failure PANICS with the path + error rather than
/// degrading to `""` — the harness's whole verdict is "did the child write the
/// expected line", so an unreadable capture silently reading as "the child
/// wrote nothing" would turn an infrastructure fault into a bogus product
/// failure (or, at the ceiling, into a bogus assertion message).
///
/// Bytes + `from_utf8_lossy` (not `read_to_string`): the file is read WHILE the
/// child writes it, so a sample can land mid-multi-byte-character. That must
/// degrade one glyph, not raise `InvalidData` — and it costs nothing, because
/// the readiness predicates only ever accept COMPLETE lines
/// (see [`any_complete_line`]).
fn read_file(p: &Path) -> String {
    match std::fs::read(p) {
        Ok(bytes) => String::from_utf8_lossy(&bytes).into_owned(),
        Err(e) => panic!(
            "failed to read the child capture at {}: {e} — an unreadable capture is an \
             infrastructure fault, never evidence that the child wrote nothing",
            p.display()
        ),
    }
}

/// Like [`read_file`] but DEGRADES to an explanatory placeholder instead of
/// panicking — for use inside a panic message that is ALREADY reporting a more
/// important failure.
///
/// Why this exists: the publisher-death diagnostic hard-codes
/// `root.join(RUN_STDERR)`, a file only [`spawn_graph_run`] creates, while the
/// watcher's signature takes a generic `publisher: Option<&mut Child>` whose doc
/// explicitly contemplates other callers. The `RUN_STDERR` const prevents a NAME
/// drift but cannot express that EXISTENCE coupling in the type. With the strict
/// [`read_file`], a future caller passing a differently-spawned publisher would
/// have its panic-message arguments evaluated FIRST, so the strict reader would
/// panic and REPLACE the "the publisher died" attribution with a bare
/// "failed to read the child capture" — destroying exactly the attribution that
/// diagnostic exists to provide. Degrading here keeps the real cause on screen
/// and names the missing capture as a secondary note. Every readiness-polling
/// site keeps the strict [`read_file`]: there an unreadable capture IS the
/// primary fault and must be loud.
fn read_file_or_note(p: &Path) -> String {
    match std::fs::read(p) {
        Ok(bytes) => String::from_utf8_lossy(&bytes).into_owned(),
        Err(e) => format!(
            "<no capture at {}: {e} — this diagnostic expects a publisher spawned by \
             `spawn_graph_run`, which creates that file; the publisher-death attribution below \
             still stands>",
            p.display()
        ),
    }
}

/// Generous overall ceiling for a `topic` verb child to produce the
/// output its caller is waiting for. Sized for a heavily loaded desk (the
/// publisher, the observer, and every other test on the machine compete for
/// cores); a healthy run exits at the first matching line, ~the old fixed
/// window, so the ceiling costs nothing when things are well.
const VERB_OUTPUT_DEADLINE: Duration = Duration::from_secs(30);

/// How often the growing stdout is re-read while waiting. Tight
/// enough that the poll adds no meaningful latency over the old sleep.
const VERB_OUTPUT_POLL: Duration = Duration::from_millis(50);

/// True iff some FULLY-TERMINATED line of `s` satisfies `pred`.
///
/// The `\n` requirement is the anti-partial-read guard: the verb children
/// write through `std::io::Stdout` (a `LineWriter`, so each `writeln!`
/// flushes the WHOLE line in one write), but a reader that samples the file
/// mid-write must never accept a half-formed line — the `hz` arm goes on to
/// PARSE a float out of the matched line.
fn any_complete_line(s: &str, pred: impl Fn(&str) -> bool) -> bool {
    s.split_inclusive('\n')
        .filter(|l| l.ends_with('\n'))
        .any(|l| pred(l.trim_end_matches('\n')))
}

/// Spawn `cerulion topic <verb> <topic>` and POLL its (growing) stdout until
/// `ready` accepts the captured text or `deadline` elapses (neither subcommand
/// has a `--count`/`--duration` flag — confirmed absent from `TopicAction` in
/// `cli.rs`), then SIGINT it (the same graceful `setup_ctrlc_handler` path
/// `graph run`'s Ctrl-C uses) and return its FINAL captured stdout (re-read
/// after exit, so anything written between the last poll and the signal is
/// still seen).
///
/// POLL, don't fixed-sleep. A fixed sleep of 1000/1800 ms would leave the
/// child attached but with no frame observed in that window on a loaded
/// desk, yielding empty stdout + exit 0 and a red assertion (3-of-4
/// loaded vs 6-of-6 idle). Waiting on the OUTPUT instead of on the clock makes
/// the harness load-insensitive without weakening anything: the deadline is a
/// ceiling, not a target, and a verb that produces nothing real still returns
/// empty stdout to the caller's oracle assertion, which fails loudly with the
/// full capture. A child that DIES before producing output fails immediately
/// (never silently polled to the ceiling).
///
/// `publisher` is the `graph run` child that feeds the topic (`None` when a
/// caller deliberately observes without one). It is polled alongside the verb:
/// if the PUBLISHER dies the panic names IT — otherwise a dead publisher burns
/// the full ceiling and then surfaces as "the verb produced nothing", blaming
/// the observer for its data source's death.
///
/// Exit accounting is ordering-safe. The stdout sample at the top of a poll
/// iteration is older than the `try_wait` at the bottom, so a child that
/// flushed the awaited line and exited inside the SAME 50 ms window would look
/// like a death-before-output: on a `try_wait` hit the capture is re-read ONCE
/// and the predicate re-checked before that verdict is reached. A child already
/// reaped by `try_wait` is never signalled afterwards (its pid may already be
/// recycled) — its own exit status is used instead.
///
/// LOCAL-ONLY: the `graph run` child above already carries
/// `CERULION_NETWORK=off`, but this `topic` verb child inherited nothing. `topic
/// echo`/`hz`'s `ensure_topic_available` routes a topic that is NOT (yet) locally
/// listed to the netd demand rung on the automagic scouting-ON path, which
/// `connect_or_spawn()`s the WELL-KNOWN `cerulion-netd` socket — so in the race
/// window before the publisher's service lands in the local registry this test
/// could SPAWN a machine-wide network daemon on the developer's desk and burn a LAN
/// gather inside its bounded window. The kill switch is consulted FIRST in
/// `ensure_topic_available`, and both arms here echo/hz a LOCAL topic, so the
/// assertions are unchanged.
fn run_topic_verb_until(
    root: &Path,
    verb: &str,
    topic: &str,
    deadline: Duration,
    publisher: Option<&mut Child>,
    ready: impl Fn(&str) -> bool,
) -> String {
    let stdout_path = root.join(format!("{verb}.stdout"));
    // CAPTURE stderr, never `Stdio::null`. Every refusal the CLI
    // can print — an unresolvable topic, a schema mismatch — goes there, so
    // discarding it would make the fast-fail panic report a bare exit status.
    let stderr_path = root.join(format!("{verb}.stderr"));
    let child = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["topic", verb, topic])
        .current_dir(root)
        .env("CERULION_NETWORK", "off")
        .stdout(Stdio::from(std::fs::File::create(&stdout_path).unwrap()))
        .stderr(Stdio::from(std::fs::File::create(&stderr_path).unwrap()))
        .spawn()
        .unwrap_or_else(|e| panic!("spawn cerulion topic {verb}: {e}"));
    // SIGKILL+reap on any panic below (early-exit, deadline assertion in the
    // caller, …) so a failing run never leaks the verb child.
    let mut guard = ChildGuard(child);
    let mut publisher = publisher;

    // BOTH streams in every diagnostic — stdout carries the data the oracle
    // wants, stderr the reason it is missing.
    let capture = |stdout_path: &Path, stderr_path: &Path| -> String {
        format!(
            "--- cerulion topic {verb} stdout ---\n{}--- cerulion topic {verb} stderr ---\n{}",
            read_file(stdout_path),
            read_file(stderr_path)
        )
    };

    let start = Instant::now();
    // `Some(status)` = the child exited ON ITS OWN having produced the awaited
    // output (already reaped — never signal it); `None` = still running.
    let exited: Option<ExitStatus> = loop {
        if ready(&read_file(&stdout_path)) {
            break None;
        }
        if start.elapsed() >= deadline {
            break None;
        }
        // The publisher must outlive the observer: if the data SOURCE died, the
        // verb is the victim, not the culprit — attribute it correctly instead
        // of polling a doomed observer to the ceiling.
        if let Some(pubchild) = publisher.as_deref_mut() {
            if let Some(status) = pubchild.try_wait().expect("try_wait publisher") {
                panic!(
                    "the `cerulion graph run` PUBLISHER exited ({status:?}) while `cerulion topic \
                     {verb}` was waiting for output — the verb never had a topic to observe.\n\
                     --- graph run stderr ---\n{}{}",
                    // DEGRADING read: never let a missing capture displace the
                    // publisher-death attribution this panic exists to deliver.
                    read_file_or_note(&root.join(RUN_STDERR)),
                    capture(&stdout_path, &stderr_path)
                );
            }
        }
        // These verbs run until Ctrl-C, so an exit here is a real failure
        // (e.g. topic resolution refused) — surface it now with the capture
        // rather than burning the whole deadline on a dead process.
        if let Some(status) = guard.0.try_wait().expect("try_wait") {
            // Re-read ONCE before judging: the sample at the top of this
            // iteration predates the exit, so a child that flushed its line and
            // exited inside this same window HAS produced its output.
            if ready(&read_file(&stdout_path)) {
                break Some(status);
            }
            panic!(
                "cerulion topic {verb} exited early ({status:?}) before producing the awaited \
                 output.\n{}",
                capture(&stdout_path, &stderr_path)
            );
        }
        std::thread::sleep(VERB_OUTPUT_POLL);
    };

    let status = match exited {
        Some(status) => status,
        None => {
            send_signal(guard.0.id(), libc::SIGINT);
            wait_bounded(&mut guard.0, Duration::from_secs(10)).unwrap_or_else(|| {
                panic!(
                    "cerulion topic {verb} did not exit after SIGINT.\n{}",
                    capture(&stdout_path, &stderr_path)
                )
            })
        }
    };
    assert!(
        status.success(),
        "cerulion topic {verb} must exit 0, got {status:?}.\n{}",
        capture(&stdout_path, &stderr_path)
    );
    read_file(&stdout_path)
}

/// The headline: `cerulion topic echo <topic>` as a real subprocess against a
/// real `cerulion graph run` publisher displays at least one delivered
/// frame, in `topic_cmd::topic_echo`'s wire-header format.
#[test]
#[serial]
fn topic_echo_subcommand_e2e_displays_delivered_frames() {
    let tmp = tempfile::tempdir().unwrap();
    let prefix = "topicintroa";
    build_workspace(tmp.path(), prefix);
    let (mut guard, stderr_path) = spawn_graph_run(tmp.path());

    // Readiness: the publisher service is built before this line prints (see
    // `node_run_e2e_test.rs`'s identical readiness note). It is a
    // `tracing::info!` → STDERR (see `spawn_graph_run`).
    wait_for_log_line(
        &stderr_path,
        "starting graph (live)",
        Duration::from_secs(30),
    );

    let topic = format!("/{prefix}/ticker/cmd");
    // Wait for the FIRST complete frame-header line rather than a
    // fixed window — the ticker's 50ms period delivers one in ~milliseconds on
    // an idle box and in however long it takes on a loaded one.
    let echo_log = run_topic_verb_until(
        tmp.path(),
        "echo",
        &topic,
        VERB_OUTPUT_DEADLINE,
        Some(&mut guard.0),
        |s| any_complete_line(s, |l| l.contains("seq=") && l.contains("schema=0x")),
    );
    assert!(
        echo_log.contains("seq=") && echo_log.contains("schema=0x"),
        "cerulion topic echo '{topic}' must display at least one delivered \
         frame (the `seq=.. ts=..ns schema=0x.. size=..` header line from \
         `topic_cmd::topic_echo`); stdout was:\n{echo_log}"
    );

    send_signal(guard.0.id(), libc::SIGINT);
    let status = wait_bounded(&mut guard.0, Duration::from_secs(40))
        .expect("graph run did not exit after SIGINT");
    assert!(status.success(), "graph run must exit 0, got {status:?}");
}

/// `cerulion topic hz <topic>` as a real subprocess reports a computed rate
/// after observing several of the ticker's 50ms-period publishes.
#[test]
#[serial]
fn topic_hz_subcommand_e2e_reports_a_rate() {
    let tmp = tempfile::tempdir().unwrap();
    let prefix = "topicintrob";
    build_workspace(tmp.path(), prefix);
    let (mut guard, stderr_path) = spawn_graph_run(tmp.path());

    // "starting graph (live)" is a `tracing::info!` → STDERR (see spawn_graph_run).
    wait_for_log_line(
        &stderr_path,
        "starting graph (live)",
        Duration::from_secs(30),
    );

    let topic = format!("/{prefix}/ticker/cmd");
    // `topic_hz` reports once per second (`HZ_REPORT_INTERVAL`); a 50ms
    // period yields ~20 samples within the first report window. Wait
    // for that report LINE (complete, so the float below is always parseable)
    // instead of a fixed 1800ms window that a loaded desk can overrun.
    let hz_log = run_topic_verb_until(
        tmp.path(),
        "hz",
        &topic,
        VERB_OUTPUT_DEADLINE,
        Some(&mut guard.0),
        |s| any_complete_line(s, |l| l.starts_with("average rate: ")),
    );
    assert!(
        hz_log.contains("average rate:"),
        "cerulion topic hz '{topic}' must report a computed rate over the \
         ticker's ~20 Hz (50ms period) publishes; stdout was:\n{hz_log}"
    );
    let hz_value: f64 = hz_log
        .lines()
        .find_map(|l| l.strip_prefix("average rate: "))
        .and_then(|rest| rest.split(' ').next())
        .and_then(|s| s.parse().ok())
        .unwrap_or_else(|| panic!("could not parse a Hz value out of:\n{hz_log}"));
    assert!(
        (1.0..=100.0).contains(&hz_value),
        "reported rate {hz_value} Hz is not in a plausible range for a \
         50ms-period publisher (~20 Hz); stdout was:\n{hz_log}"
    );

    send_signal(guard.0.id(), libc::SIGINT);
    let status = wait_bounded(&mut guard.0, Duration::from_secs(40))
        .expect("graph run did not exit after SIGINT");
    assert!(status.success(), "graph run must exit 0, got {status:?}");
}

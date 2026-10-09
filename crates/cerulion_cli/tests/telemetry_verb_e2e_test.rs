// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion telemetry status|on|off` and the first-run notice over the real
//! binary. Each test isolates `CERULION_HOME` in its own tempdir and clears
//! every variable that could decide consent, so the machine's own settings
//! never leak in. The key, where one is set, points at a reserved dead port:
//! nothing can be delivered anywhere.
#![cfg(feature = "telemetry")]

use std::path::Path;
use std::process::Command;

use cerulion_cli_engine::auth;

struct Out {
    code: Option<i32>,
    stdout: String,
    stderr: String,
}

/// The binary, isolated under `home`, with every consent variable cleared
/// before `env` is applied. `cwd` must outlive the returned command.
fn command(home: &Path, env: &[(&str, &str)], args: &[&str], cwd: &Path) -> Command {
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    cmd.args(args)
        .current_dir(cwd)
        .env("CERULION_HOME", home)
        .env("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:1")
        .env_remove("DO_NOT_TRACK")
        .env_remove("CERULION_TELEMETRY")
        .env_remove("POSTHOG_API_KEY")
        .env_remove("POSTHOG_HOST");
    for (k, v) in env {
        cmd.env(k, v);
    }
    cmd
}

fn cerulion(home: &Path, env: &[(&str, &str)], args: &[&str]) -> Out {
    let cwd = tempfile::tempdir().unwrap();
    let out = command(home, env, args, cwd.path())
        .output()
        .expect("spawn cerulion");
    Out {
        code: out.status.code(),
        stdout: String::from_utf8_lossy(&out.stdout).into_owned(),
        stderr: String::from_utf8_lossy(&out.stderr).into_owned(),
    }
}

#[test]
fn status_needs_no_login_and_writes_nothing() {
    let home = tempfile::tempdir().unwrap();
    let out = cerulion(
        home.path(),
        &[("CERULION_LOGIN_GATE", "on")],
        &["telemetry", "status"],
    );
    assert_eq!(out.code, Some(0), "stderr={}", out.stderr);
    assert!(
        out.stdout.contains("telemetry: on (default)"),
        "{}",
        out.stdout
    );
    assert!(!home.path().join("telemetry.json").exists());
    // "on" is the decision, not a promise that data leaves: a keyless build
    // says so, and a build with a key does not.
    const NO_KEY: &str = "no telemetry key in this build: nothing is sent";
    assert!(out.stdout.contains(NO_KEY), "{}", out.stdout);
    let blank_key = cerulion(
        home.path(),
        &[("POSTHOG_API_KEY", " ")],
        &["telemetry", "status"],
    );
    assert!(blank_key.stdout.contains(NO_KEY), "{}", blank_key.stdout);
    let keyed = cerulion(
        home.path(),
        &[("POSTHOG_API_KEY", "k")],
        &["telemetry", "status"],
    );
    assert_eq!(keyed.code, Some(0), "stderr={}", keyed.stderr);
    assert!(
        keyed.stdout.contains("telemetry: on (default)"),
        "{}",
        keyed.stdout
    );
    assert!(!keyed.stdout.contains(NO_KEY), "{}", keyed.stdout);
    assert!(!home.path().join("telemetry.json").exists());
}

#[test]
fn off_then_on_persist_in_the_consent_file() {
    let home = tempfile::tempdir().unwrap();
    let off = cerulion(home.path(), &[], &["telemetry", "off"]);
    assert_eq!(off.code, Some(0), "stderr={}", off.stderr);
    assert!(
        off.stdout.contains("telemetry: off (set by"),
        "{}",
        off.stdout
    );
    let file = std::fs::read_to_string(home.path().join("telemetry.json")).unwrap();
    assert!(
        file.contains("\"enabled\":false") || file.contains("\"enabled\": false"),
        "{file}"
    );
    let status = cerulion(home.path(), &[], &["telemetry", "status"]);
    assert!(
        status.stdout.contains("telemetry: off"),
        "{}",
        status.stdout
    );
    let on = cerulion(home.path(), &[], &["telemetry", "on"]);
    assert_eq!(on.code, Some(0), "stderr={}", on.stderr);
    assert!(on.stdout.contains("telemetry: on"), "{}", on.stdout);
    let file = std::fs::read_to_string(home.path().join("telemetry.json")).unwrap();
    assert!(
        file.contains("\"enabled\":true") || file.contains("\"enabled\": true"),
        "{file}"
    );
    let status = cerulion(home.path(), &[], &["telemetry", "status"]);
    assert!(status.stdout.contains("telemetry: on"), "{}", status.stdout);
}

#[test]
fn do_not_track_wins_over_an_opt_in_and_says_so() {
    let home = tempfile::tempdir().unwrap();
    let out = cerulion(home.path(), &[("DO_NOT_TRACK", "1")], &["telemetry", "on"]);
    assert_eq!(out.code, Some(0), "stderr={}", out.stderr);
    assert!(
        out.stdout.contains("telemetry: off (set by DO_NOT_TRACK)"),
        "{}",
        out.stdout
    );
    assert!(
        out.stdout.contains("overrides the consent file"),
        "{}",
        out.stdout
    );
}

#[test]
fn the_notice_prints_once_and_only_when_a_key_could_send() {
    let home = tempfile::tempdir().unwrap();
    auth::seed_logged_in_at(home.path(), "8d1f4e6c-0b2a-4c5d-9e7f-123456789abc").unwrap();
    let args = ["graph", "list"];
    let keyless = cerulion(home.path(), &[], &args);
    assert!(
        !keyless.stderr.contains("usage events"),
        "{}",
        keyless.stderr
    );
    let key = [
        ("POSTHOG_API_KEY", "k"),
        ("POSTHOG_HOST", "http://127.0.0.1:1"),
    ];
    let first = cerulion(home.path(), &key, &args);
    assert_eq!(
        first.stderr.matches("cerulion telemetry off").count(),
        1,
        "{}",
        first.stderr
    );
    let second = cerulion(home.path(), &key, &args);
    assert!(!second.stderr.contains("usage events"), "{}", second.stderr);
    assert_eq!(
        first.code, second.code,
        "the notice never changes the exit code"
    );
}

#[test]
fn an_opted_out_machine_never_sees_the_notice() {
    let home = tempfile::tempdir().unwrap();
    auth::seed_logged_in_at(home.path(), "8d1f4e6c-0b2a-4c5d-9e7f-123456789abc").unwrap();
    cerulion(home.path(), &[], &["telemetry", "off"]);
    let key = [
        ("POSTHOG_API_KEY", "k"),
        ("POSTHOG_HOST", "http://127.0.0.1:1"),
    ];
    let out = cerulion(home.path(), &key, &["graph", "list"]);
    assert!(!out.stderr.contains("usage events"), "{}", out.stderr);
}

/// A loopback `/batch` sink: answers every request `200 {}` and hands the
/// request bodies back when dropped into [`Sink::bodies`].
struct Sink {
    url: String,
    bodies: std::sync::mpsc::Receiver<String>,
}

fn sink() -> Sink {
    use std::io::{BufRead, BufReader, Read, Write};
    let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let url = format!("http://{}", listener.local_addr().unwrap());
    let (tx, bodies) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        for stream in listener.incoming().flatten() {
            let mut reader = BufReader::new(stream);
            let mut len = 0usize;
            let mut line = String::new();
            let _ = reader.read_line(&mut line);
            if !line.starts_with("POST /batch ") {
                let _ = tx.send(format!("unexpected request line: {line}"));
                continue;
            }
            line.clear();
            while reader.read_line(&mut line).is_ok_and(|n| n > 0) && line != "\r\n" {
                if let Some(v) = line.to_ascii_lowercase().strip_prefix("content-length:") {
                    len = v.trim().parse().unwrap_or(0);
                }
                line.clear();
            }
            let mut body = vec![0; len];
            let _ = reader.read_exact(&mut body);
            let _ = tx.send(String::from_utf8_lossy(&body).into_owned());
            let _ = reader
                .get_mut()
                .write_all(b"HTTP/1.1 200 OK\r\ncontent-length: 2\r\nconnection: close\r\n\r\n{}");
        }
    });
    Sink { url, bodies }
}

fn sent_after_notice(home: &Path, sink: &Sink) -> String {
    let key = [
        ("POSTHOG_API_KEY", "k"),
        ("POSTHOG_HOST", sink.url.as_str()),
    ];
    let keyless = cerulion(home, &[], &["graph", "list"]);
    let notice = cerulion(home, &key, &["graph", "list"]);
    assert!(
        notice.stderr.contains("cerulion telemetry off"),
        "{}",
        notice.stderr
    );
    assert_eq!(
        notice.code, keyless.code,
        "the notice run still dispatches the command"
    );
    assert_nothing_sent(sink, "the notice run sends nothing");
    let out = cerulion(home, &key, &["graph", "list"]);
    assert!(!out.stderr.contains("usage events"), "{}", out.stderr);
    sink.bodies
        .recv_timeout(std::time::Duration::from_secs(5))
        .expect("the second run delivers one batch")
}

#[test]
fn a_hosted_account_is_the_distinct_id_and_only_allowlisted_props_leave() {
    let home = tempfile::tempdir().unwrap();
    let sub = "8d1f4e6c-0b2a-4c5d-9e7f-123456789abc";
    auth::seed_logged_in_at(home.path(), sub).unwrap();
    let sink = sink();
    let body = sent_after_notice(home.path(), &sink);
    assert!(body.contains("\"cli_command_run\""), "{body}");
    assert!(
        body.contains(&format!("\"distinct_id\":\"{sub}\"")),
        "{body}"
    );
    for key in [
        "\"verb\":\"graph\"",
        "\"subverb\":\"list\"",
        "\"exit_code\"",
        "\"duration_bucket\"",
    ] {
        assert!(body.contains(key), "{key} missing: {body}");
    }
    let batch: serde_json::Value = serde_json::from_str(&body).unwrap();
    let events = batch["batch"].as_array().unwrap();
    assert_eq!(events.len(), 1, "{body}");
    let allowed = [
        "$lib",
        "$lib_version",
        "app_version",
        "channel",
        "duration_bucket",
        "env",
        "exit_code",
        "subverb",
        "surface",
        "verb",
    ];
    for event in events {
        let props = event["properties"].as_object().unwrap();
        let keys: Vec<&str> = props.keys().map(String::as_str).collect();
        assert!(keys.iter().all(|k| allowed.contains(k)), "{keys:?}");
    }
    assert!(!body.contains(home.path().to_str().unwrap()), "{body}");
}

#[test]
fn a_non_uuid_account_id_falls_back_to_the_anonymous_id() {
    let home = tempfile::tempdir().unwrap();
    let base64url_id = "q83vEjRWeJCrze8SNFZ4kKvN7xI0VniQq83vEjRWeJA";
    auth::seed_logged_in_at(home.path(), base64url_id).unwrap();
    let sink = sink();
    let body = sent_after_notice(home.path(), &sink);
    assert!(!body.contains(base64url_id), "{body}");
    assert!(body.contains("\"distinct_id\":\"anon:"), "{body}");
}

#[test]
fn an_account_saved_without_its_rotation_gets_a_fresh_anonymous_id() {
    let home = tempfile::tempdir().unwrap();
    let account = "q83vEjRWeJCrze8SNFZ4kKvN7xI0VniQq83vEjRWeJA";
    auth::seed_logged_in_at(home.path(), account).unwrap();
    let sink = sink();
    let first = sent_after_notice(home.path(), &sink);
    let bound = home.path().join("telemetry_anon_account");
    assert_eq!(std::fs::read_to_string(&bound).unwrap(), account);
    let anon_id = |body: &str| -> String {
        let batch: serde_json::Value = serde_json::from_str(body).unwrap();
        batch["batch"][0]["distinct_id"]
            .as_str()
            .unwrap()
            .to_owned()
    };
    let old = anon_id(&first);
    assert!(old.starts_with("anon:"), "{first}");

    // A login that saved this account but failed before it rotated the id
    // leaves the previous account recorded.
    std::fs::write(&bound, "previous-account").unwrap();
    let key = [
        ("POSTHOG_API_KEY", "k"),
        ("POSTHOG_HOST", sink.url.as_str()),
    ];
    cerulion(home.path(), &key, &["graph", "list"]);
    let body = sink
        .bodies
        .recv_timeout(std::time::Duration::from_secs(5))
        .expect("a batch");
    let fresh = anon_id(&body);
    assert!(fresh.starts_with("anon:"), "{body}");
    assert_ne!(fresh, old, "the previous account's id is not reused");
    assert_eq!(std::fs::read_to_string(&bound).unwrap(), account);
}

#[test]
fn an_alias_left_pending_by_the_notice_run_is_merged_by_the_next_send() {
    let home = tempfile::tempdir().unwrap();
    let sub = "8d1f4e6c-0b2a-4c5d-9e7f-123456789abc";
    auth::seed_logged_in_at(home.path(), sub).unwrap();
    let marker = home.path().join("telemetry_alias_pending");
    std::fs::write(&marker, b"").unwrap();
    let sink = sink();
    let mut body = sent_after_notice(home.path(), &sink);
    while let Ok(more) = sink.bodies.recv_timeout(std::time::Duration::from_secs(2)) {
        body.push_str(&more);
    }
    let consent: serde_json::Value =
        serde_json::from_slice(&std::fs::read(home.path().join("telemetry.json")).unwrap())
            .unwrap();
    let anon_id = consent["anon_id"]
        .as_str()
        .expect("the consent file carries an anon id");
    let events: Vec<serde_json::Value> = serde_json::Deserializer::from_str(&body)
        .into_iter::<serde_json::Value>()
        .flat_map(|batch| batch.unwrap()["batch"].as_array().unwrap().clone())
        .collect();
    let aliases: Vec<&serde_json::Value> = events
        .iter()
        .filter(|e| e["event"] == "$create_alias")
        .collect();
    assert_eq!(aliases.len(), 1, "exactly one alias: {body}");
    let alias = aliases[0];
    assert_eq!(alias["properties"]["alias"], anon_id, "{alias}");
    assert_eq!(alias["distinct_id"], sub, "{alias}");
    assert!(
        events.iter().any(|e| e["event"] == "cli_command_run"),
        "{body}"
    );
    assert!(!marker.exists());
}

/// The `/batch` sink is given 500 ms to deliver a body, and must not.
fn assert_nothing_sent(sink: &Sink, why: &str) {
    match sink
        .bodies
        .recv_timeout(std::time::Duration::from_millis(500))
    {
        Err(_) => {}
        Ok(body) => panic!("{why}, but the sink received: {body}"),
    }
}

/// Opens `fifo` for writing without blocking, retrying until the command
/// has opened it for reading. Panics if the command exits first, so a
/// command that never reaches the read fails the test instead of hanging it.
#[cfg(unix)]
fn open_fifo_once_read(fifo: &Path, child: &mut std::process::Child) -> std::fs::File {
    use std::os::unix::fs::OpenOptionsExt;
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(30);
    loop {
        match std::fs::OpenOptions::new()
            .write(true)
            .custom_flags(libc::O_NONBLOCK)
            .open(fifo)
        {
            Ok(file) => return file,
            Err(e) if e.raw_os_error() == Some(libc::ENXIO) => {
                if let Some(status) = child.try_wait().unwrap() {
                    panic!("the command exited ({status}) before it read the trace");
                }
                assert!(
                    std::time::Instant::now() < deadline,
                    "the command never opened the trace file"
                );
                std::thread::sleep(std::time::Duration::from_millis(10));
            }
            Err(e) => panic!("open fifo for writing: {e}"),
        }
    }
}

/// `cerulion telemetry off` from another terminal while a command runs
/// stops that command's event: consent is read again when it finishes.
/// `trace inspect` over a FIFO holds the command mid-run under the test's
/// control; a plain file first proves the same verb does send.
#[cfg(unix)]
#[test]
fn an_opt_out_while_a_command_runs_stops_its_event() {
    let home = tempfile::tempdir().unwrap();
    auth::seed_logged_in_at(home.path(), "8d1f4e6c-0b2a-4c5d-9e7f-123456789abc").unwrap();
    let sink = sink();
    let key = [
        ("POSTHOG_API_KEY", "k"),
        ("POSTHOG_HOST", sink.url.as_str()),
    ];
    let notice = cerulion(home.path(), &key, &["graph", "list"]);
    assert!(
        notice.stderr.contains("cerulion telemetry off"),
        "{}",
        notice.stderr
    );
    assert_nothing_sent(&sink, "the notice run sends nothing");

    let traces = tempfile::tempdir().unwrap();
    let dir = traces.path().to_str().unwrap();
    std::fs::write(traces.path().join("trace_0.jsonl"), "").unwrap();
    let control = cerulion(home.path(), &key, &["trace", "inspect", dir]);
    let body = sink
        .bodies
        .recv_timeout(std::time::Duration::from_secs(5))
        .expect("an uninterrupted trace inspect sends its event");
    assert!(body.contains("\"verb\":\"trace\""), "{body}");
    assert!(body.contains("\"subverb\":\"inspect\""), "{body}");

    let fifo = traces.path().join("trace_0.jsonl");
    std::fs::remove_file(&fifo).unwrap();
    assert!(Command::new("mkfifo")
        .arg(&fifo)
        .status()
        .unwrap()
        .success());
    let cwd = tempfile::tempdir().unwrap();
    let mut child = command(home.path(), &key, &["trace", "inspect", dir], cwd.path())
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped())
        .spawn()
        .unwrap();
    // Returns only once the command is blocked reading the trace: its
    // `CommandRun` started with consent on, and it is still running.
    let writer = open_fifo_once_read(&fifo, &mut child);
    let off = cerulion(home.path(), &[], &["telemetry", "off"]);
    assert_eq!(off.code, Some(0), "stderr={}", off.stderr);
    drop(writer);
    let out = child.wait_with_output().unwrap();
    assert_eq!(
        out.status.code(),
        control.code,
        "the opt-out never changes the exit code: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    assert_nothing_sent(&sink, "a command opted out mid-run sends nothing");
}

/// A consent file this user cannot read may hold an opt-out: no notice, no
/// event, and `status` reports off.
#[cfg(unix)]
#[test]
fn an_unreadable_consent_file_shows_no_notice_and_sends_nothing() {
    use std::os::unix::fs::PermissionsExt;
    let home = tempfile::tempdir().unwrap();
    auth::seed_logged_in_at(home.path(), "8d1f4e6c-0b2a-4c5d-9e7f-123456789abc").unwrap();
    let on = cerulion(home.path(), &[], &["telemetry", "on"]);
    assert_eq!(on.code, Some(0), "stderr={}", on.stderr);
    let file = home.path().join("telemetry.json");
    std::fs::set_permissions(&file, std::fs::Permissions::from_mode(0o000)).unwrap();
    if std::fs::read(&file).is_ok() {
        // A superuser reads a mode 000 file; nothing to prove here.
        return;
    }
    let sink = sink();
    let key = [
        ("POSTHOG_API_KEY", "k"),
        ("POSTHOG_HOST", sink.url.as_str()),
    ];
    let out = cerulion(home.path(), &key, &["graph", "list"]);
    assert!(!out.stderr.contains("usage events"), "{}", out.stderr);
    assert_nothing_sent(&sink, "an unreadable consent file sends nothing");
    let status = cerulion(home.path(), &[], &["telemetry", "status"]);
    assert_eq!(status.code, Some(0), "stderr={}", status.stderr);
    assert!(
        status.stdout.contains("telemetry: off"),
        "{}",
        status.stdout
    );
}

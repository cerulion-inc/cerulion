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

fn cerulion(home: &Path, env: &[(&str, &str)], args: &[&str]) -> Out {
    let cwd = tempfile::tempdir().unwrap();
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    cmd.args(args)
        .current_dir(cwd.path())
        .env("CERULION_HOME", home)
        .env("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:1")
        .env_remove("DO_NOT_TRACK")
        .env_remove("CERULION_TELEMETRY")
        .env_remove("POSTHOG_API_KEY")
        .env_remove("POSTHOG_HOST");
    for (k, v) in env {
        cmd.env(k, v);
    }
    let out = cmd.output().expect("spawn cerulion");
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
    assert!(
        first.stderr.contains("cerulion telemetry off"),
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
    cerulion(home, &key, &["graph", "list"]);
    assert!(
        sink.bodies
            .recv_timeout(std::time::Duration::from_millis(500))
            .is_err(),
        "the notice run sends nothing"
    );
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

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

/// One HTTP/1.1 request off `reader`: its request line (`METHOD /path`) and
/// body, read up to `content-length`.
fn read_request(reader: &mut std::io::BufReader<std::net::TcpStream>) -> (String, String) {
    use std::io::{BufRead, Read};
    let mut line = String::new();
    let _ = reader.read_line(&mut line);
    let request = line.split(' ').take(2).collect::<Vec<_>>().join(" ");
    let mut len = 0usize;
    line.clear();
    while reader.read_line(&mut line).is_ok_and(|n| n > 0) && line != "\r\n" {
        if let Some(v) = line.to_ascii_lowercase().strip_prefix("content-length:") {
            len = v.trim().parse().unwrap_or(0);
        }
        line.clear();
    }
    let mut body = vec![0; len];
    let _ = reader.read_exact(&mut body);
    (request, String::from_utf8_lossy(&body).into_owned())
}

/// Answer one request with a JSON body and close the connection.
fn respond(stream: &mut std::net::TcpStream, status: &str, body: &str) {
    use std::io::Write;
    let _ = write!(
        stream,
        "HTTP/1.1 {status}\r\ncontent-type: application/json\r\ncontent-length: {}\r\n\
         connection: close\r\n\r\n{body}",
        body.len()
    );
}

fn sink() -> Sink {
    let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let url = format!("http://{}", listener.local_addr().unwrap());
    let (tx, bodies) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        for stream in listener.incoming().flatten() {
            let mut reader = std::io::BufReader::new(stream);
            let (request, body) = read_request(&mut reader);
            if request != "POST /batch" {
                let _ = tx.send(format!("unexpected request line: {request}"));
                continue;
            }
            let _ = tx.send(body);
            respond(reader.get_mut(), "200 OK", "{}");
        }
    });
    Sink { url, bodies }
}

/// A loopback stand-in for the hosted account service, which issues no device
/// certificates: `device/start` and `device/poll` answer at once, the device
/// endpoints are 404 and `/v1/me` names `account_id`. Each `device/start`
/// request body is handed back on [`Issuer::starts`].
struct Issuer {
    url: String,
    starts: std::sync::mpsc::Receiver<String>,
}

fn issuer(account_id: &str) -> Issuer {
    issuer_where(account_id, || {}, None)
}

/// [`issuer`] that runs `on_start` when a `device/start` request arrives,
/// before it is answered: the machine's state can be changed while the
/// login is in flight.
fn issuer_with(account_id: &str, on_start: impl Fn() + Send + 'static) -> Issuer {
    issuer_where(account_id, on_start, None)
}

/// [`issuer`] whose `device/poll` denies the authorization, so the login
/// fails after its device-start request.
fn issuer_denying(account_id: &str) -> Issuer {
    issuer_where(
        account_id,
        || {},
        Some(("400 Bad Request", r#"{"error":"access_denied"}"#)),
    )
}

fn issuer_where(
    account_id: &str,
    on_start: impl Fn() + Send + 'static,
    poll: Option<(&'static str, &'static str)>,
) -> Issuer {
    let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let url = format!("http://{}", listener.local_addr().unwrap());
    let (tx, starts) = std::sync::mpsc::channel();
    let me = serde_json::json!({ "account_id": account_id }).to_string();
    let start = serde_json::json!({
        "device_code": "device-code",
        "user_code": "BCDF-GHJK",
        "verification_uri": format!("{url}/device"),
        "verification_uri_complete": format!("{url}/device?user_code=BCDF-GHJK"),
        "expires_in": 60,
        "interval": 0,
    })
    .to_string();
    let tokens = serde_json::json!({
        "session_token": "session-token",
        "refresh_token": "refresh-token",
        "expires_in": 3600,
    })
    .to_string();
    std::thread::spawn(move || {
        for stream in listener.incoming().flatten() {
            let mut reader = std::io::BufReader::new(stream);
            let (request, body) = read_request(&mut reader);
            let (status, reply) = match request.as_str() {
                "POST /v1/auth/device/start" => {
                    on_start();
                    let _ = tx.send(body);
                    ("200 OK", start.as_str())
                }
                "POST /v1/auth/device/poll" => poll.unwrap_or(("200 OK", tokens.as_str())),
                "GET /v1/me" => ("200 OK", me.as_str()),
                _ => ("404 Not Found", "{}"),
            };
            respond(reader.get_mut(), status, reply);
        }
    });
    Issuer { url, starts }
}

/// The events in every batch the sink has received by half a second after
/// the last one, in order.
fn events_sent(sink: &Sink) -> Vec<serde_json::Value> {
    let mut events = Vec::new();
    while let Ok(body) = sink
        .bodies
        .recv_timeout(std::time::Duration::from_millis(500))
    {
        let batch: serde_json::Value = serde_json::from_str(&body).unwrap_or_else(|e| {
            panic!("not a batch ({e}): {body}");
        });
        events.extend(batch["batch"].as_array().expect("a batch array").clone());
    }
    events
}

/// The events named `name` among `events`.
fn named<'a>(events: &'a [serde_json::Value], name: &str) -> Vec<&'a serde_json::Value> {
    events.iter().filter(|e| e["event"] == name).collect()
}

/// `cerulion login` against `issuer` in a run that sends to `sink`: the
/// device-start body it sent and the events the run delivered.
fn login_sending(home: &Path, issuer: &Issuer, sink: &Sink) -> (String, Vec<serde_json::Value>) {
    let env = [
        ("POSTHOG_API_KEY", "k"),
        ("POSTHOG_HOST", sink.url.as_str()),
        ("CERULION_ACCOUNT_SERVICE", issuer.url.as_str()),
    ];
    let out = cerulion(home, &env, &["login"]);
    assert_eq!(out.code, Some(0), "stderr={}", out.stderr);
    assert!(out.stderr.contains("Signed in as"), "{}", out.stderr);
    let start = issuer
        .starts
        .recv_timeout(std::time::Duration::from_secs(1))
        .expect("one device-start request");
    (start, events_sent(sink))
}

fn anon_id_in(home: &Path) -> String {
    let consent: serde_json::Value =
        serde_json::from_slice(&std::fs::read(home.join("telemetry.json")).unwrap()).unwrap();
    consent["anon_id"]
        .as_str()
        .expect("the consent file carries an anon id")
        .to_owned()
}

fn bound_account_in(home: &Path) -> String {
    std::fs::read_to_string(home.join("telemetry_anon_account")).expect("the account record")
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

#[test]
fn a_first_login_in_a_sending_run_carries_merges_and_records_the_anonymous_id() {
    let home = tempfile::tempdir().unwrap();
    let sink = sink();
    let key = [
        ("POSTHOG_API_KEY", "k"),
        ("POSTHOG_HOST", sink.url.as_str()),
    ];
    // The notice run mints the consent file and sends nothing; the login
    // gate then refuses the command on a machine that never signed in.
    let notice = cerulion(home.path(), &key, &["graph", "list"]);
    assert!(
        notice.stderr.contains("cerulion telemetry off"),
        "{}",
        notice.stderr
    );
    assert_nothing_sent(&sink, "the notice run sends nothing");
    let anon = anon_id_in(home.path());

    let sub = "8d1f4e6c-0b2a-4c5d-9e7f-123456789abc";
    let (start, events) = login_sending(home.path(), &issuer(sub), &sink);
    assert_eq!(
        serde_json::from_str::<serde_json::Value>(&start).unwrap(),
        serde_json::json!({ "telemetry_anon_id": anon }),
        "the device-start body carries exactly the anonymous id"
    );
    let aliases = named(&events, "$create_alias");
    assert_eq!(aliases.len(), 1, "exactly one alias: {events:?}");
    assert_eq!(aliases[0]["distinct_id"], sub, "{events:?}");
    assert_eq!(aliases[0]["properties"]["alias"], anon, "{events:?}");
    let logins = named(&events, "cli_login_completed");
    assert_eq!(logins.len(), 1, "exactly one login event: {events:?}");
    assert_eq!(logins[0]["distinct_id"], sub, "{events:?}");
    assert_eq!(
        logins[0]["properties"]["is_account_switch"], false,
        "{events:?}"
    );
    let runs = named(&events, "cli_command_run");
    assert_eq!(runs.len(), 1, "{events:?}");
    assert_eq!(runs[0]["distinct_id"], sub, "{events:?}");
    assert_eq!(runs[0]["properties"]["verb"], "login", "{events:?}");
    assert_eq!(events.len(), 3, "{events:?}");
    assert_eq!(anon_id_in(home.path()), anon, "a first login keeps the id");
    assert_eq!(bound_account_in(home.path()), sub);
}

#[test]
fn a_login_as_another_account_rotates_the_anonymous_id_and_carries_nothing() {
    let home = tempfile::tempdir().unwrap();
    let first = "8d1f4e6c-0b2a-4c5d-9e7f-123456789abc";
    auth::seed_logged_in_at(home.path(), first).unwrap();
    let sink = sink();
    sent_after_notice(home.path(), &sink);
    assert_eq!(bound_account_in(home.path()), first);
    let anon = anon_id_in(home.path());

    // A switch: `auth.json` names the first account.
    let second = "2b7c9d1e-3f4a-4b5c-8d6e-7f8091a2b3c4";
    let (start, events) = login_sending(home.path(), &issuer(second), &sink);
    assert_eq!(start, "{}", "a signed-in machine carries no id");
    assert!(named(&events, "$create_alias").is_empty(), "{events:?}");
    let logins = named(&events, "cli_login_completed");
    assert_eq!(logins.len(), 1, "{events:?}");
    assert_eq!(logins[0]["distinct_id"], second, "{events:?}");
    assert_eq!(
        logins[0]["properties"]["is_account_switch"], true,
        "{events:?}"
    );
    let rotated = anon_id_in(home.path());
    assert_ne!(rotated, anon, "the switch replaces the id");
    assert!(rotated.starts_with("anon:"), "{rotated}");
    assert_eq!(bound_account_in(home.path()), second);

    // The sign-in state is removed but the id stays the second account's:
    // a login as a third account carries nothing and rotates it again.
    std::fs::remove_file(home.path().join("auth.json")).unwrap();
    let third = "c3d4e5f6-a7b8-4c9d-8e0f-1a2b3c4d5e6f";
    let (start, events) = login_sending(home.path(), &issuer(third), &sink);
    assert_eq!(start, "{}", "an id used for an account is not carried");
    assert!(named(&events, "$create_alias").is_empty(), "{events:?}");
    let logins = named(&events, "cli_login_completed");
    assert_eq!(logins.len(), 1, "{events:?}");
    assert_eq!(logins[0]["distinct_id"], third, "{events:?}");
    assert_eq!(
        logins[0]["properties"]["is_account_switch"], false,
        "without `auth.json` a login is not a switch: {events:?}"
    );
    assert_ne!(anon_id_in(home.path()), rotated, "{events:?}");
    assert_eq!(bound_account_in(home.path()), third);
}

#[test]
fn a_switch_whose_rotation_fails_sends_nothing_and_disowns_the_id() {
    let home = tempfile::tempdir().unwrap();
    let first = "8d1f4e6c-0b2a-4c5d-9e7f-123456789abc";
    auth::seed_logged_in_at(home.path(), first).unwrap();
    let sink = sink();
    sent_after_notice(home.path(), &sink);
    let anon = anon_id_in(home.path());

    // The consent file is written under `telemetry.json.lock`; a directory
    // there cannot be opened as the lock, so the rotation fails.
    let lock = home.path().join("telemetry.json.lock");
    let _ = std::fs::remove_file(&lock);
    std::fs::create_dir(&lock).unwrap();
    let second = "2b7c9d1e-3f4a-4b5c-8d6e-7f8091a2b3c4";
    let issuer = issuer(second);
    let env = [
        ("POSTHOG_API_KEY", "k"),
        ("POSTHOG_HOST", sink.url.as_str()),
        ("CERULION_ACCOUNT_SERVICE", issuer.url.as_str()),
    ];
    let out = cerulion(home.path(), &env, &["login"]);
    assert_eq!(out.code, Some(0), "stderr={}", out.stderr);
    assert_nothing_sent(&sink, "a run whose rotation failed sends nothing");
    assert_eq!(anon_id_in(home.path()), anon, "the id was not rotated");
    assert_eq!(
        bound_account_in(home.path()),
        "",
        "the id is recorded as no account's"
    );

    // The next run that may send rotates first, then sends as the account.
    std::fs::remove_dir(&lock).unwrap();
    let key = [
        ("POSTHOG_API_KEY", "k"),
        ("POSTHOG_HOST", sink.url.as_str()),
    ];
    cerulion(home.path(), &key, &["graph", "list"]);
    let events = events_sent(&sink);
    let runs = named(&events, "cli_command_run");
    assert_eq!(runs.len(), 1, "{events:?}");
    assert_eq!(runs[0]["distinct_id"], second, "{events:?}");
    assert_ne!(anon_id_in(home.path()), anon, "rotated before sending");
    assert_eq!(bound_account_in(home.path()), second);
}

#[cfg(unix)]
#[test]
fn an_unreadable_account_record_keeps_the_id_out_of_a_login() {
    use std::os::unix::fs::PermissionsExt;
    let home = tempfile::tempdir().unwrap();
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
    let anon = anon_id_in(home.path());
    // A record that exists but cannot be read: whose the id was is unknown.
    let record = home.path().join("telemetry_anon_account");
    std::fs::write(&record, "8d1f4e6c-0b2a-4c5d-9e7f-123456789abc").unwrap();
    std::fs::set_permissions(&record, std::fs::Permissions::from_mode(0o000)).unwrap();
    if std::fs::read(&record).is_ok() {
        // A superuser reads a mode 000 file; nothing to prove here.
        return;
    }

    let sub = "2b7c9d1e-3f4a-4b5c-8d6e-7f8091a2b3c4";
    let (start, events) = login_sending(home.path(), &issuer(sub), &sink);
    assert_eq!(start, "{}", "an id whose account is unknown is not carried");
    // A record this user can neither read nor write cannot take the account
    // either: like an unreadable consent file, it sends nothing.
    assert!(
        events.is_empty(),
        "an id whose account cannot be recorded sends nothing: {events:?}"
    );
    assert_ne!(
        anon_id_in(home.path()),
        anon,
        "an id of unknown account is replaced before it is used"
    );
}

/// `cerulion logout` keeps the account in `auth.json` without a session.
/// That record alone, with no account record for the id, keeps the id out
/// of the next login: the machine was signed in, so the next sign-in can be
/// someone else's, and it is a switch.
#[test]
fn a_signed_out_machine_carries_nothing_into_its_next_login() {
    let home = tempfile::tempdir().unwrap();
    let first = "8d1f4e6c-0b2a-4c5d-9e7f-123456789abc";
    auth::seed_logged_in_at(home.path(), first).unwrap();
    let sink = sink();
    sent_after_notice(home.path(), &sink);
    let anon = anon_id_in(home.path());
    let auth_path = home.path().join("auth.json");
    let signed_out = auth::signed_out_store(&std::fs::read(&auth_path).unwrap()).unwrap();
    std::fs::write(&auth_path, signed_out).unwrap();
    std::fs::remove_file(home.path().join("telemetry_anon_account")).unwrap();

    let second = "2b7c9d1e-3f4a-4b5c-8d6e-7f8091a2b3c4";
    let (start, events) = login_sending(home.path(), &issuer(second), &sink);
    assert_eq!(start, "{}", "a signed-out machine carries no id");
    assert!(named(&events, "$create_alias").is_empty(), "{events:?}");
    let logins = named(&events, "cli_login_completed");
    assert_eq!(logins.len(), 1, "{events:?}");
    assert_eq!(logins[0]["distinct_id"], second, "{events:?}");
    assert_eq!(
        logins[0]["properties"]["is_account_switch"], true,
        "the signed-out account is the one before: {events:?}"
    );
    assert_ne!(anon_id_in(home.path()), anon, "the switch replaces the id");
    assert_eq!(bound_account_in(home.path()), second);
}

/// The id is claimed, as an empty account record, before a login carries
/// it. A login that finds the claim is the second one: it carries nothing
/// (concurrent first logins lose the exclusive creation this way too), and
/// an id claimed by no completed login is replaced before it is used.
#[test]
fn an_id_another_login_claimed_is_not_carried_and_is_replaced() {
    let home = tempfile::tempdir().unwrap();
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
    let anon = anon_id_in(home.path());
    std::fs::write(home.path().join("telemetry_anon_account"), "").unwrap();

    let sub = "8d1f4e6c-0b2a-4c5d-9e7f-123456789abc";
    let (start, events) = login_sending(home.path(), &issuer(sub), &sink);
    assert_eq!(start, "{}", "a claimed id is not carried twice");
    assert!(named(&events, "$create_alias").is_empty(), "{events:?}");
    let logins = named(&events, "cli_login_completed");
    assert_eq!(logins.len(), 1, "{events:?}");
    assert_eq!(logins[0]["distinct_id"], sub, "{events:?}");
    assert_eq!(
        logins[0]["properties"]["is_account_switch"], false,
        "{events:?}"
    );
    assert_ne!(
        anon_id_in(home.path()),
        anon,
        "an id claimed by no completed login is replaced before it is used"
    );
    assert_eq!(bound_account_in(home.path()), sub);
}

/// The claim is made before the device-start request leaves. When the
/// account then cannot be written over it, the run sends nothing, and the
/// claim keeps the carried id out of every later login, with or without
/// `auth.json`.
#[cfg(unix)]
#[test]
fn a_login_whose_account_cannot_be_recorded_sends_nothing_and_keeps_its_claim() {
    use std::os::unix::fs::PermissionsExt;
    let home = tempfile::tempdir().unwrap();
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
    let anon = anon_id_in(home.path());
    let record = home.path().join("telemetry_anon_account");
    assert!(!record.exists(), "no login has claimed the id yet");

    // By the time the device-start request arrives the id is claimed; the
    // record is made read-only there, so the account cannot be written.
    let claimed = record.clone();
    let sub = "8d1f4e6c-0b2a-4c5d-9e7f-123456789abc";
    let read_only_record = issuer_with(sub, move || {
        let _ = std::fs::set_permissions(&claimed, std::fs::Permissions::from_mode(0o444));
    });
    let (start, events) = login_sending(home.path(), &read_only_record, &sink);
    assert_eq!(
        serde_json::from_str::<serde_json::Value>(&start).unwrap(),
        serde_json::json!({ "telemetry_anon_id": anon }),
        "the first login carries the id"
    );
    if std::fs::OpenOptions::new()
        .write(true)
        .open(&record)
        .is_ok()
    {
        // A superuser writes a read-only file; nothing to prove here.
        return;
    }
    assert!(
        events.is_empty(),
        "an id whose account is not on disk sends nothing: {events:?}"
    );
    assert_eq!(bound_account_in(home.path()), "", "the claim stays");

    // Without `auth.json`, the claimed id is still no login's to carry.
    std::fs::remove_file(home.path().join("auth.json")).unwrap();
    let second = "2b7c9d1e-3f4a-4b5c-8d6e-7f8091a2b3c4";
    let (start, events) = login_sending(home.path(), &issuer(second), &sink);
    assert_eq!(start, "{}", "a claimed id is never carried again");
    assert!(named(&events, "$create_alias").is_empty(), "{events:?}");
    assert_ne!(anon_id_in(home.path()), anon, "the claimed id is replaced");
}

/// The gate asks for the id only once it runs a login. A command the gate
/// refuses (not a terminal, so no login runs) claims nothing, and the login
/// that follows still carries the id.
#[test]
fn a_refused_command_claims_nothing_and_the_login_after_it_carries_the_id() {
    let home = tempfile::tempdir().unwrap();
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
    let anon = anon_id_in(home.path());

    // The repository's own runs switch the gate off (`.cargo/config.toml`);
    // this run needs it on, and no terminal, so the gate refuses.
    let gate_on = [
        ("POSTHOG_API_KEY", "k"),
        ("POSTHOG_HOST", sink.url.as_str()),
        ("CERULION_LOGIN_GATE", "on"),
    ];
    let refused = cerulion(home.path(), &gate_on, &["graph", "list"]);
    assert!(
        refused.stderr.contains("never signed in"),
        "the gate refuses the command: {}",
        refused.stderr
    );
    assert_eq!(
        refused.code,
        Some(i32::from(
            cerulion_cli_engine::login_cmd::EXIT_AUTH_REQUIRED
        )),
        "stderr={}",
        refused.stderr
    );
    let _ = events_sent(&sink);
    assert!(
        !home.path().join("telemetry_anon_account").exists(),
        "a refusal that ran no login claims nothing"
    );

    let sub = "8d1f4e6c-0b2a-4c5d-9e7f-123456789abc";
    let (start, events) = login_sending(home.path(), &issuer(sub), &sink);
    assert_eq!(
        serde_json::from_str::<serde_json::Value>(&start).unwrap(),
        serde_json::json!({ "telemetry_anon_id": anon }),
        "the login after the refusal still carries the id"
    );
    assert_eq!(named(&events, "$create_alias").len(), 1, "{events:?}");
    assert_eq!(bound_account_in(home.path()), sub);
}

/// A login that fails after its device-start request signed nothing in, so
/// its claim is released: the next login carries the same id.
#[test]
fn a_failed_login_releases_its_claim_for_the_next_login() {
    let home = tempfile::tempdir().unwrap();
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
    let anon = anon_id_in(home.path());

    let sub = "8d1f4e6c-0b2a-4c5d-9e7f-123456789abc";
    let denying = issuer_denying(sub);
    let env = [
        ("POSTHOG_API_KEY", "k"),
        ("POSTHOG_HOST", sink.url.as_str()),
        ("CERULION_ACCOUNT_SERVICE", denying.url.as_str()),
    ];
    let out = cerulion(home.path(), &env, &["login"]);
    assert_ne!(out.code, Some(0), "the login fails: {}", out.stderr);
    assert!(out.stderr.contains("access_denied"), "{}", out.stderr);
    let start = denying
        .starts
        .recv_timeout(std::time::Duration::from_secs(1))
        .expect("one device-start request");
    assert_eq!(
        serde_json::from_str::<serde_json::Value>(&start).unwrap(),
        serde_json::json!({ "telemetry_anon_id": anon }),
        "the failed login carried the id"
    );
    let events = events_sent(&sink);
    assert!(named(&events, "$create_alias").is_empty(), "{events:?}");
    assert!(
        named(&events, "cli_login_completed").is_empty(),
        "{events:?}"
    );
    assert!(
        !home.path().join("telemetry_anon_account").exists(),
        "the claim is released"
    );

    let (start, events) = login_sending(home.path(), &issuer(sub), &sink);
    assert_eq!(
        serde_json::from_str::<serde_json::Value>(&start).unwrap(),
        serde_json::json!({ "telemetry_anon_id": anon }),
        "the next login carries the same id"
    );
    let aliases = named(&events, "$create_alias");
    assert_eq!(aliases.len(), 1, "{events:?}");
    assert_eq!(aliases[0]["properties"]["alias"], anon, "{events:?}");
    assert_eq!(bound_account_in(home.path()), sub);
}

/// While a login that carried the id is in flight, another login completes
/// and binds the id to its account. The carrying login then rotates the id
/// for its own account and does not merge the id it carried from here: the
/// device-start request alone merges it, never a second account.
#[test]
fn an_id_bound_to_another_account_mid_login_is_not_aliased_from_here() {
    let home = tempfile::tempdir().unwrap();
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
    let anon = anon_id_in(home.path());

    // Once device-start arrives, the id is claimed; another login completes
    // as the first account and records it over the claim.
    let first = "2b7c9d1e-3f4a-4b5c-8d6e-7f8091a2b3c4";
    let record = home.path().join("telemetry_anon_account");
    let bound_by_another = issuer_with("8d1f4e6c-0b2a-4c5d-9e7f-123456789abc", move || {
        std::fs::write(&record, first).unwrap()
    });
    let (start, events) = login_sending(home.path(), &bound_by_another, &sink);
    assert_eq!(
        serde_json::from_str::<serde_json::Value>(&start).unwrap(),
        serde_json::json!({ "telemetry_anon_id": anon }),
        "this login carried the id"
    );
    assert!(
        named(&events, "$create_alias").is_empty(),
        "an id another account took meanwhile is not merged into this one: {events:?}"
    );
    let logins = named(&events, "cli_login_completed");
    assert_eq!(logins.len(), 1, "{events:?}");
    assert_eq!(
        logins[0]["distinct_id"], "8d1f4e6c-0b2a-4c5d-9e7f-123456789abc",
        "{events:?}"
    );
    assert_ne!(anon_id_in(home.path()), anon, "rotated for this account");
    assert_eq!(
        bound_account_in(home.path()),
        "8d1f4e6c-0b2a-4c5d-9e7f-123456789abc"
    );
}

/// A login in a run that sends nothing (no key) records no account for the
/// id: nothing was sent under it, so it stays the first sending login's to
/// carry, and the consent file is left as it was.
#[test]
fn a_login_that_sends_nothing_records_no_account_for_the_id() {
    let home = tempfile::tempdir().unwrap();
    let on = cerulion(home.path(), &[], &["telemetry", "on"]);
    assert_eq!(on.code, Some(0), "stderr={}", on.stderr);
    let anon = anon_id_in(home.path());
    let sub = "8d1f4e6c-0b2a-4c5d-9e7f-123456789abc";
    let issuer = issuer(sub);
    let out = cerulion(
        home.path(),
        &[("CERULION_ACCOUNT_SERVICE", issuer.url.as_str())],
        &["login"],
    );
    assert_eq!(out.code, Some(0), "stderr={}", out.stderr);
    assert!(out.stderr.contains("Signed in as"), "{}", out.stderr);
    let start = issuer
        .starts
        .recv_timeout(std::time::Duration::from_secs(1))
        .expect("one device-start request");
    assert_eq!(start, "{}", "a run that sends nothing carries nothing");
    assert!(
        !home.path().join("telemetry_anon_account").exists(),
        "no account is recorded for an id nothing was sent under"
    );
    assert_eq!(anon_id_in(home.path()), anon, "the id is kept");
}

/// The `/batch` sink is given 500 ms to deliver a body, and must not. Only a
/// timeout proves that: a sink that stopped listening observed nothing, so
/// a closed channel fails the test instead of passing it.
fn assert_nothing_sent(sink: &Sink, why: &str) {
    use std::sync::mpsc::RecvTimeoutError;
    match sink
        .bodies
        .recv_timeout(std::time::Duration::from_millis(500))
    {
        Err(RecvTimeoutError::Timeout) => {}
        Err(RecvTimeoutError::Disconnected) => {
            panic!("{why}, but the sink stopped before it could tell")
        }
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

// SPDX-License-Identifier: AGPL-3.0-only
//! Property guard rejection cases and golden `/batch` JSON (hand-written
//! oracles; `uuid`/`timestamp` are supplied so the bytes are fixed).
#![cfg(feature = "posthog")]

use cerulion_telemetry::guard::{self, Rejection};
use cerulion_telemetry::payload::{batch_json, event_json, Event, SET_ONCE_ALLOWLIST};
use cerulion_telemetry::{url_hash, Allowlist, Common, EventSpec, Value, LIB_VERSION};

const CMD_ALLOWLIST: Allowlist = &["command", "duration_ms", "is_tty", "exit_code"];
const SUB: &str = "8d1f4e6c-0b2a-4c5d-9e7f-123456789abc";
const ANON: &str = "anon:6ba7b810-9dad-41d1-80b4-00c04fd430c8";
const CMD: EventSpec = EventSpec {
    name: "cli_command_run",
    allowlist: CMD_ALLOWLIST,
};
const FIRST_RUN: EventSpec = EventSpec {
    name: "cli_first_run",
    allowlist: CMD_ALLOWLIST,
};

/// `guard::dropped_count` is process-wide, so tests asserting exact deltas
/// take turns.
static COUNTER: std::sync::Mutex<()> = std::sync::Mutex::new(());

fn counter_turn() -> std::sync::MutexGuard<'static, ()> {
    COUNTER.lock().unwrap_or_else(|p| p.into_inner())
}

fn common() -> Common {
    Common {
        surface: "cli".into(),
        env: "prod".into(),
        app_version: "0.1.0".into(),
        channel: Some("stable".into()),
    }
}

#[test]
fn guard_rejects_every_class_and_counts_them() {
    let _turn = counter_turn();
    let before = guard::dropped_count();
    let long = "x".repeat(129);
    let props = vec![
        ("command".to_owned(), Value::from("graph run")),
        ("duration_ms".to_owned(), Value::from(42_i64)),
        ("is_tty".to_owned(), Value::from(true)),
        ("topic".to_owned(), Value::from("camera")),
        ("$set".to_owned(), Value::from("x")),
        ("command".to_owned(), Value::from("https://example.com")),
        ("command".to_owned(), Value::from("see HTTP://x")),
        ("command".to_owned(), Value::from("bob@example.com")),
        ("command".to_owned(), Value::from("/etc/passwd")),
        ("command".to_owned(), Value::from(r"C:\Users\bob")),
        ("command".to_owned(), Value::from(long.as_str())),
        ("command".to_owned(), Value::from("é".repeat(128))),
        ("duration_ms".to_owned(), Value::Float(f64::NAN)),
        ("duration_ms".to_owned(), Value::Float(f64::INFINITY)),
    ];
    let (kept, dropped) = guard::filter(props, CMD_ALLOWLIST);
    assert_eq!(
        kept,
        vec![
            ("command".to_owned(), Value::from("graph run")),
            ("duration_ms".to_owned(), Value::from(42_i64)),
            ("is_tty".to_owned(), Value::from(true)),
            ("command".to_owned(), Value::from("é".repeat(128))),
        ]
    );
    let reasons: Vec<Rejection> = dropped.iter().map(|(_, r)| *r).collect();
    assert_eq!(
        reasons,
        [
            Rejection::KeyNotAllowed,
            Rejection::KeyNotAllowed,
            Rejection::LooksLikeUrl,
            Rejection::LooksLikeUrl,
            Rejection::ContainsAt,
            Rejection::ContainsPathSeparator,
            Rejection::ContainsPathSeparator,
            Rejection::TooLong,
            Rejection::NonFiniteFloat,
            Rejection::NonFiniteFloat,
        ]
    );
    assert_eq!(guard::dropped_count(), before + 10);
}

#[test]
fn guard_value_rules_do_not_apply_to_numbers_or_booleans() {
    assert_eq!(
        guard::check("duration_ms", &Value::Int(-1), CMD_ALLOWLIST),
        Ok(())
    );
    assert_eq!(
        guard::check("duration_ms", &Value::Float(1.5), CMD_ALLOWLIST),
        Ok(())
    );
    assert_eq!(
        guard::check("is_tty", &Value::Bool(false), CMD_ALLOWLIST),
        Ok(())
    );
    assert_eq!(guard::check_str(&"y".repeat(128)), Ok(()));
    assert_eq!(guard::check_str("http:not-a-url"), Ok(()));
    assert_eq!(guard::check_str("stable"), Ok(()));
    assert_eq!(
        guard::check_str("ftp://x"),
        Err(Rejection::ContainsPathSeparator),
        "non-http schemes still fail on the separator"
    );
}

#[test]
fn golden_capture_event_is_guarded_at_construction() {
    let _turn = counter_turn();
    let before = guard::dropped_count();
    let event = Event::capture(
        CMD,
        "8d1f4e6c-0b2a-4c5d-9e7f-123456789abc",
        "0191f4d0-0000-7000-8000-000000000001".into(),
        "2026-09-09T21:00:00.005Z".into(),
        vec![
            ("command".into(), Value::from("graph run")),
            ("duration_ms".into(), Value::from(42_i64)),
            ("is_tty".into(), Value::from(true)),
            ("command_path".into(), Value::from("/usr/bin/cerulion")),
            ("command".into(), Value::from("bob@example.com")),
        ],
    )
    .expect("well-formed sub passes the guard");
    assert_eq!(guard::dropped_count(), before + 2);
    let want = serde_json::json!({
        "event": "cli_command_run",
        "distinct_id": "8d1f4e6c-0b2a-4c5d-9e7f-123456789abc",
        "uuid": "0191f4d0-0000-7000-8000-000000000001",
        "timestamp": "2026-09-09T21:00:00.005Z",
        "properties": {
            "$lib": "cerulion_telemetry",
            "$lib_version": LIB_VERSION,
            "surface": "cli",
            "env": "prod",
            "app_version": "0.1.0",
            "channel": "stable",
            "command": "graph run",
            "duration_ms": 42,
            "is_tty": true
        }
    });
    assert_eq!(event_json(&event, &common()), want);
}

#[test]
fn every_identifier_that_fails_the_guard_drops_the_event_and_counts_once() {
    let _turn = counter_turn();
    let bad = [
        "",
        "bob@example.com",
        "/home/bob/.cerulion",
        "https://example.com",
        &"x".repeat(129),
        "unknown",
        "hosted-mcp",
        "cli",
        "anonymous",
        "sub",
        "anon:ok",
        "anon:",
        "8D1F4E6C-0B2A-4C5D-9E7F-123456789ABC",
        "{8d1f4e6c-0b2a-4c5d-9e7f-123456789abc}",
        "8d1f4e6c0b2a4c5d9e7f123456789abc",
        "anon:anon:6ba7b810-9dad-41d1-80b4-00c04fd430c8",
    ];
    let ok_props = || vec![("is_tty".into(), Value::from(true))];
    let once = || vec![("first_surface".into(), Value::from("cli"))];
    for id in bad {
        let before = guard::dropped_count();
        assert!(Event::capture(CMD, id, "u".into(), "t".into(), ok_props()).is_none());
        assert!(
            Event::capture_anonymous(FIRST_RUN, id, "u".into(), "t".into(), ok_props()).is_none()
        );
        assert!(Event::alias(id, ANON, "u".into(), "t".into()).is_none());
        assert!(Event::alias(SUB, id, "u".into(), "t".into()).is_none());
        assert!(Event::set_once(id, "u".into(), "t".into(), once()).is_none());
        assert_eq!(guard::dropped_count() - before, 5, "{id:?}");
        let before = guard::dropped_count();
        let leaky = vec![("email".into(), Value::from("bob@example.com"))];
        assert!(Event::set_once(id, "u".into(), "t".into(), leaky).is_none());
        assert_eq!(
            guard::dropped_count() - before,
            2,
            "bad sub counts even when no property survives: {id:?}"
        );
    }
    let before = guard::dropped_count();
    assert!(Event::capture(CMD, SUB, "u".into(), "t".into(), ok_props()).is_some());
    assert!(
        Event::capture_anonymous(FIRST_RUN, ANON, "u".into(), "t".into(), ok_props()).is_some()
    );
    assert!(Event::alias(SUB, ANON, "u".into(), "t".into()).is_some());
    assert!(Event::set_once(SUB, "u".into(), "t".into(), once()).is_some());
    assert_eq!(guard::dropped_count(), before);
}

#[test]
fn event_names_outside_the_snake_case_contract_drop_the_event_and_count_once() {
    let _turn = counter_turn();
    const LONG: &str = "a_very_long_event_name_that_keeps_going_past_the_sixty_four_byte_limit";
    let bad: &[(&str, Rejection)] = &[
        ("", Rejection::Empty),
        ("signup_alice@example.com", Rejection::NotAnEventName),
        ("https://example.com", Rejection::NotAnEventName),
        ("home/bob", Rejection::NotAnEventName),
        ("CliCommandRun", Rejection::NotAnEventName),
        ("cli-command-run", Rejection::NotAnEventName),
        ("cli command run", Rejection::NotAnEventName),
        ("_leading", Rejection::NotAnEventName),
        ("1st_run", Rejection::NotAnEventName),
        ("$capture", Rejection::NotAnEventName),
        ("café_opened", Rejection::NotAnEventName),
        (LONG, Rejection::TooLong),
    ];
    assert!(LONG.len() > guard::MAX_EVENT_NAME_LEN);
    let props = || vec![("is_tty".into(), Value::from(true))];
    for (name, why) in bad {
        let spec = EventSpec {
            name,
            allowlist: CMD_ALLOWLIST,
        };
        let before = guard::dropped_count();
        assert_eq!(guard::check_event_name(name), Err(*why), "{name:?}");
        assert!(Event::capture(spec, SUB, "u".into(), "t".into(), props()).is_none());
        assert!(Event::capture_anonymous(spec, ANON, "u".into(), "t".into(), props()).is_none());
        assert_eq!(guard::dropped_count() - before, 3, "{name:?}");
    }
    for name in ["x", "cli_command_run", "node_build_failed", "v2_step3"] {
        assert_eq!(guard::check_event_name(name), Ok(()), "{name:?}");
    }
    let spec = EventSpec {
        name: "signup_alice@example.com",
        allowlist: CMD_ALLOWLIST,
    };
    let event = Event::capture(spec, SUB, "u".into(), "t".into(), props());
    let batch = batch_json("phc_x", &event.into_iter().collect::<Vec<_>>(), &common());
    assert_eq!(batch["batch"].as_array().map(Vec::len), Some(0));
    assert!(!batch.to_string().contains("alice"));
}

#[test]
fn golden_anonymous_alias_and_set_once_events_in_one_batch() {
    let _turn = counter_turn();
    let no_channel = Common {
        channel: None,
        ..common()
    };
    let events = vec![
        Event::capture_anonymous(
            FIRST_RUN,
            "anon:6ba7b810-9dad-41d1-80b4-00c04fd430c8",
            "u1".into(),
            "t1".into(),
            vec![("is_tty".into(), Value::from(false))],
        )
        .expect("well-formed anon id passes the guard"),
        Event::alias(
            "8d1f4e6c-0b2a-4c5d-9e7f-123456789abc",
            "anon:6ba7b810-9dad-41d1-80b4-00c04fd430c8",
            "u2".into(),
            "t2".into(),
        )
        .expect("well-formed anon id passes the guard"),
        Event::set_once(
            "8d1f4e6c-0b2a-4c5d-9e7f-123456789abc",
            "u3".into(),
            "t3".into(),
            vec![
                ("first_surface".into(), Value::from("cli")),
                (
                    "first_cli_login_at".into(),
                    Value::from("2026-09-09T21:00:00.000Z"),
                ),
                ("email".into(), Value::from("bob@example.com")),
            ],
        )
        .expect("two allowed keys survive"),
    ];
    let want = serde_json::json!({
        "api_key": "phc_test",
        "batch": [
            {
                "event": "cli_first_run",
                "distinct_id": "anon:6ba7b810-9dad-41d1-80b4-00c04fd430c8",
                "uuid": "u1",
                "timestamp": "t1",
                "properties": {
                    "$lib": "cerulion_telemetry",
                    "$lib_version": LIB_VERSION,
                    "surface": "cli",
                    "env": "prod",
                    "app_version": "0.1.0",
                    "$process_person_profile": false,
                    "is_tty": false
                }
            },
            {
                "event": "$create_alias",
                "distinct_id": "8d1f4e6c-0b2a-4c5d-9e7f-123456789abc",
                "uuid": "u2",
                "timestamp": "t2",
                "properties": {
                    "$lib": "cerulion_telemetry",
                    "$lib_version": LIB_VERSION,
                    "surface": "cli",
                    "env": "prod",
                    "app_version": "0.1.0",
                    "alias": "anon:6ba7b810-9dad-41d1-80b4-00c04fd430c8"
                }
            },
            {
                "event": "$set",
                "distinct_id": "8d1f4e6c-0b2a-4c5d-9e7f-123456789abc",
                "uuid": "u3",
                "timestamp": "t3",
                "properties": {
                    "$lib": "cerulion_telemetry",
                    "$lib_version": LIB_VERSION,
                    "surface": "cli",
                    "env": "prod",
                    "app_version": "0.1.0",
                    "$set_once": {
                        "first_surface": "cli",
                        "first_cli_login_at": "2026-09-09T21:00:00.000Z"
                    }
                }
            }
        ]
    });
    assert_eq!(batch_json("phc_test", &events, &no_channel), want);
}

#[test]
fn reserved_common_keys_cannot_be_spoofed_even_when_allowlisted() {
    let _turn = counter_turn();
    const LEAKY: Allowlist = &["surface", "env", "app_version", "channel", "command"];
    const SPEC: EventSpec = EventSpec {
        name: "x",
        allowlist: LEAKY,
    };
    let before = guard::dropped_count();
    let event = Event::capture(
        SPEC,
        SUB,
        "u".into(),
        "t".into(),
        vec![
            ("surface".into(), Value::from("web")),
            ("env".into(), Value::from("dev")),
            ("app_version".into(), Value::from("9.9.9")),
            ("channel".into(), Value::from("nightly")),
            ("command".into(), Value::from("ok")),
        ],
    )
    .expect("well-formed sub passes the guard");
    assert_eq!(guard::dropped_count(), before + 4);
    let json = event_json(&event, &common());
    let p = &json["properties"];
    assert_eq!(p["surface"], "cli");
    assert_eq!(p["env"], "prod");
    assert_eq!(p["app_version"], "0.1.0");
    assert_eq!(p["channel"], "stable");
    assert_eq!(p["command"], "ok");
    for key in guard::RESERVED_KEYS {
        assert_eq!(
            guard::check(key, &Value::from("x"), LEAKY),
            Err(Rejection::KeyNotAllowed)
        );
    }
}

#[test]
fn set_once_with_nothing_allowed_builds_no_event() {
    let _turn = counter_turn();
    assert_eq!(
        Event::set_once(
            SUB,
            "u".into(),
            "t".into(),
            vec![("email".into(), Value::from("bob@example.com"))],
        ),
        None
    );
}

#[test]
fn set_once_allowlist_is_exactly_the_contract() {
    assert_eq!(
        SET_ONCE_ALLOWLIST,
        [
            "created_at",
            "first_signed_in_at",
            "first_surface",
            "first_cli_login_at",
            "first_workspace_at",
        ]
    );
}

#[test]
fn url_hash_matches_hindsight_convention() {
    // printf '%s' "$url" | sha256sum | cut -c1-12
    assert_eq!(url_hash("https://example.com/a"), "2dce0a4c5044");
    assert_eq!(url_hash(""), "e3b0c44298fc");
    assert_eq!(url_hash("abc"), "ba7816bf8f01");
}

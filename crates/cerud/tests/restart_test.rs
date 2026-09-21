// SPDX-License-Identifier: AGPL-3.0-only
//! Restart verb: command-builder oracles, graph-name validation, the
//! injected-runner dispatch path, and the unsupported-host arm.

use std::sync::Mutex;

use cerud::error::CerudError;
use cerud::verbs::restart::{
    build_start_command, build_stop_command, unit_name, validate_graph_name, RestartRunner,
    RestartVerb, UnsupportedHostRunner,
};
use cerud::verbs::VerbHandler;

#[test]
fn unit_name_oracle() {
    assert_eq!(unit_name("perception"), "cerulion-graph-perception");
}

#[test]
fn restart_is_declared_mutating() {
    // Restart has a side effect → it must declare is_mutating,
    // so the server writes a durable INTENT receipt before dispatching it.
    assert!(RestartVerb::new().is_mutating());
}

#[test]
fn command_builders_produce_the_systemd_pattern() {
    assert_eq!(
        build_stop_command("perception").unwrap(),
        vec!["systemctl", "stop", "cerulion-graph-perception"]
    );
    assert_eq!(
        build_start_command("perception").unwrap(),
        vec![
            "systemd-run",
            "--unit=cerulion-graph-perception",
            "--collect",
            "cerulion",
            "graph",
            "run",
            "perception",
        ]
    );
}

#[test]
fn validate_graph_name_accepts_reasonable_names() {
    for good in ["perception", "planning_2", "nav-stack", "g0"] {
        validate_graph_name(good).unwrap();
    }
}

#[test]
fn validate_graph_name_rejects_injection_vectors() {
    for bad in [
        "",            // empty
        "../etc",      // traversal
        "a/b",         // path separator
        "graph name",  // whitespace
        "g;rm -rf /",  // shell metachars
        "g$(whoami)",  // command substitution
        "g.service",   // dot (would confuse the unit name)
        "-rf",         // leading '-' (would be read as a flag by systemd-run)
        "--unit=evil", // leading '-' flag-injection
    ] {
        let err = validate_graph_name(bad).unwrap_err();
        assert!(matches!(err, CerudError::Verb(_)), "{bad:?} -> {err:?}");
    }
    // A HYPHEN mid-name is fine; only a LEADING hyphen is rejected.
    validate_graph_name("nav-stack").unwrap();
    // Over-length is rejected.
    assert!(validate_graph_name(&"g".repeat(200)).is_err());
}

/// A runner that records every argv it is handed (supported=true).
struct RecordingRunner {
    calls: Mutex<Vec<Vec<String>>>,
}

impl RestartRunner for RecordingRunner {
    fn supported(&self) -> bool {
        true
    }
    fn run(&self, argv: &[String]) -> cerud::error::CerudResult<()> {
        self.calls.lock().unwrap().push(argv.to_vec());
        Ok(())
    }
}

#[test]
fn restart_dispatches_stop_then_start_through_the_runner() {
    // The runner is shared so we can inspect the captured argv after execute.
    struct Shared(std::sync::Arc<RecordingRunner>);
    impl RestartRunner for Shared {
        fn supported(&self) -> bool {
            self.0.supported()
        }
        fn run(&self, argv: &[String]) -> cerud::error::CerudResult<()> {
            self.0.run(argv)
        }
    }
    let inner = std::sync::Arc::new(RecordingRunner {
        calls: Mutex::new(Vec::new()),
    });
    let verb = RestartVerb::with_runner(Box::new(Shared(inner.clone())));

    let result = verb
        .execute(&serde_json::json!({"graph": "perception"}))
        .unwrap();
    assert_eq!(result["graph"], "perception");
    assert_eq!(result["unit"], "cerulion-graph-perception");
    assert_eq!(result["restarted"], true);

    let calls = inner.calls.lock().unwrap();
    assert_eq!(calls.len(), 2, "expected a stop then a start");
    assert_eq!(calls[0], build_stop_command("perception").unwrap());
    assert_eq!(calls[1], build_start_command("perception").unwrap());
}

#[test]
fn restart_rejects_bad_graph_name_before_running_anything() {
    struct Shared(std::sync::Arc<RecordingRunner>);
    impl RestartRunner for Shared {
        fn supported(&self) -> bool {
            self.0.supported()
        }
        fn run(&self, argv: &[String]) -> cerud::error::CerudResult<()> {
            self.0.run(argv)
        }
    }
    let inner = std::sync::Arc::new(RecordingRunner {
        calls: Mutex::new(Vec::new()),
    });
    let verb = RestartVerb::with_runner(Box::new(Shared(inner.clone())));

    let err = verb
        .execute(&serde_json::json!({"graph": "../../etc"}))
        .unwrap_err();
    assert!(matches!(err, CerudError::Verb(_)), "got {err:?}");
    // Nothing was run — validation happens before the runner is touched.
    assert!(inner.calls.lock().unwrap().is_empty());
}

/// A runner that fails the START command (but tolerates the best-effort stop).
struct FailingStartRunner;
impl RestartRunner for FailingStartRunner {
    fn supported(&self) -> bool {
        true
    }
    fn run(&self, argv: &[String]) -> cerud::error::CerudResult<()> {
        // The stop is best-effort (ignored by the verb); the start must fail.
        if argv.first().map(|s| s.as_str()) == Some("systemd-run") {
            return Err(CerudError::Verb(
                "systemd-run failed: unit start error".to_string(),
            ));
        }
        Ok(())
    }
}

#[test]
fn restart_surfaces_a_start_command_failure() {
    let verb = RestartVerb::with_runner(Box::new(FailingStartRunner));
    let err = verb
        .execute(&serde_json::json!({"graph": "perception"}))
        .unwrap_err();
    match err {
        CerudError::Verb(msg) => assert!(msg.contains("systemd-run failed")),
        other => panic!("expected the start-failure Verb error, got {other:?}"),
    }
}

#[test]
fn restart_on_unsupported_host_is_a_structured_error_not_a_panic() {
    let verb = RestartVerb::with_runner(Box::new(UnsupportedHostRunner));
    let err = verb
        .execute(&serde_json::json!({"graph": "perception"}))
        .unwrap_err();
    match err {
        CerudError::UnsupportedHost(msg) => assert!(msg.contains("Linux")),
        other => panic!("expected UnsupportedHost, got {other:?}"),
    }
}

#[test]
fn restart_missing_graph_arg_is_a_verb_error() {
    let verb = RestartVerb::with_runner(Box::new(UnsupportedHostRunner));
    let err = verb.execute(&serde_json::json!({})).unwrap_err();
    assert!(matches!(err, CerudError::Verb(_)), "got {err:?}");
}

// SPDX-License-Identifier: AGPL-3.0-only

use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Duration;

use cerulion_ops_proto::{ExecRequest, ShellRequest, WindowSize};

use super::*;

fn context(calls: &Arc<AtomicUsize>, parent: &std::path::Path) -> SessionContext {
    let auth_calls = Arc::clone(calls);
    let receipt_calls = Arc::clone(calls);
    SessionContext {
        session: SessionKind::Exec(ExecRequest {
            argv: vec!["echo".into(), "literal argument".into()],
        }),
        caller: CallerIdentity::verified("verified-peer"),
        account_id: Some("account".into()),
        recording_dir: parent.join("sessions"),
        clock: RemotedClock::fixed(123),
        authorize: Arc::new(move || {
            auth_calls.fetch_add(1, Ordering::SeqCst);
            Ok(())
        }),
        receipt: Arc::new(move |_, _| {
            receipt_calls.fetch_add(1, Ordering::SeqCst);
            Ok(())
        }),
        limits: SessionLimits::default(),
    }
}

fn shell() -> SessionKind {
    SessionKind::Shell(ShellRequest {
        term: "xterm-256color".into(),
        size: WindowSize {
            rows: 24,
            cols: 80,
            pixel_width: 0,
            pixel_height: 0,
        },
    })
}

#[test]
fn resource_defaults_match_the_declared_operator_profile() {
    // Independent oracle for the public resource defaults, not execution results.
    for limits in [SessionLimits::default(), SessionLimits::default()] {
        assert_eq!(limits.reauthorize_interval, Duration::from_secs(1));
        assert_eq!(limits.io_timeout, Duration::from_secs(2));
        assert_eq!(limits.max_recording_bytes, 67_108_864);
        assert_eq!(limits.max_stdin_bytes, 1_048_576);
        assert_eq!(limits.max_stdout_bytes, 1_048_576);
        assert_eq!(limits.max_stderr_bytes, 1_048_576);
        assert_eq!(limits.exec_timeout, Some(Duration::from_secs(60)));
        assert_eq!(limits.shell_timeout, Some(Duration::from_secs(28_800)));
        assert_eq!(limits.shell_idle_timeout, Some(Duration::from_secs(1_800)));
    }
}

#[test]
fn valid_exec_and_shell_validation_never_invokes_callbacks_or_rewrites_limits() {
    let directory = tempfile::tempdir().unwrap();
    let calls = Arc::new(AtomicUsize::new(0));
    let mut value = context(&calls, directory.path());
    value.limits.max_recording_bytes = 0;
    value.limits.max_stdin_bytes = 7;
    value.limits.max_stdout_bytes = 9;
    value.limits.max_stderr_bytes = 11;
    value.limits.exec_timeout = None;
    value.limits.shell_timeout = None;
    value.limits.shell_idle_timeout = None;
    value.validate().unwrap();
    value.session = shell();
    value.validate().unwrap();
    assert_eq!(calls.load(Ordering::SeqCst), 0);
    assert_eq!(std::fs::read_dir(directory.path()).unwrap().count(), 0);
    assert_eq!(value.limits.max_recording_bytes, 0);
    assert_eq!(
        (
            value.limits.max_stdin_bytes,
            value.limits.max_stdout_bytes,
            value.limits.max_stderr_bytes
        ),
        (7, 9, 11)
    );
    assert_eq!(
        (
            value.limits.exec_timeout,
            value.limits.shell_timeout,
            value.limits.shell_idle_timeout
        ),
        (None, None, None)
    );
}

#[test]
fn configured_limits_enter_the_context_directly_and_remain_unchanged() {
    let directory = tempfile::tempdir().unwrap();
    let calls = Arc::new(AtomicUsize::new(0));
    let mut configured = crate::config::RemotedConfig::from_state_root(
        directory.path(),
        cerulion_link::RelayConfig::Disabled,
        false,
    );
    configured.remote_shell_limits = crate::remote_session::SessionLimits {
        reauthorize_interval: Duration::from_millis(333),
        io_timeout: Duration::from_secs(7),
        max_recording_bytes: 2039,
        max_stdin_bytes: 3,
        max_stdout_bytes: 5,
        max_stderr_bytes: 7,
        exec_timeout: Some(Duration::from_secs(19)),
        shell_timeout: Some(Duration::from_secs(23)),
        shell_idle_timeout: Some(Duration::from_secs(29)),
    };
    let mut value = context(&calls, directory.path());
    // This assignment also checks the public types are identical, without a
    // conversion that could silently replace configured values with defaults.
    value.limits = configured.remote_shell_limits;
    for session in [value.session.clone(), shell()] {
        value.session = session;
        value.validate().unwrap();
        assert_eq!(
            value.limits.reauthorize_interval,
            Duration::from_millis(333)
        );
        assert_eq!(value.limits.io_timeout, Duration::from_secs(7));
        assert_eq!(value.limits.max_recording_bytes, 2039);
        assert_eq!(value.limits.max_stdin_bytes, 3);
        assert_eq!(value.limits.max_stdout_bytes, 5);
        assert_eq!(value.limits.max_stderr_bytes, 7);
        assert_eq!(value.limits.exec_timeout, Some(Duration::from_secs(19)));
        assert_eq!(value.limits.shell_timeout, Some(Duration::from_secs(23)));
        assert_eq!(
            value.limits.shell_idle_timeout,
            Some(Duration::from_secs(29))
        );
    }
    assert_eq!(calls.load(Ordering::SeqCst), 0);
    assert_eq!(std::fs::read_dir(directory.path()).unwrap().count(), 0);
}

#[test]
fn unauthenticated_and_ordinary_ops_contexts_refuse_before_callbacks() {
    let directory = tempfile::tempdir().unwrap();
    let calls = Arc::new(AtomicUsize::new(0));
    let mut value = context(&calls, directory.path());
    value.caller.authenticated = false;
    let error = value.validate().unwrap_err();
    assert_eq!(error.kind(), io::ErrorKind::InvalidInput);
    assert_eq!(
        error.to_string(),
        "invalid authenticated session configuration"
    );
    value.caller.authenticated = true;
    value.session = SessionKind::Ops;
    assert_eq!(
        value.validate().unwrap_err().kind(),
        io::ErrorKind::InvalidInput
    );
    assert_eq!(calls.load(Ordering::SeqCst), 0);
    assert_eq!(std::fs::read_dir(directory.path()).unwrap().count(), 0);
}

#[test]
fn timer_boundaries_are_exact_and_refusals_do_not_mutate_configuration() {
    let directory = tempfile::tempdir().unwrap();
    let calls = Arc::new(AtomicUsize::new(0));
    for (interval, expected_ok) in [
        (Duration::ZERO, false),
        (Duration::from_nanos(1), true),
        (Duration::from_secs(1), true),
        (Duration::from_secs(1) + Duration::from_nanos(1), false),
    ] {
        let mut value = context(&calls, directory.path());
        value.limits.reauthorize_interval = interval;
        assert_eq!(value.validate().is_ok(), expected_ok);
        assert_eq!(value.limits.reauthorize_interval, interval);
    }
    for (timeout, expected_ok) in [(Duration::ZERO, false), (Duration::from_nanos(1), true)] {
        let mut value = context(&calls, directory.path());
        value.limits.io_timeout = timeout;
        assert_eq!(value.validate().is_ok(), expected_ok);
        assert_eq!(value.limits.io_timeout, timeout);
    }
    assert_eq!(calls.load(Ordering::SeqCst), 0);
    assert_eq!(std::fs::read_dir(directory.path()).unwrap().count(), 0);
}

#[test]
fn malformed_terminal_and_exec_parameters_are_refused_before_callbacks() {
    let directory = tempfile::tempdir().unwrap();
    let calls = Arc::new(AtomicUsize::new(0));
    let mut invalid_size = shell();
    if let SessionKind::Shell(request) = &mut invalid_size {
        request.size.rows = 0;
    }
    let mut invalid_term = shell();
    if let SessionKind::Shell(request) = &mut invalid_term {
        request.term = "bad\nterm".into();
    }
    for session in [
        SessionKind::Exec(ExecRequest { argv: vec![] }),
        SessionKind::Exec(ExecRequest {
            argv: vec![String::new()],
        }),
        SessionKind::Exec(ExecRequest {
            argv: vec!["echo".into(), "bad\0arg".into()],
        }),
        invalid_size,
        invalid_term,
    ] {
        let mut value = context(&calls, directory.path());
        value.session = session;
        assert!(value.validate().is_err());
    }
    assert_eq!(calls.load(Ordering::SeqCst), 0);
    assert_eq!(std::fs::read_dir(directory.path()).unwrap().count(), 0);
}

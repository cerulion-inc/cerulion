// SPDX-License-Identifier: AGPL-3.0-only

use std::ffi::OsStr;
use std::time::Duration;

use cerulion_link::RelayConfig;
use clap::{error::ErrorKind, Parser};

use super::{hard_off, ShellFlags};
use crate::RemotedConfig;

#[derive(Debug, Parser)]
struct Options {
    #[command(flatten)]
    shell: ShellFlags,
}

fn config() -> RemotedConfig {
    // Configuration construction derives paths; these tests open no state.
    RemotedConfig::from_state_root("shell-options-test-state", RelayConfig::Disabled, true)
}

#[test]
fn defaults_replace_existing_settings_with_literal_resource_bounds() {
    for _ in 0..2 {
        let flags = Options::try_parse_from(["settings"]).unwrap().shell;
        let mut config = config();
        config.remote_shell_off = true;
        config.remote_shell_max_sessions = 97;
        config.remote_shell_max_per_device = 97;
        config.remote_shell_limits.io_timeout = Duration::from_secs(97);
        config.remote_shell_limits.max_recording_bytes = 97;
        config.remote_shell_limits.max_stdin_bytes = 97;
        config.remote_shell_limits.max_stdout_bytes = 97;
        config.remote_shell_limits.max_stderr_bytes = 97;
        config.remote_shell_limits.exec_timeout = None;
        config.remote_shell_limits.shell_timeout = None;
        config.remote_shell_limits.shell_idle_timeout = None;

        flags.apply(&mut config, None).unwrap();

        assert!(!config.remote_shell_off);
        assert_eq!(config.remote_shell_max_sessions, 4);
        assert_eq!(config.remote_shell_max_per_device, 2);
        let limits = config.remote_shell_limits;
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
fn configured_io_timeout_replaces_the_supplied_value() {
    for (argument, seconds) in [("1", 1), ("5", 5)] {
        let flags =
            Options::try_parse_from(["settings", "--remote-shell-io-timeout-secs", argument])
                .unwrap()
                .shell;
        let mut config = config();
        config.remote_shell_limits.io_timeout = Duration::from_secs(97);
        flags.apply(&mut config, None).unwrap();
        assert_eq!(
            config.remote_shell_limits.io_timeout,
            Duration::from_secs(seconds)
        );
    }
}

#[test]
fn invalid_io_timeout_is_refused_with_the_flag_name() {
    for invalid in ["0", "not-a-number"] {
        let error =
            Options::try_parse_from(["settings", "--remote-shell-io-timeout-secs", invalid])
                .unwrap_err();
        assert_eq!(error.kind(), ErrorKind::ValueValidation);
        let message = error.to_string();
        assert!(message.contains("--remote-shell-io-timeout-secs"));
        assert!(message.contains(invalid));
    }
}

#[test]
fn each_configured_limit_reaches_its_distinct_field() {
    let flags = Options::try_parse_from([
        "settings",
        "--remote-shell",
        "on",
        "--remote-shell-max-sessions",
        "7",
        "--remote-shell-max-per-device",
        "5",
        "--remote-shell-recording-bytes",
        "10000000",
        "--remote-exec-stdin-bytes",
        "123",
        "--remote-exec-stdout-bytes",
        "456",
        "--remote-exec-stderr-bytes",
        "789",
        "--remote-exec-timeout-secs",
        "120",
        "--remote-shell-timeout-secs",
        "300",
        "--remote-shell-idle-secs",
        "60",
    ])
    .unwrap()
    .shell;
    let mut config = config();
    flags.apply(&mut config, Some(OsStr::new("off"))).unwrap();
    assert!(config.remote_shell_off);
    assert_eq!(config.remote_shell_max_sessions, 7);
    assert_eq!(config.remote_shell_max_per_device, 5);
    let limits = config.remote_shell_limits;
    assert_eq!(limits.max_recording_bytes, 10_000_000);
    assert_eq!(limits.max_stdin_bytes, 123);
    assert_eq!(limits.max_stdout_bytes, 456);
    assert_eq!(limits.max_stderr_bytes, 789);
    assert_eq!(limits.exec_timeout, Some(Duration::from_secs(120)));
    assert_eq!(limits.shell_timeout, Some(Duration::from_secs(300)));
    assert_eq!(limits.shell_idle_timeout, Some(Duration::from_secs(60)));
}

#[test]
fn zero_disables_optional_timeouts_and_allows_empty_exec_stream_budgets() {
    let flags = Options::try_parse_from([
        "settings",
        "--remote-exec-timeout-secs",
        "0",
        "--remote-shell-timeout-secs",
        "0",
        "--remote-shell-idle-secs",
        "0",
        "--remote-exec-stdin-bytes",
        "0",
        "--remote-exec-stdout-bytes",
        "0",
        "--remote-exec-stderr-bytes",
        "0",
    ])
    .unwrap()
    .shell;
    let mut config = config();
    flags.apply(&mut config, None).unwrap();
    let limits = config.remote_shell_limits;
    assert_eq!(limits.exec_timeout, None);
    assert_eq!(limits.shell_timeout, None);
    assert_eq!(limits.shell_idle_timeout, None);
    assert_eq!(limits.max_stdin_bytes, 0);
    assert_eq!(limits.max_stdout_bytes, 0);
    assert_eq!(limits.max_stderr_bytes, 0);
    assert_eq!(limits.io_timeout, Duration::from_secs(2));
}

#[test]
fn invalid_concurrency_or_recording_bound_leaves_config_unchanged() {
    for flag in [
        "--remote-shell-max-sessions",
        "--remote-shell-max-per-device",
        "--remote-shell-recording-bytes",
    ] {
        let options = Options::try_parse_from(["settings", flag, "0"]).unwrap();
        let mut config = config();
        config.remote_shell_off = true;
        config.remote_shell_max_sessions = 7;
        config.remote_shell_limits.max_recording_bytes = 99;
        assert!(options.shell.apply(&mut config, None).is_err());
        assert!(config.remote_shell_off);
        assert_eq!(config.remote_shell_max_sessions, 7);
        assert_eq!(config.remote_shell_limits.max_recording_bytes, 99);
    }
}

#[test]
fn either_off_wins_and_invalid_environment_text_refuses_configuration() {
    for (flag, environment, off) in [
        ("on", "off", true),
        ("off", "on", true),
        ("on", " OFF ", true),
        ("on", "", false),
        ("on", "on", false),
    ] {
        let options = Options::try_parse_from(["settings", "--remote-shell", flag]).unwrap();
        let mut config = config();
        options
            .shell
            .apply(&mut config, Some(OsStr::new(environment)))
            .unwrap();
        assert_eq!(config.remote_shell_off, off);
    }
    for environment in ["of", "false"] {
        let options = Options::try_parse_from(["settings", "--remote-shell", "off"]).unwrap();
        let mut config = config();
        config.remote_shell_max_sessions = 7;
        assert!(options
            .shell
            .apply(&mut config, Some(OsStr::new(environment)))
            .is_err());
        assert!(!config.remote_shell_off);
        assert_eq!(config.remote_shell_max_sessions, 7);
    }
    assert!(hard_off(Some("of"), None).is_err());
    assert!(!hard_off(None, None).unwrap());
}

#[cfg(unix)]
#[test]
fn non_unicode_environment_is_refused_even_with_an_explicit_off_flag() {
    use std::os::unix::ffi::OsStrExt;
    let options = Options::try_parse_from(["settings", "--remote-shell", "off"]).unwrap();
    let mut config = config();
    config.remote_shell_max_sessions = 7;
    assert!(options
        .shell
        .apply(&mut config, Some(OsStr::from_bytes(&[0xff])))
        .is_err());
    assert!(!config.remote_shell_off);
    assert_eq!(config.remote_shell_max_sessions, 7);
}

#[test]
fn unknown_or_overflowing_flag_values_are_refused_by_the_parser() {
    assert!(Options::try_parse_from(["settings", "--remote-shell", "of"]).is_err());
    assert!(Options::try_parse_from([
        "settings",
        "--remote-exec-stdin-bytes",
        "18446744073709551616",
    ])
    .is_err());
    assert!(Options::try_parse_from(["settings", "--remote-shell-max-sessions", "-1"]).is_err());
}

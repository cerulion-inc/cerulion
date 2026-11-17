// SPDX-License-Identifier: AGPL-3.0-only
//! The delivery-log flags of `cerulion ros2 run` / `cerulion ros2 launch`:
//! `--record-deliveries <DIR>` and `--replay-deliveries <LOG>`, leading
//! Cerulion flags staged as `CERULION_RMW_DELIVERY_LOG` / `CERULION_RMW_REPLAY`
//! for the child. Pinned: the split (both spellings, the forwarded remainder
//! verbatim, only LEADING tokens), every refusal and its exit code, the staged
//! env pair with an absolute path, the ambient-variable conflict, the
//! `--adopt-take` refusal still answering after a delivery flag, and the
//! operator notice.

#![cfg(unix)]

use std::path::{Path, PathBuf};

use cerulion_cli_engine::error::CliError;
use cerulion_cli_engine::ros2_cmd::{
    build_ros2_passthrough_plan_for_host, classify, delivery_child_env, delivery_notice,
    split_delivery_flags, DeliveryRequest, HostAbi, Ros2NativeVerb, DELIVERY_LOG_CHILD_ENV,
    EXIT_UNAVAILABLE, EXIT_USAGE, REPLAY_CHILD_ENV, RMW_LIB_FILENAME, ROS2_PRELOAD_ENV,
};
use serial_test::serial;

fn args(list: &[&str]) -> Vec<std::ffi::OsString> {
    list.iter().map(std::ffi::OsString::from).collect()
}

fn env_value<'a, V: AsRef<std::ffi::OsStr>>(env: &'a [(String, V)], key: &str) -> Option<&'a str> {
    env.iter()
        .find(|(k, _)| k == key)
        .map(|(_, v)| v.as_ref().to_str().expect("a test-staged value is UTF-8"))
}

/// Panic-safe env override.
struct EnvVarGuard {
    key: &'static str,
    prior: Option<std::ffi::OsString>,
}

impl EnvVarGuard {
    fn set(key: &'static str, value: &str) -> Self {
        let prior = std::env::var_os(key);
        std::env::set_var(key, value);
        Self { key, prior }
    }
    fn unset(key: &'static str) -> Self {
        let prior = std::env::var_os(key);
        std::env::remove_var(key);
        Self { key, prior }
    }
}

impl Drop for EnvVarGuard {
    fn drop(&mut self) {
        match &self.prior {
            Some(v) => std::env::set_var(self.key, v),
            None => std::env::remove_var(self.key),
        }
    }
}

struct Fixture {
    root: tempfile::TempDir,
    lib_dir: PathBuf,
    ament_prefix: PathBuf,
}

fn fixture() -> Fixture {
    let root = tempfile::tempdir().expect("tempdir");
    let lib_dir = root.path().join("libs");
    std::fs::create_dir_all(&lib_dir).unwrap();
    std::fs::write(lib_dir.join(RMW_LIB_FILENAME), b"not a real so").unwrap();
    let ament_prefix = root.path().join("prefix");
    std::fs::create_dir_all(&ament_prefix).unwrap();
    Fixture {
        root,
        lib_dir,
        ament_prefix,
    }
}

fn usage_err(e: CliError) -> String {
    assert_eq!(
        classify(&e),
        EXIT_USAGE,
        "a delivery-flag refusal is usage (2): {e}"
    );
    e.to_string()
}

#[test]
fn leading_flags_are_split_in_both_spellings_and_the_rest_is_verbatim() {
    let (req, rest) = split_delivery_flags(
        Ros2NativeVerb::Run,
        &args(&[
            "--record-deliveries",
            "logs",
            "pkg",
            "exe",
            "--record-deliveries",
            "x",
        ]),
    )
    .unwrap();
    assert_eq!(req, Some(DeliveryRequest::Record("logs".into())));
    assert_eq!(
        rest,
        args(&["pkg", "exe", "--record-deliveries", "x"]),
        "only LEADING tokens"
    );
    let (req, rest) = split_delivery_flags(
        Ros2NativeVerb::Run,
        &args(&[
            "--replay-deliveries=/d/a.rdl.mcap",
            "pkg",
            "exe",
            "--ros-args",
        ]),
    )
    .unwrap();
    assert_eq!(req, Some(DeliveryRequest::Replay("/d/a.rdl.mcap".into())));
    assert_eq!(rest, args(&["pkg", "exe", "--ros-args"]));
    let (req, rest) =
        split_delivery_flags(Ros2NativeVerb::Launch, &args(&["f.launch.py"])).unwrap();
    assert_eq!(req, None);
    assert_eq!(rest, args(&["f.launch.py"]));
    let (req, _) = split_delivery_flags(
        Ros2NativeVerb::Launch,
        &args(&["--record-deliveries", "d", "f.launch.py"]),
    )
    .unwrap();
    assert_eq!(
        req,
        Some(DeliveryRequest::Record("d".into())),
        "launch records"
    );
}

#[test]
fn every_malformed_use_is_refused_as_usage() {
    for bad in [
        &["--record-deliveries"][..],
        &["--record-deliveries="],
        &["--record-deliveries", "--ros-args"],
        &["--replay-deliveries", "-x"],
    ] {
        let e = split_delivery_flags(Ros2NativeVerb::Run, &args(bad)).unwrap_err();
        assert!(usage_err(e).contains("needs a value"), "{bad:?}");
    }
    let e = split_delivery_flags(
        Ros2NativeVerb::Run,
        &args(&[
            "--record-deliveries",
            "a",
            "--replay-deliveries",
            "b",
            "pkg",
        ]),
    )
    .unwrap_err();
    assert!(usage_err(e).contains("exclude each other"));
    let e = split_delivery_flags(
        Ros2NativeVerb::Run,
        &args(&["--record-deliveries", "a", "--record-deliveries", "b"]),
    )
    .unwrap_err();
    assert!(usage_err(e).contains("exclude each other"));
    let e = split_delivery_flags(
        Ros2NativeVerb::Launch,
        &args(&["--replay-deliveries", "x.rdl.mcap", "f.launch.py"]),
    )
    .unwrap_err();
    let msg = usage_err(e);
    assert!(
        msg.contains("ONE process") && msg.contains("cerulion ros2 run"),
        "{msg}"
    );
}

#[test]
#[serial]
fn record_stages_an_absolute_directory_for_the_child() {
    let f = fixture();
    let _p = EnvVarGuard::unset(ROS2_PRELOAD_ENV);
    let _r = EnvVarGuard::unset(REPLAY_CHILD_ENV);
    let plan = build_ros2_passthrough_plan_for_host(
        Ros2NativeVerb::Run,
        &args(&[
            "--record-deliveries",
            "rel/logs",
            "demo_nodes_cpp",
            "talker",
        ]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect("plan builds");
    assert_eq!(
        plan.args,
        args(&["run", "demo_nodes_cpp", "talker"]),
        "flag not forwarded"
    );
    let dir = env_value(&plan.env, DELIVERY_LOG_CHILD_ENV).expect("staged");
    assert!(Path::new(dir).is_absolute(), "{dir}");
    assert!(dir.ends_with("rel/logs"), "{dir}");
    assert_eq!(env_value(&plan.env, REPLAY_CHILD_ENV), None);
    let notice = delivery_notice(&plan).expect("a notice");
    assert!(
        notice.contains(dir) && notice.contains(".rdl.mcap"),
        "{notice}"
    );
}

#[test]
#[serial]
fn replay_stages_an_existing_log_and_refuses_a_missing_one() {
    let f = fixture();
    let _p = EnvVarGuard::unset(ROS2_PRELOAD_ENV);
    let _d = EnvVarGuard::unset(DELIVERY_LOG_CHILD_ENV);
    let log = f.root.path().join("talker-1.rdl.mcap");
    std::fs::write(&log, b"x").unwrap();
    let log_s = log.display().to_string();
    let plan = build_ros2_passthrough_plan_for_host(
        Ros2NativeVerb::Run,
        &args(&["--replay-deliveries", &log_s, "demo_nodes_cpp", "talker"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect("plan builds");
    assert_eq!(env_value(&plan.env, REPLAY_CHILD_ENV), Some(log_s.as_str()));
    let notice = delivery_notice(&plan).unwrap();
    assert!(
        notice.contains(&format!("{log_s}.verdict.json")),
        "{notice}"
    );

    let missing = f.root.path().join("absent.rdl.mcap").display().to_string();
    let e = build_ros2_passthrough_plan_for_host(
        Ros2NativeVerb::Run,
        &args(&["--replay-deliveries", &missing, "pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .unwrap_err();
    assert!(usage_err(e).contains("is not a file"));
}

#[test]
#[serial]
fn an_ambient_value_of_the_other_variable_is_refused_before_anything_starts() {
    let f = fixture();
    let log = f.root.path().join("a.rdl.mcap");
    std::fs::write(&log, b"x").unwrap();
    {
        let _r = EnvVarGuard::set(REPLAY_CHILD_ENV, "/some/log");
        let e = delivery_child_env(&DeliveryRequest::Record("d".into())).unwrap_err();
        let msg = usage_err(e);
        assert!(msg.contains(REPLAY_CHILD_ENV), "{msg}");
    }
    {
        let _d = EnvVarGuard::set(DELIVERY_LOG_CHILD_ENV, "/some/dir");
        let e = delivery_child_env(&DeliveryRequest::Replay(log.clone())).unwrap_err();
        let msg = usage_err(e);
        assert!(msg.contains(DELIVERY_LOG_CHILD_ENV), "{msg}");
    }
    let _r = EnvVarGuard::unset(REPLAY_CHILD_ENV);
    let _d = EnvVarGuard::unset(DELIVERY_LOG_CHILD_ENV);
    assert!(delivery_child_env(&DeliveryRequest::Replay(log)).is_ok());
}

#[test]
#[serial]
fn adopt_take_after_a_delivery_flag_is_still_refused() {
    let f = fixture();
    let _p = EnvVarGuard::unset(ROS2_PRELOAD_ENV);
    let e = build_ros2_passthrough_plan_for_host(
        Ros2NativeVerb::Run,
        &args(&["--record-deliveries", "d", "--adopt-take", "pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .unwrap_err();
    assert_eq!(classify(&e), EXIT_UNAVAILABLE, "{e}");
}

/// `OsString`s from raw bytes, for arguments that are not valid UTF-8.
fn raw_args(list: &[&[u8]]) -> Vec<std::ffi::OsString> {
    use std::os::unix::ffi::OsStringExt as _;
    list.iter()
        .map(|a| std::ffi::OsString::from_vec(a.to_vec()))
        .collect()
}

/// A path that is not valid UTF-8 is a path: both spellings keep its bytes,
/// and every forwarded token keeps its own.
#[test]
fn a_path_that_is_not_utf8_keeps_its_bytes_in_both_spellings() {
    for (spelled, verb) in [
        (
            raw_args(&[b"--record-deliveries", b"logs/caf\xE9", b"pkg", b"exe\xFF"]),
            Ros2NativeVerb::Launch,
        ),
        (
            raw_args(&[b"--record-deliveries=logs/caf\xE9", b"pkg", b"exe\xFF"]),
            Ros2NativeVerb::Run,
        ),
    ] {
        let (req, rest) = split_delivery_flags(verb, &spelled).unwrap();
        let dir = raw_args(&[b"logs/caf\xE9"]).remove(0);
        assert_eq!(
            req,
            Some(DeliveryRequest::Record(dir.into())),
            "{spelled:?}"
        );
        assert_eq!(rest, raw_args(&[b"pkg", b"exe\xFF"]), "{spelled:?}");
    }
    let (req, _) = split_delivery_flags(
        Ros2NativeVerb::Run,
        &raw_args(&[b"--replay-deliveries=\xFE.rdl.mcap", b"pkg"]),
    )
    .unwrap();
    assert_eq!(
        req,
        Some(DeliveryRequest::Replay(
            raw_args(&[b"\xFE.rdl.mcap"]).remove(0).into()
        ))
    );
}

/// The edges of the byte match: a value that starts with `-` is still the next
/// flag, and a token that only starts like a flag is forwarded.
#[test]
fn a_value_or_token_that_is_not_utf8_meets_the_same_rules_as_text() {
    let e = split_delivery_flags(
        Ros2NativeVerb::Run,
        &raw_args(&[b"--record-deliveries", b"-\xFF", b"pkg"]),
    )
    .unwrap_err();
    assert!(usage_err(e).contains("needs a value"));
    let spelled = raw_args(&[b"--record-deliveries\xFF=d", b"pkg"]);
    let (req, rest) = split_delivery_flags(Ros2NativeVerb::Run, &spelled).unwrap();
    assert_eq!(req, None);
    assert_eq!(rest, spelled, "not a delivery flag, so forwarded whole");
}

/// The plan forwards an argument that is not UTF-8 to `ros2` byte for byte and
/// stages the record directory for the child with its bytes, made absolute.
#[test]
#[serial]
fn the_plan_forwards_and_stages_bytes_that_are_not_utf8_as_given() {
    let f = fixture();
    let _p = EnvVarGuard::unset(ROS2_PRELOAD_ENV);
    let _r = EnvVarGuard::unset(REPLAY_CHILD_ENV);
    let dir = f.root.path().join(raw_args(&[b"caf\xE9"]).remove(0));
    let mut spelled = raw_args(&[b"--record-deliveries"]);
    spelled.push(dir.clone().into_os_string());
    spelled.extend(raw_args(&[
        b"demo_nodes_cpp",
        b"talker",
        b"--ros-args",
        b"\xFF",
    ]));
    let plan = build_ros2_passthrough_plan_for_host(
        Ros2NativeVerb::Run,
        &spelled,
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect("plan builds");
    assert_eq!(
        plan.args,
        raw_args(&[b"run", b"demo_nodes_cpp", b"talker", b"--ros-args", b"\xFF"])
    );
    let staged = plan
        .env
        .iter()
        .find(|(k, _)| k == DELIVERY_LOG_CHILD_ENV)
        .map(|(_, v)| v.clone());
    assert_eq!(staged, Some(dir.into_os_string()));
}

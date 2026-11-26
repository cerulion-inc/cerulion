// SPDX-License-Identifier: AGPL-3.0-only
//! Real-binary offline acceptance for a machine that has never signed in:
//! `cerulion graph validate` and `cerulion graph run` (with the permissive
//! default, no `--network off`) succeed on a local workspace graph without an
//! account session, degrade network serving to local-only with one loud
//! warning, contact no `cerulion-netd`, and write no `auth.json`. Account
//! verbs refuse with exit 7 and name the Cerulion account service.
//!
//! Prerequisite: `cargo build -p test_node_macro_period_cdylib`.

#![cfg(unix)]

use std::io::Read as _;
use std::os::unix::net::UnixListener;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use cerulion_cli_engine::login_cmd::EXIT_AUTH_REQUIRED;
use serial_test::serial;

mod mp_support;
use mp_support::ChildGuard;

fn send_signal(pid: u32, sig: libc::c_int) {
    // SAFETY: kill(2) with a valid child pid and signal; no memory is touched.
    unsafe {
        libc::kill(pid as libc::pid_t, sig);
    }
}

fn dylib_file(name: &str) -> String {
    if cfg!(target_os = "macos") {
        format!("lib{name}.dylib")
    } else {
        format!("lib{name}.so")
    }
}

fn fixture_cdylib() -> PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_macro_period_cdylib")
}

fn build_workspace(root: &Path, prefix: &str) {
    std::fs::create_dir_all(root.join("graphs")).unwrap();
    std::fs::create_dir_all(root.join("nodes/ticker/src")).unwrap();
    std::fs::create_dir_all(root.join("target/debug")).unwrap();
    std::fs::write(
        root.join("Cargo.toml"),
        "[workspace]\nresolver = \"2\"\nmembers = []\n",
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
    let graph_yaml = format!(
        "name: demo\nprefix: {prefix}\nnodes:\n  - id: ticker\n    type: ticker\n    inputs: []\n    outputs:\n      - name: cmd\n        schema: geometry_msgs/Vector3\n"
    );
    std::fs::write(root.join("graphs/demo.yaml"), graph_yaml).unwrap();
}

fn wait_for_log_line(path: &Path, needle: &str, timeout: Duration) -> String {
    let start = Instant::now();
    loop {
        let content = std::fs::read_to_string(path).unwrap_or_default();
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

fn read_file(p: &Path) -> String {
    let mut s = String::new();
    if let Ok(mut f) = std::fs::File::open(p) {
        let _ = f.read_to_string(&mut s);
    }
    s
}

#[test]
#[serial]
fn graph_validate_and_graph_run_succeed_without_sign_in() {
    let tmp = tempfile::tempdir().unwrap();
    let home = tempfile::tempdir().unwrap();
    let prefix = format!("offline{}", std::process::id());
    build_workspace(tmp.path(), &prefix);

    let netd_sock = tmp.path().join("netd.sock");
    let netd_listener = UnixListener::bind(&netd_sock).expect("bind stand-in netd socket");
    netd_listener.set_nonblocking(true).unwrap();
    let missing_netd = tmp.path().join("absent-netd");

    // 1. `cerulion graph validate demo` succeeds on a never-signed-in machine.
    let validate = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["graph", "validate", "demo"])
        .current_dir(tmp.path())
        .env("CERULION_HOME", home.path())
        .env("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:1")
        .env("CERULION_NETD_SOCK", &netd_sock)
        .env("CERULION_NETD_BIN", &missing_netd)
        .env_remove("CERULION_LOGIN_GATE")
        .env_remove("CERULION_NETWORK")
        .env_remove("CARGO_TARGET_DIR")
        .stdin(Stdio::null())
        .output()
        .expect("spawn cerulion graph validate");
    let validate_stderr = String::from_utf8_lossy(&validate.stderr);
    assert!(
        validate.status.success(),
        "graph validate must succeed without sign-in; code={:?} stderr={validate_stderr}",
        validate.status.code()
    );
    assert!(
        !home.path().join("auth.json").exists(),
        "graph validate must not write auth.json"
    );

    // 2. Permissive-default `cerulion graph run demo --single-process` degrades
    //    to local-only with a loud warning, opens no network listener, and
    //    delivers frames over local shared memory.
    let stdout_path = tmp.path().join("run.stdout");
    let stderr_path = tmp.path().join("run.stderr");
    let child = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["graph", "run", "demo", "--single-process"])
        .current_dir(tmp.path())
        .env("CERULION_HOME", home.path())
        .env("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:1")
        .env("CERULION_NETD_SOCK", &netd_sock)
        .env("CERULION_NETD_BIN", &missing_netd)
        // Turn off the Flashback window recorder so the stand-in netd socket
        // assertion observes only `graph run`'s own egress path (bagd probes
        // any existing netd socket read-only to check mirrors).
        .env("CERULION_FLASHBACK", "off")
        .env("NO_COLOR", "1")
        .env("RUST_LOG", "cerulion=info,cerulion_cli_engine=info")
        .env_remove("CERULION_LOGIN_GATE")
        .env_remove("CERULION_NETWORK")
        .env_remove("CARGO_TARGET_DIR")
        .stdin(Stdio::null())
        .stdout(Stdio::from(std::fs::File::create(&stdout_path).unwrap()))
        .stderr(Stdio::from(std::fs::File::create(&stderr_path).unwrap()))
        .spawn()
        .expect("spawn cerulion graph run");
    let mut guard = ChildGuard::single_process(child);

    let live_stderr = wait_for_log_line(
        &stderr_path,
        "starting graph (live)",
        Duration::from_secs(30),
    );
    assert!(
        live_stderr.contains("local-only")
            && live_stderr.contains("demo")
            && live_stderr.contains("cerulion login"),
        "permissive-default graph run without sign-in must log a loud local-only serving warning; stderr:\n{live_stderr}"
    );

    // Prove delivery on the same never-signed-in machine via `topic echo --local`.
    let topic = format!("/{prefix}/ticker/cmd");
    let echo_stdout = tmp.path().join("echo.stdout");
    let echo_child = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["topic", "echo", "--local", &topic])
        .current_dir(tmp.path())
        .env("CERULION_HOME", home.path())
        .env("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:1")
        .env("CERULION_NETD_SOCK", &netd_sock)
        .env("CERULION_NETD_BIN", &missing_netd)
        .env_remove("CERULION_LOGIN_GATE")
        .env_remove("CERULION_NETWORK")
        .stdin(Stdio::null())
        .stdout(Stdio::from(std::fs::File::create(&echo_stdout).unwrap()))
        .stderr(Stdio::null())
        .spawn()
        .expect("spawn cerulion topic echo");
    let mut echo_guard = ChildGuard::single_process(echo_child);
    let echo_log = wait_for_log_line(&echo_stdout, "seq=", Duration::from_secs(10));
    send_signal(echo_guard.id(), libc::SIGINT);
    let echo_status = echo_guard
        .wait_bounded(Duration::from_secs(10))
        .expect("topic echo did not exit after SIGINT");
    echo_guard.finish().assert_clean();
    assert!(
        echo_status.success(),
        "topic echo must exit 0 on SIGINT, got {echo_status:?}"
    );
    assert!(
        echo_log.contains("seq=") && echo_log.contains("schema=0x"),
        "topic echo on '{topic}' must observe a delivered frame; stdout:\n{echo_log}"
    );

    send_signal(guard.id(), libc::SIGINT);
    let status = guard
        .wait_bounded(Duration::from_secs(40))
        .expect("graph run did not exit after SIGINT");
    guard.finish().assert_clean();
    assert!(
        status.success(),
        "graph run must exit 0 on SIGINT, got {status:?}; stderr:\n{}",
        read_file(&stderr_path)
    );
    let netd_accept = netd_listener.accept();
    assert!(
        matches!(&netd_accept, Err(e) if e.kind() == std::io::ErrorKind::WouldBlock),
        "offline graph run must not connect to cerulion-netd; accept={netd_accept:?}; run stderr:\n{}",
        read_file(&stderr_path)
    );
    assert!(
        !home.path().join("auth.json").exists(),
        "offline graph run must not write auth.json"
    );
}

#[test]
fn account_verb_refuses_and_names_account_service_without_sign_in() {
    let cwd = tempfile::tempdir().unwrap();
    for (override_url, expected_service) in [
        (Some("http://127.0.0.1:1"), "http://127.0.0.1:1"),
        (None, "https://app.cerulion.com"),
    ] {
        let home = tempfile::tempdir().unwrap();
        let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
        cmd.args(["account", "devices", "list"])
            .current_dir(cwd.path())
            .env("CERULION_HOME", home.path())
            .env_remove("CERULION_LOGIN_GATE")
            .env_remove("CERULION_ACCOUNT_SERVICE")
            .stdin(Stdio::null());
        if let Some(url) = override_url {
            cmd.env("CERULION_ACCOUNT_SERVICE", url);
        }
        let out = cmd.output().expect("spawn cerulion account devices list");
        let stderr = String::from_utf8_lossy(&out.stderr);
        let stdout = String::from_utf8_lossy(&out.stdout);
        assert_eq!(
            out.status.code(),
            Some(i32::from(EXIT_AUTH_REQUIRED)),
            "account verb must exit 7 without sign-in; stderr={stderr}"
        );
        assert!(stdout.is_empty(), "stdout={stdout}");
        let expected = format!(
            "Error: This command requires the Cerulion account service ({expected_service}), and this machine has never signed in.\nRun `cerulion login` once in a terminal, or for automation point CERULION_HOME at a directory holding the state of a machine that did."
        );
        assert!(
            stderr.contains(&expected),
            "refusal must name the Cerulion account service ({expected_service}); want:\n{expected}\ngot:\n{stderr}"
        );
        assert!(
            !home.path().join("auth.json").exists(),
            "refused account verb must not write auth.json"
        );
    }
}

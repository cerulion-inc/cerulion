// SPDX-License-Identifier: AGPL-3.0-only
use super::*;

#[test]
fn sibling_resolution_uses_the_executable_then_its_symlink_target() {
    let current = Path::new("/launcher/cerulion-netd");
    let canonical = Path::new("/release/cerulion-netd");
    assert_eq!(
        resolve_sibling("cerulion", current, Some(canonical), |path| path
            == Path::new("/release/cerulion"))
        .unwrap(),
        PathBuf::from("/release/cerulion")
    );
    assert_eq!(
        resolve_sibling("cerulion-remoted", current, Some(canonical), |_| true).unwrap(),
        PathBuf::from("/launcher/cerulion-remoted")
    );
    let missing = resolve_sibling("cerulion", current, Some(canonical), |_| false).unwrap_err();
    assert!(missing.contains("/launcher/cerulion"));
    assert!(missing.contains("/release/cerulion"));
}

fn prepared() -> Prepared {
    Prepared {
        version: 1,
        registration_bundle: PathBuf::from("/state/robot-registration.json"),
        robot_id: "11".repeat(32),
        endpoint_id: "22".repeat(32),
        owner_account: "33".repeat(32),
    }
}

#[test]
fn machine_results_require_supported_identity_fields_and_a_contained_bundle() {
    assert!(validate_prepared(&prepared(), Path::new("/state")).is_ok());
    for change in 0..6 {
        let mut result = prepared();
        match change {
            0 => result.version = 2,
            1 => result.endpoint_id = "zz".repeat(32),
            2 => result.robot_id.pop().map(|_| ()).unwrap(),
            3 => result.owner_account = "AA".repeat(32),
            4 => result.registration_bundle = PathBuf::from("/other/registration.json"),
            _ => result.registration_bundle = PathBuf::from("/state/../other/registration.json"),
        }
        assert!(
            validate_prepared(&result, Path::new("/state")).is_err(),
            "case {change}"
        );
    }
}

#[test]
fn cancellation_before_spawn_does_not_execute_a_child() {
    let running = AtomicBool::new(false);
    let error = run_json::<Prepared>(
        Command::new("a-command-that-does-not-exist"),
        "registration",
        Instant::now() + Duration::from_secs(1),
        &running,
    )
    .err()
    .unwrap();
    assert!(error.contains("cancelled"));
    assert!(!error.contains("No such file"));
}

// A real leaf process exercises kill/reap and pipe EOF. It emits no fabricated
// registration result; this proves lifecycle handling, not robot provisioning.
fn sleeping_child(pid_file: &Path) -> Command {
    let mut command = Command::new("/bin/sh");
    command
        .arg("-c")
        .arg(r#"printf '%s\n' "$$" > "$1"; exec /bin/sleep 60"#)
        .arg("supervisor-test")
        .arg(pid_file);
    command
}

fn assert_process_reaped(pid_file: &Path) {
    let pid = std::fs::read_to_string(pid_file).expect("child published its actual PID");
    let pid: u32 = pid.trim().parse().expect("numeric child PID");
    assert!(
        !Command::new("/bin/kill")
            .arg("-0")
            .arg(pid.to_string())
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status()
            .unwrap()
            .success(),
        "owned child {pid} survived supervisor cleanup"
    );
}

#[test]
fn cancelling_a_running_child_kills_and_reaps_it_without_waiting_for_deadline() {
    let temp = tempfile::tempdir().unwrap();
    let pid_file = temp.path().join("child.pid");
    let running = Arc::new(AtomicBool::new(true));
    let observed = Arc::clone(&running);
    let marker = pid_file.clone();
    let cancel = std::thread::spawn(move || {
        let deadline = Instant::now() + Duration::from_secs(3);
        let has_pid = || {
            std::fs::read_to_string(&marker)
                .ok()
                .is_some_and(|text| text.trim().parse::<u32>().is_ok())
        };
        while !has_pid() && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(10));
        }
        let started = has_pid();
        observed.store(false, Ordering::SeqCst);
        started
    });
    let start = Instant::now();
    let result = run_json::<Prepared>(
        sleeping_child(&pid_file),
        "worker",
        start + Duration::from_secs(30),
        &running,
    );
    assert!(
        cancel.join().unwrap(),
        "cancellation must occur after actual spawn"
    );
    assert!(result.err().unwrap().contains("cancelled"));
    assert!(start.elapsed() < Duration::from_secs(5));
    assert_process_reaped(&pid_file);
}

#[test]
fn a_running_child_past_the_deadline_is_killed_and_reaped() {
    let temp = tempfile::tempdir().unwrap();
    let pid_file = temp.path().join("child.pid");
    let start = Instant::now();
    let result = run_json::<Prepared>(
        sleeping_child(&pid_file),
        "worker",
        start + Duration::from_secs(2),
        &AtomicBool::new(true),
    );
    assert!(result.err().unwrap().contains("deadline"));
    assert!(start.elapsed() < Duration::from_secs(5));
    assert_process_reaped(&pid_file);
}

#[test]
fn retry_delays_double_from_five_seconds_to_a_five_minute_cap_then_stop() {
    let secs: Vec<u64> = (1..=31)
        .map(|attempt| retry_delay(attempt).map_or(0, |d| d.as_secs()))
        .collect();
    let mut expected = vec![5, 10, 20, 40, 80, 160];
    expected.extend(std::iter::repeat_n(300, 23));
    expected.extend([0, 0]);
    assert_eq!(secs, expected);
}

#[test]
fn a_cleared_running_flag_ends_the_retry_wait_early() {
    let running = AtomicBool::new(false);
    let started = Instant::now();
    assert!(!wait_while_running(&running, Duration::from_secs(60)));
    assert!(started.elapsed() < Duration::from_secs(5));
    let running = AtomicBool::new(true);
    assert!(wait_while_running(&running, Duration::from_millis(1)));
}

#[test]
fn a_failure_after_a_healthy_run_counts_as_the_first_attempt() {
    assert_eq!(counted_attempt(29, Duration::from_secs(599)), 29);
    assert_eq!(counted_attempt(29, Duration::from_secs(600)), 1);
    assert_eq!(counted_attempt(3, Duration::from_secs(86_400)), 1);
    assert_eq!(counted_attempt(1, Duration::ZERO), 1);
}

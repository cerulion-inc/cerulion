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

#[test]
fn epoch_sync_delays_double_from_five_seconds_to_a_five_minute_cap_without_stopping() {
    let secs: Vec<u64> = [1, 2, 3, 4, 5, 6, 7, 100]
        .iter()
        .map(|failures| epoch_sync_delay(*failures, DEFAULT_EPOCH_SYNC_PERIOD).as_secs())
        .collect();
    assert_eq!(secs, vec![5, 10, 20, 40, 80, 160, 300, 300]);
    // Never longer than a shorter configured period.
    let short: Vec<u64> = [1, 3, 4, 100]
        .iter()
        .map(|failures| epoch_sync_delay(*failures, Duration::from_secs(30)).as_secs())
        .collect();
    assert_eq!(short, vec![5, 20, 30, 30]);
}

#[test]
fn the_epoch_sync_period_is_read_from_the_environment() {
    let prior = std::env::var_os(EPOCH_SYNC_PERIOD_ENV);
    std::env::set_var(EPOCH_SYNC_PERIOD_ENV, "75");
    let configured = epoch_sync_period_from_env();
    match prior {
        Some(value) => std::env::set_var(EPOCH_SYNC_PERIOD_ENV, value),
        None => std::env::remove_var(EPOCH_SYNC_PERIOD_ENV),
    }
    assert_eq!(configured, Duration::from_secs(75));
}

/// A stand-in worker: appends one line to `log` per run, exits with `code`.
fn fake_worker(dir: &Path, code: i32) -> (PathBuf, PathBuf) {
    use std::os::unix::fs::PermissionsExt;
    let log = dir.join("runs.log");
    let script = dir.join("cerulion");
    std::fs::write(
        &script,
        format!(
            "#!/bin/sh\necho \"$@\" >> '{}'\nexit {code}\n",
            log.display()
        ),
    )
    .unwrap();
    std::fs::set_permissions(&script, std::fs::Permissions::from_mode(0o755)).unwrap();
    (script, log)
}

fn runs(log: &Path) -> usize {
    std::fs::read_to_string(log).map_or(0, |text| text.lines().count())
}

/// Poll at `now` until the running worker has been reaped.
fn settle(sync: &mut EpochSync, now: Instant) {
    let until = Instant::now() + Duration::from_secs(10);
    while sync.child.is_some() && Instant::now() < until {
        std::thread::sleep(Duration::from_millis(10));
        sync.poll(now);
    }
    assert!(sync.child.is_none(), "the worker finished");
}

#[test]
fn epoch_sync_fetches_at_start_then_each_period_and_early_on_request() {
    let dir = tempfile::tempdir().unwrap();
    let (cli, log) = fake_worker(dir.path(), 0);
    let root = dir.path().join("state");
    std::fs::create_dir_all(root.join("remoted")).unwrap();
    let t0 = Instant::now();
    let mut sync = EpochSync::new(cli, root.clone(), DEFAULT_EPOCH_SYNC_PERIOD, t0);
    sync.poll(t0);
    settle(&mut sync, t0);
    assert_eq!(runs(&log), 1);
    assert_eq!(
        std::fs::read_to_string(&log).unwrap().trim(),
        format!("robot-sync-epoch --state-root {}", root.display())
    );
    // Nothing more until the period, unless remoted asks.
    sync.poll(t0 + Duration::from_secs(60));
    assert!(sync.child.is_none());
    let request = root.join("remoted").join("epoch.request");
    std::fs::write(&request, b"").unwrap();
    // Within the early-fetch gap of the last start: deferred to the gap's end.
    sync.poll(t0 + Duration::from_secs(2));
    assert!(!request.exists(), "the request is consumed");
    assert!(sync.child.is_none());
    sync.poll(t0 + Duration::from_secs(5));
    settle(&mut sync, t0 + Duration::from_secs(5));
    assert_eq!(runs(&log), 2);
    let t1 = t0 + Duration::from_secs(5);
    sync.poll(t1 + DEFAULT_EPOCH_SYNC_PERIOD - Duration::from_secs(1));
    assert!(sync.child.is_none());
    sync.poll(t1 + DEFAULT_EPOCH_SYNC_PERIOD);
    settle(&mut sync, t1 + DEFAULT_EPOCH_SYNC_PERIOD);
    assert_eq!(runs(&log), 3);
}

#[test]
fn a_failing_epoch_sync_backs_off_and_keeps_trying() {
    let dir = tempfile::tempdir().unwrap();
    let (cli, log) = fake_worker(dir.path(), 1);
    let root = dir.path().join("state");
    std::fs::create_dir_all(root.join("remoted")).unwrap();
    let t0 = Instant::now();
    let mut sync = EpochSync::new(cli, root.clone(), DEFAULT_EPOCH_SYNC_PERIOD, t0);
    sync.poll(t0);
    settle(&mut sync, t0);
    assert_eq!(sync.failures, 1);
    // A request does not cut a backoff short.
    std::fs::write(root.join("remoted").join("epoch.request"), b"").unwrap();
    sync.poll(t0 + Duration::from_secs(4));
    assert!(sync.child.is_none());
    sync.poll(t0 + Duration::from_secs(5));
    settle(&mut sync, t0 + Duration::from_secs(5));
    assert_eq!(runs(&log), 2);
    assert_eq!(sync.failures, 2);
    assert_eq!(
        sync.next,
        t0 + Duration::from_secs(5) + Duration::from_secs(10)
    );
}

#[test]
fn an_overdue_epoch_sync_worker_is_killed_and_counts_as_a_failure() {
    use std::os::unix::fs::PermissionsExt;
    let dir = tempfile::tempdir().unwrap();
    let script = dir.path().join("cerulion");
    std::fs::write(&script, "#!/bin/sh\nexec sleep 30\n").unwrap();
    std::fs::set_permissions(&script, std::fs::Permissions::from_mode(0o755)).unwrap();
    let root = dir.path().join("state");
    let t0 = Instant::now();
    let mut sync = EpochSync::new(script, root, DEFAULT_EPOCH_SYNC_PERIOD, t0);
    sync.poll(t0);
    assert!(sync.child.is_some());
    sync.poll(t0 + EPOCH_SYNC_DEADLINE);
    assert!(
        sync.child.is_none(),
        "the overdue worker was killed and reaped"
    );
    assert_eq!(sync.failures, 1);
}

#[test]
fn the_server_loop_runs_the_revocation_sync_while_the_server_lives() {
    let dir = tempfile::tempdir().unwrap();
    let (cli, log) = fake_worker(dir.path(), 0);
    let root = dir.path().join("state");
    std::fs::create_dir_all(root.join("remoted")).unwrap();
    let mut server = OwnedChild(Command::new("sleep").arg("30").spawn().unwrap());
    let mut sync = EpochSync::new(cli, root, DEFAULT_EPOCH_SYNC_PERIOD, Instant::now());
    let running = Arc::new(AtomicBool::new(true));
    let stopper = Arc::clone(&running);
    let watched = log.clone();
    let stop = std::thread::spawn(move || {
        let until = Instant::now() + Duration::from_secs(10);
        while runs(&watched) == 0 && Instant::now() < until {
            std::thread::sleep(Duration::from_millis(20));
        }
        stopper.store(false, Ordering::SeqCst);
    });
    let result = supervise_server(&mut server, &mut sync, &running, "eid", &|_| Ok(false));
    stop.join().unwrap();
    assert!(result.is_ok(), "{result:?}");
    assert_eq!(runs(&log), 1, "the loop started the revocation sync");
}

#[test]
fn the_epoch_sync_period_comes_from_its_setting_within_bounds() {
    let secs = |v: Option<&str>| resolve_epoch_sync_period(v).as_secs();
    assert_eq!(secs(None), 300);
    assert_eq!(secs(Some("")), 300);
    assert_eq!(secs(Some(" 120 ")), 120);
    assert_eq!(secs(Some("30")), 30);
    assert_eq!(secs(Some("86400")), 86_400);
    for bad in ["29", "86401", "0", "-5", "5m", "abc"] {
        assert_eq!(secs(Some(bad)), 300, "{bad}");
    }
}

#[test]
fn the_configured_epoch_sync_period_is_honoured() {
    let dir = tempfile::tempdir().unwrap();
    let (cli, log) = fake_worker(dir.path(), 0);
    let root = dir.path().join("state");
    std::fs::create_dir_all(root.join("remoted")).unwrap();
    let t0 = Instant::now();
    let period = Duration::from_secs(45);
    let mut sync = EpochSync::new(cli, root, period, t0);
    sync.poll(t0);
    settle(&mut sync, t0);
    assert_eq!(runs(&log), 1);
    sync.poll(t0 + Duration::from_secs(44));
    assert!(sync.child.is_none());
    sync.poll(t0 + period);
    settle(&mut sync, t0 + period);
    assert_eq!(runs(&log), 2);
}

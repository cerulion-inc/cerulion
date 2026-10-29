// SPDX-License-Identifier: AGPL-3.0-only
//! The process token and the watchdog (T5), on one rclpy process that runs two
//! single-threaded executors on two threads (`two_exec.py`).
//!
//! - `two_executors_on_two_threads_replay_identically`: `left` folds `/a` and
//!   publishes `/left`; `right`, on the other thread, folds `/left`. Live, the
//!   two threads race; replayed alone, the token serves each thread's windows
//!   in the recorded order, so both nodes' callback logs equal the recording,
//!   twice, with an `identical` verdict.
//! - `a_callback_blocked_on_the_other_executor_trips_the_watchdog`: with
//!   `--block`, `right`'s callback waits for `left`'s next callback. Live that
//!   is fine; replayed, `right` holds the token while `left` waits for it, and
//!   the watchdog names the holder's entity within 10 s.
//!
//! `#[ignore]`: needs ROS 2 and the staged `.so`; the recipe is in
//! `replay_harness`. Linux only.

#![cfg(target_os = "linux")]

#[path = "replay_harness/mod.rs"]
mod harness;

use std::time::{Duration, Instant};

use harness::*;

fn two_exec(extra: &[&str]) -> Vec<String> {
    let mut v = vec!["python3".to_string(), script("two_exec.py")];
    v.extend(extra.iter().map(|s| s.to_string()));
    v.extend(SIM.iter().map(|s| s.to_string()));
    v
}

#[test]
#[ignore = "needs ROS 2 with librmw_cerulion.so staged; see replay_harness"]
fn two_executors_on_two_threads_replay_identically() {
    preconditions();
    let mut run = Run::new("token");
    // Two /clock subscribers in two_exec (left and right) plus talker_a.
    let rec = record_probe_with_subs(&mut run, two_exec(&[]), 3);
    let left = callbacks(&rec.out.join("left.jsonl"));
    let right = callbacks(&rec.out.join("right.jsonl"));
    assert!(
        left.len() >= 100 && right.len() >= 100,
        "count floors: {} {}",
        left.len(),
        right.len()
    );
    for i in 1..=2 {
        let (out, st) = replay(
            &mut run,
            "two_exec",
            two_exec(&[]),
            &[],
            &rec.log,
            &format!("rep{i}"),
        );
        assert!(st.success() || st.code().is_none(), "replay {i}: {st:?}");
        assert_eq!(callbacks(&out.join("left.jsonl")), left, "replay {i}: left");
        assert_eq!(
            callbacks(&out.join("right.jsonl")),
            right,
            "replay {i}: right"
        );
        let v = verdict(&rec.log);
        assert_eq!(v["verdict"], "identical", "replay {i}: {v:#}");
    }
    println!(
        "REPLAY T5 token: two executor threads, {} + {} callbacks identical in 2/2 replays",
        left.len(),
        right.len()
    );
}

#[test]
#[ignore = "needs ROS 2 with librmw_cerulion.so staged; see replay_harness"]
fn a_callback_blocked_on_the_other_executor_trips_the_watchdog() {
    preconditions();
    let mut run = Run::new("watchdog");
    let rec = record_probe_with_subs(&mut run, two_exec(&["--block"]), 3);
    let out = run.path("rep");
    let verdict_path = std::path::PathBuf::from(format!("{}.verdict.json", rec.log.display()));
    let _ = std::fs::remove_file(&verdict_path);
    let started = Instant::now();
    run.spawn(
        "two_exec",
        &two_exec(&["--block"]),
        &[("CERULION_RMW_REPLAY", rec.log.display().to_string())],
        &out,
    );
    let mut seen = None;
    while started.elapsed() < Duration::from_secs(30) {
        if verdict_path.exists() {
            let v = verdict(&rec.log);
            if v["verdict"] == "watchdog" {
                seen = Some((v, started.elapsed()));
                break;
            }
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    run.stop("two_exec");
    let (v, after) = seen.unwrap_or_else(|| {
        panic!(
            "no watchdog verdict; stderr: {}",
            stderr_of(&out, "two_exec")
        )
    });
    assert!(
        after <= Duration::from_secs(10) + Duration::from_secs(3),
        "fired after {after:?}"
    );
    let w = &v["watchdog"];
    assert_eq!(
        w["entity"], "sub:/left#0",
        "the holder's entity is named: {v:#}"
    );
    assert_eq!(w["node"], "right", "{v:#}");
    assert!(stderr_of(&out, "two_exec").contains("WATCHDOG"));
    println!("REPLAY T5 watchdog: fired after {after:?}, naming {}", w);
}

/// `record_probe` with `clockpub` waiting for `subs` `/clock` subscribers.
fn record_probe_with_subs(run: &mut Run, node: Vec<String>, subs: u32) -> Recorded {
    let out = run.path("rec");
    let logdir = run.path("rec/log");
    run.spawn(
        "two_exec",
        &node,
        &[("CERULION_RMW_DELIVERY_LOG", logdir.display().to_string())],
        &out,
    );
    let mut talker = vec!["python3".to_string(), script("talker.py")];
    talker.extend(SIM.iter().map(|s| s.to_string()));
    run.spawn("talker_a", &talker, &[], &out);
    std::thread::sleep(Duration::from_secs(3));
    run.spawn(
        "clockpub",
        &["python3".to_string(), script("clockpub.py")],
        &[
            ("TICKS", "4000".into()),
            ("CLOCK_PERIOD", "0.0005".into()),
            ("CLOCK_SUBS", subs.to_string()),
        ],
        &out,
    );
    assert!(
        wait_for_file(&out.join("clockpub.done"), Duration::from_secs(180)),
        "clockpub never finished; {}",
        stderr_of(&out, "clockpub")
    );
    std::thread::sleep(Duration::from_secs(1));
    run.stop("two_exec");
    run.stop("talker_a");
    run.stop("clockpub");
    Recorded {
        log: find_log(&logdir, "two_exec"),
        out,
    }
}

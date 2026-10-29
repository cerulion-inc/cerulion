// SPDX-License-Identifier: AGPL-3.0-only
//! What a replay refuses, by name (T4 and the refusal arms of T6, on real
//! processes).
//!
//! - `an_rclpy_multithreaded_executor_is_refused_and_a_single_thread_is_not`:
//!   `mixer --mt` (an rclpy `MultiThreadedExecutor`) records a log marked not
//!   replayable; its replay exits nonzero with `MultiThreadedExecutor` and the
//!   node name on stderr and a `refused` verdict. The single-threaded control
//!   on the same graph is not refused and replays `identical`.
//! - `both_variables_set_is_refused_by_name`: a process with both
//!   `CERULION_RMW_DELIVERY_LOG` and `CERULION_RMW_REPLAY` set fails
//!   `rmw_init` and names both.
//! - `a_log_of_another_process_is_refused`: replaying a log under a different
//!   command line fails `rmw_init`, naming the difference.
//!
//! `#[ignore]`: needs ROS 2 and the staged `.so`; the recipe is in
//! `replay_harness`. Linux only.

#![cfg(target_os = "linux")]

#[path = "replay_harness/mod.rs"]
mod harness;

use std::time::Duration;

use harness::*;

const TICKS: u32 = 1000;

fn mixer(extra: &[&str]) -> Vec<String> {
    let mut v = vec!["python3".to_string(), script("mixer.py")];
    v.extend(extra.iter().map(|s| s.to_string()));
    v.extend(SIM.iter().map(|s| s.to_string()));
    v
}

#[test]
#[ignore = "needs ROS 2 with librmw_cerulion.so staged; see replay_harness"]
fn an_rclpy_multithreaded_executor_is_refused_and_a_single_thread_is_not() {
    preconditions();
    // The multi-threaded recording is marked, and its replay refused.
    let mut run = Run::new("refuse-mt");
    let rec = record_probe(&mut run, "mixer", mixer(&["--mt"]), &[], "mixer", TICKS);
    assert!(
        process_channel_contains(&rec.log, "MultiThreadedExecutor, not replayable"),
        "the --mt recording is marked not replayable"
    );
    assert!(
        process_channel_contains(&rec.log, "(nodes mixer)"),
        "and names the node"
    );
    let out = run.path("rep-mt");
    run.spawn(
        "mixer",
        &mixer(&["--mt"]),
        &[("CERULION_RMW_REPLAY", rec.log.display().to_string())],
        &out,
    );
    let st = run
        .wait_exit("mixer", Duration::from_secs(60))
        .expect("a refused replay exits by itself");
    assert!(!st.success(), "a refused replay exits nonzero: {st:?}");
    let err = stderr_of(&out, "mixer");
    assert!(
        err.contains("MultiThreadedExecutor"),
        "stderr names the executor: {err}"
    );
    assert!(err.contains("mixer"), "stderr names the node: {err}");
    let v = verdict(&rec.log);
    assert_eq!(v["verdict"], "refused", "{v:#}");
    assert_ne!(v["verdict"], "identical");

    // The single-threaded control on the same graph.
    let mut run = Run::new("refuse-st-control");
    let rec = record_probe(&mut run, "mixer", mixer(&[]), &[], "mixer", TICKS);
    assert!(
        !process_channel_contains(&rec.log, "not replayable"),
        "the single-threaded recording is not marked"
    );
    let (out, st) = replay(&mut run, "mixer", mixer(&[]), &[], &rec.log, "rep-st");
    assert!(
        st.success(),
        "control exited {st:?}: {}",
        stderr_of(&out, "mixer")
    );
    let v = verdict(&rec.log);
    assert_eq!(
        v["verdict"], "identical",
        "the control is not refused: {v:#}"
    );
    println!("REPLAY T4 rclpy: --mt refused by name, single-threaded control identical");
}

#[test]
#[ignore = "needs ROS 2 with librmw_cerulion.so staged; see replay_harness"]
fn both_variables_set_is_refused_by_name() {
    preconditions();
    let mut run = Run::new("refuse-both");
    let out = run.path("both");
    run.spawn(
        "mixer",
        &mixer(&[]),
        &[
            (
                "CERULION_RMW_DELIVERY_LOG",
                run.path("d").display().to_string(),
            ),
            (
                "CERULION_RMW_REPLAY",
                run.path("x.rdl.mcap").display().to_string(),
            ),
        ],
        &out,
    );
    let st = run
        .wait_exit("mixer", Duration::from_secs(60))
        .expect("refused at rmw_init");
    assert!(!st.success());
    let err = stderr_of(&out, "mixer");
    assert!(
        err.contains("CERULION_RMW_DELIVERY_LOG") && err.contains("CERULION_RMW_REPLAY"),
        "both variables named: {err}"
    );
}

#[test]
#[ignore = "needs ROS 2 with librmw_cerulion.so staged; see replay_harness"]
fn a_log_of_another_process_is_refused() {
    preconditions();
    let mut run = Run::new("refuse-identity");
    let rec = record_probe(&mut run, "mixer", mixer(&[]), &[], "mixer", 200);
    let out = run.path("other");
    // The same script under another command line (an extra parameter).
    let mut other = mixer(&[]);
    other.extend(["-p".to_string(), "unused:=1".to_string()]);
    run.spawn(
        "mixer",
        &other,
        &[("CERULION_RMW_REPLAY", rec.log.display().to_string())],
        &out,
    );
    let st = run
        .wait_exit("mixer", Duration::from_secs(60))
        .expect("refused at rmw_init");
    assert!(!st.success());
    let err = stderr_of(&out, "mixer");
    assert!(err.contains("recorded by a different process"), "{err}");
    assert!(
        err.contains("command line"),
        "the differing field is named: {err}"
    );
    assert_eq!(verdict(&rec.log)["verdict"], "refused");
}

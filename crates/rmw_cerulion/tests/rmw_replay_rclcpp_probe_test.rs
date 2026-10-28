// SPDX-License-Identifier: AGPL-3.0-only
//! Replay of rclcpp processes (T2 in part, and the rclcpp half of T3), on the
//! stock `demo_nodes_cpp` executables.
//!
//! - `the_rclcpp_listener_replays_every_take_in_order`: `listener` is recorded
//!   while the stock `talker` publishes `/chatter`, then replayed ALONE. Its
//!   only outputs are log lines, so the judge is its own log (stderr): the
//!   replayed `I heard: [...]` lines equal the recorded ones, in order, and
//!   the verdict is `identical` with every unit served and no input mismatch.
//!   This exercises rclcpp's take path and wait-set handling against the gate.
//! - `the_rclcpp_wall_timer_talker_is_named`: the stock `talker` publishes
//!   from a WALL timer (`create_wall_timer`). Recorded alone and replayed
//!   alone, its verdict names node `talker`, topic `/chatter`: the replayed
//!   wait returns at once, rcl finds the steady timer not yet due, and the
//!   recorded publish is missing.
//!
//! Not covered here (no colcon fixture in this slice): an rclcpp node on ROS
//! time with the TimeSource clock thread, and the rclcpp MultiThreadedExecutor
//! refusal.
//!
//! `#[ignore]`: needs ROS 2, `demo_nodes_cpp` and the staged `.so`; the recipe
//! is in `replay_harness`. Linux only.

#![cfg(target_os = "linux")]

#[path = "replay_harness/mod.rs"]
mod harness;

use std::path::PathBuf;
use std::time::Duration;

use harness::*;

fn demo(exe: &str) -> Vec<String> {
    let distro = std::env::var("ROS_DISTRO").expect("ROS_DISTRO");
    let p = PathBuf::from(format!("/opt/ros/{distro}/lib/demo_nodes_cpp/{exe}"));
    assert!(
        p.exists(),
        "{} missing: install ros-{distro}-demo-nodes-cpp",
        p.display()
    );
    vec![p.display().to_string()]
}

fn heard(stdout: &str) -> Vec<String> {
    stdout
        .lines()
        .filter_map(|l| l.split_once("I heard: ").map(|(_, rest)| rest.to_string()))
        .collect()
}

#[test]
#[ignore = "needs ROS 2 with librmw_cerulion.so staged; see replay_harness"]
fn the_rclcpp_listener_replays_every_take_in_order() {
    preconditions();
    let mut run = Run::new("rclcpp-listener");
    let out = run.path("rec");
    let logdir = run.path("rec/log");
    run.spawn(
        "listener",
        &demo("listener"),
        &[("CERULION_RMW_DELIVERY_LOG", logdir.display().to_string())],
        &out,
    );
    std::thread::sleep(Duration::from_secs(2));
    run.spawn("talker", &demo("talker"), &[], &out);
    // The stock talker publishes at 1 Hz.
    std::thread::sleep(Duration::from_secs(6));
    run.stop("talker");
    std::thread::sleep(Duration::from_millis(500));
    run.stop("listener");
    let log = find_log(&logdir, "listener");
    // rcutils logs to stderr.
    let recorded = heard(&stderr_of(&out, "listener"));
    assert!(
        recorded.len() >= 4,
        "the recording heard {} messages (floor 4): {}",
        recorded.len(),
        stderr_of(&out, "listener")
    );
    let (rout, st) = replay(&mut run, "listener", demo("listener"), &[], &log, "rep1");
    assert!(st.success() || st.code().is_none(), "replay exited {st:?}");
    let replayed = heard(&stderr_of(&rout, "listener"));
    assert_eq!(replayed, recorded, "every recorded take served, in order");
    let v = verdict(&log);
    assert_eq!(v["verdict"], "identical", "{v:#}");
    assert_eq!(v["units_served"], v["units_recorded"], "{v:#}");
    assert!(v["input_mismatch"].is_null(), "{v:#}");
    println!(
        "REPLAY T2 rclcpp listener: {} of {} takes replayed in order; verdict {}",
        replayed.len(),
        recorded.len(),
        v
    );
}

#[test]
#[ignore = "needs ROS 2 with librmw_cerulion.so staged; see replay_harness"]
fn the_rclcpp_wall_timer_talker_is_named() {
    preconditions();
    let mut run = Run::new("rclcpp-talker");
    let out = run.path("rec");
    let logdir = run.path("rec/log");
    run.spawn(
        "talker",
        &demo("talker"),
        &[("CERULION_RMW_DELIVERY_LOG", logdir.display().to_string())],
        &out,
    );
    std::thread::sleep(Duration::from_secs(4));
    run.stop("talker");
    let log = find_log(&logdir, "talker");
    let (rout, _) = replay(&mut run, "talker", demo("talker"), &[], &log, "rep1");
    let v = verdict(&log);
    assert_eq!(v["verdict"], "diverged", "{v:#}");
    assert_eq!(v["culprit"]["node"], "talker", "{v:#}");
    assert_eq!(v["culprit"]["topic"], "/chatter", "{v:#}");
    assert!(
        stderr_of(&rout, "talker").contains("DIVERGED"),
        "named on stderr"
    );
    println!(
        "REPLAY T3 rclcpp wall timer: diverged at node {} topic {} window {}",
        v["culprit"]["node"], v["culprit"]["topic"], v["culprit"]["window"]
    );
}

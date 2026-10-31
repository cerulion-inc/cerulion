// SPDX-License-Identifier: AGPL-3.0-only
//! Exact replay of one rclpy process (T1) and the named wall-clock control
//! (T3, rclpy half).
//!
//! The probe graph is `clockpub` (deterministic `/clock`), `talker_a` (a 10 Hz
//! ROS-time timer on `/a`) and `mixer` (subscribes `/a`; a 7 Hz ROS-time timer
//! publishes a hash of every input and fire time in callback order on `/out`).
//! `mixer` is recorded once with the delivery log, then replayed ALONE: no
//! talker, no clock, no live input of any kind.
//!
//! - `the_mixer_replays_byte_identical_five_times`: 140 of 140 `/out` values
//!   equal the recording in 5 of 5 replays, every verdict is `identical` with
//!   no divergent segment, every unit served, and the five verdict files are
//!   byte-identical.
//! - `a_wall_clock_read_is_named_as_the_divergence`: the same recording
//!   replayed with `MIXER_WALLCLOCK=1` (the timer folds `time.time_ns()`) is
//!   named `diverged` at node `mixer`, topic `/out`, inputs matched, 2 of 2.
//!
//! `#[ignore]`: needs ROS 2 and the staged `.so`; the recipe is in
//! `replay_harness`. Linux only (rmw_cerulion's runtime).

#![cfg(target_os = "linux")]

#[path = "replay_harness/mod.rs"]
mod harness;

use harness::*;

/// 4000 ticks of 5 ms = 20 s of ROS time: 140 mixer timer outputs.
const TICKS: u32 = 4000;
const OUTPUTS: usize = 140;

fn mixer_argv() -> Vec<String> {
    let mut v = vec!["python3".to_string(), script("mixer.py")];
    v.extend(SIM.iter().map(|s| s.to_string()));
    v
}

#[test]
#[ignore = "needs ROS 2 with librmw_cerulion.so staged; see replay_harness"]
fn the_mixer_replays_byte_identical_five_times() {
    preconditions();
    let mut run = Run::new("rclpy-identical");
    let rec = record_probe(&mut run, "mixer", mixer_argv(), &[], "mixer", TICKS);
    let recorded = callback_values(&rec.out.join("mixer.jsonl"), "timer", "out");
    assert_eq!(
        recorded.len(),
        OUTPUTS,
        "the recording itself must carry every /out output (count floor)"
    );
    let recorded_callbacks = callbacks(&rec.out.join("mixer.jsonl"));
    let mut verdicts = Vec::new();
    let mut sets = Vec::new();
    for i in 1..=5 {
        let (out, st) = replay(
            &mut run,
            "mixer",
            mixer_argv(),
            &[],
            &rec.log,
            &format!("rep{i}"),
        );
        assert!(
            st.success(),
            "replay {i} exited {st:?}: {}",
            stderr_of(&out, "mixer")
        );
        let replayed = callback_values(&out.join("mixer.jsonl"), "timer", "out");
        assert_eq!(replayed.len(), OUTPUTS, "replay {i}: /out count");
        assert_eq!(replayed, recorded, "replay {i}: /out byte-identical");
        assert_eq!(
            callbacks(&out.join("mixer.jsonl")),
            recorded_callbacks,
            "replay {i}: the callback ORDER equals the recording"
        );
        let v = verdict(&rec.log);
        assert_eq!(v["verdict"], "identical", "replay {i} verdict: {v:#}");
        assert_eq!(v["segments_divergent"], 0, "replay {i}: {v:#}");
        assert_eq!(v["units_served"], v["units_recorded"], "replay {i}: {v:#}");
        assert!(
            v["segments_compared"].as_u64().unwrap() > 0,
            "replay {i}: {v:#}"
        );
        assert!(
            v["culprit"].is_null() && v["input_mismatch"].is_null(),
            "{v:#}"
        );
        sets.push(v["windows_served_as_sets"].as_u64().unwrap());
        verdicts.push(verdict_bytes(&rec.log));
    }
    for (i, v) in verdicts.iter().enumerate().skip(1) {
        assert_eq!(v, &verdicts[0], "verdict {} differs from verdict 1", i + 1);
    }
    println!(
        "REPLAY T1: {OUTPUTS}/{OUTPUTS} /out identical in 5/5 replays; windows served as sets: {sets:?}; verdict: {}",
        String::from_utf8_lossy(&verdicts[0])
    );
}

#[test]
#[ignore = "needs ROS 2 with librmw_cerulion.so staged; see replay_harness"]
fn a_wall_clock_read_is_named_as_the_divergence() {
    preconditions();
    let mut run = Run::new("rclpy-wallclock");
    let rec = record_probe(&mut run, "mixer", mixer_argv(), &[], "mixer", TICKS);
    let recorded = callback_values(&rec.out.join("mixer.jsonl"), "timer", "out");
    assert_eq!(recorded.len(), OUTPUTS);
    for i in 1..=2 {
        let (out, _) = replay(
            &mut run,
            "mixer",
            mixer_argv(),
            &[("MIXER_WALLCLOCK", "1".into())],
            &rec.log,
            &format!("neg{i}"),
        );
        let replayed = callback_values(&out.join("mixer.jsonl"), "timer", "out");
        assert_ne!(
            replayed.first(),
            recorded.first(),
            "control {i}: the wall clock changed /out"
        );
        let v = verdict(&rec.log);
        assert_eq!(v["verdict"], "diverged", "control {i}: {v:#}");
        let c = &v["culprit"];
        assert_eq!(c["node"], "mixer", "control {i}: {v:#}");
        assert_eq!(c["topic"], "/out", "control {i}: {v:#}");
        assert!(v["input_mismatch"].is_null(), "inputs matched: {v:#}");
        assert_eq!(
            c["output_index"], 0,
            "the first /out of the segment differs: {v:#}"
        );
        assert!(
            stderr_of(&out, "mixer").contains("DIVERGED"),
            "the divergence is named on stderr"
        );
        println!(
            "REPLAY T3 rclpy control {i}: diverged at node {} topic {} window {} delivery {}",
            c["node"], c["topic"], c["window"], c["delivery"]
        );
    }
}

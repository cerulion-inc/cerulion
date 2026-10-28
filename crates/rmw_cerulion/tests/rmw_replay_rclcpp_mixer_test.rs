// SPDX-License-Identifier: AGPL-3.0-only
//! Exact replay of rclcpp processes on ROS time (T2), the rclcpp wall-clock
//! control (T3) and the rclcpp `MultiThreadedExecutor` refusal (T4), on the
//! ament_cmake fixture `tests/fixtures/replay_probe_cpp/`.
//!
//! The graph is `clockpub` (rclpy, deterministic `/clock`), `talker_cpp` (a
//! 10 Hz ROS-time timer on `/a`), `mixer_cpp` (node `mixer`: subscribes `/a`;
//! a 7 Hz ROS-time timer publishes the fold of every input and fire time on
//! `/out`) and `listener_cpp` (folds `/out`, republishes the fold on
//! `/heard`). The three rclcpp nodes run on a `SingleThreadedExecutor` with
//! `use_sim_time:=true` and rclcpp's default clock thread (`/clock` is taken on
//! a second thread with its own executor). All three are recorded in one run,
//! then each is replayed ALONE five times.
//!
//! - `the_rclcpp_probe_replays_byte_identical_five_times`: for each of the
//!   three processes, every replay's callback log equals the recording (count
//!   floors on the recording), every verdict is `identical` with every unit
//!   served, and the five verdict files are byte-identical except the
//!   `guards_not_triggered_live` diagnostic, which is printed.
//! - `an_rclcpp_wall_clock_read_is_named_as_the_divergence`: the mixer
//!   recording replayed with `MIXER_WALLCLOCK=1` is named `diverged` at node
//!   `mixer`, topic `/out`, inputs matched, 2 of 2.
//! - `an_rclcpp_multithreaded_executor_is_refused_and_a_single_thread_is_not`:
//!   `PROBE_MT=1` (a 2-thread `MultiThreadedExecutor`) records a log marked not
//!   replayable naming the node; its replay exits nonzero naming
//!   `MultiThreadedExecutor` and `mixer`, verdict `refused`. A single-threaded
//!   control is not refused. The /a callback does 5 ms of work in this test:
//!   with near-instant callbacks one executor thread re-takes the wait mutex
//!   every time, the second never reaches the rmw, and the recording carries
//!   no mark (the executor ran as a single thread; see `docs/ros2_replay.md`).
//!
//! The fixture is built once per test binary with colcon into
//! `CARGO_TARGET_TMPDIR`, or taken from `CERULION_REPLAY_CPP_PREFIX` (a colcon
//! install base that already holds it). `#[ignore]`: needs ROS 2, colcon and
//! the staged `.so`; the recipe is in `replay_harness`. Linux only.

#![cfg(target_os = "linux")]

#[path = "replay_harness/mod.rs"]
mod harness;

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::OnceLock;
use std::time::Duration;

use harness::*;

/// 4000 ticks of 5 ms = 20 s of ROS time.
const TICKS: u32 = 4000;

/// The colcon install base holding `replay_probe_cpp`.
fn install_base() -> &'static Path {
    static BASE: OnceLock<PathBuf> = OnceLock::new();
    BASE.get_or_init(|| {
        if let Some(p) = std::env::var_os("CERULION_REPLAY_CPP_PREFIX") {
            return PathBuf::from(p);
        }
        let fixture = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/replay_probe_cpp");
        let base = Path::new(env!("CARGO_TARGET_TMPDIR")).join("replay_probe_cpp");
        // A fresh build each time: a CMake cache from an earlier configure keeps
        // the environment that configure saw.
        let _ = std::fs::remove_dir_all(&base);
        // ament_cmake checks RMW_IMPLEMENTATION against the rmw packages it finds
        // at configure time; the staged rmw_cerulion prefix is not one, and the
        // fixture links no rmw, so its build runs without it.
        let out = Command::new("colcon")
            .env_remove("RMW_IMPLEMENTATION")
            .arg("--log-base")
            .arg(base.join("log"))
            .arg("build")
            .arg("--base-paths")
            .arg(&fixture)
            .arg("--build-base")
            .arg(base.join("build"))
            .arg("--install-base")
            .arg(base.join("install"))
            .args(["--cmake-args", "-DCMAKE_BUILD_TYPE=Release"])
            .output()
            .unwrap_or_else(|e| panic!("colcon (needed to build the rclcpp fixture): {e}"));
        assert!(
            out.status.success(),
            "colcon build of {} failed: {}\n{}",
            fixture.display(),
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        );
        base.join("install")
    })
}

fn exe(name: &str) -> Vec<String> {
    let p = install_base().join(format!("replay_probe_cpp/lib/replay_probe_cpp/{name}"));
    assert!(p.exists(), "{} missing after the colcon build", p.display());
    let mut v = vec![p.display().to_string()];
    v.extend(SIM.iter().map(|s| s.to_string()));
    v
}

/// The three rclcpp processes: (child name, executable, log stem, callback log).
const NODES: [(&str, &str, &str); 3] = [
    ("talker_cpp", "talker_cpp", "talker_cpp"),
    ("mixer", "mixer_cpp", "mixer"),
    ("listener_cpp", "listener_cpp", "listener_cpp"),
];

/// Record the C++ graph: every rclcpp process with the delivery log, then
/// `clockpub` once all three subscribe `/clock`. Returns the out dir and each
/// process's log, in `NODES` order.
fn record_cpp(run: &mut Run, mixer_env: &[(&str, String)], ticks: u32) -> (PathBuf, Vec<PathBuf>) {
    let out = run.path("rec");
    let logdir = run.path("rec/log");
    for (name, exe_name, _) in NODES {
        let mut env: Vec<(&str, String)> =
            vec![("CERULION_RMW_DELIVERY_LOG", logdir.display().to_string())];
        if name == "mixer" {
            env.extend(mixer_env.iter().cloned());
        }
        run.spawn(name, &exe(exe_name), &env, &out);
    }
    std::thread::sleep(Duration::from_secs(3));
    run.spawn(
        "clockpub",
        &["python3".to_string(), script("clockpub.py")],
        &[
            ("TICKS", ticks.to_string()),
            ("CLOCK_PERIOD", "0.0005".into()),
            ("CLOCK_SUBS", "3".into()),
        ],
        &out,
    );
    assert!(
        wait_for_file(&out.join("clockpub.done"), Duration::from_secs(180)),
        "clockpub never finished its {ticks} ticks; stderr: {}",
        stderr_of(&out, "clockpub")
    );
    std::thread::sleep(Duration::from_secs(1));
    for (name, _, _) in NODES {
        let st = run.stop(name);
        assert!(
            st.success() || st.code().is_none(),
            "{name} exited {st:?}; stderr: {}",
            stderr_of(&out, name)
        );
    }
    run.stop("clockpub");
    let logs = NODES
        .iter()
        .map(|(_, exe_name, _)| find_log(&logdir, exe_name))
        .collect();
    (out, logs)
}

/// The distinct recorded threads that called `rmw_wait` (the `rdl/waits`
/// record: version u8, tag u8, k u64, thread u32 ...).
fn waiting_threads(log: &Path) -> usize {
    channel_payloads(log, "rdl/waits")
        .iter()
        .filter(|p| p.len() >= 14)
        .map(|p| u32::from_le_bytes([p[10], p[11], p[12], p[13]]))
        .collect::<BTreeSet<u32>>()
        .len()
}

#[test]
#[ignore = "needs ROS 2, colcon and librmw_cerulion.so staged; see replay_harness"]
fn the_rclcpp_probe_replays_byte_identical_five_times() {
    preconditions();
    let mut run = Run::new("rclcpp-identical");
    let (out, logs) = record_cpp(&mut run, &[], TICKS);
    let mut report = Vec::new();
    for ((name, exe_name, cblog), log) in NODES.iter().zip(&logs) {
        let recorded = callbacks(&out.join(format!("{cblog}.jsonl")));
        let floor = match *name {
            "talker_cpp" => 190,
            "mixer" => 320,
            _ => 130,
        };
        assert!(
            recorded.len() >= floor,
            "{name}: the recording carries {} callbacks (floor {floor})",
            recorded.len()
        );
        let threads = waiting_threads(log);
        let mut verdicts = Vec::new();
        let mut sets = Vec::new();
        for i in 1..=5 {
            let (rout, st) = replay(
                &mut run,
                name,
                exe(exe_name),
                &[],
                log,
                &format!("rep-{name}-{i}"),
            );
            assert!(
                st.success() || st.code().is_none(),
                "{name} replay {i} exited {st:?}: {}",
                stderr_of(&rout, name)
            );
            let replayed = callbacks(&rout.join(format!("{cblog}.jsonl")));
            assert_eq!(
                replayed, recorded,
                "{name} replay {i}: every callback and output equals the recording, in order"
            );
            let v = verdict(log);
            assert_eq!(v["verdict"], "identical", "{name} replay {i}: {v:#}");
            assert_eq!(v["segments_divergent"], 0, "{name} replay {i}: {v:#}");
            assert_eq!(
                v["units_served"], v["units_recorded"],
                "{name} replay {i}: {v:#}"
            );
            assert!(
                v["outputs_compared"].as_u64().unwrap() > 0,
                "{name} replay {i}: {v:#}"
            );
            sets.push(v["windows_served_as_sets"].as_u64().unwrap());
            verdicts.push(verdict_bytes(log));
        }
        // Every verdict field but `guards_not_triggered_live` is equal byte
        // for byte. That counter (recorded-ready guards with no live trigger
        // when served) moves between 1 and 2 across replays of one rclcpp
        // log: the last windows carry the shutdown guards the recording's
        // SIGINT triggered, and the replay is stopped after its verdict.
        let strip = |b: &[u8]| -> String {
            String::from_utf8_lossy(b)
                .lines()
                .filter(|l| !l.contains("\"guards_not_triggered_live\""))
                .collect::<Vec<_>>()
                .join("\n")
        };
        let mut guard_counts = Vec::new();
        for (i, v) in verdicts.iter().enumerate() {
            assert_eq!(
                strip(v),
                strip(&verdicts[0]),
                "{name}: verdict {} differs from verdict 1",
                i + 1
            );
            let parsed: serde_json::Value = serde_json::from_slice(v).expect("verdict json");
            guard_counts.push(parsed["guards_not_triggered_live"].as_u64().unwrap());
        }
        let v = verdict(log);
        report.push(format!(
            "{name}: {} callbacks identical 5/5, waiting threads {threads}, windows {} \
             (sets {sets:?}, split {}), outputs compared {}, guards not triggered live \
             {guard_counts:?}",
            recorded.len(),
            v["windows_recorded"],
            v["windows_split"],
            v["outputs_compared"]
        ));
    }
    for r in report {
        println!("REPLAY T2 rclcpp {r}");
    }
}

#[test]
#[ignore = "needs ROS 2, colcon and librmw_cerulion.so staged; see replay_harness"]
fn an_rclcpp_wall_clock_read_is_named_as_the_divergence() {
    preconditions();
    let mut run = Run::new("rclcpp-wallclock");
    let (out, logs) = record_cpp(&mut run, &[], TICKS);
    let recorded = callback_values(&out.join("mixer.jsonl"), "timer", "out");
    assert!(recorded.len() >= 130, "recorded {} outputs", recorded.len());
    for i in 1..=2 {
        let (rout, _) = replay(
            &mut run,
            "mixer",
            exe("mixer_cpp"),
            &[("MIXER_WALLCLOCK", "1".into())],
            &logs[1],
            &format!("neg{i}"),
        );
        let replayed = callback_values(&rout.join("mixer.jsonl"), "timer", "out");
        assert_ne!(
            replayed.first(),
            recorded.first(),
            "control {i}: the wall clock changed /out"
        );
        let v = verdict(&logs[1]);
        assert_eq!(v["verdict"], "diverged", "control {i}: {v:#}");
        let c = &v["culprit"];
        assert_eq!(c["node"], "mixer", "control {i}: {v:#}");
        assert_eq!(c["topic"], "/out", "control {i}: {v:#}");
        assert!(v["input_mismatch"].is_null(), "inputs matched: {v:#}");
        assert!(
            stderr_of(&rout, "mixer").contains("DIVERGED"),
            "the divergence is named on stderr"
        );
        println!(
            "REPLAY T3 rclcpp control {i}: diverged at node {} topic {} window {} delivery {}",
            c["node"], c["topic"], c["window"], c["delivery"]
        );
    }
}

#[test]
#[ignore = "needs ROS 2, colcon and librmw_cerulion.so staged; see replay_harness"]
fn an_rclcpp_multithreaded_executor_is_refused_and_a_single_thread_is_not() {
    preconditions();
    // The /a callback sleeps 5 ms: with near-instant callbacks the thread that
    // just ran one re-takes the executor's wait mutex first, the second
    // thread never reaches the rmw, and no mark is left (measured on Humble).
    let mt = [
        ("PROBE_MT", "1".to_string()),
        ("PROBE_CB_SLEEP_US", "5000".to_string()),
    ];
    let mut run = Run::new("rclcpp-refuse-mt");
    let (_, logs) = record_cpp(&mut run, &mt, 1000);
    let log = &logs[1];
    assert!(
        process_channel_contains(log, "MultiThreadedExecutor, not replayable"),
        "the PROBE_MT recording is marked not replayable"
    );
    assert!(
        process_channel_contains(log, "(nodes mixer)"),
        "and names the node"
    );
    let out = run.path("rep-mt");
    let mut env: Vec<(&str, String)> = vec![("CERULION_RMW_REPLAY", log.display().to_string())];
    env.extend(mt.iter().cloned());
    run.spawn("mixer", &exe("mixer_cpp"), &env, &out);
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
    let v = verdict(log);
    assert_eq!(v["verdict"], "refused", "{v:#}");
    let first_err = err
        .lines()
        .find(|l| l.contains("MultiThreadedExecutor"))
        .unwrap_or_default()
        .to_string();

    let mut run = Run::new("rclcpp-refuse-st-control");
    let (_, logs) = record_cpp(&mut run, &[], 1000);
    assert!(
        !process_channel_contains(&logs[1], "not replayable"),
        "the single-threaded recording is not marked"
    );
    let (rout, st) = replay(&mut run, "mixer", exe("mixer_cpp"), &[], &logs[1], "rep-st");
    assert!(
        st.success() || st.code().is_none(),
        "control exited {st:?}: {}",
        stderr_of(&rout, "mixer")
    );
    assert_eq!(
        verdict(&logs[1])["verdict"],
        "identical",
        "the control is not refused"
    );
    println!("REPLAY T4 rclcpp: PROBE_MT refused by name ({first_err}); single-threaded control identical");
}

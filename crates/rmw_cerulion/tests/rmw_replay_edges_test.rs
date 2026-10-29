// SPDX-License-Identifier: AGPL-3.0-only
//! The edges of exact replay, on real processes: torn logs, threads outside
//! every executor, serialized takes and services.
//!
//! - `a_log_torn_anywhere_replays_to_log_exhausted_early`: one long `mixer`
//!   recording is cut at chunk boundaries (what a SIGKILL leaves) and inside
//!   chunks (an arbitrary record), and each cut log is replayed. Every replay
//!   says `log_exhausted_early` with no culprit and no input mismatch, and
//!   its `/out` values are a prefix of the recorded ones.
//! - `a_publishing_thread_outside_every_executor_is_refused`: `mixer` with
//!   `MIXER_SIDE_THREAD=1` also publishes `/side` from a plain
//!   `threading.Thread`. Its replay is not `identical`: the verdict is
//!   `refused`, naming the cause and `/side`.
//! - `a_serialized_take_is_refused_under_its_own_name`: `mixer` with
//!   `MIXER_RAW=1` also subscribes `/a` with `raw=True`. The log is marked
//!   `serialized take, not replayable` (not `MultiThreadedExecutor`) and its
//!   replay is refused by that name.
//! - `a_replay_that_stops_taking_is_never_identical`: `mixer` is recorded,
//!   then replayed with `MIXER_EXIT_AFTER_A=40`, so the replay exits after
//!   its 40th `/a` callback and never makes the recording's later takes.
//!   The verdict is not `identical` and counts the takes not replayed.
//! - `a_killed_replay_never_leaves_an_earlier_verdict`: after an `identical`
//!   replay, a second replay of the same log blocks in its 5th `/a`
//!   callback (`MIXER_HANG_AFTER_A=5`) and is killed with SIGKILL. The
//!   verdict file says `running`, not the earlier `identical`.
//! - `an_rclpy_service_pair_replays_identically_on_both_sides`: `svc.py
//!   server` and `svc.py client` are recorded together on ROS time, then each
//!   is replayed alone: every request and response callback matches the
//!   recording, and the verdicts are `identical`.
//! - `the_rclcpp_add_two_ints_pair_replays_identically_on_both_sides`: the
//!   same with the stock `demo_nodes_cpp` `add_two_ints_server` and
//!   `add_two_ints_client`. The stock client sends its request before its
//!   first `rmw_wait`, so only the server's response is compared; the
//!   client's replay must take the recorded response and print its sum.
//!
//! `#[ignore]`: needs ROS 2 and the staged `.so`; the recipe is in
//! `replay_harness`. Linux only.

#![cfg(target_os = "linux")]

#[path = "replay_harness/mod.rs"]
mod harness;

use std::path::PathBuf;
use std::process::Command;
use std::time::Duration;

use harness::*;

fn mixer() -> Vec<String> {
    let mut v = vec!["python3".to_string(), script("mixer.py")];
    v.extend(SIM.iter().map(|s| s.to_string()));
    v
}

/// The byte offset of every Chunk record (opcode 0x06) of an MCAP file.
fn chunk_offsets(bytes: &[u8]) -> Vec<usize> {
    let mut v = Vec::new();
    let mut at = 8; // magic
    while at + 9 <= bytes.len() {
        let op = bytes[at];
        let len = u64::from_le_bytes(bytes[at + 1..at + 9].try_into().unwrap()) as usize;
        if op == 0x06 {
            v.push(at);
        }
        if op == 0x0F || at + 9 + len > bytes.len() {
            break;
        }
        at += 9 + len;
    }
    v
}

#[test]
#[ignore = "needs ROS 2 with librmw_cerulion.so staged; see replay_harness"]
fn a_log_torn_anywhere_replays_to_log_exhausted_early() {
    preconditions();
    let mut run = Run::new("torn");
    // 100 s of ROS time: a log of several 1 MiB chunks.
    let rec = record_probe(&mut run, "mixer", mixer(), &[], "mixer", 20_000);
    let recorded = callback_values(&rec.out.join("mixer.jsonl"), "timer", "out");
    assert_eq!(recorded.len(), 700, "the recording's own /out count");
    let full = std::fs::read(&rec.log).expect("read the log");
    let chunks = chunk_offsets(&full);
    assert!(
        chunks.len() >= 4,
        "the recording has a header chunk and at least three data chunks: {chunks:?}"
    );
    // Cut at the start of the second, the middle and the last data chunk
    // (what a SIGKILL leaves), and inside two chunks (an arbitrary record).
    let data = &chunks[1..];
    let mut cuts = vec![
        ("boundary-early", data[1]),
        ("boundary-mid", data[data.len() / 2]),
        ("boundary-last", data[data.len() - 1]),
        ("inside-early", data[0] + (data[1] - data[0]) / 2 + 7),
        (
            "inside-late",
            data[data.len() - 2] + (data[data.len() - 1] - data[data.len() - 2]) / 3 + 3,
        ),
    ];
    cuts.sort_by_key(|c| c.1);
    let mut results = Vec::new();
    for (tag, cut) in cuts {
        let torn = run.path(&format!("{tag}.rdl.mcap"));
        std::fs::write(&torn, &full[..cut]).expect("write the torn log");
        let (out, _) = replay(&mut run, "mixer", mixer(), &[], &torn, tag);
        let v = verdict(&torn);
        assert_eq!(
            v["verdict"], "log_exhausted_early",
            "{tag} at byte {cut}: {v:#}"
        );
        assert_eq!(v["log_complete"], false, "{tag}: {v:#}");
        assert!(
            v["culprit"].is_null() && v["input_mismatch"].is_null(),
            "{tag}: a torn log blames nobody: {v:#}"
        );
        let windows = v["windows_recorded"].as_u64().unwrap();
        assert!(
            windows > 100,
            "{tag}: the cut log still holds windows: {v:#}"
        );
        assert_eq!(v["units_served"], v["units_recorded"], "{tag}: {v:#}");
        let replayed = callback_values(&out.join("mixer.jsonl"), "timer", "out");
        // The last chunk can hold only records after the last /out (the
        // final /clock takes and the shutdown flush), so a cut there may
        // still replay every /out; an earlier cut cannot.
        assert!(
            !replayed.is_empty() && replayed.len() <= recorded.len(),
            "{tag}: {} replayed /out values",
            replayed.len()
        );
        if tag != "boundary-last" {
            assert!(
                replayed.len() < recorded.len(),
                "{tag}: {} replayed /out values, the cut lost some",
                replayed.len()
            );
        }
        assert_eq!(
            replayed[..],
            recorded[..replayed.len()],
            "{tag}: the replayed /out values are the recording's first ones"
        );
        results.push(format!(
            "{tag}@{cut}: {windows} windows, {} /out, {} inputs and {} outputs past the end",
            replayed.len(),
            v["inputs_past_log_end"],
            v["outputs_past_log_end"]
        ));
    }
    println!(
        "REPLAY torn: {} chunks, log {} bytes; {}",
        chunks.len(),
        full.len(),
        results.join("; ")
    );
}

#[test]
#[ignore = "needs ROS 2 with librmw_cerulion.so staged; see replay_harness"]
fn a_publishing_thread_outside_every_executor_is_refused() {
    preconditions();
    let mut run = Run::new("side-thread");
    let env = [("MIXER_SIDE_THREAD", "1".to_string())];
    let rec = record_probe(&mut run, "mixer", mixer(), &env, "mixer", 1000);
    let (out, _) = replay(&mut run, "mixer", mixer(), &env, &rec.log, "rep");
    let v = verdict(&rec.log);
    assert_eq!(v["verdict"], "refused", "{v:#}");
    let why = v["refused"].as_str().unwrap_or_default();
    assert!(
        why.contains("publishing thread outside every executor, not replayable"),
        "{why}"
    );
    assert!(
        why.contains("/side") && why.contains("(nodes mixer)"),
        "{why}"
    );
    assert!(v["outputs_unordered"].as_u64().unwrap() > 0, "{v:#}");
    assert_eq!(
        v["refused_at"], "end",
        "the process ran to the end first: {v:#}"
    );
    assert!(
        stderr_of(&out, "mixer").contains("publishing thread outside every executor"),
        "named on stderr"
    );
    // The control: the same graph without the side thread is identical.
    let mut run = Run::new("side-thread-control");
    let rec = record_probe(&mut run, "mixer", mixer(), &[], "mixer", 1000);
    replay(&mut run, "mixer", mixer(), &[], &rec.log, "rep");
    let c = verdict(&rec.log);
    assert_eq!(c["verdict"], "identical", "{c:#}");
    assert_eq!(c["outputs_unordered"], 0, "{c:#}");
    println!(
        "REPLAY side thread: refused ({} unordered outputs); control identical",
        v["outputs_unordered"]
    );
}

#[test]
#[ignore = "needs ROS 2 with librmw_cerulion.so staged; see replay_harness"]
fn a_serialized_take_is_refused_under_its_own_name() {
    preconditions();
    let mut run = Run::new("raw");
    let env = [("MIXER_RAW", "1".to_string())];
    let rec = record_probe(&mut run, "mixer", mixer(), &env, "mixer", 1000);
    assert!(
        process_channel_contains(&rec.log, "serialized take, not replayable"),
        "the log is marked under the serialized-take cause"
    );
    assert!(
        !process_channel_contains(&rec.log, "MultiThreadedExecutor"),
        "and not under the multi-threaded one"
    );
    let out = run.path("rep");
    run.spawn(
        "mixer",
        &mixer(),
        &[
            ("MIXER_RAW", "1".to_string()),
            ("CERULION_RMW_REPLAY", rec.log.display().to_string()),
        ],
        &out,
    );
    let st = run
        .wait_exit("mixer", Duration::from_secs(60))
        .expect("a refused replay exits by itself");
    assert!(!st.success(), "a refused replay exits nonzero: {st:?}");
    let err = stderr_of(&out, "mixer");
    assert!(err.contains("serialized take, not replayable"), "{err}");
    assert!(!err.contains("MultiThreadedExecutor"), "{err}");
    let v = verdict(&rec.log);
    assert_eq!(v["verdict"], "refused", "{v:#}");
    assert_eq!(
        v["refused_at"], "start",
        "refused before the node ran: {v:#}"
    );
    println!("REPLAY serialized take: refused as `serialized take`");
}

#[test]
#[ignore = "needs ROS 2 with librmw_cerulion.so staged; see replay_harness"]
fn a_replay_that_stops_taking_is_never_identical() {
    preconditions();
    let mut run = Run::new("stops-taking");
    let rec = record_probe(&mut run, "mixer", mixer(), &[], "mixer", 1000);
    let recorded_a = callbacks(&rec.out.join("mixer.jsonl"))
        .iter()
        .filter(|(cb, _)| cb == "a")
        .count();
    assert!(recorded_a > 40, "the recording took /a {recorded_a} times");
    let out = run.path("rep");
    let _ = std::fs::remove_file(format!("{}.verdict.json", rec.log.display()));
    run.spawn(
        "mixer",
        &mixer(),
        &[
            ("MIXER_EXIT_AFTER_A", "40".to_string()),
            ("CERULION_RMW_REPLAY", rec.log.display().to_string()),
        ],
        &out,
    );
    run.wait_exit("mixer", Duration::from_secs(60))
        .expect("the replay exits by itself after its 40th /a callback");
    let v = verdict(&rec.log);
    assert_ne!(v["verdict"], "identical", "{v:#}");
    assert!(
        ["replay_stopped_early", "diverged"].contains(&v["verdict"].as_str().unwrap_or_default()),
        "{v:#}"
    );
    let not_replayed = v["takes_not_replayed"].as_u64().unwrap();
    assert!(not_replayed > 0, "the takes never made are counted: {v:#}");
    let replayed_a = callbacks(&out.join("mixer.jsonl"))
        .iter()
        .filter(|(cb, _)| cb == "a")
        .count();
    assert_eq!(replayed_a, 40, "the replay stopped where it was told to");
    println!(
        "REPLAY stops taking: {} with {not_replayed} takes not replayed ({recorded_a} /a recorded, 40 replayed)",
        v["verdict"]
    );
}

#[test]
#[ignore = "needs ROS 2 with librmw_cerulion.so staged; see replay_harness"]
fn a_killed_replay_never_leaves_an_earlier_verdict() {
    preconditions();
    let mut run = Run::new("killed");
    let rec = record_probe(&mut run, "mixer", mixer(), &[], "mixer", 1000);
    replay(&mut run, "mixer", mixer(), &[], &rec.log, "rep1");
    assert_eq!(
        verdict(&rec.log)["verdict"],
        "identical",
        "the first replay"
    );
    let out = run.path("rep2");
    run.spawn(
        "mixer",
        &mixer(),
        &[
            ("MIXER_HANG_AFTER_A", "5".to_string()),
            ("CERULION_RMW_REPLAY", rec.log.display().to_string()),
        ],
        &out,
    );
    let deadline = std::time::Instant::now() + Duration::from_secs(60);
    let a_taken = |out: &PathBuf| {
        callbacks(&out.join("mixer.jsonl"))
            .iter()
            .filter(|(cb, _)| cb == "a")
            .count()
    };
    while a_taken(&out) < 5 {
        assert!(
            std::time::Instant::now() < deadline,
            "the replay never reached its 5th /a callback: {}",
            stderr_of(&out, "mixer")
        );
        std::thread::sleep(Duration::from_millis(50));
    }
    let st = run.kill("mixer");
    assert!(!st.success(), "killed: {st:?}");
    let v = verdict(&rec.log);
    assert_eq!(
        v["verdict"], "running",
        "a killed replay leaves `running`, never the earlier verdict: {v:#}"
    );
    println!("REPLAY killed: verdict `running` after SIGKILL ({st:?})");
}

fn svc(role: &str) -> Vec<String> {
    let mut v = vec!["python3".to_string(), script("svc.py"), role.to_string()];
    v.extend(SIM.iter().map(|s| s.to_string()));
    v
}

#[test]
#[ignore = "needs ROS 2 with librmw_cerulion.so staged; see replay_harness"]
fn an_rclpy_service_pair_replays_identically_on_both_sides() {
    preconditions();
    let ok = Command::new("python3")
        .args(["-c", "import example_interfaces.srv"])
        .status()
        .map(|s| s.success())
        .unwrap_or(false);
    assert!(ok, "python3 cannot import example_interfaces: {RECIPE}");
    let mut run = Run::new("svc-py");
    let out = run.path("rec");
    // One log directory per process: both logs carry the stem `svc`.
    let logdir = |role: &str| run.path(&format!("rec/log-{role}"));
    let (server_dir, client_dir) = (logdir("server"), logdir("client"));
    run.spawn(
        "adder",
        &svc("server"),
        &[(
            "CERULION_RMW_DELIVERY_LOG",
            server_dir.display().to_string(),
        )],
        &out,
    );
    run.spawn(
        "asker",
        &svc("client"),
        &[(
            "CERULION_RMW_DELIVERY_LOG",
            client_dir.display().to_string(),
        )],
        &out,
    );
    std::thread::sleep(Duration::from_secs(3));
    run.spawn(
        "clockpub",
        &["python3".to_string(), script("clockpub.py")],
        &[("TICKS", "2000".into()), ("CLOCK_PERIOD", "0.0005".into())],
        &out,
    );
    assert!(
        wait_for_file(&out.join("clockpub.done"), Duration::from_secs(120)),
        "clockpub never finished: {}",
        stderr_of(&out, "clockpub")
    );
    std::thread::sleep(Duration::from_secs(1));
    run.stop("asker");
    run.stop("adder");
    run.stop("clockpub");
    let asked = callbacks(&out.join("asker.jsonl"));
    let added = callbacks(&out.join("adder.jsonl"));
    let responses = asked.iter().filter(|(cb, _)| cb == "resp").count();
    assert_eq!(responses, 30, "the recording got every response: {asked:?}");
    assert_eq!(added.len(), 30, "the server served every request");
    let mut lines = Vec::new();
    for (name, role, recorded, dir) in [
        ("adder", "server", &added, &server_dir),
        ("asker", "client", &asked, &client_dir),
    ] {
        let log = find_log(dir, "svc");
        for i in 1..=2 {
            let (rout, _) = replay(&mut run, name, svc(role), &[], &log, &format!("{name}{i}"));
            assert_eq!(
                &callbacks(&rout.join(format!("{name}.jsonl"))),
                recorded,
                "{name} replay {i}: every callback as recorded"
            );
            let v = verdict(&log);
            assert_eq!(v["verdict"], "identical", "{name} replay {i}: {v:#}");
            assert!(v["outputs_compared"].as_u64().unwrap() >= 30, "{v:#}");
            assert_eq!(v["takes_not_replayed"], 0, "{v:#}");
            lines.push(format!(
                "{name} {i}: {} outputs compared",
                v["outputs_compared"]
            ));
        }
    }
    println!("REPLAY rclpy services: identical; {}", lines.join(", "));
}

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

#[test]
#[ignore = "needs ROS 2 with librmw_cerulion.so staged; see replay_harness"]
fn the_rclcpp_add_two_ints_pair_replays_identically_on_both_sides() {
    preconditions();
    let mut run = Run::new("svc-cpp");
    let out = run.path("rec");
    let logdir = run.path("rec/log");
    let env = [("CERULION_RMW_DELIVERY_LOG", logdir.display().to_string())];
    run.spawn("server", &demo("add_two_ints_server"), &env, &out);
    std::thread::sleep(Duration::from_secs(2));
    run.spawn("client", &demo("add_two_ints_client"), &env, &out);
    let st = run
        .wait_exit("client", Duration::from_secs(30))
        .expect("the stock client exits after its one response");
    assert!(
        st.success(),
        "client exited {st:?}: {}",
        stderr_of(&out, "client")
    );
    std::thread::sleep(Duration::from_millis(500));
    run.stop("server");
    let result = "Result of add_two_ints: 5";
    assert!(
        stderr_of(&out, "client").contains(result),
        "{}",
        stderr_of(&out, "client")
    );
    assert!(stderr_of(&out, "server").contains("Incoming request"));
    let server_log = find_log(&logdir, "add_two_ints_server");
    let client_log = find_log(&logdir, "add_two_ints_client");
    let (rout, _) = replay(
        &mut run,
        "server",
        demo("add_two_ints_server"),
        &[],
        &server_log,
        "srv",
    );
    let v = verdict(&server_log);
    assert_eq!(v["verdict"], "identical", "server: {v:#}");
    assert!(
        v["outputs_compared"].as_u64().unwrap() >= 1,
        "the response compared: {v:#}"
    );
    assert!(
        stderr_of(&rout, "server").contains("a: 2 b: 3"),
        "{}",
        stderr_of(&rout, "server")
    );
    let (rout, _) = replay(
        &mut run,
        "client",
        demo("add_two_ints_client"),
        &[],
        &client_log,
        "cli",
    );
    let c = verdict(&client_log);
    assert_eq!(c["verdict"], "identical", "client: {c:#}");
    // The stock client sends its one request from main before its first
    // rmw_wait (service availability is answered from the recording), so the
    // request is construction time: counted, not compared.
    assert!(
        c["outputs_before_first_event"].as_u64().unwrap() >= 1,
        "the request counted: {c:#}"
    );
    assert_eq!(c["takes_not_replayed"], 0, "the response was taken: {c:#}");
    assert!(
        stderr_of(&rout, "client").contains(result),
        "the replayed client got the recorded response: {}",
        stderr_of(&rout, "client")
    );
    println!(
        "REPLAY rclcpp services: server {} / client {} outputs compared, identical",
        v["outputs_compared"], c["outputs_compared"]
    );
}

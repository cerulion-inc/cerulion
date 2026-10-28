// SPDX-License-Identifier: AGPL-3.0-only
//! Shared harness of the exact-replay tests (`rmw_replay_*_test.rs`).
//!
//! The tests run REAL ROS 2 processes (`python3` rclpy nodes from
//! `tests/fixtures/replay_probe/`, and `demo_nodes_cpp` executables) over the
//! staged `librmw_cerulion.so`, record one of them with
//! `CERULION_RMW_DELIVERY_LOG`, replay it alone with `CERULION_RMW_REPLAY`,
//! and judge the replay by the node's own callback log and the rmw's verdict
//! file. They are `#[ignore]`: a plain `cargo test` has no ROS 2 to run them
//! against. Recipe (inside a ROS 2 environment, Humble measured):
//!
//! ```bash
//! set +u; source /opt/ros/<distro>/setup.bash
//! cargo build --release -p rmw_cerulion
//! PREFIX=/tmp/rmw_prefix; mkdir -p "$PREFIX/lib"
//! cp target/release/librmw_cerulion.so "$PREFIX/lib/"
//! export AMENT_PREFIX_PATH="$PREFIX:$AMENT_PREFIX_PATH"
//! export LD_LIBRARY_PATH="$PREFIX/lib:$LD_LIBRARY_PATH"
//! export RMW_IMPLEMENTATION=rmw_cerulion
//! cargo test --release -p rmw_cerulion --test rmw_replay_rclpy_probe_test -- --ignored --test-threads=1
//! ```
//!
//! Every run has its own iceoryx2 root and prefix (a `config/iceoryx2.toml` in
//! the run's working directory, which iceoryx2 reads first), so runs never see
//! each other's services; the run's `/dev/shm` segments are removed when it
//! ends. Every child is started and stopped by its own pid; SIGINT is reset
//! to its default disposition in the child, because a test binary launched
//! from a non-interactive shell's background job inherits SIGINT ignored.

#![allow(dead_code)]

use std::io::Read;
use std::os::unix::process::CommandExt;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, ExitStatus, Stdio};
use std::time::{Duration, Instant};

/// The precondition recipe, embedded in every precondition panic.
pub const RECIPE: &str = "\
replay test PRECONDITION failed. Run inside a ROS 2 environment with \
librmw_cerulion.so staged:\n  \
set +u; source /opt/ros/<distro>/setup.bash\n  \
cargo build --release -p rmw_cerulion && PREFIX=/tmp/rmw_prefix && mkdir -p $PREFIX/lib && \
cp target/release/librmw_cerulion.so $PREFIX/lib/\n  \
export AMENT_PREFIX_PATH=$PREFIX:$AMENT_PREFIX_PATH LD_LIBRARY_PATH=$PREFIX/lib:$LD_LIBRARY_PATH \
RMW_IMPLEMENTATION=rmw_cerulion\n  \
cargo test --release -p rmw_cerulion --test <this test> -- --ignored --test-threads=1";

/// Panic with the recipe unless the environment can run ROS 2 processes on
/// the staged rmw.
pub fn preconditions() {
    assert_eq!(
        std::env::var("RMW_IMPLEMENTATION").ok().as_deref(),
        Some("rmw_cerulion"),
        "{RECIPE}"
    );
    assert!(std::env::var_os("AMENT_PREFIX_PATH").is_some(), "{RECIPE}");
    assert!(std::env::var_os("ROS_DISTRO").is_some(), "{RECIPE}");
    for v in ["CERULION_RMW_DELIVERY_LOG", "CERULION_RMW_REPLAY"] {
        assert!(
            std::env::var_os(v).is_none(),
            "{v} is set in the test's environment; the harness sets it per child"
        );
    }
    let ok = Command::new("python3")
        .args(["-c", "import rclpy, std_msgs.msg, rosgraph_msgs.msg"])
        .status()
        .map(|s| s.success())
        .unwrap_or(false);
    assert!(ok, "python3 cannot import rclpy: {RECIPE}");
}

/// The fixture directory.
pub fn fixtures() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/replay_probe")
}

/// A fixture script's absolute path, as a string.
pub fn script(name: &str) -> String {
    fixtures().join(name).display().to_string()
}

/// The ROS-time args every probe node runs with.
pub const SIM: [&str; 3] = ["--ros-args", "-p", "use_sim_time:=true"];

/// One isolated run directory with its own iceoryx2 root.
pub struct Run {
    pub dir: PathBuf,
    /// The run's iceoryx2 root. Short and under /tmp: iceoryx2 names unix
    /// sockets below it, and a socket path has a 107-byte limit.
    iox_root: PathBuf,
    prefix: String,
    children: Vec<(String, Child)>,
}

static RUN_COUNTER: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);

impl Run {
    pub fn new(tag: &str) -> Run {
        let n = RUN_COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let base = std::env::var_os("CERULION_REPLAY_TEST_DIR")
            .map(PathBuf::from)
            .unwrap_or_else(std::env::temp_dir);
        let dir = base.join(format!("rdl-{}-{}-{}", std::process::id(), n, tag));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(dir.join("config")).expect("run dir");
        let iox_root = PathBuf::from(format!("/tmp/rdl{}x{}", std::process::id(), n));
        let _ = std::fs::remove_dir_all(&iox_root);
        std::fs::create_dir_all(&iox_root).expect("iox root");
        let prefix = format!("rdlt{}x{}_", std::process::id(), n);
        std::fs::write(
            dir.join("config/iceoryx2.toml"),
            format!(
                "[global]\nroot-path = \"{}/\"\nprefix = \"{prefix}\"\n",
                iox_root.display()
            ),
        )
        .expect("iceoryx2 config");
        Run {
            dir,
            iox_root,
            prefix,
            children: Vec::new(),
        }
    }

    pub fn path(&self, rel: &str) -> PathBuf {
        self.dir.join(rel)
    }

    /// Start a child in the run directory. `out` is its `PROBE_OUT`.
    pub fn spawn(&mut self, name: &str, argv: &[String], env: &[(&str, String)], out: &Path) {
        std::fs::create_dir_all(out).expect("probe out dir");
        let stdout = std::fs::File::create(out.join(format!("{name}.stdout"))).unwrap();
        let stderr = std::fs::File::create(out.join(format!("{name}.stderr"))).unwrap();
        let mut cmd = Command::new(&argv[0]);
        cmd.args(&argv[1..])
            .current_dir(&self.dir)
            .env("PROBE_OUT", out)
            .env("PYTHONUNBUFFERED", "1")
            .env(
                "PYTHONPATH",
                prepend_env("PYTHONPATH", &fixtures().display().to_string()),
            )
            .env("ROS_LOCALHOST_ONLY", "1")
            .env_remove("CERULION_RMW_DELIVERY_LOG")
            .env_remove("CERULION_RMW_REPLAY")
            .stdin(Stdio::null())
            .stdout(stdout)
            .stderr(stderr);
        for (k, v) in env {
            cmd.env(k, v);
        }
        // SAFETY: async-signal-safe: one signal(2) call before exec.
        unsafe {
            cmd.pre_exec(|| {
                libc::signal(libc::SIGINT, libc::SIG_DFL);
                Ok(())
            });
        }
        let child = cmd
            .spawn()
            .unwrap_or_else(|e| panic!("spawn {argv:?}: {e}"));
        self.children.push((name.to_string(), child));
    }

    /// Whether the named child is still running.
    pub fn running(&mut self, name: &str) -> bool {
        self.children
            .iter_mut()
            .find(|(n, _)| n == name)
            .map(|(_, c)| matches!(c.try_wait(), Ok(None)))
            .unwrap_or(false)
    }

    /// Stop one child (SIGINT, then SIGTERM, then SIGKILL) and return its exit
    /// status.
    pub fn stop(&mut self, name: &str) -> ExitStatus {
        let i = self
            .children
            .iter()
            .position(|(n, _)| n == name)
            .unwrap_or_else(|| panic!("no child {name}"));
        let (_, mut child) = self.children.remove(i);
        stop_child(&mut child)
    }

    /// Wait for a child to exit by itself (bounded), then return its status.
    pub fn wait_exit(&mut self, name: &str, timeout: Duration) -> Option<ExitStatus> {
        let deadline = Instant::now() + timeout;
        loop {
            let i = self.children.iter().position(|(n, _)| n == name)?;
            if let Ok(Some(st)) = self.children[i].1.try_wait() {
                self.children.remove(i);
                return Some(st);
            }
            if Instant::now() >= deadline {
                return None;
            }
            std::thread::sleep(Duration::from_millis(50));
        }
    }

    pub fn stop_all(&mut self) {
        while let Some((_, mut c)) = self.children.pop() {
            stop_child(&mut c);
        }
    }
}

impl Drop for Run {
    fn drop(&mut self) {
        self.stop_all();
        // This run's iceoryx2 segments only (its own prefix).
        if let Ok(rd) = std::fs::read_dir("/dev/shm") {
            for e in rd.flatten() {
                if e.file_name().to_string_lossy().starts_with(&self.prefix) {
                    let _ = std::fs::remove_file(e.path());
                }
            }
        }
        let _ = std::fs::remove_dir_all(&self.iox_root);
        if std::env::var_os("CERULION_REPLAY_TEST_KEEP").is_none() {
            let _ = std::fs::remove_dir_all(&self.dir);
        }
    }
}

fn stop_child(child: &mut Child) -> ExitStatus {
    let pid = child.id() as libc::pid_t;
    for (sig, grace) in [
        (libc::SIGINT, Duration::from_secs(10)),
        (libc::SIGTERM, Duration::from_secs(5)),
        (libc::SIGKILL, Duration::from_secs(5)),
    ] {
        if let Ok(Some(st)) = child.try_wait() {
            return st;
        }
        // SAFETY: signals this harness's own child by pid.
        unsafe {
            libc::kill(pid, sig);
        }
        let deadline = Instant::now() + grace;
        while Instant::now() < deadline {
            if let Ok(Some(st)) = child.try_wait() {
                return st;
            }
            std::thread::sleep(Duration::from_millis(50));
        }
    }
    child.wait().expect("reap child")
}

fn prepend_env(var: &str, entry: &str) -> String {
    match std::env::var(var) {
        Ok(v) if !v.is_empty() => format!("{entry}:{v}"),
        _ => entry.to_string(),
    }
}

/// Wait for `path` to exist (bounded). Returns whether it appeared.
pub fn wait_for_file(path: &Path, timeout: Duration) -> bool {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        if path.exists() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    path.exists()
}

/// The `out` values of one kind of callback in a node's callback log.
pub fn callback_values(log: &Path, cb: &str, field: &str) -> Vec<String> {
    let text = std::fs::read_to_string(log).unwrap_or_default();
    text.lines()
        .filter_map(|l| serde_json::from_str::<serde_json::Value>(l).ok())
        .filter(|v| v["cb"] == cb)
        .filter_map(|v| v[field].as_str().map(str::to_string))
        .collect()
}

/// Every callback of a node's log, as `(cb, value)` in order.
pub fn callbacks(log: &Path) -> Vec<(String, String)> {
    let text = std::fs::read_to_string(log).unwrap_or_default();
    text.lines()
        .filter_map(|l| serde_json::from_str::<serde_json::Value>(l).ok())
        .map(|v| {
            let cb = v["cb"].as_str().unwrap_or_default().to_string();
            let val = v["out"]
                .as_str()
                .or_else(|| v["inp"].as_str())
                .unwrap_or_default()
                .to_string();
            (cb, val)
        })
        .collect()
}

/// The single delivery log in `dir` whose name starts with `stem-`.
pub fn find_log(dir: &Path, stem: &str) -> PathBuf {
    let mut found: Vec<PathBuf> = std::fs::read_dir(dir)
        .unwrap_or_else(|e| panic!("read {}: {e}", dir.display()))
        .flatten()
        .map(|e| e.path())
        .filter(|p| {
            let n = p.file_name().unwrap().to_string_lossy().to_string();
            n.starts_with(&format!("{stem}-")) && n.ends_with(".rdl.mcap")
        })
        .collect();
    assert_eq!(
        found.len(),
        1,
        "exactly one {stem} delivery log in {}: {found:?}",
        dir.display()
    );
    found.remove(0)
}

/// The verdict of a replayed log.
pub fn verdict(log: &Path) -> serde_json::Value {
    let p = PathBuf::from(format!("{}.verdict.json", log.display()));
    let text =
        std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("verdict {}: {e}", p.display()));
    serde_json::from_str(&text).unwrap_or_else(|e| panic!("verdict {}: {e}: {text}", p.display()))
}

/// The verdict file's raw bytes.
pub fn verdict_bytes(log: &Path) -> Vec<u8> {
    std::fs::read(format!("{}.verdict.json", log.display())).expect("verdict bytes")
}

/// The records of one delivery-log channel, as raw payloads.
pub fn channel_payloads(log: &Path, channel: &str) -> Vec<Vec<u8>> {
    let reader = cerulion_bag::BagReader::open(log).expect("open delivery log");
    let (msgs, _) = reader.recover_messages().expect("read delivery log");
    msgs.into_iter()
        .filter(|m| m.topic == channel)
        .map(|m| m.data)
        .collect()
}

/// Whether any record on the log's process channel carries `needle`.
pub fn process_channel_contains(log: &Path, needle: &str) -> bool {
    channel_payloads(log, "rdl/process")
        .iter()
        .any(|p| String::from_utf8_lossy(p).contains(needle))
}

/// A child's captured stderr.
pub fn stderr_of(out: &Path, name: &str) -> String {
    let mut s = String::new();
    if let Ok(mut f) = std::fs::File::open(out.join(format!("{name}.stderr"))) {
        let _ = f.read_to_string(&mut s);
    }
    s
}

/// A child's captured stdout.
pub fn stdout_of(out: &Path, name: &str) -> String {
    std::fs::read_to_string(out.join(format!("{name}.stdout"))).unwrap_or_default()
}

/// Strings of one argv.
pub fn argv(parts: &[&str]) -> Vec<String> {
    parts.iter().map(|s| s.to_string()).collect()
}

/// The probe graph: `mixer`-like node under test recorded with the delivery
/// log, plus `talker_a` and `clockpub` live. Returns the log path.
pub struct Recorded {
    pub log: PathBuf,
    pub out: PathBuf,
}

/// Record the node `node_argv` (named `name`, logging `<name>.jsonl`) on the
/// probe graph: the node and `talker_a` start, `clockpub` starts after both
/// matched `/clock`, ticks `ticks` times at 2 kHz, and every process is
/// stopped by SIGINT once it is done.
pub fn record_probe(
    run: &mut Run,
    name: &str,
    node_argv: Vec<String>,
    node_env: &[(&str, String)],
    log_stem: &str,
    ticks: u32,
) -> Recorded {
    let out = run.path("rec");
    let logdir = run.path("rec/log");
    let mut env: Vec<(&str, String)> =
        vec![("CERULION_RMW_DELIVERY_LOG", logdir.display().to_string())];
    env.extend(node_env.iter().cloned());
    run.spawn(name, &node_argv, &env, &out);
    let mut talker = argv(&["python3"]);
    talker.push(script("talker.py"));
    talker.extend(SIM.iter().map(|s| s.to_string()));
    run.spawn("talker_a", &talker, &[], &out);
    std::thread::sleep(Duration::from_secs(3));
    let clockpub = vec!["python3".to_string(), script("clockpub.py")];
    run.spawn(
        "clockpub",
        &clockpub,
        &[
            ("TICKS", ticks.to_string()),
            ("CLOCK_PERIOD", "0.0005".into()),
        ],
        &out,
    );
    assert!(
        wait_for_file(&out.join("clockpub.done"), Duration::from_secs(180)),
        "clockpub never finished its {ticks} ticks; stderr: {}",
        stderr_of(&out, "clockpub")
    );
    std::thread::sleep(Duration::from_secs(1));
    let st = run.stop(name);
    run.stop("talker_a");
    run.stop("clockpub");
    let log = find_log(&logdir, log_stem);
    assert!(
        st.success() || st.code().is_none() || st.code() == Some(0),
        "{name} exited {st:?}; stderr: {}",
        stderr_of(&out, name)
    );
    Recorded { log, out }
}

/// Replay `node_argv` alone against `log`. Waits for the verdict file, then
/// stops the process. Returns the probe out dir and the exit status.
pub fn replay(
    run: &mut Run,
    name: &str,
    node_argv: Vec<String>,
    node_env: &[(&str, String)],
    log: &Path,
    tag: &str,
) -> (PathBuf, ExitStatus) {
    let out = run.path(tag);
    let verdict = PathBuf::from(format!("{}.verdict.json", log.display()));
    let _ = std::fs::remove_file(&verdict);
    let mut env: Vec<(&str, String)> = vec![("CERULION_RMW_REPLAY", log.display().to_string())];
    env.extend(node_env.iter().cloned());
    run.spawn(name, &node_argv, &env, &out);
    let deadline = Instant::now() + Duration::from_secs(120);
    while Instant::now() < deadline && !verdict.exists() && run.running(name) {
        std::thread::sleep(Duration::from_millis(50));
    }
    // The verdict is written once the log is exhausted; give the process a
    // moment to finish its last callbacks before it is stopped.
    std::thread::sleep(Duration::from_millis(500));
    let st = run.stop(name);
    assert!(
        verdict.exists(),
        "replay of {} wrote no verdict; stderr: {}",
        log.display(),
        stderr_of(&out, name)
    );
    (out, st)
}

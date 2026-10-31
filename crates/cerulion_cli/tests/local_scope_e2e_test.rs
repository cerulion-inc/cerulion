// SPDX-License-Identifier: AGPL-3.0-only
//! Real-binary local-scope refusals and precedence over network locators.
//! Each spawn reads a private shared-memory registry and Cerulion home.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

fn registry_config(root: &Path) -> PathBuf {
    let config = root.join("config");
    std::fs::create_dir_all(&config).unwrap();
    let registry = root.join("shm");
    let path = registry
        .to_string_lossy()
        .replace('\\', "\\\\")
        .replace('"', "\\\"");
    let prefix = root.file_name().unwrap().to_str().unwrap();
    let file = config.join("iceoryx2.toml");
    std::fs::write(
        &file,
        format!("[global]\nroot-path = \"{path}\"\nprefix = \"scope_{prefix}_\"\n"),
    )
    .unwrap();
    file
}

fn run(root: &Path, args: &[&str], network: Option<&str>) -> Output {
    match network {
        Some(value) => run_with(root, args, &[("CERULION_NETWORK", value)]),
        None => run_with(root, args, &[]),
    }
}

/// Run the real CLI with `env` added over a scrubbed environment: no network
/// kill switch, no log filter, a private netd and vizd socket, and a netd
/// binary path that does not exist. On Unix a listener stands in for a running
/// netd at the private netd socket, and the run fails if the command connected
/// to it.
fn run_with(root: &Path, args: &[&str], env: &[(&str, &str)]) -> Output {
    #[cfg(unix)]
    let netd = StandInNetd::bind(&root.join("netd.sock"));
    let home = root.join("home");
    cerulion_cli_engine::auth::seed_logged_in_at(&home, "local-scope-test").unwrap();
    registry_config(root);
    let mut command = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    command
        .args(args)
        .current_dir(root)
        .env("CERULION_HOME", &home)
        .env("CERULION_NETD_SOCK", root.join("netd.sock"))
        .env("CERULION_NETD_BIN", root.join("absent-netd"))
        .env("CERULION_VIZD_SOCK", root.join("vizd.sock"))
        .env_remove("CERULION_NETWORK")
        .env_remove("CERULION_NETD_NETWORK")
        .env_remove("CERULION_VIZD_NETWORK")
        .env_remove("RUST_LOG")
        .env_remove("CERULION_LOGIN_GATE")
        .env("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:1");
    for (name, value) in env {
        command.env(name, value);
    }
    let output = command.output().expect("run the real CLI");
    #[cfg(unix)]
    match netd.listener.accept() {
        Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {}
        other => panic!(
            "a local command connected to the network daemon ({other:?}); stderr:\n{}",
            String::from_utf8_lossy(&output.stderr)
        ),
    }
    #[cfg(not(unix))]
    assert!(
        !root.join("netd.sock").exists(),
        "a local command created a network-daemon socket"
    );
    output
}

/// A listener at the private netd socket path, unlinked on drop so the next
/// run under the same root can bind it again.
#[cfg(unix)]
struct StandInNetd {
    listener: std::os::unix::net::UnixListener,
    path: PathBuf,
}

#[cfg(unix)]
impl StandInNetd {
    fn bind(path: &Path) -> Self {
        let listener =
            std::os::unix::net::UnixListener::bind(path).expect("bind the stand-in netd socket");
        listener.set_nonblocking(true).unwrap();
        Self {
            listener,
            path: path.to_path_buf(),
        }
    }
}

#[cfg(unix)]
impl Drop for StandInNetd {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.path);
    }
}

/// The log filter that surfaces the two network markers: `cerulion_core`'s
/// DEBUG line when a zenoh session (and with it zenoh's scouting sockets)
/// opens, and the discovery ladder's INFO line when its mDNS browse, peer cache
/// and hostname rungs ran. A release build compiles the DEBUG line out
/// (`tracing/release_max_level_info`), so the tests that read it run in debug
/// builds only.
#[cfg(debug_assertions)]
const NETWORK_MARKER_FILTER: &str =
    "cerulion_core::transport::network=debug,cerulion_cli_engine::discovery_ladder=info";
#[cfg(debug_assertions)]
const SESSION_MARKER: &str = "zenoh session opened";
#[cfg(debug_assertions)]
const LADDER_MARKER: &str = "discovery ladder";

/// Each kill-switch setting a query verb honors, as the environment that sets it.
#[cfg(debug_assertions)]
const KILL_SWITCHES: [&[(&str, &str)]; 4] = [
    &[("CERULION_NETWORK", "off")],
    &[("CERULION_NETWORK", "typo")],
    &[("CERULION_NETD_NETWORK", "off")],
    &[
        ("CERULION_NETWORK", "off"),
        ("CERULION_NETD_NETWORK", "off"),
    ],
];

/// `switches` plus the network-marker log filter.
#[cfg(debug_assertions)]
fn with_markers<'a>(switches: &[(&'a str, &'a str)]) -> Vec<(&'a str, &'a str)> {
    let mut env = switches.to_vec();
    env.push(("RUST_LOG", NETWORK_MARKER_FILTER));
    env
}

#[test]
#[cfg(debug_assertions)]
fn topic_list_under_a_kill_switch_opens_no_zenoh_session_or_discovery_ladder() {
    let args = ["topic", "list", "--listen", "tcp/127.0.0.1:0"];
    // Control: with no switch the same command opens a session and runs the
    // ladder, so the markers' absence below is the switch's doing.
    let root = tempfile::tempdir().unwrap();
    let control = run_with(root.path(), &args, &[("RUST_LOG", NETWORK_MARKER_FILTER)]);
    let log = String::from_utf8_lossy(&control.stderr);
    assert!(control.status.success(), "{log}");
    assert!(
        log.contains(SESSION_MARKER),
        "control opened no session:\n{log}"
    );
    assert!(log.contains(LADDER_MARKER), "control ran no ladder:\n{log}");

    for switches in KILL_SWITCHES {
        let root = tempfile::tempdir().unwrap();
        let output = run_with(root.path(), &args, &with_markers(switches));
        let log = String::from_utf8_lossy(&output.stderr);
        assert!(output.status.success(), "{switches:?}: {log}");
        assert_eq!(
            String::from_utf8_lossy(&output.stdout),
            "No active local topics.\n",
            "{switches:?}"
        );
        assert!(!log.contains(SESSION_MARKER), "{switches:?}:\n{log}");
        assert!(!log.contains(LADDER_MARKER), "{switches:?}:\n{log}");
        assert!(!log.contains("remote:"), "{switches:?}:\n{log}");
    }
}

#[test]
#[cfg(debug_assertions)]
fn observers_under_the_netd_switch_refuse_without_a_session_or_daemon() {
    for verb in ["echo", "hz", "info", "type", "bw"] {
        let root = tempfile::tempdir().unwrap();
        let output = run_with(
            root.path(),
            &["topic", verb, "/local-scope-test/missing"],
            &with_markers(&[("CERULION_NETD_NETWORK", "off")]),
        );
        let error = String::from_utf8_lossy(&output.stderr);
        assert_eq!(output.status.code(), Some(1), "{verb}: {error}");
        assert!(
            error.contains(
                "not found locally, and remote discovery is disabled by \
                 CERULION_NETD_NETWORK=off; unset it to search robots on the LAN"
            ),
            "{verb}: {error}"
        );
        assert!(!error.contains(SESSION_MARKER), "{verb}: {error}");
        assert!(output.stdout.is_empty(), "{verb}");
    }
}

#[test]
#[cfg(debug_assertions)]
fn topic_find_under_a_kill_switch_reads_local_topics_only() {
    let args = [
        "topic",
        "find",
        "std_msgs/String",
        "--listen",
        "tcp/127.0.0.1:0",
    ];
    // Control: with no switch the same command opens a session and runs the
    // ladder, so the markers' absence below is the switch's doing.
    let root = tempfile::tempdir().unwrap();
    let control = run_with(root.path(), &args, &[("RUST_LOG", NETWORK_MARKER_FILTER)]);
    let log = String::from_utf8_lossy(&control.stderr);
    assert!(control.status.success(), "{log}");
    assert!(
        log.contains(SESSION_MARKER),
        "control opened no session:\n{log}"
    );
    assert!(log.contains(LADDER_MARKER), "control ran no ladder:\n{log}");

    // A genuine local `std_msgs/String` publisher in the runs' own registry,
    // sending for the whole loop, so an empty answer cannot pass: under each
    // switch the local scan must still find it.
    let root = short_tempdir();
    let topic = "/scope/find-local-string";
    let publisher = LocalStringPublisher::start(root.path(), topic);
    for switches in KILL_SWITCHES {
        let output = run_with(root.path(), &args, &with_markers(switches));
        let log = String::from_utf8_lossy(&output.stderr);
        assert!(output.status.success(), "{switches:?}: {log}");
        let stdout = String::from_utf8(output.stdout).unwrap();
        assert_eq!(stdout, format!("{topic}\n"), "{switches:?}:\n{log}");
        assert!(!log.contains(SESSION_MARKER), "{switches:?}:\n{log}");
        assert!(!log.contains(LADDER_MARKER), "{switches:?}:\n{log}");
    }
    assert!(publisher.stop() > 0, "the local publisher sent nothing");
}

/// A temporary root with a short path, so iceoryx2's event socket paths stay
/// below the Unix pathname length limit.
#[cfg(debug_assertions)]
fn short_tempdir() -> tempfile::TempDir {
    #[cfg(unix)]
    return tempfile::tempdir_in("/tmp").unwrap();
    #[cfg(not(unix))]
    return tempfile::tempdir().unwrap();
}

/// A local publisher of `std_msgs/String` frames on `topic` in the private
/// registry under `root`, sending one frame every 10 ms on its own thread
/// until [`stop`](Self::stop).
#[cfg(debug_assertions)]
struct LocalStringPublisher {
    stop: std::sync::Arc<std::sync::atomic::AtomicBool>,
    thread: std::thread::JoinHandle<u64>,
}

#[cfg(debug_assertions)]
impl LocalStringPublisher {
    fn start(root: &Path, topic: &str) -> Self {
        use cerulion_core::wire::MaxSliceLen;
        use cerulion_core::{TransportConfig, TransportManager, WireHeader};
        use std::sync::atomic::{AtomicBool, Ordering};
        let file = registry_config(root);
        let config =
            iceoryx2::config::Config::from_file(&file.to_str().unwrap().try_into().unwrap())
                .unwrap();
        let manager = TransportManager::init_for_test(TransportConfig::default(), config).unwrap();
        // A `std_msgs/String` frame carrying "x": the string's offset and
        // length, then its byte.
        let mut frame = vec![0u8; WireHeader::SIZE + 9];
        let mut header = WireHeader::new(
            cerulion_cli_engine::topic_cmd::STD_MSGS_STRING_SCHEMA_HASH,
            0,
            1_000,
        );
        header.total_size = frame.len() as u32;
        header.write_to_buf(&mut frame[..WireHeader::SIZE]);
        let payload = &mut frame[WireHeader::SIZE..];
        payload[0..4].copy_from_slice(&8u32.to_le_bytes());
        payload[4..8].copy_from_slice(&1u32.to_le_bytes());
        payload[8] = b'x';
        let mut publisher = manager
            .create_publisher(topic, MaxSliceLen::const_new(256), 0)
            .unwrap();
        let stop = std::sync::Arc::new(AtomicBool::new(false));
        let thread = {
            let stop = stop.clone();
            std::thread::spawn(move || {
                let _manager = manager;
                let mut sent = 0u64;
                while !stop.load(Ordering::Relaxed) {
                    publisher.publish_raw(&frame).unwrap();
                    sent += 1;
                    std::thread::sleep(std::time::Duration::from_millis(10));
                }
                sent
            })
        };
        Self { stop, thread }
    }

    /// Stop sending and return how many frames were sent.
    fn stop(self) -> u64 {
        self.stop.store(true, std::sync::atomic::Ordering::Relaxed);
        self.thread.join().unwrap()
    }
}

#[test]
#[cfg(debug_assertions)]
fn schema_info_under_a_kill_switch_opens_no_session_or_daemon() {
    for switches in KILL_SWITCHES {
        let root = tempfile::tempdir().unwrap();
        let output = run_with(
            root.path(),
            &["schema", "info", "local_scope_msgs/Missing"],
            &with_markers(switches),
        );
        let error = String::from_utf8_lossy(&output.stderr);
        assert!(!output.status.success(), "{switches:?}: {error}");
        assert!(
            error.contains("local_scope_msgs/Missing"),
            "{switches:?}: {error}"
        );
        assert!(!error.contains(SESSION_MARKER), "{switches:?}: {error}");
    }
}

#[test]
fn pair_under_the_network_kill_switch_refuses_before_discovery() {
    // Control: with no switch, an unpinned robot name is looked up over mDNS
    // and the miss names that lookup.
    let root = tempfile::tempdir().unwrap();
    let control = run_with(root.path(), &["pair", "scope-bot"], &[]);
    let error = String::from_utf8_lossy(&control.stderr);
    assert_eq!(control.status.code(), Some(1), "{error}");
    assert!(error.contains("did not answer mDNS"), "{error}");

    for value in ["off", "typo"] {
        let root = tempfile::tempdir().unwrap();
        let output = run_with(
            root.path(),
            &["pair", "scope-bot"],
            &[("CERULION_NETWORK", value)],
        );
        let error = String::from_utf8_lossy(&output.stderr);
        assert_eq!(output.status.code(), Some(1), "{value}: {error}");
        assert!(
            error.contains(&format!(
                "cerulion pair reaches the robot over the network, which CERULION_NETWORK={value}"
            )),
            "{value}: {error}"
        );
        assert!(
            error.contains("unset CERULION_NETWORK to pair"),
            "{value}: {error}"
        );
        assert!(!error.contains("mDNS"), "{value}: {error}");
    }
}

#[test]
#[cfg(unix)]
fn remote_viz_under_a_kill_switch_refuses_before_any_daemon() {
    for (name, value) in [
        ("CERULION_NETWORK", "off"),
        ("CERULION_NETWORK", "typo"),
        ("CERULION_NETD_NETWORK", "off"),
        ("CERULION_VIZD_NETWORK", "off"),
    ] {
        let root = tempfile::tempdir().unwrap();
        let output = run_with(
            root.path(),
            &["viz", "--robot", "scope-bot", "/scope/cloud"],
            &[(name, value)],
        );
        let error = String::from_utf8_lossy(&output.stderr);
        assert_eq!(output.status.code(), Some(1), "{name}={value}: {error}");
        assert!(
            error.contains("cerulion viz --robot scope-bot: remote visualization is disabled by"),
            "{name}={value}: {error}"
        );
        assert!(
            error.contains(&format!("unset {name}")),
            "{name}={value}: {error}"
        );
        assert!(output.stdout.is_empty(), "{name}={value}");
        assert!(
            !root.path().join("vizd.sock").exists(),
            "{name}={value}: a refused remote viz reached a vizd socket"
        );
    }
}

#[test]
fn local_observers_refuse_missing_topics_without_remote_search_or_daemon() {
    for verb in ["echo", "hz", "info", "type", "bw"] {
        let root = tempfile::tempdir().unwrap();
        let output = run(
            root.path(),
            &["topic", verb, "/local-scope-test/missing", "--local"],
            None,
        );
        let error = String::from_utf8_lossy(&output.stderr);
        assert_eq!(output.status.code(), Some(1), "{error}");
        assert!(error.contains("not found locally"), "{error}");
        assert!(error.contains("disabled by --local"), "{error}");
        assert!(!error.contains("on any discovered robot"), "{error}");
        assert!(output.stdout.is_empty());
    }
}

#[test]
fn local_list_and_legacy_alias_override_scan_and_explicit_locators() {
    for flag in ["--local", "--no-network"] {
        let root = tempfile::tempdir().unwrap();
        let output = run(
            root.path(),
            &[
                "topic",
                "list",
                flag,
                "--scan",
                "--connect",
                "tcp/127.0.0.1:1",
                "--listen",
                "tcp/127.0.0.1:0",
            ],
            None,
        );
        assert!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
        let listing = String::from_utf8_lossy(&output.stdout);
        assert!(listing == "No active local topics.\n", "{listing}");
        assert!(!listing.contains("remote:"), "{listing}");
        assert!(
            output.stderr.is_empty(),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
    }
}

#[test]
fn environment_kill_switch_also_suppresses_topic_list() {
    for value in ["off", " OFF ", "typo"] {
        let root = tempfile::tempdir().unwrap();
        let output = run(
            root.path(),
            &["topic", "list", "--scan", "--connect", "tcp/127.0.0.1:1"],
            Some(value),
        );
        let error = String::from_utf8_lossy(&output.stderr);
        assert!(output.status.success(), "{error}");
        let listing = String::from_utf8_lossy(&output.stdout);
        assert_eq!(listing, "No active local topics.\n", "{listing}");
        assert!(
            !error.contains("remote: discovery unavailable"),
            "the environment kill-switch must skip the query, not fail it: {error}"
        );
        if value == "typo" {
            assert!(error.contains("failing CLOSED"), "{error}");
        } else {
            assert!(error.is_empty(), "{error}");
        }
    }
}

#[test]
#[cfg(unix)]
fn unattributed_network_mirrors_stay_remote_in_real_local_list_and_observers() {
    use cerulion_core::wire::MaxSliceLen;
    use cerulion_core::{TransportConfig, TransportManager};
    for failed in [false, true] {
        // Keep event socket paths below the Unix pathname length limit.
        let root = tempfile::tempdir_in("/tmp").unwrap();
        let file = registry_config(root.path());
        let config =
            iceoryx2::config::Config::from_file(&file.to_str().unwrap().try_into().unwrap())
                .unwrap();
        let manager = TransportManager::init_for_test(TransportConfig::default(), config).unwrap();
        let topic = "/scope/remote-unattributed";
        let _injector = manager
            .create_remote_ingress_injector(topic, 0x1234, MaxSliceLen::const_new(256))
            .unwrap();
        if failed {
            assert!(manager
                .register_mirror_provenance(topic, &"x".repeat(257))
                .is_err());
        }
        let output = run(root.path(), &["topic", "list", "--local"], None);
        assert!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
        let listing = String::from_utf8(output.stdout).unwrap();
        assert!(listing.contains("REMOTE TOPICS"), "{listing}");
        assert!(listing.contains(topic), "{listing}");
        assert!(listing.contains("origin unavailable"), "{listing}");
        assert!(
            !listing
                .split("REMOTE TOPICS")
                .next()
                .unwrap()
                .contains(topic),
            "{listing}"
        );
        for verb in ["echo", "hz", "info", "type", "bw"] {
            let output = run(root.path(), &["topic", verb, topic, "--local"], None);
            let error = String::from_utf8_lossy(&output.stderr);
            assert_eq!(output.status.code(), Some(1), "{error}");
            assert!(
                error.contains("network mirror with origin unavailable"),
                "{error}"
            );
            assert!(output.stdout.is_empty());
        }
    }
}

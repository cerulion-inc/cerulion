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
    let (output, netd_contact) = run_watching_netd(root, args, env);
    if let Some(contact) = netd_contact {
        panic!(
            "a local command reached the network daemon ({contact}); stderr:\n{}",
            String::from_utf8_lossy(&output.stderr)
        );
    }
    output
}

/// [`run_with`]'s run, returning with the output how the command reached the
/// stand-in netd (`None` when it did not) instead of failing on it.
fn run_watching_netd(root: &Path, args: &[&str], env: &[(&str, &str)]) -> (Output, Option<String>) {
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
    let contact = match netd.listener.accept() {
        Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => None,
        other => Some(format!("connected: {other:?}")),
    };
    #[cfg(not(unix))]
    let contact = root
        .join("netd.sock")
        .exists()
        .then(|| "created a network-daemon socket".to_string());
    (output, contact)
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

/// The exact refusal `main.rs` prints for the hidden `cerulion pair` stub
/// (its `PAIR_UNAVAILABLE_MSG`), as the whole of stderr. Kept as a duplicate
/// literal on purpose: `cerulion_cli` has no lib target, so the duplicate is
/// what pins the shipped text, and an edit to either side fails here.
const PAIR_UNAVAILABLE: &str = "Error: code pairing is not available in this release; robots on \
                                your account are reached by signing in with `cerulion login`\n";

#[test]
fn hidden_pair_refuses_every_spelling_before_discovery_the_switch_and_the_gate() {
    let spellings: [&[&str]; 5] = [
        &["pair"],
        &["pair", "scope-bot"],
        &["pair", "scope-bot", "--code", "SWAN-42"],
        &["pair", "--eid", "ab", "--relay-disabled"],
        &["pair", "--help"],
    ];
    let quiet_iceoryx2 = ("IOX2_LOG_LEVEL", "error");
    for args in spellings {
        // A signed-in machine, with no switch and under two spellings of it:
        // the same refusal, and `run_with` fails the run had it reached netd.
        for network in [None, Some("off"), Some("typo")] {
            let root = tempfile::tempdir().unwrap();
            let mut env = vec![quiet_iceoryx2];
            if let Some(value) = network {
                env.push(("CERULION_NETWORK", value));
            }
            let output = run_with(root.path(), args, &env);
            let error = String::from_utf8_lossy(&output.stderr);
            assert_eq!(
                output.status.code(),
                Some(2),
                "{args:?} {network:?}: {error}"
            );
            assert_eq!(error, PAIR_UNAVAILABLE, "{args:?} {network:?}");
            assert!(output.stdout.is_empty(), "{args:?} {network:?}");
        }
        // A machine that never signed in, with no terminal: the login gate
        // would refuse with exit 7 here, so exit 2 proves the stub answers
        // above it.
        let root = tempfile::tempdir().unwrap();
        let fresh = root.path().join("never-signed-in");
        std::fs::create_dir_all(&fresh).unwrap();
        let fresh = fresh.to_str().unwrap();
        let output = run_with(
            root.path(),
            args,
            &[quiet_iceoryx2, ("CERULION_HOME", fresh)],
        );
        let error = String::from_utf8_lossy(&output.stderr);
        assert_eq!(
            output.status.code(),
            Some(2),
            "{args:?} never signed in: {error}"
        );
        assert_eq!(error, PAIR_UNAVAILABLE, "{args:?} never signed in");
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

/// A machine device key in the run's Cerulion home, and a script standing in
/// for `cerulion-connectd` that writes the returned marker file when it runs
/// and answers `{}`. Returns the script's path and the marker's.
#[cfg(unix)]
fn stand_in_connectd(root: &Path) -> (PathBuf, PathBuf) {
    use std::os::unix::fs::PermissionsExt;
    let home = root.join("home");
    std::fs::create_dir_all(&home).unwrap();
    let key = home.join("desk.key");
    std::fs::write(&key, [7u8; 32]).unwrap();
    std::fs::set_permissions(&key, std::fs::Permissions::from_mode(0o600)).unwrap();
    let marker = root.join("connectd-ran");
    let helper = root.join("connectd-helper");
    std::fs::write(
        &helper,
        format!(
            "#!/bin/sh\nprintf ran > '{}'\nprintf '{{}}'\n",
            marker.display()
        ),
    )
    .unwrap();
    std::fs::set_permissions(&helper, std::fs::Permissions::from_mode(0o755)).unwrap();
    (helper, marker)
}

#[test]
#[cfg(unix)]
fn robot_verbs_under_the_network_switch_refuse_before_the_helper_runs() {
    let robot = "ab".repeat(32);
    let verbs: [(&str, Vec<&str>); 3] = [
        (
            "topic-publish",
            vec![
                "topic",
                "pub",
                "/scope/cmd",
                "std_msgs/String",
                "{data: hi}",
            ],
        ),
        ("topic-snapshot", vec!["topic", "snapshot", "/scope/cam"]),
        (
            "service-call",
            vec!["service", "call", "/scope/trigger", "std_srvs/srv/Trigger"],
        ),
    ];
    for (verb, mut args) in verbs {
        args.extend(["--robot", robot.as_str()]);
        // Control: with no switch the CLI reaches the helper, so the marker's
        // absence below is the switch's doing.
        let root = tempfile::tempdir().unwrap();
        let (helper, marker) = stand_in_connectd(root.path());
        let helper = helper.to_str().unwrap();
        let control = run_with(root.path(), &args, &[("CERULION_CONNECTD_BIN", helper)]);
        assert!(
            marker.exists(),
            "{verb}: the control run did not reach the helper: {}",
            String::from_utf8_lossy(&control.stderr)
        );

        // `off`, and an unrecognized value, which fails closed: both refuse
        // and the refusal names the value.
        for (network, label) in [
            ("off", "CERULION_NETWORK=off"),
            (
                "typo",
                "CERULION_NETWORK=typo (an unrecognized value, which fails closed to local-only)",
            ),
        ] {
            let root = tempfile::tempdir().unwrap();
            let (helper, marker) = stand_in_connectd(root.path());
            let helper = helper.to_str().unwrap();
            let output = run_with(
                root.path(),
                &args,
                &[
                    ("CERULION_CONNECTD_BIN", helper),
                    ("CERULION_NETWORK", network),
                ],
            );
            let error = String::from_utf8_lossy(&output.stderr);
            assert!(
                !marker.exists(),
                "{verb} {network}: the helper ran: {error}"
            );
            assert_eq!(output.status.code(), Some(2), "{verb} {network}: {error}");
            assert!(
                error.contains(&format!(
                    "--robot {robot} reaches the robot over the network, which {label} \
                     disables; unset CERULION_NETWORK to call its {verb} verb"
                )),
                "{verb} {network}: {error}"
            );
            assert!(output.stdout.is_empty(), "{verb} {network}");
        }
    }
}

#[test]
#[cfg(unix)]
fn rcp_under_the_network_switch_refuses_before_the_helper_runs() {
    let robot = "ab".repeat(32);
    // Control: with no switch the CLI reaches the helper, so the marker's
    // absence below is the switch's doing.
    let root = tempfile::tempdir().unwrap();
    let (helper, marker) = stand_in_connectd(root.path());
    let helper = helper.to_str().unwrap();
    let control = run_with(
        root.path(),
        &["rcp", robot.as_str()],
        &[("CERULION_CONNECTD_BIN", helper)],
    );
    assert!(
        marker.exists(),
        "the control run did not reach the helper: {}",
        String::from_utf8_lossy(&control.stderr)
    );

    // `off` in the variable or the flag, and an unrecognized value, which
    // fails closed: each refuses and the refusal names the value.
    let unrecognized =
        "CERULION_NETWORK=typo (an unrecognized value, which fails closed to local-only)";
    for (flag, network, label, remedy) in [
        (
            None,
            Some("off"),
            "CERULION_NETWORK=off",
            "unset CERULION_NETWORK",
        ),
        (None, Some("typo"), unrecognized, "unset CERULION_NETWORK"),
        (Some("off"), None, "--network off", "drop --network"),
    ] {
        let root = tempfile::tempdir().unwrap();
        let (helper, marker) = stand_in_connectd(root.path());
        let helper = helper.to_str().unwrap();
        let mut args = vec!["rcp", robot.as_str()];
        if let Some(flag) = flag {
            args.extend(["--network", flag]);
        }
        let mut env = vec![("CERULION_CONNECTD_BIN", helper)];
        if let Some(network) = network {
            env.push(("CERULION_NETWORK", network));
        }
        let output = run_with(root.path(), &args, &env);
        let error = String::from_utf8_lossy(&output.stderr);
        assert!(!marker.exists(), "{label}: the helper ran: {error}");
        assert_eq!(output.status.code(), Some(2), "{label}: {error}");
        assert!(
            error.contains(&format!(
                "cerulion rcp reaches the robot over the network, which {label} disables; \
                 {remedy} to reach the robot's rcpd"
            )),
            "{label}: {error}"
        );
        assert!(output.stdout.is_empty(), "{label}");
    }
}

#[test]
#[cfg(all(unix, debug_assertions))]
fn topic_pub_under_a_kill_switch_offers_nothing_on_the_lan() {
    let args = [
        "topic",
        "pub",
        "/scope/pub-local",
        "std_msgs/String",
        "{data: hi}",
        "--wait-subscribers",
        "0",
    ];
    let published = "published: topic=/scope/pub-local type=std_msgs/String frames=1 \
                     subscribers=0\n";
    // Control: with no switch the publish registers the topic with netd, so
    // the stand-in netd is reached.
    let root = short_tempdir();
    let (control, contact) = run_watching_netd(root.path(), &args, &[]);
    let log = String::from_utf8_lossy(&control.stderr);
    assert!(contact.is_some(), "control reached no netd:\n{log}");
    assert_eq!(String::from_utf8_lossy(&control.stdout), published, "{log}");

    for switches in KILL_SWITCHES {
        let root = short_tempdir();
        let output = run_with(root.path(), &args, &with_markers(switches));
        let log = String::from_utf8_lossy(&output.stderr);
        assert!(output.status.success(), "{switches:?}: {log}");
        assert_eq!(
            String::from_utf8_lossy(&output.stdout),
            published,
            "{switches:?}"
        );
        assert!(!log.contains(SESSION_MARKER), "{switches:?}:\n{log}");
    }
}

#[test]
#[cfg(debug_assertions)]
fn topic_pub_of_an_unknown_type_under_a_kill_switch_asks_no_robot() {
    for switches in KILL_SWITCHES {
        let root = tempfile::tempdir().unwrap();
        let output = run_with(
            root.path(),
            &[
                "topic",
                "pub",
                "/scope/pub-missing",
                "scope_msgs/Missing",
                "{}",
            ],
            &with_markers(switches),
        );
        let error = String::from_utf8_lossy(&output.stderr);
        assert_eq!(output.status.code(), Some(2), "{switches:?}: {error}");
        assert!(
            error.contains("unknown message type 'scope_msgs/Missing'"),
            "{switches:?}: {error}"
        );
        assert!(
            !error.contains("no robot on the LAN served it"),
            "{switches:?}: {error}"
        );
        assert!(!error.contains(SESSION_MARKER), "{switches:?}: {error}");
        assert!(output.stdout.is_empty(), "{switches:?}");
    }
}

#[test]
#[cfg(debug_assertions)]
fn topic_snapshot_under_a_kill_switch_demands_no_remote_topic() {
    for switches in KILL_SWITCHES {
        let root = short_tempdir();
        let output = run_with(
            root.path(),
            &[
                "topic",
                "snapshot",
                "/scope/snap-missing",
                "--timeout",
                "0.3",
            ],
            &with_markers(switches),
        );
        let error = String::from_utf8_lossy(&output.stderr);
        assert_eq!(output.status.code(), Some(3), "{switches:?}: {error}");
        assert!(
            error.contains("'/scope/snap-missing' could not be opened within 0.3 s"),
            "{switches:?}: {error}"
        );
        assert!(
            error.contains("remote discovery is disabled by"),
            "{switches:?}: {error}"
        );
        assert!(!error.contains(SESSION_MARKER), "{switches:?}: {error}");
        assert!(!error.contains(LADDER_MARKER), "{switches:?}: {error}");
        assert!(output.stdout.is_empty(), "{switches:?}");
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

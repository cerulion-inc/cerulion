// SPDX-License-Identifier: AGPL-3.0-only
//! Prerequisite: `cargo build -p cerulion_cli`. Paired remoted wire/2, real
//! desk CLI, local iroh with relays disabled, independent frame oracles.

use super::*;
use cerulion_core::transport::demand_authorizer::AllowAllAuthorizer;
use cerulion_netd::account_controller::{AccountControllerConfig, AccountWanController};
use std::process::{Child, Command, Output, Stdio};

struct ChildGuard(Option<Child>);
impl std::ops::Deref for ChildGuard {
    type Target = Child;
    fn deref(&self) -> &Child {
        self.0.as_ref().unwrap()
    }
}
impl std::ops::DerefMut for ChildGuard {
    fn deref_mut(&mut self) -> &mut Child {
        self.0.as_mut().unwrap()
    }
}
impl ChildGuard {
    fn output(&mut self) -> Output {
        self.0.take().unwrap().wait_with_output().unwrap()
    }
}
impl Drop for ChildGuard {
    fn drop(&mut self) {
        if let Some(child) = &mut self.0 {
            let _ = child.kill();
            let _ = child.wait();
        }
    }
}

struct Producer {
    stop: Arc<AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
}
impl Producer {
    fn start(
        manager: Arc<TransportManager>,
        topic: String,
        hash: u64,
        sequence: u32,
        stamp: u64,
        value: f64,
    ) -> Self {
        let stop = Arc::new(AtomicBool::new(false));
        let stopping = stop.clone();
        let (ready, started) = std::sync::mpsc::channel();
        let thread = std::thread::spawn(move || {
            let mut publisher = manager
                .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
                .unwrap();
            ready.send(()).unwrap();
            while !stopping.load(Ordering::Relaxed) {
                publisher
                    .publish_raw(&make_wire_frame(
                        hash,
                        sequence,
                        stamp,
                        &value.to_le_bytes(),
                    ))
                    .unwrap();
                std::thread::sleep(Duration::from_millis(10));
            }
        });
        started.recv_timeout(Duration::from_secs(5)).unwrap();
        Self {
            stop,
            thread: Some(thread),
        }
    }
}
impl Drop for Producer {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.take() {
            thread.join().unwrap();
        }
    }
}

fn command(home: &std::path::Path, socket: &std::path::Path, args: &[&str]) -> Command {
    let test = std::env::current_exe().unwrap();
    let cli = test
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .join(format!("cerulion{}", std::env::consts::EXE_SUFFIX));
    assert!(
        cli.is_file(),
        "build the CLI first: cargo build -p cerulion_cli"
    );
    let mut command = Command::new(cli);
    command
        .args(args)
        .env("CERULION_HOME", home)
        .env("CERULION_NETD_SOCK", socket)
        .env("CERULION_NETD_RELAY_DISABLED", "1")
        .env("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:9")
        .env("CERULION_TELEMETRY", "off")
        .env("CERULION_LAN_ACCESS", "paired")
        .env_remove("CERULION_NETWORK")
        .env_remove("CERULION_NETD_NETWORK")
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    command
}

fn run(mut command: Command) -> Output {
    let mut child = ChildGuard(Some(command.spawn().unwrap()));
    let deadline = Instant::now() + Duration::from_secs(20);
    while child.try_wait().unwrap().is_none() {
        if Instant::now() >= deadline {
            let _ = child.kill();
            let _ = child.wait();
            panic!("CLI command exceeded its deadline");
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    child.output()
}

fn successful(output: Output) -> String {
    assert!(
        !String::from_utf8_lossy(&output.stderr)
            .contains("robot refused to serve its topic catalog"),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(
        output.status.success(),
        "stdout={}, stderr={}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    String::from_utf8(output.stdout).unwrap()
}

#[test]
fn automatic_paired_cli_delivers_frames_and_selected_sources_never_collide() {
    let _ = tracing_subscriber::fmt()
        .with_env_filter("cerulion_netd=debug,cerulion_remoted=debug,iroh=warn")
        .with_test_writer()
        .try_init();
    let topic = format!("/automatic/{}", unique_id());
    let desk_seed = unique_secret();
    let doc = cerulion_core::SchemaDoc {
        qualified: "remote_msgs/Reading".into(),
        encoding: cerulion_core::SchemaEncoding::Msg,
        text: "float64 value\n".into(),
        deps: Vec::new(),
    };
    let schema =
        cerulion_core::codegen::parse_rosmsg(&doc.text, "Reading", Some("remote_msgs")).unwrap();
    let (walker, warnings) = cerulion_core::codegen::FrameWalker::new(vec![schema]);
    assert!(warnings.is_empty());
    let hash = walker.schema_hash_for("remote_msgs/Reading").unwrap();
    let robot = RobotFixture::start_with_schema(
        "automatic",
        Arc::new(claimed_paired(public_of(&desk_seed))),
        Some((topic.clone(), doc)),
    );
    let manager = TransportManager::get_or_init().unwrap();
    let home = tempfile::tempdir_in("/tmp").unwrap();
    std::fs::write(home.path().join("desk.key"), desk_seed).unwrap();
    std::fs::write(
        home.path().join("robots.toml"),
        format!("[robots]\npaired = \"{}\"\n", robot.eid),
    )
    .unwrap();
    // Expired login does no HTTP; explicit pairing is the wire-read authority.
    std::fs::write(home.path().join("auth.json"), br#"{"account_id":"offline","session_token":"","refresh_token":"","expires_at_ns":0,"logged_in_ever":true}"#).unwrap();
    let controller = Arc::new(
        AccountWanController::new(
            manager.clone(),
            one_robot_registry("paired", &robot, desk_seed),
            Arc::new(AllowAllAuthorizer),
            AccountControllerConfig {
                config_home: Some(home.path().into()),
                network_enabled: true,
                serving_gateway: false,
                epoch_dir: None,
                trusted_direct: HashMap::new(),
            },
        )
        .unwrap(),
    );
    let socket = home.path().join("netd.sock");
    let mut daemon = cerulion_netd::start(
        socket.clone(),
        controller,
        cerulion_netd::NetdConfig::default(),
    )
    .unwrap();
    let _remote = Producer::start(
        robot.manager.clone(),
        topic.clone(),
        hash,
        17,
        123_456_789,
        42.5,
    );
    let info = successful(run(command(
        home.path(),
        &socket,
        &["topic", "info", &topic],
    )));
    let early_cache: std::collections::BTreeMap<String, SocketAddr> =
        serde_json::from_slice(&std::fs::read(home.path().join("robot-direct.json")).unwrap())
            .unwrap();
    assert!(
        early_cache
            .get(&robot.eid.to_string())
            .is_some_and(|address| robot.addrs.contains(address)),
        "first authenticated catalog cached {early_cache:?}; robot sockets {:?}",
        robot.addrs
    );
    assert!(
        info.contains("remote_msgs/Reading") && info.contains("17") && info.contains("123456789"),
        "{info}"
    );
    let listing = successful(run(command(home.path(), &socket, &["topic", "list", "-t"])));
    assert!(
        listing.contains("paired")
            && listing.contains("online, direct")
            && listing.contains("remote_msgs/Reading")
            && listing.contains(&topic),
        "{listing}"
    );
    let found = successful(run(command(
        home.path(),
        &socket,
        &["topic", "find", "remote_msgs/Reading"],
    )));
    assert!(found.lines().any(|line| line == topic), "{found}");
    let snapshot = run(command(
        home.path(),
        &socket,
        &["topic", "snapshot", &topic, "--timeout", "15"],
    ));
    assert!(!snapshot.status.success());
    assert!(
        String::from_utf8_lossy(&snapshot.stderr).contains("remote_msgs/Reading"),
        "{}",
        String::from_utf8_lossy(&snapshot.stderr)
    );
    let _local = Producer::start(manager.clone(), topic.clone(), hash, 23, 987_654_321, -1.25);
    let ambiguous = run(command(home.path(), &socket, &["topic", "info", &topic]));
    assert_eq!(ambiguous.status.code(), Some(2));
    let refusal = String::from_utf8(ambiguous.stderr).unwrap();
    assert!(
        refusal.contains("ambiguous: local, paired") && refusal.contains("--robot NAME"),
        "{refusal}"
    );
    let remote = successful(run(command(
        home.path(),
        &socket,
        &["topic", "info", &topic, "--robot", "paired"],
    )));
    assert!(
        remote.contains("123456789") && !remote.contains("987654321"),
        "{remote}"
    );
    let local = successful(run(command(
        home.path(),
        &socket,
        &["topic", "info", &topic, "--local"],
    )));
    assert!(
        local.contains("987654321") && !local.contains("123456789"),
        "{local}"
    );
    let echo_path = home.path().join("echo.out");
    let mut echo = command(
        home.path(),
        &socket,
        &["topic", "echo", &topic, "--robot", "paired"],
    );
    echo.stdout(std::fs::File::create(&echo_path).unwrap());
    let mut child = ChildGuard(Some(echo.spawn().unwrap()));
    let deadline = Instant::now() + Duration::from_secs(20);
    while !std::fs::read_to_string(&echo_path)
        .unwrap()
        .contains("42.5")
    {
        if child.try_wait().unwrap().is_some() || Instant::now() >= deadline {
            let _ = child.kill();
            let output = child.output();
            panic!(
                "remote scalar was not delivered: {}",
                String::from_utf8_lossy(&output.stderr)
            );
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    let all = successful(run(command(
        home.path(),
        &socket,
        &["topic", "list", "--all", "-t"],
    )));
    let internal = all
        .lines()
        .find(|line| {
            line.starts_with(&cerulion_netd::TopicKey::observed("paired", &topic).local_topic())
        })
        .expect("live observer channel is listed under --all");
    assert!(
        internal.contains("[internal]")
            && internal.contains("robot=paired")
            && internal.contains("remote_msgs/Reading"),
        "{internal}"
    );
    let plain = successful(run(command(home.path(), &socket, &["topic", "list"])));
    assert!(!plain.contains("/__cerulion/observers/"), "{plain}");
    // Signal only the child PID returned by this command; normal Ctrl-C releases the lease.
    let signaled = Command::new("/bin/kill")
        .args(["-INT", &child.id().to_string()])
        .status()
        .unwrap();
    assert!(signaled.success());
    let shutdown = Instant::now() + Duration::from_secs(5);
    while child.try_wait().unwrap().is_none() {
        if Instant::now() >= shutdown {
            let _ = child.kill();
            let _ = child.wait();
            panic!("echo did not stop after Ctrl-C");
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    let output = child.output();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let released = Instant::now() + Duration::from_secs(5);
    while daemon.active_demand_count() != 0 {
        assert!(Instant::now() < released, "CLI exit leaked a demand");
        std::thread::sleep(Duration::from_millis(10));
    }
    let cache: std::collections::BTreeMap<String, SocketAddr> =
        serde_json::from_slice(&std::fs::read(home.path().join("robot-direct.json")).unwrap())
            .unwrap();
    assert!(
        cache
            .get(&robot.eid.to_string())
            .is_some_and(|address| robot.addrs.contains(address)),
        "authenticated direct cache {cache:?} must name the robot's bound sockets {:?}",
        robot.addrs
    );
    daemon.shutdown();
}

// SPDX-License-Identifier: AGPL-3.0-only
//! Prerequisite: cargo build -p cerulion_cli -p test_node_macro_data_trigger_cdylib.
//! Real CLI supervisor, graph workers, canonical netd mirror and iroh delivery.

use super::authenticated_graph::{frame, source_graph};
use super::*;
use cerulion_core::transport::demand_authorizer::AllowAllAuthorizer;
use cerulion_core::ShmMessage;
use cerulion_netd::account_controller::{AccountControllerConfig, AccountWanController};
use cerulion_netd::egress::{EgressError, EgressId, EgressPlane};
use cerulion_netd::{NetdClient, NetdConfig};
use std::os::unix::process::CommandExt;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, ExitStatus, Stdio};

// An ingress-only test daemon exposes its real namespace. Registering a local
// producer is a test failure; all data admission uses the real account controller.
struct ConsumerNamespace(Arc<TransportManager>);
impl EgressPlane for ConsumerNamespace {
    fn serving_schema_snapshot(
        &self,
    ) -> Result<cerulion_netd::protocol::ServingSchemaSnapshot, String> {
        Ok(cerulion_netd::protocol::ServingSchemaSnapshot {
            schema_serving: Default::default(),
            ix_config_json: serde_json::to_string(&self.0.iox_config()).unwrap(),
        })
    }
    fn lan_access(&self) -> Option<cerulion_core::LanAccess> {
        Some(cerulion_core::LanAccess::platform_default())
    }
    fn register_egress(
        &self,
        _id: EgressId,
        _plan: &cerulion_core::GatewayPlan,
        _serving: &cerulion_core::SchemaServing,
        _namespace: Option<&str>,
    ) -> Result<bool, EgressError> {
        Err(EgressError::Server {
            reason: "an ingress-only graph must start no serving endpoint".into(),
        })
    }
    fn release_egress(&self, _id: EgressId) {}
}

struct GraphChild {
    child: Child,
    reaped: bool,
}
impl GraphChild {
    fn spawn(command: &mut Command) -> Self {
        command.process_group(0);
        Self {
            child: command.spawn().unwrap(),
            reaped: false,
        }
    }
    fn poll(&mut self) -> Option<ExitStatus> {
        let status = self.child.try_wait().unwrap();
        if status.is_some() {
            self.reaped = true;
        }
        status
    }
    fn signal(&self, signal: rustix::process::Signal) -> rustix::io::Result<()> {
        if self.reaped {
            return Ok(());
        }
        let pid = i32::try_from(self.child.id())
            .ok()
            .and_then(rustix::process::Pid::from_raw)
            .ok_or(rustix::io::Errno::INVAL)?;
        // The group belongs to this unreaped child; no external PID lookup.
        match rustix::process::kill_process_group(pid, signal) {
            Ok(()) | Err(rustix::io::Errno::SRCH) => Ok(()),
            Err(error) => Err(error),
        }
    }
    fn stop(&mut self) -> ExitStatus {
        self.signal(rustix::process::Signal::INT)
            .expect("signal the owned graph process group");
        let deadline = Instant::now() + Duration::from_secs(15);
        loop {
            if let Some(status) = self.poll() {
                return status;
            }
            assert!(
                Instant::now() < deadline,
                "graph supervisor did not reap its workers"
            );
            std::thread::sleep(Duration::from_millis(20));
        }
    }
}
impl Drop for GraphChild {
    fn drop(&mut self) {
        if self.reaped {
            return;
        }
        if let Err(error) = self.signal(rustix::process::Signal::TERM) {
            tracing::warn!(error = %error, "owned graph process group refused graceful shutdown");
        }
        let deadline = Instant::now() + Duration::from_secs(15);
        while self.poll().is_none() && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(20));
        }
        if !self.reaped {
            if let Err(error) = self.signal(rustix::process::Signal::KILL) {
                tracing::warn!(error = %error, "owned graph process group refused forced shutdown");
                let _killed = self.child.kill();
            }
            let _reaped = self.child.wait();
            self.reaped = true;
        }
    }
}

fn stage_workspace(root: &Path, prefix: &str, input: &str) -> PathBuf {
    let fixture = "test_node_macro_data_trigger_cdylib";
    let lib = cerulion_core::testing::find_fixture_cdylib(fixture);
    let nodes = root.join("nodes/consumer/src");
    std::fs::create_dir_all(&nodes).unwrap();
    std::fs::create_dir_all(root.join("target/debug")).unwrap();
    std::fs::create_dir_all(root.join("graphs")).unwrap();
    std::fs::write(
        root.join("Cargo.toml"),
        "[workspace]\nresolver = \"2\"\nmembers = []\n",
    )
    .unwrap();
    std::fs::copy(
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .unwrap()
            .join("test_fixtures")
            .join(fixture)
            .join("src/lib.rs"),
        nodes.join("lib.rs"),
    )
    .unwrap();
    std::fs::copy(
        lib,
        root.join("target/debug").join(format!(
            "{}consumer{}",
            std::env::consts::DLL_PREFIX,
            std::env::consts::DLL_SUFFIX
        )),
    )
    .unwrap();
    let output = format!("/{prefix}/consumer/cmd");
    std::fs::write(root.join("graphs/ingress.yaml"), format!("name: ingress\nprefix: {prefix}\nprocess_groups:\n  worker: [consumer]\nnodes:\n  - id: consumer\n    type: consumer\n    inputs:\n      - name: trigger_in\n        source: {input}\n    outputs:\n      - name: cmd\n        schema: geometry_msgs/Vector3\nnetwork:\n  mode: peer\n  ingress: [{input}]\n")).unwrap();
    PathBuf::from(output)
}

fn cli(root: &Path, home: &Path, socket: &Path, stdout: &Path, stderr: &Path) -> Command {
    let executable = std::env::current_exe()
        .unwrap()
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .join(format!("cerulion{}", std::env::consts::EXE_SUFFIX));
    assert!(
        executable.is_file(),
        "build the CLI and data-trigger fixture first"
    );
    let mut command = Command::new(executable);
    command
        .args(["graph", "run", "ingress", "--no-validate"])
        .current_dir(root)
        .env("CERULION_HOME", home)
        .env("CERULION_NETD_SOCK", socket)
        .env("CERULION_NETD_RELAY_DISABLED", "1")
        .env("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:9")
        .env("CERULION_TELEMETRY", "off")
        .env("CERULION_FLASHBACK", "off")
        .env_remove("CERULION_LAN_ACCESS")
        .env_remove("CERULION_NETWORK")
        .env_remove("CERULION_NETD_NETWORK")
        .env_remove("CERULION_EXECUTION_MODE")
        .env_remove("CARGO_TARGET_DIR")
        .env_remove("CERULION_LOGIN_GATE")
        .env("RUST_LOG", "cerulion_cli_engine=info")
        .stdin(Stdio::null())
        .stdout(std::fs::File::create(stdout).unwrap())
        .stderr(std::fs::File::create(stderr).unwrap());
    command
}

#[test]
fn paired_supervisor_delivers_remote_oracle_payload_and_releases_canonical_ingress() {
    let topic = "/robot/source/vector";
    let source_manager = test_manager("supervisor_source");
    let (mut source, frames) = source_graph(&source_manager);
    let manager = TransportManager::get_or_init().unwrap();
    let home = tempfile::tempdir_in("/tmp").unwrap();
    let desk_seed = unique_secret();
    let robot = RobotFixture::start_graph(
        "supervisor",
        Arc::new(claimed_paired(public_of(&desk_seed))),
        source_manager,
        cerulion_core::SchemaServing {
            served_topics: vec![topic.into()],
            egress_qos: vec![cerulion_core::EgressQos {
                topic: topic.into(),
                reliability: cerulion_core::EgressReliability::Reliable,
                send_queue: None,
            }],
            ..Default::default()
        },
        unique_secret(),
    );
    std::fs::write(home.path().join("desk.key"), desk_seed).unwrap();
    std::fs::write(
        home.path().join("robots.toml"),
        format!("[robots]\npaired = \"{}\"\n", robot.eid),
    )
    .unwrap();
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
    let mut daemon = cerulion_netd::start_with_egress(
        socket.clone(),
        controller,
        Arc::new(ConsumerNamespace(manager.clone())),
        NetdConfig::default(),
    )
    .unwrap();
    let workspace = tempfile::tempdir_in("/tmp").unwrap();
    let prefix = format!("graph_ingress_{}", unique_id());
    let output = stage_workspace(workspace.path(), &prefix, topic);
    let stdout = workspace.path().join("run.out");
    let stderr = workspace.path().join("run.err");
    let mut graph = GraphChild::spawn(&mut cli(
        workspace.path(),
        home.path(),
        &socket,
        &stdout,
        &stderr,
    ));
    // While catalog lookup peeks the real source, publish only the first oracle.
    // This node has a data trigger; its output is proof of input delivery.
    let deadline = Instant::now() + Duration::from_secs(25);
    let mut output_tap = None;
    let mut samples = Vec::new();
    let mut delivered = false;
    while Instant::now() < deadline {
        assert!(
            graph.poll().is_none(),
            "{}",
            std::fs::read_to_string(&stderr).unwrap()
        );
        *frames.lock().unwrap() = Some(0);
        source.step(Duration::from_millis(1));
        if output_tap.is_none() {
            output_tap = manager
                .create_data_only_subscriber(output.to_str().unwrap())
                .ok();
        }
        if let Some(tap) = &mut output_tap {
            tap.drain_owned(1, &mut samples).unwrap();
            for sample in samples.drain(..) {
                if sample.payload().get(40..48) == Some(frame(0)[40..48].as_ref()) {
                    delivered = true;
                }
            }
        }
        if delivered {
            break;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    assert!(delivered, "{}", std::fs::read_to_string(&stderr).unwrap());
    let log = std::fs::read_to_string(&stderr).unwrap();
    assert!(
        log.contains("authenticated graph network demands are held"),
        "{log}"
    );
    assert!(log.contains("GO signaled"), "{log}");
    let status = graph.stop();
    assert_eq!(
        status.code(),
        Some(0),
        "{}",
        std::fs::read_to_string(&stderr).unwrap()
    );
    let deadline = Instant::now() + Duration::from_secs(5);
    while daemon.active_demand_count() != 0 {
        assert!(
            Instant::now() < deadline,
            "supervisor leaked canonical demand"
        );
        std::thread::sleep(Duration::from_millis(10));
    }
    manager
        .create_remote_ingress_injector(
            topic,
            native_ros2_messages::geometry_msgs::Vector3::SCHEMA_HASH,
            MaxSliceLen::const_new(256),
        )
        .expect("supervisor exit frees canonical writer");
    // A declaration with no authorized publisher refuses before the GO gate.
    stage_workspace(workspace.path(), &prefix, "/missing/declared/input");
    let mut refused = GraphChild::spawn(&mut cli(
        workspace.path(),
        home.path(),
        &socket,
        &stdout,
        &stderr,
    ));
    let deadline = Instant::now() + Duration::from_secs(25);
    let status = loop {
        if let Some(status) = refused.poll() {
            break status;
        }
        assert!(Instant::now() < deadline);
        std::thread::sleep(Duration::from_millis(20));
    };
    assert!(!status.success());
    let log = std::fs::read_to_string(&stderr).unwrap();
    assert!(log.contains("0 authorized producers"), "{log}");
    assert!(!log.contains("GO signaled"), "{log}");
    let mut client = NetdClient::connect_or_spawn_at(socket).unwrap();
    assert!(client.status().unwrap().demands.is_empty());
    daemon.shutdown();
    robot.kill();
}

#[test]
fn pairing_only_ros2_attachment_passes_local_serving_authority() {
    let home = tempfile::tempdir_in("/tmp").unwrap();
    let workspace = tempfile::tempdir_in("/tmp").unwrap();
    std::fs::create_dir(workspace.path().join("graphs")).unwrap();
    std::fs::write(
        workspace.path().join("Cargo.toml"),
        "[workspace]\nmembers = []\n",
    )
    .unwrap();
    let state = home.path().join("robot-state/remoted");
    std::fs::create_dir_all(&state).unwrap();
    let seed = [0x45; 32];
    let mac = b"paired-cli-authority-integrity-key";
    let identity = DeviceIdentity::from_seed(&seed).public_key();
    std::fs::write(state.join("device_key"), seed).unwrap();
    std::fs::write(state.join("trust_store.mac_key"), mac).unwrap();
    let mut trust = TrustStore::provision(
        RobotId([0x46; 32]),
        identity,
        RootSet::new(vec![DeviceIdentity::from_seed(&[0x47; 32]).public_key()], 1).unwrap(),
        CHASSIS,
        T_NOW,
    )
    .unwrap()
    .with_path(state.join("trust_store"));
    trust
        .claim(OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    trust.save(mac).unwrap();
    let mut index = DeviceAccountIndex::new().with_path(state.join("device_index.json"));
    index.bind(DeviceIdentity::from_seed(&[0x48; 32]).public_key(), OWNER);
    index.save(mac).unwrap();
    assert!(!home.path().join("auth.json").exists());
    let executable = std::env::current_exe()
        .unwrap()
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .join(format!("cerulion{}", std::env::consts::EXE_SUFFIX));
    let stdout_path = workspace.path().join("attach.stdout");
    let stderr_path = workspace.path().join("attach.stderr");
    let mut command = Command::new(executable);
    command
        .args([
            "ros2",
            "attach",
            "--iface",
            "127.0.0.1",
            "--domain",
            "211",
            "--timeout",
            "0.02",
            "--yes",
        ])
        .current_dir(workspace.path())
        .env("CERULION_HOME", home.path())
        .env("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:9")
        .env("CERULION_TELEMETRY", "off")
        .env("RUST_LOG", "cerulion_cli_engine=info")
        .env_remove("CERULION_LOGIN_GATE")
        .env_remove("CERULION_NETWORK")
        .env_remove("CERULION_NETD_NETWORK")
        .env_remove("CERULION_LAN_ACCESS")
        .stdin(Stdio::null())
        .stdout(std::fs::File::create(stdout_path).unwrap())
        .stderr(std::fs::File::create(&stderr_path).unwrap());
    let mut child = GraphChild::spawn(&mut command);
    let deadline = Instant::now() + Duration::from_secs(10);
    while child.poll().is_none() {
        assert!(
            Instant::now() < deadline,
            "ROS 2 authority check did not complete"
        );
        std::thread::sleep(Duration::from_millis(20));
    }
    let stderr = std::fs::read_to_string(stderr_path).unwrap();
    assert!(
        stderr.contains("ros2 attach: starting DDS discovery"),
        "pairing must pass both CLI serving gates: {stderr}"
    );
    assert!(
        !home.path().join("auth.json").exists(),
        "the gate never enrolls"
    );
}

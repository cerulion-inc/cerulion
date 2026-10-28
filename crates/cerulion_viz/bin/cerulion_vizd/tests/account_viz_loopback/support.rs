// SPDX-License-Identifier: AGPL-3.0-only

use std::io::{BufRead, BufReader, Write};
use std::os::unix::fs::DirBuilderExt;
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::transport::network::NetworkConfig;
use cerulion_core::{TransportConfig, TransportManager};
use serde_json::{json, Value};

pub const BOUND: Duration = Duration::from_secs(15);
pub const TOPIC: &str = "/account_loopback/telemetry";
pub const TYPE: &str = "account_loopback/Telemetry";
// Public, fixed TEST-ONLY seeds; never production credentials.
pub const DESK_SEED: [u8; 32] = [0x61; 32];
pub const ROBOT_SEED: [u8; 32] = [0x62; 32];

/// One owned tree: auth, robot state, robot SHM, desk SHM, and all three sockets.
/// Managers and daemons drop before this guard, including during unwinding.
pub struct Directory(pub PathBuf);
impl Directory {
    pub fn new() -> Self {
        let path = PathBuf::from(format!("/tmp/cer-account-viz-{}", std::process::id()));
        std::fs::DirBuilder::new()
            .mode(0o700)
            .create(&path)
            .expect("exclusive private fixture root");
        Self(path)
    }
    pub fn child(&self, name: &str) -> PathBuf {
        let path = self.0.join(name);
        std::fs::DirBuilder::new()
            .mode(0o700)
            .create(&path)
            .unwrap();
        path
    }
}
impl Drop for Directory {
    fn drop(&mut self) {
        let result = std::fs::remove_dir_all(&self.0);
        if !std::thread::panicking() {
            result.expect("remove all fixture-owned files and isolated SHM roots");
        }
    }
}

pub struct EnvGuard(&'static str, Option<std::ffi::OsString>);
impl EnvGuard {
    pub fn set(name: &'static str, value: impl AsRef<std::ffi::OsStr>) -> Self {
        let old = std::env::var_os(name);
        std::env::set_var(name, value);
        Self(name, old)
    }
}
impl Drop for EnvGuard {
    fn drop(&mut self) {
        match self.1.take() {
            Some(old) => std::env::set_var(self.0, old),
            None => std::env::remove_var(self.0),
        }
    }
}

pub fn manager(root: &Path, name: &str) -> Arc<TransportManager> {
    // Use the existing serde config surface to set the private root without
    // adding an iceoryx system-types dependency solely for this test.
    let mut config = serde_json::to_value(cerulion_core::testing::iceoryx_test_config()).unwrap();
    config["global"]["root-path"] = json!(root);
    TransportManager::init_for_test(
        TransportConfig {
            node_name: name.into(),
            network: Some(NetworkConfig {
                listen_endpoints: vec!["tcp/127.0.0.1:0".into()],
                multicast_scouting: false,
                gossip_scouting: false,
                robot_identity: Some(name.into()),
                ..Default::default()
            }),
            ..Default::default()
        },
        serde_json::from_value(config).unwrap(),
    )
    .unwrap()
}

pub struct Client {
    writer: UnixStream,
    reader: BufReader<UnixStream>,
}
impl Client {
    pub fn connect(socket: &Path) -> Self {
        let writer = UnixStream::connect(socket).unwrap();
        writer.set_read_timeout(Some(BOUND)).unwrap();
        writer.set_write_timeout(Some(BOUND)).unwrap();
        let mut reader = BufReader::new(writer.try_clone().unwrap());
        let mut hello = String::new();
        assert_ne!(reader.read_line(&mut hello).unwrap(), 0);
        let _: Value = serde_json::from_str(&hello).unwrap();
        Self { writer, reader }
    }
    pub fn request(&mut self, request: Value) -> Value {
        writeln!(self.writer, "{request}").unwrap();
        let mut reply = String::new();
        assert_ne!(self.reader.read_line(&mut reply).unwrap(), 0);
        let reply: Value = serde_json::from_str(&reply).unwrap();
        assert_eq!(reply["id"], request["id"]);
        reply
    }
}

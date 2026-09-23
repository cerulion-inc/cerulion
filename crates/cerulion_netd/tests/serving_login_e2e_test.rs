// SPDX-License-Identifier: AGPL-3.0-only
//! The real LISTEN binary must refuse before any peer probe or listener.
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

#[test]
fn listen_without_prior_login_exits_before_creating_daemon_state() {
    let root = tempfile::tempdir().unwrap();
    let home = root.path().join("never-logged-in");
    let socket = root.path().join("netd.sock");
    let error_path = root.path().join("stderr");
    let mut child = Command::new(env!("CARGO_BIN_EXE_cerulion-netd"))
        .env("CERULION_HOME", &home)
        .env("CERULION_NETD_SOCK", &socket)
        // No usable locator exists: if the guard is removed, startup still
        // cannot open an ambient LAN listener. The login refusal wins first.
        .env("CERULION_NETD_LISTEN", "invalid-listen-locator")
        .env_remove("CERULION_NETD_NETWORK")
        .env_remove("CERULION_NETD_CONNECT")
        .env("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:1")
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(std::fs::File::create(&error_path).unwrap())
        .spawn()
        .unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    let status = loop {
        if let Some(status) = child.try_wait().unwrap() {
            break status;
        }
        if Instant::now() >= deadline {
            let _ = child.kill();
            let _ = child.wait();
            panic!("LISTEN without prior login must fail without prompting or waiting");
        }
        std::thread::sleep(Duration::from_millis(10));
    };
    let error = std::fs::read_to_string(&error_path).unwrap();
    assert_eq!(status.code(), Some(1), "{error}");
    assert!(
        error.contains("serving the network requires a prior login; run `cerulion login`"),
        "{error}"
    );
    assert!(!socket.exists());
    assert!(!home.exists());
}

struct OwnedDaemon(std::process::Child);

impl Drop for OwnedDaemon {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

#[test]
fn raw_egress_on_real_daemon_checks_login_before_the_namespace_sentinel() {
    use cerulion_core::{GatewayEgressPolicy, GatewayPlan, SchemaServing};
    use cerulion_netd::protocol::{Hello, Request, Response};
    use cerulion_netd::serving_login::REFUSAL;
    use std::io::{BufRead, BufReader, Write};
    use std::os::unix::net::UnixStream;

    for network_off in [false, true] {
        let root = tempfile::tempdir().unwrap();
        let home = root.path().join("account");
        let socket = root.path().join("netd.sock");
        let error_path = root.path().join("stderr");
        let mut command = Command::new(env!("CARGO_BIN_EXE_cerulion-netd"));
        command
            .env("CERULION_HOME", &home)
            .env("HOME", root.path())
            .env("CERULION_NETD_SOCK", &socket)
            .env("CERULION_NETD_IDLE_GRACE_MS", "60000")
            .env_remove("CERULION_NETD_LISTEN")
            .env_remove("CERULION_NETD_CONNECT")
            .env_remove("CERULION_NETD_WAN_ROBOTS")
            .env_remove("CERULION_NETD_DESK_KEY")
            .env("CERULION_NETD_RELAY_DISABLED", "1")
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(std::fs::File::create(&error_path).unwrap());
        if network_off {
            command.env("CERULION_NETD_NETWORK", "off");
        } else {
            command.env_remove("CERULION_NETD_NETWORK");
        }
        let mut child = OwnedDaemon(command.spawn().unwrap());
        let deadline = Instant::now() + Duration::from_secs(8);
        let mut stream = loop {
            if let Ok(stream) = UnixStream::connect(&socket) {
                break stream;
            }
            assert!(
                child.0.try_wait().unwrap().is_none(),
                "{}",
                std::fs::read_to_string(&error_path).unwrap()
            );
            assert!(
                Instant::now() < deadline,
                "daemon did not open control socket"
            );
            std::thread::sleep(Duration::from_millis(10));
        };
        stream
            .set_read_timeout(Some(Duration::from_secs(3)))
            .unwrap();
        stream
            .set_write_timeout(Some(Duration::from_secs(3)))
            .unwrap();
        let mut reader = BufReader::new(stream.try_clone().unwrap());
        let mut line = String::new();
        assert!(reader.read_line(&mut line).unwrap() > 0);
        let _: Hello = serde_json::from_str(&line).unwrap();
        for (id, saved_login) in [(1, false), (2, true), (3, false)] {
            let auth_path = home.join("auth.json");
            if saved_login {
                std::fs::create_dir_all(&home).unwrap();
                std::fs::write(&auth_path, br#"{"account_id":"saved-account","session_token":"expired-session","refresh_token":"expired-refresh","expires_at_ns":1,"logged_in_ever":true}"#).unwrap();
            } else if auth_path.exists() {
                std::fs::remove_file(&auth_path).unwrap();
            }
            let request = Request::RegisterEgress {
                id,
                plan: GatewayPlan {
                    egress_policy: GatewayEgressPolicy::AllowAll,
                    announce: vec!["/login/guard".into()],
                    ingress: vec![],
                },
                schema_serving: SchemaServing::default(),
                // The real plane validates this before session/gateway creation,
                // so even a regressed login check cannot advertise on the LAN.
                ix_config_json: Some("invalid-json-before-transport".into()),
            };
            writeln!(stream, "{}", request.to_json_line()).unwrap();
            line.clear();
            assert!(reader.read_line(&mut line).unwrap() > 0);
            let Response::Error(reply) = serde_json::from_str(&line).unwrap() else {
                panic!("expected a pre-transport refusal: {line}");
            };
            assert_eq!(reply.id, Some(id));
            if network_off || saved_login {
                assert!(
                    reply.error.starts_with("egress plan registration failed:"),
                    "{}",
                    reply.error
                );
                assert!(
                    reply
                        .error
                        .contains("forwarded iceoryx2 Config JSON is not a valid Config"),
                    "{}",
                    reply.error
                );
                assert!(!reply.error.contains(REFUSAL));
            } else {
                assert_eq!(reply.error, REFUSAL);
            }
        }
        drop(reader);
        drop(stream);
        drop(child);
    }
}

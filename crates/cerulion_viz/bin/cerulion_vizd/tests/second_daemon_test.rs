// SPDX-License-Identifier: AGPL-3.0-only
//! A second `cerulion-vizd` that loses the single-instance election must exit
//! WITHOUT ever owning a Rerun stream or a transport node — over the REAL binary.
//!
//! The daemon's worker sends the boot blueprint the moment it spawns. Before the
//! control socket was claimed first, a loser launched with `$CERULION_RERUN_URL`
//! pointed at the live daemon's proxy (what a supervisor that decided to USE a
//! foreign daemon still did) pushed its Scene-only default blueprint through
//! that proxy on its way out — and a supervisor respawning it every few seconds
//! kept snapping the viewer back to the default, undoing every representation
//! change and attach reflow. The pin: with the socket held here, the binary exits
//! nonzero naming the holder, and its log carries no transport-init, stream or
//! blueprint line.

use std::io::Read;
use std::net::TcpListener;
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use cerulion_vizd::hygiene::acquire_socket;

#[test]
fn a_losing_second_daemon_exits_before_it_owns_a_stream() {
    let dir = std::env::temp_dir().join(format!("cer_vizd_second_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    let socket = dir.join("vizd.sock");
    let (_listener, guard) = acquire_socket(socket.clone()).expect("hold the socket");

    // A stand-in for the live daemon's proxy: any connection here is the loser
    // reaching a viewer it must never reach.
    let proxy = TcpListener::bind("127.0.0.1:0").expect("bind proxy stand-in");
    proxy.set_nonblocking(true).expect("nonblocking");
    let rerun_url = format!("rerun+http://{}/proxy", proxy.local_addr().expect("addr"));

    let bin = env!("CARGO_BIN_EXE_cerulion-vizd");
    let mut child = Command::new(bin)
        .env("CERULION_VIZD_SOCK", &socket)
        .env("CERULION_RERUN_URL", &rerun_url)
        .env("CERULION_VIZD_NETWORK", "off")
        // `debug`, not `info`: the transport-init breadcrumbs the forbidden
        // list below watches for ("transport manager initialized", the
        // startup dead-node sweep) ride `debug!`, so at `info` their absence
        // would prove nothing. A dev-profile binary, where `debug!` exists.
        .env("RUST_LOG", "debug")
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn cerulion-vizd");

    let deadline = Instant::now() + Duration::from_secs(30);
    let status = loop {
        match child.try_wait().expect("try_wait") {
            Some(status) => break status,
            None if Instant::now() >= deadline => {
                let _ = child.kill();
                let _ = child.wait();
                panic!("a second cerulion-vizd did not exit within 30s while the socket was held");
            }
            None => std::thread::sleep(Duration::from_millis(25)),
        }
    };
    let mut stderr = String::new();
    if let Some(mut e) = child.stderr.take() {
        let _ = e.read_to_string(&mut stderr);
    }

    assert_ne!(
        status.code(),
        Some(0),
        "the loser exits nonzero. stderr:\n{stderr}"
    );
    assert!(
        stderr.contains("already running")
            && stderr.contains(&format!("(pid {})", std::process::id())),
        "the refusal names the live holder (this test's pid). stderr:\n{stderr}"
    );
    for forbidden in [
        "transport manager initialized",
        "startup dead-node sweep",
        "connected gRPC sink",
        "sent the dashboard blueprint",
    ] {
        assert!(
            !stderr.contains(forbidden),
            "a loser must not log `{forbidden}` — it touched shared state before losing. stderr:\n{stderr}"
        );
    }
    assert!(
        matches!(proxy.accept(), Err(e) if e.kind() == std::io::ErrorKind::WouldBlock),
        "the loser must never connect to the live daemon's proxy"
    );

    drop(guard);
    let _ = std::fs::remove_dir_all(&dir);
}

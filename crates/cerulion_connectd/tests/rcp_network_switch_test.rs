// SPDX-License-Identifier: AGPL-3.0-only
//! The real `cerulion-connectd rcp` under the network kill-switch: with
//! `CERULION_NETWORK=off` or `--network off` it exits 1 with the refusal on
//! stderr, writes nothing to stdout and sends nothing. The robot's direct
//! address is a UDP socket this test holds, so a dial would leave a datagram
//! in its queue; the control run with no switch shows that it does.
//!
//! Relays are disabled (no discovery, no relay contact). The control run is
//! stopped once its datagram has arrived.

use std::net::UdpSocket;
use std::process::{Child, Command, Output, Stdio};
use std::time::{Duration, Instant};

/// A well-formed endpoint id no robot in this test owns.
const EID: &str = "d75a980182b10ab7d54bfed3c964073a0ee172f3daa62325af021a68f707511a";

/// The longest any run here may take; it only stops a regression, every
/// wait ends on its own event first.
const BOUND: Duration = Duration::from_secs(20);

/// `rcp` at a direct address on `robot`, with `extra` flags and
/// `CERULION_NETWORK` set to `network` (`None` = unset), stdin empty.
fn spawn_rcp(robot: &UdpSocket, extra: &[&str], network: Option<&str>) -> Child {
    let addr = robot.local_addr().unwrap().to_string();
    let mut command = Command::new(env!("CARGO_BIN_EXE_cerulion-connectd"));
    command
        .args(["rcp", "--eid", EID, "--addr", &addr, "--relay-disabled"])
        .args(extra)
        .env_remove("CERULION_NETWORK")
        .env_remove("CERULION_RELAY_URL")
        .env("RUST_LOG", "warn")
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    if let Some(value) = network {
        command.env("CERULION_NETWORK", value);
    }
    command.spawn().expect("run cerulion-connectd rcp")
}

/// `child`'s output once it exits, or `None` (after killing it) when it is
/// still running at [`BOUND`].
fn exited(mut child: Child) -> Option<Output> {
    let until = Instant::now() + BOUND;
    while child.try_wait().expect("poll the child").is_none() {
        if Instant::now() >= until {
            let _ = child.kill();
            let _ = child.wait();
            return None;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    Some(child.wait_with_output().expect("collect the output"))
}

#[test]
fn rcp_under_the_kill_switch_is_refused_before_any_dial() {
    // Control: with no switch the same command dials the address, so a
    // datagram reaches the socket.
    let robot = UdpSocket::bind("127.0.0.1:0").expect("bind the stand-in robot socket");
    let mut control = spawn_rcp(&robot, &[], None);
    robot.set_read_timeout(Some(BOUND)).unwrap();
    let mut datagram = [0u8; 2048];
    let dialed = robot.recv_from(&mut datagram).is_ok();
    let _ = control.kill();
    let _ = control.wait();
    assert!(dialed, "the control run sent nothing to the robot address");

    for (extra, network) in [(&[][..], Some("off")), (&["--network", "off"][..], None)] {
        let robot = UdpSocket::bind("127.0.0.1:0").expect("bind the stand-in robot socket");
        let output = exited(spawn_rcp(&robot, extra, network));
        robot.set_nonblocking(true).unwrap();
        let dialed = robot.recv_from(&mut datagram).is_ok();
        assert!(
            !dialed,
            "{extra:?} {network:?}: a datagram reached the robot address"
        );
        let output = output.unwrap_or_else(|| panic!("{extra:?} {network:?}: rcp kept running"));
        let stderr = String::from_utf8_lossy(&output.stderr);
        assert_eq!(
            output.status.code(),
            Some(1),
            "{extra:?} {network:?}: {stderr}"
        );
        assert!(
            stderr.contains(
                "cerulion rcp: network kill-switch engaged (--network off / \
                 CERULION_NETWORK=off): refusing to open the robot's rcp session; nothing \
                 was sent"
            ),
            "{extra:?} {network:?}: {stderr}"
        );
        assert!(output.stdout.is_empty(), "{extra:?} {network:?}");
    }
}

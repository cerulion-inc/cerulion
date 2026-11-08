// SPDX-License-Identifier: AGPL-3.0-only
//! The real `cerulion-connectd act` under the network kill-switch: with
//! `CERULION_NETWORK=off` or `--network off` it exits 1 with the refusal on
//! its `act-error: ` line and sends nothing. The robot's direct address is a
//! UDP socket this test holds, so a dial would leave a datagram in its queue;
//! the control run with no switch shows that it does.
//!
//! Relays are disabled (no discovery, no relay contact), and `--timeout`
//! bounds the control run's dial.

use std::net::UdpSocket;
use std::process::{Command, Output};

/// A well-formed endpoint id no robot in this test owns.
const EID: &str = "d75a980182b10ab7d54bfed3c964073a0ee172f3daa62325af021a68f707511a";

/// Run `act` at a direct address on a local UDP socket, with `extra` flags
/// and `CERULION_NETWORK` set to `network` (`None` = unset). Returns the
/// output and whether a datagram reached the socket.
fn run_act(extra: &[&str], network: Option<&str>) -> (Output, bool) {
    let robot = UdpSocket::bind("127.0.0.1:0").expect("bind the stand-in robot socket");
    let addr = robot.local_addr().unwrap().to_string();
    let mut command = Command::new(env!("CARGO_BIN_EXE_cerulion-connectd"));
    command
        .args(["act", "--eid", EID, "--addr", &addr, "--relay-disabled"])
        .args(["--timeout", "3", "--verb", "topic-publish", "--args", "{}"])
        .args(extra)
        .env_remove("CERULION_NETWORK")
        .env_remove("CERULION_RELAY_URL")
        .env("RUST_LOG", "warn");
    if let Some(value) = network {
        command.env("CERULION_NETWORK", value);
    }
    let output = command.output().expect("run cerulion-connectd act");
    robot.set_nonblocking(true).unwrap();
    let mut datagram = [0u8; 2048];
    let dialed = robot.recv_from(&mut datagram).is_ok();
    (output, dialed)
}

#[test]
fn act_under_the_kill_switch_is_refused_before_any_dial() {
    // Control: with no switch the same call dials the address, so a datagram
    // waits in the socket's queue, and the call ends unreachable (exit 3).
    let (control, dialed) = run_act(&[], None);
    let stderr = String::from_utf8_lossy(&control.stderr);
    assert!(
        dialed,
        "the control run sent nothing to the robot address:\n{stderr}"
    );
    assert_eq!(control.status.code(), Some(3), "{stderr}");

    for (extra, network) in [(&[][..], Some("off")), (&["--network", "off"][..], None)] {
        let (output, dialed) = run_act(extra, network);
        let stderr = String::from_utf8_lossy(&output.stderr);
        assert!(
            !dialed,
            "{extra:?} {network:?}: a datagram reached the robot address:\n{stderr}"
        );
        assert_eq!(
            output.status.code(),
            Some(1),
            "{extra:?} {network:?}: {stderr}"
        );
        assert!(
            stderr.lines().any(|line| line
                == "act-error: network kill-switch engaged (--network off / \
                    CERULION_NETWORK=off): refusing to call the robot's `topic-publish` \
                    verb; nothing was sent"),
            "{extra:?} {network:?}: {stderr}"
        );
        assert!(output.stdout.is_empty(), "{extra:?} {network:?}");
    }
}

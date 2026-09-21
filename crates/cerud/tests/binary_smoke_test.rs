// SPDX-License-Identifier: AGPL-3.0-only
//! Real-binary end-to-end smoke: spawn the actual `cerud` process, drive a
//! verb through it over the Unix-socket dev transport, and confirm it was
//! receipted. Exercises `main` + `serve_forever` (the library e2e tests only
//! cover `serve_connection`).

#![cfg(unix)]

use std::process::{Child, Command};
use std::time::{Duration, Instant};

use cerud::client::OpsClient;
use cerud::receipt::{read_all, verify_chain, ReceiptOutcome};
use cerud::transport::connect_unix;

/// Kill + reap the child on drop so a failing assertion never leaks a process.
struct ChildGuard(Child);
impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

#[test]
fn real_binary_serves_a_verb_and_receipts_it() {
    let dir = tempfile::tempdir().unwrap();
    let socket = dir.path().join("ops.sock");
    let receipt_log = dir.path().join("receipts.log");

    let child = Command::new(env!("CARGO_BIN_EXE_cerud"))
        .arg("--dev-permissive")
        .arg("--socket")
        .arg(&socket)
        .arg("--receipt-log")
        .arg(&receipt_log)
        .arg("--log-root")
        .arg(dir.path())
        .spawn()
        .expect("spawn cerud binary");
    let _guard = ChildGuard(child);

    // Bounded connect-retry: the socket file can appear (bind) a moment before
    // the process is accept()ing, so a connect can transiently ECONNREFUSED.
    // Retry under a hard deadline so a genuinely dead server fails (not hangs).
    let deadline = Instant::now() + Duration::from_secs(10);
    let stream = loop {
        if Instant::now() > deadline {
            panic!("cerud did not accept a connection in time");
        }
        if socket.exists() {
            match connect_unix(&socket) {
                Ok(s) => break s,
                // Not yet listening — back off and retry.
                Err(_) => std::thread::sleep(Duration::from_millis(20)),
            }
        } else {
            std::thread::sleep(Duration::from_millis(20));
        }
    };
    // Bound every read/write so a server wedge fails the test, never hangs.
    stream
        .set_read_timeout(Some(Duration::from_secs(10)))
        .unwrap();
    stream
        .set_write_timeout(Some(Duration::from_secs(10)))
        .unwrap();

    let mut client = OpsClient::connect_current(stream).expect("handshake");
    assert_eq!(client.negotiated_version(), 1);
    assert_eq!(client.server_name(), "cerud");

    let result = client
        .call("inventory", serde_json::json!({}))
        .expect("inventory call");
    assert!(result["arch"].is_string());

    // The call was receipted with an Ok outcome, in an intact chain.
    let receipts = read_all(&receipt_log).expect("read receipts");
    assert_eq!(receipts.len(), 1);
    assert_eq!(receipts[0].verb, "inventory");
    assert_eq!(receipts[0].outcome, ReceiptOutcome::Ok);
    verify_chain(&receipts).unwrap();
}

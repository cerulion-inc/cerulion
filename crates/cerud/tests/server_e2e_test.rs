// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end server tests over the Unix-domain-socket dev transport: the
//! full handshake → authz → verb dispatch → receipt path with a real client.
//!
//! Each test uses a unique temp socket + receipt log (no process-global
//! state), so the suite is parallel-safe.

#![cfg(unix)]

use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;

use cerud::authz::{Authorizer, DenyAllAuthorizer, PermissiveDevAuthorizer};
use cerud::client::OpsClient;
use cerud::error::CerudError;
use cerud::receipt::{read_all, verify_chain, ReceiptLog, ReceiptOutcome};
use cerud::server::OpsServer;
use cerud::transport::{connect_unix, OpsListener, UnixSocketListener};
use cerud::verbs::restart::RestartRunner;
use cerud::verbs::{InventoryVerb, LogTailVerb, RestartVerb, VerbRegistry};

/// Bound on any client-side read/write so a SERVER WEDGE fails the test (with a
/// timeout error) instead of hanging CI forever.
const IO_TIMEOUT: Duration = Duration::from_secs(10);

/// Connect a client with bounded read/write timeouts + handshake.
fn connect_client(socket: &Path) -> OpsClient<UnixStream> {
    let stream = connect_unix(socket).expect("connect to ops socket");
    stream.set_read_timeout(Some(IO_TIMEOUT)).unwrap();
    stream.set_write_timeout(Some(IO_TIMEOUT)).unwrap();
    OpsClient::connect_current(stream).expect("handshake")
}

/// Spawn a server that serves EXACTLY one connection over `listener`, then
/// returns. The listener is bound by the CALLER (so the socket exists before
/// the client connects), then moved in.
fn spawn_serving(mut listener: UnixSocketListener, server: OpsServer) -> JoinHandle<()> {
    std::thread::spawn(move || {
        let conn = listener.accept().unwrap();
        // Swallow the per-connection result: a version-mismatch handshake
        // returns Err here by design, a clean close returns Ok.
        let _ = server.serve_connection(conn);
    })
}

/// Build a server with the three standard verbs.
fn standard_server(
    receipt_log: &Path,
    log_root: PathBuf,
    authorizer: Box<dyn Authorizer>,
) -> OpsServer {
    let mut verbs = VerbRegistry::new();
    verbs
        .register(Box::new(InventoryVerb))
        .register(Box::new(LogTailVerb::new(log_root)))
        .register(Box::new(RestartVerb::new()));
    let receipts = ReceiptLog::open(receipt_log).unwrap();
    OpsServer::new("cerud-test", authorizer, verbs, receipts)
}

/// Spin up a standard server that serves EXACTLY one connection.
fn spawn_one_connection_server(
    socket: &Path,
    receipt_log: &Path,
    log_root: PathBuf,
    authorizer: Box<dyn Authorizer>,
) -> JoinHandle<()> {
    let listener = UnixSocketListener::bind(socket).unwrap();
    let server = standard_server(receipt_log, log_root, authorizer);
    spawn_serving(listener, server)
}

#[test]
fn permissive_server_serves_inventory_and_receipts_it() {
    let dir = tempfile::tempdir().unwrap();
    let socket = dir.path().join("ops.sock");
    let receipt_log = dir.path().join("receipts.log");

    let handle = spawn_one_connection_server(
        &socket,
        &receipt_log,
        dir.path().to_path_buf(),
        Box::new(PermissiveDevAuthorizer),
    );

    let mut client = connect_client(&socket);
    assert_eq!(client.negotiated_version(), 1);
    assert_eq!(client.server_name(), "cerud-test");

    let result = client.call("inventory", serde_json::json!({})).unwrap();
    assert!(result["arch"].is_string());
    assert!(result["cerulion_version"].is_string());

    drop(client); // clean close ends the server's request loop
    handle.join().unwrap();

    // The invocation was receipted with an Ok outcome, in an intact chain.
    let receipts = read_all(&receipt_log).unwrap();
    assert_eq!(receipts.len(), 1);
    assert_eq!(receipts[0].verb, "inventory");
    assert_eq!(receipts[0].caller, "local-dev");
    assert_eq!(receipts[0].outcome, ReceiptOutcome::Ok);
    verify_chain(&receipts).unwrap();
}

#[test]
fn deny_by_default_server_refuses_and_receipts_the_denial() {
    let dir = tempfile::tempdir().unwrap();
    let socket = dir.path().join("ops.sock");
    let receipt_log = dir.path().join("receipts.log");

    let handle = spawn_one_connection_server(
        &socket,
        &receipt_log,
        dir.path().to_path_buf(),
        Box::new(DenyAllAuthorizer),
    );

    let mut client = connect_client(&socket);

    // A verb without a capability is refused server-side.
    let err = client.call("inventory", serde_json::json!({})).unwrap_err();
    match err {
        CerudError::Remote { kind, message } => {
            assert_eq!(kind, "denied");
            assert!(message.contains("deny-by-default"));
        }
        other => panic!("expected a remote denial, got {other:?}"),
    }

    drop(client);
    handle.join().unwrap();

    // The DENIAL itself is receipted (an audit trail of refused attempts).
    let receipts = read_all(&receipt_log).unwrap();
    assert_eq!(receipts.len(), 1);
    assert_eq!(receipts[0].verb, "inventory");
    assert_eq!(receipts[0].outcome, ReceiptOutcome::Denied);
    verify_chain(&receipts).unwrap();
}

#[test]
fn unknown_verb_is_a_remote_error_and_receipted() {
    let dir = tempfile::tempdir().unwrap();
    let socket = dir.path().join("ops.sock");
    let receipt_log = dir.path().join("receipts.log");

    let handle = spawn_one_connection_server(
        &socket,
        &receipt_log,
        dir.path().to_path_buf(),
        Box::new(PermissiveDevAuthorizer),
    );

    let mut client = connect_client(&socket);

    let err = client
        .call("frobnicate", serde_json::json!({}))
        .unwrap_err();
    match err {
        CerudError::Remote { kind, message } => {
            assert_eq!(kind, "unknown_verb");
            assert!(message.contains("frobnicate"));
        }
        other => panic!("expected an unknown-verb error, got {other:?}"),
    }

    drop(client);
    handle.join().unwrap();

    let receipts = read_all(&receipt_log).unwrap();
    assert_eq!(receipts.len(), 1);
    assert_eq!(
        receipts[0].outcome,
        ReceiptOutcome::Error("unknown_verb".to_string())
    );
}

#[test]
fn version_mismatch_is_refused_at_the_handshake() {
    let dir = tempfile::tempdir().unwrap();
    let socket = dir.path().join("ops.sock");
    let receipt_log = dir.path().join("receipts.log");

    let handle = spawn_one_connection_server(
        &socket,
        &receipt_log,
        dir.path().to_path_buf(),
        Box::new(PermissiveDevAuthorizer),
    );

    // The client only speaks versions 2..=3; the server only speaks 1 → refused.
    let stream = connect_unix(&socket).unwrap();
    stream.set_read_timeout(Some(IO_TIMEOUT)).unwrap();
    stream.set_write_timeout(Some(IO_TIMEOUT)).unwrap();
    // OpsClient is not Debug, so match without formatting the Ok arm's client.
    match OpsClient::connect(stream, 2, 3) {
        Err(CerudError::HandshakeRejected {
            reason,
            server_min,
            server_max,
        }) => {
            // The server's range travels the Reject frame verbatim.
            assert_eq!((server_min, server_max), (1, 1));
            // The reason is the server's OWN explanation, surfaced verbatim —
            // not a client-manufactured string. Here it is the server-side
            // version-mismatch message naming both ranges.
            assert!(
                reason.contains("version mismatch"),
                "reason should carry the server's version-mismatch explanation, got: {reason}"
            );
            assert!(
                reason.contains("1..=1") && reason.contains("2..=3"),
                "got: {reason}"
            );
        }
        Err(other) => panic!("expected HandshakeRejected, got a different error: {other:?}"),
        Ok(_) => panic!("expected HandshakeRejected, but the handshake succeeded"),
    }

    handle.join().unwrap();

    // No verb ran, so nothing is receipted.
    let receipts = read_all(&receipt_log).unwrap();
    assert!(receipts.is_empty());
}

/// A non-version rejection (an inverted client range → the server's `Decode`
/// error) is surfaced FAITHFULLY as `HandshakeRejected` carrying the server's
/// own reason — NOT mislabeled a version mismatch. This pins the client's
/// Reject-frame fidelity: whatever the server puts in `Reject.message` (a
/// future pairing-bootstrap refusal included) reaches the caller verbatim.
#[test]
fn non_version_rejection_is_surfaced_faithfully_not_as_version_mismatch() {
    let dir = tempfile::tempdir().unwrap();
    let socket = dir.path().join("ops.sock");
    let receipt_log = dir.path().join("receipts.log");

    let handle = spawn_one_connection_server(
        &socket,
        &receipt_log,
        dir.path().to_path_buf(),
        Box::new(PermissiveDevAuthorizer),
    );

    // The client advertises an INVERTED range (min > max); the server's
    // negotiate_version returns a `Decode` ("inverted...") error, sent back in
    // the Reject frame's message.
    let stream = connect_unix(&socket).unwrap();
    stream.set_read_timeout(Some(IO_TIMEOUT)).unwrap();
    stream.set_write_timeout(Some(IO_TIMEOUT)).unwrap();
    match OpsClient::connect(stream, 5, 2) {
        Err(CerudError::HandshakeRejected {
            reason,
            server_min,
            server_max,
        }) => {
            assert_eq!((server_min, server_max), (1, 1));
            // The reason is the server's inverted-range explanation — proving
            // the client did NOT reinterpret this non-version rejection as a
            // version mismatch.
            assert!(
                reason.contains("inverted"),
                "reason should carry the server's inverted-range explanation, got: {reason}"
            );
        }
        Err(other) => panic!("expected HandshakeRejected, got a different error: {other:?}"),
        Ok(_) => panic!("expected HandshakeRejected, but the handshake succeeded"),
    }

    handle.join().unwrap();

    let receipts = read_all(&receipt_log).unwrap();
    assert!(receipts.is_empty());
}

#[test]
fn registry_lists_the_three_verbs_sorted() {
    let mut verbs = VerbRegistry::new();
    verbs
        .register(Box::new(RestartVerb::new()))
        .register(Box::new(InventoryVerb))
        .register(Box::new(LogTailVerb::new("/var/log/cerulion")));
    // BTreeMap keeps names sorted regardless of registration order.
    assert_eq!(verbs.names(), vec!["inventory", "log-tail", "restart"]);
    assert_eq!(verbs.len(), 3);
}

/// A restart runner that records the argv of each command it is handed.
struct SharedRecordingRunner(Arc<Mutex<Vec<Vec<String>>>>);
impl RestartRunner for SharedRecordingRunner {
    fn supported(&self) -> bool {
        true
    }
    fn run(&self, argv: &[String]) -> cerud::error::CerudResult<()> {
        self.0.lock().unwrap().push(argv.to_vec());
        Ok(())
    }
}

#[test]
fn mutating_verb_writes_a_durable_intent_receipt_before_the_side_effect() {
    let dir = tempfile::tempdir().unwrap();
    let socket = dir.path().join("ops.sock");
    let receipt_log = dir.path().join("receipts.log");

    let calls = Arc::new(Mutex::new(Vec::new()));
    let mut verbs = VerbRegistry::new();
    verbs
        .register(Box::new(InventoryVerb))
        .register(Box::new(RestartVerb::with_runner(Box::new(
            SharedRecordingRunner(calls.clone()),
        ))));
    let receipts = ReceiptLog::open(&receipt_log).unwrap();
    let server = OpsServer::new(
        "cerud-test",
        Box::new(PermissiveDevAuthorizer),
        verbs,
        receipts,
    );
    let listener = UnixSocketListener::bind(&socket).unwrap();
    let handle = spawn_serving(listener, server);

    let mut client = connect_client(&socket);
    let result = client
        .call("restart", serde_json::json!({"graph": "perception"}))
        .unwrap();
    assert_eq!(result["restarted"], true);
    drop(client);
    handle.join().unwrap();

    // The side effect ran (stop + start).
    assert_eq!(calls.lock().unwrap().len(), 2);

    // Two receipts: an INTENT (written before the side effect) then the Ok
    // OUTCOME — and the chain is intact.
    let receipts = read_all(&receipt_log).unwrap();
    assert_eq!(receipts.len(), 2, "mutating verb writes intent + outcome");
    assert_eq!(receipts[0].verb, "restart");
    assert_eq!(receipts[0].outcome, ReceiptOutcome::Intent);
    assert_eq!(receipts[1].verb, "restart");
    assert_eq!(receipts[1].outcome, ReceiptOutcome::Ok);
    verify_chain(&receipts).unwrap();
}

#[test]
fn read_only_verb_writes_no_intent_receipt() {
    let dir = tempfile::tempdir().unwrap();
    let socket = dir.path().join("ops.sock");
    let receipt_log = dir.path().join("receipts.log");
    let handle = spawn_one_connection_server(
        &socket,
        &receipt_log,
        dir.path().to_path_buf(),
        Box::new(PermissiveDevAuthorizer),
    );

    let mut client = connect_client(&socket);
    client.call("inventory", serde_json::json!({})).unwrap();
    drop(client);
    handle.join().unwrap();

    // Exactly ONE receipt (the outcome) — a read-only verb gets no intent.
    let receipts = read_all(&receipt_log).unwrap();
    assert_eq!(receipts.len(), 1);
    assert_eq!(receipts[0].outcome, ReceiptOutcome::Ok);
    assert_ne!(receipts[0].outcome, ReceiptOutcome::Intent);
}

#[test]
fn audit_write_failure_makes_the_response_a_loud_error() {
    let dir = tempfile::tempdir().unwrap();
    let socket = dir.path().join("ops.sock");
    let receipt_log = dir.path().join("receipts.log");

    // Arm the receipt log so the (only) outcome write fails.
    let mut receipts = ReceiptLog::open(&receipt_log).unwrap();
    receipts.fail_next_record_for_test();
    let mut verbs = VerbRegistry::new();
    verbs.register(Box::new(InventoryVerb));
    let server = OpsServer::new(
        "cerud-test",
        Box::new(PermissiveDevAuthorizer),
        verbs,
        receipts,
    );
    let listener = UnixSocketListener::bind(&socket).unwrap();
    let handle = spawn_serving(listener, server);

    let mut client = connect_client(&socket);
    let err = client.call("inventory", serde_json::json!({})).unwrap_err();
    match err {
        CerudError::Remote { kind, message } => {
            assert_eq!(kind, "audit_failure", "caller must learn the audit failed");
            assert!(message.contains("audit write failed"));
        }
        other => panic!("expected an audit_failure remote error, got {other:?}"),
    }
    drop(client);
    handle.join().unwrap();

    // The outcome receipt could not be written, so nothing is on disk.
    assert!(read_all(&receipt_log).unwrap().is_empty());
}

#[test]
fn intent_write_failure_fails_closed_with_no_side_effect() {
    let dir = tempfile::tempdir().unwrap();
    let socket = dir.path().join("ops.sock");
    let receipt_log = dir.path().join("receipts.log");

    // Arm the log so the INTENT write (the first record for a mutating verb)
    // fails — the side effect must then NOT run (fail-closed).
    let mut receipts = ReceiptLog::open(&receipt_log).unwrap();
    receipts.fail_next_record_for_test();
    let calls = Arc::new(Mutex::new(Vec::new()));
    let mut verbs = VerbRegistry::new();
    verbs.register(Box::new(RestartVerb::with_runner(Box::new(
        SharedRecordingRunner(calls.clone()),
    ))));
    let server = OpsServer::new(
        "cerud-test",
        Box::new(PermissiveDevAuthorizer),
        verbs,
        receipts,
    );
    let listener = UnixSocketListener::bind(&socket).unwrap();
    let handle = spawn_serving(listener, server);

    let mut client = connect_client(&socket);
    let err = client
        .call("restart", serde_json::json!({"graph": "perception"}))
        .unwrap_err();
    match err {
        CerudError::Remote { kind, .. } => assert_eq!(kind, "audit_failure"),
        other => panic!("expected audit_failure, got {other:?}"),
    }
    drop(client);
    handle.join().unwrap();

    // FAIL-CLOSED: the restart command was never run.
    assert!(
        calls.lock().unwrap().is_empty(),
        "a failed intent receipt must prevent the side effect"
    );
    assert!(read_all(&receipt_log).unwrap().is_empty());
}

#[test]
fn multiple_requests_over_one_connection_are_each_receipted() {
    let dir = tempfile::tempdir().unwrap();
    let socket = dir.path().join("ops.sock");
    let receipt_log = dir.path().join("receipts.log");
    let handle = spawn_one_connection_server(
        &socket,
        &receipt_log,
        dir.path().to_path_buf(),
        Box::new(PermissiveDevAuthorizer),
    );

    // Two requests over ONE connection (the server's request loop).
    let mut client = connect_client(&socket);
    client.call("inventory", serde_json::json!({})).unwrap();
    client.call("inventory", serde_json::json!({})).unwrap();
    drop(client);
    handle.join().unwrap();

    let receipts = read_all(&receipt_log).unwrap();
    assert_eq!(receipts.len(), 2, "each request is receipted");
    assert_eq!(receipts[0].seq, 0);
    assert_eq!(receipts[1].seq, 1);
    verify_chain(&receipts).unwrap();
}

#[test]
fn dev_socket_and_created_parent_dir_have_owner_only_permissions() {
    use std::os::unix::fs::PermissionsExt;
    let dir = tempfile::tempdir().unwrap();
    // A parent dir that does NOT exist yet — bind must create it 0o700.
    let subdir = dir.path().join("run");
    let socket = subdir.join("ops.sock");
    assert!(!subdir.exists());

    let listener = UnixSocketListener::bind(&socket).unwrap();

    let sock_mode = std::fs::metadata(&socket).unwrap().permissions().mode() & 0o777;
    assert_eq!(sock_mode, 0o600, "socket must be owner-only");
    let dir_mode = std::fs::metadata(&subdir).unwrap().permissions().mode() & 0o777;
    assert_eq!(dir_mode, 0o700, "created parent dir must be owner-only");

    drop(listener);
}

#[test]
fn bind_tightens_every_created_ancestor_not_just_the_leaf() {
    // The state-root tree holds the trust store + audit receipts. When bind has
    // to create MULTIPLE missing ancestor directories, EACH created component
    // must be 0o700 — not just the leaf (a plain `create_dir_all` + leaf-only
    // chmod would leave the intermediates at the world-readable umask default).
    // A pre-existing ancestor must be left UNTOUCHED.
    use std::os::unix::fs::PermissionsExt;
    let dir = tempfile::tempdir().unwrap();

    // Give the PRE-EXISTING root a distinctive, non-0o700 mode so we can prove
    // bind does not chmod ancestors it did not create.
    std::fs::set_permissions(dir.path(), std::fs::Permissions::from_mode(0o755)).unwrap();

    // Three missing levels below the pre-existing tempdir.
    let a = dir.path().join("a");
    let b = a.join("b");
    let c = b.join("c");
    let socket = c.join("ops.sock");
    assert!(!a.exists());

    let listener = UnixSocketListener::bind(&socket).unwrap();

    // Every component WE created is exactly 0o700 (umask-proof via the explicit
    // chmod) — the intermediates `a`/`b`, not only the leaf `c`.
    for created in [&a, &b, &c] {
        let mode = std::fs::metadata(created).unwrap().permissions().mode() & 0o777;
        assert_eq!(
            mode,
            0o700,
            "created ancestor {} must be owner-only, got {mode:o}",
            created.display()
        );
    }

    // The pre-existing tempdir is left as we set it (0o700 was NOT forced onto
    // an ancestor bind did not create).
    let root_mode = std::fs::metadata(dir.path()).unwrap().permissions().mode() & 0o777;
    assert_eq!(
        root_mode, 0o755,
        "a pre-existing ancestor must NOT be re-chmod'd"
    );

    // The socket itself stays owner-only (0o600), unchanged by the fix.
    let sock_mode = std::fs::metadata(&socket).unwrap().permissions().mode() & 0o777;
    assert_eq!(sock_mode, 0o600, "socket must be owner-only");

    drop(listener);
}

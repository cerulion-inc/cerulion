// SPDX-License-Identifier: AGPL-3.0-only
//! The daemon's Rerun-endpoint hosting ([`cerulion_vizd::host`]).
//!
//! The checkbox→scene product bar requires the daemon to HOST the gRPC message
//! proxy itself (nobody else does on a bare machine). These pins prove:
//!
//! 1. **HOST (default):** with no `$CERULION_RERUN_URL`, `resolve_stream` binds a
//!    real proxy on loopback and advertises `rerun+http://127.0.0.1:{port}/proxy`
//!    — and the proxy ACTUALLY ACCEPTS a TCP connection (the bind signal; a
//!    silent async bind-failure would leave nothing listening). No viewer needed.
//! 2. **CLIENT (override):** with `$CERULION_RERUN_URL` set, `resolve_stream`
//!    routes to that external endpoint (advertises it VERBATIM) and binds NO
//!    listening socket of its own.
//! 3. **The REMOVED legacy name is IGNORED**: setting
//!    `RERUN_VIZ_ADDR` no longer routes anything anywhere.
//!
//! All three tests mutate the process env, so a file-local mutex + an RAII guard
//! serialize them (the `reference_file_local_mutex` pattern — no `serial_test`
//! dep in this bin's dev-deps).

use std::sync::Mutex;
use std::time::Duration;

use cerulion_vizd::host;
// Import the canonical env-var-name constant rather than redeclaring it:
// a divergence from the source of truth (`cerulion_viz::stream`) would then
// be a compile error, not a silent test/production drift.
use cerulion_viz::stream::{endpoint_override, ADDR_ENV};

/// The legacy spelling, which is no longer read as a fallback. It is
/// a literal here and nowhere in production, which is the point: no constant
/// names it any more, so a test asserting it is inert has to spell it out.
const REMOVED_ADDR_ENV: &str = "RERUN_VIZ_ADDR";

/// Serialize the env-mutating endpoint tests (all three touch `CERULION_RERUN_URL`).
static ENV_LOCK: Mutex<()> = Mutex::new(());

/// RAII: snapshot + restore a set of env vars around a test.
struct EnvGuard(Vec<(&'static str, Option<String>)>);
impl EnvGuard {
    fn capture(keys: &[&'static str]) -> Self {
        EnvGuard(keys.iter().map(|k| (*k, std::env::var(k).ok())).collect())
    }
}
impl Drop for EnvGuard {
    fn drop(&mut self) {
        for (k, v) in &self.0 {
            match v {
                Some(val) => std::env::set_var(k, val),
                None => std::env::remove_var(k),
            }
        }
    }
}

/// Parse `rerun+http://127.0.0.1:{port}/proxy` → the port.
fn port_of(url: &str) -> u16 {
    url.rsplit_once(':')
        .and_then(|(_, tail)| tail.split('/').next())
        .and_then(|p| p.parse().ok())
        .unwrap_or_else(|| panic!("host URL carries a port: {url}"))
}

#[test]
fn host_mode_binds_a_real_proxy_and_advertises_it() {
    let _lock = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _env = EnvGuard::capture(&[ADDR_ENV]);
    // No override → HOST mode.
    std::env::remove_var(ADDR_ENV);

    let res = host::resolve_stream();
    let url = res
        .rerun_url
        .as_deref()
        .expect("host mode advertises a bound proxy URL");
    assert!(
        url.starts_with("rerun+http://127.0.0.1:") && url.ends_with("/proxy"),
        "the advertised URL is a loopback proxy: {url}"
    );

    // THE signal: the proxy actually came up (a silent async bind-failure would
    // leave nothing listening). A viewer needs only this TCP endpoint.
    let port = port_of(url);
    assert!(
        host::wait_port_listening(port, Duration::from_secs(5)),
        "the hosted gRPC proxy on port {port} must accept a TCP connection (nothing is listening → \
         a silent bind failure, the reported checkbox→scene bar broken)"
    );

    // THE NONZERO CONTROL for the client arm below, read on the SAME channel: the
    // daemon's own record of what it bound. Host mode records the port it handed to
    // `serve_grpc_opts`, and the connect above proves that record is the truth
    // rather than a hopeful `Some`.
    assert_eq!(
        res.hosted_port,
        Some(port),
        "host mode records the port it bound, and it is the port it advertises"
    );

    // A flush into the hosted stream does not panic/block (the sink is live).
    // Then dropping `res` shuts the server down (GrpcServerSink::drop).
    let _ = res.rec.flush_blocking();
    drop(res);
}

/// CLIENT mode binds NO listening socket of its own, read off the daemon's own
/// record of what it bound rather than off the machine.
///
/// The predecessor of this arm connected to the override port and required the
/// connect to FAIL. That is a MACHINE-WIDE claim ("nothing anywhere is listening
/// on 127.0.0.1:{port}") and no daemon can promise it: a shared machine hands the
/// same loopback ports to every job on it, so an unrelated process listening there
/// failed this test with the daemon behaving perfectly. Widening the port band only
/// made the collision rarer.
///
/// `hosted_port` is the property the code DOES own: the daemon records the port it
/// handed to `serve_grpc_opts`, at the one function here that binds one. Client
/// mode bound nothing, so it reads `None` whatever else runs on the machine. The
/// NONZERO CONTROL is `host_mode_binds_a_real_proxy_and_advertises_it`, which reads
/// the SAME field, requires `Some(port)`, and proves that record is the truth by
/// connecting to it.
#[test]
fn client_override_routes_to_the_external_endpoint_and_hosts_nothing() {
    let _lock = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _env = EnvGuard::capture(&[ADDR_ENV]);
    // A parseable-but-arbitrary external endpoint. Nothing here depends on the port
    // being free, so the number is free to be a constant.
    let external = "rerun+http://127.0.0.1:39000/proxy";
    std::env::set_var(ADDR_ENV, external);

    let res = host::resolve_stream();
    assert_eq!(
        res.rerun_url.as_deref(),
        Some(external),
        "client mode advertises the override endpoint VERBATIM"
    );
    assert_eq!(
        res.hosted_port, None,
        "client mode must not host a proxy: it bound a listening socket on port {:?}",
        res.hosted_port
    );
    drop(res);
}

/// The legacy `RERUN_VIZ_ADDR` fallback is REMOVED, so
/// setting it must produce NO endpoint override at all.
///
/// `endpoint_override()` is the exact discriminator `host::resolve_stream`
/// branches on (`Some` ⇒ CLIENT, `None` ⇒ HOST), so pinning it here pins the
/// daemon's routing without binding a proxy: with only the removed name set the
/// daemon takes the HOST arm, which is what the sibling arm above already
/// covers for the no-override case.
///
/// The `ADDR_ENV` half is the ANTI-TAUTOLOGY: without it, "the removed name
/// yields None" is satisfied by a read site that resolves NOTHING.
#[test]
fn the_removed_pre_pt33_env_name_is_ignored() {
    let _lock = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _env = EnvGuard::capture(&[ADDR_ENV, REMOVED_ADDR_ENV]);

    // ONLY the removed name is set — an operator whose launch script still
    // exports it gets the daemon's DEFAULT (host) behaviour, not a silent
    // route to a stale endpoint.
    std::env::remove_var(ADDR_ENV);
    std::env::set_var(REMOVED_ADDR_ENV, "rerun+http://127.0.0.1:39999/proxy");
    assert_eq!(
        endpoint_override(),
        None,
        "{REMOVED_ADDR_ENV} was removed as a fallback — it must route nothing"
    );

    // Anti-tautology: the ONE supported name still resolves, and WINS over the
    // removed one being set alongside it.
    let supported = "rerun+http://127.0.0.1:39998/proxy";
    std::env::set_var(ADDR_ENV, supported);
    assert_eq!(
        endpoint_override().as_deref(),
        Some(supported),
        "{ADDR_ENV} is the ONE viewer-endpoint override"
    );
}

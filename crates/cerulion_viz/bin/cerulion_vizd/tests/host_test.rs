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
//!    routes to that external endpoint (advertises it VERBATIM) and hosts NOTHING.
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

    // A flush into the hosted stream does not panic/block (the sink is live).
    // Then dropping `res` shuts the server down (GrpcServerSink::drop).
    let _ = res.rec.flush_blocking();
    drop(res);
}

/// Find a port that is currently free AND outside the OS's ephemeral range.
///
/// The arm below asserts `!wait_port_listening(port, ..)` — a MACHINE-WIDE
/// property ("nothing anywhere is listening on 127.0.0.1:{port}"), not a
/// property of the code under test. A hardcoded port such as 59321
/// sits inside macOS's default ephemeral range (`net.inet.ip.portrange`
/// 49152-65535) — the range vizd's OWN host mode draws from
/// (`resolve_stream` → `probe_free_port` → a real listener), including in this
/// file's sibling arm. So any process on the desk that happened to be handed
/// 59321 by the kernel fails this test with nothing wrong: an
/// unattributed transient, and this is the cause.
///
/// The band below is above the registered range and below `portrange.first`, so
/// the kernel never hands it out; the bind probe then rules out a service that
/// deliberately sits there. A bind that SUCCEEDS proves nothing was listening at
/// that instant, and the listener is dropped so the arm's own assertion holds.
fn pick_unhosted_port() -> u16 {
    for port in 39_000..39_100_u16 {
        if std::net::TcpListener::bind(("127.0.0.1", port)).is_ok() {
            return port;
        }
    }
    panic!("no free non-ephemeral port in 39000..39100 — is something bound to all of them?");
}

#[test]
fn client_override_routes_to_the_external_endpoint_and_hosts_nothing() {
    let _lock = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _env = EnvGuard::capture(&[ADDR_ENV]);
    // A parseable-but-arbitrary external endpoint on a port we do NOT host, and
    // which the kernel will not hand to anybody else while this runs.
    let port = pick_unhosted_port();
    let external = format!("rerun+http://127.0.0.1:{port}/proxy");
    std::env::set_var(ADDR_ENV, &external);

    // PRECONDITION, stated before the measurement so a foreign listener fails as
    // a foreign listener rather than as "client mode hosted a proxy".
    assert!(
        !host::wait_port_listening(port, Duration::from_millis(50)),
        "precondition: port {port} must be free BEFORE resolve_stream runs — this \
         arm asserts a machine-wide absence, so something else listening here is a \
         desk problem, not a client-mode one"
    );

    let res = host::resolve_stream();
    assert_eq!(
        res.rerun_url.as_deref(),
        Some(external.as_str()),
        "client mode advertises the override endpoint VERBATIM"
    );

    // Client mode HOSTS nothing: the external port is not bound locally. (No
    // server was started, and nothing is listening there → the connect fails fast.)
    assert_eq!(
        port_of(&external),
        port,
        "the URL carries the port we chose"
    );
    assert!(
        !host::wait_port_listening(port, Duration::from_millis(300)),
        "client mode must not host a proxy — port {port} should have no listener"
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

// SPDX-License-Identifier: AGPL-3.0-only
//! A bounded check that a QUIC endpoint answers on a UDP address, run before
//! netd's session opens so the boot-time fold can put one locator per peer
//! address into zenoh's connect list.
//!
//! # Why a probe at all
//!
//! zenoh dials every endpoint in `connect/endpoints` for the life of the
//! session, so listing a QUIC locator and a TCP locator for one peer keeps both
//! connectors running. Its only dial bound is `transport/unicast/open_timeout`
//! (10 s by default), and a QUIC dial to a UDP port that silently drops packets
//! waits that whole time. So the fold asks first, with a short bound, and then
//! lists one locator.
//!
//! # What is sent
//!
//! One QUIC long-header packet carrying a reserved version (`0x1a2a3a4a`, of the
//! `0x?a?a?a?a` form RFC 9000 section 15 keeps for exactly this), padded to
//! 1200 bytes, on a CONNECTED UDP socket. A QUIC server answers an
//! unsupported version in a datagram of at least 1200 bytes with a Version
//! Negotiation packet (RFC 9000 sections 5.2.2 and 6.1; quinn-proto 0.11.19, which zenoh's
//! QUIC and unencrypted QUIC listeners run on, does so in `Endpoint::handle`). A
//! reply counts only if it is a long header with version 0 and echoes this
//! probe's connection ids, swapped. No connection is created on the server.
//!
//! Outcomes: a matching reply is [`QuicProbeOutcome::Answered`]. A closed port
//! answers ICMP port unreachable, which a connected socket reports as
//! `ECONNREFUSED` ([`QuicProbeOutcome::Refused`], typically well under a
//! millisecond on a LAN). A filter that drops the datagram leaves nothing to read:
//! the probe is resent every [`QUIC_PROBE_RESEND`] and given up at the bound
//! ([`QuicProbeOutcome::NoAnswer`]).
//!
//! # The bound
//!
//! [`DEFAULT_QUIC_PROBE_TIMEOUT`] is 1000 ms: RFC 9002's first probe timeout for
//! a connection with no RTT sample yet (`kInitialRtt` 333 ms, PTO = 333 + 4 x
//! 166.5 ms, section 6.2.2), the point at which a QUIC client itself declares its
//! first flight lost. Resending every 333 ms gives a path that drops one datagram
//! two more tries inside it. It is paid only when UDP is silently filtered, once
//! per peer per netd start, with every peer probed in parallel. Raise it with
//! [`QUIC_PROBE_TIMEOUT_ENV`] only for a path whose measured round trip is above
//! about 300 ms.
//!
//! A larger value is clamped to [`MAX_QUIC_PROBE_TIMEOUT`] (6000 ms) with one
//! warning. The probe runs before netd binds its control socket, and a client
//! that starts netd waits 10 s for that socket; 6000 ms plus the boot browse, the
//! TCP probe round, both collectors' slack and the transport start-up that follows
//! the fold stays inside that wait (a compile-time assertion below holds the sum).

use std::collections::HashMap;
use std::net::{SocketAddr, ToSocketAddrs, UdpSocket};
use std::time::{Duration, Instant};

/// Overrides [`DEFAULT_QUIC_PROBE_TIMEOUT`], in milliseconds (a positive integer).
pub const QUIC_PROBE_TIMEOUT_ENV: &str = "CERULION_NETD_QUIC_PROBE_TIMEOUT_MS";

/// How long a probe waits for a QUIC answer before the fold falls back. See the
/// module docs for why it is 1000 ms.
pub const DEFAULT_QUIC_PROBE_TIMEOUT: Duration = Duration::from_millis(1000);

/// The largest probe bound [`resolve_quic_probe_timeout`] returns. See the
/// module docs for why it is 6000 ms.
pub const MAX_QUIC_PROBE_TIMEOUT: Duration = Duration::from_millis(6000);

/// The transport start-up netd runs after the fold and before it binds its
/// control socket (iceoryx2 init plus the start-up dead-node sweep), as measured
/// once on a desk-class machine (see the calibration note on the client's spawn
/// wait in `client.rs`).
const TRANSPORT_INIT_COLD_BOOT: Duration = Duration::from_millis(1100);

// netd's start-up holds the control socket back for the browse, the TCP probe
// round (bound plus its 200 ms collector slack), this probe round (bound plus
// COLLECT_SLACK) and then the transport start-up; the sum must fit the wait of a
// client that started netd.
const _: () = assert!(
    crate::discovery_fold::MDNS_FOLD_BUDGET.as_millis()
        + cerulion_discovery::ladder::LADDER_PROBE_TIMEOUT.as_millis()
        + 200
        + MAX_QUIC_PROBE_TIMEOUT.as_millis()
        + COLLECT_SLACK.as_millis()
        + TRANSPORT_INIT_COLD_BOOT.as_millis()
        < crate::client::SPAWN_READY_TIMEOUT.as_millis()
);

/// How often an unanswered probe is resent inside the bound (`kInitialRtt`).
pub const QUIC_PROBE_RESEND: Duration = Duration::from_millis(333);

/// The reserved QUIC version the probe carries; a server with no version in
/// common answers it with Version Negotiation (quinn-proto does).
const PROBE_VERSION: u32 = 0x1a2a_3a4a;

/// The smallest datagram a QUIC server answers an unknown version in
/// (RFC 9000 section 14.1).
const PROBE_DATAGRAM_LEN: usize = 1200;

/// Connection id length the probe uses for both ids.
const PROBE_CID_LEN: usize = 8;

/// What one probe found.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum QuicProbeOutcome {
    /// A QUIC endpoint answered with Version Negotiation.
    Answered,
    /// The host answered that nothing listens on that UDP port.
    Refused,
    /// No QUIC answer within the bound (the datagram, or the answer, was dropped).
    NoAnswer {
        /// The bound that elapsed.
        after: Duration,
    },
    /// The probe could not be sent: the locator did not resolve, or a socket
    /// call failed. Carries the reason.
    Failed(String),
}

impl QuicProbeOutcome {
    /// Whether the peer answered QUIC.
    pub fn answered(&self) -> bool {
        matches!(self, Self::Answered)
    }
}

impl std::fmt::Display for QuicProbeOutcome {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Answered => f.write_str("answered"),
            Self::Refused => f.write_str("the UDP port is closed (connection refused)"),
            Self::NoAnswer { after } => {
                write!(f, "no QUIC answer within {} ms", after.as_millis())
            }
            Self::Failed(reason) => write!(f, "the probe could not be sent: {reason}"),
        }
    }
}

/// The probe bound from a [`QUIC_PROBE_TIMEOUT_ENV`] value: a positive integer
/// in milliseconds, clamped to [`MAX_QUIC_PROBE_TIMEOUT`], else
/// [`DEFAULT_QUIC_PROBE_TIMEOUT`]. An unparseable, zero or clamped value logs one
/// `warn!` naming it.
pub fn resolve_quic_probe_timeout(value: Option<&str>) -> Duration {
    let Some(raw) = value.map(str::trim).filter(|v| !v.is_empty()) else {
        return DEFAULT_QUIC_PROBE_TIMEOUT;
    };
    match raw.parse::<u64>() {
        Ok(ms) if ms > 0 && Duration::from_millis(ms) <= MAX_QUIC_PROBE_TIMEOUT => {
            Duration::from_millis(ms)
        }
        Ok(ms) if ms > 0 => {
            tracing::warn!(
                value = raw,
                max_ms = MAX_QUIC_PROBE_TIMEOUT.as_millis() as u64,
                "cerulion-netd: {QUIC_PROBE_TIMEOUT_ENV} is above the largest probe bound; \
                 using the largest"
            );
            MAX_QUIC_PROBE_TIMEOUT
        }
        _ => {
            tracing::warn!(
                value = raw,
                default_ms = DEFAULT_QUIC_PROBE_TIMEOUT.as_millis() as u64,
                "cerulion-netd: {QUIC_PROBE_TIMEOUT_ENV} is not a positive whole number of \
                 milliseconds; using the default"
            );
            DEFAULT_QUIC_PROBE_TIMEOUT
        }
    }
}

/// [`resolve_quic_probe_timeout`] over the process environment.
pub fn quic_probe_timeout_from_env() -> Duration {
    resolve_quic_probe_timeout(std::env::var(QUIC_PROBE_TIMEOUT_ENV).ok().as_deref())
}

/// The probe datagram for the two connection ids. Pure.
fn probe_datagram(dcid: [u8; PROBE_CID_LEN], scid: [u8; PROBE_CID_LEN]) -> Vec<u8> {
    let mut packet = Vec::with_capacity(PROBE_DATAGRAM_LEN);
    // Long header, fixed bit set, Initial type.
    packet.push(0xc0);
    packet.extend_from_slice(&PROBE_VERSION.to_be_bytes());
    packet.push(PROBE_CID_LEN as u8);
    packet.extend_from_slice(&dcid);
    packet.push(PROBE_CID_LEN as u8);
    packet.extend_from_slice(&scid);
    packet.resize(PROBE_DATAGRAM_LEN, 0);
    packet
}

/// Whether `reply` is a Version Negotiation packet answering the probe sent
/// with `dcid` / `scid`: long header, version 0, its destination id is the
/// probe's source id and its source id is the probe's destination id. Pure.
fn is_version_negotiation_for(
    reply: &[u8],
    dcid: [u8; PROBE_CID_LEN],
    scid: [u8; PROBE_CID_LEN],
) -> bool {
    let Some((&first, rest)) = reply.split_first() else {
        return false;
    };
    if first & 0x80 == 0 || rest.len() < 4 || rest[..4] != [0, 0, 0, 0] {
        return false;
    }
    let rest = &rest[4..];
    let Some((&dlen, rest)) = rest.split_first() else {
        return false;
    };
    let dlen = usize::from(dlen);
    if dlen != PROBE_CID_LEN || rest.len() < dlen || rest[..dlen] != scid {
        return false;
    }
    let rest = &rest[dlen..];
    let Some((&slen, rest)) = rest.split_first() else {
        return false;
    };
    let slen = usize::from(slen);
    slen == PROBE_CID_LEN && rest.len() >= slen && rest[..slen] == dcid
}

/// Eight unpredictable bytes from std's per-process hash seed and the clock.
fn random_cid(salt: u64) -> [u8; PROBE_CID_LEN] {
    use std::hash::{BuildHasher, Hasher};
    let mut h = std::collections::hash_map::RandomState::new().build_hasher();
    h.write_u64(salt);
    h.write_u128(
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0, |d| d.as_nanos()),
    );
    h.finish().to_be_bytes()
}

/// Probe one UDP address for a QUIC endpoint, waiting at most `timeout` clamped
/// to [`MAX_QUIC_PROBE_TIMEOUT`].
pub fn probe_quic(addr: SocketAddr, timeout: Duration) -> QuicProbeOutcome {
    let bind: SocketAddr = if addr.is_ipv4() {
        ([0, 0, 0, 0], 0).into()
    } else {
        ([0u16; 8], 0).into()
    };
    let socket = match UdpSocket::bind(bind).and_then(|s| s.connect(addr).map(|()| s)) {
        Ok(s) => s,
        Err(e) => return QuicProbeOutcome::Failed(format!("cannot open a UDP socket: {e}")),
    };
    let dcid = random_cid(1);
    let scid = random_cid(2);
    let packet = probe_datagram(dcid, scid);
    let timeout = timeout.min(MAX_QUIC_PROBE_TIMEOUT);
    let start = Instant::now();
    // `timeout` is at most MAX_QUIC_PROBE_TIMEOUT, so neither add can fail; the
    // fallbacks keep the loop finite if one did.
    let deadline = start.checked_add(timeout).unwrap_or(start);
    let mut next_send = start;
    let mut buf = [0u8; 1500];
    loop {
        let now = Instant::now();
        if now >= deadline {
            return QuicProbeOutcome::NoAnswer { after: timeout };
        }
        if now >= next_send {
            match socket.send(&packet) {
                Ok(_) => {}
                Err(e) if e.kind() == std::io::ErrorKind::ConnectionRefused => {
                    return QuicProbeOutcome::Refused
                }
                Err(e) => return QuicProbeOutcome::Failed(format!("send failed: {e}")),
            }
            next_send = now.checked_add(QUIC_PROBE_RESEND).unwrap_or(deadline);
        }
        let wait = deadline
            .min(next_send)
            .saturating_duration_since(Instant::now())
            .max(Duration::from_millis(1));
        if let Err(e) = socket.set_read_timeout(Some(wait)) {
            return QuicProbeOutcome::Failed(format!("cannot set the read timeout: {e}"));
        }
        match socket.recv(&mut buf) {
            Ok(n) if is_version_negotiation_for(&buf[..n], dcid, scid) => {
                return QuicProbeOutcome::Answered
            }
            Ok(_) => {}
            Err(e) if e.kind() == std::io::ErrorKind::ConnectionRefused => {
                return QuicProbeOutcome::Refused
            }
            Err(e)
                if matches!(
                    e.kind(),
                    std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                ) => {}
            Err(e) => return QuicProbeOutcome::Failed(format!("receive failed: {e}")),
        }
    }
}

/// The `<host>:<port>` of a `udp/<host>:<port>?...` locator, resolved to its
/// first address.
fn udp_locator_addr(locator: &str) -> Result<SocketAddr, String> {
    let rest = locator
        .strip_prefix("udp/")
        .ok_or_else(|| format!("'{locator}' is not a udp/ locator"))?;
    let address = rest.split(['?', '#']).next().unwrap_or(rest);
    let mut addrs = address
        .to_socket_addrs()
        .map_err(|e| format!("cannot resolve '{address}': {e}"))?;
    addrs
        .next()
        .ok_or_else(|| format!("'{address}' resolved to no address"))
}

/// Slack the collector of [`probe_quic_locators`] allows past the bound for
/// the probe threads to report.
pub const COLLECT_SLACK: Duration = Duration::from_millis(200);

/// Probe every `udp/` locator in parallel, each bounded by `timeout` clamped to
/// [`MAX_QUIC_PROBE_TIMEOUT`], and return within that bound plus [`COLLECT_SLACK`] whatever the resolver or the
/// network does. Every input locator has an entry in the result; one whose
/// probe thread could not start, or had not reported by then (a hostname whose
/// lookup stalls), is [`QuicProbeOutcome::Failed`]. Such a thread is detached
/// and its late result dropped.
pub fn probe_quic_locators(
    locators: &[String],
    timeout: Duration,
) -> HashMap<String, QuicProbeOutcome> {
    let (tx, rx) = std::sync::mpsc::channel();
    let mut out = HashMap::with_capacity(locators.len());
    let mut pending = 0usize;
    for locator in locators {
        let tx = tx.clone();
        let owned = locator.clone();
        let spawned = std::thread::Builder::new()
            .name("netd-quic-probe".to_string())
            .spawn(move || {
                let outcome = match udp_locator_addr(&owned) {
                    Ok(addr) => probe_quic(addr, timeout),
                    Err(reason) => QuicProbeOutcome::Failed(reason),
                };
                let _ = tx.send((owned, outcome));
            });
        match spawned {
            Ok(_) => pending += 1,
            Err(e) => {
                out.insert(
                    locator.clone(),
                    QuicProbeOutcome::Failed(format!("cannot start the probe thread: {e}")),
                );
            }
        }
    }
    drop(tx);
    let timeout = timeout.min(MAX_QUIC_PROBE_TIMEOUT);
    let now = Instant::now();
    let deadline = now.checked_add(timeout + COLLECT_SLACK).unwrap_or(now);
    while pending > 0 {
        let left = deadline.saturating_duration_since(Instant::now());
        match rx.recv_timeout(left) {
            Ok((locator, outcome)) => {
                out.insert(locator, outcome);
                pending -= 1;
            }
            Err(_) => break,
        }
    }
    for locator in locators {
        out.entry(locator.clone()).or_insert_with(|| {
            QuicProbeOutcome::Failed(format!(
                "the probe did not finish within {} ms",
                (timeout + COLLECT_SLACK).as_millis()
            ))
        });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    const D: [u8; 8] = [1, 2, 3, 4, 5, 6, 7, 8];
    const S: [u8; 8] = [9, 10, 11, 12, 13, 14, 15, 16];

    #[test]
    fn the_probe_is_a_padded_long_header_with_the_reserved_version() {
        let p = probe_datagram(D, S);
        assert_eq!(p.len(), 1200);
        assert_eq!(p[0], 0xc0);
        assert_eq!(&p[1..5], &[0x1a, 0x2a, 0x3a, 0x4a]);
        assert_eq!(p[5], 8);
        assert_eq!(&p[6..14], &D);
        assert_eq!(p[14], 8);
        assert_eq!(&p[15..23], &S);
        assert!(p[23..].iter().all(|&b| b == 0));
    }

    fn vn(dcid: &[u8], scid: &[u8], version: [u8; 4], first: u8) -> Vec<u8> {
        let mut r = vec![first];
        r.extend_from_slice(&version);
        r.push(dcid.len() as u8);
        r.extend_from_slice(dcid);
        r.push(scid.len() as u8);
        r.extend_from_slice(scid);
        r.extend_from_slice(&[0, 0, 0, 1]);
        r
    }

    #[test]
    fn only_a_version_negotiation_echoing_both_ids_counts() {
        assert!(is_version_negotiation_for(&vn(&S, &D, [0; 4], 0xc5), D, S));
        // Ids not swapped, wrong version, short header, truncated, empty.
        assert!(!is_version_negotiation_for(&vn(&D, &S, [0; 4], 0xc5), D, S));
        assert!(!is_version_negotiation_for(
            &vn(&S, &D, [0, 0, 0, 1], 0xc5),
            D,
            S
        ));
        assert!(!is_version_negotiation_for(&vn(&S, &D, [0; 4], 0x45), D, S));
        assert!(!is_version_negotiation_for(
            &vn(&S, &D, [0; 4], 0xc5)[..12],
            D,
            S
        ));
        assert!(!is_version_negotiation_for(&[], D, S));
    }

    #[test]
    fn the_bound_reads_positive_milliseconds_and_defaults_otherwise() {
        assert_eq!(resolve_quic_probe_timeout(None), DEFAULT_QUIC_PROBE_TIMEOUT);
        assert_eq!(
            resolve_quic_probe_timeout(Some("")),
            DEFAULT_QUIC_PROBE_TIMEOUT
        );
        assert_eq!(
            resolve_quic_probe_timeout(Some("2500")),
            Duration::from_millis(2500)
        );
        assert_eq!(
            resolve_quic_probe_timeout(Some("6000")),
            MAX_QUIC_PROBE_TIMEOUT
        );
        // Above the cap, up to u64::MAX: the cap.
        for big in ["6001", "8000", "60000", "18446744073709551615"] {
            assert_eq!(
                resolve_quic_probe_timeout(Some(big)),
                MAX_QUIC_PROBE_TIMEOUT,
                "{big}"
            );
        }
        for bad in ["0", "-5", "1.5", "1s", "lots", "18446744073709551616"] {
            assert_eq!(
                resolve_quic_probe_timeout(Some(bad)),
                DEFAULT_QUIC_PROBE_TIMEOUT,
                "{bad}"
            );
        }
        // The documented derivation: RFC 9002 first PTO with kInitialRtt 333 ms.
        assert_eq!(QUIC_PROBE_RESEND.as_millis(), 333);
        let first_pto_ms = 333.0_f64 + 4.0 * 166.5;
        assert!((DEFAULT_QUIC_PROBE_TIMEOUT.as_millis() as f64 - first_pto_ms).abs() <= 1.0);
    }

    #[test]
    fn an_oversized_bound_does_not_overflow_the_deadline() {
        // A closed loopback port: answered at once, whatever the bound.
        let port = UdpSocket::bind("127.0.0.1:0")
            .and_then(|s| s.local_addr())
            .expect("bind")
            .port();
        let addr: SocketAddr = ([127, 0, 0, 1], port).into();
        // Before the clamp, `Duration::MAX` overflowed the deadline and panicked.
        assert_eq!(probe_quic(addr, Duration::MAX), QuicProbeOutcome::Refused);
        let out = probe_quic_locators(&[format!("udp/127.0.0.1:{port}?rel=1")], Duration::MAX);
        assert_eq!(
            out.get(&format!("udp/127.0.0.1:{port}?rel=1")),
            Some(&QuicProbeOutcome::Refused)
        );
    }

    #[test]
    fn a_locator_that_does_not_resolve_fails_without_sending() {
        let out = probe_quic_locators(
            &["udp/no-such-host.invalid:7683?rel=1".to_string()],
            Duration::from_millis(100),
        );
        assert!(
            matches!(
                out.get("udp/no-such-host.invalid:7683?rel=1"),
                Some(QuicProbeOutcome::Failed(_))
            ),
            "{out:?}"
        );
    }
}

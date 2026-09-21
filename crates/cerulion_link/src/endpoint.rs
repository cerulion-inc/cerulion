// SPDX-License-Identifier: MIT OR Apache-2.0
//! Endpoint construction, dialing, and accepting — dial-by-key QUIC.

use std::net::{IpAddr, SocketAddr};
use std::sync::Arc;

use iroh::endpoint::Connection;
use iroh::{Endpoint, EndpointAddr, EndpointId, SecretKey};

use crate::alpn;
use crate::error::LinkError;
use crate::relay::RelayConfig;

/// How to build a Cerulion iroh [`Endpoint`].
///
/// The device key IS the identity: the endpoint is constructed from an existing
/// 32-byte ed25519 secret, so its [`EndpointId`] (public key) is stable across
/// runs. Higher layers pair on that key.
#[derive(Clone)]
pub struct EndpointConfig {
    /// The 32-byte ed25519 secret key. Its public key becomes the endpoint's
    /// [`EndpointId`].
    pub secret_key: [u8; 32],
    /// ALPN protocols this endpoint accepts on incoming connections (defaults
    /// to [`alpn::ALL`] — both the wire and ops planes).
    pub alpns: Vec<Vec<u8>>,
    /// Relay configuration (defaults to [`RelayConfig::N0Default`]).
    pub relay: RelayConfig,
}

impl std::fmt::Debug for EndpointConfig {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let public = SecretKey::from_bytes(&self.secret_key).public();
        f.write_str("EndpointConfig { secret_key: [REDACTED], endpoint_id: \"")?;
        for byte in public.as_bytes() {
            write!(f, "{byte:02x}")?;
        }
        f.write_str("\" }")
    }
}

impl EndpointConfig {
    /// A config from a 32-byte device secret, accepting BOTH planes over the
    /// dev-default n0 relays.
    pub fn new(secret_key: [u8; 32]) -> Self {
        Self {
            secret_key,
            alpns: alpn::ALL.iter().map(|a| a.to_vec()).collect(),
            relay: RelayConfig::default(),
        }
    }

    /// Override the relay configuration (builder style).
    pub fn with_relay(mut self, relay: RelayConfig) -> Self {
        self.relay = relay;
        self
    }

    /// Override the accepted ALPN set (builder style).
    pub fn with_alpns(mut self, alpns: Vec<Vec<u8>>) -> Self {
        self.alpns = alpns;
        self
    }
}

/// Build and bind an [`Endpoint`] from an [`EndpointConfig`].
///
/// The endpoint's identity is derived from `config.secret_key` — its
/// [`EndpointId`] is that key's public half.
pub async fn build_endpoint(config: EndpointConfig) -> Result<Endpoint, LinkError> {
    let secret = SecretKey::from_bytes(&config.secret_key);
    let endpoint = config
        .relay
        .into_builder()
        .secret_key(secret)
        .alpns(config.alpns)
        // iroh reads the crypto provider from the BUILDER (not the rustls
        // process-default): `bind()` returns `InvalidCryptoProvider` if it is
        // unset. We supply ring — the provider the iroh tree already pulls —
        // so a bare consumer needn't wire this up.
        .crypto_provider(ring_crypto_provider())
        .bind()
        .await
        .map_err(|e| LinkError::Bind(Box::new(e)))?;
    tracing::info!(endpoint_id = %endpoint.id(), "cerulion_link endpoint bound");
    Ok(endpoint)
}

/// Dial a peer over `alpn`, returning the established [`Connection`].
///
/// `peer` may be a bare [`EndpointId`] (resolved via discovery — requires a
/// relay/discovery-enabled endpoint) or a full [`EndpointAddr`] carrying direct
/// addresses (works with [`RelayConfig::Disabled`] on the LAN). The returned
/// connection is mutually TLS-authenticated; its
/// [`Connection::remote_id`] is the peer's real key.
pub async fn dial(
    endpoint: &Endpoint,
    peer: impl Into<EndpointAddr>,
    alpn: &[u8],
) -> Result<Connection, LinkError> {
    let addr = peer.into();
    let peer_hex = crate::hex32(addr.id.as_bytes());
    let connection = endpoint
        .connect(addr, alpn)
        .await
        .map_err(|e| LinkError::Connect {
            peer: peer_hex,
            source: Box::new(e),
        })?;
    tracing::debug!(
        remote_id = %connection.remote_id(),
        "cerulion_link dialed peer"
    );
    Ok(connection)
}

/// An accepted, handshake-complete incoming connection plus its identity and
/// negotiated plane.
#[derive(Debug)]
pub struct Accepted {
    /// The established connection.
    pub connection: Connection,
    /// The peer's mutually-authenticated identity (from its TLS certificate).
    pub remote_id: EndpointId,
    /// The negotiated ALPN (which plane the dialer opened — compare against
    /// [`alpn::WIRE`] / [`alpn::OPS`]).
    pub alpn: Vec<u8>,
}

/// Accept ONE incoming connection.
///
/// Returns `Ok(None)` when the endpoint is closed (its accept stream ended) —
/// the loop-terminating signal. Otherwise completes the handshake and returns
/// the [`Accepted`] triple `(connection, remote_id, alpn)`. Drive it in a loop
/// to service many connections:
///
/// ```no_run
/// # async fn run(endpoint: &iroh::Endpoint) -> Result<(), cerulion_link::LinkError> {
/// while let Some(accepted) = cerulion_link::accept_one(endpoint).await? {
///     // dispatch on accepted.alpn / accepted.remote_id …
/// }
/// # Ok(())
/// # }
/// ```
pub async fn accept_one(endpoint: &Endpoint) -> Result<Option<Accepted>, LinkError> {
    let Some(incoming) = endpoint.accept().await else {
        // The endpoint has been closed; no more connections will arrive.
        return Ok(None);
    };
    let connection = incoming.await.map_err(|e| LinkError::Accept(Box::new(e)))?;
    let remote_id = connection.remote_id();
    let alpn = connection.alpn().to_vec();
    tracing::debug!(remote_id = %remote_id, "cerulion_link accepted connection");
    Ok(Some(Accepted {
        connection,
        remote_id,
        alpn,
    }))
}

/// Assert that a connection's mutually-authenticated peer identity matches an
/// expected 32-byte device key.
///
/// [`Connection::remote_id`] is derived from the peer's TLS certificate, so
/// iroh already enforces cert-subject == remote_id. This helper adds the
/// application-level check the higher layers need: remote_id == the expected
/// paired device key. Returns the confirmed [`EndpointId`] on success, or
/// [`LinkError::IdentityMismatch`] otherwise.
pub fn assert_peer_identity(
    connection: &Connection,
    expected: &[u8; 32],
) -> Result<EndpointId, LinkError> {
    let remote = connection.remote_id();
    if remote.as_bytes() == expected {
        Ok(remote)
    } else {
        Err(LinkError::IdentityMismatch {
            expected: crate::hex32(expected),
            actual: crate::hex32(remote.as_bytes()),
        })
    }
}

/// The rustls `ring` crypto provider iroh's TLS builds on.
///
/// iroh's endpoint builder REQUIRES a provider (its `bind()` errors with
/// `InvalidCryptoProvider` otherwise) and reads it from the builder, not the
/// rustls process-default. The iroh dependency tree pulls `ring`, so ring is
/// the provider we hand it; its AES-GCM + ed25519 support satisfies iroh's
/// raw-public-key QUIC TLS.
fn ring_crypto_provider() -> Arc<rustls::crypto::CryptoProvider> {
    Arc::new(rustls::crypto::ring::default_provider())
}

/// Build an [`EndpointAddr`] from an identity plus a set of direct socket
/// addresses — the LAN / direct-dial form that needs no discovery.
///
/// The addresses are attached verbatim; the caller is responsible for passing
/// dialable addresses (a bound `0.0.0.0` socket is not dialable — resolve it to
/// a concrete interface / loopback address first).
pub fn direct_addr(id: EndpointId, addrs: impl IntoIterator<Item = SocketAddr>) -> EndpointAddr {
    let mut addr = EndpointAddr::new(id);
    for socket in addrs {
        addr = addr.with_ip_addr(socket);
    }
    addr
}

/// The endpoint's actually-BOUND UDP port — the value the robot advertises for
/// LAN direct-dial (the mDNS `iroh_port` TXT key). `None` only if the endpoint
/// has no bound socket carrying a non-zero port (never expected after a
/// successful [`build_endpoint`]).
///
/// iroh binds one socket per address family (`0.0.0.0:P` and/or `[::]:P`,
/// usually the SAME `P`); this returns the first bound port, so the caller can
/// pair it with a concrete discovered IP via [`lan_direct_dial_addr`].
pub fn bound_port(endpoint: &Endpoint) -> Option<u16> {
    pick_bound_port(&endpoint.bound_sockets())
}

/// Pick a single UDP port from an endpoint's bound sockets: the first with a
/// non-zero port (an unbound / ephemeral-unresolved socket reports port 0).
/// Pure so it is oracle-testable without binding a real endpoint.
pub(crate) fn pick_bound_port(sockets: &[SocketAddr]) -> Option<u16> {
    sockets.iter().map(SocketAddr::port).find(|&p| p != 0)
}

/// Build a LAN direct-dial [`EndpointAddr`] from a discovered peer's advertised
/// parts — the pure target-assembly the LAN client dials with
/// [`RelayConfig::Disabled`](crate::RelayConfig). Nothing dials here.
///
/// - `eid` — the peer's hex [`EndpointId`] (the mDNS `eid=` TXT value or a
///   pinned key). Bad hex / wrong length / a non-canonical key →
///   [`LinkError::BadEndpointId`].
/// - `ip` — the peer's resolved IP (from its mDNS SRV / A / AAAA record).
/// - `iroh_port` — the peer's iroh UDP port (the mDNS `iroh_port=` TXT value,
///   DISTINCT from the zenoh SRV/TCP port). `None` OR `Some(0)` (the peer
///   advertised no usable port — a bogus / hand-edited facts file could carry
///   `0`, which is not a dialable target) → [`LinkError::MissingIrohPort`],
///   signalling the caller to fall back to relay / discovery dialing rather than
///   building a dead `ip:0` address.
///
/// Never panics — every failure is a typed [`LinkError`].
pub fn lan_direct_dial_addr(
    eid: &str,
    ip: IpAddr,
    iroh_port: Option<u16>,
) -> Result<EndpointAddr, LinkError> {
    let id = parse_endpoint_id(eid)?;
    // `Some(0)` is not a dialable port — treat it like an absent one so the
    // caller degrades to relay/discovery rather than a dead `ip:0` target.
    let port = iroh_port
        .filter(|&p| p != 0)
        .ok_or(LinkError::MissingIrohPort)?;
    Ok(direct_addr(id, [SocketAddr::new(ip, port)]))
}

/// Parse a hex-encoded 32-byte [`EndpointId`] (an mDNS `eid=` TXT value or a
/// pinned key). Bad hex, the wrong length, or a non-canonical ed25519 key →
/// [`LinkError::BadEndpointId`]. Never panics.
pub fn parse_endpoint_id(eid: &str) -> Result<EndpointId, LinkError> {
    let bytes = decode_hex32(eid).map_err(|reason| LinkError::BadEndpointId {
        eid: eid.to_string(),
        reason,
    })?;
    EndpointId::from_bytes(&bytes).map_err(|e| LinkError::BadEndpointId {
        eid: eid.to_string(),
        reason: format!("not a valid ed25519 public key: {e}"),
    })
}

/// Decode exactly 64 hex chars into 32 bytes (dependency-free — this permissive
/// crate carries no `hex` dep). Wrong length / non-hex char → a diagnosable
/// message.
fn decode_hex32(s: &str) -> Result<[u8; 32], String> {
    if s.len() != 64 {
        return Err(format!(
            "expected 64 hex characters (32 bytes), got {}",
            s.len()
        ));
    }
    let mut out = [0u8; 32];
    for (i, chunk) in s.as_bytes().as_chunks::<2>().0.iter().enumerate() {
        let hi = hex_nibble(chunk[0]).ok_or_else(|| {
            format!(
                "non-hex character '{}' at position {}",
                chunk[0] as char,
                i * 2
            )
        })?;
        let lo = hex_nibble(chunk[1]).ok_or_else(|| {
            format!(
                "non-hex character '{}' at position {}",
                chunk[1] as char,
                i * 2 + 1
            )
        })?;
        out[i] = (hi << 4) | lo;
    }
    Ok(out)
}

/// One hex digit → its 0..=15 value, or `None` for a non-hex byte.
fn hex_nibble(b: u8) -> Option<u8> {
    match b {
        b'0'..=b'9' => Some(b - b'0'),
        b'a'..=b'f' => Some(b - b'a' + 10),
        b'A'..=b'F' => Some(b - b'A' + 10),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::{Ipv4Addr, Ipv6Addr};

    /// The public key an `iroh::SecretKey::from_bytes(secret)` yields — an
    /// INDEPENDENT identity oracle (does not go through the parse path).
    fn public_hex_of(secret: &[u8; 32]) -> String {
        let pk = SecretKey::from_bytes(secret).public();
        crate::hex32(pk.as_bytes())
    }

    #[test]
    fn endpoint_config_debug_redacts_the_secret_and_reports_only_public_identity() {
        // RFC 8032, section 7.1, test vector 1: independent public-key oracle.
        let seed = [
            0x9d, 0x61, 0xb1, 0x9d, 0xef, 0xfd, 0x5a, 0x60, 0xba, 0x84, 0x4a, 0xf4, 0x92, 0xec,
            0x2c, 0xc4, 0x44, 0x49, 0xc5, 0x69, 0x7b, 0x32, 0x69, 0x19, 0x70, 0x3b, 0xac, 0x03,
            0x1c, 0xae, 0x7f, 0x60,
        ];
        let expected_public = "d75a980182b10ab7d54bfed3c964073a0ee172f3daa62325af021a68f707511a";
        let secret_hex = "9d61b19deffd5a60ba844af492ec2cc44449c5697b326919703bac031cae7f60";
        let config = EndpointConfig::new(seed);
        let compact = format!("{config:?}");
        let pretty = format!("{config:#?}");
        assert_eq!(
            compact,
            format!(
                "EndpointConfig {{ secret_key: [REDACTED], endpoint_id: \"{expected_public}\" }}"
            )
        );
        for output in [compact, pretty] {
            assert!(output.contains("[REDACTED]"));
            assert!(output.contains(expected_public));
            assert!(!output.contains(secret_hex));
            assert!(!output.contains(&format!("{seed:?}")));
            assert!(!output.contains(&format!("{seed:#?}")));
        }
    }

    #[test]
    fn pick_bound_port_takes_the_first_non_zero_port() {
        // Empty → None; an all-zero-port set → None (nothing bound); the first
        // non-zero port wins, skipping a leading unresolved (:0) socket.
        assert_eq!(pick_bound_port(&[]), None);
        let zero_v6 = SocketAddr::new(IpAddr::V6(Ipv6Addr::UNSPECIFIED), 0);
        let zero_v4 = SocketAddr::new(IpAddr::V4(Ipv4Addr::UNSPECIFIED), 0);
        assert_eq!(pick_bound_port(&[zero_v6, zero_v4]), None);
        let bound_v4 = SocketAddr::new(IpAddr::V4(Ipv4Addr::UNSPECIFIED), 41234);
        assert_eq!(pick_bound_port(&[bound_v4]), Some(41234));
        // A leading :0 v6 socket is skipped for the bound v4 sibling.
        assert_eq!(pick_bound_port(&[zero_v6, bound_v4]), Some(41234));
    }

    #[test]
    fn parse_endpoint_id_round_trips_a_real_public_key() {
        // Oracle: the hex of a real ed25519 public key parses back to the same
        // EndpointId bytes.
        let hex = public_hex_of(&[7u8; 32]);
        let id = parse_endpoint_id(&hex).expect("a real public key parses");
        assert_eq!(crate::hex32(id.as_bytes()), hex);
    }

    #[test]
    fn parse_endpoint_id_rejects_bad_hex_and_wrong_length() {
        // Wrong length (too short).
        let short = parse_endpoint_id("dead").unwrap_err();
        assert!(matches!(short, LinkError::BadEndpointId { .. }));
        assert!(short.to_string().contains("expected 64 hex"), "{short}");
        // Wrong length (too long — 66 chars).
        let long = parse_endpoint_id(&"a".repeat(66)).unwrap_err();
        assert!(long.to_string().contains("expected 64 hex"), "{long}");
        // A non-hex character (64 chars, but 'z' is not hex) — the offending
        // char + position ride the message.
        let bad = parse_endpoint_id(&"z".repeat(64)).unwrap_err();
        assert!(matches!(bad, LinkError::BadEndpointId { .. }));
        assert!(bad.to_string().contains("non-hex character"), "{bad}");
        // NOTE: a structurally-hex-valid-but-non-canonical ed25519 key is
        // rejected by iroh's `EndpointId::from_bytes` (the second guard in
        // `parse_endpoint_id`), but ed25519-dalek accepts many "weak" encodings
        // by default, so there is no STABLE 32-byte input to pin that arm
        // without coupling to the crypto lib's canonicalization policy.
    }

    #[test]
    fn lan_direct_dial_addr_builds_the_expected_target() {
        // A real key + a concrete IP + an iroh_port → an EndpointAddr carrying
        // exactly that id and that one socket address (the hand oracle).
        let hex = public_hex_of(&[9u8; 32]);
        let ip = IpAddr::V4(Ipv4Addr::new(192, 168, 1, 7));
        let addr = lan_direct_dial_addr(&hex, ip, Some(41234)).expect("builds");
        assert_eq!(crate::hex32(addr.id.as_bytes()), hex, "id preserved");
        let sockets: Vec<SocketAddr> = addr.ip_addrs().copied().collect();
        assert_eq!(
            sockets,
            vec![SocketAddr::new(ip, 41234)],
            "one direct socket"
        );
    }

    #[test]
    fn lan_direct_dial_addr_missing_or_zero_port_is_a_typed_error() {
        let hex = public_hex_of(&[3u8; 32]);
        let ip = IpAddr::V4(Ipv4Addr::LOCALHOST);
        // Absent port → the relay-fallback signal.
        let none = lan_direct_dial_addr(&hex, ip, None).unwrap_err();
        assert!(matches!(none, LinkError::MissingIrohPort), "{none:?}");
        // A bogus port 0 (hand-edited facts file) is NOT a dialable target —
        // it degrades to the SAME relay-fallback signal, never a dead ip:0.
        let zero = lan_direct_dial_addr(&hex, ip, Some(0)).unwrap_err();
        assert!(matches!(zero, LinkError::MissingIrohPort), "{zero:?}");
    }

    #[test]
    fn lan_direct_dial_addr_bad_eid_is_a_typed_error_before_any_port_check() {
        // A malformed eid is rejected even when a valid port is supplied.
        let err =
            lan_direct_dial_addr("not-hex", IpAddr::V4(Ipv4Addr::LOCALHOST), Some(1)).unwrap_err();
        assert!(matches!(err, LinkError::BadEndpointId { .. }), "{err:?}");
    }
}

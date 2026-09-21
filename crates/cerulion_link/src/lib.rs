// SPDX-License-Identifier: MIT OR Apache-2.0
//! # cerulion_link
//!
//! A thin wrapper over [iroh](https://docs.rs/iroh) giving Cerulion
//! **dial-by-key QUIC**: an endpoint's identity IS its device key, so peers
//! connect by key alone. This is the REMOTE plane of Cerulion's dual-plane
//! gateway (**zenoh on the LAN, iroh across the internet**), over which it
//! tunnels **raw Cerulion wire frames verbatim** (zenoh is never tunneled over iroh;
//! this crate never touches zenoh, and it never depends on `cerulion_core`).
//!
//! ## The pieces
//!
//! - [`build_endpoint`]: bind an [`Endpoint`] from a 32-byte
//!   ed25519 device secret (`SecretKey::from_bytes`; the endpoint's
//!   [`EndpointId`] is that key's public half).
//! - [`dial`]: connect to a peer (by [`EndpointId`] or a
//!   direct [`EndpointAddr`]) over an ALPN plane.
//! - [`accept_one`]: the accept-loop primitive, yielding
//!   `(connection, remote_id, alpn)` as an [`Accepted`].
//! - [`assert_peer_identity`]: bind a connection's mutually-authenticated
//!   peer key to an expected paired device key (the cert-subject == remote_id
//!   check the higher layers assert).
//! - [`alpn`]: the ALPN plane constants ([`alpn::WIRE`], [`alpn::OPS`]).
//! - [`RelayConfig`]: the data-driven relay seam (n0 dev relays / disabled /
//!   self-hosted prod URLs; no prod URLs hardcoded).
//!   [`RelayConfig::resolve`] is the ONE shared precedence helper that turns
//!   `disabled`, the environment and the config into a `RelayConfig` (robot + desk client; a bad URL is a loud
//!   [`RelayParseError`], never a silent default).
//! - [`bound_port`]: the endpoint's bound UDP port (the mDNS `iroh_port` a
//!   robot advertises for LAN direct-dial).
//! - [`lan_direct_dial_addr`] / [`parse_endpoint_id`]: build a LAN direct-dial
//!   [`EndpointAddr`] from a discovered peer's hex `eid` + IP + `iroh_port`
//!   (pure; nothing dials).
//! - [`open_frame_stream`] / [`accept_frame_stream`] + [`write_frame`] /
//!   [`read_frame`]: length-prefixed bidi framing suitable for carrying raw
//!   Cerulion wire frames.
//! - [`open_uni_frame_stream`] / [`accept_uni_frame_stream`]: the one-way
//!   (robot to desk) sibling stream primitive for the remote wire plane's
//!   per-topic data streams.
//! - [`QuicOpsStream`]: a synchronous [`Read`](std::io::Read) +
//!   [`Write`](std::io::Write) adapter over a QUIC bidi stream (channel-backed,
//!   cancel-safe by construction) bridging async QUIC to a synchronous ops
//!   consumer.
//!
//! ## Example
//!
//! ```no_run
//! use cerulion_link::{accept_one, alpn, build_endpoint, EndpointConfig, RelayConfig};
//!
//! # async fn run() -> Result<(), cerulion_link::LinkError> {
//! let endpoint = build_endpoint(
//!     EndpointConfig::new([7u8; 32]).with_relay(RelayConfig::N0Default),
//! )
//! .await?;
//!
//! while let Some(accepted) = accept_one(&endpoint).await? {
//!     if accepted.alpn == alpn::WIRE {
//!         // tunnel raw wire frames on accepted.connection
//!     }
//! }
//! # Ok(())
//! # }
//! ```

// P12 (the project logging rule): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

pub mod alpn;
mod endpoint;
mod error;
mod framing;
mod ops_stream;
mod relay;

pub use endpoint::{
    accept_one, assert_peer_identity, bound_port, build_endpoint, dial, direct_addr,
    lan_direct_dial_addr, parse_endpoint_id, Accepted, EndpointConfig,
};
pub use error::{BoxError, LinkError};
pub use framing::{
    accept_frame_stream, accept_uni_frame_stream, open_frame_stream, open_uni_frame_stream,
    read_frame, write_frame, DEFAULT_MAX_FRAME_LEN,
};
pub use ops_stream::QuicOpsStream;
pub use relay::{RelayConfig, RelayParseError, CERULION_RELAY_URL_ENV};

// Re-export the iroh types that appear in this crate's public API, so callers
// need not add an iroh dependency (or risk a version skew) just to name them.
pub use iroh::endpoint::{Connection, RecvStream, SendStream};
pub use iroh::{Endpoint, EndpointAddr, EndpointId, RelayUrl};

/// Hex-encode a 32-byte key for error/log display (no external dep).
pub(crate) fn hex32(bytes: &[u8]) -> String {
    use std::fmt::Write;
    let mut s = String::with_capacity(bytes.len() * 2);
    for b in bytes {
        // Infallible: writing to a String never errors.
        let _ = write!(s, "{b:02x}");
    }
    s
}

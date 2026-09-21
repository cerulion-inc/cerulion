// SPDX-License-Identifier: MIT OR Apache-2.0
//! ALPN protocol identifiers for Cerulion's iroh planes.
//!
//! ALPN (Application-Layer Protocol Negotiation) is negotiated during the QUIC
//! TLS handshake; the accepting endpoint sees which plane a dialer opened via
//! [`Connection::alpn`](iroh::endpoint::Connection::alpn). Keeping these in ONE
//! place makes the wire contract auditable. The trailing `/1` is the plane's
//! protocol-version boundary.

/// The raw wire-frame tunnel: carries Cerulion [`WireHeader`]-framed frames
/// VERBATIM over an iroh bidi stream (the remote half of the dual-plane
/// gateway — zenoh on the LAN, iroh across the internet).
///
/// [`WireHeader`]: https://docs.rs/cerulion-wire
pub const WIRE: &[u8] = b"cerulion/wire/1";

/// The ops plane (cerud — the Cerulion control daemon).
pub const OPS: &[u8] = b"cerulion/ops/1";

/// Every ALPN this crate defines, in a form suitable for
/// [`Builder::alpns`](iroh::endpoint::Builder::alpns) — an endpoint built with
/// this accepts BOTH planes.
pub const ALL: &[&[u8]] = &[WIRE, OPS];

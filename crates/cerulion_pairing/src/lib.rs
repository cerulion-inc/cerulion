// SPDX-License-Identifier: MIT OR Apache-2.0
//! # cerulion_pairing
//!
//! Offline-verifiable pairing identity for Cerulion robots. This crate contains
//! only **formats, cryptography, and state machines**: there is **no network
//! I/O and no iroh dependency**. The transport channel is abstracted: the
//! already-authenticated peer key and any channel-binding material are passed in
//! as parameters, so this crate embeds cleanly in firmware (the verifier) and in
//! the closed Studio app (the client library).
//!
//! ## Identity model
//!
//! The device key **is** the transport identity: an ed25519 keypair whose 32-byte
//! public key doubles as the iroh `EndpointId`. The transport layer (elsewhere)
//! mutually authenticates the raw keys via TLS 1.3. App-layer pairing then:
//!
//! 1. verifies the offline certificate chain (root set, then intermediate, then
//!    device cert and grant), and
//! 2. asserts `device_cert.device_key == already_authenticated_peer_key`
//!    (the [`verify`] API takes the authenticated peer key as an input).
//!
//! The chain is: an air-gapped **M-of-N root set** (robots trust a *set*, not one
//! key) → a rotatable online **intermediate** → short-lived **device certs**
//! ("device key K belongs to account X until D") and issuer-signed **grants**
//! ("account B may access robot R"). Grants travel *with* the client; the robot
//! never polls the cloud. Ownership is attested at the **account** level, so
//! account-key rotation never strands robots.
//!
//! ## Revocation is not expiry
//!
//! A robot-local, durable **access list** is the source of truth: established
//! pairings never rot offline. Short-lived certs gate *new* pairings only.
//! Revocation propagates via a monotonic, signed **access-list epoch** (CRL-lite)
//! that every trusted client pushes on connect, plus direct owner "revoke-now".
//! A **tamper-evident MAC'd trust store** persists an **anti-rollback high-water**
//! timestamp so a clock-rollback cannot resurrect expired certs.
//!
//! ## Fallback pairing ceremony
//!
//! When the strong path is unavailable, a deliberate (never auto-negotiated)
//! **CPace** ceremony runs *inside* the already-authenticated channel with both
//! transport identities transcript-bound. It enforces bounded attempts +
//! code-burn + a short TTL + single session ([`pake`]).
//!
//! ## Modules
//!
//! | Module | Role |
//! |--------|------|
//! | [`mod@format`] | Cert / grant / epoch types + canonical deterministic signing bytes + serde |
//! | [`mod@verify`] | Robot-side chain verifier + access list + MAC'd, anti-rollback trust store |
//! | [`mod@pake`]   | CPace ceremony state machine (bounded attempts, TTL, code-burn) |
//! | [`mod@pop`]    | Proof-of-possession gesture: sign/verify a device-key challenge |
//! | [`mod@client`] | Device-side key seam, cert carry, and pairing driver |

// P12 (Logging, see AGENTS.md): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

pub mod client;
mod crypto;
mod error;
pub mod format;
pub mod pake;
pub mod pop;
pub mod verify;

pub use error::{CertKind, PairingError};

/// Maximum delegation depth the format permits. The format decides this on day
/// one: depth is **encoded** in every grant and **capped** at verify time. A
/// grant with `delegation_depth > MAX_DELEGATION_DEPTH` is rejected loudly.
pub const MAX_DELEGATION_DEPTH: u8 = 1;

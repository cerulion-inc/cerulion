// SPDX-License-Identifier: AGPL-3.0-only
//! Consent resolver and property guard for the Cerulion desk surfaces (the
//! `cerulion` CLI and `cerulion-vizd`).
//!
//! * **Feature `posthog` off (the default): every API is a no-op.**
//!   [`consent::status`] reports [`consent::Source::NotCompiled`], no file is
//!   read or written, and the crate pulls no network or serialization
//!   dependency. Robot and runtime crates never depend on this crate at all
//!   (`tests/hot_path_dependency_gate.rs`).
//! * **Feature `posthog` on:** [`consent`] resolves enabled or disabled
//!   (`DO_NOT_TRACK` > `CERULION_TELEMETRY` > `telemetry.json` > default on),
//!   and `payload` renders PostHog `/batch` JSON from events that were
//!   guarded when they were built.
//! * **Every property is guarded** (the `guard` module): keys must be in the
//!   event's [`Allowlist`], and string values may not look like URLs, emails
//!   or paths, nor exceed 128 chars. A rejected property is dropped and
//!   counted, never sent.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

pub mod consent;
mod error;
pub mod guard;
#[cfg(feature = "posthog")]
pub mod payload;
pub mod rfc3339;
mod value;

pub use error::Error;
pub use value::{Allowlist, Common, EventSpec, Props, Value};

use sha2::{Digest, Sha256};

/// The library name PostHog sees as `$lib`.
pub const LIB_NAME: &str = "cerulion_telemetry";
/// The library version PostHog sees as `$lib_version`.
pub const LIB_VERSION: &str = env!("CARGO_PKG_VERSION");

/// Hash an identifier before sending it: the first 12 hex chars of SHA-256,
/// so the same input hashes the same on every surface.
///
/// ```
/// assert_eq!(cerulion_telemetry::url_hash("https://example.com/a"), "2dce0a4c5044");
/// ```
pub fn url_hash(url: &str) -> String {
    use std::fmt::Write;
    let digest = Sha256::digest(url.as_bytes());
    let mut out = String::with_capacity(12);
    for byte in &digest[..6] {
        let _ = write!(out, "{byte:02x}");
    }
    out
}

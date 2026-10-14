// SPDX-License-Identifier: AGPL-3.0-only
//! Embedded runtime and JNI support for Android applications.

/// Runtime configuration and input validation.
pub mod config;
/// JSON-to-wire-frame encoding for app publishers.
pub mod encode;
/// Stable error types and error-code constants.
pub mod error;
/// Synchronous runtime lifecycle and status types.
pub mod runtime;

/// Stable error-code constants.
pub use error::{codes, EmbedError};
/// Embedded runtime API and observable status types.
pub use runtime::{EmbeddedRuntime, Status, StopSummary};

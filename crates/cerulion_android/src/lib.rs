// SPDX-License-Identifier: AGPL-3.0-only
//! Embedded runtime and JNI support for Android applications.
//!
//! Publishing parses JSON and allocates; this is an app-side convenience path, not the zero-copy node path.

/// Runtime configuration and input validation.
pub mod config;
/// JSON-to-wire-frame encoding for app publishers.
pub mod encode;
/// Stable error types and error-code constants.
pub mod error;
/// Synchronous runtime lifecycle and status types.
pub mod runtime;

mod host;
mod jni;

/// Embedded runtime API and observable status types.
pub use config::LanAccessConfig;
/// Stable error-code constants.
pub use error::{codes, EmbedError};
pub use runtime::{EmbeddedRuntime, RecorderSummary, RuntimeState, Status};

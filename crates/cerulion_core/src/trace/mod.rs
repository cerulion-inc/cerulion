// SPDX-License-Identifier: AGPL-3.0-only
//! Publish trace — metadata-only ring buffer of recent publishes
//! (Regime A).
//!
//! # Scope
//!
//! This module captures **metadata only** about each publish event
//! (topic, sequence, timestamp, schema hash) — not the wire payload.
//! Wire-payload persistence ("bagging") is Regime B and is a separate
//! concern (the `cerulion_bag` crate owns it).
//!
//! Entry size is fixed at 32 bytes (one `Arc<str>` + three integers),
//! so the ring buffer scales with **publish rate, not payload size**.
//! A 4 MB image topic at 30 Hz incurs the same trace overhead as a
//! 16-byte pose topic at 30 Hz.
//!
//! # Memory budget examples
//!
//! | Workload | Trace memory for 1 hour |
//! |---|---|
//! | 1 publisher × 60 Hz | ~6.6 MB |
//! | 10 publishers × 60 Hz | ~66 MB |
//! | 1 publisher × 1000 Hz | ~110 MB |
//!
//! Time-windowed depth (`HistoryDepth::Window(Duration)`) is the
//! recommended default — the trace size scales only with rate, not
//! with wall-clock retention beyond the window.
//!
//! # Pieces
//!
//! [`publish`] is the in-memory ring buffer and the type surface. [`bag`]
//! is the disk writer. A publisher pushes entries once a trace is attached
//! to it, and the `cerulion trace inspect` CLI reads the files back.

pub mod bag;
pub mod publish;

pub use bag::{BagRetention, BagWriter};
pub use publish::{HistoryDepth, PublishTrace, PublishTraceEntry};

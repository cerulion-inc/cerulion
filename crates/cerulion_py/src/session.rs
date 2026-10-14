// SPDX-License-Identifier: AGPL-3.0-only
//! Process-wide transport entry points: `connect()` and `real_ns()`.
//!
//! There is exactly ONE `TransportManager` per process (core singleton);
//! `connect` is a thin map over `TransportManager::init`, which is itself
//! idempotent - a second call returns the existing manager.

use crate::errors::map_transport_err;
use cerulion_core::{TransportConfig, TransportManager};
use pyo3::prelude::*;

/// Initialise the process-wide Cerulion transport under `node_name`.
///
/// Idempotent: the core singleton returns the already-initialised
/// manager on repeat calls.
#[pyfunction]
pub fn connect(node_name: &str) -> PyResult<()> {
    TransportManager::init(TransportConfig {
        node_name: node_name.to_string(),
        ..Default::default()
    })
    .map_err(map_transport_err)?;
    Ok(())
}

/// Nanoseconds on the same stable monotonic clock the transport stamps
/// frames with.
#[pyfunction]
pub fn real_ns() -> u64 {
    cerulion_core::clock::real_ns()
}

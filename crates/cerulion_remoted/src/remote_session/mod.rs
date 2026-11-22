// SPDX-License-Identifier: AGPL-3.0-only
//! Process identity, bounded cleanup and resource configuration for caller-owned sessions.

mod limits;

pub use limits::SessionLimits;

/// Linux process identity and bounded cleanup for a caller-owned session.
#[cfg(target_os = "linux")]
pub mod processes;

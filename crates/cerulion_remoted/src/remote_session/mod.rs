// SPDX-License-Identifier: AGPL-3.0-only
//! Process identity and bounded cleanup for caller-owned sessions.

/// Linux process identity and bounded cleanup for a caller-owned session.
#[cfg(target_os = "linux")]
pub mod processes;

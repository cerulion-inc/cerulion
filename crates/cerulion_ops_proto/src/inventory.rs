// SPDX-License-Identifier: MIT OR Apache-2.0
//! The result payload of the `inventory` verb.
//!
//! `cerud`'s `inventory` verb fills this from best-effort host probes and
//! returns it as the `result` of a [`crate::protocol::Response::Ok`]; a client
//! reads that `result` back into this type.

use serde::{Deserialize, Serialize};

/// The robot's platform inventory. Absent facts are `None`.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Inventory {
    /// Target architecture (`std::env::consts::ARCH`, e.g. `aarch64`).
    pub arch: String,
    /// Operating system (`std::env::consts::OS`, e.g. `linux`).
    pub os: String,
    /// Kernel release (`uname -r`), or `None` if it could not be read.
    pub kernel: Option<String>,
    /// glibc version, or `None` (e.g. non-Linux / musl / read failed).
    pub glibc: Option<String>,
    /// CUDA toolkit version, or `None` if not present.
    pub cuda: Option<String>,
    /// NVIDIA JetPack / L4T (Tegra) release, or `None` if not a Jetson.
    pub jetpack_l4t: Option<String>,
    /// Free bytes on the root filesystem, or `None` if it could not be read.
    pub disk_free_bytes: Option<u64>,
    /// The Cerulion version the responding `cerud` build ships.
    pub cerulion_version: String,
}

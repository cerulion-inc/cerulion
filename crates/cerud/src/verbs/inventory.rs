// SPDX-License-Identifier: AGPL-3.0-only
//! The `inventory` verb: report the robot's platform facts (arch, os/kernel,
//! glibc, CUDA/JetPack/L4T, disk free, cerulion version).
//!
//! All probes are BEST-EFFORT and never guess: a fact that cannot be determined is
//! `None`, never fabricated (Principle #13). This gates deploy validation
//! later (a bundle's `target_glibc` vs the robot's actual glibc, etc.).
//!
//! The probes are split into pure parsers (`parse_*`, oracle-tested against
//! real tool-output formats) and thin OS-facing collectors, so the parsing
//! logic is verified without a specific host.

use serde::{Deserialize, Serialize};

use crate::error::CerudResult;
use crate::verbs::VerbHandler;

/// The `inventory` verb.
#[derive(Debug, Default, Clone, Copy)]
pub struct InventoryVerb;

impl VerbHandler for InventoryVerb {
    fn name(&self) -> &'static str {
        "inventory"
    }

    fn is_mutating(&self) -> bool {
        false // read-only platform probe
    }

    fn execute(&self, _args: &serde_json::Value) -> CerudResult<serde_json::Value> {
        Ok(serde_json::to_value(probe_inventory())?)
    }
}

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
    /// The Cerulion version this `cerud` build ships.
    pub cerulion_version: String,
}

/// Assemble an [`Inventory`] from the running host (best-effort probes).
pub fn probe_inventory() -> Inventory {
    Inventory {
        arch: std::env::consts::ARCH.to_string(),
        os: std::env::consts::OS.to_string(),
        kernel: kernel_release(),
        glibc: probe_glibc(),
        cuda: probe_cuda(),
        jetpack_l4t: probe_l4t(),
        disk_free_bytes: statvfs_avail_bytes(std::path::Path::new("/")),
        cerulion_version: env!("CARGO_PKG_VERSION").to_string(),
    }
}

// ─────────────────────────────── Pure parsers ──────────────────────────────

/// Extract a glibc version from the first line of `ldd --version`, e.g.
/// `"ldd (Ubuntu GLIBC 2.31-0ubuntu9.9) 2.31"` → `Some("2.31")`. The version
/// is the trailing whitespace-separated dotted-numeric token.
pub fn parse_ldd_version(first_line: &str) -> Option<String> {
    let token = first_line.split_whitespace().last()?;
    if looks_like_version(token) {
        Some(token.to_string())
    } else {
        None
    }
}

/// Extract a CUDA version from a `version.txt`-style line, e.g.
/// `"CUDA Version 12.2.140"` → `Some("12.2.140")`.
pub fn parse_cuda_version_txt(content: &str) -> Option<String> {
    for line in content.lines() {
        // Find the token after the word "Version".
        let mut it = line.split_whitespace().peekable();
        while let Some(w) = it.next() {
            if w.eq_ignore_ascii_case("version") {
                if let Some(tok) = it.peek() {
                    if looks_like_version(tok) {
                        return Some((*tok).to_string());
                    }
                }
            }
        }
    }
    None
}

/// Extract an L4T release from `/etc/nv_tegra_release`, e.g. the first line
/// `"# R35 (release), REVISION: 4.1, GCID: ..., BOARD: ..."` →
/// `Some("35.4.1")` (major from `R<n>`, minor from `REVISION: <x.y>`).
pub fn parse_l4t_release(content: &str) -> Option<String> {
    let line = content.lines().next()?;
    // Major: the first token shaped `R<digits>` (e.g. `R35` → `35`).
    let major: String = line.split_whitespace().find_map(|tok| {
        let rest = tok.strip_prefix('R')?;
        let digits: String = rest.chars().take_while(|c| c.is_ascii_digit()).collect();
        if digits.is_empty() {
            None
        } else {
            Some(digits)
        }
    })?;
    // Revision: the token after "REVISION:" (strip a trailing comma).
    let mut revision: Option<String> = None;
    let toks: Vec<&str> = line.split_whitespace().collect();
    for (i, tok) in toks.iter().enumerate() {
        if tok.eq_ignore_ascii_case("REVISION:") {
            if let Some(next) = toks.get(i + 1) {
                revision = Some(next.trim_end_matches(',').to_string());
            }
        }
    }
    match revision {
        Some(rev) if looks_like_version(&rev) => Some(format!("{major}.{rev}")),
        _ => Some(major),
    }
}

/// Whether a token looks like a dotted-numeric version (digits + `.` only,
/// at least one digit).
fn looks_like_version(token: &str) -> bool {
    !token.is_empty()
        && token.chars().any(|c| c.is_ascii_digit())
        && token.chars().all(|c| c.is_ascii_digit() || c == '.')
}

// ───────────────────────────── OS-facing probes ────────────────────────────

/// Kernel release via `uname(2)`.
#[cfg(unix)]
fn kernel_release() -> Option<String> {
    // SAFETY: `uname` fills a zeroed `utsname`; on success we read the
    // NUL-terminated `release` field. The struct is owned + sized here.
    let mut uts: libc::utsname = unsafe { std::mem::zeroed() };
    let rc = unsafe { libc::uname(&mut uts) };
    if rc != 0 {
        return None;
    }
    let release: String = uts
        .release
        .iter()
        .take_while(|&&c| c != 0)
        .map(|&c| c as u8 as char)
        .collect();
    if release.is_empty() {
        None
    } else {
        Some(release)
    }
}

#[cfg(not(unix))]
fn kernel_release() -> Option<String> {
    None
}

/// Free bytes on the filesystem containing `path`, via `statvfs(3)`.
#[cfg(unix)]
fn statvfs_avail_bytes(path: &std::path::Path) -> Option<u64> {
    use std::os::unix::ffi::OsStrExt;
    let cpath = std::ffi::CString::new(path.as_os_str().as_bytes()).ok()?;
    // SAFETY: `cpath` is a valid NUL-terminated path; `stat` is zeroed and
    // written by `statvfs`. We only read scalar fields on success.
    let mut stat: libc::statvfs = unsafe { std::mem::zeroed() };
    let rc = unsafe { libc::statvfs(cpath.as_ptr(), &mut stat) };
    if rc != 0 {
        return None;
    }
    // Widening casts, kept under a scoped allow: `f_bavail` is `fsblkcnt_t`
    // (`c_uint` on Apple, `c_ulong` = u32 on 32-bit glibc) and `f_frsize` is
    // `c_ulong` (u32 on 32-bit), so both widen on those targets and are no-ops
    // on 64-bit Linux/macOS. `allow` rather than `expect`: the lint fires on
    // some targets and not others.
    #[allow(clippy::unnecessary_cast)]
    let avail = stat.f_bavail as u64;
    #[allow(clippy::unnecessary_cast)]
    let frsize = stat.f_frsize as u64;
    Some(avail.saturating_mul(frsize))
}

#[cfg(not(unix))]
fn statvfs_avail_bytes(_path: &std::path::Path) -> Option<u64> {
    None
}

/// glibc version via `ldd --version` (Linux only; `None` elsewhere).
#[cfg(target_os = "linux")]
fn probe_glibc() -> Option<String> {
    let output = std::process::Command::new("ldd")
        .arg("--version")
        .output()
        .ok()?;
    if !output.status.success() {
        return None;
    }
    let text = String::from_utf8_lossy(&output.stdout);
    let first = text.lines().next()?;
    parse_ldd_version(first)
}

#[cfg(not(target_os = "linux"))]
fn probe_glibc() -> Option<String> {
    None
}

/// CUDA toolkit version from the well-known `version.txt` location.
fn probe_cuda() -> Option<String> {
    let content = std::fs::read_to_string("/usr/local/cuda/version.txt").ok()?;
    parse_cuda_version_txt(&content)
}

/// NVIDIA L4T (Tegra/Jetson) release from `/etc/nv_tegra_release`.
fn probe_l4t() -> Option<String> {
    let content = std::fs::read_to_string("/etc/nv_tegra_release").ok()?;
    parse_l4t_release(&content)
}

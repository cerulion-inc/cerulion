// SPDX-License-Identifier: AGPL-3.0-only
//! Compiler provenance for diagnostics and reproducible workspace scaffolding.
//! Stable node loading checks boundary major and layout, not this fingerprint.

use std::ffi::CString;
use std::os::raw::c_char;
use std::sync::OnceLock;

/// The rustc release and commit hash that compiled this copy of
/// `cerulion_core`, for example
/// `"1.97.1 (8bab26f4f68e0e26f0bb7960be334d5b520ea452)"`: `rustc -vV`'s
/// `commit-hash:` field verbatim, the full 40-character SHA-1, never truncated
/// to git's short form. Captured by `build.rs` from `rustc -vV` at compile
/// time, so it is always the compiler cargo actually invoked (it honors
/// `rustup` overrides), never merely the first `rustc` on `$PATH`.
///
/// A compiler without a commit hash reports its available provenance verbatim.
/// This value never determines whether a stable node can load.
pub static RUSTC_FINGERPRINT: &str = env!("CERULION_RUSTC_FINGERPRINT");

/// Just the `release` field of [`RUSTC_FINGERPRINT`] (for example `"1.97.1"`,
/// without the parenthesized commit hash): the exact string a
/// `rustup toolchain install <this>` or `RUSTUP_TOOLCHAIN=<this>` command
/// names.
pub static RUSTC_RELEASE: &str = env!("CERULION_RUSTC_RELEASE");

/// Lazily-built, process-lifetime `CString` backing
/// [`rustc_fingerprint_cstr`]'s returned pointer.
static RUSTC_FINGERPRINT_CSTRING: OnceLock<CString> = OnceLock::new();

/// A pointer to a static, null-terminated C string holding
/// [`RUSTC_FINGERPRINT`], available for compiler provenance diagnostics.
/// The pointer has process lifetime and is never freed by a caller. Stable
/// loading does not require this legacy export or compare its value.
///
/// # Panics
///
/// If `RUSTC_FINGERPRINT` somehow embedded a NUL byte, which `rustc -vV`'s
/// single-line `release:` and `commit-hash:` fields never do in practice
/// (`build.rs` takes each field off one line of that output, so an embedded
/// NUL would have to appear inside a compiler version string).
pub fn rustc_fingerprint_cstr() -> *const c_char {
    RUSTC_FINGERPRINT_CSTRING
        .get_or_init(|| {
            CString::new(RUSTC_FINGERPRINT)
                .expect("RUSTC_FINGERPRINT unexpectedly contained a NUL byte")
        })
        .as_ptr()
}

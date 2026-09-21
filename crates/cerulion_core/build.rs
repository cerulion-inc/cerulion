// SPDX-License-Identifier: AGPL-3.0-only
//! Captures the exact rustc (release plus commit hash) that compiles THIS copy
//! of `cerulion_core`, exposed as `cerulion_core::RUSTC_FINGERPRINT` and
//! `cerulion_core::RUSTC_RELEASE`. See `src/rustc_fingerprint.rs` for the
//! public surface and `src/abi_layout.rs` for the layout pin this guard
//! complements.
//!
//! # Why this exists
//!
//! `NodeContext`, and every `repr(Rust)` type reachable through it including
//! `CerulionSubscriber.frozen: Option<FrozenSlot>`, crosses the cdylib
//! `init()` FFI boundary as a raw `Box`, and the cdylib links its OWN copy of
//! `cerulion_core`. `abi_layout.rs` pins struct SIZES and OFFSETS across that
//! boundary, keyed to `CERULION_ABI_VERSION`, but rustc 1.97.0 changed the bit
//! pattern `Option<T>::None` writes for a niche-holding 4-variant `T` WITHOUT
//! moving any offset (`0x4` on rustc up to 1.94.1,
//! `0xFFFF_FFFF_FFFF_FFFF` on 1.97.0 and later), a divergence the size and
//! offset pin cannot see at all. A host built by one rustc release loading a
//! node cdylib built by another then reads a live `Some(FrozenSlot::Err(..))`
//! where the writer meant `None`; `drop_glue<Option<FrozenSlot>>` frees
//! uninitialised bytes and `libmalloc` calls `abort()`, with no panic text and
//! no backtrace, just SIGABRT.
//!
//! Comparing the exact rustc that compiled each side is the only guard for
//! this class: the ABI version alone proves layout agreement WITHIN one
//! rustc's encoding, never ACROSS rustc releases.

use std::env;
use std::process::Command;

fn main() {
    // Cargo always sets `RUSTC` for build scripts: the compiler cargo is
    // ACTUALLY invoking to build this crate (it honors an active `rustup`
    // toolchain override, `RUSTUP_TOOLCHAIN`, and `rust-toolchain.toml`), so
    // this reports the SAME rustc that compiles the rest of the crate, never a
    // different ambient one that happens to be first on `$PATH`.
    let rustc = env::var("RUSTC").unwrap_or_else(|_| "rustc".to_string());
    let output = Command::new(&rustc)
        .arg("-vV")
        .output()
        .unwrap_or_else(|e| panic!("cerulion_core/build.rs: failed to run `{rustc} -vV`: {e}"));
    if !output.status.success() {
        panic!(
            "cerulion_core/build.rs: `{rustc} -vV` exited with {}: {}",
            output.status,
            String::from_utf8_lossy(&output.stderr)
        );
    }
    let text = String::from_utf8_lossy(&output.stdout);

    // `-vV` is a flat `key: value` block, one field per line. Take the two
    // fields honestly, never the whole blob, so no other line's content (and
    // in particular no embedded newline, which would corrupt cargo's
    // line-oriented directive channel) ever reaches a `cargo:rustc-env=`
    // directive below.
    let mut release = None;
    let mut commit_hash = None;
    for line in text.lines() {
        if let Some(v) = line.strip_prefix("release: ") {
            release = Some(v.trim().to_string());
        } else if let Some(v) = line.strip_prefix("commit-hash: ") {
            commit_hash = Some(v.trim().to_string());
        }
    }
    let release = release.unwrap_or_else(|| {
        panic!("cerulion_core/build.rs: `{rustc} -vV` output had no `release:` line:\n{text}")
    });
    let commit_hash = commit_hash.unwrap_or_else(|| "unknown".to_string());

    println!("cargo:rustc-env=CERULION_RUSTC_FINGERPRINT={release} ({commit_hash})");
    println!("cargo:rustc-env=CERULION_RUSTC_RELEASE={release}");
    // Re-run whenever the compiler cargo selects changes (a `rustup` toolchain
    // switch, or a per-directory `rustup` override), not merely when a source
    // file changes, since none of them govern this fact.
    println!("cargo:rerun-if-env-changed=RUSTC");
    println!("cargo:rerun-if-changed=build.rs");
}

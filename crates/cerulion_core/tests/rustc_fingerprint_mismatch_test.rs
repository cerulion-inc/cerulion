// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end proof that `DylibNodeEntry::load` REJECTS a cdylib compiled by a
//! DIFFERENT rustc than the one that compiled this host.
//!
//! # Why this matters
//!
//! `NodeContext` crosses the cdylib `init()` FFI boundary as a raw `Box`, and
//! the cdylib links its OWN copy of `cerulion_core`, so every `repr(Rust)`
//! type reachable through it is laid out however THAT PARTICULAR rustc's
//! niche-filling pass decides. `CERULION_ABI_VERSION` proves struct SIZES and
//! OFFSETS agree, but rustc 1.97.0 changed the bit pattern `Option<T>::None`
//! writes for a niche-holding 4-variant `T` (`CerulionSubscriber.frozen:
//! Option<FrozenSlot>` is the one that breaks) WITHOUT moving any offset, a
//! class of divergence the ABI-version and `abi_layout` pins cannot see at
//! all, because they only measure size, align and offset. A host built by one
//! rustc release loading a node cdylib built by another then reads a live
//! `Some(..)` where the writer meant `None`, and the resulting `drop_glue`
//! frees uninitialised bytes, so `libmalloc` calls `abort()` and the process
//! dies on SIGABRT with no panic text, no backtrace, and no clue for the
//! operator. This test proves the load-time guard closes that hole BEFORE any
//! node ever ticks, exactly as `abi_mismatch_cdylib_is_rejected_at_load`
//! (`abi_version_mismatch_test.rs`) proves the ABI-number half.
//!
//! # How the fault is induced
//!
//! The hand-written `test_node_cdylib` fixture's `cerulion_rustc_fingerprint()`
//! export honors a `CER_RUSTC_FAULT` env var with two fault shapes: set to
//! anything other than `"null"`, it returns a deliberately-fake fingerprint
//! (`"9.9.9 (deadbeefdead)"`) instead of `cerulion_core::RUSTC_FINGERPRINT`
//! (exercises the MISMATCH arm below); set to exactly `"null"`, it returns a
//! null pointer instead (exercises the separate null-pointer arm). Because
//! `dlopen`/`libloading` resolves the env at CALL time (the export body reads
//! `std::env::var` each call), toggling the env before `DylibNodeEntry::load`
//! flips the reported fingerprint without rebuilding, mirroring
//! `CER_ABI_FAULT`'s pattern exactly.
//!
//! # Serial
//!
//! These tests mutate `CER_RUSTC_FAULT` (process-global env). Each is
//! `#[serial]` so they don't race on the env within this binary; cross-file
//! isolation is free (cargo runs each `tests/*.rs` in its own process). An
//! `EnvVarGuard` removes the var on drop so a mid-test panic can't leak the
//! fault to the control test.

use cerulion_core::graph::node::DylibNodeEntry;
use serial_test::serial;

/// The fixture's deliberately-fake fingerprint under `CER_RUSTC_FAULT=1`
/// (kept in one place so the assertion below and the fixture source can't
/// silently drift apart; see `test_node_cdylib/src/lib.rs`).
const FAKE_FINGERPRINT: &str = "9.9.9 (deadbeefdead)";

/// RAII guard that removes a process-global env var on drop: panic-safe
/// cleanup so the armed fault never leaks to the next test. Mirrors
/// `abi_version_mismatch_test.rs::EnvVarGuard`.
struct EnvVarGuard {
    name: &'static str,
}

impl EnvVarGuard {
    fn set(name: &'static str, value: &str) -> Self {
        std::env::set_var(name, value);
        Self { name }
    }
}

impl Drop for EnvVarGuard {
    fn drop(&mut self) {
        std::env::remove_var(self.name);
    }
}

/// Locate the test cdylib in the target directory. Mirrors
/// `abi_version_mismatch_test.rs::find_test_cdylib`.
fn find_test_cdylib() -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_cdylib")
}

// ============================================================
// A cdylib built by a DIFFERENT rustc is REJECTED at load.
// ============================================================

#[test]
#[serial]
fn rustc_fingerprint_mismatch_cdylib_is_rejected_at_load() {
    let dylib_path = find_test_cdylib();

    // Arm the fixture's `cerulion_rustc_fingerprint()` to report a
    // deliberately-fake fingerprint (the env is read inside the export at call
    // time). The guard removes it on drop, even if an assertion below panics,
    // so the control test stays clean.
    let _guard = EnvVarGuard::set("CER_RUSTC_FAULT", "1");

    let result = DylibNodeEntry::load(&dylib_path);

    let err = match result {
        Ok(_) => panic!(
            "DylibNodeEntry::load must REJECT a cdylib whose rustc fingerprint \
             ({FAKE_FINGERPRINT}) disagrees with the host's ({}); it returned Ok",
            cerulion_core::RUSTC_FINGERPRINT
        ),
        Err(e) => e,
    };

    // Pin the shape of the rejection: it must name BOTH compilers (so an
    // operator can tell what to install) and say "rustc" (so it reads as a
    // toolchain problem, not a generic load failure).
    let msg = err.to_string();
    assert!(
        msg.to_lowercase().contains("rustc"),
        "rejection error must name the mismatch as a rustc/toolchain problem; got: {msg}"
    );
    assert!(
        msg.contains(FAKE_FINGERPRINT),
        "rejection error must report the cdylib's (fake) fingerprint {FAKE_FINGERPRINT}; \
         got: {msg}"
    );
    assert!(
        msg.contains(cerulion_core::RUSTC_FINGERPRINT),
        "rejection error must report the host's own fingerprint ({}); got: {msg}",
        cerulion_core::RUSTC_FINGERPRINT
    );
    // Pin the part of the message the user can actually ACT on, not merely
    // that it names both compilers: the exact `RUSTUP_TOOLCHAIN=<release>`
    // override, with the host's own `RUSTC_RELEASE` (not the full fingerprint,
    // since a rustup toolchain name has no commit hash in it).
    let expected_override = format!("RUSTUP_TOOLCHAIN={}", cerulion_core::RUSTC_RELEASE);
    assert!(
        msg.contains(&expected_override),
        "rejection error must give the copy-pasteable override {expected_override:?} \
         (from cerulion_core::RUSTC_RELEASE), not just name the two compilers; got: {msg}"
    );
    let expected_install = format!("rustup toolchain install {}", cerulion_core::RUSTC_RELEASE);
    assert!(
        msg.contains(&expected_install),
        "rejection error must give the copy-pasteable {expected_install:?} step; got: {msg}"
    );
}

// ============================================================
// A cdylib whose fingerprint export returns NULL is REJECTED at load,
// distinctly from a mismatch (the null-pointer arm, not the compare arm).
// ============================================================

#[test]
#[serial]
fn rustc_fingerprint_null_cdylib_is_rejected_at_load() {
    let dylib_path = find_test_cdylib();

    // Arm the fixture's `cerulion_rustc_fingerprint()` to return a NULL
    // pointer instead of a (fake or real) C string: the defensive arm in
    // `DylibNodeEntry::load` that guards a symbol which RESOLVES but returns
    // nothing readable, distinct from `libloading`'s own missing-symbol
    // refusal (that arm needs no fixture support, since an old fixture simply
    // lacks the export).
    let _guard = EnvVarGuard::set("CER_RUSTC_FAULT", "null");

    let result = DylibNodeEntry::load(&dylib_path);

    let err = match result {
        Ok(_) => panic!(
            "DylibNodeEntry::load must REJECT a cdylib whose \
             cerulion_rustc_fingerprint() returns a null pointer; it returned Ok"
        ),
        Err(e) => e,
    };

    let msg = err.to_string();
    assert!(
        msg.to_lowercase().contains("null"),
        "rejection error must name the null pointer, not read through it as if it \
         were a valid C string; got: {msg}"
    );
}

// ============================================================
// Control: WITHOUT the fault, the SAME cdylib loads clean.
// ============================================================

#[test]
#[serial]
fn rustc_fingerprint_match_cdylib_loads_clean() {
    // Defensive: ensure no prior test leaked the env var (the guard should
    // have cleaned it, but a hard process abort elsewhere could not). This
    // makes the control independent of scheduling order.
    std::env::remove_var("CER_RUSTC_FAULT");

    let dylib_path = find_test_cdylib();

    // No fault armed, so the fixture reports `cerulion_core::RUSTC_FINGERPRINT`,
    // which equals the host's (both are compiled by the SAME rustc invoking
    // this test binary), and the load succeeds. This proves the fault is
    // env-GATED rather than always-on: the rejection in the sibling test is
    // caused by the fake fingerprint, not by something structurally broken in
    // the fixture.
    let result = DylibNodeEntry::load(&dylib_path);
    assert!(
        result.is_ok(),
        "without CER_RUSTC_FAULT the cdylib must load clean (proving the fault is \
         env-gated, not always-on); got: {:?}",
        result.err()
    );
}

// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end proof that
//! `DylibNodeEntry::load` REJECTS a cdylib whose `cerulion_abi_version()`
//! export disagrees with the host's `CERULION_ABI_VERSION`.
//!
//! # Why this matters
//!
//! ABI rejection is cdylib-only BY NATURE — in-process nodes are compiled
//! into the same binary against the same `CERULION_ABI_VERSION`, so there is
//! no version to check. Only a dynamically-loaded cdylib can be stale
//! relative to the host. This validates ABI-bump safety: when the
//! host bumps `CERULION_ABI_VERSION` (e.g. the v5→v6→v7 QoS-FFI bumps, the
//! v8 depth-FFI bump, and the v9 input-trigger bump), a
//! cdylib built against the old ABI is rejected at load time with a loud,
//! actionable error rather than being called with mismatched FFI signatures
//! (undefined behavior).
//!
//! # How the fault is induced
//!
//! The hand-written `test_node_cdylib` fixture's `cerulion_abi_version()`
//! export honors a `CER_ABI_FAULT` env var: when set, it returns a
//! deliberately-wrong `9999` instead of `cerulion_core::CERULION_ABI_VERSION`.
//! Because `dlopen`/`libloading` resolves the env at CALL time (the export
//! body reads `std::env::var` each call), toggling the env before
//! `DylibNodeEntry::load` flips the reported version without rebuilding.
//!
//! # Serial
//!
//! These tests mutate `CER_ABI_FAULT` (process-global env). Each is
//! `#[serial]` (from the `serial_test` dev-dep) so they don't race on the env
//! within this binary; cross-file isolation is free (cargo runs each
//! `tests/*.rs` in its own process). An `EnvVarGuard` removes the var on drop
//! so a mid-test panic can't leak the fault to the control test.

use cerulion_core::graph::node::DylibNodeEntry;
use serial_test::serial;

/// RAII guard that removes a process-global env var on drop — panic-safe
/// cleanup so the armed fault never leaks to the next test. Mirrors the
/// pattern in `chunk_c_ffi_codes_3_4_test.rs`.
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
/// `node_test.rs::find_test_cdylib` (platform-specific lib naming; tests
/// always run in debug mode).
fn find_test_cdylib() -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_cdylib")
}

// ============================================================
// A stale cdylib (wrong ABI) is REJECTED at load.
// ============================================================

#[test]
#[serial]
fn abi_mismatch_cdylib_is_rejected_at_load() {
    let dylib_path = find_test_cdylib();

    // Arm the fixture's `cerulion_abi_version()` to report `9999` (the env
    // is read inside the export at call time). The guard removes it on drop —
    // even if an assertion below panics — so the control test stays clean.
    let _guard = EnvVarGuard::set("CER_ABI_FAULT", "1");

    let result = DylibNodeEntry::load(&dylib_path);

    let err = match result {
        Ok(_) => panic!(
            "DylibNodeEntry::load must REJECT a cdylib whose ABI version (9999) \
             disagrees with the host CERULION_ABI_VERSION; it returned Ok"
        ),
        Err(e) => e,
    };

    // Pin the actual error string shape from `DylibNodeEntry::load`:
    // "ABI version mismatch: library exports version {version}, expected {expected}".
    // Asserting BOTH the category substring AND the forced wrong number `9999`
    // is non-vacuous: a generic load failure (missing symbol, bad path) would
    // not mention "ABI version" + "9999", so this rejects only the genuine
    // version-mismatch path.
    let msg = err.to_string();
    assert!(
        msg.contains("ABI version"),
        "rejection error must name the ABI version mismatch; got: {msg}"
    );
    assert!(
        msg.contains("9999"),
        "rejection error must report the cdylib's (wrong) reported version 9999; \
         got: {msg}"
    );
}

// ============================================================
// Control: WITHOUT the fault, the SAME cdylib loads clean.
// ============================================================

#[test]
#[serial]
fn abi_match_cdylib_loads_clean() {
    // Defensive: ensure no prior test leaked the env var (the guard should have
    // cleaned it, but a hard process abort elsewhere could not). This makes the
    // control independent of scheduling order.
    std::env::remove_var("CER_ABI_FAULT");

    let dylib_path = find_test_cdylib();

    // No fault armed → the fixture reports `cerulion_core::CERULION_ABI_VERSION`,
    // which equals the host's, so the load succeeds. This proves the fault is
    // env-GATED (not always-on): the rejection in the sibling test is caused by
    // the wrong version, not by something structurally broken in the fixture.
    let result = DylibNodeEntry::load(&dylib_path);
    assert!(
        result.is_ok(),
        "without CER_ABI_FAULT the cdylib must load clean (proving the fault is \
         env-gated, not always-on); got: {:?}",
        result.err()
    );
}

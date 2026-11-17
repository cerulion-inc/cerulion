// SPDX-License-Identifier: AGPL-3.0-only
//! A different stable boundary major is refused before a node is created.

use cerulion_core::graph::node::DylibNodeEntry;
use serial_test::serial;

/// RAII guard that removes a process-global env var on drop, panic-safe
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

    // Arm the fixture's `stable table header` to report `9999` (the env
    // is read inside the export at call time). The guard removes it on drop —
    // even if an assertion below panics, so the control test stays clean.
    let _guard = EnvVarGuard::set("CER_ABI_FAULT", "1");

    let result = DylibNodeEntry::load(&dylib_path);

    let err = match result {
        Ok(_) => panic!(
            "DylibNodeEntry::load must REJECT a cdylib whose ABI version (9999) \
             disagrees with the host boundary major; it returned Ok"
        ),
        Err(e) => e,
    };

    // Pin the actual error string shape from `DylibNodeEntry::load`:
    // "boundary major mismatch: library exports version {version}, expected {expected}".
    // Asserting BOTH the category substring AND the forced wrong number `9999`
    // is non-vacuous: a generic load failure (missing symbol, bad path) would
    // not mention "major" + "9999", so this rejects only the genuine
    // version-mismatch path.
    let msg = err.to_string();
    assert!(
        msg.contains("major"),
        "rejection error must name the boundary major mismatch; got: {msg}"
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

    // No fault armed  to  the fixture reports `cerulion_core::boundary major`,
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

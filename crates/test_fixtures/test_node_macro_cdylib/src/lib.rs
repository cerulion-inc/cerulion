// SPDX-License-Identifier: AGPL-3.0-only
//! Test cdylib node using `#[cerulion_node]` + `#[cerulion_node_impl]`
//! (the minimal macro-pair fixture).
//!
//! Verifies that the macro pair correctly generates cdylib FFI entry points
//! compatible with `DylibNodeEntry::load()`. Field types are the unit-struct
//! markers (`Vector3`); the impl macro rewrites every `self.<port>` access in
//! `tick` to dispatch through a per-tick `OutputProxy<'_, T>` /
//! `InputView<'_, T>` so direct field assignment lands straight in iceoryx2
//! SHM with no memcpy on the local hot path.
//!
//! # Marker import
//!
//! `#[cerulion_node]`'s declarative-mode emission uses the field type as a
//! type argument (`loan_proxy::<Vector3>()`, `try_view::<Vector3, _>(...)`,
//! `OutputProxy<'_, Vector3>`, `InputView<'_, Vector3>`). Type-argument
//! position counts as a use for `unused_imports`, so the marker import is
//! never flagged. Verified by the crate-level `#![deny(unused_imports)]`.

// Deny unused-imports at the crate level to prove the marker imports below
// (`Vector3`) are actually consumed by the macro emission (see the marker-
// import section in the module doc).
#![deny(unused_imports)]
// Principle #12 (logging): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;
use std::sync::atomic::{AtomicU64, Ordering};

static TICK_COUNT: AtomicU64 = AtomicU64::new(0);

#[cerulion_node]
struct MacroCounter {
    // Marker fields: the impl macro rewrites `self.velocity_in.<f>` to the
    // per-tick `__cer_velocity_in: &InputView<'_, Vector3>` and
    // `self.cmd_out.<f>` to `__cer_cmd_out: &mut OutputProxy<'_, Vector3>`.
    // The unit-struct fields themselves are never read directly by user code.
    #[input(trigger, depth = 1)]
    velocity_in: Vector3,

    #[output]
    cmd_out: Vector3,

    panic_on_shutdown: bool,
}

#[cerulion_node_impl]
impl MacroCounter {
    fn init(&mut self, ctx: &mut NodeContext) -> Result<(), NodeError> {
        let phase = ctx.env_opt("CER_TEST_STABLE_PANIC_PHASE");
        assert_ne!(phase.as_deref(), Some("init"), "fixture init panic");
        self.panic_on_shutdown = phase.as_deref() == Some("shutdown");
        Ok(())
    }

    fn shutdown(&mut self) -> Result<(), NodeError> {
        assert!(!self.panic_on_shutdown, "fixture shutdown panic");
        Ok(())
    }

    fn tick(&mut self) -> Result<(), NodeError> {
        // Direct SHM-to-SHM copy of the three f64 fields. The impl macro
        // wraps the body in a nested `try_view` for `velocity_in` plus a
        // `loan_proxy::<Vector3>()` for `cmd_out`, so each line below
        // compiles to a single `mov` from one shared-memory region into
        // another with no intermediate snapshot or memcpy.
        TICK_COUNT.fetch_add(1, Ordering::Relaxed);
        self.cmd_out.x = self.velocity_in.x;
        self.cmd_out.y = self.velocity_in.y;
        self.cmd_out.z = self.velocity_in.z;
        Ok(())
    }
}

/// Helper for tests to read tick count via FFI.
#[no_mangle]
pub extern "C" fn cerulion_test_get_tick_count() -> u64 {
    TICK_COUNT.load(Ordering::Relaxed)
}

#[cfg(test)]
mod context_compile_tests {
    use std::collections::BTreeSet;
    use std::path::{Path, PathBuf};

    /// The context-helper programs that must compile under the plugin
    /// expansion, by file stem. Each is an example target of this crate, so
    /// `cargo test -p test_node_macro_cdylib` builds it with the default
    /// `cdylib` feature and no `cfg(test)`; the test below runs it.
    const PROGRAMS: [&str; 3] = [
        "annotated_context_helper",
        "data_only_helpers",
        "qualified_context_helper",
    ];

    /// The one helper shape the migration refuses.
    const REFUSED: &str = "free_context_helper";

    fn manifest_dir() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
    }

    #[test]
    fn a_free_helper_taking_the_native_context_is_refused() {
        let cases = trybuild::TestCases::new();
        // The stderr oracle pins rustc 1.99.1 context names; compiler upgrades
        // may require re-blessing while preserving the migration error.
        cases.compile_fail("tests/ui/free_context_helper.rs");
    }

    #[test]
    fn the_listed_programs_are_the_ui_directory_and_the_manifest_examples() {
        let ui = manifest_dir().join("tests/ui");
        let on_disk: BTreeSet<String> = std::fs::read_dir(&ui)
            .unwrap_or_else(|e| panic!("read {}: {e}", ui.display()))
            .map(|entry| entry.expect("a directory entry").path())
            .filter(|path| path.extension().is_some_and(|ext| ext == "rs"))
            .map(|path| {
                path.file_stem()
                    .and_then(|stem| stem.to_str())
                    .expect("a UTF-8 file stem")
                    .to_owned()
            })
            .filter(|stem| stem != REFUSED)
            .collect();
        let listed: BTreeSet<String> = PROGRAMS.iter().map(|name| name.to_string()).collect();
        assert_eq!(
            on_disk, listed,
            "tests/ui/ and PROGRAMS differ: list a new program here, or drop a removed one"
        );
        let manifest = std::fs::read_to_string(manifest_dir().join("Cargo.toml"))
            .expect("read this crate's Cargo.toml");
        let mut declared = BTreeSet::new();
        let mut name: Option<String> = None;
        for line in manifest.lines().map(str::trim) {
            if line.starts_with('[') {
                name = None;
            } else if let Some(value) = line.strip_prefix("name = \"") {
                name = value.strip_suffix('"').map(str::to_owned);
            } else if let Some(stem) = line
                .strip_prefix("path = \"tests/ui/")
                .and_then(|rest| rest.strip_suffix(".rs\""))
            {
                assert_eq!(
                    name.as_deref(),
                    Some(format!("pass_{stem}").as_str()),
                    "the [[example]] for tests/ui/{stem}.rs must be named pass_{stem}"
                );
                declared.insert(stem.to_owned());
            }
        }
        assert_eq!(
            declared, listed,
            "Cargo.toml's [[example]] entries and PROGRAMS differ: a program without an entry \
             is never built, and a binary left from an older build would run in its place"
        );
    }

    #[test]
    fn stable_context_helpers_compile_with_the_author_migration() {
        // `target/<profile>/deps/<this binary>` to `target/<profile>/examples`.
        let exe = std::env::current_exe().expect("this test binary's own path");
        let deps = exe
            .parent()
            .expect("the directory holding this test binary");
        assert_eq!(
            deps.file_name().and_then(|name| name.to_str()),
            Some("deps"),
            "{} is not in a cargo `deps` directory",
            exe.display()
        );
        let examples = deps
            .parent()
            .expect("the profile directory above `deps`")
            .join("examples");
        let modified = |file: &Path| {
            std::fs::metadata(file)
                .and_then(|meta| meta.modified())
                .unwrap_or_else(|e| panic!("read the modification time of {}: {e}", file.display()))
        };
        let mut failures = Vec::new();
        for name in PROGRAMS {
            let path = examples.join(format!("pass_{name}{}", std::env::consts::EXE_SUFFIX));
            assert!(
                path.is_file(),
                "{} was not built: run `cargo test -p test_node_macro_cdylib`, which builds \
                 every example",
                path.display()
            );
            let source = manifest_dir().join(format!("tests/ui/{name}.rs"));
            assert!(
                modified(&path) >= modified(&source),
                "{} is older than {}: run `cargo test -p test_node_macro_cdylib`, which rebuilds it",
                path.display(),
                source.display()
            );
            let output = std::process::Command::new(&path)
                .output()
                .unwrap_or_else(|e| panic!("start {}: {e}", path.display()));
            if !output.status.success() {
                failures.push(format!(
                    "{name} exited with {}\nstderr:\n{}",
                    output.status,
                    String::from_utf8_lossy(&output.stderr)
                ));
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n\n"));
    }
}

// SPDX-License-Identifier: AGPL-3.0-only
//! The directory a test scaffolds a workspace into, removed when its guard
//! drops, and the cargo target directory such a scaffold builds into.
//!
//! [`ScaffoldDir`] owns a `tempfile::TempDir` and is held by the test that
//! created it, never by a `static` (a `static` is never dropped, so its
//! directory outlives the process). Dropping the guard removes the directory
//! on every way out of the test that unwinds: a pass, a panic, an early return.
//!
//! [`KEEP_ENV`]` = 1` keeps the directory of a FAILED test, one whose thread is
//! panicking when the guard drops, and prints its path to stderr. A test that
//! passes removes its directory whatever the variable says.
//!
//! [`scaffold_target_dir`] names the target directory, outside the scaffold,
//! that every scaffold of one test binary builds into (unless a relative
//! `CARGO_TARGET_DIR` puts it inside), so a scaffold's directory holds sources
//! only and the dependencies' artifacts are reused by
//! the next scaffold instead of being compiled into a fresh target each time.

// Each test binary that includes this module uses a subset of it.
#![allow(dead_code)]

use std::ffi::OsStr;
use std::path::{Path, PathBuf};

/// Set to `1` to keep a failed test's scaffold directory.
pub const KEEP_ENV: &str = "CERULION_KEEP_TEST_WS";

/// Whether a [`KEEP_ENV`] value asks to keep a failed test's directory:
/// exactly `1`. Unset, empty, `0` and every other value remove it.
pub fn keep_requested(value: Option<&OsStr>) -> bool {
    value == Some(OsStr::new("1"))
}

/// A test's scaffold directory, removed when this guard drops.
pub struct ScaffoldDir {
    /// `None` only inside `drop`, after the directory was taken.
    dir: Option<tempfile::TempDir>,
    keep_on_failure: bool,
}

impl ScaffoldDir {
    /// A new empty directory under the system temp dir, named after `label`;
    /// whether a failed test keeps it is read from [`KEEP_ENV`].
    pub fn new(label: &str) -> Self {
        let keep = keep_requested(std::env::var_os(KEEP_ENV).as_deref());
        Self::with_keep_on_failure(label, keep)
    }

    /// [`ScaffoldDir::new`] with the keep decision passed in rather than read
    /// from the environment.
    pub fn with_keep_on_failure(label: &str, keep_on_failure: bool) -> Self {
        let dir = tempfile::Builder::new()
            .prefix(&format!(".tmp-{label}-"))
            .tempdir()
            .unwrap_or_else(|e| panic!("create the `{label}` scaffold directory: {e}"));
        Self {
            dir: Some(dir),
            keep_on_failure,
        }
    }

    pub fn path(&self) -> &Path {
        self.dir
            .as_ref()
            .expect("the directory is present until drop")
            .path()
    }
}

impl Drop for ScaffoldDir {
    fn drop(&mut self) {
        let Some(dir) = self.dir.take() else {
            return;
        };
        let failing = std::thread::panicking();
        if failing && self.keep_on_failure {
            let kept = dir.keep();
            eprintln!(
                "{KEEP_ENV}=1: kept the failed test's scaffold at {}",
                kept.display()
            );
            return;
        }
        let path = dir.path().to_path_buf();
        if let Err(e) = dir.close() {
            let message = format!("remove the scaffold directory {}: {e}", path.display());
            if failing {
                // A second panic during unwinding aborts the process and hides
                // the test's own failure, so report this one and keep unwinding.
                eprintln!("{message}");
            } else {
                panic!("{message}");
            }
        }
    }
}

/// The target directory a scaffold at `ws_root` builds into.
///
/// An ambient `CARGO_TARGET_DIR` (or, when that is unset, `CARGO_BUILD_TARGET_DIR`,
/// which also overrides a config `target-dir`) is honoured: an absolute value
/// as is, a relative one against `ws_root`, the way cargo run from `ws_root`
/// and `graph_cmd::cdylib_target_base` resolve it. A relative value therefore
/// lands inside the scaffold directory and is removed with it. Unset, the
/// directory is `<CARGO_TARGET_TMPDIR>/<binary>-scaffold-target`: inside this
/// build's own target directory, one per test binary, and reused across runs.
/// Every scaffold of one binary writes the same node sources, so an artifact
/// cargo reuses from an earlier scaffold is the one this scaffold would have
/// built.
///
/// Two scaffolds of one binary must not run at the same time: one's build
/// replaces `debug/lib<node>.so` while the other's workers may be loading it.
/// The suites run with `--test-threads=1` under `cargo test`, one test at a time.
pub fn scaffold_target_dir(ws_root: &Path, binary: &str) -> PathBuf {
    for name in ["CARGO_TARGET_DIR", "CARGO_BUILD_TARGET_DIR"] {
        match std::env::var_os(name) {
            Some(v) if !v.is_empty() => return ws_root.join(v),
            _ => {}
        }
    }
    Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("{binary}-scaffold-target"))
}

/// Point the scaffold at `ws_root` at `target_dir` by appending `[build]
/// target-dir` to the `.cargo/config.toml` the scaffold wrote, so a `cargo
/// build` run inside it writes there. Processes that locate the built cdylibs
/// read `CARGO_TARGET_DIR`, so the caller passes `target_dir` to them in that
/// variable.
pub fn pin_target_dir(ws_root: &Path, target_dir: &Path) {
    let config = ws_root.join(".cargo").join("config.toml");
    let mut text = std::fs::read_to_string(&config)
        .unwrap_or_else(|e| panic!("read {}: {e}", config.display()));
    let path = target_dir
        .to_str()
        .unwrap_or_else(|| panic!("target dir {} is not UTF-8", target_dir.display()));
    let value = toml::Value::String(path.to_string());
    text.push_str(&format!("\n[build]\ntarget-dir = {value}\n"));
    std::fs::write(&config, text).unwrap_or_else(|e| panic!("write {}: {e}", config.display()));
}

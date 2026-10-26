// SPDX-License-Identifier: AGPL-3.0-only
//! The scaffold directory guard the Linux-only multi-process suites hold their
//! workspaces in: the directory is gone after a pass, an early return and a
//! panic, and a panic keeps it only when keeping was asked for.

mod scaffold_dir;

use std::ffi::OsStr;
use std::panic::{self, AssertUnwindSafe};
use std::path::{Path, PathBuf};

use scaffold_dir::{keep_requested, pin_target_dir, ScaffoldDir};

/// Create a guard with a file inside it, run `body` with the directory's path
/// while the guard is alive, and return the path and whether `body` panicked.
fn run_with_guard(keep_on_failure: bool, body: impl FnOnce(&Path)) -> (PathBuf, bool) {
    let mut path = PathBuf::new();
    let outcome = panic::catch_unwind(AssertUnwindSafe(|| {
        let dir = ScaffoldDir::with_keep_on_failure("guard_test", keep_on_failure);
        path = dir.path().to_path_buf();
        std::fs::create_dir_all(path.join("target/debug")).unwrap();
        std::fs::write(path.join("target/debug/artifact"), b"bytes").unwrap();
        body(&path);
    }));
    assert!(!path.as_os_str().is_empty(), "the guard was never created");
    (path, outcome.is_err())
}

#[test]
fn the_directory_is_removed_when_the_test_passes() {
    let (path, panicked) = run_with_guard(false, |p| assert!(p.is_dir()));
    assert!(!panicked);
    assert!(!path.exists(), "{} survived a pass", path.display());
}

#[test]
fn the_directory_is_removed_on_an_early_return() {
    fn returns_early(keep: bool) -> PathBuf {
        let dir = ScaffoldDir::with_keep_on_failure("guard_test", keep);
        let path = dir.path().to_path_buf();
        if path.is_dir() {
            return path;
        }
        unreachable!("a fresh guard's directory exists");
    }
    let path = returns_early(true);
    assert!(
        !path.exists(),
        "{} survived an early return",
        path.display()
    );
}

#[test]
fn the_directory_is_removed_after_a_panic() {
    let (path, panicked) = run_with_guard(false, |_| panic!("the arm failed"));
    assert!(panicked);
    assert!(!path.exists(), "{} survived a panic", path.display());
}

#[test]
fn a_panic_keeps_the_directory_when_keeping_is_asked_for() {
    let (path, panicked) = run_with_guard(true, |_| panic!("the arm failed"));
    assert!(panicked);
    let kept = path.join("target/debug/artifact").is_file();
    std::fs::remove_dir_all(&path).ok();
    assert!(
        kept,
        "{} was removed although keeping was asked for",
        path.display()
    );
}

#[test]
fn a_pass_removes_the_directory_even_when_keeping_is_asked_for() {
    let (path, panicked) = run_with_guard(true, |_| {});
    assert!(!panicked);
    assert!(!path.exists(), "{} survived a pass", path.display());
}

#[test]
fn only_the_value_one_asks_to_keep() {
    assert!(keep_requested(Some(OsStr::new("1"))));
    for value in ["", "0", "true", "yes", " 1", "11"] {
        assert!(!keep_requested(Some(OsStr::new(value))), "{value:?}");
    }
    assert!(!keep_requested(None));
}

#[test]
fn pinning_the_target_dir_appends_a_build_table_cargo_parses() {
    let dir = ScaffoldDir::with_keep_on_failure("guard_test", false);
    let ws = dir.path();
    std::fs::create_dir_all(ws.join(".cargo")).unwrap();
    std::fs::write(ws.join(".cargo/config.toml"), "[env]\nA = \"b\"\n").unwrap();
    let target = ws.join("a \"quoted\" \\ dir");
    pin_target_dir(ws, &target);
    let text = std::fs::read_to_string(ws.join(".cargo/config.toml")).unwrap();
    let table: toml::Table = text.parse().expect("the config stays valid TOML");
    assert_eq!(table["env"]["A"].as_str(), Some("b"));
    assert_eq!(
        table["build"]["target-dir"].as_str(),
        Some(target.to_str().unwrap())
    );
}

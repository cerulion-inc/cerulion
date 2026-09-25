// SPDX-License-Identifier: AGPL-3.0-only
//! How this copy of `cerulion` was installed, reported as the
//! `install_method` of `cli_command_run`. Each installer writes a small JSON marker next to the files it
//! installs; a copy built from source has none.
//!
//! * `install.sh` writes `.cerulion-provenance.json` beside the binaries.
//! * The Debian package and the Homebrew formula ship
//!   `share/cerulion/install.json` one level above their `bin` directory.
//!
//! Only a method from [`KNOWN_METHODS`] is ever returned, so the marker's
//! contents never reach an event.

use std::path::{Path, PathBuf};

/// The methods a marker may name.
pub const KNOWN_METHODS: &[&str] = &["install.sh", "deb", "brew"];

/// The method a marker's JSON object names in its `method` string, if it
/// is exactly one of [`KNOWN_METHODS`].
pub fn parse_method(json: &str) -> Option<&'static str> {
    let marker: serde_json::Value = serde_json::from_str(json).ok()?;
    let method = marker.as_object()?.get("method")?.as_str()?;
    KNOWN_METHODS.iter().copied().find(|known| *known == method)
}

/// Where the markers for a binary in `bin_dir` live, in lookup order.
pub fn marker_paths(bin_dir: &Path) -> Vec<PathBuf> {
    let mut paths = vec![bin_dir.join(".cerulion-provenance.json")];
    if let Some(prefix) = bin_dir.parent() {
        paths.push(prefix.join("share").join("cerulion").join("install.json"));
    }
    paths
}

/// The first known method among `paths`. Only regular files are read, so a
/// symlink or directory in a marker's place is ignored.
pub fn method_at(paths: &[PathBuf]) -> Option<&'static str> {
    paths.iter().find_map(|path| {
        std::fs::symlink_metadata(path)
            .ok()?
            .is_file()
            .then_some(())?;
        parse_method(&std::fs::read_to_string(path).ok()?)
    })
}

/// How the running binary was installed, or `None` for a source build.
pub fn current_method() -> Option<&'static str> {
    let exe = std::env::current_exe().ok()?.canonicalize().ok()?;
    method_at(&marker_paths(exe.parent()?))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_known_methods_parse() {
        assert_eq!(
            parse_method(r#"{"method":"install.sh","version":"1.0.0"}"#),
            Some("install.sh")
        );
        assert_eq!(parse_method(r#"{"method":"deb"}"#), Some("deb"));
        assert_eq!(parse_method(r#"{"method":"brew"}"#), Some("brew"));
        assert_eq!(parse_method(r#"{"method":"../elsewhere"}"#), None);
        assert_eq!(parse_method(r#"{"version":"1.0.0"}"#), None);
        assert_eq!(parse_method("not json"), None);
        assert_eq!(parse_method(r#"{"method":"deb2"}"#), None);
        assert_eq!(parse_method(r#"{"method":"d eb"}"#), None);
        assert_eq!(parse_method(r#"{ "method" : "deb" }"#), Some("deb"));
        assert_eq!(parse_method(r#"{"method":["deb"]}"#), None);
        assert_eq!(parse_method(r#"["deb"]"#), None);
    }

    #[test]
    fn the_marker_beside_the_binary_wins() {
        let dir = tempfile::tempdir().unwrap();
        let bin = dir.path().join("bin");
        let share = dir.path().join("share").join("cerulion");
        std::fs::create_dir_all(&bin).unwrap();
        std::fs::create_dir_all(&share).unwrap();
        let paths = marker_paths(&bin);
        assert_eq!(method_at(&paths), None, "no marker is a source build");
        std::fs::write(share.join("install.json"), r#"{"method":"deb"}"#).unwrap();
        assert_eq!(method_at(&paths), Some("deb"));
        std::fs::write(
            bin.join(".cerulion-provenance.json"),
            r#"{"method":"install.sh"}"#,
        )
        .unwrap();
        assert_eq!(method_at(&paths), Some("install.sh"));
    }

    #[test]
    fn a_directory_or_unknown_marker_is_skipped() {
        let dir = tempfile::tempdir().unwrap();
        let bin = dir.path().join("bin");
        std::fs::create_dir_all(bin.join(".cerulion-provenance.json")).unwrap();
        let share = dir.path().join("share").join("cerulion");
        std::fs::create_dir_all(&share).unwrap();
        std::fs::write(share.join("install.json"), r#"{"method":"other"}"#).unwrap();
        assert_eq!(method_at(&marker_paths(&bin)), None);
    }

    #[cfg(unix)]
    #[test]
    fn a_symlinked_marker_is_skipped() {
        let dir = tempfile::tempdir().unwrap();
        let target = dir.path().join("target.json");
        std::fs::write(&target, r#"{"method":"brew"}"#).unwrap();
        let bin = dir.path().join("bin");
        std::fs::create_dir_all(&bin).unwrap();
        std::os::unix::fs::symlink(&target, bin.join(".cerulion-provenance.json")).unwrap();
        assert_eq!(method_at(&marker_paths(&bin)), None);
    }
}

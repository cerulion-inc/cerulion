// SPDX-License-Identifier: AGPL-3.0-only
//! How this copy of `cerulion` was installed, reported as the
//! `install_method` of `cli_command_run`. Each installer writes a small JSON marker next to the files it
//! installs; a copy built from source has none.
//!
//! * `install.sh` writes `.cerulion-provenance.json` beside the binaries.
//! * The Debian package and the Homebrew formula ship
//!   `share/cerulion/install.json` one level above their `bin` directory.
//!   When both markers exist, the one written last wins: each installer
//!   rewrites its own marker whenever it installs, so the newer marker
//!   names what last replaced the binary. Markers with the same change
//!   time resolve to the package's.
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

/// Where the markers for a binary in `bin_dir` live, the package manager's
/// first, which [`method_at`] prefers when both changed at the same time.
pub fn marker_paths(bin_dir: &Path) -> Vec<PathBuf> {
    let mut paths = Vec::with_capacity(2);
    if let Some(prefix) = bin_dir.parent() {
        paths.push(prefix.join("share").join("cerulion").join("install.json"));
    }
    paths.push(bin_dir.join(".cerulion-provenance.json"));
    paths
}

/// The known method of the marker among `paths` whose inode changed last,
/// the earliest in `paths` on a tie. The change time is when the marker was
/// put in place on this machine, which a package manager cannot backdate
/// the way it restores a file's modification time. Only regular files are
/// read, so a symlink or directory in a marker's place is ignored.
pub fn method_at(paths: &[PathBuf]) -> Option<&'static str> {
    let mut newest: Option<(&'static str, (i64, i64))> = None;
    for path in paths {
        let Some((method, meta)) = read_marker(path) else {
            continue;
        };
        let changed = changed_at(&meta);
        if newest.is_none_or(|(_, at)| changed > at) {
            newest = Some((method, changed));
        }
    }
    newest.map(|(method, _)| method)
}

/// The method a regular file at `path` names, with the metadata of the
/// same open file, so a marker replaced while it is read cannot pair one
/// marker's method with another's change time.
fn read_marker(path: &Path) -> Option<(&'static str, std::fs::Metadata)> {
    if !std::fs::symlink_metadata(path).ok()?.is_file() {
        return None;
    }
    let mut file = std::fs::File::open(path).ok()?;
    let meta = file.metadata().ok()?;
    if !meta.is_file() {
        return None;
    }
    let mut json = String::new();
    std::io::Read::read_to_string(&mut file, &mut json).ok()?;
    Some((parse_method(&json)?, meta))
}

/// A marker's inode change time as seconds and nanoseconds.
#[cfg(unix)]
fn changed_at(meta: &std::fs::Metadata) -> (i64, i64) {
    use std::os::unix::fs::MetadataExt;
    (meta.ctime(), meta.ctime_nsec())
}

/// No package manager writes a marker off Unix, so order alone decides.
#[cfg(not(unix))]
fn changed_at(_: &std::fs::Metadata) -> (i64, i64) {
    (0, 0)
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
    fn the_package_marker_wins_over_a_stale_script_marker() {
        let dir = tempfile::tempdir().unwrap();
        let bin = dir.path().join("bin");
        let share = dir.path().join("share").join("cerulion");
        std::fs::create_dir_all(&bin).unwrap();
        std::fs::create_dir_all(&share).unwrap();
        let paths = marker_paths(&bin);
        assert_eq!(method_at(&paths), None, "no marker is a source build");
        std::fs::write(
            bin.join(".cerulion-provenance.json"),
            r#"{"method":"install.sh"}"#,
        )
        .unwrap();
        assert_eq!(method_at(&paths), Some("install.sh"));
        wait_for_a_later_change_time(&bin.join(".cerulion-provenance.json"));
        std::fs::write(share.join("install.json"), r#"{"method":"deb"}"#).unwrap();
        assert_eq!(method_at(&paths), Some("deb"));
    }

    #[cfg(unix)]
    #[test]
    fn a_script_install_over_a_package_wins_over_the_package_marker() {
        let dir = tempfile::tempdir().unwrap();
        let bin = dir.path().join("bin");
        let share = dir.path().join("share").join("cerulion");
        std::fs::create_dir_all(&bin).unwrap();
        std::fs::create_dir_all(&share).unwrap();
        let package = share.join("install.json");
        std::fs::write(&package, r#"{"method":"deb"}"#).unwrap();
        let paths = marker_paths(&bin);
        assert_eq!(method_at(&paths), Some("deb"));
        wait_for_a_later_change_time(&package);
        std::fs::write(
            bin.join(".cerulion-provenance.json"),
            r#"{"method":"install.sh"}"#,
        )
        .unwrap();
        assert_eq!(method_at(&paths), Some("install.sh"));
        wait_for_a_later_change_time(&bin.join(".cerulion-provenance.json"));
        std::fs::write(&package, r#"{"method":"deb"}"#).unwrap();
        assert_eq!(
            method_at(&paths),
            Some("deb"),
            "a package upgrade wins back"
        );
    }

    /// Wait until a file written now gets a later change time than `path`,
    /// so a test does not depend on the filesystem's timestamp resolution.
    fn wait_for_a_later_change_time(path: &Path) {
        let before = changed_at(&std::fs::symlink_metadata(path).unwrap());
        let probe = path.with_extension("probe");
        loop {
            std::fs::write(&probe, b"").unwrap();
            let now = changed_at(&std::fs::symlink_metadata(&probe).unwrap());
            std::fs::remove_file(&probe).unwrap();
            if now > before || cfg!(not(unix)) {
                return;
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
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

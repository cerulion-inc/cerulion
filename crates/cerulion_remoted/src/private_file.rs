// SPDX-License-Identifier: AGPL-3.0-only
//! Reading the robot's private files: the device key, the trust-store MAC key
//! and the device-account index.
//!
//! [`read_private`] reads the whole file. On Unix it first clears the group and
//! other permission bits of a file at that path that others can access, through
//! the open handle, without following a symbolic link (see [`narrow_to_owner`]).

use std::path::Path;

/// Read the file at `path`, one of the robot's private files named by `what` in the
/// log. On Unix, when `path` is itself a regular file with a single link that group
/// or other users can access, its group and other permission bits are cleared
/// through the open handle before it is read, logged at `warn`. When clearing them
/// fails, that is logged at `warn` and the read goes on. A symbolic link at `path`
/// is read through and its target is not changed.
///
/// Errors are those of opening and reading the file, unchanged.
pub(crate) fn read_private(path: &Path, what: &'static str) -> std::io::Result<Vec<u8>> {
    use std::io::Read as _;
    #[cfg(unix)]
    let at_path = std::fs::symlink_metadata(path).ok();
    let mut file = std::fs::File::open(path)?;
    #[cfg(unix)]
    narrow_to_owner(&file, path, at_path.as_ref(), what);
    #[cfg(not(unix))]
    let _ = what;
    let mut bytes = Vec::new();
    file.read_to_end(&mut bytes)?;
    Ok(bytes)
}

/// Clear the group and other permission bits of the open `file`.
///
/// `at_path` is the metadata of `path` itself, read before the open without
/// following a link. The bits are cleared only when the open handle is the same
/// device and inode as the regular file found at `path` when its metadata was
/// read, and that file has a single link. A path that is a symbolic link or not
/// a regular file, that shares its file with another name, that was replaced
/// between the two reads, or whose metadata could not be read is read as usual
/// and its mode is left alone, logged at `warn`.
#[cfg(unix)]
fn narrow_to_owner(
    file: &std::fs::File,
    path: &Path,
    at_path: Option<&std::fs::Metadata>,
    what: &'static str,
) {
    use std::os::unix::fs::{MetadataExt as _, PermissionsExt as _};
    let Some(at_path) = at_path else {
        tracing::warn!(
            file = what,
            path = %path.display(),
            "a private file's metadata could not be read before it was opened; its mode \
             was not checked"
        );
        return;
    };
    if at_path.file_type().is_symlink() {
        tracing::warn!(
            file = what,
            path = %path.display(),
            "a private file's path is a symbolic link; the mode of its target is left alone"
        );
        return;
    }
    if !at_path.is_file() {
        tracing::warn!(
            file = what,
            path = %path.display(),
            "a private file's path is not a regular file; its mode is left alone"
        );
        return;
    }
    let meta = match file.metadata() {
        Ok(meta) => meta,
        Err(e) => {
            tracing::warn!(
                file = what,
                path = %path.display(),
                error = %e,
                "could not read a private file's mode"
            );
            return;
        }
    };
    if meta.dev() != at_path.dev() || meta.ino() != at_path.ino() {
        tracing::warn!(
            file = what,
            path = %path.display(),
            "a private file's path changed while it was opened; its mode is left alone"
        );
        return;
    }
    if meta.nlink() != 1 {
        tracing::warn!(
            file = what,
            path = %path.display(),
            links = meta.nlink(),
            "a private file has more than one link; its mode is left alone"
        );
        return;
    }
    let mode = meta.permissions().mode() & 0o7777;
    if mode & 0o077 == 0 {
        return;
    }
    let narrowed = mode & !0o077;
    match file.set_permissions(std::fs::Permissions::from_mode(narrowed)) {
        Ok(()) => tracing::warn!(
            file = what,
            path = %path.display(),
            mode = format_args!("{mode:o}"),
            narrowed = format_args!("{narrowed:o}"),
            "a private file was accessible to other users; cleared its group and other bits"
        ),
        Err(e) => tracing::warn!(
            file = what,
            path = %path.display(),
            mode = format_args!("{mode:o}"),
            error = %e,
            "a private file is accessible to other users and its group and other bits \
             could not be cleared"
        ),
    }
}

#[cfg(all(test, unix))]
mod tests {
    use super::*;
    use std::os::unix::fs::PermissionsExt as _;

    fn mode_of(path: &Path) -> u32 {
        std::fs::metadata(path).unwrap().permissions().mode() & 0o7777
    }

    fn write_with_mode(path: &Path, bytes: &[u8], mode: u32) {
        std::fs::write(path, bytes).unwrap();
        std::fs::set_permissions(path, std::fs::Permissions::from_mode(mode)).unwrap();
    }

    #[test]
    fn a_0644_file_is_read_and_narrowed_to_0600() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("key");
        write_with_mode(&path, b"secret", 0o644);
        assert_eq!(read_private(&path, "test file").unwrap(), b"secret");
        assert_eq!(mode_of(&path), 0o600);
    }

    #[test]
    fn owner_bits_and_special_bits_are_kept() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("key");
        write_with_mode(&path, b"x", 0o1474);
        read_private(&path, "test file").unwrap();
        assert_eq!(
            mode_of(&path),
            0o1400,
            "only group and other bits are cleared"
        );
    }

    #[test]
    fn a_symlink_at_the_path_is_read_through_and_its_target_keeps_its_mode() {
        let dir = tempfile::tempdir().unwrap();
        let target = dir.path().join("target");
        write_with_mode(&target, b"through the link", 0o644);
        let link = dir.path().join("key");
        std::os::unix::fs::symlink(&target, &link).unwrap();
        assert_eq!(
            read_private(&link, "test file").unwrap(),
            b"through the link"
        );
        assert_eq!(mode_of(&target), 0o644, "the link's target is not changed");
    }

    #[test]
    fn a_handle_that_is_not_the_file_at_the_path_is_left_alone() {
        // The metadata of another file stands in for the path having been
        // replaced between the read of its metadata and the open.
        let dir = tempfile::tempdir().unwrap();
        let opened = dir.path().join("opened");
        let other = dir.path().join("other");
        write_with_mode(&opened, b"x", 0o644);
        write_with_mode(&other, b"y", 0o644);
        let file = std::fs::File::open(&opened).unwrap();
        let other_meta = std::fs::symlink_metadata(&other).unwrap();
        narrow_to_owner(&file, &opened, Some(&other_meta), "test file");
        assert_eq!(mode_of(&opened), 0o644, "a different inode is not changed");
        let own_meta = std::fs::symlink_metadata(&opened).unwrap();
        narrow_to_owner(&file, &opened, Some(&own_meta), "test file");
        assert_eq!(mode_of(&opened), 0o600, "the same inode is narrowed");
    }

    #[test]
    fn a_file_with_another_hard_link_is_left_alone() {
        let dir = tempfile::tempdir().unwrap();
        let first = dir.path().join("first");
        write_with_mode(&first, b"shared", 0o644);
        let path = dir.path().join("key");
        std::fs::hard_link(&first, &path).unwrap();
        assert_eq!(read_private(&path, "test file").unwrap(), b"shared");
        assert_eq!(
            mode_of(&first),
            0o644,
            "a file with two links keeps its mode"
        );
    }

    /// A FIFO at `path` with `mode`, made with the system `mkfifo`, and a handle
    /// on it opened read-write so the open does not wait for a peer.
    fn fifo_with_mode(path: &Path, mode: u32) -> std::fs::File {
        let made = std::process::Command::new("mkfifo")
            .arg("-m")
            .arg(format!("{mode:o}"))
            .arg(path)
            .status()
            .expect("run mkfifo");
        assert!(made.success(), "mkfifo failed: {made}");
        std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open(path)
            .unwrap()
    }

    #[test]
    fn a_fifo_at_the_path_is_not_changed() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("key");
        let file = fifo_with_mode(&path, 0o644);
        let at_path = std::fs::symlink_metadata(&path).unwrap();
        narrow_to_owner(&file, &path, Some(&at_path), "test file");
        assert_eq!(mode_of(&path), 0o644, "a non-regular file keeps its mode");
    }

    #[test]
    fn a_directory_at_the_path_is_not_changed() {
        let dir = tempfile::tempdir().unwrap();
        let sub = dir.path().join("sub");
        std::fs::create_dir(&sub).unwrap();
        std::fs::set_permissions(&sub, std::fs::Permissions::from_mode(0o755)).unwrap();
        let _ = read_private(&sub, "test file");
        assert_eq!(mode_of(&sub), 0o755);
    }

    #[test]
    fn a_missing_file_is_not_found() {
        let dir = tempfile::tempdir().unwrap();
        let err = read_private(&dir.path().join("absent"), "test file").unwrap_err();
        assert_eq!(err.kind(), std::io::ErrorKind::NotFound);
    }
}

//! Runtime-internal compatibility seam for POSIX named shared memory.
//!
//! This module is stability-exempt. Its libc-shaped interface is available to
//! the CLI engine for named shared-memory operations.

#[cfg(not(target_os = "android"))]
pub use libc::{shm_open, shm_unlink};

#[cfg(test)]
use std::ffi::OsStr;
#[cfg(any(target_os = "android", test))]
use std::ffi::{CStr, CString, OsString};
#[cfg(any(target_os = "android", test))]
use std::io;
#[cfg(any(target_os = "android", test))]
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
#[cfg(any(target_os = "android", test))]
use std::os::unix::ffi::OsStrExt;
#[cfg(any(target_os = "android", test))]
use std::path::{Path, PathBuf};

#[cfg(any(target_os = "android", test))]
fn object_name(name: &CStr) -> Result<&[u8], libc::c_int> {
    let name_bytes = name.to_bytes();
    if name_bytes.len() < 2 || name_bytes[0] != b'/' {
        return Err(libc::EINVAL);
    }

    let object_name = &name_bytes[1..];
    if object_name.len() > 255 {
        return Err(libc::ENAMETOOLONG);
    }
    if object_name.contains(&b'/') || object_name == b"." || object_name == b".." {
        return Err(libc::EINVAL);
    }

    Ok(object_name)
}

#[cfg(test)]
fn object_path(dir: &Path, name: &CStr) -> Result<PathBuf, libc::c_int> {
    Ok(dir.join(OsStr::from_bytes(object_name(name)?)))
}

#[cfg(any(target_os = "android", test))]
fn open_dir(dir: &Path) -> Result<OwnedFd, libc::c_int> {
    let path = CString::new(dir.as_os_str().as_bytes()).map_err(|_| libc::EINVAL)?;
    // SAFETY: `path` is a live NUL-terminated string for the duration of the call.
    let fd = unsafe {
        libc::open(
            path.as_ptr(),
            libc::O_RDONLY | libc::O_DIRECTORY | libc::O_NOFOLLOW | libc::O_CLOEXEC,
        )
    };
    if fd < 0 {
        return Err(last_error_code());
    }
    // SAFETY: `fd` is a valid descriptor whose ownership moves into the OwnedFd,
    // which closes it exactly once on every return path.
    let dir_fd = unsafe { OwnedFd::from_raw_fd(fd) };
    // SAFETY: `status` is a fully owned stack struct written by `fstat` on a
    // descriptor we hold open.
    let mut status = unsafe { std::mem::zeroed::<libc::stat>() };
    if unsafe { libc::fstat(dir_fd.as_raw_fd(), &mut status) } != 0 {
        return Err(last_error_code());
    }
    // SAFETY: `geteuid` takes no arguments and has no effects.
    if status.st_uid != unsafe { libc::geteuid() } || status.st_mode & 0o022 != 0 {
        return Err(libc::EACCES);
    }
    Ok(dir_fd)
}

#[cfg(any(target_os = "android", test))]
fn shm_dir_from(shm_dir: Option<OsString>, tmpdir: Option<OsString>) -> PathBuf {
    if let Some(shm_dir) = shm_dir {
        let candidate = PathBuf::from(shm_dir);
        if !candidate.as_os_str().is_empty() && candidate.is_absolute() {
            return candidate;
        }
    }

    if let Some(tmpdir) = tmpdir {
        let candidate = PathBuf::from(tmpdir);
        if candidate.is_absolute() {
            return candidate.join("cerulion-shm");
        }
    }

    PathBuf::from("/data/local/tmp/cerulion-shm")
}

#[cfg(target_os = "android")]
fn resolved_shm_dir() -> &'static PathBuf {
    static SHM_DIR: std::sync::OnceLock<PathBuf> = std::sync::OnceLock::new();

    SHM_DIR.get_or_init(|| {
        let shm_dir = std::env::var_os("CERULION_SHM_DIR");
        if let Some(configured) = shm_dir.as_ref() {
            let path = Path::new(configured);
            if path.as_os_str().is_empty() || !path.is_absolute() {
                tracing::warn!(
                    path = %path.display(),
                    error = "CERULION_SHM_DIR must be a non-empty absolute path",
                    "ignoring invalid named shared-memory directory"
                );
            }
        }

        shm_dir_from(shm_dir, std::env::var_os("TMPDIR"))
    })
}

#[cfg(any(target_os = "android", test))]
fn io_error_code(error: io::Error) -> libc::c_int {
    error.raw_os_error().unwrap_or(libc::EIO)
}

#[cfg(any(target_os = "android", test))]
fn last_error_code() -> libc::c_int {
    io_error_code(io::Error::last_os_error())
}

#[cfg(any(target_os = "android", test))]
fn open_in(
    dir: &Path,
    name: &CStr,
    oflag: libc::c_int,
    mode: libc::mode_t,
) -> Result<libc::c_int, libc::c_int> {
    let object = object_name(name)?;
    if oflag & libc::O_CREAT != 0 {
        let created = !dir.exists();
        std::fs::create_dir_all(dir).map_err(io_error_code)?;
        if created {
            use std::os::unix::fs::PermissionsExt;

            std::fs::set_permissions(dir, std::fs::Permissions::from_mode(0o700))
                .map_err(io_error_code)?;
        }
    }

    let dir_fd = open_dir(dir)?;
    let path = CString::new(object).map_err(|_| libc::EINVAL)?;
    // SAFETY: `dir_fd` is an open O_DIRECTORY descriptor for `dir`, and `path`
    // is a NUL-terminated validated object name containing no '/'.
    let fd = unsafe {
        libc::openat(
            dir_fd.as_raw_fd(),
            path.as_ptr(),
            oflag | libc::O_CLOEXEC | libc::O_NOFOLLOW,
            mode as libc::c_uint,
        )
    };
    if fd < 0 {
        Err(last_error_code())
    } else {
        Ok(fd)
    }
}

#[cfg(any(target_os = "android", test))]
fn unlink_in(dir: &Path, name: &CStr) -> Result<(), libc::c_int> {
    let object = object_name(name)?;
    let dir_fd = open_dir(dir)?;
    let path = CString::new(object).map_err(|_| libc::EINVAL)?;
    // SAFETY: `dir_fd` is an open O_DIRECTORY descriptor for `dir`, and `path`
    // is a NUL-terminated validated object name containing no '/'.
    if unsafe { libc::unlinkat(dir_fd.as_raw_fd(), path.as_ptr(), 0) } == 0 {
        Ok(())
    } else {
        Err(last_error_code())
    }
}

#[cfg(target_os = "android")]
/// Opens a named object using Android's file-backed POSIX shared-memory seam.
///
/// # Safety
/// `name` must point to a valid NUL-terminated C string.
pub unsafe fn shm_open(
    name: *const libc::c_char,
    oflag: libc::c_int,
    mode: libc::mode_t,
) -> libc::c_int {
    // SAFETY: the caller guarantees that `name` points to a valid C string.
    match open_in(
        resolved_shm_dir(),
        unsafe { CStr::from_ptr(name) },
        oflag,
        mode,
    ) {
        Ok(fd) => fd,
        Err(error) => {
            // SAFETY: bionic exposes the calling thread's errno location.
            unsafe { *libc::__errno() = error };
            -1
        }
    }
}

#[cfg(target_os = "android")]
/// Removes a named object from Android's file-backed POSIX shared-memory seam.
///
/// # Safety
/// `name` must point to a valid NUL-terminated C string.
pub unsafe fn shm_unlink(name: *const libc::c_char) -> libc::c_int {
    // SAFETY: the caller guarantees that `name` points to a valid C string.
    match unlink_in(resolved_shm_dir(), unsafe { CStr::from_ptr(name) }) {
        Ok(()) => 0,
        Err(error) => {
            // SAFETY: bionic exposes the calling thread's errno location.
            unsafe { *libc::__errno() = error };
            -1
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{object_path, open_in, shm_dir_from, unlink_in};
    use std::ffi::{CStr, CString, OsString};
    use std::os::unix::fs::{symlink, PermissionsExt};
    use std::path::{Path, PathBuf};

    fn private_tempdir() -> tempfile::TempDir {
        let temp = tempfile::tempdir().expect("temporary directory");
        // Android's adb shell runs umask 0, so a tempdir can be group/other
        // writable; the shm dir contract refuses that.
        std::fs::set_permissions(temp.path(), std::fs::Permissions::from_mode(0o700))
            .expect("private temporary directory");
        temp
    }

    fn c_name(name: &str) -> CString {
        CString::new(name).expect("test names contain no NUL bytes")
    }

    #[test]
    fn named_object_is_created_mapped_reopened_and_unlinked() {
        const LENGTH: usize = 4096;
        let temp = private_tempdir();
        let name = c_name("/round-trip");
        let pattern = [0x31, 0x52, 0x73, 0x94, 0xb5, 0xd6, 0xf7, 0x18];
        let fd = open_in(
            temp.path(),
            &name,
            libc::O_CREAT | libc::O_EXCL | libc::O_RDWR,
            0o600,
        )
        .expect("create the named object");

        // SAFETY: `fd` is open, the mapping is sized to the truncated file, and
        // the pointer is unmapped before the descriptor is closed.
        unsafe {
            assert_eq!(libc::ftruncate(fd, LENGTH as libc::off_t), 0);
            let mapping = libc::mmap(
                std::ptr::null_mut(),
                LENGTH,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                fd,
                0,
            );
            assert_ne!(mapping, libc::MAP_FAILED);
            std::ptr::copy_nonoverlapping(pattern.as_ptr(), mapping.cast::<u8>(), pattern.len());
            assert_eq!(libc::munmap(mapping, LENGTH), 0);
            assert_eq!(libc::close(fd), 0);
        }

        let reopened = open_in(temp.path(), &name, libc::O_RDWR, 0).expect("reopen by name");
        // SAFETY: the reopened descriptor names the same 4096-byte object.
        unsafe {
            let mapping = libc::mmap(
                std::ptr::null_mut(),
                LENGTH,
                libc::PROT_READ,
                libc::MAP_SHARED,
                reopened,
                0,
            );
            assert_ne!(mapping, libc::MAP_FAILED);
            let bytes = std::slice::from_raw_parts(mapping.cast::<u8>(), pattern.len());
            assert_eq!(bytes, pattern.as_slice());
            assert_eq!(libc::munmap(mapping, LENGTH), 0);
            assert_eq!(libc::close(reopened), 0);
        }

        unlink_in(temp.path(), &name).expect("unlink the named object");
        assert_eq!(
            open_in(temp.path(), &name, libc::O_RDWR, 0),
            Err(libc::ENOENT)
        );
    }

    #[test]
    fn create_makes_a_private_directory_and_preserves_an_existing_directory() {
        let temp = private_tempdir();
        let new_dir = temp.path().join("new").join("nested");
        let name = c_name("/created");
        let fd = open_in(
            &new_dir,
            &name,
            libc::O_CREAT | libc::O_EXCL | libc::O_RDWR,
            0o600,
        )
        .expect("create under a new directory");
        // SAFETY: `fd` was returned by open_in.
        unsafe { assert_eq!(libc::close(fd), 0) };
        assert_eq!(
            std::fs::metadata(&new_dir)
                .expect("created directory metadata")
                .permissions()
                .mode()
                & 0o777,
            0o700
        );

        let existing_dir = temp.path().join("existing");
        std::fs::create_dir(&existing_dir).expect("existing directory");
        std::fs::set_permissions(&existing_dir, std::fs::Permissions::from_mode(0o755))
            .expect("set existing mode");
        let fd = open_in(
            &existing_dir,
            &name,
            libc::O_CREAT | libc::O_EXCL | libc::O_RDWR,
            0o600,
        )
        .expect("create under an existing directory");
        // SAFETY: `fd` was returned by open_in.
        unsafe { assert_eq!(libc::close(fd), 0) };
        assert_eq!(
            std::fs::metadata(&existing_dir)
                .expect("existing directory metadata")
                .permissions()
                .mode()
                & 0o777,
            0o755
        );
    }

    #[test]
    fn object_name_accepts_the_filesystem_name_limit() {
        let name = c_name(&format!("/{}", "n".repeat(255)));
        assert!(object_path(Path::new("/tmp/shm"), &name).is_ok());
    }

    #[test]
    fn invalid_names_are_rejected_before_filesystem_access() {
        let dir = Path::new("/unused");
        for name in ["", "missing-slash", "/", "/has/slash", "/.", "/.."] {
            let name = c_name(name);
            assert_eq!(object_path(dir, &name), Err(libc::EINVAL));
        }
        let too_long = c_name(&format!("/{}", "n".repeat(256)));
        assert_eq!(object_path(dir, &too_long), Err(libc::ENAMETOOLONG));
    }

    #[test]
    fn symlink_objects_are_refused() {
        let temp = private_tempdir();
        let target = temp.path().join("target");
        let object = temp.path().join("link");
        std::fs::write(&target, b"target").expect("write target");
        symlink(&target, &object).expect("create symlink");

        assert_eq!(
            open_in(temp.path(), &c_name("/link"), libc::O_RDONLY, 0),
            Err(libc::ELOOP)
        );
    }

    #[test]
    fn posix_errors_are_reported_for_invalid_open_unlink_and_directory_cases() {
        let temp = private_tempdir();
        let name = c_name("/object");
        assert_eq!(
            open_in(
                temp.path(),
                &name,
                libc::O_CREAT | libc::O_EXCL | libc::O_RDWR,
                0o600
            )
            .map(|fd| {
                // SAFETY: `fd` was returned by open_in.
                unsafe { libc::close(fd) };
            }),
            Ok(())
        );
        assert_eq!(
            open_in(
                temp.path(),
                &name,
                libc::O_CREAT | libc::O_EXCL | libc::O_RDWR,
                0o600
            ),
            Err(libc::EEXIST)
        );

        let missing = c_name("/missing");
        assert_eq!(
            open_in(temp.path(), &missing, libc::O_RDWR, 0),
            Err(libc::ENOENT)
        );
        assert_eq!(unlink_in(temp.path(), &missing), Err(libc::ENOENT));
        assert_eq!(
            unlink_in(temp.path(), &c_name("/has/slash")),
            Err(libc::EINVAL)
        );

        let regular_file = temp.path().join("not-a-directory");
        std::fs::write(&regular_file, b"file").expect("create regular file");
        let error = open_in(
            &regular_file,
            &c_name("/object"),
            libc::O_CREAT | libc::O_RDWR,
            0o600,
        )
        .expect_err("a regular file cannot be a directory");
        assert!(error == libc::ENOTDIR || error == libc::EEXIST);
    }

    #[test]
    fn repeated_object_names_resolve_to_the_same_path() {
        let name = c_name("/stable");
        let first = object_path(Path::new("/tmp/shm"), &name).expect("first path");
        let second = object_path(Path::new("/tmp/shm"), &name).expect("second path");
        assert_eq!(first, second);
    }

    #[test]
    fn shared_memory_directory_resolution_obeys_the_platform_fallbacks() {
        let cases = [
            (
                Some(OsString::from("/configured")),
                Some(OsString::from("/tmp")),
                PathBuf::from("/configured"),
            ),
            (
                None,
                Some(OsString::from("/tmp")),
                PathBuf::from("/tmp/cerulion-shm"),
            ),
            (None, None, PathBuf::from("/data/local/tmp/cerulion-shm")),
            (
                Some(OsString::from("relative")),
                Some(OsString::from("/tmp")),
                PathBuf::from("/tmp/cerulion-shm"),
            ),
            (
                Some(OsString::from("")),
                Some(OsString::from("relative-tmp")),
                PathBuf::from("/data/local/tmp/cerulion-shm"),
            ),
            (
                Some(OsString::from("/configured")),
                Some(OsString::from("relative-tmp")),
                PathBuf::from("/configured"),
            ),
        ];

        for (shm_dir, tmpdir, expected) in cases {
            assert_eq!(shm_dir_from(shm_dir, tmpdir), expected);
        }
    }

    #[test]
    fn object_path_uses_the_name_without_its_leading_slash() {
        let name: &CStr = &c_name("/item");
        assert_eq!(
            object_path(Path::new("/tmp/shm"), name),
            Ok(PathBuf::from("/tmp/shm/item"))
        );
    }

    #[test]
    fn symlinked_directories_are_refused_for_open_and_unlink() {
        let temp = private_tempdir();
        let real = temp.path().join("real");
        std::fs::create_dir(&real).expect("real directory");
        std::fs::set_permissions(&real, std::fs::Permissions::from_mode(0o700))
            .expect("private real directory");
        let link = temp.path().join("link");
        symlink(&real, &link).expect("directory symlink");

        for oflag in [libc::O_RDWR, libc::O_CREAT | libc::O_EXCL | libc::O_RDWR] {
            let error = open_in(&link, &c_name("/object"), oflag, 0o600)
                .expect_err("a symlinked directory must not redirect an object");
            assert!(
                error == libc::ELOOP || error == libc::ENOTDIR,
                "expected ELOOP or ENOTDIR, got {error}"
            );
        }
        let error = unlink_in(&link, &c_name("/object"))
            .expect_err("a symlinked directory must not redirect an unlink");
        assert!(
            error == libc::ELOOP || error == libc::ENOTDIR,
            "expected ELOOP or ENOTDIR, got {error}"
        );
        assert!(!real.join("object").exists());
    }

    #[test]
    fn group_or_other_writable_directories_are_refused_for_open_and_unlink() {
        let temp = private_tempdir();
        for mode in [0o777, 0o770] {
            let dir = temp.path().join(format!("dir-{mode:o}"));
            std::fs::create_dir(&dir).expect("writable directory");
            std::fs::set_permissions(&dir, std::fs::Permissions::from_mode(mode))
                .expect("set directory mode");

            let error = open_in(
                &dir,
                &c_name("/object"),
                libc::O_CREAT | libc::O_EXCL | libc::O_RDWR,
                0o600,
            )
            .expect_err("a group/other-writable directory must be refused");
            assert_eq!(error, libc::EACCES);
            let error = unlink_in(&dir, &c_name("/object"))
                .expect_err("a group/other-writable directory must be refused");
            assert_eq!(error, libc::EACCES);
            assert!(!dir.join("object").exists());
        }
    }
}

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
use std::path::{Component, Path, PathBuf};

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

/// The directory path with every `.` dropped, so its last component is the
/// directory itself and `O_NOFOLLOW` refuses it when it is a symlink (`link/.`
/// or a trailing `link/` would otherwise follow `link`). A `..` component or a
/// relative path is refused. Ancestor symlinks are followed on purpose:
/// security rests on `open_dir`'s owner/mode check of the opened descriptor
/// and on every object operation being `openat`/`unlinkat` relative to it.
#[cfg(any(target_os = "android", test))]
fn normalized_dir(dir: &Path) -> Result<PathBuf, libc::c_int> {
    if !dir.is_absolute()
        || dir
            .components()
            .any(|component| matches!(component, Component::ParentDir))
    {
        return Err(libc::EINVAL);
    }
    Ok(dir.components().collect())
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

/// Whether a uid can use the `/data/local/tmp` fallback. Android app ids
/// start at AID_APP_START (10000) within each 100000-user slice, so an app
/// process has `uid % 100000 >= 10000` and cannot write there; shell and root
/// (`uid % 100000 < 10000`) can.
#[cfg(any(target_os = "android", test))]
fn is_app_uid(uid: u32) -> bool {
    uid % 100_000 >= 10_000
}

/// Pure resolution: an explicit `set_shm_dir` wins, then an absolute
/// `CERULION_SHM_DIR`, then `$TMPDIR/cerulion-shm` for an absolute `TMPDIR`.
/// The `/data/local/tmp` fallback exists only for non-app uids; an app process
/// with nothing configured gets EACCES rather than a directory it cannot use.
#[cfg(any(target_os = "android", test))]
fn shm_dir_from(
    explicit: Option<&Path>,
    shm_dir: Option<OsString>,
    tmpdir: Option<OsString>,
    uid: u32,
) -> Result<PathBuf, libc::c_int> {
    if let Some(explicit) = explicit {
        return Ok(explicit.to_path_buf());
    }

    if let Some(shm_dir) = shm_dir {
        let candidate = PathBuf::from(shm_dir);
        if !candidate.as_os_str().is_empty() && candidate.is_absolute() {
            return Ok(candidate);
        }
    }

    if let Some(tmpdir) = tmpdir {
        let candidate = PathBuf::from(tmpdir);
        if candidate.is_absolute() {
            return Ok(candidate.join("cerulion-shm"));
        }
    }

    if is_app_uid(uid) {
        return Err(libc::EACCES);
    }
    Ok(PathBuf::from("/data/local/tmp/cerulion-shm"))
}

/// The resolved directory and the explicit override behind it. `resolved`
/// latches only on a successful resolution: a `set_shm_dir` that arrives
/// afterwards may only confirm the same path, while a failed resolution
/// latches nothing, so a later `set` still recovers the process.
#[cfg(any(target_os = "android", test))]
struct ShmDirState {
    explicit: Option<PathBuf>,
    resolved: Option<PathBuf>,
}

#[cfg(any(target_os = "android", test))]
impl ShmDirState {
    const fn new() -> Self {
        Self {
            explicit: None,
            resolved: None,
        }
    }

    fn set(&mut self, dir: &Path) -> Result<(), libc::c_int> {
        if !dir.is_absolute() {
            return Err(libc::EINVAL);
        }
        match &self.resolved {
            Some(resolved) if resolved == dir => return Ok(()),
            Some(_) => return Err(libc::EBUSY),
            None => {}
        }
        self.explicit = Some(dir.to_path_buf());
        Ok(())
    }

    fn resolve(
        &mut self,
        shm_dir: Option<OsString>,
        tmpdir: Option<OsString>,
        uid: u32,
    ) -> Result<PathBuf, libc::c_int> {
        if let Some(resolved) = &self.resolved {
            return Ok(resolved.clone());
        }
        let resolved = shm_dir_from(self.explicit.as_deref(), shm_dir, tmpdir, uid);
        if let Ok(dir) = &resolved {
            self.resolved = Some(dir.clone());
        }
        resolved
    }
}

/// Sets the directory for file-backed named shared-memory objects.
///
/// `dir` must be an absolute path and must be set before the first named
/// object is opened or unlinked. It takes precedence over `CERULION_SHM_DIR`
/// and `TMPDIR`. App processes cannot use the `/data/local/tmp` fallback, so
/// an app embedder must call this (or export `CERULION_SHM_DIR`) with a
/// directory the app owns, such as one under its files directory. Repeating
/// the call with the already-resolved directory is a no-op; a different
/// directory after a successful resolution, or a non-absolute path, is an
/// error. A failed resolution latches nothing, so `set` still succeeds after
/// one.
///
/// This function is stability-exempt like the rest of this module.
#[cfg(target_os = "android")]
pub fn set_shm_dir(dir: &Path) -> io::Result<()> {
    shm_dir_state()
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
        .set(dir)
        .map_err(io::Error::from_raw_os_error)
}

#[cfg(target_os = "android")]
fn shm_dir_state() -> &'static std::sync::Mutex<ShmDirState> {
    static STATE: std::sync::Mutex<ShmDirState> = std::sync::Mutex::new(ShmDirState::new());
    &STATE
}

#[cfg(target_os = "android")]
fn resolved_shm_dir() -> Result<PathBuf, libc::c_int> {
    let mut state = shm_dir_state()
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    if state.resolved.is_none() {
        warn_on_invalid_shm_dir_env();
    }
    state
        .resolve(
            std::env::var_os("CERULION_SHM_DIR"),
            std::env::var_os("TMPDIR"),
            // SAFETY: `getuid` takes no arguments and has no effects.
            unsafe { libc::getuid() },
        )
        .inspect_err(|error| {
            if *error == libc::EACCES {
                tracing::error!(
                    error = "an app process cannot use the /data/local/tmp fallback",
                    "no named shared-memory directory configured: call set_shm_dir or export CERULION_SHM_DIR with an absolute app-writable directory"
                );
            }
        })
}

#[cfg(target_os = "android")]
fn warn_on_invalid_shm_dir_env() {
    if let Some(configured) = std::env::var_os("CERULION_SHM_DIR") {
        let path = Path::new(&configured);
        if path.as_os_str().is_empty() || !path.is_absolute() {
            tracing::warn!(
                path = %path.display(),
                error = "CERULION_SHM_DIR must be a non-empty absolute path",
                "ignoring invalid named shared-memory directory"
            );
        }
    }
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
    let dir = &normalized_dir(dir)?;
    let creating = oflag & libc::O_CREAT != 0;
    let created = creating && !dir.exists();
    if creating {
        use std::os::unix::fs::DirBuilderExt;

        // `recursive` mkdirs every missing component 0700 at birth (umask can
        // only strip bits, never grant group/other access); std treats an
        // already-existing directory - or one another process races into
        // existence - as success, never EEXIST.
        std::fs::DirBuilder::new()
            .recursive(true)
            .mode(0o700)
            .create(dir)
            .map_err(io_error_code)?;
    }

    let dir_fd = open_dir(dir)?;
    // Restore the owner bits a restrictive umask could strip through the
    // descriptor `open_dir` checked, never through the path: a path swapped
    // for a symlink after the mkdir would redirect the mode change.
    // SAFETY: `dir_fd` is an open directory descriptor owned by this frame.
    if created && unsafe { libc::fchmod(dir_fd.as_raw_fd(), 0o700) } != 0 {
        return Err(last_error_code());
    }
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
    let dir_fd = open_dir(&normalized_dir(dir)?)?;
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
    let dir = match resolved_shm_dir() {
        Ok(dir) => dir,
        Err(error) => {
            // SAFETY: bionic exposes the calling thread's errno location.
            unsafe { *libc::__errno() = error };
            return -1;
        }
    };
    match open_in(&dir, unsafe { CStr::from_ptr(name) }, oflag, mode) {
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
    let dir = match resolved_shm_dir() {
        Ok(dir) => dir,
        Err(error) => {
            // SAFETY: bionic exposes the calling thread's errno location.
            unsafe { *libc::__errno() = error };
            return -1;
        }
    };
    match unlink_in(&dir, unsafe { CStr::from_ptr(name) }) {
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
    use super::{object_path, open_in, shm_dir_from, unlink_in, ShmDirState};
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
        assert_eq!(
            std::fs::metadata(new_dir.parent().expect("intermediate directory"))
                .expect("intermediate directory metadata")
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
                Ok(PathBuf::from("/configured")),
            ),
            (
                None,
                Some(OsString::from("/tmp")),
                Ok(PathBuf::from("/tmp/cerulion-shm")),
            ),
            (
                None,
                None,
                Ok(PathBuf::from("/data/local/tmp/cerulion-shm")),
            ),
            (
                Some(OsString::from("relative")),
                Some(OsString::from("/tmp")),
                Ok(PathBuf::from("/tmp/cerulion-shm")),
            ),
            (
                Some(OsString::from("")),
                Some(OsString::from("relative-tmp")),
                Ok(PathBuf::from("/data/local/tmp/cerulion-shm")),
            ),
            (
                Some(OsString::from("/configured")),
                Some(OsString::from("relative-tmp")),
                Ok(PathBuf::from("/configured")),
            ),
        ];

        // uid 0 (root) may use the /data/local/tmp fallback.
        for (shm_dir, tmpdir, expected) in cases {
            assert_eq!(shm_dir_from(None, shm_dir, tmpdir, 0), expected);
        }
    }

    #[test]
    fn an_explicit_directory_wins_over_the_environment() {
        assert_eq!(
            shm_dir_from(
                Some(Path::new("/explicit")),
                Some(OsString::from("/configured")),
                Some(OsString::from("/tmp")),
                10_123
            ),
            Ok(PathBuf::from("/explicit"))
        );
    }

    #[test]
    fn an_app_uid_needs_a_configured_directory() {
        // AID_APP_START is 10000 in each 100000-user slice, so 10123 and the
        // secondary-user 110123 are app uids; shell (2000) and root (0) are not.
        for app_uid in [10_000, 10_123, 110_123] {
            assert_eq!(
                shm_dir_from(None, None, None, app_uid),
                Err(libc::EACCES),
                "app uid {app_uid} must not fall back to /data/local/tmp"
            );
            assert_eq!(
                shm_dir_from(None, Some(OsString::from("relative")), None, app_uid),
                Err(libc::EACCES)
            );
            assert_eq!(
                shm_dir_from(None, None, Some(OsString::from("/data/data/pkg")), app_uid),
                Ok(PathBuf::from("/data/data/pkg/cerulion-shm"))
            );
        }
        for non_app_uid in [0, 1_000, 2_000] {
            assert_eq!(
                shm_dir_from(None, None, None, non_app_uid),
                Ok(PathBuf::from("/data/local/tmp/cerulion-shm"))
            );
        }
    }

    #[test]
    fn set_shm_dir_rules_and_the_resolution_latch() {
        let mut state = ShmDirState::new();
        assert_eq!(
            state.set(Path::new("relative/dir")),
            Err(libc::EINVAL),
            "a non-absolute directory is refused"
        );

        state
            .set(Path::new("/explicit"))
            .expect("set before resolution");
        assert_eq!(
            state.resolve(Some(OsString::from("/configured")), None, 0),
            Ok(PathBuf::from("/explicit")),
            "the explicit directory wins over the environment"
        );

        // After resolution only the same directory may be set again.
        assert_eq!(
            state.set(Path::new("/other")),
            Err(libc::EBUSY),
            "a different directory after resolution is an error"
        );
        state
            .set(Path::new("/explicit"))
            .expect("re-setting the resolved directory is a no-op");

        // A failed resolution latches nothing: set can still recover the
        // process, and the next resolve uses the directory it provides.
        let mut unconfigured = ShmDirState::new();
        assert_eq!(unconfigured.resolve(None, None, 10_123), Err(libc::EACCES));
        unconfigured
            .set(Path::new("/late"))
            .expect("set after a failed resolution still succeeds");
        assert_eq!(
            unconfigured.resolve(None, None, 10_123),
            Ok(PathBuf::from("/late")),
            "the next resolution uses the late-set directory"
        );

        // A second explicit set before resolution replaces the first.
        let mut pending = ShmDirState::new();
        pending
            .set(Path::new("/first"))
            .expect("first set before resolution");
        pending
            .set(Path::new("/second"))
            .expect("re-set before resolution");
        assert_eq!(pending.resolve(None, None, 0), Ok(PathBuf::from("/second")));
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
        // `link`, `link/` and `link/.` all resolve to the same directory; the
        // raw path bytes differ, and a trailing `/` or `/.` would hide the
        // symlink from `O_NOFOLLOW`, so every spelling is exercised.
        let spellings = [
            link.clone(),
            link.join("."),
            PathBuf::from(format!("{}/", link.display())),
        ];
        for dir in &spellings {
            for oflag in [libc::O_RDWR, libc::O_CREAT | libc::O_EXCL | libc::O_RDWR] {
                let error = open_in(dir, &c_name("/object"), oflag, 0o600)
                    .expect_err("a symlinked directory must not redirect an object");
                assert!(
                    error == libc::ELOOP || error == libc::ENOTDIR,
                    "expected ELOOP or ENOTDIR, got {error}"
                );
            }
            let error = unlink_in(dir, &c_name("/object"))
                .expect_err("a symlinked directory must not redirect an unlink");
            assert!(
                error == libc::ELOOP || error == libc::ENOTDIR,
                "expected ELOOP or ENOTDIR, got {error}"
            );
        }
        assert!(!real.join("object").exists());
    }

    #[test]
    fn relative_directory_paths_are_refused() {
        let relative = Path::new("relative/dir");
        for oflag in [libc::O_RDWR, libc::O_CREAT | libc::O_EXCL | libc::O_RDWR] {
            assert_eq!(
                open_in(relative, &c_name("/object"), oflag, 0o600),
                Err(libc::EINVAL)
            );
        }
        assert_eq!(unlink_in(relative, &c_name("/object")), Err(libc::EINVAL));
    }

    #[test]
    fn trailing_dot_on_a_real_private_directory_still_works() {
        let temp = private_tempdir();
        let dir = temp.path().join(".");
        let name = c_name("/round-trip");
        let fd = open_in(
            &dir,
            &name,
            libc::O_CREAT | libc::O_EXCL | libc::O_RDWR,
            0o600,
        )
        .expect("create through a trailing-dot path");
        // SAFETY: `fd` was returned by open_in.
        unsafe { assert_eq!(libc::close(fd), 0) };
        assert!(temp.path().join("round-trip").exists());
        unlink_in(&dir, &name).expect("unlink through a trailing-dot path");
        assert!(!temp.path().join("round-trip").exists());
    }

    #[test]
    fn a_trailing_dot_does_not_hide_a_symlinked_directory() {
        let temp = private_tempdir();
        let real = temp.path().join("real");
        std::fs::create_dir(&real).expect("real directory");
        std::fs::set_permissions(&real, std::fs::Permissions::from_mode(0o700))
            .expect("private real directory");
        std::fs::write(real.join("existing"), b"kept").expect("existing object");
        symlink(&real, temp.path().join("link")).expect("directory symlink");
        let through_dot = temp.path().join("link").join(".");

        for oflag in [libc::O_RDWR, libc::O_CREAT | libc::O_EXCL | libc::O_RDWR] {
            let error = open_in(&through_dot, &c_name("/object"), oflag, 0o600)
                .expect_err("`link/.` must not redirect an object");
            assert!(
                error == libc::ELOOP || error == libc::ENOTDIR,
                "expected ELOOP or ENOTDIR, got {error}"
            );
        }
        let error = unlink_in(&through_dot, &c_name("/existing"))
            .expect_err("`link/.` must not redirect an unlink");
        assert!(
            error == libc::ELOOP || error == libc::ENOTDIR,
            "expected ELOOP or ENOTDIR, got {error}"
        );
        assert!(!real.join("object").exists());
        assert!(real.join("existing").exists());
    }

    #[test]
    fn a_parent_component_is_refused_for_open_and_unlink() {
        let temp = private_tempdir();
        let real = temp.path().join("real");
        std::fs::create_dir(&real).expect("real directory");
        std::fs::set_permissions(&real, std::fs::Permissions::from_mode(0o700))
            .expect("private real directory");
        let via_parent = real.join("..").join("real");

        for oflag in [libc::O_RDWR, libc::O_CREAT | libc::O_EXCL | libc::O_RDWR] {
            assert_eq!(
                open_in(&via_parent, &c_name("/object"), oflag, 0o600),
                Err(libc::EINVAL)
            );
        }
        assert_eq!(
            unlink_in(&via_parent, &c_name("/object")),
            Err(libc::EINVAL)
        );
        assert!(!real.join("object").exists());
    }

    #[test]
    fn concurrent_creators_of_a_missing_directory_both_succeed() {
        for iteration in 0..50 {
            let temp = private_tempdir();
            let dir = temp.path().join("race").join("nested");
            let barrier = std::sync::Barrier::new(2);
            std::thread::scope(|scope| {
                let results: Vec<_> = (0..2)
                    .map(|index| {
                        let dir = &dir;
                        let barrier = &barrier;
                        scope.spawn(move || {
                            barrier.wait();
                            let name = c_name(&format!("/object-{iteration}-{index}"));
                            open_in(
                                dir,
                                &name,
                                libc::O_CREAT | libc::O_EXCL | libc::O_RDWR,
                                0o600,
                            )
                            .inspect(|fd| {
                                // SAFETY: `fd` was returned by open_in.
                                unsafe { libc::close(*fd) };
                            })
                        })
                    })
                    .collect();
                for handle in results {
                    handle
                        .join()
                        .expect("creator thread panicked")
                        .expect("concurrent creator must succeed");
                }
            });
        }
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

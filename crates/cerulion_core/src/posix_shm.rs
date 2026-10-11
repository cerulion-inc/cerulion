//! POSIX named shared memory, with a file-backed fallback for Android.
//!
//! SPIKE (`rk/android-arm64-spike`): bionic has no `shm_open` / `shm_unlink`,
//! so on `target_os = "android"` a named object `/name` is a regular file
//! `$CERULION_SHM_DIR/name` (default `/data/local/tmp/cerulion/shm`). Two
//! processes with the same UID that `mmap(MAP_SHARED)` that file share the
//! pages, which is the property every caller in this crate needs. Every other
//! target re-exports the libc functions, so nothing changes there.

#[cfg(not(target_os = "android"))]
pub(crate) use libc::{shm_open, shm_unlink};

#[cfg(target_os = "android")]
mod android {
    use std::ffi::{CStr, CString};
    use std::os::unix::ffi::OsStrExt;
    use std::path::PathBuf;

    const DEFAULT_DIR: &str = "/data/local/tmp/cerulion/shm";

    fn path_for(name: *const libc::c_char) -> Option<CString> {
        if name.is_null() {
            return None;
        }
        // SAFETY: the caller passes a NUL-terminated C string (shm_open contract).
        let raw = unsafe { CStr::from_ptr(name) }.to_bytes();
        let leaf: Vec<u8> = raw
            .iter()
            .copied()
            .skip_while(|b| *b == b'/')
            .map(|b| if b == b'/' { b'_' } else { b })
            .collect();
        if leaf.is_empty() {
            return None;
        }
        let dir = std::env::var_os("CERULION_SHM_DIR")
            .map_or_else(|| PathBuf::from(DEFAULT_DIR), PathBuf::from);
        let _ = std::fs::create_dir_all(&dir);
        let mut bytes = dir.as_os_str().as_bytes().to_vec();
        bytes.push(b'/');
        bytes.extend_from_slice(&leaf);
        CString::new(bytes).ok()
    }

    fn einval() -> libc::c_int {
        // SAFETY: __errno() returns the calling thread's errno slot.
        unsafe { *libc::__errno() = libc::EINVAL };
        -1
    }

    /// File-backed stand-in for `shm_open(3)`.
    ///
    /// # Safety
    /// `name` must be a valid NUL-terminated C string.
    pub(crate) unsafe fn shm_open(
        name: *const libc::c_char,
        oflag: libc::c_int,
        mode: libc::mode_t,
    ) -> libc::c_int {
        let Some(path) = path_for(name) else {
            return einval();
        };
        // SAFETY: `path` is a valid C string; open(2) has no other precondition.
        unsafe {
            libc::open(
                path.as_ptr(),
                oflag | libc::O_CLOEXEC | libc::O_NOFOLLOW,
                libc::c_uint::from(mode),
            )
        }
    }

    /// File-backed stand-in for `shm_unlink(3)`.
    ///
    /// # Safety
    /// `name` must be a valid NUL-terminated C string.
    pub(crate) unsafe fn shm_unlink(name: *const libc::c_char) -> libc::c_int {
        let Some(path) = path_for(name) else {
            return einval();
        };
        // SAFETY: `path` is a valid C string.
        unsafe { libc::unlink(path.as_ptr()) }
    }
}

#[cfg(target_os = "android")]
pub(crate) use android::{shm_open, shm_unlink};

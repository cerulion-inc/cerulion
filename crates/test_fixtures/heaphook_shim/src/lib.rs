// SPDX-License-Identifier: AGPL-3.0-only
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]
//! `libheaphook_shim.so` — a trivial `malloc`-family interposer that forwards to
//! glibc's real allocator (`__libc_*`, always present and never itself
//! interposed, so there is no dlsym-bootstrap recursion to handle).
//!
//! Its only purpose is to WIN `malloc` resolution when preloaded AHEAD of
//! `libcerulion_heaphook.so` (`LD_PRELOAD=libheaphook_shim.so:libcerulion_heaphook.so`),
//! so the hook's `won_malloc` module-identity check must report
//! `status_won = 0` — the foreign-allocator degrade the whole handshake gate
//! rests on. Linux/GNU only; an empty lib elsewhere (it never runs off-Linux).

#[cfg(all(target_os = "linux", target_env = "gnu"))]
mod shim {
    use core::ffi::c_void;

    extern "C" {
        fn __libc_malloc(size: usize) -> *mut c_void;
        fn __libc_free(ptr: *mut c_void);
        fn __libc_calloc(nmemb: usize, size: usize) -> *mut c_void;
        fn __libc_realloc(ptr: *mut c_void, size: usize) -> *mut c_void;
    }

    /// # Safety
    /// Standard C `malloc`.
    #[no_mangle]
    pub unsafe extern "C" fn malloc(size: usize) -> *mut c_void {
        __libc_malloc(size)
    }
    /// # Safety
    /// Standard C `free`.
    #[no_mangle]
    pub unsafe extern "C" fn free(ptr: *mut c_void) {
        __libc_free(ptr)
    }
    /// # Safety
    /// Standard C `calloc`.
    #[no_mangle]
    pub unsafe extern "C" fn calloc(nmemb: usize, size: usize) -> *mut c_void {
        __libc_calloc(nmemb, size)
    }
    /// # Safety
    /// Standard C `realloc`.
    #[no_mangle]
    pub unsafe extern "C" fn realloc(ptr: *mut c_void, size: usize) -> *mut c_void {
        __libc_realloc(ptr, size)
    }
}

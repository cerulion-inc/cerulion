// SPDX-License-Identifier: AGPL-3.0-only
//! C ABI types for the rmw surface.
//!
//! Source selection (see build.rs):
//! - `cerulion_rmw_generated_bindings`: bindgen output from the
//!   installed ROS distro's headers (deployment path).
//! - `cerulion_rmw_vendored_bindings`: the committed
//!   `vendored_bindings` (private module) generated from pinned source
//!   clones (development fallback).
//!
//! Plus a small set of hand-written helpers over the generated types:
//! return-code constants, gid construction, and allocator call-throughs
//! (allocation for rcl-owned out-params MUST go through the caller's
//! `rcutils_allocator_t` function pointers — rcl frees with the same
//! allocator).

#[cfg(cerulion_rmw_generated_bindings)]
// Same allows the vendored file carries inline: bindgen output contains
// unused items (dead_code = deny would reject the whole distro
// header surface), plus C naming and bindgen's own clippy noise. Note
// bindgen ALSO emits RMW_RET_* constants — our hand-written ones below
// shadow them at the re-export (explicit items win over glob imports).
#[allow(non_camel_case_types, non_snake_case, non_upper_case_globals)]
#[allow(dead_code)]
#[allow(clippy::all)]
// C doc comments (`\param[in]`, bare URLs, `<T>`) trip rustdoc lints.
#[allow(
    rustdoc::broken_intra_doc_links,
    rustdoc::bare_urls,
    rustdoc::invalid_html_tags
)]
mod generated {
    include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
}
#[cfg(cerulion_rmw_generated_bindings)]
pub use generated::*;

#[cfg(cerulion_rmw_vendored_bindings)]
#[path = "vendored_bindings.rs"]
mod vendored_bindings;
#[cfg(cerulion_rmw_vendored_bindings)]
pub use vendored_bindings::*;

pub mod introspection_cpp;

// Per-era struct-size pins keyed on the capability cfgs —
// compile-time only (`const _` asserts), nothing to export.
mod era_pins;

use std::os::raw::{c_char, c_void};
#[cfg(feature = "test-seams")]
use std::sync::Mutex;

// ====================================================================
// Return codes (rmw/ret_types.h — stable numeric contract).
// ====================================================================

pub const RMW_RET_OK: rmw_ret_t = 0;
pub const RMW_RET_ERROR: rmw_ret_t = 1;
pub const RMW_RET_TIMEOUT: rmw_ret_t = 2;
pub const RMW_RET_UNSUPPORTED: rmw_ret_t = 3;
pub const RMW_RET_BAD_ALLOC: rmw_ret_t = 10;
pub const RMW_RET_INVALID_ARGUMENT: rmw_ret_t = 11;
pub const RMW_RET_INCORRECT_RMW_IMPLEMENTATION: rmw_ret_t = 12;
pub const RMW_RET_NODE_NAME_NON_EXISTENT: rmw_ret_t = 203;

// ====================================================================
// Typesupport identifiers (string contents are the contract; pointer
// identity comparison is an optimization rcl does NOT rely on).
// ====================================================================

/// The identifier this implementation reports.
pub const IMPLEMENTATION_IDENTIFIER: &[u8] = b"rmw_cerulion\0";

/// Introspection typesupport identifier (C string) — the handle we ask
/// rosidl's typesupport dispatcher for.
pub const INTROSPECTION_C_IDENTIFIER: &[u8] = b"rosidl_typesupport_introspection_c\0";

/// Serialization format reported by `rmw_get_serialization_format`.
/// Cerulion publishes its native flat wire format, not CDR.
pub const SERIALIZATION_FORMAT: &[u8] = b"cerulion\0";

/// Implementation identifier as a `*const c_char` for struct stamping.
#[inline]
pub fn implementation_identifier_ptr() -> *const c_char {
    IMPLEMENTATION_IDENTIFIER.as_ptr() as *const c_char
}

/// True when `id` (a NUL-terminated C string) names this implementation.
///
/// # Safety
/// `id` must be null or a valid NUL-terminated C string.
pub unsafe fn is_our_identifier(id: *const c_char) -> bool {
    if id.is_null() {
        return false;
    }
    let ours = IMPLEMENTATION_IDENTIFIER;
    for (i, &b) in ours.iter().enumerate() {
        let c = *id.add(i) as u8;
        if c != b {
            return false;
        }
        if b == 0 {
            break;
        }
    }
    true
}

// ====================================================================
// Allocator call-throughs.
// ====================================================================

/// Allocate `len` bytes through a caller-provided rcutils allocator.
/// Returns null when the allocator is missing its function pointer or
/// reports failure.
///
/// # Safety
/// `allocator` must be a valid `rcutils_allocator_t` (zero-initialized
/// is handled: missing fn pointer → null).
pub unsafe fn allocator_alloc(allocator: *const rcutils_allocator_t, len: usize) -> *mut c_void {
    if allocator.is_null() {
        return std::ptr::null_mut();
    }
    let a = &*allocator;
    match a.allocate {
        Some(f) => f(len, a.state),
        None => std::ptr::null_mut(),
    }
}

/// Copy a Rust string into freshly allocated, NUL-terminated memory
/// owned by the caller's allocator. Returns null on allocation failure.
///
/// # Safety
/// `allocator` must be a valid `rcutils_allocator_t`.
pub unsafe fn allocator_strdup(allocator: *const rcutils_allocator_t, s: &str) -> *const c_char {
    let mem = allocator_alloc(allocator, s.len() + 1) as *mut u8;
    if mem.is_null() {
        return std::ptr::null();
    }
    std::ptr::copy_nonoverlapping(s.as_ptr(), mem, s.len());
    *mem.add(s.len()) = 0;
    mem as *const c_char
}

// ====================================================================
// Panic containment.
// ====================================================================

/// Run an rmw entry-point body, converting any Rust panic into `fail`
/// instead of unwinding across the `extern "C"` boundary (which aborts
/// the whole host process — e.g. all of MoveIt — on what should be a
/// recoverable per-call error).
pub(crate) fn ffi_guard<T>(fail: T, f: impl FnOnce() -> T) -> T {
    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(f)) {
        Ok(v) => v,
        Err(payload) => {
            // The whole reporting block runs under a SECOND
            // `catch_unwind`: it installs tracing,
            // allocates, calls into libdl and emits a `tracing` event —
            // and `install_tracing` deliberately lets a host's OWN
            // subscriber win, so a foreign `on_event` that panics would
            // unwind out of the recovery arm and abort the host, the
            // exact outcome this guard exists to prevent. A failure to
            // REPORT must still return the caller's failure value.
            let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                // On the PRE-RUNTIME exports no subscriber
                // exists yet — a real host has no Rust `tracing` subscriber —
                // so without this the containment line below would go to
                // `NoSubscriber` and the caller would see a bare error code.
                // Idempotent, panic path only.
                crate::runtime::install_tracing();
                let msg = payload
                    .downcast_ref::<&str>()
                    .copied()
                    .or_else(|| payload.downcast_ref::<String>().map(|s| s.as_str()))
                    .unwrap_or("<non-string panic payload>");
                // The same rcl channel the era refusal uses: rclpy/rclcpp
                // otherwise report a bare code with "error not set".
                let rcl = rcutils_set_error_state_best_effort(&format!(
                    "rmw_cerulion: rmw entry point panicked: {msg}"
                ));
                tracing::error!(
                    panic = %msg,
                    rcl_error_channel = %rcl,
                    "rmw entry point panicked; returning error to caller"
                );
            }));
            fail
        }
    }
}

// ====================================================================
// rcutils error-state hygiene.
// ====================================================================

/// Resolve `symbol` from rcl's ALREADY-LOADED `librcutils`. Null when
/// absent, e.g. outside a ROS process.
///
/// Two lookups, because RTLD_DEFAULT is not enough:
/// it searches the global scope plus this library's own dependency
/// closure, and a C++ host (move_group, any rclcpp executable) has
/// `librcutils` in the executable's closure — global — so it resolves;
/// but an **rclpy** host (every `ros2 …` CLI verb, the bench scripts)
/// gets `librcutils` only through `_rclpy_pybind11.so`, which CPython
/// dlopens `RTLD_LOCAL`, invisible to RTLD_DEFAULT — the mpi4py/plugin
/// class. `dlopen(soname, RTLD_NOLOAD)` returns the handle of an
/// object that is already mapped whatever scope it was loaded into,
/// and never loads anything new, so it is tried FIRST; RTLD_DEFAULT is
/// the fallback.
unsafe fn resolve_rcutils_symbol(symbol: &std::ffi::CStr) -> *mut std::ffi::c_void {
    extern "C" {
        fn dlopen(filename: *const c_char, flags: std::os::raw::c_int) -> *mut std::ffi::c_void;
        fn dlsym(handle: *mut std::ffi::c_void, symbol: *const c_char) -> *mut std::ffi::c_void;
    }
    // Per-platform constants (the libc crate is a dev-dependency only).
    #[cfg(target_os = "macos")]
    const RTLD_LAZY_NOLOAD: std::os::raw::c_int = 0x1 | 0x10;
    #[cfg(not(target_os = "macos"))]
    const RTLD_LAZY_NOLOAD: std::os::raw::c_int = 0x1 | 0x4;
    #[cfg(target_os = "macos")]
    const RCUTILS_SONAME: &std::ffi::CStr = c"librcutils.dylib";
    #[cfg(not(target_os = "macos"))]
    const RCUTILS_SONAME: &std::ffi::CStr = c"librcutils.so";
    let handle = dlopen(RCUTILS_SONAME.as_ptr(), RTLD_LAZY_NOLOAD);
    if !handle.is_null() {
        let sym = dlsym(handle, symbol.as_ptr());
        if !sym.is_null() {
            return sym;
        }
    }
    // The RTLD_DEFAULT constant differs by platform: glibc/musl use 0,
    // macOS uses -2.
    #[cfg(target_os = "macos")]
    let rtld_default = -2isize as *mut std::ffi::c_void;
    #[cfg(not(target_os = "macos"))]
    let rtld_default = std::ptr::null_mut::<std::ffi::c_void>();
    dlsym(rtld_default, symbol.as_ptr())
}

/// Whether a refusal's reason reached rcl's error channel — the value
/// every diagnostic site stamps as `rcl_error_channel=`. An enum, not a
/// `bool` mapped to `"set"`/`"unavailable"` at each site (three
/// sites, one spelling).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RclErrorChannel {
    /// `rcutils_set_error_state` took the message: `rmw_get_error_string()`
    /// will report it to rclpy/rclcpp.
    Set,
    /// The symbol did not resolve, or the text could not be rendered —
    /// the reason is on stderr only.
    Unavailable,
}

impl std::fmt::Display for RclErrorChannel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            RclErrorChannel::Set => "set",
            RclErrorChannel::Unavailable => "unavailable",
        })
    }
}

/// The C string handed to `rcutils_set_error_state`: `msg` with every
/// embedded NUL rendered as the two characters `\0`, so a panic payload
/// carrying one still reaches rcl (a `CString` cannot hold a NUL, and a
/// conversion failure would otherwise read as "channel unavailable").
///
/// `None` only if the checked constructor still refuses. `replace`
/// leaves no NUL, so that is unreachable — but the arm must exist and
/// must NOT substitute a placeholder: this runs on `ffi_guard`'s
/// panic-recovery path, OUTSIDE its `catch_unwind`, where an `expect`
/// would unwind through the C ABI, while a constant stand-in would let
/// the caller report `rcl_error_channel=set` over text rcl never
/// received. `None` makes the caller say
/// `unavailable`, which sends the operator to stderr — where the real
/// message is.
pub(crate) fn rcl_error_text(msg: &str) -> Option<std::ffi::CString> {
    // Backslash FIRST, else a payload containing the
    // two characters `\0` renders identically to one carrying a real NUL
    // and the operator cannot tell which they sent.
    std::ffi::CString::new(msg.replace('\\', "\\\\").replace('\0', "\\0")).ok()
}

/// Best-effort `rcutils_set_error_state(msg, file, line)` — the channel
/// rcl reads back as `rmw_get_error_string()` and hands to rclpy/rclcpp
/// on a failed return (a refusal that only reaches
/// `tracing` leaves the ROS user with "error not set"). The caller carries
/// the returned [`RclErrorChannel`] on its own diagnostic line so an
/// operator can tell "rcl has the reason" from "only stderr does".
/// Any embedded NUL in `msg` is rendered visibly ([`rcl_error_text`]) —
/// a panic payload can carry one, and a message that then failed to
/// convert would otherwise leave the channel silently unset.
///
/// The resolution is cached in a `OnceLock` — including a FAILURE:
/// the first call that resolves nothing pins
/// `Unavailable` for the process lifetime. That is correct for the
/// shipping shape (a `.so` is dlopen'd into a host whose `librcutils`
/// is either mapped for the whole run or never), and it is why a
/// diagnostic must not read the value as a live probe.
///
/// [`RclErrorChannel::Unavailable`] states the OUTCOME, never a cause:
/// the usual reason is that no `librcutils` is mapped
/// — this is not a ROS process — but [`resolve_rcutils_symbol`] folds
/// every `dlopen`/`dlsym` failure into a null pointer without reading
/// `dlerror`, so a resolution that failed for some other reason inside a
/// real ROS host reports the same value. The line must not assert which.
/// The last message handed to `rcutils_set_error_state`, recorded only
/// under `test-seams`. EVERY cargo test process maps no
/// `librcutils`, so the reported outcome is always
/// [`RclErrorChannel::Unavailable`] and asserting that value pins
/// NOTHING — replacing the call with a hardcoded `Unavailable` would pass
/// the whole crate suite. This records the ASK, whatever the outcome, so
/// a test can prove the channel was really given the reason.
///
/// The record itself must
/// not allocate on the per-MESSAGE path — the C++ refusal calls the
/// `CStr` setter on every resolve, so rendering each ask
/// into a fresh `String` would allocate per message. A `'static` ask is recorded as the REFERENCE
/// (`Static`, no allocation); only the load-time `&str` entry point,
/// which renders and allocates anyway, records an owned copy.
#[cfg(feature = "test-seams")]
enum LastRclAsk {
    Static(&'static std::ffi::CStr),
    Owned(String),
}

#[cfg(feature = "test-seams")]
static LAST_RCL_ERROR_TEXT: Mutex<Option<LastRclAsk>> = Mutex::new(None);

/// How many times rcl's error channel has been ASKED (`test-seams`
/// only) — every setter call, whatever the outcome. A test
/// that reads only the LAST text cannot tell "set on every
/// refusal" from "set once, then never refreshed"; the count can.
#[cfg(feature = "test-seams")]
static RCL_ERROR_ASKS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

#[cfg(feature = "test-seams")]
fn record_rcl_ask(ask: LastRclAsk) {
    RCL_ERROR_ASKS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    *LAST_RCL_ERROR_TEXT
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner()) = Some(ask);
}

/// The last message rcl's error channel was asked to publish, or `None`
/// if it has never been asked (`test-seams` only). Renders on READ, so
/// the recording side stays allocation-free.
#[cfg(feature = "test-seams")]
pub fn last_rcl_error_text() -> Option<String> {
    match &*LAST_RCL_ERROR_TEXT
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
    {
        Some(LastRclAsk::Static(c)) => Some(c.to_string_lossy().into_owned()),
        Some(LastRclAsk::Owned(s)) => Some(s.clone()),
        None => None,
    }
}

/// The running count of asks (`test-seams` only); see [`RCL_ERROR_ASKS`].
#[cfg(feature = "test-seams")]
pub fn rcl_error_asks() -> u64 {
    RCL_ERROR_ASKS.load(std::sync::atomic::Ordering::Relaxed)
}

/// The allocation-free half of [`rcutils_set_error_state_best_effort`]:
/// hand rcl a message that is ALREADY a `CStr`.
/// A per-MESSAGE refusal path must not render and
/// allocate a fresh string on every call, so its caller builds the C
/// string ONCE — the text is a build-time constant — and passes it here.
pub(crate) fn rcutils_set_error_state_cstr(msg: &'static std::ffi::CStr) -> RclErrorChannel {
    // `'static` is the contract that keeps the seam allocation-free: the
    // per-message caller hands a once-built string, and the record keeps
    // the reference, not a copy.
    #[cfg(feature = "test-seams")]
    record_rcl_ask(LastRclAsk::Static(msg));
    set_error_state_raw(msg)
}

/// The FFI half shared by both entry points: hand `msg` to the resolved
/// `rcutils_set_error_state`, or report the channel unavailable.
fn set_error_state_raw(msg: &std::ffi::CStr) -> RclErrorChannel {
    let Some(f) = resolved_set_error_state() else {
        return RclErrorChannel::Unavailable;
    };
    // SAFETY: the symbol was resolved from librcutils, whose
    // rcutils_set_error_state copies both strings into thread-local
    // error state and keeps no pointer to them.
    unsafe { f(msg.as_ptr(), c"rmw_cerulion".as_ptr(), 0) }
    RclErrorChannel::Set
}

/// The resolved `rcutils_set_error_state`, or `None` outside a ROS
/// process. Resolved ONCE — see the caller's docs for why a cached
/// FAILURE is right for a dlopen'd `.so`.
fn resolved_set_error_state() -> Option<SetErrorState> {
    static SET: std::sync::OnceLock<Option<SetErrorState>> = std::sync::OnceLock::new();
    *SET.get_or_init(|| unsafe {
        let sym = resolve_rcutils_symbol(c"rcutils_set_error_state");
        if sym.is_null() {
            None
        } else {
            Some(std::mem::transmute::<*mut std::ffi::c_void, SetErrorState>(
                sym,
            ))
        }
    })
}

type SetErrorState = unsafe extern "C" fn(*const c_char, *const c_char, usize);

pub(crate) fn rcutils_set_error_state_best_effort(msg: &str) -> RclErrorChannel {
    // Renders (and therefore ALLOCATES) — correct for the callers that
    // build a per-call message anyway (a panic payload, a distro pair);
    // the per-MESSAGE refusal path uses `rcutils_set_error_state_cstr`
    // with a once-built string instead.
    let Some(rendered) = rcl_error_text(msg) else {
        return RclErrorChannel::Unavailable;
    };
    // This path already rendered; recording an owned copy adds nothing
    // per message (it is the load-time entry point).
    #[cfg(feature = "test-seams")]
    record_rcl_ask(LastRclAsk::Owned(msg.to_owned()));
    set_error_state_raw(&rendered)
}

/// Best-effort `rcutils_reset_error()`.
///
/// A *speculative* typesupport probe that fails (asking a dispatcher
/// for the other language's identifier) goes through rosidl's
/// `type_support_dispatch`, which SETS the process rcutils error
/// state. When the fallback probe then succeeds, that stale string is
/// what rcl reports on the next unrelated failure — a misleading
/// diagnostic rmw_fastrtps avoids by resetting between probes.
///
/// Resolved at runtime via `dlsym` so this crate (and its tests, which
/// run without any ROS installation) never links `librcutils`
/// directly: in a real ROS 2 host process rcl has `librcutils` loaded,
/// so the symbol resolves; in tests it is absent and this is a no-op.
pub(crate) fn rcutils_reset_error_best_effort() {
    use std::sync::OnceLock;
    static RESET: OnceLock<Option<unsafe extern "C" fn()>> = OnceLock::new();
    let f = RESET.get_or_init(|| unsafe {
        let sym = resolve_rcutils_symbol(c"rcutils_reset_error");
        if sym.is_null() {
            None
        } else {
            Some(std::mem::transmute::<
                *mut std::ffi::c_void,
                unsafe extern "C" fn(),
            >(sym))
        }
    });
    if let Some(f) = f {
        // SAFETY: the symbol was resolved from librcutils, whose
        // rcutils_reset_error takes no arguments and only clears
        // thread-local error state.
        unsafe { f() }
    }
}

// ====================================================================
// C-string helpers.
// ====================================================================

/// Borrow a NUL-terminated C string as `&str`. Returns `None` for null
/// pointers or non-UTF-8 contents (ROS names are ASCII by spec).
///
/// # Safety
/// `ptr` must be null or a valid NUL-terminated C string that outlives
/// the returned borrow.
pub unsafe fn cstr<'a>(ptr: *const c_char) -> Option<&'a str> {
    if ptr.is_null() {
        return None;
    }
    std::ffi::CStr::from_ptr(ptr).to_str().ok()
}

#[cfg(test)]
mod tests {
    use super::ffi_guard;

    #[test]
    fn ffi_guard_passes_through_ok_values() {
        assert_eq!(ffi_guard(-1i32, || 0), 0);
    }

    // `#[traced_test]` on both: the panic arm installs the crate's
    // tracing subscriber, and a plain test doing that would
    // take the lib binary's global-subscriber slot from the traced
    // tests that share it. The capture also pins that the containment
    // line is emitted at all.
    #[tracing_test::traced_test]
    #[test]
    fn ffi_guard_converts_panics_to_fail_value() {
        assert_eq!(ffi_guard(-1i32, || panic!("boom")), -1);
        assert!(logs_contain("rmw entry point panicked"));
        assert!(logs_contain("panic=boom"));
        // Outside a ROS process no librcutils is mapped: the line says so.
        assert!(logs_contain("rcl_error_channel=unavailable"));
    }

    #[test]
    fn rcl_error_text_renders_embedded_nuls_visibly() {
        // A panic payload can carry a NUL, which no
        // CString can; the text must still reach rcl, with the NUL
        // visible rather than the whole message dropped.
        let rendered = |m: &str| {
            super::rcl_error_text(m)
                .unwrap_or_else(|| panic!("the NUL-free rendering of {m:?} must convert"))
        };
        assert_eq!(rendered("bad\0byte").as_bytes(), b"bad\\0byte");
        assert_eq!(rendered("clean").as_bytes(), b"clean");
        assert_eq!(rendered("\0\0").as_bytes(), b"\\0\\0");
        // A real NUL and the literal two characters `\0` must not render
        // the same: the backslash is escaped first.
        assert_eq!(rendered("bad\\0byte").as_bytes(), b"bad\\\\0byte");
        assert_ne!(
            rendered("bad\0byte").as_bytes(),
            rendered("bad\\0byte").as_bytes()
        );
    }

    #[tracing_test::traced_test]
    #[test]
    fn ffi_guard_converts_string_panics_to_fail_value() {
        let v: *mut u8 = ffi_guard(std::ptr::null_mut(), || {
            panic!("{}", String::from("heap msg"))
        });
        assert!(v.is_null());
        assert!(logs_contain("panic=heap msg"));
    }
}

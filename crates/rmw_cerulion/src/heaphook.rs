// SPDX-License-Identifier: AGPL-3.0-only
//! The CONSUMER side of the borrow-window heap hook
//! handshake — the rmw half of the seam `cerulion_heaphook`
//! exports for exactly this caller.
//!
//! # What this module is
//!
//! `libcerulion_heaphook.so` is an `LD_PRELOAD` payload the `cerulion ros2
//! run|launch` launcher auto-injects (see `CERULION_ROS2_PRELOAD` in
//! USER_API.md). While a thread-local BORROW WINDOW is armed over a loan
//! slot's tail, the hook bump-allocates every `malloc`-family request on that
//! thread into the tail — so a stock node's `std::vector`/`std::string` fill
//! lands directly in shared memory. This module resolves the hook at runtime
//! and gates the whole publish-side feature on the versioned handshake:
//! every degrade outcome (hook absent, version skew, a foreign allocator
//! ahead of the hook) is the copy path, reported loudly ONCE — never a
//! failed publish, never silence.
//!
//! # Why dlsym'd STANDALONE symbols, not the vtable
//!
//! The hook exports its ABI two ways: one `cerulion_heaphook_abi()` symbol
//! returning a `repr(C)` vtable, AND each entry as a standalone
//! `cerulion_heaphook_*` symbol (the hook's `exports.rs` documents the
//! standalone set as the layout-free handshake path: "a consumer can
//! handshake with two dlsyms — `version` + `status` — and never needs the
//! vtable struct layout"). This consumer takes the standalone path on
//! purpose: the rmw must NOT link the hook crate (linking its rlib would
//! compile the `malloc`-family interposers into `librmw_cerulion.so` /
//! every rmw test binary — a second, un-preloaded copy of the allocator
//! seam), so using the vtable would mean MIRRORING its struct layout here
//! and trusting the mirror. Per-symbol resolution leaves only SCALARS to
//! mirror (symbol names, the version number, the status bit, the return
//! codes), and `heaphook_source_parity` pins each of those against the hook
//! crate's own source so a drift fails a test instead of skewing at runtime.
//! The version gate still guards SEMANTIC drift exactly as it does for a
//! vtable consumer.
//!
//! # The window contract this consumer drives (the hook's `exports.rs` doc)
//!
//! 1. borrow: loan a slot, `arm_window(tail_base, tail_limit)` on the
//!    filling thread;
//! 2. the stock fill bumps into the tail;
//! 3. publish: `window_range_test(ptr, len)` per variable member (adopted ⇒
//!    zero-copy; not ⇒ copy, loudly), read `window_escape()`, THEN
//!    `disarm_window()`;
//! 4. the slot's quarantine entry (registered at arm time) outlives the
//!    window until `retire_slot(tail_base)` — the rmw retires a slot's
//!    extent when it observes the transport REUSING that slot (the only
//!    reclamation signal an rmw can see), never at publish/return time,
//!    because a caller-held incidental allocation bump-allocated during the
//!    fill may be freed long after the frame shipped and must keep hitting
//!    the quarantine no-op instead of glibc `free` on an SHM address.
//!    `retire_slot` ENDS this rmw's coverage obligation for the slot;
//!    the rc is deliberately ignored. (Hook-side disposition: retire may
//!    DELETE the quarantine entry, or
//!    TOMBSTONE it with revive, where a straggling in-slot free still hits
//!    a counted no-op. Under EITHER model the rmw's re-arm on slot reuse
//!    re-establishes coverage, which is why no rmw code depends on the
//!    disposition; same symbol, signature and return codes either way.)
//!
//! `window_range_test` answers only while the window is ARMED, so the
//! publish path probes (range tests + the cursor bisection below) BEFORE
//! disarming.
//!
//! # Cursor bisection ([`bisect_window_cursor`])
//!
//! The seal needs the window's bump CURSOR — copied (non-adopted) members
//! must be placed at/above it, because every byte BELOW the cursor was
//! handed to some allocation on the filling thread and may be a LIVE object
//! the caller still holds (writing there is memory corruption, the one
//! outcome this design forbids). The v2 ABI exports no cursor read, so the
//! consumer recovers it exactly through the adopt test's own semantics:
//! `window_range_test(ptr, 0)` is `ptr >= base && ptr <= cursor` (a
//! zero-length range is adopted iff it ends at or below the cursor), which
//! is monotone in `ptr` — a ~25-step bisection over the tail recovers the
//! cursor with no ABI change. A dedicated `window_used` export would need
//! an ABI bump; the bisection keeps the rmw strictly additive on the
//! hook's v2 ABI.

use std::os::raw::c_void;
use std::sync::OnceLock;

// ── Mirrored scalars (pinned against the hook crate's source by the
//    `heaphook_source_parity` test below — never edit one side alone) ──────

/// Mirror of `cerulion_heaphook::abi::HEAPHOOK_ABI_VERSION` — the version
/// this consumer was written against. A hook reporting anything else is
/// REFUSED (copy path, loud once): the version covers the semantics of every
/// entry this module drives.
pub const HEAPHOOK_ABI_VERSION: u32 = 2;

/// Mirror of `cerulion_heaphook::abi::HEAPHOOK_STATUS_WON_MALLOC`: the hook
/// resolved as the process's `malloc`. Clear means another interposer
/// (jemalloc/tcmalloc/ASan) is ahead of it and a window would never bump.
pub const HEAPHOOK_STATUS_WON_MALLOC: u32 = 1 << 0;

/// Mirror of `cerulion_heaphook::abi::RC_OK`.
pub const RC_OK: i32 = 0;
/// Mirror of `RC_ERR_ALREADY_ARMED`: `arm_window` while this thread already
/// has a window (a second outstanding borrow on one thread — the windowless
/// borrow arm).
pub const RC_ERR_ALREADY_ARMED: i32 = -1;
/// Mirror of `RC_ERR_NOT_ARMED`: a window query/disarm with no window on
/// this thread (the wrong-thread publish shape).
pub const RC_ERR_NOT_ARMED: i32 = -2;
/// Mirror of `RC_ERR_BAD_ARG` (adopt-take — the take-side
/// registration error space): a null start or zero-length range handed to
/// `register_segment`.
pub const RC_ERR_BAD_ARG: i32 = -3;
/// Mirror of `RC_ERR_OVERLAP` (adopt-take): `register_segment` over a
/// range overlapping a LIVE registration. Ranges of one adopted sample are
/// disjoint (distinct offset-table entries), so this arm means a stale
/// leaked registration — "should be never", handled loudly anyway.
pub const RC_ERR_OVERLAP: i32 = -4;
/// Mirror of `RC_ERR_UNKNOWN`: `retire_slot` for a base no quarantine
/// extent starts at, or `unregister_segment` for a start no registration
/// begins at.
pub const RC_ERR_UNKNOWN: i32 = -5;
/// Mirror of `RC_ERR_UNAVAILABLE`: the window thread-local is being torn
/// down (late thread teardown) — an arm is impossible.
pub const RC_ERR_UNAVAILABLE: i32 = -6;

/// The diagnostic-counter indices of `cerulion_heaphook_counter` (mirror of
/// the hook's `state::counter` match): releases with no callback set,
/// bootstrap-arena exhaustions, pre-resolution real-heap-free leaks,
/// quarantine no-op frees, tombstone hits (a retired slot's in-slot pointer
/// freed late — tombstone-on-retire coverage working), atfork-interval
/// registered-range leaks (a foreign prepare handler freed a registered SHM
/// range inside the prepare-lock interval). Surfaced at rmw shutdown — the
/// hook's own docs name this rmw as these counters' in-process reader.
///
/// SENTINEL RULE: `u64::MAX` is the hook's documented
/// unknown-index sentinel (`state::counter`'s `_ => return u64::MAX` arm —
/// the vtable itself is unchanged; counter KINDS are data, not layout). Kinds
/// are only ever ADDED, so an OLDER v2 hook — a stale `.so` in a mixed
/// deployment — answers `MAX` for a kind this consumer knows about (4/5 were
/// added after v2 shipped). The consumer therefore treats `MAX` as "this
/// hook does not serve this kind" and DEGRADES that kind gracefully
/// ([`hook_counters`] reports it `None`; [`log_hook_counters`] renders it
/// `unavailable`) — never as a real count of 18446744073709551615.
pub const COUNTER_KINDS: [(u32, &str); 6] = [
    (0, "release_without_callback"),
    (1, "bootstrap_exhausted"),
    (2, "pre_resolution_leak"),
    (3, "quarantine_noop_frees"),
    (4, "tombstone_hits"),
    (5, "atfork_interval_leaks"),
];

// ── The handshake decision (pure twin of `cerulion_heaphook::abi`) ─────────

/// The consumer-side handshake outcome — the pure twin of
/// `cerulion_heaphook::abi::HandshakeVerdict`, re-implemented here because
/// the rmw deliberately does not link the hook crate (see the module doc).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HandshakeVerdict {
    /// Drive the window: the borrow path may offer unbounded-type loans.
    Active,
    /// No hook symbol in the process — nothing preloaded. Copy path.
    DegradeAbsent,
    /// The hook's version differs from [`HEAPHOOK_ABI_VERSION`]. Copy path.
    DegradeVersionMismatch,
    /// Another malloc interposer won resolution — a window would never
    /// bump. Copy path.
    DegradeForeignAllocator,
}

impl HandshakeVerdict {
    /// Whether the hook may be used.
    pub fn is_active(self) -> bool {
        matches!(self, HandshakeVerdict::Active)
    }
}

/// The pure handshake decision, mirroring the hook's own
/// `decide_handshake` order: absence first (nothing else is readable),
/// version second (a skewed hook's `status` semantics are untrusted), the
/// won-malloc bit last.
pub fn decide_handshake(
    present: bool,
    hook_version: u32,
    expected_version: u32,
    status: u32,
) -> HandshakeVerdict {
    if !present {
        return HandshakeVerdict::DegradeAbsent;
    }
    if hook_version != expected_version {
        return HandshakeVerdict::DegradeVersionMismatch;
    }
    if status & HEAPHOOK_STATUS_WON_MALLOC == 0 {
        return HandshakeVerdict::DegradeForeignAllocator;
    }
    HandshakeVerdict::Active
}

// ── The resolved hook API ──────────────────────────────────────────────────

/// The hook entries this rmw drives, resolved per-symbol at init (or
/// installed by a test seam). Plain `Copy` fn pointers — nothing here owns
/// anything, and the pointees are `'static` for the process lifetime (the
/// preloaded `.so` is never unloaded; a test hook's fns are Rust items).
#[derive(Clone, Copy)]
pub struct HookApi {
    /// `cerulion_heaphook_arm_window(base, tail_limit)`.
    pub arm_window: unsafe extern "C" fn(*mut c_void, *mut c_void) -> i32,
    /// `cerulion_heaphook_disarm_window()` → latched escape code (0 = none)
    /// or [`RC_ERR_NOT_ARMED`].
    pub disarm_window: unsafe extern "C" fn() -> i32,
    /// `cerulion_heaphook_window_escape()` → latched escape code (0 = none)
    /// or [`RC_ERR_NOT_ARMED`].
    pub window_escape: unsafe extern "C" fn() -> i32,
    /// `cerulion_heaphook_window_range_test(ptr, len)` → 1 adopted / 0 not /
    /// [`RC_ERR_NOT_ARMED`].
    pub window_range_test: unsafe extern "C" fn(*const c_void, usize) -> i32,
    /// `cerulion_heaphook_retire_slot(base)` → [`RC_OK`] / [`RC_ERR_UNKNOWN`].
    pub retire_slot: unsafe extern "C" fn(*mut c_void) -> i32,
    /// `cerulion_heaphook_counter(kind)` → the diagnostic counter, or
    /// `u64::MAX` for an unknown kind.
    pub counter: unsafe extern "C" fn(u32) -> u64,
    /// Adopt-take: `cerulion_heaphook_register_segment(start, len,
    /// cookie)` — register an adopted SHM byte range (exact per-field
    /// sub-range, per the hook's granularity contract) so a later `free()`
    /// of any address inside it routes to the release callback instead of
    /// glibc → [`RC_OK`] / [`RC_ERR_BAD_ARG`] / [`RC_ERR_OVERLAP`].
    pub register_segment: unsafe extern "C" fn(*mut c_void, usize, usize) -> i32,
    /// Adopt-take: `cerulion_heaphook_unregister_segment(start)` —
    /// withdraw a registration by its start address WITHOUT firing the
    /// callback (the caller-side rollback, not a release) → [`RC_OK`] /
    /// [`RC_ERR_UNKNOWN`].
    pub unregister_segment: unsafe extern "C" fn(*mut c_void) -> i32,
    /// Adopt-take: `cerulion_heaphook_set_release_callback(cb)` — set
    /// (or clear, with `None`) the ONE process-global release callback the
    /// hook fires when a `free`/`realloc` hits a registered range →
    /// [`RC_OK`]. Mirror of the hook's `ReleaseCallback` type.
    pub set_release_callback: unsafe extern "C" fn(HookReleaseCallback) -> i32,
}

/// A hook that ANSWERS and INTERPOSES NOTHING — the test-double base that
/// makes widening [`HookApi`] ONE edit instead of six.
///
/// Six test binaries install a fake (`TestHookGuard::install`). Were each
/// to hand-build the whole literal, every new field would break
/// all six AT ONCE — and `git merge` would report a clean textual merge while
/// it did (one side builds the literal, the other widens the struct,
/// `error[E0063]`).
/// With a base to spread from, a widening compiles everywhere that does not
/// care about the new entry.
///
/// NOT `Default`, deliberately. Functional record update takes any
/// expression of the struct type, so `..HookApi::inert()` costs a fake
/// nothing over `..Default::default()` — and it says at the call site that
/// the remainder answers nothing, where `default()` would only say
/// "unremarkable". The canonical "no hook" in this crate is
/// [`active_hook`] returning `None`; a `Default` would mint a SECOND,
/// `Some`-shaped spelling of it, reachable by the most reflexive
/// constructor call in Rust — and an inert `HookApi` yields a valid
/// `AdoptTakeGrant` whose `register_segment` reports success and registers
/// nothing.
///
/// What the stubs answer, and why that story is coherent: a window ARMS
/// (`RC_OK`), nothing escapes it (`0` from `disarm_window`/`window_escape`
/// is the ABI's "no latched escape", not an error), and no address is in an
/// adopted range (`0` from `window_range_test` — a fake that claimed
/// adoption would hand a caller a range nothing registered). `retire_slot`,
/// `register_segment`, `unregister_segment` and `set_release_callback`
/// report `RC_OK` because that is what a double that does nothing
/// successfully must say.
///
/// `counter` reads `0` — the ONE deliberate departure from the ABI this
/// struct mirrors, which reserves `u64::MAX` for an UNKNOWN kind (see the
/// field's own doc). An inert counter answers `0` for every kind because
/// the suite that cares about that sentinel supplies its own `counter`
/// (`rmw_heaphook_counter_log_test`'s stale-v2 fake), and a base that
/// returned `u64::MAX` for unknown kinds would make every unread counter
/// look like a stale hook.
///
/// All six fakes override `disarm_window`, `window_escape` and
/// `window_range_test` with entries of their own, so none of them rests on
/// the inert defaults for window state. That is load-bearing rather than
/// incidental: `arm_window` answers `RC_OK` and `disarm_window` answers
/// "no latched escape", so a SEVENTH suite that spread from here without
/// overriding them would read a window as armed and cleanly disarmed — not
/// NOT-ARMED — which is the shape to watch for when adding one.
///
/// The trade this makes, stated rather than left to be discovered: a
/// widening is SILENT in test code — an `E0063` across six
/// binaries is noise, but it is also the detector that forces someone to
/// decide, per suite, about the new entry. A suite that needs to drive a
/// new entry must notice on its own. The ONE production literal —
/// `resolve_process_hook`'s, built per-symbol through `dlsym` (private, so
/// not intra-doc-linked — the docs gate denies that warning) — spells
/// every field out ON PURPOSE, so the widening still fails loudly there,
/// which is where it must.
///
/// Gated on `test-seams`. That feature is in no shipping recipe, and a
/// hand-built `--features test-seams` cdylib IS deployable (see
/// `test_seams.rs`) — but nothing in this crate constructs an inert hook,
/// so it is present-and-unreachable there rather than armed.
#[cfg(feature = "test-seams")]
impl HookApi {
    /// See the impl docs: every entry answers successfully and does
    /// nothing. Test doubles spread from it (`..HookApi::inert()`).
    pub fn inert() -> Self {
        unsafe extern "C" fn inert_arm(_base: *mut c_void, _tail: *mut c_void) -> i32 {
            RC_OK
        }
        unsafe extern "C" fn inert_disarm() -> i32 {
            0
        }
        unsafe extern "C" fn inert_escape() -> i32 {
            0
        }
        unsafe extern "C" fn inert_range_test(_p: *const c_void, _len: usize) -> i32 {
            0
        }
        unsafe extern "C" fn inert_retire(_base: *mut c_void) -> i32 {
            RC_OK
        }
        unsafe extern "C" fn inert_counter(_kind: u32) -> u64 {
            0
        }
        unsafe extern "C" fn inert_register(_s: *mut c_void, _len: usize, _cookie: usize) -> i32 {
            RC_OK
        }
        unsafe extern "C" fn inert_unregister(_s: *mut c_void) -> i32 {
            RC_OK
        }
        unsafe extern "C" fn inert_set_release_callback(_cb: HookReleaseCallback) -> i32 {
            RC_OK
        }
        Self {
            arm_window: inert_arm,
            disarm_window: inert_disarm,
            window_escape: inert_escape,
            window_range_test: inert_range_test,
            retire_slot: inert_retire,
            counter: inert_counter,
            register_segment: inert_register,
            unregister_segment: inert_unregister,
            set_release_callback: inert_set_release_callback,
        }
    }
}

/// Mirror of `cerulion_heaphook::abi::ReleaseCallback` (adopt-take):
/// the process-global callback `set_release_callback` installs — fired with
/// the freed address and the registration's cookie; `None` clears it.
pub type HookReleaseCallback = Option<unsafe extern "C" fn(*mut c_void, usize)>;

/// The process-wide resolution outcome: the verdict plus (iff Active) the
/// resolved API.
struct ResolvedHook {
    verdict: HandshakeVerdict,
    api: Option<HookApi>,
}

static RESOLVED: OnceLock<ResolvedHook> = OnceLock::new();

/// The active hook API, or `None` on any degrade. Resolves ONCE per process
/// (the preload set cannot change after load) and logs the outcome once —
/// `info!` when active, `warn!` on the two skew degrades (a deployment put
/// the hook in the process and it cannot be used — actionable), `debug!` on
/// plain absence (the ordinary non-launcher run; not a condition).
///
/// Under `test-seams`, an installed test hook (`TestHookGuard::install` —
/// not intra-doc-linked because the item exists only under the feature)
/// takes precedence and is consulted per-call, so serial test binaries can
/// install and tear down fakes around each test.
pub fn active_hook() -> Option<HookApi> {
    #[cfg(feature = "test-seams")]
    if let Some(api) = test_hook() {
        return Some(api);
    }
    let resolved = RESOLVED.get_or_init(|| {
        let r = resolve_process_hook();
        match r.verdict {
            HandshakeVerdict::Active => tracing::info!(
                abi_version = HEAPHOOK_ABI_VERSION,
                "heap hook handshake ACTIVE — unbounded-type publish borrows may adopt \
                 in-slot storage (escapes still copy, loudly)"
            ),
            HandshakeVerdict::DegradeAbsent => tracing::debug!(
                "no cerulion heap hook in this process — unbounded-type publishes keep \
                 the copy path (fixed-type zero-copy is unaffected)"
            ),
            HandshakeVerdict::DegradeVersionMismatch => tracing::warn!(
                expected_abi = HEAPHOOK_ABI_VERSION,
                "cerulion heap hook is preloaded but its ABI version differs from the one \
                 this rmw was built against — unbounded-type publishes DEGRADE to the copy \
                 path (redeploy the hook and the rmw together); fixed-type zero-copy is \
                 unaffected"
            ),
            HandshakeVerdict::DegradeForeignAllocator => tracing::warn!(
                "cerulion heap hook is preloaded but another malloc interposer won symbol \
                 resolution (jemalloc/tcmalloc/ASan ahead of it) — unbounded-type publishes \
                 DEGRADE to the copy path; fixed-type zero-copy is unaffected"
            ),
        }
        r
    });
    resolved.api
}

/// The one-shot process verdict (for diagnostics; [`active_hook`] is the
/// gate). Does not itself log.
pub fn process_verdict() -> HandshakeVerdict {
    #[cfg(feature = "test-seams")]
    if test_hook().is_some() {
        return HandshakeVerdict::Active;
    }
    RESOLVED.get_or_init(resolve_process_hook_quietly).verdict
}

fn resolve_process_hook_quietly() -> ResolvedHook {
    // Reached only when `process_verdict` runs before the first
    // `active_hook` call; the verdict is identical, just unlogged here (the
    // first `active_hook` call after this will find RESOLVED set and log
    // nothing — acceptable: every production path goes through
    // `active_hook` first).
    resolve_process_hook()
}

// ── Per-symbol resolution (Linux — the hook's only platform) ──────────────

#[cfg(target_os = "linux")]
fn resolve_process_hook() -> ResolvedHook {
    use std::ffi::CStr;

    unsafe fn sym(name: &CStr) -> *mut c_void {
        // SAFETY: dlsym with RTLD_DEFAULT and a NUL-terminated name is
        // defined on glibc; the result is only ever transmuted to the exact
        // exported signature.
        unsafe { libc::dlsym(libc::RTLD_DEFAULT, name.as_ptr()) }
    }

    // Presence = the version symbol resolves (the layout-free handshake the
    // hook's exports.rs documents).
    let version_sym = unsafe { sym(c"cerulion_heaphook_version") };
    if version_sym.is_null() {
        return ResolvedHook {
            verdict: HandshakeVerdict::DegradeAbsent,
            api: None,
        };
    }
    // SAFETY: non-null dlsym result for the exported `extern "C" fn() -> u32`.
    let version_fn =
        unsafe { std::mem::transmute::<*mut c_void, unsafe extern "C" fn() -> u32>(version_sym) };
    let hook_version = unsafe { version_fn() };
    if hook_version != HEAPHOOK_ABI_VERSION {
        return ResolvedHook {
            verdict: HandshakeVerdict::DegradeVersionMismatch,
            api: None,
        };
    }
    let status_sym = unsafe { sym(c"cerulion_heaphook_status") };
    // A matching version with a missing sibling symbol is a broken build of
    // the hook — treat it as the version-skew degrade (same remedy:
    // redeploy the pair), never a partial API.
    let Some(status_fn) = (!status_sym.is_null()).then(|| unsafe {
        std::mem::transmute::<*mut c_void, unsafe extern "C" fn() -> u32>(status_sym)
    }) else {
        return ResolvedHook {
            verdict: HandshakeVerdict::DegradeVersionMismatch,
            api: None,
        };
    };
    let status = unsafe { status_fn() };
    let verdict = decide_handshake(true, hook_version, HEAPHOOK_ABI_VERSION, status);
    if !verdict.is_active() {
        return ResolvedHook { verdict, api: None };
    }

    // Resolve the entries. Any hole ⇒ the same broken-build degrade.
    macro_rules! resolve {
        ($name:literal, $ty:ty) => {{
            let p = unsafe { sym($name) };
            if p.is_null() {
                return ResolvedHook {
                    verdict: HandshakeVerdict::DegradeVersionMismatch,
                    api: None,
                };
            }
            // SAFETY: non-null dlsym result for the exported symbol whose
            // signature `$ty` mirrors (pinned by `heaphook_source_parity`).
            unsafe { std::mem::transmute::<*mut c_void, $ty>(p) }
        }};
    }
    let api = HookApi {
        arm_window: resolve!(
            c"cerulion_heaphook_arm_window",
            unsafe extern "C" fn(*mut c_void, *mut c_void) -> i32
        ),
        disarm_window: resolve!(
            c"cerulion_heaphook_disarm_window",
            unsafe extern "C" fn() -> i32
        ),
        window_escape: resolve!(
            c"cerulion_heaphook_window_escape",
            unsafe extern "C" fn() -> i32
        ),
        window_range_test: resolve!(
            c"cerulion_heaphook_window_range_test",
            unsafe extern "C" fn(*const c_void, usize) -> i32
        ),
        retire_slot: resolve!(
            c"cerulion_heaphook_retire_slot",
            unsafe extern "C" fn(*mut c_void) -> i32
        ),
        counter: resolve!(
            c"cerulion_heaphook_counter",
            unsafe extern "C" fn(u32) -> u64
        ),
        register_segment: resolve!(
            c"cerulion_heaphook_register_segment",
            unsafe extern "C" fn(*mut c_void, usize, usize) -> i32
        ),
        unregister_segment: resolve!(
            c"cerulion_heaphook_unregister_segment",
            unsafe extern "C" fn(*mut c_void) -> i32
        ),
        set_release_callback: resolve!(
            c"cerulion_heaphook_set_release_callback",
            unsafe extern "C" fn(HookReleaseCallback) -> i32
        ),
    };
    ResolvedHook {
        verdict: HandshakeVerdict::Active,
        api: Some(api),
    }
}

/// Non-Linux: the hook is a Linux/GNU `LD_PRELOAD` payload and cannot exist
/// in this process — the verdict is structurally Absent (macOS test binaries
/// exercise the window paths through the `test-seams` fake hook instead).
#[cfg(not(target_os = "linux"))]
fn resolve_process_hook() -> ResolvedHook {
    ResolvedHook {
        verdict: HandshakeVerdict::DegradeAbsent,
        api: None,
    }
}

// ── Cursor recovery ────────────────────────────────────────────────────────

/// Recover the armed window's bump cursor from the monotone zero-length
/// adopt test (see the module doc): `test(p)` must answer
/// `window_range_test(p as ptr, 0) == 1`, i.e. `p >= base && p <= cursor`.
///
/// Returns `None` when `test(base)` is false — no window is armed on this
/// thread, or the thread's window is some OTHER slot's (whose disjoint
/// range can never contain `base`); either way the caller must not treat
/// any storage as adopted. Otherwise returns the cursor: the greatest
/// `p ∈ [base, tail_limit]` with `test(p)` true.
///
/// Pure over the injected predicate, so the boundary arithmetic is
/// oracle-tested without a hook; the FFI wrapper is
/// [`bisect_window_cursor`].
pub fn bisect_cursor_with(
    base: usize,
    tail_limit: usize,
    mut test: impl FnMut(usize) -> bool,
) -> Option<usize> {
    if tail_limit < base || !test(base) {
        return None;
    }
    // Invariant: test(lo) == true, test(hi) == false (hi starts one past
    // the last candidate — `tail_limit + 1` is address arithmetic only,
    // never dereferenced; the hook's range test reads addresses, not
    // memory).
    let mut lo = base;
    let mut hi = tail_limit.checked_add(1)?;
    while hi - lo > 1 {
        let mid = lo + (hi - lo) / 2;
        if test(mid) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    Some(lo)
}

/// The hook's six diagnostic counters, read through the resolved API —
/// `None` when no hook is active. Per kind, `None` means the hook answered
/// its `u64::MAX` unknown-index sentinel — an older v2 hook that predates
/// that kind (the [`COUNTER_KINDS`] sentinel rule): the kind is UNSERVED,
/// never a count. This rmw is these counters' in-process reader (the
/// hook's own docs name this dependency): a stock ROS 2 process installs no
/// `tracing` subscriber for the hook's breadcrumbs, so the rmw surfaces them
/// at shutdown.
pub fn hook_counters() -> Option<[(&'static str, Option<u64>); 6]> {
    let api = active_hook()?;
    Some(COUNTER_KINDS.map(|(kind, name)| {
        // SAFETY: a resolved hook's `counter` reads an atomic by index.
        let v = unsafe { (api.counter)(kind) };
        (name, (v != u64::MAX).then_some(v))
    }))
}

/// Renders a per-kind counter reading: the count, or `unavailable` for a
/// kind the hook does not serve (the [`COUNTER_KINDS`] sentinel rule — a
/// stale v2 hook in a mixed deployment; never rendered as a real MAX count).
struct CounterField(Option<u64>);

impl std::fmt::Display for CounterField {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.0 {
            Some(v) => write!(f, "{v}"),
            None => f.write_str("unavailable"),
        }
    }
}

/// Log the hook's diagnostic counters once (the rmw shutdown reconciliation
/// line). Silent when no hook is active; `info!` — these are Principle-#3
/// observations, not failures (each nonzero one already breadcrumbed at
/// first occurrence inside the hook). Field names are the [`COUNTER_KINDS`]
/// names, spelled constant because `tracing` fields are static; a kind the
/// hook does not serve renders `unavailable` (the sentinel rule), so a mixed
/// deployment's stale hook degrades in the log instead of reporting
/// `u64::MAX` tombstone/atfork counts as real.
pub fn log_hook_counters() {
    if let Some(counters) = hook_counters() {
        tracing::info!(
            release_without_callback = %CounterField(counters[0].1),
            bootstrap_exhausted = %CounterField(counters[1].1),
            pre_resolution_leak = %CounterField(counters[2].1),
            quarantine_noop_frees = %CounterField(counters[3].1),
            tombstone_hits = %CounterField(counters[4].1),
            atfork_interval_leaks = %CounterField(counters[5].1),
            "heap hook diagnostic counters at shutdown"
        );
    }
}

/// The FFI form of [`bisect_cursor_with`] over a resolved [`HookApi`]:
/// recovers THIS thread's window cursor for the slot armed over
/// `[base, tail_limit)`. Must run BEFORE `disarm_window` (the test answers
/// [`RC_ERR_NOT_ARMED`] after).
///
/// # Safety
/// `api` must be a hook resolved by [`active_hook`] (or a test hook whose
/// fns uphold the same contract); addresses are probed but never
/// dereferenced.
pub unsafe fn bisect_window_cursor(api: &HookApi, base: usize, tail_limit: usize) -> Option<usize> {
    bisect_cursor_with(base, tail_limit, |p| unsafe {
        (api.window_range_test)(p as *const c_void, 0) == 1
    })
}

// ── test-seams: install a fake hook (macOS-runnable window-path tests) ────

#[cfg(feature = "test-seams")]
mod seam {
    use super::HookApi;
    use std::sync::RwLock;

    static TEST_HOOK: RwLock<Option<HookApi>> = RwLock::new(None);

    pub(super) fn test_hook() -> Option<HookApi> {
        *TEST_HOOK.read().unwrap_or_else(|e| e.into_inner())
    }

    /// RAII installation of a fake [`HookApi`] — consulted by
    /// [`super::active_hook`] BEFORE the process dlsym resolution, so a
    /// serial test binary can drive the window arms on a platform where the
    /// real hook cannot exist (macOS). Dropped (normally or by
    /// unwind) ⇒ uninstalled. The install is process-global: hold it only
    /// in `#[serial]` tests.
    #[must_use = "the fake hook is installed only while the guard is held"]
    pub struct TestHookGuard(());

    impl TestHookGuard {
        /// Install `api` for the lifetime of the returned guard.
        pub fn install(api: HookApi) -> Self {
            *TEST_HOOK.write().unwrap_or_else(|e| e.into_inner()) = Some(api);
            Self(())
        }
    }

    impl Drop for TestHookGuard {
        fn drop(&mut self) {
            *TEST_HOOK.write().unwrap_or_else(|e| e.into_inner()) = None;
        }
    }
}

#[cfg(feature = "test-seams")]
use seam::test_hook;
#[cfg(feature = "test-seams")]
pub use seam::TestHookGuard;

#[cfg(test)]
mod tests {
    use super::*;

    // ── decide_handshake: the pure twin must agree with the hook's own
    //    decision table (mirrored oracle vectors, same order-of-checks) ────

    #[test]
    fn an_absent_symbol_degrades_regardless_of_the_other_fields() {
        assert_eq!(
            decide_handshake(
                false,
                HEAPHOOK_ABI_VERSION,
                HEAPHOOK_ABI_VERSION,
                HEAPHOOK_STATUS_WON_MALLOC
            ),
            HandshakeVerdict::DegradeAbsent
        );
    }

    #[test]
    fn a_version_mismatch_degrades_before_status_is_trusted() {
        assert_eq!(
            decide_handshake(true, 3, 2, HEAPHOOK_STATUS_WON_MALLOC),
            HandshakeVerdict::DegradeVersionMismatch
        );
        assert_eq!(
            decide_handshake(true, 1, 2, HEAPHOOK_STATUS_WON_MALLOC),
            HandshakeVerdict::DegradeVersionMismatch
        );
    }

    #[test]
    fn a_matching_hook_that_did_not_win_malloc_degrades_foreign() {
        assert_eq!(
            decide_handshake(true, 2, 2, 0),
            HandshakeVerdict::DegradeForeignAllocator
        );
    }

    #[test]
    fn a_present_matching_won_malloc_hook_is_active() {
        let v = decide_handshake(true, 2, 2, HEAPHOOK_STATUS_WON_MALLOC);
        assert_eq!(v, HandshakeVerdict::Active);
        assert!(v.is_active());
    }

    #[test]
    fn extra_status_bits_do_not_disturb_the_won_malloc_test() {
        assert_eq!(
            decide_handshake(true, 2, 2, HEAPHOOK_STATUS_WON_MALLOC | 0b1010),
            HandshakeVerdict::Active
        );
    }

    #[test]
    fn only_active_reports_is_active() {
        assert!(!HandshakeVerdict::DegradeAbsent.is_active());
        assert!(!HandshakeVerdict::DegradeVersionMismatch.is_active());
        assert!(!HandshakeVerdict::DegradeForeignAllocator.is_active());
    }

    // ── bisect_cursor_with: boundary oracles (the seal's copy placement
    //    keys on this — an off-by-one under-read corrupts a live object,
    //    an over-read wrongly refuses adoption) ──────────────────────────

    /// The predicate a REAL armed window answers: `p >= base && p <= cursor`.
    fn window(base: usize, cursor: usize) -> impl FnMut(usize) -> bool {
        move |p| p >= base && p <= cursor
    }

    #[test]
    fn cursor_at_base_zero_bumps_is_recovered() {
        assert_eq!(
            bisect_cursor_with(1000, 2000, window(1000, 1000)),
            Some(1000)
        );
    }

    #[test]
    fn cursor_mid_tail_is_recovered_exactly() {
        for cursor in [1001, 1024, 1500, 1999] {
            assert_eq!(
                bisect_cursor_with(1000, 2000, window(1000, cursor)),
                Some(cursor),
                "cursor {cursor}"
            );
        }
    }

    #[test]
    fn cursor_at_tail_limit_is_recovered() {
        // A window bumped to exactly the tail limit: cursor == tail_limit.
        assert_eq!(
            bisect_cursor_with(1000, 2000, window(1000, 2000)),
            Some(2000)
        );
    }

    #[test]
    fn no_window_or_a_foreign_window_yields_none() {
        // No window at all.
        assert_eq!(bisect_cursor_with(1000, 2000, |_| false), None);
        // Another slot's window (disjoint range below ours): base tests false.
        assert_eq!(bisect_cursor_with(1000, 2000, window(100, 900)), None);
    }

    #[test]
    fn degenerate_bounds_yield_none_or_base() {
        // tail_limit below base is refused.
        assert_eq!(bisect_cursor_with(1000, 999, window(1000, 1000)), None);
        // A zero-capacity window (tail_limit == base) recovers cursor == base.
        assert_eq!(
            bisect_cursor_with(1000, 1000, window(1000, 1000)),
            Some(1000)
        );
    }

    #[test]
    fn probe_count_is_logarithmic() {
        // The publish path runs this per frame — pin that a 32 MiB tail
        // costs ~25 probes, not a scan.
        let mut probes = 0usize;
        let base = 1usize << 30;
        let tail = base + (32 << 20);
        let cursor = base + 12_345_678;
        let got = bisect_cursor_with(base, tail, |p| {
            probes += 1;
            p >= base && p <= cursor
        });
        assert_eq!(got, Some(cursor));
        assert!(probes <= 32, "expected O(log) probes, got {probes}");
    }

    // ── Source parity: the mirrored scalars must match the hook crate's
    //    own source (the rmw deliberately does not LINK it — see the module
    //    doc — so the pin reads the workspace source instead) ─────────────

    fn hook_src(file: &str) -> String {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../cerulion_heaphook/src")
            .join(file);
        std::fs::read_to_string(&path).unwrap_or_else(|e| {
            panic!(
                "heaphook source parity pin could not read {}: {e} — if the hook \
                 crate moved, update this consumer's mirror AND this pin together",
                path.display()
            )
        })
    }

    #[test]
    fn heaphook_source_parity_version_status_and_return_codes() {
        let abi = hook_src("abi.rs");
        for (needle, what) in [
            (
                format!("pub const HEAPHOOK_ABI_VERSION: u32 = {HEAPHOOK_ABI_VERSION};"),
                "ABI version",
            ),
            (
                "pub const HEAPHOOK_STATUS_WON_MALLOC: u32 = 1 << 0;".to_string(),
                "won-malloc status bit",
            ),
            (format!("pub const RC_OK: i32 = {RC_OK};"), "RC_OK"),
            (
                format!("pub const RC_ERR_ALREADY_ARMED: i32 = {RC_ERR_ALREADY_ARMED};"),
                "RC_ERR_ALREADY_ARMED",
            ),
            (
                format!("pub const RC_ERR_NOT_ARMED: i32 = {RC_ERR_NOT_ARMED};"),
                "RC_ERR_NOT_ARMED",
            ),
            (
                format!("pub const RC_ERR_BAD_ARG: i32 = {RC_ERR_BAD_ARG};"),
                "RC_ERR_BAD_ARG",
            ),
            (
                format!("pub const RC_ERR_OVERLAP: i32 = {RC_ERR_OVERLAP};"),
                "RC_ERR_OVERLAP",
            ),
            (
                format!("pub const RC_ERR_UNKNOWN: i32 = {RC_ERR_UNKNOWN};"),
                "RC_ERR_UNKNOWN",
            ),
            (
                format!("pub const RC_ERR_UNAVAILABLE: i32 = {RC_ERR_UNAVAILABLE};"),
                "RC_ERR_UNAVAILABLE",
            ),
        ] {
            assert!(
                abi.contains(&needle),
                "hook abi.rs no longer carries `{needle}` ({what}) — the consumer \
                 mirror in rmw_cerulion/src/heaphook.rs has drifted from \
                 cerulion_heaphook/src/abi.rs; update BOTH together (a runtime skew \
                 is exactly what the version gate exists to refuse)"
            );
        }
    }

    #[test]
    fn heaphook_source_parity_standalone_symbols() {
        let exports = hook_src("exports.rs");
        for symbol in [
            "cerulion_heaphook_version",
            "cerulion_heaphook_status",
            "cerulion_heaphook_arm_window",
            "cerulion_heaphook_disarm_window",
            "cerulion_heaphook_window_escape",
            "cerulion_heaphook_window_range_test",
            "cerulion_heaphook_retire_slot",
            "cerulion_heaphook_counter",
            "cerulion_heaphook_register_segment",
            "cerulion_heaphook_unregister_segment",
            "cerulion_heaphook_set_release_callback",
        ] {
            assert!(
                exports.contains(&format!("pub unsafe extern \"C\" fn {symbol}(")),
                "hook exports.rs no longer defines `{symbol}` as a standalone export — \
                 the consumer dlsym set in rmw_cerulion/src/heaphook.rs has drifted; \
                 update BOTH together"
            );
        }
    }

    #[test]
    fn counter_kinds_mirror_the_hooks_counter_match() {
        // The hook's `state::counter` documents kinds 0..=5 in this order;
        // pin the mirrored table's indices are dense and start at 0 so a
        // future kind is added deliberately on both sides.
        for (i, (kind, name)) in COUNTER_KINDS.iter().enumerate() {
            assert_eq!(*kind as usize, i, "counter kinds must be dense from 0");
            assert!(!name.is_empty());
        }
        let state = hook_src("state.rs");
        for arm in [
            "3 => &QUARANTINE_NOOP_FREES",
            "4 => &TOMBSTONE_HITS",
            "5 => &ATFORK_INTERVAL_LEAKS",
        ] {
            assert!(
                state.contains(arm),
                "hook state.rs counter table changed (`{arm}` missing) — \
                 update COUNTER_KINDS with it"
            );
        }
    }
}

// SPDX-License-Identifier: AGPL-3.0-only
//! rmw init / shutdown / context / identifier surface.

use std::os::raw::c_char;

use crate::ffi::{self, rmw_ret_t, RMW_RET_ERROR, RMW_RET_INVALID_ARGUMENT, RMW_RET_OK};
use crate::runtime;

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_get_implementation_identifier() -> *const c_char {
    ffi::implementation_identifier_ptr()
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_get_serialization_format() -> *const c_char {
    ffi::SERIALIZATION_FORMAT.as_ptr() as *const c_char
}

/// The load-time era guard,
/// HOISTED to the first rmw entry points that touch caller memory.
///
/// rmw.h's documented order is `rmw_init_options_init` → (rcl may
/// `rmw_init_options_copy`) → `rmw_init` → … → `rmw_init_options_fini`.
/// With the guard only in `rmw_init`, this library would ALREADY have written
/// `allocator`/`impl_` at its BUILT-FOR offsets into the caller's
/// `rmw_init_options_t` (jazzy: 120/160) before refusing — a struct a
/// Kilted runtime lays out at 112/152 in 160 bytes, so the 8-byte
/// `impl_` write lands PAST the caller's object; `copy` memcpys
/// this build's size over the caller's smaller `dst`; `fini` zeroes
/// it. So every one of those entry points asks this first.
///
/// The decision reads NOTHING from the caller's struct: it is the baked
/// claim ([`crate::era::baked_distro`]) against the runtime `ROS_DISTRO`
/// — exactly the evidence `rmw_init` uses. For the record, the
/// fields that ARE layout-invariant across every candidate layout
/// (pre-Iron 104 B / iron-jazzy 168 B / kilted-and-later 160 B — the
/// offsets are pinned in `ffi/era_pins.rs`) form the common prefix
/// `instance_id`@0, `implementation_identifier`@8, `domain_id`@16 and
/// `security_options`@24..40; everything from offset 40 on differs. The
/// guard runs BEFORE even those are read, so a refusal touches zero
/// caller bytes. `rmw_init` keeps its own call as defense in depth: a
/// caller can skip `rmw_init_options_init` with a static struct.
///
/// `Err(RMW_RET_ERROR)` after the loud diagnostic on a mismatch; `Ok(())`
/// means proceed. On the mismatch path tracing is installed before the
/// refusal is emitted so it is visible — `runtime()`, the usual
/// installer, is never reached there.
///
/// Runs INSIDE each entry point's `ffi::ffi_guard`:
/// this helper allocates (the normalized claim
/// strings) and installs tracing, so a panic here must degrade to the
/// entry point's failure code — never unwind through the C ABI, which
/// aborts the whole host process. The `test-seams` panic seam at the top
/// is how that containment is proven without a real allocator failure.
fn refuse_on_era_mismatch(entry: crate::era::GuardedEntry) -> Result<(), rmw_ret_t> {
    #[cfg(feature = "test-seams")]
    crate::test_seams::maybe_panic_inside_era_guard();
    let Some((baked, requested)) = crate::era::runtime_distro_mismatch(crate::era::baked_distro())
    else {
        return Ok(());
    };
    // ALLOCATION CADENCE: the `format!` below and the two `String`s
    // `classify_distro_pair` returns are per-CALL, and that is fine here
    // — these four exports are called per rmw CONTEXT at process
    // start-up, never per message. The sibling C++ bridge gate sits on
    // the per-MESSAGE resolve path and therefore caches its rcl text; the
    // difference is the cadence, not the taste.
    //
    // The refusal must also reach rcl's error channel — rcl reports
    // `rmw_get_error_string()` to rclpy/rclcpp on the failed return, and
    // without this the operator's first line reads "error not set"
    // while the real reason sits on a stderr the host may never show.
    // That string carries the entry point and both distros too (rcl has
    // no structured fields); the tracing line then records whether rcl
    // got it. Best-effort: outside a ROS process there is no librcutils.
    // `requested` is env-derived: rendered with control
    // characters escaped, the same rule as the tracing line below.
    let rcl_channel = ffi::rcutils_set_error_state_best_effort(&format!(
        "{} entry={} baked_ros_distro={baked} runtime_ros_distro={}",
        crate::era::DISTRO_MISMATCH_REFUSAL,
        entry.name(),
        crate::era_check::escape_control_chars(&requested)
    ));
    runtime::install_tracing();
    crate::era::emit_distro_refusal(entry, &baked, &requested, rcl_channel);
    Err(RMW_RET_ERROR)
}

/// # Safety
/// `options` must be a valid pointer to a zero-initialized
/// `rmw_init_options_t`.
#[no_mangle]
pub unsafe extern "C" fn rmw_init_options_init(
    options: *mut ffi::rmw_init_options_t,
    allocator: ffi::rcutils_allocator_t,
) -> rmw_ret_t {
    // The whole body — the era guard included, which allocates
    // and installs tracing — runs under ffi_guard so a panic degrades to
    // RMW_RET_ERROR instead of unwinding through the C ABI.
    ffi::ffi_guard(RMW_RET_ERROR, || unsafe {
        if options.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        // Before ANY read or write of `*options` — its layout is the
        // runtime distro's, and only the guard's verdict says it is
        // also ours.
        if let Err(refused) = refuse_on_era_mismatch(crate::era::GuardedEntry::InitOptionsInit) {
            return refused;
        }
        let opts = &mut *options;
        if !opts.implementation_identifier.is_null() {
            // Already initialized.
            return RMW_RET_INVALID_ARGUMENT;
        }
        opts.implementation_identifier = ffi::implementation_identifier_ptr();
        opts.allocator = allocator;
        opts.instance_id = 0;
        opts.impl_ = std::ptr::null_mut();
        RMW_RET_OK
    })
}

/// # Safety
/// `src`/`dst` must be valid pointers; `dst` zero-initialized.
#[no_mangle]
pub unsafe extern "C" fn rmw_init_options_copy(
    src: *const ffi::rmw_init_options_t,
    dst: *mut ffi::rmw_init_options_t,
) -> rmw_ret_t {
    ffi::ffi_guard(RMW_RET_ERROR, || unsafe {
        if src.is_null() || dst.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        // Before the identifier reads and the memcpy: the copy is
        // size_of::<rmw_init_options_t>() of THIS build, which over a
        // smaller runtime layout writes past the caller's `dst`.
        if let Err(refused) = refuse_on_era_mismatch(crate::era::GuardedEntry::InitOptionsCopy) {
            return refused;
        }
        if !ffi::is_our_identifier((*src).implementation_identifier) {
            return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
        }
        if !(*dst).implementation_identifier.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        // Shallow copy is sufficient: we keep no owned pointers in
        // options (enclave/security copied by rcl above us; impl_
        // unused).
        std::ptr::copy_nonoverlapping(src, dst, 1);
        RMW_RET_OK
    })
}

/// # Safety
/// `options` must be a valid, initialized options pointer.
#[no_mangle]
pub unsafe extern "C" fn rmw_init_options_fini(options: *mut ffi::rmw_init_options_t) -> rmw_ret_t {
    ffi::ffi_guard(RMW_RET_ERROR, || unsafe {
        if options.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        // Before the identifier read and the zeroing (this build's size).
        if let Err(refused) = refuse_on_era_mismatch(crate::era::GuardedEntry::InitOptionsFini) {
            return refused;
        }
        if !ffi::is_our_identifier((*options).implementation_identifier) {
            return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
        }
        std::ptr::write_bytes(options, 0, 1);
        RMW_RET_OK
    })
}

/// # Safety
/// `options` valid + initialized; `context` valid + zero-initialized.
#[no_mangle]
pub unsafe extern "C" fn rmw_init(
    options: *const ffi::rmw_init_options_t,
    context: *mut ffi::rmw_context_t,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if options.is_null() || context.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        // ORDER CONTRACT: null check → load-time era
        // guard → first touch of caller memory. The era guard lands HERE —
        // immediately after the null check, BEFORE the `*options`
        // identifier read below and the `*context` read after it — so that
        // a distro mismatch paired with a foreign identifier is refused as a
        // mismatch rather than surfacing as an identifier error.
        //
        // (The guard itself: a librmw_cerulion.so built for one distro and
        // dlopen'd into another SUCCEEDS through rmw_init and dies at the
        // FIRST TYPED OPERATION — on Foxy a SIGSEGV in strlen under
        // rmw_create_publisher. The FIRST line of defense is in
        // rmw_init_options_init / _copy / _fini; this call is DEFENSE IN
        // DEPTH for a caller that skipped options_init with a static struct.)
        if let Err(refused) = refuse_on_era_mismatch(crate::era::GuardedEntry::Init) {
            return refused;
        }
        if !ffi::is_our_identifier((*options).implementation_identifier) {
            return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
        }
        // rmw.h: "The given context must be zero initialized ... If context
        // has been already initialized (`rmw_init()` was called on it), then
        // `RMW_RET_INVALID_ARGUMENT` is returned." Checked before anything
        // is touched — the identity-keyed lifecycle below assumes one init
        // per context.
        if !(*context).implementation_identifier.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        match runtime::runtime() {
            Ok(_) => {
                // One more live context, keyed by the
                // context's IDENTITY under the adopt-take lifecycle LOCK
                // (the live set decides which shutdown may clear the
                // process-global release callback, and the transition must
                // serialize with a concurrent final shutdown's clear — see
                // `adopt_take::LIFECYCLE`). Inserted BEFORE the context is
                // written, so a context somehow already live (a caller that
                // re-zeroed a live struct by hand) is refused per rmw.h with
                // the struct untouched.
                if !crate::adopt_take::context_started(context) {
                    return RMW_RET_INVALID_ARGUMENT;
                }
                let ctx = &mut *context;
                ctx.instance_id = (*options).instance_id;
                ctx.implementation_identifier = ffi::implementation_identifier_ptr();
                ctx.actual_domain_id = 0;
                ctx.impl_ = std::ptr::null_mut();
                // Everything below is
                // fallible in a way this module does not control — the
                // `warn!`/`info!` go to the HOST's subscriber, and
                // `install_tracing` deliberately lets the host's win, so a
                // foreign `on_event` that panics unwinds from here. Without
                // this catch, `ffi_guard` would report `RMW_RET_ERROR` while
                // the live-set entry and the written context REMAIN: the
                // caller has been told the init failed, so it will never
                // call `rmw_shutdown`, and the entry it would have removed
                // is the one that decides whether the last shutdown may
                // clear the process-global release callback. A failed init
                // must leave no lifecycle state.
                let announced = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    // The guard ADMITTED this build. If it is the unclaimed
                    // vendored snapshot, that admission rests on layout
                    // identity alone (ROS_DISTRO named lyrical/rolling — any
                    // other name was refused above) and nothing verified it
                    // against a real install of that distro: announce it,
                    // loudly. Dev convenience, never a deployment posture.
                    // The same reader the guard uses (`var_os`, lossy): one
                    // variable, one way of reading it.
                    let runtime_distro =
                        std::env::var_os("ROS_DISTRO").map(|v| v.to_string_lossy().into_owned());
                    if crate::era::warns_unclaimed_bindings(
                        crate::era::baked_distro(),
                        runtime_distro.as_deref(),
                    ) {
                        tracing::warn!(
                            built_for = %crate::era::built_for(),
                            // Env-derived: control characters escaped.
                            runtime_ros_distro = %crate::era_check::escape_control_chars(
                                runtime_distro.as_deref().unwrap_or("<absent>")
                            ),
                            "rmw_cerulion: this .so carries UNCLAIMED bindings — the vendored \
                             rolling-era snapshot — admitted under this ROS distro only because \
                             it shares the snapshot's layout, never verified against a real \
                             install of it: a dev convenience, NEVER a deployment posture. A \
                             deployed .so must be built inside its target distro (bindgen \
                             against installed headers, ROS_DISTRO set)."
                        );
                    }
                    tracing::info!(
                        built_for = %crate::era::built_for(),
                        "rmw_cerulion initialized"
                    );
                    // The stand-in for a host subscriber that panics here.
                    #[cfg(feature = "test-seams")]
                    crate::test_seams::maybe_panic_in_init_announce();
                }));
                if let Err(payload) = announced {
                    // Undo exactly what this init added, in reverse order:
                    // the caller's struct goes back to the zeroed shape the
                    // entry check above requires, then the live-set entry
                    // goes. `ffi_guard` still converts the panic into
                    // `RMW_RET_ERROR` and reports it.
                    *context = std::mem::zeroed();
                    crate::adopt_take::context_start_rolled_back(context);
                    std::panic::resume_unwind(payload);
                }
                RMW_RET_OK
            }
            Err(e) => {
                tracing::error!(error = %e, "rmw_init failed");
                RMW_RET_ERROR
            }
        }
    })
}

/// # Safety
/// `context` must be a valid, initialized context.
#[no_mangle]
pub unsafe extern "C" fn rmw_shutdown(context: *mut ffi::rmw_context_t) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if context.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        if !ffi::is_our_identifier((*context).implementation_identifier) {
            return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
        }
        // Adopt-take: this context is done, and its
        // removal from the identity-keyed live set, the heap hook's
        // diagnostic counters (read here by their in-process reader, the hook's
        // documented dependency: a leaked release range or an unretired
        // quarantine backlog must be visible at teardown, not only as the
        // hook's one-shot debug-gated breadcrumbs), the final-context
        // decision, the adopt-take proof line and (final context only, with
        // samples outstanding) the destructive clear-and-leak arm all run
        // under the ONE adopt-take lifecycle lock, so an intermediate
        // shutdown can never clear the callback a surviving context's frees
        // still need, and a concurrent init/grant-install can never
        // interleave the decision and the clear.
        //
        // A REPEAT shutdown of an already-shut-down context is the rmw.h
        // no-op ("this function is a no-op and `RMW_RET_OK` is returned"):
        // the context is absent from the live set, so NOTHING above runs —
        // in particular no second decrement and no clear of the callback a
        // still-live context's frees need (an unkeyed
        // count would let that repeat drive the count to zero under a live
        // context).
        if !crate::adopt_take::context_ended(context) {
            tracing::debug!("rmw_shutdown on an already-shut-down context — the rmw.h no-op");
            return RMW_RET_OK;
        }
        tracing::info!("rmw_cerulion shutdown");
        RMW_RET_OK
    })
}

/// # Safety
/// `context` must be a valid, shutdown context.
#[no_mangle]
pub unsafe extern "C" fn rmw_context_fini(context: *mut ffi::rmw_context_t) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if context.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        if !ffi::is_our_identifier((*context).implementation_identifier) {
            return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
        }
        // rmw.h: "If context is initialized and valid (`rmw_shutdown()` was
        // not called on it), then `RMW_RET_INVALID_ARGUMENT` is returned"
        // — and a logical error leaves the context UNCHANGED. Zeroing a
        // still-live context would also orphan its entry in the
        // identity-keyed live set, so this check is what keeps that set
        // accurate.
        if crate::adopt_take::context_is_live(context) {
            tracing::warn!(
                "rmw_context_fini on a context that was never shut down — refused per \
                 rmw.h, context left untouched (call rmw_shutdown first)"
            );
            return RMW_RET_INVALID_ARGUMENT;
        }
        std::ptr::write_bytes(context, 0, 1);
        RMW_RET_OK
    })
}

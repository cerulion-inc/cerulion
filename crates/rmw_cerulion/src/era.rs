// SPDX-License-Identifier: AGPL-3.0-only
//! The baked build-era identity and the load-time distro
//! guard.
//!
//! A `librmw_cerulion.so` built for one ROS distro dlopen'd into another
//! SUCCEEDS through `rmw_init` and dies at the FIRST TYPED OPERATION —
//! measured on a Foxy robot: a Jazzy-shaped
//! introspection `MessageMember` stride walked over Foxy's smaller
//! struct SIGSEGV'd in `strlen` under `rmw_create_publisher` (the rosout
//! publisher inside `rcl_node_init`). Struct layouts (introspection
//! members, `rmw_init_options_t`, GID width) are per-distro ABI.
//!
//! This module turns that SIGSEGV class into a one-line refusal:
//! build.rs bakes a SELECTION-DERIVED distro claim plus the capability
//! fingerprint into the `.so` ([`built_for`], logged in the `rmw_init`
//! banner), and `rmw_init` refuses — loudly, naming both sides — when
//! the baked distro and the runtime `ROS_DISTRO` disagree. The claim
//! derives from the bindings the crate actually compiled against, never
//! from the raw build env: a distro NAME is baked only for bindings
//! GENERATED against installed headers whose capability fingerprint is
//! consistent with that distro's era (verified by the shared
//! `era_check` classifier; a contradiction fails the BUILD), while every
//! VENDORED build bakes the unclaimed [`VENDORED_DEV_DISTRO`] marker
//! regardless of what `ROS_DISTRO` said at build time (an
//! env-derived claim would let a headerless `ROS_DISTRO=foxy`
//! build label the pinned ROLLING vendored ABI "foxy", and the guard
//! would then VOUCH for exactly the skew it exists to refuse). A runtime
//! with no `ROS_DISTRO` passes — the guard refuses only a POSITIVE
//! contradiction — and the vendored `.so`'s unclaimed marker is NOT an
//! absence: it names the pinned ROLLING-era snapshot, so under a named
//! runtime it is admitted only for that era's layout-identical members
//! (`lyrical`/`rolling`) and refused for every other name. Where it IS
//! admitted (its own era, unverified against a real install) the
//! `rmw_init` banner announces it loudly ([`warns_unclaimed_bindings`]):
//! a dev convenience, never a deployment posture. Every guarded init
//! export (`rmw_init_options_init`/`_copy`/`_fini`, then `rmw_init` as
//! defense in depth) refuses before touching caller memory.
//!
//! Guard ladder context: compile time = bindgen layout
//! tests + `src/ffi/era_pins.rs`; load time = THIS guard; runtime = the
//! registration-time plausibility check in `type_bridge_cpp`.

use cerulion_core::transport::failure_regime_latch::{
    lock_regime_latch, FailureRegimeLatch, RegimeDecision,
};
use std::sync::Mutex;

/// The distro this `.so` was built for — SELECTION-derived by build.rs:
/// a distro name ONLY when the bindings were GENERATED against that
/// distro's installed headers; [`VENDORED_DEV_DISTRO`] for every
/// vendored build (regardless of the build env's `ROS_DISTRO`); a
/// fingerprint-derived `era:<token>` claim for a generated build whose
/// environment named no distro — or a name the era table does not know
/// (compared by era MEMBERSHIP at runtime; only `era:lyrical` is
/// ever baked, the one multi-member layout-identical era — never the
/// unclaimed marker: one distro's real ABI must not run everywhere).
pub const BUILT_FOR_DISTRO: &str = env!("CERULION_RMW_BUILT_FOR_DISTRO");

/// Which bindings the crate compiled against: `"generated"` (bindgen
/// over installed headers) or `"vendored"` (the committed rolling
/// snapshot).
pub const BINDINGS_SOURCE: &str = env!("CERULION_RMW_BINDINGS_SOURCE");

/// Comma-joined capability fingerprint (`cerulion_has_*` suffixes)
/// build.rs derived from the selected bindings; `"none"` when empty.
pub const CAPABILITY_FINGERPRINT: &str = env!("CERULION_RMW_CAPS");

/// The UNCLAIMED marker — the vendored snapshot's claim spelling: at
/// runtime it is admitted only under that snapshot's own era
/// (`lyrical`/`rolling`) or an absent `ROS_DISTRO`, and refused under
/// every other name. Baked ONLY for vendored-bindings builds (even
/// when the build env set `ROS_DISTRO`: the snapshot is pinned ROLLING,
/// so honoring the env would label a rolling ABI with a real distro
/// name); a generated build ALWAYS carries a claim — exact, or
/// fingerprint-derived `era:<token>`.
/// RESERVED: a generated build whose env EXPLICITLY
/// normalizes to this marker fails at build
/// (`era_check::DistroClaimVerdict::ReservedCollision`) — baking it
/// would mark a distro-specific ABI unclaimed and bypass this guard.
pub const VENDORED_DEV_DISTRO: &str = crate::era_check::RESERVED_UNCLAIMED_MARKER;

/// One-line build identity for banners and diagnostics, e.g.
/// `distro=jazzy bindings=generated caps=fetch_function,is_key,…`.
pub fn built_for() -> &'static str {
    concat!(
        "distro=",
        env!("CERULION_RMW_BUILT_FOR_DISTRO"),
        " bindings=",
        env!("CERULION_RMW_BINDINGS_SOURCE"),
        " caps=",
        env!("CERULION_RMW_CAPS"),
    )
}

/// Verdict of [`classify_cpp_bridge`] — whether THIS build may resolve
/// the C++ introspection typesupport (the rclcpp path).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CppBridgeGate {
    /// The build's era matches the hand-mirrored C++ layout — resolve
    /// the C++ arm as always.
    Supported,
    /// A pre-Jazzy build: the C++ typesupport arm REFUSES at
    /// registration with [`CPP_BRIDGE_PRE_JAZZY_REFUSAL`] instead of
    /// misreading every rclcpp publisher's introspection structs.
    RefusePreJazzy,
}

impl CppBridgeGate {
    /// The registration-time refusal TEXT this verdict carries, if any
    /// — the paragraph [`Self::emit_refusal`] renders. Kept as a pure
    /// accessor so the oracle tests can pin the text per verdict, and
    /// so the traced binary can cross-check that the EMITTED line
    /// carries exactly this text (the two cannot drift apart).
    pub fn refusal_message(self) -> Option<&'static str> {
        match self {
            CppBridgeGate::Supported => None,
            CppBridgeGate::RefusePreJazzy => Some(CPP_BRIDGE_PRE_JAZZY_REFUSAL),
        }
    }

    /// Emit this verdict's refusal — the ONE seam both resolvers call
    /// before abandoning the C++ arm. Returns `true` iff the verdict
    /// REFUSES (the caller's cue to return `None`); `Supported` logs
    /// nothing and returns `false`.
    ///
    /// One log line per refusing resolve, each with a CONSTANT message:
    /// the `{SCREAMING_CONST}` capture is the shape the repo's
    /// tracing-discipline walk accepts, whereas an
    /// `error!("{}", refusal)` is a positional pass-through it refuses.
    /// The operator still reads the whole
    /// remedy paragraph — it IS the message — and the verdict rides a
    /// structured `verdict=` field a log consumer can key on.
    ///
    /// The refusal ALSO reaches rcl's error channel, exactly as the
    /// distro guard's does:
    /// every caller of `resolve_introspection` maps `None` onto
    /// `RMW_RET_INVALID_ARGUMENT`, so without this an rclcpp/MoveIt user
    /// reads "the caller passed a bad typesupport" plus whatever rosidl's
    /// failed probe left in the channel — or "error not set". The
    /// outcome rides `rcl_error_channel=` on the same line, exactly as
    /// `refuse_on_era_mismatch` records it.
    ///
    /// FLOOD-LATCHED through the repo's ONE shared machine: this
    /// seam is NOT on a "registration cadence" — `rmw_serialize` and
    /// `rmw_deserialize` resolve the
    /// typesupport per MESSAGE, so an unlatched refusing build republishing
    /// serialized messages would emit this ~500-byte paragraph at frame
    /// rate (the disk-fill class). Loud head,
    /// `debug!` repeats, a loud re-announcement at each DECADE of the
    /// running total, and the unconditional
    /// [`cpp_bridge_refusals_fired`] counter — so a refusing build can
    /// never go silent, and never floods either. One latch: which
    /// refusing verdict a build carries is a compile-time constant, so no
    /// SHIPPED build can interleave the two (the tests drive both, and
    /// re-arm between arms).
    /// This verdict's rcl error text, rendered ONCE for the process.
    ///
    /// Only a REFUSING verdict has one; `Supported` never reaches here
    /// (its caller returns before asking). One cell per refusing verdict
    /// rather than one shared cell, so a test that drives both — the
    /// only place both are reachable in one process, since a shipped
    /// build's verdict is a compile-time constant — cannot serve the
    /// second verdict the first one's text.
    fn rcl_error_text(self) -> &'static std::ffi::CStr {
        static PRE_JAZZY: std::sync::OnceLock<std::ffi::CString> = std::sync::OnceLock::new();
        // `Supported` is unreachable (see above); the one refusing verdict
        // owns the one cell, which keeps this total without an `unwrap`.
        let cell = &PRE_JAZZY;
        cell.get_or_init(|| {
            let paragraph = self.refusal_message().unwrap_or("");
            // Through the SAME renderer the `&str` entry point uses, so
            // a NUL can never make the cached text unrenderable.
            crate::ffi::rcl_error_text(&format!(
                "{paragraph} built_for={} verdict={self:?}",
                built_for()
            ))
            .unwrap_or_else(|| c"rmw_cerulion: C++ bridge refusal text unrenderable".to_owned())
        })
    }

    #[must_use = "a refusing verdict means the caller must abandon the C++ arm"]
    pub fn emit_refusal(self) -> bool {
        // `Supported` carries no paragraph and refuses nothing.
        if self.refusal_message().is_none() {
            return false;
        }
        CPP_BRIDGE_REFUSALS_FIRED.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        // The rcl text is built ONCE per verdict, never per call:
        // this seam sits on the
        // per-MESSAGE resolve path, and a `format!` here would make a refusing
        // build allocate on every publish and every deserialize —
        // against Principle #1 and unbounded in a real-time loop. Every
        // input is a build-time constant (the paragraph, `built_for()`,
        // the verdict), so a `OnceLock<CString>` renders it on the FIRST
        // refusal and hands rcl the same bytes forever after. The
        // per-call CONTRACT is kept: rcl's thread-local error state
        // is set on EVERY refusal, including a suppressed repeat —
        // it is state the caller reads on the failed return, so
        // suppressing it would change the rmw API's behaviour rather
        // than only the log volume.
        let rcl = crate::ffi::rcutils_set_error_state_cstr(self.rcl_error_text());
        // The guard drops at the end of this statement, so the latch is
        // never held across a `tracing` macro (nor across the rcl call
        // above, which is deliberately made on EVERY refusal — including
        // a suppressed repeat: the channel is per-call state the caller
        // reads immediately on the failed return, so suppressing it would
        // change the rmw API's behaviour, not just its log volume).
        let decision = lock_regime_latch(&CPP_BRIDGE_REFUSAL_LATCH).on_failure();
        match self {
            // Unreachable: `refusal_message()` returned above.
            CppBridgeGate::Supported => {}
            CppBridgeGate::RefusePreJazzy => match decision {
                RegimeDecision::Loud => tracing::error!(
                    built_for = %built_for(),
                    verdict = ?self,
                    rcl_error_channel = %rcl,
                    "{CPP_BRIDGE_PRE_JAZZY_REFUSAL}"
                ),
                RegimeDecision::StillFailing { total, suppressed } => tracing::error!(
                    built_for = %built_for(),
                    verdict = ?self,
                    rcl_error_channel = %rcl,
                    total_failures = total,
                    suppressed_count = suppressed,
                    "{CPP_BRIDGE_PRE_JAZZY_REFUSAL}"
                ),
                RegimeDecision::Suppressed { suppressed } => tracing::debug!(
                    built_for = %built_for(),
                    verdict = ?self,
                    rcl_error_channel = %rcl,
                    suppressed_count = suppressed,
                    "{CPP_BRIDGE_PRE_JAZZY_REFUSAL}"
                ),
            },
        }
        true
    }
}

/// How (whether) the C++ bridge bypass applies to THIS build.
/// `test-seams` is a PUBLIC cargo feature, so `cargo build -p
/// rmw_cerulion --features test-seams` produces a DEPLOYABLE vendored
/// cdylib, and a feature-only bypass would make the "test-surface-only"
/// claim false at the build boundary. The composite: `cfg(test)`
/// proper is silently bypassed (the lib's own unit tests — not
/// reachable by any shipped artifact); a `test-seams` build without
/// `cfg(test)` (the resolver-routed e2e binaries, but ALSO a
/// hand-built `--features test-seams` cdylib) is bypassed LOUDLY — a
/// flood-latched [`CPP_BYPASS_SEAMS_WARN`] — loud on the FIRST resolve
/// and at each decade of the running total, `debug!` in between, the
/// counter unconditional — so production silence is impossible;
/// everything else takes the normal two-edged gate.
/// `test-seams` is in no default feature set
/// and no shipping recipe (docker/, docs/, CI, scripts) enables it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CppBypassMode {
    /// `cfg(test)`: the lib's own unit tests — bypass, silently.
    TestSilent,
    /// `test-seams` without `cfg(test)`: e2e test binaries (the
    /// intended surface) or a hand-built seams cdylib (misuse) —
    /// bypass, LOUDLY: a warn on the first resolve and at each decade
    /// of the running total, `debug!` in between (flood-latched because
    /// the resolvers run per MESSAGE, not per registration).
    SeamsLoud,
    /// No bypass: the two-edged gate decides.
    Off,
}

/// Classify the bypass for this build (pure; the resolver passes
/// [`bindings_source`] and [`test_surface`], each derived ONCE from its
/// cfg). Vendored bindings are a precondition — the bypass never
/// applies to generated bindings at all.
pub fn classify_cpp_bypass(source: BindingsSource, surface: TestSurface) -> CppBypassMode {
    if source != BindingsSource::Vendored {
        return CppBypassMode::Off;
    }
    match surface {
        TestSurface::CfgTest => CppBypassMode::TestSilent,
        TestSurface::SeamsFeature => CppBypassMode::SeamsLoud,
        TestSurface::None => CppBypassMode::Off,
    }
}

/// Which bindings this `.so` was compiled against — the
/// `cerulion_rmw_vendored_bindings` cfg as a type, so a classifier input
/// cannot be confused with the test-surface flags (three positional
/// `bool`s at the resolver's call site would let a swapped
/// `cfg!(test)` / `cfg!(feature = "test-seams")` compile and turn a
/// shipped seams cdylib SILENT — the exact class the bypass rule forbids).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BindingsSource {
    /// bindgen output against installed ROS headers.
    Generated,
    /// The pinned rolling-era snapshot in `ffi/vendored_bindings.rs`.
    Vendored,
}

/// The bindings source of THIS build — the ONE derivation from the cfg.
pub fn bindings_source() -> BindingsSource {
    if cfg!(cerulion_rmw_vendored_bindings) {
        BindingsSource::Vendored
    } else {
        BindingsSource::Generated
    }
}

/// Which test surface, if any, this compilation carries — derived ONCE
/// from `cfg!(test)` / `cfg!(feature = "test-seams")` so the two can never
/// be passed in the wrong order. `cfg(test)` wins: the lib's own unit
/// tests carry the feature too (the dev-dependency self-reference).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TestSurface {
    /// A shipped artifact: neither `cfg(test)` nor the feature.
    None,
    /// The crate's own unit tests (`cargo test --lib`).
    CfgTest,
    /// The `test-seams` feature WITHOUT `cfg(test)`: the resolver-routed
    /// e2e binaries — or a hand-built `--features test-seams` cdylib.
    SeamsFeature,
}

/// The test surface of THIS compilation — the ONE derivation.
pub fn test_surface() -> TestSurface {
    if cfg!(test) {
        TestSurface::CfgTest
    } else if cfg!(feature = "test-seams") {
        TestSurface::SeamsFeature
    } else {
        TestSurface::None
    }
}

/// The C introspection era the compiled bindings carry — derived ONCE
/// from the two capability cfgs (`is_key_` arrived at Jazzy, the lower
/// bound; `is_rosidl_buffer_` at Lyrical, the upper), so the bridge
/// classifier cannot receive them swapped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CIntrospectionEra {
    /// Before `is_key_` (Foxy … Iron).
    PreJazzy,
    /// `is_key_` without `is_rosidl_buffer_` (Jazzy, Kilted) — the era the
    /// hand-mirrored C++ bridge is shaped for.
    Jazzy,
    /// `is_rosidl_buffer_` present (Lyrical, Rolling).
    PostJazzy,
}

/// The introspection era of THIS build's bindings — the ONE derivation.
pub fn c_introspection_era() -> CIntrospectionEra {
    match (
        cfg!(cerulion_has_is_key),
        cfg!(cerulion_has_is_rosidl_buffer),
    ) {
        (false, _) => CIntrospectionEra::PreJazzy,
        (true, false) => CIntrospectionEra::Jazzy,
        (true, true) => CIntrospectionEra::PostJazzy,
    }
}

/// The C++ bridge verdict for THIS build under a given bypass mode — the
/// call-site seam the resolvers use (`classify_cpp_bridge` over the
/// build's own [`c_introspection_era`]), split out so the wiring is
/// pinnable: a vendored build (rolling-pinned, post-Jazzy bits) classifies
/// `Supported` in every bypass mode now that the mirror carries the
/// Lyrical tail field; a pre-Jazzy build classifies `RefusePreJazzy` with
/// the bypass `Off` and `Supported` under either bypass.
pub fn cpp_bridge_gate_for(bypass: CppBypassMode) -> CppBridgeGate {
    classify_cpp_bridge(bypass, c_introspection_era())
}

/// The warning a [`CppBypassMode::SeamsLoud`] build emits from the
/// per-resolve path.
///
/// FLOOD-LATCHED. This is NOT a
/// "registration cadence, not per-frame" site:
/// `rmw_serialize` and `rmw_deserialize` resolve the
/// typesupport per MESSAGE, so an unlatched seams build republishing serialized
/// messages would emit this ~340-byte paragraph at frame rate (~3 GB/day
/// at 100 Hz — the disk-fill class the latch
/// exists to prevent). The design intent survives the latch intact: the
/// FIRST use is loud, every DECADE of the running total re-announces
/// loudly, and [`cpp_bypass_warns_fired`] counts unconditionally — so a
/// shipped `--features test-seams` cdylib can still never go silent.
pub const CPP_BYPASS_SEAMS_WARN: &str = concat!(
    "rmw_cerulion: test-seams C++ bypass active — the hand-mirrored C++ bridge is ",
    "accepted on VENDORED bindings only because this build carries the test-seams ",
    "feature (the e2e test surface). This build must never deploy: a deployed ",
    "vendored .so misreads live rclcpp introspection. Build without ",
    "--features test-seams for anything that ships."
);

/// Emit [`CPP_BYPASS_SEAMS_WARN`] — the one seam both resolvers call on
/// the [`CppBypassMode::SeamsLoud`] arm, split out so the level and
/// message are pinnable by `#[traced_test]` (own binary: the
/// global-subscriber-slot precedent). Loud head, `debug!` repeats, a
/// loud re-announcement at each DECADE of the running total.
pub fn emit_cpp_bypass_warn() {
    CPP_BYPASS_WARNS_FIRED.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    // A `{CONST}` capture, not `"{}", CONST`: the message stays a
    // compile-time constant the tracing-discipline walk can see.
    // The guard is dropped BEFORE the emission (a `match` scrutinee's
    // temporaries live for the whole match, which would hold the latch
    // across a `tracing` macro and any subscriber it calls).
    let decision = lock_regime_latch(&CPP_BYPASS_WARN_LATCH).on_failure();
    match decision {
        RegimeDecision::Loud => {
            tracing::warn!(built_for = %built_for(), "{CPP_BYPASS_SEAMS_WARN}")
        }
        RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
            built_for = %built_for(),
            total_failures = total,
            suppressed_count = suppressed,
            "{CPP_BYPASS_SEAMS_WARN}"
        ),
        RegimeDecision::Suppressed { suppressed } => tracing::debug!(
            built_for = %built_for(),
            suppressed_count = suppressed,
            "{CPP_BYPASS_SEAMS_WARN}"
        ),
    }
}

static CPP_BYPASS_WARNS_FIRED: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// Flood latch for [`emit_cpp_bypass_warn`]. A seams build's bypass is a
/// build-time constant, so the regime opens on the first resolve and
/// never recovers: nothing in PRODUCTION calls `on_success` (only the
/// `test-seams` re-arm below does, so the traced arms each get a head).
static CPP_BYPASS_WARN_LATCH: Mutex<FailureRegimeLatch> = Mutex::new(FailureRegimeLatch::new());

/// Flood latch for [`CppBridgeGate::emit_refusal`]. Which refusing
/// verdict a build carries is a compile-time constant, so ONE latch
/// serves both: no SHIPPED build can interleave them, and the regime
/// never recovers in production either (the tests drive both verdicts
/// and re-arm between arms).
static CPP_BRIDGE_REFUSAL_LATCH: Mutex<FailureRegimeLatch> = Mutex::new(FailureRegimeLatch::new());

static CPP_BRIDGE_REFUSALS_FIRED: std::sync::atomic::AtomicU64 =
    std::sync::atomic::AtomicU64::new(0);

/// How many times [`CppBridgeGate::emit_refusal`] has REFUSED in this
/// process — unconditional and never reset (Principle #3), read by the
/// tests that pin the latch cadence. NOT an operator surface: the
/// shipped artifact is a cdylib loaded through the standardized rmw C
/// ABI, which exposes no Rust symbol, so a ROS user cannot read it —
/// which is exactly why the DECADE re-announcement exists, and why
/// suppressing repeats is safe only WITH it.
pub fn cpp_bridge_refusals_fired() -> u64 {
    CPP_BRIDGE_REFUSALS_FIRED.load(std::sync::atomic::Ordering::Relaxed)
}

/// Re-arm the two C++-gate flood latches so the NEXT emission is loud
/// again — the test seam that stops an arm inheriting a sibling's OPEN
/// regime. Both latches are process-global (the
/// verdict and the bypass mode are build-time constants, so production
/// never recovers), which without this would give whichever test ran
/// first the only loud head. It re-arms the REGIME only: `total_failures`
/// and the decade ladder are deliberately untouched, so a re-arm cannot
/// buy a still-refusing build a fresh quota of silence — which also means
/// an arm that cares WHERE a decade falls must derive it from the running
/// counter rather than assume it runs first.
#[cfg(feature = "test-seams")]
pub fn rearm_cpp_gate_latches_for_test() {
    let _ = lock_regime_latch(&CPP_BRIDGE_REFUSAL_LATCH).on_success();
    let _ = lock_regime_latch(&CPP_BYPASS_WARN_LATCH).on_success();
}

/// How many times [`emit_cpp_bypass_warn`] has fired in this process —
/// the observable (Principle #3) that lets a resolver-routed e2e test
/// pin that the RESOLVER calls the seam on a C++ typesupport resolve
/// (pinning the seam itself does not pin the resolver's call of
/// it).
pub fn cpp_bypass_warns_fired() -> u64 {
    CPP_BYPASS_WARNS_FIRED.load(std::sync::atomic::Ordering::Relaxed)
}

/// The registration-time refusal the C++ typesupport arm emits on a
/// pre-Jazzy build (the generated path builds and
/// claims Foxy/Humble bindings, but `ffi/introspection_cpp.rs`
/// hand-mirrors the JAZZY-era C++ `MessageMember`/`MessageMembers`
/// shape, so the resolvers must not accept C++ typesupports there).
pub const CPP_BRIDGE_PRE_JAZZY_REFUSAL: &str = concat!(
    "rmw_cerulion: this build targets a pre-Jazzy distro, whose C++ introspection ",
    "layout differs from the Jazzy/Rolling shape the C++ bridge currently ",
    "hand-mirrors — refusing the rclcpp (C++ typesupport) path at registration ",
    "rather than misreading its introspection structs. Per-era C++ bridge variants ",
    "are not implemented yet; rclpy and C-typesupport ",
    "consumers are unaffected (the C introspection path is bindgen-generated ",
    "against this distro's real headers and correct on every era)."
);

/// Gate the hand-mirrored C++ introspection bridge on the
/// build's era — BOUNDED ON BOTH EDGES. `ffi/introspection_cpp.rs`
/// hardcodes the JAZZY/KILTED-era C++ shape EXACTLY: `is_key_` /
/// `has_any_key_member_` arrived at Jazzy (the lower bound) and
/// `is_rosidl_buffer_` widened the C++ `MessageMember` at Lyrical (the
/// upper bound — stride 112 → 120, so a `Supported` verdict there
/// would walk rclcpp member arrays at the wrong stride). Every one of
/// those fields was added to the C AND C++ introspection structs by
/// the same rosidl release, so the C capability tokens
/// (`cerulion_has_is_key`, `cerulion_has_is_rosidl_buffer`) are the
/// build-time proxies for the C++ mirror's era, folded ONCE into
/// [`c_introspection_era`] — a compile-time constant, so the gated arm
/// compiles to the right refusal per build (never a runtime sniff);
/// [`cpp_bridge_gate_for`] is the resolver's seam over it.
///
/// The VENDORED-TEST BYPASS (see [`classify_cpp_bypass`] — by design
/// the bypass rides `cfg(test)` silently or `test-seams` LOUDLY, with
/// a flood-latched [`CPP_BYPASS_SEAMS_WARN`] on the feature path, since
/// `test-seams` is a public feature a deployable build can enable).
/// Either bypass mode admits: the crate's own unit tests and — via the
/// dev-dependency self-reference that unifies
/// `test-seams` ON for every command building a test target — the
/// resolver-routed C++ e2e binaries (rmw_e2e / shadow-take /
/// slice-ceiling register `rosidl_typesupport_introspection_cpp`
/// typesupports through the C ABI), whose fixtures are hand-built
/// against the COMPILED `CppMessageMember` and therefore
/// layout-self-consistent by construction. The SHIPPED vendored cdylib
/// (`cargo build`, no test features) takes the normal gate: its
/// ROLLING-pinned bits classify `Supported`, because the mirror is
/// compiled with the Lyrical tail field under the same capability cfg,
/// and a vendored `.so` under any OTHER named runtime is already refused
/// at init by the era guard. SELECTION of the correct gate, nothing
/// more: the pre-Jazzy C++ variant is not implemented; the Jazzy and
/// Lyrical container lanes execute the
/// `Supported` arm, a lane for another era would execute its own, and this
/// classifier's oracle covers every input because one build can only
/// ever exercise one.
pub fn classify_cpp_bridge(bypass: CppBypassMode, era: CIntrospectionEra) -> CppBridgeGate {
    if bypass != CppBypassMode::Off {
        return CppBridgeGate::Supported;
    }
    match era {
        CIntrospectionEra::Jazzy => CppBridgeGate::Supported,
        CIntrospectionEra::PreJazzy => CppBridgeGate::RefusePreJazzy,
        CIntrospectionEra::PostJazzy => CppBridgeGate::Supported,
    }
}

/// The baked distro every guarded init export compares against the
/// runtime env — [`BUILT_FOR_DISTRO`], overridable through a
/// `test-seams` RAII guard so a test can pick a SPECIFIC pair
/// (`jazzy` vs `kilted`/`foxy`) in-process; a vendored dev build bakes
/// [`VENDORED_DEV_DISTRO`] and refuses on its own under any named
/// runtime outside the snapshot's era.
pub fn baked_distro() -> &'static str {
    #[cfg(feature = "test-seams")]
    if let Some(overridden) = crate::test_seams::baked_distro_override() {
        return overridden;
    }
    BUILT_FOR_DISTRO
}

/// Pure mismatch classifier (oracle-testable, no env access).
///
/// Returns `Some((baked, runtime))` — the NORMALIZED pair to refuse on,
/// both named — exactly when BOTH sides make a positive claim and the
/// claims differ. An absent/empty runtime `ROS_DISTRO` passes (the
/// environment makes no claim), as does an EMPTY baked value (a build
/// that made no claim at all — build.rs never bakes one). The unclaimed
/// [`VENDORED_DEV_DISTRO`] marker is NOT an absence:
/// the vendored snapshot is the pinned ROLLING-era ABI, so under
/// a NAMED runtime it admits exactly that era's layout-identical members
/// (`lyrical`/`rolling`, via [`crate::era_check::VENDORED_SNAPSHOT_ERA_TOKEN`]
/// and the one membership table) and refuses every other name — a
/// vendored `.so` ADMITTED under `ROS_DISTRO=jazzy` would
/// write `allocator`/`impl_` at rolling offsets into Jazzy's struct and
/// die at the first typed operation, the exact class this guard exists
/// to refuse. BOTH operands go through
/// the ONE normalization ([`crate::era_check::normalize_distro_claim`],
/// trim + ASCII lowercase, belt and braces): build.rs already
/// bakes the checker's normalized claim, and normalizing the runtime
/// side too means an odd env spelling (`ROS_DISTRO=JAZZY`, trailing
/// whitespace) can never manufacture a refusal of a COMPATIBLE library
/// — nor mask a genuine mismatch.
///
/// A LITERAL claim admits by LAYOUT identity, not by name:
/// build.rs bakes the concrete name (`lyrical`, `rolling`)
/// for a generated build under a named `ROS_DISTRO`, and those two are
/// verified byte-identical on every pinned axis — so the literal path
/// consults the SAME [`crate::era_check::layout_identical_distros`]
/// table the `era:` path does; a `lyrical` .so under `rolling` (and vice
/// versa) is a compatible library, not a contradiction, while
/// jazzy/kilted (168 vs 160 bytes) stay separated on both paths.
pub fn classify_distro_pair(baked: &str, runtime: Option<&str>) -> Option<(String, String)> {
    let baked = crate::era_check::normalize_distro_claim(baked);
    // The environment makes no claim: nothing to contradict, whatever
    // the build claims.
    let runtime = runtime
        .map(crate::era_check::normalize_distro_claim)
        .filter(|r| !r.is_empty())?;
    // A build that made NO claim at all (the empty spelling; build.rs
    // never bakes it — it fails the build on an env normalizing to
    // empty — so this arm exists for the override seam's sake).
    if baked.is_empty() {
        return None;
    }
    // The unclaimed marker names the vendored snapshot, whose era IS
    // known: admit its layout-identical members, refuse every other
    // named runtime.
    if baked == VENDORED_DEV_DISTRO {
        if crate::era_check::era_claim_admits(
            crate::era_check::VENDORED_SNAPSHOT_ERA_TOKEN,
            &runtime,
        ) {
            return None;
        }
        return Some((baked, runtime));
    }
    // Fingerprint-derived era claim (`era:lyrical` …): compare by
    // LAYOUT-identical membership (era_check::ERA_CLAIM_ADMITTED_MEMBERS
    // — rank membership would admit kilted under
    // era:jazzy while their rmw_init_options_t layouts differ, 168 vs
    // 160). Layout-identical members admit; anything else — an unknown
    // runtime name included — refuses, naming the era claim and the
    // runtime distro.
    if let Some(era_token) = baked.strip_prefix(crate::era_check::ERA_CLAIM_PREFIX) {
        if crate::era_check::era_claim_admits(era_token, &runtime) {
            return None;
        }
        return Some((baked, runtime));
    }
    if runtime == baked || crate::era_check::layout_identical_distros(&baked, &runtime) {
        return None;
    }
    Some((baked, runtime))
}

/// Env-reading half: [`classify_distro_pair`] against the process's
/// `ROS_DISTRO`. Every guarded rmw entry point (`rmw_init_options_init`
/// / `_copy` / `_fini`, and `rmw_init` as defense in depth) calls this
/// with [`baked_distro`]; tests call it with an injected baked side
/// under a faked env. Reads the ENVIRONMENT only — never a byte of any
/// caller-supplied struct — which is what lets the guard run before an
/// entry point touches memory whose layout the runtime distro decides.
pub fn runtime_distro_mismatch(baked: &str) -> Option<(String, String)> {
    // `var_os`, not `var`: a NON-UTF-8 value is a positive claim the
    // classifier cannot recognise, so it must REFUSE (against any claim)
    // — `var(..).ok()` would fold it into "no claim" and admit.
    // The lossy rendering carries U+FFFD and therefore equals no known
    // name.
    let runtime = std::env::var_os("ROS_DISTRO").map(|v| v.to_string_lossy().into_owned());
    classify_distro_pair(baked, runtime.as_deref())
}

/// The load-time distro/ABI refusal — ONE constant paragraph for every
/// guarded entry point; WHICH entry point refused rides the structured
/// `entry=` field of [`emit_distro_refusal`]. Named `rmw_init_options_t`
/// first: the guard sits at the FIRST entry points that write
/// caller memory, not only in `rmw_init`, because
/// `rmw_init_options_init` stamps `allocator`/`impl_` at
/// this build's offsets — and `rmw_init_options_copy`
/// memcpys this build's struct size — into a struct the runtime distro
/// lays out differently, before `rmw_init` ever runs (jazzy 168 B vs
/// kilted 160 B: the 8-byte `impl_` write lands PAST the caller's
/// struct).
pub const DISTRO_MISMATCH_REFUSAL: &str = concat!(
    "rmw_cerulion: this .so was built for a DIFFERENT ROS distro than this environment — ",
    "refusing at the named entry point (see entry=) BEFORE touching the caller's memory. ",
    "Per-distro ABI (rmw_init_options_t layout, introspection MessageMember stride, GID ",
    "width) means proceeding would corrupt the caller's init options or crash at the first ",
    "typed operation. Rebuild rmw_cerulion inside the runtime distro (bindgen against its ",
    "installed headers), or fix ROS_DISTRO."
);

/// The exports that carry the era guard — a CLOSED set, so a call site
/// cannot name an entry that does not exist and the `entry=` field the
/// tests pin is spelled in exactly one place. It lives beside
/// [`DISTRO_MISMATCH_REFUSAL`] so
/// [`emit_distro_refusal`] can take it BY TYPE: two of its four parameters
/// would otherwise be `&'static str`s that swap silently.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GuardedEntry {
    /// `rmw_init_options_init`.
    InitOptionsInit,
    /// `rmw_init_options_copy`.
    InitOptionsCopy,
    /// `rmw_init_options_fini`.
    InitOptionsFini,
    /// `rmw_init` — defense in depth (a caller may hand it a static
    /// struct that skipped `rmw_init_options_init`).
    Init,
}

impl GuardedEntry {
    /// The exported symbol's name — the `entry=` field value.
    pub fn name(self) -> &'static str {
        match self {
            GuardedEntry::InitOptionsInit => "rmw_init_options_init",
            GuardedEntry::InitOptionsCopy => "rmw_init_options_copy",
            GuardedEntry::InitOptionsFini => "rmw_init_options_fini",
            GuardedEntry::Init => "rmw_init",
        }
    }
}

/// Emit [`DISTRO_MISMATCH_REFUSAL`] at `error!` naming the refusing entry
/// point, both sides of the contradiction and whether rcl's error
/// channel received the reason (`rcl_error_channel=set|unavailable`;
/// the value states the OUTCOME, never a cause — see
/// [`crate::ffi::RclErrorChannel`]). A `{CONST}` capture (the
/// tracing-discipline walk's accepted shape), never a `"{}"` pass-through.
///
/// Deliberately NOT flood-latched, unlike the C++ bridge refusal: these
/// four exports are called per CONTEXT at process start-up (rmw.h's
/// documented order), never per message, so there is no cadence to
/// suppress. A refusing library does reach them more than once — rcl's
/// cleanup runs `rmw_init_options_fini` after a failed
/// `rmw_init_options_init`, and a host may build several contexts — but
/// the count is bounded by context count, not by message rate.
pub fn emit_distro_refusal(
    entry: GuardedEntry,
    baked: &str,
    runtime: &str,
    rcl_error_channel: crate::ffi::RclErrorChannel,
) {
    tracing::error!(
        entry = %entry.name(),
        baked_ros_distro = %baked,
        // Env-derived: rendered with control characters
        // escaped, so a hostile ROS_DISTRO cannot forge a second record.
        runtime_ros_distro = %crate::era_check::escape_control_chars(runtime),
        rcl_error_channel = %rcl_error_channel,
        built_for = %built_for(),
        "{DISTRO_MISMATCH_REFUSAL}"
    );
}

/// Pure banner predicate: should `rmw_init` announce, loudly, that this
/// binary carries UNCLAIMED bindings while the runtime environment
/// names a real distro?
///
/// The banner runs only where the guard ADMITTED an unclaimed build —
/// the vendored snapshot under its own era (`lyrical`/`rolling`; any
/// other named runtime is refused before this is consulted) — and
/// silently running bindings never verified against a real install of
/// that distro is a dev convenience, never a deployment posture, so the
/// admission is announced. True iff the (normalized) baked claim is
/// absent/unclaimed AND the trimmed runtime `ROS_DISTRO` is non-empty;
/// the empty spelling is kept for the override seam (build.rs never
/// bakes it — a generated build always carries a claim). An ACTIVE
/// claim (a named distro, an `era:` label, the test-seams override)
/// never triggers it: that posture belongs to the refusal guard.
pub fn warns_unclaimed_bindings(baked: &str, runtime: Option<&str>) -> bool {
    // The ONE normalization on the baked side too: the
    // classifier normalizes it, so an odd spelling of the unclaimed
    // marker must not be admitted there yet un-announced here.
    let baked = crate::era_check::normalize_distro_claim(baked);
    (baked.is_empty() || baked == VENDORED_DEV_DISTRO)
        && runtime.map(str::trim).is_some_and(|r| !r.is_empty())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_seams::EnvVarGuard;
    use serial_test::serial;
    use tracing_test::traced_test;

    #[test]
    fn the_refusal_rcl_text_is_built_once_and_is_per_verdict() {
        // A real-time allocation hazard: the C++ gate sits
        // on the per-MESSAGE resolve path, so its rcl text must be
        // rendered ONCE, not formatted per refusal. Pointer identity is
        // the oracle a `format!`-per-call cannot satisfy — it could not
        // even return `&'static`.
        let a = CppBridgeGate::RefusePreJazzy.rcl_error_text();
        let b = CppBridgeGate::RefusePreJazzy.rcl_error_text();
        assert!(
            std::ptr::eq(a, b),
            "the cached rcl text must be the SAME object on every refusal"
        );
        let a = a.to_string_lossy();
        // EXACT, not a prefix: the whole text rcl would report.
        assert_eq!(
            a.as_ref(),
            format!(
                "{CPP_BRIDGE_PRE_JAZZY_REFUSAL} built_for={} verdict=RefusePreJazzy",
                built_for()
            ),
            "the cached rcl text for RefusePreJazzy is wrong"
        );
    }

    #[test]
    fn classifier_refuses_only_a_positive_contradiction() {
        // Hand oracle vectors — every arm of the rule, both verdicts.
        type Case<'a> = (&'a str, Option<&'a str>, Option<(String, String)>);
        let mismatch = |b: &str, r: &str| Some((b.to_string(), r.to_string()));
        let cases: &[Case] = &[
            // Positive contradiction: refuse, naming both.
            ("jazzy", Some("foxy"), mismatch("jazzy", "foxy")),
            ("foxy", Some("jazzy"), mismatch("foxy", "jazzy")),
            // Agreement passes.
            ("jazzy", Some("jazzy"), None),
            // Whitespace never manufactures a refusal.
            ("jazzy", Some(" jazzy "), None),
            // NEITHER does case, on either operand — an odd
            // env spelling must not refuse a COMPATIBLE library.
            ("jazzy", Some("JAZZY "), None),
            (" JAZZY", Some("jazzy"), None),
            // … and normalization never MASKS a genuine mismatch.
            ("jazzy", Some(" FOXY "), mismatch("jazzy", "foxy")),
            // Fingerprint-derived era claims compare by era MEMBERSHIP
            // (a distro-less generated build bakes
            // `era:<token>`, never the unclaimed marker): members of
            // the claimed era are admitted …
            ("era:jazzy", Some("jazzy"), None),
            ("era:lyrical", Some("rolling"), None),
            ("era:lyrical", Some("lyrical"), None),
            // … membership is LAYOUT-identity, not fingerprint-identity
            // because kilted shares jazzy's fingerprint but
            // NOT its rmw_init_options_t layout (168 vs 160), so an
            // era:jazzy label must REFUSE it — reverting the kilted handling
            // fails here …
            ("era:jazzy", Some("kilted"), mismatch("era:jazzy", "kilted")),
            (
                "Era:Jazzy",
                Some(" KILTED "),
                mismatch("era:jazzy", "kilted"),
            ),
            // … normalization applies to era claims too …
            ("Era:Lyrical", Some(" ROLLING "), None),
            // … and anything OUTSIDE the era refuses, naming the era
            // claim and the runtime distro — unknown names included.
            ("era:jazzy", Some("foxy"), mismatch("era:jazzy", "foxy")),
            (
                "era:jazzy",
                Some("lyrical"),
                mismatch("era:jazzy", "lyrical"),
            ),
            (
                "era:lyrical",
                Some("jazzy"),
                mismatch("era:lyrical", "jazzy"),
            ),
            ("era:jazzy", Some("m_next"), mismatch("era:jazzy", "m_next")),
            // A LITERAL claim admits its
            // layout-identical siblings — build.rs bakes the concrete
            // `lyrical` / `rolling` under a named ROS_DISTRO, and the two
            // are byte-identical, so neither refuses the other …
            ("lyrical", Some("rolling"), None),
            ("rolling", Some("lyrical"), None),
            ("Rolling", Some(" LYRICAL "), None),
            // … while the literal jazzy/kilted pair stays SEPARATED on
            // the literal path too (168 vs 160) …
            ("jazzy", Some("kilted"), mismatch("jazzy", "kilted")),
            ("kilted", Some("jazzy"), mismatch("kilted", "jazzy")),
            // … and layout identity never crosses eras.
            ("lyrical", Some("jazzy"), mismatch("lyrical", "jazzy")),
            ("rolling", Some("kilted"), mismatch("rolling", "kilted")),
            ("jazzy", Some("rolling"), mismatch("jazzy", "rolling")),
            // Absent / empty runtime claim passes.
            ("jazzy", None, None),
            ("jazzy", Some(""), None),
            ("jazzy", Some("   "), None),
            // The EMPTY spelling (a build that made no claim at all)
            // never refuses …
            ("", Some("foxy"), None),
            // … but the unclaimed marker names the vendored snapshot —
            // the ROLLING-era ABI — so it admits exactly that era's
            // layout-identical members and refuses every other NAMED
            // runtime (a blanket admission would
            // let a vendored .so run under jazzy and scribble rolling
            // offsets into Jazzy's rmw_init_options_t).
            (VENDORED_DEV_DISTRO, Some("rolling"), None),
            (VENDORED_DEV_DISTRO, Some("lyrical"), None),
            (VENDORED_DEV_DISTRO, Some(" Rolling "), None),
            (
                VENDORED_DEV_DISTRO,
                Some("jazzy"),
                mismatch(VENDORED_DEV_DISTRO, "jazzy"),
            ),
            (
                VENDORED_DEV_DISTRO,
                Some("kilted"),
                mismatch(VENDORED_DEV_DISTRO, "kilted"),
            ),
            (
                VENDORED_DEV_DISTRO,
                Some("foxy"),
                mismatch(VENDORED_DEV_DISTRO, "foxy"),
            ),
            (
                VENDORED_DEV_DISTRO,
                Some("m_next"),
                mismatch(VENDORED_DEV_DISTRO, "m_next"),
            ),
            // … while an ABSENT runtime still admits it (dev convenience:
            // no ROS environment at all).
            (VENDORED_DEV_DISTRO, None, None),
            (VENDORED_DEV_DISTRO, Some("  "), None),
            // An era claim under an absent/blank runtime admits too — the
            // absence check must precede the era branch.
            ("era:lyrical", None, None),
            ("era:lyrical", Some(" "), None),
            // Trimmed runtime still refuses when genuinely different.
            ("jazzy", Some(" foxy "), mismatch("jazzy", "foxy")),
        ];
        for (baked, runtime, expected) in cases {
            assert_eq!(
                classify_distro_pair(baked, *runtime),
                *expected,
                "baked={baked:?} runtime={runtime:?}"
            );
        }
    }

    #[test]
    fn the_baked_distro_is_a_normalization_fixed_point() {
        // build.rs bakes either the literal "vendored-dev" or
        // the checker's NORMALIZED claim, so the baked value is always
        // already canonical (trimmed, ASCII-lowercase). Trivially green
        // on dev builds (vendored-dev); BITES in a container lane whose
        // env spells the distro oddly — the raw-bake defect class at
        // the build.rs call site, which the pure era_check arm cannot
        // see.
        assert_eq!(
            BUILT_FOR_DISTRO,
            crate::era_check::normalize_distro_claim(BUILT_FOR_DISTRO)
        );
    }

    #[test]
    fn the_cpp_bypass_is_cfg_test_silent_or_seams_loud_never_free() {
        // test-seams is a PUBLIC feature, so the bypass must
        // never be free with it — cfg(test) proper bypasses silently
        // (unreachable by any shipped artifact), the feature path
        // bypasses LOUDLY (a latched warn: head + decades), and everything
        // else is Off. Args: (BindingsSource, TestSurface) — each derived
        // ONCE from its cfg, so they cannot be passed swapped.
        use super::BindingsSource::*;
        use super::CppBypassMode::*;
        use super::TestSurface;
        assert_eq!(
            classify_cpp_bypass(Vendored, TestSurface::CfgTest),
            TestSilent
        );
        // The ONE derivations, pinned to hand facts about THIS compilation:
        // the lib's unit tests are `cfg(test)`, and the cfg naming the
        // bindings source must agree with the env string baked beside it.
        assert_eq!(test_surface(), TestSurface::CfgTest);
        assert_eq!(bindings_source() == Vendored, BINDINGS_SOURCE == "vendored");
        assert_eq!(
            classify_cpp_bypass(Vendored, TestSurface::SeamsFeature),
            SeamsLoud,
            "a seams build without cfg(test) — e2e binaries AND a \
             hand-built seams cdylib — must be the LOUD mode"
        );
        assert_eq!(classify_cpp_bypass(Vendored, TestSurface::None), Off);
        // Generated bindings never bypass, whatever the test bits say.
        for surface in [
            TestSurface::None,
            TestSurface::CfgTest,
            TestSurface::SeamsFeature,
        ] {
            assert_eq!(classify_cpp_bypass(Generated, surface), Off);
        }
        // The bypass warn names the three facts an operator needs.
        for phrase in [
            "test-seams C++ bypass active",
            "must never deploy",
            "Build without ",
        ] {
            assert!(
                CPP_BYPASS_SEAMS_WARN.contains(phrase),
                "seams warn must contain {phrase:?}"
            );
        }
    }

    #[test]
    fn the_cpp_bridge_gate_is_bounded_on_both_edges() {
        // One build can only exercise one input combination (the cfgs
        // are fixed at compile time), so the classifier is pinned on
        // ALL of them — the Jazzy container lane executes the Supported
        // arm; a lane for another era would execute its own. Args:
        // (CppBypassMode, CIntrospectionEra).
        // The mirror is the Jazzy/Kilted shape EXACTLY:
        assert_eq!(
            classify_cpp_bridge(CppBypassMode::Off, CIntrospectionEra::Jazzy),
            CppBridgeGate::Supported
        );
        // Lower bound (pre-Jazzy), contradictory-bits arm included:
        assert_eq!(
            classify_cpp_bridge(CppBypassMode::Off, CIntrospectionEra::PreJazzy),
            CppBridgeGate::RefusePreJazzy,
            "a pre-Jazzy build must refuse the hand-mirrored C++ bridge"
        );
        assert_eq!(
            classify_cpp_bridge(CppBypassMode::Off, CIntrospectionEra::PreJazzy),
            CppBridgeGate::RefusePreJazzy
        );
        // Post-Jazzy (Lyrical/Rolling): the mirror carries the appended
        // `is_rosidl_buffer_` under the same capability cfg that classifies
        // this era, so its stride is the distro's 120 bytes and the arm is
        // Supported; the compile-time size pins in `ffi/introspection_cpp.rs`
        // hold the two in lockstep.
        assert_eq!(
            classify_cpp_bridge(CppBypassMode::Off, CIntrospectionEra::PostJazzy),
            CppBridgeGate::Supported,
            "a Lyrical/Rolling build compiles the Lyrical-shaped C++ mirror"
        );
        // The vendored-TEST bypass (test surface only — the
        // bypass mode is TestSilent or SeamsLoud, never Off, on a
        // vendored test surface): hand-built fixtures are
        // layout-self-consistent, so every test target stays
        // Supported whatever the capability bits say …
        for bypass in [CppBypassMode::TestSilent, CppBypassMode::SeamsLoud] {
            for era in [
                CIntrospectionEra::PreJazzy,
                CIntrospectionEra::Jazzy,
                CIntrospectionEra::PostJazzy,
            ] {
                assert_eq!(
                    classify_cpp_bridge(bypass, era),
                    CppBridgeGate::Supported,
                    "vendored-test bypass: {bypass:?} {era:?}"
                );
            }
        }
        // … and a SHIPPED vendored cdylib (no test features, the bypass
        // Off) classifies by its ROLLING-pinned bits as Supported too: the
        // snapshot IS the Lyrical-era layout, and the era guard refuses it
        // at init under any other named runtime.
        assert_eq!(
            classify_cpp_bridge(CppBypassMode::Off, CIntrospectionEra::PostJazzy),
            CppBridgeGate::Supported,
            "a vendored-dev .so admits the C++ arm on its own era"
        );
        // Each refusal names the layout boundary, the missing per-era support, and
        // the unaffected consumers — the three things an operator
        // needs — and the verdict-to-message seam agrees.
        // The call-site seam over THIS build (the resolver's wiring needs
        // executing coverage): a vendored build carries the
        // rolling-era C introspection shape, so it classifies PostJazzy and
        // the gate is Supported in every bypass mode.
        if bindings_source() == BindingsSource::Vendored {
            assert_eq!(c_introspection_era(), CIntrospectionEra::PostJazzy);
            assert_eq!(
                cpp_bridge_gate_for(CppBypassMode::Off),
                CppBridgeGate::Supported,
                "a vendored-dev .so admits the C++ arm on its own era"
            );
        }
        assert_eq!(
            cpp_bridge_gate_for(CppBypassMode::TestSilent),
            CppBridgeGate::Supported
        );
        assert_eq!(
            cpp_bridge_gate_for(CppBypassMode::SeamsLoud),
            CppBridgeGate::Supported
        );
        for phrase in [
            "pre-Jazzy",
            "Jazzy/Rolling shape",
            "refusing the rclcpp (C++ typesupport) path",
            "are not implemented yet",
            "rclpy and C-typesupport consumers are unaffected",
        ] {
            assert!(
                CPP_BRIDGE_PRE_JAZZY_REFUSAL.contains(phrase),
                "pre-Jazzy refusal must contain {phrase:?}"
            );
        }
        assert_eq!(CppBridgeGate::Supported.refusal_message(), None);
        assert_eq!(
            CppBridgeGate::RefusePreJazzy.refusal_message(),
            Some(CPP_BRIDGE_PRE_JAZZY_REFUSAL)
        );
    }

    #[test]
    fn a_generated_bindings_binary_never_bakes_the_unclaimed_marker() {
        // A distro-less GENERATED build must not bake
        // the unclaimed marker and run under every distro despite
        // carrying one distro's real ABI. build.rs derives an era
        // claim from the fingerprint (or fails the build), so a
        // generated binary is NEVER unclaimed. Trivially green on the
        // vendored dev path; BITES in the container lanes (generated
        // bindings) — a bake-the-marker call-site defect class a
        // vendored-only build cannot see (the pure-seam
        // case is caught by the era_claim_for_observed oracles).
        assert!(
            BINDINGS_SOURCE == "vendored" || BUILT_FOR_DISTRO != VENDORED_DEV_DISTRO,
            "generated bindings must never bake the unclaimed marker:              bindings_source={BINDINGS_SOURCE} built_for_distro={BUILT_FOR_DISTRO}"
        );
    }

    #[test]
    fn a_vendored_bindings_binary_never_carries_a_distro_claim() {
        // Arm (a) catches baking the raw build
        // env: the claim derives from the SELECTED bindings source, and
        // rolling-shaped vendored bindings labeled with a real distro
        // name would make the load-time guard VOUCH for exactly the
        // skew it exists to refuse. Trivially green when ROS_DISTRO was
        // unset at build time; the discriminating run is
        // `ROS_DISTRO=foxy cargo test -p rmw_cerulion --lib` — build.rs
        // re-runs (rerun-if-env-changed=ROS_DISTRO) and a variant that
        // bakes the env fails here with "foxy". A generated lane (the
        // Jazzy container) passes via the first disjunct.
        //
        // That recipe WORKS only because the two
        // fixtures it drives name an admitted runtime for the duration
        // of their own setup call (`rmw_init_refuses_a_baked_runtime_-
        // distro_mismatch` unsets `ROS_DISTRO`; the era-guard binary's
        // `initialized_options` sets an admitted one). It is a
        // DISCRIMINATOR for this assertion, not a general invocation:
        // running the rest of the crate's suite under a contradicting
        // `ROS_DISTRO` is refused BY DESIGN — see the "Consequence for
        // the test suite" paragraph in docs/internals/rmw.md.
        assert!(
            BINDINGS_SOURCE == "generated" || BUILT_FOR_DISTRO == VENDORED_DEV_DISTRO,
            "vendored bindings must bake the unclaimed marker, got a distro claim: \
             bindings_source={BINDINGS_SOURCE} built_for_distro={BUILT_FOR_DISTRO}"
        );
    }

    #[test]
    fn unclaimed_bindings_admit_only_the_snapshots_era_and_are_announced() {
        // The unclaimed marker is the vendored
        // ROLLING-era snapshot, so it is ADMITTED under that era's
        // layout-identical members only …
        assert_eq!(
            classify_distro_pair(VENDORED_DEV_DISTRO, Some("rolling")),
            None
        );
        // … and REFUSED under any other named distro — a
        // blanket admission would let it scribble rolling offsets into a Jazzy
        // host's init options before dying at the first typed operation.
        assert_eq!(
            classify_distro_pair(VENDORED_DEV_DISTRO, Some("foxy")),
            Some((VENDORED_DEV_DISTRO.to_string(), "foxy".to_string()))
        );
        // … and where it IS admitted the admission is ANNOUNCED: an
        // unclaimed bake + a named runtime trips the banner predicate
        // (the predicate itself is claim-shape-only — it also fires for
        // the empty spelling, which only the override seam can produce).
        assert!(warns_unclaimed_bindings(VENDORED_DEV_DISTRO, Some("foxy")));
        assert!(warns_unclaimed_bindings("", Some("foxy")));
        // The baked side is normalized here exactly as the
        // classifier normalizes it — an odd spelling of the marker is
        // announced, never admitted-yet-silent.
        assert!(warns_unclaimed_bindings(" Vendored-Dev ", Some("rolling")));
        // No announcement without a runtime distro claim …
        assert!(!warns_unclaimed_bindings(VENDORED_DEV_DISTRO, None));
        assert!(!warns_unclaimed_bindings(VENDORED_DEV_DISTRO, Some("   ")));
        // … nor while an ACTIVE claim is in force (a claimed build, or
        // the test-seams override shape): that posture belongs to the
        // refusal guard.
        assert!(!warns_unclaimed_bindings("jazzy", Some("jazzy")));
        assert!(!warns_unclaimed_bindings("jazzy", Some("foxy")));
    }

    #[test]
    #[serial]
    fn env_wrapper_reads_ros_distro_through_the_classifier() {
        {
            let _g = EnvVarGuard::set("ROS_DISTRO", "foxy");
            assert_eq!(
                runtime_distro_mismatch("jazzy"),
                Some(("jazzy".to_string(), "foxy".to_string()))
            );
            assert_eq!(runtime_distro_mismatch("foxy"), None);
            // The unclaimed marker under a NAMED non-rolling
            // runtime is a positive contradiction.
            assert_eq!(
                runtime_distro_mismatch(VENDORED_DEV_DISTRO),
                Some((VENDORED_DEV_DISTRO.to_string(), "foxy".to_string()))
            );
        }
        {
            let _g = EnvVarGuard::set("ROS_DISTRO", "rolling");
            assert_eq!(runtime_distro_mismatch(VENDORED_DEV_DISTRO), None);
        }
        {
            let _g = EnvVarGuard::unset("ROS_DISTRO");
            assert_eq!(runtime_distro_mismatch("jazzy"), None);
            assert_eq!(runtime_distro_mismatch(VENDORED_DEV_DISTRO), None);
        }
    }

    #[test]
    #[serial]
    fn a_non_utf8_ros_distro_is_a_claim_that_matches_nothing() {
        // `var(..).ok()` would fold a NotUnicode value into "no
        // claim" and ADMIT; `var_os` keeps it as a positive claim the
        // classifier cannot recognise, so every claiming build refuses.
        use std::os::unix::ffi::OsStringExt as _;
        let _g = EnvVarGuard::set(
            "ROS_DISTRO",
            std::ffi::OsString::from_vec(vec![b'j', 0xff, b'z']),
        );
        assert!(
            runtime_distro_mismatch("jazzy").is_some(),
            "a claiming build must refuse an unreadable runtime claim"
        );
        assert!(
            runtime_distro_mismatch(VENDORED_DEV_DISTRO).is_some(),
            "so must the vendored snapshot"
        );
    }

    #[test]
    fn the_capability_fingerprint_names_exactly_the_cfgs_this_build_carries() {
        // `built_for()` is pinned here against the cfgs, not
        // against itself. Each capability token is present in the fingerprint iff
        // its cfg is set on THIS compilation, and the banner is the three
        // baked strings in the documented shape.
        let caps: Vec<&str> = CAPABILITY_FINGERPRINT
            .split(',')
            .filter(|c| !c.is_empty())
            .collect();
        let mut any_present = false;
        for (token, present) in [
            ("fetch_function", cfg!(cerulion_has_fetch_function)),
            ("is_key", cfg!(cerulion_has_is_key)),
            ("any_key_member", cfg!(cerulion_has_any_key_member)),
            ("is_rosidl_buffer", cfg!(cerulion_has_is_rosidl_buffer)),
            (
                "content_filter_options",
                cfg!(cerulion_has_content_filter_options),
            ),
            ("event_callback", cfg!(cerulion_has_event_callback)),
            ("qos_compatibility", cfg!(cerulion_has_qos_compatibility)),
            (
                "message_info_sequence_numbers",
                cfg!(cerulion_has_message_info_sequence_numbers),
            ),
            ("discovery_options", cfg!(cerulion_has_discovery_options)),
            ("matched_events", cfg!(cerulion_has_matched_events)),
            ("message_lost_event", cfg!(cerulion_has_message_lost_event)),
            ("event_type_max", cfg!(cerulion_has_event_type_max)),
            ("type_hash", cfg!(cerulion_has_type_hash)),
            ("features", cfg!(cerulion_has_features)),
            ("network_flow", cfg!(cerulion_has_network_flow)),
        ] {
            any_present |= present;
            assert_eq!(caps.contains(&token), present, "capability token {token}");
        }
        // The token set is CLOSED: a 16th capability
        // added to build.rs must land here too, and the `"none"` spelling
        // is correct only when every cfg is really off.
        const KNOWN: [&str; 15] = [
            "fetch_function",
            "is_key",
            "any_key_member",
            "is_rosidl_buffer",
            "content_filter_options",
            "event_callback",
            "qos_compatibility",
            "message_info_sequence_numbers",
            "discovery_options",
            "matched_events",
            "message_lost_event",
            "event_type_max",
            "type_hash",
            "features",
            "network_flow",
        ];
        if caps == ["none"] {
            assert!(
                !any_present,
                "\"none\" is correct only when every cfg is off"
            );
        } else {
            for token in &caps {
                assert!(KNOWN.contains(token), "unknown capability token {token}");
            }
        }
        // The bridge era and the resolver's gate, derived from the
        // fingerprint — an INDEPENDENT build.rs output — so this pin
        // executes in every lane, not only on the vendored build (a
        // vendored-only arm would be vacuous in a generated
        // lane, where a swapped cfg pair would refuse PreJazzy
        // unnoticed).
        let expected_era = match (caps.contains(&"is_key"), caps.contains(&"is_rosidl_buffer")) {
            (false, _) => CIntrospectionEra::PreJazzy,
            (true, false) => CIntrospectionEra::Jazzy,
            (true, true) => CIntrospectionEra::PostJazzy,
        };
        assert_eq!(c_introspection_era(), expected_era);
        assert_eq!(
            cpp_bridge_gate_for(CppBypassMode::Off),
            classify_cpp_bridge(CppBypassMode::Off, expected_era)
        );
        // The vendored snapshot's era token is pinned to the snapshot's
        // OWN fingerprint (pinning it to a hand-written
        // set would let a re-snapshot move the era while the token
        // stayed "lyrical").
        if bindings_source() == BindingsSource::Vendored {
            let derived = crate::era_check::era_claim_for_observed(&caps, None);
            let crate::era_check::GeneratedClaim::Bake(claim) = &derived else {
                panic!("the vendored snapshot's fingerprint must derive a bakeable claim, got {derived:?}");
            };
            assert_eq!(
                claim.to_string(),
                format!(
                    "{}{}",
                    crate::era_check::ERA_CLAIM_PREFIX,
                    crate::era_check::VENDORED_SNAPSHOT_ERA_TOKEN
                )
            );
        }
        assert_eq!(
            built_for(),
            format!("distro={BUILT_FOR_DISTRO} bindings={BINDINGS_SOURCE} caps={CAPABILITY_FINGERPRINT}")
        );
        assert!(
            !BUILT_FOR_DISTRO.is_empty(),
            "build.rs always bakes a claim spelling"
        );
    }

    /// The lane's opt-in expectation (`CERULION_RMW_EXPECT_BINDINGS`),
    /// or `None` when unset. A SET value the pin cannot read is a broken
    /// lane, not an absent one: `var(..).ok()` would fold it into "no
    /// expectation" and the pin would go vacuous (the
    /// same fold `runtime_distro_mismatch` refuses on `ROS_DISTRO`).
    fn lane_expected_bindings_source() -> Option<String> {
        std::env::var_os(EXPECT_BINDINGS_ENV).map(|raw| {
            raw.into_string().unwrap_or_else(|raw| {
                panic!(
                    "{EXPECT_BINDINGS_ENV} is set but not UTF-8 ({raw:?}) — it cannot name a \
                     bindings source; fix it or unset it"
                )
            })
        })
    }

    const EXPECT_BINDINGS_ENV: &str = "CERULION_RMW_EXPECT_BINDINGS";

    #[test]
    #[serial]
    fn a_lane_can_pin_the_bindings_source_it_expects() {
        // A lane that silently falls back to the
        // vendored bindings cannot tell — every claim test is a
        // disjunction over both sources. An opt-in expectation makes the
        // fallback loud: `CERULION_RMW_EXPECT_BINDINGS=generated|vendored`.
        //
        // OPT-IN means VACUOUS where it is unset, which
        // is every local `--lib` run and every container lane. It fires
        // in exactly one place: CI's `rmw_cerulion tests (serial)`
        // step on both OS matrices, which exports `vendored`. Read that
        // way it is a CI pin, not local coverage — which is also why the
        // unreadable-value fold below is loud rather than
        // silently disabling it.
        if let Some(expected) = lane_expected_bindings_source() {
            assert_eq!(
                BINDINGS_SOURCE, expected,
                "this lane expected {expected} bindings but built {BINDINGS_SOURCE}"
            );
        }
    }

    #[test]
    #[serial]
    #[should_panic(expected = "CERULION_RMW_EXPECT_BINDINGS is set but not UTF-8")]
    fn an_expectation_the_lane_pin_cannot_read_is_refused_not_ignored() {
        // A set-but-unreadable expectation must
        // fail the lane loudly, never silently disable its pin.
        use std::os::unix::ffi::OsStringExt as _;
        let _g = EnvVarGuard::set(
            EXPECT_BINDINGS_ENV,
            std::ffi::OsString::from_vec(vec![b'v', 0xff]),
        );
        let _ = lane_expected_bindings_source();
    }

    #[test]
    fn a_literal_claim_admits_exactly_what_its_era_label_admits() {
        // Drift tripwire: the literal path and the `era:` path
        // must never diverge — for every layout-identical group, each
        // member as a LITERAL baked claim admits precisely the runtimes
        // its era label admits, over a candidate set spanning every
        // group plus the separated and unknown names. Both are ALSO
        // pinned to the table itself, so a table edit moves both paths.
        let candidates = [
            "foxy", "humble", "iron", "jazzy", "kilted", "lyrical", "rolling", "m_next",
        ];
        for (token, members) in crate::era_check::ERA_CLAIM_ADMITTED_MEMBERS {
            let label = format!("{}{token}", crate::era_check::ERA_CLAIM_PREFIX);
            for member in *members {
                for runtime in candidates {
                    let literal = classify_distro_pair(member, Some(runtime)).is_none();
                    let labelled = classify_distro_pair(&label, Some(runtime)).is_none();
                    assert_eq!(
                        literal, labelled,
                        "literal `{member}` and `{label}` disagree on runtime `{runtime}`"
                    );
                    assert_eq!(
                        literal,
                        members.contains(&runtime),
                        "`{member}` under `{runtime}` must follow the table"
                    );
                }
            }
        }
    }

    #[test]
    #[serial]
    // `#[traced_test]`: this arm drives
    // the refusal path, which calls `runtime::install_tracing()` →
    // `set_global_default`. `tracing_test`'s own installer then hits its
    // `.expect(..)` and PANICS for any later `#[traced_test]` in this
    // binary — so without this attribute the lib suite is green only
    // because `api::pubsub`'s traced test sorts before `era::` and
    // claims the slot first. Any filtered subset, any rename, or a new
    // traced test in a module sorting after `era` breaks it, and
    // `#[serial]` cannot help (serialization is not ordering). Owning the
    // slot legitimately also lets the arm ASSERT the refusal line rather
    // than discard it.
    #[traced_test]
    fn rmw_init_refuses_a_baked_runtime_distro_mismatch() {
        // The refusal arm through the REAL C ABI entry point: the baked
        // side is injected via the test-seams override (this dev build
        // bakes "vendored-dev", which would refuse `foxy` on its own —
        // the override picks the SPECIFIC jazzy/foxy pair), the runtime side
        // via the env. Refusal happens BEFORE runtime()/transport init,
        // so the test needs no iceoryx2 and must leave the context
        // untouched. The options fixture is built BEFORE the mismatch is
        // armed: `rmw_init_options_init` carries the guard
        // itself and would refuse first (its own arms live in
        // `tests/rmw_era_guard_test.rs`) — this test pins the `rmw_init`
        // defense-in-depth line.
        let mut options: crate::ffi::rmw_init_options_t = unsafe { std::mem::zeroed() };
        // Hand-built zeroed allocator: rmw_init_options_init only stores
        // it, and linking rcutils_get_default_allocator would need
        // librcutils (absent outside a ROS process).
        let allocator: crate::ffi::rcutils_allocator_t = unsafe { std::mem::zeroed() };
        // The fixture is built under an ADMITTED runtime — ROS_DISTRO
        // unset, which every claim shape admits:
        // the guard is in rmw_init_options_init too, and the documented
        // headerless run `ROS_DISTRO=foxy cargo test -p rmw_cerulion
        // --lib` inherits a Foxy env the vendored snapshot REFUSES,
        // so under it this call would fail before the mismatch assertion
        // below ever ran.
        let ret = {
            let _admitted = EnvVarGuard::unset("ROS_DISTRO");
            unsafe { crate::rmw_init_options_init(&mut options, allocator) }
        };
        assert_eq!(ret, crate::ffi::RMW_RET_OK);

        let _baked = crate::test_seams::BakedDistroOverrideGuard::set("jazzy");
        // The runtime name carries a CONTROL character (a
        // log-injection hazard): `ROS_DISTRO` reaches the refusal through
        // `to_string_lossy`, and an unknown name refuses through the same
        // arm as `foxy` — so this one arm pins both the refusal and that
        // the line renders the byte ESCAPED (`foxy\u{1}`), on one record.
        let _env = EnvVarGuard::set("ROS_DISTRO", "foxy\u{1}");

        let mut context: crate::ffi::rmw_context_t = unsafe { std::mem::zeroed() };
        let ret = unsafe { crate::rmw_init(&options, &mut context) };
        assert_eq!(
            ret,
            crate::ffi::RMW_RET_ERROR,
            "a jazzy-built .so under ROS_DISTRO=foxy must refuse rmw_init"
        );
        // Refused BEFORE any context mutation.
        assert!(
            context.implementation_identifier.is_null(),
            "a refused rmw_init must leave the context untouched"
        );

        // Attribution controls: (a) the override really is what
        // rmw_init compared (the seam plumbs), (b) an AGREEING pair
        // classifies clean under the same env — so the refusal above is
        // the guard's verdict, not some other failure. (The agreeing arm
        // is deliberately NOT driven through rmw_init: a passing guard
        // proceeds to transport init, which the lib tests stay free of.)
        // (c) dropping the override restores the build's own baked value.
        assert_eq!(super::baked_distro(), "jazzy");
        {
            // The agreeing-pair control needs a NAMEABLE runtime: the
            // control-character value above refuses against every claim
            // by design, so the control runs under a plain `foxy` (the
            // guard nests LIFO and restores the hostile value on drop).
            let _agreeing = EnvVarGuard::set("ROS_DISTRO", "foxy");
            assert_eq!(runtime_distro_mismatch("foxy"), None);
        }
        assert_eq!(
            runtime_distro_mismatch("foxy"),
            Some(("foxy".to_string(), "foxy\u{1}".to_string())),
            "the control-character runtime must refuse even against its own prefix"
        );
        drop(_baked);
        assert_eq!(super::baked_distro(), BUILT_FOR_DISTRO);

        // Because this arm owns the traced slot, the
        // refusal LINE is asserted rather than discarded: exactly one,
        // at ERROR, naming the entry point and both sides. The
        // `rcl_error_channel=unavailable` reading is provable here — a
        // cargo test process maps no librcutils.
        logs_assert(|lines: &[&str]| {
            let hits: Vec<&str> = lines
                .iter()
                .copied()
                .filter(|l| l.contains("built for a DIFFERENT ROS distro"))
                .collect();
            if hits.len() != 1 {
                return Err(format!(
                    "expected exactly 1 refusal line, got {}:\n{}",
                    hits.len(),
                    lines.join("\n")
                ));
            }
            // Whole WHITESPACE tokens: `entry=rmw_init` is a PREFIX of
            // `entry=rmw_init_options_init`, and this arm exists to pin
            // the `rmw_init` defense-in-depth line specifically.
            for field in [
                "entry=rmw_init",
                "baked_ros_distro=jazzy",
                "runtime_ros_distro=foxy\\u{1}",
                "rcl_error_channel=unavailable",
            ] {
                if !hits[0].split_whitespace().any(|token| token == field) {
                    return Err(format!("refusal missing {field}: {}", hits[0]));
                }
            }
            // The raw control byte must not reach the record.
            if hits[0].chars().any(char::is_control) {
                return Err(format!(
                    "refusal carries a raw control character: {:?}",
                    hits[0]
                ));
            }
            Ok(())
        });
    }
}

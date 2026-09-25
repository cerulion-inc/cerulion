// SPDX-License-Identifier: AGPL-3.0-only
//! Build script for rmw_cerulion (era scaffolding).
//!
//! Decides where the C ABI type definitions come from:
//!
//! 1. **Installed ROS distro** — `CERULION_RMW_SYS_INCLUDE` naming
//!    existing include dirs, or `AMENT_PREFIX_PATH` carrying at least one
//!    prefix whose `include/` holds the core ROS package namespaces: run
//!    bindgen against the REAL headers. This is the only ABI-safe path
//!    for a deployed `librmw_cerulion.so` — introspection struct layouts
//!    drift across distros (e.g. `MessageMember.is_key_` exists on Jazzy+
//!    but not on older distros).
//! 2. **No ROS available** (dev machines, repo CI): use the committed
//!    vendored bindings (`src/ffi/vendored_bindings.rs`), generated from
//!    pinned source clones and stamped with their provenance. Good for
//!    compilation + unit tests; NOT for deployment against an arbitrary
//!    distro.
//! 3. **A prefix variable set but UNUSABLE**: neither of the above — the
//!    build FAILS. `CERULION_RMW_SYS_INCLUDE` naming no existing
//!    directory is a hard error whatever else is set; a set-and-non-empty
//!    prefix variable together with a set `ROS_DISTRO` is refused too,
//!    because falling back would put the pinned ROLLING snapshot behind a
//!    build the operator labelled with their distro. A headerless build
//!    with NO prefix variable is case 2 and only WARNS. The second
//!    refusal is the strictest default;
//!    the softer warn-and-fall-back contract is the alternative,
//!    described in `docs/internals/rmw.md`.
//!
//! The selection is signalled to `src/ffi/mod.rs` purely by the
//! `cerulion_rmw_generated_bindings` / `cerulion_rmw_vendored_bindings`
//! rustc cfgs emitted below — there is no env-var override knob.
//!
//! # Era scaffolding
//!
//! On top of source selection, this script derives the crate's
//! **capability cfgs** (`cerulion_has_*`) from the bindings it just
//! selected — never from a distro NAME — so each cfg is a fact about the
//! very bindings the crate compiles against and cannot be mispaired:
//!
//! - **Header probes** (bindgen path only): each post-Foxy rmw header in
//!   [`HEADER_PROBES`] is filesystem-probed under the collected include
//!   dirs and, when present, passed to clang as a `-DCERULION_HAS_*_H`
//!   define; `wrapper.h` gates the corresponding `#include` on it. This
//!   is what lets bindgen run cleanly on Foxy AND Humble
//!   (`rmw/discovery_options.h` is Iron+ — an unconditional include
//!   aborts the build on both).
//! - **Bindings-output probes**: the chosen bindings file (generated OR
//!   vendored — ONE code path, no special case) is grepped for the
//!   marker tokens in [`CAPABILITIES`] and a
//!   `cargo:rustc-cfg=cerulion_has_<cap>` is emitted per hit, each
//!   pre-declared via `cargo:rustc-check-cfg`. `src/ffi/era_pins.rs`
//!   keys per-era struct-size asserts on these cfgs (a header/bindings
//!   contradiction dies at `cargo build`), and `src/era.rs` bakes the
//!   fingerprint — plus the SELECTION-DERIVED distro claim — into the
//!   .so for the load-time distro guard in the init-options exports and
//!   `rmw_init`: a distro NAME is
//!   baked only when the bindings were GENERATED against installed
//!   headers whose capability fingerprint is CONSISTENT with that
//!   distro's era (`ROS_DISTRO` alone is not proof of the
//!   headers' distro; the shared `src/era_check.rs` classifier FAILS
//!   THE BUILD on a contradiction, naming the claim, the observed era,
//!   and the stale-env vs wrong-prefix causes); every vendored build
//!   bakes the unclaimed "vendored-dev" marker regardless of
//!   `ROS_DISTRO` (with a loud `cargo:warning` when the env named a
//!   distro the build cannot honor — and, per case 3 above, no vendored
//!   build is REACHED at all when a prefix variable was set alongside
//!   it). Capabilities distinguish ERAS, not
//!   every adjacent distro — see `era_check.rs` for the
//!   jazzy/kilted + lyrical/rolling residual.

use std::env;
use std::path::PathBuf;

// The header-provenance checker is SHARED with the
// crate (single source of truth — `src/era_check.rs` is also
// `pub mod era_check` in lib.rs, where oracle-vector tests pin the SAME
// code this script enforces; `#[cfg(test)]` strips them from this copy).
#[path = "src/era_check.rs"]
mod era_check;
use era_check::{check_distro_claim, observed_era_label, DistroClaimVerdict};

/// Every doc comment bindgen lifts from the ROS headers is wrapped in a
/// `text` code fence, so the doxygen text ships verbatim on the generated
/// bindings while rustdoc never reads an indented line of it as a Rust
/// code block (which `cargo test` would compile as a doctest). Any triple
/// backtick inside the text is spaced out so it cannot close the fence.
#[derive(Debug)]
struct DocCommentsAsText;

impl bindgen::callbacks::ParseCallbacks for DocCommentsAsText {
    fn process_comment(&self, comment: &str) -> Option<String> {
        let body = comment.replace("```", "` ` `");
        Some(format!("```text\n{body}\n```"))
    }
}

/// Capability cfg table: (`cerulion_has_<cap>` suffix, marker token
/// grepped — case-sensitive substring — in the selected bindings file).
///
/// First-appearance boundaries, verified per release branch against the
/// raw upstream headers (ros2/rosidl `message_introspection.h`,
/// ros2/rmw `types.h`/`event.h`/`features.h` etc.):
///
/// | cap | first distro | introduced by |
/// |---|---|---|
/// | `fetch_function` | Humble | ros2/rosidl#652 (2022-01-25; Galactic is byte-identical to Foxy) |
/// | `is_key` / `any_key_member` | Jazzy | ros2/rosidl#796 (`is_key_` lands MID-struct) |
/// | `is_rosidl_buffer` | Lyrical | ros2/rosidl#942 (appended last) |
/// | `content_filter_options` | Humble | content-filtered topics |
/// | `event_callback` | Humble | `rmw/event_callback_type.h` |
/// | `qos_compatibility` | Galactic | `rmw_qos_profile_check_compatible` |
/// | `message_info_sequence_numbers` | Humble | `rmw_message_info_t` growth |
/// | `discovery_options` | Iron | embedded in `rmw_init_options_t` |
/// | `matched_events` | Iron | matched/incompatible-type events |
/// | `message_lost_event` | Galactic | `RMW_EVENT_MESSAGE_LOST` |
/// | `event_type_max` | Lyrical | `RMW_EVENT_TYPE_MAX` sentinel |
/// | `type_hash` | Iron | typesupport type-hash trio (RIHS) |
/// | `features` | Humble | `rmw/features.h` |
/// | `network_flow` | Galactic | network-flow endpoints |
///
/// Token-choice rule: each token appears in the bindgen output IFF the
/// capability's type/field/constant exists in the sourced headers (all
/// 15 verified present in the vendored rolling snapshot, and absent
/// per-era in the downloaded foxy/humble/jazzy/lyrical branch headers).
/// Substring matching is deliberate — `discovery_options` matching
/// `rmw_discovery_options_t` AND the `rmw_init_options_s` field is the
/// same capability either way.
const CAPABILITIES: &[(&str, &str)] = &[
    ("fetch_function", "fetch_function"),
    ("is_key", "is_key_"),
    ("any_key_member", "has_any_key_member_"),
    ("is_rosidl_buffer", "is_rosidl_buffer_"),
    (
        "content_filter_options",
        "rmw_subscription_content_filter_options_t",
    ),
    ("event_callback", "rmw_event_callback_t"),
    ("qos_compatibility", "rmw_qos_compatibility_type_t"),
    (
        "message_info_sequence_numbers",
        "publication_sequence_number",
    ),
    ("discovery_options", "discovery_options"),
    ("matched_events", "RMW_EVENT_SUBSCRIPTION_MATCHED"),
    ("message_lost_event", "RMW_EVENT_MESSAGE_LOST"),
    ("event_type_max", "RMW_EVENT_TYPE_MAX"),
    ("type_hash", "get_type_hash_func"),
    ("features", "rmw_feature_t"),
    ("network_flow", "rmw_network_flow_endpoint_array_t"),
];

/// Post-Foxy headers probed on the filesystem (bindgen path only): when
/// `<include dir>/<relative path>` exists under ANY collected include
/// dir — the same resolution clang applies to the `#include` — the
/// define is passed to clang and `wrapper.h` compiles the include.
///
/// A probe enters this table WITH its `wrapper.h` consumer, never ahead
/// of it: every probed
/// header is also a capability whose include root must agree with the
/// anchor's, so a probe nothing consumes still REFUSES a valid two-root
/// overlay whose later root merely provides that header.
/// `rmw/dynamic_message_type_support.h` (Iron+) is such a header —
/// nothing in `wrapper.h` needs its types (the Iron+
/// dynamic-typesupport stubs are plain `RMW_RET_UNSUPPORTED` with untyped
/// params) — and is therefore NOT probed; probe it only together with
/// the include it gates.
const HEADER_PROBES: &[(&str, &str)] = &[
    (
        "CERULION_HAS_DISCOVERY_OPTIONS_H",
        "rmw/discovery_options.h",
    ), // Iron+
    (
        "CERULION_HAS_EVENT_CALLBACK_TYPE_H",
        "rmw/event_callback_type.h",
    ), // Humble+
    ("CERULION_HAS_FEATURES_H", "rmw/features.h"), // Humble+
    (
        "CERULION_HAS_NETWORK_FLOW_ENDPOINT_ARRAY_H",
        "rmw/network_flow_endpoint_array.h",
    ), // Galactic+
    (
        "CERULION_HAS_SUBSCRIPTION_CONTENT_FILTER_OPTIONS_H",
        "rmw/subscription_content_filter_options.h",
    ), // Humble+
    (
        "CERULION_HAS_QOS_STRING_CONVERSIONS_H",
        "rmw/qos_string_conversions.h",
    ), // Galactic+
];

/// Render `message` safe for cargo's DIRECTIVE channel, then print it as
/// a `cargo:warning=` line. THE ONE SITE that writes a warning, so
/// the directive-injection hazard below is handled once.
///
/// A build script's STDOUT is not a log — cargo parses every `cargo:`
/// line as a directive, and these warnings interpolate values that come
/// from the ENVIRONMENT (an `AMENT_PREFIX_PATH` entry, a `ROS_DISTRO`
/// name). See [`era_check::escape_for_cargo_directive`] for the rule, and
/// for the measured limit of what is exploitable — escaping is done at the
/// CHANNEL so no present or future feed has to be audited one at a time.
fn cargo_warning(message: &str) {
    // The escaper itself lives in the SHARED `era_check.rs` so it can be
    // oracle-tested from the lib (a build script has no test harness —
    // an untested escaper is how this class comes back).
    println!(
        "cargo:warning={}",
        era_check::escape_for_cargo_directive(message)
    );
}

/// What may be baked into `CERULION_RMW_BUILT_FOR_DISTRO`.
/// Two producers, both vetted: the VENDORED path's reserved
/// unclaimed marker, and a claim one of `era_check`'s classifiers
/// returned as a [`era_check::BakeableClaim`]. A raw `String` — a
/// `Contradiction`'s claimed name, an `UnknownDistro`'s normalized name,
/// or the raw env value — does not type-check here, which is what makes
/// the newtype a gate rather than a convention.
enum BakedDistroClaim<'a> {
    /// The vendored snapshot: pinned ROLLING whatever `ROS_DISTRO` says,
    /// so it always bakes the marker and never the env's distro name.
    VendoredUnclaimed,
    /// A claim `check_distro_claim` / `era_claim_for_observed` vetted.
    Vetted(&'a era_check::BakeableClaim),
}

/// The ONE site that writes `CERULION_RMW_BUILT_FOR_DISTRO` — the env
/// name is spelled exactly once, so the two producers cannot drift and a
/// third cannot appear without coming through this type.
fn bake_distro_claim(claim: BakedDistroClaim<'_>) {
    let value: &dyn std::fmt::Display = match &claim {
        BakedDistroClaim::VendoredUnclaimed => &era_check::RESERVED_UNCLAIMED_MARKER,
        BakedDistroClaim::Vetted(vetted) => *vetted,
    };
    // The bake channel is a DIRECTIVE too. Unlike a warning, a mangled
    // claim must never be silently escaped into the binary — the value
    // is already control-char-free by construction (a claim is either
    // `era_check`'s own table text or a `ROS_DISTRO` the read refused a
    // control character in), so this asserts that construction rather
    // than repairing it (the directive-channel class).
    let value = value.to_string();
    assert!(
        !value.chars().any(char::is_control),
        "rmw_cerulion: refusing to bake a distro claim carrying a control character \
         ({value:?}) — it would emit a second cargo directive"
    );
    println!("cargo:rustc-env=CERULION_RMW_BUILT_FOR_DISTRO={value}");
}

fn main() {
    println!("cargo:rerun-if-env-changed=CERULION_RMW_SYS_INCLUDE");
    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");
    println!("cargo:rerun-if-env-changed=ROS_DISTRO");
    println!("cargo:rerun-if-changed=wrapper.h");
    println!("cargo:rerun-if-changed=src/ffi/vendored_bindings.rs");
    println!("cargo:rerun-if-changed=src/era_check.rs");
    println!("cargo:rerun-if-changed=shim/cppstring_shim.cpp");

    // Declare every capability cfg up front (both paths) so the
    // `unexpected_cfgs` lint knows the names; emission below is
    // conditional on the bindings content.
    for (cap, _) in CAPABILITIES {
        println!("cargo:rustc-check-cfg=cfg(cerulion_has_{cap})");
    }

    // Load-time era guard input, read here but BAKED only after
    // source selection below: the distro claim derives from the
    // selected bindings, never from the raw env (baking the env's
    // name onto a vendored fallback would label the pinned
    // ROLLING ABI with a real distro, and the guard would then VOUCH for
    // exactly the cross-distro skew it exists to refuse).
    // `var_os`: a NON-UTF-8 value cannot name a distro and must not fold
    // into "no ROS_DISTRO set" — fail with the bytes.
    let build_env_distro = env::var_os("ROS_DISTRO")
        .map(|v| {
            v.into_string().unwrap_or_else(|bad| {
                panic!(
                    "rmw_cerulion: ROS_DISTRO is set but is not valid UTF-8 ({bad:?}) — it \
                     cannot name a distro; fix it or unset it"
                )
            })
        })
        .map(|s| s.trim().to_string())
        .inspect(|s| {
            // Control characters are refused for the SAME reason as
            // non-UTF-8: this value is echoed into
            // `cargo:warning=` lines on build-script STDOUT, which cargo
            // parses as directives — an embedded newline emits a second
            // one, and `cargo:rustc-env=` is last-wins, so
            // `ROS_DISTRO=$'x\ncargo:rustc-env=CERULION_RMW_BUILT_FOR_DISTRO=jazzy'`
            // would ship a vendored rolling `.so` baked as jazzy with a
            // green build. The `BakeableClaim` newtype gates the typed
            // WRITER; it cannot gate the channel, so the value is refused
            // at the read.
            assert!(
                !s.chars().any(char::is_control),
                "rmw_cerulion: ROS_DISTRO contains a control character ({s:?}) — it cannot \
                 name a distro, and this build script echoes it onto the cargo directive \
                 channel; fix it or unset it"
            );
        })
        .filter(|s| !s.is_empty());

    // std::string ABI shim (introspection_cpp bridge).
    // Compiled C++ — the platform's REAL string ABI, never a guess.
    // Unconditional: pure libstdc++/libc++, no ROS headers required,
    // so the cpp bridge is unit-testable on dev machines too.
    cc::Build::new()
        .cpp(true)
        // Pin the C++ standard: the shim uses `noexcept` + `std::string`
        // (C++11). Linux g++ defaults high enough to hide this, but macOS
        // clang rejects `noexcept` without an explicit -std.
        // flag_if_supported keeps it
        // portable across compilers that spell the flag differently.
        .flag_if_supported("-std=c++14")
        .file("shim/cppstring_shim.cpp")
        .compile("rmw_cerulion_cppstring_shim");

    let include_dirs = collect_include_dirs();

    let (bindings_path, generated) = if include_dirs.is_empty() {
        // Path 2: vendored bindings.
        println!("cargo:rustc-cfg=cerulion_rmw_vendored_bindings");
        println!("cargo:rustc-env=CERULION_RMW_BINDINGS_SOURCE=vendored");
        // The vendored snapshot is pinned ROLLING no matter what
        // ROS_DISTRO says, so a vendored build ALWAYS bakes the
        // unclaimed marker — never the env's distro name.
        bake_distro_claim(BakedDistroClaim::VendoredUnclaimed);
        if let Some(distro) = &build_env_distro {
            // The operator thinks they are building for a distro. With a
            // prefix variable ALSO set they clearly meant to build
            // against installed headers, and a silent vendored fallback
            // would ship the rolling-era snapshot under their distro's
            // name in the deploy log — fail the build (because
            // the `-q`-swallowed warning below is not enough there).
            // ROS_DISTRO alone (a shell habit, no install) keeps the
            // warning: that is the documented dev posture.
            // A SET-BUT-EMPTY prefix variable (`${VAR:-}` shell rc, an
            // empty Dockerfile ENV) names nothing and keeps the warning.
            if env::var_os("AMENT_PREFIX_PATH").is_some_and(|v| !v.is_empty())
                || env::var_os("CERULION_RMW_SYS_INCLUDE").is_some_and(|v| !v.is_empty())
            {
                panic!(
                    "rmw_cerulion: ROS_DISTRO={distro} and a header prefix variable \
                     (AMENT_PREFIX_PATH / CERULION_RMW_SYS_INCLUDE) are both set, but NO usable \
                     ROS headers were found under the prefix — refusing to fall back to the \
                     VENDORED rolling bindings for a build that names {distro}. Point the prefix \
                     at the {distro} install (its include/ must carry rmw/ and rosidl_runtime_c/), \
                     or unset ROS_DISTRO to build the unclaimed dev .so deliberately."
                );
            }
            // No prefix variable at all: tell them at BUILD time, not at
            // a runtime refusal — no headers were found, so this .so
            // carries the rolling-era vendored snapshot, makes no claim
            // for their distro, and will REFUSE under that
            // ROS_DISTRO unless it is lyrical/rolling: the same shell
            // that built it cannot run it until ROS_DISTRO is unset.
            cargo_warning(&format!(
                "rmw_cerulion: ROS_DISTRO={distro} is set but NO ROS headers were \
                 found — this build uses the VENDORED ROLLING bindings and makes no '{distro}' \
                 claim (baked distro: vendored-dev). It will REFUSE rmw_init_options_init under \
                 ROS_DISTRO={distro} (the snapshot admits only lyrical/rolling): unset ROS_DISTRO \
                 to run this dev .so here. A deployed .so for {distro} must be built where its \
                 headers are installed (AMENT_PREFIX_PATH or CERULION_RMW_SYS_INCLUDE)."
            ));
        } else {
            cargo_warning(
                "rmw_cerulion: no ROS headers found — using VENDORED bindings \
                 (fine for development; build inside the target ROS distro for deployment)",
            );
        }
        (PathBuf::from("src/ffi/vendored_bindings.rs"), false)
    } else {
        // Path 1: bindgen against real headers.
        let mut builder = bindgen::Builder::default()
            .header("wrapper.h")
            // The doxygen text from the ROS headers stays on the generated
            // bindings (a consumer reading the docs gets it), but as
            // preformatted text: copied verbatim, its indented lines are
            // rustdoc code blocks that `cargo test` compiles as doctests and
            // fails on (23 of them on the first real-header run), and a
            // `text` fence is never a doctest. See `DocCommentsAsText`.
            .parse_callbacks(Box::new(DocCommentsAsText))
            .layout_tests(true)
            .derive_default(true)
            .prepend_enum_name(false)
            .allowlist_type("rmw_.*")
            .allowlist_type("rcutils_.*")
            .allowlist_type("rosidl_.*")
            .allowlist_var("RMW_.*")
            .allowlist_var("rosidl_typesupport_introspection_c__.*")
            .allowlist_function("rcutils_get_default_allocator")
            .allowlist_function("rmw_get_zero_initialized_.*")
            .allowlist_function("rcutils_string_array_.*")
            .allowlist_function("rmw_names_and_types_.*")
            .allowlist_function("rmw_topic_endpoint_info_.*");

        for dir in &include_dirs {
            builder = builder.clang_arg(format!("-I{}", dir.display()));
        }

        // Probe the post-Foxy headers and tell clang which
        // ones exist, so `wrapper.h`'s gated includes match the sourced
        // distro (Foxy/Humble lack several — see HEADER_PROBES).
        for define in probe_headers(&include_dirs) {
            builder = builder.clang_arg(format!("-D{define}=1"));
        }

        let bindings = builder
            .generate()
            .expect("rmw_cerulion: bindgen failed against the provided ROS headers");

        let out_path = PathBuf::from(env::var("OUT_DIR").expect("OUT_DIR")).join("bindings.rs");
        bindings
            .write_to_file(&out_path)
            .expect("rmw_cerulion: failed to write bindings.rs");
        println!("cargo:rustc-cfg=cerulion_rmw_generated_bindings");
        println!("cargo:rustc-env=CERULION_RMW_BINDINGS_SOURCE=generated");
        (out_path, true)
    };

    let bindings_contents = std::fs::read_to_string(&bindings_path).unwrap_or_else(|e| {
        panic!(
            "rmw_cerulion: cannot read the selected bindings file {} for capability \
             probing: {e}",
            bindings_path.display()
        )
    });
    let observed_caps = emit_capability_cfgs(&bindings_contents);
    // The layout discriminator the fingerprint lacks:
    // jazzy/kilted share one capability fingerprint while
    // their rmw_init_options_t layouts DIFFER (168 vs 160 — kilted
    // dropped localhost_only), and the claim IS the runtime guard's
    // input at every guarded export — an ambiguous claim could refuse a
    // wrong-layout write at none of them — so the pair must be separated
    // at BUILD time, and the bindings' own bindgen layout test carries
    // the size.
    let init_options_size = era_check::probe_init_options_size(&bindings_contents);

    if generated {
        // Selection-derived claim: a distro NAME is baked only
        // for bindings actually GENERATED against installed headers
        // whose capability fingerprint is CONSISTENT with that distro's
        // era — ROS_DISTRO alone is not proof of the headers' distro
        // (the env can say jazzy while AMENT_PREFIX_PATH /
        // CERULION_RMW_SYS_INCLUDE supplies Humble headers; baking
        // "jazzy" then lets rmw_init admit a layout-incompatible .so on
        // a Jazzy robot — the guard defeated from the inside). A
        // contradiction FAILS THE BUILD. A generated build whose
        // environment names no distro (e.g. CERULION_RMW_SYS_INCLUDE
        // without sourcing ROS) bakes a claim DERIVED from the
        // fingerprint AND the layout (baking the
        // unclaimed marker here would let one distro's real generated ABI run
        // under EVERY runtime distro, and an era label spanning
        // layout-DIVERGENT members would admit an ABI break): the concrete
        // distro name when the era has one member or the init-options
        // size separates the jazzy/kilted pair, the `era:<token>` label
        // only where every member's pinned layout is verified IDENTICAL
        // (lyrical/rolling), and a hard build failure when the
        // fingerprint matches no known era or the pair cannot be
        // disambiguated.
        let claim = match &build_env_distro {
            None => match era_check::era_claim_for_observed(&observed_caps, init_options_size) {
                era_check::GeneratedClaim::Bake(claim) => {
                    cargo_warning(&format!(
                        "rmw_cerulion: no usable ROS_DISTRO (unset, or blank \
                         after trimming) — the generated \
                         bindings' fingerprint + layout imply '{claim}', baking that (the \
                         runtime guard admits only its layout-identical members). Set \
                         ROS_DISTRO to name the target distro exactly."
                    ));
                    claim
                }
                era_check::GeneratedClaim::AmbiguousEra {
                    era_label,
                    members,
                    init_options_size: era_size,
                } => panic!(
                    "rmw_cerulion: no ROS_DISTRO is set and the generated bindings' \
                     capability fingerprint cannot distinguish {members:?} (the \
                     '{era_label}' era) — and their rmw_init_options_t layouts DIFFER \
                     (jazzy 168 vs kilted 160 bytes; kilted dropped localhost_only). The \
                     bindings' init-options layout test read {era_size:?}, which \
                     disambiguates neither, and an ambiguous claim is what the runtime \
                     guard would compare — it could refuse a wrong-layout write at no \
                     entry point. Build \
                     with ROS_DISTRO set to make a verified claim, or fix \
                     AMENT_PREFIX_PATH / CERULION_RMW_SYS_INCLUDE."
                ),
                era_check::GeneratedClaim::NoKnownEra => panic!(
                    "rmw_cerulion: no ROS_DISTRO is set and the generated bindings' \
                     capability fingerprint ({observed_caps:?}) matches no known era — \
                     no era can be claimed for this .so, and an unclaimed \
                     generated build would run under every distro despite carrying one \
                     distro's ABI. Fix AMENT_PREFIX_PATH / CERULION_RMW_SYS_INCLUDE \
                     (mixed include roots?), set ROS_DISTRO to the intended distro (the \
                     claim is then fingerprint-verified), or — for a genuinely new \
                     upstream shape — extend the capability table in build.rs."
                ),
            },
            Some(distro) => match check_distro_claim(distro, &observed_caps) {
                // Bake the verdict's NORMALIZED claim, never
                // the raw env string — ROS_DISTRO=JAZZY passes the
                // check (the classifier normalizes) but a raw bake of
                // "JAZZY" then fails era.rs's runtime comparison
                // against a conventionally-sourced `jazzy`, refusing a
                // COMPATIBLE library. What the checker judged is what
                // gets baked.
                DistroClaimVerdict::Consistent { normalized } => {
                    // The jazzy/kilted pair is fingerprint-identical, so
                    // an env claim naming one member with the OTHER's
                    // headers passes the fingerprint check — the
                    // init-options size does not (closing the
                    // env-lie hole whenever the size is readable).
                    // The size probe is the ONLY
                    // cross-check separating the fingerprint-identical
                    // jazzy/kilted pair — an UNREADABLE layout test
                    // must refuse those claims, never admit them
                    // unverified (unambiguous claims stay non-fatal:
                    // nothing to disambiguate).
                    if era_check::divergent_claim_unverifiable(
                        normalized.as_ref(),
                        init_options_size,
                    ) {
                        panic!(
                            "rmw_cerulion: ROS_DISTRO={normalized} is one of the \
                             fingerprint-identical jazzy/kilted pair, and the generated \
                             bindings carry NO readable rmw_init_options_t layout test — the \
                             init-options size is the only cross-check separating the two \
                             (168 vs 160), so this claim cannot be verified. bindgen's \
                             layout tests are enabled by build.rs (layout_tests(true)); if \
                             they are present but unparsed, a bindgen format change broke \
                             the probe — regenerate the bindings and update \
                             probe_init_options_size in src/era_check.rs to the new \
                             spelling."
                        );
                    }
                    if let Some((expected, got)) = era_check::init_options_size_contradicts(
                        normalized.as_ref(),
                        init_options_size,
                    ) {
                        panic!(
                            "rmw_cerulion: ROS_DISTRO={normalized}, but the generated \
                             bindings lay out rmw_init_options_t at {got} bytes where \
                             {normalized} requires {expected} — the headers are the OTHER \
                             member of the fingerprint-identical jazzy/kilted pair (jazzy \
                             168 / kilted 160; kilted dropped localhost_only). Baking this \
                             claim would admit a layout-incompatible .so at runtime. Fix \
                             ROS_DISTRO or AMENT_PREFIX_PATH / CERULION_RMW_SYS_INCLUDE \
                             to agree."
                        );
                    }
                    normalized
                }
                DistroClaimVerdict::UnknownDistro { normalized } => {
                    // An unknown name is NOT a
                    // validation bypass — the claim comes from the
                    // fingerprint + layout, exactly like the no-env
                    // path, and the unknown name is never baked.
                    match era_check::claim_for_unknown_distro(
                        &normalized,
                        &observed_caps,
                        init_options_size,
                    ) {
                        era_check::GeneratedClaim::Bake(derived) => {
                            cargo_warning(&format!(
                                "rmw_cerulion: ROS_DISTRO={distro} is not a \
                                 distro this build's era table knows; the generated bindings' \
                                 fingerprint + layout imply '{derived}', baking THAT — never \
                                 the unknown name, whose ABI cannot be verified (the runtime \
                                 guard admits only '{derived}''s layout-identical members). If \
                                 '{distro}' is a typo, fix the environment; a genuinely new \
                                 distro needs the era table in src/era_check.rs extended."
                            ));
                            derived
                        }
                        era_check::GeneratedClaim::AmbiguousEra {
                            era_label,
                            members,
                            init_options_size: era_size,
                        } => panic!(
                            "rmw_cerulion: ROS_DISTRO={distro} is not a distro this build's \
                             era table knows, and the generated bindings' fingerprint matches \
                             the layout-DIVERGENT {members:?} pair (the '{era_label}' era) \
                             with an init-options layout test reading {era_size:?} — \
                             nothing verifiable can be claimed. Fix ROS_DISTRO to a known \
                             distro, or fix AMENT_PREFIX_PATH / CERULION_RMW_SYS_INCLUDE."
                        ),
                        era_check::GeneratedClaim::NoKnownEra => panic!(
                            "rmw_cerulion: ROS_DISTRO={distro} is not a distro this build's \
                             era table knows AND the generated bindings' capability \
                             fingerprint ({observed_caps:?}) matches no known era — nothing \
                             verifiable can be claimed for this .so. Fix ROS_DISTRO and/or \
                             AMENT_PREFIX_PATH / CERULION_RMW_SYS_INCLUDE; for a genuinely \
                             new upstream shape, extend the capability table in build.rs."
                        ),
                    }
                }
                DistroClaimVerdict::ReservedCollision { normalized } => {
                    // Refuse to forge the unclaimed marker.
                    panic!(
                        "rmw_cerulion: ROS_DISTRO='{distro}' normalizes to '{normalized}', \
                         which is the RESERVED unclaimed marker only the vendored-bindings \
                         path may bake — baking it from a GENERATED build would make the \
                         runtime guard treat this distro-specific ABI as the rolling-era \
                         vendored snapshot and admit it under lyrical/rolling. ROS_DISTRO must \
                         not name the reserved vendored-dev marker: unset it (a distro-less \
                         generated build bakes a fingerprint-derived claim) or name a real \
                         distro."
                    );
                }
                DistroClaimVerdict::Contradiction {
                    claimed,
                    expected,
                    observed,
                    missing,
                    unexpected,
                } => {
                    let observed_label = observed_era_label(&observed)
                        .map(|era| format!("the {era} era"))
                        .unwrap_or_else(|| "no single known era (a MIXED header set)".to_string());
                    panic!(
                        "rmw_cerulion: ROS_DISTRO={claimed}, but the generated bindings' \
                         capability fingerprint looks like {observed_label} — refusing to bake \
                         a distro claim the selected headers contradict (rmw_init would \
                         otherwise see matching names and admit a layout-incompatible .so on a \
                         {claimed} robot).\n\
                         expected capabilities for {claimed}: {expected:?}\n\
                         observed in the generated bindings: {observed:?}\n\
                         missing (headers OLDER than the claim): {missing:?}\n\
                         unexpected (headers NEWER than the claim): {unexpected:?}\n\
                         Two likely causes:\n\
                         1. STALE ROS_DISTRO in the environment (a different distro's setup.bash \
                         was sourced earlier) — re-source the intended distro's setup.bash, or \
                         unset ROS_DISTRO to build without a distro claim.\n\
                         2. WRONG AMENT_PREFIX_PATH / CERULION_RMW_SYS_INCLUDE (pointing at a \
                         different distro's install than ROS_DISTRO names) — point it at the \
                         {claimed} install this .so is meant for.\n\
                         Note: capabilities distinguish ERAS, not every adjacent distro \
                         (jazzy/kilted and lyrical/rolling share fingerprints; within those \
                         pairs the pinned layouts genuinely agree)."
                    );
                }
            },
        };
        bake_distro_claim(BakedDistroClaim::Vetted(&claim));
    }
}

/// Which post-Foxy headers the SELECTED root serves, with clang's own
/// provenance: bindgen resolves the anchor
/// `rmw/init_options.h` by FIRST match along the ordered `-I` list, so a
/// capability counts only when its header sits under that same root. A
/// plain existence probe would enable a capability if ANY dir had its header —
/// an older root first and a newer root later would then keep a pre-Iron
/// `rmw_init_options_t` beside an Iron `discovery_options` capability,
/// and the fingerprint would bake an `iron` claim an Iron runtime admits. The
/// `era_pins` size asserts do not catch that
/// pairing — the jazzy/kilted size table is the only
/// init-options pin, and the MessageMember pins are the same size for
/// Humble and Iron. A MIXED tree is therefore REFUSED, naming both roots.
fn probe_headers(include_dirs: &[PathBuf]) -> Vec<&'static str> {
    match era_check::resolve_capability_headers(
        include_dirs,
        "rmw/init_options.h",
        HEADER_PROBES,
        |p| p.is_file(),
    ) {
        Ok(defines) => defines,
        Err(era_check::CapabilityProbeRefusal::NoAnchor { anchor }) => panic!(
            "rmw_cerulion: none of the selected include dirs serves {anchor} — the rmw headers \
             are not where the prefix points; fix AMENT_PREFIX_PATH / CERULION_RMW_SYS_INCLUDE"
        ),
        Err(era_check::CapabilityProbeRefusal::MixedRoots {
            anchor_root,
            capability,
            found_in,
        }) => panic!(
            "rmw_cerulion: MIXED include tree — rmw/init_options.h resolves from {} (clang's \
             first match) but {capability} is served only by {}: the generated bindings would \
             carry that root's layout with this root's capability, a pairing no runtime \
             admits. Point the prefix at ONE distro install.",
            anchor_root.display(),
            found_in.display()
        ),
    }
}

/// Grep the SELECTED bindings file (generated or vendored — one code
/// path) for each capability's marker token; emit the matching
/// `cerulion_has_*` cfgs plus the comma-joined fingerprint `src/era.rs`
/// bakes into the .so. Returns the observed capability set — the
/// header-derived fact the distro-claim check validates
/// against.
fn emit_capability_cfgs(contents: &str) -> Vec<&'static str> {
    let mut caps = Vec::new();
    for (cap, token) in CAPABILITIES {
        if contents.contains(token) {
            println!("cargo:rustc-cfg=cerulion_has_{cap}");
            caps.push(*cap);
        }
    }
    let fingerprint = if caps.is_empty() {
        "none".to_string()
    } else {
        caps.join(",")
    };
    println!("cargo:rustc-env=CERULION_RMW_CAPS={fingerprint}");
    caps
}

/// Include-dir discovery: explicit env var first, then every
/// `<prefix>/include` (+ per-package subdirs, the ament layout) under
/// AMENT_PREFIX_PATH.
fn collect_include_dirs() -> Vec<PathBuf> {
    if let Some(spec) = env::var_os("CERULION_RMW_SYS_INCLUDE") {
        let spec = spec.into_string().unwrap_or_else(|bad| {
            panic!(
                "rmw_cerulion: CERULION_RMW_SYS_INCLUDE is set but is not valid UTF-8 ({bad:?}) \
                 — fix the path, or unset it to fall back to the vendored bindings"
            )
        });
        let dirs: Vec<PathBuf> = spec
            .split(':')
            .filter(|s| !s.is_empty())
            .map(PathBuf::from)
            .filter(|p| p.is_dir())
            .collect();
        // An EXPLICIT override that resolves to NO existing directory is a
        // misconfiguration (e.g. a typo'd prefix), not a reason to silently
        // fall back to the vendored bindings — which would ship a possibly
        // ABI-mismatched librmw_cerulion.so while the operator believes they
        // built against system headers. Fail the build loudly.
        // The vendored path is reached only when the var is unset.
        assert!(
            !dirs.is_empty(),
            "rmw_cerulion: CERULION_RMW_SYS_INCLUDE is set ('{spec}') but none of its \
             ':'-separated entries is an existing directory. Fix the path(s), or UNSET the \
             variable to fall back to the vendored bindings."
        );
        return dirs;
    }
    let mut dirs = Vec::new();
    if let Some(prefixes) = env::var_os("AMENT_PREFIX_PATH") {
        let prefixes = prefixes.into_string().unwrap_or_else(|bad| {
            panic!(
                "rmw_cerulion: AMENT_PREFIX_PATH is set but is not valid UTF-8 ({bad:?}) — a \
                 prefix list this build cannot read must not silently select the vendored \
                 bindings; fix it or unset it"
            )
        });
        let prefix_list: Vec<&str> = prefixes.split(':').filter(|s| !s.is_empty()).collect();
        // SELECTION, not validation: a prefix whose
        // include/ exists but carries no core ROS package namespace is
        // an unrelated include tree — adding it would select bindgen and
        // PANIC where the docs promise the vendored fallback. A
        // usable prefix's headers still face the fingerprint/claim
        // gates and the era size pins.
        let mut dir_probe = |path: String| PathBuf::from(path).is_dir();
        let (usable, skipped) = era_check::select_ros_prefixes(&prefix_list, &mut dir_probe);
        for prefix in skipped {
            cargo_warning(&format!(
                "rmw_cerulion: AMENT prefix '{prefix}' has an include/ \
                 directory but none of the core ROS package header namespaces \
                 ({core:?}) — skipping it for bindgen (an unrelated include tree must \
                 not select the bindgen path; if no prefix qualifies, the build falls \
                 back to the vendored bindings).",
                core = era_check::CORE_ROS_INCLUDE_PACKAGES
            ));
        }
        for prefix in usable {
            let include = PathBuf::from(prefix).join("include");
            // Modern ament installs headers under per-package dirs
            // (include/rmw/rmw/..., include/rcutils/rcutils/...).
            for pkg in [
                "rmw",
                "rcutils",
                "rosidl_runtime_c",
                "rosidl_typesupport_interface",
                "rosidl_typesupport_introspection_c",
                "rosidl_dynamic_typesupport",
                "type_description_interfaces",
                "service_msgs",
                "builtin_interfaces",
            ] {
                let pkg_dir = include.join(pkg);
                if pkg_dir.is_dir() {
                    dirs.push(pkg_dir);
                }
            }
            dirs.push(include);
        }
    }
    dirs
}

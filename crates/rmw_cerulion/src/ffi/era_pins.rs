// SPDX-License-Identifier: AGPL-3.0-only
//! Per-era struct-size pins keyed on the capability cfgs
//! build.rs derives from the SELECTED bindings (see build.rs).
//!
//! Second layer of the fail-loud ladder, on top of bindgen's own
//! generated layout tests: bindgen pins whatever it generated against
//! itself, while these pins assert the generated layout is the shape
//! the CAPABILITY SET implies. A header/bindings contradiction — e.g. a
//! mixed include path serving Humble's `rmw/types.h` beside Jazzy's
//! introspection headers — trips a pin and dies at `cargo build`
//! instead of shipping a skewed `.so` that SIGSEGVs at the first typed
//! operation (the proven failure shape). The arms
//! deliberately OVERLAP on contradictory cfg sets (e.g. `is_key` without
//! `fetch_function`): two arms then assert two different sizes and the
//! build fails, which is the point.
//!
//! # Scope of the older-era arms
//!
//! Only the arms this repo's builds REACH are executed: the vendored
//! rolling snapshot (every dev build) and the Jazzy container lane
//! (generated bindings). The Foxy/Galactic/Humble/Iron arms and the
//! `kilted` claim-split arm are DERIVED from upstream field lists and do
//! not run in-tree — no pre-Jazzy or Kilted lane exists here. Without
//! one, a pin passing on a dev build says nothing about those eras,
//! and calling them "supported" is a claim the tree cannot back.
//!
//! # Where the expected sizes come from
//!
//! The **rolling shape** (120/64/48, GID 16) is measured: it is what
//! bindgen lays out for the vendored rolling snapshot on an LP64
//! target, byte-pinned by the bindgen layout tests inside
//! `vendored_bindings.rs`. The **older-era sizes** are derived from the
//! per-branch upstream field lists (ros2/rosidl
//! `message_introspection.h`, ros2/rmw `types.h`, ros2/rosidl
//! `message_type_support_struct.h` on the `foxy`/`humble`/`jazzy`/
//! `lyrical` release branches) under the LP64
//! C layout rules every supported target uses; the arithmetic is spelled
//! out per arm below. A per-distro container lane — bindgen against that
//! distro's real headers, these `const _` asserts compiling (or refusing)
//! against the real layout — is the executable proof of an arm;
//! only the Jazzy lane exists (see "Scope of the older-era arms" above).
//!
//! All pins are additionally gated on `target_pointer_width = "64"`
//! (the derivations are LP64; the crate is 64-bit-only anyway via
//! `introspection_cpp`'s `compile_error!`).

use super::{
    rmw_init_options_t, rosidl_message_type_support_t,
    rosidl_typesupport_introspection_c__MessageMember,
    rosidl_typesupport_introspection_c__MessageMembers, RMW_GID_STORAGE_SIZE,
};
use std::mem::size_of;

// =====================================================================
// rosidl_typesupport_introspection_c__MessageMember
// =====================================================================

// Foxy/Galactic era (13 fields; Galactic is byte-identical to Foxy —
// ros2/rosidl#652 landed in Humble). LP64 offsets:
//   name_ 0 · type_id_ 8 · string_upper_bound_ 16 · members_ 24 ·
//   is_array_ 32 · array_size_ 40 · is_upper_bound_ 48 · offset_ 52 ·
//   default_value_ 56 · size_function 64 · get_const_function 72 ·
//   get_function 80 · resize_function 88 → size 96, align 8.
#[cfg(all(target_pointer_width = "64", not(cerulion_has_fetch_function)))]
const _: () = assert!(
    size_of::<rosidl_typesupport_introspection_c__MessageMember>() == 96,
    "MessageMember: capability set says Foxy/Galactic shape (no fetch_function) but the \
     bindings do not lay out 13 fields / 96 bytes — mixed-era headers?"
);

// Humble era (15 fields — +fetch_function/assign_function between
// get_function and resize_function, ros2/rosidl#652):
//   ... get_function 80 · fetch_function 88 · assign_function 96 ·
//   resize_function 104 → size 112.
#[cfg(all(
    target_pointer_width = "64",
    cerulion_has_fetch_function,
    not(cerulion_has_is_key)
))]
const _: () = assert!(
    size_of::<rosidl_typesupport_introspection_c__MessageMember>() == 112,
    "MessageMember: capability set says Humble/Iron shape (fetch_function, no is_key_) but \
     the bindings do not lay out 15 fields / 112 bytes — mixed-era headers?"
);

// Jazzy era (16 fields — is_key_ inserted MID-struct between members_
// and is_array_, ros2/rosidl#796). The size does NOT change: is_key_
// lands in the padding byte at offset 32 and pushes is_array_ to 33,
// leaving every later offset identical to Humble's:
//   members_ 24 · is_key_ 32 · is_array_ 33 · array_size_ 40 · ... ·
//   resize_function 104 → size 112.
// A size pin therefore CANNOT discriminate Humble from Jazzy — the two
// differ in what the byte at offset 32 MEANS, not in size. That axis is
// carried by the capability cfg itself (is_key_ is present in the
// bindings iff the sourced headers declare it) — executed by the Jazzy
// container lane; a Humble lane would execute the other side.
#[cfg(all(
    target_pointer_width = "64",
    cerulion_has_is_key,
    not(cerulion_has_is_rosidl_buffer)
))]
const _: () = assert!(
    size_of::<rosidl_typesupport_introspection_c__MessageMember>() == 112,
    "MessageMember: capability set says Jazzy shape (is_key_, no is_rosidl_buffer_) but the \
     bindings do not lay out 16 fields / 112 bytes — mixed-era headers?"
);

// Lyrical/Rolling era (17 fields — is_rosidl_buffer_ APPENDED after
// resize_function, ros2/rosidl#942): resize_function 104 ·
// is_rosidl_buffer_ 112 → size 120. Matches the vendored rolling
// snapshot's own bindgen layout tests (size 120, align 8).
#[cfg(all(target_pointer_width = "64", cerulion_has_is_rosidl_buffer))]
const _: () = assert!(
    size_of::<rosidl_typesupport_introspection_c__MessageMember>() == 120,
    "MessageMember: capability set says Lyrical/Rolling shape (is_rosidl_buffer_) but the \
     bindings do not lay out 17 fields / 120 bytes — mixed-era headers?"
);

// =====================================================================
// rosidl_typesupport_introspection_c__MessageMembers
// =====================================================================

// Foxy through Humble (7 fields):
//   message_namespace_ 0 · message_name_ 8 · member_count_ 16 (u32) ·
//   size_of_ 24 · members_ 32 · init_function 40 · fini_function 48
//   → size 56.
#[cfg(all(target_pointer_width = "64", not(cerulion_has_any_key_member)))]
const _: () = assert!(
    size_of::<rosidl_typesupport_introspection_c__MessageMembers>() == 56,
    "MessageMembers: capability set says pre-Jazzy shape (no has_any_key_member_) but the \
     bindings do not lay out 7 fields / 56 bytes — mixed-era headers?"
);

// Jazzy and later (8 fields — has_any_key_member_ inserted MID-struct
// between size_of_ and members_, ros2/rosidl#796; unchanged since):
//   size_of_ 24 · has_any_key_member_ 32 · members_ 40 ·
//   init_function 48 · fini_function 56 → size 64. Matches the vendored
//   rolling snapshot's bindgen layout tests.
#[cfg(all(target_pointer_width = "64", cerulion_has_any_key_member))]
const _: () = assert!(
    size_of::<rosidl_typesupport_introspection_c__MessageMembers>() == 64,
    "MessageMembers: capability set says Jazzy+ shape (has_any_key_member_) but the bindings \
     do not lay out 8 fields / 64 bytes — mixed-era headers?"
);

// =====================================================================
// rosidl_message_type_support_t (era coherence: type-hash trio, Iron+)
// =====================================================================

// Foxy/Humble (3 fields): typesupport_identifier 0 · data 8 · func 16
// → size 24.
#[cfg(all(target_pointer_width = "64", not(cerulion_has_type_hash)))]
const _: () = assert!(
    size_of::<rosidl_message_type_support_t>() == 24,
    "rosidl_message_type_support_t: capability set says pre-Iron shape (no \
     get_type_hash_func) but the bindings do not lay out 3 fields / 24 bytes"
);

// Iron and later (6 fields — +get_type_hash_func /
// get_type_description_func / get_type_description_sources_func at
// 24/32/40) → size 48. Matches the vendored rolling snapshot.
#[cfg(all(target_pointer_width = "64", cerulion_has_type_hash))]
const _: () = assert!(
    size_of::<rosidl_message_type_support_t>() == 48,
    "rosidl_message_type_support_t: capability set says Iron+ shape (get_type_hash_func) \
     but the bindings do not lay out 6 fields / 48 bytes"
);

// =====================================================================
// RMW_GID_STORAGE_SIZE (era coherence: 24 pre-Iron, 16 Iron+)
// =====================================================================

// The GID shrink and the typesupport type-hash trio both landed in
// Iron, so keying the width on `cerulion_has_type_hash` cross-checks
// two independent axes of the same era — a header set mixing them dies
// here. (ros2/rmw types.h per branch: 24u on foxy/galactic/humble, 16u
// on iron/jazzy/lyrical/rolling.)
#[cfg(not(cerulion_has_type_hash))]
const _: () = assert!(
    RMW_GID_STORAGE_SIZE == 24,
    "RMW_GID_STORAGE_SIZE: pre-Iron capability set implies 24"
);
#[cfg(cerulion_has_type_hash)]
const _: () = assert!(
    RMW_GID_STORAGE_SIZE == 16,
    "RMW_GID_STORAGE_SIZE: Iron+ capability set implies 16"
);

// Every era's storage holds the 16 deterministic bytes this crate
// writes (`g.data[..16]` in api/misc.rs and api/services.rs) — the
// write is in-bounds on 16-wide AND 24-wide GIDs.
const _: () = assert!(
    RMW_GID_STORAGE_SIZE >= 16,
    "rmw_gid_t cannot hold the 16 gid bytes this crate writes"
);

// =====================================================================
// rmw_init_options_t (mixed-include-root defense)
// =====================================================================

// `probe_headers` answers "will the gated `#include` RESOLVE?", which
// matches clang's ordered `-I` search on existence. What it cannot see
// is a MIXED include tree: an early `-I` dir serving a pre-Iron
// `rmw/init_options.h` while a later dir satisfies the
// discovery-options probe — the fingerprint then implies a layout the
// bindings do not have. These pins make that pairing die at build.
// LP64 arithmetic from the per-release-branch headers:
//   pre-Iron (no discovery_options): instance_id 0 · impl_id 8 ·
//     domain_id 16 · security_options 24 (16 B) · localhost_only 40 ·
//     enclave 48 · allocator 56 (40 B) · impl 96 → size 104.
//   iron/jazzy (+64-byte discovery_options after localhost_only):
//     discovery 48 · enclave 112 · allocator 120 · impl 160 → 168.
//   kilted/lyrical/rolling (localhost_only REMOVED): discovery 40 ·
//     enclave 104 · allocator 112 · impl 152 → size 160 (matches the
//     vendored rolling bindgen pin).
// The middle arm is CLAIM-SPLIT (one arm admitting
// BOTH 168 and 160 would let an Iron claim admit the Kilted layout: the
// same 168/160 class the claim layer separates, resurfacing at the
// size-pin layer). The baked claim is a COMPILE-TIME constant
// (`era::BUILT_FOR_DISTRO`), and by the time rustc evaluates these
// pins, build.rs has already verified/derived it against the
// fingerprint AND the probed init-options size — so the only claims
// that can reach this arm are iron/jazzy (168) and kilted (160), and
// the pin asserts the claim's SPECIFIC size
// (`init_options_size_for_claim`; any other claim here fails loud,
// per the overlapping-arms pattern). A pre-Iron 104-byte struct under
// a discovery-bearing fingerprint — the actual mixed-root hazard —
// fails either discovery arm.
#[cfg(all(target_pointer_width = "64", not(cerulion_has_discovery_options)))]
const _: () = assert!(
    size_of::<rmw_init_options_t>() == 104,
    "rmw_init_options_t: capability set says pre-Iron (no discovery_options) but the bindings \
     do not lay out the 104-byte shape — mixed include roots?"
);

/// Claim → the `rmw_init_options_t` size that claim's distro lays out,
/// for every claim that can legitimately reach the
/// discovery-without-buffer pin arm. `None` = a claim that should be
/// impossible there (build.rs bakes only iron/jazzy/kilted for
/// iron-era fingerprints) — the pin fails loud on it.
#[allow(dead_code)] // referenced by the cfg-gated pin arm (inactive on
                    // vendored/rolling builds, where is_rosidl_buffer is set) and by the
                    // oracle tests below — never dead on a build that needs it.
pub(crate) const fn init_options_size_for_claim(claim: &str) -> Option<usize> {
    if const_str_eq(claim, "iron") || const_str_eq(claim, "jazzy") {
        Some(168)
    } else if const_str_eq(claim, "kilted") {
        Some(160)
    } else {
        None
    }
}

/// Const string equality (`==` on `&str` is not const-stable).
#[allow(dead_code)] // reachable only through init_options_size_for_claim.
const fn const_str_eq(a: &str, b: &str) -> bool {
    let (a, b) = (a.as_bytes(), b.as_bytes());
    if a.len() != b.len() {
        return false;
    }
    let mut i = 0;
    while i < a.len() {
        if a[i] != b[i] {
            return false;
        }
        i += 1;
    }
    true
}

#[cfg(all(
    target_pointer_width = "64",
    cerulion_has_discovery_options,
    not(cerulion_has_is_rosidl_buffer)
))]
const _: () = {
    let expected = init_options_size_for_claim(crate::era::BUILT_FOR_DISTRO);
    assert!(
        expected.is_some(),
        "rmw_init_options_t pin: an iron-era capability set carries a baked claim outside \
         iron/jazzy/kilted — the claim layer should have refused this build"
    );
    let expected = match expected {
        Some(size) => size,
        None => 0,
    };
    assert!(
        size_of::<rmw_init_options_t>() == expected,
        "rmw_init_options_t: the bindings lay out a DIFFERENT init-options size than the baked \
         claim's distro requires (iron/jazzy = 168, kilted = 160) — mixed include roots or a \
         mislabeled claim"
    );
};
#[cfg(all(target_pointer_width = "64", cerulion_has_is_rosidl_buffer))]
const _: () = assert!(
    size_of::<rmw_init_options_t>() == 160,
    "rmw_init_options_t: capability set says Lyrical/Rolling but the bindings do not lay out \
     the 160-byte shape (localhost_only removed) — mixed include roots?"
);

#[cfg(test)]
mod tests {
    use super::init_options_size_for_claim;

    #[test]
    fn the_divergent_pin_arm_expects_the_claims_specific_size() {
        // A 168-OR-160 disjunction on the discovery-without-buffer arm
        // would let an Iron claim admit the Kilted layout — the
        // pin asserts the claim's SPECIFIC size, so collapsing the
        // distinction fails here.
        assert_eq!(init_options_size_for_claim("iron"), Some(168));
        assert_eq!(init_options_size_for_claim("jazzy"), Some(168));
        assert_eq!(
            init_options_size_for_claim("kilted"),
            Some(160),
            "kilted dropped localhost_only — 160, never 168"
        );
        // Claims that cannot legitimately reach the arm fail the pin
        // loud rather than borrowing either size.
        for impossible in [
            "humble",
            "lyrical",
            "rolling",
            "era:jazzy",
            "vendored-dev",
            "",
        ] {
            assert_eq!(
                init_options_size_for_claim(impossible),
                None,
                "claim {impossible:?} must fail the pin, not pick a size"
            );
        }
    }
}

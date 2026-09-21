//! The header-provenance check for a generated distro
//! claim — a PURE classifier shared, as a `#[path]` module, between build.rs
//! (which enforces it) and the crate (whose oracle-vector tests pin it;
//! `#[cfg(test)]` strips the tests from the build-script copy).
//!
//! `ROS_DISTRO` alone is not proof of the generated headers' distro: the
//! env can say `jazzy` while `AMENT_PREFIX_PATH` / `CERULION_RMW_SYS_INCLUDE`
//! supplies Humble headers — `rmw_init`'s guard then sees matching
//! strings and admits a layout-incompatible `.so`, defeated from the
//! inside. But the build already computes a header-DERIVED fact: the
//! capability fingerprint (the 15 marker tokens grepped from the
//! generated bindings). This module maps a claimed distro to the
//! capability subset its era must produce — the same era boundaries the
//! size pins in `ffi/era_pins.rs` use, verified against the per-branch
//! upstream headers — and reports a contradiction
//! when the observed fingerprint disagrees. build.rs FAILS THE BUILD on
//! a contradiction (compile-time-prevention-first, strictest default).
//!
//! # Boundaries — what capabilities can and cannot distinguish
//!
//! Capabilities distinguish ERAS, not every adjacent distro:
//!
//! - **Indistinguishable pairs**: `jazzy` ↔ `kilted` share one
//!   fingerprint (kilted added none of our 15 markers), and `lyrical` ↔
//!   `rolling` are identical today. `lyrical` ↔ `rolling` genuinely
//!   agree on every pinned axis. `jazzy` ↔ `kilted` agree on all but
//!   ONE: kilted dropped `localhost_only` from `rmw_init_options_t`
//!   (168 → 160 bytes), which the fingerprint cannot see — the
//!   init-options pin asserts the CLAIM-specific size on that arm
//!   (the baked claim is a compile-time constant, so an
//!   Iron claim cannot admit the Kilted layout). An era LABEL
//!   therefore admits exactly the members whose pinned layouts are
//!   IDENTICAL: `era:jazzy` is never baked (the bindings' own
//!   init-options layout test — [`probe_init_options_size`] — bakes
//!   the concrete member or fails the build), and an env-set
//!   jazzy/kilted claim is CROSS-CHECKED against that size
//!   ([`init_options_size_contradicts`]), closing the
//!   env-lie hole — and an UNREADABLE size does not admit a
//!   divergent-pair claim either ([`divergent_claim_unverifiable`]
//!   fails the build; a bindgen layout-test format change refuses
//!   jazzy/kilted claims until the probe learns the new spelling,
//!   while unambiguous claims stay non-fatal). Kilted remains outside
//!   the support matrix. If rolling later drifts past Lyrical, the new marker
//!   lands in the capability table and the pair separates.
//! - **`foxy` ↔ `galactic`** ARE separated (galactic adds
//!   `qos_compatibility`/`message_lost_event`/`network_flow`), and their
//!   layouts agree on every pinned axis anyway.
//! - **`rolling` means CURRENT upstream** (the Lyrical-era set): an
//!   outdated rolling source tree fails the check as a Jazzy-era
//!   contradiction — deliberate; the failure message's remediation
//!   applies.
//! - An era-INTERIOR mispairing that slips a same-size boundary is
//!   caught by the size pins where sizes differ (e.g. Humble↔Iron via
//!   the typesupport 24→48), and only genuinely layout-agreeing pairs
//!   remain admissible.

/// Era ranks (ordered). `kilted` shares the Jazzy era; `rolling` shares
/// the Lyrical era (current upstream).
pub const ERA_FOXY: usize = 0;
pub const ERA_GALACTIC: usize = 1;
pub const ERA_HUMBLE: usize = 2;
pub const ERA_IRON: usize = 3;
pub const ERA_JAZZY: usize = 4;
pub const ERA_LYRICAL: usize = 5;

/// The era of the VENDORED bindings snapshot. The unclaimed
/// `vendored-dev` marker is NOT "no information" — the
/// snapshot is the pinned ROLLING-era ABI (its fingerprint is the full
/// Lyrical capability set; `ffi/era_pins.rs` pins its 160-byte
/// `rmw_init_options_t` and 120-byte `MessageMember`), so at runtime it
/// admits exactly this era's layout-identical members (`lyrical`,
/// `rolling`) and refuses every other NAMED distro. Pinned to the
/// table by `the_vendored_snapshot_era_is_the_lyrical_era`, and to the
/// snapshot's OWN baked fingerprint by era.rs's
/// `the_capability_fingerprint_names_exactly_the_cfgs_this_build_carries`
/// (a re-snapshot that moves era fails there).
#[allow(dead_code)] // runtime-side only; the build-script copy bakes claims, never compares them.
pub const VENDORED_SNAPSHOT_ERA_TOKEN: &str = "lyrical";

/// Era rank → human label for diagnostics.
pub const ERA_NAMES: &[&str] = &[
    "Foxy",
    "Galactic",
    "Humble",
    "Iron",
    "Jazzy/Kilted",
    "Lyrical/Rolling",
];

/// Capability cfg suffix → the FIRST era whose headers produce its
/// marker token (verified per release branch
/// against the raw upstream headers; same boundaries as the era-pin
/// table in `ffi/era_pins.rs` and the build.rs `CAPABILITIES` table).
pub const CAPABILITY_MIN_ERA: &[(&str, usize)] = &[
    ("qos_compatibility", ERA_GALACTIC),
    ("message_lost_event", ERA_GALACTIC),
    ("network_flow", ERA_GALACTIC),
    ("fetch_function", ERA_HUMBLE),
    ("content_filter_options", ERA_HUMBLE),
    ("event_callback", ERA_HUMBLE),
    ("message_info_sequence_numbers", ERA_HUMBLE),
    ("features", ERA_HUMBLE),
    ("discovery_options", ERA_IRON),
    ("matched_events", ERA_IRON),
    ("type_hash", ERA_IRON),
    ("is_key", ERA_JAZZY),
    ("any_key_member", ERA_JAZZY),
    ("is_rosidl_buffer", ERA_LYRICAL),
    ("event_type_max", ERA_LYRICAL),
];

/// Known distro names → era rank. Includes out-of-matrix distros
/// (galactic, iron, kilted): this is provenance verification, not
/// support policy.
pub const DISTRO_ERAS: &[(&str, usize)] = &[
    ("foxy", ERA_FOXY),
    ("galactic", ERA_GALACTIC),
    ("humble", ERA_HUMBLE),
    ("iron", ERA_IRON),
    ("jazzy", ERA_JAZZY),
    ("kilted", ERA_JAZZY),
    ("lyrical", ERA_LYRICAL),
    ("rolling", ERA_LYRICAL),
];

/// The ONE claim normalization (trim + ASCII lowercase), shared by the
/// classifier, by build.rs's bake, and by era.rs's runtime comparison —
/// if the classifier normalized for CLASSIFICATION while build.rs
/// baked the ORIGINAL env string, `ROS_DISTRO=JAZZY` at build would pass
/// the check, bake "JAZZY", and then fail the case-sensitive runtime
/// comparison against a conventionally-sourced `jazzy` — refusing a
/// compatible library. What the checker judges is what gets baked, and
/// both comparison operands go through this same function.
pub fn normalize_distro_claim(claimed: &str) -> String {
    claimed.trim().to_ascii_lowercase()
}

/// The RESERVED unclaimed marker — the ONE value era.rs's runtime guard
/// treats as "the vendored snapshot" (admitted under that snapshot's
/// own era — [`VENDORED_SNAPSHOT_ERA_TOKEN`] membership — or an absent
/// `ROS_DISTRO`; refused under every other name). Exactly ONE producer
/// may bake it: the VENDORED
/// path (a GENERATED distro-less build must not ride it
/// too, which would let one distro's real ABI run everywhere — it bakes a
/// fingerprint-derived claim, [`era_claim_for_observed`], or fails the
/// build). A generated build whose env EXPLICITLY normalizes
/// to it is forging the marker — a `ROS_DISTRO=VENDORED-DEV` that
/// slipped through UnknownDistro would get baked, and the guard would then
/// admit a distro-specific generated ABI under ANY runtime distro.
/// [`check_distro_claim`] refuses it (and the empty-after-trim claim,
/// whose baked form era.rs also treats as unclaimed) before any era
/// lookup, so every caller inherits the rule.
pub const RESERVED_UNCLAIMED_MARKER: &str = "vendored-dev";

/// The package header namespaces `wrapper.h` cannot build without —
/// the load-bearing marker set for AMENT prefix selection. A prefix
/// whose `include/` carries NONE of these is an unrelated include tree
/// (added unconditionally it would select the
/// bindgen path and PANIC when the ROS headers are absent, while
/// the docs promise the vendored fallback). Deliberately a SET, not a
/// single `rmw/rmw.h` marker: an ISOLATED colcon workspace puts each
/// package in its OWN prefix, so the rcutils / rosidl prefixes carry no
/// rmw headers at all — a single-header marker would skip them and lose
/// the very headers bindgen needs.
pub const CORE_ROS_INCLUDE_PACKAGES: &[&str] = &[
    "rmw",
    "rcutils",
    "rosidl_runtime_c",
    "rosidl_typesupport_interface",
    "rosidl_typesupport_introspection_c",
];

/// Partition AMENT prefixes for bindgen selection — SELECTION only,
/// never validation: a usable prefix's headers still go through the
/// fingerprint/claim gates and the era size pins, so a marker-bearing
/// prefix with mismatched-era headers is caught by them.
///
/// Three-way classification per prefix, probed via the injected
/// `dir_exists` (paths are `<prefix>/include` and
/// `<prefix>/include/<pkg>` — the `<pkg>` namespace is a directory in
/// BOTH ament layouts: per-package nests `<pkg>/<pkg>/*.h` beneath it,
/// flat holds `<pkg>/*.h` directly):
/// - no `include/` at all ⇒ silently dropped (pure-python / launch
///   package prefixes — warning on these would spam every build);
/// - `include/` present but NO core package namespace ⇒ SKIPPED, second
///   tuple element (the caller warns, naming the prefix);
/// - `include/` with at least one core namespace ⇒ USABLE, first
///   element. When NOTHING is usable the caller's include set is empty
///   and the build takes the documented vendored-fallback warning path.
pub fn select_ros_prefixes<'a>(
    prefixes: &[&'a str],
    dir_exists: &mut dyn FnMut(String) -> bool,
) -> (Vec<&'a str>, Vec<&'a str>) {
    let mut usable = Vec::new();
    let mut skipped = Vec::new();
    for prefix in prefixes {
        if !dir_exists(format!("{prefix}/include")) {
            continue;
        }
        if CORE_ROS_INCLUDE_PACKAGES
            .iter()
            .any(|pkg| dir_exists(format!("{prefix}/include/{pkg}")))
        {
            usable.push(*prefix);
        } else {
            skipped.push(*prefix);
        }
    }
    (usable, skipped)
}

/// A capability header found under an include root OTHER than the one
/// that serves the anchor header — a MIXED include tree, refused rather
/// than fingerprinted (a plain existence probe would enable a
/// capability if ANY `-I` dir had its header, while bindgen resolves the
/// anchor `rmw/init_options.h` by FIRST match; an older root first and a
/// newer root later would then keep a pre-Iron `rmw_init_options_t` beside an
/// Iron `discovery_options` capability, and the fingerprint would bake an
/// `iron` claim an Iron runtime admits — the jazzy/kilted size table
/// cannot see it, and the MessageMember pins are the same size for
/// Humble and Iron).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CapabilityProbeRefusal {
    /// No include dir serves the anchor header at all.
    NoAnchor { anchor: String },
    /// `capability` resolves only under `found_in`, not under
    /// `anchor_root`, the root clang takes the anchor from.
    MixedRoots {
        anchor_root: std::path::PathBuf,
        capability: &'static str,
        found_in: std::path::PathBuf,
    },
}

/// Resolve the capability headers with clang's provenance: the anchor
/// header's FIRST-match root is the one every capability must come from.
/// `exists` is the filesystem (injected so the decision is testable
/// without one). A capability present in the anchor root is enabled; one
/// present ONLY in a later root is a mixed tree and refuses the build.
pub fn resolve_capability_headers(
    include_dirs: &[std::path::PathBuf],
    anchor: &str,
    probes: &[(&'static str, &'static str)],
    exists: impl Fn(&std::path::Path) -> bool,
) -> Result<Vec<&'static str>, CapabilityProbeRefusal> {
    let Some(anchor_root) = include_dirs.iter().find(|dir| exists(&dir.join(anchor))) else {
        return Err(CapabilityProbeRefusal::NoAnchor {
            anchor: anchor.to_string(),
        });
    };
    let mut enabled = Vec::new();
    for (define, relative) in probes {
        // EVERY header resolves by its own first match, the way clang
        // walks `-I` (checking the anchor root
        // first would accept its copy even when an EARLIER root also carried
        // the header — the copy clang actually compiles). Enabled iff the
        // capability's first-match root IS the anchor's; any other root,
        // earlier or later, is a mixed tree.
        match include_dirs.iter().find(|dir| exists(&dir.join(relative))) {
            None => {}
            Some(first) if first == anchor_root => enabled.push(*define),
            Some(first) => {
                return Err(CapabilityProbeRefusal::MixedRoots {
                    anchor_root: anchor_root.clone(),
                    capability: define,
                    found_in: first.clone(),
                });
            }
        }
    }
    Ok(enabled)
}

/// Render `message` safe for cargo's DIRECTIVE channel (an injection
/// hazard otherwise): every control character becomes a visible
/// `\u{..}` escape, everything else is byte-identical.
///
/// A build script's STDOUT is not a log — cargo parses each `cargo:` line
/// as a directive, and `build.rs`'s warnings interpolate values that come
/// from the ENVIRONMENT (`AMENT_PREFIX_PATH` entries, `ROS_DISTRO`), so a
/// newline in one splits the warning and hands cargo a line the operator
/// never wrote.
///
/// IMPACT, measured rather than assumed: with the
/// CURRENT feeds a complete `cargo:rustc-cfg=…` cannot be forged this
/// way, because both prefix variables are split on `':'` before a value
/// is interpolated — so `AMENT_PREFIX_PATH=$'/x\ncargo:rustc-cfg=y'`
/// arrives as the two entries `/x\ncargo` and `rustc-cfg=y`, and the
/// injected line lacks the `cargo:` prefix a directive needs. What IS
/// demonstrable is a MANGLED, multi-line warning. Escaping is done at the
/// CHANNEL anyway, and deliberately: the hazard is real in
/// principle, the `':'` split is an accident of two variables' formats
/// rather than a guarantee anyone stated, `ROS_DISTRO` needs its own
/// refusal at the READ for the same reason, and a
/// future feed (a package name, a path from a config file) carries no
/// such accident. Escaping here covers every present and future one, and
/// renders a control character in a genuine path visibly rather than
/// dropping the warning.
///
/// Lives here rather than in `build.rs` because a build script has no
/// test harness, and an untested escaper is how this class comes back;
/// `build.rs` `#[path]`-includes this file.
pub fn escape_for_cargo_directive(message: &str) -> String {
    escape_control_chars(message)
}

/// Render every control character in `s` visibly as `\u{..}` — the ONE
/// escaper behind both the cargo-directive channel above and the runtime
/// log lines that echo an env-derived value (`ROS_DISTRO`
/// reaches the mismatch refusal through `to_string_lossy`,
/// so a newline in it would forge a second log record). A `%` render of any
/// env-derived string goes through here.
pub fn escape_control_chars(message: &str) -> String {
    let mut out = String::with_capacity(message.len());
    for c in message.chars() {
        // `is_control` covers CR and LF — the directive separators — plus
        // every other C0/C1 code point.
        if c.is_control() {
            out.push_str(&format!("\\u{{{:x}}}", c as u32));
        } else {
            out.push(c);
        }
    }
    out
}

/// A claim build.rs may BAKE: already through
/// [`normalize_distro_claim`], and NOT the reserved unclaimed marker,
/// NOT empty and NOT an env-supplied `era:` label. Two classes (a raw env
/// string baked; a reserved marker forged from a generated
/// build) would otherwise each depend on a `build.rs` arm remembering to refuse them;
/// the field is private and the only constructors are inside the two
/// classifiers below.
///
/// The type is a GATE, not a label, only because build.rs writes
/// `CERULION_RMW_BUILT_FOR_DISTRO` at exactly ONE site — its
/// `bake_distro_claim`, whose `BakedDistroClaim` argument admits a
/// `&BakeableClaim` or the vendored path's reserved marker and nothing
/// else. The newtype alone does not do it: when the bake is a bare
/// `println!("…={claim}")`, every `String` is equally `Display`
/// and nothing is gated.
/// [`DistroClaimVerdict::ReservedCollision`], `Contradiction` and
/// `UnknownDistro` carry plain `String`s, so their payloads reach no
/// bake site.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BakeableClaim(String);

impl BakeableClaim {
    /// Only the classifiers construct one: after the reserved/empty/
    /// `era:` refusals in [`check_distro_claim`], or from the era table
    /// itself in [`era_claim_for_observed`].
    fn vetted(claim: String) -> Self {
        Self(claim)
    }
}

impl AsRef<str> for BakeableClaim {
    fn as_ref(&self) -> &str {
        &self.0
    }
}

impl std::fmt::Display for BakeableClaim {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

/// Verdict of [`check_distro_claim`]. The bakeable `normalized` payloads
/// are [`BakeableClaim`]s — never the raw env string; the
/// refusing variants carry plain `String`s that no bake site accepts.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DistroClaimVerdict {
    /// The observed fingerprint is exactly what the claimed distro's
    /// era produces — bake the NORMALIZED claim.
    Consistent {
        /// The normalized claim to bake.
        normalized: BakeableClaim,
    },
    /// The claimed name is not in [`DISTRO_ERAS`] (a future distro or a
    /// typo): no expectation is derivable from the NAME, so build.rs
    /// hands it to [`claim_for_unknown_distro`], which ignores it and
    /// derives the claim from the fingerprint + layout exactly as the
    /// no-env path does (baking `jazzyy` would let a same-typo
    /// runtime admit an unvalidated ABI). Refusing outright would break
    /// every future distro until this table learns its name, so the
    /// build proceeds with a loud `cargo:warning` naming both the
    /// unknown name and the DERIVED claim.
    ///
    /// The payload is a plain `String`, NOT a [`BakeableClaim`]:
    /// this is the one normalized claim that must
    /// never be baked, so it must not carry the bake-permitting type.
    UnknownDistro {
        /// The normalized claim — reported to the operator, never baked.
        normalized: String,
    },
    /// The claim normalizes to the RESERVED unclaimed marker
    /// ([`RESERVED_UNCLAIMED_MARKER`]) or to the empty string — values
    /// the runtime guard treats as "no claim". Baking either from a
    /// claimed generated build would bypass the guard entirely, so
    /// build.rs fails the build naming the collision.
    ReservedCollision {
        /// What the claim normalized to ("vendored-dev" or "").
        normalized: String,
    },
    /// The observed fingerprint contradicts the claimed distro's era —
    /// build.rs fails the build.
    Contradiction {
        /// The (normalized) claimed distro name.
        claimed: String,
        /// Every capability the claimed era must produce.
        expected: Vec<&'static str>,
        /// Every KNOWN capability observed in the bindings (unknown
        /// future tokens are ignored — no expectation exists for them).
        observed: Vec<&'static str>,
        /// Expected but absent — the headers are OLDER than the claim.
        missing: Vec<&'static str>,
        /// Present but above the claimed era — the headers are NEWER
        /// than the claim.
        unexpected: Vec<&'static str>,
    },
}

/// Era rank for a known distro name (input must already be normalized).
fn distro_era_rank(distro: &str) -> Option<usize> {
    DISTRO_ERAS
        .iter()
        .find(|(name, _)| *name == distro)
        .map(|(_, rank)| *rank)
}

/// The era rank an observed fingerprint matches EXACTLY, if any.
pub fn observed_era_rank(observed: &[&str]) -> Option<usize> {
    (ERA_FOXY..=ERA_LYRICAL).find(|rank| {
        CAPABILITY_MIN_ERA
            .iter()
            .all(|(cap, min)| (*min <= *rank) == observed.contains(cap))
    })
}

/// Human label for the era an observed fingerprint matches exactly, if
/// any — for the failure message ("the headers look like `<era>`").
pub fn observed_era_label(observed: &[&str]) -> Option<&'static str> {
    observed_era_rank(observed).map(|rank| ERA_NAMES[rank])
}

/// Prefix of a fingerprint-DERIVED era claim (`era:jazzy`,
/// `era:lyrical`). Baked by build.rs for a GENERATED build whose
/// environment named no distro (baking the unclaimed
/// marker there would let one distro's real generated ABI run under EVERY
/// runtime distro — the dev seam is meant for the vendored snapshot,
/// and a headers-generated build has real provenance). The namespace is
/// RESERVED like the unclaimed marker: an env claim spelling it is
/// refused by [`check_distro_claim`], so it cannot be forged.
pub const ERA_CLAIM_PREFIX: &str = "era:";

/// Canonical claim token per era rank (used in `era:<token>` claims).
const ERA_CLAIM_TOKENS: &[&str] = &["foxy", "galactic", "humble", "iron", "jazzy", "lyrical"];

// The rank tables are indexed by the `ERA_*` constants; a table that is
// shorter than the highest rank panics at the index, so pin the lengths.
const _: () = assert!(ERA_NAMES.len() == ERA_LYRICAL + 1);
const _: () = assert!(ERA_CLAIM_TOKENS.len() == ERA_LYRICAL + 1);

/// The `rmw_init_options_t` size the bindings actually lay out, read
/// from bindgen's OWN layout test — the discriminator the capability
/// fingerprint lacks (jazzy and kilted share
/// one fingerprint while their init-options layouts differ, 168 vs 160
/// — kilted dropped `localhost_only` — and the claim IS the guard's
/// input at every guarded export, so an ambiguous claim could refuse a
/// wrong-layout write at none of them: only a build-time decision is
/// sound).
/// Parses both bindgen layout-test spellings — the const form
/// (`size_of::<rmw_init_options_s>() - 160usize`, bindgen 0.71+, the
/// vendored snapshot) and the assert form
/// (`size_of::<rmw_init_options_s>(), 168usize`, bindgen 0.70, the
/// generated path) — and the pre-Galactic `_t` tag. `None` when no
/// layout test is found (a bindgen format change): the caller treats
/// the pair as UNDISAMBIGUATED and fails the build rather than guess.
pub fn probe_init_options_size(bindings: &str) -> Option<usize> {
    for tag in ["rmw_init_options_s", "rmw_init_options_t"] {
        let needle = format!("size_of::<{tag}>()");
        let mut search = bindings;
        while let Some(at) = search.find(&needle) {
            let rest = &search[at + needle.len()..];
            let rest = rest.trim_start();
            if let Some(rest) = rest.strip_prefix(['-', ',']) {
                let rest = rest.trim_start();
                let digits: String = rest.chars().take_while(char::is_ascii_digit).collect();
                if !digits.is_empty() && rest[digits.len()..].starts_with("usize") {
                    // A digit run that does not fit a `usize` cannot be a
                    // struct size; keep SEARCHING rather than returning
                    // `None`, so a malformed first hit
                    // cannot mask a good later one.
                    if let Ok(size) = digits.parse() {
                        return Some(size);
                    }
                }
            }
            search = &search[at + needle.len()..];
        }
    }
    None
}

/// Expected `rmw_init_options_t` size for the claims the fingerprint
/// alone cannot verify — the jazzy/kilted layout-divergent pair. Used
/// both to DISAMBIGUATE a no-env fingerprint into a concrete claim and
/// to CROSS-CHECK an env-set claim against the bindings (closing the
/// jazzy↔kilted env-lie hole whenever the size is
/// readable).
const CLAIM_INIT_OPTIONS_SIZE: &[(&str, usize)] = &[("jazzy", 168), ("kilted", 160)];

/// A jazzy/kilted claim whose bindings lay out a DIFFERENT
/// `rmw_init_options_t` size than the claim requires:
/// `Some((expected, got))` — build.rs fails the build naming both.
/// `None` when the claim is not in the divergent pair, or the size was
/// unreadable — the unreadable-size case is NOT admitted for divergent
/// claims: [`divergent_claim_unverifiable`] refuses it separately.
pub fn init_options_size_contradicts(claim: &str, probed: Option<usize>) -> Option<(usize, usize)> {
    let expected = CLAIM_INIT_OPTIONS_SIZE
        .iter()
        .find(|(c, _)| *c == claim)
        .map(|(_, size)| *size)?;
    let got = probed?;
    (got != expected).then_some((expected, got))
}

/// Outcome of deriving the claim for a GENERATED build with NO env
/// distro.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GeneratedClaim {
    /// Bake this claim: a concrete distro name, or an era label whose
    /// members' pinned layouts are verified IDENTICAL.
    Bake(BakeableClaim),
    /// The fingerprint matches an era whose members' LAYOUTS DIFFER
    /// (jazzy/kilted) and the bindings' init-options size disambiguated
    /// nothing — build.rs FAILS the build: an ambiguous claim admits an
    /// ABI break in one direction or the other, and the claim is the
    /// guard's input at every guarded export, so nothing downstream can
    /// refuse what the claim admits.
    AmbiguousEra {
        /// The era's claim token.
        era_label: &'static str,
        /// The fingerprint-identical members the size could not separate.
        members: Vec<&'static str>,
        /// What the layout-test probe read (None = not found).
        init_options_size: Option<usize>,
    },
    /// No known era — build.rs fails the build.
    NoKnownEra,
}

/// The claim build.rs bakes for a GENERATED build with NO env distro:
/// derived from the fingerprint, DISAMBIGUATED by layout where the
/// fingerprint is not enough. The concrete distro name is baked when
/// the matched era has exactly ONE member — or when the bindings' own
/// `rmw_init_options_t` layout test separates the jazzy/kilted pair
/// (168 ⇒ jazzy, 160 ⇒ kilted: `era:jazzy` is NEVER baked, because its
/// members' layouts differ and the label would admit an ABI break).
/// The `era:<token>` label is baked only where every member's pinned
/// layout is verified IDENTICAL — lyrical/rolling, byte-identical
/// (verified 2026-09-02 on the release branches) across
/// `message_introspection.h`, `rmw/types.h`, `rmw/init_options.h`,
/// `rmw/discovery_options.h`, `rmw/security_options.h` and
/// `message_type_support_struct.h`.
pub fn era_claim_for_observed(
    observed: &[&str],
    init_options_size: Option<usize>,
) -> GeneratedClaim {
    let Some(rank) = observed_era_rank(observed) else {
        return GeneratedClaim::NoKnownEra;
    };
    let members: Vec<&'static str> = DISTRO_ERAS
        .iter()
        .filter(|(_, r)| *r == rank)
        .map(|(name, _)| *name)
        .collect();
    if let [only] = members.as_slice() {
        return GeneratedClaim::Bake(BakeableClaim::vetted((*only).to_string()));
    }
    if rank == ERA_JAZZY {
        // jazzy/kilted: fingerprint-identical, layout-DIVERGENT.
        for (claim, size) in CLAIM_INIT_OPTIONS_SIZE {
            if init_options_size == Some(*size) {
                return GeneratedClaim::Bake(BakeableClaim::vetted((*claim).to_string()));
            }
        }
        return GeneratedClaim::AmbiguousEra {
            era_label: ERA_CLAIM_TOKENS[rank],
            members,
            init_options_size,
        };
    }
    GeneratedClaim::Bake(BakeableClaim::vetted(format!(
        "{ERA_CLAIM_PREFIX}{}",
        ERA_CLAIM_TOKENS[rank]
    )))
}

/// The claim for an env-set `ROS_DISTRO` the era table does NOT know
/// (baking the unknown name
/// AS-IS would bypass every ABI check — a Kilted-shaped binding set with
/// `ROS_DISTRO=jazzyy` would bake "jazzyy" and a same-typo runtime would admit
/// it with the ABI never validated for that identity). The unknown
/// NAME deliberately does not influence the claim — that IS the
/// bypass; the `_normalized` parameter exists to make that contract
/// explicit and checkable by tests. The claim comes from the SAME
/// fingerprint + layout machinery as the no-env path: a recognized,
/// disambiguated fingerprint bakes the derived claim (the runtime
/// guard then admits only its layout-identical members — a genuinely
/// NEW distro therefore needs the era table extended before its own
/// name admits, the strictest default rather than a
/// bake-as-is posture); an ambiguous or unrecognized fingerprint
/// FAILS the build via the caller.
pub fn claim_for_unknown_distro(
    _normalized: &str,
    observed: &[&str],
    init_options_size: Option<usize>,
) -> GeneratedClaim {
    era_claim_for_observed(observed, init_options_size)
}

/// A jazzy/kilted claim whose bindings carry NO
/// readable `rmw_init_options_t` layout test is UNVERIFIABLE — the
/// size is the ONLY cross-check separating the fingerprint-identical
/// pair, so "no judgement" there would admit an unvalidated
/// ABI. True exactly when the claim is in
/// the divergent pair and the probe read nothing; the caller FAILS the
/// build. Unambiguous claims keep a probe failure non-fatal — there is
/// nothing to disambiguate.
pub fn divergent_claim_unverifiable(claim: &str, probed: Option<usize>) -> bool {
    probed.is_none() && CLAIM_INIT_OPTIONS_SIZE.iter().any(|(c, _)| *c == claim)
}

/// Members an `era:<token>` claim admits at runtime: EXACTLY the
/// distros whose pinned layouts are verified IDENTICAL — LAYOUT truth,
/// never fingerprint truth (rank-membership
/// admission would let `era:jazzy` admit kilted, whose `rmw_init_options_t`
/// is 160 bytes against jazzy's 168 — a Jazzy-shaped .so overreading
/// Kilted's shorter ABI). The `jazzy` row admits only jazzy and exists
/// as defense-in-depth for the label as a string: `era:jazzy` is never
/// BAKED (the init-options size probe bakes a concrete name
/// or fails the build). lyrical/rolling are verified byte-identical on
/// every pinned axis (see [`era_claim_for_observed`]).
pub(crate) const ERA_CLAIM_ADMITTED_MEMBERS: &[(&str, &[&str])] =
    &[("jazzy", &["jazzy"]), ("lyrical", &["lyrical", "rolling"])];

/// Era-membership admission for an `era:<token>` claim (both inputs
/// already normalized): true iff the runtime distro is one of the
/// claim's LAYOUT-IDENTICAL members (`ERA_CLAIM_ADMITTED_MEMBERS`).
/// Unknown tokens or distros admit NOTHING — an era-labeled binary
/// under an unrecognizable runtime name refuses.
///
/// (Runtime-side only: era.rs's classifier calls it; the build-script
/// copy of this shared file bakes claims and never compares them —
/// hence the allow, which the crate-side use justifies.)
#[allow(dead_code)]
pub fn era_claim_admits(era_token: &str, runtime_distro: &str) -> bool {
    era_claim_members(era_token).is_some_and(|members| members.contains(&runtime_distro))
}

/// The concrete distros an `era:<token>` claim admits at runtime — the
/// ONE table [`era_claim_admits`] consults, exposed so a test that must
/// name a runtime the build's OWN claim admits derives it from here and
/// never from a literal (a no-env generated build
/// bakes `era:lyrical`, which is a CLAIM, not a runtime distro name —
/// the guard admits only that era's concrete members, so `ROS_DISTRO=
/// era:lyrical` refuses). `None` for an unknown token.
#[allow(dead_code)]
pub fn era_claim_members(era_token: &str) -> Option<&'static [&'static str]> {
    ERA_CLAIM_ADMITTED_MEMBERS
        .iter()
        .find(|(token, _)| *token == era_token)
        .map(|(_, members)| *members)
}

/// Layout-identity admission for a LITERAL distro claim:
/// true iff `claimed` and `runtime` (both normalized) are
/// members of ONE `ERA_CLAIM_ADMITTED_MEMBERS` group — the SAME table
/// the `era:<token>` path consults, so a generated build that bakes the
/// concrete name `lyrical` (or `rolling`) under a named `ROS_DISTRO`
/// admits the other exactly as `era:lyrical` does, while jazzy/kilted
/// (jazzy's group is jazzy alone: 168 vs 160 bytes) stay separated.
/// Equality is deliberately NOT special-cased here — the caller checks
/// `runtime == claimed` first — so a name outside every group admits
/// only itself, never a sibling this table has not vouched for.
///
/// (Runtime-side only, like [`era_claim_admits`]: the build-script copy
/// of this shared file never compares claims — hence the allow.)
#[allow(dead_code)]
pub fn layout_identical_distros(claimed: &str, runtime: &str) -> bool {
    ERA_CLAIM_ADMITTED_MEMBERS
        .iter()
        .any(|(_, members)| members.contains(&claimed) && members.contains(&runtime))
}

/// PURE header-provenance check: does the capability fingerprint
/// OBSERVED in the generated bindings agree with the era the CLAIMED
/// distro's headers must produce?
///
/// `observed` is the capability-cfg suffix set build.rs derived from
/// the bindings (tokens not in [`CAPABILITY_MIN_ERA`] are ignored — no
/// expectation exists for them). The claimed name is normalized via
/// [`normalize_distro_claim`] before lookup, and every verdict carries
/// the NORMALIZED claim — the value to bake.
pub fn check_distro_claim(claimed: &str, observed: &[&str]) -> DistroClaimVerdict {
    let normalized = normalize_distro_claim(claimed);
    // The reserved-marker gate comes BEFORE the era lookup —
    // "vendored-dev" is not in DISTRO_ERAS, so without this arm it
    // would ride the UnknownDistro bake and reach era.rs as a forged
    // "no claim". The empty-after-trim claim is refused for the same
    // reason (its baked form is also treated as unclaimed), and so is
    // the `era:` namespace (fingerprint-derived claims are BAKED, never
    // env-supplied — an env `ROS_DISTRO=era:jazzy` riding UnknownDistro
    // would forge era-membership admission the same way).
    if normalized.is_empty()
        || normalized == RESERVED_UNCLAIMED_MARKER
        || normalized.starts_with(ERA_CLAIM_PREFIX)
    {
        return DistroClaimVerdict::ReservedCollision { normalized };
    }
    let Some(rank) = distro_era_rank(&normalized) else {
        return DistroClaimVerdict::UnknownDistro { normalized };
    };
    let expected: Vec<&'static str> = CAPABILITY_MIN_ERA
        .iter()
        .filter(|(_, min)| *min <= rank)
        .map(|(cap, _)| *cap)
        .collect();
    let known_observed: Vec<&'static str> = CAPABILITY_MIN_ERA
        .iter()
        .filter(|(cap, _)| observed.contains(cap))
        .map(|(cap, _)| *cap)
        .collect();
    let missing: Vec<&'static str> = expected
        .iter()
        .filter(|cap| !known_observed.contains(cap))
        .copied()
        .collect();
    let unexpected: Vec<&'static str> = known_observed
        .iter()
        .filter(|cap| !expected.contains(cap))
        .copied()
        .collect();
    if missing.is_empty() && unexpected.is_empty() {
        DistroClaimVerdict::Consistent {
            normalized: BakeableClaim::vetted(normalized),
        }
    } else {
        DistroClaimVerdict::Contradiction {
            claimed: normalized,
            expected,
            observed: known_observed,
            missing,
            unexpected,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- capability headers resolve by the anchor's FIRST-match root ----
    // Hand oracles over an injected filesystem; no real dirs.
    fn fs<'a>(present: &'a [&'a str]) -> impl Fn(&std::path::Path) -> bool + 'a {
        move |p| present.iter().any(|q| std::path::Path::new(q) == p)
    }
    const PROBES: &[(&str, &str)] = &[
        ("CAP_DISCOVERY", "rmw/discovery_options.h"),
        ("CAP_FEATURES", "rmw/features.h"),
    ];
    const ANCHOR: &str = "rmw/init_options.h";

    #[test]
    fn capabilities_come_from_the_root_that_serves_the_anchor() {
        let dirs = [std::path::PathBuf::from("/iron")];
        let present = ["/iron/rmw/init_options.h", "/iron/rmw/discovery_options.h"];
        assert_eq!(
            resolve_capability_headers(&dirs, ANCHOR, PROBES, fs(&present)),
            Ok(vec!["CAP_DISCOVERY"])
        );
    }

    #[test]
    fn a_capability_served_only_by_a_later_root_is_a_mixed_tree_and_refuses() {
        // Humble first (serves the anchor: a pre-Iron rmw_init_options_t),
        // Iron later (serves discovery_options): a per-capability probe would
        // enable the Iron capability against the Humble struct.
        let dirs = [
            std::path::PathBuf::from("/humble"),
            std::path::PathBuf::from("/iron"),
        ];
        let present = [
            "/humble/rmw/init_options.h",
            "/iron/rmw/init_options.h",
            "/iron/rmw/discovery_options.h",
        ];
        assert_eq!(
            resolve_capability_headers(&dirs, ANCHOR, PROBES, fs(&present)),
            Err(CapabilityProbeRefusal::MixedRoots {
                anchor_root: std::path::PathBuf::from("/humble"),
                capability: "CAP_DISCOVERY",
                found_in: std::path::PathBuf::from("/iron"),
            })
        );
    }

    #[test]
    fn the_anchor_root_wins_when_a_later_root_also_serves_the_capability() {
        // Same shape as clang: the first root decides; a duplicate later
        // is not a mix.
        let dirs = [
            std::path::PathBuf::from("/iron"),
            std::path::PathBuf::from("/humble"),
        ];
        let present = [
            "/iron/rmw/init_options.h",
            "/iron/rmw/discovery_options.h",
            "/humble/rmw/init_options.h",
        ];
        assert_eq!(
            resolve_capability_headers(&dirs, ANCHOR, PROBES, fs(&present)),
            Ok(vec!["CAP_DISCOVERY"])
        );
    }

    #[test]
    fn a_capability_served_first_by_an_earlier_root_than_the_anchors_is_a_mixed_tree() {
        // The anchor resolves from the LATER root only, while an
        // EARLIER root also carries the capability header — clang compiles
        // that earlier copy against the later root's struct. Checking the
        // anchor root first would accept its copy.
        let dirs = [
            std::path::PathBuf::from("/iron-headers-only"),
            std::path::PathBuf::from("/humble"),
        ];
        let present = [
            "/iron-headers-only/rmw/discovery_options.h",
            "/humble/rmw/init_options.h",
            "/humble/rmw/discovery_options.h",
        ];
        assert_eq!(
            resolve_capability_headers(&dirs, ANCHOR, PROBES, fs(&present)),
            Err(CapabilityProbeRefusal::MixedRoots {
                anchor_root: std::path::PathBuf::from("/humble"),
                capability: "CAP_DISCOVERY",
                found_in: std::path::PathBuf::from("/iron-headers-only"),
            })
        );
    }

    #[test]
    fn no_anchor_anywhere_is_refused_not_fingerprinted() {
        let dirs = [std::path::PathBuf::from("/x")];
        let present = ["/x/rmw/discovery_options.h"];
        assert_eq!(
            resolve_capability_headers(&dirs, ANCHOR, PROBES, fs(&present)),
            Err(CapabilityProbeRefusal::NoAnchor {
                anchor: ANCHOR.to_string()
            })
        );
    }

    // HAND-WRITTEN era fingerprints (deliberately NOT derived from the
    // production table — the verified per-branch boundaries,
    // restated so a table edit cannot silently agree with itself).
    const FOXY_SET: &[&str] = &[];
    const GALACTIC_SET: &[&str] = &["qos_compatibility", "message_lost_event", "network_flow"];
    const HUMBLE_SET: &[&str] = &[
        "qos_compatibility",
        "message_lost_event",
        "network_flow",
        "fetch_function",
        "content_filter_options",
        "event_callback",
        "message_info_sequence_numbers",
        "features",
    ];
    const IRON_SET: &[&str] = &[
        "qos_compatibility",
        "message_lost_event",
        "network_flow",
        "fetch_function",
        "content_filter_options",
        "event_callback",
        "message_info_sequence_numbers",
        "features",
        "discovery_options",
        "matched_events",
        "type_hash",
    ];
    const JAZZY_SET: &[&str] = &[
        "qos_compatibility",
        "message_lost_event",
        "network_flow",
        "fetch_function",
        "content_filter_options",
        "event_callback",
        "message_info_sequence_numbers",
        "features",
        "discovery_options",
        "matched_events",
        "type_hash",
        "is_key",
        "any_key_member",
    ];
    const LYRICAL_SET: &[&str] = &[
        "qos_compatibility",
        "message_lost_event",
        "network_flow",
        "fetch_function",
        "content_filter_options",
        "event_callback",
        "message_info_sequence_numbers",
        "features",
        "discovery_options",
        "matched_events",
        "type_hash",
        "is_key",
        "any_key_member",
        "is_rosidl_buffer",
        "event_type_max",
    ];

    const GALACTIC_ONLY: &[&str] = &["qos_compatibility", "message_lost_event", "network_flow"];
    const HUMBLE_ONLY: &[&str] = &[
        "fetch_function",
        "content_filter_options",
        "event_callback",
        "message_info_sequence_numbers",
        "features",
    ];
    const IRON_ONLY: &[&str] = &["discovery_options", "matched_events", "type_hash"];
    const JAZZY_ONLY: &[&str] = &["is_key", "any_key_member"];
    const LYRICAL_ONLY: &[&str] = &["is_rosidl_buffer", "event_type_max"];

    fn contradiction(
        verdict: DistroClaimVerdict,
    ) -> (Vec<&'static str>, Vec<&'static str>, Vec<&'static str>) {
        match verdict {
            DistroClaimVerdict::Contradiction {
                expected,
                missing,
                unexpected,
                ..
            } => (expected, missing, unexpected),
            other => panic!("expected a Contradiction, got {other:?}"),
        }
    }

    #[test]
    fn every_cargo_warning_in_build_rs_goes_through_the_escaping_seam() {
        // Escaping only
        // protects while EVERY warning routes through the seam. A build script
        // has no test harness, so the guard is a source walk — the same
        // shape the repo uses for the hand-written cdylib inits. The
        // seam's own `println!` carries the marker below; nothing else
        // may print onto the directive channel by hand.
        let src = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/build.rs"))
            .expect("build.rs readable");
        let code = code_only(&src);
        // ANTI-TAUTOLOGY: the stripped view must still hold the real
        // call sites, or every absence assertion below is vacuous.
        assert!(
            code.matches("cargo_warning(&format!(").count() >= 4,
            "the stripped view lost the warning call sites"
        );
        let raw: Vec<&str> = code
            .lines()
            .filter(|l| l.contains("cargo:warning="))
            .filter(|l| !l.contains("cargo:warning={}"))
            .collect();
        assert!(
            raw.is_empty(),
            "every `cargo:warning=` must go through `cargo_warning` (which escapes control \
             characters before they can forge a second cargo directive); found:\n{}",
            raw.join("\n")
        );
    }

    /// `src` with `//` line comments and `/* … */` blocks removed (Rust
    /// block comments NEST), so prose naming a forbidden pattern is not a
    /// false positive. String literals are deliberately NOT modelled —
    /// the one literal that spells the pattern is the seam's own, which
    /// the walk allows by its `{}` form.
    fn code_only(src: &str) -> String {
        let mut out = String::with_capacity(src.len());
        let b: Vec<char> = src.chars().collect();
        let (mut i, mut depth) = (0usize, 0usize);
        while i < b.len() {
            if depth == 0 && b[i] == '/' && i + 1 < b.len() && b[i + 1] == '/' {
                while i < b.len() && b[i] != '\n' {
                    i += 1;
                }
                continue;
            }
            if b[i] == '/' && i + 1 < b.len() && b[i + 1] == '*' {
                depth += 1;
                i += 2;
                continue;
            }
            if depth > 0 && b[i] == '*' && i + 1 < b.len() && b[i + 1] == '/' {
                depth -= 1;
                i += 2;
                continue;
            }
            if depth == 0 {
                out.push(b[i]);
            } else if b[i] == '\n' {
                out.push('\n');
            }
            i += 1;
        }
        out
    }

    #[test]
    fn the_cargo_directive_escape_neutralises_every_control_character() {
        // The cargo-directive injection hazard. Hand oracles.
        // THE attack: a newline in an interpolated env value ends the
        // `cargo:warning=` line and starts a directive of the attacker's
        // choosing. After escaping, the rendered text contains no line
        // break at all, so cargo reads exactly one directive.
        let hostile = "prefix=/x\ncargo:rustc-cfg=cerulion_has_is_key";
        let escaped = escape_for_cargo_directive(hostile);
        assert!(
            !escaped.contains('\n') && !escaped.contains('\r'),
            "no line break may survive: {escaped:?}"
        );
        assert_eq!(
            escaped, "prefix=/x\\u{a}cargo:rustc-cfg=cerulion_has_is_key",
            "the injected text stays VISIBLE — the warning is not dropped"
        );
        // Carriage return (a lone CR also ends a line for some readers),
        // NUL, and an escape sequence.
        assert_eq!(escape_for_cargo_directive("a\rb"), "a\\u{d}b");
        assert_eq!(escape_for_cargo_directive("a\0b"), "a\\u{0}b");
        assert_eq!(escape_for_cargo_directive("a\u{1b}[31mb"), "a\\u{1b}[31mb");
        // ANTI-TAUTOLOGY: ordinary text — including every character a
        // real path or distro name uses — is byte-identical, so the
        // escaper cannot be "escape everything" and pass.
        for clean in [
            "rmw_cerulion: ROS_DISTRO=jazzy is set",
            "/opt/ros/jazzy:/opt/ros/lyrical",
            "distro=vendored-dev bindings=vendored caps=fetch_function,is_key",
            "unicode ok: é 日本語 — ✓",
            "",
        ] {
            assert_eq!(escape_for_cargo_directive(clean), clean, "{clean:?}");
        }
    }

    fn consistent(normalized: &str) -> DistroClaimVerdict {
        DistroClaimVerdict::Consistent {
            normalized: BakeableClaim::vetted(normalized.to_string()),
        }
    }

    fn bake(claim: &str) -> GeneratedClaim {
        GeneratedClaim::Bake(BakeableClaim::vetted(claim.to_string()))
    }

    #[test]
    fn a_consistent_pairing_is_consistent_for_every_known_distro() {
        for (claimed, set) in [
            ("foxy", FOXY_SET),
            ("galactic", GALACTIC_SET),
            ("humble", HUMBLE_SET),
            ("iron", IRON_SET),
            ("jazzy", JAZZY_SET),
            ("kilted", JAZZY_SET),
            ("lyrical", LYRICAL_SET),
            ("rolling", LYRICAL_SET),
        ] {
            assert_eq!(
                check_distro_claim(claimed, set),
                consistent(claimed),
                "claimed={claimed}"
            );
        }
    }

    #[test]
    fn every_era_boundary_contradicts_in_both_directions() {
        // NEWER headers than the claim (unexpected present) …
        for (claimed, observed, want_unexpected) in [
            ("foxy", GALACTIC_SET, GALACTIC_ONLY),
            ("galactic", HUMBLE_SET, HUMBLE_ONLY),
            ("humble", IRON_SET, IRON_ONLY),
            ("iron", JAZZY_SET, JAZZY_ONLY),
            ("jazzy", LYRICAL_SET, LYRICAL_ONLY),
            ("kilted", LYRICAL_SET, LYRICAL_ONLY),
        ] {
            let (_, missing, unexpected) = contradiction(check_distro_claim(claimed, observed));
            assert!(missing.is_empty(), "claimed={claimed}: {missing:?}");
            assert_eq!(unexpected, want_unexpected, "claimed={claimed}");
        }
        // … and OLDER headers than the claim (expected missing).
        for (claimed, observed, want_missing) in [
            ("galactic", FOXY_SET, GALACTIC_ONLY),
            ("humble", GALACTIC_SET, HUMBLE_ONLY),
            ("iron", HUMBLE_SET, IRON_ONLY),
            ("jazzy", IRON_SET, JAZZY_ONLY),
            ("lyrical", JAZZY_SET, LYRICAL_ONLY),
            ("rolling", JAZZY_SET, LYRICAL_ONLY),
        ] {
            let (_, missing, unexpected) = contradiction(check_distro_claim(claimed, observed));
            assert!(unexpected.is_empty(), "claimed={claimed}: {unexpected:?}");
            assert_eq!(missing, want_missing, "claimed={claimed}");
        }
    }

    #[test]
    fn the_headline_env_says_jazzy_headers_are_humble_fails_naming_the_gap() {
        // The exact motivating scenario: ROS_DISTRO=jazzy while
        // AMENT_PREFIX_PATH / CERULION_RMW_SYS_INCLUDE supplies Humble
        // headers — without the check this would bake "jazzy" and rmw_init
        // would admit a layout-incompatible .so on a Jazzy robot.
        let (expected, missing, unexpected) =
            contradiction(check_distro_claim("jazzy", HUMBLE_SET));
        assert_eq!(expected, JAZZY_SET, "expected = the full jazzy-era set");
        assert_eq!(
            missing,
            &[
                "discovery_options",
                "matched_events",
                "type_hash",
                "is_key",
                "any_key_member"
            ],
            "the Iron+Jazzy markers are what Humble headers cannot produce"
        );
        assert!(unexpected.is_empty());
    }

    #[test]
    fn indistinguishable_adjacent_pairs_are_admitted_as_compatible() {
        // jazzy ↔ kilted and lyrical ↔ rolling share one fingerprint —
        // capabilities distinguish ERAS, not every adjacent distro. The
        // residual: within each pair the pinned layout axes
        // genuinely agree, so admitted-as-compatible is compatible.
        assert_eq!(
            check_distro_claim("kilted", JAZZY_SET),
            consistent("kilted")
        );
        assert_eq!(check_distro_claim("jazzy", JAZZY_SET), consistent("jazzy"));
        assert_eq!(
            check_distro_claim("rolling", LYRICAL_SET),
            consistent("rolling")
        );
        assert_eq!(
            check_distro_claim("lyrical", LYRICAL_SET),
            consistent("lyrical")
        );
    }

    #[test]
    fn an_unknown_distro_name_is_reported_unknown_never_judged() {
        assert_eq!(
            check_distro_claim("m_next", LYRICAL_SET),
            DistroClaimVerdict::UnknownDistro {
                normalized: "m_next".to_string()
            }
        );
        // An unknown name is normalized too — the baked value must be
        // canonical whichever arm produced it.
        assert_eq!(
            check_distro_claim(" M_Next ", LYRICAL_SET),
            DistroClaimVerdict::UnknownDistro {
                normalized: "m_next".to_string()
            }
        );
        // The empty-after-trim claim is NOT judged unknown — it is a
        // reserved collision (its baked form reads as "no claim").
        assert_eq!(
            check_distro_claim("", FOXY_SET),
            DistroClaimVerdict::ReservedCollision {
                normalized: String::new()
            }
        );
    }

    #[test]
    fn a_claim_normalizing_to_the_reserved_marker_is_refused_never_baked() {
        // Reserved-marker arms: a GENERATED build with ROS_DISTRO naming the
        // reserved marker (any spelling) must be a hard build failure —
        // baking it would mark a distro-specific ABI "unclaimed" and
        // the runtime guard would then admit it under ANY distro. The
        // drop-reserved-check variant fails exactly here (the spellings
        // fall through to UnknownDistro instead).
        for spelling in ["vendored-dev", "VENDORED-DEV", " Vendored-Dev "] {
            assert_eq!(
                check_distro_claim(spelling, JAZZY_SET),
                DistroClaimVerdict::ReservedCollision {
                    normalized: "vendored-dev".to_string()
                },
                "spelling={spelling:?}"
            );
        }
        // Fingerprint-independent: the gate fires before any era logic.
        assert_eq!(
            check_distro_claim("VENDORED-DEV", LYRICAL_SET),
            DistroClaimVerdict::ReservedCollision {
                normalized: "vendored-dev".to_string()
            }
        );
        assert_eq!(
            check_distro_claim("   ", FOXY_SET),
            DistroClaimVerdict::ReservedCollision {
                normalized: String::new()
            }
        );
        // The shared constant is the guard's own marker spelling.
        assert_eq!(RESERVED_UNCLAIMED_MARKER, "vendored-dev");
    }

    #[test]
    fn claimed_name_is_trimmed_and_case_normalized() {
        assert_eq!(
            check_distro_claim(" Jazzy ", JAZZY_SET),
            consistent("jazzy")
        );
        let (_, missing, _) = contradiction(check_distro_claim("JAZZY", HUMBLE_SET));
        assert!(!missing.is_empty());
    }

    #[test]
    fn the_verdict_carries_the_normalized_claim_which_is_what_gets_baked() {
        // Build-side arm: build.rs bakes the verdict's
        // `normalized` payload VERBATIM, so this oracle pins the baked
        // value for an odd env spelling — ROS_DISTRO=JAZZY must bake
        // "jazzy", or a conventionally-sourced runtime `jazzy` fails
        // the guard's comparison and rmw_init refuses a COMPATIBLE
        // library. The raw-bake variant (returning the claim un-
        // normalized) fails exactly here.
        assert_eq!(check_distro_claim("JAZZY", JAZZY_SET), consistent("jazzy"));
        assert_eq!(check_distro_claim("Foxy", FOXY_SET), consistent("foxy"));
        assert_eq!(
            check_distro_claim("ROLLING ", LYRICAL_SET),
            consistent("rolling")
        );
        // And the one normalization is shared: the baked fixed point.
        assert_eq!(normalize_distro_claim(" JAZZY "), "jazzy");
        assert_eq!(normalize_distro_claim("vendored-dev"), "vendored-dev");
    }

    #[test]
    fn future_unknown_capability_tokens_are_ignored() {
        // A 16th marker rolling grows tomorrow carries no expectation
        // in this table — it must not fail today's consistent pairing.
        let mut observed: Vec<&str> = LYRICAL_SET.to_vec();
        observed.push("some_future_cap");
        assert_eq!(
            check_distro_claim("rolling", &observed),
            consistent("rolling")
        );
    }

    #[test]
    fn a_far_mispairing_reports_missing_and_unexpected_together() {
        // Claimed humble, observed only an Iron marker: everything the
        // humble era needs is missing AND the Iron marker is above the
        // claim — both halves must be named.
        let (_, missing, unexpected) =
            contradiction(check_distro_claim("humble", &["discovery_options"]));
        assert_eq!(missing, HUMBLE_SET, "all eight humble-era caps absent");
        assert_eq!(unexpected, &["discovery_options"]);
    }

    #[test]
    fn an_env_claim_in_the_era_namespace_is_refused_never_baked() {
        // `era:` claims are BAKED (fingerprint-derived), never accepted
        // from the environment — an env `ROS_DISTRO=era:jazzy` riding
        // the UnknownDistro bake would forge era-membership admission
        // (the reserved-marker class, era flavor).
        for spelling in ["era:jazzy", "ERA:JAZZY", " Era:Lyrical ", "era:anything"] {
            match check_distro_claim(spelling, JAZZY_SET) {
                DistroClaimVerdict::ReservedCollision { normalized } => {
                    assert!(normalized.starts_with(ERA_CLAIM_PREFIX), "{normalized}");
                }
                other => panic!("spelling={spelling:?}: expected ReservedCollision, got {other:?}"),
            }
        }
    }

    #[test]
    fn a_distro_less_generated_build_bakes_the_fingerprint_derived_claim() {
        // NO env claim on the generated path must not
        // bake the unclaimed marker — that would be one distro's real ABI
        // admitted under every runtime distro. The claim comes FROM the
        // fingerprint: the concrete name for single-member eras. The
        // bake-vendored-dev variant fails here.
        assert_eq!(era_claim_for_observed(FOXY_SET, None), bake("foxy"));
        assert_eq!(era_claim_for_observed(GALACTIC_SET, None), bake("galactic"));
        assert_eq!(era_claim_for_observed(HUMBLE_SET, None), bake("humble"));
        assert_eq!(era_claim_for_observed(IRON_SET, None), bake("iron"));
        // A fingerprint matching no known era derives NOTHING — the
        // caller fails the build rather than claim dishonestly.
        assert_eq!(
            era_claim_for_observed(&["fetch_function", "is_rosidl_buffer"], None),
            GeneratedClaim::NoKnownEra
        );
        // Unknown future tokens are ignored, as everywhere else.
        let mut observed: Vec<&str> = LYRICAL_SET.to_vec();
        observed.push("some_future_cap");
        assert_eq!(
            era_claim_for_observed(&observed, None),
            bake("era:lyrical"),
            "lyrical/rolling verified layout-identical — the era label covers both"
        );
    }

    #[test]
    fn the_jazzy_kilted_pair_is_disambiguated_by_layout_or_refused_never_labeled() {
        // The jazzy/kilted pair shares one fingerprint
        // while their rmw_init_options_t layouts DIFFER (168 vs 160 —
        // kilted dropped localhost_only), and the claim is the guard's
        // input at every guarded export, so an era label covering both
        // admits an ABI break in one direction or the other. The
        // bindings' own layout test is the discriminator: the concrete
        // member is baked, and `era:jazzy` is NEVER produced.
        assert_eq!(era_claim_for_observed(JAZZY_SET, Some(168)), bake("jazzy"));
        assert_eq!(era_claim_for_observed(JAZZY_SET, Some(160)), bake("kilted"));
        // Unreadable or unrecognizable size ⇒ AMBIGUOUS ⇒ the caller
        // FAILS the build (never a guess, never the era label).
        for probed in [None, Some(104), Some(96)] {
            assert_eq!(
                era_claim_for_observed(JAZZY_SET, probed),
                GeneratedClaim::AmbiguousEra {
                    era_label: "jazzy",
                    members: vec!["jazzy", "kilted"],
                    init_options_size: probed,
                },
                "probed={probed:?}"
            );
        }
        // The lyrical pair does NOT consult the size: its members are
        // verified layout-identical, so the label is accurate as-is.
        assert_eq!(
            era_claim_for_observed(LYRICAL_SET, Some(160)),
            bake("era:lyrical")
        );
        assert_eq!(
            era_claim_for_observed(LYRICAL_SET, None),
            bake("era:lyrical")
        );
    }

    #[test]
    fn the_init_options_probe_reads_both_bindgen_layout_test_formats() {
        // The const form (bindgen 0.71+ — the vendored snapshot).
        let const_form = r#"["Size of rmw_init_options_s"][::std::mem::size_of::<rmw_init_options_s>() - 168usize];"#;
        assert_eq!(probe_init_options_size(const_form), Some(168));
        // The assert form (bindgen 0.70 — the generated path).
        let assert_form = r#"assert_eq!(::std::mem::size_of::<rmw_init_options_s>(), 160usize,"#;
        assert_eq!(probe_init_options_size(assert_form), Some(160));
        // The pre-Galactic `_t` tag spelling.
        let t_tag = r#"[::std::mem::size_of::<rmw_init_options_t>() - 104usize];"#;
        assert_eq!(probe_init_options_size(t_tag), Some(104));
        // No layout test at all ⇒ None (the caller refuses to guess).
        assert_eq!(
            probe_init_options_size("pub struct rmw_init_options_s {}"),
            None
        );
        // A digit run that does not fit a `usize` must
        // not END the search — the probe `continue`s past it. Without
        // that, this reads `None` and a jazzy/kilted claim fails the
        // build on a snapshot that DOES carry a readable layout test.
        let overflow_then_real = concat!(
            "[::std::mem::size_of::<rmw_init_options_s>() - 99999999999999999999999usize];",
            "[::std::mem::size_of::<rmw_init_options_s>() - 160usize];"
        );
        assert_eq!(probe_init_options_size(overflow_then_real), Some(160));
        // A similarly-named OTHER struct's test does not satisfy it.
        assert_eq!(
            probe_init_options_size(
                r#"[::std::mem::size_of::<rmw_security_options_s>() - 16usize];"#
            ),
            None
        );
        // Production parity: the REAL vendored rolling snapshot reads
        // 160 (localhost_only removed) through the same probe build.rs
        // runs — the no-inert pin for the whole mechanism.
        let vendored = std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/src/ffi/vendored_bindings.rs"
        ))
        .expect("vendored bindings readable");
        assert_eq!(probe_init_options_size(&vendored), Some(160));
    }

    #[test]
    fn an_env_set_jazzy_or_kilted_claim_is_cross_checked_against_the_layout() {
        // The same discriminator closes the jazzy↔kilted
        // env-lie hole: ROS_DISTRO naming one member of the pair
        // while the headers are the other's passes the fingerprint
        // check (identical sets) — the init-options size does not.
        assert_eq!(init_options_size_contradicts("jazzy", Some(168)), None);
        assert_eq!(init_options_size_contradicts("kilted", Some(160)), None);
        assert_eq!(
            init_options_size_contradicts("jazzy", Some(160)),
            Some((168, 160)),
            "kilted headers under a jazzy claim must contradict"
        );
        assert_eq!(
            init_options_size_contradicts("kilted", Some(168)),
            Some((160, 168)),
            "jazzy headers under a kilted claim must contradict"
        );
        // Claims outside the divergent pair carry no expectation …
        assert_eq!(init_options_size_contradicts("humble", Some(104)), None);
        assert_eq!(init_options_size_contradicts("lyrical", Some(999)), None);
        // … and an unreadable size makes no judgement (the documented
        // narrowed residual).
        assert_eq!(init_options_size_contradicts("jazzy", None), None);
    }

    #[test]
    fn the_vendored_snapshot_era_is_the_lyrical_era() {
        // The token the unclaimed marker admits through is the
        // era the vendored snapshot's OWN fingerprint classifies to, and
        // it has a row in the admitted-members table.
        assert_eq!(observed_era_rank(LYRICAL_SET), Some(ERA_LYRICAL));
        assert_eq!(ERA_CLAIM_TOKENS[ERA_LYRICAL], VENDORED_SNAPSHOT_ERA_TOKEN);
        assert_eq!(
            era_claim_members(VENDORED_SNAPSHOT_ERA_TOKEN),
            Some(&["lyrical", "rolling"][..])
        );
    }

    #[test]
    fn a_literal_claim_admits_exactly_its_layout_identical_members() {
        // Symmetric within a group …
        assert!(layout_identical_distros("lyrical", "rolling"));
        assert!(layout_identical_distros("rolling", "lyrical"));
        assert!(layout_identical_distros("lyrical", "lyrical"));
        // … never across groups, and the jazzy/kilted separation holds
        // (jazzy's group is jazzy alone: 168 vs 160 bytes).
        assert!(!layout_identical_distros("jazzy", "kilted"));
        assert!(!layout_identical_distros("kilted", "jazzy"));
        assert!(!layout_identical_distros("jazzy", "lyrical"));
        assert!(!layout_identical_distros("rolling", "jazzy"));
        assert!(!layout_identical_distros("jazzy", "m_next"));
        // A name in no group vouches for nothing — not even itself
        // (equality is the caller's arm, so this can never widen it).
        assert!(!layout_identical_distros("kilted", "kilted"));
        assert!(!layout_identical_distros("m_next", "m_next"));
        // Drift tripwire: every group is symmetric and closed, and no
        // member of one group is admitted by another.
        for (i, (_, members)) in ERA_CLAIM_ADMITTED_MEMBERS.iter().enumerate() {
            for a in *members {
                for b in *members {
                    assert!(layout_identical_distros(a, b) && layout_identical_distros(b, a));
                }
                for (j, (_, others)) in ERA_CLAIM_ADMITTED_MEMBERS.iter().enumerate() {
                    if i != j {
                        for o in *others {
                            assert!(!layout_identical_distros(a, o), "{a} vs {o}");
                        }
                    }
                }
            }
        }
        // The table's own SHAPE — every token is
        // unique (`era_claim_members` takes the FIRST row, so a duplicate
        // would be silently ignored), every token is the label of an era
        // rank, and every member sits in exactly that rank. The loop
        // above cannot see a widening that stays symmetric and disjoint
        // (an `iron` added to the lyrical row would pass it). It does NOT
        // subsume the hard-coded `!layout_identical_distros("jazzy",
        // "kilted")` literal above: `kilted` shares the
        // jazzy RANK, so a `kilted` added to the jazzy row satisfies
        // every clause here — the literal is still the only thing that
        // catches it, and there is no totality claim over
        // `ERA_CLAIM_TOKENS` either.
        let mut seen = std::collections::HashSet::new();
        for (token, members) in ERA_CLAIM_ADMITTED_MEMBERS {
            assert!(seen.insert(*token), "duplicate era token `{token}`");
            let rank = distro_era_rank(token)
                .unwrap_or_else(|| panic!("era token `{token}` is not a known distro name"));
            assert_eq!(
                ERA_CLAIM_TOKENS[rank], *token,
                "the token must be its rank's label"
            );
            for member in *members {
                assert_eq!(
                    distro_era_rank(member),
                    Some(rank),
                    "`{member}` is not a member of the `{token}` era"
                );
            }
        }
    }

    #[test]
    fn era_claim_admission_is_layout_identical_membership_only() {
        // LAYOUT truth, not fingerprint truth: era:jazzy admits ONLY
        // jazzy — kilted shares the fingerprint but not the
        // rmw_init_options_t layout (168 vs 160), so admitting it would let a
        // Jazzy-shaped .so overread Kilted's shorter ABI (the
        // revert-kilted variant fails here and in the
        // classifier vectors).
        assert!(era_claim_admits("jazzy", "jazzy"));
        assert!(
            !era_claim_admits("jazzy", "kilted"),
            "kilted is NOT layout-identical to jazzy — must be refused"
        );
        // lyrical/rolling: verified byte-identical on every pinned axis
        // (2026-09-02, release branches) — both admitted.
        assert!(era_claim_admits("lyrical", "lyrical"));
        assert!(era_claim_admits("lyrical", "rolling"));
        // Everything outside an era refuses, unknown names included,
        // in BOTH directions.
        assert!(!era_claim_admits("jazzy", "foxy"));
        assert!(!era_claim_admits("jazzy", "lyrical"));
        assert!(!era_claim_admits("lyrical", "jazzy"));
        assert!(!era_claim_admits("jazzy", "m_next"));
        assert!(!era_claim_admits("bogus", "jazzy"));
    }

    #[test]
    fn an_unrelated_include_tree_never_selects_the_bindgen_path() {
        // An AMENT prefix whose include/ EXISTS but
        // carries no ROS headers (e.g. /usr/local), if added
        // unconditionally, would select bindgen and PANIC, while
        // the docs promise the vendored fallback. Such a prefix must
        // be SKIPPED (warned), and with nothing usable the caller's
        // empty include set takes the documented vendored path. The
        // drop-the-marker-check variant fails here.
        let dirs: std::collections::HashSet<String> = [
            "/usr/local/include",
            "/usr/local/include/zlib",
            "/opt/notros/include",
        ]
        .into_iter()
        .map(String::from)
        .collect();
        let mut probe = |path: String| dirs.contains(&path);
        let (usable, skipped) = select_ros_prefixes(&["/usr/local", "/opt/notros"], &mut probe);
        assert!(
            usable.is_empty(),
            "unrelated include trees must not qualify: {usable:?}"
        );
        assert_eq!(
            skipped,
            ["/usr/local", "/opt/notros"],
            "both warned by name"
        );
    }

    #[test]
    fn ros_prefix_selection_is_a_marker_partition_not_an_existence_check() {
        // The control + the isolated-colcon shape a single rmw/rmw.h
        // marker would break: each core package in its OWN prefix, so
        // the rcutils prefix carries no rmw headers — both must still
        // qualify, an includeless pure-python package prefix is dropped
        // SILENTLY (no warning spam), and only the unrelated tree is
        // skipped-with-warning.
        let dirs: std::collections::HashSet<String> = [
            // merged underlay (per-package layout dirs)
            "/opt/ros/x/include",
            "/opt/ros/x/include/rmw",
            "/opt/ros/x/include/rcutils",
            // isolated colcon split: one package per prefix
            "/ws/install/rcutils/include",
            "/ws/install/rcutils/include/rcutils",
            "/ws/install/rmw/include",
            "/ws/install/rmw/include/rmw",
            // an overlay carrying only user packages
            "/ws/install/mypkg/include",
            "/ws/install/mypkg/include/mypkg",
            // an unrelated tree
            "/usr/local/include",
        ]
        .into_iter()
        .map(String::from)
        .collect();
        let mut probe = |path: String| dirs.contains(&path);
        let (usable, skipped) = select_ros_prefixes(
            &[
                "/opt/ros/x",
                "/ws/install/rcutils",
                "/ws/install/rmw",
                "/ws/install/mypkg",
                "/ws/install/purepython", // no include/ at all
                "/usr/local",
            ],
            &mut probe,
        );
        assert_eq!(
            usable,
            ["/opt/ros/x", "/ws/install/rcutils", "/ws/install/rmw"],
            "core-marker prefixes qualify, incl. the rmw-less rcutils prefix"
        );
        assert_eq!(
            skipped,
            ["/ws/install/mypkg", "/usr/local"],
            "include-without-core-markers is warned; includeless is silent"
        );
        // Empty input ⇒ empty partition (the vendored path upstream).
        let (u, k) = select_ros_prefixes(&[], &mut probe);
        assert!(u.is_empty() && k.is_empty());
    }

    #[test]
    fn an_unknown_distro_name_is_no_longer_a_validation_bypass() {
        // The unknown NAME must not be baked — the
        // claim comes from the fingerprint + layout, exactly like the
        // no-env path. The revert-to-bake-as-is variant fails here.
        assert_eq!(
            claim_for_unknown_distro("jazzyy", JAZZY_SET, Some(168)),
            bake("jazzy"),
            "a typo'd name with jazzy-layout headers bakes the DERIVED claim"
        );
        assert_eq!(
            claim_for_unknown_distro("jazzyy", JAZZY_SET, Some(160)),
            bake("kilted"),
            "Kilted-shaped bindings under ROS_DISTRO=jazzyy"
        );
        assert_eq!(
            claim_for_unknown_distro("m_next", LYRICAL_SET, None),
            bake("era:lyrical")
        );
        // Ambiguous or unrecognized fingerprints FAIL the build via
        // the caller — never a baked unknown name.
        assert_eq!(
            claim_for_unknown_distro("jazzyy", JAZZY_SET, None),
            GeneratedClaim::AmbiguousEra {
                era_label: "jazzy",
                members: vec!["jazzy", "kilted"],
                init_options_size: None,
            }
        );
        assert_eq!(
            claim_for_unknown_distro("bogus", &["fetch_function", "is_rosidl_buffer"], None),
            GeneratedClaim::NoKnownEra
        );
    }

    #[test]
    fn an_unverifiable_divergent_claim_refuses_instead_of_admitting() {
        // The size probe is the ONLY cross-check for the
        // fingerprint-identical jazzy/kilted pair, so a missing layout
        // test must REFUSE those claims — not admit them unverified.
        // The drop-the-refusal variant fails here.
        assert!(
            divergent_claim_unverifiable("jazzy", None),
            "a jazzy claim with no readable layout test is unverifiable"
        );
        assert!(divergent_claim_unverifiable("kilted", None));
        // A readable size is handled by the contradiction check, not
        // this refusal …
        assert!(!divergent_claim_unverifiable("jazzy", Some(168)));
        assert!(!divergent_claim_unverifiable("jazzy", Some(160)));
        // … and unambiguous claims keep a probe failure non-fatal:
        // there is nothing to disambiguate.
        assert!(!divergent_claim_unverifiable("humble", None));
        assert!(!divergent_claim_unverifiable("lyrical", None));
        assert!(!divergent_claim_unverifiable("foxy", None));
    }

    #[test]
    fn observed_era_labels_exact_fingerprints_only() {
        assert_eq!(observed_era_label(HUMBLE_SET), Some("Humble"));
        assert_eq!(observed_era_label(FOXY_SET), Some("Foxy"));
        assert_eq!(observed_era_label(LYRICAL_SET), Some("Lyrical/Rolling"));
        // A mixed set matches no era exactly.
        assert_eq!(
            observed_era_label(&["fetch_function", "is_rosidl_buffer"]),
            None
        );
    }
}

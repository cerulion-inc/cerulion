// SPDX-License-Identifier: AGPL-3.0-only
//! The CONTAINER RULE over the whole vendored corpus — a variable
//! schema's slice tier must be at least as large as every variable member it
//! embeds.
//!
//! # Why this rule, and why it needs a walk rather than a comment
//!
//! `variable_schema_max_slice_len` gives each variable schema a byte budget,
//! and a publisher of that schema loans exactly that many bytes from a
//! `Static` iceoryx2 pool that CANNOT GROW. So when a container's ceiling sits
//! BELOW a member's, the member is by construction able to produce a payload
//! its own container cannot hold, and the publish hard-fails
//! `PayloadTooLarge`. The upstream-drift fix found that shape by hand on
//! `visualization_msgs/Marker` (it had gained an embedded `CompressedImage` +
//! `MeshFile`, both TIER_LARGE, while sitting at TIER_MEDIUM). The tier
//! audit found two MORE live instances by hand —
//! `control_msgs/JointWrenchTrajectoryPoint` over
//! `trajectory_msgs/JointTrajectoryPoint`, and `shape_msgs/SolidPrimitive`
//! over `geometry_msgs/Polygon` — which is three hand-discoveries of one
//! mechanical property, and the argument for mechanizing it.
//!
//! Hand-checking also does not survive the next edit. The table is one 300-line
//! `match`; a retier that is locally correct can silently invert a pair three
//! packages away, and nothing else in the repo reads both sides.
//!
//! # What is checked, precisely
//!
//! For every schema in `native_ros2_messages/msg/` that the generator routes
//! down the VARIABLE path (`!is_definitely_fixed()` — the same predicate
//! `generate_schema` branches on, so "consults the tier table" and "is checked
//! here" are the same set), every field is unwrapped through
//! `FixedArray`/`DynamicArray` to its element type. If that element is a
//! `Nested` reference to a schema that is ITSELF variable, the pair is
//! checked: `tier(container) >= tier(member)`.
//!
//! A member that resolves FIXED is deliberately NOT checked — it is inlined
//! into the container's fixed section and never consults the table at all.
//!
//! # ONE LEVEL, and what that does and does not buy
//!
//! The walk compares DIRECT members only. With no waivers that is equivalent
//! to the transitive closure, because `>=` is transitive: if every container
//! dominates its direct members, it dominates everything beneath them. A
//! WAIVER breaks exactly that step — see the waiver's own note below, which
//! names the containers it therefore also reaches.
//!
//! # Incidental totality
//!
//! `variable_schema_max_slice_len` panics for a schema from an
//! `IN_REPO_PACKAGES` package with no explicit arm, and this walk calls it for
//! every variable schema in the corpus. So it also gives the exhaustive
//! coverage `max_slice_len_test.rs` documents as missing ("a schema added to
//! `native_ros2_messages/` without an explicit arm ... these tests will not
//! catch it"). That is a side effect of the walk, not its purpose.

use cerulion_core::codegen::{
    parse_rosmsg, resolve_fixed_nested, variable_schema_max_slice_len, FieldType, MessageSchema,
};
use std::collections::{BTreeMap, BTreeSet};
use std::path::PathBuf;

// ─────────────────────────────── the waiver ───────────────────────────────

/// The ONLY container/member pairs allowed to violate the rule, as
/// `(container, field, member)`.
///
/// Exactly one ships, and the assertions below require it to be
/// EXERCISED — a waiver that stops describing a real violation fails this
/// test rather than sitting here pre-authorising a future one.
///
/// ## `visualization_msgs/Marker.texture` → `sensor_msgs/CompressedImage`
///
/// The tier audit moved `CompressedImage` LARGE → HUGE because a single 4K PNG from
/// `image_transport` can exceed 16 MiB outright (raw 3840×2160×3 = 24.9 MB;
/// PNG lands at 55–95 % of raw), which is a live hard-fail on the CAMERA path.
/// `Marker` embeds a `CompressedImage texture` and stays at LARGE.
///
/// Cascading `Marker` to HUGE — and with it `MarkerArray`,
/// `InteractiveMarkerControl`, `InteractiveMarker`, `InteractiveMarkerInit`
/// and `InteractiveMarkerUpdate`, since the rule is transitive — was
/// considered and rejected: a Marker `texture` is a MESH DECAL, not a camera
/// frame. Upstream's own field doc describes it as the image a textured mesh
/// samples from, and 16 MiB of decal is already past any realistic authoring
/// pipeline. The cost of being wrong is bounded and LOUD (`PayloadTooLarge`
/// names the topic; `Static` pools cannot grow, so nothing is silent), and the
/// escape hatch is explicit and per-topic: `max_slice_len:` on the output.
///
/// The waiver is what breaks the transitivity argument in the module docs, so
/// state its reach plainly: the five container types listed above can each
/// transitively hold a `CompressedImage` above their own ceiling, for the same
/// reason and with the same remedy. That is the whole blast radius — nothing
/// else in the corpus embeds `CompressedImage`.
const WAIVED: &[(&str, &str, &str)] = &[(
    "visualization_msgs/Marker",
    "texture",
    "sensor_msgs/CompressedImage",
)];

// ────────────────────────────── corpus loading ──────────────────────────────

/// Every vendored `.msg`, parsed and nested-resolved as one set — the same
/// shape `build.rs` compiles, so "variable" here means what it means at
/// codegen time.
fn resolved_corpus() -> Vec<MessageSchema> {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("msg");
    let mut pkgs: Vec<PathBuf> = std::fs::read_dir(&root)
        .unwrap_or_else(|e| panic!("read {}: {e}", root.display()))
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.is_dir())
        .collect();
    pkgs.sort();

    let mut schemas = Vec::new();
    for pkg_dir in pkgs {
        // Mirror build.rs's package-name sanitization.
        let pkg = pkg_dir
            .file_name()
            .expect("package dir name")
            .to_string_lossy()
            .replace('-', "_");
        let mut files: Vec<PathBuf> = std::fs::read_dir(&pkg_dir)
            .unwrap_or_else(|e| panic!("read {}: {e}", pkg_dir.display()))
            .flatten()
            .map(|e| e.path())
            .filter(|p| p.extension().is_some_and(|x| x == "msg"))
            .collect();
        files.sort();
        for f in files {
            let name = f
                .file_stem()
                .expect("msg file stem")
                .to_string_lossy()
                .to_string();
            let text = std::fs::read_to_string(&f).unwrap_or_else(|e| panic!("read {f:?}: {e}"));
            // FAIL CLOSED: a file this walk cannot parse is a file whose
            // container rule goes unchecked. `build.rs` parses the same set and
            // escalates any warning to a build failure, so an unparseable file
            // here means the corpus and the build disagree.
            let schema = parse_rosmsg(&text, &name, Some(&pkg))
                .unwrap_or_else(|e| panic!("cannot parse vendored {pkg}/{name}: {e}"));
            schemas.push(schema);
        }
    }
    resolve_fixed_nested(&mut schemas);
    schemas
}

/// Every `Nested` reference reachable from `ft` through array wrappers, as
/// `(reference package, name)`. A `Nested` inside a `FixedArray` or
/// `DynamicArray` is the element type; both put the referenced schema's bytes
/// inside the parent's payload, so both are container relationships.
fn nested_refs(ft: &FieldType, out: &mut Vec<(Option<String>, String)>) {
    match ft {
        FieldType::Nested {
            schema_name,
            package,
            ..
        } => out.push((package.clone(), schema_name.clone())),
        FieldType::FixedArray { element_type, .. } | FieldType::DynamicArray { element_type } => {
            nested_refs(element_type, out)
        }
        _ => {}
    }
}

/// Resolve a reference to a qualified corpus name.
///
/// The vendored corpus is UNIFORMLY QUALIFIED (since the qualification pass, held at zero by
/// `upstream_drift_test::the_vendored_corpus_declares_no_bare_nested_refs`), so
/// the qualified arm is the only one a passing repo ever takes. The bare arms
/// mirror `resolve_fixed_nested`'s own ladder — same-package, then the
/// `Header` → `std_msgs/Header` legacy case, then a unique bare name — so that
/// if the other gate ever fails, THIS one degrades to checking the same pairs
/// the resolver would bind rather than silently skipping them.
fn resolve_ref(
    reference_pkg: &Option<String>,
    name: &str,
    parent_pkg: &str,
    known: &BTreeSet<String>,
) -> Option<String> {
    if let Some(pkg) = reference_pkg {
        let q = format!("{pkg}/{name}");
        return known.contains(&q).then_some(q);
    }
    let same = format!("{parent_pkg}/{name}");
    if known.contains(&same) {
        return Some(same);
    }
    if name == "Header" && known.contains("std_msgs/Header") {
        return Some("std_msgs/Header".to_string());
    }
    let mut hits = known
        .iter()
        .filter(|q| q.split_once('/').is_some_and(|(_, bare)| bare == name));
    match (hits.next(), hits.next()) {
        (Some(only), None) => Some(only.clone()),
        _ => None,
    }
}

/// One container/member relationship the rule applies to.
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord)]
struct Pair {
    container: String,
    field: String,
    member: String,
    container_tier: usize,
    member_tier: usize,
}

/// Every checked pair in the corpus, in a stable order.
fn container_member_pairs() -> Vec<Pair> {
    let schemas = resolved_corpus();
    let known: BTreeSet<String> = schemas.iter().map(|s| s.qualified_name()).collect();
    let variable: BTreeMap<String, bool> = schemas
        .iter()
        .map(|s| (s.qualified_name(), !s.is_definitely_fixed()))
        .collect();

    let mut pairs = Vec::new();
    for schema in &schemas {
        let container = schema.qualified_name();
        // Only variable schemas consult the tier table — a fixed schema's
        // MAX_SLICE_LEN is the auto-computed exact frame size.
        if !variable[&container] {
            continue;
        }
        let parent_pkg = schema.package.clone().unwrap_or_default();
        let container_tier = variable_schema_max_slice_len(&container);

        for field in &schema.fields {
            let mut refs = Vec::new();
            nested_refs(&field.field_type, &mut refs);
            for (ref_pkg, ref_name) in refs {
                let member =
                    resolve_ref(&ref_pkg, &ref_name, &parent_pkg, &known).unwrap_or_else(|| {
                        // FAIL CLOSED: an unresolvable reference is a pair this
                        // walk cannot check, not a pair that passes.
                        panic!(
                            "container rule: {container}.{} references '{}' which is not in the \
                             vendored corpus — the container rule cannot be checked for it",
                            field.name,
                            ref_pkg
                                .as_ref()
                                .map(|p| format!("{p}/{ref_name}"))
                                .unwrap_or_else(|| ref_name.clone())
                        )
                    });
                // A FIXED member is inlined into the container's fixed section
                // and never consults the table.
                if !variable[&member] {
                    continue;
                }
                pairs.push(Pair {
                    container: container.clone(),
                    field: field.name.clone(),
                    member: member.clone(),
                    container_tier,
                    member_tier: variable_schema_max_slice_len(&member),
                });
            }
        }
    }
    pairs.sort();
    pairs
}

// ──────────────────────────────── the gate ────────────────────────────────

#[test]
fn every_container_tier_dominates_its_variable_members() {
    let pairs = container_member_pairs();

    // Anti-tautology: a walk that found nothing would pass vacuously. The
    // corpus has hundreds of these relationships; the floor is deliberately
    // far below the real count so it pins "the walk ran", not a census.
    assert!(
        pairs.len() >= 100,
        "container rule: the container walk found only {} pairs — it is not reaching the corpus, \
         so every assertion below is vacuous",
        pairs.len()
    );

    let waived: BTreeSet<(&str, &str, &str)> = WAIVED.iter().copied().collect();
    let mut violations = Vec::new();
    let mut waivers_used: BTreeSet<(&str, &str, &str)> = BTreeSet::new();

    for p in &pairs {
        if p.container_tier >= p.member_tier {
            continue;
        }
        let key = (p.container.as_str(), p.field.as_str(), p.member.as_str());
        match waived.get(&key) {
            Some(hit) => {
                waivers_used.insert(*hit);
            }
            None => violations.push(format!(
                "  {} (tier {}) embeds {} (tier {}) as field `{}`",
                p.container, p.container_tier, p.member, p.member_tier, p.field
            )),
        }
    }

    assert!(
        violations.is_empty(),
        "CONTAINER RULE VIOLATED — {} container(s) cap BELOW a variable member \
         they embed:\n{}\n\n\
         Each of these is a latent `PayloadTooLarge`: the member can produce a payload its own \
         container cannot hold, and a `Static` iceoryx2 pool cannot grow to take it.\n\n\
         THE FIX IS ALMOST ALWAYS TO RAISE THE CONTAINER, or to lower the member if the member's \
         own worst case × 4 fits the smaller tier (both sides of the pair are edited in \
         `cerulion_core/src/codegen/generator/wire_impl.rs`). A WAIVER (the `WAIVED` table in \
         this file) is the last resort and needs the same thing the existing Marker waiver has: a \
         stated reason why the member's realistic worst case is far below its own tier, plus the \
         blast radius through every container above it.",
        violations.len(),
        violations.join("\n")
    );

    // A waiver that no longer describes a real violation is a live licence to
    // introduce one, so it must be earned on every run.
    let unused: Vec<_> = waived.difference(&waivers_used).collect();
    assert!(
        unused.is_empty(),
        "container rule: {} waiver(s) in `WAIVED` no longer describe a real container-rule \
         violation: {unused:?}\n\n\
         Delete them. A stale waiver silently pre-authorises the exact violation this gate \
         exists to catch — if the pair is re-created later, nothing fails.",
        unused.len()
    );
}

/// The waiver's SCOPE, pinned independently of the gate above.
///
/// The gate proves the waived pair is a real violation; this proves it is the
/// only one, by naming both halves and the two facts that make it exceptional
/// — the member really is a tier above, and every OTHER container of the same
/// member is not (nothing else in the corpus embeds `CompressedImage`).
#[test]
fn the_marker_texture_waiver_is_the_only_one_and_is_scoped_to_that_field() {
    assert_eq!(
        WAIVED.len(),
        1,
        "exactly one container-rule waiver ships; adding a second is a design \
         decision, not a table edit"
    );
    assert_eq!(
        WAIVED[0],
        (
            "visualization_msgs/Marker",
            "texture",
            "sensor_msgs/CompressedImage"
        )
    );

    // The pair really is a tier inversion — stated as the two NUMBERS, so a
    // future retier that removes the inversion fails the unused-waiver arm
    // above with this line as the explanation.
    assert_eq!(
        variable_schema_max_slice_len("visualization_msgs/Marker"),
        16 * 1024 * 1024,
        "Marker stays TIER_LARGE — the waiver's whole subject"
    );
    assert_eq!(
        variable_schema_max_slice_len("sensor_msgs/CompressedImage"),
        128 * 1024 * 1024,
        "CompressedImage is TIER_HUGE — a 4K PNG exceeds 16 MiB outright"
    );

    // Nothing else embeds CompressedImage, so the waiver's blast radius is
    // Marker's own container chain and nothing wider. Asserted from the walk
    // rather than from memory.
    let embedders: BTreeSet<String> = container_member_pairs()
        .into_iter()
        .filter(|p| p.member == "sensor_msgs/CompressedImage")
        .map(|p| p.container)
        .collect();
    assert_eq!(
        embedders,
        ["visualization_msgs/Marker".to_string()]
            .into_iter()
            .collect::<BTreeSet<_>>(),
        "the waiver assumes Marker is the only embedder of CompressedImage — if that changed, \
         the waiver's stated blast radius is wrong"
    );
}

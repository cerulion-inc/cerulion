// SPDX-License-Identifier: AGPL-3.0-only
//! `impl ShmMessage for <Name>` emission for codegen-emitted schemas.
//!
//! Per schema, codegen emits exactly one `impl ShmMessage for <Name>` block
//! on the unit marker `<Name>`. The GAT slots `Reader<'a>` / `Writer<'a>`
//! point at the SHM-backed accessor type:
//!
//! - Fixed schemas: `&'a <Name>Shm` (read) / `&'a mut <Name>Shm` (write).
//! - Variable schemas: `<Name>Shm<'a>` (one type does both, gated on
//!   `&self` / `&mut self` for read/write paths).
//!
//! The legacy `Message` trait + `<Name>Build` / Measurer / Writer / View /
//! `from_wire_bytes` codegen used to live in this module; all of it has been
//! deleted.

use crate::codegen::schema::MessageSchema;
use std::fmt::Write;

/// Generate `impl ShmMessage for <Name>` for a fixed schema.
///
/// `<Name>` is the unit marker; the GATs `Reader<'a>` / `Writer<'a>` resolve
/// to `&'a <Name>Shm` / `&'a mut <Name>Shm` so `OutputProxy<'_, Name>` derefs
/// to the SHM-backed accessor and user code can write fields directly.
pub(super) fn generate_shm_message_trait_impl(out: &mut String, schema: &MessageSchema) {
    let name = &schema.name;
    let shm_name = format!("{name}Shm");
    let hash_const = format!("{}_SCHEMA_HASH", name.to_uppercase());

    writeln!(
        out,
        "// ShmMessage trait implementation for fixed-schema SHM-backed pub/sub."
    )
    .unwrap();
    writeln!(out, "impl cerulion_core::message::ShmMessage for {name} {{").unwrap();
    writeln!(out, "    const SCHEMA_HASH: u64 = {hash_const};").unwrap();
    writeln!(out, "    const VARIABLE_FIELD_COUNT: usize = 0;").unwrap();
    writeln!(
        out,
        "    const WIRE_FIXED_SIZE: usize = ::std::mem::size_of::<{shm_name}>();"
    )
    .unwrap();
    // Fixed-only schemas have a provably-correct upper
    // bound: WireHeader + the entire fixed section. No offset table, no
    // variable payload — every published frame is exactly this size.
    // The budget is typed as `Option<MaxSliceLen>`. The newtype
    // encodes BOTH the upper bound `u32::MAX` and the lower bound
    // `>= WireHeader::SIZE`. `assert!` in const context (stable since
    // 1.57) catches overflow BEFORE the `as u32` cast — pathological
    // schemas with WIRE_FIXED_SIZE pushing the sum past u32::MAX
    // FAIL TO COMPILE. (`u32::try_from` is not yet const-stable as
    // a trait method, hence the `assert! + as` pattern.)
    writeln!(
        out,
        "    const MAX_SLICE_LEN: ::std::option::Option<::cerulion_core::wire::MaxSliceLen> = {{"
    )
    .unwrap();
    writeln!(
        out,
        "        let sum: usize = ::cerulion_core::wire::WireHeader::SIZE"
    )
    .unwrap();
    writeln!(
        out,
        "            + <Self as cerulion_core::message::ShmMessage>::WIRE_FIXED_SIZE;"
    )
    .unwrap();
    writeln!(out, "        assert!(").unwrap();
    writeln!(out, "            sum <= u32::MAX as usize,").unwrap();
    writeln!(
        out,
        "            \"fixed schema MAX_SLICE_LEN (WireHeader::SIZE + WIRE_FIXED_SIZE) overflows u32 — codegen bug\","
    )
    .unwrap();
    writeln!(out, "        );").unwrap();
    writeln!(
        out,
        "        ::std::option::Option::Some(::cerulion_core::wire::MaxSliceLen::const_new(sum as u32))"
    )
    .unwrap();
    writeln!(out, "    }};").unwrap();
    writeln!(out, "    type Reader<'a> = &'a {shm_name};").unwrap();
    writeln!(out, "    type Writer<'a> = &'a mut {shm_name};").unwrap();
    writeln!(out).unwrap();
    writeln!(
        out,
        "    fn build_reader(bytes: &[u8]) -> Self::Reader<'_> {{"
    )
    .unwrap();
    writeln!(out, "        {shm_name}::from_bytes(bytes)").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
    // `build_writer` takes max_capacity + topic for variable-
    // schema overflow accounting. Fixed schemas cannot overflow (constant
    // wire size), so both parameters are unused here. The
    // topic is an `Arc<str>` (was `&'a str`); the underscore-prefix
    // suppresses the unused-variable warning.
    writeln!(
        out,
        "    fn build_writer<'a>(bytes: &'a mut [u8], _max_capacity: ::cerulion_core::wire::MaxPayloadCapacity, _topic: ::std::sync::Arc<str>) -> Self::Writer<'a> {{"
    )
    .unwrap();
    writeln!(out, "        {shm_name}::from_bytes_mut(bytes)").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
    writeln!(
        out,
        "    fn payload_wire_size(_writer: &Self::Writer<'_>) -> usize {{"
    )
    .unwrap();
    writeln!(
        out,
        "        <Self as cerulion_core::message::ShmMessage>::WIRE_FIXED_SIZE"
    )
    .unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
    writeln!(
        out,
        "    fn all_variables_written(_writer: &Self::Writer<'_>) -> bool {{"
    )
    .unwrap();
    writeln!(out, "        true").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out, "}}").unwrap();
    writeln!(out).unwrap();
}

/// Generate `impl ShmMessage for <Name>` for a variable schema.
///
/// Mirror of [`generate_shm_message_trait_impl`] for variable schemas. Both
/// `Reader<'a>` and `Writer<'a>` resolve to `<Name>Shm<'a>` — one type does
/// both. `WIRE_FIXED_SIZE` is `size_of::<<Name>FixedSection>()` (it
/// matches the `#[repr(C)]` overlay including any natural-alignment trailing
/// padding); `VARIABLE_FIELD_COUNT` is the schema's variable-field count.
pub(super) fn generate_variable_shm_message_trait_impl(out: &mut String, schema: &MessageSchema) {
    let name = &schema.name;
    let shm_name = format!("{name}Shm");
    let fixed_section_name = format!("{name}FixedSection");
    let hash_const = format!("{}_SCHEMA_HASH", name.to_uppercase());
    let n_var = schema.variable_field_count();

    writeln!(
        out,
        "// ShmMessage trait implementation for variable-schema SHM-backed pub/sub."
    )
    .unwrap();
    let max_slice_len = variable_schema_max_slice_len(&schema.qualified_name());
    writeln!(out, "impl cerulion_core::message::ShmMessage for {name} {{").unwrap();
    writeln!(out, "    const SCHEMA_HASH: u64 = {hash_const};").unwrap();
    writeln!(out, "    const VARIABLE_FIELD_COUNT: usize = {n_var};").unwrap();
    writeln!(
        out,
        "    const WIRE_FIXED_SIZE: usize = ::std::mem::size_of::<{fixed_section_name}>();"
    )
    .unwrap();
    // Variable schemas in `native_ros2_messages/` ship
    // with a per-schema budget so users do not have to write
    // `max_slice_len:` in graph YAML. Lookup is by QUALIFIED schema
    // name ("pkg/Name" — bare names collide across packages)
    // in `variable_schema_max_slice_len` (categorized into five
    // tiers from 16 KiB to 128 MiB — the launch bump). User-defined
    // schemas (the `_` catch-all in that function) fall through to
    // TIER_HUGE (128 MiB) so unfamiliar schemas reserve enough SHM by
    // default; in-repo schemas must be listed explicitly so tiny ones
    // (Header, ...) do not silently reserve 128 MiB per topic.
    // The budget is typed as `Option<MaxSliceLen>`. Tier budgets
    // are all >= WireHeader::SIZE (16 KiB minimum) and ≤ u32::MAX
    // (128 MiB max), so `MaxSliceLen::const_new` always succeeds.
    // Compile-time panic surface stays — if a future tier change
    // emits an invalid literal, the build fails.
    writeln!(
        out,
        "    const MAX_SLICE_LEN: ::std::option::Option<::cerulion_core::wire::MaxSliceLen> = ::std::option::Option::Some(::cerulion_core::wire::MaxSliceLen::const_new({max_slice_len}));"
    )
    .unwrap();
    writeln!(out, "    type Reader<'a> = {shm_name}<'a>;").unwrap();
    writeln!(out, "    type Writer<'a> = {shm_name}<'a>;").unwrap();
    writeln!(out).unwrap();
    writeln!(
        out,
        "    fn build_reader(bytes: &[u8]) -> Self::Reader<'_> {{"
    )
    .unwrap();
    writeln!(out, "        {shm_name}::from_bytes(bytes)").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
    // `build_writer` plumbs `max_capacity` (the configured
    // max_slice_len minus WireHeader::SIZE) + the publisher's topic into
    // <Name>Shm's `from_bytes_mut` for overflow accounting and error
    // attribution. The topic is an `Arc<str>` (was `&'a
    // str`) so `loan_proxy` no longer needs the raw-pointer reborrow
    // `unsafe` block — Arc::clone is owned and borrow-checker-safe.
    writeln!(
        out,
        "    fn build_writer<'a>(bytes: &'a mut [u8], max_capacity: ::cerulion_core::wire::MaxPayloadCapacity, topic: ::std::sync::Arc<str>) -> Self::Writer<'a> {{"
    )
    .unwrap();
    writeln!(
        out,
        "        {shm_name}::from_bytes_mut(bytes, max_capacity, topic)"
    )
    .unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
    writeln!(
        out,
        "    fn payload_wire_size(writer: &Self::Writer<'_>) -> usize {{"
    )
    .unwrap();
    writeln!(out, "        writer.cursor() as usize").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
    writeln!(
        out,
        "    fn all_variables_written(writer: &Self::Writer<'_>) -> bool {{"
    )
    .unwrap();
    writeln!(out, "        writer.all_variables_written()").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
    // Variable-schema overrides of the trait's default
    // `has_overflow` / `overflow_view_bytes` so `OutputProxy::Drop` can
    // detect the heap-spill path and memcpy the bytes into a re-loan.
    writeln!(
        out,
        "    fn has_overflow(writer: &Self::Writer<'_>) -> bool {{"
    )
    .unwrap();
    writeln!(out, "        writer.has_overflow()").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
    writeln!(out, "    fn overflow_view_bytes<'w>(writer: &'w Self::Writer<'_>) -> ::std::option::Option<&'w [u8]> {{").unwrap();
    writeln!(out, "        writer.overflow_view_bytes()").unwrap();
    writeln!(out, "    }}").unwrap();
    // Schemas with complex-nested variable fields override the
    // trait's no-op `flush_staged_nested` so `OutputProxy::Drop` lands the
    // staged nested writes (the `with_<f>` / leaf-assignment sugar) through
    // `set_<f>_bytes` BEFORE the all-variables gate. Schemas without such
    // fields keep the default (no staging slots exist; the inherent
    // `__cer_flush_staged` — emitted on every variable schema for
    // recursion — is a no-op there anyway).
    if super::structs::schema_has_complex_variable(schema) {
        writeln!(out).unwrap();
        writeln!(
            out,
            "    fn flush_staged_nested(writer: &mut Self::Writer<'_>) -> ::std::result::Result<(), ::cerulion_core::TransportError> {{"
        )
        .unwrap();
        writeln!(out, "        writer.__cer_flush_staged()").unwrap();
        writeln!(out, "    }}").unwrap();
    }
    writeln!(out, "}}").unwrap();
    writeln!(out).unwrap();
}

/// Per-schema `MAX_SLICE_LEN` budget for variable schemas.
///
/// Returns the byte budget (header + fixed section + offset table +
/// variable payload) the publisher will allocate by default when the
/// graph YAML omits an explicit `max_slice_len:` for an output topic
/// of this schema.
///
/// Lookup is by QUALIFIED schema name (`"pkg/Name"`, via
/// `MessageSchema::qualified_name()`), never the bare name — bare names
/// can collide across packages (the same argument that moved the schema
/// hash to FQN input). Package-less user schemas (workspace YAML) have
/// no arm here and fall through to the catch-all.
///
/// Buckets are sized for the realistic worst case + ~4x headroom.
/// Users can always override per-topic in graph YAML; this table is
/// the silent default that keeps in-repo schemas warning-free.
///
/// **Every variable schema in `native_ros2_messages/` must be
/// explicitly listed below.** The `_` catch-all is reserved for
/// user-defined schemas (defaulted to `TIER_HUGE` = 128 MiB so unfamiliar
/// schemas reserve enough SHM by default — free, since pools are lazy).
/// The catch-all is NOT a soft default for in-repo schemas; tiny in-repo
/// schemas would otherwise silently reserve 128 MiB of SHM per topic.
///
/// Categories (launch bump — 4× scale-up; free because `Static`
/// pools are lazy/demand-paged on Linux+macOS, resident = working set):
/// - **TIER_HUGE (128 MiB)** — image-class frames raw AND compressed (8K RGB
///   ≈ 95 MiB; a 4K PNG can exceed 16 MiB), dense point clouds, occupancy
///   grids, octomaps, whole-planning-scene containers, point-cloud-bearing
///   recognition types.
/// - **TIER_LARGE (16 MiB)** — mesh-bearing collision / constraint types,
///   single trajectories and states, marker + interactive-marker containers.
/// - **TIER_MEDIUM (4 MiB)** — paths, pose arrays, whole trajectories,
///   grasp/place descriptors, free-text string payloads (`/robot_description`
///   URDFs run to hundreds of KB). Sized for ~1000 elements of mixed
///   primitive + nested data.
/// - **TIER_SMALL (256 KiB)** — laser scans, joint states/commands, single
///   trajectory points, polygons, controller-state aggregates, multi-arrays of
///   primitives, bulk transform broadcasts. Sized for ~10K primitive elements
///   (or ~1,700 `TransformStamped`s; see the frame-loss note on
///   `tf2_msgs/TFMessage`, the one arm placed for a frame-loss reason rather
///   than a capacity one).
/// - **TIER_TINY (16 KiB)** — Header-stamped scalar wrappers, odometry and
///   camera-calibration records, single-joint/link records, human-input
///   devices. Sized for ~50-byte strings + small fixed payload.
///
/// A tier audit re-checked all 169 arms against realistic worst case × 4 and moved
/// 43 (the audit proposed 44; `sensor_msgs/JoyFeedbackArray` was reverted in
/// review — its arm records why). Its FOLLOW-ON then closed the two items that
/// batch left open, and they are the reason the table is now 194 arms:
///
/// - `control_msgs/JointTrajectoryControllerState` MEDIUM → SMALL, which the
///   audit could only record as blocked (its floor was the two
///   `trajectory_msgs` point types, which that same batch moved to SMALL).
/// - The **catch-all sweep**: six vendored package directories
///   (`autoware_perception_msgs`, `autoware_planning_msgs`, `grid_map_msgs`,
///   `radar_msgs`, `unique_identifier_msgs`, `vision_msgs`) were never
///   registered in `IN_REPO_PACKAGES`, so their 24 variable schemas took the
///   128 MiB user-defined fallback SILENTLY — the panic below cannot see an
///   unregistered package, which is exactly how a `vision_msgs/ObjectHypothesis`
///   (a string and a float) came to reserve the same 128 MiB as an 8K image.
///   All 24 now have arms, all six packages are registered, and the
///   registration is machine-checked against the `msg/` tree by
///   `crates/native_ros2_messages/tests/tier_catch_all_test.rs`. Two of the 24
///   (`grid_map_msgs/GridMap` and `autoware_perception_msgs/PredictedObjects`)
///   KEEP 128 MiB, now as a stated decision with arithmetic rather than as a
///   default nobody had looked at.
///
/// Two rules govern a placement and are stated once here rather than at every
/// arm:
///
/// 1. **Worst case × 4 picks the tier.** The realistic maximum payload for the
///    type, times four for headroom, must fit; the next tier down must not.
///    Every byte count and breakage boundary quoted by an arm the audit's
///    follow-on added or moved is REBUILT AS REAL BYTES and re-checked against
///    this rule on every run by `crates/native_ros2_messages/tests/tier_capacity_test.rs`
///    — which encodes each scenario with `CanonicalBodyBuilder`, the same
///    encoder `CdrCodec::decode` uses for a bridged frame. The older arms'
///    numbers are still hand-computed and unchecked; extending that gate
///    backwards over them is the obvious follow-on.
/// 2. **A container is at least its largest VARIABLE member.** A container
///    below a member is a latent `PayloadTooLarge` — `Static` pools cannot
///    grow — and it is mechanically checked over the whole vendored corpus by
///    `crates/native_ros2_messages/tests/tier_container_rule_test.rs`, which carries
///    the single deliberate waiver (`Marker.texture`, recorded at Marker's arm
///    below).
///
/// Lowering a tier is not free — and, on one path, lowering it is the whole
/// point. A dynamically-created ingress route (a `ros2 attach` bridge route)
/// derives its receive-queue DEPTH from its slice
/// ([`crate::transport::ingress_route_buffer_depth`] = `clamp(64 MiB / slice,
/// 16, 1024)`), and that depth is the frame-loss boundary.
/// The bridge composes it as `min(bridge default 1 MiB, tier)`
/// ([`crate::codegen::route_budget`]), so in bridge terms TIER_TINY buys depth
/// 1024, TIER_SMALL 256, and every tier from MEDIUM up stays at the default's
/// 64. That is why the SMALL and TINY downshifts below are described as
/// "buying 4x depth": each moves a genuinely streaming type across one of
/// those two steps. Every one of them was checked to still fit by rule 1
/// first — absorption never bought at the cost of a reachable
/// `PayloadTooLarge`.
///
/// `pub` (re-exported as [`crate::codegen::variable_schema_max_slice_len`])
/// because a dynamically-created ingress route resolves its schema by NAME at
/// run time and has no generated type to read `MAX_SLICE_LEN` from — see
/// [`crate::codegen::route_budget`].
///
/// This wrapper is the CODEGEN-policy face over the shared table
/// match (`variable_schema_listed_tier`) — an unlisted in-repo name PANICS
/// here (a build-time gate: codegen only ever asks for variable schemas, so
/// an unlisted in-repo variable schema is a forgotten arm). The RUNTIME
/// lookup ([`crate::codegen::slice_ceiling_for_type`]) resolves through the
/// SAME match but degrades loudly instead of panicking, because at run time
/// the caller may hand it any name (a fixed-layout type's, say) and a library
/// must not panic on that.
pub fn variable_schema_max_slice_len(qualified_name: &str) -> usize {
    match variable_schema_listed_tier(qualified_name) {
        Some(bytes) => bytes,
        None => {
            if let Some(pkg) = in_repo_package(qualified_name) {
                panic!(
                    "variable schema '{qualified_name}' from in-repo package '{pkg}' has no \
                     explicit MAX_SLICE_LEN tier arm — add one to \
                     variable_schema_max_slice_len (silent 128 MiB catch-all is \
                     reserved for user-defined schemas)"
                );
            }
            UNLISTED_SCHEMA_FALLBACK_BYTES
        }
    }
}

/// The catch-all budget an UNLISTED schema takes (user-defined schemas — see
/// the `None`-arm policy on [`variable_schema_max_slice_len`] and on the
/// runtime lookup). ONE number, exported crate-wide, so the runtime lookup
/// (`codegen::slice_ceiling`) can never drift from the codegen
/// wrapper.
pub(crate) const UNLISTED_SCHEMA_FALLBACK_BYTES: usize = TIER_HUGE;

/// The package half of the catch-all guard: `Some(pkg)` iff `qualified_name`
/// is `pkg/Type`-spelled AND `pkg` is a vendored `crates/native_ros2_messages/msg/`
/// directory ([`IN_REPO_PACKAGES`]). Shared by the codegen wrapper's panic arm
/// above and the runtime lookup's loud-degrade arm
/// ([`crate::codegen::slice_ceiling_for_type`]) so the two faces can never
/// disagree on what "in-repo" means.
pub(crate) fn in_repo_package(qualified_name: &str) -> Option<&str> {
    let (pkg, _) = qualified_name.split_once('/')?;
    IN_REPO_PACKAGES.contains(&pkg).then_some(pkg)
}

// Launch bump: full 4× scale-up. iceoryx2 `Static`
// pools are LAZY/demand-paged on Linux+macOS (measured: a 16 MiB-tier
// topic with 1 KiB payloads = ~20 KB resident vs 2.3 GB apparent; 24
// pools = 54.8 GiB apparent → 0.84 MiB resident on a 15 GiB Jetson), so
// the per-slot tier-max reservation costs ~0 real RAM — bumping is free.
// The old 16 MiB HUGE hard-FAILED `PayloadTooLarge` for a 4K RGB Image
// (~24 MiB) / 8K (~95 MiB) since Static can't grow.
const TIER_HUGE: usize = 128 * 1024 * 1024;
const TIER_LARGE: usize = 16 * 1024 * 1024;
const TIER_MEDIUM: usize = 4 * 1024 * 1024;
const TIER_SMALL: usize = 256 * 1024;
const TIER_TINY: usize = 16 * 1024;

/// The five-tier table MATCH itself — the ONE source of truth.
///
/// `Some(bytes)` for every explicitly listed variable schema; `None` for an
/// unlisted name. POLICY for `None` deliberately lives in the callers, because
/// the two faces need different failure shapes:
///
/// * [`variable_schema_max_slice_len`] (codegen): unlisted in-repo ⇒ PANIC
///   (build-time gate), otherwise the 128 MiB user-schema catch-all.
/// * [`crate::codegen::slice_ceiling_for_type`] (runtime): unlisted
///   in-repo ⇒ loud `error!` + the catch-all — never a panic in a library.
///
/// Do NOT add a third copy of the table; add callers of this match.
pub(crate) fn variable_schema_listed_tier(qualified_name: &str) -> Option<usize> {
    match qualified_name {
        // ─────────────── TIER_HUGE — 128 MiB ───────────────
        // sensor_msgs: raw frames + dense clouds.
        "sensor_msgs/Image"
        | "sensor_msgs/PointCloud2"
        // LARGE -> HUGE. A typical 1080p JPEG is ~250 KB, but the
        // realistic worst is a 4K PNG over `image_transport`: raw 3840x2160x3
        // is 24.9 MB and PNG lands at 55-95 % of raw (13.7-24 MB), so a
        // high-entropy 4K PNG, a 4K lossless WebP, and ANY 8K PNG exceed the
        // 16 MiB LARGE ceiling OUTRIGHT: the same hard-fail the launch-bump note
        // above records for a raw 4K Image, on a path that is live today.
        // Worst x 4 ~= 55 MB. `visualization_msgs/Marker` embeds a
        // `CompressedImage texture` and deliberately STAYS at LARGE; the
        // waiver and its blast radius are recorded at Marker's arm.
        | "sensor_msgs/CompressedImage"
        // nav_msgs: full 2D map.
        | "nav_msgs/OccupancyGrid"
        // moveit_msgs: whole-scene containers (meshes + octomap) and multi-trajectory responses.
        | "moveit_msgs/PlanningScene"
        | "moveit_msgs/PlanningSceneWorld"
        | "moveit_msgs/PlanningOptions"
        | "moveit_msgs/DisplayTrajectory"
        | "moveit_msgs/MotionPlanDetailedResponse"
        | "moveit_msgs/MotionSequenceResponse"
        // octomap_msgs: full 3D occupancy maps.
        | "octomap_msgs/Octomap"
        | "octomap_msgs/OctomapWithPose"
        // object_recognition_msgs: arrays of objects each carrying point clouds + meshes.
        | "object_recognition_msgs/RecognizedObjectArray"
        // Both moved LARGE -> HUGE, and both were LIVE container-rule
        // violations — each embeds a `sensor_msgs/PointCloud2` (HUGE), so each
        // could be handed a payload its own ceiling cannot hold. The arithmetic
        // agrees independently: one organized 848x480 cloud @ 32 B/pt ~= 13 MB
        // and three 200k-point sensor clusters ~= 19.2 MB (RecognizedObject);
        // a 2M-point ground-truth scan @ 16 B = 32 MB and a 1M-triangle scan
        // mesh ~= 24 MB (ObjectInformation — whose upstream field comment says
        // the mesh "can be big"). RecognizedObject moving here also repairs an
        // inverted element/array floor: RecognizedObjectArray was already HUGE.
        | "object_recognition_msgs/RecognizedObject"
        | "object_recognition_msgs/ObjectInformation"
        // TIER FOLLOW-ON: the two CATCH-ALL types that genuinely need
        // this size. They KEEP 128 MiB, but they keep it as a decision with an
        // arm and arithmetic rather than as the user-defined fallback nobody
        // had looked at. The audit's job is to make every tier deliberate, not
        // to make every tier smaller.
        //
        //   grid_map_msgs/GridMap   The multi-layer float generalisation of
        //        `nav_msgs/OccupancyGrid` (already TIER_HUGE) — same 2D grid,
        //        4 B per cell instead of 1, times N layers. MEASURED: a 3-layer
        //        10 m map at 3 cm (333x333) is 1,320,763 B (x 4 = 5,283,052,
        //        which alone would say LARGE), an 8-layer 500x500 is
        //        8,001,693 B (x 4 = 32,006,772 — already past LARGE), and an
        //        8-layer 100 m map at 10 cm (1000x1000) is 32,001,693 B, whose
        //        x 4 = 128,006,772 sits within a rounding error of this tier's
        //        own ceiling. Elevation-mapping stacks publish exactly that
        //        shape. Floor also holds: its member `std_msgs/Float32MultiArray`
        //        is SMALL
        //   autoware_perception_msgs/PredictedObjects   The
        //        `map_based_prediction` output: an array of objects each of
        //        which can reach 564,816 B at the IDL's own per-object bound
        //        (see PredictedObject below). A realistic dense-urban frame —
        //        100 objects at a realistic 6-paths-x-50-poses each — is
        //        1,851,693 B, and x 4 = 7,406,772 would say LARGE; but LARGE's
        //        16 MiB hard-fails at 29 objects filled to the per-object
        //        bound, which is inside the object count a dense scene reaches.
        //        HUGE holds 237 of them. When the realistic-average tier and
        //        the conforming-producer tier disagree, take the safe one —
        //        the same call as the VDA5050State amendment
        | "grid_map_msgs/GridMap"
        | "autoware_perception_msgs/PredictedObjects"
        => Some(TIER_HUGE),

        // ─────────────── TIER_LARGE — 16 MiB ───────────────
        // sensor_msgs: legacy point cloud.
        // (sensor_msgs/CompressedImage moved to TIER_HUGE — see the note there.)
        "sensor_msgs/PointCloud"
        // visualization_msgs: arrays of large complex types.
        | "visualization_msgs/MarkerArray"
        | "visualization_msgs/InteractiveMarkerInit"
        | "visualization_msgs/InteractiveMarkerUpdate"
        | "visualization_msgs/MeshFile"
        // Marker gained upstream Jazzy's `sensor_msgs/CompressedImage
        // texture` + `visualization_msgs/MeshFile mesh_file`, both of which are
        // themselves TIER_LARGE. A container whose ceiling is below its own
        // members' is a latent PayloadTooLarge, so Marker moves MEDIUM -> LARGE.
        //
        // CONTAINER-RULE WAIVER for Marker.texture:
        // CompressedImage moved LARGE -> HUGE (a 4K PNG exceeds 16 MiB
        // outright), and Marker deliberately does NOT follow it. A Marker
        // `texture` is a MESH DECAL sampled by a textured mesh, not a camera
        // frame; 16 MiB of decal is already past any realistic authoring
        // pipeline. Cascading Marker to HUGE would drag MarkerArray,
        // InteractiveMarkerControl, InteractiveMarker and
        // InteractiveMarkerInit/Update with it, sizing the entire marker plane
        // for a payload no marker produces.
        //
        // Stating the cost plainly, because a waiver breaks the transitivity
        // the rule otherwise gives for free: those five containers can each
        // transitively hold a CompressedImage above their own ceiling, for the
        // same reason and with the same remedy. Being wrong is BOUNDED and
        // LOUD — `PayloadTooLarge` names the topic, `Static` pools cannot grow
        // so nothing is silent — and the escape hatch is per-topic and
        // explicit: `max_slice_len:` on the output. Nothing else in the
        // vendored corpus embeds CompressedImage, so that is the whole blast
        // radius; the waiver is machine-scoped in
        // `native_ros2_messages/tests/tier_container_rule_test.rs`, which also
        // fails if it ever stops describing a real violation.
        | "visualization_msgs/Marker"
        // MEDIUM -> LARGE, as a pair — the cascade has no valid
        // intermediate state. InteractiveMarkerControl carries `Marker[]
        // markers` and Marker is LARGE (it moved there for exactly the
        // embedded texture/MeshFile this note is about), so a control holding
        // one 100K-triangle embedded-STL MeshFile marker (~5 MB) already
        // overflows the 4 MiB MEDIUM ceiling today; InteractiveMarker carries
        // those controls. Worst ~5 MiB x 4 ~= 20 MiB, so LARGE at member
        // equality, the same convention used for Marker itself. This
        // also removes a pre-existing inconsistency: InteractiveMarkerInit and
        // InteractiveMarkerUpdate were already LARGE over a MEDIUM transitive
        // member. Zero depth cost: both sides of MEDIUM -> LARGE derive depth
        // 64, and these are interaction-rate types with no burst stake.
        | "visualization_msgs/InteractiveMarkerControl"
        | "visualization_msgs/InteractiveMarker"
        // moveit_msgs: mesh-bearing collision/constraint types + single trajectories/states.
        | "moveit_msgs/CollisionObject"
        | "moveit_msgs/AttachedCollisionObject"
        | "moveit_msgs/RobotState"
        | "moveit_msgs/DisplayRobotState"
        | "moveit_msgs/RobotTrajectory"
        | "moveit_msgs/GenericTrajectory"
        | "moveit_msgs/CartesianTrajectory"
        | "moveit_msgs/MotionPlanRequest"
        | "moveit_msgs/MotionPlanResponse"
        | "moveit_msgs/MotionSequenceItem"
        | "moveit_msgs/MotionSequenceRequest"
        // (moveit_msgs/Grasp + PlaceLocation moved to TIER_MEDIUM — see the note there.)
        | "moveit_msgs/BoundingVolume"
        | "moveit_msgs/PositionConstraint"
        | "moveit_msgs/Constraints"
        | "moveit_msgs/TrajectoryConstraints"
        | "moveit_msgs/PositionIKRequest"
        // (object_recognition_msgs/RecognizedObject + ObjectInformation moved to
        // TIER_HUGE, and TableArray to TIER_MEDIUM — see the notes there.)
        => Some(TIER_LARGE),

        // ─────────────── TIER_MEDIUM — 4 MiB ───────────────
        // nav_msgs: paths and grid-cell arrays.
        "nav_msgs/Path"
        | "nav_msgs/GridCells"
        // shape_msgs: triangulated mesh.
        | "shape_msgs/Mesh"
        // visualization_msgs: 2D image-plane markers.
        // (visualization_msgs/Marker moved to TIER_LARGE — see the note there;
        // InteractiveMarker + InteractiveMarkerControl followed it in the tier audit.)
        | "visualization_msgs/ImageMarker"
        // geometry_msgs: pose-array container.
        // (the four geometry_msgs/Polygon* types moved to TIER_SMALL — see the
        // tier-audit note there.)
        | "geometry_msgs/PoseArray"
        // (tf2_msgs/TFMessage moved to TIER_SMALL — see the note there.)
        // diagnostic_msgs: array of statuses.
        | "diagnostic_msgs/DiagnosticArray"
        // trajectory_msgs: whole trajectories (arrays of points).
        // (the two *Point types are single points and moved to TIER_SMALL —
        // see the note there.)
        | "trajectory_msgs/JointTrajectory"
        | "trajectory_msgs/MultiDOFJointTrajectory"
        // sensor_msgs: multi-echo + channel arrays.
        | "sensor_msgs/MultiEchoLaserScan"
        | "sensor_msgs/ChannelFloat32"
        // moveit_msgs: N-by-N link matrices.
        | "moveit_msgs/AllowedCollisionMatrix"
        // LARGE -> MEDIUM. Both are grasp descriptors whose largest
        // members are `trajectory_msgs/JointTrajectory` postures (MEDIUM), so
        // MEDIUM is their container FLOOR and equality-at-floor is the table's
        // own established convention. Own worst case is far below it: Grasp ~5.4 KB
        // (x 4 = 22 KB), PlaceLocation ~4 KB — LARGE missed the rule by ~700x.
        // Zero depth cost (both sides derive 64); taken for rule consistency.
        | "moveit_msgs/Grasp"
        | "moveit_msgs/PlaceLocation"
        // LARGE -> MEDIUM. 25 dense tables x 5 KB = 125 KB; x 4 =
        // 500 KB, which is over SMALL's 256 KiB and 8x under MEDIUM's 4 MiB.
        // LARGE missed the rule by ~32x. Floor holds: its member `Table` is
        // SMALL. Zero depth cost; taken for rule consistency.
        | "object_recognition_msgs/TableArray"
        // control_msgs: controller-state aggregates + primitive sequences.
        // AdmittanceControllerState STAYS here (a tier-audit refutation): its own
        // worst case is ~8 KB (x 4 = 32 KB) and SMALL was proposed, but it
        // embeds `std_msgs/String ft_sensor_frame`, which the tier audit moved to
        // MEDIUM — so MEDIUM is its container floor and there is no tier
        // between SMALL and MEDIUM to amend into. Re-open only if String is
        // ever moved back below MEDIUM.
        | "control_msgs/AdmittanceControllerState"
        | "control_msgs/HardwareDiagnostics"
        | "control_msgs/HardwareStatus"
        // (control_msgs/JointTrajectoryControllerState moved to TIER_SMALL —
        // see the follow-on note there.)
        | "control_msgs/JointWrenchTrajectory"
        | "control_msgs/MotionPrimitiveSequence"
        // std_msgs/String: TINY -> MEDIUM. THE correctness fix of the
        // batch: `/robot_description` is a latched String published by
        // `robot_state_publisher` on essentially every ROS robot, and a URDF is
        // ~60 KB on a Go2-class quadruped, ~150 KB on an arm+gripper cell, and
        // 400-500 KB humanoid-class — ALL above TINY's ~16.3 KB payload budget,
        // so publishing one hard-fails `PayloadTooLarge` TODAY. SMALL fails the
        // 500 KB case outright before headroom; worst ~1 MiB x 4 = 4 MiB makes
        // MEDIUM the MINIMUM correct tier. The depth cost is real and accepted
        // (1024 -> 64): the >16 KB payloads are one-shot latched documents,
        // while high-rate String telemetry is tens of bytes and cannot overflow
        // 64 frames. Correctness outranks absorption.
        | "std_msgs/String"
        // TIER FOLLOW-ON: the CATCH-ALL packages, MEDIUM group. These are
        // the ones a CONFORMING producer can genuinely fill past SMALL, so
        // SMALL is not merely tight for them, it is a hard fail. Measured
        // through `CanonicalBodyBuilder`, as above.
        //
        //   autoware PredictedObjectKinematics  Upstream declares
        //        `PredictedPath[<=100]` and each path `Pose[<=100]`, so the
        //        IDL maximum is 563,460 B — SMALL (256 KiB) cannot hold ONE
        //        such message, and 563,460 x 4 = 2,253,840 fits MEDIUM. A
        //        realistic `map_based_prediction` object (6 lane hypotheses
        //        x 50 poses) is only 18,004 B, but the tier has to hold what
        //        the IDL permits, not the average
        //   autoware PredictedObject            Same shape plus a UUID, a
        //        classification list and a `Shape`: 564,816 B at the IDL
        //        maximum (x 4 = 2,259,264). Its FLOOR is MEDIUM anyway, via
        //        the kinematics member above
        //   autoware LaneletRoute               A mission route. 500 segments
        //        x 4 candidate primitives = 122,729 B (x 4 = 490,916), which
        //        already exceeds SMALL; SMALL would break at 1,069 segments,
        //        reachable by a long urban route. Published ONCE per mission
        //        (latched), so SMALL's 4x depth would buy nothing to weigh
        //        against that — the JoyFeedbackArray call, same reasoning
        //   autoware Trajectory                 The planner output the
        //        trajectory follower consumes, upstream-declared
        //        `TrajectoryPoint[<=10000]` over an 88 B FIXED element:
        //        880,089 B at the bound, x 4 = 3,520,356, just inside MEDIUM.
        //        SMALL holds only 2,977 points — under the declared bound, so
        //        a conforming planner hard-fails there. Breaks past 47,661
        //   radar RadarScan                     A DETECTION list, unlike
        //        RadarTracks. Conventional automotive radar emits ~800
        //        returns (16,089 B), but shipping 4D imaging radar emits up to
        //        ~30,000 (600,089 B) — which SMALL cannot hold at all;
        //        x 4 = 2,400,356. Breaks past 209,710 returns
        //   vision Detection2DArray / 3DArray   The published per-frame
        //        detection topics. 300 detections x 1 hypothesis (YOLO's
        //        `max_det` default) = 166,900 B and 500 x 1 (the nuScenes 3D
        //        cap) = 298,100 B; x 4 = 667,600 / 1,192,400. SMALL holds 471
        //        / 439 single-hypothesis detections, inside the range a dense
        //        scene reaches. Floor also holds: their members
        //        `Detection2D`/`Detection3D` are SMALL
        //   vision LabelInfo                    The class-name map. A
        //        1,000-class map with 64-char names is 78,136 B, and
        //        x 4 = 312,544 exceeds SMALL. Latched and published once, so
        //        again there is no depth to trade for the risk. Breaks past
        //        53,771 classes
        //
        // Every one of these lands on a bridged ingress depth of 64 — the same
        // depth the 128 MiB catch-all gave them, because the bridge composes
        // `min(1 MiB default, tier)`. The win here is not absorption, it is
        // that the reservation now describes the type.
        | "autoware_perception_msgs/PredictedObject"
        | "autoware_perception_msgs/PredictedObjectKinematics"
        | "autoware_planning_msgs/LaneletRoute"
        | "autoware_planning_msgs/Trajectory"
        | "radar_msgs/RadarScan"
        | "vision_msgs/Detection2DArray"
        | "vision_msgs/Detection3DArray"
        | "vision_msgs/LabelInfo"
        => Some(TIER_MEDIUM),

        // ─────────────── TIER_SMALL — 256 KiB ───────────────
        // sensor_msgs: typical scans + joint states.
        // (Joy / CameraInfo / BatteryState / LaserEcho moved to TIER_TINY —
        // see the notes there.)
        "sensor_msgs/LaserScan"
        | "sensor_msgs/JointState"
        // REFUTED in review: a move to TINY was proposed and
        // REVERTED. It fails on two independent grounds, and the second is the
        // one that decides it.
        //
        // (1) The proposed bound was not a WIRE bound. The argument was
        // "JoyFeedback.id is uint8, so at most 256 addressable elements ~= 2 KB".
        // But the wire carries an unbounded `JoyFeedback[]` with no uniqueness
        // constraint anywhere in the pipeline, so a message repeating one id
        // 5,000 times is well-formed and would hard-fail `PayloadTooLarge` at
        // TINY where the 256 KiB tier accepted it.
        //
        // (2) Even taken as a SEMANTIC bound, the arithmetic was wrong: a
        // JoyFeedback command is addressed by the PAIR `(type, id)`, and `type`
        // is also uint8. Restricting `type` to its three DEFINED constants
        // (LED / RUMBLE / BUZZER) gives 3 x 256 = 768 distinct commands, and
        // MEASURED against the real writer the element stride is 8 B, so that
        // is 6,144 B — and 6,144 x 4 = 24 KiB, which does NOT fit TINY's
        // 16 KiB. The table's own worst-case-x-4 rule therefore rejects the
        // move before the wire question is even asked. (Formally the pair space
        // is 65,536 = 512 KiB, which would not fit SMALL either — but that is
        // not a realistic worst case, so SMALL is the right landing and MEDIUM
        // is not.)
        //
        // MEASURED boundaries: TINY holds 2,047 elements, SMALL holds 32,767.
        // And the move bought NOTHING to weigh against that: this is a
        // low-rate command topic addressed TO a joystick, so the 4x ingress
        // depth is worth nothing here. A nonzero breakage class for a nil
        // benefit is never the right trade — the same call, for the same
        // reason, as the VDA5050State amendment (an unbounded free-text field
        // means the semantic bound is not a wire bound, so take the safe tier).
        //
        // Its ten SMALL -> TINY siblings were re-checked against this exact
        // test and all ten clear TINY with an order of magnitude to spare;
        // this arm is the only one that did not.
        | "sensor_msgs/JoyFeedbackArray"
        // MEDIUM -> SMALL. A 100-joint multi-DOF aggregator is
        // ~17.6 KB; x 4 = ~70 KB, comfortably inside 256 KiB (real robots run
        // under 20 multi-DOF joints). SMALL breaks only past ~1,500 multi-DOF
        // joints in one message. Buys 4x ingress depth (64 -> 256) on a
        // robot-state-rate topic.
        | "sensor_msgs/MultiDOFJointState"
        // (nav_msgs/Odometry moved to TIER_TINY — see the note there.)
        // tf2_msgs: bulk transform broadcasts. This arm was deliberately moved
        // MEDIUM -> SMALL, and it is the one arm in this table whose tier
        // was chosen for a FRAME-LOSS reason rather than a capacity one.
        //
        // The capacity arithmetic first, because the move has to be safe before
        // it can be useful: TIER_MEDIUM's own stated design target is ~1000
        // elements of mixed primitive + nested data, and a `TransformStamped` is
        // ~150 B on the wire (a Header with a string frame_id, a string
        // child_frame_id, and a 56-byte Transform), so 1000 of them is ~150 KB
        // and fits TIER_SMALL's 256 KiB with margin. The move therefore PRESERVES
        // the table's own sizing rule for this type; it does not bend it.
        //
        // Why it was worth making: a topic's `max_slice_len` sets the receive
        // queue DEPTH an ingress route gets
        // (`transport::ingress_route_buffer_depth`: clamp(64 MiB / slice, 16,
        // 1024)), the depth is pinned at service CREATE, and the depth is the
        // frame-loss boundary. At 4 MiB a bridged `/tf` derived the
        // bridge-wide 1 MiB default and sat at depth 64 = 38.6 ms of absorption
        // at the Go2's 1.66 kHz — MEASURED losing 12.67 % / 13.15 % under
        // the burst bench, against 0.00 % for a victim at depth 256. At
        // 256 KiB it derives depth 256 = 154 ms, past the 107.9 ms worst drive
        // pass the live robot recorded.
        //
        // THE BREAKAGE BOUNDARY, stated because this is a shared table and the
        // change reaches every graph-declared TFMessage publisher, not just
        // bridged ones: a SINGLE TFMessage above ~1,700 transforms no longer
        // fits and fails its publish LOUDLY (`PayloadTooLarge` — `Static` pools
        // cannot grow), where before the ceiling was ~27,000. Nothing in this
        // repo or on a real robot broadcasts a four-figure transform tree in one
        // message (a humanoid has ~50 links), and the escape hatch is explicit:
        // `max_slice_len:` on the topic's `OutputDef`, or on the bridge mapping.
        | "tf2_msgs/TFMessage"
        // shape_msgs: solid primitive dimensions. Tier-audit refutation: a raise
        // to MEDIUM was proposed on the sole ground that its member
        // `geometry_msgs/Polygon` was MEDIUM — but the container rule is
        // checked against PROPOSED tiers, and Polygon moves to SMALL in this
        // same change, so the violation evaporates and the raise would mint a
        // fresh over-tier. Its own sizes are generous here (a 1000-vertex prism
        // is 12 KB; x 4 = 48 KB). Re-open only if Polygon is moved back up.
        | "shape_msgs/SolidPrimitive"
        // diagnostic_msgs: single status with key/value pairs.
        | "diagnostic_msgs/DiagnosticStatus"
        // std_msgs: every primitive multi-array.
        // (std_msgs/MultiArrayLayout moved to TIER_TINY — see the note there.)
        | "std_msgs/Int8MultiArray"
        | "std_msgs/Int16MultiArray"
        | "std_msgs/Int32MultiArray"
        | "std_msgs/Int64MultiArray"
        | "std_msgs/UInt8MultiArray"
        | "std_msgs/UInt16MultiArray"
        | "std_msgs/UInt32MultiArray"
        | "std_msgs/UInt64MultiArray"
        | "std_msgs/Float32MultiArray"
        | "std_msgs/Float64MultiArray"
        | "std_msgs/ByteMultiArray"
        // action_msgs: bulk goal status.
        | "action_msgs/GoalStatusArray"
        // The four Polygon types moved MEDIUM -> SMALL, ATOMICALLY —
        // container == member at SMALL, so any intermediate state violates the
        // rule. A 5,000-vertex surveyed geofence is ~60 KB; x 4 = 240 KB, just
        // inside 256 KiB, while MEDIUM implied a ~349K-vertex polygon that no
        // scenario produces (nav2 footprints are tens of vertices). SMALL
        // breaks past ~21,800 vertices. Buys 4x ingress depth (64 -> 256) at
        // costmap-update rate, and fixes the live SolidPrimitive(SMALL) ⊃
        // Polygon(MEDIUM) violation from the member side.
        | "geometry_msgs/Polygon"
        | "geometry_msgs/PolygonStamped"
        | "geometry_msgs/PolygonInstance"
        | "geometry_msgs/PolygonInstanceStamped"
        // MEDIUM -> SMALL. These are single POINTS, not point arrays
        // (the old shared "all four are point-array shaped" comment was wrong
        // for them): a 60-DOF JointTrajectoryPoint is 4 arrays x 60 f64 =
        // 1,920 B (x 4 = 8 KB), and a 10-joint MultiDOF point is 1.6 KB
        // (x 4 = 6.4 KB) — MEDIUM was ~2,600x the worst case. SMALL breaks past
        // ~8,100 joints. These stream at 100 Hz-1 kHz as servo setpoints, so
        // the 4x depth (64 -> 256) is operational; the JointTrajectoryPoint
        // move ALSO fixes the live container violation
        // JointWrenchTrajectoryPoint(SMALL) ⊃ JointTrajectoryPoint(MEDIUM)
        // from the member side.
        | "trajectory_msgs/JointTrajectoryPoint"
        | "trajectory_msgs/MultiDOFJointTrajectoryPoint"
        // MEDIUM -> SMALL. rclcpp publishes a CLOSED 5-point
        // statistic vocabulary (~300 B); even a generous 1000-point aggregator
        // is 17 KB, x 4 = 68 KB. TINY was rejected deliberately — 68 KB has no
        // headroom under 16 KiB.
        | "statistics_msgs/MetricsMessage"
        // MEDIUM -> SMALL. A 2,000-point convex hull is 48 KB;
        // x 4 = 193 KB, inside 256 KiB. Breaks past a sustained ~2,700-point
        // hull, which no segmentation stack emits. Buys 4x depth at
        // segmentation rate. Its container TableArray is MEDIUM ≥ SMALL.
        | "object_recognition_msgs/Table"
        // moveit_msgs: contact reports and solver/planner descriptors.
        // (moveit_msgs/AllowedCollisionEntry + ContactInformation moved to
        // TIER_TINY — see the notes there.)
        //
        // All three moved MEDIUM -> SMALL by the table's own
        // worst-case-x-4 rule. A 150-joint work cell's KinematicSolverInfo is
        // ~24 KB (x 4 = 96 KB; MEDIUM was 43x over); 200 planner ids are 8.8 KB
        // (x 4 = 35 KB — over 16 KiB, so SMALL and not TINY); 100 planner
        // params with prose descriptions are 20 KB (x 4 = 80 KB; MEDIUM 200x
        // over). Little rate stake — taken for rule consistency.
        | "moveit_msgs/KinematicSolverInfo"
        | "moveit_msgs/PlannerInterfaceDescription"
        | "moveit_msgs/PlannerParams"
        // control_msgs: per-joint command/state vectors.
        // (control_msgs/EtherCATState + SteeringControllerCommand moved to
        // TIER_TINY — see the notes there.)
        | "control_msgs/InterfaceValue"
        | "control_msgs/JointCommand"
        | "control_msgs/JointJog"
        // Tier-audit refutation: a raise to MEDIUM was proposed because its
        // member `trajectory_msgs/JointTrajectoryPoint` was MEDIUM — but that
        // member moves to SMALL in this same change, so the floor is satisfied
        // here (its other member `WrenchFramed` is TINY; own worst 3.3 KB x 4 =
        // 13 KB). Raising would mint a fresh over-tier and lose depth.
        // Re-open only if JointTrajectoryPoint is moved back up.
        | "control_msgs/JointWrenchTrajectoryPoint"
        | "control_msgs/Keys"
        | "control_msgs/MultiDOFCommand"
        // MEDIUM -> SMALL, the high-rate controller aggregates. Each
        // is orders of magnitude under 256 KiB at its realistic worst, and each
        // streams at controller rate, so the 4x ingress depth (64 -> 256) is
        // operational rather than cosmetic:
        //   DynamicJointState        90-DOF x 12 interfaces ~32 KB (x4 = 130 KB)
        //   MultiDOFStateStamped     100-DOF ~12 KB (x4 = 48 KB)
        //   Float64Values            ~1000 values 8 KB (x4 = 32 KB); its twin
        //                            std_msgs/Float64MultiArray was ALREADY
        //                            SMALL, so MEDIUM was internally inconsistent
        //   DynamicInterfaceValues   100 interfaces/side ~10 KB (x4 = 40 KB)
        //   DynamicInterfaceGroupVal 50 groups x 20 interfaces ~45 KB (x4 = 180 KB)
        //   SteeringControllerStatus 16-wheel platform 740 B (x4 = 3 KB)
        //   MotionPrimitive          worst 15 KB (x4 = 60 KB); its container
        //                            MotionPrimitiveSequence stays MEDIUM >= SMALL
        | "control_msgs/DynamicJointState"
        | "control_msgs/MultiDOFStateStamped"
        | "control_msgs/Float64Values"
        | "control_msgs/DynamicInterfaceValues"
        | "control_msgs/DynamicInterfaceGroupValues"
        | "control_msgs/SteeringControllerStatus"
        | "control_msgs/MotionPrimitive"
        // MEDIUM -> SMALL, the ~1 Hz diagnostic/telemetry aggregates.
        //   HardwareDeviceDiagnostics 100 x 200-B KeyValue = 20 KB (x4 = 80 KB);
        //                             DiagnosticStatus, a LARGER shape, was
        //                             already SMALL
        //   BatteryStateArray         10 packs x 2 KB = 20 KB (x4 = 80 KB)
        //   GenericHardwareState      verbose worst ~3 KB (x4 = 12 KB); SMALL
        //                             not TINY, for free-text KeyValue margin
        //   VDA5050State              AMENDED from a proposed TINY: worst 2.5 KB
        //                             (dominated by free-text `error_description`)
        //                             x 4 = 10 KB would consume 62 % of TINY,
        //                             the type is Lyrical-new with no deployment
        //                             history to bound vendor behaviour, and
        //                             under-tiering here hard-fails on the ERROR
        //                             path. At its 1-2 Hz cadence TINY's depth
        //                             buys nothing over SMALL's, so take the
        //                             safe tier
        //   HardwareDeviceStatus      4 standards x worst records ~15 KB
        //                             (x4 = 60 KB). Its container floor is
        //                             satisfied by the three moves above it in
        //                             this list plus EtherCATState (TINY) and
        //                             CANopenState (fixed) — which is why all
        //                             five land in ONE change
        | "control_msgs/HardwareDeviceDiagnostics"
        | "control_msgs/BatteryStateArray"
        | "control_msgs/GenericHardwareState"
        | "control_msgs/VDA5050State"
        | "control_msgs/HardwareDeviceStatus"
        // TIER FOLLOW-ON: MEDIUM -> SMALL. The audit recorded this one as
        // "checked_ok at MEDIUM only because it was pinned by the
        // trajectory-point members" and deferred it; the audit's own batch then
        // moved `trajectory_msgs/JointTrajectoryPoint` and
        // `MultiDOFJointTrajectoryPoint` MEDIUM -> SMALL, which lowered this
        // container's FLOOR to SMALL and unblocked it. Both rules now agree,
        // which is why it lands at SMALL and not lower:
        //
        //   Rule 1 (worst x 4). A JTC instance covers ONE ros2_control joint
        //   group, and the message carries the group's names plus FOUR
        //   JointTrajectoryPoints (reference/feedback/error/output), each with
        //   four float64 arrays over those joints. MEASURED through the
        //   canonical encoder (25-char joint names, 20-char frame_id, no
        //   multi-DOF joints): 6 joints (arm) = 1,407 B; 12 = 2,349 B;
        //   30 (humanoid whole-body) = 5,175 B; 60 = 9,885 B. Taking the
        //   60-DOF humanoid as the realistic worst, 9,885 x 4 = 39,540 B fits
        //   SMALL's 256 KiB and does NOT fit TINY's 16 KiB — rule 1 lands on
        //   SMALL from both sides.
        //
        //   Rule 2 (container floor). Its variable members are
        //   `std_msgs/Header` (TINY) and the two trajectory-point types
        //   (SMALL), so SMALL is exactly the floor — equality-at-floor, the
        //   table's own established convention.
        //
        // THE BREAKAGE BOUNDARY, stated because this is a shared table and the
        // change reaches every graph-declared publisher of this type, not just
        // bridged ones: MEASURED, SMALL holds a state message for 1,666 joints
        // (or 410 multi-DOF joints), where MEDIUM held ~26,700. Above that the
        // publish fails LOUDLY (`PayloadTooLarge` — `Static` pools cannot grow),
        // and the escape hatch is explicit and per-topic: `max_slice_len:` on
        // the topic's `OutputDef`, or on the bridge mapping. 1,666 is 28x the
        // 60-DOF humanoid whole-body group and there is no four-figure single
        // controller group — ros2_control instantiates one JTC per group, so
        // a 1,000-joint machine is many controllers, not one message.
        //
        // Why it was worth making: `~/controller_state` is published at the
        // controller's own rate (100 Hz - 1 kHz on a real servo loop), and
        // MEDIUM -> SMALL takes a bridged route's ingress depth from 64 to 256
        // (the bridge composes `min(1 MiB default, tier)`, so MEDIUM and every
        // tier above it collapse to the default's depth 64 — see
        // `crate::codegen::route_budget`). That is the absorption
        // lever, on one of the few controller topics that genuinely streams.
        | "control_msgs/JointTrajectoryControllerState"
        // TIER FOLLOW-ON: the CATCH-ALL packages, SMALL group. Six
        // package directories were vendored without being registered in
        // `IN_REPO_PACKAGES`, so their 24 variable schemas took the 128 MiB
        // user-defined catch-all silently. Every number below is MEASURED
        // through `CanonicalBodyBuilder` — the encoder `CdrCodec::decode`
        // itself uses to build a bridged frame — and re-measured on every run
        // by `native_ros2_messages/tests/tier_capacity_test.rs`.
        //
        //   autoware PredictedPath   Upstream declares `Pose[<=100]`, and a
        //                            `Pose` resolves FIXED at a 56 B stride, so
        //                            the IDL maximum is an exact 5,652 B;
        //                            x 4 = 22,608 clears TINY's 16 KiB, so
        //                            SMALL. Breaks past 4,680 poses (47x the
        //                            declared bound)
        //   autoware Shape           FLOOR-placed, not size-placed: an object
        //                            footprint of 100 vertices is 1,280 B
        //                            (x 4 = 5,120, which is TINY's class), but
        //                            it embeds `geometry_msgs/Polygon` (SMALL),
        //                            so SMALL is the floor and TINY is
        //                            unavailable
        //   radar RadarTracks        A radar OBJECT list, not a detection list:
        //                            the track table is bounded by the sensor
        //                            (ARS-class 100, byte-indexed protocols
        //                            255). 255 tracks x a 216 B FIXED
        //                            `RadarTrack` = 55,169 B; x 4 = 220,676
        //                            fits SMALL, TINY cannot hold one frame.
        //                            Breaks past 1,213 tracks — a per-sensor
        //                            topic cannot reach it; a FUSION node
        //                            aggregating four sensors into one
        //                            `RadarTracks` could, and takes
        //                            `max_slice_len:`
        //   vision Classification    A full 1,000-class softmax dump (the
        //                            widest thing this type is for) with
        //                            32-char class ids is 52,100 B;
        //                            x 4 = 208,400. Breaks past 5,039
        //                            hypotheses, so a 21k-class vocabulary
        //                            needs `max_slice_len:`
        //   vision BoundingBox2DArray 1,000 boxes x a 40 B FIXED element =
        //                            40,089 B (x 4 = 160,356); YOLO's own
        //                            `max_det` default is 300. Breaks past
        //                            6,551
        //   vision BoundingBox3DArray 500 boxes x an 80 B FIXED element (the
        //                            nuScenes/Waymo per-frame cap) = 40,089 B;
        //                            x 4 = 160,356. Breaks past 3,275
        //   vision Detection2D/3D    One detection's hypothesis list. Standard
        //                            ROS detectors emit 1 (occasionally top-k);
        //                            100 hypotheses is 40,580 / 40,620 B,
        //                            x 4 = ~162 KB. Breaks past 648 (at
        //                            32-char class ids)
        //
        // All seven are per-frame perception outputs at 10-30 Hz, so the
        // 4x ingress depth (64 -> 256) is operational; and the floor holds in
        // every case (`Header` and `ObjectHypothesisWithPose` are TINY).
        | "autoware_perception_msgs/PredictedPath"
        | "autoware_perception_msgs/Shape"
        | "radar_msgs/RadarTracks"
        | "vision_msgs/BoundingBox2DArray"
        | "vision_msgs/BoundingBox3DArray"
        | "vision_msgs/Classification"
        | "vision_msgs/Detection2D"
        | "vision_msgs/Detection3D"
        => Some(TIER_SMALL),

        // ─────────────── TIER_TINY — 16 KiB ───────────────
        // std_msgs: header + key-value-ish.
        // (std_msgs/String moved to TIER_MEDIUM — see the note there.)
        "std_msgs/Header"
        | "std_msgs/MultiArrayDimension"
        // SMALL -> TINY. An 8-dimension tensor layout is ~1 KB;
        // x 4 = 4 KB. Its own member MultiArrayDimension was already TINY, and
        // all 12 MultiArray containers are SMALL >= TINY. Breaks past ~300
        // dimensions. Essentially never published standalone.
        | "std_msgs/MultiArrayLayout"
        // sensor_msgs: scalar measurements + small structs.
        | "sensor_msgs/Imu"
        | "sensor_msgs/MagneticField"
        | "sensor_msgs/NavSatFix"
        | "sensor_msgs/Range"
        | "sensor_msgs/Temperature"
        | "sensor_msgs/Illuminance"
        | "sensor_msgs/RelativeHumidity"
        | "sensor_msgs/FluidPressure"
        | "sensor_msgs/PointField"
        | "sensor_msgs/TimeReference"
        // SMALL -> TINY, the high-rate sensor/input records. Each has
        // an unbounded array on the wire, so the bound below is always a bound
        // on the PRODUCER, never on the framing — what makes each of these safe
        // is that the realistic worst x 4 clears TINY with an order of
        // magnitude to spare, so the gap between the two is not reachable by
        // anything a real device emits. (`sensor_msgs/JoyFeedbackArray` was in
        // this list and was REVERTED for failing exactly that test — see its
        // arm in TIER_SMALL.) Each is also lockstepped to a stream where 4x
        // ingress depth (256 -> 1024) matters:
        //   Joy         joydev caps axes at 64, so a HOTAS worst is 850 B
        //               (x4 = 3.4 KB). Breaks past ~4,000 elements — no human
        //               input device. 50-100 Hz, safety-relevant teleop
        //   CameraInfo  d[] is <= 14-18 coefficients across EVERY real
        //               distortion model, so worst ~500 B (x4 = 2 KB). Breaks
        //               past a ~2,000-coefficient model. Published in lockstep
        //               with every Image topic at 30-100 Hz
        //   BatteryState voltage entries are bounded by the pack's SERIES count
        //               (96S Tesla-style, ~216S truck pack), worst ~1.7 KB
        //               (x4 = 7 KB). Breaks past ~2,000 cell entries
        //   LaserEcho   real devices report <= 5 echoes per beam, ~60 B
        //               (x4 = 240 B). Its real home MultiEchoLaserScan stays
        //               MEDIUM >= TINY
        | "sensor_msgs/Joy"
        | "sensor_msgs/CameraInfo"
        | "sensor_msgs/BatteryState"
        | "sensor_msgs/LaserEcho"
        // SMALL -> TINY. Odometry's wire size is near-CONSTANT —
        // 688 B of fixed section (including two 288-B covariance matrices) plus
        // two frame strings, so ~1 KB worst; x 4 = 4 KB. It breaks only if
        // frame_id + child_frame_id together exceed ~15.6 KB, which is not a TF
        // name. This also matches its own halves, which were already TINY
        // (PoseWithCovarianceStamped, TwistWithCovarianceStamped). 50-400 Hz on
        // every mobile base; 4x ingress depth (256 -> 1024).
        | "nav_msgs/Odometry"
        // visualization_msgs: per-marker pose + menu + feedback.
        | "visualization_msgs/InteractiveMarkerPose"
        | "visualization_msgs/InteractiveMarkerFeedback"
        | "visualization_msgs/MenuEntry"
        // tf2_msgs: error wrapper.
        | "tf2_msgs/TF2Error"
        // diagnostic_msgs: single key/value.
        | "diagnostic_msgs/KeyValue"
        // geometry_msgs: Header-stamped wrappers around now-FIXED geometry payloads (the nested-only geometry types that used to sit here are reclassified FIXED by resolve_fixed_nested and never consult this table).
        | "geometry_msgs/PoseStamped"
        | "geometry_msgs/PointStamped"
        | "geometry_msgs/QuaternionStamped"
        | "geometry_msgs/Vector3Stamped"
        | "geometry_msgs/TwistStamped"
        | "geometry_msgs/WrenchStamped"
        | "geometry_msgs/AccelStamped"
        | "geometry_msgs/InertiaStamped"
        | "geometry_msgs/TransformStamped"
        | "geometry_msgs/VelocityStamped"
        | "geometry_msgs/PoseWithCovarianceStamped"
        | "geometry_msgs/TwistWithCovarianceStamped"
        | "geometry_msgs/AccelWithCovarianceStamped"
        // moveit_msgs: single-joint/link scalar records.
        | "moveit_msgs/JointConstraint"
        | "moveit_msgs/JointLimits"
        // MoveItErrorCodes gained upstream Jazzy's `string message`
        // / `string source`, which flips it FIXED -> VARIABLE, so it now
        // consults this table. Two short diagnostic strings + an int32.
        | "moveit_msgs/MoveItErrorCodes"
        | "moveit_msgs/LinkPadding"
        | "moveit_msgs/LinkScale"
        | "moveit_msgs/ObjectColor"
        | "moveit_msgs/OrientationConstraint"
        | "moveit_msgs/VisibilityConstraint"
        | "moveit_msgs/WorkspaceParameters"
        | "moveit_msgs/GripperTranslation"
        // SMALL -> TINY. A 500-body AllowedCollisionEntry row is
        // ~500 B (x 4 = 2 KB; SMALL was 128x over) and breaks past a
        // ~4,000-body matrix row; its container AllowedCollisionMatrix stays
        // MEDIUM >= TINY. A ContactInformation report is ~500 B worst
        // (x 4 = 2 KB — literally TINY's documented class) and breaks only on
        // two ~2 KB body-name strings; it bursts at collision-check rate during
        // servoing, so its 4x depth (256 -> 1024) is operational.
        | "moveit_msgs/AllowedCollisionEntry"
        | "moveit_msgs/ContactInformation"
        // object_recognition_msgs: key + db strings.
        | "object_recognition_msgs/ObjectType"
        // SMALL -> TINY. An EtherCATState carries ESI id strings,
        // worst ~300 B (x 4 = 1.2 KB); it breaks past ~16 KB of vendor strings,
        // which no ESI file produces. A SteeringControllerCommand is a Header
        // plus two f64 ~= 120 B (x 4 = 480 B) — the TwistStamped precedent is
        // already TINY — and streams at 10-100 Hz, so it takes the 4x depth.
        | "control_msgs/EtherCATState"
        | "control_msgs/SteeringControllerCommand"
        // control_msgs: single-DOF scalar state records.
        | "control_msgs/JointComponentTolerance"
        | "control_msgs/JointControllerState"
        | "control_msgs/JointTolerance"
        | "control_msgs/MecanumDriveControllerState"
        | "control_msgs/MotionArgument"
        | "control_msgs/PidState"
        | "control_msgs/SingleDOFState"
        | "control_msgs/SingleDOFStateStamped"
        | "control_msgs/VDA5050SafetyState"
        | "control_msgs/WrenchFramed"
        // TIER FOLLOW-ON (the CATCH-ALL packages, TINY group): the small
        // records that were reserving 128 MiB apiece, i.e. the shape this
        // audit exists to kill. Measured through `CanonicalBodyBuilder`.
        //
        //   autoware LaneletPrimitive   int64 id + a short type string
        //        ("lane", "crosswalk"): 73 B; x 4 = 292. Breaks only on a
        //        ~16 KB `primitive_type` string
        //   autoware LaneletSegment     The preferred primitive plus the
        //        candidates at one route segment. Real roads carry <= 8
        //        parallel lanelets; at a generous 20 the message is 993 B
        //        (x 4 = 3,972). Breaks past 362 primitives in ONE segment.
        //        Floor holds at equality — its member LaneletPrimitive is TINY
        //   vision ObjectHypothesis     A class id string + a score: 80 B at a
        //        32-char id (x 4 = 320)
        //   vision ObjectHypothesisWithPose  The same plus a FIXED 344 B
        //        `PoseWithCovariance`: 432 B (x 4 = 1,728). Floor holds at
        //        equality
        //   vision VisionClass          uint16 + a class name: 106 B at a
        //        64-char name (x 4 = 424)
        //   vision VisionInfo           Header + the method + the model
        //        database location + a version. At 512-char paths for BOTH
        //        strings it is 1,612 B (x 4 = 6,448). Breaks only when the two
        //        strings together approach 16 KB
        //
        // Each takes a bridged ingress depth of 1024 where the catch-all gave
        // 64, and each drops a per-topic reservation by ~8,000x.
        | "autoware_planning_msgs/LaneletPrimitive"
        | "autoware_planning_msgs/LaneletSegment"
        | "vision_msgs/ObjectHypothesis"
        | "vision_msgs/ObjectHypothesisWithPose"
        | "vision_msgs/VisionClass"
        | "vision_msgs/VisionInfo"
        => Some(TIER_TINY),

        // ─────────────── Unlisted: `None` — POLICY LIVES IN THE CALLERS ───
        // The 128 MiB user-defined fallback and the in-repo forgotten-arm
        // panic both moved into `variable_schema_max_slice_len` above
        // (the policy split; the fallback is identical to the runtime
        // `DEFAULT_MAX_SLICE_LEN` — 128 MiB after the launch bump —
        // kept congruent so the codegen const matches the tier-3 fallback).
        // Package-less workspace YAML schemas land on `None` by construction
        // (no package prefix). **Every variable schema added to
        // native_ros2_messages MUST get an explicit arm above** — enforced by
        // the wrapper's IN_REPO_PACKAGES panic, which turns a forgotten arm
        // into a build failure instead of a silent 128 MiB SHM reservation
        // per topic.
        _ => None,
    }
}

/// Packages vendored in `crates/native_ros2_messages/msg/`. Schemas from these
/// MUST have explicit tier arms (the catch-all panics for them at codegen
/// time). Third-party packaged schemas fall through to the catch-all
/// silently and can override per-topic in graph YAML.
///
/// MAINTENANCE: add new vendored package directories here AND give their
/// variable schemas tier arms. That instruction was here before the tier audit's
/// follow-on and was not followed: six package directories were vendored
/// (a benchmark message corpus) without being registered, so
/// their 24 variable schemas took the 128 MiB catch-all SILENTLY — the panic
/// below only fires for a package that is LISTED, so an unlisted package is
/// exactly the case it cannot see. A comment is not a gate, so the follow-on
/// added one: `crates/native_ros2_messages/tests/tier_catch_all_test.rs` asserts this
/// list is EQUAL to the set of directories under `msg/`, in both directions.
///
/// `unique_identifier_msgs` is listed while contributing NO arm below, which
/// is correct and deliberate: its only message (`UUID` = `uint8[16]`) resolves
/// FIXED, so it never consults this table at all. Listing it is what makes a
/// future variable field on it a BUILD FAILURE rather than a silent 128 MiB.
///
/// `pub` so the equality gate above can read it; it is descriptive data about
/// the vendored tree, not a knob.
pub const IN_REPO_PACKAGES: &[&str] = &[
    "action_msgs",
    "autoware_perception_msgs",
    "autoware_planning_msgs",
    "builtin_interfaces",
    "control_msgs",
    "diagnostic_msgs",
    "geometry_msgs",
    "grid_map_msgs",
    "moveit_msgs",
    "nav_msgs",
    "object_recognition_msgs",
    "octomap_msgs",
    "radar_msgs",
    "sensor_msgs",
    "shape_msgs",
    "statistics_msgs",
    "std_msgs",
    "tf2_msgs",
    "trajectory_msgs",
    "unique_identifier_msgs",
    "vision_msgs",
    "visualization_msgs",
];

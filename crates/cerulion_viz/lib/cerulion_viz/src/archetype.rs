// SPDX-License-Identifier: AGPL-3.0-only
//! Rerun archetype BUILDERS + the field extractors they run on.
//!
//! The archetype-family builders and the [`FrameValue`] extractors that feed
//! them. The schema-name → archetype TABLE (the "one adapter" that decides
//! WHICH schema becomes WHICH archetype) plus the STRUCTURAL inference for
//! unmapped schemas both live in [`crate::sink`]; this
//! module hosts the builders they dispatch to:
//!
//! | archetype | builder | schemas (see [`crate::sink::classify_schema`]) |
//! |---|---|---|
//! | [`rerun::Points3D`] (+ intensity colours) | [`log_points3d`] | `sensor_msgs/PointCloud2` |
//! | [`rerun::Points3D`] (single point) | [`log_point3d`] | `geometry_msgs/PointStamped` |
//! | [`rerun::Points3D`] (projected ring) | [`log_laserscan`] | `sensor_msgs/LaserScan` |
//! | [`rerun::EncodedImage`] (JPEG, zero decode) | [`log_encoded_image`] | `sensor_msgs/CompressedImage` |
//! | [`rerun::Image`] (raw rgb8/bgr8/mono8) | [`log_raw_image`] | `sensor_msgs/Image` · inferred image-shape |
//! | [`rerun::Image`] (grayscale map) | [`log_occupancy_grid`] | `nav_msgs/OccupancyGrid` |
//! | [`rerun::Transform3D`] | [`log_transform3d_in_frame`] · [`log_rotation3d_in_frame`] | `geometry_msgs/Pose[Stamped]` · `geometry_msgs/Transform[Stamped]` · `sensor_msgs/Imu` · `nav_msgs/Odometry` · inferred pose/transform-shape · inferred planar `{x,y,theta}` · inferred orientation-only |
//! | [`rerun::Boxes3D`] | [`log_boxes3d`] | inferred box-shape (`vision_msgs/BoundingBox3D`) |
//! | [`rerun::Scalars`] (per-component plots) | [`log_scalar`] | `Twist` · `Joy` · `Imu` · `Odometry` · `unitree_go/SportModeState` · inferred numeric harvest ([`harvest_series`]: scalars + nested structs + numeric arrays) |
//! | [`rerun::TextLog`] (+ a rolling `TextDocument` mirror at `<entity>/text`) | [`log_text`] | inferred single-string |
//! | [`rerun::TextDocument`] structured dump ([`structured_dump`]) | [`log_field_dump`] | anything else (AnyValues fallback) |
//!
//! These are BUILDERS: [`crate::sink::dispatch_frame`] is the one entry point
//! that classifies a decoded [`FrameValue`] and calls them (there is no second,
//! state-free dispatch ladder — see `dispatch_frame`'s own doc). All Rerun
//! logging is best-effort: a log error is warned, never propagated into
//! `tick()`.

use std::sync::{Mutex, OnceLock};

use cerulion_core::codegen::{FrameValue, FrameValueKind, NamedValue, PrimArray, PrimType};
use rerun::RecordingStream;

use crate::pointcloud::{
    decode_pointcloud, point_count, resolve_fields, DecodedCloud, FieldsWarnLatch,
};

/// Flood latch for the GENERIC (stateless `FrameValue`) PointCloud2 path —
/// `cloud_from_frame_value` has no per-node state to hang the latch on, so
/// the process shares one (the typed lidar sink holds its OWN latch as node
/// state). See [`FieldsWarnLatch`] for the once-per-regime contract.
static GENERIC_FIELDS_LATCH: Mutex<FieldsWarnLatch> = Mutex::new(FieldsWarnLatch::new());

/// The custom timeline name — the sender's wire `timestamp_ns` IS the
/// timeline (skew is visible, never hidden; clock-skew correction is not implemented).
/// Robots commonly stamp CLOCK_MONOTONIC (time since boot), not wall-clock;
/// [`set_robot_time`] classifies which clock the sender uses so a
/// boot-relative stamp renders as an elapsed duration, never as a misleading
/// 1970-era absolute date.
pub const ROBOT_TIME: &str = "robot_time";

/// The wall-vs-uptime classification threshold: 2001-01-01T00:00:00Z in ns.
/// A stamp at/after it is only plausible as a wall clock (a monotonic stamp
/// would need ~31 years of uptime to reach it); a stamp below it is only
/// plausible as uptime (no wall clock this side of 2001 runs this code).
const WALL_CLOCK_MIN_NS: u64 = 978_307_200 * 1_000_000_000;

/// True when a wire stamp is wall-clock scale (at/after
/// [`WALL_CLOCK_MIN_NS`]), false for an uptime/CLOCK_MONOTONIC-scale stamp.
/// Pure (oracle-tested); the once-per-process latch lives in
/// [`set_robot_time`].
fn wire_clock_is_wall(ts_ns: u64) -> bool {
    ts_ns >= WALL_CLOCK_MIN_NS
}

/// The robot's clock class, latched from the FIRST stamp seen: a Rerun
/// timeline must keep ONE time type, so the classification is decided once
/// per process and never re-evaluated per frame.
static ROBOT_CLOCK_IS_WALL: OnceLock<bool> = OnceLock::new();

/// Stamp the shared robot-time timeline from a wire `timestamp_ns`.
///
/// The sender's clock class is latched ONCE per process from the first stamp
/// (`wire_clock_is_wall`): a wall-clock stamp rides
/// `set_timestamp_nanos_since_epoch` (ns-precise `i64` — Rerun 0.34's
/// replacement for `set_time_nanos`; the float `set_duration_secs` would lose
/// ns precision on epoch-scale stamps), while an uptime stamp (a robot
/// publishing CLOCK_MONOTONIC — common; the Go2 does) rides
/// `set_duration_secs`, which the viewer renders as a human-readable elapsed
/// duration instead of a 1970-era absolute date. f64 seconds preserve ns
/// precision up to ~104 days of uptime, so the duration path is
/// precision-safe by construction (only below-threshold stamps take it). A
/// mixed-clock sender set (some topics wall, some monotonic) gets the
/// first-seen type for ALL frames — an accepted degrade, since one timeline
/// cannot carry two time types.
pub fn set_robot_time(rec: &RecordingStream, timestamp_ns: u64) {
    if *ROBOT_CLOCK_IS_WALL.get_or_init(|| wire_clock_is_wall(timestamp_ns)) {
        rec.set_timestamp_nanos_since_epoch(ROBOT_TIME, timestamp_ns as i64);
    } else {
        rec.set_duration_secs(ROBOT_TIME, timestamp_ns as f64 / 1e9);
    }
}

/// Log an XYZ point cloud (with optional per-point colours) under `entity`.
pub fn log_points3d(rec: &RecordingStream, entity: &str, timestamp_ns: u64, cloud: &DecodedCloud) {
    set_robot_time(rec, timestamp_ns);
    let mut archetype = rerun::Points3D::new(cloud.positions.iter().copied());
    if let Some(colors) = &cloud.colors {
        archetype = archetype.with_colors(
            colors
                .iter()
                .map(|c| rerun::Color::from_rgb(c[0], c[1], c[2])),
        );
    }
    if let Err(e) = rec.log(entity.to_string(), &archetype) {
        tracing::warn!(error = %e, entity, "Rerun: Points3D log failed");
    }
}

/// Log a compressed (JPEG) image under `entity` — zero decode, Rerun renders
/// it natively. `from_file_contents` sniffs the media type from the JPEG
/// magic bytes (the `format` string is available but not required).
pub fn log_encoded_image(rec: &RecordingStream, entity: &str, timestamp_ns: u64, data: &[u8]) {
    set_robot_time(rec, timestamp_ns);
    let archetype = rerun::EncodedImage::from_file_contents(data.to_vec());
    if let Err(e) = rec.log(entity.to_string(), &archetype) {
        tracing::warn!(error = %e, entity, "Rerun: EncodedImage log failed");
    }
}

/// Log one named scalar sample onto its own timeline plot under
/// `entity/name`.
pub fn log_scalar(rec: &RecordingStream, entity: &str, name: &str, timestamp_ns: u64, value: f64) {
    set_robot_time(rec, timestamp_ns);
    let path = format!("{entity}/{name}");
    if let Err(e) = rec.log(path.clone(), &rerun::Scalars::new(std::iter::once(value))) {
        tracing::warn!(error = %e, path, "Rerun: Scalars log failed");
    }
}

/// Log an arbitrary decoded frame as a human-readable STRUCTURED field document
/// so ANY topic is at least inspectable in the viewer (the AnyValues-fallback
/// goal, via the low-drift `TextDocument` archetype). See [`structured_dump`]
/// for the rendering rules.
pub fn log_field_dump(rec: &RecordingStream, entity: &str, timestamp_ns: u64, fv: &FrameValue) {
    set_robot_time(rec, timestamp_ns);
    let text = structured_dump(fv);
    if let Err(e) = rec.log(entity.to_string(), &rerun::TextDocument::new(text)) {
        tracing::warn!(error = %e, entity, "Rerun: TextDocument log failed");
    }
}

/// Zero-copy view of a frame's `data` byte field (the JPEG blob for
/// `sensor_msgs/CompressedImage`), or `None` when absent / not bytes. The
/// generic sink's [`crate::sink`] Image arm uses it.
pub fn image_data_from_frame_value<'a>(fv: &FrameValue<'a>) -> Option<&'a [u8]> {
    field_bytes(fv, "data")
}

/// Extract a `DecodedCloud` from a walker-decoded PointCloud2 frame. The
/// `fields` channel descriptors come from the DEFINED packed encoding when
/// present, else the point_step-inferred default (loud — a documented
/// assumption; see `pointcloud` module docs).
pub fn cloud_from_frame_value(fv: &FrameValue) -> DecodedCloud {
    let point_step = field_u32(fv, "point_step").unwrap_or(0);
    let width = field_u32(fv, "width").unwrap_or(0);
    let height = field_u32(fv, "height").unwrap_or(0);
    let data = field_bytes(fv, "data").unwrap_or(&[]);
    // Every array-shaped variant yields the SAME bytes: a `NestedArray`
    // (a `PointField[]` the walker decoded under the canonical element framing —
    // which the `ros2 attach` CDR path produces, `PointField` being variable)
    // carries the field's slice verbatim in `raw`. Reading it keeps
    // `resolve_fields`'s input byte-for-byte what it was before element framing, so a
    // canonically-framed cloud gets the TRUTHFUL "blob undecodable" diagnosis
    // from the bespoke parser instead of "`fields` empty — inferring the
    // standard XYZ layout" for descriptors that are present and decoded.
    // The walker's decoded `elements` are not consumed here.
    let fields_blob = match fv.field("fields") {
        Some(FrameValueKind::NestedArrayOpaque(b)) => *b,
        Some(FrameValueKind::Bytes(b)) => *b,
        Some(FrameValueKind::NestedArray { raw, .. }) => *raw,
        _ => &[],
    };
    let fields = {
        let mut latch = GENERIC_FIELDS_LATCH
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        resolve_fields(fields_blob, point_step, &mut latch)
    };
    let n_points = point_count(width, height, point_step, data.len());
    // Honor the cloud's byte order (the bridge preserves `is_bigendian`
    // faithfully — hardcoding LE would silently garble a BE producer).
    let big_endian = field_bool(fv, "is_bigendian").unwrap_or(false);
    decode_pointcloud(&fields, data, point_step, n_points, big_endian)
}

/// The named scalar samples for a scalar-ish schema — the six velocity
/// components of a `geometry_msgs/Twist` (or the `twist` inside a
/// `TwistStamped`), or the per-index `axes/i` of a `sensor_msgs/Joy`. Pure —
/// returns `(entity-relative name, value)` pairs the caller plots via
/// [`log_scalar`]. Any other schema yields an empty vec (the caller then
/// falls through to the field dump).
pub fn scalars_from_frame_value(fv: &FrameValue) -> Vec<(String, f64)> {
    match fv.schema_name.as_str() {
        "geometry_msgs/Twist" => twist_scalars(fv),
        // TwistStamped nests the velocities one level down under `twist`.
        "geometry_msgs/TwistStamped" => match fv.field("twist") {
            Some(FrameValueKind::Nested(inner)) => twist_scalars(inner),
            _ => Vec::new(),
        },
        "sensor_msgs/Joy" => match fv.field("axes") {
            Some(FrameValueKind::PrimArray(pa)) => pa
                .iter_f64()
                .enumerate()
                .map(|(i, v)| (format!("axes/{i}"), v))
                .collect(),
            _ => Vec::new(),
        },
        _ => Vec::new(),
    }
}

/// The six Twist velocity components as named scalars.
fn twist_scalars(fv: &FrameValue) -> Vec<(String, f64)> {
    const COMPONENTS: [(&str, &str); 6] = [
        ("linear", "x"),
        ("linear", "y"),
        ("linear", "z"),
        ("angular", "x"),
        ("angular", "y"),
        ("angular", "z"),
    ];
    let mut out = Vec::new();
    for (group, axis) in COMPONENTS {
        if let Some(FrameValueKind::Nested(inner)) = fv.field(group) {
            if let Some(v) = field_f64(inner, axis) {
                out.push((format!("{group}/{axis}"), v));
            }
        }
    }
    out
}

fn field_u32(fv: &FrameValue, name: &str) -> Option<u32> {
    match fv.field(name) {
        Some(FrameValueKind::U32(v)) => Some(*v),
        _ => None,
    }
}

fn field_bool(fv: &FrameValue, name: &str) -> Option<bool> {
    match fv.field(name) {
        Some(FrameValueKind::Bool(b)) => Some(*b),
        _ => None,
    }
}

fn field_f64(fv: &FrameValue, name: &str) -> Option<f64> {
    match fv.field(name) {
        Some(FrameValueKind::F64(v)) => Some(*v),
        Some(FrameValueKind::F32(v)) => Some(*v as f64),
        _ => None,
    }
}

fn field_bytes<'a>(fv: &FrameValue<'a>, name: &str) -> Option<&'a [u8]> {
    match fv.field(name) {
        Some(FrameValueKind::Bytes(b)) => Some(*b),
        _ => None,
    }
}

/// How many nested-message hops the structured dump expands INLINE before it
/// stops and says so.
///
/// **A HOP IS NOT A LEVEL: what this constant buys depends on the PATH SHAPE
/// (see the table).** The dump indents by ROW, and an array costs an extra row that
/// a plain nested message does not:
///
/// | path step | dump rows spent | where the child's row lands |
/// |---|---|---|
/// | plain nested field | 1 | `depth + 1` ([`dump_fields`] recurses directly) |
/// | struct-array element | 2 | `depth + 2` (an `[i]` row from [`dump_elements`], THEN the members) |
///
/// So "how deep does the dump reach" has no single answer — a chain of plain
/// hops reaches `DUMP_MAX_DEPTH - 1`, while a member under one array element
/// reaches it two rows sooner. Reading this constant as a hop budget suggests,
/// wrongly, that "the dump reaches one hop DEEPER than the harvest"; reading
/// it as a fixed two-number rule models plain nesting only, electing a
/// bank-of-banks at the message root whose members render as `Type {…}`.
///
/// The durable answer is not more arithmetic here: the harvest THREADS the dump
/// row a field will land on (`dump_depth`, incremented per the table above) and
/// asks [`dump_row_renders`]. Nothing derives it from a hop count.
const DUMP_MAX_DEPTH: u8 = 4;

/// Is a row at dump depth `dump_depth` within the dump's DEPTH
/// reach?
///
/// Uniform once the depth is right: [`dump_fields`] is entered for depths
/// `0 ..= DUMP_MAX_DEPTH - 1`, and [`dump_elements`] gates an element's members
/// by the same comparison one row further in. Everything path-shape-specific
/// lives in how `dump_depth` was ACCUMULATED (see [`DUMP_MAX_DEPTH`]), never
/// here.
///
/// # What this does NOT model — the LINE budget
///
/// **Depth is one of THREE reach dimensions, and this answers only that one.**
/// [`DUMP_MAX_LINES`] is a DOCUMENT-WIDE budget shared across every field, so a
/// row well inside the depth ceiling can still be cut for BREADTH — and the
/// election deliberately does not model it. Earlier drafts called this
/// "the ONE reach rule", which was an overclaim: a rule has to name what it
/// leaves out, or the next reader inherits the gap as a guarantee.
///
/// The reaching shape is constructible and is pinned by
/// `the_line_ceiling_is_a_dimension_the_election_does_not_model`: 250 plain
/// scalars (BELOW [`MAX_TOTAL_SERIES`], so no `TotalCap` de-election) followed by
/// one `*_covariance` array LAST — the message's only withholding. It elects
/// (its row is at depth 0) while the dump exhausts its 200 lines long before
/// field 251, so the matrix never renders.
///
/// **Deliberately not modelled, on cost.** Predicting the line budget means
/// predicting breadth, per-array preview counts and field ORDERING — expensive,
/// and re-derivable wrong in exactly the way the depth arithmetic was three
/// times. What makes leaving it out acceptable is that this gap DISCLOSES ITSELF,
/// which the depth gaps did not: [`structured_dump`] appends its
/// `(truncated — message has more fields than the dump renders)` marker, and the
/// operator line names the 200-line bound outright. The reader is told the pane
/// stopped early; they are not told a field is absent that the copy claimed was
/// present.
///
/// Nothing vendored or Go2-shaped comes close (`unitree_go/LowState` declares 22
/// top-level fields), and it is breadth- AND ordering-dependent: the same fields
/// with the covariance FIRST render it fine.
const fn dump_row_renders(dump_depth: u8) -> bool {
    dump_depth < DUMP_MAX_DEPTH
}

/// Dump rows spent by descending into a STRUCT-ARRAY element: the `[i]` row plus
/// the member row under it.
const DUMP_ROWS_PER_ELEMENT_HOP: u8 = 2;

/// Dump rows spent by descending into a plain nested message.
const DUMP_ROWS_PER_PLAIN_HOP: u8 = 1;

/// Can the dump show the MEMBERS of an element of a struct array whose
/// own row is at `array_dump_depth`?
///
/// The election in [`declares_withheld_series`] promises the operator that the
/// dump SHOWS what the curation withheld, so a struct-array class may only elect
/// where this holds. Expressed THROUGH [`dump_row_renders`] rather than as its
/// own comparison, so the two can never drift: the members are exactly one
/// element hop below the array's row.
const fn dump_shows_struct_array_members(array_dump_depth: u8) -> bool {
    dump_row_renders(array_dump_depth + DUMP_ROWS_PER_ELEMENT_HOP)
}
/// How many elements of an array are previewed before an `…, N total` tail.
const DUMP_ARRAY_PREVIEW: usize = 8;
/// How many characters of a string field are shown before an ellipsis.
const DUMP_STRING_CHARS: usize = 120;
/// Hard ceiling on the lines one dump renders — a pathological message can be
/// arbitrarily wide, and the document is rebuilt on EVERY frame.
const DUMP_MAX_LINES: usize = 200;

/// Render a decoded frame as a READABLE structured document — the
/// fallback for a payload no archetype can draw.
///
/// The earlier dump was one flat line per TOP-LEVEL field, so everything
/// interesting collapsed to a placeholder: a nested message read
/// `std_msgs/Header { 2 fields }`, an array read `<9 × F64>`, a blob read
/// `<2560000 bytes>`. The document now:
///
/// - expands NESTED messages inline as an indented sub-list (to
///   `DUMP_MAX_DEPTH` hops, then `(nested, not expanded)`), so the actual
///   values are visible;
/// - previews numeric arrays (`float64[4]: [0.1, 0.2, 0.3, 0.4]`, first
///   `DUMP_ARRAY_PREVIEW` elements then `…, N total`);
/// - ENUMERATES an array of MESSAGE elements — fixed (`MotorState[20]`) or
///   dynamic (`PoseStamped[]`) alike — as `[i]` sub-dumps carrying the elements'
///   own fields, capped at `DUMP_ARRAY_PREVIEW` elements with a `… N total` tail
///   (a fixed struct array used to collapse to
///   `[Type {…}, Type {…}, …]`, which told a reader nothing about the one thing
///   the dump exists to show);
/// - names a byte blob's size AND its leading bytes in hex, so an operator can
///   recognise a JPEG/PNG magic;
/// - says EXPLICITLY that a dynamic array of nested messages is opaque and WHY
///   (no framework element framing), instead of an unexplained byte count.
///
/// Bounded by construction (`DUMP_MAX_LINES`, the array/string previews), so a
/// hostile or merely enormous message cannot make the per-frame document
/// unbounded. Pure + oracle-tested.
pub fn structured_dump(fv: &FrameValue) -> String {
    let mut out = String::new();
    out.push_str(&format!("**{}**\n\n", fv.schema_name));
    if fv.fields.is_empty() {
        out.push_str("_(no fields)_\n");
        return out;
    }
    let mut lines = 0usize;
    dump_fields(&mut out, &mut lines, fv, 0);
    if lines >= DUMP_MAX_LINES {
        out.push_str("- _(truncated — message has more fields than the dump renders)_\n");
    }
    out
}

/// Emit one message's fields as an indented bullet list. `depth` is the nesting
/// level (0 at the root); `lines` is the shared `DUMP_MAX_LINES` budget.
fn dump_fields(out: &mut String, lines: &mut usize, fv: &FrameValue, depth: u8) {
    let pad = "  ".repeat(depth as usize);
    for f in &fv.fields {
        if *lines >= DUMP_MAX_LINES {
            return;
        }
        *lines += 1;
        match &f.value {
            FrameValueKind::Nested(inner) => {
                if inner.fields.is_empty() {
                    out.push_str(&format!(
                        "{pad}- `{}` ({}) — _(empty)_\n",
                        f.name, inner.schema_name
                    ));
                } else if depth + 1 >= DUMP_MAX_DEPTH {
                    out.push_str(&format!(
                        "{pad}- `{}` ({}, {} fields) — _(nested, not expanded)_\n",
                        f.name,
                        inner.schema_name,
                        inner.fields.len()
                    ));
                } else {
                    out.push_str(&format!("{pad}- `{}` ({})\n", f.name, inner.schema_name));
                    dump_fields(out, lines, inner, depth + 1);
                }
            }
            // A FIXED array. Two shapes, deliberately rendered differently.
            //
            // Reported from a live robot: elements that are MESSAGES are
            // ENUMERATED, exactly like their dynamic `NestedArray` twin below. A
            // one-line preview renders a struct element through
            // `describe_scalar`, i.e. `Type {…}` — so a `MotorState[20]` read
            // `[unitree_go/MotorState {…}, unitree_go/MotorState {…}, …]` and the
            // dump, whose whole job is to make an undrawable payload INSPECTABLE,
            // showed nothing about the message it was dumping. The walker has the
            // fields decoded; hiding them behind a placeholder was the wrong
            // answer, and it hid exactly the messages a joint-bank robot cares
            // about. `dump_elements` bounds the expansion the same way the
            // dynamic arm is bounded (first `DUMP_ARRAY_PREVIEW` elements, then a
            // `… N total` line), so 20 motors are structure, not a 240-line wall.
            //
            // A fixed array of PRIMITIVES keeps its one-line preview: there is no
            // structure to expand and `[0.1, 0.2, 0.3]` on one line is strictly
            // more readable than three bullets.
            FrameValueKind::Array(items)
                if items.iter().any(
                    |el| matches!(el, FrameValueKind::Nested(inner) if !inner.fields.is_empty()),
                ) =>
            {
                out.push_str(&format!(
                    "{pad}- `{}` — {} element(s)\n",
                    f.name,
                    items.len()
                ));
                if depth + 1 < DUMP_MAX_DEPTH {
                    dump_elements(out, lines, items, depth + 1);
                }
            }
            FrameValueKind::Array(items) => {
                out.push_str(&format!(
                    "{pad}- `{}` — {} element(s): {}\n",
                    f.name,
                    items.len(),
                    preview_list(
                        items.iter().take(DUMP_ARRAY_PREVIEW).map(describe_scalar),
                        items.len()
                    )
                ));
            }
            // A DYNAMIC array of nested messages / strings the walker
            // decoded under the canonical element framing — ENUMERATE the
            // elements. A `Marker[]`, a `DiagnosticStatus[]`, a
            // `JointTrajectoryPoint[]` is only inspectable if the elements are
            // visible; a bare
            // `[N element(s)]` count through [`describe`] is not.
            FrameValueKind::NestedArray { elements, .. } => {
                out.push_str(&format!(
                    "{pad}- `{}` — {} element(s)\n",
                    f.name,
                    elements.len()
                ));
                if depth + 1 < DUMP_MAX_DEPTH {
                    dump_elements(out, lines, elements, depth + 1);
                }
            }
            other => out.push_str(&format!("{pad}- `{}` = {}\n", f.name, describe(other))),
        }
    }
}

/// Enumerate an array's elements as `[i]` bullets, sharing [`dump_fields`]'s
/// `DUMP_MAX_LINES` budget: a MESSAGE element expands as its own indented
/// sub-dump (within [`DUMP_MAX_DEPTH`]), a `string[]` element renders as one leaf
/// line.
///
/// Shared by BOTH array shapes — a
/// [`FrameValueKind::NestedArray`] (dynamic) and a
/// [`FrameValueKind::Array`] of message elements (fixed, e.g. a `MotorState[20]`
/// joint bank). They differ only in wire framing; to a reader they are the same
/// thing, and rendering one of them as `[Type {…}, Type {…}, …]` made the dump
/// useless on exactly the messages a joint-bank robot publishes.
///
/// At most [`DUMP_ARRAY_PREVIEW`] elements are shown — a per-element expansion is
/// far more verbose than the single-line preview a fixed
/// [`FrameValueKind::Array`] gets — with a trailing `… N total` line naming the
/// real count so a truncated dump never reads as a complete one.
fn dump_elements(out: &mut String, lines: &mut usize, elements: &[FrameValueKind], depth: u8) {
    let pad = "  ".repeat(depth as usize);
    for (i, el) in elements.iter().take(DUMP_ARRAY_PREVIEW).enumerate() {
        if *lines >= DUMP_MAX_LINES {
            return;
        }
        *lines += 1;
        match el {
            FrameValueKind::Nested(inner)
                if !inner.fields.is_empty() && depth + 1 < DUMP_MAX_DEPTH =>
            {
                out.push_str(&format!("{pad}- [{i}] ({})\n", inner.schema_name));
                dump_fields(out, lines, inner, depth + 1);
            }
            other => out.push_str(&format!("{pad}- [{i}] = {}\n", describe_scalar(other))),
        }
    }
    if elements.len() > DUMP_ARRAY_PREVIEW && *lines < DUMP_MAX_LINES {
        *lines += 1;
        out.push_str(&format!("{pad}- … {} total\n", elements.len()));
    }
}

/// `[a, b, c, …, N total]` for a previewed sequence — `shown` is what the
/// iterator yields, `total` the real element count.
fn preview_list(shown: impl Iterator<Item = String>, total: usize) -> String {
    let items: Vec<String> = shown.collect();
    let more = total.saturating_sub(items.len());
    if more == 0 {
        format!("[{}]", items.join(", "))
    } else {
        format!("[{}, …, {total} total]", items.join(", "))
    }
}

/// A leaf value's short rendering (used for array element previews).
fn describe_scalar(v: &FrameValueKind) -> String {
    match v {
        FrameValueKind::Nested(inner) => {
            format!("{} {{…}}", inner.schema_name)
        }
        other => describe(other),
    }
}

/// One-line human description of a value for the [`structured_dump`] fallback.
/// `Nested` / `Array` / `NestedArray` are expanded by [`dump_fields`]
/// (the last via [`dump_elements`]) instead of reaching here at the top level —
/// for them this is the array-element preview's fallback, which is why the
/// `NestedArray` arm below still renders a one-line element COUNT (a nested array
/// nested INSIDE an enumerated element, at the depth cap, is summarized rather
/// than expanded again).
fn describe(v: &FrameValueKind) -> String {
    match v {
        FrameValueKind::Bool(b) => b.to_string(),
        FrameValueKind::I8(x) => x.to_string(),
        FrameValueKind::U8(x) => x.to_string(),
        FrameValueKind::I16(x) => x.to_string(),
        FrameValueKind::U16(x) => x.to_string(),
        FrameValueKind::I32(x) => x.to_string(),
        FrameValueKind::U32(x) => x.to_string(),
        FrameValueKind::I64(x) => x.to_string(),
        FrameValueKind::U64(x) => x.to_string(),
        FrameValueKind::F32(x) => format!("{x:?}"),
        FrameValueKind::F64(x) => format!("{x:?}"),
        FrameValueKind::Str(s) => format!("{:?}", truncate_str(s)),
        FrameValueKind::Bytes(b) => describe_bytes(b),
        FrameValueKind::PrimArray(pa) => format!(
            "{}[{}]: {}",
            prim_type_name(pa.elem),
            pa.count,
            preview_list(
                pa.iter_f64()
                    .take(DUMP_ARRAY_PREVIEW)
                    .map(|v| format!("{v:?}")),
                pa.count
            )
        ),
        FrameValueKind::Nested(inner) => {
            format!("{} {{ {} fields }}", inner.schema_name, inner.fields.len())
        }
        FrameValueKind::Array(items) => format!("[{} items]", items.len()),
        // A dynamic array of nested messages / strings the walker
        // decoded element-by-element under the canonical element framing.
        FrameValueKind::NestedArray { elements, .. } => {
            format!("[{} element(s)]", elements.len())
        }
        // Bytes that do NOT satisfy the canonical element framing — say WHY,
        // so an operator does not go looking for a mapping that cannot exist.
        // (the framework DOES define an element framing now, so the
        // accurate reason is that THESE bytes are not written in it.)
        FrameValueKind::NestedArrayOpaque(b) => format!(
            "{} bytes — opaque (not canonically framed: a producer-defined element convention \
             the framework walker does not decode)",
            b.len()
        ),
    }
}

/// A byte blob's size plus its leading bytes in hex (so a JPEG / PNG magic is
/// recognisable at a glance).
fn describe_bytes(b: &[u8]) -> String {
    if b.is_empty() {
        return "0 bytes".to_string();
    }
    let head: Vec<String> = b
        .iter()
        .take(DUMP_ARRAY_PREVIEW)
        .map(|x| format!("{x:02x}"))
        .collect();
    let ellipsis = if b.len() > head.len() { " …" } else { "" };
    format!("{} bytes (hex: {}{ellipsis})", b.len(), head.join(" "))
}

/// The ROS-familiar element type name of a numeric array.
fn prim_type_name(t: PrimType) -> &'static str {
    match t {
        PrimType::I16 => "int16",
        PrimType::U16 => "uint16",
        PrimType::I32 => "int32",
        PrimType::U32 => "uint32",
        PrimType::I64 => "int64",
        PrimType::U64 => "uint64",
        PrimType::F32 => "float32",
        PrimType::F64 => "float64",
    }
}

/// A string field trimmed to `DUMP_STRING_CHARS` characters (CHARACTER
/// boundaries, so a multi-byte UTF-8 string is never split mid-codepoint).
fn truncate_str(s: &str) -> String {
    if s.chars().count() <= DUMP_STRING_CHARS {
        return s.to_string();
    }
    let mut out: String = s.chars().take(DUMP_STRING_CHARS).collect();
    out.push('…');
    out
}

// ============================================================================
// High-value archetype mappings + the generic shape-inference feeders.
//
// The extractors below are PURE (over a decoded `FrameValue`) and oracle-tested,
// so the "smart map for a schema never seen before" claim (shape inference in
// `crate::sink`) is testable without Rerun or a transport. The `log_*` builders
// are the thin Rerun half (best-effort: a log error warns, never propagates).
// ============================================================================

/// Translation + optional rotation extracted from a pose-shaped value — the
/// oracle-comparable intermediate the [`log_transform3d_in_frame`] builder consumes
/// (so pose extraction is tested against hand data, not a `rerun::Transform3D`
/// that has no easy equality).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PoseParts {
    /// XYZ translation, `f64 → f32`-narrowed (Rerun is f32).
    pub translation: [f32; 3],
    /// `[x, y, z, w]` rotation quaternion, when the value carried one (a bare
    /// position without an orientation → `None`).
    pub rotation: Option<[f32; 4]>,
}

/// A raw (un-encoded) image pixel layout Rerun can render natively.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RawEncoding {
    /// 8-bit RGB, interleaved `RGBRGB…`.
    Rgb8,
    /// 8-bit BGR (OpenCV / ROS default), interleaved `BGRBGR…`.
    Bgr8,
    /// 8-bit single-channel grayscale.
    Mono8,
}

/// Map a ROS `image.encoding` string to a natively-renderable [`RawEncoding`],
/// or `None` for an encoding not decoded here (the caller degrades to a field
/// dump — never mis-renders unknown pixels). Only the lossless 8-bit layouts
/// are covered; 16-bit/float/Bayer/YUV encodings intentionally degrade.
pub fn classify_raw_encoding(encoding: &str) -> Option<RawEncoding> {
    match encoding {
        "rgb8" => Some(RawEncoding::Rgb8),
        "bgr8" => Some(RawEncoding::Bgr8),
        // `8UC1` is the OpenCV alias many drivers use for a mono8 buffer.
        "mono8" | "8UC1" => Some(RawEncoding::Mono8),
        _ => None,
    }
}

// ---- pure FrameValue readers ---------------------------------------------

/// Any numeric scalar kind (NOT `bool` — a flag is not a plot) widened to
/// `f64`. `u64`/`i64` beyond 2^53 lose precision (acceptable for viz).
fn num_f64(k: &FrameValueKind) -> Option<f64> {
    Some(match k {
        FrameValueKind::I8(v) => *v as f64,
        FrameValueKind::U8(v) => *v as f64,
        FrameValueKind::I16(v) => *v as f64,
        FrameValueKind::U16(v) => *v as f64,
        FrameValueKind::I32(v) => *v as f64,
        FrameValueKind::U32(v) => *v as f64,
        FrameValueKind::I64(v) => *v as f64,
        FrameValueKind::U64(v) => *v as f64,
        FrameValueKind::F32(v) => *v as f64,
        FrameValueKind::F64(v) => *v,
        _ => return None,
    })
}

/// A named field read as any numeric scalar (`f64`).
fn field_num(fv: &FrameValue, name: &str) -> Option<f64> {
    fv.field(name).and_then(num_f64)
}

/// A named field read as an `f32` (viz precision).
fn field_f32(fv: &FrameValue, name: &str) -> Option<f32> {
    field_num(fv, name).map(|v| v as f32)
}

/// A named nested-message field.
fn nested_field<'a, 'f>(fv: &'a FrameValue<'f>, name: &str) -> Option<&'a FrameValue<'f>> {
    match fv.field(name) {
        Some(FrameValueKind::Nested(inner)) => Some(inner.as_ref()),
        _ => None,
    }
}

/// A named numeric-primitive-array field.
fn prim_array<'a, 'f>(fv: &'a FrameValue<'f>, name: &str) -> Option<&'a PrimArray<'f>> {
    match fv.field(name) {
        Some(FrameValueKind::PrimArray(pa)) => Some(pa),
        _ => None,
    }
}

/// Read `{x, y, z}` (numeric, any type) from a value as an `f32` triple.
fn xyz_of(fv: &FrameValue) -> Option<[f32; 3]> {
    Some([
        field_num(fv, "x")? as f32,
        field_num(fv, "y")? as f32,
        field_num(fv, "z")? as f32,
    ])
}

/// Read `{x, y, z, w}` (numeric) as an `[x, y, z, w]` quaternion.
fn xyzw_of(fv: &FrameValue) -> Option<[f32; 4]> {
    Some([
        field_num(fv, "x")? as f32,
        field_num(fv, "y")? as f32,
        field_num(fv, "z")? as f32,
        field_num(fv, "w")? as f32,
    ])
}

/// Push each present `{group}/{x,y,z}` axis of a nested `Vector3`-shaped field
/// as a named scalar sample (missing axes are skipped).
fn push_group_xyz(out: &mut Vec<(String, f64)>, fv: &FrameValue, group: &str) {
    if let Some(inner) = nested_field(fv, group) {
        for axis in ["x", "y", "z"] {
            if let Some(v) = field_num(inner, axis) {
                out.push((format!("{group}/{axis}"), v));
            }
        }
    }
}

/// Push the first `names.len()` elements of a numeric-primitive-array field as
/// named scalars (a shorter array pushes only what it has — defensive against
/// schema variation on the store-only Unitree types).
fn push_prim_indexed(out: &mut Vec<(String, f64)>, fv: &FrameValue, field: &str, names: &[&str]) {
    if let Some(pa) = prim_array(fv, field) {
        for (v, name) in pa.iter_f64().zip(names.iter().copied()) {
            out.push((name.to_string(), v));
        }
    }
}

// ---- named / inferred spatial extractors ---------------------------------

/// Extract the translation (+ optional rotation) from a pose- OR
/// transform-shaped value: top-level `{position, orientation}` (a bare
/// `geometry_msgs/Pose` or inferred pose-shape) or `{translation, rotation}`
/// (a `geometry_msgs/Transform` or inferred transform-shape — the same rigid
/// motion under the TF field names), else following a nested `pose`
/// (`PoseStamped.pose`, `Odometry.pose.pose`) or `transform`
/// (`TransformStamped.transform`) field up to two hops. `None` when no
/// translation is reachable.
pub fn pose_transform_parts(fv: &FrameValue) -> Option<PoseParts> {
    pose_parts_rec(fv, 2).map(|(parts, _)| parts)
}

/// [`pose_transform_parts`] plus the field PATHS it consumed (`a/b`-qualified,
/// relative to `fv`) — the input to the sibling harvest (see
/// [`infer_spatial_shape`]), so the numbers the pose itself is made of are never
/// re-plotted as scalars.
///
/// Paths, not bare root names: a `{pose: {position, orientation, confidence}}`
/// consumes `pose/position` + `pose/orientation`, leaving `pose/confidence`
/// harvestable. Excluding the whole `pose` wrapper would silently drop it — the
/// exact class of bug this function exists to prevent.
fn pose_parts_rec(fv: &FrameValue, depth: u8) -> Option<(PoseParts, Vec<String>)> {
    // Pose shape: `{position, orientation}`.
    if let Some(translation) = nested_field(fv, "position").and_then(xyz_of) {
        let rotation = nested_field(fv, "orientation").and_then(xyzw_of);
        let mut roots = vec!["position".to_string()];
        if rotation.is_some() {
            roots.push("orientation".to_string());
        }
        return Some((
            PoseParts {
                translation,
                rotation,
            },
            roots,
        ));
    }
    // Transform shape: `{translation, rotation}` — the identical rigid motion
    // under the TF field names (`geometry_msgs/Transform[Stamped]`).
    if let Some(translation) = nested_field(fv, "translation").and_then(xyz_of) {
        let rotation = nested_field(fv, "rotation").and_then(xyzw_of);
        let mut roots = vec!["translation".to_string()];
        if rotation.is_some() {
            roots.push("rotation".to_string());
        }
        return Some((
            PoseParts {
                translation,
                rotation,
            },
            roots,
        ));
    }
    // One hop down through a nested `pose` (PoseStamped, Odometry.pose.pose) or
    // `transform` (TransformStamped.transform), within the depth budget. The
    // inner paths are RE-QUALIFIED under the wrapper, so only what the pose
    // actually read is consumed — a telemetry sibling beside it survives.
    if depth > 0 {
        for wrapper in ["pose", "transform"] {
            if let Some(inner) = nested_field(fv, wrapper) {
                return pose_parts_rec(inner, depth - 1)
                    .map(|(parts, inner_paths)| (parts, qualify(wrapper, inner_paths)));
            }
        }
    }
    None
}

/// Re-qualify a nested value's consumed PATHS under the wrapper field that
/// reached it (`position` under `pose` → `pose/position`), matching the
/// `a/b`-qualified paths [`harvest_series`] builds.
fn qualify(wrapper: &str, paths: Vec<String>) -> Vec<String> {
    paths
        .into_iter()
        .map(|p| format!("{wrapper}/{p}"))
        .collect()
}

/// The `orientation` quaternion of an IMU-shaped value (`[x, y, z, w]`).
pub fn imu_rotation(fv: &FrameValue) -> Option<[f32; 4]> {
    nested_field(fv, "orientation").and_then(xyzw_of)
}

/// The plottable IMU scalars: `linear_acceleration/{x,y,z}` +
/// `angular_velocity/{x,y,z}` (six time series). Covariances are omitted.
pub fn imu_scalars(fv: &FrameValue) -> Vec<(String, f64)> {
    let mut out = Vec::new();
    push_group_xyz(&mut out, fv, "linear_acceleration");
    push_group_xyz(&mut out, fv, "angular_velocity");
    out
}

/// The `twist.twist.{linear,angular}/{x,y,z}` velocity scalars of a
/// `nav_msgs/Odometry` (six series; the pose half rides
/// [`pose_transform_parts`]).
pub fn odometry_twist_scalars(fv: &FrameValue) -> Vec<(String, f64)> {
    let Some(twist_cov) = nested_field(fv, "twist") else {
        return Vec::new();
    };
    let Some(twist) = nested_field(twist_cov, "twist") else {
        return Vec::new();
    };
    let mut out = Vec::new();
    push_group_xyz(&mut out, twist, "linear");
    push_group_xyz(&mut out, twist, "angular");
    out
}

/// A single 3D point from a value: a nested `point` or `position` field
/// (`PointStamped`, or any `{header, position}` shape), else the value's own
/// top-level `{x, y, z}` (defensive fallback for a point-shaped variant that
/// carries its coordinates at top level).
pub fn single_point_of(fv: &FrameValue) -> Option<[f32; 3]> {
    if let Some(p) = nested_point_of(fv) {
        return Some(p);
    }
    xyz_of(fv)
}

/// A 3D point reachable through a NESTED `point` / `position` field — the
/// INFERENCE half of [`single_point_of`].
///
/// Deliberately never reads the value's OWN top-level `{x, y, z}`. This is by
/// design: a bare xyz message is a velocity / force / RPY and plots as
/// three scalars (a velocity drawn as a point is a misleading dot at the origin).
/// A field NAMED `position` (or `point`) is unambiguous, so a `{header, position}`
/// message renders as a real point instead of a field dump.
pub fn nested_point_of(fv: &FrameValue) -> Option<[f32; 3]> {
    nested_point_with_root(fv).map(|(p, _)| p)
}

/// [`nested_point_of`] plus the field PATH it consumed (`point` or `position`) —
/// the sibling-harvest exclusion.
///
/// A nested `point` SHADOWS a nested `position`: the FIELD decides, then its
/// shape is read. A value that has a `point` commits to it, so a `point` that is
/// not xyz-shaped yields nothing rather than silently rendering a sibling
/// `position` as if it were the point. (The path-carrying refactor
/// widened this into a try-each-in-turn loop — a behaviour change inside a
/// change advertised as extraction-only.)
fn nested_point_with_root(fv: &FrameValue) -> Option<([f32; 3], String)> {
    let (root, inner) = match nested_field(fv, "point") {
        Some(inner) => ("point", inner),
        None => ("position", nested_field(fv, "position")?),
    };
    Some((xyz_of(inner)?, root.to_string()))
}

/// The rotation of an ORIENTATION-ONLY value: a nested `orientation` /
/// `rotation` / `quaternion` field read as an `[x, y, z, w]` quaternion, following
/// a nested `pose` / `transform` wrapper up to two hops
/// (`geometry_msgs/QuaternionStamped`, `sensor_msgs/Imu`-shaped attitude, or any
/// `{header, orientation}` custom message).
///
/// The caller uses this only AFTER [`pose_transform_parts`] found no translation,
/// so a full pose never degrades to a rotation. A value's own top-level
/// `{x, y, z, w}` is deliberately NOT matched — that shape stays a four-series
/// scalar bag by design; only a field whose NAME declares an orientation
/// is treated as one.
pub fn rotation_only_of(fv: &FrameValue) -> Option<[f32; 4]> {
    rotation_only_rec(fv, 2).map(|(q, _)| q)
}

/// [`rotation_only_of`] plus the field PATH it consumed (`a/b`-qualified,
/// relative to `fv`) — the sibling-harvest exclusion (the quaternion's
/// own components ARE the rotation, never four extra plot series). A wrapper hop
/// re-qualifies the inner path (`pose/orientation`), so a sibling INSIDE the
/// wrapper still plots.
fn rotation_only_rec(fv: &FrameValue, depth: u8) -> Option<([f32; 4], String)> {
    for name in ["orientation", "rotation", "quaternion"] {
        if let Some(q) = nested_field(fv, name).and_then(xyzw_of) {
            return Some((q, name.to_string()));
        }
    }
    if depth > 0 {
        for wrapper in ["pose", "transform"] {
            if let Some(inner) = nested_field(fv, wrapper) {
                return rotation_only_rec(inner, depth - 1)
                    .map(|(q, inner_path)| (q, format!("{wrapper}/{inner_path}")));
            }
        }
    }
    None
}

/// A PLANAR pose: `{x, y, theta}` at top level (`geometry_msgs/Pose2D`)
/// or `{position: {x, y}, theta}` (`vision_msgs/Pose2D`), lifted to a full rigid
/// transform on the z=0 plane — translation `[x, y, 0]` and a yaw-only
/// quaternion. Every ground robot's 2D pose then MOVES its frame in the 3D scene
/// instead of plotting as three disconnected numbers.
///
/// `theta` is the discriminator (radians, ROS convention), so a bare `{x, y, z}`
/// never matches.
pub fn planar_pose_of(fv: &FrameValue) -> Option<PoseParts> {
    planar_pose_with_roots(fv).map(|(parts, _)| parts)
}

/// [`planar_pose_of`] plus the field PATHS it consumed (`{x, y, theta}` or
/// `{position/x, position/y, theta}`) — the sibling-harvest exclusion.
/// The nested arm consumes the two COMPONENTS it read, not the whole `position`
/// wrapper, so a `{position: {x, y, z}, theta}` still plots `position/z`.
fn planar_pose_with_roots(fv: &FrameValue) -> Option<(PoseParts, Vec<String>)> {
    let theta = field_num(fv, "theta")?;
    let (x, y, mut roots) = match (field_num(fv, "x"), field_num(fv, "y")) {
        (Some(x), Some(y)) => (x, y, vec!["x".to_string(), "y".to_string()]),
        _ => {
            let p = nested_field(fv, "position")?;
            (
                field_num(p, "x")?,
                field_num(p, "y")?,
                vec!["position/x".to_string(), "position/y".to_string()],
            )
        }
    };
    roots.push("theta".to_string());
    Some((
        PoseParts {
            translation: [x as f32, y as f32, 0.0],
            rotation: Some(yaw_quaternion(theta)),
        },
        roots,
    ))
}

/// Which spatial primitive an inferred SHAPE yields.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpatialKind {
    /// A rigid transform ([`log_pose`] renders it).
    Transform,
    /// A single 3D point ([`log_single_point`] renders it).
    Point,
}

/// The spatial primitive a value RENDERS as, carrying the geometry itself — the
/// ONE decision [`infer_spatial_primitive`] makes, consumed by BOTH the
/// classifier (via [`infer_spatial_shape`], which keeps only the
/// [`SpatialKind`]) and the render arms (via [`log_pose`], which draws exactly
/// this).
///
/// Keeping the geometry and the consumed-path set in ONE return value is the
/// fix for the two ladders drifting: the classifier used to
/// re-derive the precedence independently of `log_pose`, so a
/// `{position: {x, y, z}, theta}` was CLASSIFIED planar (consuming `theta`) but
/// RENDERED as a full 3D pose (drawing `position/z`, never drawing `theta`) —
/// `theta` was excluded from the plots yet drawn nowhere (silently lost) and
/// `position/z` was both drawn and plotted.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SpatialPrimitive {
    /// A rigid transform (translation + optional rotation) — [`log_transform3d_in_frame`].
    Transform(PoseParts),
    /// An orientation-only `[x, y, z, w]` rotation — [`log_rotation3d_in_frame`].
    Rotation([f32; 4]),
    /// A single 3D point — [`log_point3d`].
    Point([f32; 3]),
}

impl SpatialPrimitive {
    /// Which archetype family this primitive belongs to (a rotation is still a
    /// transform — it moves the entity's frame).
    pub fn kind(&self) -> SpatialKind {
        match self {
            Self::Transform(_) | Self::Rotation(_) => SpatialKind::Transform,
            Self::Point(_) => SpatialKind::Point,
        }
    }
}

/// A value's inferred spatial primitive PLUS the field paths that primitive
/// consumed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SpatialShape {
    /// Which primitive the shape yields.
    pub kind: SpatialKind,
    /// The `a/b`-qualified field PATHS the primitive read (the same path shape
    /// [`harvest_series`] builds). Everything else in the message is SIBLING
    /// telemetry — see [`spatial_sibling_series`].
    pub consumed: Vec<String>,
}

/// **THE** spatial decision — the single ladder BOTH the classifier
/// ([`crate::sink::infer_archetype_from_shape`], via [`infer_spatial_shape`]) and
/// the render arms ([`log_pose`]) consume, so a value that CLASSIFIES as a
/// transform / point RENDERS as exactly that primitive AND agrees on which fields
/// the primitive already used.
///
/// Returns the primitive to draw plus the `a/b`-qualified field PATHS it read.
/// The two halves MUST come from one decision: the harvest excludes exactly the
/// returned paths, so a path that is excluded but not drawn is silently lost
/// telemetry, and a field that is drawn but not excluded is plotted twice. Both
/// were live bugs while the classifier and `log_pose` walked separate ladders.
///
/// Precedence (most specific first):
///
/// 1. a full pose / transform WITH a rotation → [`SpatialPrimitive::Transform`]
/// 2. a planar `{x, y, theta}` pose (yaw-lifted, z = 0) →
///    [`SpatialPrimitive::Transform`]
/// 3. an orientation-only nested quaternion → [`SpatialPrimitive::Rotation`]
/// 4. a nested `point` / `position` with no orientation → [`SpatialPrimitive::Point`]
/// 5. a translation with no rotation → [`SpatialPrimitive::Transform`]
///
/// `None` when no spatial primitive is reachable (the value falls to the numeric
/// harvest / text / dump rules).
///
/// Rule 2 sits ABOVE rule 5 deliberately: a value carrying BOTH a 3D translation
/// and a `theta` (`{position: {x, y, z}, theta}`) is drawn as the yaw-lifted
/// planar pose, and its `position/z` — which the lift does NOT draw — plots as a
/// sibling series. Nothing is lost either way; the ordering is what the two
/// consumers agree on.
pub fn infer_spatial_primitive(fv: &FrameValue) -> Option<(SpatialPrimitive, Vec<String>)> {
    let full_pose = pose_parts_rec(fv, 2);
    if let Some((parts, consumed)) = &full_pose {
        if parts.rotation.is_some() {
            return Some((SpatialPrimitive::Transform(*parts), consumed.clone()));
        }
    }
    if let Some((parts, consumed)) = planar_pose_with_roots(fv) {
        return Some((SpatialPrimitive::Transform(parts), consumed));
    }
    if let Some((q, root)) = rotation_only_rec(fv, 2) {
        return Some((SpatialPrimitive::Rotation(q), vec![root]));
    }
    if let Some((p, root)) = nested_point_with_root(fv) {
        return Some((SpatialPrimitive::Point(p), vec![root]));
    }
    full_pose.map(|(parts, consumed)| (SpatialPrimitive::Transform(parts), consumed))
}

/// The CLASSIFIER's view of [`infer_spatial_primitive`]: which archetype family
/// the value belongs to, plus the paths the primitive consumed (the geometry
/// itself is the render arms' business).
pub fn infer_spatial_shape(fv: &FrameValue) -> Option<SpatialShape> {
    infer_spatial_primitive(fv).map(|(primitive, consumed)| SpatialShape {
        kind: primitive.kind(),
        consumed,
    })
}

/// The plottable numbers a SPATIALLY-classified frame carries BESIDE its spatial
/// primitive — so a spatial classification never drops telemetry silently.
///
/// A message like `mavros_msgs/PositionTarget` or a custom
/// `acme/DroneState { Point position; Vector3 velocity; float64 battery_v }`
/// classifies to a point, but its remaining numbers are real telemetry: dropping
/// them rendered ONE dot and no plot. This returns exactly those numbers (the
/// full [`harvest_series`] MINUS the paths [`infer_spatial_shape`] reports the
/// primitive consumed, so the pose's own `position/x` / quaternion components are
/// never re-plotted), plus the fields the harvest deliberately skipped.
///
/// Empty samples ⇒ the value is PURELY spatial (a bare `geometry_msgs/Pose`), which
/// is exactly the classifier's pure-[`crate::sink::ArchetypeKind::Transform3D`] /
/// pure-[`crate::sink::ArchetypeKind::Point3D`] predicate; non-empty elects the
/// both-views [`crate::sink::ArchetypeKind::Transform3DWithScalars`] /
/// [`crate::sink::ArchetypeKind::Point3DWithScalars`] instead.
pub fn spatial_sibling_series(fv: &FrameValue) -> (Vec<(String, f64)>, Vec<SkippedSeries>) {
    match infer_spatial_shape(fv) {
        Some(shape) => harvest_series_excluding(fv, &shape.consumed),
        None => (Vec::new(), Vec::new()),
    }
}

/// The `[x, y, z, w]` quaternion for a yaw-only rotation of `theta` radians
/// about +Z (the planar-pose lift).
fn yaw_quaternion(theta: f64) -> [f32; 4] {
    let half = theta * 0.5;
    [0.0, 0.0, half.sin() as f32, half.cos() as f32]
}

/// An oriented 3D BOX extracted from a detection-shaped value — the
/// oracle-comparable intermediate [`log_boxes3d`] consumes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoxParts {
    /// Box centre in the parent frame.
    pub center: [f32; 3],
    /// FULL box size (edge lengths) along the box's own axes.
    pub size: [f32; 3],
    /// `[x, y, z, w]` box orientation, when the centre carried one.
    pub rotation: Option<[f32; 4]>,
}

/// A 3D bounding box from a box-shaped value: a pose-shaped `center`
/// (or `pose`) field plus an xyz-shaped `size` (or `extents` / `dimensions`)
/// field — `vision_msgs/BoundingBox3D`, `moveit_msgs/OrientedBoundingBox`, and
/// any custom perception detection built the same way.
///
/// The centre goes through [`pose_transform_parts`], so a `{position, orientation}`
/// and a `{translation, rotation}` centre both work, and the box picks up the
/// orientation when one is present. Checked BEFORE the plain pose rule (a
/// `{pose, extents}` value would otherwise degrade to a size-less transform).
pub fn box3d_parts(fv: &FrameValue) -> Option<BoxParts> {
    let size = ["size", "extents", "dimensions"]
        .iter()
        .find_map(|n| nested_field(fv, n))
        .and_then(xyz_of)?;
    let center = ["center", "pose"]
        .iter()
        .find_map(|n| nested_field(fv, n))
        .and_then(pose_transform_parts)?;
    Some(BoxParts {
        center: center.translation,
        size,
        rotation: center.rotation,
    })
}

// ============================================================================
// The viz ladder over DECODED ELEMENT ARRAYS.
//
// `FrameWalker` decodes the canonical element framing into
// `FrameValueKind::NestedArray { elements, raw }`. These PURE extractors turn
// those elements into geometry, so a nav2 robot's `/plan` DRAWS instead of
// dumping text — classified by element SHAPE (an unseen vendor type with the same
// shape draws too), never by the array field's name.
//
// The ONE ladder both the classifier (`crate::sink::infer_archetype_from_shape`)
// and the render arms (`log_element_array_from_frame`) consume, so a value that
// CLASSIFIES as a path / pose array / detection set RENDERS as exactly that —
// the lesson (two independent ladders drift).
// ============================================================================

/// The most elements one array field renders as geometry. Elements BEYOND this
/// are not rendered and the truncation is REPORTED (see
/// [`ElementArrayParts::truncated`]), never silently dropped.
///
/// # Why 300 000
///
/// A cap of 10 000 would be a guess with **no
/// performance measurement**. `tests/element_cap_bench.rs` is the measurement
/// — real wire frames, the real [`FrameWalker`](cerulion_core::codegen::FrameWalker),
/// the real `dispatch_frame`, floor/p50/p99 across 1 000 → 500 000 elements
/// (M-series Mac, release). Two facts decided this number:
///
/// **1. This cap does not govern the expensive half.** The walker decodes EVERY
/// element into a `FrameValue` before the cap is consulted, at ~1.2 µs/element
/// (`nav_msgs/Path`) — 3-4× the cap-governed cost. What the cap governs (the
/// geometry scan + Rerun archetype construction + log) measured a LINEAR
/// 0.32 µs/element for polylines and 0.27 µs/element for boxes, flat across the
/// whole sweep. So raising the cap adds only that marginal term; it buys back
/// geometry whose decode was already paid unconditionally.
///
/// (A later re-measurement of the sweep reports the same marginal term as the
/// DIRECT `scan` floor — 0.29 µs/element for polylines, 0.22 for boxes — rather
/// than as the `render − walk` difference these two figures came from, whose two
/// noisy terms make the small rows unreadable. Taken as `render − walk` on that
/// run's clean large rows it reads 0.29 / 0.29. The derivations agree to within
/// run-to-run noise and the cap arithmetic below is unchanged;
/// `tests/element_cap_bench.rs` carries both, with the reason.)
///
/// **2. 300 000 is where that marginal term reaches a 10 Hz frame budget**:
/// 100 000 µs ÷ 0.32 µs/element = 312 500. Rounded down, and it clears the
/// densest common 3D lidar frame — an Ouster OS1-128 at 2048×10 Hz is
/// 128 × 2048 = 262 144 points (published datasheet) — so if a later change
/// unifies the point-cloud path onto this one, a full sweep is NOT clipped.
/// (VLP-16 @5 Hz = 57 600, OS1-64 @1024 = 65 536, HDL-64E @10 Hz ≈ 130 000, the
/// Go2's L1 ≈ 2 000/revolution — all far below.)
///
/// # What this cap is NOT
///
/// It is **not** the frame-budget guard: the total per-frame cost crosses a
/// 30 Hz budget at ~17-22 k elements and a 10 Hz budget at ~50-66 k, driven by
/// the uncapped walker decode. (A re-measurement corrected the 30 Hz figure, which read
/// "~15-21 k" — the harness's own render floors put the crossing at ~22 k for a
/// polyline and ~17 k for boxes, and the bench module doc always said so.) And
/// it is **not** the hostile-input defense —
/// that is the walker's own byte budget (`count > remaining / 4` ⇒ opaque,
/// `frame_walker.rs`), which refuses a frame DECLARING a billion elements
/// without ever sizing a `Vec`, long before this ceiling is consulted. This is a
/// RENDER ceiling: it bounds the geometry buffer and the Rerun instance count.
/// It still bites — the 128 MiB default `max_slice_len` admits ~1.5 M
/// `PoseStamped` elements on the wire, so 300 000 is a real refusal, not an
/// unreachable-by-construction number.
///
/// # This constant does NOT bind a `MarkerArray`
///
/// A `MarkerArray` rides its OWN pair —
/// [`crate::marker::MAX_MARKER_INSTANCES`] (10 000 markers inspected per frame),
/// [`crate::marker::MAX_LIVE_MARKERS`] (10 000 tracked live per input) and
/// [`crate::marker::MAX_MARKER_VERTICES`] (200 000 vertices summed across them)
/// — and this constant is not consulted on that path at all.
///
/// The reason is the measurement above: it is per-INSTANCE cost, for N elements
/// becoming ONE archetype with N instances at ONE entity. A marker is its own
/// ENTITY carrying its own `Transform3D` plus its own geometry archetype, so N
/// markers are N entities and ~2N rerun log calls — a per-ENTITY axis this
/// number was never measured on, and 300 000 markers would be 600 000 log calls
/// in one frame. (Reusing this constant elsewhere would be wrong — it was raised
/// from 10 000, which is exactly what breaks such a reuse.)
/// The plain element path, where one element contributes exactly one instance,
/// is bound only by this constant.
///
/// Deliberately independent of [`MAX_ARRAY_SERIES`] (64), which governs PLOT
/// SERIES: 64 vertices would truncate an ordinary global plan, while 64 plot
/// series is already a humanoid's whole joint bank.
pub const MAX_ELEMENT_INSTANCES: usize = 300_000;

/// How an element array of `len` elements splits into `(drawn, reported)` at the
/// [`MAX_ELEMENT_INSTANCES`] ceiling.
///
/// ONE place, because the two halves of the ceiling MUST agree: the extractor
/// (`geometry_of_elements`) slices the head it will render, and
/// [`scan_element_arrays`] computes the remainder it will REPORT. Those were two
/// independently-written expressions (`len.min(CAP)` and
/// `len.saturating_sub(CAP)`); a drift between them would under- or over-report
/// the drop — the exact silent-truncation failure the report exists to prevent.
///
/// Pure `const` arithmetic, so the ceiling's contract is oracle-testable at ANY
/// magnitude (`usize::MAX` included) without materializing a cap-sized array.
pub const fn element_render_split(len: usize) -> (usize, usize) {
    let drawn = if len < MAX_ELEMENT_INSTANCES {
        len
    } else {
        MAX_ELEMENT_INSTANCES
    };
    (drawn, len - drawn)
}

/// The geometry a DECODED element array renders as — the
/// oracle-comparable intermediate the element render arms consume, so element
/// extraction is tested against hand data rather than a Rerun archetype that has
/// no easy equality.
#[derive(Debug, Clone, PartialEq)]
pub enum ElementGeometry {
    /// ORDERED vertices — a trajectory or a polygon ring, drawn as a polyline
    /// plus its vertices ([`rerun::LineStrips3D`] + [`rerun::Points3D`]).
    Path(Vec<[f32; 3]>),
    /// UNORDERED poses / points — a bag of samples with no meaningful
    /// element-to-element connection, drawn as [`rerun::Points3D`]. Connecting
    /// them with a line would invent structure the message does not carry.
    Points(Vec<[f32; 3]>),
    /// N oriented boxes — a detection SET, drawn as one N-instance
    /// [`rerun::Boxes3D`].
    Boxes(Vec<BoxParts>),
}

impl ElementGeometry {
    /// Promote an UNORDERED point bag to an ORDERED [`ElementGeometry::Path`]
    /// (the classify-vs-render fix). Used when the CLASSIFIED archetype
    /// says the array is a semantic sequence even though its elements are
    /// unstamped — a name-mapped `geometry_msgs/Polygon` classifies as `Path3D`
    /// but its `Point32` elements carry no per-element stamp, so the shape ladder
    /// [`scan_element_arrays`] returns `Points`. `Boxes` is never a path and an
    /// already-`Path` scan is returned unchanged, so this only ever turns `Points`
    /// into `Path` (the same vertices, now drawn as a polyline).
    fn into_ordered(self) -> Self {
        match self {
            ElementGeometry::Points(vertices) => ElementGeometry::Path(vertices),
            other => other,
        }
    }
}

/// A decoded element array's renderable geometry plus where it came from.
#[derive(Debug, Clone, PartialEq)]
pub struct ElementArrayParts {
    /// The `a/b`-qualified field path that carried the array (`poses`,
    /// `detections`, `polygon/points`).
    pub field: String,
    /// The per-element geometry.
    pub geometry: ElementGeometry,
    /// How many elements were NOT rendered because the array exceeded
    /// [`MAX_ELEMENT_INSTANCES`] — 0 for a fully-rendered array. Reported by the
    /// sink once per input rather than dropped silently.
    pub truncated: usize,
}

/// What [`scan_element_arrays`] found — THREE outcomes, because "an idle `/plan`
/// carrying zero poses" and "a producer whose element framing cannot be decoded"
/// are different facts that must not share a code path.
#[derive(Debug, Clone, PartialEq)]
pub enum ElementArrayScan {
    /// Renderable geometry.
    Geometry(ElementArrayParts),
    /// A decoded element array is present but carries ZERO elements this frame
    /// (an idle `/plan`, an empty detection set). Nothing to draw and NOT a
    /// failure: the render arms draw nothing and do NOT degrade to a field dump,
    /// so a topic does not flip between a polyline and a `TextDocument` as its
    /// array empties and refills.
    ///
    /// Deliberately NOT classifiable: with no elements there is no shape, so
    /// [`crate::sink::infer_archetype_from_shape`] cannot tell a path from a point
    /// bag from a detection set. The well-known ROS array types are therefore
    /// NAME-mapped in [`crate::sink::classify_schema`] — that is what keeps a
    /// nav2 `/plan` whose FIRST frame is empty from being frozen into the
    /// view-less archetype (the empty-panel failure mode).
    Empty {
        /// The `a/b`-qualified field path of the empty array.
        field: String,
    },
    /// No renderable element array: the value has no `NestedArray` field at all,
    /// its arrays are `NestedArrayOpaque` (a bespoke or rmw-framed blob the walker
    /// refused to decode — see [`opaque_element_arrays`]), or the elements carry
    /// no extractable geometry (a `JointTrajectoryPoint[]`, a
    /// `DiagnosticStatus[]`).
    Absent,
}

impl ElementArrayScan {
    /// The geometry, when this scan found any.
    pub fn parts(&self) -> Option<&ElementArrayParts> {
        match self {
            Self::Geometry(p) => Some(p),
            Self::Empty { .. } | Self::Absent => None,
        }
    }
}

/// **THE** element-array decision — the single ladder BOTH
/// [`crate::sink::infer_archetype_from_shape`] and the render arms consume.
///
/// Scans for a decoded [`FrameValueKind::NestedArray`] in declaration order —
/// top-level fields first, then ONE hop through each non-timestamp nested field
/// (so `geometry_msgs/PolygonStamped.polygon.points`, every nav2 robot's
/// published footprint, is reached) — and returns the FIRST array whose elements
/// all yield geometry. Precedence within an array, most specific first:
///
/// 1. every element yields [`box3d_parts`] (directly, or one hop through a
///    `bbox` / `box` sub-message — `vision_msgs/Detection3D`) ⇒
///    [`ElementGeometry::Boxes`]. Checked FIRST for the same reason
///    [`crate::sink::infer_archetype_from_shape`] checks boxes before poses: a
///    box's centre alone would degrade a detection set to a size-less point bag.
/// 2. every element yields a translation ([`pose_transform_parts`], else a bare
///    `{x, y, z}` / nested `point` via [`single_point_of`]) ⇒
///    [`ElementGeometry::Path`] when the elements are STAMPED (each carries a
///    `header` / `stamp` — a trajectory has a per-sample time, so its order is
///    semantic) else [`ElementGeometry::Points`].
///
/// **All-or-nothing.** Every inspected element must yield the shape or the array
/// does not match (the scan moves to the next candidate field). A half-rendered
/// array is exactly the fabricated value the walker itself refuses — see its
/// `NestedArray` docs.
///
/// Pure + oracle-tested, so the "a never-seen robot's `Path`-shaped topic draws"
/// claim is testable without Rerun or a transport.
pub fn scan_element_arrays(fv: &FrameValue) -> ElementArrayScan {
    let candidates = element_array_candidates(fv);
    // Pass 1: the first array that actually yields geometry.
    for (path, elements) in &candidates {
        if elements.is_empty() {
            continue;
        }
        if let Some(geometry) = geometry_of_elements(elements) {
            let (_, truncated) = element_render_split(elements.len());
            return ElementArrayScan::Geometry(ElementArrayParts {
                field: path.clone(),
                geometry,
                truncated,
            });
        }
    }
    // Pass 2: an array that is present but EMPTY this frame. Second, so a
    // message carrying both an empty and a populated array still draws.
    for (path, elements) in &candidates {
        if elements.is_empty() {
            return ElementArrayScan::Empty {
                field: path.clone(),
            };
        }
    }
    ElementArrayScan::Absent
}

/// [`scan_element_arrays`] with the CLASSIFIED archetype honored (the
/// classify-vs-render fix). The shape ladder decides Path-vs-Points purely from
/// element STAMPING (`element_is_stamped`), but a name-mapped
/// `geometry_msgs/Polygon` classifies as `Path3D` (a ring's order is semantic —
/// see [`crate::sink::classify_schema`]) while its `Point32` elements are
/// unstamped, so the shape half returns [`ElementGeometry::Points`] and the
/// render would disagree with both the classification and the `LineStrips3D`
/// blueprint components advertised for `Path3D`.
///
/// When the caller's classified kind forces ordered elements
/// (`force_ordered` — [`crate::sink::ArchetypeKind::forces_ordered_elements`],
/// true ONLY for `Path3D`), an unstamped `Points` scan is promoted to
/// [`ElementGeometry::Path`]. A genuinely-inferred vendor array (kind read from
/// SHAPE, not a name map) never sets `force_ordered`, so an unmapped unstamped
/// array still renders as points — the promotion is scoped to the classified
/// kind. `Boxes` and an already-ordered `Path` are unchanged, and
/// `Empty`/`Absent` scans pass through untouched. Pure + oracle-tested, so both
/// render entry points draw from ONE ladder (the two-ladders lesson).
pub fn scan_element_arrays_for_kind(fv: &FrameValue, force_ordered: bool) -> ElementArrayScan {
    let scan = scan_element_arrays(fv);
    if !force_ordered {
        return scan;
    }
    match scan {
        ElementArrayScan::Geometry(mut parts) => {
            parts.geometry = parts.geometry.into_ordered();
            ElementArrayScan::Geometry(parts)
        }
        other => other,
    }
}

/// Every decoded element array reachable from `fv`, as
/// `(a/b`-qualified field path, elements)`: top-level fields in DECLARATION order
/// first, then ONE hop through each non-timestamp nested field. One hop is enough
/// for the real ROS shapes (`PolygonStamped.polygon.points`) and keeps the scan
/// bounded and predictable.
fn element_array_candidates<'a, 'f>(
    fv: &'a FrameValue<'f>,
) -> Vec<(String, &'a [FrameValueKind<'f>])> {
    let mut out = Vec::new();
    for nv in &fv.fields {
        if let FrameValueKind::NestedArray { elements, .. } = &nv.value {
            out.push((nv.name.clone(), elements.as_slice()));
        }
    }
    for nv in &fv.fields {
        if let FrameValueKind::Nested(inner) = &nv.value {
            if is_time_like_nested(&nv.name, inner) {
                continue;
            }
            for sub in &inner.fields {
                if let FrameValueKind::NestedArray { elements, .. } = &sub.value {
                    out.push((format!("{}/{}", nv.name, sub.name), elements.as_slice()));
                }
            }
        }
    }
    out
}

/// The geometry a NON-EMPTY element list yields, or `None` when any inspected
/// element does not carry the shape (all-or-nothing — see
/// [`scan_element_arrays`]). Only the first [`MAX_ELEMENT_INSTANCES`] elements are
/// inspected, since only those are rendered.
fn geometry_of_elements(elements: &[FrameValueKind]) -> Option<ElementGeometry> {
    let (drawn, _) = element_render_split(elements.len());
    let head = &elements[..drawn];
    // A `string[]`'s elements are `Str` / `Bytes`, so this yields an empty
    // message list and every rung below fails — a `string[]` is never geometry.
    let messages: Vec<&FrameValue> = head.iter().filter_map(element_message).collect();
    if messages.len() != head.len() {
        return None;
    }
    if let Some(boxes) = messages
        .iter()
        .map(|el| element_box_parts(el))
        .collect::<Option<Vec<_>>>()
    {
        return Some(ElementGeometry::Boxes(boxes));
    }
    let points = messages
        .iter()
        .map(|el| element_translation(el))
        .collect::<Option<Vec<_>>>()?;
    if messages.iter().all(|el| element_is_stamped(el)) {
        Some(ElementGeometry::Path(points))
    } else {
        Some(ElementGeometry::Points(points))
    }
}

/// A message element's decoded value (`None` for a `string[]` element, which is
/// [`FrameValueKind::Str`] / [`FrameValueKind::Bytes`]).
fn element_message<'a, 'f>(k: &'a FrameValueKind<'f>) -> Option<&'a FrameValue<'f>> {
    match k {
        FrameValueKind::Nested(inner) => Some(inner.as_ref()),
        _ => None,
    }
}

/// One element's oriented box: the element itself is box-shaped
/// (`vision_msgs/BoundingBox3D[]`), or it WRAPS one in a `bbox` / `box`
/// sub-message (`vision_msgs/Detection3D`, whose own fields are `header`,
/// `results`, `bbox`, `id`).
fn element_box_parts(el: &FrameValue) -> Option<BoxParts> {
    box3d_parts(el).or_else(|| {
        ["bbox", "box"]
            .iter()
            .find_map(|n| nested_field(el, n))
            .and_then(box3d_parts)
    })
}

/// One element's 3D position: the pose / transform ladder's translation
/// (`PoseStamped`, `Pose`, `TransformStamped`) else a bare `{x, y, z}` or nested
/// `point` / `position` (`Point32`, `Point`) via [`single_point_of`].
///
/// [`single_point_of`] reads a BARE top-level `{x, y, z}`, which the scalar
/// spatial ladder deliberately does not (an unmapped xyz MESSAGE is usually a
/// velocity, and drawing a velocity as a dot is misleading). Inside an ARRAY the
/// ambiguity is gone: a `Point32[]` / `Point[]` is a vertex list by construction —
/// there is no such thing as an array of velocities drawn at the origin.
fn element_translation(el: &FrameValue) -> Option<[f32; 3]> {
    pose_transform_parts(el)
        .map(|p| p.translation)
        .or_else(|| single_point_of(el))
}

/// True when an element carries its own timestamp — a `header` or `stamp` field
/// that is time-shaped (the same [`is_time_like_nested`] test the harvest uses).
///
/// This is the ORDERED-vs-unordered discriminator for a pose/point array: a
/// per-element stamp means the elements are samples of ONE thing over time (a
/// trajectory), so their order is semantic and a polyline is the faithful drawing.
/// `geometry_msgs/PoseArray` (bare `Pose` elements) and `nav_msgs/GridCells`
/// (bare `Point` elements) carry no per-element time and are drawn as unordered
/// points.
///
/// It cannot separate an ordered RING from an unordered point bag when neither is
/// stamped (`geometry_msgs/Polygon.points` and `nav_msgs/GridCells.cells` have
/// byte-identical element shapes), which is precisely why those two are
/// NAME-mapped in [`crate::sink::classify_schema`] — the [`OccupancyGrid`
/// precedent](crate::sink::classify_schema): semantics a field shape cannot carry
/// get a named mapping, not a guess.
fn element_is_stamped(el: &FrameValue) -> bool {
    el.fields.iter().any(|nv| match &nv.value {
        FrameValueKind::Nested(inner) => is_time_like_nested(&nv.name, inner),
        _ => false,
    })
}

/// Every array field the walker REFUSED to decode (`NestedArrayOpaque`), as
/// `(field name, byte length)` — what the sink's `FallbackReason` resolution reads
/// to tell "this producer's array elements are not decodable by this build" (the
/// specific, field-naming reason) from "this message has no usable array" (the
/// archetype-level one).
///
/// The walker is a pure, per-frame, non-logging decoder: it degrades to opaque bytes and
/// says nothing, so without this the two are indistinguishable to an operator. The
/// live case is an **rmw**-published variable-element array (`nav_msgs/Path`,
/// `visualization_msgs/MarkerArray`, `vision_msgs/Detection3DArray`): the rmw
/// bridge agrees with canonical v1 on the ARRAY framing but packs an element BODY
/// with no offset table, so the walker's element audit refuses it (the rmw bridge owns
/// the reconciliation, after which those topics decode and never reach this path —
/// which is why the operator-facing message names the ENCODING gap and not that
/// bridge). Top-level fields only — an opaque array nested inside another message
/// is not a class anything produces today.
pub fn opaque_element_arrays(fv: &FrameValue) -> Vec<(String, usize)> {
    fv.fields
        .iter()
        .filter_map(|nv| match &nv.value {
            FrameValueKind::NestedArrayOpaque(b) => Some((nv.name.clone(), b.len())),
            _ => None,
        })
        .collect()
}

/// The most pixels an occupancy grid is rendered at — a defensive ceiling on the
/// per-frame grayscale buffer (a 32-megapixel map is already 4× a 4K image; a
/// corrupt `info` claiming more is refused, not allocated).
const MAX_OCCUPANCY_PIXELS: usize = 32 * 1024 * 1024;

/// A decoded occupancy grid ready to log as a grayscale [`rerun::Image`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OccupancyImage {
    /// Grid width in cells.
    pub width: u32,
    /// Grid height in cells.
    pub height: u32,
    /// Row-major `width * height` grayscale bytes, TOP row first (see
    /// [`occupancy_image`] for the row flip).
    pub gray: Vec<u8>,
}

/// Decode a `nav_msgs/OccupancyGrid` into a grayscale image — the
/// single biggest "renders as a text dump" gap on any nav2 robot (`/map`).
///
/// Reads `info.width` / `info.height` and the `int8[] data` cell buffer (which
/// the frame walker surfaces as raw bytes), mapping each ROS occupancy value to
/// the rviz-familiar grayscale: `0` (free) → white, `100` (occupied) → black,
/// the linear ramp in between, and `-1` (UNKNOWN) → mid gray. Rows are EMITTED
/// BOTTOM-UP: a ROS grid's row 0 is its `origin` row (+y up), while a rerun image's
/// row 0 is the TOP, so the flip is what makes the map match rviz instead of
/// appearing mirrored.
///
/// `None` (→ the caller degrades to an inspectable field dump, never a
/// mis-render) when the dimensions are absent, the cell buffer is shorter than
/// `width * height`, or the declared size is degenerate / beyond
/// `MAX_OCCUPANCY_PIXELS`.
pub fn occupancy_image(fv: &FrameValue) -> Option<OccupancyImage> {
    let info = nested_field(fv, "info")?;
    let width = field_u32(info, "width")?;
    let height = field_u32(info, "height")?;
    let cells = (width as usize).checked_mul(height as usize)?;
    if cells == 0 || cells > MAX_OCCUPANCY_PIXELS {
        return None;
    }
    let data = field_bytes(fv, "data")?;
    if data.len() < cells {
        return None;
    }
    let w = width as usize;
    let mut gray = Vec::with_capacity(cells);
    // Bottom-up: ROS row 0 is the grid origin (+y up); rerun row 0 is the top.
    for row in (0..height as usize).rev() {
        let start = row * w;
        for &b in &data[start..start + w] {
            gray.push(occupancy_to_gray(b));
        }
    }
    Some(OccupancyImage {
        width,
        height,
        gray,
    })
}

/// One occupancy cell byte → its grayscale level. The ROS cell is a SIGNED
/// `int8`: `-1` means UNKNOWN (mid gray), `0..=100` is percent-occupied mapped
/// linearly from white (free) to black (occupied). Any other negative value is
/// treated as unknown, and a nominally out-of-range positive value clamps to
/// fully occupied — total, never a panic.
fn occupancy_to_gray(cell: u8) -> u8 {
    let v = cell as i8;
    if v < 0 {
        return 128;
    }
    let occ = (v as u32).min(100);
    // 0 → 255 (white/free), 100 → 0 (black/occupied).
    (255 - (occ * 255).div_ceil(100)) as u8
}

/// Project a `sensor_msgs/LaserScan` to XY points on the z=0 plane:
/// `point[i] = (r·cos θ, r·sin θ, 0)` with `θ = angle_min + i·angle_increment`.
/// A range that is non-finite or outside `[range_min, range_max]` is SKIPPED
/// (no fabricated point). An absent `ranges` array yields no points.
pub fn laserscan_points(fv: &FrameValue) -> Vec<[f32; 3]> {
    let Some(ranges) = prim_array(fv, "ranges") else {
        return Vec::new();
    };
    let angle_min = field_f32(fv, "angle_min").unwrap_or(0.0);
    let angle_increment = field_f32(fv, "angle_increment").unwrap_or(0.0);
    let range_min = field_f32(fv, "range_min").unwrap_or(0.0);
    let range_max = field_f32(fv, "range_max").unwrap_or(f32::INFINITY);
    let mut out = Vec::new();
    for (i, r) in ranges.iter_f32().enumerate() {
        if !r.is_finite() || r < range_min || r > range_max {
            continue;
        }
        let theta = angle_min + i as f32 * angle_increment;
        out.push([r * theta.cos(), r * theta.sin(), 0.0]);
    }
    out
}

/// The plottable `unitree_go/SportModeState` telemetry (best-effort against the
/// Unitree Go2 SDK layout — any field absent on a slightly different store
/// definition is simply skipped, never a failure): body `velocity` xyz +
/// `position` xyz, plus `yaw_speed` / `body_height` / `foot_raise_height`.
pub fn sportmode_scalars(fv: &FrameValue) -> Vec<(String, f64)> {
    let mut out = Vec::new();
    push_prim_indexed(
        &mut out,
        fv,
        "velocity",
        &["velocity/x", "velocity/y", "velocity/z"],
    );
    push_prim_indexed(
        &mut out,
        fv,
        "position",
        &["position/x", "position/y", "position/z"],
    );
    for name in ["yaw_speed", "body_height", "foot_raise_height"] {
        if let Some(v) = field_num(fv, name) {
            out.push((name.to_string(), v));
        }
    }
    out
}

/// The most elements a numeric ARRAY field is expanded into indexed plot series.
/// A robot's joint bank (`sensor_msgs/JointState.position`, a
/// `std_msgs/Float64MultiArray.data`) is the motivating case — a humanoid's 30-60
/// joints must all plot — while an unbounded expansion would turn a single
/// megapixel `uint16[]` scan into a million `log_scalar` calls per frame. An array
/// LONGER than this is skipped and REPORTED (see [`SkippedSeries`]), never
/// silently truncated.
pub const MAX_ARRAY_SERIES: usize = 64;

/// The most plot series ONE fixed array of nested STRUCTS expands into.
///
/// [`MAX_ARRAY_SERIES`] caps a numeric array's ELEMENT count, which is the whole
/// story for a `float64[]` joint bank (one element = one series). A struct array
/// multiplies: a `MotorState[20]` whose element declares 12 numeric fields is 240
/// series from ONE field, comfortably under the 64-ELEMENT cap and therefore
/// invisible to it. That is the shape reported from a live robot — a joint bank published as
/// a bank of STRUCTS rather than a bank of numbers, which is what every humanoid,
/// arm and quadruped firmware actually sends.
///
/// The cap is spent FIELD-first, not element-first: see the
/// [`FrameValueKind::Array`] arm of `push_field`, which regroups a struct
/// array's series FIELD-major and then truncates at a whole-field boundary. So a
/// 20-motor bank over budget plots `q` for all 20 motors, then `dq` for all 20,
/// and stops — rather than plotting the first five motors completely and leaving
/// fifteen joints with no trace at all. What is dropped is REPORTED
/// ([`SkippedSeries::StructArrayFields`]), never silently truncated.
///
/// 64 matches [`MAX_ARRAY_SERIES`] deliberately: both answer "how many lines may
/// ONE declared field contribute to a plot", and a reader who has learned one
/// number has learned the other. They are separate constants because they cap
/// different quantities (elements vs series) and may need to diverge.
pub const MAX_STRUCT_ARRAY_SERIES: usize = 64;

/// The most plot series ONE topic's harvest emits, across every field.
///
/// [`MAX_ARRAY_SERIES`] and [`MAX_STRUCT_ARRAY_SERIES`] each bound what ONE
/// declared field may contribute. Neither bounds their SUM: a message declaring
/// five struct arrays is five times under both caps and still 320 lines in one
/// panel. This is the per-MESSAGE backstop — the one number that makes "how many
/// plot lines can a single topic produce" answerable without reading a schema.
///
/// Applied LAST, over the finished harvest, so the per-field curation above has
/// already run and what survives is the earliest-declared 256 series. Declaration
/// order is the message author's own ordering, which is the only ordering the
/// harvest can claim to understand; the overflow is REPORTED
/// ([`SkippedSeries::TotalCap`]), never silently dropped.
///
/// 256 is four times a single field's budget: a message needs to declare four
/// full-width banks before the backstop is what stops it, so on any ordinary
/// telemetry message it never binds and the per-field reports stay the ones an
/// operator reads.
pub const MAX_TOTAL_SERIES: usize = 256;

/// How many NESTED message hops the numeric harvest descends. 3 is the
/// deepest reach any ROS common-interfaces telemetry needs:
/// `TwistWithCovarianceStamped.twist(1).twist(2).linear(3).x`. A deeper tree is
/// left to the structured field dump.
const MAX_NESTED_SERIES_DEPTH: u8 = 3;

/// A numeric field the series harvest DELIBERATELY did not expand, so the caller
/// can say so ONCE per schema instead of dropping data silently (Principle: loud
/// over silent). Pure — produced alongside the samples by [`harvest_series`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SkippedSeries {
    /// A numeric array longer than [`MAX_ARRAY_SERIES`]: expanding it would emit
    /// one plot series per element (see the const).
    Oversized {
        /// The `a/b/c`-qualified field path.
        field: String,
        /// The array's element count.
        count: usize,
    },
    /// A ROS covariance MATRIX (`covariance` / `*_covariance` — the
    /// `PoseWithCovariance` / `Imu` / `NavSatFix` convention). Its entries are a
    /// row-major matrix, not a bank of time series, so plotting them would be
    /// meaningless noise beside the real signals.
    Covariance {
        /// The `a/b/c`-qualified field path.
        field: String,
        /// The matrix's element count.
        count: usize,
    },
    /// A nested message carrying numbers BELOW the
    /// `MAX_NESTED_SERIES_DEPTH`-hop harvest limit. Previously a SILENT skip
    /// while its array / covariance siblings were loud — an inconsistency that
    /// made a deep telemetry struct look like a bug.
    TooDeep {
        /// The `a/b/c`-qualified field path of the un-descended nested message.
        field: String,
    },
    /// A DYNAMIC array of nested messages (a `PoseStamped[]`, a
    /// `DiagnosticStatus[]`, a `JointTrajectoryPoint[]`) the walker decoded
    /// element-by-element. Deliberately NOT expanded into plot series, for two
    /// independent reasons:
    ///
    /// - **magnitude** — a 5 000-pose `/plan` would emit 15 000 scalar series per
    ///   frame (the fixed [`FrameValueKind::Array`] arm's per-index expansion is
    ///   safe only because a FixedArray's length is pinned by the schema);
    /// - **frame-invariance** — a dynamic array's LENGTH varies frame to frame, so
    ///   a length-gated expansion would make the plottable-series answer (and
    ///   therefore a topic's once-resolved layout) depend on which frame arrived
    ///   first. See [`declares_plottable_series`].
    ///
    /// Earlier this class was INVISIBLE: a `NestedArray` fell to the
    /// harvest's numeric catch-all, plotted nothing, and reported nothing.
    ElementArray {
        /// The `a/b/c`-qualified field path.
        field: String,
        /// The array's element count IN THIS FRAME (a dynamic array, so this
        /// varies — the reason it is never expanded).
        count: usize,
    },
    /// A FIXED array of nested structs whose full expansion would exceed
    /// [`MAX_STRUCT_ARRAY_SERIES`], so only the first `fields_kept` numeric fields
    /// of each element are plotted (FIELD-major — every element keeps the fields
    /// that survive, see the constant). `fields_kept == 0` means the element count
    /// alone is over budget and NOTHING from this field is plotted.
    ///
    /// Reported per FIELD rather than per dropped series: a 20×12 struct array
    /// over budget drops 180 series, and naming 180 paths is not a diagnostic
    /// anyone reads. The element count × the field count reconstructs exactly what
    /// was withheld.
    StructArrayFields {
        /// The `a/b/c`-qualified field path of the struct array.
        field: String,
        /// The array's element count (fixed by the schema).
        elements: usize,
        /// How many numeric FIELDS one element declares.
        ///
        /// Fields, not series. At nesting depth ≥ 2 one field can be
        /// worth many series (a `MotorState[3]` inside a `LegState`), and reporting
        /// the series count under this name told an operator their element declared
        /// 36 fields when it declares one.
        fields_per_element: usize,
        /// How many numeric SERIES one element expands into — `fields_per_element`
        /// only at depth 1, larger when a field is itself an array.
        series_per_element: usize,
        /// How many of those FIELDS survived the cap (0 when none did).
        fields_kept: usize,
        /// How many SERIES per element those kept fields are worth — again equal
        /// to `fields_kept` only at depth 1. Carried so the report can state the
        /// exact number of series withheld rather than a count that is right only
        /// when every field happens to be one series.
        series_kept: usize,
    },
    /// The whole message declares more plottable series than
    /// [`MAX_TOTAL_SERIES`], so the earliest-declared `MAX_TOTAL_SERIES` are
    /// plotted and the rest are not. The per-FIELD curation has already run when
    /// this is produced, so reaching it means the message is wide in its own
    /// right — not that one field ran away.
    TotalCap {
        /// How many series past the cap were not plotted.
        dropped: usize,
        /// The `a/b/c`-qualified path of the FIRST series that did not make it —
        /// the point in declaration order where the plot stops, which is what an
        /// operator needs to know. The remaining paths are deliberately not
        /// enumerated: a diagnostic naming hundreds of paths is not read.
        first_dropped: String,
    },
}

impl SkippedSeries {
    /// The `a/b/c`-qualified field path this skip names, when it names one.
    ///
    /// [`SkippedSeries::TotalCap`] is the exception: it reports a per-MESSAGE
    /// overflow rather than one field, so it carries the first dropped path under
    /// a different name and is deliberately not addressable here (nothing that
    /// filters by field position should ever match it).
    fn field_path(&self) -> Option<&str> {
        match self {
            Self::Oversized { field, .. }
            | Self::Covariance { field, .. }
            | Self::TooDeep { field }
            | Self::ElementArray { field, .. }
            | Self::StructArrayFields { field, .. } => Some(field),
            Self::TotalCap { .. } => None,
        }
    }
}

impl std::fmt::Display for SkippedSeries {
    /// The operator-facing phrase for one skip — the field path plus WHY it was
    /// not plotted, so the caller's diagnostic needs no per-variant formatting.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Oversized { field, count } => write!(
                f,
                "{field} ({count} elements, over the {MAX_ARRAY_SERIES}-series plot cap)"
            ),
            Self::Covariance { field, count } => {
                write!(f, "{field} ({count} elements, a covariance matrix)")
            }
            Self::TooDeep { field } => write!(
                f,
                "{field} (nested deeper than the {MAX_NESTED_SERIES_DEPTH}-hop harvest limit)"
            ),
            Self::ElementArray { field, count } => write!(
                f,
                "{field} ({count} message element(s), a dynamic element array — never expanded \
                 into plot series)"
            ),
            Self::StructArrayFields {
                field,
                elements,
                fields_per_element,
                series_per_element,
                fields_kept,
                series_kept,
            } => {
                let total = elements * series_per_element;
                // Saturating, like the `TotalCap` arm below: `SkippedSeries` is a
                // PUBLIC type and `cerulion-vizd` can construct one, so an
                // inconsistent `(series_per_element, series_kept)` pair reaching
                // this arm must render a wrong number rather than panic on debug
                // overflow. Every pair the harvest itself produces satisfies
                // `series_kept <= series_per_element`, so this cannot saturate on
                // any path in this crate.
                let dropped = total.saturating_sub(elements * series_kept);
                // Every multiplication PRINTED here has to be one
                // that actually holds. At depth 1 a field is one series, so
                // `elements × fields = total` is true and reads naturally. At
                // depth >= 2 it is NOT (a `MotorState[3]` field is worth 3
                // series), and printing it anyway produced lines like
                // "20 × 3 = 80" — arithmetic an operator can check in their head
                // and find wrong, which discredits the whole diagnostic. The deep
                // form multiplies by SERIES and reports the field count beside it.
                if fields_per_element == series_per_element {
                    write!(
                        f,
                        "{field} ({elements} struct element(s) × {fields_per_element} numeric \
                         field(s) = {total} series; the first {fields_kept} field(s) of EVERY \
                         element are plotted, {dropped} series not plotted — over the \
                         {MAX_STRUCT_ARRAY_SERIES}-series per-struct-array plot cap)"
                    )
                } else {
                    write!(
                        f,
                        "{field} ({elements} struct element(s) × {series_per_element} series each \
                         = {total} series, across {fields_per_element} numeric field(s) per \
                         element; the first {fields_kept} field(s) ({series_kept} series) of \
                         EVERY element are plotted, {dropped} series not plotted — over the \
                         {MAX_STRUCT_ARRAY_SERIES}-series per-struct-array plot cap)"
                    )
                }
            }
            Self::TotalCap {
                dropped,
                first_dropped,
            } => write!(
                f,
                "{first_dropped} and {} later series ({dropped} in total past the \
                 {MAX_TOTAL_SERIES}-series per-topic plot cap; the earliest-declared \
                 {MAX_TOTAL_SERIES} series are plotted)",
                dropped.saturating_sub(1)
            ),
        }
    }
}

/// Record a [`SkippedSeries::TooDeep`] for a nested value the harvest could not
/// descend into — but ONLY when it actually carries numbers within a bounded
/// look-ahead. A deep block of strings / bools is not lost telemetry, so
/// reporting it would be noise (the diagnostic must stay worth reading).
///
/// Without this report the over-depth skip is SILENT while its array /
/// covariance siblings are loud — an inconsistency that makes a deep telemetry
/// struct look like a bug instead of a documented limit.
///
/// **This deliberately does NOT latch `withholds` here** — an over-depth block
/// is the one withholding class the field dump cannot widen, so electing the dual
/// view for it would hand the operator a pane that does not answer the question.
/// The two ceilings are the SAME hop, which is easy to get wrong from the
/// constants alone (`DUMP_MAX_DEPTH` = 4 reads as deeper than
/// `MAX_NESTED_SERIES_DEPTH` = 3):
///
/// - the harvest enters at `budget = MAX_NESTED_SERIES_DEPTH` and refuses a
///   nested field once `budget == 0` — the FOURTH hop;
/// - the dump enters at `depth = 0` and refuses one once `depth + 1 >=
///   DUMP_MAX_DEPTH` — also the FOURTH hop.
///
/// So the dump prints such a block's NAME and `(nested, not expanded)`, never its
/// values, and this report — which already names the path — is strictly more
/// informative. Pinned by
/// `the_dump_cannot_widen_an_over_depth_block_so_it_does_not_elect`.
fn report_too_deep(out: &mut HarvestOut, path: &str, inner: &FrameValue) {
    if contains_numeric(inner, MAX_NESTED_SERIES_DEPTH) {
        out.skipped.push(SkippedSeries::TooDeep {
            field: path.to_string(),
        });
    }
}

/// True when `fv` carries at least one plottable number within `depth` further
/// hops — the guard that keeps the [`SkippedSeries::TooDeep`] report ACCURATE (a
/// deep nested block of strings / bools is not lost telemetry, so reporting it
/// would be noise). Bounded, so it is total on any tree.
///
/// Applies the SAME time-like exclusion the harvest itself applies
/// ([`is_time_like_nested`]): a `header` / `stamp` is the frame's own clock, not
/// telemetry, so the harvest skips it SILENTLY at every depth. Counting its
/// `sec` / `nanosec` here would report a deep block that carries only a timestamp
/// as lost telemetry — a skip diagnostic naming a value the harvest would have
/// dropped anyway, which is exactly the noise this guard exists to prevent.
fn contains_numeric(fv: &FrameValue, depth: u8) -> bool {
    fv.fields.iter().any(|nv| match &nv.value {
        FrameValueKind::PrimArray(pa) => pa.count > 0,
        FrameValueKind::Nested(inner) => {
            !is_time_like_nested(&nv.name, inner) && depth > 0 && contains_numeric(inner, depth - 1)
        }
        FrameValueKind::Array(items) => items.iter().any(|item| match item {
            FrameValueKind::Nested(inner) => depth > 0 && contains_numeric(inner, depth - 1),
            other => num_f64(other).is_some(),
        }),
        // A DYNAMIC element array is NEVER harvested (it is reported as
        // [`SkippedSeries::ElementArray`] instead), so counting its numbers here
        // would make a `TooDeep` report claim lost telemetry the harvest would have
        // skipped anyway — exactly the noise this guard exists to prevent. Explicit
        // rather than left to the catch-all so the decision is visible.
        FrameValueKind::NestedArray { .. } => false,
        other => num_f64(other).is_some(),
    })
}

/// True for a ROS covariance matrix field (`covariance`, or any `*_covariance`
/// such as `position_covariance` / `orientation_covariance` / `twist_covariance`).
/// Case-insensitive; the ROS common-interfaces naming convention, not a
/// robot-specific name.
fn is_covariance_field(name: &str) -> bool {
    let lower = name.to_ascii_lowercase();
    lower == "covariance" || lower.ends_with("_covariance")
}

/// True for a nested field that carries a TIMESTAMP rather than telemetry — a
/// `std_msgs/Header` (or a bare `stamp` / `builtin_interfaces` Time/Duration).
/// Its `sec`/`nanosec` are the frame's own clock (already the plot's X axis), so
/// plotting them as series is noise. Matched on the nested value's SCHEMA first
/// (shape, not name) with the conventional field names as the fallback for a
/// schema-less producer.
fn is_time_like_nested(name: &str, inner: &FrameValue) -> bool {
    matches!(
        inner.schema_name.as_str(),
        "std_msgs/Header" | "builtin_interfaces/Time" | "builtin_interfaces/Duration"
    ) || matches!(name, "header" | "stamp")
}

/// The GENERIC numeric harvest — every plottable number in a decoded
/// frame, with the fields it deliberately skipped.
///
/// Walks the value in declaration order and yields, as `(series path, value)`:
///
/// - every numeric scalar field (`voltage`, `sec`, …) under its own name;
/// - every numeric ARRAY field of at most [`MAX_ARRAY_SERIES`] elements, expanded
///   to `field/0 .. field/n-1` (the `JointState.position` / `Float64MultiArray.data`
///   win — a whole joint bank becomes one plot of N lines);
/// - every NESTED message's numbers, path-qualified `group/field`, up to
///   `MAX_NESTED_SERIES_DEPTH` hops (so `Wrench.force.x`, `MagneticField.magnetic_field.z`
///   and `TwistWithCovarianceStamped.twist.twist.linear.x` all plot) — EXCEPT a
///   timestamp-shaped nested (`is_time_like_nested`).
///
/// `bool` is deliberately NOT harvested (the existing decision that bools/enums
/// are not auto-plotted; a lone-bool message stays an inspectable field dump), and
/// strings / byte blobs are not numbers. EVERY numeric skip — a covariance matrix,
/// an over-long array, AND a nested message below the hop limit that
/// still carries numbers — lands in the returned [`SkippedSeries`] list so the
/// caller reports it. No numeric field is dropped silently.
///
/// Pure + oracle-tested: the "a never-seen robot's telemetry still plots" claim is
/// testable without Rerun or a transport.
pub fn harvest_series(fv: &FrameValue) -> (Vec<(String, f64)>, Vec<SkippedSeries>) {
    harvest_series_excluding(fv, &[])
}

/// [`harvest_series`] with a set of already-consumed field PATHS removed — the
/// sibling harvest for a SPATIALLY-classified frame (see
/// [`spatial_sibling_series`]).
///
/// An excluded path prunes exactly that node (and its subtree): the pose's own
/// `position` / `orientation` are the transform, not four extra plot series.
/// Everything else — including a sibling INSIDE an excluded node's parent, e.g.
/// `pose/confidence` beside a consumed `pose/position` — is harvested normally,
/// so no number is dropped by the exclusion.
pub fn harvest_series_excluding(
    fv: &FrameValue,
    exclude: &[String],
) -> (Vec<(String, f64)>, Vec<SkippedSeries>) {
    let out = harvest_walk(fv, exclude);
    (out.samples, out.skipped)
}

/// True when the frame's SHAPE declares at least one plottable series, EVEN IF
/// this particular frame carries no values for it — the frame-INVARIANT
/// classification predicate.
///
/// [`harvest_series`] answers "what does THIS frame plot"; a declared
/// `float64[] joint_positions` that happens to be EMPTY in this frame plots
/// nothing, yet the topic is unquestionably a plotting topic. Every consumer that
/// decides a topic's LAYOUT must ask this question instead, because a layout is
/// resolved ONCE per topic (`cerulion-vizd` resolves a topic's archetype from its
/// first decodable frame and keeps it): deciding from one frame's VALUES froze a
/// topic whose first frame was momentarily empty into the view-less archetype,
/// after which every later frame's series were logged into a view that did not
/// exist — invisible data, permanently, on exactly the never-seen-robot path.
///
/// Only a `PrimArray`'s LENGTH varies frame to frame (the schema fixes the field
/// set, and a nested-message `Array` is a FixedArray), so shape-invariance is
/// achieved by counting a declared numeric array whatever its current length.
/// An array OVER [`MAX_ARRAY_SERIES`] still does not count: it is never expanded,
/// so counting it would hand the topic a permanently EMPTY plot panel (it is
/// reported instead — see [`SkippedSeries`]).
pub fn declares_plottable_series(fv: &FrameValue) -> bool {
    declares_plottable_series_excluding(fv, &[])
}

/// [`declares_plottable_series`] with a spatial primitive's consumed paths
/// pruned — the predicate that elects the `…WithScalars` twin of a
/// spatial archetype (see [`crate::sink::infer_archetype_from_shape`]).
pub fn declares_plottable_series_excluding(fv: &FrameValue, exclude: &[String]) -> bool {
    harvest_walk(fv, exclude).plottable
}

/// Whether the frame's SHAPE declares at least one TEXT field the
/// numeric harvest silently drops — the classification predicate that elects
/// [`crate::sink::ArchetypeKind::ScalarsWithText`] over the plots-only
/// [`crate::sink::ArchetypeKind::Scalars`].
///
/// **The bug this closes.** The `Scalars` render arm logs numeric samples and
/// NOTHING else, and its layout is a lone `time_series` view, so a message that
/// mixes numbers and strings plots the numbers and loses the strings with no
/// diagnostic anywhere — on a vendor status/response/diagnostic message the
/// strings are usually the PAYLOAD and the numbers are the envelope.
///
/// Rides the SAME walk as [`declares_plottable_series`], which is what makes it
/// correct rather than merely convenient: the descent rules are shared, so a
/// timestamp-shaped nested (`is_time_like_nested` — `std_msgs/Header` and any
/// field named `header`/`stamp`) is skipped HERE too. Without that, every
/// `*Stamped` message in ROS would qualify on its `header.frame_id` alone and
/// the dual view would be noise on topics whose real payload is pure numbers.
///
/// **Frame-INVARIANT, deliberately narrow.** Only a decoded
/// [`FrameValueKind::Str`] counts, because a declared `string` field decodes to
/// `Str` on EVERY frame whatever its value (an empty string included), and a
/// topic's layout is resolved ONCE from its first decodable frame — the standing
/// rule. THREE shapes are therefore knowingly NOT counted, and the
/// reason differs per shape:
///
/// - a DYNAMIC `string[]` (a `NestedArray` of `Str`): its element types are only
///   visible when the array is NON-EMPTY, so counting it would let an idle first
///   frame freeze the topic into the plots-only twin — the exact failure this
///   rule exists to avoid. Such a message stays `Scalars`.
/// - a FIXED-length `string[N]` of variable strings: the walker decodes it
///   through the SAME canonical element path, yielding a `NestedArray` that
///   carries NO arity marker — so this predicate cannot tell it from the dynamic
///   case above even though a fixed array's length really is frame-invariant.
///   One rule covers both rather than a distinction the decoded value does not
///   support. (A `string_fixed[N]` in the FIXED section is a third spelling and
///   is not counted either, for a stronger reason: the walker surfaces it as
///   [`FrameValueKind::NestedArrayOpaque`] — undecoded bytes, with no elements
///   to inspect at all.)
/// - a `string` field whose bytes fail UTF-8 validation, which the walker
///   surfaces as [`FrameValueKind::Bytes`] (indistinguishable from a `uint8[]`
///   blob, which must NOT count — a camera's `data` is not text).
///
/// `bool` is likewise not counted: it is dropped by the numeric harvest too (the
/// standing no-auto-plot decision), so a `{speed, estopped}` message still loses
/// its flag. That is a REAL residual of the same class, deliberately left for its
/// own issue rather than widened here — counting bools would hand a text pane to
/// a large share of ordinary telemetry.
///
/// Pure + oracle-tested.
pub fn declares_text_fields(fv: &FrameValue) -> bool {
    harvest_walk(fv, &[]).declares_text
}

/// Whether the curation WITHHELD numeric fields from the plot on
/// evidence that cannot change frame to frame — the second predicate (beside
/// [`declares_text_fields`]) that elects
/// [`crate::sink::ArchetypeKind::ScalarsWithText`] over the plots-only
/// [`crate::sink::ArchetypeKind::Scalars`].
///
/// **The bug this closes.** Series curation gave a wide message REAL curated plots — a
/// `MotorState[20]` plots `q` for all twenty motors instead of drowning the panel
/// in 240 lines — but the fields it withholds are then rendered NOWHERE: a
/// `Scalars` topic's only view is the plot, so the standing contract was
/// "read them with `cerulion topic echo`", i.e. leave the viewer. A topic whose
/// curation withheld something therefore also earns the structured field dump —
/// PROVIDED the dump can actually show it, which is the second condition below.
/// The dump is bounded: it previews each array, enumerates the first elements of
/// a struct array with their members, and stops at its own depth and line
/// ceilings. It does NOT out-reach the harvest — a claim that it
/// "expands one hop deeper" is false in both directions (see
/// condition (2) and `report_too_deep`).
///
/// TWO conditions must BOTH hold for a withholding class to elect, and the second
/// is the one that is easy to miss:
///
/// **(1) The verdict must be frame-INVARIANT.** A topic's layout is resolved
/// ONCE, from whichever frame arrived first (`cerulion-vizd`'s
/// `resolve_from_frame`), while the sink re-classifies EVERY frame — so a
/// predicate that could answer differently on two frames of one topic would log a
/// `TextDocument` into a topic whose views do not include one. That is the
/// silent-dump bug exactly, and it is why this reads the curation's SHAPE evidence rather
/// than its per-frame skip list.
///
/// **(2) The DUMP must actually be able to show what was withheld.** Electing a
/// class the dump cannot widen hands the operator a second pane that does not
/// answer the question and a diagnostic that says it does — strictly worse than
/// the earlier contract, which at least pointed at `cerulion topic echo`.
///
/// So the elected set is exactly the classes where the plot shows NOTHING for a
/// field while the dump shows a real preview of its values:
///
/// - a COVARIANCE matrix (decided by the field's name and kind — the count is
///   never consulted; the dump previews its first `DUMP_ARRAY_PREVIEW`
///   entries);
/// - a FIXED array of nested structs over [`MAX_ARRAY_SERIES`] elements, or
///   curated by [`MAX_STRUCT_ARRAY_SERIES`] — a `FrameValueKind::Array`'s length
///   and its element's field set are pinned by the schema, and the dump
///   ENUMERATES its first `DUMP_ARRAY_PREVIEW` elements with every member
///   (**the `/lowstate` case**: the plot keeps 3 of 12 members for all 20 motors,
///   the dump shows all 12 for the first 8).
///
/// FOUR withholding classes are knowingly NOT counted, and they fail DIFFERENT
/// conditions — worth separating, because the first two are about frames and the
/// last two are about the dump's own ceilings:
///
/// - a numeric array over [`MAX_ARRAY_SERIES`] — fails (1): the walker cannot
///   tell a dynamic `float64[]` from a fixed `float64[100]` (both decode to a
///   `PrimArray`, which carries no arity marker), so a 3-element first frame and
///   a 1081-element second frame would classify differently;
/// - a DYNAMIC element array ([`SkippedSeries::ElementArray`]) — fails (1): it is
///   reported only when non-empty;
/// - an over-depth nested block ([`SkippedSeries::TooDeep`]) — fails (2), ALWAYS:
///   the harvest refuses a nested field at its FOURTH hop (entry
///   `budget = MAX_NESTED_SERIES_DEPTH`, refusing at `budget == 0`) and the dump
///   refuses at the same fourth hop (entry `depth = 0`, refusing at
///   `depth + 1 >= DUMP_MAX_DEPTH`). The constants LOOK asymmetric (4 vs 3) and
///   the reach is identical; the dump prints such a block's NAME and
///   `(nested, not expanded)`, never its values, which the skip report already
///   says better;
/// - [`MAX_TOTAL_SERIES`] — fails (2) on the shape that reaches it most plainly,
///   and (1) in general: `DUMP_MAX_LINES` (200) is BELOW the cap (256), so on a
///   flat wide message the dump stops before the first withheld field and renders
///   a strict PREFIX of what the plot already drew; and a dynamic array's length
///   moves the total anyway.
///
/// Every one of those is still REPORTED (`SkippedSeries`), so nothing becomes
/// silent — such a topic keeps the earlier contract and its diagnostic says
/// exactly that. Pure + oracle-tested, including the two dump-reach facts
/// (`the_dump_cannot_widen_an_over_depth_block_so_it_does_not_elect` and
/// `the_dump_cannot_widen_a_flat_wide_message_so_it_does_not_elect`), which are
/// asserted against the RENDERED document rather than argued from the constants.
pub fn declares_withheld_series(fv: &FrameValue) -> bool {
    harvest_walk(fv, &[]).withholds
}

/// The three SHAPE answers the classifier's scalar rung needs, from ONE walk.
///
/// The rung asks all three of every frame of every unmapped plot topic, and the
/// walk that answers them allocates a `String` per sample — a `/lowstate`-class
/// bank at 500 Hz is ~240 of them per walk. Asking them separately would run it
/// three times over; bundling makes the answer FREE and halves what the
/// earlier rung already paid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScalarShape {
    /// [`declares_plottable_series`].
    pub plottable: bool,
    /// [`declares_text_fields`].
    pub declares_text: bool,
    /// [`declares_withheld_series`].
    pub withholds: bool,
}

/// [`ScalarShape`] for `fv` — one walk, three answers. See
/// [`crate::sink::infer_archetype_with_stability`], its one production caller.
pub fn scalar_shape(fv: &FrameValue) -> ScalarShape {
    let out = harvest_walk(fv, &[]);
    ScalarShape {
        plottable: out.plottable,
        declares_text: out.declares_text,
        withholds: out.withholds,
    }
}

/// ONE walk yielding all three answers: the samples THIS frame carries, the
/// fields deliberately not expanded, and whether the frame's SHAPE declares any
/// plottable series at all. Single traversal so the value view and the shape view
/// can never disagree about which fields are plottable.
fn harvest_walk(fv: &FrameValue, exclude: &[String]) -> HarvestOut {
    let mut out = HarvestOut::default();
    let mut path = String::new();
    push_series(
        &mut out,
        fv,
        &mut path,
        MAX_NESTED_SERIES_DEPTH,
        MAX_STRUCT_ARRAY_SERIES,
        exclude,
        // The root's own fields render at dump row 0.
        0,
    );
    apply_total_series_cap(&mut out);
    out
}

/// Enforce [`MAX_TOTAL_SERIES`] over the FINISHED harvest and report the
/// overflow.
///
/// Applied here, once, rather than inside the walk, for two reasons. The walk must
/// run to completion whatever the sample count, because it also latches
/// `plottable` and `declares_text` — the two answers that decide a topic's
/// ARCHETYPE, and therefore which views it is given. A cap that short-circuited
/// the walk would let a wide message's trailing `string` field go unseen and hand
/// the topic the plots-only archetype, losing its text with no diagnostic (the
/// silent-text-drop failure mode, re-opened by the curation fix). And the per-field
/// curation upstream is what decides which series exist at all, so the backstop
/// must run after it or it would truncate series the curation was about to
/// re-order anyway.
///
/// `plottable` deliberately stays latched: a topic at the cap plots
/// `MAX_TOTAL_SERIES` real lines, so it has unquestionably earned its plot view.
///
/// **This deliberately does NOT latch `withholds` here.** Two independent
/// reasons, either one sufficient:
///
/// - the dump cannot be relied on to SHOW what this cap withheld. It stops at
///   [`DUMP_MAX_LINES`] = 200 lines while the cap admits [`MAX_TOTAL_SERIES`] =
///   256 series, so on a flat wide message the dump ends BEFORE the first
///   withheld field — it renders a strict PREFIX of what the plot already drew;
/// - the overflow is not frame-invariant in general: a dynamic array's length
///   moves the total, so "this frame produced 300 series" is not evidence the
///   next frame will.
///
/// Such a message is still REPORTED and still plots its earliest-declared series.
/// Pinned by `the_dump_cannot_widen_a_flat_wide_message_so_it_does_not_elect`.
fn apply_total_series_cap(out: &mut HarvestOut) {
    if out.samples.len() <= MAX_TOTAL_SERIES {
        return;
    }
    let dropped = out.samples.len() - MAX_TOTAL_SERIES;
    let first_dropped = out.samples[MAX_TOTAL_SERIES].0.clone();
    out.samples.truncate(MAX_TOTAL_SERIES);
    out.skipped.push(SkippedSeries::TotalCap {
        dropped,
        first_dropped,
    });
}

/// Everything ONE [`harvest_walk`] yields. Bundled rather than returned as a
/// tuple so each consumer names what it reads, and so a new answer (such as
/// `declares_text`) does not push the recursive walkers past a readable arity.
#[derive(Default)]
struct HarvestOut {
    /// The numeric samples THIS frame carries, as `(series path, value)`.
    samples: Vec<(String, f64)>,
    /// Fields deliberately not expanded, for the caller to report.
    skipped: Vec<SkippedSeries>,
    /// Whether the frame's SHAPE declares any plottable series — see
    /// [`declares_plottable_series`].
    plottable: bool,
    /// Whether the frame's SHAPE declares any TEXT the numeric harvest
    /// cannot plot — see [`declares_text_fields`].
    declares_text: bool,
    /// Whether the curation WITHHELD numeric fields on evidence that
    /// cannot change frame to frame — see [`declares_withheld_series`].
    withholds: bool,
    /// When `Some((path_len, marks))`, [`push_series`] appends
    /// `out.samples.len()` after each field it completes AT the level whose path
    /// prefix is exactly `path_len` bytes long — i.e. the FIELD BOUNDARIES of one
    /// struct-array element.
    ///
    /// Keyed on the path LENGTH rather than a hop counter because a hop counter
    /// cannot separate an element's own fields from a nested struct's: both sit
    /// one recursion deeper, while only the element's fields share its exact path
    /// prefix. The boundaries are what makes the whole-field truncation rule
    /// correct when a field is itself worth several series (a nested array), which
    /// a flat `produced / elements` arithmetic cannot see — see
    /// [`regroup_struct_array_field_major`].
    field_marks: Option<(usize, Vec<(String, usize)>)>,
}

/// One level of [`harvest_series`]'s declaration-order DFS. `path` is the
/// REUSED path buffer: each field pushes its own segment, recurses, then
/// truncates back, so a `String` is allocated only where one is actually
/// EMITTED (a sample or a skip) rather than once per visited field. This walk
/// runs on every spatial frame — twice, since classification must stay a pure
/// function of the frame while the render arm harvests again — so a bare
/// `{header, pose}` at 500 Hz paid a handful of throwaway allocations per frame
/// for paths nothing ever kept.
///
/// `budget` is the remaining nested-hop allowance; `series_budget` is the most
/// plot series a struct array AT THIS POINT of the tree may expand into (see
/// [`MAX_STRUCT_ARRAY_SERIES`]); `exclude` holds the `a/b`-qualified paths a
/// spatial primitive already consumed (empty for the plain harvest);
/// `out.plottable` is latched true by any field the harvest WOULD expand,
/// whatever this frame's array lengths happen to be — see
/// [`declares_plottable_series`] — and `out.declares_text` by any text field it
/// would drop (see [`declares_text_fields`]).
fn push_series(
    out: &mut HarvestOut,
    fv: &FrameValue,
    path: &mut String,
    budget: u8,
    series_budget: usize,
    exclude: &[String],
    dump_depth: u8,
) {
    // Are THESE fields the ones a struct-array regroup asked to have
    // boundaries recorded for? Compared on the path prefix, so a nested struct one
    // hop deeper (whose path is strictly longer) never contributes a boundary.
    let record_marks = matches!(&out.field_marks, Some((at, _)) if *at == path.len());
    for nv in &fv.fields {
        let parent_len = path.len();
        if !path.is_empty() {
            path.push('/');
        }
        path.push_str(&nv.name);
        push_field(out, nv, path, budget, series_budget, exclude, dump_depth);
        path.truncate(parent_len);
        if record_marks {
            let so_far = out.samples.len();
            if let Some((_, marks)) = &mut out.field_marks {
                marks.push((nv.name.clone(), so_far));
            }
        }
    }
}

/// One FIELD of [`push_series`]'s DFS, with `path` already carrying that field's
/// qualified path. Split out so every arm can `return` early while the caller
/// still restores the shared path buffer exactly once.
fn push_field(
    out: &mut HarvestOut,
    nv: &NamedValue,
    path: &mut String,
    budget: u8,
    series_budget: usize,
    exclude: &[String],
    dump_depth: u8,
) {
    // Already rendered as part of the spatial primitive — not a lost number.
    if exclude.iter().any(|e| e == path) {
        return;
    }
    match &nv.value {
        FrameValueKind::PrimArray(pa) => {
            if is_covariance_field(&nv.name) {
                // Withheld on the field's NAME and KIND alone — the
                // count is never consulted — so a declared covariance matrix is
                // withheld on every frame. Frame-invariant ⇒ it may elect the
                // dual view.
                //
                // **R2': gated on the threaded dump row.** A matrix renders its
                // preview on its OWN row, so unlike a struct array it needs no
                // extra allowance — but that row still has to EXIST. This site
                // shipped ungated because "a leaf renders wherever it is reached"
                // holds under plain nesting and stops holding the moment an
                // element row eats a level: a covariance member of a struct-array
                // element can sit past the ceiling while the harvest still
                // curates it.
                out.withholds |= dump_row_renders(dump_depth);
                out.skipped.push(SkippedSeries::Covariance {
                    field: path.clone(),
                    count: pa.count,
                });
                return;
            }
            if pa.count > MAX_ARRAY_SERIES {
                // Deliberately does NOT latch `withholds`. This arm is
                // gated on THIS FRAME's element count, and a dynamic `float64[]`
                // is 3 elements on one frame and 1081 on the next — the walker
                // cannot even tell it from a fixed `float64[100]` (both are a
                // `PrimArray`, which carries no arity marker). Electing an
                // archetype on it would let an idle first frame decide a topic's
                // layout, which is the empty-panel failure mode the whole
                // ladder is built to avoid. Such a field is still REPORTED.
                out.skipped.push(SkippedSeries::Oversized {
                    field: path.clone(),
                    count: pa.count,
                });
                return;
            }
            // A DECLARED numeric array is a plotting field even when this
            // frame's copy is empty (the shape half — the classification must
            // not flip with the array's length).
            out.plottable = true;
            for (i, v) in pa.iter_f64().enumerate() {
                let field_len = path.len();
                push_index(path, i);
                out.samples.push((path.clone(), v));
                path.truncate(field_len);
            }
        }
        FrameValueKind::Nested(inner) => {
            // A timestamp-shaped nested is the frame's own clock (already the
            // plot X axis) — deliberately not telemetry, so it is a SILENT
            // skip, not a report. The text-dump election inherits this skip: a `header`'s
            // `frame_id` is envelope, not the message's text payload, so it must
            // not elect the dual view on every stamped message in ROS.
            if is_time_like_nested(&nv.name, inner) {
                return;
            }
            if budget == 0 {
                report_too_deep(out, path, inner);
                return;
            }
            // A nested STRUCT does not multiply, so it inherits the budget as-is.
            // Its fields render ONE dump row in (`dump_fields` recurses directly).
            push_series(
                out,
                inner,
                path,
                budget - 1,
                series_budget,
                exclude,
                dump_depth + DUMP_ROWS_PER_PLAIN_HOP,
            );
        }
        // A FIXED array of nested-fixed messages (`geometry_msgs/Point[4]` in
        // a custom schema) — the same "a bank of numbers" class as a
        // `PrimArray`, so it expands per index, each element recursing under
        // `field/i/...`.
        //
        // A struct array MULTIPLIES (elements × the element's own numeric
        // fields), so this arm additionally curates. It expands in declaration
        // order into a scratch tail, then re-emits FIELD-major and truncates at a
        // whole-field boundary under [`MAX_STRUCT_ARRAY_SERIES`] — see that
        // constant for why field-first beats element-first on a joint bank.
        FrameValueKind::Array(items) => {
            // Whether the DUMP can show this array's element members
            // AT THIS DEPTH — the election's second condition, ENFORCED here
            // rather than asserted in prose. Two hops shallower than
            // `DUMP_MAX_DEPTH` reads; see `dump_shows_struct_array_members`.
            let dump_shows_members = dump_shows_struct_array_members(dump_depth);
            if is_covariance_field(&nv.name) {
                // The name-and-kind rule again — frame-invariant. It
                // elects only where the dump would render the matrix's own
                // values; deeper, the elements collapse to a `Type {…}`
                // placeholder and there is nothing to point the operator at.
                out.withholds |= dump_shows_members;
                out.skipped.push(SkippedSeries::Covariance {
                    field: path.clone(),
                    count: items.len(),
                });
                return;
            }
            if items.len() > MAX_ARRAY_SERIES {
                // UNLIKE its `PrimArray` twin above, this one CAN latch.
                // A `FrameValueKind::Array` is a FIXED array of fixed-nested
                // elements, so `items.len()` is pinned by the schema and is the
                // same on every frame (the walker surfaces a dynamic array of
                // messages as `NestedArray` instead — see below). Dump reach still
                // gates it: past the member ceiling the dump lists placeholders,
                // not values.
                out.withholds |= dump_shows_members;
                out.skipped.push(SkippedSeries::Oversized {
                    field: path.clone(),
                    count: items.len(),
                });
                return;
            }
            // A ZERO-length fixed array (`Type[0]`, which the
            // rosmsg parser accepts and the walker surfaces as an empty `Array`)
            // declares no series, withholds nothing, and must not reach the
            // division below. The element count is REMOTE-SUPPLIED — it comes out
            // of a robot's schema text — so this is reachable on any desk that
            // attaches to a robot declaring one, not a hand-built corner. The
            // regroup's own `elements == 0` guard sits AFTER the budget
            // division, which makes it unreachable here, so
            // the guard lives at the point that needs it.
            if items.is_empty() {
                return;
            }
            // Whether this array contributed anything is decided BELOW: an array
            // whose element count alone busts the budget plots nothing, so it must
            // not leave `plottable` latched and hand the topic an empty panel (the
            // same rule the `MAX_ARRAY_SERIES` arm above obeys by
            // returning early).
            let plottable_before = out.plottable;
            let first = out.samples.len();
            // The budget DIVIDES on the way down, so a nested array
            // curates itself to what its share of the outer budget allows instead
            // of expanding freely and being cut mid-field by the outer regroup.
            let element_budget = series_budget / items.len();
            // Element 0's own field boundaries — the units the truncation below
            // must land on. Saved/restored so a nested array's own recording
            // cannot clobber an outer one in progress.
            let outer_marks = out.field_marks.take();
            let mut element_marks: Vec<(String, usize)> = Vec::new();
            // Where this array's OWN skip reports begin, so the
            // regroup can retract the ones describing a group it then dropped.
            let skipped_before = out.skipped.len();
            for (i, item) in items.iter().enumerate() {
                let field_len = path.len();
                push_index(path, i);
                if i == 0 {
                    out.field_marks = Some((path.len(), Vec::new()));
                }
                match item {
                    // An element's members render TWO dump rows in: `dump_elements`
                    // emits an `[i]` row, THEN the members under it. One harvest
                    // hop, two dump rows — the exchange rate that made a derived
                    // depth wrong.
                    FrameValueKind::Nested(inner) if budget > 0 => push_series(
                        out,
                        inner,
                        path,
                        budget - 1,
                        element_budget,
                        exclude,
                        dump_depth + DUMP_ROWS_PER_ELEMENT_HOP,
                    ),
                    // An element that cannot be descended into (out of hop budget) is
                    // reported for the same reason a lone over-depth nested is.
                    FrameValueKind::Nested(inner) => report_too_deep(out, path, inner),
                    other => {
                        if let Some(v) = num_f64(other) {
                            out.plottable = true;
                            out.samples.push((path.clone(), v));
                        }
                    }
                }
                if i == 0 {
                    element_marks = out.field_marks.take().map(|(_, m)| m).unwrap_or_default();
                }
                path.truncate(field_len);
            }
            out.field_marks = outer_marks;
            regroup_struct_array_field_major(
                out,
                &StructArrayExpansion {
                    path,
                    elements: items.len(),
                    first,
                    plottable_before,
                    series_budget,
                    element_marks: &element_marks,
                    skipped_before,
                    dump_shows_members,
                },
            );
        }
        // A DYNAMIC array of nested messages / strings. Reported, never
        // expanded — see [`SkippedSeries::ElementArray`] for the two reasons
        // (magnitude and frame-invariance). Without this arm it falls to the numeric
        // catch-all below, which plots nothing AND reports nothing: the field
        // is silently invisible to the ladder. An EMPTY array is not reported (a
        // dynamic array with no elements this frame is not withheld data).
        //
        // For that same reason this arm does NOT latch `withholds` —
        // whether it reports at all is decided by THIS frame's element count, so
        // an idle first frame would elect a different archetype from a busy one.
        FrameValueKind::NestedArray { elements, .. } => {
            if !elements.is_empty() {
                out.skipped.push(SkippedSeries::ElementArray {
                    field: path.clone(),
                    count: elements.len(),
                });
            }
        }
        // A declared `string` field decodes to `Str` on every frame
        // whatever its value, so this latch is frame-INVARIANT — which is what a
        // once-resolved layout requires. Deliberately NOT extended to `Bytes` (a
        // `uint8[]` blob is not text, and the walker cannot tell a non-UTF-8
        // string from one) nor to a `string[]`'s elements (visible only when the
        // array is non-empty). See [`declares_text_fields`].
        FrameValueKind::Str(_) => out.declares_text = true,
        other => {
            if let Some(v) = num_f64(other) {
                out.plottable = true;
                out.samples.push((path.clone(), v));
            }
        }
    }
}

/// Everything [`regroup_struct_array_field_major`] needs to know about the array
/// expansion that just finished — bundled so each field is NAMED at the call site
/// rather than riding an eight-argument positional list where two `usize`
/// watermarks sit next to each other.
#[derive(Clone, Copy)]
struct StructArrayExpansion<'a> {
    /// The ARRAY's own field path (the per-element index segment already
    /// truncated off) — what the report names.
    path: &'a str,
    /// The array's element count.
    elements: usize,
    /// `out.samples` index where this array's expansion began.
    first: usize,
    /// `out.plottable` BEFORE the expansion, restored when the array ends up
    /// contributing nothing.
    plottable_before: bool,
    /// The most series this array may expand into at this point of the tree.
    series_budget: usize,
    /// Element 0's own field boundaries, as `(field name, absolute sample index)`.
    element_marks: &'a [(String, usize)],
    /// `out.skipped` length before the expansion — everything at or after it was
    /// pushed by this array's own elements and may need retracting.
    skipped_before: usize,
    /// Whether the structured dump expands this array's element
    /// MEMBERS at this depth — see `dump_shows_struct_array_members`. The
    /// curation's `withholds` latch is gated on it, because a topic may only be
    /// promised a text window onto fields that window actually renders.
    dump_shows_members: bool,
}

/// Append `/{index}` to the shared path buffer in place (no intermediate
/// `String`). Writing into a `String` is infallible.
fn push_index(path: &mut String, index: usize) {
    use std::fmt::Write as _;
    let _ = write!(path, "/{index}");
}

/// Re-emit the struct-array series sitting at `out.samples[first..]`
/// FIELD-major, truncated at a whole-field boundary under `series_budget`, and
/// report what was withheld.
///
/// The caller has just expanded `elements` array items in declaration order, so
/// the tail is ELEMENT-major: element 0's fields, then element 1's, and so on.
/// Two things happen here, in this order:
///
/// 1. **Regroup.** Element-major order means a budget truncation would keep the
///    first few ELEMENTS complete and drop the rest entirely — on a 20-motor bank
///    that is "five joints in full detail, fifteen joints invisible", which is
///    never what an operator wants. Field-major keeps every element and drops
///    trailing FIELDS instead: `q` for all 20 motors, then `dq` for all 20. The
///    regroup is applied even when nothing is truncated, so the emission order (and
///    therefore the plot legend) is the same shape whether or not the cap bit —
///    one behaviour, not two.
/// 2. **Truncate at a FIELD boundary.** A kept field is kept for EVERY element; a
///    half-populated field (`tau_est` for 4 of 20 motors) would look like a data
///    bug rather than a documented limit.
///
/// **A field is not always one series.** At nesting depth ≥ 2 an
/// element's own field can itself be a struct array — a `LegState[4]` whose
/// element declares `MotorState[3] motors` contributes 36 series through ONE
/// field — so an arithmetic of `fields_kept = budget / elements` (counting
/// SERIES as fields) cuts inside such a field (motor 0 keeping `f5` while motors
/// 1-2 do not) and reports "36 numeric field(s)" for an element that declares
/// one. `element_marks` carries element 0's real field boundaries (recorded by
/// [`push_series`]), so the truncation walks whole GROUPS and the report counts
/// FIELDS. The two are equal at depth 1 — every field is one series — which is
/// why the simpler form looked right.
///
/// The budget also DIVIDES on the way down (see the `series_budget` parameter of
/// [`push_series`]): a nested array is expanded under its share of the outer
/// budget, so by the time this runs the inner curation has usually already made
/// the element fit. The group-exact truncation here is what covers the rest — an
/// element mixing many plain scalars with a nested array can still overflow at a
/// point that would otherwise land inside the array's group.
///
/// `path` carries the ARRAY's own field path (the per-element index segment has
/// already been truncated back off by the caller's loop), which is what the report
/// names. `plottable_before` is the caller's latch snapshot: an array that ends up
/// contributing NOTHING must restore it, or a topic whose only numeric field is an
/// over-budget struct array would classify `Scalars` and be handed a permanently
/// empty plot panel — the empty-panel failure mode.
///
/// A tail whose length is not a whole multiple of `elements` (an element the walk
/// could not descend into, so the elements disagree about their own shape) is left
/// in declaration order: field-major regrouping is only well defined when every
/// element yielded the same series count. Nothing is dropped in that case. The
/// same applies when `element_marks` does not describe the tail it should (a
/// defensive guard — the marks and the samples are produced by the same walk).
fn regroup_struct_array_field_major(out: &mut HarvestOut, x: &StructArrayExpansion<'_>) {
    let StructArrayExpansion {
        path,
        elements,
        first,
        plottable_before,
        series_budget,
        element_marks,
        skipped_before,
        dump_shows_members,
    } = *x;
    let produced = out.samples.len() - first;
    if produced == 0 || elements == 0 {
        return;
    }
    if !produced.is_multiple_of(elements) {
        return;
    }
    let series_per_element = produced / elements;

    // Element 0's field boundaries, as (name, offset, len) within one element.
    // The marks are ABSOLUTE sample indices; element 0 starts at `first`. The
    // NAME rides along so a dropped group's own skip reports can be retracted
    // below — they describe series this level then removed.
    let mut groups: Vec<(&str, usize, usize)> = Vec::new();
    let mut prev = first;
    for (name, mark) in element_marks {
        if *mark < prev || *mark > first + series_per_element {
            // Not a description of this tail — bail to declaration order rather
            // than reorder on a boundary set that cannot be trusted.
            return;
        }
        if *mark > prev {
            groups.push((name.as_str(), prev - first, mark - prev));
        }
        prev = *mark;
    }
    if groups.iter().map(|(_, _, len)| len).sum::<usize>() != series_per_element {
        return;
    }

    // Keep whole fields while they fit the budget, for EVERY element.
    //
    // The COMPARISON form is not what makes this correct — `(kept + len) *
    // elements <= budget` and `kept + len <= budget / elements` are equivalent
    // over integers, and swapping one for the other is output-identical
    // (measured). What is load-bearing is that the loop steps by GROUP: the old
    // body stepped by SERIES, which is why it could stop in the middle of a field
    // that was worth several.
    let mut kept_series = 0usize;
    let mut fields_kept = 0usize;
    for (_, _, len) in &groups {
        if (kept_series + len) * elements > series_budget {
            break;
        }
        kept_series += len;
        fields_kept += 1;
    }

    let tail = out.samples.split_off(first);
    if fields_kept == 0 {
        // Nothing from this field reaches a plot, so it must not have claimed one
        // — the empty-panel failure mode. Reachable when ONE field of the
        // element is worth more series than the whole array's budget allows (a
        // deeply nested bank whose inner curation could not shrink it far enough).
        out.plottable = plottable_before;
    } else {
        let mut tail: Vec<Option<(String, f64)>> = tail.into_iter().map(Some).collect();
        for (_, offset, len) in groups.iter().take(fields_kept) {
            for element_idx in 0..elements {
                let base = element_idx * series_per_element + offset;
                for slot in tail[base..base + len].iter_mut() {
                    if let Some(sample) = slot.take() {
                        out.samples.push(sample);
                    }
                }
            }
        }
    }
    if fields_kept < groups.len() {
        // Second order: a group this level DROPPED may have
        // pushed skip reports of its own while it was being expanded — a nested
        // array reporting "the first N field(s) of EVERY element are plotted".
        // None of those series reached a plot, so those lines are false, and one
        // is emitted PER ELEMENT (twenty of them on a twenty-element bank). They
        // are retracted here; what the operator is told instead is this level's
        // own line, which reports the whole dropped field.
        let dropped_names: Vec<&str> = groups[fields_kept..].iter().map(|(n, _, _)| *n).collect();
        let mut idx = skipped_before.min(out.skipped.len());
        while idx < out.skipped.len() {
            let retract = out.skipped[idx]
                .field_path()
                .is_some_and(|f| path_lies_in_group(f, path, &dropped_names));
            if retract {
                out.skipped.remove(idx);
            } else {
                idx += 1;
            }
        }
        // A FIXED array's element count and its element's field set are
        // both schema facts, so this curation withholds the same fields on every
        // frame — frame-invariant, and therefore allowed to elect the dual view.
        // THE `/lowstate` CASE: a `MotorState[20]` keeps `q` for all twenty
        // motors and drops the rest, and the dump is where the rest becomes
        // readable again (it enumerates the first `DUMP_ARRAY_PREVIEW` elements
        // with every member, so the withheld members really are shown).
        //
        // **The invariance claim is scoped to what the WALKER guarantees**, not
        // asserted of nested values generally: `FrameValueKind::Array`
        // is documented as a FixedArray of nested-FIXED messages, and a fixed
        // nested target has no variable section — so the empty-variable-nested
        // shape (`set_<f>_bytes(&[])`, which decodes to a PRESENT zero-field
        // `Nested` and could make a field-set walk answer differently frame to
        // frame) cannot arise inside these elements. That shape is additionally
        // `PayloadAudit::Frame`-only, so it never appears in an element body at
        // all. It remains a property of `contains_numeric`, which since the R1
        // narrowing feeds ONLY the `TooDeep` REPORT and no longer reaches this
        // predicate.
        //
        // **R2: gated on DUMP REACH.** At a depth past
        // `dump_shows_struct_array_members` the dump enumerates this array but
        // collapses every element to `Type {…}`, so the members this curation
        // withheld are readable in NEITHER pane — electing there would promise a
        // window that renders the very placeholder the element enumeration removed.
        out.withholds |= dump_shows_members;
        out.skipped.push(SkippedSeries::StructArrayFields {
            field: path.to_string(),
            elements,
            fields_per_element: groups.len(),
            series_per_element,
            fields_kept,
            series_kept: kept_series,
        });
    }
}

/// Does `field` name something INSIDE one of `groups` of the
/// struct array at `array_path`?
///
/// A skip report generated while expanding element `i`'s group `g` has the path
/// `{array_path}/{i}/{g}` or something under it, so the test is: strip the array
/// path, strip one all-digits element-index segment, then match a dropped group
/// name at a SEGMENT boundary. The segment-boundary check is what keeps a field
/// named `motors_raw` from being retracted along with `motors`.
///
/// Pure + oracle-tested.
fn path_lies_in_group(field: &str, array_path: &str, groups: &[&str]) -> bool {
    let Some(rest) = field.strip_prefix(array_path) else {
        return false;
    };
    let Some(rest) = rest.strip_prefix('/') else {
        return false;
    };
    let Some((index, rest)) = rest.split_once('/') else {
        return false;
    };
    if index.is_empty() || !index.bytes().all(|b| b.is_ascii_digit()) {
        return false;
    }
    groups
        .iter()
        .any(|name| rest == *name || rest.strip_prefix(*name).is_some_and(|r| r.starts_with('/')))
}

/// The inferred-scalar samples for an UNMAPPED schema — [`harvest_series`]'s
/// sample half (generalizes the old top-level-numerics-plus-twist-shape harvest
/// to every nested struct and numeric array; see that function for the
/// exact rules).
///
/// This is what THIS frame plots. The `Scalars` CLASSIFICATION predicate is
/// [`declares_plottable_series`] instead — a frame whose only numeric array is
/// momentarily empty samples nothing yet is still a plotting topic, and the
/// classification is resolved once per topic.
pub fn inferred_scalars(fv: &FrameValue) -> Vec<(String, f64)> {
    harvest_series(fv).0
}

/// The scalar samples for a `Scalars`-kind frame, mapped OR inferred: a named
/// `Twist`/`TwistStamped`/`Joy` uses [`scalars_from_frame_value`]; any other
/// (inferred) schema uses [`inferred_scalars`]. The `.0` projection of
/// [`scalar_samples_with_skips`], which is what
/// [`dispatch_frame`](crate::sink::dispatch_frame) drives — ONE walk, so a
/// caller that does not need the skip list cannot pick a different mapped-vs-
/// inferred rule by accident.
pub fn scalar_samples(fv: &FrameValue) -> Vec<(String, f64)> {
    scalar_samples_with_skips(fv).0
}

/// [`scalar_samples`] plus the fields the harvest deliberately did not expand —
/// ONE walk yielding both halves, so the sink's once-per-schema
/// "these fields are not plotted" diagnostic costs nothing extra on the hot path.
/// A NAMED extractor (`Twist`/`TwistStamped`/`Joy`) reads exactly the
/// fields it declares, so it skips nothing by construction.
pub fn scalar_samples_with_skips(fv: &FrameValue) -> (Vec<(String, f64)>, Vec<SkippedSeries>) {
    let named = scalars_from_frame_value(fv);
    if named.is_empty() {
        harvest_series(fv)
    } else {
        (named, Vec::new())
    }
}

/// The single string field of a value (exactly one `Str` field, and no
/// numeric fields) — the inferred `TextLog` case. `None` otherwise.
pub fn single_string_of<'f>(fv: &FrameValue<'f>) -> Option<&'f str> {
    if fv.fields.iter().any(|nv| num_f64(&nv.value).is_some()) {
        return None;
    }
    let mut strings = fv.fields.iter().filter_map(|nv| match &nv.value {
        FrameValueKind::Str(s) => Some(*s),
        _ => None,
    });
    match (strings.next(), strings.next()) {
        (Some(s), None) => Some(s),
        _ => None,
    }
}

// ---- image-shape readers -------------------------------------------------

/// The `(width, height)` of an image-shaped value (fixed `u32` fields).
pub fn image_dimensions(fv: &FrameValue) -> Option<(u32, u32)> {
    Some((field_u32(fv, "width")?, field_u32(fv, "height")?))
}

/// The `encoding` string of an image-shaped value.
pub fn image_encoding<'f>(fv: &FrameValue<'f>) -> Option<&'f str> {
    match fv.field("encoding") {
        Some(FrameValueKind::Str(s)) => Some(*s),
        _ => None,
    }
}

/// Bytes per pixel for a [`RawEncoding`] — the row-stride unit
/// [`log_raw_image`] renders under (rgb8/bgr8 are 3-byte interleaved, mono8 is
/// 1 byte), used to validate a producer-declared `step` against the
/// tightly-packed assumption.
fn bytes_per_pixel(enc: RawEncoding) -> u32 {
    match enc {
        RawEncoding::Rgb8 | RawEncoding::Bgr8 => 3,
        RawEncoding::Mono8 => 1,
    }
}

/// The render plan for a RAW image value: `(width, height, encoding)` when it
/// carries dimensions, a decodable encoding, AND non-empty pixel bytes;
/// otherwise `None` (the caller degrades to a field dump).
pub fn raw_image_plan(fv: &FrameValue) -> Option<(u32, u32, RawEncoding)> {
    let (w, h) = image_dimensions(fv)?;
    let enc = classify_raw_encoding(image_encoding(fv)?)?;
    let data = image_data_from_frame_value(fv)?;
    if data.is_empty() {
        return None;
    }
    // A ROS image row may carry trailing padding (`step` = bytes per row). The
    // native render assumes tightly-packed rows, so a declared `step` that
    // disagrees with `width * bytes_per_pixel` would render sheared — degrade
    // to a field dump instead. An ABSENT `step` (a shape-inferred image) keeps
    // the packed assumption.
    if field_u32(fv, "step").is_some_and(|step| step != w * bytes_per_pixel(enc)) {
        return None;
    }
    Some((w, h, enc))
}

// ---- Rerun builders ------------------------------------------------------

/// Log a single rigid transform (translation + optional rotation) at `entity`,
/// so that entity's frame MOVES in the 3D view, with an explicit PARENT FRAME.
///
/// `parent_frame` names the frame this transform is expressed IN; `None` leaves the
/// transform relative to the entity's PATH parent, rerun's default.
///
/// **This — not [`log_coordinate_frame`] — is how a transform PAYLOAD is posed.**
/// In rerun 0.34 the two components answer different questions:
///
/// - `Transform3D:parent_frame` names the frame the logged transform is expressed
///   in, i.e. it RE-PARENTS the frame this entity defines. `re_tf`'s transform
///   forest reads exactly this field when walking the chain; when it is null the
///   parent falls back to the entity's PATH parent
///   (`transform_forest::implicit_transform_parent`).
/// - `CoordinateFrame:frame` relocates the entity's own visualizer DATA into
///   another frame. The transform cache never reads it.
///
/// So a `CoordinateFrame` on an entity whose payload IS a `Transform3D` is worse
/// than inert: the transform keeps composing through the PATH parent while the
/// entity's own geometry jumps to the named frame — which, on the robot root,
/// visibly TEARS the skeleton (the base-link marker moves to the odom pose while
/// every leg link, chained through the path-parented `Transform3D`, stays put).
///
/// **The frame must be written on EVERY row.** rerun 0.34 resolves a `Transform3D` ATOMICALLY PER ROW —
/// `re_tf`'s `query_and_resolve_tree_transform_at_entity` finds the last CHANGED
/// row and reads every component off THAT row id ("we don't have to do latest-at
/// for individual components"), and `get_parent_frame` falls back to the entity's
/// PATH parent whenever the row carries none. So a row without `parent_frame` is
/// RESET to the path parent, not left at the previous row's value: the frame must
/// be written on EVERY row, and the change-triggered dedup `emit_frame_at` uses for
/// `CoordinateFrame` would un-pose every message after the first.
///
/// `None` here therefore means exactly what omitting the component means (the path
/// parent). The caller nonetheless passes the entity's own path-parent frame
/// EXPLICITLY (see `crate::tf::implicit_parent_frame_of`) so that the un-posed case
/// is a visible, asserted value rather than an absence — the withhold arm is a
/// deliberate decision, and a reader (or a test) can see it was taken.
pub fn log_transform3d_in_frame(
    rec: &RecordingStream,
    entity: &str,
    timestamp_ns: u64,
    parts: &PoseParts,
    parent_frame: Option<&str>,
) {
    set_robot_time(rec, timestamp_ns);
    let archetype = match parts.rotation {
        Some(q) => rerun::Transform3D::from_translation_rotation(
            parts.translation,
            rerun::Quaternion::from_xyzw(q),
        ),
        None => rerun::Transform3D::from_translation(parts.translation),
    };
    let archetype = match parent_frame {
        Some(f) => archetype.with_parent_frame(f),
        None => archetype,
    };
    if let Err(e) = rec.log(entity.to_string(), &archetype) {
        tracing::warn!(error = %e, entity, "Rerun: Transform3D log failed");
    }
}

/// Assign `entity` the coordinate FRAME named `frame` (a rerun 0.34
/// [`rerun::CoordinateFrame`]), so its geometry is posed by that frame's place
/// in the transform tree regardless of where the entity sits in the PATH tree.
///
/// This is the mechanism that lets a topic own a unique entity path
/// (`world/<topic>`) and still render posed: `frame` names the implicit frame of
/// the entity `/tf` logs the corresponding transform at, so Rerun composes the
/// chain itself and nothing here does coordinate math.
///
/// **Only for a DATA payload.** This component relocates the entity's own
/// visualizer data; `re_tf`'s transform forest never reads it. An entity whose
/// payload IS a `Transform3D` (Odometry, Pose, Imu, the robot root) must instead
/// be posed through that transform's own `parent_frame` — see
/// [`log_transform3d_in_frame`]. Using this one there is worse than inert: the
/// chain keeps composing through the PATH parent while the entity's geometry
/// moves, which on the skeleton root tears the stick figure in two. Pinned by
/// `coordinate_frame_test::an_odometry_payload_transform_is_posed_by_its_parent_frame_not_a_coordinate_frame`.
///
/// Logged TEMPORAL, at the message timestamp — never `log_static`. Two reasons:
/// a topic whose `frame_id` varies per message genuinely needs temporal, and
/// static-vs-temporal precedence in rerun is not something to bet the pose on
/// (change-triggered temporal is correct under either precedence order, whereas
/// "static first, temporal on change" is silently wrong if static wins).
pub fn log_coordinate_frame(rec: &RecordingStream, entity: &str, timestamp_ns: u64, frame: &str) {
    set_robot_time(rec, timestamp_ns);
    let archetype = rerun::CoordinateFrame::new(frame);
    if let Err(e) = rec.log(entity.to_string(), &archetype) {
        tracing::warn!(error = %e, entity, frame, "Rerun: CoordinateFrame log failed");
    }
}

/// Log an orientation-only transform (a `[x, y, z, w]` quaternion) at `entity`
/// — the IMU case (the sensor reports attitude, not position) — with an explicit
/// PARENT FRAME.
///
/// The rotation payload's half of the [`log_transform3d_in_frame`] contract (an
/// attitude is a transform, so it is posed the same way and NOT by a
/// `CoordinateFrame`).
pub fn log_rotation3d_in_frame(
    rec: &RecordingStream,
    entity: &str,
    timestamp_ns: u64,
    quat: [f32; 4],
    parent_frame: Option<&str>,
) {
    set_robot_time(rec, timestamp_ns);
    let archetype = rerun::Transform3D::from_rotation(rerun::Quaternion::from_xyzw(quat));
    let archetype = match parent_frame {
        Some(f) => archetype.with_parent_frame(f),
        None => archetype,
    };
    if let Err(e) = rec.log(entity.to_string(), &archetype) {
        tracing::warn!(error = %e, entity, "Rerun: Transform3D (rotation) log failed");
    }
}

/// Log a single 3D point under `entity`.
pub fn log_point3d(rec: &RecordingStream, entity: &str, timestamp_ns: u64, point: [f32; 3]) {
    set_robot_time(rec, timestamp_ns);
    let archetype = rerun::Points3D::new(std::iter::once(point));
    if let Err(e) = rec.log(entity.to_string(), &archetype) {
        tracing::warn!(error = %e, entity, "Rerun: Points3D (point) log failed");
    }
}

/// Log a raw (un-encoded) image, decoding a common 8-bit pixel layout into a
/// native [`rerun::Image`] — zero re-encode. `width`/`height` are the pixel
/// resolution; `data` the interleaved pixel bytes.
pub fn log_raw_image(
    rec: &RecordingStream,
    entity: &str,
    timestamp_ns: u64,
    width: u32,
    height: u32,
    encoding: RawEncoding,
    data: &[u8],
) {
    set_robot_time(rec, timestamp_ns);
    let resolution = [width, height];
    let archetype = match encoding {
        RawEncoding::Rgb8 => rerun::Image::from_rgb24(data.to_vec(), resolution),
        RawEncoding::Mono8 => rerun::Image::from_l8(data.to_vec(), resolution),
        RawEncoding::Bgr8 => rerun::Image::from_color_model_and_bytes(
            data.to_vec(),
            resolution,
            rerun::ColorModel::BGR,
            rerun::ChannelDatatype::U8,
        ),
    };
    if let Err(e) = rec.log(entity.to_string(), &archetype) {
        tracing::warn!(error = %e, entity, "Rerun: raw Image log failed");
    }
}

/// Log one text-log line under `entity` (the inferred single-string case) AND
/// mirror the SAME line as a rolling-latest `rerun::TextDocument` under
/// `<entity>/text`.
///
/// Rerun 0.34's blueprint builder API (`rerun::blueprint`) ships NO
/// `TextLogView` — only `TextDocumentView`, which displays ONLY `TextDocument`
/// entities. The deterministic [`crate::blueprint::views_for_archetype`] mapping
/// places a `text_document` view for a [`crate::sink::ArchetypeKind::TextLog`]
/// topic, so the `TextLog` component alone would render that view EMPTY. Mirroring
/// the latest line as a `TextDocument` makes the mapping GENUINELY displayable;
/// the child `<entity>/text` keeps the scrolling `TextLog` and the rolling doc
/// distinct, matching the Imu/Odometry primary-on-entity / derived-on-child
/// convention (and folds a TextLog topic's latest line into the Go2 default's
/// "Status & Field Dumps" `TextDocumentView` at `/world`).
pub fn log_text(rec: &RecordingStream, entity: &str, timestamp_ns: u64, text: &str) {
    set_robot_time(rec, timestamp_ns);
    if let Err(e) = rec.log(entity.to_string(), &rerun::TextLog::new(text.to_string())) {
        tracing::warn!(error = %e, entity, "Rerun: TextLog log failed");
    }
    // The rolling-latest TextDocument mirror (see the fn doc). A child entity so a
    // TextDocumentView rooted at the topic entity (or `/world`) displays it.
    let doc_entity = format!("{entity}/text");
    if let Err(e) = rec.log(
        doc_entity.clone(),
        &rerun::TextDocument::new(text.to_string()),
    ) {
        tracing::warn!(error = %e, entity = %doc_entity, "Rerun: TextDocument (text mirror) log failed");
    }
}

// ---- per-family "build + log" helpers

/// The attitude transform of an IMU frame, posed in `parent_frame` — see
/// [`log_transform3d_in_frame`]. GEOMETRY only, no scalar plots.
///
/// The sink's dispatch calls this and routes the accel/gyro plots
/// through its ONE rate-gated curated-series choke point, so an IMU's telemetry
/// is charged against the same per-topic sample budget as every other plot topic
/// while its attitude keeps rendering on every frame.
pub fn log_imu_geometry_in_frame(
    rec: &RecordingStream,
    entity: &str,
    timestamp_ns: u64,
    fv: &FrameValue,
    parent_frame: Option<&str>,
) {
    if let Some(q) = imu_rotation(fv) {
        log_rotation3d_in_frame(rec, entity, timestamp_ns, q, parent_frame);
    }
}

/// The moving pose transform of an `Odometry` frame, posed in `parent_frame` —
/// GEOMETRY only, no twist plots. See [`log_imu_geometry_in_frame`] for why the
/// split exists (the twist plots are charged against the topic's
/// sample budget; the pose is whole state and renders every frame).
///
/// This is the arm that motivated the posing mechanism: an `Odometry` pose is
/// expressed in the message's own `frame_id` (canonically `odom`), so posing it
/// with a `CoordinateFrame` moved the arrow's DATA while the transform it
/// defines kept composing through the path parent.
pub fn log_odometry_pose_in_frame(
    rec: &RecordingStream,
    entity: &str,
    timestamp_ns: u64,
    fv: &FrameValue,
    parent_frame: Option<&str>,
) {
    if let Some(parts) = pose_transform_parts(fv) {
        log_transform3d_in_frame(rec, entity, timestamp_ns, &parts, parent_frame);
    }
}

/// Pose / PoseStamped / inferred pose-, planar- or rotation-shape → a single
/// rigid transform.
///
/// Draws EXACTLY the primitive [`infer_spatial_primitive`] resolved — the ONE
/// ladder the classifier also reads. It is not "a ladder that mirrors" the
/// classifier's (the earlier state, which had already drifted): a second
/// ladder can disagree, and when it did, the field paths the classifier reported
/// as CONSUMED were not the ones this function drew — `theta` was excluded from
/// the sibling plots and drawn nowhere, while `position/z` was drawn AND plotted.
pub fn log_pose(rec: &RecordingStream, entity: &str, timestamp_ns: u64, fv: &FrameValue) {
    log_pose_in_frame(rec, entity, timestamp_ns, fv, None);
}

/// [`log_pose`] posed in `parent_frame` — see [`log_transform3d_in_frame`].
pub fn log_pose_in_frame(
    rec: &RecordingStream,
    entity: &str,
    timestamp_ns: u64,
    fv: &FrameValue,
    parent_frame: Option<&str>,
) {
    match infer_spatial_primitive(fv) {
        Some((SpatialPrimitive::Transform(parts), _)) => {
            log_transform3d_in_frame(rec, entity, timestamp_ns, &parts, parent_frame)
        }
        Some((SpatialPrimitive::Rotation(q), _)) => {
            log_rotation3d_in_frame(rec, entity, timestamp_ns, q, parent_frame)
        }
        // The ladder resolved a POINT — reachable here only from a NAME-mapped
        // `Transform3D` schema (`geometry_msgs/Pose` and friends) whose frame
        // carries no readable orientation, since an INFERRED point classifies to
        // `Point3D` and renders through [`log_single_point`]. The caller asked for
        // a transform, so the named position becomes a translation-only one (the
        // earlier behaviour for a rotation-less `Pose`) rather than nothing.
        Some((SpatialPrimitive::Point(translation), _)) => log_transform3d_in_frame(
            rec,
            entity,
            timestamp_ns,
            &PoseParts {
                translation,
                rotation: None,
            },
            parent_frame,
        ),
        None => {}
    }
}

/// Log an oriented 3D bounding box at `entity` — one
/// [`rerun::Boxes3D`] instance from the box's centre, full size and (optional)
/// orientation. Delegates to [`log_boxes3d_instances`] so the single-box and
/// N-box (detection-set) paths build the archetype ONE way.
pub fn log_boxes3d(rec: &RecordingStream, entity: &str, timestamp_ns: u64, parts: &BoxParts) {
    log_boxes3d_instances(rec, entity, timestamp_ns, std::slice::from_ref(parts));
}

/// Log N oriented 3D boxes as ONE multi-instance [`rerun::Boxes3D`] (a
/// `vision_msgs/Detection3DArray`'s whole detection set at one entity, replaced
/// wholesale by the next frame under Rerun's latest-at semantics).
///
/// Quaternions are supplied only when at least ONE box carries an orientation; a
/// box without one contributes the IDENTITY quaternion (an axis-aligned box IS
/// identity-rotated, so this adds no information and invents none). Supplying a
/// short quaternion list would silently mis-orient the tail instances.
pub fn log_boxes3d_instances(
    rec: &RecordingStream,
    entity: &str,
    timestamp_ns: u64,
    boxes: &[BoxParts],
) {
    if boxes.is_empty() {
        return;
    }
    set_robot_time(rec, timestamp_ns);
    let mut archetype = rerun::Boxes3D::from_centers_and_sizes(
        boxes.iter().map(|b| b.center),
        boxes.iter().map(|b| b.size),
    );
    if boxes.iter().any(|b| b.rotation.is_some()) {
        archetype = archetype.with_quaternions(
            boxes
                .iter()
                .map(|b| rerun::Quaternion::from_xyzw(b.rotation.unwrap_or([0.0, 0.0, 0.0, 1.0]))),
        );
    }
    if let Err(e) = rec.log(entity.to_string(), &archetype) {
        tracing::warn!(error = %e, entity, "Rerun: Boxes3D log failed");
    }
}

/// The child entity a [`ElementGeometry::Path`]'s VERTICES are logged under.
/// A child rather than the topic entity so the polyline's
/// [`rerun::LineStrips3D`] and the vertex [`rerun::Points3D`] never share a
/// component path — the same primary-on-entity / derived-on-child convention
/// [`log_text`]'s `TextDocument` mirror and the cloud sweep ring use. A
/// `spatial3d` view rooted at the topic entity includes the subtree, so both
/// render.
///
/// **The `-` is load-bearing.** This segment is SYNTHESIZED
/// under a topic's entity, so it shares a namespace with real topics: a robot
/// publishing both `/plan` and `/plan/vertices` would put the second topic's data
/// on the first's synthetic child, one overwriting the other — the collision class
/// the entity scheme exists to remove, and the mechanical `world/<full topic>` rule made it
/// reachable (earlier every topic entity was a flat sibling, so no topic could sit
/// under another). `crate::tf::sanitize_segment` emits only `[A-Za-z0-9_]`, so a
/// segment containing `-` is unreachable from ANY topic name — the same structural
/// argument that reserves `crate::tf::FRAME_ROOT`.
pub const PATH_VERTICES_CHILD: &str = "viz-vertices";

/// Log an ORDERED vertex list as one [`rerun::LineStrips3D`] polyline at `entity`
/// PLUS its vertices as [`rerun::Points3D`] under
/// `<entity>/`[`PATH_VERTICES_CHILD`] — the nav2 `/plan` drawing: a
/// visible line the eye can follow, with the individual waypoints legible on it.
pub fn log_path3d(rec: &RecordingStream, entity: &str, timestamp_ns: u64, vertices: &[[f32; 3]]) {
    if vertices.is_empty() {
        return;
    }
    set_robot_time(rec, timestamp_ns);
    let strip = rerun::LineStrips3D::new([vertices.to_vec()]);
    if let Err(e) = rec.log(entity.to_string(), &strip) {
        tracing::warn!(error = %e, entity, "Rerun: LineStrips3D log failed");
    }
    let vertex_entity = format!("{entity}/{PATH_VERTICES_CHILD}");
    let points = rerun::Points3D::new(vertices.iter().copied());
    if let Err(e) = rec.log(vertex_entity.clone(), &points) {
        tracing::warn!(error = %e, entity = %vertex_entity, "Rerun: Points3D (path vertices) log failed");
    }
}

/// Log an UNORDERED position list as one [`rerun::Points3D`] at `entity`
/// — a `geometry_msgs/PoseArray`'s samples, a `nav_msgs/GridCells`'
/// occupied cells. Deliberately NOT connected by a line: the message declares no
/// element order, so a polyline would invent structure.
pub fn log_element_points3d(
    rec: &RecordingStream,
    entity: &str,
    timestamp_ns: u64,
    points: &[[f32; 3]],
) {
    if points.is_empty() {
        return;
    }
    set_robot_time(rec, timestamp_ns);
    let archetype = rerun::Points3D::new(points.iter().copied());
    if let Err(e) = rec.log(entity.to_string(), &archetype) {
        tracing::warn!(error = %e, entity, "Rerun: Points3D (element array) log failed");
    }
}

/// Draw whatever geometry an element-array scan resolved — the ONE
/// render entry point for [`ElementGeometry`], so the `Path3D` / `PoseArray3D` /
/// `Boxes3D` dispatch arms all draw identically.
pub fn log_element_array(
    rec: &RecordingStream,
    entity: &str,
    timestamp_ns: u64,
    parts: &ElementArrayParts,
) {
    match &parts.geometry {
        ElementGeometry::Path(vertices) => log_path3d(rec, entity, timestamp_ns, vertices),
        ElementGeometry::Points(points) => log_element_points3d(rec, entity, timestamp_ns, points),
        ElementGeometry::Boxes(boxes) => log_boxes3d_instances(rec, entity, timestamp_ns, boxes),
    }
}

/// Element-array frame → its geometry, returning the SCAN so the caller can tell
/// the three outcomes apart: [`ElementArrayScan::Geometry`] drew,
/// [`ElementArrayScan::Empty`] is an idle array (nothing drawn, nothing wrong),
/// [`ElementArrayScan::Absent`] carried no decodable element array (the caller
/// degrades to a field dump and names the reason).
pub fn log_element_array_from_frame(
    rec: &RecordingStream,
    entity: &str,
    timestamp_ns: u64,
    fv: &FrameValue,
    force_ordered: bool,
) -> ElementArrayScan {
    // `force_ordered` carries the caller's CLASSIFIED archetype (Path3D forces a
    // polyline even for unstamped elements — a name-mapped Polygon); see
    // [`scan_element_arrays_for_kind`] and
    // [`crate::sink::ArchetypeKind::forces_ordered_elements`].
    let scan = scan_element_arrays_for_kind(fv, force_ordered);
    if let Some(parts) = scan.parts() {
        log_element_array(rec, entity, timestamp_ns, parts);
    }
    scan
}

/// Box-shaped frame → one [`rerun::Boxes3D`]; `true` when it rendered, `false`
/// when the value carried no extractable box (the caller degrades to a field
/// dump rather than drawing nothing).
pub fn log_boxes3d_from_frame(
    rec: &RecordingStream,
    entity: &str,
    timestamp_ns: u64,
    fv: &FrameValue,
) -> bool {
    match box3d_parts(fv) {
        Some(parts) => {
            log_boxes3d(rec, entity, timestamp_ns, &parts);
            true
        }
        None => false,
    }
}

/// Log a decoded occupancy grid as a native grayscale [`rerun::Image`].
pub fn log_occupancy_grid(
    rec: &RecordingStream,
    entity: &str,
    timestamp_ns: u64,
    grid: &OccupancyImage,
) {
    set_robot_time(rec, timestamp_ns);
    let archetype = rerun::Image::from_l8(grid.gray.clone(), [grid.width, grid.height]);
    if let Err(e) = rec.log(entity.to_string(), &archetype) {
        tracing::warn!(error = %e, entity, "Rerun: OccupancyGrid Image log failed");
    }
}

/// Occupancy-grid frame → a grayscale [`rerun::Image`]; `true` when it rendered,
/// `false` when the grid was undecodable (missing `info` dimensions or a short
/// cell buffer) so the caller can degrade to an inspectable field dump.
pub fn log_occupancy_from_frame(
    rec: &RecordingStream,
    entity: &str,
    timestamp_ns: u64,
    fv: &FrameValue,
) -> bool {
    match occupancy_image(fv) {
        Some(grid) => {
            log_occupancy_grid(rec, entity, timestamp_ns, &grid);
            true
        }
        None => false,
    }
}

/// PointStamped → a single 3D point.
pub fn log_single_point(rec: &RecordingStream, entity: &str, timestamp_ns: u64, fv: &FrameValue) {
    if let Some(p) = single_point_of(fv) {
        log_point3d(rec, entity, timestamp_ns, p);
    }
}

/// LaserScan → a projected 2D scan ring as Points3D (nothing when empty).
pub fn log_laserscan(rec: &RecordingStream, entity: &str, timestamp_ns: u64, fv: &FrameValue) {
    let points = laserscan_points(fv);
    if points.is_empty() {
        return;
    }
    set_robot_time(rec, timestamp_ns);
    let archetype = rerun::Points3D::new(points);
    if let Err(e) = rec.log(entity.to_string(), &archetype) {
        tracing::warn!(error = %e, entity, "Rerun: Points3D (laser scan) log failed");
    }
}

/// Inferred single-string → one TextLog line (nothing when absent).
pub fn log_single_text(rec: &RecordingStream, entity: &str, timestamp_ns: u64, fv: &FrameValue) {
    if let Some(s) = single_string_of(fv) {
        log_text(rec, entity, timestamp_ns, s);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- hand-built FrameValue helpers (oracle inputs, never a self-compare)

    fn f64v(name: &str, v: f64) -> cerulion_core::codegen::NamedValue<'static> {
        cerulion_core::codegen::NamedValue {
            name: name.to_string(),
            value: FrameValueKind::F64(v),
        }
    }

    fn vec3(x: f64, y: f64, z: f64) -> FrameValue<'static> {
        FrameValue {
            schema_name: "geometry_msgs/Vector3".to_string(),
            fields: vec![f64v("x", x), f64v("y", y), f64v("z", z)],
        }
    }

    fn quat(x: f64, y: f64, z: f64, w: f64) -> FrameValue<'static> {
        FrameValue {
            schema_name: "geometry_msgs/Quaternion".to_string(),
            fields: vec![f64v("x", x), f64v("y", y), f64v("z", z), f64v("w", w)],
        }
    }

    fn nested_named(
        name: &str,
        inner: FrameValue<'static>,
    ) -> cerulion_core::codegen::NamedValue<'static> {
        cerulion_core::codegen::NamedValue {
            name: name.to_string(),
            value: FrameValueKind::Nested(Box::new(inner)),
        }
    }

    fn pose_fv(pos: [f64; 3], rot: [f64; 4]) -> FrameValue<'static> {
        FrameValue {
            schema_name: "geometry_msgs/Pose".to_string(),
            fields: vec![
                nested_named("position", vec3(pos[0], pos[1], pos[2])),
                nested_named("orientation", quat(rot[0], rot[1], rot[2], rot[3])),
            ],
        }
    }

    #[test]
    fn wire_clock_is_wall_boundary_oracle() {
        // Hand oracle for the threshold: 2001-01-01T00:00:00Z in ns (re-derived
        // here, never read from the production const — not a self-compare).
        const THRESHOLD_NS: u64 = 978_307_200_000_000_000;
        // Below the threshold → uptime/CLOCK_MONOTONIC scale.
        assert!(!wire_clock_is_wall(0));
        assert!(!wire_clock_is_wall(1));
        // ~88 minutes since boot — a realistic robot uptime stamp.
        assert!(!wire_clock_is_wall(5_300 * 1_000_000_000));
        assert!(!wire_clock_is_wall(THRESHOLD_NS - 1));
        // At/after the threshold → wall clock.
        assert!(wire_clock_is_wall(THRESHOLD_NS));
        // A modern wall stamp (~2026-06).
        assert!(wire_clock_is_wall(1_783_000_000 * 1_000_000_000));
    }

    #[test]
    fn set_robot_time_latch_smoke() {
        // The clock-class latch is PROCESS-GLOBAL (OnceLock), so this is the
        // ONE test in this binary that exercises it (the classifier's boundary
        // matrix is the pure oracle above; asserting opposite latch outcomes
        // in one process is impossible by design). No other unit test in this
        // lib calls set_robot_time. The first stamp is uptime-scale → the
        // duration type latches; a later WALL-scale stamp (which the pure
        // classifier maps to the opposite class — the decide-once point) must
        // ride the SAME latched type: both frames record, no mixed-type
        // timeline, no panic.
        let (rec, storage) = rerun::RecordingStreamBuilder::new("go2_test")
            .recording_id("go2_archetype_unit_latch_smoke")
            .memory()
            .expect("memory sink");
        let uptime_ns = 5_300 * 1_000_000_000;
        let wall_ns = 1_783_000_000 * 1_000_000_000;
        assert_ne!(
            wire_clock_is_wall(uptime_ns),
            wire_clock_is_wall(wall_ns),
            "the two stamps must straddle the threshold for this smoke to bite"
        );
        let base = storage.num_msgs();
        // Distinct scalar names → distinct entity paths, so the two logs stay
        // two chunks (same-entity logs would compact into one).
        log_scalar(&rec, "world/latch", "a", uptime_ns, 1.0);
        log_scalar(&rec, "world/latch", "b", wall_ns, 2.0);
        rec.flush_blocking().expect("flush");
        assert_eq!(
            storage.num_msgs() - base,
            2,
            "both stamps record on the one latched time type"
        );
    }

    #[test]
    fn classify_raw_encoding_oracle() {
        assert_eq!(classify_raw_encoding("rgb8"), Some(RawEncoding::Rgb8));
        assert_eq!(classify_raw_encoding("bgr8"), Some(RawEncoding::Bgr8));
        assert_eq!(classify_raw_encoding("mono8"), Some(RawEncoding::Mono8));
        assert_eq!(classify_raw_encoding("8UC1"), Some(RawEncoding::Mono8));
        // Unhandled encodings degrade (no mis-render).
        assert_eq!(classify_raw_encoding("16UC1"), None);
        assert_eq!(classify_raw_encoding("yuv422"), None);
        assert_eq!(classify_raw_encoding(""), None);
    }

    #[test]
    fn raw_image_plan_stride_guard_oracle() {
        use cerulion_core::codegen::NamedValue;
        let u32f = |name: &str, v: u32| NamedValue {
            name: name.to_string(),
            value: FrameValueKind::U32(v),
        };
        // A 2×2 rgb8 image: 3 bytes/pixel, so a tightly-packed row is
        // width*bpp = 6 bytes, 12 pixel bytes total.
        let img = |step: Option<u32>| {
            let mut fields = vec![
                u32f("width", 2),
                u32f("height", 2),
                NamedValue {
                    name: "encoding".to_string(),
                    value: FrameValueKind::Str("rgb8"),
                },
                NamedValue {
                    name: "data".to_string(),
                    value: FrameValueKind::Bytes(&[0u8; 12]),
                },
            ];
            if let Some(s) = step {
                fields.push(u32f("step", s));
            }
            FrameValue {
                schema_name: "sensor_msgs/Image".to_string(),
                fields,
            }
        };
        // (a) step == width*bpp → tightly packed → a render plan.
        assert_eq!(
            raw_image_plan(&img(Some(6))),
            Some((2, 2, RawEncoding::Rgb8))
        );
        // (b) step = width*bpp + padding → None (degrade; a padded row rendered
        // as packed would shear).
        assert_eq!(raw_image_plan(&img(Some(8))), None);
        // (c) no `step` field (a shape-inferred image) → keep the packed
        // assumption (back-compat).
        assert_eq!(raw_image_plan(&img(None)), Some((2, 2, RawEncoding::Rgb8)));
    }

    #[test]
    fn pose_transform_parts_top_level_and_nested() {
        // Top-level {position, orientation} (a bare Pose).
        let parts = pose_transform_parts(&pose_fv([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0]))
            .expect("pose parts");
        assert_eq!(parts.translation, [1.0, 2.0, 3.0]);
        assert_eq!(parts.rotation, Some([0.0, 0.0, 0.0, 1.0]));

        // PoseStamped shape: nested `pose` one hop down.
        let stamped = FrameValue {
            schema_name: "geometry_msgs/PoseStamped".to_string(),
            fields: vec![nested_named(
                "pose",
                pose_fv([4.0, 5.0, 6.0], [1.0, 0.0, 0.0, 0.0]),
            )],
        };
        let parts = pose_transform_parts(&stamped).expect("stamped pose parts");
        assert_eq!(parts.translation, [4.0, 5.0, 6.0]);
        assert_eq!(parts.rotation, Some([1.0, 0.0, 0.0, 0.0]));

        // Odometry shape: pose.pose two hops down.
        let with_cov = FrameValue {
            schema_name: "geometry_msgs/PoseWithCovariance".to_string(),
            fields: vec![nested_named(
                "pose",
                pose_fv([7.0, 8.0, 9.0], [0.0, 1.0, 0.0, 0.0]),
            )],
        };
        let odom = FrameValue {
            schema_name: "nav_msgs/Odometry".to_string(),
            fields: vec![nested_named("pose", with_cov)],
        };
        let parts = pose_transform_parts(&odom).expect("odom pose parts");
        assert_eq!(parts.translation, [7.0, 8.0, 9.0]);
        assert_eq!(parts.rotation, Some([0.0, 1.0, 0.0, 0.0]));

        // No reachable position → None.
        assert_eq!(pose_transform_parts(&vec3(1.0, 2.0, 3.0)), None);
    }

    #[test]
    fn pose_transform_parts_from_transform_shape() {
        // A bare geometry_msgs/Transform: {translation, rotation} (the TF field
        // names for the same rigid motion as {position, orientation}).
        let tf = FrameValue {
            schema_name: "geometry_msgs/Transform".to_string(),
            fields: vec![
                nested_named("translation", vec3(1.0, 2.0, 3.0)),
                nested_named("rotation", quat(0.0, 0.0, 0.0, 1.0)),
            ],
        };
        let parts = pose_transform_parts(&tf).expect("transform parts");
        assert_eq!(parts.translation, [1.0, 2.0, 3.0]);
        assert_eq!(parts.rotation, Some([0.0, 0.0, 0.0, 1.0]));

        // TransformStamped shape: nested `transform` one hop down.
        let stamped = FrameValue {
            schema_name: "geometry_msgs/TransformStamped".to_string(),
            fields: vec![nested_named(
                "transform",
                FrameValue {
                    schema_name: "geometry_msgs/Transform".to_string(),
                    fields: vec![
                        nested_named("translation", vec3(4.0, 5.0, 6.0)),
                        nested_named("rotation", quat(1.0, 0.0, 0.0, 0.0)),
                    ],
                },
            )],
        };
        let parts = pose_transform_parts(&stamped).expect("stamped transform parts");
        assert_eq!(parts.translation, [4.0, 5.0, 6.0]);
        assert_eq!(parts.rotation, Some([1.0, 0.0, 0.0, 0.0]));
    }

    #[test]
    fn imu_extractors_oracle() {
        let imu = FrameValue {
            schema_name: "sensor_msgs/Imu".to_string(),
            fields: vec![
                nested_named("orientation", quat(0.1, 0.2, 0.3, 0.4)),
                nested_named("angular_velocity", vec3(0.5, 0.6, 0.7)),
                nested_named("linear_acceleration", vec3(1.0, 2.0, 3.0)),
            ],
        };
        assert_eq!(imu_rotation(&imu), Some([0.1, 0.2, 0.3, 0.4]));
        assert_eq!(
            imu_scalars(&imu),
            vec![
                ("linear_acceleration/x".to_string(), 1.0),
                ("linear_acceleration/y".to_string(), 2.0),
                ("linear_acceleration/z".to_string(), 3.0),
                ("angular_velocity/x".to_string(), 0.5),
                ("angular_velocity/y".to_string(), 0.6),
                ("angular_velocity/z".to_string(), 0.7),
            ]
        );
    }

    #[test]
    fn odometry_twist_scalars_oracle() {
        let twist = FrameValue {
            schema_name: "geometry_msgs/Twist".to_string(),
            fields: vec![
                nested_named("linear", vec3(1.0, 0.0, 0.0)),
                nested_named("angular", vec3(0.0, 0.0, 0.5)),
            ],
        };
        let twist_cov = FrameValue {
            schema_name: "geometry_msgs/TwistWithCovariance".to_string(),
            fields: vec![nested_named("twist", twist)],
        };
        let odom = FrameValue {
            schema_name: "nav_msgs/Odometry".to_string(),
            fields: vec![nested_named("twist", twist_cov)],
        };
        assert_eq!(
            odometry_twist_scalars(&odom),
            vec![
                ("linear/x".to_string(), 1.0),
                ("linear/y".to_string(), 0.0),
                ("linear/z".to_string(), 0.0),
                ("angular/x".to_string(), 0.0),
                ("angular/y".to_string(), 0.0),
                ("angular/z".to_string(), 0.5),
            ]
        );
    }

    #[test]
    fn single_point_of_nested_and_top_level() {
        // PointStamped: nested `point`.
        let stamped = FrameValue {
            schema_name: "geometry_msgs/PointStamped".to_string(),
            fields: vec![nested_named("point", vec3(1.5, -2.5, 3.5))],
        };
        assert_eq!(single_point_of(&stamped), Some([1.5, -2.5, 3.5]));
        // Bare Vector3: top-level xyz.
        assert_eq!(single_point_of(&vec3(9.0, 8.0, 7.0)), Some([9.0, 8.0, 7.0]));
    }

    fn f32_prim(vals: &[f32]) -> Vec<u8> {
        vals.iter().flat_map(|v| v.to_le_bytes()).collect()
    }

    #[test]
    fn laserscan_projects_and_skips_out_of_range() {
        // angle_min 0, increment π/2 → rays along +x, +y, -x. ranges include a
        // non-finite and an out-of-range value that must be SKIPPED.
        let bytes = f32_prim(&[1.0, 2.0, f32::INFINITY, 100.0]);
        let scan = FrameValue {
            schema_name: "sensor_msgs/LaserScan".to_string(),
            fields: vec![
                cerulion_core::codegen::NamedValue {
                    name: "angle_min".to_string(),
                    value: FrameValueKind::F32(0.0),
                },
                cerulion_core::codegen::NamedValue {
                    name: "angle_increment".to_string(),
                    value: FrameValueKind::F32(std::f32::consts::FRAC_PI_2),
                },
                cerulion_core::codegen::NamedValue {
                    name: "range_min".to_string(),
                    value: FrameValueKind::F32(0.0),
                },
                cerulion_core::codegen::NamedValue {
                    name: "range_max".to_string(),
                    value: FrameValueKind::F32(10.0),
                },
                cerulion_core::codegen::NamedValue {
                    name: "ranges".to_string(),
                    value: FrameValueKind::PrimArray(PrimArray {
                        elem: cerulion_core::codegen::PrimType::F32,
                        bytes: bytes.as_slice(),
                        count: 4,
                    }),
                },
            ],
        };
        let pts = laserscan_points(&scan);
        // ray 0 (θ=0, r=1) → (1,0,0); ray 1 (θ=π/2, r=2) → (0,2,0); ray 2 is
        // INFINITY → skipped; ray 3 (r=100 > range_max) → skipped.
        assert_eq!(pts.len(), 2);
        assert!((pts[0][0] - 1.0).abs() < 1e-6 && pts[0][1].abs() < 1e-6);
        assert!(pts[1][0].abs() < 1e-6 && (pts[1][1] - 2.0).abs() < 1e-6);
    }

    #[test]
    fn sportmode_scalars_oracle() {
        let vel = f32_prim(&[0.5, 0.0, -0.3]);
        let pos = f32_prim(&[1.0, 2.0, 0.3]);
        let sms = FrameValue {
            schema_name: "unitree_go/SportModeState".to_string(),
            fields: vec![
                cerulion_core::codegen::NamedValue {
                    name: "velocity".to_string(),
                    value: FrameValueKind::PrimArray(PrimArray {
                        elem: cerulion_core::codegen::PrimType::F32,
                        bytes: vel.as_slice(),
                        count: 3,
                    }),
                },
                cerulion_core::codegen::NamedValue {
                    name: "position".to_string(),
                    value: FrameValueKind::PrimArray(PrimArray {
                        elem: cerulion_core::codegen::PrimType::F32,
                        bytes: pos.as_slice(),
                        count: 3,
                    }),
                },
                cerulion_core::codegen::NamedValue {
                    name: "yaw_speed".to_string(),
                    value: FrameValueKind::F32(0.1),
                },
                cerulion_core::codegen::NamedValue {
                    name: "body_height".to_string(),
                    value: FrameValueKind::F32(0.32),
                },
            ],
        };
        let got = sportmode_scalars(&sms);
        // f32 → f64 widening is exact for these hand values' bit patterns only
        // approximately, so compare names + rounded values.
        let names: Vec<&str> = got.iter().map(|(n, _)| n.as_str()).collect();
        assert_eq!(
            names,
            vec![
                "velocity/x",
                "velocity/y",
                "velocity/z",
                "position/x",
                "position/y",
                "position/z",
                "yaw_speed",
                "body_height",
            ]
        );
        assert!((got[0].1 - 0.5).abs() < 1e-6);
        assert!((got[7].1 - 0.32).abs() < 1e-6);
        // foot_raise_height absent → not fabricated.
        assert!(!got.iter().any(|(n, _)| n == "foot_raise_height"));
    }

    #[test]
    fn generic_and_inferred_scalars_oracle() {
        // builtin_interfaces/Time shape: two top-level ints → a numeric bag,
        // under their BARE names (a root field carries no path prefix).
        let time = FrameValue {
            schema_name: "builtin_interfaces/Time".to_string(),
            fields: vec![
                cerulion_core::codegen::NamedValue {
                    name: "sec".to_string(),
                    value: FrameValueKind::I32(12),
                },
                cerulion_core::codegen::NamedValue {
                    name: "nanosec".to_string(),
                    value: FrameValueKind::U32(500),
                },
            ],
        };
        assert_eq!(
            inferred_scalars(&time),
            vec![("sec".to_string(), 12.0), ("nanosec".to_string(), 500.0)]
        );
        // inferred_scalars on a twist-shape yields the six velocity components,
        // NOT the (empty) top-level bag.
        let twist = FrameValue {
            schema_name: "unknown/Cmd".to_string(),
            fields: vec![
                nested_named("linear", vec3(1.0, 0.0, 0.0)),
                nested_named("angular", vec3(0.0, 0.0, 2.0)),
            ],
        };
        assert_eq!(inferred_scalars(&twist).len(), 6);
    }

    // ---- The widened numeric harvest ------------------------------

    fn u32f(name: &str, v: u32) -> cerulion_core::codegen::NamedValue<'static> {
        cerulion_core::codegen::NamedValue {
            name: name.to_string(),
            value: FrameValueKind::U32(v),
        }
    }

    fn prim_named(
        name: &str,
        bytes: &'static [u8],
        count: usize,
    ) -> cerulion_core::codegen::NamedValue<'static> {
        cerulion_core::codegen::NamedValue {
            name: name.to_string(),
            value: FrameValueKind::PrimArray(PrimArray {
                elem: PrimType::F64,
                bytes,
                count,
            }),
        }
    }

    /// `[1.0, 2.0, 3.0]` as LE f64 bytes — a leaked slice so the fixture can be
    /// `'static` (test-only, three allocations for the whole binary).
    fn f64_bytes(vals: &[f64]) -> &'static [u8] {
        let v: Vec<u8> = vals.iter().flat_map(|x| x.to_le_bytes()).collect();
        Box::leak(v.into_boxed_slice())
    }

    #[test]
    fn harvest_series_expands_nested_structs_by_path() {
        // geometry_msgs/WrenchStamped shape: {header, wrench{force, torque}}.
        // Earlier this harvested NOTHING (the twist-shape special case only
        // knew `linear`/`angular`) and the topic fell to a text dump.
        let wrench = FrameValue {
            schema_name: "geometry_msgs/Wrench".to_string(),
            fields: vec![
                nested_named("force", vec3(1.0, 2.0, 3.0)),
                nested_named("torque", vec3(4.0, 5.0, 6.0)),
            ],
        };
        let stamped = FrameValue {
            schema_name: "geometry_msgs/WrenchStamped".to_string(),
            fields: vec![
                nested_named(
                    "header",
                    FrameValue {
                        schema_name: "std_msgs/Header".to_string(),
                        fields: vec![u32f("seq", 7)],
                    },
                ),
                nested_named("wrench", wrench),
            ],
        };
        // Hand oracle: six path-qualified series; the `header` nested is SKIPPED
        // (a timestamp is the plot's X axis, not a series).
        assert_eq!(
            harvest_series(&stamped).0,
            vec![
                ("wrench/force/x".to_string(), 1.0),
                ("wrench/force/y".to_string(), 2.0),
                ("wrench/force/z".to_string(), 3.0),
                ("wrench/torque/x".to_string(), 4.0),
                ("wrench/torque/y".to_string(), 5.0),
                ("wrench/torque/z".to_string(), 6.0),
            ]
        );
    }

    #[test]
    fn harvest_series_reaches_three_nested_hops() {
        // TwistWithCovarianceStamped shape: twist.twist.linear.x is THREE nested
        // hops from the root — the deepest reach MAX_NESTED_SERIES_DEPTH must cover.
        let twist = FrameValue {
            schema_name: "geometry_msgs/Twist".to_string(),
            fields: vec![nested_named("linear", vec3(9.0, 0.0, 0.0))],
        };
        let with_cov = FrameValue {
            schema_name: "geometry_msgs/TwistWithCovariance".to_string(),
            fields: vec![nested_named("twist", twist)],
        };
        let stamped = FrameValue {
            schema_name: "geometry_msgs/TwistWithCovarianceStamped".to_string(),
            fields: vec![nested_named("twist", with_cov)],
        };
        assert_eq!(
            harvest_series(&stamped).0,
            vec![
                ("twist/twist/linear/x".to_string(), 9.0),
                ("twist/twist/linear/y".to_string(), 0.0),
                ("twist/twist/linear/z".to_string(), 0.0),
            ]
        );
    }

    #[test]
    fn a_nested_message_past_the_hop_limit_is_reported_not_dropped_silently() {
        // An over-depth nested must not be the ONE numeric
        // skip that is SILENT while its array / covariance siblings are loud —
        // or a deep telemetry struct looks like a bug instead of a documented
        // limit. `a/b/c` spends the whole MAX_NESTED_SERIES_DEPTH budget, so
        // `a/b/c/deep` is the first un-descendable hop.
        let leaf = FrameValue {
            schema_name: "acme/Leaf".to_string(),
            fields: vec![f64v("value", 7.0)],
        };
        let c = FrameValue {
            schema_name: "acme/C".to_string(),
            fields: vec![f64v("shallow", 1.0), nested_named("deep", leaf)],
        };
        let b = FrameValue {
            schema_name: "acme/B".to_string(),
            fields: vec![nested_named("c", c)],
        };
        let a = FrameValue {
            schema_name: "acme/B".to_string(),
            fields: vec![nested_named("b", b)],
        };
        let root = FrameValue {
            schema_name: "acme/Deep".to_string(),
            fields: vec![nested_named("a", a)],
        };
        let (samples, skipped) = harvest_series(&root);
        // Everything WITHIN the budget still plots (anti-tautology: the walk
        // really did spend all three hops before giving up).
        assert_eq!(samples, vec![("a/b/c/shallow".to_string(), 1.0)]);
        // And the un-descended node is NAMED.
        assert_eq!(
            skipped,
            vec![SkippedSeries::TooDeep {
                field: "a/b/c/deep".to_string(),
            }]
        );
        assert_eq!(
            skipped[0].to_string(),
            "a/b/c/deep (nested deeper than the 3-hop harvest limit)"
        );
    }

    #[test]
    fn an_over_depth_nested_with_no_numbers_is_not_reported() {
        // The noise guard (the NEGATIVE arm): a deep block of strings / bools is
        // not lost telemetry, so reporting it would be pure noise in the operator
        // diagnostic. Same over-depth shape as above, non-numeric leaf.
        let leaf = FrameValue {
            schema_name: "acme/Leaf".to_string(),
            fields: vec![
                cerulion_core::codegen::NamedValue {
                    name: "label".to_string(),
                    value: FrameValueKind::Str("hi"),
                },
                cerulion_core::codegen::NamedValue {
                    name: "flag".to_string(),
                    value: FrameValueKind::Bool(true),
                },
            ],
        };
        let c = FrameValue {
            schema_name: "acme/C".to_string(),
            fields: vec![nested_named("deep", leaf)],
        };
        let b = FrameValue {
            schema_name: "acme/B".to_string(),
            fields: vec![nested_named("c", c)],
        };
        let a = FrameValue {
            schema_name: "acme/A".to_string(),
            fields: vec![nested_named("b", b)],
        };
        let root = FrameValue {
            schema_name: "acme/Deep".to_string(),
            fields: vec![nested_named("a", a)],
        };
        let (samples, skipped) = harvest_series(&root);
        assert!(samples.is_empty(), "got {samples:?}");
        assert!(
            skipped.is_empty(),
            "a string/bool block is not lost telemetry: {skipped:?}"
        );
    }

    /// The noise guard must apply the SAME time-like exclusion the harvest
    /// itself applies. A `header` / `stamp` is the frame's own clock — the
    /// harvest skips it SILENTLY at every depth — so a deep block carrying only a
    /// timestamp is NOT lost telemetry, and naming it in the operator diagnostic
    /// points at a value the harvest would have dropped anyway. (Without the
    /// exclusion `contains_numeric` counts the header's `sec` / `nanosec` and
    /// reports the block.) The POSITIVE control shares the body: the same block with ONE real
    /// number beside the header IS still reported.
    #[test]
    fn an_over_depth_nested_carrying_only_a_timestamp_is_not_reported() {
        let stamp = || FrameValue {
            schema_name: "std_msgs/Header".to_string(),
            fields: vec![f64v("sec", 12.0), f64v("nanosec", 34.0)],
        };
        // `a/b/c/deep` — one hop past the 3-hop harvest limit, so `deep` is the
        // un-descended block the guard decides about.
        let deep = |leaf_fields: Vec<cerulion_core::codegen::NamedValue<'static>>| {
            let leaf = FrameValue {
                schema_name: "acme/Leaf".to_string(),
                fields: leaf_fields,
            };
            let c = FrameValue {
                schema_name: "acme/C".to_string(),
                fields: vec![nested_named("deep", leaf)],
            };
            let b = FrameValue {
                schema_name: "acme/B".to_string(),
                fields: vec![nested_named("c", c)],
            };
            let a = FrameValue {
                schema_name: "acme/A".to_string(),
                fields: vec![nested_named("b", b)],
            };
            FrameValue {
                schema_name: "acme/Deep".to_string(),
                fields: vec![nested_named("a", a)],
            }
        };
        // NEGATIVE: the un-descended block holds only a timestamp.
        let (samples, skipped) = harvest_series(&deep(vec![nested_named("header", stamp())]));
        assert!(samples.is_empty(), "got {samples:?}");
        assert!(
            skipped.is_empty(),
            "a clock is not lost telemetry — the harvest drops it silently anyway: {skipped:?}"
        );
        // POSITIVE control: one REAL number beside the same header ⇒ reported.
        let (_, skipped) = harvest_series(&deep(vec![
            nested_named("header", stamp()),
            f64v("voltage", 12.4),
        ]));
        assert_eq!(
            skipped,
            vec![SkippedSeries::TooDeep {
                field: "a/b/c/deep".to_string()
            }],
            "real telemetry under the hop limit is still named"
        );
    }

    /// A nested `point` SHADOWS a nested `position` — the FIELD decides, then its
    /// shape is read. The path-carrying refactor turned the original
    /// `nested_field("point").or_else(…"position").and_then(xyz_of)` into a
    /// try-each-in-turn loop, so a `point` that is not xyz-shaped silently fell
    /// through and rendered a sibling `position` as if it were the point — a
    /// behaviour change inside a refactor advertised as extraction-only.
    #[test]
    fn a_malformed_point_does_not_fall_through_to_a_sibling_position() {
        let both = FrameValue {
            schema_name: "acme/Detection".to_string(),
            fields: vec![
                // A `point` that is NOT xyz-shaped (a 2D pixel coordinate).
                nested_named(
                    "point",
                    FrameValue {
                        schema_name: "acme/Pixel".to_string(),
                        fields: vec![f64v("u", 10.0), f64v("v", 20.0)],
                    },
                ),
                nested_named("position", vec3(1.0, 2.0, 3.0)),
            ],
        };
        assert_eq!(
            nested_point_of(&both),
            None,
            "the value HAS a `point`; a sibling `position` is not a substitute"
        );
        // Anti-tautology: with no `point` field at all, `position` IS read.
        let position_only = FrameValue {
            schema_name: "acme/Waypoint".to_string(),
            fields: vec![nested_named("position", vec3(1.0, 2.0, 3.0))],
        };
        assert_eq!(nested_point_of(&position_only), Some([1.0, 2.0, 3.0]));
        // …and a well-formed `point` wins over a sibling `position`.
        let point_wins = FrameValue {
            schema_name: "acme/Detection".to_string(),
            fields: vec![
                nested_named("point", vec3(7.0, 8.0, 9.0)),
                nested_named("position", vec3(1.0, 2.0, 3.0)),
            ],
        };
        assert_eq!(nested_point_of(&point_wins), Some([7.0, 8.0, 9.0]));
    }

    #[test]
    fn harvest_series_excluding_prunes_only_the_named_paths() {
        // The primitive behind `spatial_sibling_series`: an
        // excluded path prunes exactly that node and its subtree — a SIBLING
        // sharing the parent survives (excluding whole wrappers was the
        // one-level-down form of the silent-drop bug).
        let inner = FrameValue {
            schema_name: "acme/Inner".to_string(),
            fields: vec![
                nested_named("position", vec3(1.0, 2.0, 3.0)),
                f64v("confidence", 0.5),
            ],
        };
        let root = FrameValue {
            schema_name: "acme/Outer".to_string(),
            fields: vec![nested_named("pose", inner), f64v("battery_v", 12.0)],
        };
        // Nothing excluded ⇒ identical to the plain harvest.
        assert_eq!(
            harvest_series_excluding(&root, &[]).0,
            harvest_series(&root).0
        );
        // Excluding the CONSUMED path leaves both siblings.
        assert_eq!(
            harvest_series_excluding(&root, &["pose/position".to_string()]).0,
            vec![
                ("pose/confidence".to_string(), 0.5),
                ("battery_v".to_string(), 12.0),
            ]
        );
        // An exclusion that matches nothing changes nothing (never a silent
        // wholesale prune on a typo'd path).
        assert_eq!(
            harvest_series_excluding(&root, &["position".to_string()]).0,
            harvest_series(&root).0
        );
    }

    // ---- Struct-array curation ------------------------------------

    /// A fixed array of `elements` structs, each declaring `fields` numeric
    /// members named `f0..f{fields-1}`. Element `i`'s member `j` carries the value
    /// `i * 100 + j`, so every sample's VALUE identifies the exact (element, field)
    /// it came from — the oracle can therefore check the regrouped ORDER without
    /// reading any path, and a path/value mismatch cannot pass.
    fn struct_array(field: &str, elements: usize, fields: usize) -> FrameValue<'static> {
        let items: Vec<FrameValueKind<'static>> = (0..elements)
            .map(|i| {
                FrameValueKind::Nested(Box::new(FrameValue {
                    schema_name: "acme/Element".to_string(),
                    fields: (0..fields)
                        .map(|j| f64v(&format!("f{j}"), (i * 100 + j) as f64))
                        .collect(),
                }))
            })
            .collect();
        FrameValue {
            schema_name: "acme/Bank".to_string(),
            fields: vec![cerulion_core::codegen::NamedValue {
                name: field.to_string(),
                value: FrameValueKind::Array(items),
            }],
        }
    }

    /// The REGROUP, on an array comfortably UNDER the cap: a struct array is
    /// emitted FIELD-major (`f0` for every element, then `f1` for every element),
    /// not element-major. Applied whether or not the cap bites, so the plot legend
    /// has one shape rather than two.
    ///
    /// The oracle is written out by hand for all 6 samples — it is NOT a
    /// permutation of `harvest_series`'s own output.
    #[test]
    fn a_struct_array_under_the_cap_is_emitted_field_major() {
        let (samples, skipped) = harvest_series(&struct_array("motors", 3, 2));
        assert_eq!(
            samples,
            vec![
                ("motors/0/f0".to_string(), 0.0),
                ("motors/1/f0".to_string(), 100.0),
                ("motors/2/f0".to_string(), 200.0),
                ("motors/0/f1".to_string(), 1.0),
                ("motors/1/f1".to_string(), 101.0),
                ("motors/2/f1".to_string(), 201.0),
            ]
        );
        assert!(
            skipped.is_empty(),
            "nothing is withheld under the cap: {skipped:?}"
        );
    }

    /// A regroup must never LOSE or INVENT a sample — the anti-tautology partner
    /// of the ordering oracle above, which on its own would pass an implementation
    /// that dropped a field and reordered the rest. Compared as SETS against a
    /// hand-built expectation of every (path, value) pair.
    #[test]
    fn the_field_major_regroup_preserves_the_whole_sample_set() {
        let (samples, _) = harvest_series(&struct_array("motors", 4, 3));
        let got: std::collections::BTreeSet<(String, u64)> = samples
            .iter()
            .map(|(p, v)| (p.clone(), v.to_bits()))
            .collect();
        let want: std::collections::BTreeSet<(String, u64)> = (0..4)
            .flat_map(|i| {
                (0..3).map(move |j| (format!("motors/{i}/f{j}"), ((i * 100 + j) as f64).to_bits()))
            })
            .collect();
        assert_eq!(got, want, "every element × field survives under the cap");
        assert_eq!(samples.len(), 12, "no duplicates");
    }

    /// THE headline: the 20-motor bank. 20 elements × 12 numeric fields =
    /// 240 series from ONE field — under the 64-ELEMENT cap and therefore invisible
    /// to it, which is why it needs a cap of its own.
    ///
    /// The oracle is the FIELD-boundary rule: `64 / 20 = 3` whole fields survive
    /// for EVERY element (60 series), and the report names the 180 that did not.
    /// Element-first truncation would instead have kept motors 0-4 complete and
    /// left fifteen joints with no trace — asserted against explicitly below.
    #[test]
    fn a_twenty_element_struct_array_keeps_whole_fields_for_every_element() {
        let (samples, skipped) = harvest_series(&struct_array("motor_state", 20, 12));
        assert_eq!(samples.len(), 60, "64 / 20 = 3 whole fields × 20 elements");

        // EVERY element is represented (the joint-bank win) …
        for i in 0..20 {
            for j in 0..3 {
                assert!(
                    samples.contains(&(format!("motor_state/{i}/f{j}"), (i * 100 + j) as f64)),
                    "element {i} field {j} must survive"
                );
            }
        }
        // … and the discriminator against element-first truncation: motor 19 is
        // present while motor 0's LATER fields are gone. Element-first would show
        // exactly the opposite.
        assert!(samples.iter().any(|(p, _)| p == "motor_state/19/f0"));
        assert!(
            !samples.iter().any(|(p, _)| p == "motor_state/0/f3"),
            "field 3 is past the boundary for every element, motor 0 included"
        );

        assert_eq!(
            skipped,
            vec![SkippedSeries::StructArrayFields {
                field: "motor_state".to_string(),
                elements: 20,
                fields_per_element: 12,
                series_per_element: 12,
                fields_kept: 3,
                series_kept: 3,
            }]
        );
        let line = skipped[0].to_string();
        for marker in [
            "motor_state",
            "20 struct element(s)",
            "12 numeric field(s)",
            "240 series",
            "first 3 field(s) of EVERY element",
            "180 series not plotted",
            "64-series per-struct-array plot cap",
        ] {
            assert!(
                line.contains(marker),
                "the operator line must carry {marker:?}: {line}"
            );
        }
    }

    /// The cap is a THRESHOLD, pinned on both sides at one field. A widened
    /// divisor would still curate the 20×12 bank above and pass that test.
    #[test]
    fn the_struct_array_cap_is_a_threshold_pinned_on_both_sides() {
        // Exactly AT the cap (32 × 2 = 64): everything plots, nothing reported.
        let (at, at_skipped) = harvest_series(&struct_array("bank", 32, 2));
        assert_eq!(at.len(), MAX_STRUCT_ARRAY_SERIES);
        assert!(at_skipped.is_empty(), "at the cap nothing is withheld");

        // One field PAST it (32 × 3 = 96): the whole third field goes, for every
        // element — 64 survive, not 65.
        let (over, over_skipped) = harvest_series(&struct_array("bank", 32, 3));
        assert_eq!(over.len(), MAX_STRUCT_ARRAY_SERIES);
        assert_eq!(
            over_skipped,
            vec![SkippedSeries::StructArrayFields {
                field: "bank".to_string(),
                elements: 32,
                fields_per_element: 3,
                series_per_element: 3,
                fields_kept: 2,
                series_kept: 2,
            }]
        );
    }

    /// The ELEMENT cap still runs FIRST and still owns its own report — a struct
    /// array past [`MAX_ARRAY_SERIES`] is refused whole, not curated. Without this
    /// the two caps could quietly swap precedence and a 65-element bank would
    /// start plotting one field per element instead of being reported.
    #[test]
    fn the_element_cap_still_precedes_the_struct_array_cap() {
        let (samples, skipped) = harvest_series(&struct_array("bank", MAX_ARRAY_SERIES + 1, 2));
        assert!(samples.is_empty(), "got {samples:?}");
        assert_eq!(
            skipped,
            vec![SkippedSeries::Oversized {
                field: "bank".to_string(),
                count: MAX_ARRAY_SERIES + 1,
            }]
        );
    }

    /// A FIXED array of PRIMITIVES decodes through the same arm with one series
    /// per element, so field-major and element-major coincide — the regroup must
    /// be a no-op there (the earlier behaviour, byte for byte).
    #[test]
    fn a_fixed_primitive_array_is_unchanged_by_the_regroup() {
        let root = FrameValue {
            schema_name: "acme/Bank".to_string(),
            fields: vec![cerulion_core::codegen::NamedValue {
                name: "corners".to_string(),
                value: FrameValueKind::Array(vec![
                    FrameValueKind::F64(1.0),
                    FrameValueKind::F64(2.0),
                    FrameValueKind::F64(3.0),
                ]),
            }],
        };
        let (samples, skipped) = harvest_series(&root);
        assert_eq!(
            samples,
            vec![
                ("corners/0".to_string(), 1.0),
                ("corners/1".to_string(), 2.0),
                ("corners/2".to_string(), 3.0),
            ]
        );
        assert!(skipped.is_empty());
    }

    /// Elements that disagree about their own series count (one carries a numeric
    /// member the others do not) leave the regroup UNDEFINED, so declaration order
    /// is kept and NOTHING is dropped. Silent partial reordering of a ragged array
    /// would scramble which value sits under which path.
    #[test]
    fn a_ragged_struct_array_keeps_declaration_order_and_loses_nothing() {
        let element = |fields: Vec<cerulion_core::codegen::NamedValue<'static>>| {
            FrameValueKind::Nested(Box::new(FrameValue {
                schema_name: "acme/Element".to_string(),
                fields,
            }))
        };
        let root = FrameValue {
            schema_name: "acme/Bank".to_string(),
            fields: vec![cerulion_core::codegen::NamedValue {
                name: "bank".to_string(),
                value: FrameValueKind::Array(vec![
                    element(vec![f64v("a", 1.0), f64v("b", 2.0)]),
                    // Ragged: one member only, so 3 samples over 2 elements.
                    element(vec![f64v("a", 3.0)]),
                ]),
            }],
        };
        let (samples, skipped) = harvest_series(&root);
        assert_eq!(
            samples,
            vec![
                ("bank/0/a".to_string(), 1.0),
                ("bank/0/b".to_string(), 2.0),
                ("bank/1/a".to_string(), 3.0),
            ]
        );
        assert!(skipped.is_empty(), "a ragged array withholds nothing");
    }

    /// The ENTITY-PATH sub-grouping contract: every series a struct
    /// array contributes is path-qualified `field/index/member`, so the topic's
    /// scalars land in a per-field entity SUBTREE the viewer can collapse — not
    /// as N flat siblings of the topic entity. Blueprint views scope by
    /// `/entity/**`, so this shape is what makes a wide plot topic navigable.
    #[test]
    fn struct_array_series_are_path_grouped_under_their_own_field() {
        let root = FrameValue {
            schema_name: "acme/Bank".to_string(),
            fields: vec![
                f64v("power_v", 24.0),
                struct_array("motors", 2, 2).fields.remove(0),
            ],
        };
        let (samples, _) = harvest_series(&root);
        let paths: Vec<&str> = samples.iter().map(|(p, _)| p.as_str()).collect();
        assert_eq!(
            paths,
            vec![
                "power_v",
                "motors/0/f0",
                "motors/1/f0",
                "motors/0/f1",
                "motors/1/f1",
            ]
        );
        // The group prefix is what the viewer collapses: every array series sits
        // under `motors/`, the bare scalar does not.
        assert!(paths
            .iter()
            .filter(|p| p.starts_with("motors/"))
            .count()
            .eq(&4));
        assert!(
            !paths[0].contains('/'),
            "a bare scalar keeps no group prefix"
        );
    }

    /// A DEPTH-2 bank: `outer` elements, each declaring ONE field that is itself
    /// an array of `inner` structs with `fields` numeric members. Element (i, k)
    /// member j carries `i * 1000 + k * 100 + j`, so every sample identifies its
    /// exact origin.
    fn nested_struct_array(
        field: &str,
        outer: usize,
        inner: usize,
        fields: usize,
    ) -> FrameValue<'static> {
        let leg = |i: usize| {
            FrameValueKind::Nested(Box::new(FrameValue {
                schema_name: "acme/Leg".to_string(),
                fields: vec![cerulion_core::codegen::NamedValue {
                    name: "motors".to_string(),
                    value: FrameValueKind::Array(
                        (0..inner)
                            .map(|k| {
                                FrameValueKind::Nested(Box::new(FrameValue {
                                    schema_name: "acme/Motor".to_string(),
                                    fields: (0..fields)
                                        .map(|j| {
                                            f64v(&format!("f{j}"), (i * 1000 + k * 100 + j) as f64)
                                        })
                                        .collect(),
                                }))
                            })
                            .collect(),
                    ),
                }],
            }))
        };
        FrameValue {
            schema_name: "acme/Bank".to_string(),
            fields: vec![cerulion_core::codegen::NamedValue {
                name: field.to_string(),
                value: FrameValueKind::Array((0..outer).map(leg).collect()),
            }],
        }
    }

    /// A bank of `elements` structs, each declaring a nested `motors[inner]`
    /// array of `{q, dq}` followed by `scalars` plain members. The ONLY shape in
    /// which one field is worth more than one series, which is what makes the
    /// deep-form report (and the mid-field hazard) reachable.
    fn mixed_bank(
        field: &str,
        elements: usize,
        inner: usize,
        scalars: usize,
    ) -> FrameValue<'static> {
        let element = |i: usize| {
            FrameValueKind::Nested(Box::new(FrameValue {
                schema_name: "acme/Element".to_string(),
                fields: std::iter::once(cerulion_core::codegen::NamedValue {
                    name: "motors".to_string(),
                    value: FrameValueKind::Array(
                        (0..inner)
                            .map(|k| {
                                FrameValueKind::Nested(Box::new(FrameValue {
                                    schema_name: "acme/Motor".to_string(),
                                    fields: vec![
                                        f64v("q", (i * 10 + k) as f64),
                                        f64v("dq", (i * 10 + k + 5) as f64),
                                    ],
                                }))
                            })
                            .collect(),
                    ),
                })
                .chain((0..scalars).map(|j| f64v(&format!("s{j}"), (i * 100 + j) as f64)))
                .collect(),
            }))
        };
        FrameValue {
            schema_name: "acme/Bank".to_string(),
            fields: vec![cerulion_core::codegen::NamedValue {
                name: field.to_string(),
                value: FrameValueKind::Array((0..elements).map(element).collect()),
            }],
        }
    }

    /// **The whole-field rule must RECURSE.**
    ///
    /// At depth 2 the outer regroup's per-element unit is a SERIES count, not a
    /// FIELD count: a `LegState[4]` whose element declares `MotorState[3]`
    /// contributes 36 series through ONE field. A naive arithmetic
    /// (`budget / elements`, counting series as fields) therefore cuts INSIDE a
    /// field — motor 0 keeping `f5` while motors 1 and 2 do not — which is the
    /// "four of twenty motors' tau_est" shape the whole rule exists to forbid.
    ///
    /// The rule has two halves and this pins both: the budget DIVIDES on the way
    /// down (so the inner array curates itself to its share), and the truncation
    /// walks the element's REAL field boundaries.
    ///
    /// Hand oracle: outer budget 64 / 4 legs = 16 per leg; the inner 3-motor array
    /// keeps whole fields while `k * 3 <= 16`, i.e. 5 of 12 ⇒ 15 series per leg ⇒
    /// 60 total, inside the 64 cap.
    #[test]
    fn a_depth_two_bank_truncates_on_whole_fields_for_every_element() {
        let (samples, skipped) = harvest_series(&nested_struct_array("legs", 4, 3, 12));
        assert_eq!(samples.len(), 60, "4 legs x 3 motors x 5 whole fields");

        // EVERY (leg, motor) pair keeps EXACTLY fields f0..f4 …
        for i in 0..4 {
            for k in 0..3 {
                for j in 0..5 {
                    assert!(
                        samples.contains(&(
                            format!("legs/{i}/motors/{k}/f{j}"),
                            (i * 1000 + k * 100 + j) as f64
                        )),
                        "leg {i} motor {k} field {j} must survive"
                    );
                }
            }
        }
        // … and NOBODY keeps f5. The mid-field cut this test exists for shows up
        // as motor 0 having it while motors 1-2 do not, so both halves are named.
        assert!(
            !samples.iter().any(|(p, _)| p.ends_with("/f5")),
            "the cut lands on a whole-field boundary for EVERY element"
        );

        // The report counts FIELDS, not series (a series-counting line would claim
        // the element declares 36 numeric fields; it declares 12), and there is one
        // per outer element because each leg's inner array is its own array.
        assert_eq!(skipped.len(), 4, "one per leg: {skipped:?}");
        for (i, skip) in skipped.iter().enumerate() {
            assert_eq!(
                skip,
                &SkippedSeries::StructArrayFields {
                    field: format!("legs/{i}/motors"),
                    elements: 3,
                    fields_per_element: 12,
                    series_per_element: 12,
                    fields_kept: 5,
                    series_kept: 5,
                }
            );
        }
        // The RENDERED line is what an operator reads, so it is pinned as
        // rendered — an unpinned line is how false arithmetic ships. Here a field IS one
        // series (12 == 12), so the simple form is used and 3 x 12 = 36 holds.
        assert_eq!(
            skipped[0].to_string(),
            "legs/0/motors (3 struct element(s) × 12 numeric field(s) = 36 series; the first 5 \
             field(s) of EVERY element are plotted, 21 series not plotted — over the 64-series \
             per-struct-array plot cap)"
        );
    }

    /// The OUTER level's own report must count fields too. An element that mixes
    /// plain scalars with a nested array overflows at the outer level, and the cut
    /// must still land on a field boundary — here after the two scalars, dropping
    /// the whole nested-array field rather than part of it.
    ///
    /// Hand oracle: 20 outer elements ⇒ 3 series per element may be kept
    /// (64 / 20 = 3); the element's fields are `a` (1), `b` (1), `motors` (6 after
    /// its own budget-3 curation ⇒ 1 whole field x 6? no — budget 64/20 = 3, so
    /// the inner 2-motor array keeps 1 field = 2 series). Element = 1 + 1 + 2 = 4
    /// series over 3 fields; 3 series fit only 2 whole fields (a, b), so `motors`
    /// goes whole.
    #[test]
    fn an_outer_overflow_drops_a_nested_array_field_whole() {
        let element = |i: usize| {
            FrameValueKind::Nested(Box::new(FrameValue {
                schema_name: "acme/Element".to_string(),
                fields: vec![
                    f64v("a", i as f64),
                    f64v("b", (i + 50) as f64),
                    cerulion_core::codegen::NamedValue {
                        name: "motors".to_string(),
                        value: FrameValueKind::Array(
                            (0..2)
                                .map(|k| {
                                    FrameValueKind::Nested(Box::new(FrameValue {
                                        schema_name: "acme/Motor".to_string(),
                                        fields: vec![
                                            f64v("q", (i * 10 + k) as f64),
                                            f64v("dq", (i * 10 + k + 5) as f64),
                                        ],
                                    }))
                                })
                                .collect(),
                        ),
                    },
                ],
            }))
        };
        let root = FrameValue {
            schema_name: "acme/Bank".to_string(),
            fields: vec![cerulion_core::codegen::NamedValue {
                name: "bank".to_string(),
                value: FrameValueKind::Array((0..20).map(element).collect()),
            }],
        };
        let (samples, skipped) = harvest_series(&root);
        // `a` and `b` for all 20 elements; nothing from `motors`.
        assert_eq!(samples.len(), 40);
        assert!(!samples.iter().any(|(p, _)| p.contains("/motors/")));
        for i in 0..20 {
            assert!(samples.contains(&(format!("bank/{i}/a"), i as f64)));
            assert!(samples.contains(&(format!("bank/{i}/b"), (i + 50) as f64)));
        }
        // The OUTER report: the element declares 3 fields (a, b, motors) worth 4
        // series, of which 2 fields survived.
        //
        // **Second order: it is the ONLY report.** Each element's inner
        // `motors` array is itself over ITS share of the budget, so expanding it
        // pushed a `StructArrayFields` per element — twenty lines each claiming
        // "the first 1 field(s) of EVERY element are plotted" for a group this
        // level then dropped ENTIRELY. Zero of those series reached a plot, so
        // every one of those lines was false. They are retracted; what the
        // operator is told is this level's own line.
        assert_eq!(
            skipped,
            vec![SkippedSeries::StructArrayFields {
                field: "bank".to_string(),
                elements: 20,
                fields_per_element: 3,
                series_per_element: 4,
                fields_kept: 2,
                series_kept: 2,
            }],
            "the dropped group's own reports are retracted: {skipped:?}"
        );
        // First order: the rendered line. A field here is worth MORE than
        // one series (3 fields, 4 series), and a field-counting form prints
        // "20 × 3 = 80" — arithmetic an operator can check in their head and find
        // wrong. Every multiplication printed must hold: 20 × 4 = 80.
        assert_eq!(
            skipped[0].to_string(),
            "bank (20 struct element(s) × 4 series each = 80 series, across 3 numeric field(s) \
             per element; the first 2 field(s) (2 series) of EVERY element are plotted, 40 series \
             not plotted — over the 64-series per-struct-array plot cap)"
        );
    }

    /// **The retraction predicate's own oracle vector.**
    ///
    /// [`path_lies_in_group`] decides which skip reports a dropped group's
    /// expansion pushed, so a wrong answer either leaves a FALSE line in the
    /// operator diagnostic (too narrow) or swallows a true one from a surviving
    /// sibling (too broad). Its doc claimed "pure + oracle-tested" while nothing
    /// tested it — `motors_raw`, the adversarial name the doc cites as the whole
    /// reason for the segment-boundary check, existed only in a comment.
    ///
    /// Hand-written truth table, every arm naming what it defends:
    #[test]
    fn path_lies_in_group_matches_only_whole_segments_under_a_numeric_index() {
        let dropped = ["motors"];

        // The exact group of an element — the ordinary case.
        assert!(path_lies_in_group("bank/0/motors", "bank", &dropped));
        // Anything BENEATH it: a nested array's own report sits deeper, and it is
        // dropped along with its parent group.
        assert!(path_lies_in_group(
            "bank/0/motors/2/inner",
            "bank",
            &dropped
        ));
        // A double-digit index is still an index.
        assert!(path_lies_in_group("bank/17/motors", "bank", &dropped));

        // THE load-bearing arm: a SIBLING sharing the group's name as a prefix
        // must NOT be retracted. Without the segment-boundary check `motors_raw`
        // is swallowed by `motors` and its own report disappears.
        assert!(!path_lies_in_group("bank/0/motors_raw", "bank", &dropped));
        assert!(!path_lies_in_group("bank/0/motorsX/1/q", "bank", &dropped));

        // The index segment must be an INDEX. A field literally named `x` under
        // the array path is not element `x`, and treating it as one would retract
        // reports from a completely different subtree.
        assert!(!path_lies_in_group("bank/x/motors", "bank", &dropped));
        assert!(!path_lies_in_group("bank//motors", "bank", &dropped));
        assert!(!path_lies_in_group("bank/1a/motors", "bank", &dropped));

        // A different array entirely — the prefix must match, and it must match
        // at a segment boundary too.
        assert!(!path_lies_in_group("other/0/motors", "bank", &dropped));
        assert!(!path_lies_in_group("bankX/0/motors", "bank", &dropped));

        // START-ANCHORED: a same-named group nested under a SURVIVING sibling is
        // not this level's `motors`. Retracting it would delete a report about
        // series that really were plotted.
        assert!(!path_lies_in_group(
            "bank/0/kept/2/motors",
            "bank",
            &dropped
        ));

        // Structural non-matches: no index segment at all, and the array's own
        // path (which is what the OUTER report names — it must never retract
        // itself).
        assert!(!path_lies_in_group("bank/0", "bank", &dropped));
        assert!(!path_lies_in_group("bank", "bank", &dropped));

        // Multiple dropped groups: any one of them matches, and a name in
        // neither does not.
        let two = ["motors", "sensors"];
        assert!(path_lies_in_group("bank/3/sensors", "bank", &two));
        assert!(!path_lies_in_group("bank/3/actuators", "bank", &two));
        // An empty dropped set retracts nothing (the regroup never calls it with
        // one, but a predicate that answered `true` here would be catastrophic).
        assert!(!path_lies_in_group("bank/0/motors", "bank", &[]));
    }

    /// **The printed multiplication, as a CLASS rather than two instances.** Whatever shape produced
    /// it, a `StructArrayFields` line must never print a multiplication that does
    /// not hold: the reader can do the arithmetic, and a line that fails it
    /// discredits every other number in the diagnostic.
    ///
    /// Parses `A × B = C` back out of the rendered line across a MATRIX of shapes
    /// (depth 1 and depth 2, several widths) and checks `A * B == C`. Instance
    /// pins give exact wording; this gives coverage.
    ///
    /// **SCOPE, stated so it is not over-trusted: this is a SELF-CONSISTENCY
    /// check.** It reads the three numbers back out of the line `Display` just
    /// wrote, so it proves the sentence does not contradict itself — which is
    /// exactly the bug that shipped (a total computed from one quantity, printed
    /// against another). It cannot catch a triple that is internally consistent
    /// and still WRONG about the harvest, because it never consults the harvest's
    /// own counts. The two verbatim instance pins are what tie the rendered
    /// numbers to real curated shapes; this arm is what stops a NEW shape from
    /// re-introducing the contradiction.
    #[test]
    fn every_rendered_struct_array_line_states_arithmetic_that_holds() {
        // (label, frame, plotted-sample count)
        let cases: Vec<(String, FrameValue<'static>)> = vec![
            ("depth1 20x12".to_string(), struct_array("bank", 20, 12)),
            ("depth1 32x3".to_string(), struct_array("bank", 32, 3)),
            ("depth1 64x2".to_string(), struct_array("bank", 64, 2)),
            (
                "depth2 4x3x12".to_string(),
                nested_struct_array("legs", 4, 3, 12),
            ),
            (
                "depth2 8x2x9".to_string(),
                nested_struct_array("legs", 8, 2, 9),
            ),
            // The MIXED shapes — a field worth several series beside plain
            // scalars. These are what render the deep form; without one in the
            // matrix this test only pins the branch that was already correct.
            ("mixed 20x2+30".to_string(), mixed_bank("bank", 20, 2, 30)),
            ("mixed 10x3+8".to_string(), mixed_bank("bank", 10, 3, 8)),
        ];
        let mut saw_deep_form = false;
        for (label, fv) in cases {
            let (samples, skipped) = harvest_series(&fv);
            for skip in &skipped {
                let SkippedSeries::StructArrayFields { .. } = skip else {
                    continue;
                };
                let line = skip.to_string();
                let (a, b, c) = parse_multiplication(&line)
                    .unwrap_or_else(|| panic!("{label}: no `A × B = C` in {line}"));
                assert_eq!(a * b, c, "{label}: printed {a} × {b} = {c} in {line}");
                if line.contains("series each") {
                    saw_deep_form = true;
                }
            }
            // Anti-vacuity: these shapes really are over budget and really do
            // plot something, so the lines above are describing real curation.
            assert!(!skipped.is_empty(), "{label}: nothing was withheld");
            assert!(!samples.is_empty(), "{label}: nothing was plotted");
        }
        assert!(
            saw_deep_form,
            "the matrix must exercise the depth->=2 form, or it only pins the easy branch"
        );
    }

    /// Pull `A × B = C` out of a rendered skip line — the three numbers around
    /// the first `×` and the `=` that follows it.
    fn parse_multiplication(line: &str) -> Option<(usize, usize, usize)> {
        let (before, rest) = line.split_once('×')?;
        let (mid, after) = rest.split_once('=')?;
        let last_number = |s: &str| -> Option<usize> {
            s.split(|c: char| !c.is_ascii_digit())
                .rfind(|t| !t.is_empty())?
                .parse()
                .ok()
        };
        let first_number = |s: &str| -> Option<usize> {
            s.split(|c: char| !c.is_ascii_digit())
                .find(|t| !t.is_empty())?
                .parse()
                .ok()
        };
        Some((
            last_number(before)?,
            first_number(mid)?,
            first_number(after)?,
        ))
    }

    /// The retraction must be SCOPED to the dropped group — a sibling field's own
    /// report, and a report from a group that SURVIVED, must both be kept. Without
    /// this the retraction could silently swallow every nested diagnostic.
    #[test]
    fn retraction_spares_reports_from_groups_that_survived() {
        // The MIXED shape with the nested array FIRST: 20 elements ⇒ 3 series of
        // budget each, and `motors` (curated to 2 series of its own) plus `s0`
        // exactly fill it, so the nested group SURVIVES while the later scalars
        // are cut. The inverse of the outer-overflow arm, where `motors` came
        // last and was the group dropped.
        let (samples, skipped) = harvest_series(&mixed_bank("bank", 20, 2, 30));

        // `motors` survived, so its 20 inner reports are NOT retracted — the
        // retraction is scoped to DROPPED groups, not applied wholesale.
        let inner = skipped
            .iter()
            .filter(|s| {
                matches!(s, SkippedSeries::StructArrayFields { field, .. } if field.ends_with("/motors"))
            })
            .count();
        assert_eq!(
            inner, 20,
            "a SURVIVING group keeps its own reports: {skipped:?}"
        );
        // … and the samples confirm it really survived, for EVERY element.
        for i in 0..20 {
            assert!(samples
                .iter()
                .any(|(p, _)| p == &format!("bank/{i}/motors/0/q")));
            assert!(samples.iter().any(|(p, _)| p == &format!("bank/{i}/s0")));
            assert!(!samples.iter().any(|(p, _)| p == &format!("bank/{i}/s1")));
        }
        // The outer's own line is present too (the scalars really were cut).
        assert!(
            skipped.iter().any(
                |s| matches!(s, SkippedSeries::StructArrayFields { field, .. } if field == "bank")
            ),
            "{skipped:?}"
        );
    }

    /// **An EMPTY fixed struct array must not panic the sink.**
    ///
    /// `Type[0]` is accepted by the rosmsg parser and the walker surfaces it as
    /// `FrameValueKind::Array(vec![])`, so the element count is REMOTE-SUPPLIED: a
    /// robot whose schema text declares a zero-length fixed array reaches this
    /// path on a desk that never chose to. The budget division runs BEFORE the
    /// regroup's `elements == 0` guard (unreachable on this
    /// path), so without its own guard the harvest divides by zero and takes the whole sink down.
    #[test]
    fn an_empty_fixed_struct_array_is_harvested_without_panicking() {
        let root = FrameValue {
            schema_name: "acme/Bank".to_string(),
            fields: vec![
                f64v("power_v", 24.0),
                cerulion_core::codegen::NamedValue {
                    name: "motors".to_string(),
                    value: FrameValueKind::Array(Vec::new()),
                },
            ],
        };
        let (samples, skipped) = harvest_series(&root);
        // The sibling scalar still plots; the empty array contributes nothing and
        // withholds nothing (there is no data to withhold).
        assert_eq!(samples, vec![("power_v".to_string(), 24.0)]);
        assert!(
            skipped.is_empty(),
            "an empty array withholds nothing: {skipped:?}"
        );
        // …and it must not claim a plot series of its own: a `Type[0]` declares
        // zero series, so a topic carrying ONLY one is not a plotting topic.
        let only_empty = FrameValue {
            schema_name: "acme/Bank".to_string(),
            fields: vec![cerulion_core::codegen::NamedValue {
                name: "motors".to_string(),
                value: FrameValueKind::Array(Vec::new()),
            }],
        };
        assert!(harvest_series(&only_empty).0.is_empty());
        assert!(!declares_plottable_series(&only_empty));
    }

    // ---- The per-topic total-series backstop -----------------------

    /// A message declaring `n` top-level numeric scalars named `s{i}` valued `i`.
    fn wide_message(n: usize) -> FrameValue<'static> {
        FrameValue {
            schema_name: "acme/Wide".to_string(),
            fields: (0..n).map(|i| f64v(&format!("s{i}"), i as f64)).collect(),
        }
    }

    /// The backstop is a THRESHOLD pinned on both sides: exactly at
    /// [`MAX_TOTAL_SERIES`] nothing is withheld, one past it exactly one series is
    /// — with the report naming WHERE the plot stops. A cap applied at the
    /// wrong boundary (or an off-by-one truncation) fails one side or the other.
    #[test]
    fn the_total_series_cap_is_a_threshold_pinned_on_both_sides() {
        let (at, at_skipped) = harvest_series(&wide_message(MAX_TOTAL_SERIES));
        assert_eq!(at.len(), MAX_TOTAL_SERIES);
        assert!(at_skipped.is_empty(), "at the cap nothing is withheld");

        let (over, over_skipped) = harvest_series(&wide_message(MAX_TOTAL_SERIES + 1));
        assert_eq!(over.len(), MAX_TOTAL_SERIES);
        assert_eq!(
            over_skipped,
            vec![SkippedSeries::TotalCap {
                dropped: 1,
                first_dropped: format!("s{MAX_TOTAL_SERIES}"),
            }]
        );
    }

    /// What SURVIVES is the earliest-declared prefix, in order — the message
    /// author's own ordering, not an arbitrary subset. Asserted on the first and
    /// last kept series plus the whole length, so a cap that kept the LAST 256 (or
    /// a shuffled 256) fails.
    #[test]
    fn the_total_series_cap_keeps_the_earliest_declared_prefix() {
        let over = MAX_TOTAL_SERIES + 40;
        let (samples, skipped) = harvest_series(&wide_message(over));
        assert_eq!(samples.len(), MAX_TOTAL_SERIES);
        assert_eq!(samples[0], ("s0".to_string(), 0.0));
        assert_eq!(
            samples[MAX_TOTAL_SERIES - 1],
            (
                format!("s{}", MAX_TOTAL_SERIES - 1),
                (MAX_TOTAL_SERIES - 1) as f64
            )
        );
        assert_eq!(
            skipped,
            vec![SkippedSeries::TotalCap {
                dropped: 40,
                first_dropped: format!("s{MAX_TOTAL_SERIES}"),
            }]
        );
        let line = skipped[0].to_string();
        for marker in [
            format!("s{MAX_TOTAL_SERIES}"),
            "39 later series".to_string(),
            "40 in total".to_string(),
            format!("{MAX_TOTAL_SERIES}-series per-topic plot cap"),
        ] {
            assert!(
                line.contains(&marker),
                "the operator line must carry {marker:?}: {line}"
            );
        }
    }

    /// The backstop must NOT blind the classifier. The walk still runs to
    /// completion past the cap, so a `string` field declared AFTER the 256th
    /// series is still seen — otherwise a wide mixed message would classify
    /// plots-only and silently lose its text (the silent-text-drop failure mode, re-opened).
    /// The companion claim — a capped topic still counts as plottable — is
    /// asserted in the same body, since a cap that cleared the latch would hand a
    /// 256-line topic no plot view at all.
    #[test]
    fn the_total_series_cap_does_not_blind_the_shape_predicates() {
        let mut fv = wide_message(MAX_TOTAL_SERIES + 10);
        fv.fields.push(cerulion_core::codegen::NamedValue {
            name: "status".to_string(),
            value: FrameValueKind::Str("armed"),
        });
        assert!(
            declares_text_fields(&fv),
            "a text field past the cap is still SEEN by the classifier"
        );
        assert!(
            declares_plottable_series(&fv),
            "a topic plotting the full cap has earned its plot view"
        );
        // Anti-tautology: the same message WITHOUT the trailing string does not
        // claim text, so the assertion above is reading the field, not a constant.
        assert!(!declares_text_fields(&wide_message(MAX_TOTAL_SERIES + 10)));
    }

    /// The two caps COMPOSE in the documented order: per-field curation first,
    /// then the backstop over what survived. Five 20×12 struct arrays declare 1200
    /// series; each is curated to 60 (the field-boundary rule), leaving 300, which
    /// the backstop then trims to 256 — so BOTH reports appear and the survivor
    /// count is the composed one, never 1200 trimmed blindly to 256.
    #[test]
    fn the_per_field_curation_runs_before_the_total_backstop() {
        let fields: Vec<cerulion_core::codegen::NamedValue<'static>> = (0..5)
            .map(|k| struct_array(&format!("bank{k}"), 20, 12).fields.remove(0))
            .collect();
        let root = FrameValue {
            schema_name: "acme/FiveBanks".to_string(),
            fields,
        };
        let (samples, skipped) = harvest_series(&root);
        assert_eq!(samples.len(), MAX_TOTAL_SERIES);
        // Five per-field reports (60 survive each ⇒ 300) …
        let struct_reports = skipped
            .iter()
            .filter(|s| matches!(s, SkippedSeries::StructArrayFields { .. }))
            .count();
        assert_eq!(
            struct_reports, 5,
            "one report per curated bank: {skipped:?}"
        );
        // … then the backstop over the 300 that survived, dropping exactly 44.
        assert!(
            skipped.contains(&SkippedSeries::TotalCap {
                dropped: 300 - MAX_TOTAL_SERIES,
                first_dropped: "bank4/16/f0".to_string(),
            }),
            "the backstop reports the COMPOSED overflow: {skipped:?}"
        );
    }

    /// The SHAPE predicate's own contract, independent of
    /// any classifier: it answers "could this schema ever plot", where
    /// [`harvest_series`] answers "what does THIS frame plot". They agree
    /// wherever the frame has values; they differ EXACTLY on the frame-varying
    /// case — a declared numeric array that is momentarily empty — which is the
    /// whole reason the classification cannot read the samples.
    #[test]
    fn declares_plottable_series_is_the_shape_question_not_the_value_question() {
        let arr = |bytes: &'static [u8], count: usize, name: &str| FrameValue {
            schema_name: "acme/Bank".to_string(),
            fields: vec![prim_named(name, bytes, count)],
        };
        // A declared numeric array with NO values this frame: nothing to plot
        // now, but unquestionably a plotting topic.
        let empty = arr(&[], 0, "position");
        assert!(harvest_series(&empty).0.is_empty(), "no values this frame");
        assert!(
            declares_plottable_series(&empty),
            "…but the SHAPE declares a plot series"
        );
        // Populated: both agree.
        let filled = arr(f64_bytes(&[1.0, 2.0]), 2, "position");
        assert_eq!(harvest_series(&filled).0.len(), 2);
        assert!(declares_plottable_series(&filled));
        // A covariance matrix is never plotted — not even by shape (it would earn
        // a permanently empty panel).
        let cov = arr(f64_bytes(&[1.0, 2.0]), 2, "covariance");
        assert!(!declares_plottable_series(&cov));
        // Nor is an array over the plot cap (also never expanded).
        let big: Vec<f64> = (0..=MAX_ARRAY_SERIES).map(|i| i as f64).collect();
        let over = arr(f64_bytes(&big), MAX_ARRAY_SERIES + 1, "bins");
        assert!(!declares_plottable_series(&over));
        // Nothing numeric at all ⇒ false (strings / bools are not plots).
        let text = FrameValue {
            schema_name: "acme/Status".to_string(),
            fields: vec![cerulion_core::codegen::NamedValue {
                name: "message".to_string(),
                value: FrameValueKind::Str("ok"),
            }],
        };
        assert!(!declares_plottable_series(&text));
        // The exclusion half prunes the same nodes as the sample half: a pose
        // whose ONLY other field is its consumed position declares nothing.
        let pose_only = FrameValue {
            schema_name: "acme/Waypoint".to_string(),
            fields: vec![nested_named("position", vec3(1.0, 2.0, 3.0))],
        };
        assert!(declares_plottable_series(&pose_only));
        assert!(!declares_plottable_series_excluding(
            &pose_only,
            &["position".to_string()]
        ));
    }

    /// `declares_text_fields` against a HAND classification of each
    /// shape it must separate — the predicate that elects `ScalarsWithText`.
    #[test]
    fn declares_text_fields_separates_a_text_payload_from_an_envelope() {
        let str_named = |name: &str, v: &'static str| cerulion_core::codegen::NamedValue {
            name: name.to_string(),
            value: FrameValueKind::Str(v),
        };
        let fields = |name: &str, f: Vec<cerulion_core::codegen::NamedValue<'static>>| FrameValue {
            schema_name: name.to_string(),
            fields: f,
        };

        // THE shape: a vendor status message — numeric envelope, string payload.
        let status = fields(
            "acme/Response",
            vec![f64v("code", 0.0), str_named("data", "{\"battery\":97}")],
        );
        assert!(
            declares_text_fields(&status),
            "a declared string field is text the numeric harvest cannot plot"
        );
        // …and it stays true when THIS frame's string is EMPTY — the
        // frame-invariance the once-resolved layout depends on.
        let empty_payload = fields(
            "acme/Response",
            vec![f64v("code", 0.0), str_named("data", "")],
        );
        assert!(
            declares_text_fields(&empty_payload),
            "an empty string is still a DECLARED string — the layout must not flip"
        );

        // ANTI-TAUTOLOGY: a purely numeric message declares no text.
        let numbers = fields("acme/Telemetry", vec![f64v("voltage", 48.0)]);
        assert!(!declares_text_fields(&numbers));

        // THE NOISE CONTROL, and the reason the predicate rides the harvest walk:
        // a `std_msgs/Header`'s `frame_id` is envelope. Counting it would hand a
        // text pane to EVERY stamped message in ROS.
        let header = FrameValue {
            schema_name: "std_msgs/Header".to_string(),
            fields: vec![f64v("seq", 7.0), str_named("frame_id", "base_link")],
        };
        let stamped = fields(
            "acme/StampedTelemetry",
            vec![nested_named("header", header.clone()), f64v("speed", 1.5)],
        );
        assert!(
            !declares_text_fields(&stamped),
            "a header's frame_id is envelope, not the message's text payload"
        );
        // The same nested value under a NON-header name IS counted — so the arm
        // above is the time-like skip, not blindness to nested strings.
        let nested_text = fields(
            "acme/Versions",
            vec![
                f64v("speed", 1.5),
                nested_named(
                    "firmware",
                    fields("acme/Version", vec![str_named("build", "1.2.3")]),
                ),
            ],
        );
        assert!(
            declares_text_fields(&nested_text),
            "a string one hop down under an ordinary name is real text"
        );

        // The two DECLARED narrowings (documented on the predicate): a byte blob
        // is not text, and a `string[]`'s element types are frame-dependent.
        let blob = fields(
            "acme/Frame",
            vec![
                f64v("width", 640.0),
                cerulion_core::codegen::NamedValue {
                    name: "data".to_string(),
                    value: FrameValueKind::Bytes(&[1, 2, 3]),
                },
            ],
        );
        assert!(
            !declares_text_fields(&blob),
            "a uint8[] payload is not text (and a non-UTF-8 string is indistinguishable)"
        );
        let string_array = fields(
            "acme/Names",
            vec![
                f64v("count", 2.0),
                cerulion_core::codegen::NamedValue {
                    name: "names".to_string(),
                    value: FrameValueKind::NestedArray {
                        elements: vec![FrameValueKind::Str("a"), FrameValueKind::Str("b")],
                        raw: &[],
                    },
                },
            ],
        );
        assert!(
            !declares_text_fields(&string_array),
            "a string[] is knowingly NOT counted — its element types vanish when the \
             array is empty, so counting it would make the layout frame-dependent"
        );
        // A FIXED-length `string[N]` of variable strings decodes through the SAME
        // canonical element path, so its decoded value is byte-for-byte the shape
        // above — no arity marker anywhere in it. That walker fact is DOCUMENTED
        // here, not walker-verified (no test in this file walks a fixed
        // `string[N]` blob; `decode_element_array` routes FieldType::String with
        // a declared_length through the same decode_counted_elements). This arm
        // is therefore executable documentation of the predicate's indifference:
        // it is deliberately the same decoded value as the arm above, and catches
        // nothing the arm above does not.
        let fixed_string_array = fields(
            "acme/FixedNames",
            vec![
                f64v("count", 2.0),
                cerulion_core::codegen::NamedValue {
                    name: "names".to_string(),
                    value: FrameValueKind::NestedArray {
                        elements: vec![FrameValueKind::Str("a"), FrameValueKind::Str("b")],
                        raw: &[],
                    },
                },
            ],
        );
        assert!(
            !declares_text_fields(&fixed_string_array),
            "a FIXED-length string[N] decodes to the same NestedArray with no arity \
             marker, so it takes the same narrowing"
        );
        // The THIRD spelling — a `string_fixed[N]` in the fixed section — reaches
        // the predicate as OPAQUE BYTES (the walker declines to decode it), so
        // there are no elements to inspect even in principle.
        let opaque_fixed_strings = fields(
            "acme/PaddedNames",
            vec![
                f64v("count", 2.0),
                cerulion_core::codegen::NamedValue {
                    name: "names".to_string(),
                    value: FrameValueKind::NestedArrayOpaque(b"a\0\0\0b\0\0\0"),
                },
            ],
        );
        assert!(
            !declares_text_fields(&opaque_fixed_strings),
            "an opaque fixed-string array carries no decoded elements at all"
        );
    }

    /// `declares_withheld_series` against a HAND classification of every
    /// curation verdict — the predicate that elects `ScalarsWithText` for a topic
    /// whose plots are curated. Each arm names WHY its shape is (or is not)
    /// frame-invariant, because that is the only rule the predicate follows.
    #[test]
    fn declares_withheld_series_counts_only_the_shape_determined_curation() {
        // (1) A COVARIANCE matrix — decided by the field's name and kind; the
        // element count is never consulted, so the verdict is the same on every
        // frame.
        let cov = FrameValue {
            schema_name: "acme/Estimate".to_string(),
            fields: vec![
                f64v("yaw", 0.5),
                prim_named("pose_covariance", f64_bytes(&[0.0; 9]), 9),
            ],
        };
        assert!(declares_withheld_series(&cov));

        // (1b) The SECOND spelling of the same rule. `is_covariance_field` is
        // asked in TWO arms — a numeric `float64[36]` (above) and a FIXED array
        // of nested structs — because a vendor may declare either. Only the first
        // was pinned, and deleting the latch from the struct-array arm passed the
        // whole suite. 4x2 = 8 series is comfortably under
        // every cap, so this fixture withholds ONLY because it is covariance.
        let mut struct_cov = struct_array("pose_covariance", 4, 2);
        struct_cov.fields.push(f64v("yaw", 0.5));
        assert!(
            declares_withheld_series(&struct_cov),
            "a covariance declared as a struct array is withheld by the same rule"
        );
        // Anti-tautology: the identical shape under any OTHER name plots fine and
        // withholds nothing — so the arm reads the covariance rule, not the shape.
        let mut struct_plain = struct_array("motors", 4, 2);
        struct_plain.fields.push(f64v("yaw", 0.5));
        assert!(!declares_withheld_series(&struct_plain));

        // (2) A FIXED struct array the series cap curates — THE `/lowstate` case.
        // 20 x 12 = 240 series against a 64 budget, and both numbers are schema
        // facts (a fixed array's length, and its element's field set).
        assert!(declares_withheld_series(&struct_array(
            "motor_state",
            20,
            12
        )));

        // (3) A FIXED struct array whose ELEMENT COUNT alone busts MAX_ARRAY_SERIES.
        assert!(declares_withheld_series(&struct_array(
            "wide",
            MAX_ARRAY_SERIES + 1,
            1
        )));

        // (4) An over-depth nested block that declares numbers is NOT elected.
        // The constants LOOK asymmetric (DUMP_MAX_DEPTH 4 vs
        // MAX_NESTED_SERIES_DEPTH 3) but the REACH is identical — the harvest
        // refuses a nested field at its fourth hop and so does the dump — so the
        // dump cannot show `buried` either. Asserted against the RENDERED
        // document in `the_dump_cannot_widen_an_over_depth_block_...` below.
        let deep = FrameValue {
            schema_name: "acme/Deep".to_string(),
            fields: vec![
                f64v("top", 1.0),
                nested_named(
                    "a",
                    FrameValue {
                        schema_name: "acme/A".to_string(),
                        fields: vec![nested_named(
                            "b",
                            FrameValue {
                                schema_name: "acme/B".to_string(),
                                fields: vec![nested_named(
                                    "c",
                                    FrameValue {
                                        schema_name: "acme/C".to_string(),
                                        fields: vec![nested_named(
                                            "d",
                                            FrameValue {
                                                schema_name: "acme/D".to_string(),
                                                fields: vec![f64v("buried", 7.0)],
                                            },
                                        )],
                                    },
                                )],
                            },
                        )],
                    },
                ),
            ],
        };
        assert!(!declares_withheld_series(&deep));

        // (5) The SAME depth carrying only text/bools withholds no NUMBERS, so it
        // does not elect on this predicate (it may still elect on the text predicate).
        let deep_text = FrameValue {
            schema_name: "acme/Deep".to_string(),
            fields: vec![
                f64v("top", 1.0),
                nested_named(
                    "a",
                    FrameValue {
                        schema_name: "acme/A".to_string(),
                        fields: vec![nested_named(
                            "b",
                            FrameValue {
                                schema_name: "acme/B".to_string(),
                                fields: vec![nested_named(
                                    "c",
                                    FrameValue {
                                        schema_name: "acme/C".to_string(),
                                        fields: vec![nested_named(
                                            "d",
                                            FrameValue {
                                                schema_name: "acme/D".to_string(),
                                                fields: vec![cerulion_core::codegen::NamedValue {
                                                    name: "label".to_string(),
                                                    value: FrameValueKind::Str("x"),
                                                }],
                                            },
                                        )],
                                    },
                                )],
                            },
                        )],
                    },
                ),
            ],
        };
        assert!(!declares_withheld_series(&deep_text));

        // (6) The TOTAL backstop is NOT elected either:
        // DUMP_MAX_LINES (200) sits BELOW MAX_TOTAL_SERIES (256), so on a flat
        // wide message the dump ends BEFORE the first withheld field and renders
        // a strict prefix of what the plot already drew. Asserted against the
        // rendered document in `the_dump_cannot_widen_a_flat_wide_message_...`.
        assert!(!declares_withheld_series(&wide_message(
            MAX_TOTAL_SERIES + 1
        )));

        // (7) THE ANTI-TAUTOLOGY CONTROL: a message whose numbers ALL plot
        // withholds nothing. Without this the arms above would pass a predicate
        // hardcoded to `true`.
        assert!(!declares_withheld_series(&wide_message(MAX_TOTAL_SERIES)));
        let twist = FrameValue {
            schema_name: "geometry_msgs/Twist".to_string(),
            fields: vec![
                nested_named("linear", vec3(1.0, 2.0, 3.0)),
                nested_named("angular", vec3(0.1, 0.2, 0.3)),
            ],
        };
        assert!(!declares_withheld_series(&twist));
    }

    /// The TWO withholding classes excluded for FRAME-VARIANCE — each
    /// because its trigger is a LENGTH this frame happens to carry.
    ///
    /// This is not the whole excluded set.
    /// Two further classes are excluded for a DIFFERENT
    /// reason, that the dump cannot show what they withheld: an over-depth block
    /// and a flat over-cap message, each pinned by its own
    /// `the_dump_cannot_widen_…` test. See [`declares_withheld_series`] for the
    /// four-way split.
    ///
    /// This is the frame-invariance contract stated as a test: a topic's layout is
    /// resolved once from whichever frame arrived first, so a predicate that
    /// answered differently on two frames of one topic would log a `TextDocument`
    /// into a topic whose views do not include one. Each arm drives the SAME
    /// schema shape twice — once idle, once busy — and requires ONE answer.
    #[test]
    fn a_length_that_varies_frame_to_frame_never_elects_the_dual_view() {
        let bytes = f64_bytes(&[1.0; MAX_ARRAY_SERIES + 1]);

        // (1) A numeric array over MAX_ARRAY_SERIES. The walker cannot tell a
        // dynamic `float64[]` from a fixed `float64[100]` — both are a PrimArray
        // with no arity marker — so the over-cap frame must not elect what the
        // empty frame would not.
        let scan = |count: usize| FrameValue {
            schema_name: "acme/Scan".to_string(),
            fields: vec![f64v("stamp", 1.0), prim_named("ranges", bytes, count)],
        };
        assert!(
            !declares_withheld_series(&scan(MAX_ARRAY_SERIES + 1)),
            "an over-cap array is REPORTED but must not elect: the next frame's \
             array may be short and elect nothing"
        );
        assert!(
            !declares_withheld_series(&scan(0)),
            "…and neither does empty"
        );
        // Anti-vacuity: that same over-cap frame really IS withholding something —
        // the skip is reported, so the arm above is about the ELECTION, not about
        // the curation failing to notice.
        let (_, skipped) = harvest_series(&scan(MAX_ARRAY_SERIES + 1));
        assert!(
            skipped
                .iter()
                .any(|s| matches!(s, SkippedSeries::Oversized { .. })),
            "the over-cap array is still reported: {skipped:?}"
        );

        // (2) A DYNAMIC element array, which is reported only when NON-EMPTY.
        let elements = |n: usize| FrameValue {
            schema_name: "acme/Plan".to_string(),
            fields: vec![
                f64v("seq", 1.0),
                cerulion_core::codegen::NamedValue {
                    name: "poses".to_string(),
                    value: FrameValueKind::NestedArray {
                        elements: (0..n)
                            .map(|i| {
                                FrameValueKind::Nested(Box::new(FrameValue {
                                    schema_name: "acme/P".to_string(),
                                    fields: vec![f64v("x", i as f64)],
                                }))
                            })
                            .collect(),
                        raw: &[0xAB],
                    },
                },
            ],
        };
        assert!(!declares_withheld_series(&elements(0)));
        assert!(
            !declares_withheld_series(&elements(5)),
            "a populated dynamic element array must answer as the empty one does"
        );

        // (3) The TOTAL backstop reached only WITH array expansions counted. The
        // shape floor here is 1 scalar; everything else is array length.
        let arrays = |count: usize| FrameValue {
            schema_name: "acme/Banks".to_string(),
            fields: (0..8)
                .map(|k| prim_named(&format!("bank{k}"), bytes, count))
                .chain(std::iter::once(f64v("power_v", 24.0)))
                .collect(),
        };
        // 8 x 60 = 480 series > the 256 cap, so the cap really bites …
        let (samples, skipped) = harvest_series(&arrays(60));
        assert_eq!(samples.len(), MAX_TOTAL_SERIES, "the cap bit");
        assert!(
            skipped
                .iter()
                .any(|s| matches!(s, SkippedSeries::TotalCap { .. })),
            "and it is reported: {skipped:?}"
        );
        // … and yet the election refuses, because next frame those arrays may be
        // short and the same message would clear the cap.
        assert!(!declares_withheld_series(&arrays(60)));
        assert!(!declares_withheld_series(&arrays(1)));
    }

    /// **The dump cannot widen an OVER-DEPTH block, so that class
    /// does not elect — asserted against the RENDERED DOCUMENT.**
    ///
    /// Electing `TooDeep` on the rationale that the dump
    /// "expands one hop DEEPER than the harvest reaches" (`DUMP_MAX_DEPTH` = 4 vs
    /// `MAX_NESTED_SERIES_DEPTH` = 3) is FALSE, and the constants are what
    /// make it look true: the harvest ENTERS at `budget = 3` and refuses a nested
    /// field once `budget == 0`, while the dump ENTERS at `depth = 0` and refuses
    /// one once `depth + 1 >= 4` — the same fourth hop. The operator would be told
    /// "they ARE shown in this topic's field dump" about a value present in
    /// neither pane.
    ///
    /// This test reads the document rather than the constants, so a future change
    /// to EITHER ceiling re-opens the question here instead of silently making
    /// the copy true or false.
    #[test]
    fn the_dump_cannot_widen_an_over_depth_block_so_it_does_not_elect() {
        let deep = deep_nested_number_fixture();

        // The harvest genuinely withholds it — REPORTED, so this is about where
        // the value can be READ, not about the curation failing to notice.
        let (samples, skipped) = harvest_series(&deep);
        assert!(
            skipped
                .iter()
                .any(|s| matches!(s, SkippedSeries::TooDeep { .. })),
            "the over-depth block is reported: {skipped:?}"
        );
        assert!(
            !samples.iter().any(|(p, _)| p.ends_with("buried")),
            "and it is NOT plotted: {samples:?}"
        );

        // THE PIN: it is not in the dump either. The dump names the block and
        // says it stopped — which the skip report already said, better.
        let text = structured_dump(&deep);
        assert!(
            !text.contains("buried"),
            "the dump must not be claimed to show what it cannot: {text}"
        );
        assert!(
            text.contains("(nested, not expanded)"),
            "…and it says so plainly: {text}"
        );

        // Therefore: no election, so no pane and no promise.
        assert!(!declares_withheld_series(&deep));
    }

    /// **`contains_numeric`'s `pa.count > 0`
    /// guard, pinned.**
    ///
    /// That guard is what keeps the over-depth REPORT accurate — a deep block whose
    /// only numeric field is an EMPTY array carries no lost telemetry, so naming
    /// it would be the noise the guard exists to prevent. Without this test
    /// nothing covers it: mutating the arm to a
    /// bare `true` passes the rest of the `cerulion_viz` suite.
    ///
    /// Both sides asserted over the same fixture, so neither a widened nor a
    /// narrowed guard survives: a POPULATED deep array reports, an EMPTY one does
    /// not. The empty case is the frame a real robot publishes at startup, so
    /// this is the shape the diagnostic would spam on.
    #[test]
    fn an_over_depth_block_reports_only_when_its_array_actually_carries_numbers() {
        let deep_array = |count: usize| FrameValue {
            schema_name: "acme/Deep".to_string(),
            fields: vec![
                f64v("top", 1.0),
                nested_named(
                    "a",
                    FrameValue {
                        schema_name: "acme/A".to_string(),
                        fields: vec![nested_named(
                            "b",
                            FrameValue {
                                schema_name: "acme/B".to_string(),
                                fields: vec![nested_named(
                                    "c",
                                    FrameValue {
                                        schema_name: "acme/C".to_string(),
                                        fields: vec![nested_named(
                                            "d",
                                            FrameValue {
                                                schema_name: "acme/D".to_string(),
                                                fields: vec![prim_named(
                                                    "buried",
                                                    f64_bytes(&[1.0, 2.0]),
                                                    count,
                                                )],
                                            },
                                        )],
                                    },
                                )],
                            },
                        )],
                    },
                ),
            ],
        };
        let reported = |fv: &FrameValue| {
            harvest_series(fv)
                .1
                .iter()
                .any(|s| matches!(s, SkippedSeries::TooDeep { .. }))
        };

        assert!(
            reported(&deep_array(2)),
            "a populated over-depth array IS lost telemetry and must be named"
        );
        assert!(
            !reported(&deep_array(0)),
            "an EMPTY one is not — the harvest would have dropped nothing, so \
             reporting it is noise (this is the arm a bare `true` guard fails)"
        );
        // The same pair through the ELECTION: neither elects (an over-depth block
        // fails the dump-reach condition whatever its array carries), so the
        // report and the election cannot be conflated.
        assert!(!declares_withheld_series(&deep_array(2)));
        assert!(!declares_withheld_series(&deep_array(0)));
    }

    /// **The dump cannot widen a FLAT over-cap message either.**
    ///
    /// [`DUMP_MAX_LINES`] (200) is BELOW [`MAX_TOTAL_SERIES`] (256), so on a
    /// message whose series are plain scalars the dump stops rendering before it
    /// reaches the first field the cap withheld: it shows a strict PREFIX of what
    /// the plot already drew. Electing this class would add a pane that is
    /// literally less informative than the plot beside it.
    #[test]
    fn the_dump_cannot_widen_a_flat_wide_message_so_it_does_not_elect() {
        let wide = wide_message(MAX_TOTAL_SERIES + 1);
        let (samples, skipped) = harvest_series(&wide);

        // The cap really bit, and `s256` is the field it withheld.
        assert_eq!(samples.len(), MAX_TOTAL_SERIES);
        assert!(
            skipped.contains(&SkippedSeries::TotalCap {
                dropped: 1,
                first_dropped: format!("s{MAX_TOTAL_SERIES}"),
            }),
            "the cap reports the first dropped field: {skipped:?}"
        );

        // THE PIN: the dump does not reach it.
        let text = structured_dump(&wide);
        assert!(
            !text.contains(&format!("`s{MAX_TOTAL_SERIES}`")),
            "the withheld field is past the dump's line ceiling"
        );
        assert!(
            text.contains("truncated"),
            "…and the dump says it stopped: {text}"
        );
        // Anti-tautology: the dump DID render this message — the assertion above
        // is about where it stopped, not about an empty document.
        assert!(text.contains("`s0`"), "the dump rendered the early fields");

        assert!(!declares_withheld_series(&wide));
    }

    /// One step of the path from the root down to a withheld field.
    ///
    /// **A HOP IS NOT A LEVEL, and this is the axis a depth-only oracle
    /// does not sweep.** An `at_depth` that wraps only in plain `Nested` makes the
    /// biconditional exercise exactly one exchange rate
    /// — and the bugs live on the other one: a plain hop costs
    /// the dump ONE row, a struct-array element hop costs TWO (the `[i]` row plus
    /// the member row), while the HARVEST spends one budget hop either way.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum Hop {
        /// A plain nested message: `dump_fields` recurses directly.
        Plain,
        /// A struct-array element: `dump_elements` emits an `[i]` row first.
        Element,
    }

    /// Wrap `field` so it sits below `path` — the fixture generator both
    /// depth-range oracles drive, parameterized by DEPTH *and* PATH KIND.
    ///
    /// Built inside-out, so `path[0]` is the OUTERMOST hop (the one nearest the
    /// root). An `Element` hop wraps in a 2-element fixed array of nested
    /// messages, which is what the walker surfaces a `Type[2]` as.
    fn nest_via(
        field: cerulion_core::codegen::NamedValue<'static>,
        path: &[Hop],
    ) -> FrameValue<'static> {
        let mut fv = FrameValue {
            schema_name: "acme/Leaf".to_string(),
            fields: vec![field],
        };
        for (i, hop) in path.iter().enumerate().rev() {
            fv = match hop {
                Hop::Plain => FrameValue {
                    schema_name: format!("acme/W{i}"),
                    fields: vec![nested_named(&format!("w{i}"), fv)],
                },
                Hop::Element => FrameValue {
                    schema_name: format!("acme/E{i}"),
                    fields: vec![cerulion_core::codegen::NamedValue {
                        name: format!("e{i}"),
                        value: FrameValueKind::Array(vec![
                            FrameValueKind::Nested(Box::new(fv.clone())),
                            FrameValueKind::Nested(Box::new(fv)),
                        ]),
                    }],
                },
            };
        }
        fv
    }

    /// How many plain scalar fields to declare AHEAD of a withheld
    /// field to exhaust the dump's line budget.
    ///
    /// Chosen to sit BELOW [`MAX_TOTAL_SERIES`] (256) so the message earns no
    /// `TotalCap` skip — otherwise the topic would be de-elected for the flat
    /// over-cap reason and the line ceiling would never be the operative one —
    /// while comfortably clearing [`DUMP_MAX_LINES`] (200).
    const LINE_CEILING_PREFIX: usize = 250;

    /// Put `field` behind [`LINE_CEILING_PREFIX`] plain scalars, so it is the
    /// message's LAST field — the BREADTH axis, orthogonal to depth and path
    /// kind. The withheld field stays at dump depth 0, so depth is provably not
    /// what hides it.
    fn behind_a_wide_prefix(
        field: cerulion_core::codegen::NamedValue<'static>,
    ) -> FrameValue<'static> {
        let mut fields: Vec<cerulion_core::codegen::NamedValue<'static>> = (0..LINE_CEILING_PREFIX)
            .map(|i| f64v(&format!("s{i}"), i as f64))
            .collect();
        fields.push(field);
        FrameValue {
            schema_name: "acme/Wide".to_string(),
            fields,
        }
    }

    /// Every path of up to `max_len` hops over both kinds — the sweep the
    /// biconditional runs. Includes the empty path (the field at the root).
    fn all_paths(max_len: usize) -> Vec<Vec<Hop>> {
        let mut out = vec![Vec::new()];
        let mut frontier = vec![Vec::new()];
        for _ in 0..max_len {
            let mut next = Vec::new();
            for p in &frontier {
                for hop in [Hop::Plain, Hop::Element] {
                    let mut q = p.clone();
                    q.push(hop);
                    next.push(q);
                }
            }
            out.extend(next.iter().cloned());
            frontier = next;
        }
        out
    }

    /// **The election rule is ENFORCED across DEPTH, not
    /// asserted at depth 0 and hoped for elsewhere.**
    ///
    /// The election is narrowed to "classes the dump can show" and pinned with
    /// one fixture per class — every one of them at the ROOT. But the dump's
    /// element-member ceiling is TWO hops shallower than `DUMP_MAX_DEPTH` reads
    /// (see [`dump_shows_struct_array_members`]), so a struct array declared 2
    /// hops down is ENUMERATED while every element collapses to the `Type {…}`
    /// placeholder — elected on a promise the dump does not keep, and rendering
    /// the exact placeholder the element enumeration removed because it "told a reader nothing
    /// about the one thing the dump exists to show".
    ///
    /// This drives EVERY elected class across every reachable depth and asserts
    /// the BICONDITIONAL against the RENDERED document: a class elects at a depth
    /// IF AND ONLY IF the withheld value is readable there. A class added later
    /// inherits the check rather than the failure mode.
    ///
    /// No vendored or Go2 schema nests a bank that deep today, so this is a
    /// never-seen-robot bug, not a live one — but `ros2 attach` resolves vendor
    /// schemas off the wire, so the shape is reachable by construction.
    #[test]
    fn the_election_matches_dump_reach_at_every_depth() {
        // A 20x12 bank: over MAX_STRUCT_ARRAY_SERIES, so the curation withholds
        // `f11` from the plot wherever the bank is harvested at all.
        let bank_field = || struct_array("motor_state", 20, 12).fields.remove(0);
        // A covariance matrix: withheld by NAME at every depth.
        let cov_field = || prim_named("pose_covariance", f64_bytes(&[7.5; 9]), 9);

        let mut elected_somewhere = 0usize;
        let mut refused_somewhere = 0usize;

        for path in all_paths(2) {
            // ---- the curated struct array ------------------------------------
            let fv = nest_via(bank_field(), &path);
            let (samples, _) = harvest_series(&fv);
            let text = structured_dump(&fv);
            // `f11` is the last member — never plotted, wherever it sits.
            assert!(
                !samples.iter().any(|(p, _)| p.ends_with("/1/f11")),
                "{path:?}: f11 is never plotted"
            );
            let in_dump = text.contains("`f11` = 111");
            let elects = declares_withheld_series(&fv);
            assert_eq!(
                elects, in_dump,
                "{path:?}: a struct-array topic must elect IF AND ONLY IF the dump \
                 shows the members the curation withheld. Rendered:\n{text}"
            );
            if elects {
                elected_somewhere += 1;
            } else {
                refused_somewhere += 1;
            }

            // ---- the covariance matrix ---------------------------------------
            let fv = nest_via(cov_field(), &path);
            let (samples, _) = harvest_series(&fv);
            let text = structured_dump(&fv);
            assert!(
                !samples.iter().any(|(p, _)| p.contains("pose_covariance")),
                "{path:?}: a covariance is never plotted"
            );
            let in_dump = text.contains("7.5");
            let elects = declares_withheld_series(&fv);
            assert_eq!(
                elects, in_dump,
                "{path:?}: a covariance topic must elect IF AND ONLY IF the dump \
                 shows the matrix. Rendered:\n{text}"
            );
            if elects {
                elected_somewhere += 1;
            } else {
                refused_somewhere += 1;
            }
        }

        // ANTI-VACUITY: the sweep must straddle BOTH answers, or a predicate
        // hardcoded either way would satisfy every arm above.
        assert!(
            elected_somewhere > 0 && refused_somewhere > 0,
            "the sweep must exercise both verdicts (elected {elected_somewhere}, \
             refused {refused_somewhere})"
        );

        // ---- the BREADTH axis ----------------------------------
        //
        // Depth and path kind are the two axes the election MODELS, and the
        // biconditional above holds on both. The dump has a THIRD ceiling it does
        // NOT model — the document-wide line budget — so every elected class is
        // ALSO driven through the breadth shape here, asserted at the actual
        // CURRENT behaviour rather than at the biconditional.
        //
        // Running it inside this sweep, per class, is what makes it durable: a
        // class added to the table above inherits the breadth check instead of
        // silently acquiring an unexamined gap. The claim it pins is weaker on
        // purpose — elects, value absent, truncation DISCLOSED — and
        // `the_line_ceiling_is_a_dimension_the_election_does_not_model` states
        // why that is the right stopping point.
        for (label, fv, marker) in [
            (
                "curated struct array",
                behind_a_wide_prefix(bank_field()),
                "`f11` = 111",
            ),
            (
                "covariance matrix",
                behind_a_wide_prefix(cov_field()),
                "7.5",
            ),
        ] {
            let text = structured_dump(&fv);
            assert!(
                declares_withheld_series(&fv),
                "{label}: the election models DEPTH, and this field's row is at \
                 depth 0 — so it elects"
            );
            assert!(
                !text.contains(marker),
                "{label}: …yet the line budget ends the document before it"
            );
            assert!(
                text.contains("truncated"),
                "{label}: the gap is DISCLOSED — the dump says it stopped"
            );
        }
    }

    /// The LINE ceiling is a third reach dimension, and the
    /// election does not model it — pinned AT the actual current behaviour.**
    ///
    /// The depth sweep asserts a BICONDITIONAL, because for depth the election
    /// and the dump can be made to agree. For breadth they cannot, cheaply: the
    /// line budget depends on field ordering and per-array preview counts, and
    /// predicting it is exactly the kind of derived arithmetic that came back
    /// wrong three times. So this arm asserts the KNOWN GAP instead — elects,
    /// value absent, truncation disclosed — which is a weaker claim than the
    /// biconditional and a true one.
    ///
    /// Its job is to stop the gap from moving silently. If the budget, the
    /// ordering or the marker ever changes, this test names it rather than the
    /// `dump_row_renders` doc quietly rotting into a false guarantee — which is
    /// the exact failure mode this test exists to prevent.
    #[test]
    fn the_line_ceiling_is_a_dimension_the_election_does_not_model() {
        let fv = behind_a_wide_prefix(prim_named("pose_covariance", f64_bytes(&[7.5; 9]), 9));
        let (samples, skipped) = harvest_series(&fv);

        // PRECONDITIONS — the line ceiling must be the OPERATIVE limit, not a
        // by-product of some other de-election.
        assert_eq!(
            samples.len(),
            LINE_CEILING_PREFIX,
            "the prefix plots in full"
        );
        assert!(
            !skipped
                .iter()
                .any(|s| matches!(s, SkippedSeries::TotalCap { .. })),
            "under MAX_TOTAL_SERIES, so no flat-over-cap de-election: {skipped:?}"
        );
        assert!(
            skipped
                .iter()
                .any(|s| matches!(s, SkippedSeries::Covariance { .. })),
            "the covariance IS the message's only withholding: {skipped:?}"
        );

        let text = structured_dump(&fv);
        // THE GAP, stated as it actually is.
        assert!(
            declares_withheld_series(&fv),
            "it elects — its row is at depth 0, and depth is all the gate models"
        );
        assert!(
            !text.contains("7.5"),
            "…yet the matrix never renders: the document ran out of lines first"
        );
        // …and the TWO disclosures that keep the gap visible. Without these the
        // election would be telling an operator to read a pane that silently
        // omits their field; with them the pane says it stopped early and the
        // operator line names the bound.
        assert!(
            text.contains("truncated"),
            "the dump DISCLOSES that it stopped: {text}"
        );
        assert!(
            text.contains("`s0`"),
            "anti-vacuity: the dump did render, it just ran out"
        );

        // ORDERING-dependence, asserted rather than claimed: the same fields with
        // the covariance FIRST render it fine, so this is a budget-exhaustion
        // gap and not a covariance-rendering bug.
        let mut reordered =
            behind_a_wide_prefix(prim_named("pose_covariance", f64_bytes(&[7.5; 9]), 9));
        let last = reordered.fields.pop().expect("the covariance");
        reordered.fields.insert(0, last);
        let text = structured_dump(&reordered);
        assert!(
            text.contains("7.5"),
            "covariance-first renders: the gap is breadth+ordering, not the field"
        );
        assert!(declares_withheld_series(&reordered));
    }

    /// The depth-range oracle above is only meaningful if the depths it sweeps
    /// really do straddle both answers — otherwise a predicate hardcoded either
    /// way would satisfy it. This pins the SHAPE of that sweep for the struct
    /// array: shallow depths elect, deep ones do not, and the boundary is where
    /// [`dump_shows_struct_array_members`] says it is.
    #[test]
    fn the_struct_array_election_boundary_is_where_the_dump_stops_showing_members() {
        let bank =
            |path: &[Hop]| nest_via(struct_array("motor_state", 20, 12).fields.remove(0), path);

        // PLAIN hops cost ONE dump row each, so the bank still shows members at
        // one hop down and stops at two.
        assert!(declares_withheld_series(&bank(&[])));
        assert!(declares_withheld_series(&bank(&[Hop::Plain])));
        let text = structured_dump(&bank(&[Hop::Plain, Hop::Plain]));
        assert!(
            text.contains("element(s)") && text.contains("{…}"),
            "two plain hops down, elements are placeholders: {text}"
        );
        assert!(!text.contains("`f11`"), "…with no members: {text}");
        assert!(!declares_withheld_series(&bank(&[Hop::Plain, Hop::Plain])));

        // An ELEMENT hop costs TWO rows, so ONE of them is already enough to put
        // the bank's members past the ceiling — the R2' CASE B shape (a
        // bank-of-banks at the message ROOT, squarely the /lowstate class), which
        // a depth-only sweep cannot express.
        let case_b = bank(&[Hop::Element]);
        let text = structured_dump(&case_b);
        assert!(
            !text.contains("`f11`"),
            "one element hop already hides the members: {text}"
        );
        assert!(
            !declares_withheld_series(&case_b),
            "CASE B: a bank nested inside a struct-array element must NOT elect — \
             its members render as `Type {{…}}` placeholders"
        );

        // And the arithmetic the gate is written from, stated as an oracle so a
        // change to either ceiling shows up here.
        assert!(dump_shows_struct_array_members(0));
        assert!(dump_shows_struct_array_members(1));
        assert!(!dump_shows_struct_array_members(2));
        assert!(!dump_shows_struct_array_members(3));
    }

    /// The ELECTED classes' half of the same question: for these the dump really
    /// does show values the plot does not, so the copy that points at it is TRUE.
    /// Without this arm the two negative tests above would be satisfied by a
    /// predicate that elected nothing at all.
    #[test]
    fn the_dump_widens_exactly_the_classes_that_elect() {
        // A covariance matrix: ZERO series plotted, real values in the dump.
        let cov = FrameValue {
            schema_name: "acme/Estimate".to_string(),
            fields: vec![
                f64v("yaw", 0.5),
                prim_named("pose_covariance", f64_bytes(&[7.5; 9]), 9),
            ],
        };
        let (samples, _) = harvest_series(&cov);
        assert!(!samples
            .iter()
            .any(|(p, _)| p.starts_with("pose_covariance")));
        let text = structured_dump(&cov);
        assert!(
            text.contains("`pose_covariance`") && text.contains("7.5"),
            "the dump carries the matrix's actual values: {text}"
        );
        assert!(declares_withheld_series(&cov));

        // A curated struct-array bank (the `/lowstate` case): the plot keeps the
        // earliest fields for every element, the dump carries EVERY field for the
        // first `DUMP_ARRAY_PREVIEW` elements — including one the plot dropped.
        let bank = struct_array("motor_state", 20, 12);
        let (samples, _) = harvest_series(&bank);
        assert!(
            !samples.iter().any(|(p, _)| p == "motor_state/1/f11"),
            "the last member is withheld from the plot"
        );
        let text = structured_dump(&bank);
        // element 1 member 11 carries 1 * 100 + 11 = 111 (the fixture's rule).
        assert!(
            text.contains("`f11` = 111"),
            "the dump carries the withheld member's value: {text}"
        );
        assert!(declares_withheld_series(&bank));
    }

    /// The over-depth fixture both dump-reach tests drive: `a/b/c/d/buried`, one
    /// hop past what either the harvest or the dump will expand.
    fn deep_nested_number_fixture() -> FrameValue<'static> {
        FrameValue {
            schema_name: "acme/Deep".to_string(),
            fields: vec![
                f64v("top", 1.0),
                nested_named(
                    "a",
                    FrameValue {
                        schema_name: "acme/A".to_string(),
                        fields: vec![nested_named(
                            "b",
                            FrameValue {
                                schema_name: "acme/B".to_string(),
                                fields: vec![nested_named(
                                    "c",
                                    FrameValue {
                                        schema_name: "acme/C".to_string(),
                                        fields: vec![nested_named(
                                            "d",
                                            FrameValue {
                                                schema_name: "acme/D".to_string(),
                                                fields: vec![f64v("buried", 7.0)],
                                            },
                                        )],
                                    },
                                )],
                            },
                        )],
                    },
                ),
            ],
        }
    }

    #[test]
    fn harvest_series_expands_numeric_arrays_by_index() {
        // sensor_msgs/JointState shape: the joint bank lives in float64[] arrays,
        // which the earlier harvest ignored entirely (→ a text dump).
        let js = FrameValue {
            schema_name: "sensor_msgs/JointState".to_string(),
            fields: vec![
                prim_named("position", f64_bytes(&[0.5, -1.5]), 2),
                prim_named("velocity", f64_bytes(&[0.1]), 1),
            ],
        };
        assert_eq!(
            harvest_series(&js).0,
            vec![
                ("position/0".to_string(), 0.5),
                ("position/1".to_string(), -1.5),
                ("velocity/0".to_string(), 0.1),
            ]
        );
        // An EMPTY array contributes nothing (never a fabricated zero series).
        let empty = FrameValue {
            schema_name: "sensor_msgs/JointState".to_string(),
            fields: vec![prim_named("effort", f64_bytes(&[]), 0)],
        };
        assert!(harvest_series(&empty).0.is_empty());
    }

    #[test]
    fn harvest_series_skips_oversized_arrays_and_covariance_loudly() {
        let long: Vec<f64> = (0..(MAX_ARRAY_SERIES + 1)).map(|i| i as f64).collect();
        let at_cap: Vec<f64> = (0..MAX_ARRAY_SERIES).map(|i| i as f64).collect();
        let fv = FrameValue {
            schema_name: "acme/Bank".to_string(),
            fields: vec![
                prim_named("too_long", f64_bytes(&long), long.len()),
                prim_named("at_cap", f64_bytes(&at_cap), at_cap.len()),
                prim_named("position_covariance", f64_bytes(&[1.0; 9]), 9),
                f64v("temp", 40.0),
            ],
        };
        let (samples, skipped) = harvest_series(&fv);
        // The at-cap array expands in full (boundary INCLUSIVE) and the plain
        // scalar plots; the over-cap array and the covariance matrix do not.
        assert_eq!(samples.len(), MAX_ARRAY_SERIES + 1);
        assert_eq!(samples[0], ("at_cap/0".to_string(), 0.0));
        assert_eq!(samples[MAX_ARRAY_SERIES], ("temp".to_string(), 40.0));
        // Both skips are REPORTED (never silent), each naming its field + count.
        assert_eq!(
            skipped,
            vec![
                SkippedSeries::Oversized {
                    field: "too_long".to_string(),
                    count: MAX_ARRAY_SERIES + 1,
                },
                SkippedSeries::Covariance {
                    field: "position_covariance".to_string(),
                    count: 9,
                },
            ]
        );
        // The operator-facing phrase names the field, the count AND the reason
        // (hand-written expectations — this string is what the sink logs).
        assert_eq!(
            skipped[0].to_string(),
            format!(
                "too_long ({} elements, over the {MAX_ARRAY_SERIES}-series plot cap)",
                MAX_ARRAY_SERIES + 1
            )
        );
        assert_eq!(
            skipped[1].to_string(),
            "position_covariance (9 elements, a covariance matrix)"
        );
        // A healthy frame reports NO skips (anti-tautology).
        assert!(harvest_series(&vec3(1.0, 2.0, 3.0)).1.is_empty());
    }

    #[test]
    fn harvest_series_expands_a_fixed_array_of_nested_messages() {
        // A `geometry_msgs/Point[2]`-shaped field (a FIXED array of fixed-nested
        // messages — the one array shape the walker decodes element-by-element)
        // expands per index, each element recursing under `field/i/...`.
        //
        // The curation changed the ORDER, not the set: the arm now emits FIELD-major
        // (`x` for every element, then `y`, then `z`) so that a struct array over
        // the per-array series cap keeps whole fields for EVERY element instead of
        // a few complete elements. See `regroup_struct_array_field_major`.
        let corners = cerulion_core::codegen::NamedValue {
            name: "corners".to_string(),
            value: FrameValueKind::Array(vec![
                FrameValueKind::Nested(Box::new(vec3(1.0, 2.0, 3.0))),
                FrameValueKind::Nested(Box::new(vec3(4.0, 5.0, 6.0))),
            ]),
        };
        let fv = FrameValue {
            schema_name: "acme/Quad".to_string(),
            fields: vec![corners],
        };
        assert_eq!(
            harvest_series(&fv).0,
            vec![
                ("corners/0/x".to_string(), 1.0),
                ("corners/1/x".to_string(), 4.0),
                ("corners/0/y".to_string(), 2.0),
                ("corners/1/y".to_string(), 5.0),
                ("corners/0/z".to_string(), 3.0),
                ("corners/1/z".to_string(), 6.0),
            ]
        );
    }

    #[test]
    fn scalar_samples_with_skips_reports_nothing_for_a_named_extractor() {
        // A NAMED extractor (Twist here) reads exactly the fields it declares, so
        // it skips nothing by construction — the sink's diagnostic must not fire
        // for it even when the frame carries an unplotted array sibling.
        let long: Vec<f64> = (0..(MAX_ARRAY_SERIES + 1)).map(|i| i as f64).collect();
        let twist = FrameValue {
            schema_name: "geometry_msgs/Twist".to_string(),
            fields: vec![
                nested_named("linear", vec3(1.0, 0.0, 0.0)),
                nested_named("angular", vec3(0.0, 0.0, 0.5)),
                prim_named("extra", f64_bytes(&long), long.len()),
            ],
        };
        let (samples, skipped) = scalar_samples_with_skips(&twist);
        assert_eq!(samples.len(), 6, "the six named Twist components");
        assert!(skipped.is_empty(), "a named extractor declares no skips");
        // The INFERRED path over the same frame DOES report the oversized sibling.
        assert_eq!(harvest_series(&twist).1.len(), 1);
    }

    #[test]
    fn harvest_series_skips_time_like_nesteds_by_schema_and_by_name() {
        // A `stamp`-shaped nested is excluded by its SCHEMA even under an
        // unconventional field name; a conventional `header`/`stamp` name is
        // excluded even with a schema-less producer.
        let by_schema = FrameValue {
            schema_name: "acme/Reading".to_string(),
            fields: vec![
                nested_named(
                    "captured_at",
                    FrameValue {
                        schema_name: "builtin_interfaces/Time".to_string(),
                        fields: vec![u32f("sec", 12), u32f("nanosec", 500)],
                    },
                ),
                f64v("value", 3.5),
            ],
        };
        assert_eq!(
            harvest_series(&by_schema).0,
            vec![("value".to_string(), 3.5)]
        );
        let by_name = FrameValue {
            schema_name: "acme/Reading".to_string(),
            fields: vec![
                nested_named(
                    "stamp",
                    FrameValue {
                        schema_name: "acme/OwnClock".to_string(),
                        fields: vec![u32f("sec", 12)],
                    },
                ),
                f64v("value", 3.5),
            ],
        };
        assert_eq!(harvest_series(&by_name).0, vec![("value".to_string(), 3.5)]);
    }

    #[test]
    fn harvest_series_does_not_plot_bools_or_strings() {
        // The standing decision is preserved: bools/enums are not auto-plotted, so a lone
        // bool message stays an inspectable field dump rather than a 0/1 plot.
        let flags = FrameValue {
            schema_name: "std_msgs/Bool".to_string(),
            fields: vec![
                cerulion_core::codegen::NamedValue {
                    name: "data".to_string(),
                    value: FrameValueKind::Bool(true),
                },
                cerulion_core::codegen::NamedValue {
                    name: "label".to_string(),
                    value: FrameValueKind::Str("on"),
                },
            ],
        };
        assert!(harvest_series(&flags).0.is_empty());
    }

    // ---- The new geometry extractors ------------------------------

    #[test]
    fn nested_point_of_reads_point_and_position_never_bare_xyz() {
        let stamped = FrameValue {
            schema_name: "geometry_msgs/PointStamped".to_string(),
            fields: vec![nested_named("point", vec3(1.0, 2.0, 3.0))],
        };
        assert_eq!(nested_point_of(&stamped), Some([1.0, 2.0, 3.0]));
        let positioned = FrameValue {
            schema_name: "acme/Waypoint".to_string(),
            fields: vec![nested_named("position", vec3(4.0, 5.0, 6.0))],
        };
        assert_eq!(nested_point_of(&positioned), Some([4.0, 5.0, 6.0]));
        // A BARE {x,y,z} is NOT a point here (the standing scalar-bag decision) —
        // but `single_point_of` still accepts it as the render fallback.
        assert_eq!(nested_point_of(&vec3(7.0, 8.0, 9.0)), None);
        assert_eq!(single_point_of(&vec3(7.0, 8.0, 9.0)), Some([7.0, 8.0, 9.0]));
        assert_eq!(single_point_of(&positioned), Some([4.0, 5.0, 6.0]));
    }

    #[test]
    fn rotation_only_of_reads_named_orientation_not_bare_xyzw() {
        // geometry_msgs/QuaternionStamped shape.
        let qs = FrameValue {
            schema_name: "geometry_msgs/QuaternionStamped".to_string(),
            fields: vec![nested_named("quaternion", quat(0.1, 0.2, 0.3, 0.4))],
        };
        assert_eq!(rotation_only_of(&qs), Some([0.1, 0.2, 0.3, 0.4]));
        // A custom {header, orientation} attitude message.
        let att = FrameValue {
            schema_name: "acme/Attitude".to_string(),
            fields: vec![nested_named("orientation", quat(0.0, 0.0, 0.0, 1.0))],
        };
        assert_eq!(rotation_only_of(&att), Some([0.0, 0.0, 0.0, 1.0]));
        // Through a `pose` wrapper (one hop).
        let wrapped = FrameValue {
            schema_name: "acme/Wrapped".to_string(),
            fields: vec![nested_named(
                "pose",
                FrameValue {
                    schema_name: "acme/RotOnly".to_string(),
                    fields: vec![nested_named("rotation", quat(1.0, 0.0, 0.0, 0.0))],
                },
            )],
        };
        assert_eq!(rotation_only_of(&wrapped), Some([1.0, 0.0, 0.0, 0.0]));
        // A BARE top-level {x,y,z,w} is NOT treated as a rotation (standing
        // four-series scalar-bag decision preserved).
        assert_eq!(rotation_only_of(&quat(0.0, 0.0, 0.0, 1.0)), None);
        // Nothing orientation-shaped → None.
        assert_eq!(rotation_only_of(&vec3(1.0, 2.0, 3.0)), None);
    }

    #[test]
    fn planar_pose_of_lifts_theta_to_a_yaw_quaternion() {
        // geometry_msgs/Pose2D shape: {x, y, theta} at top level. theta = π/2 →
        // a +90° yaw: [0, 0, sin(π/4), cos(π/4)] (hand-computed, not read back).
        let p2 = FrameValue {
            schema_name: "geometry_msgs/Pose2D".to_string(),
            fields: vec![
                f64v("x", 2.0),
                f64v("y", -3.0),
                f64v("theta", std::f64::consts::FRAC_PI_2),
            ],
        };
        let parts = planar_pose_of(&p2).expect("planar pose");
        assert_eq!(parts.translation, [2.0, -3.0, 0.0]);
        let q = parts.rotation.expect("yaw quaternion");
        let root_half = std::f32::consts::FRAC_1_SQRT_2;
        assert_eq!([q[0], q[1]], [0.0, 0.0]);
        assert!((q[2] - root_half).abs() < 1e-6, "sin(pi/4), got {}", q[2]);
        assert!((q[3] - root_half).abs() < 1e-6, "cos(pi/4), got {}", q[3]);

        // vision_msgs/Pose2D shape: {position: {x, y}, theta}. theta = 0 → identity.
        let nested_form = FrameValue {
            schema_name: "vision_msgs/Pose2D".to_string(),
            fields: vec![
                nested_named(
                    "position",
                    FrameValue {
                        schema_name: "vision_msgs/Point2D".to_string(),
                        fields: vec![f64v("x", 1.0), f64v("y", 4.0)],
                    },
                ),
                f64v("theta", 0.0),
            ],
        };
        assert_eq!(
            planar_pose_of(&nested_form),
            Some(PoseParts {
                translation: [1.0, 4.0, 0.0],
                rotation: Some([0.0, 0.0, 0.0, 1.0]),
            })
        );
        // No `theta` → not a planar pose (a bare {x,y,z} must never match).
        assert_eq!(planar_pose_of(&vec3(1.0, 2.0, 3.0)), None);
    }

    // ---- ONE spatial ladder ---------------------------------------

    /// THE regression pin for the two ladders drifting apart. A
    /// `{position: {x, y, z}, theta}` matches BOTH the full-pose rule (rule 5,
    /// via the nested `position`) and the planar rule (rule 2, via `theta`).
    /// With two separate ladders the CLASSIFIER takes the planar branch (consuming
    /// `position/x`, `position/y`, `theta`) while `log_pose` takes the full-pose
    /// branch (drawing `[1, 2, 3]` with NO rotation), so:
    ///   - `theta` is excluded from the sibling plots AND drawn nowhere → the
    ///     exact silent-telemetry-drop class the one ladder exists to prevent;
    ///   - `position/z` is drawn as the translation's z AND emitted as a plot
    ///     series → the same number in two places.
    ///
    /// Both halves now come from ONE decision, so the invariant is checkable:
    /// every consumed path is drawn, every drawn field is consumed.
    #[test]
    fn one_ladder_draws_exactly_the_paths_it_reports_consumed() {
        let fv = FrameValue {
            schema_name: "acme/GroundPose3".to_string(),
            fields: vec![
                nested_named("position", vec3(1.0, 2.0, 3.0)),
                f64v("theta", std::f64::consts::FRAC_PI_2),
            ],
        };
        let (primitive, consumed) = infer_spatial_primitive(&fv).expect("spatial");
        // Hand oracle: the yaw-lifted planar pose (z = 0, +90° about +Z), NOT the
        // full 3D translation the renderer used to draw.
        let root_half = std::f32::consts::FRAC_1_SQRT_2;
        match primitive {
            SpatialPrimitive::Transform(parts) => {
                assert_eq!(parts.translation, [1.0, 2.0, 0.0]);
                let q = parts.rotation.expect("theta IS the rotation — it is drawn");
                assert_eq!([q[0], q[1]], [0.0, 0.0]);
                assert!((q[2] - root_half).abs() < 1e-6, "sin(pi/4), got {}", q[2]);
                assert!((q[3] - root_half).abs() < 1e-6, "cos(pi/4), got {}", q[3]);
            }
            other => panic!("expected a yaw-lifted transform, got {other:?}"),
        }
        assert_eq!(consumed, ["position/x", "position/y", "theta"]);
        // `theta` is CONSUMED and DRAWN (it is the rotation above) — never
        // excluded-then-dropped.
        assert!(consumed.iter().any(|p| p == "theta"));
        // `position/z` is NOT drawn (the lift puts the frame on z = 0), so it is
        // NOT consumed and plots EXACTLY once as a sibling series.
        assert!(!consumed.iter().any(|p| p == "position/z"));
        assert_eq!(
            spatial_sibling_series(&fv).0,
            vec![("position/z".to_string(), 3.0)],
            "z is drawn nowhere, so it must be plotted — exactly once"
        );
    }

    /// Every rung of the one ladder, against a hand-written
    /// `(primitive, consumed)` oracle — so a reordering or a mis-reported
    /// consumed set is a failing table row, not a silent drift.
    #[test]
    fn the_spatial_ladder_rungs_oracle() {
        let ident = [0.0f32, 0.0, 0.0, 1.0];
        // 1. full pose WITH a rotation.
        let full = pose_fv([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0]);
        assert_eq!(
            infer_spatial_primitive(&full),
            Some((
                SpatialPrimitive::Transform(PoseParts {
                    translation: [1.0, 2.0, 3.0],
                    rotation: Some(ident),
                }),
                vec!["position".to_string(), "orientation".to_string()]
            ))
        );
        // 2. planar {x, y, theta} (theta = 0 → identity yaw).
        let planar = FrameValue {
            schema_name: "geometry_msgs/Pose2D".to_string(),
            fields: vec![f64v("x", 4.0), f64v("y", 5.0), f64v("theta", 0.0)],
        };
        assert_eq!(
            infer_spatial_primitive(&planar),
            Some((
                SpatialPrimitive::Transform(PoseParts {
                    translation: [4.0, 5.0, 0.0],
                    rotation: Some(ident),
                }),
                vec!["x".to_string(), "y".to_string(), "theta".to_string()]
            ))
        );
        // 3. orientation only.
        let attitude = FrameValue {
            schema_name: "acme/Attitude".to_string(),
            fields: vec![nested_named("orientation", quat(0.0, 0.0, 0.0, 1.0))],
        };
        assert_eq!(
            infer_spatial_primitive(&attitude),
            Some((
                SpatialPrimitive::Rotation(ident),
                vec!["orientation".to_string()]
            ))
        );
        // 4. a NAMED position with no orientation is a POINT.
        let waypoint = FrameValue {
            schema_name: "acme/Waypoint".to_string(),
            fields: vec![nested_named("position", vec3(7.0, 8.0, 9.0))],
        };
        assert_eq!(
            infer_spatial_primitive(&waypoint),
            Some((
                SpatialPrimitive::Point([7.0, 8.0, 9.0]),
                vec!["position".to_string()]
            ))
        );
        // 5. a translation with no rotation is still a rigid motion.
        let shift = FrameValue {
            schema_name: "acme/Shift".to_string(),
            fields: vec![nested_named("translation", vec3(1.0, 0.0, 0.0))],
        };
        assert_eq!(
            infer_spatial_primitive(&shift),
            Some((
                SpatialPrimitive::Transform(PoseParts {
                    translation: [1.0, 0.0, 0.0],
                    rotation: None,
                }),
                vec!["translation".to_string()]
            ))
        );
        // Nothing spatial → None (the value falls to the numeric / text rules).
        assert_eq!(infer_spatial_primitive(&vec3(1.0, 2.0, 3.0)), None);
        // The classifier's view agrees rung-for-rung (a rotation is a transform).
        for (fv, kind) in [
            (&full, SpatialKind::Transform),
            (&planar, SpatialKind::Transform),
            (&attitude, SpatialKind::Transform),
            (&waypoint, SpatialKind::Point),
            (&shift, SpatialKind::Transform),
        ] {
            let (primitive, consumed) = infer_spatial_primitive(fv).expect("spatial");
            assert_eq!(
                infer_spatial_shape(fv),
                Some(SpatialShape { kind, consumed }),
                "the classifier view must be the SAME decision, {primitive:?}"
            );
        }
    }

    /// A NAMED orientation the primitive RENDERS is never re-plotted as four
    /// scalar series (the standing decision), while a SECOND quaternion the
    /// primitive does NOT render is real telemetry and plots rather than
    /// vanishing. The decision governs what may be INFERRED as a rotation, not
    /// whether an un-rendered quaternion may be plotted.
    #[test]
    fn only_the_rendered_quaternion_is_excluded_from_the_plots() {
        let two_quats = FrameValue {
            schema_name: "acme/GoalTracker".to_string(),
            fields: vec![
                nested_named("orientation", quat(0.0, 0.0, 0.0, 1.0)),
                nested_named("goal_orientation", quat(0.5, 0.0, 0.0, 0.5)),
            ],
        };
        let (primitive, consumed) = infer_spatial_primitive(&two_quats).expect("spatial");
        assert_eq!(primitive, SpatialPrimitive::Rotation([0.0, 0.0, 0.0, 1.0]));
        assert_eq!(consumed, ["orientation"], "only the DRAWN one is consumed");
        assert_eq!(
            spatial_sibling_series(&two_quats).0,
            vec![
                ("goal_orientation/x".to_string(), 0.5),
                ("goal_orientation/y".to_string(), 0.0),
                ("goal_orientation/z".to_string(), 0.0),
                ("goal_orientation/w".to_string(), 0.5),
            ],
            "a quaternion nothing draws is plotted, not dropped"
        );
    }

    #[test]
    fn box3d_parts_oracle() {
        // vision_msgs/BoundingBox3D shape: {center: Pose, size: Vector3}.
        let bb = FrameValue {
            schema_name: "vision_msgs/BoundingBox3D".to_string(),
            fields: vec![
                nested_named("center", pose_fv([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0])),
                nested_named("size", vec3(0.5, 0.25, 2.0)),
            ],
        };
        assert_eq!(
            box3d_parts(&bb),
            Some(BoxParts {
                center: [1.0, 2.0, 3.0],
                size: [0.5, 0.25, 2.0],
                rotation: Some([0.0, 0.0, 0.0, 1.0]),
            })
        );
        // moveit_msgs/OrientedBoundingBox shape: {pose, extents}.
        let obb = FrameValue {
            schema_name: "moveit_msgs/OrientedBoundingBox".to_string(),
            fields: vec![
                nested_named("pose", pose_fv([0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])),
                nested_named("extents", vec3(1.0, 1.0, 1.0)),
            ],
        };
        assert_eq!(
            box3d_parts(&obb),
            Some(BoxParts {
                center: [0.0, 0.0, 0.0],
                size: [1.0, 1.0, 1.0],
                rotation: Some([1.0, 0.0, 0.0, 0.0]),
            })
        );
        // A size with no centre, and a centre with no size, are BOTH not boxes.
        let size_only = FrameValue {
            schema_name: "acme/Half".to_string(),
            fields: vec![nested_named("size", vec3(1.0, 1.0, 1.0))],
        };
        assert_eq!(box3d_parts(&size_only), None);
        let center_only = FrameValue {
            schema_name: "acme/Half".to_string(),
            fields: vec![nested_named(
                "center",
                pose_fv([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0]),
            )],
        };
        assert_eq!(box3d_parts(&center_only), None);
    }

    // ---- The occupancy grid ---------------------------------------

    fn occupancy_fv(w: u32, h: u32, data: &'static [u8]) -> FrameValue<'static> {
        FrameValue {
            schema_name: "nav_msgs/OccupancyGrid".to_string(),
            fields: vec![
                nested_named(
                    "info",
                    FrameValue {
                        schema_name: "nav_msgs/MapMetaData".to_string(),
                        fields: vec![u32f("width", w), u32f("height", h)],
                    },
                ),
                cerulion_core::codegen::NamedValue {
                    name: "data".to_string(),
                    value: FrameValueKind::Bytes(data),
                },
            ],
        }
    }

    #[test]
    fn occupancy_to_gray_value_oracle() {
        // 0 free → white; 100 occupied → black; 50 → mid; -1 unknown → 128.
        assert_eq!(occupancy_to_gray(0), 255);
        assert_eq!(occupancy_to_gray(100), 0);
        assert_eq!(occupancy_to_gray(50), 127);
        assert_eq!(occupancy_to_gray(0xFF), 128, "-1 is UNKNOWN");
        assert_eq!(occupancy_to_gray(0x80), 128, "-128 also unknown");
        // A nominally out-of-range positive clamps to fully occupied (total).
        assert_eq!(occupancy_to_gray(120), 0);
    }

    #[test]
    fn occupancy_image_flips_rows_to_rerun_orientation() {
        // A 2x2 grid whose ROS row 0 (the origin row, +y up) is [0, 100] and row
        // 1 is [-1, 50]. rerun row 0 is the TOP, so the emitted buffer must start
        // with ROS row 1. Hand oracle, never a read-back.
        //   ROS row 0 → [255, 0]   (free, occupied)
        //   ROS row 1 → [128, 127] (unknown, mid)
        // emitted (top-first) = row1 then row0.
        let grid = occupancy_image(&occupancy_fv(2, 2, &[0, 100, 0xFF, 50])).expect("grid");
        assert_eq!(grid.width, 2);
        assert_eq!(grid.height, 2);
        assert_eq!(grid.gray, vec![128, 127, 255, 0]);
    }

    #[test]
    fn occupancy_image_refuses_degenerate_and_short_grids() {
        // A cell buffer SHORTER than width*height → None (the caller degrades to
        // an inspectable dump; never a mis-rendered / half-garbage map).
        assert_eq!(occupancy_image(&occupancy_fv(4, 4, &[0, 0, 0])), None);
        // Zero-area grid → None.
        assert_eq!(occupancy_image(&occupancy_fv(0, 5, &[0])), None);
        // Missing `info` → None.
        let no_info = FrameValue {
            schema_name: "nav_msgs/OccupancyGrid".to_string(),
            fields: vec![cerulion_core::codegen::NamedValue {
                name: "data".to_string(),
                value: FrameValueKind::Bytes(&[0u8; 4]),
            }],
        };
        assert_eq!(occupancy_image(&no_info), None);
        // A grid claiming more cells than MAX_OCCUPANCY_PIXELS → None (never a
        // multi-gigabyte allocation from a corrupt header).
        assert_eq!(
            occupancy_image(&occupancy_fv(65_536, 65_536, &[0u8; 8])),
            None
        );
        // A LONGER-than-needed buffer is fine (trailing bytes ignored).
        let over = occupancy_image(&occupancy_fv(1, 1, &[0, 100, 100])).expect("grid");
        assert_eq!(over.gray, vec![255]);
    }

    // ---- The structured field dump --------------------------------

    #[test]
    fn structured_dump_expands_nested_and_previews_arrays() {
        let fv = FrameValue {
            schema_name: "acme/Reading".to_string(),
            fields: vec![
                nested_named(
                    "header",
                    FrameValue {
                        schema_name: "std_msgs/Header".to_string(),
                        fields: vec![cerulion_core::codegen::NamedValue {
                            name: "frame_id".to_string(),
                            value: FrameValueKind::Str("base"),
                        }],
                    },
                ),
                prim_named("samples", f64_bytes(&[1.0, 2.5]), 2),
                cerulion_core::codegen::NamedValue {
                    name: "blob".to_string(),
                    value: FrameValueKind::Bytes(&[0xFF, 0xD8, 0xFF, 0xE0]),
                },
                cerulion_core::codegen::NamedValue {
                    name: "detections".to_string(),
                    value: FrameValueKind::NestedArrayOpaque(&[0u8; 42]),
                },
            ],
        };
        let text = structured_dump(&fv);
        // Hand-written expectations on the rendered document — the readability
        // contract, not a self-compare.
        assert!(text.starts_with("**acme/Reading**\n\n"), "got: {text}");
        // A nested message EXPANDS inline (earlier: `std_msgs/Header { 1 fields }`).
        assert!(
            text.contains("- `header` (std_msgs/Header)\n"),
            "got: {text}"
        );
        assert!(text.contains("  - `frame_id` = \"base\"\n"), "got: {text}");
        // A numeric array shows its type, length AND values (pre: `<2 × F64>`).
        assert!(
            text.contains("- `samples` = float64[2]: [1.0, 2.5]\n"),
            "got: {text}"
        );
        // A byte blob shows its size AND leading hex (a JPEG magic is readable).
        assert!(
            text.contains("- `blob` = 4 bytes (hex: ff d8 ff e0)\n"),
            "got: {text}"
        );
        // The opaque wire shape says WHY it cannot be decoded.
        assert!(
            text.contains("`detections` = 42 bytes — opaque"),
            "got: {text}"
        );
        // The framework DOES define an element framing now, so the
        // dump's reason is that THESE bytes are not written in it.
        assert!(text.contains("not canonically framed"), "got: {text}");
    }

    /// Reported on a live robot: a FIXED array of message
    /// elements — `MotorState[20]`, the joint-bank shape — rendered as
    /// `[unitree_go/MotorState {…}, unitree_go/MotorState {…}, …]`. The walker had
    /// every element's fields decoded and the dump, whose entire job is to make an
    /// undrawable payload INSPECTABLE, showed a placeholder for the one field the
    /// reader opened it for. Its singular sibling (`imu_state`) expanded fully in
    /// the same document, which is what made it read as a bug rather than a limit.
    ///
    /// Hand-written oracle on the rendered document: the ELEMENTS are enumerated
    /// `[i]` with their own fields, `{…}` appears nowhere, and the elision is
    /// DISCLOSED — `DUMP_ARRAY_PREVIEW` elements then a `… N total` line carrying the
    /// real count, so a truncated dump never reads as a complete one.
    #[test]
    fn the_structured_dump_expands_a_fixed_struct_arrays_elements() {
        let motor = |q: f64, temp: f64| {
            FrameValueKind::Nested(Box::new(FrameValue {
                schema_name: "unitree_go/MotorState".to_string(),
                fields: vec![f64v("q", q), f64v("temperature", temp)],
            }))
        };
        let fv = FrameValue {
            schema_name: "unitree_go/LowState".to_string(),
            fields: vec![
                // The SINGULAR nested sibling, which always expanded — kept in the
                // same document because the report is precisely that the
                // two render inconsistently.
                nested_named(
                    "imu_state",
                    FrameValue {
                        schema_name: "unitree_go/IMUState".to_string(),
                        fields: vec![f64v("temperature", 42.0)],
                    },
                ),
                cerulion_core::codegen::NamedValue {
                    name: "motor_state".to_string(),
                    value: FrameValueKind::Array(vec![motor(0.5, 30.0), motor(1.5, 31.0)]),
                },
            ],
        };
        let text = structured_dump(&fv);

        assert!(
            text.contains("- `imu_state` (unitree_go/IMUState)\n")
                && text.contains("  - `temperature` = 42.0\n"),
            "the singular nested sibling still expands: {text}"
        );
        // THE FIX: the element COUNT header, then each element's own fields.
        assert!(
            text.contains("- `motor_state` — 2 element(s)\n"),
            "got: {text}"
        );
        assert!(
            text.contains("  - [0] (unitree_go/MotorState)\n")
                && text.contains("    - `q` = 0.5\n")
                && text.contains("    - `temperature` = 30.0\n"),
            "element 0's OWN fields must be in the document: {text}"
        );
        assert!(
            text.contains("  - [1] (unitree_go/MotorState)\n")
                && text.contains("    - `q` = 1.5\n"),
            "…and element 1's: {text}"
        );
        // The placeholder rendering must be GONE, not merely accompanied.
        assert!(
            !text.contains("{…}"),
            "no element may render as a placeholder: {text}"
        );
    }

    /// The ELISION contract for a wide bank: at most `DUMP_ARRAY_PREVIEW`
    /// elements are expanded and the real count is stated, so 20 motors are
    /// structure rather than a 240-line wall — and rather than a document that
    /// silently claims to be complete.
    #[test]
    fn a_wide_fixed_struct_array_dump_is_elided_honestly() {
        let items: Vec<FrameValueKind<'static>> = (0..20)
            .map(|i| {
                FrameValueKind::Nested(Box::new(FrameValue {
                    schema_name: "unitree_go/MotorState".to_string(),
                    fields: vec![f64v("q", i as f64)],
                }))
            })
            .collect();
        let fv = FrameValue {
            schema_name: "unitree_go/LowState".to_string(),
            fields: vec![cerulion_core::codegen::NamedValue {
                name: "motor_state".to_string(),
                value: FrameValueKind::Array(items),
            }],
        };
        let text = structured_dump(&fv);

        assert!(text.contains("- `motor_state` — 20 element(s)\n"), "{text}");
        // Exactly the preview budget is expanded …
        for i in 0..DUMP_ARRAY_PREVIEW {
            assert!(
                text.contains(&format!("  - [{i}] (")),
                "element {i}: {text}"
            );
        }
        assert!(
            !text.contains(&format!("  - [{DUMP_ARRAY_PREVIEW}] (")),
            "element {DUMP_ARRAY_PREVIEW} is past the preview budget: {text}"
        );
        // … and the tail NAMES the real count, so the elision is disclosed.
        assert!(text.contains("  - … 20 total\n"), "{text}");
        // Anti-vacuity: the expanded elements really carry their own values, so
        // this is an elision of a REAL expansion and not of a placeholder.
        assert!(text.contains("    - `q` = 0.0\n"), "{text}");
        assert!(
            text.contains(&format!(
                "    - `q` = {:?}\n",
                (DUMP_ARRAY_PREVIEW - 1) as f64
            )),
            "{text}"
        );
    }

    /// A fixed array of PRIMITIVES keeps its one-LINE preview — there is no
    /// structure to expand, and `[0.5, 1.5, 2.5]` on one line beats three
    /// bullets. Without this the fix could quietly turn every `float64[3]` into a
    /// three-line block.
    #[test]
    fn a_fixed_primitive_array_dump_stays_a_one_line_preview() {
        let fv = FrameValue {
            schema_name: "acme/Bank".to_string(),
            fields: vec![cerulion_core::codegen::NamedValue {
                name: "corners".to_string(),
                value: FrameValueKind::Array(vec![
                    FrameValueKind::F64(0.5),
                    FrameValueKind::F64(1.5),
                ]),
            }],
        };
        let text = structured_dump(&fv);
        assert!(
            text.contains("- `corners` — 2 element(s): [0.5, 1.5]\n"),
            "got: {text}"
        );
        assert!(!text.contains("[0] "), "no per-element bullets: {text}");
    }

    #[test]
    fn structured_dump_bounds_depth_arrays_strings_and_lines() {
        // Depth: a chain deeper than DUMP_MAX_DEPTH stops with a marker instead
        // of recursing forever.
        let mut deep = FrameValue {
            schema_name: "acme/Leaf".to_string(),
            fields: vec![f64v("v", 1.0)],
        };
        for i in 0..8 {
            deep = FrameValue {
                schema_name: format!("acme/L{i}"),
                fields: vec![nested_named("inner", deep)],
            };
        }
        let text = structured_dump(&deep);
        assert!(text.contains("_(nested, not expanded)_"), "got: {text}");

        // Arrays: only the first DUMP_ARRAY_PREVIEW elements, then the total.
        let long: Vec<f64> = (0..20).map(|i| i as f64).collect();
        let wide = FrameValue {
            schema_name: "acme/Bank".to_string(),
            fields: vec![prim_named("data", f64_bytes(&long), 20)],
        };
        let text = structured_dump(&wide);
        assert!(text.contains("float64[20]"), "got: {text}");
        assert!(text.contains(", …, 20 total]"), "got: {text}");

        // Strings: truncated at a CHARACTER boundary (multi-byte safe).
        let long_s: String = "é".repeat(400);
        let leaked: &'static str = Box::leak(long_s.into_boxed_str());
        let stringy = FrameValue {
            schema_name: "acme/Log".to_string(),
            fields: vec![cerulion_core::codegen::NamedValue {
                name: "msg".to_string(),
                value: FrameValueKind::Str(leaked),
            }],
        };
        let text = structured_dump(&stringy);
        assert!(text.contains('…'), "got: {text}");
        assert!(text.len() < 400, "truncated, got {} bytes", text.len());

        // Lines: a message wider than DUMP_MAX_LINES is truncated + says so.
        let many: Vec<cerulion_core::codegen::NamedValue<'static>> =
            (0..500).map(|i| f64v(&format!("f{i}"), i as f64)).collect();
        let text = structured_dump(&FrameValue {
            schema_name: "acme/Wide".to_string(),
            fields: many,
        });
        assert!(text.contains("_(truncated"), "got: {text}");
        assert!(
            text.lines().count() <= DUMP_MAX_LINES + 4,
            "bounded, got {} lines",
            text.lines().count()
        );

        // An EMPTY message says so instead of rendering a bare header.
        let empty = structured_dump(&FrameValue {
            schema_name: "std_msgs/Empty".to_string(),
            fields: vec![],
        });
        assert_eq!(empty, "**std_msgs/Empty**\n\n_(no fields)_\n");
    }

    #[test]
    fn inferred_scalars_twist_plus_top_level_sibling_is_seven_series() {
        // {linear, angular, battery_v}: the twist-shape must NOT short-circuit
        // its top-level numeric sibling — seven series (six velocity + one
        // battery), in implementation order (linear, angular, then the
        // top-level bag).
        let frame = FrameValue {
            schema_name: "unknown/CmdWithBattery".to_string(),
            fields: vec![
                nested_named("linear", vec3(1.0, 2.0, 3.0)),
                nested_named("angular", vec3(4.0, 5.0, 6.0)),
                f64v("battery_v", 12.4),
            ],
        };
        assert_eq!(
            inferred_scalars(&frame),
            vec![
                ("linear/x".to_string(), 1.0),
                ("linear/y".to_string(), 2.0),
                ("linear/z".to_string(), 3.0),
                ("angular/x".to_string(), 4.0),
                ("angular/y".to_string(), 5.0),
                ("angular/z".to_string(), 6.0),
                ("battery_v".to_string(), 12.4),
            ]
        );
    }

    #[test]
    fn single_string_of_oracle() {
        // Exactly one string, no numerics → Some.
        let s = FrameValue {
            schema_name: "std_msgs/String".to_string(),
            fields: vec![cerulion_core::codegen::NamedValue {
                name: "data".to_string(),
                value: FrameValueKind::Str("hello"),
            }],
        };
        assert_eq!(single_string_of(&s), Some("hello"));
        // A string alongside a numeric → not a text log (the numeric wins as a
        // plot upstream).
        let mixed = FrameValue {
            schema_name: "x/Y".to_string(),
            fields: vec![
                cerulion_core::codegen::NamedValue {
                    name: "label".to_string(),
                    value: FrameValueKind::Str("hi"),
                },
                f64v("value", 1.0),
            ],
        };
        assert_eq!(single_string_of(&mixed), None);
        // Two strings → ambiguous, not a single text log.
        let two = FrameValue {
            schema_name: "diagnostic_msgs/KeyValue".to_string(),
            fields: vec![
                cerulion_core::codegen::NamedValue {
                    name: "key".to_string(),
                    value: FrameValueKind::Str("k"),
                },
                cerulion_core::codegen::NamedValue {
                    name: "value".to_string(),
                    value: FrameValueKind::Str("v"),
                },
            ],
        };
        assert_eq!(single_string_of(&two), None);
    }
    // ========================================================================
    // The element-array ladder (pure oracles).
    //
    // Hand-built `NestedArray` values with hand-written expected geometry — never
    // a self-compare. The wire-frame end of the same ladder (real frames through
    // the real walker into the real sink) is `sink_dispatch_test.rs`.
    // ========================================================================

    /// The `raw` half of a hand-built [`FrameValueKind::NestedArray`]. The
    /// walker carries the field's verbatim bytes beside the
    /// decoded elements; the element ladder reads only `elements`, so a
    /// non-empty-but-arbitrary `raw` here is exactly right — and would catch a
    /// ladder that started guessing from the bytes.
    const RAW_SENTINEL: &[u8] = &[0xAB, 0xCD];

    fn nested_array(
        name: &str,
        elements: Vec<FrameValueKind<'static>>,
    ) -> cerulion_core::codegen::NamedValue<'static> {
        cerulion_core::codegen::NamedValue {
            name: name.to_string(),
            value: FrameValueKind::NestedArray {
                elements,
                raw: RAW_SENTINEL,
            },
        }
    }

    fn element(inner: FrameValue<'static>) -> FrameValueKind<'static> {
        FrameValueKind::Nested(Box::new(inner))
    }

    /// A `std_msgs/Header`-shaped value — what makes an element STAMPED (and so
    /// its array ORDERED).
    fn header_fv(frame_id: &'static str) -> FrameValue<'static> {
        FrameValue {
            schema_name: "std_msgs/Header".to_string(),
            fields: vec![cerulion_core::codegen::NamedValue {
                name: "frame_id".to_string(),
                value: FrameValueKind::Str(frame_id),
            }],
        }
    }

    /// A `geometry_msgs/PoseStamped`-shaped element: `{header, pose}`.
    fn pose_stamped_fv(pos: [f64; 3], rot: [f64; 4]) -> FrameValue<'static> {
        FrameValue {
            schema_name: "geometry_msgs/PoseStamped".to_string(),
            fields: vec![
                nested_named("header", header_fv("map")),
                nested_named("pose", pose_fv(pos, rot)),
            ],
        }
    }

    /// A `geometry_msgs/Point32`-shaped element: a BARE top-level `{x, y, z}`.
    fn bare_point_fv(x: f64, y: f64, z: f64) -> FrameValue<'static> {
        FrameValue {
            schema_name: "geometry_msgs/Point32".to_string(),
            fields: vec![f64v("x", x), f64v("y", y), f64v("z", z)],
        }
    }

    /// A `vision_msgs/BoundingBox3D`-shaped value: `{center: Pose, size: Vector3}`.
    fn bbox_fv(center: [f64; 3], rot: [f64; 4], size: [f64; 3]) -> FrameValue<'static> {
        FrameValue {
            schema_name: "vision_msgs/BoundingBox3D".to_string(),
            fields: vec![
                nested_named("center", pose_fv(center, rot)),
                nested_named("size", vec3(size[0], size[1], size[2])),
            ],
        }
    }

    /// A `vision_msgs/Detection3D`-shaped element: the box lives one hop down in
    /// `bbox`, beside a `header` and an `id` string.
    fn detection_fv(center: [f64; 3], rot: [f64; 4], size: [f64; 3]) -> FrameValue<'static> {
        FrameValue {
            schema_name: "vision_msgs/Detection3D".to_string(),
            fields: vec![
                nested_named("header", header_fv("camera")),
                nested_named("bbox", bbox_fv(center, rot, size)),
                cerulion_core::codegen::NamedValue {
                    name: "id".to_string(),
                    value: FrameValueKind::Str("obj-7"),
                },
            ],
        }
    }

    fn expect_geometry(fv: &FrameValue) -> ElementArrayParts {
        match scan_element_arrays(fv) {
            ElementArrayScan::Geometry(p) => p,
            other => panic!("expected Geometry, got {other:?}"),
        }
    }

    #[test]
    fn element_array_of_stamped_poses_is_an_ordered_path_with_hand_vertices() {
        // THE issue's acceptance shape: a `nav_msgs/Path`-shaped value. Each
        // element carries its own header, so the elements are samples of one thing
        // over time — an ORDERED trajectory.
        let path = FrameValue {
            schema_name: "nav_msgs/Path".to_string(),
            fields: vec![
                nested_named("header", header_fv("map")),
                nested_array(
                    "poses",
                    vec![
                        element(pose_stamped_fv([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0])),
                        element(pose_stamped_fv(
                            [-4.5, 5.25, 6.125],
                            [0.5, -0.5, 0.25, 0.75],
                        )),
                        element(pose_stamped_fv([7.0, 8.0, 9.0], [0.0, 0.0, 0.0, 1.0])),
                    ],
                ),
            ],
        };
        assert_eq!(
            expect_geometry(&path),
            ElementArrayParts {
                field: "poses".to_string(),
                // Hand-written vertices, f64 → f32 narrowed (Rerun is f32).
                geometry: ElementGeometry::Path(vec![
                    [1.0, 2.0, 3.0],
                    [-4.5, 5.25, 6.125],
                    [7.0, 8.0, 9.0],
                ]),
                truncated: 0,
            }
        );
    }

    #[test]
    fn element_array_of_unstamped_poses_is_unordered_points_not_a_path() {
        // THE ordered-vs-unordered discriminator, in its sharpest form: the SAME
        // positions as the path above, elements differing ONLY by the absence of a
        // per-element header (`geometry_msgs/PoseArray`). Connecting these with a
        // line would invent an order the message does not declare.
        let poses = FrameValue {
            schema_name: "geometry_msgs/PoseArray".to_string(),
            fields: vec![
                nested_named("header", header_fv("map")),
                nested_array(
                    "poses",
                    vec![
                        element(pose_fv([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0])),
                        element(pose_fv([-4.5, 5.25, 6.125], [0.5, -0.5, 0.25, 0.75])),
                    ],
                ),
            ],
        };
        assert_eq!(
            expect_geometry(&poses).geometry,
            ElementGeometry::Points(vec![[1.0, 2.0, 3.0], [-4.5, 5.25, 6.125]])
        );
        // The message's OWN `header` does not make its elements stamped — the test
        // above and this one differ only in the ELEMENT's shape.
        assert!(!element_is_stamped(&pose_fv(
            [1.0, 2.0, 3.0],
            [0.0, 0.0, 0.0, 1.0]
        )));
        assert!(element_is_stamped(&pose_stamped_fv(
            [1.0, 2.0, 3.0],
            [0.0, 0.0, 0.0, 1.0]
        )));
    }

    #[test]
    fn element_array_of_bare_points_yields_positions() {
        // `geometry_msgs/Polygon.points` / `nav_msgs/GridCells.cells`: elements are
        // a bare top-level `{x, y, z}`. The scalar spatial ladder deliberately does
        // NOT read a bare xyz MESSAGE (an unmapped xyz topic is usually a
        // velocity), but inside an ARRAY the ambiguity is gone — there is no such
        // thing as an array of velocities drawn at the origin.
        let polygon = FrameValue {
            schema_name: "geometry_msgs/Polygon".to_string(),
            fields: vec![nested_array(
                "points",
                vec![
                    element(bare_point_fv(0.0, 0.0, 0.0)),
                    element(bare_point_fv(1.0, 0.0, 0.0)),
                    element(bare_point_fv(1.0, 1.0, 0.0)),
                ],
            )],
        };
        // Shape alone yields UNORDERED (bare points carry no stamp). A polygon's
        // ring order is semantic, which no element shape can express — that is why
        // `Polygon`/`PolygonStamped` are NAME-mapped to `Path3D` in
        // `crate::sink::classify_schema`, and this assertion is the exact record
        // of what the SHAPE half can and cannot decide.
        assert_eq!(
            expect_geometry(&polygon).geometry,
            ElementGeometry::Points(vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0]])
        );
    }

    #[test]
    fn scan_for_kind_promotes_an_unstamped_polygon_to_an_ordered_path() {
        // The classify-vs-render fix. The SAME unstamped `Polygon.points` the
        // test above reads as `Points` (shape-only) MUST render as an ordered
        // polyline once the caller supplies the CLASSIFIED archetype, because
        // `geometry_msgs/Polygon` is NAME-mapped to `Path3D` (a ring's order is
        // semantic — no element shape carries it).
        let polygon = FrameValue {
            schema_name: "geometry_msgs/Polygon".to_string(),
            fields: vec![nested_array(
                "points",
                vec![
                    element(bare_point_fv(0.0, 0.0, 0.0)),
                    element(bare_point_fv(1.0, 0.0, 0.0)),
                    element(bare_point_fv(1.0, 1.0, 0.0)),
                ],
            )],
        };
        let vertices = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0]];

        // The kind→hint mapping: only Path3D forces ordering.
        assert_eq!(
            crate::sink::classify_schema("geometry_msgs/Polygon"),
            Some(crate::sink::ArchetypeKind::Path3D)
        );
        assert!(crate::sink::ArchetypeKind::Path3D.forces_ordered_elements());
        assert!(!crate::sink::ArchetypeKind::PoseArray3D.forces_ordered_elements());

        // force_ordered = false ⇒ the shape verdict is unchanged (anti-tautology):
        // a genuinely-inferred vendor array with these elements stays points.
        assert_eq!(
            scan_element_arrays_for_kind(&polygon, false)
                .parts()
                .map(|p| p.geometry.clone()),
            Some(ElementGeometry::Points(vertices.clone()))
        );
        // force_ordered = true ⇒ the SAME vertices, now an ordered Path (renders as
        // LineStrips3D). Reverting `into_ordered` leaves this Points and FAILS.
        assert_eq!(
            scan_element_arrays_for_kind(&polygon, true)
                .parts()
                .map(|p| p.geometry.clone()),
            Some(ElementGeometry::Path(vertices))
        );
    }

    #[test]
    fn scan_for_kind_never_reorders_boxes_paths_empty_or_absent() {
        // The promotion is scoped to Points→Path. A detection SET stays Boxes even
        // under force_ordered (a box is never a polyline vertex).
        let detections = FrameValue {
            schema_name: "vision_msgs/Detection3DArray".to_string(),
            fields: vec![nested_array(
                "detections",
                vec![element(detection_fv(
                    [1.0, 2.0, 3.0],
                    [0.0, 0.0, 0.0, 1.0],
                    [0.4, 0.4, 1.8],
                ))],
            )],
        };
        assert!(matches!(
            scan_element_arrays_for_kind(&detections, true)
                .parts()
                .map(|p| &p.geometry),
            Some(ElementGeometry::Boxes(_))
        ));
        // An already-ordered (stamped) path is unchanged under force_ordered.
        let stamped = FrameValue {
            schema_name: "nav_msgs/Path".to_string(),
            fields: vec![nested_array(
                "poses",
                vec![element(pose_stamped_fv(
                    [1.0, 2.0, 3.0],
                    [0.0, 0.0, 0.0, 1.0],
                ))],
            )],
        };
        assert_eq!(
            scan_element_arrays_for_kind(&stamped, true)
                .parts()
                .map(|p| p.geometry.clone()),
            Some(ElementGeometry::Path(vec![[1.0, 2.0, 3.0]]))
        );
        // Empty and Absent scans pass through untouched (no fabricated geometry).
        let empty = FrameValue {
            schema_name: "geometry_msgs/Polygon".to_string(),
            fields: vec![nested_array("points", vec![])],
        };
        assert!(matches!(
            scan_element_arrays_for_kind(&empty, true),
            ElementArrayScan::Empty { .. }
        ));
        let absent = FrameValue {
            schema_name: "acme/NotAnArray".to_string(),
            fields: vec![f64v("voltage", 12.4)],
        };
        assert_eq!(
            scan_element_arrays_for_kind(&absent, true),
            ElementArrayScan::Absent
        );
    }

    #[test]
    fn element_array_of_detections_yields_n_boxes_before_the_pose_rung() {
        // `vision_msgs/Detection3DArray`: the box lives one hop down in each
        // element's `bbox`. Boxes are checked BEFORE poses for the same reason the
        // scalar ladder checks them first — a box's centre alone would degrade the
        // detection set to a size-less point bag.
        let detections = FrameValue {
            schema_name: "vision_msgs/Detection3DArray".to_string(),
            fields: vec![
                nested_named("header", header_fv("camera")),
                nested_array(
                    "detections",
                    vec![
                        element(detection_fv(
                            [1.0, 2.0, 3.0],
                            [0.0, 0.0, 0.0, 1.0],
                            [0.4, 0.4, 1.8],
                        )),
                        element(detection_fv(
                            [-1.0, 0.5, 2.0],
                            [0.0, 0.0, 1.0, 0.0],
                            [2.0, 1.0, 1.5],
                        )),
                    ],
                ),
            ],
        };
        assert_eq!(
            expect_geometry(&detections).geometry,
            ElementGeometry::Boxes(vec![
                BoxParts {
                    center: [1.0, 2.0, 3.0],
                    size: [0.4, 0.4, 1.8],
                    rotation: Some([0.0, 0.0, 0.0, 1.0]),
                },
                BoxParts {
                    center: [-1.0, 0.5, 2.0],
                    size: [2.0, 1.0, 1.5],
                    rotation: Some([0.0, 0.0, 1.0, 0.0]),
                },
            ])
        );
        // A box-shaped element with no `bbox` wrapper works the same way (a
        // hypothetical `BoundingBox3D[]`), so the wrapper is an EXTRA hop, not the
        // only path.
        let bare_boxes = FrameValue {
            schema_name: "acme/BoxList".to_string(),
            fields: vec![nested_array(
                "boxes",
                vec![element(bbox_fv(
                    [5.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0],
                    [1.0, 1.0, 1.0],
                ))],
            )],
        };
        assert!(matches!(
            expect_geometry(&bare_boxes).geometry,
            ElementGeometry::Boxes(ref b) if b.len() == 1 && b[0].center == [5.0, 0.0, 0.0]
        ));
    }

    #[test]
    fn element_array_geometry_is_all_or_nothing() {
        // One element that does not carry the shape sinks the WHOLE array (the
        // walker's own `NestedArray` rule: a half-decoded array is exactly the
        // fabricated value it refuses). The array below is 2 good poses and one
        // element with no reachable translation.
        let ragged = FrameValue {
            schema_name: "acme/Ragged".to_string(),
            fields: vec![nested_array(
                "poses",
                vec![
                    element(pose_fv([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0])),
                    element(FrameValue {
                        schema_name: "acme/NotSpatial".to_string(),
                        fields: vec![f64v("voltage", 12.4)],
                    }),
                    element(pose_fv([4.0, 5.0, 6.0], [0.0, 0.0, 0.0, 1.0])),
                ],
            )],
        };
        assert_eq!(scan_element_arrays(&ragged), ElementArrayScan::Absent);
        // ANTI-TAUTOLOGY: the same array WITHOUT the bad element matches, so the
        // refusal above is the all-or-nothing rule and not a broken extractor.
        let clean = FrameValue {
            schema_name: "acme/Ragged".to_string(),
            fields: vec![nested_array(
                "poses",
                vec![
                    element(pose_fv([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0])),
                    element(pose_fv([4.0, 5.0, 6.0], [0.0, 0.0, 0.0, 1.0])),
                ],
            )],
        };
        assert_eq!(
            expect_geometry(&clean).geometry,
            ElementGeometry::Points(vec![[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
        );
    }

    #[test]
    fn an_empty_element_array_is_empty_not_absent_and_never_classifies() {
        // The THREE-outcome contract. An idle `/plan` (zero poses this frame) is
        // `Empty` — the render arms draw nothing and do NOT degrade to a field
        // dump, so a topic never flips between a polyline and a `TextDocument` as
        // its plan clears and refills. It is deliberately NOT classifiable: with no
        // elements there is no shape, which is exactly why the well-known ROS array
        // types are NAME-mapped.
        let idle = FrameValue {
            schema_name: "nav_msgs/Path".to_string(),
            fields: vec![
                nested_named("header", header_fv("map")),
                cerulion_core::codegen::NamedValue {
                    name: "poses".to_string(),
                    value: FrameValueKind::NestedArray {
                        elements: vec![],
                        raw: &[],
                    },
                },
            ],
        };
        assert_eq!(
            scan_element_arrays(&idle),
            ElementArrayScan::Empty {
                field: "poses".to_string()
            }
        );
        assert!(scan_element_arrays(&idle).parts().is_none());
        // A POPULATED array elsewhere in the same message still wins (pass 1 looks
        // for geometry across every candidate before pass 2 reports an empty one),
        // so one empty array cannot mask a drawable sibling.
        let mixed = FrameValue {
            schema_name: "acme/Both".to_string(),
            fields: vec![
                cerulion_core::codegen::NamedValue {
                    name: "empty_first".to_string(),
                    value: FrameValueKind::NestedArray {
                        elements: vec![],
                        raw: &[],
                    },
                },
                nested_array(
                    "populated",
                    vec![element(pose_fv([9.0, 8.0, 7.0], [0.0, 0.0, 0.0, 1.0]))],
                ),
            ],
        };
        let parts = expect_geometry(&mixed);
        assert_eq!(parts.field, "populated");
        assert_eq!(
            parts.geometry,
            ElementGeometry::Points(vec![[9.0, 8.0, 7.0]])
        );
    }

    #[test]
    fn an_opaque_array_is_absent_and_is_named_for_the_diagnostic() {
        // The rmw case: the walker REFUSED to decode this array's
        // elements, so there is nothing to draw — and the field is named so the
        // sink can say WHY instead of leaving an unexplained text dump.
        let rmw_path = FrameValue {
            schema_name: "nav_msgs/Path".to_string(),
            fields: vec![
                nested_named("header", header_fv("map")),
                cerulion_core::codegen::NamedValue {
                    name: "poses".to_string(),
                    value: FrameValueKind::NestedArrayOpaque(&[0u8; 96]),
                },
            ],
        };
        assert_eq!(scan_element_arrays(&rmw_path), ElementArrayScan::Absent);
        assert_eq!(
            opaque_element_arrays(&rmw_path),
            vec![("poses".to_string(), 96)]
        );
        // A DECODED array is not reported as opaque (the diagnostic must not fire
        // on a healthy topic).
        let decoded = FrameValue {
            schema_name: "nav_msgs/Path".to_string(),
            fields: vec![nested_array(
                "poses",
                vec![element(pose_stamped_fv(
                    [1.0, 2.0, 3.0],
                    [0.0, 0.0, 0.0, 1.0],
                ))],
            )],
        };
        assert!(opaque_element_arrays(&decoded).is_empty());
    }

    #[test]
    fn a_string_array_carries_no_geometry() {
        // `trajectory_msgs/JointTrajectory.joint_names` and diagnostic labels are
        // `NestedArray`s of `Str` elements. They are not geometry, and must not
        // become a polyline at the origin.
        let names = FrameValue {
            schema_name: "acme/Labels".to_string(),
            fields: vec![nested_array(
                "joint_names",
                vec![
                    FrameValueKind::Str("hip"),
                    FrameValueKind::Str("knee"),
                    FrameValueKind::Bytes(&[0xFF]),
                ],
            )],
        };
        assert_eq!(scan_element_arrays(&names), ElementArrayScan::Absent);
    }

    #[test]
    fn the_scan_reaches_one_hop_through_a_nested_field() {
        // `geometry_msgs/PolygonStamped { header, polygon: Polygon { points } }` —
        // every nav2 robot's published footprint. Top-level fields are scanned
        // first, then ONE hop through each non-timestamp nested field, so the path
        // is reported QUALIFIED.
        let stamped = FrameValue {
            schema_name: "geometry_msgs/PolygonStamped".to_string(),
            fields: vec![
                nested_named("header", header_fv("base_link")),
                nested_named(
                    "polygon",
                    FrameValue {
                        schema_name: "geometry_msgs/Polygon".to_string(),
                        fields: vec![nested_array(
                            "points",
                            vec![
                                element(bare_point_fv(0.25, 0.25, 0.0)),
                                element(bare_point_fv(-0.25, 0.25, 0.0)),
                            ],
                        )],
                    },
                ),
            ],
        };
        let parts = expect_geometry(&stamped);
        assert_eq!(parts.field, "polygon/points");
        assert_eq!(
            parts.geometry,
            ElementGeometry::Points(vec![[0.25, 0.25, 0.0], [-0.25, 0.25, 0.0]])
        );
    }

    #[test]
    fn element_render_split_is_the_one_ceiling_oracle() {
        // The ceiling's ARITHMETIC, against hand oracles, at magnitudes
        // no materialized array could reach. Both halves of the ceiling read from
        // this ONE function (`geometry_of_elements` slices `drawn`,
        // `scan_element_arrays` reports `reported`), so a drift between them —
        // which would under- or over-report the drop, the exact silent-truncation
        // failure the report exists to prevent — is impossible by construction.
        const CAP: usize = MAX_ELEMENT_INSTANCES;
        assert_eq!(element_render_split(0), (0, 0));
        assert_eq!(element_render_split(1), (1, 0));
        assert_eq!(element_render_split(CAP - 1), (CAP - 1, 0));
        // The off-by-one boundary: EXACTLY at the cap is fully drawn, not truncated.
        assert_eq!(element_render_split(CAP), (CAP, 0));
        assert_eq!(element_render_split(CAP + 1), (CAP, 1));
        assert_eq!(element_render_split(CAP + 37), (CAP, 37));
        // Saturating by construction — no overflow, no wrap, at any declared length.
        assert_eq!(element_render_split(usize::MAX), (CAP, usize::MAX - CAP));
    }

    #[test]
    fn an_over_cap_element_array_truncates_and_reports_the_remainder() {
        // The render ceiling: the first MAX_ELEMENT_INSTANCES elements are geometry,
        // the rest are REPORTED (the sink warns once per input) — never silently
        // dropped, never unbounded.
        //
        // The cap raise (10 000 -> 300 000) restructured this test rather than
        // weakening it. Two changes, both cost-only:
        //
        // 1. Elements are bare `{x, y, z}` (the cheapest shape that still yields
        //    geometry via `single_point_of`) instead of full `pose_fv`s — a `Pose`
        //    element allocates two extra nested `FrameValue`s and ~7 more `String`s
        //    EACH, which at 300 000 elements is pure harness overhead that proves
        //    nothing this shape does not.
        // 2. The exactly-at-cap arm no longer materializes a SECOND cap-sized
        //    array; that boundary is covered exactly — and for free — by
        //    `element_render_split_is_the_one_ceiling_oracle` above, which reads
        //    the same function this path consumes.
        const OVER: usize = MAX_ELEMENT_INSTANCES + 37;
        let huge = FrameValue {
            schema_name: "acme/HugePlan".to_string(),
            fields: vec![nested_array(
                "poses",
                (0..OVER)
                    .map(|i| element(vec3(i as f64, 0.0, 0.0)))
                    .collect(),
            )],
        };
        let parts = expect_geometry(&huge);
        assert_eq!(parts.truncated, 37, "the remainder is reported exactly");
        match parts.geometry {
            ElementGeometry::Points(ref p) => {
                assert_eq!(p.len(), MAX_ELEMENT_INSTANCES, "capped at the ceiling");
                // Hand oracle on the boundary vertices: the FIRST elements are kept
                // (a plan is walked from its start), not an arbitrary window.
                assert_eq!(p[0], [0.0, 0.0, 0.0]);
                assert_eq!(
                    p[MAX_ELEMENT_INSTANCES - 1],
                    [(MAX_ELEMENT_INSTANCES - 1) as f32, 0.0, 0.0]
                );
            }
            ref other => panic!("expected Points, got {other:?}"),
        }
    }

    #[test]
    fn a_dynamic_element_array_is_reported_by_the_harvest_and_never_plotted() {
        // Earlier a `NestedArray` fell to the harvest's numeric
        // catch-all — plotting nothing AND reporting nothing, so the field was
        // silently invisible to the ladder. It must now be REPORTED and still never
        // expanded (a 5 000-pose plan would be 15 000 series, and a dynamic array's
        // length varies frame to frame so a length-gated expansion would make the
        // classification frame-DEPENDENT).
        let trajectory = FrameValue {
            schema_name: "trajectory_msgs/JointTrajectory".to_string(),
            fields: vec![
                f64v("dt", 0.05),
                nested_array(
                    "points",
                    vec![
                        element(pose_fv([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0])),
                        element(pose_fv([4.0, 5.0, 6.0], [0.0, 0.0, 0.0, 1.0])),
                    ],
                ),
            ],
        };
        let (samples, skipped) = harvest_series(&trajectory);
        // ONLY the real scalar plots — not one series from inside the elements.
        assert_eq!(samples, vec![("dt".to_string(), 0.05)]);
        assert_eq!(
            skipped,
            vec![SkippedSeries::ElementArray {
                field: "points".to_string(),
                count: 2,
            }]
        );
        // The operator-facing phrase names the field, the count and the reason.
        assert_eq!(
            skipped[0].to_string(),
            "points (2 message element(s), a dynamic element array — never expanded into plot \
             series)"
        );
        // An EMPTY array is not withheld data, so it is not reported.
        let empty = FrameValue {
            schema_name: "trajectory_msgs/JointTrajectory".to_string(),
            fields: vec![cerulion_core::codegen::NamedValue {
                name: "points".to_string(),
                value: FrameValueKind::NestedArray {
                    elements: vec![],
                    raw: &[],
                },
            }],
        };
        assert!(harvest_series(&empty).1.is_empty());
        // And an element array never makes the topic a PLOT topic (the guard: a
        // 5 000-pose `/plan` must not classify as `Scalars`).
        assert!(!declares_plottable_series(&FrameValue {
            schema_name: "nav_msgs/Path".to_string(),
            fields: vec![nested_array(
                "poses",
                vec![element(pose_stamped_fv(
                    [1.0, 2.0, 3.0],
                    [0.0, 0.0, 0.0, 1.0]
                ))],
            )],
        }));
    }

    #[test]
    fn the_structured_dump_enumerates_element_array_elements() {
        // The "everything else" arm of the viz ladder: a
        // `visualization_msgs/MarkerArray`, a `DiagnosticStatus[]`, an
        // rmw-undecodable-but-here-decoded array is only INSPECTABLE if the
        // elements are visible. A bare
        // `[2 element(s)]` count line is not enough.
        let fv = FrameValue {
            schema_name: "acme/Detections".to_string(),
            fields: vec![nested_array(
                "items",
                vec![
                    element(FrameValue {
                        schema_name: "acme/Item".to_string(),
                        fields: vec![
                            f64v("score", 0.75),
                            cerulion_core::codegen::NamedValue {
                                name: "label".to_string(),
                                value: FrameValueKind::Str("cone"),
                            },
                        ],
                    }),
                    element(FrameValue {
                        schema_name: "acme/Item".to_string(),
                        fields: vec![f64v("score", 0.25)],
                    }),
                ],
            )],
        };
        let text = structured_dump(&fv);
        // Hand-written expectations on the rendered document.
        assert!(text.contains("- `items` — 2 element(s)\n"), "got: {text}");
        assert!(text.contains("  - [0] (acme/Item)\n"), "got: {text}");
        assert!(text.contains("    - `score` = 0.75\n"), "got: {text}");
        assert!(text.contains("    - `label` = \"cone\"\n"), "got: {text}");
        assert!(text.contains("  - [1] (acme/Item)\n"), "got: {text}");
        assert!(text.contains("    - `score` = 0.25\n"), "got: {text}");

        // A `string[]` element renders as one leaf line (no sub-dump to make).
        let labels = FrameValue {
            schema_name: "acme/Labels".to_string(),
            fields: vec![nested_array(
                "names",
                vec![FrameValueKind::Str("hip"), FrameValueKind::Str("knee")],
            )],
        };
        let text = structured_dump(&labels);
        assert!(text.contains("- `names` — 2 element(s)\n"), "got: {text}");
        assert!(text.contains("  - [0] = \"hip\"\n"), "got: {text}");
        assert!(text.contains("  - [1] = \"knee\"\n"), "got: {text}");

        // Over the preview budget: the shown elements are capped and the REAL total
        // is named, so a truncated dump never reads as a complete one.
        let many = FrameValue {
            schema_name: "acme/Labels".to_string(),
            fields: vec![nested_array(
                "names",
                (0..DUMP_ARRAY_PREVIEW + 5)
                    .map(|_| FrameValueKind::Str("x"))
                    .collect(),
            )],
        };
        let text = structured_dump(&many);
        assert!(
            text.contains(&format!(
                "- `names` — {} element(s)\n",
                DUMP_ARRAY_PREVIEW + 5
            )),
            "got: {text}"
        );
        assert!(
            text.contains(&format!("  - [{}] = \"x\"\n", DUMP_ARRAY_PREVIEW - 1)),
            "the last previewed element is shown, got: {text}"
        );
        assert!(
            !text.contains(&format!("  - [{}] =", DUMP_ARRAY_PREVIEW)),
            "the first element PAST the budget is not shown, got: {text}"
        );
        assert!(
            text.contains(&format!("  - … {} total\n", DUMP_ARRAY_PREVIEW + 5)),
            "got: {text}"
        );
    }
}

// SPDX-License-Identifier: AGPL-3.0-only
//! Pure PointCloud2 → 3D-points codec.
//!
//! A `sensor_msgs/PointCloud2` packs N points into one flat `data: uint8[]`
//! blob: each point is `point_step` bytes, and the `fields` list says where
//! each named channel (x/y/z/intensity/…) lives inside a point and how it is
//! typed. This module turns `(fields, data, point_step, n_points)` into
//! `Vec<[f32; 3]>` positions plus an optional per-point RGBA colour derived
//! from an `intensity` channel — with **zero** Rerun (or transport)
//! dependency, so it is exhaustively oracle-testable.
//!
//! # `fields` sourcing
//!
//! `PointCloud2.fields` is a `PointField[]` — a `DynamicArray<Nested>`. The
//! in-repo producer writes a BESPOKE packed layout through
//! `set_fields_bytes`, which is what this module reads.
//!
//! The framework has a CANONICAL element framing for this shape
//! (`u32 count` + per element `u32 len` + headerless sub-frame, since
//! `PointField` carries a `string name`), so a `fields` blob written that way
//! — by the `ros2 attach` / `dds_bridge` CDR codec — is now DECODED by
//! `cerulion_core::codegen::FrameWalker` into a `NestedArray`. The bespoke
//! layout below does NOT satisfy that framing (no count prefix, no
//! per-element length) and stays `NestedArrayOpaque`, pinned by
//! `frame_walker.rs`'s `packed_point_fields_bespoke_blob_stays_opaque`.
//! Either way `archetype::cloud_from_frame_value` hands THIS module the field's
//! raw bytes, so a canonically-framed cloud reaches
//! [`parse_point_fields`] and is reported as undecodable rather than
//! silently inferred; the walker's decoded elements are not consumed
//! here. So this module offers two descriptor sources:
//!
//! - [`parse_point_fields`] decodes the DEFINED packed layout — the
//!   sink↔driver contract. The in-repo producer is the `dds_bridge`
//!   node (`examples/go2/nodes/dds_bridge/src/mapping.rs::encode_point_fields`,
//!   this fn's byte-layout MIRROR: a layout change on either side MUST be
//!   mirrored on the other; each side pins the bytes with its own hand
//!   oracle, so a one-sided edit fails a test).
//! - [`infer_fields_from_point_step`] returns the standard XYZ / XYZI
//!   `float32` layout inferred from `point_step`, for empty-`fields`
//!   producers (e.g. pre-bridge captures). Callers using it MUST log loudly
//!   (it is a documented assumption, not decoded truth).
//!
//! Either way, the pure [`decode_pointcloud`] core is exact and tested.

/// ROS2 `sensor_msgs/PointField` datatype codes (the `PointField.datatype`
/// constant block).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PointDatatype {
    Int8,
    Uint8,
    Int16,
    Uint16,
    Int32,
    Uint32,
    Float32,
    Float64,
}

impl PointDatatype {
    /// Map a raw ROS `datatype` code (1..=8) to the enum.
    pub fn from_code(code: u8) -> Option<Self> {
        Some(match code {
            1 => Self::Int8,
            2 => Self::Uint8,
            3 => Self::Int16,
            4 => Self::Uint16,
            5 => Self::Int32,
            6 => Self::Uint32,
            7 => Self::Float32,
            8 => Self::Float64,
            _ => return None,
        })
    }

    /// Byte width of one value of this datatype.
    pub const fn size(self) -> usize {
        match self {
            Self::Int8 | Self::Uint8 => 1,
            Self::Int16 | Self::Uint16 => 2,
            Self::Int32 | Self::Uint32 | Self::Float32 => 4,
            Self::Float64 => 8,
        }
    }

    /// Read one value of this datatype from `b` (must be at least
    /// [`Self::size`] bytes) as an `f32`, honoring `big_endian` (the cloud's
    /// `is_bigendian` flag — the LE/BE dispatch precedent of
    /// `cerulion_go2_dds::cdr::decode_xyz_points`). Returns `None` if the slice
    /// is too short. Single-byte types (`Int8`/`Uint8`) are endianness-free.
    fn read_f32(self, b: &[u8], big_endian: bool) -> Option<f32> {
        if b.len() < self.size() {
            return None;
        }
        Some(match self {
            Self::Int8 => (b[0] as i8) as f32,
            Self::Uint8 => b[0] as f32,
            Self::Int16 => {
                (if big_endian {
                    i16::from_be_bytes([b[0], b[1]])
                } else {
                    i16::from_le_bytes([b[0], b[1]])
                }) as f32
            }
            Self::Uint16 => {
                (if big_endian {
                    u16::from_be_bytes([b[0], b[1]])
                } else {
                    u16::from_le_bytes([b[0], b[1]])
                }) as f32
            }
            Self::Int32 => {
                (if big_endian {
                    i32::from_be_bytes([b[0], b[1], b[2], b[3]])
                } else {
                    i32::from_le_bytes([b[0], b[1], b[2], b[3]])
                }) as f32
            }
            Self::Uint32 => {
                (if big_endian {
                    u32::from_be_bytes([b[0], b[1], b[2], b[3]])
                } else {
                    u32::from_le_bytes([b[0], b[1], b[2], b[3]])
                }) as f32
            }
            Self::Float32 => {
                if big_endian {
                    f32::from_be_bytes([b[0], b[1], b[2], b[3]])
                } else {
                    f32::from_le_bytes([b[0], b[1], b[2], b[3]])
                }
            }
            Self::Float64 => {
                (if big_endian {
                    f64::from_be_bytes([b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]])
                } else {
                    f64::from_le_bytes([b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]])
                }) as f32
            }
        })
    }
}

/// One decoded point-cloud channel descriptor (mirrors `sensor_msgs/PointField`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PointFieldDesc {
    pub name: String,
    /// Byte offset of this channel within one point (`point_step`-sized).
    pub offset: u32,
    /// Raw ROS datatype code (1..=8; see [`PointDatatype::from_code`]).
    pub datatype: u8,
    /// Element count (1 for scalar channels).
    pub count: u32,
}

/// The result of decoding a cloud.
#[derive(Debug, Clone, PartialEq)]
pub struct DecodedCloud {
    /// Finite XYZ positions, one per surviving point.
    pub positions: Vec<[f32; 3]>,
    /// Per-point RGBA colour derived from the `intensity` channel, aligned
    /// 1:1 with `positions`. `None` when the cloud has no intensity channel.
    pub colors: Option<Vec<[u8; 4]>>,
    /// Count of points skipped (out-of-bounds slice, non-finite XYZ, or
    /// unreadable datatype) — surfaced so the caller can log loudly.
    pub skipped: usize,
}

/// Error decoding the DEFINED packed `fields` blob (see module docs).
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum PointFieldParseError {
    #[error("point-field record {index} truncated at byte {at} (blob len {len})")]
    Truncated { index: usize, at: usize, len: usize },
    #[error("point-field record {index} has non-UTF-8 name bytes")]
    BadName { index: usize },
}

/// Fixed intensity normalisation divisor. Unitree L1 intensity is a
/// 0..=255-ish reflectivity; dividing by this and clamping to `[0, 1]` gives
/// a deterministic ramp input. A future auto-ranged pass is out of scope for
/// the demo (this is a documented default, not decoded truth).
pub const INTENSITY_SCALE: f32 = 255.0;

/// Decode the DEFINED packed `fields` layout: a sequence of records, each
///
/// ```text
/// [name_len: u32 LE][name: name_len bytes][offset: u32 LE][datatype: u8][count: u32 LE]
/// ```
///
/// This is the encoding the Go2 driver must write into
/// `PointCloud2.fields` for the sink to decode channels precisely. An empty
/// blob yields an empty vec (Ok). A truncated / non-UTF-8 record is a loud
/// `Err` (never a silent partial read).
pub fn parse_point_fields(blob: &[u8]) -> Result<Vec<PointFieldDesc>, PointFieldParseError> {
    let mut out = Vec::new();
    let mut cur = 0usize;
    let mut index = 0usize;
    while cur < blob.len() {
        let need_hdr = cur + 4;
        if need_hdr > blob.len() {
            return Err(PointFieldParseError::Truncated {
                index,
                at: cur,
                len: blob.len(),
            });
        }
        let name_len =
            u32::from_le_bytes([blob[cur], blob[cur + 1], blob[cur + 2], blob[cur + 3]]) as usize;
        cur += 4;
        // name + offset(4) + datatype(1) + count(4)
        let rec_end = cur.checked_add(name_len).and_then(|v| v.checked_add(9));
        let rec_end = match rec_end {
            Some(e) if e <= blob.len() => e,
            _ => {
                return Err(PointFieldParseError::Truncated {
                    index,
                    at: cur,
                    len: blob.len(),
                })
            }
        };
        let name = std::str::from_utf8(&blob[cur..cur + name_len])
            .map_err(|_| PointFieldParseError::BadName { index })?
            .to_string();
        let mut p = cur + name_len;
        let offset = u32::from_le_bytes([blob[p], blob[p + 1], blob[p + 2], blob[p + 3]]);
        p += 4;
        let datatype = blob[p];
        p += 1;
        let count = u32::from_le_bytes([blob[p], blob[p + 1], blob[p + 2], blob[p + 3]]);
        out.push(PointFieldDesc {
            name,
            offset,
            datatype,
            count,
        });
        cur = rec_end;
        index += 1;
    }
    Ok(out)
}

/// The documented default channel layout for an empty / undecodable `fields`
/// blob: `x`,`y`,`z` as `float32` at offsets 0/4/8, plus `intensity` as
/// `float32` at offset 12 when `point_step >= 16`. Callers MUST log a loud
/// warning when they fall back to this (it is an assumption about the
/// producer, not decoded metadata).
pub fn infer_fields_from_point_step(point_step: u32) -> Vec<PointFieldDesc> {
    // Raw ROS `PointField.datatype` code for FLOAT32.
    let f32_code: u8 = 7;
    let mut fields = vec![
        PointFieldDesc {
            name: "x".into(),
            offset: 0,
            datatype: f32_code,
            count: 1,
        },
        PointFieldDesc {
            name: "y".into(),
            offset: 4,
            datatype: f32_code,
            count: 1,
        },
        PointFieldDesc {
            name: "z".into(),
            offset: 8,
            datatype: f32_code,
            count: 1,
        },
    ];
    if point_step >= 16 {
        fields.push(PointFieldDesc {
            name: "intensity".into(),
            offset: 12,
            datatype: f32_code,
            count: 1,
        });
    }
    fields
}

/// Pure outcome of one [`FieldsWarnLatch`] inference observation — the
/// caller maps it to a log level (the latch never logs itself).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FieldsLogAction {
    /// First inference of a regime — log at WARN (the loud first signal).
    WarnFirst,
    /// Sustained inference — log at DEBUG carrying the running suppressed
    /// count (the flood-suppression arm; ≈15 Hz at lidar rate, measured on the
    /// wire).
    DebugSuppressed { suppressed: u64 },
}

/// Once-per-regime flood latch for the `fields`-layout-inference warning —
/// the `DrainWarnLatch` / `OutputDiscardLatch` house pattern.
/// Contract: first inference of a regime → WARN;
/// repeats → DEBUG with a running suppressed count; recovery (a decodable
/// non-empty `fields` blob appears) → one INFO carrying the total suppressed
/// count — ONLY when `suppressed > 0` (a lone-warn regime re-arms silently)
/// — then re-arm, so the next inference WARNs again.
///
/// The [`inferred_total`](Self::inferred_total) counter is UNCONDITIONAL:
/// it bumps on every inference regardless of the log-level regime and is
/// never reset (Principle #3 queryability).
///
/// Pure state machine (no logging, no clock) — oracle-tested below.
#[derive(Debug, Clone)]
pub struct FieldsWarnLatch {
    /// True when the next inference should WARN (fresh latch, or re-armed
    /// by a recovery).
    armed: bool,
    /// Inferences suppressed (DEBUG-downgraded) in the CURRENT regime.
    suppressed: u64,
    /// Unconditional total inference count across all regimes.
    pub inferred_total: u64,
}

impl FieldsWarnLatch {
    pub const fn new() -> Self {
        Self {
            armed: true,
            suppressed: 0,
            inferred_total: 0,
        }
    }

    /// Record one layout inference; returns how to log it.
    pub fn on_inferred(&mut self) -> FieldsLogAction {
        self.inferred_total += 1;
        if self.armed {
            self.armed = false;
            self.suppressed = 0;
            FieldsLogAction::WarnFirst
        } else {
            self.suppressed += 1;
            FieldsLogAction::DebugSuppressed {
                suppressed: self.suppressed,
            }
        }
    }

    /// Record a decodable (non-inferred) `fields` blob. Returns
    /// `Some(suppressed_count)` when a suppressed regime just healed (the
    /// caller logs ONE recovery `info!`), `None` otherwise (already armed,
    /// or a lone-warn regime — silent re-arm, no doubled log volume on an
    /// every-other-frame flapper).
    pub fn on_decoded(&mut self) -> Option<u64> {
        if self.armed {
            return None;
        }
        self.armed = true;
        let suppressed = self.suppressed;
        self.suppressed = 0;
        (suppressed > 0).then_some(suppressed)
    }
}

impl Default for FieldsWarnLatch {
    fn default() -> Self {
        Self::new()
    }
}

/// Resolve the channel descriptors for a cloud: decode the DEFINED packed
/// `fields` blob when it carries records, otherwise fall back to the
/// point_step-inferred XYZ/XYZI default. The inference is a documented
/// assumption (not decoded truth), so it logs — through `latch`, the
/// once-per-regime flood latch (first inference of a regime WARNs, repeats
/// are DEBUG with a running count, recovery INFOs once + re-arms; see
/// [`FieldsWarnLatch`]). Shared by the typed lidar sink (which holds its
/// latch as node state) and the generic
/// [`FrameValue`](cerulion_core::codegen::FrameValue) dispatch so both
/// sourcing paths agree.
pub fn resolve_fields(
    fields_blob: &[u8],
    point_step: u32,
    latch: &mut FieldsWarnLatch,
) -> Vec<PointFieldDesc> {
    // `Some(parse_err)` = inference needed (`None` inside = empty blob).
    let parse_err: Option<PointFieldParseError> = match parse_point_fields(fields_blob) {
        Ok(fields) if !fields.is_empty() => {
            if let Some(suppressed) = latch.on_decoded() {
                tracing::info!(
                    suppressed_count = suppressed,
                    "PointCloud2 `fields` blob decodable again — point_step layout \
                     inference no longer needed"
                );
            }
            return fields;
        }
        Ok(_) => None,
        Err(e) => Some(e),
    };
    match latch.on_inferred() {
        FieldsLogAction::WarnFirst => match &parse_err {
            None => tracing::warn!(
                point_step,
                "PointCloud2 `fields` empty — inferring the standard XYZ/XYZI float32 \
                 layout from point_step (documented default). Repeats log at debug until a \
                 decodable blob appears"
            ),
            Some(e) => tracing::warn!(
                error = %e,
                point_step,
                "PointCloud2 `fields` blob undecodable — inferring the layout from \
                 point_step. Repeats log at debug until a decodable blob appears"
            ),
        },
        FieldsLogAction::DebugSuppressed { suppressed } => tracing::debug!(
            suppressed,
            point_step,
            undecodable = parse_err.is_some(),
            "PointCloud2 `fields` layout inference sustained (warn suppressed)"
        ),
    }
    infer_fields_from_point_step(point_step)
}

/// Compute the point count for a cloud from its geometry, clamped to what the
/// `data` blob can actually hold (so a lying `width`/`height` never over-reads).
pub fn point_count(width: u32, height: u32, point_step: u32, data_len: usize) -> usize {
    let by_bytes = if point_step > 0 {
        data_len / point_step as usize
    } else {
        0
    };
    let declared = (width as usize).saturating_mul((height as usize).max(1));
    if declared > 0 {
        declared.min(by_bytes)
    } else {
        by_bytes
    }
}

/// Look up a channel by name, returning `(offset, datatype)` if present and
/// its datatype code is valid.
fn channel(fields: &[PointFieldDesc], name: &str) -> Option<(u32, PointDatatype)> {
    fields
        .iter()
        .find(|f| f.name == name)
        .and_then(|f| PointDatatype::from_code(f.datatype).map(|dt| (f.offset, dt)))
}

/// Decode `n_points` points out of `data` using the channel descriptors,
/// honoring `big_endian` (the cloud's `is_bigendian` flag — the bridge
/// preserves it faithfully, so the sink must not hardcode LE).
///
/// Pure and total: any point whose XYZ slice is out of bounds, whose XYZ is
/// non-finite, or whose datatype is unreadable is SKIPPED (counted in
/// [`DecodedCloud::skipped`]) — never a panic, never a fabricated point. If
/// x/y/z are not all present, returns empty positions with every point
/// counted as skipped (the caller logs the missing-channel case).
pub fn decode_pointcloud(
    fields: &[PointFieldDesc],
    data: &[u8],
    point_step: u32,
    n_points: usize,
    big_endian: bool,
) -> DecodedCloud {
    let step = point_step as usize;
    let (Some((ox, dx)), Some((oy, dy)), Some((oz, dz))) = (
        channel(fields, "x"),
        channel(fields, "y"),
        channel(fields, "z"),
    ) else {
        // No usable XYZ channels — nothing decodable.
        return DecodedCloud {
            positions: Vec::new(),
            colors: None,
            skipped: n_points,
        };
    };
    let intensity = channel(fields, "intensity");

    let mut positions = Vec::new();
    let mut colors: Option<Vec<[u8; 4]>> = intensity.map(|_| Vec::new());
    let mut skipped = 0usize;

    for i in 0..n_points {
        let base = match i.checked_mul(step) {
            Some(b) => b,
            None => {
                skipped += 1;
                continue;
            }
        };
        // The point's whole `point_step` window must fit in `data`.
        if step == 0 || base.checked_add(step).is_none_or(|e| e > data.len()) {
            skipped += 1;
            continue;
        }
        let point = &data[base..base + step];
        let x = read_channel(point, ox, dx, big_endian);
        let y = read_channel(point, oy, dy, big_endian);
        let z = read_channel(point, oz, dz, big_endian);
        let (Some(x), Some(y), Some(z)) = (x, y, z) else {
            skipped += 1;
            continue;
        };
        if !(x.is_finite() && y.is_finite() && z.is_finite()) {
            skipped += 1;
            continue;
        }
        positions.push([x, y, z]);
        if let (Some((oi, di)), Some(cols)) = (intensity, colors.as_mut()) {
            let t = read_channel(point, oi, di, big_endian)
                .map(|v| (v / INTENSITY_SCALE).clamp(0.0, 1.0))
                .unwrap_or(0.0);
            cols.push(intensity_to_rgba(t));
        }
    }

    DecodedCloud {
        positions,
        colors,
        skipped,
    }
}

/// Read a channel value at `offset` within a single point slice, honoring
/// `big_endian` (the cloud's `is_bigendian` flag).
fn read_channel(point: &[u8], offset: u32, dt: PointDatatype, big_endian: bool) -> Option<f32> {
    let off = offset as usize;
    point.get(off..).and_then(|b| dt.read_f32(b, big_endian))
}

/// Map a normalised intensity `t ∈ [0, 1]` to an opaque RGBA colour via a
/// deterministic 3-stop ramp: blue (0.0) → green (0.5) → red (1.0). Pure and
/// oracle-tested so the colour output is stable across runs (Principle #7).
pub fn intensity_to_rgba(t: f32) -> [u8; 4] {
    let t = t.clamp(0.0, 1.0);
    let (r, g, b) = if t < 0.5 {
        // blue → green
        let u = t / 0.5;
        (0.0, u, 1.0 - u)
    } else {
        // green → red
        let u = (t - 0.5) / 0.5;
        (u, 1.0 - u, 0.0)
    };
    [
        (r * 255.0).round() as u8,
        (g * 255.0).round() as u8,
        (b * 255.0).round() as u8,
        255,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn f32le(v: f32) -> [u8; 4] {
        v.to_le_bytes()
    }

    /// Build a `data` blob of `point_step`-byte points from XYZ(+optional I)
    /// float32 values at offsets 0/4/8(/12).
    fn build_data(points: &[[f32; 4]], point_step: usize, with_intensity: bool) -> Vec<u8> {
        let mut data = vec![0u8; points.len() * point_step];
        for (i, p) in points.iter().enumerate() {
            let base = i * point_step;
            data[base..base + 4].copy_from_slice(&f32le(p[0]));
            data[base + 4..base + 8].copy_from_slice(&f32le(p[1]));
            data[base + 8..base + 12].copy_from_slice(&f32le(p[2]));
            if with_intensity {
                data[base + 12..base + 16].copy_from_slice(&f32le(p[3]));
            }
        }
        data
    }

    #[test]
    fn xyz_only_decodes_all_points() {
        let fields = infer_fields_from_point_step(12);
        let pts = [[1.0f32, 2.0, 3.0, 0.0], [-4.0, 5.0, -6.0, 0.0]];
        let data = build_data(&pts, 12, false);
        let out = decode_pointcloud(&fields, &data, 12, 2, false);
        assert_eq!(out.positions, vec![[1.0, 2.0, 3.0], [-4.0, 5.0, -6.0]]);
        assert_eq!(out.colors, None);
        assert_eq!(out.skipped, 0);
    }

    #[test]
    fn xyzi_decodes_positions_and_colors() {
        let fields = infer_fields_from_point_step(16);
        // intensity 0.0 → blue, 255.0 → red (after /255 clamp).
        let pts = [[1.0f32, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 255.0]];
        let data = build_data(&pts, 16, true);
        let out = decode_pointcloud(&fields, &data, 16, 2, false);
        assert_eq!(out.positions, vec![[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]);
        let cols = out.colors.expect("intensity → colors");
        assert_eq!(cols[0], [0, 0, 255, 255]); // t=0 → blue
        assert_eq!(cols[1], [255, 0, 0, 255]); // t=1 → red
        assert_eq!(out.skipped, 0);
    }

    #[test]
    fn padded_stride_reads_only_declared_offsets() {
        // point_step 32 but XYZ still at 0/4/8; trailing 20 bytes are padding.
        let fields = infer_fields_from_point_step(12); // xyz-only descriptors
        let pts = [[7.0f32, 8.0, 9.0, 0.0]];
        let data = build_data(&pts, 32, false);
        let out = decode_pointcloud(&fields, &data, 32, 1, false);
        assert_eq!(out.positions, vec![[7.0, 8.0, 9.0]]);
        assert_eq!(out.skipped, 0);
    }

    #[test]
    fn empty_cloud_decodes_to_nothing() {
        let fields = infer_fields_from_point_step(12);
        let out = decode_pointcloud(&fields, &[], 12, 0, false);
        assert!(out.positions.is_empty());
        assert_eq!(out.skipped, 0);
    }

    #[test]
    fn out_of_bounds_point_is_skipped_not_panic() {
        let fields = infer_fields_from_point_step(12);
        let pts = [[1.0f32, 2.0, 3.0, 0.0]];
        let data = build_data(&pts, 12, false);
        // Claim 3 points but only 1 fits → 2 skipped.
        let out = decode_pointcloud(&fields, &data, 12, 3, false);
        assert_eq!(out.positions, vec![[1.0, 2.0, 3.0]]);
        assert_eq!(out.skipped, 2);
    }

    #[test]
    fn non_finite_point_is_skipped() {
        let fields = infer_fields_from_point_step(12);
        let pts = [
            [f32::NAN, 0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0, 0.0],
            [f32::INFINITY, 0.0, 0.0, 0.0],
        ];
        let data = build_data(&pts, 12, false);
        let out = decode_pointcloud(&fields, &data, 12, 3, false);
        assert_eq!(out.positions, vec![[1.0, 1.0, 1.0]]);
        assert_eq!(out.skipped, 2);
    }

    #[test]
    fn missing_xyz_channels_yields_empty_all_skipped() {
        // Only an intensity channel declared — no x/y/z.
        let fields = vec![PointFieldDesc {
            name: "intensity".into(),
            offset: 0,
            datatype: 7,
            count: 1,
        }];
        let data = vec![0u8; 40];
        let out = decode_pointcloud(&fields, &data, 4, 10, false);
        assert!(out.positions.is_empty());
        assert_eq!(out.skipped, 10);
    }

    #[test]
    fn float64_xyz_channel_is_narrowed() {
        // x/y/z as FLOAT64 at 0/8/16, point_step 24.
        let fields = vec![
            PointFieldDesc {
                name: "x".into(),
                offset: 0,
                datatype: 8,
                count: 1,
            },
            PointFieldDesc {
                name: "y".into(),
                offset: 8,
                datatype: 8,
                count: 1,
            },
            PointFieldDesc {
                name: "z".into(),
                offset: 16,
                datatype: 8,
                count: 1,
            },
        ];
        let mut data = vec![0u8; 24];
        data[0..8].copy_from_slice(&1.5f64.to_le_bytes());
        data[8..16].copy_from_slice(&2.5f64.to_le_bytes());
        data[16..24].copy_from_slice(&3.5f64.to_le_bytes());
        let out = decode_pointcloud(&fields, &data, 24, 1, false);
        assert_eq!(out.positions, vec![[1.5, 2.5, 3.5]]);
    }

    #[test]
    fn intensity_ramp_is_deterministic_three_stops() {
        assert_eq!(intensity_to_rgba(0.0), [0, 0, 255, 255]); // blue
        assert_eq!(intensity_to_rgba(0.5), [0, 255, 0, 255]); // green
        assert_eq!(intensity_to_rgba(1.0), [255, 0, 0, 255]); // red
                                                              // Clamps out-of-range.
        assert_eq!(intensity_to_rgba(-1.0), [0, 0, 255, 255]);
        assert_eq!(intensity_to_rgba(2.0), [255, 0, 0, 255]);
    }

    #[test]
    fn parse_point_fields_roundtrips_a_hand_built_blob() {
        // Two records: x@0 f32, intensity@12 f32.
        let mut blob = Vec::new();
        for (name, offset, dt) in [("x", 0u32, 7u8), ("intensity", 12u32, 7u8)] {
            blob.extend_from_slice(&(name.len() as u32).to_le_bytes());
            blob.extend_from_slice(name.as_bytes());
            blob.extend_from_slice(&offset.to_le_bytes());
            blob.push(dt);
            blob.extend_from_slice(&1u32.to_le_bytes());
        }
        let fields = parse_point_fields(&blob).expect("parse");
        assert_eq!(
            fields,
            vec![
                PointFieldDesc {
                    name: "x".into(),
                    offset: 0,
                    datatype: 7,
                    count: 1
                },
                PointFieldDesc {
                    name: "intensity".into(),
                    offset: 12,
                    datatype: 7,
                    count: 1
                },
            ]
        );
    }

    #[test]
    fn parse_point_fields_empty_blob_is_empty_ok() {
        assert_eq!(parse_point_fields(&[]), Ok(Vec::new()));
    }

    #[test]
    fn parse_point_fields_truncated_record_errors_loudly() {
        // name_len claims 5 but only 2 name bytes follow.
        let mut blob = Vec::new();
        blob.extend_from_slice(&5u32.to_le_bytes());
        blob.extend_from_slice(b"xy");
        assert!(matches!(
            parse_point_fields(&blob),
            Err(PointFieldParseError::Truncated { index: 0, .. })
        ));
    }

    #[test]
    fn infer_fields_adds_intensity_only_when_step_allows() {
        assert_eq!(infer_fields_from_point_step(12).len(), 3);
        let xyzi = infer_fields_from_point_step(16);
        assert_eq!(xyzi.len(), 4);
        assert_eq!(xyzi[3].name, "intensity");
        assert_eq!(xyzi[3].offset, 12);
    }

    #[test]
    fn resolve_fields_prefers_packed_blob_else_infers() {
        let mut latch = FieldsWarnLatch::new();

        // Empty blob → inferred XYZI from point_step 16 (counted).
        let inferred = resolve_fields(&[], 16, &mut latch);
        assert_eq!(inferred, infer_fields_from_point_step(16));
        assert_eq!(latch.inferred_total, 1);

        // A valid packed blob is decoded verbatim (not inferred; counter
        // untouched).
        let (name, offset, dt) = ("x", 0u32, 7u8);
        let mut blob = Vec::new();
        blob.extend_from_slice(&(name.len() as u32).to_le_bytes());
        blob.extend_from_slice(name.as_bytes());
        blob.extend_from_slice(&offset.to_le_bytes());
        blob.push(dt);
        blob.extend_from_slice(&1u32.to_le_bytes());
        let decoded = resolve_fields(&blob, 16, &mut latch);
        assert_eq!(decoded, parse_point_fields(&blob).unwrap());
        assert_eq!(latch.inferred_total, 1, "decodes never bump the counter");
    }

    /// Oracle: the canonical warn → debug(n) → recovery(total) →
    /// warn cycle, against hand-written expected actions.
    #[test]
    fn fields_warn_latch_canonical_cycle() {
        let mut l = FieldsWarnLatch::new();
        // Regime 1: first inference WARNs, next two are DEBUG 1, 2.
        assert_eq!(l.on_inferred(), FieldsLogAction::WarnFirst);
        assert_eq!(
            l.on_inferred(),
            FieldsLogAction::DebugSuppressed { suppressed: 1 }
        );
        assert_eq!(
            l.on_inferred(),
            FieldsLogAction::DebugSuppressed { suppressed: 2 }
        );
        // Recovery: reports the 2 suppressed, exactly once.
        assert_eq!(l.on_decoded(), Some(2));
        assert_eq!(l.on_decoded(), None, "already armed — no double recovery");
        // Regime 2: re-armed → WARN again.
        assert_eq!(l.on_inferred(), FieldsLogAction::WarnFirst);
        // Counter is unconditional across regimes: 3 + 1.
        assert_eq!(l.inferred_total, 4);
    }

    /// A lone-warn regime (zero suppressed) re-arms SILENTLY — no recovery
    /// info, so an every-other-frame flapper cannot double the log volume.
    #[test]
    fn fields_warn_latch_lone_warn_regime_rearms_silently() {
        let mut l = FieldsWarnLatch::new();
        assert_eq!(l.on_inferred(), FieldsLogAction::WarnFirst);
        assert_eq!(l.on_decoded(), None, "nothing suppressed — silent re-arm");
        assert_eq!(l.on_inferred(), FieldsLogAction::WarnFirst);
        assert_eq!(l.on_decoded(), None);
        assert_eq!(l.inferred_total, 2, "counter still bumps every inference");
    }

    /// A long sustained regime: exactly one WARN, N-1 DEBUGs with exact
    /// running counts, unconditional total.
    #[test]
    fn fields_warn_latch_sustained_regime_suppresses_flood() {
        let mut l = FieldsWarnLatch::new();
        assert_eq!(l.on_inferred(), FieldsLogAction::WarnFirst);
        for i in 1..100u64 {
            assert_eq!(
                l.on_inferred(),
                FieldsLogAction::DebugSuppressed { suppressed: i }
            );
        }
        assert_eq!(l.inferred_total, 100);
        assert_eq!(l.on_decoded(), Some(99));
    }

    #[test]
    fn point_count_clamps_to_available_data() {
        // Exact fit.
        assert_eq!(point_count(10, 1, 16, 10 * 16), 10);
        // Truncated data → clamped to what fits.
        assert_eq!(point_count(10, 1, 16, 5 * 16), 5);
        // Unset geometry (width=height=0) → derived from bytes.
        assert_eq!(point_count(0, 0, 16, 3 * 16), 3);
        // Zero point_step → no points (no division by zero).
        assert_eq!(point_count(5, 1, 0, 100), 0);
    }

    // ---- is_bigendian: honored, not hardcoded LE (the bridge preserves it) --

    #[test]
    fn big_endian_cloud_decodes_via_hand_be_bytes() {
        // Hand-transcribed IEEE-754 BIG-endian f32 bytes (NOT byte-reversed in
        // code under test): 1.0 = 0x3F800000, 2.0 = 0x40000000, 3.0 = 0x40400000.
        // With big_endian = true the sink reproduces the exact points.
        let fields = infer_fields_from_point_step(12);
        let data = vec![
            0x3F, 0x80, 0x00, 0x00, // 1.0 BE
            0x40, 0x00, 0x00, 0x00, // 2.0 BE
            0x40, 0x40, 0x00, 0x00, // 3.0 BE
        ];
        let out = decode_pointcloud(&fields, &data, 12, 1, true);
        assert_eq!(out.positions, vec![[1.0, 2.0, 3.0]]);
        assert_eq!(out.skipped, 0);
    }

    #[test]
    fn endianness_flag_changes_the_decode_anti_tautology() {
        // The MIXED trap proving the dispatch is LIVE: the SAME bytes read LE
        // vs BE yield DIFFERENT floats. LE bytes 00 00 80 3F = 1.0; read BE the
        // same bytes are the subnormal 0x0000_803F. Compared via to_bits.
        let fields = infer_fields_from_point_step(12);
        let le_bytes = vec![
            0x00, 0x00, 0x80, 0x3F, // LE 1.0
            0x00, 0x00, 0x00, 0x40, // LE 2.0
            0x00, 0x00, 0x40, 0x40, // LE 3.0
        ];
        let le = decode_pointcloud(&fields, &le_bytes, 12, 1, false);
        assert_eq!(
            le.positions,
            vec![[1.0, 2.0, 3.0]],
            "LE bytes read LE = the real points"
        );
        let be = decode_pointcloud(&fields, &le_bytes, 12, 1, true);
        assert_eq!(be.positions.len(), 1);
        assert_ne!(
            be.positions[0],
            [1.0, 2.0, 3.0],
            "the SAME bytes read BE must garble — proves the flag is live, not ignored"
        );
        assert_eq!(be.positions[0][0].to_bits(), 0x0000_803F);
        assert_eq!(be.positions[0][1].to_bits(), 0x0000_0040);
        assert_eq!(be.positions[0][2].to_bits(), 0x0000_4040);
    }

    #[test]
    fn big_endian_float64_channel_decodes() {
        // The BE arm also covers the widest datatype (FLOAT64 → f32 narrow).
        let fields = vec![
            PointFieldDesc {
                name: "x".into(),
                offset: 0,
                datatype: 8,
                count: 1,
            },
            PointFieldDesc {
                name: "y".into(),
                offset: 8,
                datatype: 8,
                count: 1,
            },
            PointFieldDesc {
                name: "z".into(),
                offset: 16,
                datatype: 8,
                count: 1,
            },
        ];
        let mut data = vec![0u8; 24];
        data[0..8].copy_from_slice(&1.5f64.to_be_bytes());
        data[8..16].copy_from_slice(&2.5f64.to_be_bytes());
        data[16..24].copy_from_slice(&3.5f64.to_be_bytes());
        let out = decode_pointcloud(&fields, &data, 24, 1, true);
        assert_eq!(out.positions, vec![[1.5, 2.5, 3.5]]);
    }
}

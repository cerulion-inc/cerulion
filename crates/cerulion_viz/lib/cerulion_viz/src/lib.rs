// SPDX-License-Identifier: AGPL-3.0-only
//! Cerulion Rerun visualization support library.
//!
//! Shared, tested logic behind the `cerulion-vizd` daemon's schema-generic
//! decode→archetype→log dispatch (the four bespoke
//! lidar/camera/tf/tf_static sink crates were collapsed into this one table, and
//! the last graph-embedded sink node was deleted, so the daemon is now the sole consumer;
//! visualization runs DESK-side, never on the robot). Lifted OUT of the Go2 demo into the
//! product (formerly `examples/go2/lib/go2_viz`); the
//! Go2-specific constants are now parametrized behind [`config::VizConfig`]
//! (five `Default`-carrying groups whose defaults reproduce the exact Go2
//! values, so the demo stays byte-identical). Split into layers by dependency
//! so the correctness-critical parts test without Rerun (or a transport):
//!
//! - [`pointcloud`] — a PURE `PointCloud2` → 3D-points codec (std only).
//! - [`archetype`] — the archetype BUILDERS (Points3D / EncodedImage /
//!   Scalars / field-dump) and the field extractors they run on.
//! - [`sink`] — the "one adapter": the schema-name → archetype TABLE
//!   ([`sink::classify_schema`]) + the generic
//!   [`FrameWalker`](cerulion_core::codegen::FrameWalker)-driven
//!   [`dispatch_frame`](sink::dispatch_frame) the node runs per frame
//!   (AnyValues fallback, TF routing + `/tf_static` dedup).
//! - [`video`] — the DESK-SIDE H.264 path. Classifies ANY topic whose
//!   `uint8[]` payload is an H.264 Annex-B access unit (content-shape, never a
//!   schema name), demuxes an interleaved multi-rendition topic into
//!   per-resolution child entities by parsing each keyframe's SPS, and feeds the
//!   bytes VERBATIM to [`rerun::VideoStream`] — the viewer decodes, so a robot
//!   we have never seen needs no transcoding node to be watchable.
//! - [`marker`] — the `visualization_msgs/MarkerArray` archetype — the
//!   13 `Marker.type` kinds, the per-marker entity path
//!   (`<topic>/viz-markers/<ns>/<id>`), and the DELETE / DELETEALL entity-CLEAR
//!   mechanism. The only STATEFUL archetype: markers persist until deleted, so
//!   the sink keeps a per-input live-key set and NAMES what it clears.
//! - [`tf`] — the TFMessage → Rerun transform-tree mapping:
//!   the frame_id → entity-path tree, `Transform3D` / `Pinhole` / static
//!   logging, and the drain-all walker path. The PURE element codec it decodes
//!   lives in the `go2_tf` crate (no Rerun dep, so the producer node can use
//!   it too).
//! - [`stream`] — the process/cdylib-shared `RecordingStream` lifecycle.
//! - [`worker`] — the never-block viz logging worker. The sink `tick()`
//!   drains its subscribers (data-plane, cheap) and hands a per-tick batch to a
//!   dedicated thread that owns the `RecordingStream` + [`sink::SinkState`] and
//!   runs the (blocking) `rec.log` dispatch OFF the tick thread — a bounded
//!   queue DROPS viz frames (counted, loud-once) when the viewer wedges, so the
//!   viz plane can never block the sink.
//! - [`blueprint`] — the Go2 default dashboard layout (a hero 3D view + a
//!   telemetry plot column + a status panel), sent once per recording.
//! - [`skeleton`] — the URDF-derived stick-figure archetype: a robot's
//!   joint angles animating a joints-as-points + links-as-segments skeleton, with
//!   the lidar cloud re-parented under the URDF `radar` link so the fixed
//!   extrinsic superposes cloud + robot. **Reachable only from
//!   [`sink::SinkState::install_skeleton`] and tests**: the last production
//!   installer was removed, and so was the schema row that routed frames
//!   here (the mapping rendered strictly LESS than no mapping — see
//!   [`sink::ArchetypeKind::Skeleton`]). The machinery is kept intact, so
//!   wiring an installer into `cerulion-vizd` restores it.
//! - [`schema_registry`] — builds a frame walker over every built-in schema.
//! - [`config`] — the [`config::VizConfig`] parametrization surface
//!   (the five formerly-Go2-hardcoded constant groups; `Default` == Go2).
//! - [`representation`] — the PURE per-topic REPRESENTATION choice —
//!   plot / text dump / both, as a USER override on the ladder's automagic
//!   election, resolved by one function both vizd's layout and the sink's
//!   dispatch call so the views and the render can never disagree.
//! - [`plot_rate`] — the PURE per-topic plot-sample rate gate. A
//!   `Scalars` topic's render cost is its publisher's rate × its series count and
//!   nothing bounded the product; this holds each plot topic to
//!   [`plot_rate::MAX_PLOT_SAMPLES_PER_SEC`] samples per second of WIRE time,
//!   keyed on the publisher's own timestamps so the admitted set is a pure
//!   function of the frame stream (a replay renders the same plot).
//! - [`monitor`] — MONITORS v1: the PURE per-topic rate/liveness watchdog. A
//!   standing, unconfigured consumer of the `TopicLiveness` both liveness planes
//!   already produce: it classifies with `LivenessState` rather
//!   than re-implementing a threshold, learns a frozen rate baseline, and raises
//!   or clears `stalled` / `silent` / `rate_deviation` on CONFIRMED transitions.
//!   No transport, no clock of its own, no measurement — every number it reads
//!   was already computed.
//! - [`tap_manager`] — the TRANSPORT-backed tap plane — owns a
//!   listener-less `DataOnlySubscriber` per attached topic over real iceoryx2,
//!   drains them into per-tick `InputFrames` for the worker. It owns the tap SET
//!   too: there is no separate pure `tapset` module, because nothing consumed
//!   one and its tests would have read as
//!   tap-path coverage.
//!
//! # Design (binding direction)
//!
//! Rerun ingests Cerulion messages through ONE schema-driven path: the
//! generic frame walker (`cerulion_core::codegen::frame_walker`) decodes any
//! frame into a typed value tree; [`sink::dispatch_frame`] classifies it and
//! maps it to a Rerun archetype. The two media sinks additionally take a
//! typed fast path (their generated readers' accessors → the SAME
//! [`pointcloud`] codec / [`archetype`] builders), because the PointCloud2
//! `fields` blob is opaque to the walker and typed reads are exact regardless.
//!
//! Rerun lives ONLY here + the two sink crates — never in `cerulion_core`.

// Principle #12 (structured logging): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

pub mod archetype;
pub mod blueprint;
pub mod config;
pub mod marker;
/// The pure per-topic rate/liveness watchdog engine, RE-EXPORTED from
/// `cerulion_core`.
///
/// It lived here while vizd was its only consumer. The monitors-verdict
/// Flashback trigger gave it a second one — `cerulion_bagd`, robot-side — which
/// cannot depend on this crate (rerun / openh264 / ureq / MSRV 1.93, and this
/// crate is excluded from `default-members` precisely to keep that tree out of a
/// plain build). Moving it to `cerulion_core` is what makes ONE engine serve both
/// planes; re-exporting under the original path is what keeps every
/// `cerulion_viz::monitor::…` reference in vizd byte-unchanged, so the move
/// carries no risk of a silent behaviour change on the desk side.
pub use cerulion_core::monitor;
pub mod openh264_fetch;
pub mod plot_rate;
pub mod pointcloud;
pub mod representation;
pub mod schema_registry;
pub mod sink;
pub mod skeleton;
pub mod stream;
pub mod tap_manager;
pub mod tf;
pub mod video;
pub mod video_decode;
pub mod worker;

// Crate-level TEST-ONLY serialization for the process-global blueprint
// statics. `#[cfg(test)]`, so it is compiled only for the lib test binary and
// cannot reach a shipping build.
#[cfg(test)]
mod test_support;

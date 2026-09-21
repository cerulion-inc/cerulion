// SPDX-License-Identifier: AGPL-3.0-only
//! Compile-coverage for the headline node examples in `README.md`,
//! `USER_API.md`, the root agent guide and `docs/tutorials/`: the
//! `#[cerulion_node]` / `#[cerulion_node_impl]` surface.
//!
//! # Corpus, and what is deliberately outside it
//!
//! Every `.md` in the tree carrying a ```rust fence was classified. IN the
//! corpus: `README.md`, `USER_API.md`, `docs/tutorials/01-getting-started.md`,
//! `docs/tutorials/02-latency-measurement.md`. Deliberately OUT, each for a stated
//! reason — if you add a doc with node examples, either mirror it here or add it
//! to this list, so the next gap is a choice and not an accident:
//!
//! - `.claude/skills/**` — internal reference material for maintainer tooling, not user docs.
//! - `benches/*/BENCHMARK_ANALYSIS.md` — analysis prose quoting historical
//!   code, deliberately frozen against the run it describes.
//! - `RALPH_PROMPT.md` — an agent loop prompt, not a user-facing example.
//!
//! Closes a test-coverage gap:
//! without this file the user-facing examples are not
//! compile-tested anywhere, so a future codegen / macro change that
//! breaks the documented patterns would silently rot the docs (for
//! instance, an example writing `self.cmd_out.linear.x`
//! does not compile because `Twist`'s FixedSection is
//! empty and there is no field to bind to).
//!
//! The structures here are NOT verbatim copies of the README /
//! USER_API examples — they're the simplified forms with the same
//! macro-feature surface (declarative `#[cerulion_node]` +
//! `#[cerulion_node_impl]` + field-level `#[input]` / `#[output]` +
//! direct field access via Deref). If the doc examples are revised
//! again, update these mirrors so CI re-validates the new pattern.

use cerulion_core::graph::node::{NodeEntry, NodeInfo};
use cerulion_core::prelude::*;
use cerulion_core::MacroPolicy;
use native_ros2_messages::geometry_msgs::Vector3;
use native_ros2_messages::sensor_msgs::{Image, LaserScan};

// ----------------------------------------------------------------------
// Mirrors the `SafetyController` headline example in `README.md` and
// `USER_API.md`: LaserScan trigger input + Vector3 fixed-only output.
// ----------------------------------------------------------------------

// `period_ms` and `#[input(trigger)]` cannot coexist — the validator
// rejects mixing time-driven and data-driven trigger sources. The
// headline pattern uses `period_ms` for simplicity (the data-triggered
// pattern is shown in the tutorial mirror below — `DriveBaseNode`).
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct SafetyController {
    #[input(depth = 1)]
    scan: LaserScan,

    #[output]
    linear_velocity: Vector3,

    last_min_range: f32,
}

#[cerulion_node_impl]
impl SafetyController {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Read variable field via typed accessor (the read path).
        let min_range = self
            .scan
            .ranges()
            .iter()
            .copied()
            .filter(|r| r.is_finite() && *r > 0.0)
            .fold(f32::INFINITY, f32::min);
        self.last_min_range = min_range;

        // Direct fixed-field write via Deref<Target = Vector3Shm>
        // (the write path).
        // A reading that is not finite or not positive is invalid and never counts;
        // with no valid reading at all the controller stops rather than cruises.
        self.linear_velocity.x = if min_range.is_finite() && min_range >= 0.5 {
            0.3
        } else {
            0.0
        };

        Ok(())
    }
}

#[test]
fn headline_safety_controller_example_compiles_and_reports_ports() {
    let entry: SafetyControllerEntry = SafetyControllerEntry::new();
    let info: NodeInfo = entry.info().expect("info should parse");
    assert_eq!(info.input_names(), &["scan"]);
    assert_eq!(info.output_names(), &["linear_velocity"]);
}

// ----------------------------------------------------------------------
// Mirrors the `CameraNode` example in the root agent guide's Node Macros section:
// Image variable schema with direct fixed-field write via Deref.
// ----------------------------------------------------------------------

#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct CameraNode {
    #[output]
    image: Image,

    frame_count: u32,
}

#[cerulion_node_impl]
impl CameraNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.frame_count += 1;
        // Direct fixed-field write: `Image`'s FixedSection has
        // `pub height: u32`, so this lands directly in the SHM slot
        // via `Deref<Target = ImageFixedSection>`.
        self.image.height = self.frame_count;
        Ok(())
    }
}

#[test]
fn headline_camera_node_example_compiles_and_reports_ports() {
    let entry: CameraNodeEntry = CameraNodeEntry::new();
    let info: NodeInfo = entry.info().expect("info should parse");
    assert_eq!(info.input_names(), &[] as &[&str]);
    assert_eq!(info.output_names(), &["image"]);
}

// ----------------------------------------------------------------------
// Mirrors the `LidarSensorNode` example from
// `docs/tutorials/01-getting-started.md`: the variable-length `ranges`
// field written IN PLACE through the generated `loan_ranges(n)`
// accessor (no per-tick `Vec`, no copy into the frame), plus the
// schema-blind rewriter on the remaining variable fields
// (`self.scan.intensities = &[][..]` lowers to
// `__cer_assign_intensities(&...)?`, with no `#[output]` list).
// ----------------------------------------------------------------------

#[cerulion_node(period_ms = 100)]
#[derive(Default)]
struct LidarSensorNode {
    #[output]
    scan: LaserScan,

    seq: u32,
}

#[cerulion_node_impl]
impl LidarSensorNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.seq += 1;

        // Direct fixed-field writes via Deref.
        self.scan.angle_min = -1.57;
        self.scan.angle_max = 1.57;
        self.scan.angle_increment = 0.01;
        self.scan.range_min = 0.05;
        self.scan.range_max = 10.0;

        // The tutorial's in-place variable-field write: `loan_ranges(n)`
        // hands back `&mut [f32]` over n elements of the loaned frame.
        const NUM_RAYS: usize = 314;
        let ranges = self.scan.loan_ranges(NUM_RAYS)?;
        ranges.fill(5.0);
        ranges[150..164].fill(1.0);
        // The tutorial writes every variable field each tick (publish
        // gate): empty intensities + raw-bytes header, both via plain
        // assignment (no declaration, no raw setter call).
        self.scan.intensities = &[][..];
        self.scan.header = &[][..];

        Ok(())
    }
}

#[test]
fn tutorial_lidar_sensor_example_compiles_and_reports_ports() {
    let entry: LidarSensorNodeEntry = LidarSensorNodeEntry::new();
    let info: NodeInfo = entry.info().expect("info should parse");
    assert_eq!(info.input_names(), &[] as &[&str]);
    assert_eq!(info.output_names(), &["scan"]);
}

// ----------------------------------------------------------------------
// Mirrors the tutorial's `SafetyControllerNode`: data-triggered via
// `#[input(trigger)]` (no node-level `period_ms` — the field-level
// trigger is the policy declaration). Combines a variable-schema input
// (LaserScan with the `ranges` accessor) and a fixed-only output.
// ----------------------------------------------------------------------

const SAFE_DISTANCE: f32 = 0.5;
const CRUISE_SPEED: f64 = 0.5;

#[cerulion_node]
#[derive(Default)]
struct SafetyControllerNode {
    #[input(trigger)]
    scan: LaserScan,

    #[output]
    cmd_vel: Vector3,

    closest_range: f32,
    stop_count: u32,
}

#[cerulion_node_impl]
impl SafetyControllerNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // The tutorial's fail-safe condition: a beam counts only when it
        // is finite and strictly positive, and cruising requires one such
        // beam at or beyond the safe distance. An empty, all-NaN or all-zero
        // scan leaves `closest_valid` at infinity and stops.
        let closest_valid = self
            .scan
            .ranges()
            .iter()
            .copied()
            .filter(|r| r.is_finite() && *r > 0.0)
            .fold(f32::INFINITY, f32::min);
        self.closest_range = closest_valid;

        let speed = if closest_valid.is_finite() && closest_valid >= SAFE_DISTANCE {
            CRUISE_SPEED
        } else {
            self.stop_count += 1;
            0.0
        };

        self.cmd_vel.x = speed;
        Ok(())
    }
}

#[test]
fn tutorial_safety_controller_example_compiles_and_reports_ports() {
    let entry: SafetyControllerNodeEntry = SafetyControllerNodeEntry::new();
    let info: NodeInfo = entry.info().expect("info should parse");
    assert_eq!(info.input_names(), &["scan"]);
    assert_eq!(info.output_names(), &["cmd_vel"]);
}

// ----------------------------------------------------------------------
// Mirrors the tutorial's `DriveBaseNode`: trigger-input sink, no
// output ports, direct fixed-field reads on Vector3.
// ----------------------------------------------------------------------

#[cerulion_node]
#[derive(Default)]
struct DriveBaseNode {
    #[input(trigger)]
    cmd_vel: Vector3,

    msg_count: u64,
}

#[cerulion_node_impl]
impl DriveBaseNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.msg_count += 1;
        // Direct fixed-field reads via Deref.
        let _ = (self.cmd_vel.x, self.cmd_vel.y, self.cmd_vel.z);
        Ok(())
    }
}

#[test]
fn tutorial_drive_base_example_compiles_and_reports_ports() {
    let entry: DriveBaseNodeEntry = DriveBaseNodeEntry::new();
    let info: NodeInfo = entry.info().expect("info should parse");
    assert_eq!(info.input_names(), &["cmd_vel"]);
    assert_eq!(info.output_names(), &[] as &[&str]);
}

// ----------------------------------------------------------------------
// Compile-coverage for the other `#[cerulion_node(...)]`
// trigger-policy attributes the tutorial's macroified table claims
// exist: `sync_window_ms`, `external`. Previously only
// `period_ms` (and field-level `#[input(trigger)]`) had compile
// coverage, leaving the documented attributes vulnerable to silent
// rename / removal — exactly the doc-rot this file exists to prevent.
// Each test uses the minimal shape accepted by
// `cerulion_macros/src/validate.rs`:
//   - Sync: requires `sync_window_ms` + 2+ `#[input(trigger)]` fields.
//   - External: requires `external` + 0 trigger inputs (host calls
//     `Scheduler::trigger_external`).
// The former node-level `deadline_ms` trigger was REMOVED;
// a deadline watcher is now expressed as a `Data` trigger
// (`#[input(trigger)]`) carrying the per-input `expect_within_ms` QoS
// watchdog; `DeadlineWatcherNode` below pins that decomposition.
// ----------------------------------------------------------------------

#[cerulion_node(sync_window_ms = 50)]
#[derive(Default)]
struct SyncFusionNode {
    #[input(trigger)]
    scan: LaserScan,

    #[input(trigger)]
    image: Image,

    #[output]
    fused_velocity: Vector3,
}

#[cerulion_node_impl]
impl SyncFusionNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Minimal body — reads via ranges()/height to keep the
        // typed accessors live so a future codegen change that
        // breaks them surfaces here.
        let _ = (self.scan.ranges().len(), self.image.height);
        self.fused_velocity.x = 1.0;
        Ok(())
    }
}

#[test]
fn macro_sync_window_ms_compiles_and_reports_ports() {
    let entry: SyncFusionNodeEntry = SyncFusionNodeEntry::new();
    let info: NodeInfo = entry.info().expect("info should parse");
    assert_eq!(info.input_names(), &["scan", "image"]);
    assert_eq!(info.output_names(), &["fused_velocity"]);
    // Pin the macro-attr → MacroPolicy round-trip so a regression
    // in the codegen-side serializer surfaces here (a port-name
    // reflection that still works while the policy attr was dropped
    // would otherwise pass silently).
    assert_eq!(
        info.policy(),
        Some(MacroPolicy::Sync { window_ms: 50 }),
        "macro must encode `sync_window_ms = 50` as `MacroPolicy::Sync {{ window_ms: 50 }}`"
    );
}

// The former `#[cerulion_node(deadline_ms = 100)]`
// node-level trigger decomposes into a `Data` trigger
// (`#[input(trigger)]`) + the per-input `expect_within_ms` QoS
// watchdog on the SAME input. The node fires on `image` arrival;
// `expect_within_ms` reports a miss if no fresh `image` lands within
// 100 ms. (The watchdog wiring is pinned by `expect_within_iox2_test`; this test
// pins the macro surface + inferred DataTrigger policy, NOT the miss
// counter.)
#[cerulion_node]
#[derive(Default)]
struct DeadlineWatcherNode {
    #[input(trigger, expect_within_ms = 100)]
    image: Image,

    miss_count: u32,
}

#[cerulion_node_impl]
impl DeadlineWatcherNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.image.height;
        self.miss_count += 1;
        Ok(())
    }
}

#[test]
fn macro_deadline_watcher_decomposes_to_data_trigger() {
    let entry: DeadlineWatcherNodeEntry = DeadlineWatcherNodeEntry::new();
    let info: NodeInfo = entry.info().expect("info should parse");
    assert_eq!(info.input_names(), &["image"]);
    assert_eq!(info.output_names(), &[] as &[&str]);
    // The old `deadline_ms` node-level trigger is gone: a single
    // `#[input(trigger)]` infers `MacroPolicy::DataTrigger`. The
    // `expect_within_ms = 100` QoS knob rides on the same input
    // (the watchdog half), orthogonal to the trigger policy.
    assert_eq!(
        info.policy(),
        Some(MacroPolicy::DataTrigger {
            input_name: "image".to_string()
        }),
        "a single `#[input(trigger)]` must infer `MacroPolicy::DataTrigger`"
    );
}

// `unbounded_sync`: loose-AND sync without a
// timing window. Requires 2+ `#[input(trigger)]` fields, mutually
// exclusive with `sync_window_ms`. See `cerulion_macros/src/validate.rs`.
#[cerulion_node(unbounded_sync)]
#[derive(Default)]
struct UnboundedSyncFusionNode {
    #[input(trigger)]
    scan: LaserScan,

    #[input(trigger)]
    image: Image,

    #[output]
    fused_velocity: Vector3,
}

#[cerulion_node_impl]
impl UnboundedSyncFusionNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = (self.scan.ranges().len(), self.image.height);
        self.fused_velocity.x = 2.0;
        Ok(())
    }
}

#[test]
fn macro_unbounded_sync_compiles_and_reports_ports() {
    let entry: UnboundedSyncFusionNodeEntry = UnboundedSyncFusionNodeEntry::new();
    let info: NodeInfo = entry.info().expect("info should parse");
    assert_eq!(info.input_names(), &["scan", "image"]);
    assert_eq!(info.output_names(), &["fused_velocity"]);
    // Pin the macro-attr → MacroPolicy round-trip. A regression in the
    // codegen-side serializer would otherwise pass the port-name check
    // silently.
    assert_eq!(
        info.policy(),
        Some(MacroPolicy::UnboundedSync),
        "macro must encode `unbounded_sync` as `MacroPolicy::UnboundedSync`"
    );
}

#[cerulion_node(external)]
#[derive(Default)]
struct ExternalCommandNode {
    #[output]
    velocity: Vector3,

    command_count: u64,
}

#[cerulion_node_impl]
impl ExternalCommandNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.command_count += 1;
        self.velocity.x = 0.5;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[test]
fn macro_external_compiles_and_reports_ports() {
    let entry: ExternalCommandNodeEntry = ExternalCommandNodeEntry::new();
    let info: NodeInfo = entry.info().expect("info should parse");
    assert_eq!(info.input_names(), &[] as &[&str]);
    assert_eq!(info.output_names(), &["velocity"]);
    assert_eq!(
        info.policy(),
        Some(MacroPolicy::External),
        "macro must encode `external` as `MacroPolicy::External`"
    );
}

// ----------------------------------------------------------------------
// Per-port + per-node QoS deadlines. Three
// independent attributes, each compile-tested with the matching
// `NodeInfo` accessor pinned so codegen drift surfaces here.
// ----------------------------------------------------------------------

#[cerulion_node(period_ms = 100)]
#[derive(Default)]
struct InputDeadlineNode {
    #[input(expect_within_ms = 200)]
    scan: LaserScan,
}

#[cerulion_node_impl]
impl InputDeadlineNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.scan.ranges().len();
        Ok(())
    }
}

#[test]
fn macro_input_expect_within_ms_compiles_and_propagates_to_input_meta() {
    let entry: InputDeadlineNodeEntry = InputDeadlineNodeEntry::new();
    let info: NodeInfo = entry.info().expect("info should parse");
    assert_eq!(info.input_names(), &["scan"]);
    // `#[input(expect_within_ms = N)]` must propagate through the codegen's
    // `InputMeta` emission — earlier the codegen used
    // `with_input_names_and_output_meta` which silently dropped
    // field-level metadata.
    let scan = &info.input_meta()[0];
    assert_eq!(scan.name, "scan");
    assert_eq!(
        scan.expect_within_ms,
        Some(200),
        "macro must propagate `#[input(expect_within_ms = 200)]` to `InputMeta.expect_within_ms`"
    );
}

#[cerulion_node(period_ms = 100)]
#[derive(Default)]
struct OutputDeadlineNode {
    #[output(promise_within_ms = 100)]
    cmd_vel: Vector3,
}

#[cerulion_node_impl]
impl OutputDeadlineNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.cmd_vel.x = 1.0;
        Ok(())
    }
}

#[test]
fn macro_output_promise_within_ms_compiles_and_propagates_to_output_meta() {
    let entry: OutputDeadlineNodeEntry = OutputDeadlineNodeEntry::new();
    let info: NodeInfo = entry.info().expect("info should parse");
    assert_eq!(info.output_names(), &["cmd_vel"]);
    let cmd_vel = &info.output_meta()[0];
    assert_eq!(cmd_vel.name, "cmd_vel");
    assert_eq!(
        cmd_vel.promise_within_ms,
        Some(100),
        "macro must propagate `#[output(promise_within_ms = 100)]` to `OutputMeta.promise_within_ms`"
    );
}

#[cerulion_node(period_ms = 100, tick_within_ms = 50)]
#[derive(Default)]
struct TickDeadlineNode {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl TickDeadlineNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = 1.0;
        Ok(())
    }
}

#[test]
fn macro_tick_within_ms_compiles_and_propagates_to_node_info() {
    let entry: TickDeadlineNodeEntry = TickDeadlineNodeEntry::new();
    let info: NodeInfo = entry.info().expect("info should parse");
    assert_eq!(
        info.tick_within_ms(),
        Some(50),
        "macro must propagate `#[cerulion_node(tick_within_ms = 50)]` to `NodeInfo.tick_within_ms`"
    );
    // Stacks with trigger policy:
    assert_eq!(
        info.policy(),
        Some(MacroPolicy::Period { period_ms: 100 }),
        "tick_within_ms must stack with the trigger policy, not replace it"
    );
}

// ----------------------------------------------------------------------
// Compile-coverage for the four `#[on_event]`
// handler shapes documented in `USER_API.md` ("Backpressure → #[on_event]
// callbacks", Event types 1–4). The UI compile-FAIL tests
// (`tests/ui/on_event_*`) pin the rejection cases; this pins the POSITIVE
// surface so a future macro change that breaks a documented handler
// signature — including `LivelinessEvent`, the fourth event
// type added to the input-scoped filter — surfaces here instead of
// silently rotting the docs.
//
// Three different-kind handlers (`BackpressureEvent`, `ExpectWithinEvent`,
// `LivelinessEvent`) target the SAME input `"lidar"` — pinning the
// documented "combining handlers on one port" contract (different kinds
// on one port are allowed; same (port, kind) is a compile error, covered
// by the UI tests). The `PromiseWithinEvent` handler is output-scoped on
// `"cmd_vel"`.
// ----------------------------------------------------------------------

#[cerulion_node(period_ms = 50)]
#[derive(Default)]
struct EventHandlersNode {
    #[input(backpressure = sample(2), expect_within_ms = 100)]
    lidar: LaserScan,

    #[output(promise_within_ms = 100)]
    cmd_vel: Vector3,

    events_seen: u32,
}

#[cerulion_node_impl]
impl EventHandlersNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.lidar.ranges().len();
        self.cmd_vel.x = 1.0;
        Ok(())
    }

    #[on_event(input = "lidar")]
    fn on_lidar_pressure(&mut self, event: BackpressureEvent) {
        self.events_seen += 1;
        // `dropped` is the messages-lost count (0 for `block`).
        let _ = event.dropped;
    }

    #[on_event(input = "lidar")]
    fn on_lidar_deadline(&mut self, event: ExpectWithinEvent) {
        self.events_seen += 1;
        let _ = event.elapsed_ms;
    }

    #[on_event(input = "lidar")]
    fn on_lidar_liveliness(&mut self, event: LivelinessEvent) {
        self.events_seen += 1;
        // The event's fields: `state` (LivelinessState, Copy) +
        // `publisher_count` — pins the documented struct surface.
        let _ = (event.state, event.publisher_count);
    }

    #[on_event(output = "cmd_vel")]
    fn on_cmd_vel_overdue(&mut self, event: PromiseWithinEvent) {
        self.events_seen += 1;
        let _ = event.elapsed_ms;
    }
}

#[test]
fn headline_on_event_handlers_compile_and_report_ports() {
    let entry: EventHandlersNodeEntry = EventHandlersNodeEntry::new();
    let info: NodeInfo = entry.info().expect("info should parse");
    assert_eq!(info.input_names(), &["lidar"]);
    assert_eq!(info.output_names(), &["cmd_vel"]);
    // The handlers are orthogonal to the trigger policy — `period_ms`
    // survives the four `#[on_event]` attributes.
    assert_eq!(
        info.policy(),
        Some(MacroPolicy::Period { period_ms: 50 }),
        "`#[on_event]` handlers must not displace the node's trigger policy"
    );
}

// ----------------------------------------------------------------------
// Mirrors the single-hop latency snippet in
// `docs/tutorials/02-latency-measurement.md`.
//
// This mirror exists because that page was factually wrong TWICE inside
// one PR: it called a `try_receive_typed` that does not exist, and the
// correction then called `try_receive` with the wrong arity AND on a
// trigger input — whose queue the pre-step drain has already moved into
// the frozen slot, which serves only `try_view`. Nothing caught either
// one, because this corpus watched tutorial 01 and not tutorial 02.
//
// What it pins: `InputView::wire_timestamp_ns()` is reachable on a
// trigger input from inside a `#[cerulion_node_impl]` body (an inherent
// method that must keep beating the `Deref` to the message type), and
// `cerulion_core::clock::real_ns()` is the same-domain wall reading the
// page tells the reader to subtract it from. A rename or a signature
// change on either now breaks this test instead of rotting the page.
// ----------------------------------------------------------------------

#[cerulion_node]
#[derive(Default)]
struct SingleHopLatencyNode {
    #[input(trigger)]
    cmd_vel: Vector3,

    last_latency_us: u64,
}

#[cerulion_node_impl]
impl SingleHopLatencyNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // The tutorial's snippet, verbatim in shape.
        let publish_time = self.cmd_vel.wire_timestamp_ns();
        let now = cerulion_core::clock::real_ns();
        let latency_ns = now.saturating_sub(publish_time);
        tracing::info!(latency_us = latency_ns / 1000, "single-hop latency");

        self.last_latency_us = latency_ns / 1000;
        Ok(())
    }
}

#[test]
fn tutorial_single_hop_latency_example_compiles_and_reports_ports() {
    let entry: SingleHopLatencyNodeEntry = SingleHopLatencyNodeEntry::new();
    let info: NodeInfo = entry.info().expect("info should parse");
    assert_eq!(info.input_names(), &["cmd_vel"]);
    assert_eq!(info.output_names(), &[] as &[&str]);
}

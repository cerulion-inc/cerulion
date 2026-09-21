// SPDX-License-Identifier: AGPL-3.0-only
//! The GENERIC config-mapped DDS→Cerulion ingress bridge.
//!
//! `dds_bridge` subscribes to DDS topics published by a ROS 2 / CycloneDDS
//! peer (the Unitree Go2 firmware, an unmodified ROS 2 stack, …) via the
//! pure-Rust rustdds/ros2-client stack (NO rcl/rmw linkage — the
//! `cerulion_go2_dds` wrapper) and republishes each sample zero-copy on the
//! Cerulion wire. It is deliberately NOT a Go2-specific node: the DDS side
//! is a CONFIG FILE — the Go2 is just the
//! first config (`graphs/go2.bridge.yaml`), and this node is the one
//! `cerulion ros2 attach` generates a graph for.
//!
//! # The mapping contract
//!
//! A YAML config (path in the [`config::CONFIG_ENV`] env var) declares
//! `{dds_topic, ros_type, cerulion_topic}` mappings. Each `ros_type` resolves
//! in the CODEC REGISTRY ([`registry::RosType`]) to a decoder from the
//! `cerulion_go2_dds` set and to one FIXED output port on this node:
//!
//! | ros_type | port | Cerulion schema |
//! |---|---|---|
//! | `sensor_msgs/PointCloud2` | `cloud` | `sensor_msgs/PointCloud2` (pass-through) |
//! | `unitree_go/SportModeState` | `odom` | `nav_msgs/Odometry` (pose+twist projection) |
//! | `geometry_msgs/Twist` | `twist` | `geometry_msgs/TwistStamped` (stamped) |
//! | `unitree_api/Request` | `request_json` | `std_msgs/String` (parameter JSON) |
//!
//! Ports are fixed because `#[cerulion_node]` declares ports at compile time;
//! these four are deliberate PROJECTIONS (typed wins —
//! [`generic::route_mapping`]). The GRAPH YAML owns the
//! actual Cerulion topic names via `topic:` overrides (Principle #5); the
//! config's `cerulion_topic` documents the route and must match.
//!
//! **The config-vs-graph contract:** the mapping config chooses which DDS
//! topics FLOW; the graph must ALWAYS wire ALL FOUR ports. The generated tick
//! resolves every output proxy before the node body runs, so a graph wiring
//! only a subset fails every tick ("zero-copy tick: missing publisher for
//! output `...`") and the bridge publishes nothing. Unmapped-but-wired ports
//! are harmless — they never publish (the discard gate below).
//!
//! # Raw-generic mappings: ANY schema-known type, zero per-type code
//!
//! A mapping whose `ros_type` is OUTSIDE the four-type registry is accepted
//! IFF a `MessageSchema` resolves it ([`generic::schema_supported`] — the
//! whole embedded ROS 2 registry + the Unitree schemas). Such a mapping
//! bypasses the node's ports entirely: the pump's drain thread runs a
//! raw-CDR reader ([`generic::raw`]) whose samples are transcoded by the
//! generic codec and re-injected into local SHM through a dynamically-created
//! ingress publisher ([`generic::RawIngressRoute`], the precedent) on
//! the mapping's `cerulion_topic` — which is AUTHORITATIVE for raw routes
//! (consumers wire it via absolute `source:` references, like any external
//! topic). Only a type on NEITHER path is the loud
//! [`config::ConfigError::UnknownRosType`] — "unknown type" means NO SCHEMA,
//! never "no codec"; the auto-schema chain extends the schema set.
//!
//! **Raw wire timestamps** are the TRANSPORT clock read at drain-publish time
//! — consistent with the typed ports (whose wire stamps are the publisher's
//! loan-time clock) and deliberately NOT the DDS writer's `source_timestamp`:
//! that is a REMOTE peer's wall clock, unordered against the local clock, so
//! stamping it into the wire would break local freshness/ordering semantics
//! (`InputView::wire_timestamp_ns` consumers) and hand a
//! peer-controlled value to replay tooling (Principle #7 — replay re-injects
//! recorded frames, so live-path stamps must mean ONE thing: local receive
//! time).
//!
//! # Shape: an external ingress node
//!
//! `#[cerulion_node(external)]`: [`external_source`](DdsBridge::external_source)
//! loads + validates the config, then returns an [`ExternalSource::Blocking`]
//! whose helper thread runs the [`pump::BridgePump`] — it spawns a dedicated
//! drain thread on which ALL DDS objects are created AND the per-mapping
//! typed `async_stream()` drains run (the path that works — sync `take()`
//! empirically never yields under ros2-client 0.10 against a CycloneDDS peer;
//! see the `pump` docs), pushing into the shared latest-wins
//! [`queue::SampleQueue`], with the doorbell rung per helper beat when
//! samples arrived and loud 1 s-backoff retry on init failure. A config error
//! logs `error!` and reports `HostDriven`, which `cerulion graph run` REFUSES
//! at launch as an inert external node — a misconfigured bridge is
//! a loud launch failure naming this node, never a silent no-op.
//!
//! **Graceful DDS teardown:** the pump is moved INTO the Blocking
//! closure (owned by the host's own detached helper thread), so the node keeps
//! a cloned [`pump::PumpShutdown`] handle. [`DdsBridge::shutdown`] — invoked by
//! `GraphRuntime::shutdown_all_nodes` on SIGINT/SIGTERM/SIGHUP teardown (and via
//! the cdylib `cerulion_node_shutdown` FFI) — signals the drain thread to stop
//! and JOINS it (bounded), which drops the rustdds participant and sends its
//! SPDP/SEDP disposes so remote writers unregister this node's readers immediately
//! rather than leaving ghosts until the DDS lease ages out. Without the join the
//! drain thread would be detached-for-the-process and the participant orphaned
//! on EVERY teardown, SIGINT included.
//!
//! **Replay = Live (Principle #7):** replay and the polled `step()`
//! seam NEVER query `external_source()` — external nodes replay from recorded
//! frames — so no DDS state exists on the replay path by construction. The
//! tick is a pure function of `(now_ns, drained samples)`.
//!
//! # Per-fire publish gating
//!
//! One fire drains the queue and writes ONLY the ports whose sample arrived
//! (fixed port order — deterministic). Every port's schema carries at least
//! one VARIABLE field, and each write path always writes it, so an untouched
//! port's frame is discarded by the runtime's output gate (the camera_jpeg
//! empty-fire mechanic) — no garbage frames on unmapped/quiet topics. This is
//! why the `twist` port publishes `TwistStamped` rather than the all-fixed
//! `Twist` (an all-fixed output would publish on every fire), and the header
//! stamp it gains is real teleop value anyway.
//!
//! # ONE participant per process
//!
//! The pump builds its DDS side through [`cerulion_go2_dds::Go2Participant`]
//! — the process-global guard (Principle #8 analog). Two bridge instances in
//! one process would fail the second pump init LOUDLY (backoff-retried, so a
//! reconfigure-to-one-instance heals without restart).

// P12 (Logging, see AGENTS.md): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

pub mod config;
pub mod generic;
pub mod latch;
pub mod mapping;
pub mod pump;
pub mod queue;
pub mod registry;

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use cerulion_core::prelude::*;
use cerulion_core::shm_runtime::{u64s_as_bytes, u64s_as_bytes_mut};
use cerulion_core::wire::MaxPayloadCapacity;
use cerulion_core::TransportError;
use native_ros2_messages::geometry_msgs::TwistStamped;
use native_ros2_messages::nav_msgs::Odometry;
use native_ros2_messages::sensor_msgs::PointCloud2;
use native_ros2_messages::std_msgs::Header;
use native_ros2_messages::std_msgs::String as RosString;

use crate::config::BridgeConfig;
use crate::latch::{FloodAction, FloodLatch};
use crate::queue::{SampleQueue, PORT_COUNT};
use crate::registry::{BridgeSample, RosType};

/// Tick-side observability counters (Principle #3) — the node-side sibling of
/// [`pump::PumpStats`]. Shared via an `Arc` so diagnostics + the `with_shared_*`
/// test seam read per-port write-error totals off the tick thread. Oversized
/// clouds count as write errors and publish no partial frame. Never reset.
#[derive(Debug, cerulion_core::state::CerulionState)]
pub struct TickStats {
    /// Per-port write failures (fixed [`RosType::slot`] order). A failed port
    /// write drops ONLY that sample; siblings are unaffected (per-port isolation).
    write_errors: [AtomicU64; PORT_COUNT],
}

impl Default for TickStats {
    fn default() -> Self {
        Self {
            write_errors: std::array::from_fn(|_| AtomicU64::new(0)),
        }
    }
}

impl TickStats {
    /// Bump the write-error counter for `t`'s port.
    pub fn bump_write_error(&self, t: RosType) {
        self.write_errors[t.slot()].fetch_add(1, Ordering::Relaxed);
    }
    /// Lifetime write failures for `t`'s port.
    pub fn write_errors_total(&self, t: RosType) -> u64 {
        self.write_errors[t.slot()].load(Ordering::Relaxed)
    }
    /// One-line render for diagnostics (pair with [`pump::PumpStats::render`]).
    pub fn render(&self) -> String {
        format!(
            "write_errors=[cloud={} odom={} twist={} request={}]",
            self.write_errors_total(RosType::PointCloud2),
            self.write_errors_total(RosType::SportModeState),
            self.write_errors_total(RosType::Twist),
            self.write_errors_total(RosType::Request),
        )
    }
}

/// Per-port flood latches for write-error suppression (one per fixed
/// output port). A struct-of-fields (not `[FloodLatch; N]`) so `#[derive(
/// Default)]` on the node needs no array-Default plumbing.
#[derive(Debug, Default, cerulion_core::state::CerulionState)]
struct PortWriteLatches {
    cloud: FloodLatch,
    odom: FloodLatch,
    twist: FloodLatch,
    request: FloodLatch,
}

impl PortWriteLatches {
    fn get_mut(&mut self, t: RosType) -> &mut FloodLatch {
        match t {
            RosType::PointCloud2 => &mut self.cloud,
            RosType::SportModeState => &mut self.odom,
            RosType::Twist => &mut self.twist,
            RosType::Request => &mut self.request,
        }
    }
}

/// The generic DDS→Cerulion ingress bridge node. See the module docs.
// Field notes (plain comments — port fields carry only the macro attr):
// - cloud/odom/twist/request_json: the fixed output ports (registry table).
// - queue: shared with the pump helper thread (it pushes; tick drains).
// - total_evicted: running latest-wins eviction count (observability).
// - transport: the HOST's TransportManager, stored by `init` from the
//   NodeContext (the context-carried transport; the raw
//   ingress routes publish on it). None until init / for hand-rolled
//   contexts that never carried one.
#[cerulion_node(external)]
#[derive(Default)]
pub struct DdsBridge {
    #[output]
    cloud: PointCloud2,
    #[output]
    odom: Odometry,
    #[output]
    twist: TwistStamped,
    #[output]
    request_json: RosString,
    queue: Arc<Mutex<SampleQueue>>,
    total_evicted: u64,
    // tick_stats: shared tick-side observability counters (Principle #3).
    // write_latches: per-port write-error flood latches.
    tick_stats: Arc<TickStats>,
    write_latches: PortWriteLatches,
    transport: Option<Arc<TransportManager>>,
    // The SHARED shutdown coordinator, cloned off the pump in
    // `external_source` BEFORE the pump is moved into the Blocking closure. It
    // is the ONLY path `shutdown()` retains to signal + join the drain thread
    // (so the DDS participant drops → SPDP/SEDP disposes go out → readers
    // unregister) once the pump is closure-captured. None until external_source
    // builds the pump (config-error → HostDriven path never builds one).
    //
    // A HANDLE, not state. It holds an `AtomicBool` and a
    // `Mutex<Option<DrainThreadStop>>` — a stop handle for a THREAD that a
    // restored node does not have, so a captured one would name a drain
    // thread belonging to a process that has exited. The pump is rebuilt by
    // `external_source` on the restored run, which mints a fresh coordinator.
    #[cerulion(reconstruct)]
    pump_shutdown: Option<Arc<pump::PumpShutdown>>,
}

impl DdsBridge {
    /// Construct the node sharing a caller-provided sample queue — the test
    /// seam (the camera_jpeg `with_shared_queue` pattern): e2e tests inject
    /// decoded samples into the same queue the tick drains, and the live-DDS e2e
    /// hands the same queue to a directly-driven [`pump::BridgePump`].
    pub fn with_shared_queue(queue: Arc<Mutex<SampleQueue>>) -> Self {
        Self {
            queue,
            ..Default::default()
        }
    }

    /// Construct sharing both the sample queue and tick statistics for the
    /// observability seam. A holder of the returned [`TickStats`] `Arc` reads
    /// per-port write-error totals off the tick thread, including complete
    /// cloud drops when a payload exceeds its configured ceiling.
    pub fn with_shared_queue_and_stats(
        queue: Arc<Mutex<SampleQueue>>,
        tick_stats: Arc<TickStats>,
    ) -> Self {
        Self {
            queue,
            tick_stats,
            ..Default::default()
        }
    }

    /// Build the pump over a validated config, handing it the CONTEXT-CARRIED
    /// transport: `init` stored the host's `TransportManager`
    /// from the `NodeContext`, and the raw ingress routes publish on it. A
    /// cdylib must NEVER resolve `TransportManager::get()` — the cdylib links
    /// its OWN copy of cerulion_core, whose `INSTANCE` static the host never
    /// initialized (the dry-run failure: 149 backoff retries against a
    /// never-initialized static). `None` (a hand-rolled context that never
    /// carried a manager) is passed through; the pump errs LOUDLY at raw-init
    /// iff raw mappings actually need it (typed-only configs are unaffected).
    fn build_pump(&self, cfg: BridgeConfig) -> pump::BridgePump {
        match &self.transport {
            Some(t) => pump::BridgePump::with_transport(cfg, Arc::clone(&self.queue), t.clone()),
            None => pump::BridgePump::new(cfg, Arc::clone(&self.queue)),
        }
    }

    /// Build the Blocking source over a validated config. Lives in this PLAIN
    /// impl block (NOT `#[cerulion_node_impl]`) so the determinism
    /// lint never sees helper-thread plumbing — the camera_jpeg
    /// `gstreamer_external_source` delegation pattern. The closure itself is
    /// wall-clock-free: all wall time (poll pacing, init backoff) lives in
    /// [`pump::BridgePump::run_helper_iteration`], on the helper thread,
    /// outside the deterministic replay surface.
    fn pump_blocking_source(&mut self, cfg: BridgeConfig) -> ExternalSource {
        let mut pump = self.build_pump(cfg);
        // Retain the shared shutdown handle BEFORE moving the pump into
        // the closure — `shutdown()` uses it to signal + join the drain thread
        // even though the host's detached Blocking helper thread owns the pump.
        self.pump_shutdown = Some(pump.shutdown_handle());
        ExternalSource::Blocking(Box::new(move || pump.run_helper_iteration()))
    }
}

/// Split a node-clock ns stamp into ROS `(sec, nanosec)` (the go2_tf_source
/// arithmetic; used for the TwistStamped header, whose DDS source carries no
/// stamp of its own).
fn stamp_from_ns(now_ns: u64) -> (i32, u32) {
    (
        (now_ns / 1_000_000_000) as i32,
        (now_ns % 1_000_000_000) as u32,
    )
}

fn to_node_err(e: TransportError) -> NodeError {
    NodeError::Logic(e.to_string())
}

/// Pre-encode a `std_msgs/Header` sub-frame (stamp + frame_id) via a
/// STANDALONE `HeaderShm` writer over a heap scratch — the
/// `header_payload_oracle` construction (`cdylib_portwrite_e2e_test` /
/// `rewriter_var_field_assign_test`) promoted to production. Returns the
/// `[fixed(8) | offset table(8) | frame_id bytes]` prefix, byte-identical to
/// what the staged-header flush produces.
///
/// Write this header before the fields and data so capacity accounting includes
/// all metadata. The final data write can grow the adaptive loan; a payload
/// exceeding the configured ceiling leaves the output incomplete and discarded.
fn encode_header_bytes(
    frame_id: &str,
    stamp_sec: i32,
    stamp_nanosec: u32,
) -> Result<Vec<u8>, TransportError> {
    // Sized from the generated consts so schema drift cannot under-allocate:
    // fixed section + one offset-table entry per variable field + frame_id.
    let need = Header::WIRE_FIXED_SIZE + 8 * Header::VARIABLE_FIELD_COUNT + frame_id.len();
    // `Vec<u64>` scratch viewed as bytes (8-aligned; the safe helpers the
    // staged-flush machinery itself uses).
    let mut words = vec![0u64; need.div_ceil(8).max(1)];
    let cap = MaxPayloadCapacity::const_new((words.len() * 8) as u32);
    let mut w = Header::build_writer(
        u64s_as_bytes_mut(&mut words),
        cap,
        "dds_bridge/header".into(),
    );
    w.stamp.sec = stamp_sec;
    w.stamp.nanosec = stamp_nanosec;
    w.set_frame_id(frame_id)?;
    let end = w.cursor() as usize;
    Ok(u64s_as_bytes(&words)[..end].to_vec())
}

#[cerulion_node_impl]
impl DdsBridge {
    /// Stash the CONTEXT-CARRIED transport — the host's process
    /// `TransportManager`, injected into the `NodeContext` by the graph
    /// runtime before `init()` and carried across the cdylib FFI (the same
    /// path the ports take). The raw ingress routes publish on it; resolving
    /// `TransportManager::get()` instead would consult the CDYLIB's OWN copy
    /// of the `INSTANCE` static — never initialized by the host (the
    /// cross-linkage-unit trap the live dry-run hit: 149 backoff retries).
    fn init(&mut self, ctx: &mut NodeContext) -> Result<(), NodeError> {
        self.transport = ctx.transport().cloned();
        if self.transport.is_none() {
            // Not fatal here: only RAW mappings need the manager, and the
            // pump refuses loudly at raw-init. But a runtime-built context
            // always carries it, so a None is worth a breadcrumb.
            tracing::warn!(
                "dds_bridge: NodeContext carries no TransportManager (hand-rolled \
                 context?) — raw-generic mappings will fail loudly at pump init; \
                 typed projections are unaffected"
            );
        }
        Ok(())
    }

    /// Graceful DDS teardown. Reached from `NodeEntry::shutdown` on
    /// host teardown (SIGINT/SIGTERM/SIGHUP → `GraphRuntime::shutdown_all_nodes`,
    /// and via the cdylib `cerulion_node_shutdown` FFI). Signals the pump's
    /// drain thread to stop and JOINS it within [`pump::DRAIN_JOIN_DEADLINE`]:
    /// the drain executor returns, the rustdds participant drops, and its Drop
    /// sends the SPDP participant-dispose + per-endpoint SEDP disposes — so
    /// remote writers unregister THIS node's readers immediately instead of holding them
    /// as ghosts until the DDS lease ages out. No
    /// pump (config-error → HostDriven path, or the node never entered
    /// `external_source`) means nothing to stop. Never errors: a wedged drain
    /// thread is warned-and-detached inside the pump, never a failed teardown.
    fn shutdown(&mut self) -> Result<(), NodeError> {
        if let Some(handle) = &self.pump_shutdown {
            handle.shutdown(pump::DRAIN_JOIN_DEADLINE);
        } else {
            tracing::debug!(
                "dds_bridge: shutdown with no pump (never entered external_source, or a \
                 config error reported HostDriven) — no DDS participant to dispose"
            );
        }
        Ok(())
    }

    fn tick(&mut self) -> Result<(), NodeError> {
        let now = self.now_ns();

        // Drain the latest-wins queue (recover a poisoned lock — slot data is
        // replace-only, never torn; a transient helper panic must not wedge
        // the bridge).
        let (drained, evicted_total) = {
            let mut q = self
                .queue
                .lock()
                .unwrap_or_else(|poisoned| poisoned.into_inner());
            (q.drain_all(), q.lifetime_dropped_total())
        };

        if evicted_total > self.total_evicted {
            tracing::debug!(
                evicted = evicted_total - self.total_evicted,
                lifetime = evicted_total,
                "dds_bridge: stale samples evicted (latest wins)"
            );
            self.total_evicted = evicted_total;
        }

        if drained.samples.is_empty() {
            // The benign push-after-drain race (camera_jpeg module docs): the
            // previous fire's drain already served this ring's sample. Write
            // nothing — every output's frame is discarded by the output gate.
            tracing::debug!("dds_bridge tick fired with an empty queue");
            return Ok(());
        }

        // Fixed port order (queue drain order) — deterministic write sequence.
        // Each sample's write runs INDEPENDENTLY. A hostile sample (e.g. a
        // PointCloud2 whose re-encoded fields blob overflows the output slot)
        // must NOT abort the whole fire and silently drop the already-drained
        // sibling samples — its error is counted + flood-latched loudly and the
        // tick continues. The tick always returns Ok.
        for sample in drained.samples {
            let ros_type = sample.ros_type();
            let result = match &sample {
                BridgeSample::Cloud(c) => self.write_cloud(c),
                BridgeSample::Sport(s) => self.write_odom(s),
                BridgeSample::Twist(t) => self.write_twist(t, now),
                BridgeSample::Request(r) => self.write_request(r),
            };
            self.record_write_result(ros_type, result);
        }
        Ok(())
    }

    /// Route one port write's outcome through its flood latch + counter. A
    /// failure bumps the port's error counter and logs the flood-latched loud
    /// error (first-of-regime `error!`, sustained repeats `debug!` with a
    /// running count); a success clears a suppressed regime (recovery `info!`).
    /// The sample is dropped on error, but its siblings — already applied — are
    /// unaffected.
    fn record_write_result(&mut self, ros_type: RosType, result: Result<(), NodeError>) {
        match result {
            Ok(()) => {
                if let Some(suppressed) = self.write_latches.get_mut(ros_type).on_recovered() {
                    tracing::info!(
                        port = ros_type.port_name(),
                        suppressed_count = suppressed,
                        "dds_bridge: port write recovered (writes succeeding again)"
                    );
                }
            }
            Err(e) => {
                self.tick_stats.bump_write_error(ros_type);
                let total = self.tick_stats.write_errors_total(ros_type);
                match self.write_latches.get_mut(ros_type).on_event() {
                    FloodAction::First => tracing::error!(
                        port = ros_type.port_name(),
                        error = %e,
                        total,
                        "dds_bridge: port write FAILED — this sample was dropped, its \
                         already-drained siblings are unaffected (repeats log at debug)"
                    ),
                    FloodAction::Suppressed { suppressed } => tracing::debug!(
                        port = ros_type.port_name(),
                        error = %e,
                        suppressed,
                        total,
                        "dds_bridge: port write still failing (warn suppressed)"
                    ),
                }
            }
        }
    }

    /// PointCloud2 pass-through: fixed geometry verbatim, `fields` re-encoded
    /// as the packed blob (the cerulion_viz sink contract — `mapping` docs), point
    /// bytes copied STRAIGHT into the loaned SHM slice.
    ///
    /// Write metadata before data and use the size-aware data setter. The
    /// current adaptive loan can be smaller than the graph's payload ceiling;
    /// the setter grows it when needed. A frame above the ceiling is counted
    /// by `record_write_result` and discarded, never truncated.
    fn write_cloud(
        &mut self,
        c: &cerulion_go2_dds::messages::PointCloud2,
    ) -> Result<(), NodeError> {
        let plan = mapping::plan_cloud(c);
        self.cloud.height = plan.height;
        self.cloud.width = plan.width;
        // ROS bools are u8 on the generated wire schema; the plan keeps the
        // semantic bool, the conversion happens at the write site.
        self.cloud.is_bigendian = u8::from(plan.is_bigendian);
        self.cloud.point_step = plan.point_step;
        self.cloud.row_step = plan.row_step;
        self.cloud.is_dense = u8::from(plan.is_dense);

        // Header: DDS sensor stamp + frame pass-through, written FIRST (order
        // note above).
        let header_bytes = encode_header_bytes(&plan.frame_id, plan.stamp_sec, plan.stamp_nanosec)
            .map_err(to_node_err)?;
        self.cloud
            .set_header_bytes(&header_bytes)
            .map_err(to_node_err)?;

        self.cloud
            .set_fields_bytes(&plan.fields_blob)
            .map_err(to_node_err)?;

        // The DDS decoder already owns these bytes. Copy the full payload at
        // the ingress boundary; set_data can spill beyond the current adaptive
        // loan, while fill_from only sees that loan's remaining capacity.
        self.cloud.set_data(&c.data).map_err(to_node_err)?;
        Ok(())
    }

    /// SportModeState → Odometry projection (`mapping` docs: pose + twist;
    /// quaternion reordered from Unitree `[w,x,y,z]`). The pose/twist
    /// covariances are left at the loaned slot's ZERO default because
    /// SportModeState carries none — but an all-zero covariance is NOT
    /// "unknown" to a ROS pose-fusing consumer (an EKF reads zero variance as
    /// MAXIMUM certainty). Real covariance handling MUST be added before this
    /// odometry is fused (see `mapping::OdomPlan` docs).
    fn write_odom(
        &mut self,
        s: &cerulion_go2_dds::messages::SportModeState,
    ) -> Result<(), NodeError> {
        let plan = mapping::plan_odom(s);
        // Keep metadata writes in the same order as the cloud path: complete
        // header first, then child_frame_id.
        let header_bytes =
            encode_header_bytes(go2_tf::ODOM_FRAME, plan.stamp_sec, plan.stamp_nanosec)
                .map_err(to_node_err)?;
        self.odom
            .set_header_bytes(&header_bytes)
            .map_err(to_node_err)?;
        self.odom.child_frame_id = go2_tf::BASE_FRAME;
        self.odom.pose.pose.position.x = plan.position[0];
        self.odom.pose.pose.position.y = plan.position[1];
        self.odom.pose.pose.position.z = plan.position[2];
        self.odom.pose.pose.orientation.x = plan.orientation_xyzw[0];
        self.odom.pose.pose.orientation.y = plan.orientation_xyzw[1];
        self.odom.pose.pose.orientation.z = plan.orientation_xyzw[2];
        self.odom.pose.pose.orientation.w = plan.orientation_xyzw[3];
        self.odom.twist.twist.linear.x = plan.linear[0];
        self.odom.twist.twist.linear.y = plan.linear[1];
        self.odom.twist.twist.linear.z = plan.linear[2];
        self.odom.twist.twist.angular.z = plan.yaw_speed;
        Ok(())
    }

    /// Twist → TwistStamped: values verbatim; the header stamp comes from the
    /// NODE clock (the bare Twist carries none — deterministic, Principle #7)
    /// and the frame is the robot base.
    fn write_twist(
        &mut self,
        t: &cerulion_go2_dds::messages::Twist,
        now_ns: u64,
    ) -> Result<(), NodeError> {
        let (sec, nanosec) = stamp_from_ns(now_ns);
        // Header via the immediate whole-field write (consistency with
        // write_cloud/write_odom; here the header is the port's ONLY variable
        // field, so the staged deferral was harmless — but one mechanism
        // across all three header-bearing ports keeps the hazard class dead).
        let header_bytes =
            encode_header_bytes(go2_tf::BASE_FRAME, sec, nanosec).map_err(to_node_err)?;
        self.twist
            .set_header_bytes(&header_bytes)
            .map_err(to_node_err)?;
        self.twist.twist.linear.x = t.linear.x;
        self.twist.twist.linear.y = t.linear.y;
        self.twist.twist.linear.z = t.linear.z;
        self.twist.twist.angular.x = t.angular.x;
        self.twist.twist.angular.y = t.angular.y;
        self.twist.twist.angular.z = t.angular.z;
        Ok(())
    }

    /// Request → std_msgs/String: the sport-command `parameter` JSON, for
    /// observability of commands other processes send the robot.
    fn write_request(&mut self, r: &cerulion_go2_dds::messages::Request) -> Result<(), NodeError> {
        self.request_json.data = r.parameter.as_str();
        Ok(())
    }

    /// The bridge's external ingress source (module docs). Loads the config
    /// from [`config::CONFIG_ENV`]; a config error reports `HostDriven`,
    /// which the live path REFUSES loudly at launch.
    fn external_source(&mut self) -> ExternalSource {
        let cfg = match BridgeConfig::load_from_env() {
            Ok(cfg) => cfg,
            Err(e) => {
                tracing::error!(
                    error = %e,
                    "dds_bridge: config load failed — reporting HostDriven, which \
                     `cerulion graph run` refuses at launch as an inert external \
                     node. Fix the config and relaunch"
                );
                return ExternalSource::HostDriven;
            }
        };
        tracing::info!(
            mappings = cfg.mappings.len(),
            domain = cfg.domain_id,
            "dds_bridge: config loaded"
        );
        // Delegate to the plain impl block — the tick path stays
        // wall-clock-free (see pump_blocking_source's doc).
        self.pump_blocking_source(cfg)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stamp_from_ns_hand_oracle() {
        assert_eq!(stamp_from_ns(0), (0, 0));
        assert_eq!(stamp_from_ns(1_500_000_000), (1, 500_000_000));
        assert_eq!(stamp_from_ns(12_000_000_034), (12, 34));
    }

    #[test]
    fn encode_header_bytes_matches_hand_oracle() {
        // The pre-encoded Header sub-frame byte shape: [sec i32 LE | nanosec u32 LE | offset-entry (16, len) |
        // frame_id bytes] — the same layout the e2e's parse_stamp /
        // parse_header_frame_id observers decode.
        let got = encode_header_bytes("lidar", 12, 34).expect("encode header");
        let mut oracle = Vec::new();
        oracle.extend_from_slice(&12i32.to_le_bytes()); // stamp.sec
        oracle.extend_from_slice(&34u32.to_le_bytes()); // stamp.nanosec
        oracle.extend_from_slice(&16u32.to_le_bytes()); // frame_id offset
        oracle.extend_from_slice(&5u32.to_le_bytes()); // frame_id length
        oracle.extend_from_slice(b"lidar");
        assert_eq!(got, oracle, "header sub-frame must match the hand oracle");

        // Empty frame_id: still a complete 16-byte sub-frame (offset entry
        // 16/0, no variable bytes) — the frame_walker empty-header idiom's
        // stamped sibling.
        let empty = encode_header_bytes("", -3, 7).expect("encode empty frame_id");
        let mut empty_oracle = Vec::new();
        empty_oracle.extend_from_slice(&(-3i32).to_le_bytes());
        empty_oracle.extend_from_slice(&7u32.to_le_bytes());
        empty_oracle.extend_from_slice(&16u32.to_le_bytes());
        empty_oracle.extend_from_slice(&0u32.to_le_bytes());
        assert_eq!(empty, empty_oracle);
    }

    /// Regression pin: the CONTEXT-CARRIED transport reaches the raw
    /// routes and frames land. Tests that inject `with_transport` directly
    /// never exercise the cdylib-side transport resolution: a
    /// cdylib-side `TransportManager::get()` reads the cdylib's own
    /// never-initialized `INSTANCE` static, and a route resolving it never
    /// opens (149 backoff retries in one measured run). Chain pinned here, all in-process
    /// over an isolated `init_for_test` manager (parallel-safe):
    ///
    ///   set_transport(ctx) → DdsBridge::init stores it (ptr_eq) →
    ///   build_pump carries it (ptr_eq — the external_source delegation) →
    ///   a raw route OPENED ON THE CARRIED manager publishes → the frame
    ///   lands on a subscriber of the ORIGINAL manager (hand oracle).
    ///
    /// The last hop is the namespace pin: the failure mode is the
    /// route resolving a DIFFERENT manager (wrong SHM namespace — frames
    /// silently lost), so delivery THROUGH the original manager's subscriber
    /// proves carried == original, not merely "some transport".
    #[test]
    fn context_carried_transport_reaches_the_raw_route_and_frames_land() {
        use cerulion_core::clock::VirtualClock;
        use cerulion_core::message::ShmMessage;
        use cerulion_core::wire::{MaxSliceLen, WireHeader};
        use indexmap::IndexMap;

        let mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "p0_ctx".to_string(),
                clock: Arc::new(VirtualClock::new()),
                subscriber_buffer_size: 16,
                network: None,
            },
            cerulion_core::testing::iceoryx_test_config(),
        )
        .expect("isolated transport");

        // A runtime-shaped context: transport injected before init (the
        // graph runtime does exactly this via set_transport).
        let mut ctx = NodeContext::for_tests(IndexMap::new(), IndexMap::new());
        assert!(
            ctx.transport().is_none(),
            "a hand-rolled context starts without a transport"
        );
        ctx.set_transport(Arc::clone(&mgr));

        // init (the macro-renamed user hook) stores the carried manager.
        let mut node = DdsBridge::with_shared_queue(Arc::new(Mutex::new(SampleQueue::default())));
        node.__cer_user_init(&mut ctx).expect("init succeeds");
        assert!(
            Arc::ptr_eq(node.transport.as_ref().expect("init stored it"), &mgr),
            "DdsBridge::init must store the CONTEXT's manager (ptr_eq)"
        );

        // build_pump (external_source's delegation) hands the SAME manager on.
        let cfg = BridgeConfig::from_yaml(
            "mappings:\n  - dds_topic: /imu\n    ros_type: sensor_msgs/Imu\n    \
             cerulion_topic: /p0ctx/imu\n",
            "p0 test",
        )
        .expect("raw config validates");
        let pump = node.build_pump(cfg);
        let carried = Arc::clone(
            pump.transport
                .as_ref()
                .expect("the pump must carry the context transport"),
        );
        assert!(
            Arc::ptr_eq(&carried, &mgr),
            "build_pump must hand the pump the CONTEXT's manager (ptr_eq)"
        );

        // The frame lands: a raw route opened on the CARRIED manager delivers
        // to a subscriber created from the ORIGINAL manager (same namespace).
        let codec = crate::generic::bridge_codec();
        let topic = "/p0ctx/v3";
        let mut route = crate::generic::RawIngressRoute::open(
            &carried,
            &codec,
            "geometry_msgs/Vector3",
            topic,
            MaxSliceLen::const_new(4096),
        )
        .expect("raw route opens on the carried manager");
        let sub = mgr
            .create_subscriber(topic)
            .expect("observer on the ORIGINAL manager");

        let mut payload = vec![0x00u8, 0x01, 0x00, 0x00]; // CDR_LE encapsulation
        for v in [1.5f64, -2.5, 3.25] {
            payload.extend_from_slice(&v.to_le_bytes());
        }
        route
            .publish_dds_payload(&codec, &payload, 7)
            .expect("publish on the carried manager");

        let mut frames: Vec<Vec<u8>> = Vec::new();
        sub.try_receive(|msg| {
            let mut f = vec![0u8; WireHeader::SIZE];
            msg.header().write_to_buf(&mut f);
            f.extend_from_slice(msg.payload());
            frames.push(f);
        })
        .expect("try_receive");

        // Hand oracle (the raw_route_test frame shape).
        let header = WireHeader {
            schema_hash: <native_ros2_messages::geometry_msgs::Vector3 as ShmMessage>::SCHEMA_HASH,
            total_size: (WireHeader::SIZE + 24) as u32,
            offset_table_offset: (WireHeader::SIZE + 24) as u32,
            offset_table_count: 0,
            sequence: 0,
            timestamp_ns: 7,
        };
        let mut expected = vec![0u8; WireHeader::SIZE];
        header.write_to_buf(&mut expected);
        for v in [1.5f64, -2.5, 3.25] {
            expected.extend_from_slice(&v.to_le_bytes());
        }
        assert_eq!(
            frames,
            vec![expected],
            "the context-carried transport delivers into the ORIGINAL manager's namespace"
        );
    }

    /// The node→pump shutdown SEAM (hermetic, no DDS). The production
    /// chain is `external_source` → `pump_blocking_source` (stashes the shared
    /// [`pump::PumpShutdown`] handle BEFORE moving the pump into the Blocking
    /// closure) → `DdsBridge::shutdown` signals + joins the drain thread through
    /// that retained handle. This exercises the seam without driving the
    /// closure (so no drain thread ever spawns): the handle must be present
    /// after delegation, and the shutdown hook must be idempotent + never error.
    /// The end-to-end dispose (drain thread → participant Drop → SPDP/SEDP
    /// dispose observed by a second participant) is `tests/shutdown_dispose_test.rs`.
    #[test]
    fn external_source_delegation_stashes_shutdown_handle_and_hook_is_idempotent() {
        let mut node = DdsBridge::with_shared_queue(Arc::new(Mutex::new(SampleQueue::default())));
        assert!(
            node.pump_shutdown.is_none(),
            "no pump/shutdown handle before external_source delegation"
        );

        let cfg = BridgeConfig::from_yaml(
            "mappings:\n  - dds_topic: /x\n    ros_type: geometry_msgs/Twist\n    \
             cerulion_topic: /y/x\n",
            "seam test",
        )
        .expect("typed config validates");
        // The exact delegation external_source performs (private, in-crate).
        let _source = node.pump_blocking_source(cfg);
        assert!(
            node.pump_shutdown.is_some(),
            "pump_blocking_source retains the shared shutdown handle before the pump \
             is moved into the Blocking closure"
        );

        // The production shutdown hook (macro-renamed). No drain was ever driven,
        // so it finds nothing to stop → Ok; a second call is idempotent (no
        // double-join / panic), exactly as a repeated host teardown would be.
        node.__cer_user_shutdown()
            .expect("shutdown hook returns Ok");
        node.__cer_user_shutdown()
            .expect("second shutdown is idempotent (no panic)");
    }
}

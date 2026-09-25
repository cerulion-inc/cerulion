// SPDX-License-Identifier: AGPL-3.0-only
//! END-TO-END acceptance for the `cerulion-vizd` daemon over
//! real iceoryx2 + a real UDS control connection (`#[cfg(unix)]`).
//!
//! The daemon runs IN-PROCESS over an ISOLATED per-test SHM root (the
//! `tap_manager_test.rs` precedent — `TransportManager::init_for_test`): the
//! TRANSPORT plane is per-test (its own transport, its own temp socket, its own
//! memory Rerun sink), so no test uses `#[serial]`. The BLUEPRINT plane is not
//! — see the blueprint-statics note below; 52 of the 71 tests hold a file-local mutex for their
//! whole body, so the file is only partly parallel in practice. Every frame is a
//! HAND-BUILT `geometry_msgs/Vector3` published by a REAL producer (Principle
//! #13), so the render is checked against the known `Vector3 → three Scalar
//! plots` oracle and the protocol responses against hand oracles (never a
//! self-compare).
//!
//! Bounded waits everywhere; the daemon + publisher threads stop on drop.
//!
//! **The ONE thing that is NOT per-test: the blueprint statics.**
//! `cerulion_viz::blueprint`'s `RUNTIME_BLUEPRINT` / `BLUEPRINT_SENT` /
//! `ATTACHED_ENTITIES_SNAPSHOT` are PROCESS-global, and THREE distinct code
//! paths reach them — a mutating vizd verb on a controller thread, the daemon's
//! own POLL thread on a late schema resolve, and every worker's BOOT
//! `ensure_setup`. Tests that can be affected serialize on
//! [`BLUEPRINT_STATICS_LOCK`] via [`blueprint_statics_guard`]; that static's
//! docs enumerate the three writer classes, give the mechanical rule for when a
//! test must lock, and — importantly — state precisely what the lock does NOT
//! exclude. Read them before adding a test that asserts on blueprint content.
//! Locking only the two static-READING tests is not enough: with the
//! ~26 verb-issuing writers unlocked,
//! `compose_layout_remembers_the_plan_for_reconnect_reapply` reads a sibling's
//! plan 1 run in 14 at `--test-threads=16`.
//!
//! **HERMETIC on the netd plane.** No test in this file may start the
//! daemon through the production [`cerulion_vizd::start`], because that injects
//! `NetdDemandPlane::new()` → the WELL-KNOWN `cerulion-netd` control socket
//! (`$CERULION_NETD_SOCK` / `$XDG_RUNTIME_DIR/cerulion/netd.sock` /
//! `$HOME/.cerulion/netd.sock`). On a dev desk that already runs the real netd (or
//! merely has `cerulion-netd` next to the test binary, which the client SPAWNS
//! detached) the daemon's `discover` gather and every schema-less remote attach reach
//! a LIVE netd and, through it, whatever is on the LAN — so the test's verdict
//! depends on ambient desk + network state (for example, the network-less
//! remote-attach test passes on an empty LAN and fails on a desk with a robot,
//! refused with "robot 'go2' serves a catalog but names no ROS type"
//! because a live go2 answered). Every start therefore goes through
//! [`start_hermetic`], which injects an EXPLICIT [`DemandPlane`](cerulion_vizd::DemandPlane)
//! double — [`NoNetdPlane`] by default (the "no netd on this desk" model, the
//! state these oracles were written against) — so the netd/LAN state a test asserts
//! against is DECLARED in the test, never inherited from the machine.

#![cfg(unix)]

use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::clock::{Clock, VirtualClock};
// The production canonical-v1 WRITER (`CdrCodec`, the exact
// codec `ros2 attach`'s dds_bridge runs) + the schema corpus it is built from,
// plus the walker value type for the undecodable-frame premise check.
use cerulion_core::codegen::parse_rosmsg;
use cerulion_core::codegen::{CdrCodec, CdrEndianness, FrameValueKind, FrameWalker, MessageSchema};
use cerulion_core::message::ShmMessage;
// Hand-build the negative-control (undecodable) `/plan` frame.
use cerulion_core::shm_runtime::write_offset_entry;
use cerulion_core::transport::network::NetworkConfig;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::{
    GatewayEgressPolicy, GatewayPlan, GatewayRuntime, SchemaDoc, SchemaEncoding, SchemaServing,
    TopicSchema,
};
use cerulion_netd::daemon::{self as netd_daemon, NetdConfig, RunningNetd};
use cerulion_netd::mirror::{GatewayMirrorPlane, MirrorError, MirrorPlane, MirrorRelease};
use cerulion_netd::protocol::DiscoveryState;
use cerulion_netd::registry::TopicKey;
use cerulion_netd::CatalogGather;
use cerulion_viz::blueprint::{
    apply_runtime_blueprint, blueprint_decorations, blueprint_panel_timeline,
    blueprint_property_paths, clear_runtime_blueprint, current_runtime_blueprint_plan,
    default_layout, rearm_blueprint, send_blueprint_once, BlueprintPlan, PlanNode,
};
// The PURE element-array extractor the sink ladder consumes —
// used to anchor the codec-produced `/plan` frame to its hand oracle BEFORE it
// is published, so the daemon's render is proven to be of real decoded data.
use cerulion_viz::archetype::{scan_element_arrays, ElementArrayScan, ElementGeometry};
// The PRODUCTION octave-band predicate. `stall_reachable` asks whether
// a `rate_deviation` qualifying run is pending, which is a question about the
// engine's own band — a second copy of the octave here would be free to disagree
// with the one the verdict is actually taken against.
use cerulion_viz::monitor::rate_deviates;
use cerulion_viz::schema_registry::builtin_walker;
use cerulion_viz::sink::SinkState;
use cerulion_viz::worker::VizLogWorker;
// The production `start` is deliberately NOT imported — it injects the
// real `NetdDemandPlane` (the well-known netd socket → ambient desk + LAN state).
// Tests go through `start_hermetic`, which injects an explicit plane double.
use cerulion_vizd::{
    start_with_demand_plane, NetdDemandPlane, RunningDaemon, DEFAULT_POLL_INTERVAL,
    MAX_REQUEST_LINE_BYTES,
};
use native_ros2_messages::geometry_msgs::Vector3;
use native_ros2_messages::nav_msgs::Path;
// The video-layout witness — a CompressedImage carrying H.264
// classifies as `VideoStream` by CONTENT, which is how the Go2's camera arrives.
use native_ros2_messages::sensor_msgs::CompressedImage;
use native_ros2_messages::visualization_msgs::MarkerArray;
// Read back what the daemon RENDERED — the memory sink's
// `LogMsg`s parsed into `re_chunk::Chunk`s so the `/plan` subtree's archetypes
// (a `LineStrips3D` polyline, NOT a `TextDocument` dump) are inspectable. Same
// rerun-family dev-deps `live_only_history_test.rs` already uses.
use re_log_types::{LogMsg, StoreKind};
use serde_json::Value;

/// A RECORDING spy [`cerulion_vizd::DemandPlane`] (a DI test double —
/// Principle #13, not fake data) that SUCCEEDS every demand/release but creates NO
/// real mirror, so the daemon's tap step fails on the non-existent `{topic}/data`
/// service — the fault the rollback test needs. Records every demand + release for
/// hand-oracle assertions.
#[derive(Default)]
struct RecordingDemandPlane {
    demands: std::sync::Mutex<Vec<(String, String, u64)>>,
    releases: std::sync::Mutex<Vec<(String, String)>>,
}
impl cerulion_vizd::DemandPlane for RecordingDemandPlane {
    fn demand(&self, robot: &str, topic: &str, schema_hash: u64) -> Result<(), String> {
        self.demands
            .lock()
            .unwrap()
            .push((robot.to_string(), topic.to_string(), schema_hash));
        Ok(())
    }
    fn release(&self, robot: &str, topic: &str) -> Result<(), String> {
        self.releases
            .lock()
            .unwrap()
            .push((robot.to_string(), topic.to_string()));
        Ok(())
    }
    // This rollback-focused double serves no query surface — an empty
    // (authoritative) answer; these tests never drive the catalog/schema resolve.
    fn query_catalog(&self, _robot: &str) -> Result<Vec<cerulion_core::CatalogReply>, String> {
        Ok(Vec::new())
    }
    // No LAN gather in the rollback tests — an empty authoritative answer.
    fn query_catalog_all(&self) -> Result<Vec<cerulion_core::CatalogReply>, String> {
        Ok(Vec::new())
    }
    fn query_schema(
        &self,
        _robot: &str,
        _requested: &str,
    ) -> Result<Vec<cerulion_core::SchemaReply>, String> {
        Ok(Vec::new())
    }
}

/// A spy [`cerulion_vizd::DemandPlane`] whose LAN gather (`query_catalog_all`)
/// returns HAND-BUILT catalogs (or an armed netd-unreachable `Err`) — the DI seam for
/// the `discover` ROBOTS-section e2e (no netd process, no network). Every other verb
/// is inert (this double drives only `discover`).
struct GatherPlane {
    catalogs: Vec<cerulion_core::CatalogReply>,
    fail: std::sync::atomic::AtomicBool,
    /// The [`DiscoveryState`] this double reports.
    ///
    /// The trait DEFAULT is `NotConverged` ("this plane cannot tell you"), which is
    /// the right fail-closed answer for a double that knows nothing — but on its own it
    /// means NO test can reach the leftover-mirror drop on the production
    /// `discover` path, because that drop requires a SETTLED gather. With the feature
    /// pinned only by pure fold arms passing `Settled` by hand, hardcoding
    /// `NotConverged` in `discover` would leave it fully inert with the whole suite green.
    /// A double that wants to license an absence claim has to SAY so.
    discovery: std::sync::Mutex<DiscoveryState>,
}
impl GatherPlane {
    fn new(catalogs: Vec<cerulion_core::CatalogReply>) -> Self {
        Self {
            catalogs,
            fail: std::sync::atomic::AtomicBool::new(false),
            discovery: std::sync::Mutex::new(DiscoveryState::NotConverged),
        }
    }
    fn set_fail(&self, fail: bool) {
        self.fail.store(fail, Ordering::SeqCst);
    }
    /// Declare what netd would have reported for this gather.
    fn set_discovery(&self, discovery: DiscoveryState) {
        *self.discovery.lock().unwrap() = discovery;
    }
}
impl cerulion_vizd::DemandPlane for GatherPlane {
    /// Report the DECLARED discovery state, so a test can license
    /// the leftover-mirror drop (or withhold that licence) explicitly.
    fn query_catalog_all_with_discovery(&self) -> Result<CatalogGather, String> {
        let discovery = *self.discovery.lock().unwrap();
        self.query_catalog_all().map(|catalogs| CatalogGather {
            catalogs,
            discovery,
            // This double models a netd that ANSWERED, so there is no
            // un-settled plane age to report. `None` is UNKNOWN and caps no wait.
            unsettled_for: None,
        })
    }
    fn demand(&self, _robot: &str, _topic: &str, _schema_hash: u64) -> Result<(), String> {
        Ok(())
    }
    fn release(&self, _robot: &str, _topic: &str) -> Result<(), String> {
        Ok(())
    }
    fn query_catalog(&self, _robot: &str) -> Result<Vec<cerulion_core::CatalogReply>, String> {
        Ok(Vec::new())
    }
    fn query_catalog_all(&self) -> Result<Vec<cerulion_core::CatalogReply>, String> {
        if self.fail.load(Ordering::SeqCst) {
            return Err("gather spy: netd unreachable".to_string());
        }
        Ok(self.catalogs.clone())
    }
    fn query_schema(
        &self,
        _robot: &str,
        _requested: &str,
    ) -> Result<Vec<cerulion_core::SchemaReply>, String> {
        Ok(Vec::new())
    }
}

/// A DI [`cerulion_vizd::DemandPlane`] that models the netd mirror LIFECYCLE
/// in-process (no netd, no zenoh, Principle #13): `demand` creates a REAL publisher on
/// the shared manager so vizd's tap opens the `{topic}/data` service; `release` DROPS it
/// so the mirror is torn down (`data_service_missing` flips true — the exact
/// uncheck-tears-down-the-mirror shape the round-trip needs). Its `query_catalog*`
/// serve HAND-BUILT catalogs so the schema-less remote attach resolves the type. Records
/// every `(robot, topic, hash)` demanded + every `(robot, topic)` released (hand oracle).
struct MirrorSpyPlane {
    mgr: Arc<TransportManager>,
    catalogs: Vec<cerulion_core::CatalogReply>,
    /// The [`DiscoveryState`] this double reports for its gather.
    ///
    /// Fail-closed default `NotConverged` ("this plane cannot tell you"), matching the
    /// trait default and [`GatherPlane`]'s rule: a double that wants to license an
    /// ABSENCE CLAIM — and `attach_resolving_remote`'s "no reachable robot serves it"
    /// is one — has to SAY so. Every arm that resolves to a ROBOT is unaffected (a
    /// found robot is positive evidence at any discovery state), which is why only the
    /// two absence tests below set it.
    discovery: std::sync::Mutex<DiscoveryState>,
    /// The held publisher per demanded topic (the "mirror"); dropping it removes the
    /// service — keyed by topic. `Box<dyn Any + Send>` so the concrete publisher type
    /// need not be named here.
    mirrors: std::sync::Mutex<std::collections::HashMap<String, Box<dyn std::any::Any + Send>>>,
    demands: std::sync::Mutex<Vec<(String, String, u64)>>,
    releases: std::sync::Mutex<Vec<(String, String)>>,
}
impl MirrorSpyPlane {
    fn new(mgr: Arc<TransportManager>, catalogs: Vec<cerulion_core::CatalogReply>) -> Self {
        Self {
            mgr,
            catalogs,
            mirrors: std::sync::Mutex::new(std::collections::HashMap::new()),
            demands: std::sync::Mutex::new(Vec::new()),
            releases: std::sync::Mutex::new(Vec::new()),
            discovery: std::sync::Mutex::new(DiscoveryState::NotConverged),
        }
    }
    /// Declare what netd would have reported for this gather — the licence
    /// (or withholding) for an absence claim on the attach path.
    fn set_discovery(&self, discovery: DiscoveryState) {
        *self.discovery.lock().unwrap() = discovery;
    }
    fn demand_keys(&self) -> Vec<(String, String, u64)> {
        self.demands.lock().unwrap().clone()
    }
    fn release_keys(&self) -> Vec<(String, String)> {
        self.releases.lock().unwrap().clone()
    }
}
impl cerulion_vizd::DemandPlane for MirrorSpyPlane {
    /// Report the DECLARED discovery state (fail-closed `NotConverged` unless
    /// a test explicitly licenses an absence claim via [`MirrorSpyPlane::set_discovery`]).
    fn query_catalog_all_with_discovery(&self) -> Result<CatalogGather, String> {
        let discovery = *self.discovery.lock().unwrap();
        self.query_catalog_all().map(|catalogs| CatalogGather {
            catalogs,
            discovery,
            // This double models a netd that ANSWERED, so there is no
            // un-settled plane age to report. `None` is UNKNOWN and caps no wait.
            unsettled_for: None,
        })
    }
    fn demand(&self, robot: &str, topic: &str, schema_hash: u64) -> Result<(), String> {
        // Create the mirror as a REAL publisher on the shared manager, so vizd's tap
        // opens `{topic}/data`. Held (behind the map) until `release` drops it.
        let publisher = self
            .mgr
            .create_publisher(topic, MaxSliceLen::const_new(1 << 16), 0)
            .map_err(|e| e.to_string())?;
        self.mirrors
            .lock()
            .unwrap()
            .insert(topic.to_string(), Box::new(publisher));
        self.demands
            .lock()
            .unwrap()
            .push((robot.to_string(), topic.to_string(), schema_hash));
        Ok(())
    }
    fn release(&self, robot: &str, topic: &str) -> Result<(), String> {
        self.mirrors.lock().unwrap().remove(topic); // drop → the mirror service goes away
        self.releases
            .lock()
            .unwrap()
            .push((robot.to_string(), topic.to_string()));
        Ok(())
    }
    fn query_catalog(&self, _robot: &str) -> Result<Vec<cerulion_core::CatalogReply>, String> {
        Ok(self.catalogs.clone())
    }
    fn query_catalog_all(&self) -> Result<Vec<cerulion_core::CatalogReply>, String> {
        Ok(self.catalogs.clone())
    }
    fn query_schema(
        &self,
        _robot: &str,
        _requested: &str,
    ) -> Result<Vec<cerulion_core::SchemaReply>, String> {
        Ok(Vec::new())
    }
}

/// The HERMETIC default [`cerulion_vizd::DemandPlane`] — every verb returns
/// the loud netd-unreachable `Err` a desk with NO `cerulion-netd` running produces.
/// This is what the production `NetdDemandPlane` surfaces when it cannot connect (a
/// `ClientError`), so a daemon built on it takes exactly the documented degrade paths:
/// `discover` shows LOCAL topics only, and a schema-less remote attach falls back to
/// vizd's OWN (possibly absent) network — the state every oracle in this file was
/// written against, now DECLARED instead of inherited from the machine.
///
/// A test that needs a REACHABLE netd states so explicitly (an in-process netd via
/// [`start_in_process_netd`], or a serving double like [`GatherPlane`]).
struct NoNetdPlane;

impl NoNetdPlane {
    /// The verbatim error every verb returns (the netd-unreachable shape).
    const UNREACHABLE: &'static str =
        "hermetic plane: no cerulion-netd on this desk (connect refused)";
}

impl cerulion_vizd::DemandPlane for NoNetdPlane {
    fn demand(&self, _robot: &str, _topic: &str, _schema_hash: u64) -> Result<(), String> {
        Err(Self::UNREACHABLE.to_string())
    }
    fn release(&self, _robot: &str, _topic: &str) -> Result<(), String> {
        Err(Self::UNREACHABLE.to_string())
    }
    fn query_catalog(&self, _robot: &str) -> Result<Vec<cerulion_core::CatalogReply>, String> {
        Err(Self::UNREACHABLE.to_string())
    }
    fn query_catalog_all(&self) -> Result<Vec<cerulion_core::CatalogReply>, String> {
        Err(Self::UNREACHABLE.to_string())
    }
    fn query_schema(
        &self,
        _robot: &str,
        _requested: &str,
    ) -> Result<Vec<cerulion_core::SchemaReply>, String> {
        Err(Self::UNREACHABLE.to_string())
    }
}

/// A SERVING [`cerulion_vizd::DemandPlane`] — a netd that IS reachable and
/// answers the catalog/schema GETs from HAND-BUILT replies (no netd process, no
/// zenoh). The mirror verbs succeed vacuously (no real mirror is created), so this
/// double is for arms that must get PAST the type resolve to reach a later guard.
struct ServingNetdPlane {
    catalogs: Vec<cerulion_core::CatalogReply>,
    /// The [`DiscoveryState`] this double reports for its gather.
    ///
    /// Fail-closed `NotConverged` like the trait default and the sibling doubles: the
    /// SINGLE-robot query verb feeds `catalog_resolve_type`, whose "no type for this
    /// topic" answer is an ABSENCE CLAIM, so a double has to SAY it may be believed.
    discovery: std::sync::Mutex<DiscoveryState>,
}

impl ServingNetdPlane {
    fn new(catalogs: Vec<cerulion_core::CatalogReply>) -> Self {
        Self {
            catalogs,
            discovery: std::sync::Mutex::new(DiscoveryState::NotConverged),
        }
    }
    /// Declare what netd would have reported for this gather.
    fn set_discovery(&self, discovery: DiscoveryState) {
        *self.discovery.lock().unwrap() = discovery;
    }
}

impl cerulion_vizd::DemandPlane for ServingNetdPlane {
    /// Report the DECLARED discovery state on the SINGLE-robot verb.
    fn query_catalog_with_discovery(&self, robot: &str) -> Result<CatalogGather, String> {
        let discovery = *self.discovery.lock().unwrap();
        self.query_catalog(robot).map(|catalogs| CatalogGather {
            catalogs,
            discovery,
            // See [`GatherPlane`] — a double that answered reports no age.
            unsettled_for: None,
        })
    }
    fn demand(&self, _robot: &str, _topic: &str, _schema_hash: u64) -> Result<(), String> {
        Ok(())
    }
    fn release(&self, _robot: &str, _topic: &str) -> Result<(), String> {
        Ok(())
    }
    fn query_catalog(&self, robot: &str) -> Result<Vec<cerulion_core::CatalogReply>, String> {
        Ok(self
            .catalogs
            .iter()
            .filter(|c| c.robot == robot)
            .cloned()
            .collect())
    }
    fn query_catalog_all(&self) -> Result<Vec<cerulion_core::CatalogReply>, String> {
        Ok(self.catalogs.clone())
    }
    fn query_schema(
        &self,
        _robot: &str,
        _requested: &str,
    ) -> Result<Vec<cerulion_core::SchemaReply>, String> {
        Ok(Vec::new())
    }
}

/// A trivially-succeeding `cerulion_netd` `MirrorPlane` (ensure → Ok,
/// release → Retired) — for the reconnect test, which exercises the DEMAND plane
/// against a real netd daemon without needing a real transport/mirror.
struct OkMirrorPlane;
impl MirrorPlane for OkMirrorPlane {
    fn ensure_mirror(&self, _key: &TopicKey, _schema_hash: u64) -> Result<(), MirrorError> {
        Ok(())
    }
    fn release_mirror(&self, _key: &TopicKey) -> MirrorRelease {
        MirrorRelease::Retired
    }
}

/// Start an in-process netd with the always-ok mirror plane on an explicit socket
/// (long idle grace so it never self-exits mid-test).
fn start_netd_ok_plane(socket: PathBuf) -> RunningNetd {
    let plane: Arc<dyn MirrorPlane> = Arc::new(OkMirrorPlane);
    netd_daemon::start(
        socket,
        plane,
        NetdConfig {
            idle_grace: Duration::from_secs(3600),
            idle_watch_poll: Duration::from_millis(50),
            ..NetdConfig::default()
        },
    )
    .expect("in-process netd (ok plane) starts")
}

/// Start an IN-PROCESS `cerulion-netd` sharing `mgr` (so its demand
/// plane creates the shared mirror + provenance on the SAME transport the vizd
/// daemon taps). The e2e injects `NetdDemandPlane::with_socket(<returned socket>)`
/// into `start_with_demand_plane`, so the full `attach_remote` → demand → netd
/// mirror → tap composition runs in-process (the migration proven end to end).
fn start_in_process_netd(tag: &str, mgr: Arc<TransportManager>) -> (RunningNetd, PathBuf, PathBuf) {
    use std::sync::atomic::AtomicU64;
    static N: AtomicU64 = AtomicU64::new(0);
    let n = N.fetch_add(1, Ordering::Relaxed);
    let dir = std::env::temp_dir().join(format!("netd_{tag}_{}_{n}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("mk netd tempdir");
    let sock = dir.join("netd.sock");
    // Wire the mirror plane AND the query plane over the SAME networked
    // manager, so vizd's schema-less catalog/schema resolve rides the REAL netd query
    // path (not the transient fallback) end-to-end over the serving gateway.
    let plane: Arc<dyn MirrorPlane> = Arc::new(GatewayMirrorPlane::new(Arc::clone(&mgr)));
    let query: Arc<dyn cerulion_netd::query::QueryPlane> =
        Arc::new(cerulion_netd::query::GatewayQueryPlane::new(mgr));
    let netd = netd_daemon::start_with_planes(
        sock.clone(),
        plane,
        Arc::new(cerulion_netd::egress::NoopEgressPlane),
        query,
        NetdConfig {
            idle_grace: Duration::from_secs(3600), // never idle-exit mid-test
            idle_watch_poll: Duration::from_millis(50),
            ..NetdConfig::default()
        },
    )
    .expect("in-process netd starts");
    (netd, sock, dir)
}

// ── Frame + transport helpers (crib tap_manager_test.rs) ────────────────────

/// A full `geometry_msgs/Vector3` wire frame (3 f64, fixed-only, empty offset
/// table). Byte-exact + deterministic.
/// The wire-stamp step the `Vector3` fixtures advance per frame.
///
/// Load-bearing since the plot rate gate: a `Vector3` shape-infers to `Scalars`
/// (3 series ⇒ a 1.5 ms minimum wire gap), and the fixtures publish at ~100 Hz of
/// WALL time. Stamping them 1 µs apart would model a
/// publisher whose clock runs 10 000x slower than real time, which the gate
/// correctly throttles to one render per 1 500 frames. 10 ms per frame is what a
/// 100 Hz producer actually stamps, so the fixtures match the cadence they
/// publish at.
const PUBLISH_STAMP_STEP_NS: u64 = 10_000_000;

fn build_vector3_frame(seq: u32, ts: u64, v: [f64; 3]) -> Vec<u8> {
    let mut payload = Vec::with_capacity(24);
    for x in v {
        payload.extend_from_slice(&x.to_le_bytes());
    }
    let header = WireHeader {
        schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_count: 0,
        sequence: seq,
        timestamp_ns: ts,
    };
    let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(&payload);
    frame
}

/// A full CUSTOM `probe/Widget` wire frame (two f64 `x`,`y` — padding-free,
/// so the hand-built layout needs no alignment guesswork). Stamped with the caller-
/// supplied `schema_hash` (the recipe-3 hash of the `.msg`), so the daemon's
/// gateway ingress hash-validates it against the SEEDED walker's hash — a mis-seed
/// (wrong hash) would drop every frame. Published raw via `publish_raw` (there is
/// no generated `probe::Widget` Rust type — this is a robot-only custom type).
fn build_widget_frame(schema_hash: u64, seq: u32, ts: u64, x: f64, y: f64) -> Vec<u8> {
    let mut payload = Vec::with_capacity(16);
    payload.extend_from_slice(&x.to_le_bytes());
    payload.extend_from_slice(&y.to_le_bytes());
    let header = WireHeader {
        schema_hash,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_count: 0,
        sequence: seq,
        timestamp_ns: ts,
    };
    let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(&payload);
    frame
}

/// A fresh isolated transport (per-test SHM root → parallel-safe).
fn isolated_transport(node_name: &str) -> Arc<TransportManager> {
    isolated_transport_inner(node_name, None)
}

/// A fresh isolated transport WITH a (lazy, scouting-off) network config, so the
/// remote arm's `register_ingress_topic` can declare gateway ingress (the
/// gateway-ingress seam pin). The zenoh session stays lazy — it opens only on the first
/// register — and is torn down when the manager drops (the `network_ingress_test`
/// precedent).
fn isolated_transport_with_network(node_name: &str) -> Arc<TransportManager> {
    isolated_transport_inner(node_name, Some(NetworkConfig::default()))
}

/// A network-configured daemon transport that CONNECTS to a gateway on
/// `127.0.0.1:{port}` (scouting OFF = hermetic), so the daemon's ONE zenoh session
/// reaches a test gateway over a real loopback TCP hop. The session stays lazy
/// (opens on the first catalog GET / ingress register). Per-test SHM root.
fn isolated_transport_connecting_to(node_name: &str, port: u16) -> Arc<TransportManager> {
    isolated_transport_inner(
        node_name,
        Some(NetworkConfig {
            connect_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
            ..NetworkConfig::default()
        }),
    )
}

/// The netd-load-bearing discriminator: TWO managers on
/// ONE shared (machine-B) SHM root — a NETWORKED `netd_mgr` that CONNECTS to the
/// serving gateway on `port`, and a `vizd_mgr` whose network reaches NOBODY (scouting
/// OFF, no connect/listen endpoints — the `NetworkConfig::default()` island). netd
/// owns the ONLY reachable session (query + mirror); vizd's OWN transient fallback
/// therefore CANNOT resolve — so a successful schema-less resolve PROVES netd served
/// it. `vizd_mgr` is still `network().is_some()` (it passes the daemon's 2a
/// network-configured guard), it just reaches nobody. Both share the root so netd's
/// re-injected mirror stays tappable by vizd.
fn netd_and_vizd_on_one_root(
    tag: &str,
    port: u16,
) -> (Arc<TransportManager>, Arc<TransportManager>) {
    let root = cerulion_core::testing::iceoryx_test_config();
    let netd_mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("netdmgr_{tag}"),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 16,
            network: Some(NetworkConfig {
                connect_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                ..NetworkConfig::default()
            }),
        },
        root.clone(),
    )
    .expect("init netd manager (reaches G)");
    let vizd_mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("vizdmgr_{tag}"),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 16,
            // Networked (so the daemon passes its 2a network-configured guard) but
            // reaching NOBODY: scouting off + no connect endpoints. Its transient
            // fallback session opens yet gathers nothing → it cannot resolve.
            network: Some(NetworkConfig::default()),
        },
        root,
    )
    .expect("init vizd manager (reaches nobody)");
    (netd_mgr, vizd_mgr)
}

fn isolated_transport_inner(
    node_name: &str,
    network: Option<NetworkConfig>,
) -> Arc<TransportManager> {
    let clock = Arc::new(VirtualClock::new());
    TransportManager::init_for_test(
        TransportConfig {
            node_name: node_name.to_string(),
            clock,
            subscriber_buffer_size: 16,
            network,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init isolated transport")
}

/// A never-block worker over a fresh in-memory Rerun sink. Returns the worker, a
/// flush clone of the stream, and the storage handle (the worker MOVES its rec
/// into the daemon; the test keeps `flush` + `storage`).
fn memory_worker(
    recording_id: &str,
) -> (
    VizLogWorker,
    rerun::RecordingStream,
    rerun::sink::MemorySinkStorage,
) {
    let (rec, storage) = rerun::RecordingStreamBuilder::new("vizd")
        .recording_id(recording_id)
        .memory()
        .expect("memory sink");
    let flush = rec.clone();
    let worker =
        VizLogWorker::spawn(rec, builtin_walker(), SinkState::new()).expect("spawn worker");
    (worker, flush, storage)
}

/// Start the daemon with the HERMETIC [`NoNetdPlane`] demand plane —
/// the drop-in replacement for the production [`cerulion_vizd::start`] in tests.
///
/// `start` injects `NetdDemandPlane::new()`, which targets the WELL-KNOWN
/// `cerulion-netd` socket and SPAWNS netd if it is not running: on a dev desk the
/// daemon's `discover` gather / schema-less remote attach then talk to a LIVE netd
/// and, through it, to whatever robot is on the LAN — the verdict becomes a function
/// of ambient state (see the module docs). Injecting the plane keeps the netd state
/// each test asserts against DECLARED in the test. Same signature as `start`.
fn start_hermetic(
    socket_path: PathBuf,
    poll_interval: Duration,
    manager: Arc<TransportManager>,
    worker: VizLogWorker,
    walker: FrameWalker,
    rerun_url: Option<String>,
) -> std::io::Result<RunningDaemon> {
    start_with_demand_plane(
        socket_path,
        poll_interval,
        manager,
        worker,
        walker,
        rerun_url,
        Arc::new(NoNetdPlane),
    )
}

/// A unique short temp socket path (sockaddr_un has a ~104-char limit).
fn temp_socket(tag: &str) -> (PathBuf, PathBuf) {
    use std::sync::atomic::AtomicU64;
    static N: AtomicU64 = AtomicU64::new(0);
    let n = N.fetch_add(1, Ordering::Relaxed);
    let dir = std::env::temp_dir().join(format!("cer_vizd_e2e_{tag}_{}_{n}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("mk tempdir");
    let socket = dir.join("vizd.sock");
    // FAIL HERE, NAMING THE TAG, rather than at `start_*` with an opaque
    // `path must be shorter than SUN_LEN`.
    //
    // The overflow is ORDER-DEPENDENT — `n` is a process-global counter, so a long
    // tag fits when its test runs alone and overflows once the whole file has run
    // ahead of it — which makes it a failure that appears only in the full suite and
    // points at the daemon rather than at the name.
    // 104 is the smallest `sun_path` across the platforms this runs on (macOS);
    // Linux allows 108, so the tighter bound is the portable one.
    assert!(
        socket.as_os_str().len() < 104,
        "temp_socket tag `{tag}` is too long: `{}` is {} bytes and sockaddr_un's \
         sun_path holds 103 + NUL. Shorten the TAG — the rest of the path is the \
         platform's temp dir, the pid and a run counter, none of which a test \
         controls.",
        socket.display(),
        socket.as_os_str().len()
    );
    (socket, dir)
}

/// A background publisher: continuously publishes `Vector3` frames with an
/// incrementing wire sequence at ~10 ms (≈100 Hz) until `stop` flips. Returns the
/// stop flag + the join handle (published-count via the shared `AtomicU32`).
struct Publisher {
    stop: Arc<AtomicBool>,
    handle: Option<std::thread::JoinHandle<()>>,
    /// Frames this publisher has actually PUBLISHED. The struct's doc
    /// already promised "published-count via the shared `AtomicU32`", but the
    /// counter was never retained, so no test could read it. It is the oracle
    /// for [`Publisher::achieved_hz`] — the rate the runner really managed,
    /// which on a starved CI runner is nothing like the nominal ~100 Hz.
    published: Arc<AtomicU32>,
}

impl Publisher {
    /// Frames published so far. BOTH constructors count, so
    /// [`Publisher::achieved_hz`] is meaningful on a `spawn_fixed_frame`
    /// publisher too.
    fn published(&self) -> u32 {
        self.published.load(Ordering::Relaxed)
    }

    /// Measure the rate this publisher ACTUALLY achieved over `window`, by
    /// counting its own publishes across a wall interval it also measures.
    ///
    /// This is the load-IMMUNE oracle for vizd's reported `hz`. The
    /// nominal rate is ~100 Hz, but a shared CI runner can starve the publisher
    /// thread to a fraction of that — MEASURED at 19.1 Hz on a macOS runner,
    /// which fails an absolute `(20.0..300.0)` band. Nothing is wrong with vizd there:
    /// it reports 19.12 because 19-ish Hz is what is actually being published.
    /// Comparing vizd's answer to THIS number instead of to 100 removes the
    /// runner's speed from the assertion, because both sides collapse together.
    fn achieved_hz(&self, window: Duration) -> f64 {
        let before = self.published();
        let t0 = Instant::now();
        std::thread::sleep(window);
        let delta = self.published().saturating_sub(before);
        delta as f64 / t0.elapsed().as_secs_f64()
    }
}

impl Publisher {
    /// The iceoryx2 service is created on the
    /// CALLING thread, so `spawn` returning IS the topic being discoverable.
    ///
    /// Calling `create_publisher` inside the spawned thread would make
    /// the helper's whole postcondition — "a real publisher exists on this
    /// topic" — asynchronous, with NO handshake of any kind between `spawn`
    /// returning and the `{topic}/data` service existing. Every caller that then
    /// reads an enumeration (`discover`, `list_topics`) would be racing it.
    ///
    /// MEASURED with in-thread creation on this file's 13-topic corpus, 8 trials: `spawn`
    /// returned in **82–797 µs** while the 13 services took **36–290 ms** to all
    /// become visible, and the FIRST `list_topics()` after every `spawn()` had
    /// returned was missing **13 of 13** topics in 7 trials and 11 of 13 in the
    /// eighth. The gap is ~100–3000× the spawn return; the only thing hiding it
    /// is the daemon boot + socket connect that callers happen to do next, which
    /// a contended runner can finish first. The symptom in
    /// `discover_reports_a_distinct_entity_for_every_colliding_go2_topic_e2e`
    /// is the assertion `/lf/sportmodestate must be
    /// discovered` — corpus entry 0, absent from a hermetic daemon's `discover`,
    /// which for a topic with a live publisher and no mirrors can only mean the
    /// service did not exist yet.
    ///
    /// Creating the port on the caller closes the race BY CONSTRUCTION rather than by
    /// polling: there is no window left to bound. The publish loop still runs on
    /// its own thread (ports are `Send`). A creation failure also panics on
    /// the TEST thread, naming the topic, instead of on a detached one.
    /// Postcondition pinned by [`publisher_spawn_is_visible_before_it_returns`].
    fn spawn(mgr: Arc<TransportManager>, topic: &str) -> Self {
        let stop = Arc::new(AtomicBool::new(false));
        let topic = topic.to_string();
        let seq = Arc::new(AtomicU32::new(0));
        let published = Arc::clone(&seq); // Retained, so tests can read it
        let stop_c = Arc::clone(&stop);
        let mut publisher = mgr
            .create_publisher(&topic, MaxSliceLen::const_new(1 << 16), 0)
            .unwrap_or_else(|e| panic!("producer attaches on '{topic}': {e}"));
        let handle = std::thread::spawn(move || {
            while !stop_c.load(Ordering::Relaxed) {
                let s = seq.fetch_add(1, Ordering::Relaxed);
                let frame = build_vector3_frame(
                    s,
                    PUBLISH_STAMP_STEP_NS * (s as u64 + 1),
                    [s as f64, 2.0, 3.0],
                );
                let _ = publisher.publish_raw(&frame);
                std::thread::sleep(Duration::from_millis(10));
            }
        });
        Publisher {
            stop,
            handle: Some(handle),
            published,
        }
    }

    /// The same producer, paced to a wall-clock DEADLINE at a modest
    /// rate, so the rate a tap MEASURES on it is stable under load.
    ///
    /// [`Publisher::spawn`] sleeps a fixed 10 ms per frame — ~100 Hz nominal — and
    /// what that really produces is whatever the runner's scheduler grants: this
    /// file has MEASURED 19.12 Hz from it on a macOS runner, and 15.6 Hz on an idle
    /// desk in a debug build, because a `sleep`-paced loop pays its own per-frame
    /// cost on top and absorbs nothing. A monitor arm cannot use that: the engine
    /// FREEZES a rate baseline (the median of `MONITOR_LEARN_SAMPLES` samples) and
    /// then judges later samples against an OCTAVE around it, so a producer whose
    /// achieved rate wanders is a producer whose row wanders across that band —
    /// which is a `rate_deviation` alert about the RUNNER, on a row whose verdict
    /// under test is something else entirely.
    ///
    /// A deadline loop absorbs the jitter instead of accumulating it: it sleeps
    /// until the next slot and, if a frame ran long, skips straight to the next
    /// slot rather than sliding the whole schedule. At a period with real slack in
    /// it the measured rate then sits on its nominal value, which is what keeps the
    /// arm's row inside its own band without asking anything of the runner.
    fn spawn_paced(mgr: Arc<TransportManager>, topic: &str, period: Duration) -> Self {
        let stop = Arc::new(AtomicBool::new(false));
        let topic = topic.to_string();
        let seq = Arc::new(AtomicU32::new(0));
        let published = Arc::clone(&seq);
        let stop_c = Arc::clone(&stop);
        let mut publisher = mgr
            .create_publisher(&topic, MaxSliceLen::const_new(1 << 16), 0)
            .unwrap_or_else(|e| panic!("producer attaches on '{topic}': {e}"));
        let handle = std::thread::spawn(move || {
            let start = Instant::now();
            let mut tick: u32 = 0;
            while !stop_c.load(Ordering::Relaxed) {
                let s = seq.fetch_add(1, Ordering::Relaxed);
                let frame = build_vector3_frame(
                    s,
                    PUBLISH_STAMP_STEP_NS * (s as u64 + 1),
                    [s as f64, 2.0, 3.0],
                );
                let _ = publisher.publish_raw(&frame);
                tick += 1;
                // Sleep to the next slot the publisher has not already passed, so a
                // long frame costs one slot rather than shifting every slot after
                // it. `checked_duration_since` is `None` exactly when that slot is
                // already behind us: re-anchor on the current slot and publish
                // again immediately, which is what stops a stalled runner from
                // owing the loop an ever-growing burst.
                let deadline = start + period * tick;
                if let Some(nap) = deadline.checked_duration_since(Instant::now()) {
                    std::thread::sleep(nap);
                } else {
                    tick = (start.elapsed().as_nanos() / period.as_nanos().max(1)) as u32;
                }
            }
        });
        Publisher {
            stop,
            handle: Some(handle),
            published,
        }
    }
}

impl Drop for Publisher {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

// ── UDS control client ──────────────────────────────────────────────────────

/// A test controller over the UDS control socket: reads the Hello banner on
/// connect, then does line-based request→response.
struct Client {
    writer: UnixStream,
    reader: BufReader<UnixStream>,
    banner: Value,
}

impl Client {
    fn connect(socket: &std::path::Path) -> Self {
        let stream = connect_bounded(socket);
        let writer = stream.try_clone().expect("clone stream");
        let mut reader = BufReader::new(stream);
        let mut banner_line = String::new();
        reader.read_line(&mut banner_line).expect("read banner");
        let banner = serde_json::from_str(&banner_line).expect("banner json");
        Client {
            writer,
            reader,
            banner,
        }
    }

    /// Send a raw request line, read one response line, parse to JSON.
    fn request(&mut self, line: &str) -> Value {
        writeln!(self.writer, "{line}").expect("write request");
        let mut resp = String::new();
        self.reader.read_line(&mut resp).expect("read response");
        serde_json::from_str(&resp).expect("response json")
    }
}

/// Connect to the daemon socket with a bounded retry (the accept loop binds
/// before `start` returns, so this is usually immediate).
fn connect_bounded(socket: &std::path::Path) -> UnixStream {
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        match UnixStream::connect(socket) {
            Ok(s) => return s,
            Err(_) if Instant::now() < deadline => std::thread::sleep(Duration::from_millis(10)),
            Err(e) => panic!("could not connect to {}: {e}", socket.display()),
        }
    }
}

/// Poll `predicate` until true or the bound elapses; returns whether it became
/// true.
fn wait_until(bound: Duration, mut predicate: impl FnMut() -> bool) -> bool {
    let deadline = Instant::now() + bound;
    loop {
        if predicate() {
            return true;
        }
        if Instant::now() >= deadline {
            return false;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
}

/// Flush, then wait until `storage.num_msgs()` STOPS growing (unchanged across
/// `settle`) and return that SETTLED count. PANICS if it never settles within
/// `bound` — see below.
///
/// A distinct flake class from the ambient-netd one:
/// one blueprint apply emits SEVERAL messages, so
/// `wait_until(|| num_msgs() > mark)` returns on the FIRST of them. Capturing the
/// count right there samples the send MID-FLIGHT, and a later "the count is FROZEN"
/// assertion then trips on that same send's own tail whenever the machine is loaded
/// enough for the remaining messages to land after the capture (observed 19 vs 11).
/// Taking the baseline at QUIESCENCE makes the freeze assertion sound — it still
/// fails loudly if the code under test really does send something new.
///
/// The `bound` expiry is therefore LOUD, never a silent fallback: returning a
/// still-growing count would hand back exactly the MID-FLIGHT baseline this helper
/// exists to prevent, and the failure would then surface downstream as the freeze
/// assertion — i.e. misattributed to the code under test. Panicking here keeps the
/// attribution on the harness/machine where it belongs.
fn settled_msg_count(
    flush: &rerun::RecordingStream,
    storage: &rerun::sink::MemorySinkStorage,
    settle: Duration,
    bound: Duration,
) -> usize {
    let deadline = Instant::now() + bound;
    loop {
        flush.flush_blocking().ok();
        let before = storage.num_msgs();
        std::thread::sleep(settle);
        flush.flush_blocking().ok();
        let after = storage.num_msgs();
        if before == after {
            return after;
        }
        assert!(
            Instant::now() < deadline,
            "settled_msg_count: the rerun memory sink NEVER settled within {bound:?} — \
             num_msgs was STILL growing ({before} -> {after}) across the last {settle:?} \
             window. Refusing to return a MID-FLIGHT baseline (that is the exact \
             flake this helper exists to prevent, and it would surface downstream as a \
             bogus 'the count is FROZEN' failure). This is the HARNESS/MACHINE, NOT the code \
             under test: either the machine is loaded enough that {settle:?} is too short a \
             quiescence window, or something is publishing into this sink unexpectedly."
        );
    }
}

/// Find the discover/status/list array entry for `topic`.
fn entry_for<'a>(arr: &'a Value, key: &str, topic: &str) -> Option<&'a Value> {
    arr.as_array()?
        .iter()
        .find(|e| e[key].as_str() == Some(topic))
}

/// The monitors row for `(topic, robot)` — the FULL key.
///
/// [`entry_for`] keys on the topic NAME, which is not unique on the
/// `monitors` verb, because of the DISCOVERY plane: a topic can legitimately have
/// one row per robot serving it PLUS one for a genuine desk producer of the same
/// name, and those are independent data sources with independent baselines (the
/// engine's `RowKey` doc). A topic-only lookup on that verb silently reads
/// whichever row sorts first, so any arm distinguishing a TAP's row from a
/// CATALOG's must ask for both halves of the key.
///
/// `robot: None` asks for the LOCAL row, and it is a sharp question rather than a
/// loose one: the discovery plane never mints a robot-less row (pinned by
/// `monitors::tests::the_discovery_plane_never_claims_ownership_of_a_local_row`),
/// so a row answering here is always one an attached tap owns.
/// `robot: None` means the field is **ABSENT**, not merely "not a string".
///
/// The wire contract is `#[serde(default, skip_serializing_if = "Option::is_none")]`
/// — a local row OMITS the key, and that absence IS the convention. A
/// naive `and_then(Value::as_str) == None` also accepts `"robot": null`, `{}` or
/// `42`, so it would pass a serializer regression that started emitting an
/// explicit null: every local-row assertion in this file would keep reading green
/// while the payload changed shape under it. The pre-migration spelling
/// (`row.get("robot").is_none()`) rejected that, and this keeps it.
fn row_for<'a>(arr: &'a Value, topic: &str, robot: Option<&str>) -> Option<&'a Value> {
    arr.as_array()?.iter().find(|e| {
        e["topic"].as_str() == Some(topic)
            && match robot {
                None => e.get("robot").is_none(),
                Some(robot) => e.get("robot").and_then(Value::as_str) == Some(robot),
            }
    })
}

/// `row_for` answers its hand-written vectors — including the shapes that must
/// NOT read as a local row.
///
/// The helper is what ~30 monitor assertions are asserted THROUGH, so a version
/// that answers too easily makes all of them vacuous without failing anything.
/// The `robot: null` vector is the one that matters: absence and explicit null
/// are the same value in Rust and DIFFERENT bytes on the wire, and only absence
/// is the local-row convention.
#[test]
fn row_for_answers_its_hand_written_vectors() {
    let rows = serde_json::json!([
        { "topic": "/a" },                       // LOCAL: the key is absent
        { "topic": "/a", "robot": "go2" },       // the same topic, on a robot
        { "topic": "/n", "robot": null },        // an explicit null — NOT local
        { "topic": "/o", "robot": {} },          // a non-string present value
        { "topic": "/s", "robot": "spot" },
    ]);

    // The two rows sharing a topic name are told apart by the key, which is the
    // whole reason this helper exists.
    assert_eq!(row_for(&rows, "/a", None).unwrap()["topic"], "/a");
    assert!(row_for(&rows, "/a", None).unwrap().get("robot").is_none());
    assert_eq!(row_for(&rows, "/a", Some("go2")).unwrap()["robot"], "go2");
    assert!(row_for(&rows, "/a", Some("spot")).is_none());

    // A PRESENT robot field is never a local row, whatever it holds. `null` is
    // the sharp one: `and_then(Value::as_str)` yields `None` for it, so the naive
    // spelling would call this row local and every local-row assertion in this
    // file would survive a serializer that started emitting explicit nulls.
    assert!(
        row_for(&rows, "/n", None).is_none(),
        "an explicit `robot: null` is NOT a local row — the wire contract is an \
         ABSENT key"
    );
    assert!(
        row_for(&rows, "/o", None).is_none(),
        "nor is any other present non-string value"
    );
    // …and neither of those answers a robot query either.
    assert!(row_for(&rows, "/n", Some("go2")).is_none());
    assert!(row_for(&rows, "/o", Some("go2")).is_none());

    // Absent topic, and a non-array payload, answer nothing rather than panicking.
    assert!(row_for(&rows, "/missing", None).is_none());
    assert!(row_for(&serde_json::json!({}), "/a", None).is_none());
}

/// How long the stopped-publisher arm will wait for its subject to
/// reach a state a stall verdict can be REACHED from.
///
/// A LIVENESS ceiling in seconds, never a wall anything is decided by, and never
/// stated in units of the sampler's poll interval — the recorded macOS-CI rule
/// (background-QoS runners charge timer-coalescing slack PER WAKEUP). Load moves
/// WHEN the row settles into that state; the row's own self-healing is what makes
/// it happen at all (see [`stall_reachable`]).
const STALL_READY_BOUND: Duration = Duration::from_secs(60);

/// The period the stopped-publisher arm's two producers are paced at.
///
/// 10 Hz — 25x `MONITOR_STALL_MIN_MHZ` (400 mHz), so the engine's slow-topic gate is never
/// in question, with ~90 ms of per-frame slack for a loaded runner to be late in.
/// The rate is deliberately MODEST rather than fast: what this arm needs is a rate
/// its rows measure the same way either side of the baseline freeze, and a fast
/// producer on a background-QoS runner is exactly the opposite (see
/// [`Publisher::spawn_paced`]).
const STALL_ARM_PUBLISH_PERIOD: Duration = Duration::from_millis(100);

/// Does the SUBJECT report this row in a state a `stalled` verdict can
/// be reached from — i.e. may the arm apply its stimulus NOW?
///
/// `Ok(())` iff yes; `Err(reason)` names the missing premise so a precondition
/// failure is attributable instead of merely announced.
///
/// # Why "a baseline exists" is not the precondition
///
/// Waiting on `baseline_mhz` being present and then stopping the
/// publisher is a claim about ONE field, and the state a stall needs is
/// larger.
///
/// The mechanism: the engine freezes the baseline and re-learns it in exactly one place:
/// `MonitorEngine::observe` sets `row.baseline_mhz = None` and clears the learning
/// window whenever ANY condition CLEARS on that row. Re-learning then needs
/// `MONITOR_LEARN_SAMPLES` CONSECUTIVE `Streaming` samples carrying a trustworthy
/// rate — which a DEAD publisher can never supply. So one `rate_deviation`
/// raise-then-clear landing after the stop leaves the row reporting
/// `state: "learning"`, `baseline_mhz`
/// absent, `ineligible: []` — every one of which a one-field precondition is blind
/// to. With no stall basis besides the live baseline, that row never pages:
/// MEASURED on the engine, 220 `Idle` samples (88 s at the sampler's own
/// cadence, against the arm's 90 s wait) produce ZERO stall alerts.
///
/// **The engine closes that hole itself**: it keeps the last FROZEN
/// baseline as a stall basis the wipe does not touch, so the flapped row raises
/// at the same sample the un-flapped death does (pinned deterministically by
/// `monitor::engine_tests::a_rate_flap_that_clears_after_the_publisher_dies_still_raises_stalled`).
/// The `healthy` precondition is KEPT, and deliberately: it is not the
/// difference between paging and silence, but it pins the STIMULUS to a
/// row in a known state, which is what keeps a failure of this arm attributable
/// to the feature rather than to whatever the runner's publish jitter happened to
/// do to the rate that second. Losing it would trade a deterministic precondition
/// failure for a flaky feature failure.
///
/// The reachable path is the runner's own publish jitter, which is why it is
/// macOS-and-loaded only. `Publisher::spawn` sleeps 10 ms per frame; a
/// background-QoS runner has been measured in this very file delivering 19.12 Hz
/// against that 100 Hz nominal. If the eight learning samples land in a starved
/// stretch the baseline FREEZES low, and when the runner recovers the observed
/// rate crosses the octave band ABOVE it — `rate_deviation` confirms after the
/// stop (the rate is still present while the row is `Streaming`), then clears the
/// moment `TopicStat::hz` returns `None` past its recency bound, and the wipe
/// lands squarely between the stop and the stall confirmation.
///
/// # What this checks, and why each clause is load-bearing
///
/// * `state == "healthy"` — the engine's own composite: samples admitted, a
///   classification present, NO condition raised, and no gate merely `Learning`.
///   "No condition raised" is what forbids a CLEAR after the stop, which is the
///   wipe itself. The wipe does not park the stall gate, so this
///   clause pins the STIMULUS rather than the outcome: it keeps the stop landing
///   on a row whose rate is where this helper's arithmetic says it is.
/// * `stalled` absent from `ineligible` — `healthy` checks only for `Learning`
///   gates, so a gate CLOSED on `slow_topic` / `untrusted_rate_basis` still reads
///   healthy. This asks the subject directly whether the condition under test is
///   being judged at all. Reading the served verdict beats re-deriving
///   `MONITOR_STALL_MIN_MHZ` here (one copy of a rule).
/// * `liveness_state == "streaming"` with `observed_mhz` at-or-above the baseline
///   and inside the band above it — no `rate_deviation` qualifying run is PENDING,
///   AND none can start after the stop. See the arithmetic below for why the LOWER
///   bound is the BASELINE rather than the band's own lower edge.
///
/// # Why that is enough, stated as the arithmetic rather than as a hope
///
/// After the stop the numerator of `TopicStat::hz` is FROZEN (the frames already
/// in its window) while `dt` keeps growing, so the reported rate can only DECAY.
/// Decay alone settles the HIGH edge: a rate inside the band cannot climb out of
/// it, so `observed <= 2 * baseline` at the stop holds for every later sample too.
///
/// The LOW edge is where "inside the band" is NOT enough, and it is the one place
/// this helper deliberately asks for more than `rate_deviates` does. Write `w` for
/// the span the Hz window holds at the stop and `d` for the silence since; then
/// `hz(d) = observed * w / (w + d)`, and `hz` returns `None` once `d` exceeds
/// `HZ_STREAMING_RECENCY` (5 s — the same threshold at which the row classifies
/// `Idle`). `observe_seq` prunes that window to `HZ_WINDOW` (also 5 s), so
/// `w <= 5 s` and the deepest decay a still-`Streaming` sample can show is
/// `w / (w + 5)`, i.e. at most 2x. A row sitting exactly on the band's lower edge
/// (`observed == baseline / 2`) therefore falls THROUGH it on the very first
/// post-stop sample and keeps qualifying for the whole `Streaming` tail — about a
/// dozen samples, far past the four a raise needs — so "in band" would have
/// admitted precisely the state this helper exists to refuse.
///
/// `observed >= baseline` closes it: the worst case is then
/// `baseline * w / (w + 5)`, and `rate_deviates` needs `2 * hz < baseline`, i.e.
/// `2w / (w + 5) < 1`, i.e. `d > w`. With `w == 5 s` that is unreachable, because
/// `hz` has already gone `None`. With `w < 5 s` — a loaded runner whose drain loop
/// left a gap — the qualifying region is `(w, 5]`, exactly `5 - w` wide, which IS
/// that drain gap; and the sampler runs ON the drain loop, so its cadence is at
/// least that same gap. At most ONE qualifying sample, against the four a raise
/// needs. So with a pending count of zero at the stop, `rate_deviation` cannot
/// raise, cannot clear, and the frozen baseline SURVIVES; `stalled` is then
/// reached deterministically.
fn stall_reachable(row: &Value) -> Result<(), String> {
    let state = row["state"].as_str().unwrap_or("<absent>");
    if state != "healthy" {
        return Err(format!(
            "the row reports `state: {state}` — a stall is only reachable from \
             `healthy`, which is the engine's own statement that every condition \
             is being judged and none is raised (a raised condition can CLEAR after \
             the stop and wipe the frozen baseline, which no dead publisher can \
             re-learn)"
        ));
    }
    if let Some(entries) = row["ineligible"].as_array() {
        if let Some(e) = entries
            .iter()
            .find(|e| e["condition"].as_str() == Some("stalled"))
        {
            return Err(format!(
                "the subject reports `stalled` INELIGIBLE ({}) — the eligibility gate is withholding \
                 the very condition under test",
                e["reason"].as_str().unwrap_or("<no reason>")
            ));
        }
    }
    let liveness = row["liveness_state"].as_str().unwrap_or("<absent>");
    if liveness != "streaming" {
        return Err(format!(
            "the row's last evidential sample classified `{liveness}`, not \
             `streaming` — the stop must land on a row that is genuinely running"
        ));
    }
    let (Some(baseline), Some(observed)) =
        (row["baseline_mhz"].as_u64(), row["observed_mhz"].as_u64())
    else {
        return Err(format!(
            "the row serves no baseline/observed pair to check the band against \
             (baseline_mhz={}, observed_mhz={})",
            row["baseline_mhz"], row["observed_mhz"]
        ));
    };
    if rate_deviates(baseline, observed) {
        return Err(format!(
            "the row's observed rate ({observed} mHz) is OUTSIDE its own band \
             around the frozen baseline ({baseline} mHz), so a `rate_deviation` \
             qualifying run is already pending — let it confirm and clear (which \
             RE-LEARNS the baseline against the current rate) rather than stopping \
             the publisher into it"
        ));
    }
    if observed < baseline {
        return Err(format!(
            "the row's observed rate ({observed} mHz) is BELOW its frozen baseline \
             ({baseline} mHz) — in band, but not far enough from the band's lower \
             edge: the post-stop decay is up to 2x while the row is still \
             `Streaming`, so a rate under the baseline can fall THROUGH the edge \
             and open a qualifying run the stop itself caused (see this helper's \
             arithmetic)"
        ));
    }
    Ok(())
}

/// `stall_reachable` answers its hand-written vectors.
///
/// The helper is the arm's whole precondition, so a version that answers `Ok` too
/// easily reinstates the flake without failing anything — and one that
/// answers `Err` too easily converts the arm into a precondition timeout. Both
/// directions are pinned, and each rejecting vector differs from the accepting one
/// in exactly ONE field.
#[test]
fn stall_reachable_answers_its_hand_written_vectors() {
    // The state the arm may stop a publisher in.
    let ready = serde_json::json!({
        "topic": "/monitors/dying",
        "state": "healthy",
        "conditions": [],
        "baseline_mhz": 100_000u64,
        "observed_mhz": 137_000u64,
        "liveness_state": "streaming",
        "samples": 20u64,
        "last_sample_age_ms": 0u64,
    });
    assert_eq!(stall_reachable(&ready), Ok(()));

    let with = |k: &str, v: Value| {
        let mut row = ready.clone();
        row[k] = v;
        row
    };

    // The WIPED state: the baseline was wiped, so the engine renders the row
    // `learning`. It is refused — a row that is re-learning its rate band
    // is not a row to apply this arm's stimulus to — even though the retained
    // stall basis means it is a row that can still page.
    let wiped = serde_json::json!({
        "topic": "/monitors/dying",
        "state": "learning",
        "conditions": [],
        "liveness_state": "streaming",
        "samples": 20u64,
        "last_sample_age_ms": 0u64,
    });
    assert!(stall_reachable(&wiped).is_err_and(|e| e.contains("state: learning")));

    // A RAISED condition is the wipe waiting to happen: it clears after the stop.
    assert!(stall_reachable(&with("state", "alerting".into())).is_err());

    // `healthy` does NOT imply the stall gate is open — it only excludes
    // `Learning` gates, so a CLOSED one still reads healthy. This is the clause
    // that clause cannot cover.
    let slow = {
        let mut row = ready.clone();
        row["ineligible"] = serde_json::json!([{ "condition": "stalled", "reason": "slow_topic" }]);
        row
    };
    assert!(stall_reachable(&slow).is_err_and(|e| e.contains("slow_topic")));
    // …while an ineligibility on a DIFFERENT condition says nothing about this one.
    let other = {
        let mut row = ready.clone();
        row["ineligible"] = serde_json::json!([{ "condition": "rate_deviation", "reason": "untrusted_rate_basis" }]);
        row
    };
    assert_eq!(stall_reachable(&other), Ok(()));

    // The stop has to land on a row that is genuinely running.
    assert!(stall_reachable(&with("liveness_state", "idle".into())).is_err());

    // A pending `rate_deviation` run — BOTH edges, since the reachable macOS path
    // is the HIGH one (a baseline frozen low during a starved stretch, then the
    // runner recovers).
    assert!(stall_reachable(&with("observed_mhz", 201_000u64.into()))
        .is_err_and(|e| e.contains("OUTSIDE its own band")));
    assert!(stall_reachable(&with("observed_mhz", 49_000u64.into()))
        .is_err_and(|e| e.contains("OUTSIDE its own band")));
    // Exactly ON the HIGH edge is inside the band, and the decay can only move it
    // further in — the helper must not invent a stricter rule than the engine's.
    assert_eq!(
        stall_reachable(&with("observed_mhz", 200_000u64.into())),
        Ok(())
    );

    // The LOW side is where this helper asks for MORE than `rate_deviates` does,
    // and it is the clause the arithmetic in its doc exists for. A rate below the
    // baseline is IN BAND — `rate_deviates(100_000, 51_000)` is false — yet the
    // post-stop decay walks it straight through the lower edge, which is the state
    // that opens a qualifying run the stop itself caused.
    assert!(
        !rate_deviates(100_000, 51_000),
        "the vector must be in band"
    );
    assert!(stall_reachable(&with("observed_mhz", 51_000u64.into()))
        .is_err_and(|e| e.contains("BELOW its frozen baseline")));
    assert!(stall_reachable(&with("observed_mhz", 99_999u64.into())).is_err());
    // Exactly AT the baseline is the boundary, and it is admitted: the worst-case
    // decay is 2x at the `Streaming` tail while `rate_deviates` needs a STRICT
    // halving, so the region a sample could qualify in is at most one drain gap
    // wide — narrower than the sampler's own cadence.
    assert_eq!(
        stall_reachable(&with("observed_mhz", 100_000u64.into())),
        Ok(())
    );

    // A missing half of the pair is a refusal, never a silent pass.
    let mut no_rate = ready.clone();
    no_rate.as_object_mut().unwrap().remove("observed_mhz");
    assert!(stall_reachable(&no_rate).is_err());
}

/// No monitor assertion in this file may key on the TOPIC ALONE.
///
/// Fixing the sites is not the same as closing the class. The `monitors` verb is
/// the ONE surface where a topic name is not a key — a topic legitimately has one
/// row per robot serving it plus one for a desk producer of the same name — and
/// [`entry_for`] silently answers with whichever sorts first. That failure is
/// invisible: the assertion still reads a plausible row, so it passes for the
/// wrong reason until the day the two rows disagree.
///
/// It has already happened once here.
/// `a_snapshot_past_its_ttl_is_not_adopted_and_a_real_gather_withdraws_the_hint_e2e`
/// waited for "a row with this topic" and was satisfied by the DISCOVERY plane's
/// row before the tap it was about had one.
///
/// Every OTHER surface is topic-keyed and keeps [`entry_for`] — `discover`'s
/// `topics`, `list`'s `attached`, `status`, and a robot's own topic list — which
/// is why this guard names the field rather than banning the helper.
#[test]
fn no_monitor_assertion_keys_on_the_topic_alone() {
    let src = std::fs::read_to_string(file!())
        .or_else(|_| {
            std::fs::read_to_string(
                std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/vizd_e2e_test.rs"),
            )
        })
        .expect("this test file is readable");
    let offenders: Vec<&str> = src
        .lines()
        .filter(|l| l.contains("entry_for(") && l.contains("[\"monitors\"]"))
        .collect();
    assert!(
        offenders.is_empty(),
        "monitor rows are keyed `(topic, robot)` — use `row_for(.., topic, robot)`. \
         A topic-only lookup answers with whichever row sorts first, which is a \
         pass for the wrong reason rather than a failure:\n{offenders:#?}"
    );
    // ANTI-TAUTOLOGY: the walk really can see this file's assertions, so an empty
    // offender list means "none present" rather than "nothing was read". Without
    // it a mis-resolved path makes the guard vacuous and permanently green.
    assert!(
        src.contains("fn no_monitor_assertion_keys_on_the_topic_alone")
            && src.matches("row_for(").count() > 10,
        "the guard must be reading THIS file, and it must find the monitor \
         assertions it is guarding"
    );
}

/// Serializes every test that can touch the process-global blueprint statics
/// (`RUNTIME_BLUEPRINT` / `BLUEPRINT_SENT` / `ATTACHED_ENTITIES_SNAPSHOT`, all
/// in `cerulion_viz::blueprint`).
///
/// **THE THREE WRITER CLASSES.** Do not read the rule below as "verbs write the
/// statics"; that model is incomplete:
///
/// * **(W1) a mutating vizd verb — ENQUEUED on a controller thread, APPLIED on
///   the worker thread.** `attach`, `detach`, `compose_layout`, `set_blueprint`
///   reach `Ctx::maybe_apply_default_layout` / `Ctx::apply_layout_atomic` →
///   `VizControl::set_blueprint`, and that is where the controller thread's
///   involvement ENDS: `set_blueprint` only `try_send`s a
///   `VizMsg::SetBlueprint` and returns (`worker.rs:136-162`). The
///   `apply_runtime_blueprint` call that actually WRITES `RUNTIME_BLUEPRINT`
///   runs on the WORKER thread (`worker.rs:766-775`), FIFO behind any queued
///   data batches. **So the verb answering the client does NOT mean the static
///   has been written** — a test that reads `RUNTIME_BLUEPRINT` straight after a
///   verb is racing the worker. That is exactly why test 24
///   (`compose_layout_remembers_the_plan_for_reconnect_reapply`) polls with a
///   bounded `wait_until` instead of reading once. Only the
///   `ATTACHED_ENTITIES_SNAPSHOT` half is synchronous on the controller thread:
///   attach/detach write it directly via `set_attached_entities_snapshot`.
/// * **(W2) the daemon's POLL thread, with NO verb in flight** — when a topic's
///   schema resolves LATE the poll loop calls `ctx.maybe_apply_default_layout()`
///   directly (`daemon.rs`, the `newly_resolved` branch), so a daemon that is
///   merely RUNNING can write `RUNTIME_BLUEPRINT` at an arbitrary moment.
/// * **(W3) every worker's BOOT, on the worker thread** — `VizLogWorker::spawn`
///   (i.e. every `memory_worker(..)` call) runs `ensure_setup` as its first
///   statement → `send_blueprint_once`, which WRITES `BLUEPRINT_SENT` (a `swap`)
///   and, when it wins that swap, READS `RUNTIME_BLUEPRINT` +
///   `ATTACHED_ENTITIES_SNAPSHOT` and emits whatever plan it finds into ITS OWN
///   sink. The same call runs again on **every `VizMsg::Batch` the worker
///   handles** (`worker.rs:752`, before `process_batch`) — the DOMINANT path in
///   this file, since nearly every daemon test feeds a topic — as well as on
///   every idle probe tick and after a reconnect re-arm. (Materially harmless:
///   the `BLUEPRINT_SENT` swap is idempotent once true, so only the epoch's
///   first caller reads the statics. Named anyway so the frequency is not
///   understated.)
///
/// **THE RULE, and it is mechanical:** a test MUST take this lock (via
/// [`blueprint_statics_guard`], first statement, held for the whole body) if it
/// does EITHER of these:
///
/// 1. issues a MUTATING vizd verb (W1) — directly or through a helper like
///    `attach_remote_until_ok`. The rule deliberately covers verbs the test
///    expects to FAIL too: whether a given call reaches the write is a semantic
///    judgement that rots the moment a refusal fixture starts succeeding,
///    whereas "issues the verb" is greppable and cannot be got wrong.
/// 2. reads/clears/toggles the statics itself (`current_runtime_blueprint_plan`,
///    `clear_runtime_blueprint`, `rearm_blueprint`, `send_blueprint_once`,
///    `apply_runtime_blueprint`).
///
/// **WHAT THE LOCK DOES NOT EXCLUDE** (all three are live, none is hypothetical):
///
/// * **W3 in the unlocked tests.** 19 of the 71 tests take no guard, and 18 of
///   those spawn a daemon: they issue no mutating verb, so the rule does not
///   require the lock — yet their worker still does the boot `swap` + read. That
///   is a DELIBERATE judgement, not an oversight: the boot send only emits into
///   that test's OWN memory sink, and none of those tests assert on blueprint
///   content, so locking them would serialize 15 more tests to buy nothing. The
///   price is real and worth naming: an unlocked sibling's boot send can win the
///   once-guard and emit a LOCKED test's plan into its own sink.
/// * **W2 from any daemon that is still running**, including one belonging to a
///   test that has already finished asserting.
/// * **Teardown after the lock is released.** Locals drop in reverse declaration
///   order, so a guard bound first outlives the test's daemon — but a daemon
///   MOVED into a helper thread does not obey that, which is why
///   `set_blueprint_control_does_not_wedge_shutdown` joins its shutdown thread
///   BEFORE asserting (see its body).
/// * **A DETACHED worker outliving the guard entirely.** The drop-order argument
///   above relies on the daemon's teardown draining the worker's backlog inside
///   the lock scope (`RunningDaemon::drop` → `shutdown()` → join the poll thread
///   → `VizLogWorker::drop` drains). That chain has a deliberate escape hatch:
///   `VizLogWorker::drop` joins only for `SHUTDOWN_JOIN_TIMEOUT` and then
///   **DETACHES** a worker still running (`worker.rs:626-633`, the never-block
///   guarantee extended to teardown). A detached worker can apply a queued
///   `SetBlueprint` AFTER `_statics` has been released — i.e. after the next test
///   already holds the lock. This is not hypothetical: it is precisely the
///   ~6 s wedged-viewer path `set_blueprint_control_does_not_wedge_shutdown`
///   exists to pin. It is one more reason the two content-asserting tests carry
///   their own distinctive-plan + dedicated-sink robustness rather than trusting
///   the lock.
///
/// So holding the lock is NECESSARY, NOT SUFFICIENT. The two tests that ASSERT
/// on static content — `compose_layout_remembers_the_plan_for_reconnect_reapply`
/// and `boot_send_pins_the_timeline_and_runtime_reapply_does_not_re_pin_e2e` —
/// therefore keep their own robustness (a DISTINCTIVELY-NAMED plan, a dedicated
/// sink no daemon or publisher feeds, and a bounded rearm+send retry that wins
/// the process-global once-guard). Those measures are LOAD-BEARING, not
/// belt-and-braces; each test says so in its own header. Any NEW test that
/// asserts on blueprint content must adopt the same pattern.
///
/// With only the two static-touching tests holding the lock, ~26 unlocked
/// W1 writers race them: `compose_layout_remembers_the_plan_for_reconnect_reapply`
/// was observed (1 in 14 runs at `--test-threads=16`) reading the `autoplot`
/// plan written by `attach_auto_applies_the_consolidated_default_scene_plus_plot_e2e`
/// instead of its own `COMPOSE-REAPPLY`. A lock only excludes other HOLDERS — so
/// the W1 writers must hold it, not just the readers.
static BLUEPRINT_STATICS_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

/// Take [`BLUEPRINT_STATICS_LOCK`] for the caller's whole body. Bind it
/// (`let _statics = blueprint_statics_guard();`) — a bare `let _ =` would drop
/// the guard immediately and silently restore the race. Poison-tolerant: a
/// panicking test's poison must not cascade into unrelated failures.
///
/// Make it the FIRST statement: locals drop in reverse declaration order, so a
/// guard bound first is released LAST — after the test's daemon/worker locals,
/// which means their teardown (which can still write the statics from a worker
/// thread) also happens inside the lock.
///
/// (No `#[must_use]` here — `MutexGuard` already carries it, and stacking them
/// trips `clippy::double_must_use`.)
fn blueprint_statics_guard() -> std::sync::MutexGuard<'static, ()> {
    BLUEPRINT_STATICS_LOCK
        .lock()
        .unwrap_or_else(|p| p.into_inner())
}

// ── Test 1 — the headline: discover → attach → render → status → detach ─────

#[test]
fn discover_attach_render_status_detach_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("vizd_headline");
    let topic = "/vizd/vel";
    let _pub = Publisher::spawn(Arc::clone(&mgr), topic);

    let (worker, flush, storage) = memory_worker("headline");
    let (socket, dir) = temp_socket("headline");
    let mut daemon: RunningDaemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None, // rerun_url: a memory sink hosts no endpoint
    )
    .expect("daemon starts");

    let mut client = Client::connect(&socket);
    // Banner: version handshake.
    assert_eq!(client.banner["vizd"].as_str(), Some("cerulion-vizd"));
    assert_eq!(client.banner["protocol"].as_u64(), Some(1));

    // discover lists the live topic with its RESOLVED schema + archetype.
    let disc = client.request(r#"{"id":1,"method":"discover"}"#);
    assert_eq!(disc["ok"].as_bool(), Some(true));
    let vel = entry_for(&disc["topics"], "topic", topic).expect("topic discovered");
    assert_eq!(vel["schema"].as_str(), Some("geometry_msgs/Vector3"));
    assert_eq!(vel["archetype"].as_str(), Some("Scalars"));
    // Discover carries the deterministic layout placement — the entity a
    // topic would render under, its rerun component families, and the view kind(s)
    // the agent should place it in (Scalars → ONE time_series plot, not N panels).
    assert_eq!(vel["entity"].as_str(), Some("world/vizd/vel"));
    assert_eq!(vel["components"], serde_json::json!(["Scalars"]));
    assert_eq!(vel["view_kinds"], serde_json::json!(["time_series"]));

    // Baseline num_msgs AFTER the worker's scene setup (process-global once-guards).
    flush.flush_blocking().expect("flush baseline");
    let baseline = storage.num_msgs();

    // attach → the resolved placement (hand oracle).
    let att = client.request(&format!(
        r#"{{"id":2,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(att["ok"].as_bool(), Some(true));
    assert_eq!(att["topic"].as_str(), Some(topic));
    assert_eq!(att["schema"].as_str(), Some("geometry_msgs/Vector3"));
    assert_eq!(att["archetype"].as_str(), Some("Scalars"));
    assert_eq!(att["entity"].as_str(), Some("world/vizd/vel"));
    assert_eq!(att["route"].as_str(), Some("vizd/vel"));
    assert_eq!(att["already_attached"].as_bool(), Some(false));
    // Attach reports the same deterministic layout metadata.
    assert_eq!(att["components"], serde_json::json!(["Scalars"]));
    assert_eq!(att["view_kinds"], serde_json::json!(["time_series"]));

    // Frames RENDER: the memory sink's chunk count grows (Vector3 → 3 scalars).
    assert!(
        wait_until(Duration::from_secs(3), || {
            flush.flush_blocking().ok();
            storage.num_msgs() > baseline
        }),
        "attached topic must render frames to the sink"
    );

    // list shows the attached tap with its resolved placement.
    let list = client.request(r#"{"id":3,"method":"list"}"#);
    let le = entry_for(&list["attached"], "topic", topic).expect("attached listed");
    assert_eq!(le["route"].as_str(), Some("vizd/vel"));
    assert_eq!(le["schema"].as_str(), Some("geometry_msgs/Vector3"));
    // `list` carries the SAME deterministic layout metadata as
    // attach/status/discover (Scalars → ONE time_series plot).
    assert_eq!(le["components"], serde_json::json!(["Scalars"]));
    assert_eq!(le["view_kinds"], serde_json::json!(["time_series"]));

    // status: the reported Hz must be a REAL measurement of this topic's delivery
    // rate. An absolute band such as
    // `(20.0..300.0).contains(&hz)` against a nominal ~100 Hz publisher fails on a
    // loaded macOS runner at 19.12 Hz. Nothing is wrong with vizd there — the
    // runner has starved the publisher thread to ~19 Hz and vizd reports that
    // accurately. Such a band measures THE RUNNER, and it can only ever
    // fail in the direction load pushes it.
    //
    // Two bounds, split by whether load can fake them:
    //
    //   UPPER — absolute. Contention makes a rate LOWER, never higher, so
    //     a ceiling is not load-fakeable. It is what catches a units error in the
    //     fast direction (ns/ms/s confusion is orders of magnitude).
    //   AGREEMENT — against what the publisher ACTUALLY achieved over the same
    //     window, measured independently by the test from the publisher's own
    //     counter. Both sides collapse together under starvation, so the RATIO is
    //     load-immune where an absolute floor is not.
    //
    // This is STRICTER than an absolute band, not looser: a band admitting
    // anything from 20 to 300 Hz against a 100 Hz stream lets a 3x decimation bug
    // sit comfortably inside it — nominally 33 Hz, and MEASURED at 27.5 Hz when
    // that bug was present. The ratio band rejects it (ratio 0.331).
    let achieved_hz = _pub.achieved_hz(Duration::from_millis(1200));
    let status = client.request(r#"{"id":4,"method":"status"}"#);
    let st = entry_for(&status["topics"], "topic", topic).expect("topic in status");
    let hz = st["hz"].as_f64().expect("hz present");
    // Anti-vacuity: if the runner published nothing at all, the ratio below is
    // meaningless and would divide by zero — fail loudly on the PRECONDITION
    // instead of reporting a confusing rate mismatch.
    assert!(
        achieved_hz > 1.0,
        "PRECONDITION: the publisher must actually have published during the \
         window for its rate to be an oracle, achieved {achieved_hz} Hz"
    );
    assert!(
        hz < 300.0,
        "reported Hz must not exceed a sane ceiling (load can only push a rate \
         DOWN, so this side is not load-fakeable), got {hz}"
    );
    let ratio = hz / achieved_hz;
    assert!(
        (0.5..2.0).contains(&ratio),
        "reported Hz must track the rate the publisher ACTUALLY achieved: \
         reported {hz} Hz vs achieved {achieved_hz} Hz (ratio {ratio})"
    );
    assert!(st["frames"].as_u64().unwrap_or(0) > 0, "frames counted");
    assert_eq!(st["schema"].as_str(), Some("geometry_msgs/Vector3"));
    // Status carries the same deterministic layout metadata per topic.
    assert_eq!(st["entity"].as_str(), Some("world/vizd/vel"));
    assert_eq!(st["components"], serde_json::json!(["Scalars"]));
    assert_eq!(st["view_kinds"], serde_json::json!(["time_series"]));
    assert!(status["worker"]["dropped_frames"].as_u64().is_some());

    // detach → rendering STOPS: num_msgs stabilizes after the tap is dropped.
    let det = client.request(&format!(
        r#"{{"id":5,"method":"detach","topic":"{topic}"}}"#
    ));
    assert_eq!(det["ok"].as_bool(), Some(true));
    assert_eq!(det["detached"].as_bool(), Some(true));
    // Drain any in-flight frames, then confirm the count is stable across a window.
    // The baseline is taken at QUIESCENCE (same class as the blueprint-freeze
    // site) — a FIXED 300 ms drain is not guaranteed to have flushed the ~100 Hz
    // producer's in-flight frames on a loaded box, so the pre-detach tail would land
    // inside the freeze window and fail as if detach had not stopped rendering.
    let after_detach = settled_msg_count(
        &flush,
        &storage,
        Duration::from_millis(250),
        Duration::from_secs(5),
    );
    std::thread::sleep(Duration::from_millis(400));
    flush.flush_blocking().ok();
    assert_eq!(
        storage.num_msgs(),
        after_detach,
        "after detach the tap is dropped → no further frames render"
    );
    // list is now empty.
    let list2 = client.request(r#"{"id":6,"method":"list"}"#);
    assert!(list2["attached"].as_array().unwrap().is_empty());

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

/// **The reported repro, end to end over real transport.**
///
/// Two publishers were killed and the sidebar kept showing their topics at "0.3 Hz"
/// indefinitely, while `cerulion topic hz` correctly said not-published. This test
/// runs exactly that: a real producer, a real attach, a real KILL, and then asserts
/// what the daemon serves afterwards.
///
/// The three claims, each with its own oracle:
///
/// 1. **The rate is GONE, not smaller.** `hz` must be `null`. Asserting
///    `is_null()` (not "not ≈100") is the whole point — without this check, the daemon serves a
///    real, shrinking number forever, because nothing prunes the sample window without a fresh
///    publish, so `hz` divides a fixed sequence delta by an ever-growing span. A "rate looks
///    wrong" assertion would pass on it.
/// 2. **The verdict replaces it.** `liveness_state` is `"idle"` with a DATED age —
///    this topic demonstrably produced frames, so it is never the dimmed
///    `"no_data"` (the liveness rule: a produced topic is never NoData), and the age
///    is what tells the sidebar nothing is arriving NOW.
/// 3. **Registration is observable.** `producer_count` is `Some(0)` — the row that
///    must disappear. Note the topic's SHM service still exists here
///    (vizd's own tap holds it open, so it is still enumerable), which is exactly
///    why service enumeration cannot answer this and a producer probe must.
///
/// Plus the surface-agreement pin: `list`, `status` AND `discover` must report the
/// SAME verdict for the same topic in the same instant. Two surfaces deriving one
/// answer separately is how surfaces come to disagree.
///
/// ANTI-TAUTOLOGY: every post-kill assertion is preceded by the LIVE oracle on the
/// same row — a real `hz`, `"streaming"`, `producer_count >= 1` — so a daemon that
/// simply never reported anything could not pass.
#[test]
fn a_killed_publisher_loses_its_rate_and_reports_the_liveness_verdict_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("killed_publisher");
    let topic = "/vizd/img/jpeg";
    let publisher = Publisher::spawn(Arc::clone(&mgr), topic);

    let (worker, _flush, _storage) = memory_worker("killed");
    let (socket, dir) = temp_socket("killed");
    let mut daemon: RunningDaemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    let att = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(att["ok"].as_bool(), Some(true), "attach: {att}");

    // ── LIVE oracle (anti-tautology) ────────────────────────────────────────
    // The Hz window needs two yielding polls, and the liveness BASELINE means the
    // FIRST drain is banked-but-undated on purpose (a retained-history flush must
    // not date a dead route), so `"streaming"` proves a SECOND batch really arrived.
    assert!(
        wait_until(Duration::from_secs(6), || {
            let st = client.request(r#"{"id":2,"method":"status"}"#);
            entry_for(&st["topics"], "topic", topic).is_some_and(|r| {
                r["hz"].as_f64().is_some_and(|hz| hz > 0.0)
                    && r["liveness_state"].as_str() == Some("streaming")
            })
        }),
        "a live producer must report a real rate AND classify streaming"
    );
    let live = client.request(r#"{"id":3,"method":"status"}"#);
    let live_row = entry_for(&live["topics"], "topic", topic).expect("live row");
    assert!(
        live_row["producer_count"].as_u64().is_some_and(|n| n >= 1),
        "a live producer is REGISTERED: {live_row}"
    );
    assert!(
        live_row["liveness"]["frames_observed"]
            .as_u64()
            .is_some_and(|n| n > 0),
        "the tap banked the frames it drained: {live_row}"
    );

    // ── The operator's gesture: kill the publisher. ─────────────────────────
    drop(publisher);

    // ── The three post-kill claims ──────────────────────────────────────────
    // Bounded by the streaming-recency window the daemon shares with the liveness observer
    // (5 s) plus slack; the rate must be GONE by then, not merely small.
    assert!(
        wait_until(Duration::from_secs(20), || {
            let st = client.request(r#"{"id":4,"method":"status"}"#);
            entry_for(&st["topics"], "topic", topic).is_some_and(|r| r["hz"].is_null())
        }),
        "a killed publisher's rate must go away — the pre-926 daemon served a \
         decaying number (the reported 0.3 Hz) forever"
    );
    // The wait above covers `hz` ALONE, but the claims below read THREE
    // more facts off a FRESHLY-REQUESTED response — and `producer_count` in
    // particular does not settle on the liveness recency window at all, it settles
    // when the transport observes the dropped publisher's port. So the assertions
    // could run against a response that had lost the rate but not yet the producer.
    // Wait for the whole verdict, and assert on the response the wait VALIDATED
    // rather than on a later one no predicate ever checked.
    let mut dead = Value::Null;
    assert!(
        wait_until(Duration::from_secs(20), || {
            let st = client.request(r#"{"id":5,"method":"status"}"#);
            let settled = entry_for(&st["topics"], "topic", topic).is_some_and(|r| {
                r["hz"].is_null()
                    && r["liveness_state"].as_str() == Some("idle")
                    && r["liveness"]["last_frame_age_ms"]
                        .as_u64()
                        .is_some_and(|age| age >= 5_000)
                    && r["producer_count"].as_u64() == Some(0)
            });
            dead = st;
            settled
        }),
        "the post-kill verdict must SETTLE on ONE response — rate gone, idle, a \
         dated age past the streaming window, and no registered producer"
    );
    let dead_row = entry_for(&dead["topics"], "topic", topic).expect("row still reported");
    assert!(
        dead_row["hz"].is_null(),
        "claim 1 — the rate is NULL, not a smaller number: {dead_row}"
    );
    assert_eq!(
        dead_row["liveness_state"].as_str(),
        Some("idle"),
        "claim 2 — a topic that HAS produced is idle-with-an-age, never the dimmed \
         no_data: {dead_row}"
    );
    let age = dead_row["liveness"]["last_frame_age_ms"]
        .as_u64()
        .expect("claim 2 — the staleness is DATED, not unknown");
    assert!(
        age >= 5_000,
        "claim 2 — the age must exceed the streaming window that dropped the rate, \
         got {age} ms: {dead_row}"
    );
    assert_eq!(
        dead_row["producer_count"].as_u64(),
        Some(0),
        "claim 3 — nothing is registered to publish, so the row is the one that \
         DISAPPEARS: {dead_row}"
    );

    // ── Surface agreement: list + discover say the same thing. ──────────────
    let list = client.request(r#"{"id":6,"method":"list"}"#);
    let listed = entry_for(&list["attached"], "topic", topic).expect("still attached");
    assert_eq!(listed["liveness_state"].as_str(), Some("idle"), "{listed}");
    assert_eq!(listed["producer_count"].as_u64(), Some(0), "{listed}");
    assert!(
        listed["liveness"]["last_frame_age_ms"].as_u64().is_some(),
        "list carries the dated observation too: {listed}"
    );
    let disc = client.request(r#"{"id":7,"method":"discover"}"#);
    let discovered = entry_for(&disc["topics"], "topic", topic).expect("still discoverable");
    assert_eq!(
        discovered["liveness_state"].as_str(),
        Some("idle"),
        "discover reports the tap's own observation, not a blanket UNKNOWN: \
         {discovered}"
    );

    // ── Detach: the topic leaves `status` entirely, so no rate can persist. ──
    let det = client.request(&format!(
        r#"{{"id":8,"method":"detach","topic":"{topic}"}}"#
    ));
    assert_eq!(det["ok"].as_bool(), Some(true), "detach: {det}");
    let after = client.request(r#"{"id":9,"method":"status"}"#);
    assert!(
        entry_for(&after["topics"], "topic", topic).is_none(),
        "a detached topic is not in status at all — a client that keeps rendering \
         its last rate is showing a number the daemon no longer reports: {after}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

/// A NEVER-produced attached route reports UNKNOWN, not a fabricated
/// verdict — the degrade-to-UNKNOWN half.
///
/// A tap that has just attached to a silent topic has watched too briefly to tell a
/// DEAD route from a slow one, and the liveness design is explicit that those must never be
/// conflated. UNKNOWN has exactly ONE encoding on this wire — ABSENCE — so the
/// assertion is that the key is not there at all, while `liveness` (the raw
/// observation) IS, carrying `frames_observed: 0`.
///
/// The `no_data` verdict itself arrives only after `LIVENESS_NO_DATA_MIN_MS` (10 s)
/// of watching nothing; the threshold walk is pinned by the pure oracle in
/// `daemon.rs` (`tap_liveness_banks_the_baseline_and_classifies_with_the_shared_thresholds`)
/// rather than paid for again as wall-clock here.
#[test]
fn a_freshly_attached_silent_topic_reports_unknown_not_dead_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("silent_unknown");
    let topic = "/vizd/img/silent";
    // A registered-but-silent route: a publisher creates the service and a
    // data-only tap holds it open, so the topic is attachable with a live producer
    // that never publishes.
    let _silent_pub = mgr
        .create_publisher(topic, MaxSliceLen::const_new(1 << 16), 0)
        .expect("silent producer creates the service");

    let (worker, _flush, _storage) = memory_worker("silent");
    let (socket, dir) = temp_socket("silent");
    let mut daemon: RunningDaemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    let att = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(att["ok"].as_bool(), Some(true), "attach: {att}");

    let st = client.request(r#"{"id":2,"method":"status"}"#);
    let row = entry_for(&st["topics"], "topic", topic).expect("attached row");
    assert!(row["hz"].is_null(), "nothing published, no rate: {row}");
    assert!(
        row["liveness_state"].is_null(),
        "UNKNOWN is ABSENCE — a freshly-attached silent tap must not claim a \
         verdict it has not earned: {row}"
    );
    assert_eq!(
        row["liveness"]["frames_observed"].as_u64(),
        Some(0),
        "the raw observation is still reported — we ARE watching, we just cannot \
         say yet: {row}"
    );
    assert!(
        row["producer_count"].as_u64().is_some_and(|n| n >= 1),
        "the route IS registered — this is the registered-but-silent row, not the \
         unregistered one: {row}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

/// The vizd LOCAL producer-count probe over REAL
/// transport. A live local producer surfaces `producer_count > 0` in discover (the
/// desk renders it live); a registered-but-DEAD local route — a `{topic}/data` service
/// with NO publisher — surfaces `Some(0)` so the sidebar dims it. Hand oracles (never a
/// self-compare). The probe-FAILURE → `None` (UNKNOWN, never dead) arm is not
/// deterministically triggerable over real iceoryx2 — it is pinned by the pure
/// `classify_producer_probe` oracle in cerulion_core (`mod.rs`).
#[test]
fn discover_producer_count_reflects_local_liveness_e2e() {
    let mgr = isolated_transport("local_liveness");
    let live = "/vizd/pc/live";
    let dead = "/vizd/pc/dead";
    // A live producer keeps publishing on `live`.
    let _live_pub = Publisher::spawn(Arc::clone(&mgr), live);
    // A DEAD route: a publisher CREATES the `dead` service, a data-only tap HOLDS it
    // open, then the publisher DROPS → the service persists (still discoverable) with
    // ZERO publishers (auto-dead-node cleanup is OFF, so the lingering service is stable).
    let dead_pub = mgr
        .create_publisher(dead, MaxSliceLen::const_new(1 << 16), 0)
        .expect("dead-route publisher creates the service");
    let _dead_tap = mgr
        .create_data_only_subscriber(dead)
        .expect("a data-only tap holds the dead-route service open");
    drop(dead_pub);

    let (worker, _flush, _storage) = memory_worker("local_liveness");
    let (socket, dir) = temp_socket("local_liveness");
    let mut daemon: RunningDaemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // Wait until the live producer's thread has attached (discover then shows it live).
    assert!(
        wait_until(Duration::from_secs(3), || {
            let disc = client.request(r#"{"id":1,"method":"discover"}"#);
            entry_for(&disc["topics"], "topic", live)
                .and_then(|e| e["producer_count"].as_u64())
                .is_some_and(|n| n >= 1)
        }),
        "the live local topic must surface producer_count >= 1"
    );

    // Final discover: hand oracle on BOTH rows.
    let disc = client.request(r#"{"id":2,"method":"discover"}"#);
    let live_row = entry_for(&disc["topics"], "topic", live).expect("live discovered");
    assert!(
        live_row["producer_count"].as_u64().is_some_and(|n| n >= 1),
        "live producer_count >= 1: {live_row}"
    );
    let dead_row = entry_for(&disc["topics"], "topic", dead).expect("dead route discovered");
    assert_eq!(
        dead_row["producer_count"].as_u64(),
        Some(0),
        "a dead local route (no publisher) surfaces Some(0) — the sidebar dims it: {dead_row}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

/// The compose verb PREFERS LIVE topics over REAL transport. A bare-`topics`
/// automagic compose over one LIVE producer + one registered-but-DEAD route places
/// ONLY the live topic and reports the dead one under `skipped_dead` (never silently,
/// never consuming a view slot) — the reported `/uslam/cloud_map`-vs-live-cloud case,
/// proven end-to-end through the daemon's attach + producer-count probe + compiler.
/// Hand oracles (never a self-compare). The pure compiler arms live in the lib's
/// `layout_compose_liveness_test.rs`; this pins the DAEMON probe→skip WIRING.
#[test]
fn compose_prefers_live_and_reports_dead_routes_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("compose_prefer_live");
    let live = "/vizd/compose/live";
    let dead = "/vizd/compose/dead";
    // A live producer keeps publishing on `live` (resolves to a real archetype).
    let _live_pub = Publisher::spawn(Arc::clone(&mgr), live);
    // A registered-but-DEAD route: create the service, hold it open with a data-only
    // tap, then drop the publisher → the `{dead}/data` service persists with ZERO
    // publishers (the reported dead-DDS-route shape).
    let dead_pub = mgr
        .create_publisher(dead, MaxSliceLen::const_new(1 << 16), 0)
        .expect("dead-route publisher creates the service");
    let _dead_tap = mgr
        .create_data_only_subscriber(dead)
        .expect("a data-only tap holds the dead-route service open");
    drop(dead_pub);

    let (worker, _flush, _storage) = memory_worker("compose_prefer_live");
    let (socket, dir) = temp_socket("compose_prefer_live");
    let mut daemon: RunningDaemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // Wait until discover shows the live topic with a RESOLVED archetype (so the
    // compose bounded peek will type it → it is placed, not silent).
    assert!(
        wait_until(Duration::from_secs(5), || {
            let disc = client.request(r#"{"id":1,"method":"discover"}"#);
            entry_for(&disc["topics"], "topic", live)
                .map(|e| !e["archetype"].is_null())
                .unwrap_or(false)
        }),
        "the live topic must resolve an archetype before compose"
    );

    // Bare-topics automagic compose over [live, dead].
    let req = format!(
        r#"{{"id":2,"method":"compose_layout","intent":{{"topics":["{live}","{dead}"],"include_robot":false}}}}"#
    );
    let resp = client.request(&req);
    assert_eq!(
        resp["ok"],
        serde_json::Value::Bool(true),
        "compose ok: {resp}"
    );

    // Only the LIVE topic is placed (in ANY view); the dead route is NOT.
    let placements = resp["placements"].as_array().expect("placements array");
    let placed: Vec<&str> = placements
        .iter()
        .filter_map(|p| p["topic"].as_str())
        .collect();
    assert!(
        placed.contains(&live) && !placed.contains(&dead),
        "only the live topic is placed: {resp}"
    );
    // No kept placement is flagged dead (automagic dead topics are skipped, not placed).
    assert!(
        placements.iter().all(|p| p["dead"].as_bool() != Some(true)),
        "no automagic placement is flagged dead: {resp}"
    );

    // The dead route is surfaced EXPLICITLY under skipped_dead with a reason.
    let skipped = resp["skipped_dead"].as_array().expect("skipped_dead array");
    assert_eq!(skipped.len(), 1, "exactly one skipped dead route: {resp}");
    assert_eq!(skipped[0]["topic"].as_str(), Some(dead));
    assert_eq!(
        skipped[0]["reason"].as_str(),
        Some("never published — 0 producers"),
        "the skip carries a human reason: {resp}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Decision: the Scene-only default + the scalar-attach path ────────────────
//
// The default scene DROPS the empty Telemetry + Status panels (so a robot-free desk
// can never show the 1970-era epoch axis). The critical composition the decision rests
// on: a NON-spatial (scalar) attach must STILL render somewhere. It does — via the
// default's `auto_views`: a scalar's frames log under `/world/...` (inside the 3D
// scene's `$origin/**` contents, but not visualizable there), and rerun's
// `spawn_heuristic_views` auto-spawns a `time_series` plot for it (its redundancy
// filter is scoped PER VIEW CLASS, so with NO time_series view in the default the
// scalar's plot recommendation is not pre-empted). The auto-spawned plot has real
// data → no 1970 axis. The viewer-side spawn itself needs a live re_viewer (not a
// memory-sink e2e), so this pins the CONTRACT end-to-end over the real daemon:
// Scene-only default + auto_views ON + a scalar attach that resolves to a
// time_series view kind AND flows real data.

#[test]
fn scene_only_default_scalar_attach_renders_via_auto_views_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("cer_scene_only");
    let topic = "/vizd/scene_only_vel";
    let _pub = Publisher::spawn(Arc::clone(&mgr), topic); // Vector3 → Scalars

    let (worker, flush, storage) = memory_worker("scene_only");
    let (socket, dir) = temp_socket("scene_only");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None, // memory sink hosts no endpoint
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // (1) The auto-spawn CONTRACT the Scene-only default relies on: the default PLAN is
    //     Scene-ONLY (a single 3D view) with auto_views ON. So a scalar's auto-view
    //     recommendation is NOT pre-empted — re_viewport's `spawn_heuristic_views`
    //     filters recommendations PER VIEW CLASS, and with NO time_series view in the
    //     default the scalar's plot recommendation survives and spawns. (The emitted
    //     Scene-only SHAPE + its zero telemetry decorations are pinned at the unit level
    //     in blueprint.rs's `go2_default_is_scene_only_*` /
    //     `build_blueprint_msgs_go2_default_*` tests; asserting the default over the wire
    //     here would need the process-global `BLUEPRINT_SENT` guard, deliberately touched
    //     by a single test.)
    let plan = BlueprintPlan::go2_default();
    assert!(
        plan.auto_views,
        "auto_views ON is what renders a non-spatial attach on a Scene-only default"
    );
    assert_eq!(
        plan.view_count(),
        1,
        "Scene-only: a single 3D view, none pre-empting a scalar's auto-plot"
    );

    // (2) Over the REAL daemon: attach the scalar-shaped topic → it resolves to the
    //     Scalars archetype destined for a time_series view (hand oracle) — exactly the
    //     view kind `auto_views` will spawn for it.
    let baseline = {
        flush.flush_blocking().ok();
        storage.num_msgs()
    };
    let att = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(att["ok"].as_bool(), Some(true), "attach ok: {att}");
    assert_eq!(
        att["archetype"].as_str(),
        Some("Scalars"),
        "Vector3 resolves to the Scalars archetype: {att}"
    );
    assert_eq!(
        att["view_kinds"],
        serde_json::json!(["time_series"]),
        "the scalar attach is destined for an (auto-spawned) time_series plot: {att}"
    );

    // (3) ... and its frames RENDER — real Scalars data flows to the sink, so the
    //     auto-spawned plot has DATA (never the empty 1970 axis).
    assert!(
        wait_until(Duration::from_secs(3), || {
            flush.flush_blocking().ok();
            storage.num_msgs() > baseline
        }),
        "the scalar attach renders frames (data for the auto-spawned plot)"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Test 2 — two concurrent controllers, independent topics ─────────────────

#[test]
fn concurrent_controllers_independent_topics() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("vizd_concurrent");
    let topic_a = "/vizd/a";
    let topic_b = "/vizd/b";
    let _pa = Publisher::spawn(Arc::clone(&mgr), topic_a);
    let _pb = Publisher::spawn(Arc::clone(&mgr), topic_b);

    let (worker, flush, storage) = memory_worker("concurrent");
    let (socket, dir) = temp_socket("concurrent");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None, // rerun_url: a memory sink hosts no endpoint
    )
    .expect("daemon starts");

    flush.flush_blocking().ok();
    let baseline = storage.num_msgs();

    // TWO controllers, each attaches a DIFFERENT topic.
    let mut c1 = Client::connect(&socket);
    let mut c2 = Client::connect(&socket);
    assert_eq!(
        c1.request(&format!(
            r#"{{"id":1,"method":"attach","topic":"{topic_a}"}}"#
        ))["ok"]
            .as_bool(),
        Some(true)
    );
    assert_eq!(
        c2.request(&format!(
            r#"{{"id":1,"method":"attach","topic":"{topic_b}"}}"#
        ))["ok"]
            .as_bool(),
        Some(true)
    );

    // Both render (the shared sink grows), and a third controller's `list` shows
    // BOTH taps (multi-controller visibility of one shared daemon state).
    assert!(
        wait_until(Duration::from_secs(3), || {
            flush.flush_blocking().ok();
            storage.num_msgs() > baseline
        }),
        "both topics render"
    );
    let mut c3 = Client::connect(&socket);
    let list = c3.request(r#"{"id":9,"method":"list"}"#);
    assert!(entry_for(&list["attached"], "topic", topic_a).is_some());
    assert!(entry_for(&list["attached"], "topic", topic_b).is_some());

    // c1 detaches /a → c2's /b is UNDISTURBED (still attached, still rendering).
    assert_eq!(
        c1.request(&format!(
            r#"{{"id":2,"method":"detach","topic":"{topic_a}"}}"#
        ))["detached"]
            .as_bool(),
        Some(true)
    );
    let list2 = c3.request(r#"{"id":10,"method":"list"}"#);
    assert!(
        entry_for(&list2["attached"], "topic", topic_a).is_none(),
        "/a detached"
    );
    assert!(
        entry_for(&list2["attached"], "topic", topic_b).is_some(),
        "/b undisturbed by /a's detach"
    );
    // /b keeps rendering after /a's detach.
    flush.flush_blocking().ok();
    let mark = storage.num_msgs();
    assert!(
        wait_until(Duration::from_secs(3), || {
            flush.flush_blocking().ok();
            storage.num_msgs() > mark
        }),
        "/b still renders after /a detaches"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Test 3 — attach a missing topic surfaces the actionable error VERBATIM ───

#[test]
fn attach_missing_topic_errors_through_protocol() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("vizd_missing");
    let (worker, _flush, _storage) = memory_worker("missing");
    let (socket, dir) = temp_socket("missing");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None, // rerun_url: a memory sink hosts no endpoint
    )
    .expect("daemon starts");

    let mut client = Client::connect(&socket);
    let resp = client.request(r#"{"id":1,"method":"attach","topic":"/no/such/topic"}"#);
    assert_eq!(resp["ok"].as_bool(), Some(false));
    assert_eq!(resp["topic"].as_str(), Some("/no/such/topic"));
    assert!(
        resp["error"].as_str().unwrap().contains("does not exist"),
        "the actionable 'topic does not exist' error surfaces VERBATIM: {resp}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Test 4 — slot exhaustion surfaces the no-free-slot error VERBATIM ────────

#[test]
fn slot_exhaustion_errors_verbatim_through_protocol() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("vizd_slots");
    let topic = "/vizd/full";
    // Provision EXACTLY 4 subscriber slots, fill all four with raw taps → the
    // daemon's attach is the 5th, refused.
    let mut config = mgr.default_topic_config();
    config.max_subscribers = Some(4);
    let _publisher = mgr
        .create_publisher_with_topic_config(topic, MaxSliceLen::const_new(1 << 16), 0, config)
        .expect("producer attaches");
    let _held: Vec<_> = (0..4)
        .map(|_| {
            mgr.create_data_only_subscriber(topic)
                .expect("fill an introspection slot")
        })
        .collect();

    let (worker, _flush, _storage) = memory_worker("slots");
    let (socket, dir) = temp_socket("slots");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None, // rerun_url: a memory sink hosts no endpoint
    )
    .expect("daemon starts");

    let mut client = Client::connect(&socket);
    let resp = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(resp["ok"].as_bool(), Some(false));
    assert!(
        resp["error"]
            .as_str()
            .unwrap()
            .contains("subscriber slots are attached"),
        "the 'no free introspection slot' error surfaces VERBATIM: {resp}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Test 5 — the REMOTE arm: explicit errors + the seam pin ──

#[test]
fn remote_attach_on_a_network_less_daemon_surfaces_the_honest_residual() {
    let _statics = blueprint_statics_guard();
    // A LOCAL-ONLY (network: None) daemon — e.g. `CERULION_VIZD_NETWORK=off`, or a
    // test transport with no network config. Both remote-arm entry points (a
    // schema-less catalog resolve AND a pinned register) surface the explicit
    // not-network-configured residual, never a silent drop.
    //
    // The netd state is DECLARED, not inherited. Arm (a)'s residual is
    // reachable ONLY when netd is unreachable (netd's network is INDEPENDENT of
    // vizd's, so a REACHABLE netd resolves the type and the refusal moves to guard
    // 2a — that arm is `remote_attach_on_a_network_less_daemon_with_a_reachable_netd`
    // below). Run with the PRODUCTION plane against the desk's
    // well-known netd socket, this test sees a live go2 on a desk running netd and fails
    // with "robot 'go2' serves a catalog but names no ROS type" — so the verdict follows the desk.
    let mgr = isolated_transport("vizd_remote_local");
    let (worker, _flush, _storage) = memory_worker("remote_local");
    let (socket, dir) = temp_socket("remote_local");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        // EXPLICIT: no cerulion-netd on this desk (the arm-(a) precondition).
        Arc::new(NoNetdPlane),
    )
    .expect("daemon starts");

    let mut client = Client::connect(&socket);

    // (a) schema-LESS remote attach → the daemon would catalog-resolve the type over
    //     the network; netd is unreachable so it degrades to vizd's OWN session, and
    //     a network-less daemon cannot reach the robot → the explicit
    //     "not network-configured" residual (NOT a "schema required" error — a schema
    //     pin is not required).
    let no_schema = client.request(r#"{"id":1,"method":"attach","topic":"/go2/tf","robot":"go2"}"#);
    assert_eq!(no_schema["ok"].as_bool(), Some(false));
    let err = no_schema["error"].as_str().unwrap();
    assert!(
        err.contains("not network-configured"),
        "a schema-less remote attach on a network-less daemon names the residual: {no_schema}"
    );

    // (b) robot + a KNOWN pinned schema (walker resolves the hash locally — no
    //     fetch), but the daemon has NO network → the vizd-appropriate residual.
    //     It names the daemon kill switch, NOT the GRAPH-oriented
    //     core remediation ("add the graph `network:` block …") that
    //     register_ingress_topic embeds.
    let no_net = client.request(
        r#"{"id":2,"method":"attach","topic":"/go2/vel","robot":"go2","schema":"geometry_msgs/Vector3"}"#,
    );
    assert_eq!(no_net["ok"].as_bool(), Some(false));
    let err = no_net["error"].as_str().unwrap();
    assert!(
        err.contains("CERULION_VIZD_NETWORK") && err.contains("LOCAL-ONLY"),
        "the network-less daemon names the vizd kill switch: {no_net}"
    );
    assert!(
        !err.contains("graph"),
        "the vizd residual must NOT leak the graph-oriented core remediation: {no_net}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

/// The OTHER explicit residual of a network-less vizd — the arm that ambient
/// desk state would otherwise select at random. netd is REACHABLE and serves the catalog
/// (its network plane is INDEPENDENT of vizd's), so the schema-less type resolve
/// SUCCEEDS over netd's shared query plane; the attach is then refused by the
/// daemon's own kill-switch guard (2a) with the vizd-scoped LOCAL-ONLY residual —
/// never a silent drop, and never the graph-oriented core remediation.
///
/// Together with arm (a) of the sibling test, both netd states are pinned
/// DETERMINISTICALLY: unreachable ⇒ "not network-configured"; reachable ⇒
/// "CERULION_VIZD_NETWORK … LOCAL-ONLY".
#[test]
fn remote_attach_on_a_network_less_daemon_with_a_reachable_netd_names_the_kill_switch() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("vizd_reachable_netd");
    let (worker, _flush, _storage) = memory_worker("reachable_netd");
    let (socket, dir) = temp_socket("reachable_netd");
    // A netd that IS reachable and serves go2's catalog, naming a type the walker
    // already knows (so the resolve completes with NO schema fetch).
    let plane = Arc::new(ServingNetdPlane::new(vec![oracle_catalog(
        "go2",
        &[("/go2/vel", Some("geometry_msgs/Vector3"))],
    )]));
    // This arm's resolve SUCCEEDS (the catalog names the topic), so the
    // discovery marker is never consulted — declared anyway so the double's state is
    // never ambient.
    plane.set_discovery(DiscoveryState::Settled);
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        plane as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");

    let mut client = Client::connect(&socket);
    // Schema-LESS: the type is learned from netd's catalog, then guard 2a refuses.
    let resp = client.request(r#"{"id":1,"method":"attach","topic":"/go2/vel","robot":"go2"}"#);
    assert_eq!(resp["ok"].as_bool(), Some(false));
    let err = resp["error"].as_str().unwrap();
    assert!(
        err.contains("CERULION_VIZD_NETWORK") && err.contains("LOCAL-ONLY"),
        "a reachable netd resolves the type; the network-less daemon then names its own \
         kill switch: {resp}"
    );
    assert!(
        !err.contains("not network-configured"),
        "the resolve SUCCEEDED over netd, so the catalog-resolve residual must NOT appear \
         (the discriminator against arm (a) of the sibling test): {resp}"
    );
    assert!(
        !err.contains("graph"),
        "the vizd residual must NOT leak the graph-oriented core remediation: {resp}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn schema_less_attach_never_claims_a_missing_type_off_an_unconverged_gather() {
    let _statics = blueprint_statics_guard();
    // The SINGLE-robot query verb carried the same defect the
    // fan-out one did. `catalog_resolve_type` called `query_catalog` — which DROPS the
    // discovery-state marker — and rendered an empty/typeless answer as the terminal "robot
    // 'X' serves a catalog but names no ROS type for topic 'Y'".
    //
    // This is the CALL-SITE pin, and it exists because the pure oracle cannot see the
    // wiring: with `catalog_resolve_type` reverted to the older verb plus a
    // hardcoded `Settled`, `catalog_absence_verdict`'s own test stays green (the
    // function is still correct — nothing calls it with the marker any more). MEASURED:
    // that variant passed all 127 lib tests and all 66 e2e tests before this arm existed.
    //
    // ONE fixture, ONE variable: the same daemon, the same catalog (which serves a
    // DIFFERENT topic, so the resolve always fails), asked twice with only the declared
    // discovery state changed — and opposite verdicts.
    let mgr = isolated_transport("typeless");
    let (worker, _flush, _storage) = memory_worker("typeless");
    let (socket, dir) = temp_socket("typeless");
    let plane = Arc::new(ServingNetdPlane::new(vec![oracle_catalog(
        "go2",
        &[("/go2/other", Some("geometry_msgs/Vector3"))],
    )]));
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // ── UNCONVERGED: netd has not completed a discovery pass ──────────────────────
    plane.set_discovery(DiscoveryState::NotConverged);
    let a = client.request(r#"{"id":1,"method":"attach","topic":"/go2/vel","robot":"go2"}"#);
    assert_eq!(a["ok"].as_bool(), Some(false));
    let err = a["error"].as_str().unwrap();
    assert!(
        err.contains("go2") && err.contains("/go2/vel"),
        "still names the robot + topic: {a}"
    );
    // This fixture's catalog is NON-EMPTY (go2 answered, serving a
    // DIFFERENT topic), so the accurate weakness is "the view may be incomplete" —
    // NOT "nothing on the network answered", which would contradict the very
    // gather that produced this error. And never "no robot was discovered": the
    // single-robot GET this path uses runs no announce harvest at all.
    assert!(
        err.contains("discovery has not converged")
            && err.contains("this view of the network may be incomplete")
            && err.contains("is UNKNOWN (this is not a claim that it does not)"),
        "states the verdict as UNKNOWN, not a missing type: {a}"
    );
    assert!(
        !err.contains("nothing on the network answered"),
        "must not claim nothing answered — this robot's catalog came back: {a}"
    );
    assert!(
        !err.contains("no robot was discovered"),
        "and must not claim an announce verdict this query never observed: {a}"
    );
    assert!(
        // The head does not promise a duration this attach already
        // out-waited; the retry ADVICE is still there.
        err.contains("Retry in a few seconds")
            && err.contains("check the robot is powered on and on this network"),
        "retry-worthy AND hedged — a desk that never hears a robot stays NotConverged \
         for the daemon's whole life: {a}"
    );
    assert!(
        !err.contains("within a second or two"),
        "never promises sub-second convergence — measured ~3-13s: {a}"
    );
    // THE regression pin: the terminal claim the settled arm makes must be absent.
    assert!(
        !err.contains("serves a catalog"),
        "NEVER asserts the robot served a catalog off an unconverged gather: {a}"
    );

    // ── SETTLED: the anti-tautology twin. Discovery really ran, the robot really
    // answered, and its catalog really has no type for this topic — so the terminal
    // claim is EARNED and must still be made.
    plane.set_discovery(DiscoveryState::Settled);
    let b = client.request(r#"{"id":2,"method":"attach","topic":"/go2/vel","robot":"go2"}"#);
    assert_eq!(b["ok"].as_bool(), Some(false));
    let err = b["error"].as_str().unwrap();
    assert!(
        err.contains("serves a catalog but names no ROS type"),
        "a settled gather that DID return the catalog earns the terminal error: {b}"
    );
    assert!(
        err.contains("=pkg/Type"),
        "and keeps the explicit-type escape hatch: {b}"
    );
    assert!(!err.contains("is UNKNOWN"), "and must not hedge: {b}");

    // ── EMPTY gather (a robot this plane serves no catalog for):
    // SETTLED + empty must reuse `catalog_no_answer_error`'s EXISTING message —
    // "serves a catalog but names no ROS type" asserts an answer that never came,
    // and the netd path must not invent a second wording for a condition the
    // transient path already words (with the `--connect` + gateway hints).
    plane.set_discovery(DiscoveryState::Settled);
    let c = client.request(r#"{"id":3,"method":"attach","topic":"/ghost/vel","robot":"ghost"}"#);
    assert_eq!(c["ok"].as_bool(), Some(false));
    let err = c["error"].as_str().unwrap();
    assert!(
        err.contains("did not answer a schema catalog"),
        "an empty settled gather names what actually happened: {c}"
    );
    assert!(
        !err.contains("serves a catalog"),
        "never asserts an answer that never came: {c}"
    );
    assert!(
        err.contains("--connect") && err.contains("=pkg/Type"),
        "and carries BOTH hints of the shared message: {c}"
    );

    // EMPTY and UNCONVERGED: both facts, and never the announce disjunct the
    // single-robot GET cannot observe.
    plane.set_discovery(DiscoveryState::NotConverged);
    let d = client.request(r#"{"id":4,"method":"attach","topic":"/ghost/vel","robot":"ghost"}"#);
    assert_eq!(d["ok"].as_bool(), Some(false));
    let err = d["error"].as_str().unwrap();
    assert!(
        err.contains("did not answer a schema catalog")
            && err.contains("discovery has not converged")
            && err.contains("is UNKNOWN (this is not a claim that it does not)"),
        "states both facts and errs UNKNOWN: {d}"
    );
    assert!(
        !err.contains("no robot was discovered"),
        "never an announce verdict this query shape never observed: {d}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Test 5b (lazy session) — a network-CONFIGURED daemon opens NO zenoh session ──
// until a remote attach actually needs one (the lazy-session contract). Cribs
// the network_ingress_test lazy-session pin: `is_active()` stays false through
// start + local-only control traffic.

#[test]
fn network_configured_daemon_opens_no_session_without_a_remote_attach() {
    let mgr = isolated_transport_with_network("vizd_lazy");
    let net = mgr.network().expect("network configured");
    assert!(
        !net.is_active(),
        "a network-configured manager opens NO session before any register_ingress_topic"
    );

    let (worker, _flush, _storage) = memory_worker("lazy");
    let (socket, dir) = temp_socket("lazy");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");

    // LOCAL-only control traffic (discover/list/status) never touches the network.
    let mut client = Client::connect(&socket);
    let _ = client.request(r#"{"id":1,"method":"discover"}"#);
    let _ = client.request(r#"{"id":2,"method":"list"}"#);
    let _ = client.request(r#"{"id":3,"method":"status"}"#);
    assert!(
        !net.is_active(),
        "starting the daemon + local-only control traffic must NOT open the zenoh session \
         (the purely-local daemon stays session-free)"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Test 5c — the remote-arm SEAM pin: a network-configured daemon
// DEMANDS the shared mirror from an in-process cerulion-netd on attach{robot,schema}
// and taps the netd-created local mirror. netd registers the ingress + the mirror
// provenance (on the shared `mgr`); this pins the demand + tap + provenance fold.
// The cross-machine frame FLOW is the loopback headline (test 5e).

#[test]
fn network_configured_daemon_demands_from_netd_and_taps_on_remote_attach() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport_with_network("vizd_remote_net");
    // An IN-PROCESS netd sharing `mgr` — vizd DEMANDS the mirror from it
    // (netd creates the ingress + provenance on `mgr`); vizd registers no
    // ingress topic of its own. The provenance asserted below is written by netd (on
    // the SAME `mgr`).
    let (mut netd, netd_sock, netd_dir) = start_in_process_netd("remote_net", Arc::clone(&mgr));
    let (worker, _flush, _storage) = memory_worker("remote_net");
    let (socket, dir) = temp_socket("remote_net");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::new(NetdDemandPlane::with_socket(netd_sock)),
    )
    .expect("daemon starts");

    let mut client = Client::connect(&socket);
    // A remote attach with a known, NAME-MAPPED schema (`tf2_msgs/TFMessage` →
    // `Transforms` in `classify_schema`): the daemon resolves the hash, declares
    // gateway ingress (lazy zenoh session opens here), taps the re-injected local
    // mirror, and reports the controller-provided schema + its archetype WITHOUT a
    // frame (a shape-inferred type like Vector3 would back-fill its archetype once
    // frames flow — the no-frame contract).
    let topic = "/go2net/tf";
    let resp = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}","robot":"go2","schema":"tf2_msgs/TFMessage"}}"#
    ));
    assert_eq!(
        resp["ok"].as_bool(),
        Some(true),
        "a network-configured daemon accepts the remote attach: {resp}"
    );
    assert_eq!(resp["topic"].as_str(), Some(topic));
    assert_eq!(
        resp["schema"].as_str(),
        Some("tf2_msgs/TFMessage"),
        "reports the controller-provided schema (no frame yet): {resp}"
    );
    assert_eq!(
        resp["archetype"].as_str(),
        Some("Transforms"),
        "resolves the archetype from the known name-mapped type: {resp}"
    );
    assert_eq!(resp["route"].as_str(), Some("go2net/tf"));
    // The remote-attach arm carries the same deterministic layout
    // metadata (Transforms → a Transform3D family in the 3D scene).
    // A TFMessage renders at per-frame entities under the transform
    // tree, so the reported entity is the tree ROOT (not a per-topic path).
    assert_eq!(resp["entity"].as_str(), Some("world"));
    assert_eq!(resp["components"], serde_json::json!(["Transform3D"]));
    assert_eq!(resp["view_kinds"], serde_json::json!(["spatial3d"]));

    // The tap is attached (the ingress was declared + the local mirror tapped):
    // `list` shows it. This is the DECLARATION pin; frames only flow once a real
    // gateway egresses the robot, which this test does not run.
    let list = client.request(r#"{"id":2,"method":"list"}"#);
    let le = entry_for(&list["attached"], "topic", topic).expect("remote tap listed");
    assert_eq!(le["schema"].as_str(), Some("tf2_msgs/TFMessage"));
    assert_eq!(le["archetype"].as_str(), Some("Transforms"));
    // `list` carries the deterministic layout metadata too
    // (Transforms → a Transform3D family in the 3D scene).
    assert_eq!(le["components"], serde_json::json!(["Transform3D"]));
    assert_eq!(le["view_kinds"], serde_json::json!(["spatial3d"]));

    // The PRODUCTION callsite,
    // `attach_remote` registering mirror PROVENANCE, is asserted e2e. The daemon's
    // manager registry must carry `(topic → go2)` so `topic list` folds this remote
    // mirror into REMOTE (bounded retry for the background republish + warm-up).
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut provenance_robot = None;
    while Instant::now() < deadline {
        let snapshot = mgr
            .gather_mirror_provenance(Duration::from_millis(300))
            .expect("gather provenance");
        if let Some(rec) = snapshot.iter().find(|r| r.topic == topic) {
            provenance_robot = Some(rec.origin_robot.clone());
            break;
        }
    }
    assert_eq!(
        provenance_robot.as_deref(),
        Some("go2"),
        "the netd demand must register the mirror's provenance attributed to the robot"
    );

    daemon.shutdown();
    netd.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
    let _ = std::fs::remove_dir_all(&netd_dir);
}

// ── Test 5c-rollback — a tap failure AFTER a
// successful demand RELEASES the demand + rolls back the claim, so the netd
// refcount does not leak (the controller got an error, so it never detaches). Uses
// a RecordingDemandPlane whose demand SUCCEEDS but creates NO real mirror, so the
// tap step fails on the non-existent {topic}/data service — the exact fault.

#[test]
fn remote_attach_tap_failure_rolls_back_the_netd_demand() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport_with_network("netd_rollback");
    let plane = Arc::new(RecordingDemandPlane::default());
    let (worker, _flush, _storage) = memory_worker("rollback");
    let (socket, dir) = temp_socket("rollback");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // A pinned schema the walker already knows (network-free resolution) so the
    // attach reaches the demand + tap steps. The spy demand SUCCEEDS but no real
    // mirror exists, so the tap open fails.
    let topic = "/go2net/rollback";
    let attach = format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}","robot":"go2","schema":"tf2_msgs/TFMessage"}}"#
    );
    let resp = client.request(&attach);

    // (1) The tap failure surfaces an ERROR (not a phantom ok).
    assert_eq!(
        resp["ok"].as_bool(),
        Some(false),
        "a tap failure surfaces an error: {resp}"
    );
    assert!(
        resp["error"].as_str().is_some(),
        "the error carries the tap failure text: {resp}"
    );

    // (2) The demand was made ONCE, then RELEASED (the rollback — hand oracle).
    assert_eq!(
        plane.demands.lock().unwrap().len(),
        1,
        "demanded exactly once"
    );
    assert_eq!(
        plane.releases.lock().unwrap().clone(),
        vec![("go2".to_string(), topic.to_string())],
        "the failed tap released the demand it had made"
    );

    // (3) The CLAIM was rolled back (not leaked): a SECOND attach RE-DEMANDS. A
    //     leaked claim would make this a no-op (demand_count stuck at 1).
    let _ = client.request(&format!(
        r#"{{"id":2,"method":"attach","topic":"{topic}","robot":"go2","schema":"tf2_msgs/TFMessage"}}"#
    ));
    assert_eq!(
        plane.demands.lock().unwrap().len(),
        2,
        "the rolled-back claim re-demands on a retry (proves the claim was not leaked)"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Test 5c-reconnect — the NetdDemandPlane's ONE
// persistent connection RECONNECTS after the daemon dies, so a netd crash does NOT
// permanently wedge vizd's remote arm. Drives the
// PRODUCTION NetdDemandPlane directly against a real in-process netd. Hand oracle on
// the demand outcomes: [Ok (netd1), Err (dead socket), Ok (reconnect to netd2)].

#[test]
fn netd_demand_plane_reconnects_after_the_daemon_dies() {
    use cerulion_vizd::{DemandPlane, NetdDemandPlane};

    let (dir, sock) = {
        let dir = std::env::temp_dir().join(format!(
            "reconnect_{}_{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        std::fs::create_dir_all(&dir).expect("mk tempdir");
        let sock = dir.join("netd.sock");
        (dir, sock)
    };

    let netd1 = start_netd_ok_plane(sock.clone());
    let plane = NetdDemandPlane::with_socket(sock.clone());

    // T1: connects to netd1 + succeeds.
    plane
        .demand("go2", "/tf", 1)
        .expect("first demand connects to netd1 and succeeds");

    // Kill netd1 (synchronous shutdown joins its threads + closes the socket).
    drop(netd1);

    // T2: the held client hits the dead socket → Err. Internally this DROPS the
    // client (should_drop_client_on_error is true for the Io error), which T3 proves.
    let err = plane
        .demand("go2", "/tf", 1)
        .expect_err("a demand against a dead netd errors, not hangs");
    assert!(
        !err.is_empty(),
        "the error is surfaced, not swallowed: {err}"
    );

    // T3: a FRESH netd on the SAME socket → the plane RECONNECTS + succeeds. A plane
    // that did NOT drop the wedged client would reuse the dead socket and fail here.
    let netd2 = start_netd_ok_plane(sock.clone());
    plane
        .demand("go2", "/tf", 1)
        .expect("the wedge is gone — the plane reconnects to the fresh netd and succeeds");

    drop(netd2);
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Test 5d: remote attach is idempotent + survives
// detach. A daemon DEMANDS a remote topic's netd mirror at most ONCE per topic, so
// a REPEAT remote attach reports already_attached (idempotent per-connection), a
// detach RELEASES the demand (netd's refcount-0 teardown frees the mirror slot),
// and a re-attach AFTER detach RE-DEMANDS cleanly (netd re-creates the mirror — no
// double-register refusal). The daemon does not keep the ingress marker across detach:
// detach truly releases and re-attach re-demands.

#[test]
fn remote_reattach_is_idempotent_and_survives_detach() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport_with_network("vizd_reattach");
    let (mut netd, netd_sock, netd_dir) = start_in_process_netd("reattach", Arc::clone(&mgr));
    let (worker, _flush, _storage) = memory_worker("reattach");
    let (socket, dir) = temp_socket("reattach");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::new(NetdDemandPlane::with_socket(netd_sock)),
    )
    .expect("daemon starts");

    let mut client = Client::connect(&socket);
    let topic = "/go2net/reattach";
    let attach = format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}","robot":"go2","schema":"tf2_msgs/TFMessage"}}"#
    );

    // (1) First remote attach → ok, already_attached:false (mirror demanded).
    let a1 = client.request(&attach);
    assert_eq!(
        a1["ok"].as_bool(),
        Some(true),
        "first remote attach ok: {a1}"
    );
    assert_eq!(
        a1["already_attached"].as_bool(),
        Some(false),
        "the first attach opens a new tap: {a1}"
    );

    // (2) REPEAT the SAME remote attach → ok + already_attached:TRUE (idempotent).
    //     The reserve-then-commit skips a second netd demand for the held topic.
    let a2 = client.request(&format!(
        r#"{{"id":2,"method":"attach","topic":"{topic}","robot":"go2","schema":"tf2_msgs/TFMessage"}}"#
    ));
    assert_eq!(
        a2["ok"].as_bool(),
        Some(true),
        "a repeat remote attach is idempotent, NOT a double demand: {a2}"
    );
    assert_eq!(
        a2["already_attached"].as_bool(),
        Some(true),
        "the repeat attach reports already_attached: {a2}"
    );

    // (3) detach → ok.
    let det = client.request(&format!(
        r#"{{"id":3,"method":"detach","topic":"{topic}"}}"#
    ));
    assert_eq!(det["ok"].as_bool(), Some(true));
    assert_eq!(det["detached"].as_bool(), Some(true));

    // (4) RE-attach after detach → ok, already_attached:false again (a fresh tap
    //     over a RE-DEMANDED mirror — the detach released the demand + netd's
    //     teardown freed the slot, so this re-demand cleanly re-creates it).
    let a3 = client.request(&format!(
        r#"{{"id":4,"method":"attach","topic":"{topic}","robot":"go2","schema":"tf2_msgs/TFMessage"}}"#
    ));
    assert_eq!(
        a3["ok"].as_bool(),
        Some(true),
        "re-attach after detach works (re-demands the released mirror): {a3}"
    );
    assert_eq!(
        a3["already_attached"].as_bool(),
        Some(false),
        "the re-attach opens a fresh tap: {a3}"
    );
    // The re-attached tap is listed again.
    let list = client.request(r#"{"id":5,"method":"list"}"#);
    assert!(
        entry_for(&list["attached"], "topic", topic).is_some(),
        "the re-attached tap is listed: {list}"
    );

    daemon.shutdown();
    netd.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
    let _ = std::fs::remove_dir_all(&netd_dir);
}

/// A counting demand-plane double whose `demand` really MAKES THE TOPIC
/// TAPPABLE — the only shape that can reach the compose ROLLBACK path.
///
/// A plane that merely records would fail the tap immediately after the demand, and
/// `attach_remote`'s own error arm would roll the demand back before
/// `rollback_compose_attaches` ever ran — so the leak under test would be
/// unreachable and the test vacuous. Modelling netd faithfully ("a demand makes a
/// mirror appear") is what puts a HELD demand in front of a compose that then fails
/// for an unrelated, legitimate reason.
///
/// The mirror publisher is SILENT, which is the second half of the setup: compose
/// resolves no archetype for it, refuses with `NoRenderableTopics`, and rolls back.
struct MirroringCountingPlane {
    manager: Arc<TransportManager>,
    schema_name: String,
    /// Kept alive so the mirror's `{topic}/data` service stays open for the tap.
    mirrors: std::sync::Mutex<Vec<cerulion_core::transport::publisher::CerulionPublisher>>,
    demands: std::sync::Mutex<Vec<(String, String)>>,
    releases: std::sync::Mutex<Vec<(String, String)>>,
}

impl MirroringCountingPlane {
    fn new(manager: Arc<TransportManager>, schema_name: &str) -> Self {
        Self {
            manager,
            schema_name: schema_name.to_string(),
            mirrors: std::sync::Mutex::new(Vec::new()),
            demands: std::sync::Mutex::new(Vec::new()),
            releases: std::sync::Mutex::new(Vec::new()),
        }
    }
    fn demand_count(&self) -> usize {
        self.demands.lock().unwrap().len()
    }
    fn release_count(&self) -> usize {
        self.releases.lock().unwrap().len()
    }
}

impl cerulion_vizd::DemandPlane for MirroringCountingPlane {
    fn demand(&self, robot: &str, topic: &str, _schema_hash: u64) -> Result<(), String> {
        self.demands
            .lock()
            .unwrap()
            .push((robot.to_string(), topic.to_string()));
        // Model netd: the demand creates the desk-local mirror the daemon then taps.
        // Idempotent — a second demand for a live mirror keeps the existing one
        // (SHM is single-writer, so re-creating would refuse).
        let already = self
            .manager
            .list_topics()
            .is_ok_and(|t| t.iter().any(|x| x == topic));
        if !already {
            let publisher = self
                .manager
                .create_publisher(topic, MaxSliceLen::const_new(1 << 16), 0)
                .map_err(|e| format!("spy mirror create failed: {e}"))?;
            self.mirrors.lock().unwrap().push(publisher);
        }
        Ok(())
    }
    fn release(&self, robot: &str, topic: &str) -> Result<(), String> {
        self.releases
            .lock()
            .unwrap()
            .push((robot.to_string(), topic.to_string()));
        // Model netd's refcount-0 teardown: the mirror goes, so the topic is not
        // locally visible again and the NEXT compose must re-demand it.
        self.mirrors.lock().unwrap().clear();
        Ok(())
    }
    fn query_catalog(&self, robot: &str) -> Result<Vec<cerulion_core::CatalogReply>, String> {
        self.query_catalog_all()
            .map(|all| all.into_iter().filter(|c| c.robot == robot).collect())
    }
    fn query_catalog_all(&self) -> Result<Vec<cerulion_core::CatalogReply>, String> {
        Ok(vec![oracle_catalog(
            "rollback-robot",
            &[("/rollback/leak", Some(self.schema_name.as_str()))],
        )])
    }
    fn query_schema(
        &self,
        _robot: &str,
        _requested: &str,
    ) -> Result<Vec<cerulion_core::SchemaReply>, String> {
        Ok(Vec::new())
    }
}

/// **A compose that demands a remote mirror and then FAILS must release
/// the demand.**
///
/// Detaching the tap and removing the stats entry is "the exact inverse of
/// `attach_for_compose`" only for a LOCAL attach. A remote attach also routes through
/// the demand plane, so `rollback_compose_attaches` must release the demand too.
/// Otherwise a compose that demands a mirror and then refuses (no renderable archetype,
/// a rejected layout) drops the tap while LEAVING THE DEMAND HELD: netd keeps
/// re-injecting, frames keep crossing the network for the life of this daemon, and
/// there is no row and no `detach` target to stop it. A leak with no UI.
///
/// The oracle is in two halves, and the SECOND is the load-bearing one:
///
/// * `demand_count == 1` after the failed compose — the setup actually reached the
///   demand plane (without this the rest is vacuous), but it passes the buggy code
///   too, so it proves nothing on its own;
/// * **`release_count == 1`** — the leak itself. A rollback that skips the release reads 0.
///
/// Then a THIRD assertion stands in for "the `demanded` table is empty" without
/// adding a test-only accessor to production: a SECOND compose must demand AGAIN
/// (`demand_count == 2`). If the entry had survived the rollback,
/// `attach_remote`'s reserve-then-commit would treat the topic as already demanded
/// and issue no second demand — so a stale entry is observable as a MISSING demand.
#[test]
fn a_failed_compose_releases_the_netd_demand_it_opened_e2e() {
    let _statics = blueprint_statics_guard();
    // A NETWORK-configured manager: `should_resolve_remote` requires one (a
    // network-less daemon has no remote to resolve, by design).
    let mgr = isolated_transport_with_network("rollback_leak");
    let topic = "/rollback/leak";
    // The plane creates the mirror on demand and tears it down on release.
    let plane = Arc::new(MirroringCountingPlane::new(
        Arc::clone(&mgr),
        "geometry_msgs/Vector3",
    ));

    let (worker, _flush, _storage) = memory_worker("rb");
    let (socket, dir) = temp_socket("rb");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // The topic is NOT locally visible, so compose must resolve it remotely.
    assert!(
        mgr.data_service_missing(topic),
        "precondition: the mirror does not exist until the demand creates it"
    );

    let c1 = client.request(&format!(
        r#"{{"id":1,"method":"compose_layout","intent":{{"topics":["{topic}"]}}}}"#
    ));
    // The compose FAILS — the mirror publishes nothing, so no archetype resolves.
    // That is the legitimate refusal this test needs; the leak is what it could leave behind.
    assert_eq!(
        c1["ok"].as_bool(),
        Some(false),
        "precondition: a silent mirror resolves no archetype, so this compose refuses \
         and takes the rollback path: {c1}"
    );
    assert_eq!(
        plane.demand_count(),
        1,
        "precondition (vacuous on its own): the compose really did demand the mirror"
    );

    // THE PIN.
    assert_eq!(
        plane.release_count(),
        1,
        "a failed compose must RELEASE the demand it opened — otherwise netd keeps \
         re-injecting this topic for the life of the daemon with no row and no \
         detach target"
    );

    // Stands in for "the demanded table is empty": a stale entry would suppress the
    // second demand.
    let c2 = client.request(&format!(
        r#"{{"id":2,"method":"compose_layout","intent":{{"topics":["{topic}"]}}}}"#
    ));
    assert_eq!(c2["ok"].as_bool(), Some(false), "still silent: {c2}");
    assert_eq!(
        plane.demand_count(),
        2,
        "the rollback cleared the demand table, so a re-compose re-demands (a stale \
         entry shows up here as a MISSING demand)"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

/// **The leftover-mirror drop, on the PRODUCTION `discover` path.**
///
/// The fold's snapshot-based drop gate is pinned by pure oracles that
/// pass `DiscoveryState::Settled` by hand. Those alone leave a hole: every test
/// double inherits the trait's fail-closed `NotConverged` default, so without this test
/// NOTHING exercises the drop through `Ctx::discover` — hardcoding `NotConverged` in
/// `discover`, or deleting the assignment that carries the gather's state, would leave the
/// serve-swapped-row feature fully inert with the entire suite green. A
/// feature nothing runs on the production path is not shipped, so this test
/// pins the production call site.
///
/// The row is built the way the real one is: a desk-local mirror service with a
/// provenance record attributing it to a robot, TAPPED by this daemon (so its own
/// observation exists), driven to a DATED-IDLE verdict by publishing frames and then
/// going quiet past the streaming-recency window. The robot's catalog serves OTHER
/// topics but not this one — the serve-swap signature.
///
/// Both stages, with the discovery state as the ONLY variable between them:
///
/// * `Settled` — the gather is authoritative, so the leftover is DROPPED and the
///   topic is accounted for NOWHERE (not local, not under its robot). That absence
///   is what hides the sidebar row.
/// * `NotConverged` — the same inputs prove nothing, so the row STAYS.
#[test]
fn a_leftover_mirror_is_dropped_through_discover_only_when_discovery_is_settled_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("m1_discover_drop");
    let jpeg = "/imgm/jpeg";

    // The MIRROR: a live data service + the provenance record netd's re-injector
    // writes. `_mirror` is held for the whole test — netd would keep it up while the
    // desk still demands the topic, which is exactly why the desk-side signals cannot
    // notice the robot stopped serving it.
    let mut mirror = mgr
        .create_publisher(jpeg, MaxSliceLen::const_new(1 << 16), 0)
        .expect("mirror service attaches");
    assert!(
        mgr.register_mirror_provenance(jpeg, "go2")
            .expect("register mirror provenance"),
        "the mirror is attributed to go2"
    );

    // go2's catalog serves OTHER topics and NOT the mirrored one — the serve-swap.
    let plane = Arc::new(GatherPlane::new(vec![oracle_catalog(
        "go2",
        &[("/imgm/tf", None), ("/imgm/odom", None)],
    )]));

    let (worker, _flush, _storage) = memory_worker("m1");
    let (socket, dir) = temp_socket("m1");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // TAP it, so this daemon has its own observation — the second axis of the drop's
    // evidence (a catalog alone may never hide a row).
    let att = client.request(&format!(r#"{{"id":1,"method":"attach","topic":"{jpeg}"}}"#));
    assert_eq!(att["ok"].as_bool(), Some(true), "attach the mirror: {att}");

    // Two frames: the first is the tap's banked BASELINE (never dated — it could be a
    // retained-history flush), the second dates the topic. Then go quiet.
    for seq in 0..2u32 {
        mirror
            .publish_raw(&build_vector3_frame(
                seq,
                1_000 * (seq as u64 + 1),
                [1.0, 2.0, 3.0],
            ))
            .expect("publish");
        std::thread::sleep(Duration::from_millis(120));
    }
    // Wait until the tap's own verdict is a DATED idle — the shape the gate acts on.
    assert!(
        wait_until(Duration::from_secs(20), || {
            let st = client.request(r#"{"id":2,"method":"status"}"#);
            entry_for(&st["topics"], "topic", jpeg).is_some_and(|r| {
                r["liveness_state"].as_str() == Some("idle")
                    && r["liveness"]["last_frame_age_ms"].as_u64().is_some()
            })
        }),
        "precondition: the tap observed frames and then went quiet past the \
         streaming window, so its verdict is a DATED idle"
    );

    // WHERE the row is, as three DISTINCT
    // states rather than one boolean.
    //
    // A single boolean would collapse "under its robot" and "under LOCAL topics" into
    // one `listed_anywhere`, and that hides the failure mode. `discover` folds a
    // mirror out of LOCAL only if `Ctx::attribution_snapshot` says whose it is —
    // a CACHED, best-effort SHM gather (see `ATTRIBUTION_CONVERGENCE_BOUND`). The
    // tap's own record is the OTHER half of that map and it does not rescue this:
    // `attach_local` writes it from the very snapshot it replies with, so a gather
    // that missed AT ATTACH TIME leaves the record empty too. Either way a missed
    // gather leaves the topic rendering as a genuine LOCAL row for a TTL — which
    // `listed_anywhere` would score as "the row stays", i.e. the NotConverged arm
    // passes VACUOUSLY, and the Settled arm then fails. The signature on a loaded
    // runner: `/imgm/jpeg` present in `topics` as a full
    // local row with its own archetype, while go2 rows `tf` + `odom`.
    //
    // Split three ways, each state means something different and only one of them
    // is a retry:
    //
    // * `UnderRobot` — attribution converged AND the mirror was KEPT.
    // * `Nowhere`    — attribution converged AND the mirror was DROPPED.
    // * `Local`      — attribution has NOT converged; the question was not asked.
    //
    // NOTHING is loose here: the Settled arm demands `Nowhere` exactly, and
    // the NotConverged arm is STRICTLY STRONGER than a boolean — it demands
    // `UnderRobot`, the state the drop would remove, which a boolean could
    // not tell apart from the un-converged local row.
    #[derive(Debug, PartialEq, Eq, Clone, Copy)]
    enum Placement {
        UnderRobot,
        Local,
        Nowhere,
    }
    let placement = |resp: &Value| -> Placement {
        let local = resp["topics"]
            .as_array()
            .is_some_and(|a| a.iter().any(|e| e["topic"].as_str() == Some(jpeg)));
        let remote = resp["robots"].as_array().is_some_and(|rs| {
            rs.iter().any(|r| {
                r["topics"]
                    .as_array()
                    .is_some_and(|a| a.iter().any(|e| e["topic"].as_str() == Some(jpeg)))
            })
        });
        match (remote, local) {
            (true, _) => Placement::UnderRobot,
            (false, true) => Placement::Local,
            (false, false) => Placement::Nowhere,
        }
    };

    // ── NOT CONVERGED: the same inputs prove nothing. The row STAYS. ─────────
    //
    // Polled, because `Local` here means "attribution has not resolved yet", not an
    // answer. `Nowhere` under NotConverged WOULD be an answer — the wrong one —
    // so it fails FAST rather than burning the budget: an absence claim licensed
    // by an unconverged gather is the bug this arm exists to catch.
    plane.set_discovery(DiscoveryState::NotConverged);
    let deadline = Instant::now() + ATTRIBUTION_CONVERGENCE_BOUND;
    let disc = loop {
        let disc = client.request(r#"{"id":3,"method":"discover"}"#);
        match placement(&disc) {
            Placement::UnderRobot => break disc,
            Placement::Nowhere => panic!(
                "an unconverged gather licenses NO absence claim, but the mirror was \
                 dropped anyway — accounted for nowhere: {disc}"
            ),
            Placement::Local => {
                assert!(
                    Instant::now() < deadline,
                    "the mirror never got attributed to go2 within {ATTRIBUTION_CONVERGENCE_BOUND:?} \
                     — it is still rendering as a LOCAL topic, so the drop gate was never \
                     even reached and this test proves nothing: {disc}"
                );
                std::thread::sleep(Duration::from_millis(50));
            }
        }
    };
    assert_eq!(
        placement(&disc),
        Placement::UnderRobot,
        "an unconverged gather licenses no absence claim — the row stays, under its \
         robot: {disc}"
    );

    // ── SETTLED: authoritative, so the leftover is dropped and the row hides. ──
    //
    // Attribution is PROVEN live by the arm above (it just observed the mirror
    // folded onto go2), and the daemon serves that snapshot from its cache, so
    // this read is the same inputs with ONE variable changed — the discovery
    // state. A `Local` answer here would mean attribution lapsed between the two
    // reads (a cache refresh that missed), which is not an answer to the question
    // either, so it is retried on the same bound; `UnderRobot` is the real
    // negative and fails immediately.
    //
    // `placement` is computed over `jpeg` ALONE, so `Nowhere` is equally
    // satisfied by a response in which the go2 row is ABSENT entirely — while the
    // assertions below require that row AND its own two served topics. Breaking on
    // `Nowhere` alone was therefore the subset-wait class: the loop could
    // return a response that never carried what `.expect("go2 still rows")` reads.
    // `go2_serves_its_own` mirrors that extraction so the gate cannot drift from it.
    plane.set_discovery(DiscoveryState::Settled);
    let go2_serves_its_own = |resp: &Value| -> bool {
        resp["robots"]
            .as_array()
            .and_then(|rs| rs.iter().find(|r| r["robot"].as_str() == Some("go2")))
            .and_then(|go2| go2["topics"].as_array())
            .is_some_and(|ts| {
                let served: Vec<&str> = ts.iter().filter_map(|e| e["topic"].as_str()).collect();
                served.contains(&"/imgm/tf") && served.contains(&"/imgm/odom")
            })
    };
    let deadline = Instant::now() + ATTRIBUTION_CONVERGENCE_BOUND;
    let disc = loop {
        let disc = client.request(r#"{"id":4,"method":"discover"}"#);
        match placement(&disc) {
            Placement::Nowhere if go2_serves_its_own(&disc) => break disc,
            // The leftover IS dropped, but the robot's own row has not arrived yet —
            // retry on the same bound rather than assert against a response that
            // cannot answer the surgical-drop question.
            Placement::Nowhere => {
                assert!(
                    Instant::now() < deadline,
                    "the leftover mirror was dropped, but go2's OWN row never carried \
                     its served topics within {ATTRIBUTION_CONVERGENCE_BOUND:?}, so the \
                     surgical-drop assertions below would read a response that never \
                     had them: {disc}"
                );
                std::thread::sleep(Duration::from_millis(50));
            }
            Placement::UnderRobot => panic!(
                "THE production-path pin: a settled gather that serves OTHER topics but \
                 not this one must DROP the leftover mirror, so the topic is accounted \
                 for NOWHERE and the sidebar hides the row — it is still listed under \
                 its robot: {disc}"
            ),
            Placement::Local => {
                assert!(
                    Instant::now() < deadline,
                    "attribution lapsed mid-test: the mirror fell back to a LOCAL row and \
                     never returned within {ATTRIBUTION_CONVERGENCE_BOUND:?}, so the drop \
                     gate was never reached: {disc}"
                );
                std::thread::sleep(Duration::from_millis(50));
            }
        }
    };
    assert_eq!(
        placement(&disc),
        Placement::Nowhere,
        "THE production-path pin: a settled gather that serves OTHER topics but not \
         this one drops the leftover mirror, so the topic is accounted for NOWHERE \
         and the sidebar hides the row: {disc}"
    );
    // The robot's OWN topics are untouched — the drop is surgical, not a robot-wide
    // erasure.
    let go2 = disc["robots"]
        .as_array()
        .and_then(|rs| rs.iter().find(|r| r["robot"].as_str() == Some("go2")))
        .expect("go2 still rows");
    let served: Vec<&str> = go2["topics"]
        .as_array()
        .map(|a| a.iter().filter_map(|e| e["topic"].as_str()).collect())
        .unwrap_or_default();
    assert!(
        served.contains(&"/imgm/tf") && served.contains(&"/imgm/odom"),
        "the robot's served topics survive: {served:?}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

/// **Re-attach after detach, both surfaces.**
///
/// A controller that detaches `/lowstate` (a netd mirror) must be able to bring it
/// back. A `compose_layout` with no remote arm fails with:
///
/// ```text
/// compose_layout: could not attach '/lowstate': ... PublishSubscribeOpenError::DoesNotExist
///   — the topic does not exist (the recording/replay tap is open-only and never creates services)
/// ```
///
/// Retrying discovery cannot recover from that. The same topic
/// re-attaches fine from the SIDEBAR, which is the tell: the failure is not "the
/// topic is gone", it is WHICH ATTACH PATH RAN. Detach releases the netd demand,
/// netd's refcount-0 teardown frees the local SHM service, and a `compose_layout`
/// that drives the tap manager DIRECTLY has no remote arm to fall back to,
/// so the remote routing must serve `compose_layout` as well as the `attach` verb.
///
/// Run against a REAL serving gateway (crib `schema_less_remote_attach_catalog_
/// resolves_seeds_and_taps`) rather than a bare netd, because that is what the
/// re-attach actually needs: neither surface under test can carry a schema pin —
/// `compose_layout` has no field for one and a no-`robot` `attach` supplies none —
/// so both must resolve the type from the robot's CATALOG. A harness without a
/// gateway fails for the wrong reason and would prove nothing. (Measured: with
/// a bare netd this test fails with "robot 'go2' ... is
/// unreachable", which is the remote routing WORKING and the harness missing.)
///
/// Two arms over one detach→immediate-re-attach cycle:
///
/// 1. **`compose_layout`** — the verb an agent drives, and the structurally sensitive
///    one: without a remote arm it cannot resolve a remote topic AT ALL, detach or no
///    detach, so this arm fails deterministically on that defect rather than
///    depending on teardown timing.
/// 2. **the `attach` verb with NO `robot`** — the agent's other route, and the
///    sidebar's once a session tombstone exists.
///    `remote_reattach_is_idempotent_and_survives_detach` already covers the
///    WITH-`robot` form; this pins the form that must RESOLVE where the topic lives.
///
/// Each arm asserts the tap is really HELD afterwards, not just that the verb
/// answered `ok` — a green answer with no tap is the same bug wearing a different
/// face.
///
/// KILL MATRIX — what deleting each routing arm of `attach_for_compose` does to
/// this test. Each arm alone is redundant with the other, so only deleting both
/// is caught:
///
/// * Delete the PRE-open remote routing from `attach_for_compose` (leave the
///   post-failure fallback): **SURVIVES.** The local open fails, the fallback
///   re-asks the predicate, and the re-attach succeeds anyway.
/// * Delete the POST-failure fallback (leave the pre-open routing): **SURVIVES.**
///   The pre-open check already routed remote, so the local open never ran.
/// * Delete BOTH: **FAILS**, and it fails with the
///   error quoted above, verbatim, `PublishSubscribeOpenError::
///   DoesNotExist — ... (the recording/replay tap is open-only and never creates
///   services)`.
///
/// So what this test pins is the PROPERTY — compose_layout can resolve a remote
/// topic at all — and NEITHER arm individually. That is a real limitation worth
/// naming rather than papering over: the two arms are genuinely redundant for a
/// cleanly-torn-down mirror, and they stop being redundant only inside the
/// millisecond window where the DYING mirror is still visible at routing time,
/// which this harness cannot force open without a fault-injection seam. Keeping
/// both is deliberate — the pre-check avoids a pointless failed open on the common
/// path, the fallback covers the window — but a future reader must not read either
/// one as independently guarded.
#[test]
fn a_remote_topic_re_attaches_right_after_detach_on_both_surfaces_e2e() {
    let _statics = blueprint_statics_guard();
    let robot = "reattachgw";
    let topic = "/lowstate";
    // A BUILT-IN type, so the catalog resolve needs no fetch — this test is about
    // attach ROUTING, not schema acquisition (which has its own e2e).
    let serving = SchemaServing {
        topic_schemas: vec![TopicSchema {
            topic: topic.to_string(),
            schema_name: "geometry_msgs/Vector3".to_string(),
        }],
        schema_docs: vec![],
        schema_hashes: vec![],
    };
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![topic.to_string()],
        ingress: vec![],
    };
    let (p_transport, _a_clock, gateway, port) =
        boot_serving_gateway("redetach", robot, plan, serving);

    // A background DRIVER: machine A keeps publishing while the gateway keeps
    // forwarding. Both halves are needed, and for a reason worth stating — the
    // FIRST version of this test left the gateway un-driven, and `compose_layout`
    // then failed with "none of the 1 requested topic(s) resolved to a renderable
    // archetype". That is a DIFFERENT, correct refusal (compose places only topics
    // whose archetype it can resolve, which needs a frame), and asserting through
    // it would have made this test pass on nothing. Driving from a thread is what
    // makes the oracle the one the reported failure actually asks for: after the re-attach,
    // FRAMES FLOW — proven by compose resolving `Scalars` from a real frame that
    // crossed the wire, not by an `ok` with no data behind it.
    let stop = Arc::new(AtomicBool::new(false));
    let driver = {
        let stop = Arc::clone(&stop);
        let p_transport = Arc::clone(&p_transport);
        let topic = topic.to_string();
        std::thread::spawn(move || {
            let mut gateway = gateway;
            let mut publisher = p_transport
                .create_publisher(&topic, MaxSliceLen::const_new(1 << 16), 0)
                .expect("machine-A producer attaches");
            let mut seq: u32 = 0;
            while !stop.load(Ordering::Relaxed) {
                let _ = publisher.publish_raw(&build_vector3_frame(
                    seq,
                    1_000 * (seq as u64 + 1),
                    [seq as f64, 2.0, 3.0],
                ));
                seq = seq.wrapping_add(1);
                let _ = gateway.drive_once();
                std::thread::sleep(Duration::from_millis(10));
            }
        })
    };

    let b_mgr = isolated_transport_connecting_to("reattach_vizd", port);
    let (mut netd, netd_sock, netd_dir) = start_in_process_netd("redetach", Arc::clone(&b_mgr));
    let (worker, _flush, _storage) = memory_worker("redetach");
    let (socket, dir) = temp_socket("redetach");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&b_mgr),
        worker,
        builtin_walker(),
        None,
        Arc::new(NetdDemandPlane::with_socket(netd_sock)),
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // Establish the mirror the way the sidebar does (an explicit robot), so the
    // cycle starts from the real state the agent was in.
    let a1 = attach_remote_until_ok(&mut client, 1, topic, robot);
    assert_eq!(a1["ok"].as_bool(), Some(true), "first remote attach: {a1}");

    // ── ARM 1: compose_layout, the verb an agent drives. ─────────────────
    let d1 = client.request(&format!(
        r#"{{"id":2,"method":"detach","topic":"{topic}"}}"#
    ));
    assert_eq!(d1["detached"].as_bool(), Some(true), "detach: {d1}");
    // The detach really did free the local service — otherwise the re-attach below
    // would be re-using a mirror that never went away, and would prove nothing.
    assert!(
        wait_until(Duration::from_secs(10), || b_mgr
            .data_service_missing(topic)),
        "precondition: the refcount-0 teardown frees the mirror's local SHM service"
    );

    let c1 = client.request(&format!(
        r#"{{"id":3,"method":"compose_layout","intent":{{"topics":["{topic}"]}}}}"#
    ));
    assert_eq!(
        c1["ok"].as_bool(),
        Some(true),
        "THE reported failure: compose_layout must re-attach a detached REMOTE topic \
         through the demand plane, not die DoesNotExist against a local tap: {c1}"
    );
    let listed = client.request(r#"{"id":4,"method":"list"}"#);
    let row = entry_for(&listed["attached"], "topic", topic)
        .expect("the compose-attached tap is really HELD, not just answered ok");
    assert_eq!(
        row["schema"].as_str(),
        Some("geometry_msgs/Vector3"),
        "and FRAMES FLOW through it — the schema resolved from a frame that really \
         crossed the wire after the re-attach: {row}"
    );

    // ── ARM 2: the `attach` verb with NO robot, same cycle. ─────────────────
    let d2 = client.request(&format!(
        r#"{{"id":5,"method":"detach","topic":"{topic}"}}"#
    ));
    assert_eq!(d2["detached"].as_bool(), Some(true), "second detach: {d2}");
    assert!(
        wait_until(Duration::from_secs(10), || b_mgr
            .data_service_missing(topic)),
        "precondition: the second teardown frees the service too"
    );

    let a2 = client.request(&format!(
        r#"{{"id":6,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(
        a2["ok"].as_bool(),
        Some(true),
        "a no-robot re-attach resolves where the topic lives instead of erroring \
         locally: {a2}"
    );
    assert_eq!(
        a2["already_attached"].as_bool(),
        Some(false),
        "it opens a FRESH tap over a re-demanded mirror: {a2}"
    );
    let listed2 = client.request(r#"{"id":7,"method":"list"}"#);
    assert!(
        entry_for(&listed2["attached"], "topic", topic).is_some(),
        "the re-attached tap is really HELD: {listed2}"
    );

    daemon.shutdown();
    netd.shutdown();
    stop.store(true, Ordering::Relaxed);
    let _ = driver.join();
    let _ = std::fs::remove_dir_all(&dir);
    let _ = std::fs::remove_dir_all(&netd_dir);
}

// ── Test 5e (remote-arm completion) — the HEADLINE loopback e2e: a ───────────
// SCHEMA-LESS remote attach against a real gateway serving schemas resolves the
// type from the robot's CATALOG, fetches + seeds the closure for a CUSTOM type,
// declares ingress, and taps re-injected frames — the "no schema pin!"
// bar. Machine A = producer P + gateway G (schema serving) over a real 127.0.0.1
// TCP hop; machine B = the vizd DAEMON on a distinct SHM root, driven via the
// control socket. Hand oracles (never a self-compare).

/// An ephemeral loopback port, probed + released (the standard bind-ladder — the
/// caller hands the number to the gateway's listen endpoint; a probe→bind race is
/// absorbed by the whole-setup retry in the test).
fn probe_ephemeral_port() -> u16 {
    std::net::TcpListener::bind("127.0.0.1:0")
        .expect("probe an ephemeral port")
        .local_addr()
        .expect("probe addr")
        .port()
}

/// Boot machine A: a network-free producer P + a gateway G booted via
/// `new_with_schema_serving` (so it serves the catalog + schema surface for
/// `serving`), sharing ONE SHM root, G listening on a probed port. A bounded retry
/// absorbs the probe→bind port race. Returns `(P, clock, gateway, port, root)`.
fn boot_serving_gateway(
    tag: &str,
    robot: &str,
    plan: GatewayPlan,
    serving: SchemaServing,
) -> (
    Arc<TransportManager>,
    Arc<VirtualClock>,
    GatewayRuntime,
    u16,
) {
    let root = cerulion_core::testing::iceoryx_test_config();
    let clock = Arc::new(VirtualClock::new());
    let clock_dyn: Arc<dyn Clock> = clock.clone();
    let p = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("sp_{tag}"),
            clock: clock_dyn,
            ..Default::default()
        },
        root.clone(),
    )
    .expect("init producer P");
    for attempt in 0..3 {
        let port = probe_ephemeral_port();
        let g = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("sg_{tag}_{attempt}"),
                network: Some(NetworkConfig {
                    listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                    robot_identity: Some(robot.to_string()),
                    ..NetworkConfig::default()
                }),
                ..Default::default()
            },
            root.clone(),
        )
        .expect("init gateway G");
        match GatewayRuntime::new_with_schema_serving(g, plan.clone(), serving.clone()) {
            Ok(gateway) => return (p, clock, gateway, port),
            Err(e) => {
                eprintln!("attempt {attempt}: serving gateway boot failed (port {port}): {e}")
            }
        }
    }
    panic!("could not boot the serving gateway in 3 attempts");
}

/// Retry a schema-less remote attach until the daemon's session connects to G and
/// the catalog resolves (the first GET can precede the TCP handshake). Bounded.
fn attach_remote_until_ok(client: &mut Client, id: u64, topic: &str, robot: &str) -> Value {
    let req = format!(r#"{{"id":{id},"method":"attach","topic":"{topic}","robot":"{robot}"}}"#);
    let deadline = Instant::now() + Duration::from_secs(15);
    loop {
        let resp = client.request(&req);
        if resp["ok"].as_bool() == Some(true) {
            return resp;
        }
        if Instant::now() >= deadline {
            return resp; // return the last (failing) response for the assert to show.
        }
        std::thread::sleep(Duration::from_millis(150));
    }
}

#[test]
fn schema_less_remote_attach_catalog_resolves_seeds_and_taps() {
    let _statics = blueprint_statics_guard();
    // ── The served universe: a CUSTOM type (the fetch + walker-seed + full decode
    //    loop) AND a built-in type (the catalog-resolve-without-fetch path). Both
    //    are named in the catalog so a schema-LESS attach resolves them.
    let robot = "gw";
    // A padding-free CUSTOM type (two f64) the desk NEVER compiled, so the
    // hand-built wire frame needs no alignment guesswork.
    let widget_text = "float64 x\nfloat64 y\n";
    let widget_q = "probe/Widget";
    let widget_topic = "/widget";
    let vec_topic = "/vel";
    let widget_doc = SchemaDoc {
        qualified: widget_q.to_string(),
        encoding: SchemaEncoding::Msg,
        text: widget_text.to_string(),
        deps: vec![],
    };

    // The recipe-3 wire hash of the custom type, computed INDEPENDENTLY via the
    // same seed helper the daemon uses — the producer stamps its frames with THIS,
    // so a mis-seed on the daemon side (a wrong hash) would make gateway ingress
    // REJECT every frame (hash mismatch) → zero delivery. Never a self-compare.
    let widget_hash = cerulion_viz::schema_registry::walker_with_store_and_docs(
        &[],
        std::slice::from_ref(&widget_doc),
    )
    .schema_hash_for(widget_q)
    .expect("the custom type resolves to a wire hash");
    assert_ne!(widget_hash, 0, "the custom type has a real wire hash");

    let serving = SchemaServing {
        topic_schemas: vec![
            TopicSchema {
                topic: widget_topic.to_string(),
                schema_name: widget_q.to_string(),
            },
            TopicSchema {
                topic: vec_topic.to_string(),
                schema_name: "geometry_msgs/Vector3".to_string(),
            },
        ],
        schema_docs: vec![widget_doc.clone()],
        schema_hashes: vec![],
    };
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![widget_topic.to_string(), vec_topic.to_string()],
        ingress: vec![],
    };

    // ── Machine A: producer P (real custom Widget frames) + serving gateway G.
    let (p_transport, _a_clock, mut gateway, port) =
        boot_serving_gateway("catseed", robot, plan, serving);
    let mut widget_pub = p_transport
        .create_publisher_simple(widget_topic, MaxSliceLen::const_new(256))
        .expect("P widget publisher");

    // ── Machine B: the vizd daemon on a distinct SHM root, connecting to G.
    //    An IN-PROCESS netd sharing `b_mgr` owns the ingress/mirror plane
    //    — vizd DEMANDS the topic from it; netd registers ingress on `b_mgr` (the
    //    demand token flips G's egress), re-injects the frames, and vizd taps them.
    let b_mgr = isolated_transport_connecting_to("vizd_catseed", port);
    let (mut netd, netd_sock, netd_dir) = start_in_process_netd("catseed", Arc::clone(&b_mgr));
    let (worker, _flush, _storage) = memory_worker("catseed");
    let (socket, dir) = temp_socket("catseed");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&b_mgr),
        worker,
        builtin_walker(),
        None,
        Arc::new(NetdDemandPlane::with_socket(netd_sock)),
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // ── ARM A (built-in, resolve-without-fetch): a SCHEMA-LESS attach of the
    //    built-in Vector3 topic. The catalog names `geometry_msgs/Vector3` (already
    //    known to the desk — NO fetch), and the daemon resolves the hash locally.
    let a_vec = attach_remote_until_ok(&mut client, 1, vec_topic, robot);
    assert_eq!(
        a_vec["ok"].as_bool(),
        Some(true),
        "schema-less attach of a built-in topic resolves via the catalog: {a_vec}"
    );
    assert_eq!(
        a_vec["schema"].as_str(),
        Some("geometry_msgs/Vector3"),
        "the daemon reports the catalog-resolved built-in schema (no fetch): {a_vec}"
    );

    // ── ARM B (custom, FULL LOOP — the headline): a SCHEMA-LESS attach of the
    //    CUSTOM widget topic. The daemon catalog-resolves `probe/Widget`,
    //    FETCHES its `.msg` closure over the network, SEEDS its walker, resolves the
    //    hash, and declares ingress — all with NO schema pin. Then P publishes REAL
    //    Widget frames stamped with the independently-computed hash, and the daemon
    //    INGRESSES (hash-validates == the seeded hash), TAPS, and DECODES them.
    let a_widget = attach_remote_until_ok(&mut client, 2, widget_topic, robot);
    assert_eq!(
        a_widget["ok"].as_bool(),
        Some(true),
        "schema-less attach of a CUSTOM type resolves via catalog + fetch + seed: {a_widget}"
    );
    assert_eq!(
        a_widget["schema"].as_str(),
        Some(widget_q),
        "the daemon reports the FETCHED custom schema (walker seeded): {a_widget}"
    );

    // A test-side data-only tap on the daemon's re-injected widget mirror (hand
    // oracle on the decoded fields).
    let mut probe = b_mgr
        .create_data_only_subscriber(widget_topic)
        .expect("test-side data-only tap on the re-injected custom topic");

    // Handshake: the daemon's demand token flips G's egress flag, then a drive
    // attaches the tap (before publishing — the tap has no history).
    let flag = gateway
        .manager()
        .bridge_manager()
        .register_topic(widget_topic)
        .expect("G bridge flag handle");
    let deadline = Instant::now() + Duration::from_secs(10);
    while !flag.load(Ordering::Relaxed) {
        assert!(
            Instant::now() < deadline,
            "the daemon's demand token did not flip G's egress flag within 10s"
        );
        std::thread::sleep(Duration::from_millis(20));
    }
    let attach_deadline = Instant::now() + Duration::from_secs(5);
    while gateway.active_tap_topics().is_empty() {
        assert!(
            Instant::now() < attach_deadline,
            "gateway tap did not attach"
        );
        gateway.drive_once().expect("drive attach");
        std::thread::sleep(Duration::from_millis(10));
    }

    // 3 hand-chosen CUSTOM Widget frames on P (raw, stamped with widget_hash).
    let sent: [(f64, f64); 3] = [(1.5, 2.5), (3.5, 4.5), (5.5, 6.5)];

    // PER-FRAME LOCKSTEP. Publishing all three frames and
    // THEN draining would leave up to three in flight at once and make the
    // assertion's exactness depend on the probe's queue holding every one of
    // them across an arbitrary drive/drain interleaving. That is the known
    // drain-shape class: a burst split across drives, or an eviction at a
    // shallow queue, changes what `got` accumulates without anything being wrong
    // with the seed→hash→ingress→decode loop under test. A loaded Linux runner
    // fails exactly here.
    //
    // Publishing ONE frame and awaiting THAT frame before the next keeps at most
    // one in flight, so no interleaving can drop or reorder anything — the
    // pattern `network_tf_e2e_test.rs` arm 1 documents for this same reason
    // ("publish one → bounded-await THAT frame → publish next", never the
    // drain-all whose keep-one capture collapses a queue to its newest frame).
    //
    // The ORACLE IS STRICT: all three exact (x, y) pairs, in order,
    // decoded from the full wire frame. Only the delivery shape is made
    // deterministic.
    let mut got: Vec<(f64, f64)> = Vec::new();
    let mut scratch = Vec::new();
    for (i, &(x, y)) in sent.iter().enumerate() {
        widget_pub
            .publish_raw(&build_widget_frame(
                widget_hash,
                i as u32,
                100 + i as u64,
                x,
                y,
            ))
            .expect("publish_raw widget frame");

        let deadline = Instant::now() + Duration::from_secs(15);
        while got.len() <= i && Instant::now() < deadline {
            gateway.drive_once().expect("drive forward");
            scratch.clear();
            // ONE frame per drain: the burst is one deep by construction.
            if let Ok(n) = probe.drain_owned(1, &mut scratch) {
                for frame in scratch.iter().take(n) {
                    // `payload()` on a drained owned frame is the FULL wire frame
                    // (32-byte WireHeader + data); the custom fields begin after it.
                    let full = frame.payload();
                    let base = WireHeader::SIZE;
                    if full.len() >= base + 16 {
                        let x = f64::from_le_bytes(full[base..base + 8].try_into().unwrap());
                        let y = f64::from_le_bytes(full[base + 8..base + 16].try_into().unwrap());
                        got.push((x, y));
                    }
                }
            }
            if got.len() <= i {
                std::thread::sleep(Duration::from_millis(20));
            }
        }
    }
    assert_eq!(
        got,
        sent.to_vec(),
        "the re-injected CUSTOM frames decode to the hand-chosen values — the \
         seed→hash→ingress→decode loop is closed byte-faithful (delivery itself \
         proves the seeded hash matched the producer's, or ingress would drop them)"
    );

    // The daemon itself TAPPED + DECODED the custom frames: `status` shows frames > 0
    // AND the resolved CUSTOM schema (its poll thread walked a frame via the SEEDED
    // walker — impossible without the fetch+seed).
    let mut daemon_saw = false;
    let deadline = Instant::now() + Duration::from_secs(5);
    while !daemon_saw && Instant::now() < deadline {
        let status = client.request(r#"{"id":9,"method":"status"}"#);
        if let Some(topics) = status["topics"].as_array() {
            if let Some(t) = topics
                .iter()
                .find(|t| t["topic"].as_str() == Some(widget_topic))
            {
                if t["frames"].as_u64().unwrap_or(0) > 0 {
                    assert_eq!(
                        t["schema"].as_str(),
                        Some(widget_q),
                        "the daemon decoded the tapped CUSTOM schema via the seeded walker: {status}"
                    );
                    daemon_saw = true;
                }
            }
        }
        if !daemon_saw {
            std::thread::sleep(Duration::from_millis(50));
        }
    }
    assert!(
        daemon_saw,
        "the daemon's poll thread must tap + DECODE the re-injected CUSTOM frames (frames_seen > 0)"
    );

    daemon.shutdown();
    netd.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
    let _ = std::fs::remove_dir_all(&netd_dir);
}

/// The NETD-LOAD-BEARING resolve DISCRIMINATOR. In
/// `schema_less_remote_attach_catalog_resolves_seeds_and_taps` netd AND vizd's
/// transient fallback share ONE networked manager, so a mutation skipping netd
/// (routing the resolve straight to the transient session) STILL passes — that test
/// proves resolve-succeeds, not served-by-netd. Here netd owns the ONLY session that
/// can reach the serving gateway G, while the vizd daemon's OWN manager reaches
/// NOBODY (scouting off, no connect). So vizd's transient fallback CANNOT resolve — a
/// successful schema-less attach PROVES netd served the resolve.
///
/// Reverting `catalog_resolve_type` (ARM A) or `fetch_and_seed_type` (ARM B)
/// to skip netd routes the resolve through vizd's reach-nobody transient session → it
/// gathers nothing → the attach FAILS → this test fails. Both managers share ONE SHM
/// root so netd's re-injected mirror stays tappable (attach `ok:true` needs the local
/// mirror service, which netd creates on the shared root — no frames required).
#[test]
fn schema_less_resolve_is_served_by_netd_not_vizds_own_network() {
    let _statics = blueprint_statics_guard();
    let robot = "netdgw";
    let widget_text = "float64 x\nfloat64 y\n";
    let widget_q = "netd_probe/Widget";
    let widget_topic = "/widget";
    let vec_topic = "/vel";
    let widget_doc = SchemaDoc {
        qualified: widget_q.to_string(),
        encoding: SchemaEncoding::Msg,
        text: widget_text.to_string(),
        deps: vec![],
    };
    let serving = SchemaServing {
        topic_schemas: vec![
            TopicSchema {
                topic: widget_topic.to_string(),
                schema_name: widget_q.to_string(),
            },
            TopicSchema {
                topic: vec_topic.to_string(),
                schema_name: "geometry_msgs/Vector3".to_string(),
            },
        ],
        schema_docs: vec![widget_doc.clone()],
        schema_hashes: vec![],
    };
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![widget_topic.to_string(), vec_topic.to_string()],
        ingress: vec![],
    };

    // ── Machine A: producer P + serving gateway G (its queryable answers the
    //    catalog/schema GETs; kept ALIVE for the whole test via the bindings).
    let (_p_transport, _a_clock, _gateway, port) =
        boot_serving_gateway("netdload", robot, plan, serving);

    // ── Machine B (ONE shared root): netd_mgr reaches G; vizd_mgr reaches NOBODY.
    let (netd_mgr, vizd_mgr) = netd_and_vizd_on_one_root("netdload", port);
    let (mut netd, netd_sock, netd_dir) = start_in_process_netd("netdload", Arc::clone(&netd_mgr));
    let (worker, _flush, _storage) = memory_worker("netdload");
    let (socket, dir) = temp_socket("netdload");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&vizd_mgr),
        worker,
        builtin_walker(),
        None,
        Arc::new(NetdDemandPlane::with_socket(netd_sock)),
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // ── ARM A (built-in, CATALOG resolve): the ONLY way to resolve
    //    `geometry_msgs/Vector3`'s NAME is netd's query plane — vizd's transient
    //    session reaches nobody. A `catalog_resolve_type` skip-netd mutation fails here.
    let a_vec = attach_remote_until_ok(&mut client, 1, vec_topic, robot);
    assert_eq!(
        a_vec["ok"].as_bool(),
        Some(true),
        "netd's query plane served the schema-less BUILT-IN resolve — vizd's own network \
         reaches nobody, so nothing else could: {a_vec}"
    );
    assert_eq!(
        a_vec["schema"].as_str(),
        Some("geometry_msgs/Vector3"),
        "the netd-resolved built-in schema: {a_vec}"
    );

    // ── ARM B (custom, CATALOG resolve + schema FETCH + seed): the ONLY way to
    //    resolve AND fetch `netd_probe/Widget`'s `.msg` closure is netd's query
    //    plane. A `fetch_and_seed_type` skip-netd mutation fails here.
    let a_widget = attach_remote_until_ok(&mut client, 2, widget_topic, robot);
    assert_eq!(
        a_widget["ok"].as_bool(),
        Some(true),
        "netd's query plane served the schema-less CUSTOM resolve + fetch — vizd's own \
         network reaches nobody, so nothing else could: {a_widget}"
    );
    assert_eq!(
        a_widget["schema"].as_str(),
        Some(widget_q),
        "the netd-fetched custom schema (walker seeded via netd): {a_widget}"
    );

    daemon.shutdown();
    netd.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
    let _ = std::fs::remove_dir_all(&netd_dir);
}

// ── Test 6 — malformed + unknown-method lines → structured errors, no drop ───

#[test]
fn malformed_and_unknown_method_return_structured_errors_without_disconnect() {
    let mgr = isolated_transport("vizd_malformed");
    let (worker, _flush, _storage) = memory_worker("malformed");
    let (socket, dir) = temp_socket("malformed");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None, // rerun_url: a memory sink hosts no endpoint
    )
    .expect("daemon starts");

    let mut client = Client::connect(&socket);

    // Garbage line → structured error (never a panic / disconnect).
    let bad = client.request("this is not json");
    assert_eq!(bad["ok"].as_bool(), Some(false));
    assert!(bad["error"].as_str().unwrap().contains("malformed"));

    // Unknown method → structured error carrying the recovered id.
    let unknown = client.request(r#"{"id":42,"method":"teleport"}"#);
    assert_eq!(unknown["ok"].as_bool(), Some(false));
    assert_eq!(unknown["id"].as_u64(), Some(42));

    // The connection SURVIVES both errors — a subsequent valid request works.
    let list = client.request(r#"{"id":43,"method":"list"}"#);
    assert_eq!(list["ok"].as_bool(), Some(true));
    assert_eq!(list["id"].as_u64(), Some(43));

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Test 7 — daemon hygiene: refuse a second daemon; pidfile lifecycle ──────

#[test]
fn second_daemon_refused_and_pidfile_lifecycle() {
    let mgr = isolated_transport("vizd_hygiene");
    let (worker, _flush, _storage) = memory_worker("hygiene");
    let (socket, dir) = temp_socket("hygiene");
    let pidfile = socket.with_extension("pid");

    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None, // rerun_url: a memory sink hosts no endpoint
    )
    .expect("first daemon starts");
    assert!(socket.exists(), "socket bound");
    assert_eq!(
        std::fs::read_to_string(&pidfile).unwrap().trim(),
        std::process::id().to_string(),
        "pidfile carries the daemon pid"
    );

    // A SECOND daemon on the SAME live socket is REFUSED (never clobbers). It
    // needs its own worker + walker (a second isolated manager for the worker's
    // transport is unnecessary — start fails at acquire_socket before touching
    // the transport).
    let (worker2, _f2, _s2) = memory_worker("hygiene2");
    let err = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker2,
        builtin_walker(),
        None, // rerun_url: a memory sink hosts no endpoint
    )
    .expect_err("second daemon refused");
    assert!(
        err.to_string().contains("already running"),
        "the refusal names the running daemon: {err}"
    );

    // Clean shutdown removes BOTH the socket + the pidfile.
    daemon.shutdown();
    assert!(!socket.exists(), "socket removed on shutdown");
    assert!(!pidfile.exists(), "pidfile removed on shutdown");
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Test 8 — daemon recovers a stale socket left by a crash ─────────────────

#[test]
fn daemon_recovers_a_stale_socket() {
    let mgr = isolated_transport("vizd_stale");
    let (socket, dir) = temp_socket("stale");
    // A crash-left stale socket: the file exists but nothing listens.
    {
        let l = std::os::unix::net::UnixListener::bind(&socket).expect("bind stale");
        drop(l);
    }
    assert!(socket.exists(), "stale socket file present");

    let (worker, _flush, _storage) = memory_worker("stale");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None, // rerun_url: a memory sink hosts no endpoint
    )
    .expect("daemon recovers the stale socket and starts");

    // The recovered socket is live — a controller connects + gets the banner.
    let client = Client::connect(&socket);
    assert_eq!(client.banner["protocol"].as_u64(), Some(1));

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Test 9 — silent topic attaches null, then back-fills ────────

#[test]
fn silent_topic_attaches_null_then_backfills_on_first_frame() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("vizd_silent");
    let topic = "/vizd/silent";
    // The topic EXISTS (a publisher attaches) but is SILENT — no frames yet.
    let mut publisher = mgr
        .create_publisher(topic, MaxSliceLen::const_new(1 << 16), 0)
        .expect("silent producer attaches");

    let (worker, _flush, _storage) = memory_worker("silent");
    let (socket, dir) = temp_socket("silent");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None, // rerun_url: a memory sink hosts no endpoint
    )
    .expect("daemon starts");

    let mut client = Client::connect(&socket);
    // attach a real-but-SILENT topic → OK with schema/archetype NULL (a data-only
    // tap has no history, so there is nothing to resolve — NO fabricated schema).
    let att = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(
        att["ok"].as_bool(),
        Some(true),
        "silent topic attaches: {att}"
    );
    assert!(
        att["schema"].is_null(),
        "no fabricated schema on a silent topic: {att}"
    );
    assert!(
        att["archetype"].is_null(),
        "no fabricated archetype on a silent topic: {att}"
    );
    // An unresolved archetype carries NULL components/view_kinds
    // (NOT `[]`) — `null` means "not yet resolved", distinct from a `[]` that would
    // read as "resolved: renders nothing". Matches the schema/archetype convention.
    assert!(
        att["components"].is_null(),
        "no fabricated components on a silent topic: {att}"
    );
    assert!(
        att["view_kinds"].is_null(),
        "no fabricated view_kinds on a silent topic: {att}"
    );

    // NOW the producer publishes → the poll thread drains + BACK-FILLS the schema
    // from the first decodable frame.
    for s in 0..8u32 {
        let frame = build_vector3_frame(
            s,
            PUBLISH_STAMP_STEP_NS * (s as u64 + 1),
            [s as f64, 2.0, 3.0],
        );
        publisher.publish_raw(&frame).expect("publish");
        std::thread::sleep(Duration::from_millis(10));
    }

    // A later `status` shows the RESOLVED schema (the back-fill).
    assert!(
        wait_until(Duration::from_secs(3), || {
            let status = client.request(r#"{"id":2,"method":"status"}"#);
            entry_for(&status["topics"], "topic", topic).and_then(|st| st["schema"].as_str())
                == Some("geometry_msgs/Vector3")
        }),
        "the silent topic's schema back-fills once frames flow"
    );
    // `list` shows the resolved placement too — now WITH the layout metadata that
    // back-filled alongside the schema (null → resolved).
    let list = client.request(r#"{"id":3,"method":"list"}"#);
    let le = entry_for(&list["attached"], "topic", topic).expect("attached listed");
    assert_eq!(le["schema"].as_str(), Some("geometry_msgs/Vector3"));
    assert_eq!(le["archetype"].as_str(), Some("Scalars"));
    assert_eq!(le["components"], serde_json::json!(["Scalars"]));
    assert_eq!(le["view_kinds"], serde_json::json!(["time_series"]));

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Side-loaded schema definitions make a LOCAL topic decodable ─────────────

/// THE side-loading viewer half: a purely LOCAL topic carrying a type the daemon
/// never compiled renders nothing, and the `schemas` verb fixes that at runtime.
///
/// Why this could not already happen: the daemon's walker is built ONCE at boot
/// from the built-in corpus plus `DDS_BRIDGE_CONFIG`'s `msg_dirs`, and the only
/// runtime path that grew it was a REMOTE attach's schema fetch. A bag player
/// republishes onto LOCAL SHM, so no remote attach ever runs and the definitions
/// the bag carries had nowhere to go — the topic attached `ok: true` and
/// rendered an empty scene.
///
/// Hand oracles, never a self-compare: the type name and its text are chosen
/// here; the wire hash the producer stamps is computed independently through the
/// registry helper (the value a real producer would put on the wire), and the
/// daemon must arrive at the SAME name from the frame alone.
#[test]
fn side_loaded_schemas_make_a_local_custom_topic_decodable() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("vizd_seed");
    let topic = "/widget";
    // A padding-free CUSTOM type (two f64) no desk compiles, so the hand-built
    // frame needs no alignment guesswork.
    let widget_q = "offer_probe/Widget";
    let widget_doc = SchemaDoc {
        qualified: widget_q.to_string(),
        encoding: SchemaEncoding::Msg,
        text: "float64 x\nfloat64 y\n".to_string(),
        deps: vec![],
    };
    let widget_hash = cerulion_viz::schema_registry::walker_with_store_and_docs(
        &[],
        std::slice::from_ref(&widget_doc),
    )
    .schema_hash_for(widget_q)
    .expect("the custom type resolves to a wire hash");

    let mut publisher = mgr
        .create_publisher(topic, MaxSliceLen::const_new(1 << 16), 0)
        .expect("producer attaches");

    let (worker, _flush, _storage) = memory_worker("seed");
    let (socket, dir) = temp_socket("seed");
    // A BUILTINS-ONLY walker: this daemon has never heard of the custom type.
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    let att = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(att["ok"].as_bool(), Some(true), "local attach: {att}");

    // Publish real custom frames. The daemon taps them and can resolve NOTHING —
    // the success-shaped failure this test exists for.
    let mut publish = |n: u32| {
        for s in 0..n {
            let frame = build_widget_frame(widget_hash, s, 1_000 * (s as u64 + 1), 1.5, 2.5);
            publisher.publish_raw(&frame).expect("publish");
            std::thread::sleep(Duration::from_millis(5));
        }
    };
    publish(8);
    let status = client.request(r#"{"id":2,"method":"status"}"#);
    let before = entry_for(&status["topics"], "topic", topic)
        .map(|st| st["schema"].clone())
        .unwrap_or(Value::Null);
    assert!(
        before.is_null(),
        "PRECONDITION: a daemon that never compiled this type must resolve nothing \
         — otherwise the seed below proves nothing: {status}"
    );

    // ── THE VERB: side-load the definition, exactly as `bag play` does with the
    //    definitions its bag carries.
    let seed = client.request(&format!(
        r#"{{"id":3,"method":"schemas","docs":[{}]}}"#,
        serde_json::to_string(&widget_doc).unwrap()
    ));
    assert_eq!(seed["ok"].as_bool(), Some(true), "seed accepted: {seed}");
    assert_eq!(seed["offered"].as_u64(), Some(1), "{seed}");
    assert_eq!(
        seed["accepted"].as_u64(),
        Some(1),
        "the daemon had never seen this type: {seed}"
    );
    assert_eq!(seed["known"].as_u64(), Some(1), "{seed}");

    // The SAME frames now resolve — the daemon arrives at the name from the wire
    // hash alone, through the definition it was just handed.
    publish(8);
    assert!(
        wait_until(Duration::from_secs(5), || {
            let status = client.request(r#"{"id":4,"method":"status"}"#);
            entry_for(&status["topics"], "topic", topic).and_then(|st| st["schema"].as_str())
                == Some(widget_q)
        }),
        "after the side-load the custom type must resolve: {}",
        client.request(r#"{"id":5,"method":"status"}"#)
    );

    // ── IDEMPOTENCE: re-offering the SAME definition changes nothing. The bag
    //    player re-offers on a timer, so a rebuild per offer would re-parse the
    //    whole built-in corpus every couple of seconds for the life of a playback.
    let again = client.request(&format!(
        r#"{{"id":6,"method":"schemas","docs":[{}]}}"#,
        serde_json::to_string(&widget_doc).unwrap()
    ));
    assert_eq!(again["ok"].as_bool(), Some(true), "{again}");
    assert_eq!(
        again["accepted"].as_u64(),
        Some(0),
        "a byte-identical re-offer must be a no-op: {again}"
    );
    assert_eq!(again["known"].as_u64(), Some(1), "{again}");

    // ── An UPDATED definition for the same name IS accepted (a redeployed robot,
    //    a bag recorded against another version) — silently keeping the stale one
    //    would decode frames against a definition their producer no longer uses.
    let updated = SchemaDoc {
        text: "float64 x\nfloat64 y\nfloat64 z\n".to_string(),
        ..widget_doc.clone()
    };
    let upd = client.request(&format!(
        r#"{{"id":7,"method":"schemas","docs":[{}]}}"#,
        serde_json::to_string(&updated).unwrap()
    ));
    assert_eq!(
        upd["replaced"].as_array().map(|a| a.len()),
        Some(1),
        "a CHANGED definition for a held name is a REPLACEMENT, not a duplicate: {upd}"
    );
    assert_eq!(
        upd["replaced"][0].as_str(),
        Some(widget_q),
        "the replacement names the type whose definition just changed: {upd}"
    );
    assert_eq!(
        upd["accepted"].as_u64(),
        Some(0),
        "a replacement is not a GAIN — that type was already decodable, and whatever \
         produces the PREVIOUS definition just went dark, so it must not be reported \
         as something newly learned: {upd}"
    );
    assert_eq!(
        upd["known"].as_u64(),
        Some(1),
        "updating a held name does not grow the set: {upd}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

/// `accepted` counts what the WALKER took, and a definition for a BUILT-IN is
/// refused outright.
///
/// Both are the same class of defect as the one side-loading closes, one layer up: a daemon
/// that reported `accepted: 1` for a doc it then warn-skipped would have the
/// player log "those types are now decodable" over a scene that renders nothing.
/// And a side-load applies daemon-WIDE — docs are folded in last and the layout
/// resolver is last-insert-wins — so accepting a caller-supplied
/// `geometry_msgs/Vector3` would change how every topic of that type is decoded,
/// including live taps the caller knows nothing about.
///
/// # Which fields can see the refusal
///
/// `accepted` and `ok` CANNOT: a step (4) that asserts only
/// those two stays green with the `is_builtin_schema_name` partition deleted.
/// `before` is snapshotted BEFORE the partition and already knows every
/// built-in, so `accepted = learned.filter(|q| !before.knows(q))` is 0 for an
/// ACCEPTED impostor exactly as for a refused one; and `ok` is a struct literal.
/// The two fields the partition moves are `rejected` (the impostor's name vs
/// nothing) and `known` (the accumulated set grows vs does not), so step (4)
/// asserts BOTH — and the live-topic probe below publishes the SAME type the
/// impostor names, because `layouts` is keyed by qualified name and a probe of
/// any OTHER type is blind to the redefinition by construction.
#[test]
fn accepted_counts_what_the_walker_took_and_builtins_are_refused() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("vizd_accepted");
    let (worker, _flush, _storage) = memory_worker("accepted");
    let (socket, dir) = temp_socket("accepted");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    let offer = |client: &mut Client, id: u64, doc: &SchemaDoc| -> Value {
        client.request(&format!(
            r#"{{"id":{id},"method":"schemas","docs":[{}]}}"#,
            serde_json::to_string(doc).unwrap()
        ))
    };

    // (1) A YAML-encoded doc: the daemon carries no workspace-YAML parser, so
    //     the walker warn-SKIPS it. It must NOT be reported as accepted.
    let yaml_doc = SchemaDoc {
        qualified: "offer_probe/YamlType".to_string(),
        encoding: SchemaEncoding::Yaml,
        text: "name: YamlType\nfields: []\n".to_string(),
        deps: vec![],
    };
    let r = offer(&mut client, 1, &yaml_doc);
    assert_eq!(r["ok"].as_bool(), Some(true), "{r}");
    assert_eq!(
        r["accepted"].as_u64(),
        Some(0),
        "a doc the walker discarded is not 'accepted': {r}"
    );

    // (2) An UNPARSEABLE `.msg`: same rule, different reason.
    //
    // `@@@` is a token `parse_rosmsg` genuinely REFUSES ("expected 'type name'").
    // Prose like "this is not a valid message definition" parses FINE — as a
    // field of an unknown nested type — which is why this fixture is a hostile
    // TOKEN rather than a hostile sentence. (Measured: the prose version made
    // this assertion pass for the wrong reason.)
    let broken = SchemaDoc {
        qualified: "offer_probe/Broken".to_string(),
        encoding: SchemaEncoding::Msg,
        text: "@@@\n".to_string(),
        deps: vec![],
    };
    let r = offer(&mut client, 2, &broken);
    assert_eq!(
        r["accepted"].as_u64(),
        Some(0),
        "a doc that failed to parse is not 'accepted': {r}"
    );

    // (3) ANTI-TAUTOLOGY: a GOOD doc through the identical path IS accepted, so
    //     the two zeroes above are about those docs and not about the counter
    //     being stuck at zero.
    let good = SchemaDoc {
        qualified: "offer_probe/Good".to_string(),
        encoding: SchemaEncoding::Msg,
        text: "float64 x\nfloat64 y\n".to_string(),
        deps: vec![],
    };
    let r = offer(&mut client, 3, &good);
    assert_eq!(
        r["accepted"].as_u64(),
        Some(1),
        "a parseable .msg doc must be accepted: {r}"
    );

    // (4) A BUILT-IN redefinition is REFUSED — never folded in, never counted.
    //
    // The impostor names `geometry_msgs/Vector3` — the SAME built-in the live
    // probe below publishes — deliberately. `FrameWalker`'s `layouts` are keyed
    // by qualified name, so an impostor naming some OTHER built-in cannot touch
    // Vector3's hash and the probe would pass whether or not the guard fired.
    // (Measured with an impostor naming `sensor_msgs/Image`: with the
    // guard DELETED, the real Image hash resolved to `None` daemon-wide while
    // Vector3's hash stayed byte-identical — the probe saw nothing.)
    //
    // `int32 not_a_vector` is a text `parse_rosmsg` genuinely ACCEPTS, so under
    // a deleted guard it really is folded in and really does overwrite the
    // built-in layout. A text that failed to parse would be caught by the
    // walker-refusal arm instead and would pin the wrong guard.
    let impostor = SchemaDoc {
        qualified: "geometry_msgs/Vector3".to_string(),
        encoding: SchemaEncoding::Msg,
        text: "int32 not_a_vector\n".to_string(),
        deps: vec![],
    };
    let r = offer(&mut client, 4, &impostor);
    assert_eq!(
        r["ok"].as_bool(),
        Some(true),
        "the refusal is not an error: {r}"
    );
    assert_eq!(
        r["accepted"].as_u64(),
        Some(0),
        "a definition for a BUILT-IN must be refused: {r}"
    );
    // THE mutation kill on the partition itself. Neither assertion above can see
    // it (see this test's doc comment): `accepted` zeroes for an accepted
    // impostor too, and `ok` is a struct literal. These two fields are the only
    // ones the partition moves.
    let rejected: Vec<&str> = r["rejected"]
        .as_array()
        .map(|a| a.iter().filter_map(Value::as_str).collect())
        .unwrap_or_default();
    assert_eq!(
        rejected,
        vec!["geometry_msgs/Vector3"],
        "a refused built-in must be REPORTED as rejected, or the caller cannot tell a refusal \
         from 'you already had it': {r}"
    );
    assert_eq!(
        r["known"].as_u64(),
        Some(1),
        "a refused doc never enters the accumulated set — step (3)'s `Good` is still its only \
         member (1, 2 = the impostor was folded in): {r}"
    );
    // …and the daemon's own view of that built-in is untouched: a live topic of
    // THAT SAME type still resolves to the REAL definition. `resolve_from_frame`
    // needs `schema_name_for_hash(REAL_VECTOR3_HASH)` to answer, and a folded-in
    // impostor rebuilds `hash_index` off the overwritten layout, so the real
    // hash would resolve to nothing and `status.schema` would be null.
    let topic = "/vec";
    let mut publisher = mgr
        .create_publisher(topic, MaxSliceLen::const_new(1 << 16), 0)
        .expect("producer");
    let att = client.request(&format!(
        r#"{{"id":5,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(att["ok"].as_bool(), Some(true), "{att}");
    for s in 0..8u32 {
        publisher
            .publish_raw(&build_vector3_frame(
                s,
                1_000 * (s as u64 + 1),
                [1.0, 2.0, 3.0],
            ))
            .expect("publish");
        std::thread::sleep(Duration::from_millis(5));
    }
    assert!(
        wait_until(Duration::from_secs(5), || {
            let status = client.request(r#"{"id":6,"method":"status"}"#);
            entry_for(&status["topics"], "topic", topic).and_then(|st| st["schema"].as_str())
                == Some("geometry_msgs/Vector3")
        }),
        "a refused side-load must leave the built-in corpus intact: {}",
        client.request(r#"{"id":7,"method":"status"}"#)
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

/// A malformed re-offer under an already-held name
/// must not destroy the definition it names.
///
/// If `seed_walker_with_docs` COMMITS the replacement into `seed.docs` before the
/// rebuilt walker has a chance to refuse it, `absorb` overwrites the good text, the
/// rebuild warn-SKIPS the unparseable replacement (so `layouts` loses the entry),
/// that walker is INSTALLED daemon-wide, and the `schemas` verb's `forget` then
/// removes the only copy. A side-load applies to every consumer, so one bad doc
/// takes a working type dark for the whole daemon — including live robot taps that
/// have nothing to do with the caller.
///
/// Worse on the OTHER caller: the remote-attach path
/// (`seed_and_resolve_schema`) discards its `Absorbed` and never forgets anything,
/// so a robot serving one bad doc would leave the poisoned definition in `seed.docs`
/// PERMANENTLY. That is why the guard lives inside `seed_walker_with_docs` rather
/// than at the verb.
///
/// The oracle is BEHAVIORAL on the same type throughout: real frames stamped with
/// the good definition's wire hash must keep resolving across the refusal. A
/// response-field assertion alone could not see it — `accepted` is 0 either way
/// (the name was already known), and `known` counts docs, not whether the one it
/// holds still parses.
#[test]
fn a_malformed_re_offer_never_destroys_the_definition_it_replaces() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("vizd_rollback");
    let qualified = "offer_rollback/Widget";

    // Two PARSEABLE definitions of one name, both two padding-free f64 so the
    // hand-built frame fits either layout, with DIFFERENT field names so their
    // wire hashes differ — that difference is what makes the recovery arm
    // observable rather than a self-compare.
    let doc_of = |text: &str| SchemaDoc {
        qualified: qualified.to_string(),
        encoding: SchemaEncoding::Msg,
        text: text.to_string(),
        deps: vec![],
    };
    let good = doc_of("float64 x\nfloat64 y\n");
    let corrected = doc_of("float64 a\nfloat64 b\n");
    // `@@@` is a token `parse_rosmsg` genuinely REFUSES (prose parses fine, as a
    // field of an unknown nested type — measured in the sibling arm).
    let malformed = doc_of("@@@\n");

    let hash_of = |d: &SchemaDoc| {
        cerulion_viz::schema_registry::walker_with_store_and_docs(&[], std::slice::from_ref(d))
            .schema_hash_for(qualified)
            .expect("a parseable custom doc resolves to a wire hash")
    };
    let good_hash = hash_of(&good);
    let corrected_hash = hash_of(&corrected);
    assert_ne!(
        good_hash, corrected_hash,
        "PRECONDITION: the two definitions must differ on the wire, or the \
         recovery arm cannot tell which one is installed"
    );

    let good_topic = "/rollback/good";
    // A SECOND topic carrying the SAME good-hash frames, attached only AFTER the
    // refusal. `TopicStat::resolved` is a per-topic ONE-SHOT (`if
    // stat.resolved.is_none()` in the poll thread), so re-probing `good_topic`
    // after the refusal re-reads a CACHE and is blind to which walker is
    // installed — a variant that installs the unvalidated candidate
    // walker still passes that probe. A topic whose first frame arrives after the
    // refusal resolves through the walker that is actually live.
    let after_topic = "/rollback/after";
    let corrected_topic = "/rollback/corrected";
    let mut good_pub = mgr
        .create_publisher(good_topic, MaxSliceLen::const_new(1 << 16), 0)
        .expect("good producer attaches");
    let mut after_pub = mgr
        .create_publisher(after_topic, MaxSliceLen::const_new(1 << 16), 0)
        .expect("after producer attaches");
    let mut corrected_pub = mgr
        .create_publisher(corrected_topic, MaxSliceLen::const_new(1 << 16), 0)
        .expect("corrected producer attaches");

    let (worker, _flush, _storage) = memory_worker("rollback");
    let (socket, dir) = temp_socket("rollback");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    for (id, topic) in [(1u64, good_topic), (2, corrected_topic)] {
        let att = client.request(&format!(
            r#"{{"id":{id},"method":"attach","topic":"{topic}"}}"#
        ));
        assert_eq!(att["ok"].as_bool(), Some(true), "attach {topic}: {att}");
    }

    let publish = |p: &mut cerulion_core::CerulionPublisher, hash: u64| {
        for s in 0..8u32 {
            p.publish_raw(&build_widget_frame(
                hash,
                s,
                1_000 * (s as u64 + 1),
                1.5,
                2.5,
            ))
            .expect("publish");
            std::thread::sleep(Duration::from_millis(5));
        }
    };
    let schema_of = |client: &mut Client, id: u64, topic: &str| -> Option<String> {
        let status = client.request(&format!(r#"{{"id":{id},"method":"status"}}"#));
        entry_for(&status["topics"], "topic", topic)
            .and_then(|st| st["schema"].as_str())
            .map(str::to_string)
    };
    let offer = |client: &mut Client, id: u64, doc: &SchemaDoc| -> Value {
        client.request(&format!(
            r#"{{"id":{id},"method":"schemas","docs":[{}]}}"#,
            serde_json::to_string(doc).unwrap()
        ))
    };

    // (1) Seed the GOOD definition and prove it really serves.
    let r = offer(&mut client, 3, &good);
    assert_eq!(r["accepted"].as_u64(), Some(1), "the good doc is new: {r}");
    assert_eq!(r["known"].as_u64(), Some(1), "{r}");
    publish(&mut good_pub, good_hash);
    assert!(
        wait_until(Duration::from_secs(5), || schema_of(
            &mut client,
            4,
            good_topic
        )
        .as_deref()
            == Some(qualified)),
        "PRECONDITION: the good definition must resolve before we attack it"
    );

    // (2) THE DEFECT: a malformed re-offer of that SAME name.
    let r = offer(&mut client, 5, &malformed);
    assert_eq!(
        r["ok"].as_bool(),
        Some(true),
        "a refusal is not an error: {r}"
    );
    assert_eq!(
        r["accepted"].as_u64(),
        Some(0),
        "an unusable doc is never accepted: {r}"
    );
    let rejected: Vec<&str> = r["rejected"]
        .as_array()
        .map(|a| a.iter().filter_map(Value::as_str).collect())
        .unwrap_or_default();
    assert_eq!(
        rejected,
        vec![qualified],
        "the refusal must be REPORTED, not silent: {r}"
    );
    assert_eq!(
        r["known"].as_u64(),
        Some(1),
        "the daemon still holds exactly one definition for this name — the \
         INCUMBENT (0 = it was destroyed, 2 = the bad one was kept alongside): {r}"
    );

    // THE BEHAVIORAL PIN: the incumbent still SERVES — probed on a topic
    // resolving for the FIRST time, so it reads the walker installed AFTER the
    // refusal rather than a cached verdict. (`TopicStat::resolved` is a per-topic
    // one-shot; re-probing `good_topic` here would also pass a variant
    // that installs the unvalidated candidate walker.)
    let att = client.request(&format!(
        r#"{{"id":6,"method":"attach","topic":"{after_topic}"}}"#
    ));
    assert_eq!(
        att["ok"].as_bool(),
        Some(true),
        "attach {after_topic}: {att}"
    );
    assert!(
        att["schema"].is_null(),
        "PRECONDITION: this topic is silent at attach, so nothing is cached and \
         the probe below reads the LIVE walker: {att}"
    );
    publish(&mut after_pub, good_hash);
    assert!(
        wait_until(Duration::from_secs(5), || schema_of(
            &mut client,
            7,
            after_topic
        )
        .as_deref()
            == Some(qualified)),
        "a REFUSED re-offer must leave the definition it names intact — the type \
         went dark daemon-wide: {}",
        client.request(r#"{"id":8,"method":"status"}"#)
    );

    // (3) RECOVERY / the legitimate-update path: a CORRECTED definition for the
    //     same name is still absorbed, and it really takes effect — a frame under
    //     the corrected hash (which the good definition cannot explain) resolves.
    let r = offer(&mut client, 8, &corrected);
    assert_eq!(r["ok"].as_bool(), Some(true), "{r}");
    // `rejected` is `skip_serializing_if = "Vec::is_empty"`, so "no refusal" is
    // an ABSENT key, not an empty array.
    assert!(
        r["rejected"]
            .as_array()
            .is_none_or(|a: &Vec<Value>| a.is_empty()),
        "a parseable correction is not a refusal: {r}"
    );
    assert_eq!(
        r["replaced"]
            .as_array()
            .map(|a| a.iter().filter_map(Value::as_str).collect::<Vec<_>>()),
        Some(vec![qualified]),
        "correcting a held name is a REPLACEMENT, reported as one: {r}"
    );
    assert_eq!(r["known"].as_u64(), Some(1), "still one definition: {r}");
    publish(&mut corrected_pub, corrected_hash);
    assert!(
        wait_until(Duration::from_secs(5), || schema_of(
            &mut client,
            9,
            corrected_topic
        )
        .as_deref()
            == Some(qualified)),
        "the rollback must not block a VALID replacement from landing: {}",
        client.request(r#"{"id":10,"method":"status"}"#)
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

/// One offer carrying the same qualified name twice reports
/// ONE type — on every counter, not just the refusal memory.
///
/// Measured on the wire without the per-type dedup: a duplicate NEW name answers
/// `{"accepted":2, "known":1, "offered":2}` — one type reported as accepted
/// TWICE beside a `known` of one, a pair of numbers the response's own field docs
/// contradict — and a duplicate HELD name lists `replaced` twice, while
/// `replaced`'s doc calls a replacement a distinct outcome from a gain.
///
/// `offered` is deliberately the DOC count (it is what the caller sent); every
/// other counter is per TYPE. Nothing upstream dedups: the verb takes docs
/// straight off the control seam, and `classify_schema_replies` does not dedup a
/// robot's served closure.
#[test]
fn a_duplicated_name_in_one_offer_reports_one_type() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("vizd_dup");
    let (worker, _flush, _storage) = memory_worker("dup");
    let (socket, dir) = temp_socket("dup");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    let q = "offer_probe/Dup";
    let doc_of = |text: &str| SchemaDoc {
        qualified: q.to_string(),
        encoding: SchemaEncoding::Msg,
        text: text.to_string(),
        deps: vec![],
    };
    let offer_two = |client: &mut Client, id: u64, a: &SchemaDoc, b: &SchemaDoc| -> Value {
        client.request(&format!(
            r#"{{"id":{id},"method":"schemas","docs":[{},{}]}}"#,
            serde_json::to_string(a).unwrap(),
            serde_json::to_string(b).unwrap()
        ))
    };

    // (1) A duplicated NEW name: one type became decodable, so `accepted` is 1.
    let first = doc_of("float64 x\nfloat64 y\n");
    let r = offer_two(&mut client, 1, &first, &first);
    assert_eq!(r["ok"].as_bool(), Some(true), "{r}");
    assert_eq!(
        r["offered"].as_u64(),
        Some(2),
        "`offered` counts DOCS — it is what the caller sent: {r}"
    );
    assert_eq!(
        r["accepted"].as_u64(),
        Some(1),
        "THE PIN: one TYPE became decodable, so one is accepted (2 = the same \
         type counted per doc): {r}"
    );
    assert_eq!(
        r["known"].as_u64(),
        Some(1),
        "…and it agrees with `known`, which was the contradiction: {r}"
    );

    // (2) A duplicated HELD name: one type was replaced, so `replaced` names it
    //     ONCE. `accepted` is 0 — an update is not a gain.
    let second = doc_of("float64 a\nfloat64 b\n");
    let r = offer_two(&mut client, 2, &second, &second);
    assert_eq!(
        r["replaced"]
            .as_array()
            .map(|a| a.iter().filter_map(Value::as_str).collect::<Vec<_>>()),
        Some(vec![q]),
        "THE PIN: one type replaced, one report line: {r}"
    );
    assert_eq!(
        r["accepted"].as_u64(),
        Some(0),
        "a replacement is reported as such, never as a gain: {r}"
    );
    assert_eq!(r["known"].as_u64(), Some(1), "still one definition: {r}");

    // ANTI-TAUTOLOGY: two DISTINCT new types in one offer really do count two, so
    // the ones above are about the duplicate and not a counter stuck at 1.
    let other = SchemaDoc {
        qualified: "offer_probe/DupOther".to_string(),
        encoding: SchemaEncoding::Msg,
        text: "int32 n\n".to_string(),
        deps: vec![],
    };
    let third = SchemaDoc {
        qualified: "offer_probe/DupThird".to_string(),
        encoding: SchemaEncoding::Msg,
        text: "int32 m\n".to_string(),
        deps: vec![],
    };
    let r = offer_two(&mut client, 3, &other, &third);
    assert_eq!(
        r["accepted"].as_u64(),
        Some(2),
        "two DISTINCT types is genuinely two: {r}"
    );
    assert_eq!(r["known"].as_u64(), Some(3), "{r}");

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Test 10 — discover over many silent topics is BOUNDED ───────

#[test]
fn discover_over_many_silent_topics_is_bounded() {
    let mgr = isolated_transport("vizd_bounded");
    // N SILENT topics: a publisher makes each topic EXIST, but none ever publish,
    // so each would peek for PEEK_BOUND (250ms). Peeked SEQUENTIALLY that is N × 250ms
    // (= 3s at N=12) — a controller blocked for seconds.
    const N: usize = 12;
    let _pubs: Vec<_> = (0..N)
        .map(|i| {
            mgr.create_publisher(
                &format!("/vizd/silent_{i}"),
                MaxSliceLen::const_new(1 << 16),
                0,
            )
            .expect("silent producer attaches")
        })
        .collect();

    let (worker, _flush, _storage) = memory_worker("bounded");
    let (socket, dir) = temp_socket("bounded");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None, // rerun_url: a memory sink hosts no endpoint
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    let t0 = Instant::now();
    let disc = client.request(r#"{"id":1,"method":"discover"}"#);
    let elapsed = t0.elapsed();

    assert_eq!(disc["ok"].as_bool(), Some(true));
    // The WHOLE sweep is bounded by ONE shared peek budget (~500ms), NOT N×250ms.
    // A generous ceiling that still cleanly separates the shared budget (~0.5s) from the
    // sequential 3s.
    assert!(
        elapsed < Duration::from_millis(1500),
        "discover bounded across {N} silent topics, took {elapsed:?}"
    );
    // Every silent topic returns schema null (not fabricated, not hung).
    for i in 0..N {
        let e = entry_for(&disc["topics"], "topic", &format!("/vizd/silent_{i}"))
            .expect("silent topic listed");
        assert!(e["schema"].is_null(), "silent topic {i} → schema null: {e}");
    }

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── discover gains the remote ROBOTS gather via the netd query plane ────────────

/// A hand-built multi-entry [`CatalogReply`] (a test oracle — NOT fake data).
fn oracle_catalog(robot: &str, entries: &[(&str, Option<&str>)]) -> cerulion_core::CatalogReply {
    use cerulion_core::{CatalogEntry, CatalogProvenance};
    cerulion_core::CatalogReply {
        version: 1,
        robot: robot.to_string(),
        entries: entries
            .iter()
            .map(|(topic, schema_name)| CatalogEntry {
                topic: topic.to_string(),
                schema_hash: Some(7),
                schema_name: schema_name.map(str::to_string),
                provenance: CatalogProvenance::Runtime,
                producer_count: None,
                liveness: None,
            })
            .collect(),
        error: None,
    }
}

#[test]
fn discover_folds_the_remote_robot_catalog_into_a_robots_section() {
    // The WIRING pin — `discover` calls the demand plane's LAN gather and
    // folds every announcing robot's served catalog into the response's `robots`
    // section, attributing + inferring + filtering local-present topics, while a
    // REFUSED robot surfaces its reason explicitly. A netd-unreachable gather then
    // degrades to LOCAL-ONLY (the local list never breaks).
    let mgr = isolated_transport("robots");
    // A genuine LOCAL topic so the local list is non-empty AND to prove the
    // local-present FILTER (the robot also serves `/local`).
    let _local = mgr
        .create_publisher("/local", MaxSliceLen::const_new(1 << 16), 0)
        .expect("local producer attaches");

    // The gather returns TWO robots (given out of sorted order → must sort to
    // [locked, ubuntu]): `ubuntu` serves a mapped type (/scan), an unnamed type
    // (/odom), and the LOCAL topic (/local → filtered); `locked` REFUSED.
    let catalogs = vec![
        oracle_catalog(
            "ubuntu",
            &[
                ("/local", Some("sensor_msgs/PointCloud2")), // local-present → filtered
                ("/scan", Some("sensor_msgs/LaserScan")),    // mapped → archetype hint
                ("/odom", None),                             // unnamed → schema null
            ],
        ),
        cerulion_core::CatalogReply::refused("locked", "pairing required"),
    ];
    let plane = Arc::new(GatherPlane::new(catalogs));

    let (worker, _flush, _storage) = memory_worker("robots");
    let (socket, dir) = temp_socket("robots");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // ── Arm 1: the remote gather folds into `robots` ─────────────────────────
    let disc = client.request(r#"{"id":1,"method":"discover"}"#);
    assert_eq!(disc["ok"].as_bool(), Some(true));
    // The LOCAL topic is still listed locally (with no robot attribution).
    let local = entry_for(&disc["topics"], "topic", "/local").expect("local topic listed");
    assert!(
        local.get("robot").is_none(),
        "a local topic carries no robot"
    );

    let robots = disc["robots"].as_array().expect("robots section present");
    assert_eq!(robots.len(), 2, "two robots, sorted: {robots:?}");
    // Sorted by identity: locked, then ubuntu.
    assert_eq!(robots[0]["robot"].as_str(), Some("locked"));
    assert_eq!(robots[1]["robot"].as_str(), Some("ubuntu"));

    // locked — REFUSED: its reason surfaces, no topics.
    assert_eq!(robots[0]["error"].as_str(), Some("pairing required"));
    assert!(robots[0]["topics"].as_array().unwrap().is_empty());

    // ubuntu — served: /local FILTERED (already local), /odom + /scan remain.
    let u_topics = robots[1]["topics"].as_array().unwrap();
    let names: Vec<&str> = u_topics
        .iter()
        .filter_map(|e| e["topic"].as_str())
        .collect();
    assert_eq!(
        names,
        vec!["/odom", "/scan"],
        "local-present filtered + sorted"
    );
    // Every remote entry self-attributes ubuntu (for attach).
    assert!(u_topics
        .iter()
        .all(|e| e["robot"].as_str() == Some("ubuntu")));
    // /scan carries its schema name + a NAME-inferred archetype hint (no frame).
    let scan = entry_for(&robots[1]["topics"], "topic", "/scan").expect("/scan listed");
    assert_eq!(scan["schema"].as_str(), Some("sensor_msgs/LaserScan"));
    assert_eq!(scan["archetype"].as_str(), Some("LaserScan"));
    // /odom (unnamed) → schema/archetype null (the null-not-empty convention).
    let odom = entry_for(&robots[1]["topics"], "topic", "/odom").expect("/odom listed");
    assert!(odom["schema"].is_null() && odom["archetype"].is_null());

    // ── Arm 2: netd-unreachable degrades to LOCAL-ONLY (the local list survives) ──
    plane.set_fail(true);
    let degraded = client.request(r#"{"id":2,"method":"discover"}"#);
    assert_eq!(degraded["ok"].as_bool(), Some(true), "still succeeds");
    assert!(
        entry_for(&degraded["topics"], "topic", "/local").is_some(),
        "the LOCAL list is intact under a netd-unreachable gather"
    );
    // `robots` is skip-if-empty → absent/empty on the degrade (never a hard failure).
    assert!(
        degraded
            .get("robots")
            .and_then(|r| r.as_array())
            .is_none_or(|r| r.is_empty()),
        "netd-unreachable ⇒ no robots section (local-only): {degraded}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

/// THE bound for any assertion that
/// needs a mirror's ROBOT ATTRIBUTION to have converged.
///
/// Attribution is not a property of the topic — it is the answer of
/// `Ctx::attribution_snapshot`, which unions a per-tap record (set only by
/// `attach_remote`, so a LOCALLY-attached mirror contributes nothing) with a
/// CACHED `/__cerulion/mirrors` gather. That gather is best-effort over SHM with
/// a bounded window, and a gather that misses the record is cached exactly like a
/// real one — so a registered mirror can read as a LOCAL topic for up to a full
/// [`cerulion_vizd::PROVENANCE_CACHE_TTL`], and for a multiple of it if
/// consecutive gathers miss.
///
/// Two test shapes fall to that fact: a single
/// unpolled `discover` read (the topic renders as a full
/// LOCAL row, which is reachable ONLY through a `mirrors` map lacking its key),
/// and a poll of **4 s against this 3 s TTL**, i.e.
/// ONE refresh cycle of headroom, which loses the race at a
/// "MUST NOT appear under LOCAL topics" assert.
///
/// Expressed as a MULTIPLE of the daemon's own constant so it cannot silently
/// fall under the TTL, and sized for several consecutive missed gathers.
///
/// A larger bound costs a healthy run NOTHING — every user is a poll that exits
/// the instant its predicate holds, so the budget is only ever paid by a run that
/// would otherwise have FAILED. That asymmetry is why this is generous rather
/// than tuned.
const ATTRIBUTION_CONVERGENCE_BOUND: Duration =
    Duration::from_secs(5 * cerulion_vizd::PROVENANCE_CACHE_TTL.as_secs());

/// Re-`discover` (retrying) until `predicate(&disc)` holds or the bound elapses;
/// returns the LAST discover response (converged or not) so the caller asserts on it.
/// The bounded retry absorbs the cross-thread mirror-provenance republish cadence
/// (a fresh gather reader converges within a few [`MIRROR_REPUBLISH_INTERVAL`]s).
///
/// When the predicate gates on mirror ATTRIBUTION, the bound must be
/// [`ATTRIBUTION_CONVERGENCE_BOUND`] — see that constant for why a hand-written
/// few-second budget is not enough.
fn discover_until(
    client: &mut Client,
    bound: Duration,
    mut predicate: impl FnMut(&Value) -> bool,
) -> Value {
    let deadline = Instant::now() + bound;
    loop {
        let disc = client.request(r#"{"id":1,"method":"discover"}"#);
        if predicate(&disc) || Instant::now() >= deadline {
            return disc;
        }
        std::thread::sleep(Duration::from_millis(50));
    }
}

#[test]
fn discover_folds_a_mirror_to_its_robot_and_never_flips_local_e2e() {
    // The sidebar grouping contract: when a robot topic is checked, netd
    // registers a desk-local MIRROR (a `{topic}/data` re-injection + a provenance record
    // attributing it to its origin robot). A `discover` that reported that mirror
    // as a LOCAL topic would make the sidebar MOVE it from the robot group to THIS MACHINE (and,
    // if the mirror lingered after uncheck, keep it there). What prevents it is the provenance
    // fold — a locally-visible topic in the `/__cerulion/mirrors` registry folds OUT of
    // LOCAL and INTO its origin robot's section. HAND oracle throughout; the headline
    // invariant is that the mirror NEVER appears under LOCAL `topics` (never flips to
    // THIS MACHINE) across the mirror's whole lifecycle.
    let mgr = isolated_transport("mirror_fold");

    // A GENUINE local topic (a real producer) — proves the fold is surgical (local
    // topics stay local).
    let _local = mgr
        .create_publisher("/fold/local", MaxSliceLen::const_new(1 << 16), 0)
        .expect("genuine local producer attaches");

    // The MIRROR: a live `/fold/scan/data` service (so `list_topics` shows it) + a
    // provenance record attributing it to "ubuntu" (exactly what netd's re-injector
    // does via `register_mirror_provenance`). Held in `mirror` so we
    // can DROP it in arm 3 to model netd's refcount-0 teardown.
    let mirror = mgr
        .create_publisher("/fold/scan", MaxSliceLen::const_new(1 << 16), 0)
        .expect("mirror service attaches");
    assert!(
        mgr.register_mirror_provenance("/fold/scan", "ubuntu")
            .expect("register mirror provenance"),
        "the mirror is newly attributed to ubuntu"
    );

    // A SECOND mirror attributed to a DISTINCT robot "orin" — the routing discriminator:
    // the fold must attribute each mirror to its OWN origin robot among multiple rows,
    // never cross them or lump both under one robot. Held for the whole test (never torn
    // down) so arm 3 also pins that tearing down ubuntu's mirror leaves orin's intact.
    let _mirror_orin = mgr
        .create_publisher("/fold/lidar", MaxSliceLen::const_new(1 << 16), 0)
        .expect("orin mirror service attaches");
    assert!(
        mgr.register_mirror_provenance("/fold/lidar", "orin")
            .expect("register orin mirror provenance"),
        "the second mirror is newly attributed to orin"
    );

    // The netd catalog gather serves TWO robots: ubuntu (the mirrored /fold/scan PLUS a
    // catalog-only /fold/other) and orin (the mirrored /fold/lidar).
    let catalogs = vec![
        oracle_catalog(
            "ubuntu",
            &[
                ("/fold/scan", Some("sensor_msgs/LaserScan")),
                ("/fold/other", Some("sensor_msgs/Image")),
            ],
        ),
        oracle_catalog("orin", &[("/fold/lidar", Some("sensor_msgs/PointCloud2"))]),
    ];
    let plane = Arc::new(GatherPlane::new(catalogs));

    let (worker, _flush, _storage) = memory_worker("mirror_fold");
    let (socket, dir) = temp_socket("mirror_fold");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // Helper: the `topic` entry under robot `robot` in the robots section (or None).
    let robot_entry = |disc: &Value, robot: &str, topic: &str| -> Option<Value> {
        disc["robots"].as_array()?.iter().find_map(|r| {
            if r["robot"].as_str() != Some(robot) {
                return None;
            }
            r["topics"]
                .as_array()?
                .iter()
                .find(|e| e["topic"].as_str() == Some(topic))
                .cloned()
        })
    };
    let under_robot = |disc: &Value, robot: &str, topic: &str| -> bool {
        robot_entry(disc, robot, topic).is_some()
    };

    // ── Arm 1: WITH the catalog — each mirror folds out of LOCAL, onto its OWN robot ─
    //
    // The budget is DERIVED from the daemon's 3 s provenance-cache TTL. A
    // hand-written 4 s leaves ONE
    // refresh cycle of headroom — so a single missed gather consumes most of it
    // and `discover_until` returns an UNCONVERGED response, which then fails
    // the assert three lines down (`/fold/scan` still under LOCAL topics). The
    // oracle is exact; only the convergence budget scales, and it is derived
    // from the TTL rather than guessed. See `ATTRIBUTION_CONVERGENCE_BOUND`.
    let disc = discover_until(&mut client, ATTRIBUTION_CONVERGENCE_BOUND, |d| {
        under_robot(d, "ubuntu", "/fold/scan")
            && under_robot(d, "orin", "/fold/lidar")
            && entry_for(&d["topics"], "topic", "/fold/scan").is_none()
    });
    // Neither mirror is a LOCAL topic (the headline: never flips to THIS MACHINE).
    assert!(
        entry_for(&disc["topics"], "topic", "/fold/scan").is_none()
            && entry_for(&disc["topics"], "topic", "/fold/lidar").is_none(),
        "the mirrors MUST NOT appear under LOCAL topics: {disc}"
    );
    // The genuine local topic IS local, unattributed.
    let local = entry_for(&disc["topics"], "topic", "/fold/local").expect("genuine local listed");
    assert!(
        local.get("robot").is_none(),
        "a genuine local carries no robot"
    );
    // Each mirror is attributed to its OWN robot, self-attributing for attach.
    let scan = robot_entry(&disc, "ubuntu", "/fold/scan").expect("scan mirror under ubuntu");
    assert_eq!(
        scan["robot"].as_str(),
        Some("ubuntu"),
        "scan self-attributes ubuntu"
    );
    let lidar = robot_entry(&disc, "orin", "/fold/lidar").expect("lidar mirror under orin");
    assert_eq!(
        lidar["robot"].as_str(),
        Some("orin"),
        "lidar self-attributes orin"
    );
    // ROUTING DISCRIMINATOR: never crossed / lumped under one robot.
    assert!(
        !under_robot(&disc, "orin", "/fold/scan") && !under_robot(&disc, "ubuntu", "/fold/lidar"),
        "each mirror routes to its OWN robot, never crossed: {disc}"
    );
    // /fold/other (catalog-only) also under ubuntu — the catalog fold is intact.
    assert!(
        under_robot(&disc, "ubuntu", "/fold/other"),
        "the catalog-only topic still rows under ubuntu"
    );

    // ── Arm 2: netd-unreachable — both mirrors STAY attributed (network-free) ────────
    // The fold SYNTHESIZES each robot's row from PROVENANCE ALONE — the strong pin is
    // the robot VALUE (fold-sourced, not catalog-sourced) AND the routing discriminator.
    plane.set_fail(true);
    let disc = discover_until(&mut client, ATTRIBUTION_CONVERGENCE_BOUND, |d| {
        under_robot(d, "ubuntu", "/fold/scan") && under_robot(d, "orin", "/fold/lidar")
    });
    assert!(
        entry_for(&disc["topics"], "topic", "/fold/scan").is_none()
            && entry_for(&disc["topics"], "topic", "/fold/lidar").is_none(),
        "the mirrors still MUST NOT be LOCAL under a netd-unreachable gather: {disc}"
    );
    // Fold-sourced robot VALUE (no catalog to borrow it from — provenance is the source).
    let scan = robot_entry(&disc, "ubuntu", "/fold/scan").expect("scan under ubuntu (fold-sole)");
    assert_eq!(
        scan["robot"].as_str(),
        Some("ubuntu"),
        "network-free: /fold/scan's robot VALUE is fold-sourced 'ubuntu': {disc}"
    );
    let lidar = robot_entry(&disc, "orin", "/fold/lidar").expect("lidar under orin (fold-sole)");
    assert_eq!(
        lidar["robot"].as_str(),
        Some("orin"),
        "network-free: /fold/lidar's robot VALUE is fold-sourced 'orin': {disc}"
    );
    // ROUTING DISCRIMINATOR on the fold-sole path: never crossed.
    assert!(
        !under_robot(&disc, "orin", "/fold/scan") && !under_robot(&disc, "ubuntu", "/fold/lidar"),
        "fold-sole: each mirror routes to its OWN robot, never crossed: {disc}"
    );
    // The catalog-only topic is GONE (no catalog, no mirror) — only live mirrors row.
    assert!(
        !under_robot(&disc, "ubuntu", "/fold/other"),
        "a catalog-only topic vanishes when the catalog gather fails: {disc}"
    );
    plane.set_fail(false);

    // ── Arm 3: uncheck → teardown → the mirror RETURNS to the robot, never flips local ─
    // Model netd's refcount-0 teardown of ONLY ubuntu's mirror: drop its service +
    // remove its provenance. orin's mirror is untouched (isolation).
    drop(mirror);
    let _ = mgr.unregister_mirror_provenance("/fold/scan");
    // Retry until the torn-down mirror leaves the local enumeration; it must resurface
    // under ubuntu via the (now-reachable) catalog, and NEVER as a LOCAL topic.
    //
    // The same subset-wait class as the `/plan` arm: a predicate of
    // `under_robot(ubuntu, /fold/scan)` ALONE is not what the line above
    // says it waits for and — worse — is ALREADY TRUE when arm 2 finishes, so it
    // is satisfied by the first discover after the teardown. The orin pair is the
    // real exposure: attribution rides the cached `gather_mirror_provenance`
    // (`PROVENANCE_CACHE_TTL`), so a refresh that misses reverts `/fold/lidar` to a
    // LOCAL row for a whole TTL while a scan-only wait has long since returned. The
    // predicate is exactly the fact set the assertions below read.
    let disc = discover_until(&mut client, ATTRIBUTION_CONVERGENCE_BOUND, |d| {
        under_robot(d, "ubuntu", "/fold/scan")
            && entry_for(&d["topics"], "topic", "/fold/scan").is_none()
            && under_robot(d, "orin", "/fold/lidar")
            && entry_for(&d["topics"], "topic", "/fold/lidar").is_none()
    });
    assert!(
        entry_for(&disc["topics"], "topic", "/fold/scan").is_none(),
        "after teardown the mirror NEVER flips to LOCAL (the 'stays there after uncheck' bug): {disc}"
    );
    assert!(
        under_robot(&disc, "ubuntu", "/fold/scan"),
        "after teardown the topic returns to its robot via the catalog: {disc}"
    );
    // orin's still-live mirror is unaffected by ubuntu's teardown (isolation).
    assert!(
        under_robot(&disc, "orin", "/fold/lidar")
            && entry_for(&disc["topics"], "topic", "/fold/lidar").is_none(),
        "orin's mirror stays attributed + non-local after ubuntu's teardown: {disc}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn an_attached_mirror_is_attributed_to_its_robot_on_list_and_status_e2e() {
    // Reported on a live desk: attaching a robot's topics made each one
    // appear TWICE in the Studio sidebar — once under its robot (correct) and once under
    // a "THIS MACHINE" section that materialized the moment the demand registered, with a
    // live rate on both rows. `discover` was already folding correctly, and
    // the phantom section appearing exactly ON ATTACH is the tell: the defect was on the
    // ATTACH-STATE surfaces. `list` and `status` carried NO robot attribution at all, so
    // an attached netd MIRROR of a remote robot's topic went out as a bare topic + rate,
    // indistinguishable from a desk graph's own topic — and the only place a sidebar can
    // put an attached-with-a-rate row it cannot attribute is the desk's own section.
    //
    // The pin: for ONE attached mirror, `discover`'s robot row and the `list`/`status`
    // rows must all name the SAME robot (one topic = one row), while a genuinely-local
    // attached tap carries NO robot on any of them. HAND oracles throughout.
    let mgr = isolated_transport("attach_state");

    // The MIRROR: a live publisher (so it is locally visible AND carries a real rate,
    // exactly like netd's re-inject publisher) + the provenance record netd registers.
    let _mirror_pub = Publisher::spawn(Arc::clone(&mgr), "/attr/scan");
    assert!(
        mgr.register_mirror_provenance("/attr/scan", "ubuntu")
            .expect("register mirror provenance"),
        "the mirror is newly attributed to ubuntu"
    );
    // A GENUINE desk producer — the surgical control: it must stay unattributed.
    let _local_pub = Publisher::spawn(Arc::clone(&mgr), "/attr/local");

    // The netd catalog serves ubuntu's topic, so the sidebar has a robot row for it.
    let plane = Arc::new(GatherPlane::new(vec![oracle_catalog(
        "ubuntu",
        &[("/attr/scan", Some("geometry_msgs/Vector3"))],
    )]));

    let (worker, _flush, _storage) = memory_worker("attach_state");
    let (socket, dir) = temp_socket("attach_state");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // Attach BOTH — the mirror is already locally visible (netd registered it), which is
    // exactly the live path: `attach` routes it to the LOCAL tap, so NOTHING but the
    // provenance registry can tell the daemon whose topic it is.
    for topic in ["/attr/scan", "/attr/local"] {
        let resp = client.request(&format!(
            r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
        ));
        assert_eq!(
            resp["ok"].as_bool(),
            Some(true),
            "attach '{topic}' succeeds: {resp}"
        );
    }

    // ── The headline: the attach-state rows agree with the robot section ────────────
    //
    // CONVERGE, THEN ASSERT. A single unpolled
    // `list` immediately after `attach` would assert as though the mirror's
    // attribution were available synchronously. It is not, BY DESIGN:
    // `attribution_snapshot` unions a CACHED `gather_mirror_provenance()`, and
    // that cache has a `PROVENANCE_CACHE_TTL` of 3 s (daemon.rs:438). A gather
    // that misses the record — the SHM read is best-effort and a freshly
    // attached reader can lag the already-published provenance sample — is then
    // SERVED STALE for up to the whole TTL, and the tap's own `origin_robot`,
    // recorded at attach from that same gather, is `None` too. So the row can
    // legitimately carry no robot for ~3 s.
    //
    // That transient is not a defect this test is entitled to catch: the daemon's
    // cache docs name it acceptable (the defect is a PERSISTENT mis-group), and the
    // sibling `discover_provenance_snapshot_is_cached_within_ttl_e2e` asserts the
    // very same staleness as CORRECT behaviour. A loaded macOS runner produces it
    // (`left: None, right: Some("ubuntu")`).
    //
    // THE ORACLE IS EXACT — `Some("ubuntu")` on the mirror and NO robot
    // on the desk producer, both read from ONE response. A mirror that is never
    // attributed still fails, with a bound rather than on the first sample.
    // (The `status` half below polls too — but see the note there: gating on `hz`
    // ALONE does not cover the attribution its own assertion reads, so that half is
    // exposed to this same transient.)
    let mut list = client.request(r#"{"id":2,"method":"list"}"#);
    let attribution_deadline = Instant::now() + Duration::from_secs(15);
    while Instant::now() < attribution_deadline
        && entry_for(&list["attached"], "topic", "/attr/scan")
            .and_then(|e| e["robot"].as_str().map(|s| s.to_string()))
            .is_none()
    {
        std::thread::sleep(Duration::from_millis(100));
        list = client.request(r#"{"id":2,"method":"list"}"#);
    }
    let scan = entry_for(&list["attached"], "topic", "/attr/scan").expect("mirror is attached");
    assert_eq!(
        scan["robot"].as_str(),
        Some("ubuntu"),
        "PRE-FIX REGRESSION: an attached mirror carried NO robot, so the sidebar had \
         nowhere to put it but THIS MACHINE: {list}"
    );
    let local = entry_for(&list["attached"], "topic", "/attr/local").expect("local is attached");
    assert!(
        local.get("robot").is_none(),
        "a genuine desk producer must stay unattributed (the fold is surgical): {list}"
    );

    // `status` — the row that carries the RATE, which is what made the phantom row look
    // alive ("9.9 Hz in both places"). Polled until a real rate exists, so the assertion
    // below runs against a genuinely streaming mirror.
    //
    // The poll gates on the ROBOT as well as the rate, and the assertions
    // read the response the poll VALIDATED — the same capture-then-assert shape as
    // the `list` half above. Gating on `hz` alone covers a strict SUBSET of what
    // the assertions need (the waits-for-A-asserts-B family), and
    // re-requesting afterwards means no predicate can cover them anyway:
    // the attribution the assertion reads would be a DIFFERENT response's.
    let status_line = r#"{"id":3,"method":"status"}"#;
    let mut status = client.request(status_line);
    let status_deadline = Instant::now() + Duration::from_secs(6);
    while Instant::now() < status_deadline
        && !entry_for(&status["topics"], "topic", "/attr/scan").is_some_and(|e| {
            e["hz"].as_f64().is_some_and(|hz| hz > 0.0) && e["robot"].as_str() == Some("ubuntu")
        })
    {
        std::thread::sleep(Duration::from_millis(100));
        status = client.request(status_line);
    }
    let scan_status = entry_for(&status["topics"], "topic", "/attr/scan").expect("mirror status");
    assert_eq!(
        scan_status["robot"].as_str(),
        Some("ubuntu"),
        "the RATE row must be attributed to the robot the data comes from: {status}"
    );
    assert!(
        scan_status["hz"].as_f64().is_some_and(|hz| hz > 0.0),
        "the mirror really is streaming (so this is the live shape, not a dead route): \
         {status}"
    );
    let local_status = entry_for(&status["topics"], "topic", "/attr/local").expect("local status");
    assert!(
        local_status.get("robot").is_none(),
        "the genuine local topic's rate stays unattributed: {status}"
    );

    // ── ONE topic = ONE row: discover's robot row names the SAME robot, and the mirror
    //    is still absent from LOCAL topics (the discover-fold half must not have regressed) ──
    let disc = discover_until(&mut client, ATTRIBUTION_CONVERGENCE_BOUND, |d| {
        d["robots"].as_array().is_some_and(|robots| {
            robots.iter().any(|r| {
                r["robot"].as_str() == Some("ubuntu")
                    && r["topics"].as_array().is_some_and(|t| {
                        t.iter().any(|e| e["topic"].as_str() == Some("/attr/scan"))
                    })
            })
        })
    });
    assert!(
        entry_for(&disc["topics"], "topic", "/attr/scan").is_none(),
        "the mirror must never be listed as a LOCAL topic: {disc}"
    );
    assert!(
        entry_for(&disc["topics"], "topic", "/attr/local").is_some(),
        "the genuine local topic IS listed locally: {disc}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn a_torn_down_mirror_keeps_its_attached_row_on_its_robot_e2e() {
    // The stability half: the live provenance snapshot is BEST-EFFORT and
    // CACHED, so it can momentarily fail to answer for a topic this daemon knowingly
    // attached from a robot — a gather error, or (modelled here) netd's refcount-0
    // teardown dropping the mirror while vizd still holds the tap. Without a fallback
    // the attached row would flicker into the desk's own section — re-creating exactly
    // the phantom "THIS MACHINE" row the attribution fold removes, just transiently.
    //
    // The fallback is the ORIGIN ROBOT the TAP recorded for itself at attach
    // (`TopicStat::origin_robot`) — NOT the name-keyed session tombstone, which
    // would also attribute a later, unrelated desk producer of the same name
    // (pinned by
    // `a_desk_producer_reusing_a_detached_remote_topic_name_is_never_attributed_e2e`).
    // Both attach seams record it: the REMOTE path from the robot it demanded from,
    // and `attach_local` from the very snapshot its own reply is built from, so a
    // mirror entering through the local path is attributed on every surface
    // (`a_locally_attached_mirror_is_monitored_under_its_robot_e2e`). Neither can
    // attribute a genuine desk producer — `None` writes nothing.
    // So this test drives a real `attach_remote` (the topic is NOT locally visible at
    // attach time — the demand plane creates the mirror), then tears the mirror's
    // provenance down UNDER the still-held tap and asserts the row STAYS on ubuntu.
    // Network-configured (the remote arm's precondition); the demand plane below is a
    // double, so no zenoh session is ever opened.
    let mgr = isolated_transport_with_network("tombstone");

    // A demand plane that MODELS netd's mirror lifecycle: on `demand` it creates the
    // real desk-local mirror publisher (so vizd's tap opens `{topic}/data`).
    let plane = Arc::new(MirrorSpyPlane::new(
        Arc::clone(&mgr),
        vec![oracle_catalog(
            "ubuntu",
            &[("/attr/tomb", Some("geometry_msgs/Vector3"))],
        )],
    ));

    let (worker, _flush, _storage) = memory_worker("tombstone");
    let (socket, dir) = temp_socket("tombstone");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    let resp = client.request(
        r#"{"id":1,"method":"attach","topic":"/attr/tomb","robot":"ubuntu","schema":"geometry_msgs/Vector3"}"#,
    );
    assert_eq!(
        resp["ok"].as_bool(),
        Some(true),
        "the remote attach succeeds: {resp}"
    );

    // netd registers the mirror's provenance at its re-injection point; the spy plane
    // models only the publisher half, so the test plays netd's registrar here. A second
    // UNATTACHED mirror is registered as the PROBE that makes the teardown assertion
    // non-vacuous (below).
    let _probe_pub = mgr
        .create_publisher("/attr/probe", MaxSliceLen::const_new(1 << 16), 0)
        .expect("probe mirror service attaches");
    for topic in ["/attr/tomb", "/attr/probe"] {
        assert!(
            mgr.register_mirror_provenance(topic, "ubuntu")
                .expect("register mirror provenance"),
            "'{topic}' is newly attributed to ubuntu"
        );
    }

    // While the mirror is LIVE, the live snapshot attributes it. Polled because the
    // daemon's provenance cache may still hold the pre-registration snapshot.
    wait_until(Duration::from_secs(6), || {
        entry_for(
            &client.request(r#"{"id":2,"method":"list"}"#)["attached"],
            "topic",
            "/attr/tomb",
        )
        .and_then(|e| e["robot"].as_str().map(str::to_string))
        .is_some()
    });
    let list = client.request(r#"{"id":2,"method":"list"}"#);
    assert_eq!(
        entry_for(&list["attached"], "topic", "/attr/tomb").expect("attached")["robot"].as_str(),
        Some("ubuntu"),
        "the live mirror is attributed from the provenance registry: {list}"
    );

    // The live evidence goes away under a still-held tap: BOTH provenance records are
    // removed (netd's refcount-0 teardown drops the record at its re-injection point).
    for topic in ["/attr/tomb", "/attr/probe"] {
        assert!(
            mgr.unregister_mirror_provenance(topic)
                .expect("unregister provenance"),
            "'{topic}' had a provenance record and it is now removed"
        );
    }

    // ANTI-VACUITY GATE. The daemon caches the provenance snapshot for ~3 s, so an
    // immediate assertion would be answered from the CACHED (still-attributing)
    // snapshot and would pass with or without the fallback. `/attr/probe` is the
    // discriminator: it is NOT attached, so no tap record can cover it — the
    // instant it shows up as a LOCAL topic, the daemon has provably re-gathered and the
    // FRESH snapshot attributes neither topic.
    // The SAME cache governs this (inverse) direction — here we wait for a
    // re-gather to DROP the torn-down attribution — so it takes the same derived
    // bound rather than its own hand-written multiple of the TTL.
    let regathered = discover_until(&mut client, ATTRIBUTION_CONVERGENCE_BOUND, |d| {
        entry_for(&d["topics"], "topic", "/attr/probe").is_some()
    });
    assert!(
        entry_for(&regathered["topics"], "topic", "/attr/probe").is_some(),
        "the daemon must re-gather within the cache TTL, or this test proves nothing about \
         the post-teardown snapshot: {regathered}"
    );

    // THE PIN: with the live snapshot provably silent, the attached row STAYS on ubuntu
    // — only the tap's own recorded origin robot can supply that answer.
    let list = client.request(r#"{"id":3,"method":"list"}"#);
    assert_eq!(
        entry_for(&list["attached"], "topic", "/attr/tomb").expect("still attached")["robot"]
            .as_str(),
        Some("ubuntu"),
        "the attached row must NOT fall back to THIS MACHINE when the mirror's provenance \
         is torn down under a still-held tap: {list}"
    );
    // ONE topic, ONE row: `discover` agrees — the held tap stays out of LOCAL and under
    // its robot, because both surfaces read the SAME attribution map.
    assert!(
        entry_for(&regathered["topics"], "topic", "/attr/tomb").is_none(),
        "the held tap must not reappear as a LOCAL topic while list still attributes it \
         (that split IS the attribution bug): {regathered}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn a_desk_producer_reusing_a_detached_remote_topic_name_is_never_attributed_e2e() {
    // The headline: attribution must be scoped to the TAP,
    // not to the topic NAME. The daemon also keeps a session provenance TOMBSTONE
    // (the remote-attach routing memory, `remote_provenance`) which is keyed by name and
    // DELIBERATELY never cleared — so anything that consults it by name attributes
    // a topic forever, across generations.
    //
    // The sequence that separates the two: remote-attach a robot's topic, DETACH it
    // (netd's refcount-0 teardown genuinely frees the single-writer slot — pinned in
    // `mirror_plane_iox2_test`), then let a GENUINE DESK PRODUCER take the same
    // name and attach THAT. The new tap is this desk's own data. A name-keyed
    // lookup re-satisfies on it and attributes a desk-owned topic to the historical
    // robot on ALL THREE surfaces — and `discover` would additionally DROP it from
    // the desk's own section, the exact inverse of the mis-grouping the shared attribution map prevents.
    //
    // This test is the guard for that scoping rule. Building
    // the attribution's remembered half from `remote_provenance` (or from any
    // name-keyed source) instead of the tap's own `origin_robot` fails it with
    // `Some("ubuntu")` on a topic this desk produces itself.
    let mgr = isolated_transport_with_network("generation");
    let topic = "/attr/reused";

    let plane = Arc::new(MirrorSpyPlane::new(
        Arc::clone(&mgr),
        vec![oracle_catalog(
            "ubuntu",
            &[(topic, Some("geometry_msgs/Vector3"))],
        )],
    ));

    let (worker, _flush, _storage) = memory_worker("generation");
    let (socket, dir) = temp_socket("generation");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // ── Generation 1: the topic really is ubuntu's, attached remotely ───────────
    let attached = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}","robot":"ubuntu","schema":"geometry_msgs/Vector3"}}"#
    ));
    assert_eq!(
        attached["ok"].as_bool(),
        Some(true),
        "the remote attach succeeds: {attached}"
    );
    assert_eq!(
        attached["robot"].as_str(),
        Some("ubuntu"),
        "the attach reply names the robot it demanded from: {attached}"
    );
    mgr.register_mirror_provenance(topic, "ubuntu")
        .expect("register mirror provenance");
    let list = client.request(r#"{"id":2,"method":"list"}"#);
    assert_eq!(
        entry_for(&list["attached"], "topic", topic).expect("attached")["robot"].as_str(),
        Some("ubuntu"),
        "generation 1 IS ubuntu's topic: {list}"
    );

    // ── Uncheck: detach releases the demand, and netd's teardown drops both the
    //    mirror publisher (the spy plane's `release`) and its provenance ─────────
    let detached = client.request(&format!(
        r#"{{"id":3,"method":"detach","topic":"{topic}"}}"#
    ));
    assert_eq!(
        detached["ok"].as_bool(),
        Some(true),
        "detach ok: {detached}"
    );
    mgr.unregister_mirror_provenance(topic)
        .expect("unregister provenance");

    // ── Generation 2: a GENUINE DESK PRODUCER takes the freed name ──────────────
    // Created SYNCHRONOUSLY (not on a thread) so the `{topic}/data` service exists
    // before the attach below — otherwise `attach` sees a missing service and takes
    // the remote-resolution path instead of the LOCAL one this arm is about.
    let _desk_producer = mgr
        .create_publisher(topic, MaxSliceLen::const_new(1 << 16), 0)
        .expect("the desk's own producer takes the freed single-writer slot");
    // It is locally visible, so `attach` routes it through the LOCAL tap path.
    let reattached = client.request(&format!(
        r#"{{"id":4,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(
        reattached["ok"].as_bool(),
        Some(true),
        "attaching the desk's own producer succeeds: {reattached}"
    );

    // THE PIN — all four surfaces must call it OURS.
    assert!(
        reattached.get("robot").is_none(),
        "the attach reply must not attribute a desk-owned topic to the robot whose \
         topic merely shared this name: {reattached}"
    );
    let list = client.request(r#"{"id":5,"method":"list"}"#);
    let row = entry_for(&list["attached"], "topic", topic).expect("re-attached");
    assert!(
        row.get("robot").is_none(),
        "PRE-FIX (name-keyed scoping): a desk-owned topic was attributed to the \
         historical robot on `list`: {list}"
    );
    let status = client.request(r#"{"id":6,"method":"status"}"#);
    let srow = entry_for(&status["topics"], "topic", topic).expect("status row");
    assert!(
        srow.get("robot").is_none(),
        "…and on `status`, beside its rate: {status}"
    );
    // `discover` must list it under the desk's OWN topics — a name-keyed fold would
    // hide this desk's real producer from the section it belongs in.
    // Gated on the un-registered mirror leaving the cached snapshot — the
    // same TTL, so the same derived bound (this was 6 s, only 2× the TTL).
    let disc = discover_until(&mut client, ATTRIBUTION_CONVERGENCE_BOUND, |d| {
        entry_for(&d["topics"], "topic", topic).is_some()
    });
    let drow = entry_for(&disc["topics"], "topic", topic).expect(
        "the desk's own producer must appear under LOCAL topics — a name-keyed fold \
         would have folded it out to the historical robot",
    );
    assert!(
        drow.get("robot").is_none(),
        "the local row carries no robot: {disc}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn discover_provenance_snapshot_is_cached_within_ttl_e2e() {
    // `discover` serves mirror provenance from a short-TTL cache
    // so a live mirror does not cost a 150-600 ms `/__cerulion/mirrors` gather on EVERY
    // ~2 s sidebar poll (the cerulion_core registry fast path only short-circuits a
    // registry-LESS desk — precisely NOT the live-mirror scenario). This is the ANTI-TAUTOLOGY
    // pin for the cache: it proves the cache is ACTIVE (not a no-op that re-gathers
    // every call), by observing the ACCEPTED staleness — a mirror registered AFTER the
    // first (caching) discover is NOT yet folded on an immediate second discover (it
    // shows LOCAL for one cycle), whereas a per-call re-gather would fold it instantly.
    let mgr = isolated_transport("mirror_cache");
    // Mirror A, present BEFORE the first discover → in the first cached snapshot.
    let _a = mgr
        .create_publisher("/cache/a", MaxSliceLen::const_new(1 << 16), 0)
        .expect("mirror A attaches");
    assert!(mgr
        .register_mirror_provenance("/cache/a", "bota")
        .expect("register A provenance"));

    // A FAILING catalog gather so the ROBOTS section is driven PURELY by the provenance
    // fold — isolating the cache (no catalog to independently attribute topics).
    let plane = Arc::new(GatherPlane::new(Vec::new()));
    plane.set_fail(true);

    let (worker, _flush, _storage) = memory_worker("mirror_cache");
    let (socket, dir) = temp_socket("mirror_cache");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    let under_bot = |disc: &Value, robot: &str, topic: &str| -> bool {
        disc["robots"].as_array().is_some_and(|robots| {
            robots.iter().any(|r| {
                r["robot"].as_str() == Some(robot)
                    && r["topics"]
                        .as_array()
                        .is_some_and(|t| t.iter().any(|e| e["topic"].as_str() == Some(topic)))
            })
        })
    };

    // First discover: cache MISS → gathers {a: bota} → folds /cache/a to bota.
    //
    // A tight budget here (4 s, say) would time the WRONG THING — reaching the
    // first successful gather is a CONVERGENCE wait, not the property under
    // test, and a loaded macOS runner exceeds 4 s. The
    // property is that the fold HAPPENS and is then CACHED; how many 50 ms polls
    // that takes is the runner's business.
    //
    // A generous budget cannot disturb the 3 s TTL the rest of this test turns on: that
    // window runs from the LAST gather (the poll this returns on) to the
    // immediate second discover below, and those stay back-to-back however long
    // the loop spent getting here.
    let first = discover_until(&mut client, Duration::from_secs(30), |d| {
        under_bot(d, "bota", "/cache/a")
    });
    assert!(
        under_bot(&first, "bota", "/cache/a"),
        "the first discover folds the pre-existing mirror A: {first}"
    );

    // Register mirror B (service + provenance) AFTER the first discover cached its
    // snapshot. An IMMEDIATE second discover is a cache HIT (well within the 3 s TTL),
    // so B is NOT yet in the folded provenance: it shows LOCAL (stale — the accepted
    // one-cycle mis-group), NOT under botb. A per-call re-gather would fold it instantly.
    let _b = mgr
        .create_publisher("/cache/b", MaxSliceLen::const_new(1 << 16), 0)
        .expect("mirror B attaches");
    assert!(mgr
        .register_mirror_provenance("/cache/b", "botb")
        .expect("register B provenance"));

    let second = client.request(r#"{"id":2,"method":"discover"}"#);
    assert_eq!(second["ok"].as_bool(), Some(true));
    assert!(
        !under_bot(&second, "botb", "/cache/b"),
        "cache HIT: mirror B registered after the first gather is NOT folded yet: {second}"
    );
    assert!(
        entry_for(&second["topics"], "topic", "/cache/b").is_some(),
        "cache HIT: mirror B shows LOCAL for one cycle (the accepted staleness): {second}"
    );
    // A (in the cached snapshot) is still correctly folded — the cache serves it.
    assert!(
        under_bot(&second, "bota", "/cache/a"),
        "the cached mirror A stays folded on the cache-hit discover: {second}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn discover_rows_a_fully_attached_robot_with_no_extra_topics_e2e() {
    // The e2e twin of the pure oracle: a NON-refused robot whose
    // EVERY served topic is already visible LOCALLY (the desk attached all its mirrors)
    // still ROWS in `robots` — `topics: []`, NO `error` key — the "one data
    // source = one topic" surface: its topics are already listed/attachable under the
    // local section, so the sidebar renders its "no additional topics" branch rather
    // than dropping the robot. Distinct from a refused robot (which ALSO has empty
    // topics but carries an `error`).
    let mgr = isolated_transport("fully_attached");
    // The robot's ONLY topic is also a genuine LOCAL topic → filtered from `robots`.
    let _local = mgr
        .create_publisher("/only/local", MaxSliceLen::const_new(1 << 16), 0)
        .expect("local producer attaches");
    let catalogs = vec![oracle_catalog(
        "fully_attached_bot",
        &[("/only/local", Some("sensor_msgs/PointCloud2"))],
    )];
    let plane = Arc::new(GatherPlane::new(catalogs));

    let (worker, _flush, _storage) = memory_worker("fully_attached");
    let (socket, dir) = temp_socket("fully_attached");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    let disc = client.request(r#"{"id":1,"method":"discover"}"#);
    assert_eq!(disc["ok"].as_bool(), Some(true));
    let robots = disc["robots"].as_array().expect("robots section present");
    assert_eq!(
        robots.len(),
        1,
        "the fully-attached robot still rows: {robots:?}"
    );
    assert_eq!(robots[0]["robot"].as_str(), Some("fully_attached_bot"));
    assert!(
        robots[0]["topics"].as_array().unwrap().is_empty(),
        "every served topic was local-present → an empty topic list: {robots:?}"
    );
    // The discriminator: NO `error` key (present-but-attached, not refused). `error`
    // is `skip_serializing_if = "Option::is_none"`, so a non-refused robot omits it.
    assert!(
        robots[0].get("error").is_none(),
        "a fully-attached robot is present, NOT refused: {robots:?}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── The uncheck → re-check round-trip (tombstone + catalog auto-resolve) ──────────

#[test]
fn detach_recheck_round_trip_heals_via_the_provenance_tombstone() {
    let _statics = blueprint_statics_guard();
    // HEADLINE: uncheck a remote topic → detach → the shared mirror is torn
    // down by netd → re-check → the Studio sidebar sends a plain `attach` with NO
    // `robot` (its provenance died with the mirror). vizd MUST re-demand from the
    // REMEMBERED robot (the session tombstone) instead of failing as a local topic.
    // Hand oracle: the SECOND (no-robot) attach SUCCEEDS + re-demands the SAME robot.
    let mgr = isolated_transport_with_network("roundtrip");
    let topic = "/utlidar/cloud";
    // The catalog names the topic's type under "ubuntu" (a builtin the walker resolves),
    // so BOTH the first (with-robot) and the healed (no-robot) attaches resolve it.
    let catalogs = vec![oracle_catalog(
        "ubuntu",
        &[(topic, Some("geometry_msgs/Vector3"))],
    )];
    let plane = Arc::new(MirrorSpyPlane::new(Arc::clone(&mgr), catalogs));

    let (worker, _flush, _storage) = memory_worker("roundtrip");
    let (socket, dir) = temp_socket("roundtrip");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // 1. FIRST attach WITH the robot (schema-less) → demand creates the mirror, the tap
    //    succeeds, the provenance tombstone (topic → ubuntu) is recorded.
    let a1 = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}","robot":"ubuntu"}}"#
    ));
    assert_eq!(
        a1["ok"].as_bool(),
        Some(true),
        "first remote attach succeeds: {a1}"
    );

    // 2. detach → release tears the mirror down; WAIT until its service vanishes (the
    //    exact uncheck-tears-down-the-mirror precondition of the re-check).
    let d = client.request(&format!(
        r#"{{"id":2,"method":"detach","topic":"{topic}"}}"#
    ));
    assert_eq!(d["detached"].as_bool(), Some(true));
    assert!(
        wait_until(Duration::from_secs(3), || mgr.data_service_missing(topic)),
        "the mirror service is torn down after detach → the topic is no longer local"
    );

    // 3. SECOND attach with NO `robot` (the sidebar's plain re-check) → HEALS via the
    //    tombstone: re-demands from ubuntu, re-creates the mirror, taps it.
    let a2 = client.request(&format!(
        r#"{{"id":3,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(
        a2["ok"].as_bool(),
        Some(true),
        "the no-robot re-check HEALS via the provenance tombstone: {a2}"
    );
    assert_eq!(a2["topic"].as_str(), Some(topic));

    // Hand oracle: exactly TWO demands (both from ubuntu — the SAME robot, proving the
    // tombstone re-routed the no-robot attach) + ONE release.
    let vector3_hash = <Vector3 as ShmMessage>::SCHEMA_HASH;
    assert_eq!(
        plane.demand_keys(),
        vec![
            ("ubuntu".to_string(), topic.to_string(), vector3_hash),
            ("ubuntu".to_string(), topic.to_string(), vector3_hash),
        ],
        "the healed re-attach re-demands from the REMEMBERED robot"
    );
    assert_eq!(
        plane.release_keys(),
        vec![("ubuntu".to_string(), topic.to_string())]
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn genuine_local_attach_stays_local_on_a_network_configured_daemon() {
    let _statics = blueprint_statics_guard();
    // A topic with a LIVE local producer attaches LOCALLY even on a
    // network-configured daemon — the remote-resolution path is NOT taken (no
    // demand), because the topic is locally VISIBLE. The catalog ALSO names the topic
    // under a "ghost" robot, so this proves the local producer WINS (a locally-visible
    // topic never routes to remote resolution).
    let mgr = isolated_transport_with_network("local");
    let topic = "/local/vel";
    let _pub = Publisher::spawn(Arc::clone(&mgr), topic);
    let catalogs = vec![oracle_catalog(
        "ghost",
        &[(topic, Some("geometry_msgs/Vector3"))],
    )];
    let plane = Arc::new(MirrorSpyPlane::new(Arc::clone(&mgr), catalogs));

    let (worker, _flush, _storage) = memory_worker("local");
    let (socket, dir) = temp_socket("local");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // Wait until the producer's service is visible, then attach with NO robot.
    assert!(wait_until(Duration::from_secs(3), || !mgr.data_service_missing(topic)));
    let a = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(
        a["ok"].as_bool(),
        Some(true),
        "the genuine local topic attaches: {a}"
    );
    // It attached the local producer, NOT a remote mirror — no demand ever happened.
    assert!(
        plane.demand_keys().is_empty(),
        "a locally-visible topic never routes to remote resolution: {:?}",
        plane.demand_keys()
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn no_robot_attach_of_an_unresolvable_topic_is_remote_aware_not_does_not_exist() {
    let _statics = blueprint_statics_guard();
    // A plain `attach` (no robot) of a topic that is neither local nor served
    // by any robot returns a REMOTE-aware error naming the ROBOTS list — NEVER the local
    // graph-YAML "does not exist" (nonsense for a Studio user). The loudness/message-
    // accuracy pin. The catalog serves a DIFFERENT topic, so the target resolves to
    // "no reachable robot".
    //
    // That verdict is a TERMINAL ABSENCE CLAIM, so this test must now DECLARE
    // a settled gather to earn it — the netd really did complete a discovery pass and
    // really found nobody serving the topic. The unconverged twin (where the same
    // inputs must NOT produce this message) is the sibling test below, and this one is
    // its anti-tautology partner.
    let mgr = isolated_transport_with_network("unresolvable");
    let catalogs = vec![oracle_catalog(
        "ubuntu",
        &[("/other/topic", Some("geometry_msgs/Vector3"))],
    )];
    let plane = Arc::new(MirrorSpyPlane::new(Arc::clone(&mgr), catalogs));
    plane.set_discovery(DiscoveryState::Settled);

    let (worker, _flush, _storage) = memory_worker("unresolvable");
    let (socket, dir) = temp_socket("unresolvable");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    let a = client.request(r#"{"id":1,"method":"attach","topic":"/no/such/remote"}"#);
    assert_eq!(a["ok"].as_bool(), Some(false));
    let err = a["error"].as_str().unwrap();
    assert!(
        err.contains("/no/such/remote") && err.contains("no reachable robot"),
        "a remote-aware error names the topic + the missing robot: {a}"
    );
    assert!(
        err.contains("ROBOTS list"),
        "it points the user at the ROBOTS list (the Studio surface): {a}"
    );
    assert!(
        !err.contains("does not exist"),
        "NEVER the local graph-YAML 'does not exist' for a not-locally-visible topic: {a}"
    );
    assert!(
        plane.demand_keys().is_empty(),
        "an unresolvable topic never demands"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn no_robot_attach_under_an_unconverged_gather_is_retry_worthy_not_a_terminal_absence() {
    let _statics = blueprint_statics_guard();
    // The sibling of the sidebar's unconverged-gather fold, at the ATTACH call site: the
    // BYTE-FOR-BYTE same setup as
    // `no_robot_attach_of_an_unresolvable_topic_is_remote_aware_not_does_not_exist`
    // — same manager, same catalog serving a DIFFERENT topic, same request — with
    // exactly ONE variable changed: netd reports its gather as NOT CONVERGED.
    //
    // That is the cold-start shape a desk hits constantly: `cerulion-netd` is
    // spawn-once and its first discovery pass takes seconds, so the FIRST attach after
    // a desk boots races it. Answering "no reachable robot serves it" there renders a
    // confident, terminal "not found" for a topic that exists, at the exact moment the
    // user asked for it — and nothing on that surface tells them to try again.
    //
    // The pair is the whole oracle: identical inputs, one flag, opposite verdicts.
    let mgr = isolated_transport_with_network("unconverged");
    let catalogs = vec![oracle_catalog(
        "ubuntu",
        &[("/other/topic", Some("geometry_msgs/Vector3"))],
    )];
    let plane = Arc::new(MirrorSpyPlane::new(Arc::clone(&mgr), catalogs));
    // The ONE variable. (`MirrorSpyPlane` defaults to this; stated explicitly because
    // the whole test is about which state is declared.)
    plane.set_discovery(DiscoveryState::NotConverged);

    let (worker, _flush, _storage) = memory_worker("unconverged");
    let (socket, dir) = temp_socket("unconverged");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    let a = client.request(r#"{"id":1,"method":"attach","topic":"/no/such/remote"}"#);
    // Still an error — the attach genuinely cannot be completed yet. What changes is
    // WHAT IT CLAIMS.
    assert_eq!(a["ok"].as_bool(), Some(false));
    let err = a["error"].as_str().unwrap();
    assert!(
        err.contains("/no/such/remote"),
        "still names the topic the user asked for: {a}"
    );
    assert!(
        err.contains("nothing on the network answered cerulion-netd"),
        "says WHY this is not an absence — netd read nothing from the network: {a}"
    );
    assert!(
        err.contains("is UNKNOWN (this is not a claim that it is missing)"),
        "states the verdict as UNKNOWN rather than absence: {a}"
    );
    // RETRY-WORTHY, and HEDGED. `DiscoveryMaturity` latches `settled`
    // only on a non-empty gather and never resets, so a desk that never hears a
    // robot (robot-less dev machine, robot on the wrong VLAN) is NotConverged for
    // the daemon's whole life — an unqualified "retry in a moment" would be
    // permanently unactionable there, turning a correct reachability diagnosis
    // into wrong timing advice.
    assert!(
        // Reworded — see `daemon::RETRY_CLAUSE`. The advice survives.
        err.contains("Retry in a few seconds"),
        "offers the retry: {a}"
    );
    assert!(
        !err.contains("within a second or two"),
        "never promises sub-second convergence — measured ~3-13s: {a}"
    );
    assert!(
        err.contains("check the robot is powered on and on this network"),
        "and names the checks for the case where it never converges: {a}"
    );
    // THE regression pin: the terminal claim the settled twin makes must be absent.
    assert!(
        !err.contains("no reachable robot"),
        "NEVER a confident absence off a gather netd has not finished: {a}"
    );
    assert!(
        !err.contains("does not exist"),
        "and never the local graph-YAML error either: {a}"
    );
    assert!(
        plane.demand_keys().is_empty(),
        "an unresolved topic never demands, converged or not: {:?}",
        plane.demand_keys()
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Test 11 — client disconnect mid-request + handle reaping ────

#[test]
fn client_disconnect_mid_request_keeps_daemon_healthy_and_reaps_handles() {
    let mgr = isolated_transport("vizd_disconnect");
    let (worker, _flush, _storage) = memory_worker("disconnect");
    let (socket, dir) = temp_socket("disconnect");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None, // rerun_url: a memory sink hosts no endpoint
    )
    .expect("daemon starts");

    // Churn: many controllers connect, fire a request, and DISCONNECT immediately
    // (mid-exchange) — the client-death path (a write to the gone peer + the
    // handler thread ending).
    const CHURN: usize = 12;
    for _ in 0..CHURN {
        let mut stream = connect_bounded(&socket);
        writeln!(stream, r#"{{"id":1,"method":"list"}}"#).expect("write request");
        drop(stream); // disconnect immediately
        std::thread::sleep(Duration::from_millis(15)); // let the handler finish (EOF)
    }

    // The daemon SURVIVED every mid-request drop: a fresh controller still works.
    let mut client = Client::connect(&socket);
    assert_eq!(
        client.request(r#"{"id":99,"method":"list"}"#)["ok"].as_bool(),
        Some(true)
    );

    // Reaping shrinks the handle set: CHURN+ connections were opened, but the
    // accept loop's `retain(is_finished)` prunes finished handlers on every
    // accept, so the live handle count stays a SMALL bound (nowhere near CHURN).
    // Opening a throwaway connection forces the reaping accept.
    assert!(
        wait_until(Duration::from_secs(3), || {
            let _throwaway = connect_bounded(&socket); // triggers an accept → reap
            std::thread::sleep(Duration::from_millis(30));
            daemon.live_connection_handle_count() <= 2
        }),
        "finished connection handles are reaped (count {} should be ≤2 after churning {CHURN})",
        daemon.live_connection_handle_count()
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Test 12 — a never-reading controller cannot wedge shutdown ──

#[test]
fn never_reading_controller_does_not_wedge_shutdown() {
    let mgr = isolated_transport("vizd_wedge");
    let (worker, _flush, _storage) = memory_worker("wedge");
    let (socket, dir) = temp_socket("wedge");
    let daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None, // rerun_url: a memory sink hosts no endpoint
    )
    .expect("daemon starts");

    // A hostile controller: connect, fire a LARGE batch of requests, and NEVER
    // read the responses. The daemon's send buffer fills → without a WRITE
    // timeout its handle_connection would block in writeln! FOREVER and the
    // shutdown join would never complete.
    let socket_c = socket.clone();
    let bad = std::thread::spawn(move || {
        let mut stream = connect_bounded(&socket_c);
        let line = b"{\"id\":1,\"method\":\"list\"}\n";
        for _ in 0..200_000 {
            if stream.write_all(line).is_err() {
                break; // the daemon closed the socket (shutdown) → done.
            }
        }
        let _keep = stream; // never read; hold the connection until teardown.
    });

    // Let the buffers fill so the daemon is genuinely wedged on write.
    std::thread::sleep(Duration::from_millis(800));

    // shutdown() must complete within a BOUND despite the never-reading
    // controller (the write timeout turns the blocked write into an error → the
    // handler returns → the join completes). Run it under a watchdog so a
    // REGRESSION (no write timeout → an unbounded join) FAILS cleanly instead of
    // hanging CI.
    let (tx, rx) = std::sync::mpsc::channel();
    let shutdown_thread = std::thread::spawn(move || {
        let mut daemon = daemon;
        let t0 = Instant::now();
        daemon.shutdown();
        let _ = tx.send(t0.elapsed());
        daemon // keep alive until joined (Drop is idempotent)
    });
    match rx.recv_timeout(Duration::from_secs(8)) {
        Ok(elapsed) => assert!(
            elapsed < Duration::from_secs(6),
            "shutdown completed but took too long ({elapsed:?}) — write timeout not effective"
        ),
        Err(_) => panic!(
            "daemon.shutdown() did not complete within 8s — a never-reading controller wedged \
             teardown (the CONN_WRITE_TIMEOUT regressed)"
        ),
    }
    let _ = shutdown_thread.join();
    let _ = bad.join();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Test 13 — an oversized unterminated request line drops ─
// the connection and the daemon SURVIVES (the `pending` buffer is bounded).

#[test]
fn oversized_request_line_drops_connection_and_daemon_survives() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("vizd_oversized");
    let topic = "/vizd/live";
    let _pub = Publisher::spawn(Arc::clone(&mgr), topic);
    let (worker, _flush, _storage) = memory_worker("oversized");
    let (socket, dir) = temp_socket("oversized");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None, // rerun_url: a memory sink hosts no endpoint
    )
    .expect("daemon starts");

    // A hostile/buggy controller: connect, read the banner, then stream WAY more
    // than the cap of bytes with NO newline. Without the cap `pending` grows UNBOUNDED and
    // the daemon keeps draining, so the ONLY way the connection ends is the client
    // giving up — the flooder writes the whole (way-over-cap) limit with NO write
    // error. With the cap the daemon PROACTIVELY drops the connection at the cap
    // (structured error + close), so the flooder's write fails LONG before the
    // limit. That proactive mid-stream close is attributable ONLY to the cap (an
    // EOF-driven close needs the CLIENT to close first; a malformed line never
    // closes) — the mutation kill: reverting the cap check makes this assert fail
    // (the flooder reaches `limit`, `hit_write_error` stays false).
    let mut hit_write_error = false;
    {
        let stream = connect_bounded(&socket);
        let mut writer = stream.try_clone().expect("clone stream");
        let mut reader = BufReader::new(stream);
        let mut banner = String::new();
        reader.read_line(&mut banner).expect("read banner");

        let chunk = vec![b'a'; 64 * 1024]; // NO newline anywhere
        let limit = 8 * MAX_REQUEST_LINE_BYTES; // WAY over the cap
        let mut written = 0usize;
        while written < limit {
            if writer.write_all(&chunk).is_err() || writer.flush().is_err() {
                hit_write_error = true; // the daemon dropped us MID-flood (the cap).
                break;
            }
            written += chunk.len();
        }
        assert!(
            hit_write_error,
            "the daemon must PROACTIVELY drop an unterminated over-cap flood: wrote \
             {written} of {limit} bytes with NO write error, so the connection was \
             never dropped mid-stream — the cap did not fire (unbounded-buffer OOM risk)"
        );
        assert!(
            written < limit,
            "the drop happened before the flooder reached the limit ({written} < {limit})"
        );
        // Best-effort: the structured refusal may or may not survive the close (a
        // socket RST on a peer with unread data can discard it), so assert it ONLY
        // if it arrives — the load-bearing signal is the proactive close above.
        let mut err_line = String::new();
        if reader.read_line(&mut err_line).is_ok() && !err_line.trim().is_empty() {
            let v: Value = serde_json::from_str(&err_line).expect("error json");
            assert_eq!(v["ok"].as_bool(), Some(false), "oversized → error: {v}");
            assert!(
                v["error"].as_str().unwrap_or("").contains("exceeded"),
                "the cap refusal is explicit + actionable: {v}"
            );
        }
    }

    // The daemon SURVIVED the flood: a FRESH controller connects, gets the banner,
    // and a real request round-trips end-to-end.
    let mut client = Client::connect(&socket);
    assert_eq!(client.banner["protocol"].as_u64(), Some(1));
    let att = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(
        att["ok"].as_bool(),
        Some(true),
        "fresh controller served after the flood dropped one connection: {att}"
    );
    assert_eq!(att["topic"].as_str(), Some(topic));

    // Control: a normal-size request on ANOTHER fresh connection still serves
    // (the daemon is not wedged; only the offending connection was dropped).
    let mut c2 = Client::connect(&socket);
    let list = c2.request(r#"{"id":2,"method":"list"}"#);
    assert_eq!(list["ok"].as_bool(), Some(true));
    assert_eq!(list["id"].as_u64(), Some(2));

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Test 14 — anti-tautology: a large-but-newline-terminated ─
// request JUST UNDER the cap still parses + serves (the cap does not reject a
// legitimate large line — only unbounded no-newline growth).

#[test]
fn large_but_newline_terminated_request_under_cap_serves() {
    let mgr = isolated_transport("vizd_undercap");
    let (worker, _flush, _storage) = memory_worker("undercap");
    let (socket, dir) = temp_socket("undercap");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None, // rerun_url: a memory sink hosts no endpoint
    )
    .expect("daemon starts");

    let mut client = Client::connect(&socket);

    // A VALID discover request padded with an IGNORED field (forward-compatible —
    // unknown fields are dropped) to a line length just UNDER the cap. It must
    // parse + serve exactly like a tiny discover (proving the cap is a floor on
    // unbounded no-newline growth, not a ceiling on legitimate line size).
    let prefix = r#"{"id":77,"method":"discover","pad":""#;
    let suffix = r#""}"#;
    let pad_len = MAX_REQUEST_LINE_BYTES - prefix.len() - suffix.len() - 64; // clearly under
    let line = format!("{prefix}{}{suffix}", "a".repeat(pad_len));
    assert!(
        line.len() < MAX_REQUEST_LINE_BYTES,
        "the padded line is under the cap ({} < {})",
        line.len(),
        MAX_REQUEST_LINE_BYTES
    );
    let resp = client.request(&line);
    assert_eq!(
        resp["ok"].as_bool(),
        Some(true),
        "a large-but-valid line under the cap serves: {resp}"
    );
    assert_eq!(resp["id"].as_u64(), Some(77), "the id is echoed: {resp}");
    assert!(
        resp["topics"].is_array(),
        "a discover response shape: {resp}"
    );

    // The connection is NOT dropped — a later request still works.
    let list = client.request(r#"{"id":78,"method":"list"}"#);
    assert_eq!(list["ok"].as_bool(), Some(true));
    assert_eq!(list["id"].as_u64(), Some(78));

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Test 15 — a valid request arriving in MULTIPLE partial ──
// writes (newline only in the last chunk) still parses (the cap machinery does
// not break legitimate cross-read buffering).

#[test]
fn request_split_across_partial_writes_still_parses() {
    let mgr = isolated_transport("vizd_partial");
    let (worker, _flush, _storage) = memory_worker("partial");
    let (socket, dir) = temp_socket("partial");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None, // rerun_url: a memory sink hosts no endpoint
    )
    .expect("daemon starts");

    let stream = connect_bounded(&socket);
    let mut writer = stream.try_clone().expect("clone stream");
    let mut reader = BufReader::new(stream);
    let mut banner = String::new();
    reader.read_line(&mut banner).expect("read banner");

    // Write ONE valid `list` request in several partial chunks with the newline
    // ONLY in the last chunk, small sleeps between → each chunk is a distinct
    // daemon `read()` (the cap check runs after each, seeing a small under-cap
    // `pending`), so the request is buffered across reads and only served on the
    // terminating newline.
    for part in [r#"{"id":"#, r#"55,"me"#, r#"thod":"#, r#""list"}"#, "\n"] {
        writer.write_all(part.as_bytes()).expect("partial write");
        writer.flush().expect("flush");
        std::thread::sleep(Duration::from_millis(30));
    }

    let mut resp = String::new();
    reader.read_line(&mut resp).expect("read response");
    let v: Value = serde_json::from_str(&resp).expect("response json");
    assert_eq!(
        v["ok"].as_bool(),
        Some(true),
        "a request split across reads still parses: {v}"
    );
    assert_eq!(v["id"].as_u64(), Some(55), "the id is echoed: {v}");

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Test 16 (layout verb) — set_blueprint validates, applies, renders ─────────
// + soft-warns end-to-end over the real daemon dispatch + worker + memory sink.

#[test]
fn set_blueprint_verb_validates_applies_and_soft_warns_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("vizd_setbp");
    let topic = "/vizd/bp";
    // A SILENT publisher: the topic EXISTS + is attachable (so its render entity
    // GROUNDS the soft-warn), but NO data flows — so any num_msgs growth after a
    // set_blueprint is attributable to the blueprint SEND, not sensor frames.
    let _publisher = mgr
        .create_publisher(topic, MaxSliceLen::const_new(1 << 16), 0)
        .expect("silent producer attaches");

    let (worker, flush, storage) = memory_worker("setbp");
    let (socket, dir) = temp_socket("setbp");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");

    let mut client = Client::connect(&socket);

    // Attach the topic so its render entity (world/vizd/bp) grounds a view.
    let att = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(att["ok"].as_bool(), Some(true));
    assert_eq!(att["entity"].as_str(), Some("world/vizd/bp"));

    flush.flush_blocking().ok();
    let baseline = storage.num_msgs();

    // (a) A GROUNDED layout arranging the attached entity → ok, 2 views, not a
    //     reset, NO warnings (both origins grounded), and it RENDERS: num_msgs
    //     grows with NO data flowing → attributable to the blueprint send.
    let layout = r#"{"id":2,"method":"set_blueprint","layout":{"root":{"type":"container","kind":"horizontal","shares":[2.0,1.0],"children":[{"type":"view","kind":"spatial3d","name":"Scene","origin":"world"},{"type":"view","kind":"time_series","name":"Plots","origin":"world/vizd/bp"}]}}}"#;
    let applied = client.request(layout);
    assert_eq!(
        applied["ok"].as_bool(),
        Some(true),
        "the grounded layout applies: {applied}"
    );
    assert_eq!(applied["views"].as_u64(), Some(2));
    assert_eq!(applied["reset"].as_bool(), Some(false));
    assert_eq!(
        applied["warnings"].as_array().unwrap().len(),
        0,
        "grounded origins → no warnings: {applied}"
    );
    assert!(
        wait_until(Duration::from_secs(3), || {
            flush.flush_blocking().ok();
            storage.num_msgs() > baseline
        }),
        "the applied blueprint renders to the sink (num_msgs grows with no data flowing)"
    );

    // (b) A PARTIALLY-ungrounded layout (one grounded view + one whose origin
    //     matches no attached topic) with auto_views ON → ok + a SOFT warning for
    //     the ungrounded origin (never an error — the agent may lay out that view
    //     before attaching its topic; auto_views + the grounded view keep it
    //     useful). A layout where EVERY view is ungrounded is
    //     REFUSED (test 18) — this partial shape stays the soft-warn contract.
    let ghost = r#"{"id":3,"method":"set_blueprint","layout":{"root":{"type":"container","kind":"horizontal","children":[{"type":"view","kind":"time_series","origin":"world/vizd/bp"},{"type":"view","kind":"spatial3d","origin":"world/ghost/none"}]}}}"#;
    let warned = client.request(ghost);
    assert_eq!(
        warned["ok"].as_bool(),
        Some(true),
        "a partially-ungrounded layout still applies (soft-warn, not error): {warned}"
    );
    let ws = warned["warnings"].as_array().expect("warnings array");
    assert!(
        ws.iter().any(|w| w.as_str() == Some("world/ghost/none")),
        "the ungrounded origin is soft-warned: {warned}"
    );

    // (c) An unknown view kind → structured error naming the offender + the set.
    let unknown =
        r#"{"id":4,"method":"set_blueprint","layout":{"root":{"type":"view","kind":"spatial9d"}}}"#;
    let ue = client.request(unknown);
    assert_eq!(ue["ok"].as_bool(), Some(false));
    let msg = ue["error"].as_str().unwrap();
    assert!(
        msg.contains("spatial9d") && msg.contains("spatial3d"),
        "the error names the bad kind + the supported set: {ue}"
    );

    // (d) An empty container → structured error.
    let empty = r#"{"id":5,"method":"set_blueprint","layout":{"root":{"type":"container","kind":"grid","children":[]}}}"#;
    let ee = client.request(empty);
    assert_eq!(ee["ok"].as_bool(), Some(false));
    assert!(
        ee["error"].as_str().unwrap().contains("empty"),
        "the error names the empty container: {ee}"
    );

    // (e) Reset (no layout) → the Go2 default, reset:true. Decision: the
    //     default is Scene-ONLY (a single 3D view — the empty Telemetry + Status panels
    //     are dropped), so a reset restores ONE view.
    let reset = client.request(r#"{"id":6,"method":"set_blueprint"}"#);
    assert_eq!(reset["ok"].as_bool(), Some(true));
    assert_eq!(reset["reset"].as_bool(), Some(true));
    assert_eq!(
        reset["views"].as_u64(),
        Some(1),
        "reset restores the Scene-only Go2 default (1 view): {reset}"
    );

    // The connection SURVIVED every arm — a later request still works.
    let list = client.request(r#"{"id":7,"method":"list"}"#);
    assert_eq!(list["ok"].as_bool(), Some(true));

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Test 17 — the VizControl lifecycle: a set_blueprint must not ────
// wedge shutdown (close() releases the worker exit even with a live Ctx clone).

#[test]
fn set_blueprint_control_does_not_wedge_shutdown() {
    let _statics = blueprint_statics_guard();
    // After a set_blueprint the daemon holds a shared worker-control sender.
    // shutdown() closes it FIRST so the worker reaches Disconnected even though a
    // still-connected controller's Ctx clone holds the Arc<VizControl>. WITHOUT
    // that close() the poll thread's worker-drop join spins to its ~6 s detach;
    // this watchdog at 4 s catches that regression.
    let mgr = isolated_transport("vizd_setbp_shutdown");
    let (worker, _flush, _storage) = memory_worker("setbp_shutdown");
    let (socket, dir) = temp_socket("setbp_shutdown");
    let daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");

    let mut client = Client::connect(&socket);
    let resp = client.request(
        r#"{"id":1,"method":"set_blueprint","layout":{"root":{"type":"view","kind":"spatial3d","origin":"world"}}}"#,
    );
    assert_eq!(
        resp["ok"].as_bool(),
        Some(true),
        "the layout applied: {resp}"
    );

    // Keep the client connected DURING shutdown so the daemon's connection
    // handler thread (a Ctx clone holding the control Arc) is alive while the
    // poll thread drops the worker — exactly the case close() must handle.
    let (tx, rx) = std::sync::mpsc::channel();
    let shutdown_thread = std::thread::spawn(move || {
        let mut daemon = daemon;
        let t0 = Instant::now();
        daemon.shutdown();
        let _ = tx.send(t0.elapsed());
        daemon // keep alive until joined (Drop is idempotent)
    });
    // JOIN BEFORE ASSERTING. This test is the one place a daemon is
    // MOVED into a helper thread, so it does NOT obey the "locals drop in
    // reverse declaration order, hence after `_statics`" rule the rest of the
    // file relies on. A `panic!`/failed `assert!` here would merely DETACH the
    // JoinHandle, leaving a daemon — whose worker teardown writes the blueprint
    // statics from its own thread — running after `_statics` released the lock.
    // The failure that matters most is exactly the one this test exists to
    // catch (a slow shutdown), and on that path the thread
    // HAS finished, so joining first is free.
    let outcome = rx.recv_timeout(Duration::from_secs(8));
    let wedged = matches!(outcome, Err(std::sync::mpsc::RecvTimeoutError::Timeout));
    if !wedged {
        // `Ok` = shutdown reported; `Disconnected` = the thread died without
        // reporting. Both mean it has finished, so this join is prompt.
        let _ = shutdown_thread.join();
    }
    drop(client); // hold the client (a live Ctx clone) until after the join
    let _ = std::fs::remove_dir_all(&dir);
    match outcome {
        Ok(elapsed) => assert!(
            elapsed < Duration::from_secs(4),
            "shutdown must be prompt after a set_blueprint ({elapsed:?}) — close() releases \
             the worker exit even with a live Ctx clone; a ~6 s time means the control was \
             not closed"
        ),
        Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => panic!(
            "the shutdown thread died without reporting an elapsed time — daemon.shutdown() \
             panicked after a set_blueprint"
        ),
        // The one arm that CANNOT join: shutdown is genuinely wedged, so joining
        // would hang the whole binary instead of failing this test. The daemon
        // is abandoned and may write the blueprint statics after this body drops
        // `_statics` — an accepted residual, called out here so a cascade of
        // later blueprint failures is attributable to THIS one.
        Err(std::sync::mpsc::RecvTimeoutError::Timeout) => panic!(
            "daemon.shutdown() did not complete within 8s after a set_blueprint — the VizControl \
             lifecycle wedged teardown (the shutdown thread is ABANDONED, still holding the \
             daemon; later blueprint-static failures in this binary may cascade from it)"
        ),
    }
}

// ── Test 18 — a layout whose views root at GUESSED entity paths is rejected,
// the stage's CURRENT blueprint survives (nothing applied), and a good layout
// still applies. This is the headline guardrail acceptance: set_blueprint refuses
// a PROVABLY-broken layout instead of silently applying it.

#[test]
fn set_blueprint_rejects_the_pt42_broken_layout_and_keeps_the_current_one() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("vizd_guessed");
    let topic = "/vizd/guessedvel";
    // A SILENT publisher: the topic EXISTS + is attachable (so its render entity
    // grounds a GOOD layout), but NO data flows — so any num_msgs growth is
    // attributable ONLY to a blueprint SEND, never sensor frames.
    let _publisher = mgr
        .create_publisher(topic, MaxSliceLen::const_new(1 << 16), 0)
        .expect("silent producer attaches");

    let (worker, flush, storage) = memory_worker("guessed");
    let (socket, dir) = temp_socket("guessed");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");

    let mut client = Client::connect(&socket);

    // Attach the real topic so its entity (world/vizd/guessedvel) is the truth
    // the guardrail checks the guessed origins against.
    let att = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(att["ok"].as_bool(), Some(true));
    assert_eq!(att["entity"].as_str(), Some("world/vizd/guessedvel"));

    // (1) A GOOD grounded layout — the stage's CURRENT blueprint. It applies +
    //     RENDERS (num_msgs grows with no data flowing → attributable to the send).
    flush.flush_blocking().ok();
    let before_good = storage.num_msgs();
    let good = r#"{"id":2,"method":"set_blueprint","layout":{"root":{"type":"view","kind":"time_series","name":"Vel","origin":"world/vizd/guessedvel"}}}"#;
    let applied = client.request(good);
    assert_eq!(
        applied["ok"].as_bool(),
        Some(true),
        "the good layout applies: {applied}"
    );
    assert!(
        wait_until(Duration::from_secs(3), || {
            flush.flush_blocking().ok();
            storage.num_msgs() > before_good
        }),
        "the current (good) blueprint renders to the sink"
    );

    // (2) The broken layout seen live: guessed origins (/cloud, /cmd_vel) that don't
    //     exist (real paths are world/<topic>/...), plus a LaserScan-shaped
    //     spatial2d view at a guessed /scan — the exact shape seen live. Every view is
    //     ungrounded → REFUSED (ok:false) naming the guessed origins + the fix.
    // The baseline is taken at QUIESCENCE, not on the good send's FIRST
    // message — else the good apply's own tail lands inside step (3)'s window and
    // the freeze assertion fails on a loaded box (see `settled_msg_count`).
    let after_good = settled_msg_count(
        &flush,
        &storage,
        Duration::from_millis(250),
        Duration::from_secs(5),
    );
    let guessed_layout = r#"{"id":3,"method":"set_blueprint","layout":{"root":{"type":"container","kind":"horizontal","children":[{"type":"view","kind":"spatial3d","name":"Scene","origin":"/cloud"},{"type":"view","kind":"time_series","name":"Twist","origin":"/cmd_vel"},{"type":"view","kind":"spatial2d","name":"Scan","origin":"/scan"}]}}}"#;
    let rejected = client.request(guessed_layout);
    assert_eq!(
        rejected["ok"].as_bool(),
        Some(false),
        "the all-guessed-origins layout is REJECTED, not applied: {rejected}"
    );
    let err = rejected["error"].as_str().unwrap();
    assert!(
        err.contains("/cloud") && err.contains("/cmd_vel") && err.contains("/scan"),
        "the refusal names every guessed origin: {rejected}"
    );
    assert!(
        err.contains("ungrounded") && err.contains("discover/attach"),
        "the refusal names the symptom + the fix (use real entity paths): {rejected}"
    );

    // (3) NOTHING was applied — the stage's CURRENT (good) blueprint survives: no
    //     new send happened, so num_msgs is stable across a window.
    std::thread::sleep(Duration::from_millis(300));
    flush.flush_blocking().ok();
    assert_eq!(
        storage.num_msgs(),
        after_good,
        "a REJECTED set_blueprint sends nothing — the current blueprint survives"
    );

    // (4) The daemon is still functional after the refusal: another GOOD layout
    //     applies + renders (the refusal did not wedge the verb).
    flush.flush_blocking().ok();
    let before_second = storage.num_msgs();
    let good2 = r#"{"id":4,"method":"set_blueprint","layout":{"root":{"type":"container","kind":"horizontal","shares":[3.0,1.0],"children":[{"type":"view","kind":"spatial3d","name":"Scene","origin":"world"},{"type":"view","kind":"time_series","name":"Vel","origin":"world/vizd/guessedvel"}]}}}"#;
    let applied2 = client.request(good2);
    assert_eq!(
        applied2["ok"].as_bool(),
        Some(true),
        "a good layout still applies after the refusal: {applied2}"
    );
    assert!(
        wait_until(Duration::from_secs(3), || {
            flush.flush_blocking().ok();
            storage.num_msgs() > before_second
        }),
        "the second good blueprint renders (the daemon survived the refusal)"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Test 18b — a partially-ungrounded layout with auto_views ON is
// NOT over-rejected (guardrails reject only PROVABLY-broken shapes): it applies
// with a soft warning, exactly as before the guardrails. The same layout with
// auto_views OFF IS refused (nothing fills the empty view).

#[test]
fn set_blueprint_partial_ungrounded_applies_with_auto_on_refuses_with_auto_off() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("compose_vizd_partial");
    let topic = "/vizd/partialvel";
    let _publisher = mgr
        .create_publisher(topic, MaxSliceLen::const_new(1 << 16), 0)
        .expect("silent producer attaches");
    let (worker, _flush, _storage) = memory_worker("partial");
    let (socket, dir) = temp_socket("partial");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);
    client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
    ));

    // A MIX: one grounded view (world/vizd/partialvel) + one ungrounded
    // (/ghost). auto_views ON (default) → APPLIES with a soft warning (the agent
    // may lay out ahead + auto_views fills the rest) — NOT refused.
    let auto_on = r#"{"id":2,"method":"set_blueprint","layout":{"root":{"type":"container","kind":"horizontal","children":[{"type":"view","kind":"time_series","origin":"world/vizd/partialvel"},{"type":"view","kind":"spatial3d","origin":"/ghost"}]}}}"#;
    let applied = client.request(auto_on);
    assert_eq!(
        applied["ok"].as_bool(),
        Some(true),
        "a partially-ungrounded layout with auto_views ON is NOT over-rejected: {applied}"
    );
    let ws = applied["warnings"].as_array().expect("warnings array");
    assert!(
        ws.iter().any(|w| w.as_str() == Some("/ghost")),
        "the ungrounded origin is soft-warned (not refused): {applied}"
    );

    // The SAME mix with auto_views OFF → REFUSED (nothing fills the empty view),
    // naming the ungrounded origin + the nearest attached entity.
    let auto_off = r#"{"id":3,"method":"set_blueprint","layout":{"auto_views":false,"root":{"type":"container","kind":"horizontal","children":[{"type":"view","kind":"time_series","origin":"world/vizd/partialvel"},{"type":"view","kind":"spatial3d","origin":"/ghost"}]}}}"#;
    let refused = client.request(auto_off);
    assert_eq!(
        refused["ok"].as_bool(),
        Some(false),
        "the same mix with auto_views OFF is refused: {refused}"
    );
    let err = refused["error"].as_str().unwrap();
    assert!(
        err.contains("/ghost")
            && err.contains("auto_views")
            && err.contains("world/vizd/partialvel"),
        "the refusal names the ungrounded origin, auto_views, + the nearest entity: {refused}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Test 18c (guardrail 2) — a GROUNDED LaserScan placed in a
// spatial2d view is REFUSED (a LaserScan projects to Points3D and shows
// NOTHING in 2D). Uses a REMOTE attach (schema pinned to sensor_msgs/LaserScan) so
// the archetype resolves via classify_schema without a frame, then a spatial2d
// view grounding that topic trips the guardrail. A spatial3d view over the SAME
// topic is the anti-tautology control (the correct placement applies).

#[test]
fn set_blueprint_rejects_a_grounded_laserscan_in_a_spatial2d_view() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport_with_network("vizd_scan2d");
    let (mut netd, netd_sock, netd_dir) = start_in_process_netd("scan2d", Arc::clone(&mgr));
    let (worker, _flush, _storage) = memory_worker("scan2d");
    let (socket, dir) = temp_socket("scan2d");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::new(NetdDemandPlane::with_socket(netd_sock)),
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // A remote attach with a KNOWN pinned schema (sensor_msgs/LaserScan → LaserScan
    // in classify_schema): the daemon resolves the hash locally, demands the mirror
    // from netd, taps it, and reports the LaserScan archetype WITHOUT a frame.
    let topic = "/go2net/scan";
    let att = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}","robot":"go2","schema":"sensor_msgs/LaserScan"}}"#
    ));
    assert_eq!(
        att["ok"].as_bool(),
        Some(true),
        "remote LaserScan attach: {att}"
    );
    assert_eq!(att["archetype"].as_str(), Some("LaserScan"));
    assert_eq!(att["entity"].as_str(), Some("world/go2net/scan"));
    // The deterministic placement already tells the agent LaserScan →
    // spatial3d (never spatial2d) — the guardrail below is the enforcement.
    assert_eq!(att["view_kinds"], serde_json::json!(["spatial3d"]));

    // Placing that grounded LaserScan in a spatial2d view (its ONLY grounded topic)
    // is REFUSED — the pane would render NOTHING.
    let scan_2d = r#"{"id":2,"method":"set_blueprint","layout":{"root":{"type":"view","kind":"spatial2d","name":"Scan","origin":"world/go2net/scan"}}}"#;
    let refused = client.request(scan_2d);
    assert_eq!(
        refused["ok"].as_bool(),
        Some(false),
        "a LaserScan in a spatial2d view is refused: {refused}"
    );
    let err = refused["error"].as_str().unwrap();
    assert!(
        err.contains(topic) && err.contains("LaserScan") && err.contains("spatial3d"),
        "the refusal names the topic, archetype, + the valid view kind it renders in: {refused}"
    );
    assert!(
        err.contains("spatial2d") && err.contains("render nothing"),
        "the refusal names the 2D placement + the render-nothing symptom: {refused}"
    );

    // Anti-tautology control: the SAME topic in a spatial3d view (the CORRECT
    // placement) APPLIES — the guardrail does not over-reject.
    let scan_3d = r#"{"id":3,"method":"set_blueprint","layout":{"root":{"type":"view","kind":"spatial3d","name":"Scan","origin":"world/go2net/scan"}}}"#;
    let applied = client.request(scan_3d);
    assert_eq!(
        applied["ok"].as_bool(),
        Some(true),
        "the LaserScan in a spatial3d view (correct) applies: {applied}"
    );

    // MIXED-boundary arm (fix #2): attach an Image sibling (world/go2net/cam),
    // then a broadly-rooted spatial2d view at world/go2net grounds BOTH the valid
    // Image AND the invisible LaserScan → it APPLIES (renders the image) with a SOFT
    // WARNING naming the invisible scan — NOT a refusal. The old boundary wrongly
    // refused this near-universal real-robot shape.
    let cam = "/go2net/cam";
    let a_cam = client.request(&format!(
        r#"{{"id":4,"method":"attach","topic":"{cam}","robot":"go2","schema":"sensor_msgs/Image"}}"#
    ));
    assert_eq!(
        a_cam["ok"].as_bool(),
        Some(true),
        "remote Image attach: {a_cam}"
    );
    assert_eq!(a_cam["archetype"].as_str(), Some("Image"));
    assert_eq!(a_cam["entity"].as_str(), Some("world/go2net/cam"));

    let mixed = r#"{"id":5,"method":"set_blueprint","layout":{"root":{"type":"view","kind":"spatial2d","name":"Sensors","origin":"world/go2net"}}}"#;
    let mixed_resp = client.request(mixed);
    assert_eq!(
        mixed_resp["ok"].as_bool(),
        Some(true),
        "a spatial2d view grounding a valid Image + an invisible LaserScan APPLIES: {mixed_resp}"
    );
    let ws = mixed_resp["warnings"].as_array().expect("warnings array");
    assert!(
        ws.iter().any(|w| {
            let s = w.as_str().unwrap_or("");
            s.contains("/go2net/scan") && s.contains("LaserScan") && s.contains("spatial3d")
        }),
        "the invisible LaserScan sibling is SOFT-warned (not refused): {mixed_resp}"
    );

    daemon.shutdown();
    netd.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
    let _ = std::fs::remove_dir_all(&netd_dir);
}

// ── Test 19 (guardrail 5) — attach→detach→re-attach ×N on one topic with
// a CONSISTENT entity_override yields the IDENTICAL render entity every cycle (the
// stable-mapping half). The live zombie came from inconsistent overrides
// accumulating data under a shifting root; a consistent override must never drift.

#[test]
fn attach_detach_reattach_with_consistent_override_is_stable() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("vizd_stable");
    let topic = "/vizd/stable";
    let _publisher = mgr
        .create_publisher(topic, MaxSliceLen::const_new(1 << 16), 0)
        .expect("silent producer attaches");
    let (worker, _flush, _storage) = memory_worker("stable");
    let (socket, dir) = temp_socket("stable");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // A consistent override "wrist" → the SAME render entity every cycle (hand
    // oracle: the fallthrough leaf route).
    const EXPECT: &str = "world/wrist";
    let attach = format!(r#"{{"id":1,"method":"attach","topic":"{topic}","entity":"wrist"}}"#);
    let first = client.request(&attach);
    assert_eq!(first["ok"].as_bool(), Some(true), "{first}");
    assert_eq!(first["entity"].as_str(), Some(EXPECT));
    assert_eq!(first["already_attached"].as_bool(), Some(false));

    // Cycle detach → re-attach with the SAME override N times: the entity is
    // IDENTICAL every cycle, and each re-attach opens a FRESH tap (the detach
    // cleared the prior mapping, so already_attached is false again).
    for cycle in 0..4 {
        let det = client.request(&format!(
            r#"{{"id":{},"method":"detach","topic":"{topic}"}}"#,
            100 + cycle
        ));
        assert_eq!(
            det["detached"].as_bool(),
            Some(true),
            "cycle {cycle}: {det}"
        );
        let re = client.request(&format!(
            r#"{{"id":{},"method":"attach","topic":"{topic}","entity":"wrist"}}"#,
            200 + cycle
        ));
        assert_eq!(re["ok"].as_bool(), Some(true), "cycle {cycle}: {re}");
        assert_eq!(
            re["entity"].as_str(),
            Some(EXPECT),
            "cycle {cycle}: the entity is stable across attach/detach/re-attach"
        );
        assert_eq!(
            re["already_attached"].as_bool(),
            Some(false),
            "cycle {cycle}: a detach cleared the mapping → a fresh tap"
        );
        // list agrees on the stable entity.
        let list = client.request(&format!(r#"{{"id":{},"method":"list"}}"#, 300 + cycle));
        let le = entry_for(&list["attached"], "topic", topic).expect("listed");
        assert_eq!(
            le["entity"].as_str(),
            Some(EXPECT),
            "cycle {cycle}: list entity stable"
        );
    }

    // A same-override re-attach WITHOUT a detach is idempotent (already_attached +
    // stable entity), never a second tap.
    let idem = client.request(&format!(
        r#"{{"id":9,"method":"attach","topic":"{topic}","entity":"wrist"}}"#
    ));
    assert_eq!(idem["already_attached"].as_bool(), Some(true), "{idem}");
    assert_eq!(idem["entity"].as_str(), Some(EXPECT));

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Test 20 (guardrail 5) — a re-attach with a CONFLICTING override is
// REFUSED loudly and the ORIGINAL mapping is intact (the refusal arm); a detach
// then re-point to the new override succeeds (detach clears the mapping).

#[test]
fn attach_with_conflicting_override_is_refused_original_intact() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("vizd_conflict");
    let topic = "/vizd/conflict";
    let _publisher = mgr
        .create_publisher(topic, MaxSliceLen::const_new(1 << 16), 0)
        .expect("silent producer attaches");
    let (worker, _flush, _storage) = memory_worker("conflict");
    let (socket, dir) = temp_socket("conflict");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // First attach under override A ("wrist") → world/wrist.
    let a = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}","entity":"wrist"}}"#
    ));
    assert_eq!(a["ok"].as_bool(), Some(true));
    assert_eq!(a["entity"].as_str(), Some("world/wrist"));

    // Re-attach (NO detach) under override B ("elbow") → REFUSED, naming the topic,
    // the live path, the conflicting requested path, and the fix.
    let b = client.request(&format!(
        r#"{{"id":2,"method":"attach","topic":"{topic}","entity":"elbow"}}"#
    ));
    assert_eq!(
        b["ok"].as_bool(),
        Some(false),
        "a conflicting override is refused: {b}"
    );
    assert_eq!(b["topic"].as_str(), Some(topic));
    let err = b["error"].as_str().unwrap();
    assert!(
        err.contains(topic) && err.contains("world/wrist") && err.contains("world/elbow"),
        "the refusal names the topic + both entity paths: {b}"
    );
    assert!(
        err.contains("detach") && err.contains("reuse"),
        "the refusal names the fix: {b}"
    );

    // The ORIGINAL mapping is intact — list still shows override A's entity.
    let list = client.request(r#"{"id":3,"method":"list"}"#);
    let le = entry_for(&list["attached"], "topic", topic).expect("still attached");
    assert_eq!(
        le["entity"].as_str(),
        Some("world/wrist"),
        "the refused re-point left the original mapping untouched: {list}"
    );

    // After a DETACH the mapping is cleared, so re-pointing to override B succeeds.
    let det = client.request(&format!(
        r#"{{"id":4,"method":"detach","topic":"{topic}"}}"#
    ));
    assert_eq!(det["detached"].as_bool(), Some(true));
    let repoint = client.request(&format!(
        r#"{{"id":5,"method":"attach","topic":"{topic}","entity":"elbow"}}"#
    ));
    assert_eq!(
        repoint["ok"].as_bool(),
        Some(true),
        "a detach clears the mapping → re-pointing to a new override is allowed: {repoint}"
    );
    assert_eq!(repoint["entity"].as_str(), Some("world/elbow"));

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Test 21 — compose_layout attaches + compiles + applies + ──
// renders over the live daemon; attach_missing really attaches.

#[test]
fn compose_layout_attaches_compiles_applies_and_renders_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("d_compose_headline");
    let topic = "/vizd/telemetry";
    // A LIVE Vector3 publisher → the sink resolves it to Scalars → time_series.
    let _pub = Publisher::spawn(Arc::clone(&mgr), topic);

    let (worker, flush, storage) = memory_worker("compose_headline");
    let (socket, dir) = temp_socket("compose_headline");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // The topic is NOT pre-attached. compose_layout with a BARE topic list
    // (attach_missing + include_robot default true) attaches it, compiles a
    // grounded blueprint, and applies it.
    flush.flush_blocking().ok();
    let baseline = storage.num_msgs();
    let resp = client.request(&format!(
        r#"{{"id":1,"method":"compose_layout","intent":{{"topics":["{topic}"]}}}}"#
    ));
    assert_eq!(resp["ok"].as_bool(), Some(true), "compose applies: {resp}");
    // Scalars → time_series (plots); include_robot → a hero (robot) beside plots.
    assert_eq!(resp["arrangement"].as_str(), Some("hero_sidebar"), "{resp}");
    assert_eq!(
        resp["views"].as_u64(),
        Some(2),
        "hero (robot) + plots: {resp}"
    );
    // attach_missing attached it (echoed in the response).
    assert_eq!(
        resp["attached"],
        serde_json::json!([topic]),
        "attach_missing echoes the newly-attached topic: {resp}"
    );
    // The placement echo carries topic + entity + archetype + view + components —
    // the deterministic answer the agent never has to guess.
    let placements = resp["placements"].as_array().expect("placements array");
    assert_eq!(placements.len(), 1, "one placement: {resp}");
    let p = &placements[0];
    assert_eq!(p["topic"].as_str(), Some(topic));
    assert_eq!(p["entity"].as_str(), Some("world/vizd/telemetry"));
    assert_eq!(p["archetype"].as_str(), Some("Scalars"));
    assert_eq!(p["view"].as_str(), Some("time_series"));
    assert_eq!(p["components"], serde_json::json!(["Scalars"]));
    // The composed layout SURVIVED the PR-C validation (ok:true) → no warnings.
    assert!(
        resp["warnings"].as_array().unwrap().is_empty(),
        "a grounded compiler output has no soft warnings: {resp}"
    );

    // attach_missing REALLY attached it → a later list shows the tap.
    let list = client.request(r#"{"id":2,"method":"list"}"#);
    assert!(
        entry_for(&list["attached"], "topic", topic).is_some(),
        "attach_missing attached the topic: {list}"
    );

    // The composed blueprint RENDERS (num_msgs grows — the blueprint send + the
    // live Vector3 frames both land on the sink).
    assert!(
        wait_until(Duration::from_secs(3), || {
            flush.flush_blocking().ok();
            storage.num_msgs() > baseline
        }),
        "the composed layout renders to the sink"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Test 21b — the APPLIED compose blueprint carries the trailing ──
// plot window (time_series view) + the stage background (spatial view), at the
// EXACT view property paths the viewer reads them from.
//
// Round-trip proof: the emitted chunks land at `view/<uuid>/VisibleTimeRanges` and
// `view/<uuid>/Background` — the paths `re_viewport_blueprint::ViewBlueprint::query_range`
// (window, matched by timeline name) and `re_view_spatial::configure_background`
// (background) read from (cited in `blueprint.rs`; the SERIALIZED window values are
// hand-pinned in `blueprint.rs`'s `trailing_window_ranges_*`). PARALLEL-SAFE: it
// inspects only THIS daemon's dedicated memory sink for the PRESENCE of decorated
// properties, which this test's decorated compose produces — a concurrent daemon
// writes its own blueprint to ITS OWN sink, never this one, so no false positive and
// no statics lock is needed. (The built-in default send-once is decorated with the
// stage Background; as of the Scene-only default it carries NO window/axis, so the
// window+axis presence checks below can only be satisfied by the compose — but the
// Background could still be a leftover default, so the drain-to-quiescence below keeps
// this a compose-provenance assertion, not a mere presence one.)

#[test]
fn composed_blueprint_carries_the_trailing_window_and_stage_background_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("f_decorate");
    let topic = "/vizd/telemetry_f";
    // A LIVE Vector3 publisher → Scalars → a time_series plots view; include_robot
    // (default true) adds a spatial3d hero → BOTH a windowed plot AND a backgrounded
    // spatial view in one compose.
    let _pub = Publisher::spawn(Arc::clone(&mgr), topic);

    let (worker, flush, storage) = memory_worker("compose_decorate_f");
    let (socket, dir) = temp_socket("compose_decorate_f");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // PROVENANCE ISOLATION (must precede the compose): the DEFAULT send-once,
    // fired ONCE at worker spawn (`ensure_setup` → `send_blueprint_once`, guarded so it
    // never re-fires), decorates its Scene-only scene with the `#10161f` Background (as
    // of the Scene-only default it carries NO time_series window/axis). Left in the sink, that default
    // Background would let the background presence gate + value loops below pass EVEN IF
    // compose stopped decorating (a reproducible false-pass). So
    // DRAIN the sink to
    // quiescence here — the default blueprint is already flushable (it was sent before
    // the client connected) — and only THEN compose. Because the send-once fires exactly
    // once, nothing default-shaped can re-enter afterwards, so everything accumulated
    // after the compose request is the COMPOSE's blueprint ALONE. The gate below
    // FAILS if compose stops decorating while the default still decorates.
    let mut consecutive_empty = 0;
    wait_until(Duration::from_secs(2), || {
        flush.flush_blocking().ok();
        if storage.take().is_empty() {
            consecutive_empty += 1;
        } else {
            consecutive_empty = 0;
        }
        consecutive_empty >= 3
    });

    let resp = client.request(&format!(
        r#"{{"id":1,"method":"compose_layout","intent":{{"topics":["{topic}"]}}}}"#
    ));
    assert_eq!(resp["ok"].as_bool(), Some(true), "compose applies: {resp}");
    assert_eq!(
        resp["arrangement"].as_str(),
        Some("hero_sidebar"),
        "hero (robot, spatial3d) + plots (time_series): {resp}"
    );

    // The worker applies SetBlueprint on its own thread → accumulate the drained
    // blueprint LogMsgs from THIS sink (the default was drained above, so these are the
    // COMPOSE's chunks) and decode them until BOTH decorated properties appear at their
    // view property paths — the exact VALUES the composed view carries are pinned below.
    let mut msgs = Vec::new();
    let decorated = wait_until(Duration::from_secs(5), || {
        flush.flush_blocking().ok();
        msgs.extend(storage.take());
        let paths = blueprint_property_paths(&msgs);
        paths.iter().any(|p| p.ends_with("/VisibleTimeRanges"))
            && paths.iter().any(|p| p.ends_with("/TimeAxis"))
            && paths.iter().any(|p| p.ends_with("/Background"))
    });
    assert!(
        decorated,
        "the applied COMPOSE blueprint carries a trailing (query) window + display x-axis \
         (time_series) AND a stage background (spatial view) at their view property paths (the \
         default was drained first, so this pins the compose's own decoration)"
    );

    // Decode the applied blueprint's COMPONENT VALUES (not just the
    // paths) — the SolidColor #10161f stage background + the [-30s, 0] cursor-relative
    // window are exactly what the viewer reads.
    let dec = blueprint_decorations(&msgs);
    assert!(
        !dec.backgrounds.is_empty(),
        "at least one decoded background: {dec:?}"
    );
    for bg in &dec.backgrounds {
        assert_eq!(
            bg.kind_names(),
            vec!["SolidColor".to_string()],
            "SolidColor kind: {bg:?}"
        );
        assert_eq!(
            bg.colors,
            vec![[0x10, 0x16, 0x1f, 0xff]],
            "stage color #10161f: {bg:?}"
        );
    }
    assert!(
        !dec.windows.is_empty(),
        "at least one decoded window: {dec:?}"
    );
    for w in &dec.windows {
        assert_eq!(
            w.timelines(),
            vec!["robot_time".to_string(), "log_time".to_string()],
            "both nanosecond timelines: {w:?}"
        );
        assert_eq!(
            w.cursor_relative_ns(),
            vec![
                (Some(-30_000_000_000), Some(0)),
                (Some(-30_000_000_000), Some(0))
            ],
            "cursor-relative [-30s, 0]: {w:?}"
        );
    }
    // The DISPLAY x-axis window (the empty-epoch-axis fix) also rides through the
    // real daemon → worker → memory sink, cursor-relative [-30s, 0], one range.
    assert!(
        !dec.time_axes.is_empty(),
        "at least one decoded display x-axis: {dec:?}"
    );
    for ta in &dec.time_axes {
        assert_eq!(
            ta.cursor_relative_ns(),
            vec![(Some(-30_000_000_000), Some(0))],
            "cursor-relative [-30s, 0], one timeline-agnostic range: {ta:?}"
        );
    }

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Test 22 — attach_missing:false attaches NOTHING; nothing to
// place → a loud NoRenderableTopics refusal (the compose-over-zero contract).

#[test]
fn compose_layout_attach_missing_false_attaches_nothing_and_refuses_when_empty() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("d_compose_nomissing");
    let topic = "/vizd/nomissing";
    let _pub = Publisher::spawn(Arc::clone(&mgr), topic);

    let (worker, _flush, _storage) = memory_worker("compose_nomissing");
    let (socket, dir) = temp_socket("compose_nomissing");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // attach_missing=false + a NOT-attached topic → not attached, not resolved, not
    // placed → a loud NoRenderableTopics refusal.
    let resp = client.request(&format!(
        r#"{{"id":1,"method":"compose_layout","intent":{{"topics":["{topic}"],"attach_missing":false,"include_robot":false}}}}"#
    ));
    assert_eq!(
        resp["ok"].as_bool(),
        Some(false),
        "nothing to place → refuse: {resp}"
    );
    let err = resp["error"].as_str().unwrap();
    assert!(
        err.contains("no views") && err.contains("renderable"),
        "the refusal names the symptom + fix: {resp}"
    );

    // attach_missing=false REALLY attached nothing — list is empty.
    let list = client.request(r#"{"id":2,"method":"list"}"#);
    assert!(
        list["attached"].as_array().unwrap().is_empty(),
        "attach_missing=false must not attach: {list}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Test 23 — compose over an unattachable topic rejects loudly ─
// (a non-existent topic's attach fails), and a silent-but-existing topic composes
// to a NoRenderableTopics refusal (attached, but nothing renderable yet).

#[test]
fn compose_layout_over_unattachable_or_silent_topics_refuses_loudly() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("d_compose_refuse");
    // A SILENT publisher: the topic EXISTS + attaches, but no data flows → it
    // resolves to no archetype → NoRenderableTopics.
    let silent = "/vizd/silent";
    let _publisher = mgr
        .create_publisher(silent, MaxSliceLen::const_new(1 << 16), 0)
        .expect("silent producer attaches");

    let (worker, _flush, _storage) = memory_worker("compose_refuse");
    let (socket, dir) = temp_socket("compose_refuse");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // (a) A non-existent topic with attach_missing → the attach fails → a loud,
    //     topic-named error.
    let missing = client
        .request(r#"{"id":1,"method":"compose_layout","intent":{"topics":["/does/not/exist"]}}"#);
    assert_eq!(missing["ok"].as_bool(), Some(false), "{missing}");
    assert_eq!(missing["topic"].as_str(), Some("/does/not/exist"));
    assert!(
        missing["error"]
            .as_str()
            .unwrap()
            .contains("could not attach"),
        "the refusal names the unattachable topic: {missing}"
    );

    // (b) A silent-but-existing topic attaches but resolves to no archetype →
    //     NoRenderableTopics naming the still-silent topic, AND the tap this compose
    //     opened is ROLLED BACK (transactional): a failed compose
    //     leaves nothing attached.
    let resp = client.request(&format!(
        r#"{{"id":2,"method":"compose_layout","intent":{{"topics":["{silent}"],"include_robot":true}}}}"#
    ));
    assert_eq!(resp["ok"].as_bool(), Some(false), "silent → refuse: {resp}");
    let err = resp["error"].as_str().unwrap();
    assert!(
        err.contains("no views") && err.contains(silent),
        "the refusal names the still-silent topic (robot-only is not a placement): {resp}"
    );
    assert!(
        err.contains("rolled back"),
        "the refusal discloses the rollback of the silent topic it attached: {resp}"
    );

    // The silent topic's tap was rolled back → list is empty (no leaked residue).
    let list = client.request(r#"{"id":3,"method":"list"}"#);
    assert!(
        list["attached"].as_array().unwrap().is_empty(),
        "a failed compose over a silent topic leaves nothing attached: {list}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── compose over many silent topics is bounded by ONE shared peek budget ────

/// **The COMPOSE half of the peek-budget pair.**
///
/// `compose_resolve_topic` peeks every freshly-attached silent topic for up to
/// `PEEK_BOUND` (250 ms). `compose_layout` computes ONE `COMPOSE_PEEK_BUDGET`
/// deadline and threads it across the whole batch, so N silent topics cost the
/// budget once instead of N × `PEEK_BOUND` serially — on a 90-topic robot the
/// difference is ~22 s of a blocked control connection, on a handler that is
/// required to answer in one round trip.
///
/// The `discover` half of the pair is pinned
/// (`discover_over_many_silent_topics_is_bounded`) and CANNOT stand in for this
/// one: `discover` computes its own `DISCOVER_PEEK_BUDGET` deadline and calls a
/// DIFFERENT helper (`resolve_for_discover`, which caches nothing), so a variant
/// that removes only the compose deadline leaves every existing arm green — this
/// was the untested half.
///
/// **The WALL is the only mutation-sensitive assertion here, deliberately.** A
/// silent topic resolves to `None` whether or not the cap exists, so counting
/// unresolved topics would be tautological; the refusal below is asserted as the
/// OUTCOME contract (and as proof the request really ran the resolve loop
/// over all N), not as the budget pin. The ceiling is DERIVED from the shipped
/// const — `2 × COMPOSE_PEEK_BUDGET + 500 ms` — so retuning the budget moves the
/// ceiling with it, and a hardcoded number can never quietly stop being a
/// ceiling. At N = 12 that is 2000 ms against an uncapped ~3000 ms, and against a
/// capped ~750 ms plus attach overhead: the two are cleanly separated, and the
/// slack is on the side a loaded runner can only push toward (a stalled runner
/// makes the capped run slower, never the uncapped one faster).
///
/// Shrinking the budget is constrained from the other side by
/// `a_failed_compose_releases_the_monitor_rows_it_opened_e2e`, whose harness
/// precondition needs the ~750 ms window to span one monitor sampling interval —
/// so the pair of tests brackets this const, rather than only capping it.
#[test]
fn compose_over_many_silent_topics_is_bounded_by_one_shared_peek_budget() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("d_compose_budget");
    // N SILENT topics: each publisher makes the topic EXIST (so the tap attaches)
    // and never publishes (so the peek runs to its bound and resolves nothing).
    // Without the shared budget: N × PEEK_BOUND SEQUENTIALLY.
    const N: usize = 12;
    let topics: Vec<String> = (0..N).map(|i| format!("/vizd/cbudget_{i}")).collect();
    let _pubs: Vec<_> = topics
        .iter()
        .map(|t| {
            mgr.create_publisher(t, MaxSliceLen::const_new(1 << 16), 0)
                .expect("silent producer creates the service")
        })
        .collect();

    let (worker, _flush, _storage) = memory_worker("compose_budget");
    let (socket, dir) = temp_socket("compose_budget");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    let list = topics
        .iter()
        .map(|t| format!("\"{t}\""))
        .collect::<Vec<_>>()
        .join(",");
    // ONE bare-`topics` compose request; the wall is measured around it alone, so
    // daemon startup and the client connect are outside the window.
    let t0 = Instant::now();
    let composed = client.request(&format!(
        r#"{{"id":1,"method":"compose_layout","intent":{{"topics":[{list}]}}}}"#
    ));
    let elapsed = t0.elapsed();

    // The OUTCOME: all N attach, none resolves an archetype, so the compile
    // has zero placements and the request refuses loudly (and rolls the taps back).
    // Asserted because it is what proves the request ran the resolve loop over the
    // whole batch — a compose that refused EARLY would be fast for the wrong reason.
    assert_eq!(
        composed["ok"].as_bool(),
        Some(false),
        "N silent topics resolve no archetype, so the compose must refuse: {composed}"
    );
    let err = composed["error"].as_str().unwrap_or_default();
    // `no views` is the shared head of every zero-placement refusal (silent / dead
    // / mixed — the compose pin above uses the same conservative pair), while "could
    // not attach" is the SHORT-CIRCUITING failure this arm must not have taken: an
    // attach error returns from the loop early, so a fast wall would prove nothing.
    assert!(
        err.contains("no views") && !err.contains("could not attach"),
        "the refusal must be the zero-placement one, not an attach failure (which would \
         short-circuit the batch and make the wall meaningless): {composed}"
    );
    assert!(
        err.contains(&topics[N - 1]),
        "the LAST topic must appear in the refusal — proof the loop reached the end of the \
         batch rather than bailing out partway: {composed}"
    );

    // THE PIN: the whole batch is bounded by ONE shared budget, not N × PEEK_BOUND.
    let ceiling = 2 * cerulion_vizd::daemon::COMPOSE_PEEK_BUDGET + Duration::from_millis(500);
    // Printed so a run's own transcript carries the measured separation rather
    // than only a pass/fail bit (visible under `--nocapture`).
    eprintln!(
        "compose over {N} silent topics: {elapsed:?} (ceiling {ceiling:?}, uncapped would be \
         {:?})",
        Duration::from_millis(250) * N as u32
    );
    assert!(
        elapsed < ceiling,
        "compose over {N} silent topics must be bounded by ONE shared \
         COMPOSE_PEEK_BUDGET ({:?}); took {elapsed:?} against a derived ceiling of {ceiling:?}. \
         Uncapped this is N × PEEK_BOUND ≈ {:?}.",
        cerulion_vizd::daemon::COMPOSE_PEEK_BUDGET,
        Duration::from_millis(250) * N as u32
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Test 24 — the composed plan is REMEMBERED for reconnect- ──
// reapply (the same RUNTIME_BLUEPRINT memory set_blueprint uses).
//
// PARALLEL-SAFE despite the process-global RUNTIME_BLUEPRINT / BLUEPRINT_SENT
// statics: (A) the "remembered" proof polls the runtime slot for a
// DISTINCTIVELY-NAMED plan (a concurrent daemon writing a different plan can't
// match — the name is unique to this test), and (B) the reconnect-reapply proof
// sends onto a FRESH dedicated memory sink NO publisher feeds — so num_msgs growth
// is attributable to the blueprint send ALONE, not sensor frames —
// through a BOUNDED rearm+send retry that wins the process-global once-guard even
// when a concurrent daemon's ensure_setup steals a swap. Note that the
// file-local `BLUEPRINT_STATICS_LOCK` (declared next to `blueprint_statics_guard`
// at the top of the file) is held by EVERY mutating-verb test, not just the
// static-touching pair — but that lock is NECESSARY, NOT SUFFICIENT, so (A)+(B)
// remain LOAD-BEARING, not belt-and-braces. They are what defends this test
// against the writer classes the lock cannot exclude: an UNLOCKED sibling's
// worker-boot `ensure_setup` (W3 — it takes no lock by design), a still-running
// daemon's poll thread reflowing a late-resolved default layout (W2), and a
// daemon still tearing down after its test released the lock. See the
// `BLUEPRINT_STATICS_LOCK` docs for the full enumeration.

#[test]
fn compose_layout_remembers_the_plan_for_reconnect_reapply() {
    let _statics = blueprint_statics_guard();
    clear_runtime_blueprint();

    let mgr = isolated_transport("d_compose_reapply");
    let topic = "/vizd/reapply";
    let _pub = Publisher::spawn(Arc::clone(&mgr), topic); // Vector3 → Scalars

    let (worker, _flush, _storage) = memory_worker("compose_reapply");
    let (socket, dir) = temp_socket("compose_reapply");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // Compose a DISTINCTIVE layout (a `plots` role group titled uniquely, no robot
    // → a lone time_series view named "COMPOSE-REAPPLY").
    let resp = client.request(&format!(
        r#"{{"id":1,"method":"compose_layout","intent":{{"include_robot":false,"groups":[{{"topics":["{topic}"],"role":"plots","title":"COMPOSE-REAPPLY"}}]}}}}"#
    ));
    assert_eq!(resp["ok"].as_bool(), Some(true), "compose applies: {resp}");
    assert_eq!(
        resp["arrangement"].as_str(),
        Some("single"),
        "one plots view: {resp}"
    );

    // (A) The worker applies SetBlueprint on its own thread → poll until the runtime
    //     slot holds OUR distinctively-named plan (robust to concurrent writes —
    //     the name is unique to this test, so nothing else can match it).
    let remembered = wait_until(Duration::from_secs(3), || {
        current_runtime_blueprint_plan()
            .map(|p| plan_has_view_named(&p, "COMPOSE-REAPPLY"))
            .unwrap_or(false)
    });
    assert!(
        remembered,
        "the composed plan is remembered in the runtime slot: {:?}",
        current_runtime_blueprint_plan()
    );
    let plan = current_runtime_blueprint_plan().expect("some remembered plan");
    // Re-confirm OUR name survived (a rare concurrent overwrite fails loudly HERE
    // rather than silently passing) + it is NOT the Go2 default.
    assert!(
        plan_has_view_named(&plan, "COMPOSE-REAPPLY"),
        "our composed plan is still the remembered one: {plan:?}"
    );
    assert_ne!(
        plan,
        BlueprintPlan::go2_default(),
        "the remembered plan is the composed one, NOT the Go2 default"
    );

    // (B) Reconnect-reapply, ATTRIBUTABLY: a reconnect re-arms the once-guard and
    //     re-sends the REMEMBERED runtime plan. Prove it on a FRESH dedicated memory
    //     sink NO publisher feeds — so num_msgs growth is the blueprint send ALONE
    //     — via a bounded rearm+send retry that wins the process-global
    //     once-guard even against a concurrent daemon's ensure_setup.
    let (reapply_rec, reapply_storage) = rerun::RecordingStreamBuilder::new("vizd")
        .recording_id("d_compose_reapply_probe")
        .memory()
        .expect("dedicated reapply memory sink");
    let reapplied = wait_until(Duration::from_secs(5), || {
        rearm_blueprint();
        send_blueprint_once(&reapply_rec);
        reapply_rec.flush_blocking().ok();
        reapply_storage.num_msgs() > 0
    });
    assert!(
        reapplied,
        "a reconnect re-applies the remembered composed plan to a fresh sink no publisher feeds \
         (attributable to the blueprint send alone)"
    );

    clear_runtime_blueprint();
    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Test 25 — a partial attach failure ROLLS BACK ────
// the earlier attaches (compose_layout is transactional — no half-attached residue).

#[test]
fn compose_layout_partial_attach_failure_rolls_back_the_earlier_attaches() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("d_compose_partial");
    let live_ok = "/vizd/live_ok";
    let _pub = Publisher::spawn(Arc::clone(&mgr), live_ok); // Vector3 → resolvable

    let (worker, _flush, _storage) = memory_worker("compose_partial");
    let (socket, dir) = temp_socket("compose_partial");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // A mixed [good, bad] bare list: /live_ok exists + attaches FIRST, then
    // /does/not/exist fails. A non-transactional compose leaves /live_ok attached, unreported (a leak);
    // compose_layout is TRANSACTIONAL — it rolls /live_ok back.
    let resp = client.request(&format!(
        r#"{{"id":1,"method":"compose_layout","intent":{{"topics":["{live_ok}","/does/not/exist"]}}}}"#
    ));
    assert_eq!(
        resp["ok"].as_bool(),
        Some(false),
        "the partial failure refuses: {resp}"
    );
    assert_eq!(resp["topic"].as_str(), Some("/does/not/exist"));
    let err = resp["error"].as_str().unwrap();
    assert!(
        err.contains("could not attach") && err.contains("/does/not/exist"),
        "the refusal names the unattachable topic: {resp}"
    );
    assert!(
        err.contains("rolled back"),
        "the refusal DISCLOSES the rollback of the earlier attach (loud, not silent): {resp}"
    );

    // TRANSACTIONAL: /live_ok was rolled back → the tap set is EMPTY (no residue).
    let list = client.request(r#"{"id":2,"method":"list"}"#);
    assert!(
        list["attached"].as_array().unwrap().is_empty(),
        "the earlier /live_ok attach was rolled back — no partial residue: {list}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Test 26 — an attach_missing:false role-placed ─────
// un-attached topic is NOT falsely refused when an UNRELATED topic is already
// tapped (validation grounds against attached UNION placed, not attached alone).

#[test]
fn compose_attach_missing_false_role_placed_unattached_topic_is_not_falsely_refused() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("d_compose_f1");
    let unrelated = "/vizd/unrelated";
    let _pub = Publisher::spawn(Arc::clone(&mgr), unrelated); // Vector3 → Scalars

    let (worker, _flush, _storage) = memory_worker("compose_f1");
    let (socket, dir) = temp_socket("compose_f1");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // Pre-attach the UNRELATED topic (it resolves, so the tap sticks) — the topic
    // that a validation grounding ONLY against attached taps would see.
    let att = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{unrelated}"}}"#
    ));
    assert_eq!(att["ok"].as_bool(), Some(true), "{att}");

    // Compose a plots role group over a NOT-tapped topic, attach_missing:false. The
    // ONLY view roots at /vizd/ghost's entity; validating against only the
    // attached /unrelated would raise AllViewsUngrounded — a FALSE refusal.
    let resp = client.request(
        r#"{"id":2,"method":"compose_layout","intent":{"attach_missing":false,"include_robot":false,"groups":[{"topics":["/vizd/ghost"],"role":"plots"}]}}"#,
    );
    assert_eq!(
        resp["ok"].as_bool(),
        Some(true),
        "a role-placed un-attached topic is NOT falsely refused: {resp}"
    );
    // It is placed by role with a soft warning (silent, un-attached) …
    let warnings = resp["warnings"].as_array().unwrap();
    assert!(
        warnings
            .iter()
            .any(|w| w.as_str().unwrap_or("").contains("/vizd/ghost")),
        "the placed-by-role silent topic is soft-warned: {resp}"
    );
    // … and attach_missing:false attached NOTHING new.
    assert!(
        resp["attached"].as_array().unwrap().is_empty(),
        "attach_missing:false attaches nothing: {resp}"
    );

    // /vizd/ghost stays un-attached (only /unrelated is tapped).
    let list = client.request(r#"{"id":3,"method":"list"}"#);
    assert!(
        entry_for(&list["attached"], "topic", unrelated).is_some(),
        "the unrelated topic is still attached: {list}"
    );
    assert!(
        entry_for(&list["attached"], "topic", "/vizd/ghost").is_none(),
        "the un-attached role-placed topic stays un-attached: {list}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Test 27 — a compose plan with a GROUNDED hero + a ─
// placed-but-unattached sidebar view is NOT refused by the reused guardrail (the
// compiler's soft-warning acceptance and the guardrail now AGREE).

#[test]
fn compose_attach_missing_false_grounded_hero_plus_unattached_sidebar_applies() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("d_compose_f5");
    let attached = "/vizd/attached";
    let _pub = Publisher::spawn(Arc::clone(&mgr), attached); // Vector3 → Scalars (plots)

    let (worker, _flush, _storage) = memory_worker("compose_f5");
    let (socket, dir) = temp_socket("compose_f5");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // Attach the resolvable topic (Scalars → plots-compatible).
    let att = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{attached}"}}"#
    ));
    assert_eq!(att["ok"].as_bool(), Some(true), "{att}");

    // attach_missing:false + include_robot:true → a GROUNDED robot hero (world root)
    // + a plots view grounding the attached topic + an images view over a
    // NOT-attached /vizd/silentcam. Grounded against attached taps alone, guardrail 1 (auto_views off)
    // would refuse the images view with UngroundedView; attached ∪ placed grounds it.
    let resp = client.request(&format!(
        r#"{{"id":2,"method":"compose_layout","intent":{{"attach_missing":false,"include_robot":true,"groups":[{{"topics":["{attached}"],"role":"plots"}},{{"topics":["/vizd/silentcam"],"role":"images"}}]}}}}"#
    ));
    assert_eq!(
        resp["ok"].as_bool(),
        Some(true),
        "a grounded hero + a placed-but-unattached sidebar view applies (not refused): {resp}"
    );
    assert_eq!(
        resp["arrangement"].as_str(),
        Some("hero_sidebar"),
        "a robot hero beside the sidebar: {resp}"
    );
    // The un-attached image topic is soft-warned (placed by role), attached nothing.
    let warnings = resp["warnings"].as_array().unwrap();
    assert!(
        warnings
            .iter()
            .any(|w| w.as_str().unwrap_or("").contains("/vizd/silentcam")),
        "the placed-by-role image topic is soft-warned: {resp}"
    );
    assert!(
        resp["attached"].as_array().unwrap().is_empty(),
        "attach_missing:false attaches nothing new: {resp}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Decision — the dynamic default: attach auto-applies a consolidated
// Scene + per-topic plot layout (never tabs) + the wall-clock default timeline, and an
// EXPLICIT set_blueprint WINS (a later attach does not clobber it). Observed on each
// daemon's OWN memory sink (per-daemon = non-racy, unlike the process-global
// RUNTIME_BLUEPRINT): a blueprint SEND is one `viewport` chunk (a data frame never
// produces one, so a live publisher does not perturb the count).

/// The number of blueprint SENDS captured in `msgs` — one `viewport` chunk per send.
fn blueprint_send_count(msgs: &[rerun::external::re_log_types::LogMsg]) -> usize {
    blueprint_property_paths(msgs)
        .iter()
        .filter(|p| p.trim_start_matches('/') == "viewport")
        .count()
}

/// Drain `storage` into `msgs` for `window` (flushing the worker), so a bounded
/// no-more-sends assertion sees every blueprint the worker applied in that window.
fn drain_into(
    flush: &rerun::RecordingStream,
    storage: &rerun::sink::MemorySinkStorage,
    msgs: &mut Vec<rerun::external::re_log_types::LogMsg>,
    window: Duration,
) {
    let deadline = Instant::now() + window;
    while Instant::now() < deadline {
        flush.flush_blocking().ok();
        msgs.extend(storage.take());
        std::thread::sleep(Duration::from_millis(50));
    }
}

#[test]
fn attach_auto_applies_the_consolidated_default_scene_plus_plot_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("auto_plan");
    let topic = "/vizd/autoplot";
    let _pub = Publisher::spawn(Arc::clone(&mgr), topic); // Vector3 → Scalars → a plot

    let (worker, flush, storage) = memory_worker("auto_plan");
    let (socket, dir) = temp_socket("auto_plan");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // Baseline: let the worker's INITIAL send-once (the Go2 default) flush + be counted,
    // so the assertion below is attributable to the ATTACH's default, not the boot one.
    let mut msgs = Vec::new();
    wait_until(Duration::from_secs(2), || {
        flush.flush_blocking().ok();
        msgs.extend(storage.take());
        blueprint_send_count(&msgs) >= 1
    });
    let boot_sends = blueprint_send_count(&msgs);

    // Attach the plot topic → the daemon auto-applies the consolidated default.
    let att = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(att["ok"].as_bool(), Some(true), "{att}");
    // The attach reply carries the resolved view kinds (the "which view?"
    // surface) — a plot-shaped topic reports time_series.
    let view_kinds = att["view_kinds"].as_array();
    assert!(
        view_kinds.is_some_and(|a| a.iter().any(|v| v == "time_series")),
        "the attach reply exposes the plot's view kind (time_series): {att}"
    );

    // The daemon SENT a NEW blueprint on attach (a viewport chunk beyond the boot send)
    // AND it carries BOTH a time_series window (the consolidated plot) AND a spatial
    // Background (the Scene) — Scene + plot, applied on attach.
    let applied = wait_until(Duration::from_secs(5), || {
        flush.flush_blocking().ok();
        msgs.extend(storage.take());
        let paths = blueprint_property_paths(&msgs);
        blueprint_send_count(&msgs) > boot_sends
            && paths.iter().any(|p| p.ends_with("/VisibleTimeRanges"))
            && paths.iter().any(|p| p.ends_with("/Background"))
    });
    assert!(
        applied,
        "attach auto-applies a Scene + consolidated plot default (a NEW blueprint send \
         carrying BOTH a time_series window and a spatial background)"
    );
    // (The wall-clock log_time timeline pin — Task B — is proven robustly by
    // `boot_send_pins_the_timeline_and_runtime_reapply_does_not_re_pin_e2e`, driven off
    // the process-global boot once-guard directly; asserting it HERE off a daemon's boot
    // send is fragile once an earlier test consumes that guard.)

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn explicit_layout_wins_and_a_later_attach_does_not_clobber_it_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("explicit_plan");
    let t1 = "/vizd/explicit1";
    let t2 = "/vizd/explicit2";
    let _p1 = Publisher::spawn(Arc::clone(&mgr), t1);
    let _p2 = Publisher::spawn(Arc::clone(&mgr), t2);

    let (worker, flush, storage) = memory_worker("explicit_plan");
    let (socket, dir) = temp_socket("explicit_plan");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // Attach t1 (mode Auto → a default is applied), then set an EXPLICIT blueprint (a
    // lone time_series view over t1) → mode Explicit takes over.
    assert_eq!(
        client.request(&format!(r#"{{"id":1,"method":"attach","topic":"{t1}"}}"#))["ok"].as_bool(),
        Some(true)
    );
    let ex = client.request(
        r#"{"id":2,"method":"set_blueprint","layout":{"root":{"type":"view","kind":"time_series","name":"EXPLICITVIEW","origin":"world/vizd/explicit1"}}}"#,
    );
    assert_eq!(
        ex["ok"].as_bool(),
        Some(true),
        "explicit set_blueprint applies: {ex}"
    );

    // Settle: drain every pending blueprint send (boot default + t1 auto-default + the
    // explicit) for a generous window, then snapshot the send count as the BASELINE (a
    // live publisher only adds DATA chunks, never a `viewport`).
    let mut msgs = Vec::new();
    drain_into(&flush, &storage, &mut msgs, Duration::from_millis(1500));
    let baseline = blueprint_send_count(&msgs);
    assert!(
        baseline >= 1,
        "at least the boot + explicit blueprints were sent (baseline {baseline})"
    );

    // Now attach t2. Under Explicit mode this MUST NOT re-apply the dynamic default —
    // the explicit layout wins, so NO new blueprint is sent.
    assert_eq!(
        client.request(&format!(r#"{{"id":3,"method":"attach","topic":"{t2}"}}"#))["ok"].as_bool(),
        Some(true)
    );
    drain_into(&flush, &storage, &mut msgs, Duration::from_millis(1500));
    assert_eq!(
        blueprint_send_count(&msgs),
        baseline,
        "an attach under an EXPLICIT layout sends NO new blueprint — the explicit layout WINS; \
         attach does not clobber it (anti-tautology: the Auto attach in \
         attach_auto_applies_the_consolidated_default_scene_plus_plot_e2e DOES send)"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

/// The number of timeline PINS captured in `msgs` — one `time_panel` chunk per pin.
/// Only the boot/first send pins; runtime re-applies emit none.
fn time_panel_count(msgs: &[rerun::external::re_log_types::LogMsg]) -> usize {
    blueprint_property_paths(msgs)
        .iter()
        .filter(|p| p.trim_start_matches('/') == "time_panel")
        .count()
}

#[test]
fn boot_send_pins_the_timeline_and_runtime_reapply_does_not_re_pin_e2e() {
    let _statics = blueprint_statics_guard();
    // The wall-clock `log_time` timeline pin rides the BOOT send
    // (`send_blueprint_once`) ONLY — a runtime RE-APPLY (`apply_runtime_blueprint`, the
    // attach/detach/set_blueprint path) emits NO time_panel, so a user who manually
    // switched to `robot_time` is never snapped back on a topic toggle. Drive the two
    // send functions DIRECTLY on a dedicated blueprint sink NO daemon/publisher feeds
    // (test-24 pattern), so every chunk is attributable to the explicit call and the
    // assertion never depends on daemon-boot timing / the process-global once-guard's
    // prior state. Serialized against every other blueprint-touching test (the
    // statics guard at the top of this body) — but that lock is NECESSARY, NOT
    // SUFFICIENT, so the dedicated sink + the bounded rearm+send retry are
    // LOAD-BEARING: they are what survives an UNLOCKED sibling's worker-boot
    // `ensure_setup` stealing the once-guard swap (writer class W3) and a running
    // daemon's poll thread reflowing a layout (W2). See `BLUEPRINT_STATICS_LOCK`.
    clear_runtime_blueprint();

    let (rec, storage) = rerun::RecordingStreamBuilder::new("vizd")
        .recording_id("boot_pin_probe")
        .memory()
        .expect("dedicated memory sink");

    // (1) The BOOT send (send_blueprint_once, `pin_timeline = true`) pins the timeline
    //     EXACTLY ONCE onto OUR sink — win the process-global once-guard via retry.
    let mut msgs = Vec::new();
    let boot = wait_until(Duration::from_secs(5), || {
        rearm_blueprint();
        send_blueprint_once(&rec);
        rec.flush_blocking().ok();
        msgs.extend(storage.take());
        blueprint_send_count(&msgs) >= 1
    });
    assert!(boot, "the boot send reached our dedicated sink");
    assert_eq!(
        time_panel_count(&msgs),
        1,
        "the boot send pins the timeline EXACTLY once: {:?}",
        blueprint_property_paths(&msgs)
    );
    assert_eq!(
        blueprint_panel_timeline(&msgs).as_deref(),
        Some("log_time"),
        "and it pins the wall-clock log_time"
    );
    let boot_sends = blueprint_send_count(&msgs);

    // (2) A runtime RE-APPLY (apply_runtime_blueprint, `pin_timeline = false`) sends a
    //     NEW blueprint but adds NO new time_panel pin.
    apply_runtime_blueprint(&rec, default_layout(&[]));
    rec.flush_blocking().ok();
    msgs.extend(storage.take());
    assert_eq!(
        time_panel_count(&msgs),
        1,
        "a runtime re-apply emits NO new time_panel — the timeline is never re-pinned: {:?}",
        blueprint_property_paths(&msgs)
    );
    // Anti-tautology: the re-apply DID send a blueprint (only its timeline pin is
    // suppressed) — so the "no new pin" above is a real suppression, not a no-send.
    assert!(
        blueprint_send_count(&msgs) > boot_sends,
        "the re-apply really sent a blueprint (its pin is suppressed, not the send)"
    );

    clear_runtime_blueprint();
}

#[test]
fn late_resolving_topic_reflows_into_its_view_e2e() {
    let _statics = blueprint_statics_guard();
    // A topic attached while momentarily SILENT resolves NO archetype, so
    // the dynamic default (auto_views:false) places it in NO view. When its first
    // decodable frame arrives, the poll thread's None→Some back-fill re-triggers the
    // reflow so it lands in its plot — otherwise it would render NOWHERE forever.
    let mgr = isolated_transport("late_plan");
    let topic = "/vizd/lateplot";

    // A publisher whose data service exists (→ the topic is ATTACHABLE) but which
    // publishes NOTHING until `go` flips — so the topic is attachable-but-silent.
    let go = Arc::new(AtomicBool::new(false));
    let stop = Arc::new(AtomicBool::new(false));
    let created = Arc::new(AtomicBool::new(false));
    let pub_handle = {
        let mgr = Arc::clone(&mgr);
        let (go, stop, created) = (Arc::clone(&go), Arc::clone(&stop), Arc::clone(&created));
        let topic = topic.to_string();
        std::thread::spawn(move || {
            let mut publisher = mgr
                .create_publisher(&topic, MaxSliceLen::const_new(1 << 16), 0)
                .expect("producer attaches");
            created.store(true, Ordering::Release);
            let mut seq = 0u32;
            while !stop.load(Ordering::Relaxed) {
                if go.load(Ordering::Relaxed) {
                    let frame = build_vector3_frame(
                        seq,
                        PUBLISH_STAMP_STEP_NS * (seq as u64 + 1),
                        [seq as f64, 2.0, 3.0],
                    );
                    let _ = publisher.publish_raw(&frame);
                    seq += 1;
                }
                std::thread::sleep(Duration::from_millis(10));
            }
        })
    };
    // Wait until the data service exists before attaching (else the tap would miss it).
    assert!(
        wait_until(Duration::from_secs(3), || created.load(Ordering::Acquire)),
        "the publisher created the data service"
    );

    let (worker, flush, storage) = memory_worker("late_plan");
    let (socket, dir) = temp_socket("late_plan");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // Attach the SILENT topic — it resolves NO archetype (view_kinds null), so the
    // dynamic default places it in NO plot.
    let att = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(att["ok"].as_bool(), Some(true), "{att}");
    assert!(
        att["view_kinds"].is_null(),
        "the silent topic resolves to no view yet: {att}"
    );

    // Baseline: no time_series plot exists (the topic is silent → not placed).
    let mut msgs = Vec::new();
    drain_into(&flush, &storage, &mut msgs, Duration::from_millis(900));
    assert!(
        !blueprint_property_paths(&msgs)
            .iter()
            .any(|p| p.ends_with("/VisibleTimeRanges")),
        "a silent attached topic has NO plot yet (not in any view)"
    );

    // GO: the topic starts publishing → the poll thread resolves it (None→Some) →
    // re-triggers the reflow → a time_series plot for the now-resolved Scalars topic
    // appears. Before GO there was none, so the plot is attributable to the reflow.
    go.store(true, Ordering::Relaxed);
    let reflowed = wait_until(Duration::from_secs(6), || {
        flush.flush_blocking().ok();
        msgs.extend(storage.take());
        blueprint_property_paths(&msgs)
            .iter()
            .any(|p| p.ends_with("/VisibleTimeRanges"))
    });
    assert!(
        reflowed,
        "the late-resolving topic reflowed into a time_series plot once its data arrived"
    );

    stop.store(true, Ordering::Relaxed);
    let _ = pub_handle.join();
    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

/// Whether any leaf view in `plan` carries the display `name`.
fn plan_has_view_named(plan: &BlueprintPlan, name: &str) -> bool {
    fn walk(node: &PlanNode, name: &str) -> bool {
        match node {
            PlanNode::View(v) => v.name.as_deref() == Some(name),
            PlanNode::Container(c) => c.children.iter().any(|ch| walk(ch, name)),
        }
    }
    walk(&plan.root, name)
}

// ── A big `discover` reply must round-trip UN-truncated ──────────────────────
//
// The symptom on a desk with a 75-topic robot reachable: the shell connects to vizd, vizd
// logs `vizd: controller write timed out (not reading) ... error=Resource
// temporarily unavailable (os error 35)`, and the sidebar shows "Can't reach
// vizd". The discover/list control reply exceeds the ~8 KiB UDS send buffer, and
// a write path that aborts on the FIRST WouldBlock (macOS: the accepted socket is
// nonblocking) truncates it. vizd and netd share the bounded
// resume loop (`cerulion_core::write_line_bounded`). This pin drives a REAL vizd
// daemon with a genuine large catalog (a DI test double, Principle #13) so the
// `discover` reply crosses the send buffer, and asserts the FULL line round-trips.

/// A large hand-built catalog (a test oracle — NOT fake data): one robot serving `n`
/// distinct ~150-byte remote-topic rows, so the folded `discover` reply exceeds the
/// UDS send buffer.
fn big_catalog(robot: &str, n: usize) -> cerulion_core::CatalogReply {
    use cerulion_core::{CatalogEntry, CatalogProvenance};
    cerulion_core::CatalogReply {
        version: 1,
        robot: robot.to_string(),
        entries: (0..n)
            .map(|i| CatalogEntry {
                topic: format!(
                    "/discover/big/topic/{i:06}/padding/aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa/bbbbbbbbbbbbbbbbbbbb"
                ),
                schema_hash: Some(7),
                schema_name: Some("sensor_msgs/PointCloud2".to_string()),
                provenance: CatalogProvenance::Runtime,
                producer_count: None,
                liveness: None,
            })
            .collect(),
        error: None,
    }
}

#[test]
fn big_discover_response_over_the_send_buffer_round_trips_untruncated() {
    let mgr = isolated_transport("vizd_bigdiscover");
    // ~400 remote topics (none local ⇒ none filtered) fold into ONE robots section,
    // pushing the discover reply well past the 8192-byte UDS send buffer.
    const N: usize = 400;
    let plane = Arc::new(GatherPlane::new(vec![big_catalog("bigbot", N)]));
    let (worker, _flush, _storage) = memory_worker("bigdiscover");
    let (socket, dir) = temp_socket("bigdiscover");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");

    // Raw connection so we can LAG the reader: send `discover`, then stop reading
    // briefly so the daemon's send buffer fills BEFORE the client drains — the macOS
    // truncation repro. A write that aborts on a full buffer does so in microseconds (long before this
    // lag), so its line arrives truncated + un-terminated; the 250 ms lag is
    // far under the 2 s no-progress budget, so the daemon simply resumes once
    // we read. On Linux (blocking + SO_SNDTIMEO) the write completes on both paths.
    let stream = connect_bounded(&socket);
    let mut writer = stream.try_clone().expect("clone stream");
    let mut reader = BufReader::new(stream);
    let mut banner = String::new();
    reader.read_line(&mut banner).expect("read banner");

    writeln!(writer, r#"{{"id":1,"method":"discover"}}"#).expect("send discover");
    writer.flush().expect("flush");
    std::thread::sleep(Duration::from_millis(250));

    let mut resp = String::new();
    reader
        .read_line(&mut resp)
        .expect("read the discover response line");

    // Untruncated: newline-terminated, > the send buffer, and every remote row
    // survived the big write (an aborted write truncates at ~8192 bytes, no '\n').
    assert!(
        resp.ends_with('\n'),
        "truncated: the discover reply is not newline-terminated (got {} bytes, no '\\n' — the \
         send-buffer-abort defect)",
        resp.len()
    );
    assert!(
        resp.len() > 8192,
        "the reply must exceed the 8192-byte send buffer to exercise the >buffer path (got {}) — \
         the pin cannot silently shrink below the boundary",
        resp.len()
    );
    let disc: Value =
        serde_json::from_str(resp.trim()).expect("the FULL line parses (untruncated)");
    assert_eq!(disc["ok"].as_bool(), Some(true));
    let robots = disc["robots"].as_array().expect("robots section present");
    assert_eq!(robots.len(), 1, "one robot");
    assert_eq!(robots[0]["robot"].as_str(), Some("bigbot"));
    assert_eq!(
        robots[0]["topics"].as_array().unwrap().len(),
        N,
        "every remote row survived the big write (untruncated)"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ════════════════════════════════════════════════════════════════════════════
// THE "/plan draws" acceptance e2e.
//
// The whole reason canonical element framing exists: a non-technical user attaches Cerulion to a
// robot we have NEVER seen, and its `/plan` (`nav_msgs/Path`) should just DRAW —
// a polyline in the 3D scene, not a `TextDocument` reading `[N element(s)]`.
//
// These tests drive the FULL production path over the real daemon:
//
//   hand-written CDR body
//     → CdrCodec::decode           (the EXACT codec `ros2 attach`'s dds_bridge
//                                    runs on every never-seen DDS type)
//     → a real Cerulion frame      (published over real iceoryx2)
//     → the daemon taps it         (a listener-less DataOnlySubscriber)
//     → FrameWalker decodes the canonical element framing
//     → the sink ladder
//     → the Rerun memory sink.
//
// We then read the rendered chunks back out of the memory sink (`re_chunk`, the
// pattern `live_only_history_test.rs` uses) and assert the `/plan` subtree's
// ARCHETYPES against a hand oracle: a populated plan carries a `LineStrips3D`
// polyline + its `Points3D` vertices and NO `TextDocument`; an idle empty plan
// still RESOLVES to the drawing archetype (never frozen to a text view);
// and a genuinely-undecodable element array
// degrades to the text dump — the negative control that proves the daemon
// DISCRIMINATES rather than always drawing (or always dumping). Never a
// self-compare: the CDR body and the undecodable bytes are hand-built oracles.
//
// Every test issues the `attach` verb (a W1 blueprint-static writer), so each
// takes `blueprint_statics_guard()` as its first statement per the file's rule.
// ════════════════════════════════════════════════════════════════════════════

/// A per-element `nav_msgs/Path` oracle: `(position, orientation, stamp.sec,
/// stamp.nanosec, header.frame_id)`. The `frame_id`s are DELIBERATELY different
/// lengths so every element body is a different size and the canonical
/// per-element `u32 len` genuinely has to be read (a uniform-stride decoder
/// could not carry these). Crib of `frame_walker_production_test.rs`'s
/// `PATH_ORACLE`.
type PathElementOracle = ([f64; 3], [f64; 4], i32, u32, &'static str);

const PATH_ORACLE: [PathElementOracle; 3] = [
    ([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0], 11, 12, "map"),
    (
        [-4.5, 5.25, 6.125],
        [0.5, -0.5, 0.25, 0.75],
        21,
        22,
        "odom_frame",
    ),
    (
        [7.0, 8.0, 9.0],
        [0.125, 0.25, 0.5, 0.875],
        31,
        32,
        "base_link_extra_long",
    ),
];

/// The wire timestamp every `/plan` fixture frame carries.
const PLAN_TS: u64 = 42_000;

/// A minimal CDR body writer mirroring `CdrReader`'s rules: align to a
/// primitive's own width before writing it, and spell a string as a 4-aligned
/// `u32(content_len + 1)` then the UTF-8 then a NUL. Byte 0 is the field-aligned
/// origin (the byte after the 4-byte encapsulation header, which
/// `CdrCodec::decode` takes as already stripped).
///
/// Hand-rolled — NOT the codec's own `CdrWriter` — because the CDR body is the
/// test's INPUT ORACLE and must not come from the implementation under test.
/// Crib of `frame_walker_production_test.rs`'s `CdrBody`.
#[derive(Default)]
struct CdrBody {
    buf: Vec<u8>,
}

impl CdrBody {
    fn align(&mut self, a: usize) {
        while !self.buf.len().is_multiple_of(a) {
            self.buf.push(0);
        }
    }
    fn u32(&mut self, v: u32) {
        self.align(4);
        self.buf.extend_from_slice(&v.to_le_bytes());
    }
    fn i32(&mut self, v: i32) {
        self.align(4);
        self.buf.extend_from_slice(&v.to_le_bytes());
    }
    fn f64(&mut self, v: f64) {
        self.align(8);
        self.buf.extend_from_slice(&v.to_le_bytes());
    }
    /// CDR string: `u32(len + 1)` | UTF-8 | NUL.
    fn string(&mut self, s: &str) {
        self.u32(s.len() as u32 + 1);
        self.buf.extend_from_slice(s.as_bytes());
        self.buf.push(0);
    }
    /// `std_msgs/Header` = `builtin_interfaces/Time stamp` (`int32 sec`,
    /// `uint32 nanosec`) + `string frame_id`, in declaration order.
    fn header(&mut self, sec: i32, nanosec: u32, frame_id: &str) {
        self.i32(sec);
        self.u32(nanosec);
        self.string(frame_id);
    }
    /// `geometry_msgs/Pose` = `Point position` (3 × f64) + `Quaternion
    /// orientation` (4 × f64) — 7 CDR doubles, no padding between them.
    fn pose(&mut self, position: [f64; 3], orientation: [f64; 4]) {
        for c in position {
            self.f64(c);
        }
        for c in orientation {
            self.f64(c);
        }
    }
}

/// Build the `nav_msgs/Path` CDR body. `PoseStamped` is declared `header` then
/// `pose`, so each element is written in that order.
fn path_cdr(elements: &[PathElementOracle]) -> Vec<u8> {
    let mut cdr = CdrBody::default();
    cdr.header(1, 2, "map");
    cdr.u32(elements.len() as u32);
    for (position, orientation, sec, nanosec, frame_id) in elements {
        cdr.header(*sec, *nanosec, frame_id);
        cdr.pose(*position, *orientation);
    }
    cdr.buf
}

/// The production canonical-v1 codec over the built-in ROS 2 corpus — the exact
/// `CdrCodec` `cerulion ros2 attach`'s `dds_bridge` graph runs on the
/// `MappingRoute::RawGeneric` path. Crib of `frame_walker_production_test.rs`'s
/// `builtin_codec`.
fn builtin_codec() -> CdrCodec {
    let mut schemas: Vec<MessageSchema> = Vec::new();
    for (pkg, name, text) in native_ros2_messages::BUILTIN_MSGS {
        if let Ok(s) = parse_rosmsg(text, name, Some(pkg)) {
            schemas.push(s);
        }
    }
    let (codec, warnings) = CdrCodec::new(schemas);
    assert!(
        warnings.is_empty(),
        "built-in schema set must resolve cleanly for the codec: {warnings:?}"
    );
    codec
}

/// A `nav_msgs/Path` wire frame straight off the PRODUCTION codec — the exact
/// bytes a dds-bridged `/plan` from a never-seen robot carries.
fn plan_frame(codec: &CdrCodec, elements: &[PathElementOracle]) -> Vec<u8> {
    codec
        .decode(
            "nav_msgs/Path",
            CdrEndianness::Little,
            &path_cdr(elements),
            0,
            PLAN_TS,
        )
        .expect("production CdrCodec::decode of nav_msgs/Path")
}

/// One 56-byte `geometry_msgs/Pose` fixed section (7 f64 LE: `Point{x,y,z}` then
/// `Quaternion{x,y,z,w}`) — used to build the UNDECODABLE negative control.
fn pose_fixed_section(pos: [f64; 3], quat: [f64; 4]) -> Vec<u8> {
    let mut v = Vec::with_capacity(56);
    for c in pos.iter().chain(quat.iter()) {
        v.extend_from_slice(&c.to_le_bytes());
    }
    assert_eq!(v.len(), 56, "Pose fixed section drifted");
    v
}

/// `u32 count` + per element (`u32 len`, body) — the canonical COUNTED framing a
/// VARIABLE element rides.
fn counted_blob(elements: &[Vec<u8>]) -> Vec<u8> {
    let mut v = Vec::new();
    v.extend_from_slice(&(elements.len() as u32).to_le_bytes());
    for e in elements {
        v.extend_from_slice(&(e.len() as u32).to_le_bytes());
        v.extend_from_slice(e);
    }
    v
}

/// A hand-built `nav_msgs/Path` frame whose `poses` blob is UNDECODABLE: the
/// canonical COUNTED framing wraps bare 56-byte `Pose` sections — i.e. a
/// `PoseStamped` element body with its `header` offset table MISSING, a
/// packed, non-canonical element shape. The walker's element check
/// refuses it (PoseStamped needs 56 + 8 + a `Header` sub-frame), so it stays
/// `NestedArrayOpaque` and the sink degrades to a text dump — the negative
/// control that proves the daemon DISCRIMINATES, not just draws.
///
/// `nav_msgs/Path` is `WIRE_FIXED_SIZE = 0` with two variable fields
/// (`header`, `poses`) ⇒ a 16-byte offset table; the walker premise assert in
/// the test is the drift guard on that shape.
fn build_undecodable_path_frame() -> Vec<u8> {
    let bodies: Vec<Vec<u8>> = [[1.0f64, 2.0, 3.0], [4.0, 5.0, 6.0]]
        .iter()
        .map(|p| pose_fixed_section(*p, [0.0, 0.0, 0.0, 1.0]))
        .collect();
    let poses_blob = counted_blob(&bodies);
    let table = 16usize; // 2 variable fields × 8-byte offset entries
    let mut payload = vec![0u8; table];
    // entry 0 `header`: (offset = table, length = 0) — the empty-nested idiom.
    write_offset_entry(&mut payload, 0, 0, table as u32, 0);
    // entry 1 `poses`: the undecodable blob.
    write_offset_entry(&mut payload, 0, 1, table as u32, poses_blob.len() as u32);
    payload.extend_from_slice(&poses_blob);
    let header = WireHeader {
        schema_hash: <Path as ShmMessage>::SCHEMA_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: WireHeader::SIZE as u32, // fixed size 0
        offset_table_count: 2,
        sequence: 0,
        timestamp_ns: PLAN_TS,
    };
    let mut frame = vec![0u8; WireHeader::SIZE];
    header.write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);
    frame
}

impl Publisher {
    /// Publish a FIXED frame repeatedly (~100 Hz), re-stamping
    /// only its wire `sequence` so every publish is a distinct sample the daemon
    /// taps and renders. The frame bytes are produced ONCE — by
    /// [`plan_frame`] (the production codec) or [`build_undecodable_path_frame`]
    /// (the hand-built negative control) — so the render is of oracle data.
    ///
    /// The port is created on the CALLING thread for the same reason
    /// [`Publisher::spawn`]'s is — see that doc for the measurement. Both
    /// constructors are pinned by
    /// [`publisher_spawn_is_visible_before_it_returns`].
    fn spawn_fixed_frame(mgr: Arc<TransportManager>, topic: &str, frame: Vec<u8>) -> Self {
        let stop = Arc::new(AtomicBool::new(false));
        let topic = topic.to_string();
        let published = Arc::new(AtomicU32::new(0));
        // Counted here too, so `achieved_hz` is meaningful for EVERY
        // publisher in this file rather than silently reporting 0 for this one.
        let published_c = Arc::clone(&published);
        let stop_c = Arc::clone(&stop);
        let mut publisher = mgr
            .create_publisher(&topic, MaxSliceLen::const_new(1 << 16), 0)
            .unwrap_or_else(|e| panic!("producer attaches on '{topic}': {e}"));
        let handle = std::thread::spawn(move || {
            let mut frame = frame;
            let mut seq: u32 = 0;
            while !stop_c.load(Ordering::Relaxed) {
                if let Some(mut h) = WireHeader::read_from_buf(&frame[..WireHeader::SIZE]) {
                    h.sequence = seq;
                    h.write_to_buf(&mut frame[..WireHeader::SIZE]);
                }
                let _ = publisher.publish_raw(&frame);
                published_c.fetch_add(1, Ordering::Relaxed);
                seq = seq.wrapping_add(1);
                std::thread::sleep(Duration::from_millis(10));
            }
        });
        Publisher {
            stop,
            handle: Some(handle),
            published,
        }
    }
}

/// One rendered RECORDING-store chunk: its entity path (leading `/` stripped, so
/// it matches the daemon's `attach{entity}` string) and the rerun ARCHETYPE
/// short names logged on it (e.g. `LineStrips3D`, `Points3D`, `TextDocument`).
#[derive(Debug, Clone)]
struct RenderedChunk {
    entity: String,
    archetypes: Vec<String>,
}

/// Parse a memory-sink `LogMsg` into a rendered RECORDING chunk. Blueprint-store
/// messages (the layout, not the drawn geometry) and non-chunk messages return
/// `None`.
fn recording_chunk(msg: &LogMsg) -> Option<RenderedChunk> {
    let LogMsg::ArrowMsg(store_id, arrow) = msg else {
        return None;
    };
    if store_id.kind() != StoreKind::Recording {
        return None;
    }
    let chunk = re_chunk::Chunk::from_arrow_msg(arrow).ok()?;
    let entity = chunk.entity_path().to_string();
    let entity = entity.trim_start_matches('/').to_string();
    let mut archetypes: Vec<String> = chunk
        .components()
        .component_descriptors()
        .filter_map(|d| d.archetype.as_ref().map(|a| a.short_name().to_string()))
        .collect();
    archetypes.sort();
    archetypes.dedup();
    Some(RenderedChunk { entity, archetypes })
}

/// Poll the memory sink, keeping only RECORDING chunks, ACCUMULATING across polls
/// (each `take()` empties the sink) until `done` is satisfied or `bound` elapses.
/// Returns everything accumulated — the caller then asserts against it.
fn accumulate_recording_chunks(
    flush: &rerun::RecordingStream,
    storage: &rerun::sink::MemorySinkStorage,
    bound: Duration,
    mut done: impl FnMut(&[RenderedChunk]) -> bool,
) -> Vec<RenderedChunk> {
    let deadline = Instant::now() + bound;
    let mut acc: Vec<RenderedChunk> = Vec::new();
    loop {
        flush.flush_blocking().ok();
        acc.extend(storage.take().iter().filter_map(recording_chunk));
        if done(&acc) || Instant::now() >= deadline {
            return acc;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
}

/// Chunks whose entity is `root` or a descendant `root/…` — a topic's render
/// subtree (a `Path3D` logs the polyline at `root` and its vertices at
/// `root/viz-vertices`).
fn subtree<'a>(chunks: &'a [RenderedChunk], root: &str) -> Vec<&'a RenderedChunk> {
    let child = format!("{root}/");
    chunks
        .iter()
        .filter(|c| c.entity == root || c.entity.starts_with(&child))
        .collect()
}

/// Does any chunk in `root`'s subtree carry the given rerun archetype?
fn subtree_has_archetype(chunks: &[RenderedChunk], root: &str, archetype: &str) -> bool {
    subtree(chunks, root)
        .iter()
        .any(|c| c.archetypes.iter().any(|a| a == archetype))
}

/// THE acceptance test: a nav2 `/plan` from the production CDR codec DRAWS a
/// polyline (`LineStrips3D`) + its waypoints (`Points3D`), never a text dump.
#[test]
fn plan_from_cdr_draws_a_polyline_not_text_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("plan_draws");
    let topic = "/vizd/plan";

    // The production ingress path: a hand-written CDR body → the REAL CdrCodec.
    let codec = builtin_codec();
    let frame = plan_frame(&codec, &PATH_ORACLE);

    // Anchor the produced frame to the hand oracle BEFORE publishing, so the
    // daemon's render is proven to be of the REAL decoded waypoints (not a
    // self-compare): the codec-produced canonical framing walks to exactly the
    // three hand-written positions, in order (stamped ⇒ an ORDERED path).
    let expected: Vec<[f32; 3]> = PATH_ORACLE
        .iter()
        .map(|(p, ..)| [p[0] as f32, p[1] as f32, p[2] as f32])
        .collect();
    let walker = builtin_walker();
    let fv = walker
        .walk_by_hash(&frame)
        .expect("walk codec-produced Path");
    match scan_element_arrays(&fv) {
        ElementArrayScan::Geometry(parts) => {
            assert_eq!(parts.field, "poses");
            assert_eq!(
                parts.geometry,
                ElementGeometry::Path(expected),
                "codec-produced /plan must decode to the hand-written ordered waypoints"
            );
        }
        other => panic!("expected a decodable Geometry path, got {other:?}"),
    }

    let _pub = Publisher::spawn_fixed_frame(Arc::clone(&mgr), topic, frame);
    let (worker, flush, storage) = memory_worker("plan_draws");
    let (socket, dir) = temp_socket("plan_draws");
    let mut daemon: RunningDaemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // The placement resolves to the DRAWING archetype (hand oracle) — the schema
    // is `nav_msgs/Path`, name-mapped to `Path3D` → a spatial3d view.
    let att = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(att["ok"].as_bool(), Some(true));
    assert_eq!(att["schema"].as_str(), Some("nav_msgs/Path"));
    assert_eq!(att["archetype"].as_str(), Some("Path3D"));
    assert_eq!(
        att["components"],
        serde_json::json!(["LineStrips3D", "Points3D"])
    );
    // The polyline lands in spatial3d, which is what this arm is about.
    //
    // This deliberately does not assert the `text_document` COMPANION
    // here. The companion does not go to every degradable kind UNCONDITIONALLY, so it is not a
    // fact about `Path3D` that is safe to pin at any instant; it is a fact about
    // THIS TOPIC AT THIS MOMENT — refused once the topic proves it is rendering —
    // and this reply is built microseconds after the tap is created, racing the
    // drain loop for whether the first frame has rendered yet. Pinning it here
    // would be pinning who won that race — without this restraint, these
    // two Path arms fail on ALTERNATE runs, the race resolving a different
    // way each time. The companion's behaviour is pinned where it is deterministic —
    // `cerulion_viz`'s `dump_companion_test.rs` for the decision, and
    // `a_rendering_topic_loses_its_dump_pane_and_regains_it_on_degradation_e2e`
    // for the live transition, including on the INSTALLED blueprint.
    assert!(
        att["view_kinds"]
            .as_array()
            .expect("view_kinds is a list")
            .contains(&serde_json::json!("spatial3d")),
        "a Path3D placement must put the polyline in spatial3d: {}",
        att["view_kinds"]
    );
    let plan_entity = att["entity"].as_str().expect("entity").to_string();
    let vertices_entity = format!("{plan_entity}/viz-vertices");

    // Wait until EVERY archetype the assertions below require has rendered.
    //
    // This watched `LineStrips3D` ALONE and then asserted `Points3D`
    // synchronously — waits-for-A-asserts-B, the rendezvous family, where
    // the wait condition covers a strict SUBSET of what the assertions need. The
    // two are two separate `rec.log()` calls to two distinct entity paths
    // (`archetype.rs::log_path3d` logs the strip at `entity`, then the waypoints at
    // `{entity}/viz-vertices`), so they land as SEPARATE chunks in the memory sink
    // and `accumulate_recording_chunks` RETURNS the instant its condition is met.
    // On a loaded runner the assert therefore ran in the window between them: main
    // push CI (macOS viz-tests) failed with the subtree holding exactly the one
    // `LineStrips3D` chunk.
    //
    // Widening can only make the run accumulate MORE chunks before asserting, so
    // the negative `TextDocument` assertion below is strictly stronger, never
    // weaker. The deadline stays seconds-scale — it is a liveness ceiling, never a
    // wall stated in units of the poll interval.
    let chunks = accumulate_recording_chunks(&flush, &storage, Duration::from_secs(5), |c| {
        subtree_has_archetype(c, &plan_entity, "LineStrips3D")
            && subtree_has_archetype(c, &vertices_entity, "Points3D")
    });

    // THE acceptance assertions: `/plan` drew a line-strip + its vertices, and
    // did NOT degrade to a text dump.
    assert!(
        subtree_has_archetype(&chunks, &plan_entity, "LineStrips3D"),
        "`/plan` must render a LineStrips3D polyline under {plan_entity}; rendered subtree: {:?}",
        subtree(&chunks, &plan_entity)
    );
    assert!(
        subtree_has_archetype(&chunks, &vertices_entity, "Points3D"),
        "`/plan` must render its waypoints as Points3D under {vertices_entity}; subtree: {:?}",
        subtree(&chunks, &plan_entity)
    );
    assert!(
        !subtree_has_archetype(&chunks, &plan_entity, "TextDocument"),
        "`/plan` must NOT degrade to a TextDocument dump — it DREW; subtree: {:?}",
        subtree(&chunks, &plan_entity)
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

/// The idle-plan protection: an IDLE `/plan` carrying ZERO poses still
/// resolves to the DRAWING archetype (never frozen to a text view), and does NOT
/// degrade to a text dump even though it draws nothing.
#[test]
fn an_idle_empty_plan_resolves_to_the_drawing_archetype_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("plan_empty");
    let topic = "/vizd/plan";

    // An idle nav2 robot: a well-formed `/plan` with ZERO poses (the state at
    // attach time), produced by the same production codec.
    let codec = builtin_codec();
    let frame = plan_frame(&codec, &[]);
    let _pub = Publisher::spawn_fixed_frame(Arc::clone(&mgr), topic, frame);

    let (worker, flush, storage) = memory_worker("plan_empty");
    let (socket, dir) = temp_socket("plan_empty");
    let mut daemon: RunningDaemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // The headline: an EMPTY array carries no shape, so shape inference alone
    // would freeze `/plan` into a view-less / text archetype. The NAME mapping
    // keeps it on the drawing archetype regardless.
    let att = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(att["ok"].as_bool(), Some(true));
    assert_eq!(att["schema"].as_str(), Some("nav_msgs/Path"));
    assert_eq!(
        att["archetype"].as_str(),
        Some("Path3D"),
        "an idle /plan must resolve to the DRAWING archetype, not AnyValues/text"
    );
    // The drawing archetype's own view. The companion is deliberately NOT
    // pinned at this instant — see the sibling `plan_from_cdr_...` arm for why
    // the render-proof signal makes it a race, and where it IS pinned.
    assert!(
        att["view_kinds"]
            .as_array()
            .expect("view_kinds is a list")
            .contains(&serde_json::json!("spatial3d")),
        "an idle /plan is still placed in spatial3d: {}",
        att["view_kinds"]
    );
    let plan_entity = att["entity"].as_str().expect("entity").to_string();

    // Prove the empty frames actually FLOW and are DISPATCHED (not that nothing
    // rendered because nothing arrived): status counts frames > 0.
    assert!(
        wait_until(Duration::from_secs(3), || {
            let status = client.request(r#"{"id":2,"method":"status"}"#);
            entry_for(&status["topics"], "topic", topic)
                .and_then(|st| st["frames"].as_u64())
                .unwrap_or(0)
                > 0
        }),
        "the empty /plan frames must be tapped + dispatched (status frames > 0)"
    );

    // Give any (mis)render a bounded window, then assert the empty plan drew
    // NOTHING and — crucially — did NOT degrade to a text dump (the topic must
    // not flip to TextDocument as the plan empties/refills).
    let chunks = accumulate_recording_chunks(&flush, &storage, Duration::from_secs(2), |_| false);
    assert!(
        !subtree_has_archetype(&chunks, &plan_entity, "TextDocument"),
        "an empty /plan must NOT degrade to a TextDocument dump; subtree: {:?}",
        subtree(&chunks, &plan_entity)
    );
    assert!(
        !subtree_has_archetype(&chunks, &plan_entity, "LineStrips3D"),
        "an empty /plan draws no polyline; subtree: {:?}",
        subtree(&chunks, &plan_entity)
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

/// The negative control (proves DISCRIMINATION, not just the happy path): a
/// genuinely-undecodable `/plan` element array (a packed, non-canonical element shape)
/// degrades to the element-enumerating text dump — a `TextDocument`, NEVER a
/// polyline guessed from bytes.
#[test]
fn an_undecodable_plan_degrades_to_a_text_dump_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("plan_opaque");
    let topic = "/vizd/plan";

    let frame = build_undecodable_path_frame();

    // Premise (also the layout drift guard): the walker REFUSES the packed
    // element bodies — the `poses` field stays `NestedArrayOpaque`, so the sink
    // never sees decodable geometry.
    let walker = builtin_walker();
    let fv = walker.walk_by_hash(&frame).expect("walk undecodable Path");
    assert!(
        matches!(
            fv.field("poses"),
            Some(FrameValueKind::NestedArrayOpaque(_))
        ),
        "premise: a packed element body must stay OPAQUE, got {:?}",
        fv.field("poses")
    );
    assert_eq!(scan_element_arrays(&fv), ElementArrayScan::Absent);

    let _pub = Publisher::spawn_fixed_frame(Arc::clone(&mgr), topic, frame);
    let (worker, flush, storage) = memory_worker("plan_opaque");
    let (socket, dir) = temp_socket("plan_opaque");
    let mut daemon: RunningDaemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // Placement still resolves `Path3D` by NAME (the schema IS `nav_msgs/Path`) —
    // the difference from the happy path is entirely what RENDERS.
    let att = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(att["ok"].as_bool(), Some(true));
    assert_eq!(att["archetype"].as_str(), Some("Path3D"));
    let plan_entity = att["entity"].as_str().expect("entity").to_string();

    // Wait until the daemon degrades to the text dump.
    let chunks = accumulate_recording_chunks(&flush, &storage, Duration::from_secs(5), |c| {
        subtree_has_archetype(c, &plan_entity, "TextDocument")
    });

    assert!(
        subtree_has_archetype(&chunks, &plan_entity, "TextDocument"),
        "an undecodable /plan must degrade to a TextDocument dump; subtree: {:?}",
        subtree(&chunks, &plan_entity)
    );
    assert!(
        !subtree_has_archetype(&chunks, &plan_entity, "LineStrips3D"),
        "an undecodable /plan must NOT be mis-drawn as a polyline; subtree: {:?}",
        subtree(&chunks, &plan_entity)
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

/// The [`Publisher`] helpers' POSTCONDITION is that
/// when `spawn` returns, the topic is ALREADY discoverable.
///
/// This is the one property ~70 tests in this file silently depend on.
/// A constructor that calls `create_publisher` INSIDE the spawned
/// thread returns after `thread::spawn` and before the
/// `{topic}/data` service exists, with no handshake between the two. Every
/// caller that then reads an enumeration is racing it —
/// `discover_reports_a_distinct_entity_for_every_colliding_go2_topic_e2e` loses
/// that race on a loaded macOS runner (`/lf/sportmodestate must be discovered`,
/// corpus entry 0 of 13, absent from a hermetic daemon's `discover`, which for a
/// topic with a live publisher and no mirrors can only mean the service was not
/// there yet).
///
/// MEASURED with in-thread creation on that same 13-topic corpus, 8 trials: `spawn`
/// returned in 82–797 µs while the services took 36–290 ms to all appear, and the
/// first `list_topics()` after every `spawn()` returned was missing **13 of 13**
/// in seven trials and 11 of 13 in the eighth. So this test is not probabilistic
/// against in-thread creation — it is a near-deterministic kill.
///
/// The assertion is deliberately the FIRST enumeration, with NO retry: a retry
/// would re-admit exactly the window being closed. It reads `list_topics()`
/// directly (not `discover`) so it pins the helper rather than the daemon, and it
/// spawns a BURST because the burst is what makes the gap wide.
///
/// Both constructors are covered: `spawn_fixed_frame` has the identical
/// exposure and the identical guarantee.
#[test]
fn publisher_spawn_is_visible_before_it_returns() {
    let mgr = isolated_transport("spawn_visibility");
    let streaming: Vec<String> = (0..8).map(|i| format!("/vis/stream{i}")).collect();
    let fixed: Vec<String> = (0..5).map(|i| format!("/vis/fixed{i}")).collect();

    let _pubs: Vec<Publisher> = streaming
        .iter()
        .map(|t| Publisher::spawn(Arc::clone(&mgr), t))
        .chain(fixed.iter().map(|t| {
            Publisher::spawn_fixed_frame(
                Arc::clone(&mgr),
                t,
                build_vector3_frame(0, PUBLISH_STAMP_STEP_NS, [1.0, 2.0, 3.0]),
            )
        }))
        .collect();

    // THE pin: one enumeration, taken immediately, no retry.
    let listed = mgr.list_topics().expect("enumerate");
    let missing: Vec<&String> = streaming
        .iter()
        .chain(fixed.iter())
        .filter(|t| !listed.contains(t))
        .collect();
    assert!(
        missing.is_empty(),
        "`Publisher::spawn`/`spawn_fixed_frame` returned before \
         their iceoryx2 services existed, so the very next enumeration missed \
         {}/{} topics: {missing:?} (enumerated: {listed:?})",
        missing.len(),
        streaming.len() + fixed.len()
    );
}

// ── The acceptance oracle: no two topics share an entity ────────────────────

/// The END-TO-END acceptance for collision-free entity paths, over the
/// REAL daemon's `discover` response — the surface the collision is visible on.
/// MEASURED on a Unitree Go2 with last-segment entity paths: 75 topics ⇒ **6 entity paths
/// covering 38 topics**.
///
/// The corpus is the REAL colliding topic names from that measurement, with each
/// of the 6 groups represented (the 3-way `sportmodestate` split in
/// full, plus the 2-way `state`/`lowstate`/`odom` pairs and 2 of the 15
/// `/api/*/{request,response}` pairs). Each gets a REAL publisher, so `discover`
/// reports what the daemon genuinely derived — not a unit-test call.
///
/// Two assertions, both against hand oracles: every topic's reported `entity` is
/// the exact expected path, and the reported entities are all DISTINCT. The
/// distinctness assert is what fails with last-segment entity paths (13 topics ⇒ 6 entities).
#[test]
fn discover_reports_a_distinct_entity_for_every_colliding_go2_topic_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("no_collisions");

    // (topic, the HAND-WRITTEN expected entity, the collapsed last-segment entity)
    let corpus: &[(&str, &str, &str)] = &[
        // The 3-way collision, in full.
        (
            "/lf/sportmodestate",
            "world/lf/sportmodestate",
            "world/odom/base/sportmodestate",
        ),
        (
            "/mf/sportmodestate",
            "world/mf/sportmodestate",
            "world/odom/base/sportmodestate",
        ),
        (
            "/sportmodestate",
            "world/sportmodestate",
            "world/odom/base/sportmodestate",
        ),
        // The three 2-way collisions.
        (
            "/lf/lowstate",
            "world/lf/lowstate",
            "world/odom/base/lowstate",
        ),
        ("/lowstate", "world/lowstate", "world/odom/base/lowstate"),
        (
            "/audiohub/player/state",
            "world/audiohub/player/state",
            "world/odom/base/state",
        ),
        ("/rtc/state", "world/rtc/state", "world/odom/base/state"),
        // Two Odometry topics on ONE entity is the worst case: both log a
        // Transform3D to it, overwriting each other — a wrong SPATIAL answer.
        (
            "/uslam/frontend/odom",
            "world/uslam/frontend/odom",
            "world/odom/base/odom",
        ),
        (
            "/uslam/localization/odom",
            "world/uslam/localization/odom",
            "world/odom/base/odom",
        ),
        // 2 of the 15+14 request/response pairs (the 15-way title half is pinned
        // at full width in `topic_view_identity_test`).
        (
            "/api/audiohub/request",
            "world/api/audiohub/request",
            "world/odom/base/request",
        ),
        (
            "/api/audiohub/response",
            "world/api/audiohub/response",
            "world/odom/base/response",
        ),
        (
            "/api/vui/request",
            "world/api/vui/request",
            "world/odom/base/request",
        ),
        (
            "/api/vui/response",
            "world/api/vui/response",
            "world/odom/base/response",
        ),
    ];

    // Real publishers, so `discover` sees real SHM services.
    let _pubs: Vec<Publisher> = corpus
        .iter()
        .map(|(t, _, _)| Publisher::spawn(Arc::clone(&mgr), t))
        .collect();

    let (worker, _flush, _storage) = memory_worker("no_collisions");
    let (socket, dir) = temp_socket("no_collisions");
    let mut daemon: RunningDaemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");

    let mut client = Client::connect(&socket);
    let disc = client.request(r#"{"id":1,"method":"discover"}"#);
    assert_eq!(disc["ok"].as_bool(), Some(true));

    let mut reported: Vec<(String, String)> = Vec::new();
    for (topic, expected, collapsed) in corpus {
        let e = entry_for(&disc["topics"], "topic", topic)
            .unwrap_or_else(|| panic!("{topic} must be discovered"));
        let entity = e["entity"].as_str().unwrap_or_default().to_string();
        assert_eq!(&entity, expected, "{topic} → entity");
        assert_ne!(
            &entity, collapsed,
            "{topic} must not render at the collapsed {collapsed}"
        );
        reported.push((topic.to_string(), entity));
    }

    // THE contract: N topics ⇒ N distinct entities. With last-segment entity paths these 13 topics
    // report SIX entities, so checking two of them in the sidebar draws them into
    // the same place with no indication anything is wrong.
    let distinct: std::collections::BTreeSet<&String> = reported.iter().map(|(_, e)| e).collect();
    assert_eq!(
        distinct.len(),
        corpus.len(),
        "every topic must own its entity; reported: {reported:#?}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

/// The `representation` verb end to end — over the REAL socket, against
/// a REAL attached tap, with the choice surviving a detach + re-attach.
///
/// This is the "not inert" pin for the whole feature: the pure resolver, the
/// sink and the layout are each pinned in `cerulion_viz`, and this is what proves
/// a client can actually reach them. The line it sends is byte-identical to the
/// one Studio's affordance builds (`test/sidebar-representation.test.ts`), and
/// the same line is pinned against the shipped parser in
/// `protocol::tests::parse_representation_line`.
///
/// The re-attach arm is the PERSISTENCE-SCOPE decision, made explicit: a
/// representation is a preference ABOUT a topic, not state OF an attachment, so
/// unchecking a row and checking it again must not silently revert it.
#[test]
fn representation_verb_reaches_a_live_tap_and_survives_a_re_attach() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("representation");
    let topic = "/vizd/rep";
    let _pub = Publisher::spawn(Arc::clone(&mgr), topic);

    let (worker, _flush, _storage) = memory_worker("representation");
    let (socket, dir) = temp_socket("representation");
    let mut daemon: RunningDaemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // The view kinds a row reports come from the topic's archetype until somebody
    // overrides it: a Vector3 is `Scalars` → one plot.
    //
    // Read off `list` rather than off the attach REPLY: the reply resolves the
    // archetype from a BOUNDED PEEK, so on a busy binary it legitimately comes
    // back null and settles a poll later (observed flaking once a second
    // daemon-driving arm joined this binary). Waiting asserts the same two facts
    // without pinning them to the first frame's timing.
    let att = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(att["ok"].as_bool(), Some(true));
    assert!(
        wait_until(Duration::from_secs(5), || {
            let list = client.request(r#"{"id":100,"method":"list"}"#);
            entry_for(&list["attached"], "topic", topic)
                .and_then(|r| r["archetype"].as_str().map(str::to_owned))
                .as_deref()
                == Some("Scalars")
        }),
        "the attached Vector3 must resolve to Scalars"
    );
    let settled = client.request(r#"{"id":101,"method":"list"}"#);
    let settled_row = entry_for(&settled["attached"], "topic", topic).expect("attached row");
    assert_eq!(
        settled_row["view_kinds"],
        serde_json::json!(["time_series"])
    );

    // THE gesture: the operator asks for the message dump beside the plot.
    let rep = client.request(&format!(
        r#"{{"id":2,"method":"representation","topic":"{topic}","representation":"both"}}"#
    ));
    assert_eq!(rep["ok"].as_bool(), Some(true));
    assert_eq!(rep["topic"].as_str(), Some(topic));
    assert_eq!(rep["representation"].as_str(), Some("both"));
    assert_eq!(
        rep["applied"].as_bool(),
        Some(true),
        "the topic is attached, so the choice reached the render — not merely \
         remembered: {rep}"
    );

    // …and the row now reports BOTH views, so what the daemon tells a client
    // matches what it is placing.
    let list = client.request(r#"{"id":3,"method":"list"}"#);
    let row = entry_for(&list["attached"], "topic", topic).expect("attached row");
    assert_eq!(
        row["view_kinds"],
        serde_json::json!(["time_series", "text_document"]),
        "an overridden row must not keep reporting its archetype's own views: {list}"
    );
    // …and the row carries the choice ITSELF, so the sidebar renders its control
    // from the daemon's truth rather than from what it last sent.
    assert_eq!(row["representation"].as_str(), Some("both"), "{list}");

    // A detach + re-attach — the user unchecking and re-checking a box — keeps
    // the choice. (Nothing else in the daemon survives a detach except the
    // provenance tombstone, so this is a deliberate placement, not an accident.)
    let det = client.request(&format!(
        r#"{{"id":4,"method":"detach","topic":"{topic}"}}"#
    ));
    assert_eq!(det["ok"].as_bool(), Some(true));
    let re = client.request(&format!(
        r#"{{"id":5,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(re["ok"].as_bool(), Some(true));
    assert_eq!(
        re["view_kinds"],
        serde_json::json!(["time_series", "text_document"]),
        "a re-checked row renders the way the operator left it: {re}"
    );

    // Back to automagic: the choice CLEARS rather than lingering as a default.
    let back = client.request(&format!(
        r#"{{"id":6,"method":"representation","topic":"{topic}","representation":"auto"}}"#
    ));
    assert_eq!(back["ok"].as_bool(), Some(true));
    assert_eq!(back["representation"].as_str(), Some("auto"));
    let list = client.request(r#"{"id":7,"method":"list"}"#);
    let row = entry_for(&list["attached"], "topic", topic).expect("attached row");
    assert_eq!(row["view_kinds"], serde_json::json!(["time_series"]));
    assert!(
        row.get("representation").is_none(),
        "`auto` is ABSENT on the wire, not the string \"auto\" — one encoding for \
         \"nobody chose\", and a pre-967 desk's bytes unchanged: {list}"
    );

    // An UNRECOGNIZED value is refused, naming the accepted set — never answered
    // `ok` for a click that changed nothing.
    let bad = client.request(&format!(
        r#"{{"id":8,"method":"representation","topic":"{topic}","representation":"plot"}}"#
    ));
    assert_eq!(bad["ok"].as_bool(), Some(false));
    let msg = bad["error"].as_str().unwrap_or_default();
    assert!(
        msg.contains("plot") && msg.contains("auto") && msg.contains("both"),
        "the refusal must name the offending value AND the accepted set: {msg}"
    );

    // A topic that is NOT attached is a success that is REMEMBERED, not applied —
    // the two are told apart on the wire so the row does not have to guess.
    let unattached = client.request(
        r#"{"id":9,"method":"representation","topic":"/vizd/never-attached","representation":"text"}"#,
    );
    assert_eq!(unattached["ok"].as_bool(), Some(true));
    assert_eq!(
        unattached["applied"].as_bool(),
        Some(false),
        "a remembered choice reports applied:false: {unattached}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

/// Every `(entity path, component descriptor)` pair the sink emitted — read back
/// off the REAL rerun store, so this observes what a VIEWER would rather than a
/// chunk count. (Crib: `sink_dispatch_test::logged_components`.)
fn logged_components(storage: &rerun::sink::MemorySinkStorage) -> Vec<(String, String)> {
    let mut out = Vec::new();
    for msg in storage.take() {
        let rerun::log::LogMsg::ArrowMsg(_, arrow_msg) = msg else {
            continue;
        };
        let chunk = rerun::log::Chunk::from_arrow_msg(&arrow_msg).expect("decode chunk");
        let entity = chunk
            .entity_path()
            .to_string()
            .trim_start_matches('/')
            .to_string();
        for descr in chunk.components().keys() {
            out.push((entity.clone(), descr.as_str().to_string()));
        }
    }
    out
}

/// **"Back to Automatic" must reach the RENDER, even across a
/// detach.** The quadrant a set-only reconciler cannot express.
///
/// The daemon keeps the choice per TOPIC and the worker keeps it per ROUTE KEY.
/// "Cleared" is an ABSENCE in the daemon's map, and `detach` erases the `stats`
/// entry the verb needs to reach the worker at all — so a reconciler that
/// returned early on absence could SET but never CLEAR. This sequence is the
/// proof: set `both`, DETACH, return to `auto` while detached (the verb accepts
/// it, `applied:false`), re-attach.
///
/// The oracle is the SINK, deliberately: `list` reads the daemon map, which is
/// correct in both worlds, so every wire surface would report `auto` while the
/// render stayed on the stale override — visual half suppressed entirely, a
/// `TextDocument` logged into a topic whose views carry none. Only the store can
/// tell them apart.
#[test]
fn returning_to_automatic_across_a_detach_reaches_the_render() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("clear");
    let topic = "/vizd/clear";
    let _pub = Publisher::spawn(Arc::clone(&mgr), topic);

    let (worker, flush, storage) = memory_worker("clear");
    let (socket, dir) = temp_socket("clear");
    let mut daemon: RunningDaemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
    ));
    let rep = client.request(&format!(
        r#"{{"id":2,"method":"representation","topic":"{topic}","representation":"both"}}"#
    ));
    assert_eq!(rep["applied"].as_bool(), Some(true));
    // The override really is in force at the sink: a `TextDocument` appears
    // beside the Vector3's scalars. (Without this the arm below could pass on a
    // feature that never engaged.)
    assert!(
        wait_until(Duration::from_secs(5), || {
            flush.flush_blocking().ok();
            logged_components(&storage)
                .iter()
                .any(|(_, c)| c.starts_with("TextDocument"))
        }),
        "the `both` override must reach the render before the clear is meaningful"
    );

    // Detach — this is what erases the route key the verb pushes through — then
    // return to automatic while DETACHED, then re-attach.
    client.request(&format!(
        r#"{{"id":3,"method":"detach","topic":"{topic}"}}"#
    ));
    let back = client.request(&format!(
        r#"{{"id":4,"method":"representation","topic":"{topic}","representation":"auto"}}"#
    ));
    assert_eq!(back["ok"].as_bool(), Some(true));
    assert_eq!(
        back["applied"].as_bool(),
        Some(false),
        "detached, so the clear is REMEMBERED — it is the re-attach that must apply it: {back}"
    );
    client.request(&format!(
        r#"{{"id":5,"method":"attach","topic":"{topic}"}}"#
    ));

    // Drain everything logged so far, then watch a FRESH window: the render must
    // be automagic — scalars flowing, and NO document.
    //
    // The window ACCUMULATES, because `logged_components` DRAINS the memory sink:
    // a poll loop that re-read it each tick would consume the very document this
    // arm exists to catch and then assert on an empty tail. (A re-reading poll
    // loop would pass the never-clears regression for exactly that reason.)
    flush.flush_blocking().ok();
    let _ = logged_components(&storage);
    let mut seen: Vec<(String, String)> = Vec::new();
    assert!(
        wait_until(Duration::from_secs(5), || {
            flush.flush_blocking().ok();
            seen.extend(logged_components(&storage));
            seen.iter().any(|(_, c)| c.starts_with("Scalars"))
        }),
        "the re-attached topic must render its plot again"
    );
    // …and keep watching past the first plot chunk, so a document that rides the
    // forced gate's slower cadence has a window to appear in.
    let deadline = Instant::now() + Duration::from_secs(2);
    while Instant::now() < deadline {
        flush.flush_blocking().ok();
        seen.extend(logged_components(&storage));
        std::thread::sleep(Duration::from_millis(50));
    }
    assert!(
        !seen.iter().any(|(_, c)| c.starts_with("TextDocument")),
        "a cleared choice must reach the WORKER — a stale override keeps logging a \
         document while every wire surface reports auto: {seen:?}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

/// **A refused change must not be affirmed by the next poll.**
///
/// The daemon's map is what `list`, `status`, `layout_metadata` and
/// `attached_render_infos_inner` all read, so committing it before a push that
/// can fail leaves an `ok:false` response coexisting with a `list` reporting the
/// refused value — and the next reflow PLACING a `text_document` view the sink
/// was never told to fill. Worse than a transient: Studio's corrective `list`
/// reads THIS map, so the designated corrector would re-manufacture the lie on
/// every poll.
///
/// Driven with the worker's control sender released, which is how both real
/// failure modes (`Busy` on a wedged viewer, `WorkerGone` on a dead worker) reach
/// the verb.
#[test]
fn a_refused_representation_leaves_the_daemons_map_untouched() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("refused");
    let topic = "/vizd/refused";
    let _pub = Publisher::spawn(Arc::clone(&mgr), topic);

    let (worker, _flush, _storage) = memory_worker("refused");
    let (socket, dir) = temp_socket("refused");
    let mut daemon: RunningDaemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
    ));
    let ok = client.request(&format!(
        r#"{{"id":2,"method":"representation","topic":"{topic}","representation":"both"}}"#
    ));
    assert_eq!(
        ok["ok"].as_bool(),
        Some(true),
        "the control path works first"
    );

    // Now the push cannot land.
    daemon.close_worker_control_for_test();

    // (a) A NEW value is refused — and is NOT recorded.
    let refused = client.request(&format!(
        r#"{{"id":3,"method":"representation","topic":"{topic}","representation":"text"}}"#
    ));
    assert_eq!(refused["ok"].as_bool(), Some(false), "{refused}");
    let list = client.request(r#"{"id":4,"method":"list"}"#);
    let row = entry_for(&list["attached"], "topic", topic).expect("attached row");
    assert_eq!(
        row["representation"].as_str(),
        Some("both"),
        "a REFUSED change must leave the previous choice in force, not report the \
         value the render never saw: {list}"
    );
    assert_eq!(
        row["view_kinds"],
        serde_json::json!(["time_series", "text_document"]),
        "…and the views must still describe the choice that IS in force: {list}"
    );

    // (b) The symmetric inversion: a refused CLEAR must not report automagic
    // either. This is the arm a rollback-free implementation fails in the
    // direction nobody looks — the map removes the entry, the push fails, and the
    // sink keeps the override while every surface says `auto`.
    let refused = client.request(&format!(
        r#"{{"id":5,"method":"representation","topic":"{topic}","representation":"auto"}}"#
    ));
    assert_eq!(refused["ok"].as_bool(), Some(false), "{refused}");
    let list = client.request(r#"{"id":6,"method":"list"}"#);
    let row = entry_for(&list["attached"], "topic", topic).expect("attached row");
    assert_eq!(
        row["representation"].as_str(),
        Some("both"),
        "a refused CLEAR must not report automagic: {list}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

/// **An undecodable topic says so on the wire, over REAL
/// transport, through the REAL poll loop.**
///
/// The hazard: `walk_by_hash` refuses an unrecognized `schema_hash` at the hash
/// gate, before framing is ever consulted, and a consumer that answers that refusal
/// with `None` reports a topic whose frames this build cannot read
/// EXACTLY like a silent one — `schema: null`, `archetype: null` — while `frames`
/// climbs and the row reads live. The two have opposite remedies and the sidebar
/// must be able to tell them apart.
///
/// Both topics here carry a REAL publisher publishing REAL frames at the same
/// rate; the ONLY difference is the `schema_hash` in the header. The desk holds
/// `geometry_msgs/Vector3`, so the skewed topic's bytes are decodable in
/// principle and unreachable in practice — which is precisely what
/// the 48 corpus-qualification hash bumps produce across a half-deployed desk/robot pair.
///
/// The healthy sibling is the ANTI-TAUTOLOGY half, and it is asserted in the SAME
/// body against the SAME daemon: without it, a daemon that stamped `undecodable`
/// on every row would pass.
///
/// **The skewed publisher starts only AFTER the attach, and that is load-bearing.**
/// An attach performs its own bounded schema peek, which records the verdict too —
/// so a test that publishes first is satisfied by the peek alone and says nothing
/// about the poll thread. Publishing afterwards leaves the peek with no frame to
/// look at (`ResolveFailure::NoFrame`), so the only path that can produce this
/// verdict is the poll loop. It is also the ordinary live shape: you attach a
/// topic, then the robot starts publishing. Dropping
/// `record_resolution` from the poll loop passes a publish-first ordering of this
/// test and fails this one.
#[test]
fn an_undecodable_topic_is_distinguishable_from_a_silent_one_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("undecodable");
    let skewed = "/vizd/drift/skewed";
    let healthy = "/vizd/drift/healthy";

    // Real Vector3 bytes under a hash NO schema in this build resolves. Built by
    // flipping the real hash, so the frame is otherwise indistinguishable from
    // the healthy one — the test cannot pass on a size or shape check.
    let foreign_hash = <Vector3 as ShmMessage>::SCHEMA_HASH ^ 0xFFFF_0000;
    let skewed_frame = |seq: u32| {
        let mut f = build_vector3_frame(
            seq,
            PUBLISH_STAMP_STEP_NS * (seq as u64 + 1),
            [1.0, 2.0, 3.0],
        );
        let mut h = WireHeader::read_from_buf(&f[..WireHeader::SIZE]).expect("frame header");
        h.schema_hash = foreign_hash;
        h.write_to_buf(&mut f[..WireHeader::SIZE]);
        f
    };

    // The service EXISTS (so the topic is attachable) and is SILENT — nothing is
    // published until after the attach, so the attach's own peek cannot resolve
    // it and the poll loop is the only path left.
    let mut skewed_pub = mgr
        .create_publisher(skewed, MaxSliceLen::const_new(1 << 16), 0)
        .expect("skewed producer creates the service");
    let _healthy_pub = Publisher::spawn(Arc::clone(&mgr), healthy);

    let (worker, _flush, _storage) = memory_worker("undecodable_daemon");
    let (socket, dir) = temp_socket("undecodable_daemon");
    let mut daemon: RunningDaemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    for (id, topic) in [(1u64, skewed), (2, healthy)] {
        let att = client.request(&format!(
            r#"{{"id":{id},"method":"attach","topic":"{topic}"}}"#
        ));
        assert_eq!(att["ok"].as_bool(), Some(true), "attach {topic}: {att}");
    }

    // A SILENT topic must carry no verdict: "I cannot decode it" is a claim only
    // a received frame can support, and fabricating it here would re-create the
    // very conflation this fix removes, pointing the other way.
    let quiet = client.request(r#"{"id":3,"method":"status"}"#);
    let quiet_row = entry_for(&quiet["topics"], "topic", skewed).expect("attached row");
    assert_eq!(
        quiet_row["frames"].as_u64(),
        Some(0),
        "still silent: {quiet_row}"
    );
    assert!(
        quiet_row.get("undecodable").is_none(),
        "a silent topic must not be reported as undecodable: {quiet_row}"
    );

    // NOW publish — the poll thread is the only thing that can see these.
    let saw_verdict = wait_until(Duration::from_secs(5), || {
        for seq in 0..4u32 {
            let _ = skewed_pub.publish_raw(&skewed_frame(seq));
        }
        let st = client.request(r#"{"id":4,"method":"status"}"#);
        entry_for(&st["topics"], "topic", skewed).is_some_and(|r| !r["undecodable"].is_null())
    });
    assert!(
        saw_verdict,
        "the POLL LOOP must report a topic whose frames it cannot decode"
    );

    let st = client.request(r#"{"id":5,"method":"status"}"#);
    let bad = entry_for(&st["topics"], "topic", skewed).expect("skewed row");
    // The failure shape this guards against, asserted so a regression is
    // legible: frames ARE arriving and the schema is still null.
    assert!(
        bad["frames"].as_u64().is_some_and(|n| n > 0),
        "frames are arriving on the skewed topic: {bad}"
    );
    assert!(
        bad["schema"].is_null(),
        "an unresolvable hash resolves to no schema: {bad}"
    );
    // THE contract: the row says WHY, and points somewhere.
    let report = &bad["undecodable"];
    assert!(
        !report.is_null(),
        "a topic whose frames cannot be decoded must say so — otherwise it is \
         byte-identical to a silent one: {bad}"
    );
    assert_eq!(
        report["reason"].as_str(),
        Some("schema_unidentified"),
        "a LOCAL attach pins no type name, so the accurate verdict names none: {report}"
    );
    assert_eq!(
        report["schema_hash"].as_str(),
        Some(format!("0x{foreign_hash:016X}").as_str()),
        "the wire hash is the one fact the frame always carries: {report}"
    );
    assert!(
        report["schema"].is_null() && report["local_schema_hash"].is_null(),
        "an unidentified verdict must not invent a type it cannot know: {report}"
    );
    assert!(
        report["detail"]
            .as_str()
            .is_some_and(|d| d.contains("cerulion topic info")),
        "the detail must tell a non-technical user what to DO: {report}"
    );

    // ANTI-TAUTOLOGY, same daemon, same instant: the healthy sibling decodes, so
    // its row carries NO report at all — and the field is ABSENT, not `null`, so
    // a fully-decoding desk's wire is byte-identical to earlier.
    let good = entry_for(&st["topics"], "topic", healthy).expect("healthy row");
    assert_eq!(
        good["schema"].as_str(),
        Some("geometry_msgs/Vector3"),
        "the control topic really does decode: {good}"
    );
    assert!(
        good.get("undecodable").is_none(),
        "a decodable topic must not carry the key at all: {good}"
    );

    // `list` and `status` are two views of ONE fact and must not disagree.
    let ls = client.request(r#"{"id":6,"method":"list"}"#);
    let listed = entry_for(&ls["attached"], "topic", skewed).expect("listed row");
    assert_eq!(
        listed["undecodable"]["reason"].as_str(),
        Some("schema_unidentified"),
        "`list` carries the same verdict as `status`: {listed}"
    );
    assert!(
        entry_for(&ls["attached"], "topic", healthy)
            .expect("healthy listed row")
            .get("undecodable")
            .is_none(),
        "and the same silence for the healthy one: {ls}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ─── Vizd answers PROMPTLY and says whether re-asking can help ─────
//
// A 10 s convergence wait INSIDE the attach handler cannot work:
// `cerulion_cli_engine::viz_client` arms a 5 s `SO_RCVTIMEO` on
// every reply read, so the wait would not make `cerulion viz` slower — it would make the verb
// die on an errno, on exactly the cold-desk attach the wait exists to rescue. It would
// also be nearly inert: netd's `ever_settled` latches PERMANENTLY on the first non-empty
// gather (the sidebar's own `discover` sets it), after which the wait never fires.
//
// So vizd answers the CLOSED-WORLD question ("does this LAN serve it
// NOW?") in ONE round trip, and attaches a structured `retry` hint when the answer is a
// NOT-FOUND-YET. The client loop lives in `viz_client` (its own e2e is
// `cerulion_cli_engine/tests/viz_attach_convergence_test.rs`).

/// A scripted [`cerulion_vizd::DemandPlane`] that models a `cerulion-netd`
/// whose discovery converges after a declared number of gathers — and COUNTS gathers,
/// which is what pins "the daemon answers in ONE round trip".
///
/// `demand` creates a REAL publisher on the shared manager (as `MirroringCountingPlane`
/// does) so a resolved attach can actually succeed — Principle #13: a DI double over
/// the real transport, never fake data.
struct ConvergingPlane {
    manager: Arc<TransportManager>,
    robot: String,
    /// The topic the plane's catalog SERVES once it is converged. Point it at a
    /// DIFFERENT topic than the one under attach to model a partially-discovered
    /// fleet: the gather is non-empty and netd reports `Settled`, yet the asked-for
    /// topic is not in it.
    served_topic: String,
    schema_name: String,
    /// The 1-based gather index from which the plane starts serving. `1` = a warm
    /// daemon; `usize::MAX` = never converges.
    converge_on: usize,
    /// What netd would report for `plane_unsettled_ms`.
    unsettled_for: Option<Duration>,
    gathers: Arc<AtomicU32>,
    /// When set, every query fails (netd unreachable / not network-configured).
    fail: AtomicBool,
    mirrors: std::sync::Mutex<Vec<cerulion_core::transport::publisher::CerulionPublisher>>,
}

impl ConvergingPlane {
    fn new(
        manager: Arc<TransportManager>,
        robot: &str,
        served_topic: &str,
        schema_name: &str,
        converge_on: usize,
    ) -> Self {
        Self {
            manager,
            robot: robot.to_string(),
            served_topic: served_topic.to_string(),
            schema_name: schema_name.to_string(),
            converge_on,
            unsettled_for: None,
            gathers: Arc::new(AtomicU32::new(0)),
            fail: AtomicBool::new(false),
            mirrors: std::sync::Mutex::new(Vec::new()),
        }
    }

    /// Declare the answering daemon's un-settled plane age (the cap input,
    /// forwarded to the client on the retry hint).
    fn with_plane_age(mut self, age: Duration) -> Self {
        self.unsettled_for = Some(age);
        self
    }

    fn set_fail(&self, fail: bool) {
        self.fail.store(fail, Ordering::SeqCst);
    }

    fn gathers(&self) -> u32 {
        self.gathers.load(Ordering::SeqCst)
    }

    fn gather(&self) -> Result<CatalogGather, String> {
        if self.fail.load(Ordering::SeqCst) {
            return Err("spy: netd unreachable".to_string());
        }
        let n = self.gathers.fetch_add(1, Ordering::SeqCst) as usize + 1;
        Ok(if n >= self.converge_on {
            CatalogGather {
                catalogs: vec![oracle_catalog(
                    &self.robot,
                    &[(self.served_topic.as_str(), Some(self.schema_name.as_str()))],
                )],
                discovery: DiscoveryState::Settled,
                unsettled_for: None,
            }
        } else {
            CatalogGather {
                catalogs: Vec::new(),
                discovery: DiscoveryState::NotConverged,
                unsettled_for: self.unsettled_for,
            }
        })
    }
}

impl cerulion_vizd::DemandPlane for ConvergingPlane {
    fn demand(&self, _robot: &str, topic: &str, _schema_hash: u64) -> Result<(), String> {
        let already = self
            .manager
            .list_topics()
            .is_ok_and(|t| t.iter().any(|x| x == topic));
        if !already {
            let publisher = self
                .manager
                .create_publisher(topic, MaxSliceLen::const_new(1 << 16), 0)
                .map_err(|e| format!("spy mirror create failed: {e}"))?;
            self.mirrors.lock().unwrap().push(publisher);
        }
        Ok(())
    }
    fn release(&self, _robot: &str, _topic: &str) -> Result<(), String> {
        self.mirrors.lock().unwrap().clear();
        Ok(())
    }
    fn query_catalog(&self, _robot: &str) -> Result<Vec<cerulion_core::CatalogReply>, String> {
        self.gather().map(|g| g.catalogs)
    }
    fn query_catalog_with_discovery(&self, _robot: &str) -> Result<CatalogGather, String> {
        self.gather()
    }
    fn query_catalog_all(&self) -> Result<Vec<cerulion_core::CatalogReply>, String> {
        self.gather().map(|g| g.catalogs)
    }
    fn query_catalog_all_with_discovery(&self) -> Result<CatalogGather, String> {
        self.gather()
    }
    fn query_schema(
        &self,
        _robot: &str,
        _requested: &str,
    ) -> Result<Vec<cerulion_core::SchemaReply>, String> {
        Ok(Vec::new())
    }
}

/// Start a hermetic daemon over an injected [`ConvergingPlane`].
fn start_converging_daemon(
    tag: &str,
    mgr: &Arc<TransportManager>,
    plane: &Arc<ConvergingPlane>,
) -> (RunningDaemon, PathBuf, PathBuf) {
    let (worker, _flush, _storage) = memory_worker(tag);
    std::mem::forget((_flush, _storage));
    let (socket, dir) = temp_socket(tag);
    let daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");
    (daemon, socket, dir)
}

/// The BLOCKER fix, and the headline: **a not-converged attach answers in ONE
/// round trip and SAYS that re-asking can change the answer.**
///
/// Two properties in one body, and the first is the one that was broken:
///
/// * the daemon gathers EXACTLY ONCE — so no reply can outlive the 5 s `SO_RCVTIMEO`
///   the only in-repo control client arms on this socket (`viz_client`), which a
///   daemon-side 10 s wait did, killing `cerulion viz` on an errno;
/// * the failure carries `retry.discovery == "discovering"`, which is the machine-
///   readable form of the UNKNOWN answer — the discriminator a controller could
///   otherwise only get by string-matching prose the daemon is free to reword.
#[test]
fn an_unconverged_attach_answers_in_one_round_trip_and_is_marked_retryable_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport_with_network("prompt");
    let topic = "/prompt/vel";
    let plane = Arc::new(
        ConvergingPlane::new(
            Arc::clone(&mgr),
            "go2",
            topic,
            "geometry_msgs/Vector3",
            usize::MAX,
        )
        .with_plane_age(Duration::from_millis(1234)),
    );
    let (mut daemon, socket, dir) = start_converging_daemon("prompt_daemon", &mgr, &plane);
    let mut client = Client::connect(&socket);

    let started = Instant::now();
    let a = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
    ));
    let elapsed = started.elapsed();

    assert_eq!(a["ok"].as_bool(), Some(false), "nothing serves it yet: {a}");
    assert_eq!(
        plane.gathers(),
        1,
        "the daemon must answer in ONE round trip — a longer reply is not slower, it \
         is FATAL against viz_client's 5s SO_RCVTIMEO"
    );
    assert!(
        elapsed < Duration::from_secs(2),
        "and it must be prompt in wall time too (a generous ceiling; the point is that \
         it does not wait): {elapsed:?}"
    );
    assert_eq!(
        a["retry"]["discovery"].as_str(),
        Some("discovering"),
        "the failure is machine-readably a NOT-FOUND-YET, not a terminal absence: {a}"
    );
    assert_eq!(
        a["retry"]["plane_unsettled_ms"].as_u64(),
        Some(1234),
        "and netd's plane age is forwarded so the CLIENT can run the convergence cap: {a}"
    );
    // The prose half survives alongside the structured half.
    let err = a["error"].as_str().unwrap_or_default();
    assert!(
        err.contains("is UNKNOWN") && err.contains("Retry in a few seconds"),
        "the human-readable message is still accurate + actionable: {a}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

/// **A SETTLED gather that does not serve THIS topic is still a
/// NOT-FOUND-YET** — the partially-discovered-fleet fix.
///
/// netd's `ever_settled` latches PERMANENTLY on the first non-empty gather, and the
/// Studio sidebar's own `discover` sets it within seconds of connecting. So on any desk
/// where ONE robot has ever answered, an attach for a topic on a robot that is still
/// booting gets `Settled` + a NON-empty gather — and a predicate keyed on the gather's
/// emptiness proceeds straight into a confident terminal absence for a robot seconds from
/// discovery. The predicate is the SEAM's question ("does this answer cover THIS
/// topic"), not the gather's emptiness.
///
/// The `discovering` sibling is in the headline test; here the LABEL must be `settled`,
/// because the two say different things about how much was known and a client picks its
/// policy from them.
#[test]
fn a_settled_gather_that_omits_this_topic_is_still_marked_retryable_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport_with_network("fleet");
    let missing = "/fleet/robot_b/scan";
    // Robot A answered (converged, non-empty) — but it serves a DIFFERENT topic.
    let plane = Arc::new(ConvergingPlane::new(
        Arc::clone(&mgr),
        "go2a",
        "/fleet/robot_a/vel",
        "geometry_msgs/Vector3",
        1,
    ));
    let (mut daemon, socket, dir) = start_converging_daemon("fleet_daemon", &mgr, &plane);
    let mut client = Client::connect(&socket);

    let a = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{missing}"}}"#
    ));
    assert_eq!(a["ok"].as_bool(), Some(false));
    assert_eq!(
        a["retry"]["discovery"].as_str(),
        Some("settled"),
        "a settled LAN that does not serve it YET is still re-askable — a robot can \
         join at any moment, which is what this arm's own message tells the user to \
         wait for: {a}"
    );
    assert_eq!(plane.gathers(), 1, "still exactly one round trip");

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

/// The ANTI-TAUTOLOGY half: **a TERMINAL failure carries NO hint.**
///
/// Without this, "the hint is present" proves nothing — a daemon that stamped a hint on
/// every error would pass both arms above while telling a client to re-ask questions
/// whose answers cannot change.
///
/// The AMBIGUITY arm is the sharp case: several robots DO serve the topic, so the
/// gather is as converged and as non-empty as it will ever be, and the user has to
/// pick. Absence of the key is also the fail-closed default a daemon without
/// this hint produces.
#[test]
fn a_terminal_attach_failure_carries_no_retry_hint_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport_with_network("terminal");
    let shared = "/terminal/shared";

    /// Two robots BOTH serving `shared` — the ambiguity arm.
    struct TwoRobotPlane;
    impl cerulion_vizd::DemandPlane for TwoRobotPlane {
        fn demand(&self, _r: &str, _t: &str, _h: u64) -> Result<(), String> {
            Ok(())
        }
        fn release(&self, _r: &str, _t: &str) -> Result<(), String> {
            Ok(())
        }
        fn query_catalog(&self, _r: &str) -> Result<Vec<cerulion_core::CatalogReply>, String> {
            self.query_catalog_all()
        }
        fn query_catalog_all(&self) -> Result<Vec<cerulion_core::CatalogReply>, String> {
            Ok(vec![
                oracle_catalog(
                    "go2a",
                    &[("/terminal/shared", Some("geometry_msgs/Vector3"))],
                ),
                oracle_catalog(
                    "go2b",
                    &[("/terminal/shared", Some("geometry_msgs/Vector3"))],
                ),
            ])
        }
        fn query_catalog_all_with_discovery(&self) -> Result<CatalogGather, String> {
            Ok(CatalogGather {
                catalogs: self.query_catalog_all()?,
                discovery: DiscoveryState::Settled,
                unsettled_for: None,
            })
        }
        fn query_schema(
            &self,
            _r: &str,
            _q: &str,
        ) -> Result<Vec<cerulion_core::SchemaReply>, String> {
            Ok(Vec::new())
        }
    }

    let (worker, _flush, _storage) = memory_worker("term");
    let (socket, dir) = temp_socket("term");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::new(TwoRobotPlane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    let a = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{shared}"}}"#
    ));
    assert_eq!(a["ok"].as_bool(), Some(false));
    assert!(
        a["error"]
            .as_str()
            .unwrap_or_default()
            .contains("is served by robots"),
        "the ambiguity arm: {a}"
    );
    assert!(
        a.get("retry").is_none(),
        "re-asking cannot resolve an ambiguity — the user must pick, so NO hint: {a}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

/// **`discover` carries the discovery marker, and OMITS it when netd could not
/// be reached at all.**
///
/// The sidebar is a refreshing surface (Studio re-asks, and netd's
/// catalog changes are pushed to it), so it answers in one round trip; what it needs is the
/// MARKER, or a cold-start empty sidebar is indistinguishable on the wire from a
/// settled one and a controller has no choice but "no robots".
///
/// Three arms, and the third is the fail-closed one: a netd-unreachable `discover`
/// degrades to LOCAL-ONLY and must claim NOTHING about discovery. Without it, marking
/// that arm `settled` ("we answered, so call it searched") would tell Studio the LAN was
/// searched and empty on a desk where netd is simply not running.
#[test]
fn discover_carries_the_marker_and_omits_it_when_netd_is_unreachable_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport_with_network("discover");
    let topic = "/discover/vel";
    // Converge on the SECOND gather: the first `discover` is cold, the second warm.
    let plane = Arc::new(ConvergingPlane::new(
        Arc::clone(&mgr),
        "go2",
        topic,
        "geometry_msgs/Vector3",
        2,
    ));
    let (mut daemon, socket, dir) = start_converging_daemon("disc_daemon", &mgr, &plane);
    let mut client = Client::connect(&socket);

    let cold = client.request(r#"{"id":1,"method":"discover"}"#);
    assert_eq!(cold["ok"].as_bool(), Some(true));
    assert_eq!(plane.gathers(), 1, "one round trip per refresh");
    assert_eq!(
        cold["discovery"].as_str(),
        Some("discovering"),
        "an unconverged sidebar SAYS so, so an empty robots list is not a terminal \
         absence: {cold}"
    );
    // `robots` is `skip_serializing_if = "Vec::is_empty"`, so nothing-discovered is an
    // ABSENT key — which is exactly why the marker has to carry the verdict.
    assert_eq!(
        cold["robots"].as_array().map_or(0, Vec::len),
        0,
        "nothing discovered yet: {cold}"
    );

    let warm = client.request(r#"{"id":2,"method":"discover"}"#);
    assert_eq!(warm["discovery"].as_str(), Some("settled"), "{warm}");
    assert_eq!(plane.gathers(), 2, "still one round trip per refresh");
    let robots = warm["robots"].as_array().cloned().unwrap_or_default();
    assert_eq!(
        robots.len(),
        1,
        "the robot appears on the next refresh: {warm}"
    );
    assert_eq!(robots[0]["robot"].as_str(), Some("go2"), "{warm}");

    // FAIL-CLOSED: netd unreachable ⇒ the field is OMITTED, never a claim.
    plane.set_fail(true);
    let degraded = client.request(r#"{"id":3,"method":"discover"}"#);
    assert_eq!(
        degraded["ok"].as_bool(),
        Some(true),
        "a netd-unreachable discover still lists LOCAL topics: {degraded}"
    );
    assert!(
        degraded.get("discovery").is_none(),
        "an unreachable netd claims NOTHING about discovery — absence is UNKNOWN: \
         {degraded}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

/// The SCHEMA-LESS seam (`attach` WITH a robot, no `schema`) marks a missing
/// type retryable too — the same class one hop over, on the SINGLE-robot query verb.
///
/// Its sibling `fetch_and_seed_type` deliberately does NOT: by then the catalog has
/// named the producing robot, so the robot IS discovered and a schema miss is not a
/// convergence problem (the convergence wait's own decision for the second, robot-scoped query).
#[test]
fn the_schema_less_seam_marks_a_missing_type_retryable_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport_with_network("schemaless");
    let topic = "/schemaless/vel";
    // The robot's catalog is SETTLED and names a DIFFERENT topic.
    let plane = Arc::new(ConvergingPlane::new(
        Arc::clone(&mgr),
        "go2",
        "/schemaless/other",
        "geometry_msgs/Vector3",
        1,
    ));
    let (mut daemon, socket, dir) = start_converging_daemon("sl_daemon", &mgr, &plane);
    let mut client = Client::connect(&socket);

    let a = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}","robot":"go2"}}"#
    ));
    assert_eq!(a["ok"].as_bool(), Some(false));
    assert!(
        a["error"]
            .as_str()
            .unwrap_or_default()
            .contains("serves a catalog but names no ROS type"),
        "the settled terminal PROSE is unchanged: {a}"
    );
    assert_eq!(
        a["retry"]["discovery"].as_str(),
        Some("settled"),
        "but the wire says a later answer can still name a type for it: {a}"
    );
    assert_eq!(plane.gathers(), 1, "one round trip");

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ── Pre-decode frame loss is COUNTED and LOUD, never silent ──────────────────
//
// Context, because the shape of these two tests follows directly from it:
// "Does the tap lose access units desk-internally?" must be answerable from `status`. A
// measurement (a stride-1 `CERULION_H264_LAT_PROBE` capture against a live
// robot, joined by hand) showed the tap loses NOTHING — 2353 frames drained,
// zero wire-sequence gaps, `s2 == s3`. Without a gap counter on vizd's tap, answering
// that question costs a bespoke probe run, while
// `cerulion topic echo` has a gap counter of its own. These pin the counter that
// makes it a `status` read instead.
//
// The oracle is a CONSERVATION LAW — `delivered + missed == published` — chosen
// deliberately over an exact `missed` value: which frames a given poll catches
// depends on where the 16 ms tick lands inside the burst, so an exact count
// would be a load-dependent assertion (the load-sensitive class). The identity holds
// for EVERY interleaving, and it is also the actual Principle #6 claim: every
// frame the producer committed is either delivered or accounted for.

#[test]
fn frames_evicted_before_the_drain_are_counted_and_reported() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("loss");
    let topic = "/vizd/lossy";
    // A DELIBERATELY shallow subscriber queue (4 against the stock 16), so a
    // burst larger than it forces the `drop_oldest` eviction this counter
    // exists to report, with a small burst and a fast test.
    let mut config = mgr.default_topic_config();
    config.subscriber_max_buffer_size = 4;
    let mut publisher = mgr
        .create_publisher_with_topic_config(topic, MaxSliceLen::const_new(1 << 16), 0, config)
        .expect("shallow-queue producer attaches");

    let (worker, _flush, _storage) = memory_worker("loss");
    let (socket, dir) = temp_socket("loss");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");

    let mut client = Client::connect(&socket);
    let att = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(att["ok"].as_bool(), Some(true), "attach: {att}");

    // Establish the BASELINE first: until one poll has yielded there is no
    // previous newest sequence, and the guard correctly claims nothing (a tap
    // legitimately attaches mid-stream). Bursting before this would exercise
    // the no-claim arm, not the loss arm.
    let _ = publisher.publish_raw(&build_vector3_frame(0, 1_000, [0.0, 0.0, 0.0]));
    assert!(
        wait_until(Duration::from_secs(6), || {
            let st = client.request(r#"{"id":2,"method":"status"}"#);
            entry_for(&st["topics"], "topic", topic)
                .is_some_and(|r| r["frames"].as_u64().is_some_and(|n| n >= 1))
        }),
        "the baseline frame must be drained before the burst"
    );

    // THE BURST: far more frames than a 4-deep queue can hold, published as fast
    // as the loop runs, so it overflows between polls wherever the tick lands.
    const BURST: u32 = 400;
    for seq in 1..=BURST {
        let _ = publisher.publish_raw(&build_vector3_frame(
            seq,
            1_000 + u64::from(seq),
            [f64::from(seq), 0.0, 0.0],
        ));
    }

    let published = u64::from(BURST) + 1;
    assert!(
        wait_until(Duration::from_secs(10), || {
            let st = client.request(r#"{"id":3,"method":"status"}"#);
            entry_for(&st["topics"], "topic", topic).is_some_and(|r| {
                let seen = r["frames"].as_u64().unwrap_or(0);
                let missed = r["frames_missed"].as_u64().unwrap_or(0);
                seen + missed == published
            })
        }),
        "delivered + missed must account for every published frame"
    );

    let st = client.request(r#"{"id":4,"method":"status"}"#);
    let row = entry_for(&st["topics"], "topic", topic).expect("row");
    let seen = row["frames"].as_u64().expect("frames");
    let missed = row["frames_missed"]
        .as_u64()
        .expect("frames_missed is REPORTED");

    // THE HEADLINE: the loss really happened and is really counted. Without the
    // guard this is the silent class — `frames` simply climbs more slowly than
    // the producer published, and nothing anywhere says so.
    assert!(
        missed > 0,
        "a 400-frame burst into a 4-deep queue MUST evict frames; got missed={missed} seen={seen}"
    );
    assert_eq!(
        seen + missed,
        published,
        "conservation: delivered({seen}) + missed({missed}) == published({published})"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

/// The ANTI-TAUTOLOGY control, and the promise the guard makes to a healthy
/// desk: a paced stream reports EXACTLY zero. Without it the test above would
/// pass an implementation that reported loss on every multi-frame poll — which
/// is the normal shape under the bursty arrival measured on a live desk (inter-arrival
/// std 33 ms against a 34 ms period), i.e. the false positive that would matter
/// most in production.
#[test]
fn a_paced_stream_reports_exactly_zero_pre_decode_loss() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("clean");
    let topic = "/vizd/clean";
    // The STOCK queue depth — the shipping shape.
    let mut publisher = mgr
        .create_publisher(topic, MaxSliceLen::const_new(1 << 16), 0)
        .expect("producer attaches");

    let (worker, _flush, _storage) = memory_worker("clean");
    let (socket, dir) = temp_socket("clean");
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");

    let mut client = Client::connect(&socket);
    let att = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(att["ok"].as_bool(), Some(true), "attach: {att}");

    // 40 frames at ~100 Hz — comfortably inside a 16-deep queue at a 16 ms poll,
    // but fast enough that SEVERAL frames land in one poll, so this really does
    // exercise the multi-frame-batch arm rather than a trivial one-per-poll
    // stream. That is the arm a naive "seq jumped ⇒ loss" implementation fails.
    const FRAMES: u32 = 40;
    for seq in 0..FRAMES {
        let _ = publisher.publish_raw(&build_vector3_frame(
            seq,
            1_000 + u64::from(seq),
            [f64::from(seq), 0.0, 0.0],
        ));
        std::thread::sleep(Duration::from_millis(10));
    }

    assert!(
        wait_until(Duration::from_secs(10), || {
            let st = client.request(r#"{"id":2,"method":"status"}"#);
            entry_for(&st["topics"], "topic", topic)
                .is_some_and(|r| r["frames"].as_u64() == Some(u64::from(FRAMES)))
        }),
        "a paced stream must deliver every frame"
    );

    let st = client.request(r#"{"id":3,"method":"status"}"#);
    let row = entry_for(&st["topics"], "topic", topic).expect("row");
    assert_eq!(
        row["frames_missed"].as_u64(),
        Some(0),
        "a healthy paced stream reports EXACTLY zero loss: {row}"
    );
    // `Some(0)` and absent are different claims — a checked row must say so.
    assert!(
        !row["frames_missed"].is_null(),
        "a checked row REPORTS zero rather than omitting the field: {row}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}

// ════════════════════════════════════════════════════════════════════════════
// The dump companion is refused on a LIVE signal, and COMES BACK when
// a degradation fires — over the real daemon, the real worker and real frames.
//
// The layout DECISION is pure and pinned in `cerulion_viz`'s
// `dump_companion_test.rs`; the RECORDING of the signal is pinned in
// `marker_array_test.rs` (including a frame that genuinely draws). What only an
// e2e can show is the span between them, and before this fix that span did not
// exist: nothing on the daemon watched either live signal, so a change reached
// the viewer only if some unrelated attach/detach happened to reflow afterwards.
// A green pure suite cannot see that, which is exactly the inert-shipping shape.
// ════════════════════════════════════════════════════════════════════════════

/// A whole `visualization_msgs/MarkerArray` wire frame carrying `blob` as its one
/// variable field. Self-contained (the sibling builder lives in another crate's
/// test), with the two layout facts it depends on asserted rather than assumed —
/// so a codegen change fails HERE instead of silently producing a frame the
/// walker reads as something else.
fn marker_array_frame(blob: &[u8]) -> Vec<u8> {
    assert_eq!(
        <MarkerArray as ShmMessage>::WIRE_FIXED_SIZE,
        0,
        "MarkerArray gained a fixed section"
    );
    assert_eq!(
        <MarkerArray as ShmMessage>::VARIABLE_FIELD_COUNT,
        1,
        "MarkerArray no longer has exactly one variable field (`markers`)"
    );
    // One variable field ⇒ a single 8-byte offset entry.
    const TABLE: usize = 8;
    let mut payload = vec![0u8; TABLE];
    write_offset_entry(&mut payload, 0, 0, TABLE as u32, blob.len() as u32);
    payload.extend_from_slice(blob);
    let mut frame = vec![0u8; WireHeader::SIZE];
    WireHeader {
        schema_hash: <MarkerArray as ShmMessage>::SCHEMA_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: WireHeader::SIZE as u32,
        offset_table_count: 1,
        sequence: 0,
        timestamp_ns: 42_000,
    }
    .write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);
    frame
}

#[test]
fn a_rendering_topic_loses_its_dump_pane_and_regains_it_on_degradation_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("companion");
    let topic = "/vizd/markers";

    // HEALTHY: a canonical marker array the arm SCANS successfully. It carries no
    // markers, which is a legitimate live state (an idle publisher, or one whose
    // markers were all deleted) and is exactly what the signal measures — the arm
    // ran and did not dump. The DRAWING case is pinned beside the recorder, in
    // `marker_array_test.rs::a_frame_that_draws_...`, because what is under
    // test HERE is the crossing and the reflow, not which geometry was logged.
    let healthy = marker_array_frame(&0u32.to_le_bytes());
    // DEGRADED: element bodies the walker refuses — what a real rmw/bridge skew
    // produces, and the one shape that routes this arm to the field dump.
    let junk = marker_array_frame(&[0xABu8; 37]);

    let publisher = Publisher::spawn_fixed_frame(Arc::clone(&mgr), topic, healthy);
    let (worker, _flush, _storage) = memory_worker("companion");
    let (socket, dir) = temp_socket("companion");
    let mut daemon: RunningDaemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    let att = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(att["ok"].as_bool(), Some(true), "attach: {att}");
    assert_eq!(
        att["archetype"].as_str(),
        Some("MarkerArray"),
        "the witness must be a CONVERTIBLE archetype, or this test proves nothing: {att}"
    );

    let view_kinds = |client: &mut Client, id: u64| -> Option<Value> {
        let st = client.request(&format!(r#"{{"id":{id},"method":"status"}}"#));
        entry_for(&st["topics"], "topic", topic).map(|r| r["view_kinds"].clone())
    };

    // (1) The signal crosses worker → daemon and the companion GOES. A condition
    //     wait bounded in seconds, never a wall in units of the poll interval:
    //     load can only delay the worker's per-batch mirror refresh, never
    //     manufacture the transition.
    assert!(
        wait_until(Duration::from_secs(10), || {
            view_kinds(&mut client, 2) == Some(serde_json::json!(["spatial3d"]))
        }),
        "a MarkerArray topic whose arm is provably rendering must lose the default \
         status pane — got {:?}",
        view_kinds(&mut client, 3)
    );
    // THE APPLIED PLAN, not just the report. The two assertions above read the
    // worker's mirror through the control seam; this reads what was actually
    // installed as the viewer's layout, which is the only thing the operator sees.
    //
    // Both are needed: a
    // variant that deletes the drain loop's watcher entirely — so nothing ever
    // reflows — passes the report assertions AND any counter assertion
    // whose counter is bumped where the change is DETECTED rather than
    // where the layout is RE-DERIVED. The view count of the installed plan cannot
    // be faked that way.
    //
    // A CONDITION wait, not an instant read: the report is served off the mirror by
    // whichever control call asks, while the reflow lands on the drain loop's NEXT
    // pass, so reading at the moment the report flips races a thread boundary (an
    // instant read fails exactly that way). Load can DELAY that pass, never skip
    // it. Residual worth naming: for up to one pass, `status` reports the new
    // placement while the applied blueprint is still the old one — the INVERSE of
    // the drift this arm pins, and bounded by ≈ ONE poll interval (TWO under the
    // drain-loop pacing, where a paced pass spends a second sleep before it
    // reaches the watcher) rather than permanent.
    let applied_views = || {
        current_runtime_blueprint_plan()
            .map(|p| p.view_count())
            .unwrap_or(0)
    };
    // A `MarkerArray` renders in spatial3d, and the consolidated default folds
    // every spatial3d topic into the ONE Scene view — so the status companion is
    // the only pane this topic contributes, and the installed view COUNT is a
    // direct read of whether it is there: 2 with it, 1 without. The two waits below
    // form a SEQUENCE (2 -> 1 -> 2) that a static plan cannot satisfy.
    assert!(
        wait_until(Duration::from_secs(10), || applied_views() == 1),
        "the INSTALLED layout must be re-derived WITHOUT the status pane (Scene \
         alone), got {} views",
        applied_views()
    );
    assert!(
        wait_until(Duration::from_secs(10), || daemon
            .poll_loop_layout_signal_reflows()
            >= 1),
        "and the reflow must be attributed to the render-proof change"
    );
    let reflows_after_drop = daemon.poll_loop_layout_signal_reflows();

    // (1b) `discover` — the surface the Studio sidebar POLLS — must agree. It
    //      enumerates every local topic with NO attach filter, so this rendering
    //      topic is a discover row too, and a blanket default would make it the
    //      one surface still advertising a pane the installed blueprint has
    //      dropped: one run, two answers, which is the class the
    //      render-proof signal removes everywhere else.
    let disc = client.request(r#"{"id":10,"method":"discover"}"#);
    let disc_row = entry_for(&disc["topics"], "topic", topic)
        .unwrap_or_else(|| panic!("an ATTACHED topic is still a discover row: {disc}"));
    assert_eq!(
        disc_row["view_kinds"],
        serde_json::json!(["spatial3d"]),
        "`discover` must report the placement this desk is actually rendering, the \
         same one `status` reports: {disc_row}"
    );

    // (1c) THE OTHER TWO REPORTERS, read in the SAME state — the only one that
    //      discriminates. `layout_metadata_for_route`'s own doc says the pair exists
    //      so the four ATTACHED-topic reporters "cannot drift from the applied
    //      blueprint by forgetting a signal", and without these reads exactly one of
    //      them — `status` — is pinned by anything: reverting the signal
    //      on `list` plus BOTH attach replies together leaves every other `cerulion_vizd`
    //      test green.
    //
    //      WHERE these are read is the whole of their value:
    //      read after the degradation in (2), a blank-signal reporter and a
    //      live one BOTH say `["spatial3d","text_document"]`, so the assertions would be
    //      vacuous — measured, the revert passes them there. Here the topic is PROVEN, so
    //      blank says `["spatial3d","text_document"]` and live says `["spatial3d"]`.
    let list = client.request(r#"{"id":11,"method":"list"}"#);
    assert_eq!(
        entry_for(&list["attached"], "topic", topic).map(|r| r["view_kinds"].clone()),
        Some(serde_json::json!(["spatial3d"])),
        "`list` — the surface the Studio sidebar reads — must report the SAME \
         placement `status` does: {list}"
    );
    // An idempotent RE-attach: the reply is built from the live route, so it reports
    // this topic's placement NOW rather than the one it had when first attached
    // (when nothing had rendered yet and the companion was correctly kept).
    let re_att = client.request(&format!(
        r#"{{"id":12,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(
        re_att["already_attached"].as_bool(),
        Some(true),
        "precondition: this must be the idempotent re-attach, not a fresh one: {re_att}"
    );
    assert_eq!(
        re_att["view_kinds"],
        serde_json::json!(["spatial3d"]),
        "the attach reply — what a `cerulion viz` client lays out from — must carry \
         the live placement too: {re_att}"
    );

    // (2) THE PIN: the arm degrades, and the pane the dump lands in APPEARS. This
    //     is the dump-companion guarantee held end-to-end, which is what makes the
    //     refusal in (1) affordable at all.
    drop(publisher);
    let _degrading = Publisher::spawn_fixed_frame(Arc::clone(&mgr), topic, junk);
    assert!(
        wait_until(Duration::from_secs(10), || {
            view_kinds(&mut client, 4) == Some(serde_json::json!(["spatial3d", "text_document"]))
        }),
        "once a frame DEGRADES, the pane its dump renders in must come back — got \
         {:?}",
        view_kinds(&mut client, 5)
    );
    assert!(
        wait_until(Duration::from_secs(10), || applied_views() == 2),
        "and the INSTALLED layout must grow the status pane back (Scene + \
         text_document), got {} views",
        applied_views()
    );
    assert!(
        wait_until(Duration::from_secs(10), || daemon
            .poll_loop_layout_signal_reflows()
            > reflows_after_drop),
        "attributed to the degradation, not merely reported: reflows stuck at \
         {reflows_after_drop}"
    );

    // (3) THE CEILING. `Ctx::poll_layout_signal_reflows`'s doc calls the reflow
    //     BOUNDED — and every assertion above it is a FLOOR, so a watcher that
    //     never advances its baseline (i.e. reflows on every pass) would pass
    //     those assertions and be caught only by an unrelated harness quiescence
    //     guard whose own message blames machine load.
    //
    //     Load-safe in the direction that matters: contention can only make the loop
    //     run FEWER passes, so a ceiling on genuine-transition reflows cannot be
    //     violated by a slow runner — only by a watcher that fires on something
    //     other than a transition. The publisher is still streaming identical junk
    //     frames, both proof flags are sticky and no rendition set exists, so
    //     NOTHING in this window is a change.
    let settled = daemon.poll_loop_layout_signal_reflows();
    let passes_before = daemon.poll_loop_iterations();
    std::thread::sleep(Duration::from_millis(750));
    assert!(
        daemon.poll_loop_iterations() > passes_before,
        "ANTI-VACUITY: the drain loop must actually have run passes in this window, \
         or the ceiling below asserts nothing"
    );
    assert_eq!(
        daemon.poll_loop_layout_signal_reflows(),
        settled,
        "a steady stream of frames that change NEITHER live signal must reflow \
         NOTHING — the layout is re-derived and re-sent whole, so an unbounded \
         watcher competes with frames on the worker's bounded queue"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

// ── The VIDEO half of the runtime re-apply ──────────────────────────────────

/// The REAL Go2 360p SPS/PPS/IDR head (provenance: bytes 0x18.. of the committed
/// `go2_frontvideostream_360p_idr.cdr` capture — see
/// `cerulion_viz/lib/cerulion_viz/tests/video_h264_test.rs`). Copied rather than
/// shared because they are test constants of another package.
const GO2_SPS: &[u8] = &[
    0x67, 0x64, 0x10, 0x28, 0xac, 0x1b, 0x1a, 0xa0, 0xa0, 0x2f, 0xf9, 0x61, 0x00, 0x00, 0x03, 0x00,
    0x01, 0x00, 0x00, 0x03, 0x00, 0x3c, 0x8f, 0x08, 0x84, 0x6a,
];
const GO2_PPS: &[u8] = &[0x68, 0xee, 0x31, 0xb2, 0x1b];
const GO2_IDR_HEAD: &[u8] = &[0x65, 0xb8, 0x00, 0x01, 0x40, 0x00, 0x01, 0x3f];
/// A mid-GOP access unit: ONE non-IDR slice NAL, no parameter set — what ~97 % of
/// live camera attaches see first (one IDR per 30-60 frame GOP at ~30 fps).
const MID_GOP_SLICE: &[u8] = &[0x41, 0x9a, 0x00, 0x20];

fn annex_b(nals: &[&[u8]]) -> Vec<u8> {
    let mut out = Vec::new();
    for n in nals {
        out.extend_from_slice(&[0, 0, 0, 1]);
        out.extend_from_slice(n);
    }
    out
}

/// A hand-built `sensor_msgs/CompressedImage` frame carrying `au` in `data`.
///
/// The archetype classifier tries H.264 CONTENT before the schema NAME
/// (`sink::classify_content_or_name`), so a CompressedImage whose `data` scans as
/// Annex-B is a `VideoStream` — which is exactly how the Go2's camera arrives and
/// what lets this file drive the video layout path with the BUILT-IN walker.
fn h264_frame(au: &[u8]) -> Vec<u8> {
    assert_eq!(
        <CompressedImage as ShmMessage>::WIRE_FIXED_SIZE,
        0,
        "CompressedImage gained a fixed section"
    );
    assert_eq!(
        <CompressedImage as ShmMessage>::VARIABLE_FIELD_COUNT,
        3,
        "CompressedImage no longer has exactly three variable fields \
         (header, format, data)"
    );
    const TABLE: usize = 24; // 3 variable fields × 8-byte offset entries
    let format = b"h264";
    let mut payload = vec![0u8; TABLE];
    // entry 0 `header`: the empty-nested idiom (a present nested value, no fields).
    write_offset_entry(&mut payload, 0, 0, TABLE as u32, 0);
    write_offset_entry(&mut payload, 0, 1, TABLE as u32, format.len() as u32);
    payload.extend_from_slice(format);
    write_offset_entry(
        &mut payload,
        0,
        2,
        (TABLE + format.len()) as u32,
        au.len() as u32,
    );
    payload.extend_from_slice(au);
    let mut frame = vec![0u8; WireHeader::SIZE];
    WireHeader {
        schema_hash: <CompressedImage as ShmMessage>::SCHEMA_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: WireHeader::SIZE as u32,
        offset_table_count: 3,
        sequence: 0,
        timestamp_ns: 42_000,
    }
    .write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);
    frame
}

/// THE flagship-camera arm: a topic attached MID-GOP must lose its status-pane
/// companion when the SPS finally lands, in the APPLIED blueprint.
///
/// This is the shape a proof-map-only watcher misses, and it is the one the video
/// companion refusal exists for. `blueprint::render_is_proven` reads the RENDITION SET
/// for `VideoStream` and the render proof for every other archetype, so a drain
/// loop watching only the proof map is blind here: the first pre-keyframe access
/// unit takes `VideoRoute::Drop(BeforeKeyframe)`, which is not a degradation, so
/// the proof latches to its TERMINAL value on drain pass one — a second or more
/// BEFORE the demux opens a rendition. Both flags are sticky, so across the one
/// transition the video layout turns on, the proof map is byte-identical.
///
/// The outcome with a proof-map-only watcher: the empty
/// `(empty)` tile beside the live video for the rest of the session, while
/// `status` — which reads the rendition set LIVE — reports the companion DROPPED.
/// That is reported-vs-applied drift on the flagship path, which this arm
/// forbids.
///
/// The load-bearing assertion is the INSTALLED plan's view count, not the report:
/// every report on this daemon is served off the same live mirrors, so a daemon
/// that never reflowed still reports the new placement.
#[test]
fn a_mid_gop_camera_attach_drops_its_companion_when_the_sps_lands_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("camera");
    let topic = "/vizd/camera";

    let mid_gop = h264_frame(&annex_b(&[MID_GOP_SLICE]));
    let keyframe = h264_frame(&annex_b(&[GO2_SPS, GO2_PPS, GO2_IDR_HEAD]));

    let publisher = Publisher::spawn_fixed_frame(Arc::clone(&mgr), topic, mid_gop);
    let (worker, _flush, _storage) = memory_worker("camera");
    let (socket, dir) = temp_socket("camera");
    let mut daemon: RunningDaemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    let att = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(att["ok"].as_bool(), Some(true), "attach: {att}");
    assert_eq!(
        att["archetype"].as_str(),
        Some("VideoStream"),
        "the witness must classify as VIDEO, or this test proves nothing about the \
         rendition-set signal: {att}"
    );

    let view_kinds = |client: &mut Client, id: u64| -> Option<Value> {
        let st = client.request(&format!(r#"{{"id":{id},"method":"status"}}"#));
        entry_for(&st["topics"], "topic", topic).map(|r| r["view_kinds"].clone())
    };
    let applied_views = || {
        current_runtime_blueprint_plan()
            .map(|p| p.view_count())
            .unwrap_or(0)
    };

    // (1) PRE-KEYFRAME: the proof has latched but no rendition exists, so the video
    //     render is NOT proven and the companion is correctly KEPT — reported AND
    //     applied. This is the state the flip below leaves, so asserting it here is
    //     what stops (2) passing on a layout that never had the pane.
    assert!(
        wait_until(Duration::from_secs(10), || {
            view_kinds(&mut client, 2) == Some(serde_json::json!(["spatial2d", "text_document"]))
                && applied_views() > 0
        }),
        "a camera with no decodable rendition yet must KEEP its status pane — got \
         {:?} reported / {} views applied",
        view_kinds(&mut client, 3),
        applied_views()
    );
    // The pane count is read rather than hardcoded, and the claim is the DELTA: the
    // consolidated default carries structure this arm is not about (a Scene view is
    // installed whatever a single 2D topic does), so pinning an absolute count here
    // would pin that structure instead of the companion. A drop of EXACTLY one is a
    // sequence no static plan satisfies.
    let views_with_companion = applied_views();
    let reflows_before_sps = daemon.poll_loop_layout_signal_reflows();

    // (2) THE SPS LANDS. The rendition set goes non-empty; the render-proof map does
    //     NOT move. THE PIN is the INSTALLED plan — a proof-map-only watcher reports the
    //     drop here and never re-derives, permanently.
    drop(publisher);
    let _decoding = Publisher::spawn_fixed_frame(Arc::clone(&mgr), topic, keyframe);
    assert!(
        wait_until(Duration::from_secs(10), || applied_views()
            == views_with_companion - 1),
        "once the SPS opens a rendition the INSTALLED layout must be re-derived \
         WITHOUT the empty status pane: {} views before, {} now",
        views_with_companion,
        applied_views()
    );
    assert_eq!(
        view_kinds(&mut client, 4),
        Some(serde_json::json!(["spatial2d"])),
        "…and the report must agree with it"
    );
    assert!(
        daemon.poll_loop_layout_signal_reflows() > reflows_before_sps,
        "attributed to the rendition-set change, not to some unrelated reflow: \
         stuck at {reflows_before_sps}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

/// The ANTI-TAUTOLOGY half of the `discover` fix, and the arm that pins WHICH
/// state the live signals are read for.
///
/// A `RenderProof` entry is never removed on detach — its scope is the WORKER's
/// life, not one attach (see `cerulion_viz::sink::RenderProof`) — so a `discover`
/// row keyed on "is there a proof for this route?" would keep reporting the
/// refusal for a topic this desk stopped rendering minutes ago, while the applied
/// blueprint contains no pane for it at all. The gate is a LIVE TAP.
///
/// Without this arm, "discover reports the live placement" is satisfied by a
/// helper that reads the mirrors unconditionally: for a never-attached topic the
/// mirrors hold nothing, so the live read and the default agree, and only a
/// DETACHED-after-rendering topic tells the two apart.
#[test]
fn a_detached_topic_reports_the_default_placement_again_on_discover_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("detached");
    let topic = "/vizd/detached_markers";
    let healthy = marker_array_frame(&0u32.to_le_bytes());

    let _publisher = Publisher::spawn_fixed_frame(Arc::clone(&mgr), topic, healthy);
    let (worker, _flush, _storage) = memory_worker("detached");
    let (socket, dir) = temp_socket("detached");
    let mut daemon: RunningDaemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    let att = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(att["ok"].as_bool(), Some(true), "attach: {att}");

    // Render it until the proof latches and the companion is refused.
    let status_views = |client: &mut Client, id: u64| -> Option<Value> {
        let st = client.request(&format!(r#"{{"id":{id},"method":"status"}}"#));
        entry_for(&st["topics"], "topic", topic).map(|r| r["view_kinds"].clone())
    };
    assert!(
        wait_until(Duration::from_secs(10), || {
            status_views(&mut client, 2) == Some(serde_json::json!(["spatial3d"]))
        }),
        "PRECONDITION: the proof must latch while attached, or the detach below \
         proves nothing — got {:?}",
        status_views(&mut client, 3)
    );

    let det = client.request(&format!(
        r#"{{"id":4,"method":"detach","topic":"{topic}"}}"#
    ));
    assert_eq!(det["ok"].as_bool(), Some(true), "detach: {det}");

    let disc = client.request(r#"{"id":5,"method":"discover"}"#);
    let row = entry_for(&disc["topics"], "topic", topic)
        .unwrap_or_else(|| panic!("a detached topic is still discoverable: {disc}"));
    assert_eq!(
        row["view_kinds"],
        serde_json::json!(["spatial3d", "text_document"]),
        "a topic this desk is no longer rendering must report the companion DEFAULT \
         again — the worker's proof entry outlives the attach, and reading it for \
         an un-attached row would advertise a refusal nothing is backing: {row}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

/// The Principle #3 half of the reflow counter: it must mean "the layout was
/// RE-DERIVED for this", not "the loop called a function".
///
/// `maybe_apply_default_layout` returns at its first line under
/// `LayoutMode::Explicit` — an operator's own layout is never clobbered by a
/// signal change — so counting the CALL made the observable claim a reflow on a
/// daemon that deliberately did nothing. That is the same over-claim the
/// wiring arm's own fix names one level up, and no arm could see it: every other
/// e2e arm runs under the default Auto mode, where the two readings agree.
#[test]
fn a_signal_change_under_an_explicit_layout_counts_no_reflow_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("explicit");
    let first = "/vizd/explicit_a";
    let second = "/vizd/explicit_b";
    let healthy = marker_array_frame(&0u32.to_le_bytes());

    let _pub_a = Publisher::spawn_fixed_frame(Arc::clone(&mgr), first, healthy.clone());
    let (worker, _flush, _storage) = memory_worker("explicit");
    let (socket, dir) = temp_socket("explicit");
    let mut daemon: RunningDaemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    let att = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{first}"}}"#
    ));
    assert_eq!(att["ok"].as_bool(), Some(true), "attach A: {att}");
    // ANTI-VACUITY: the counter must be able to move at all on this daemon, or the
    // "did not grow" assertion below is satisfied by a feature that never works.
    assert!(
        wait_until(Duration::from_secs(10), || daemon
            .poll_loop_layout_signal_reflows()
            >= 1),
        "the first topic's proof must drive a counted reflow under Auto"
    );

    // The operator takes over the layout. From here a signal change must re-derive
    // NOTHING — and must therefore count nothing.
    let explicit = client.request(
        r#"{"id":2,"method":"set_blueprint","layout":{"root":{"type":"view","kind":"spatial3d","name":"Mine","origin":"world"}}}"#,
    );
    assert_eq!(
        explicit["ok"].as_bool(),
        Some(true),
        "set_blueprint: {explicit}"
    );
    let counted_under_auto = daemon.poll_loop_layout_signal_reflows();

    // A SECOND, never-seen topic: its very first render proof is a genuine signal
    // change, so the watcher fires and the arm is about what the loop does with it.
    let _pub_b = Publisher::spawn_fixed_frame(Arc::clone(&mgr), second, healthy);
    let att_b = client.request(&format!(
        r#"{{"id":3,"method":"attach","topic":"{second}"}}"#
    ));
    assert_eq!(att_b["ok"].as_bool(), Some(true), "attach B: {att_b}");
    // The signal really did cross: `status` reads the worker's mirror whatever the
    // layout mode is, so this proves the watcher had something to see.
    let b_views = |client: &mut Client, id: u64| -> Option<Value> {
        let st = client.request(&format!(r#"{{"id":{id},"method":"status"}}"#));
        entry_for(&st["topics"], "topic", second).map(|r| r["view_kinds"].clone())
    };
    assert!(
        wait_until(Duration::from_secs(10), || {
            b_views(&mut client, 4) == Some(serde_json::json!(["spatial3d"]))
        }),
        "PRECONDITION: the second topic's render proof must reach the daemon, or \
         nothing below is under test — got {:?}",
        b_views(&mut client, 5)
    );
    // …and the DRAIN LOOP must have had passes in which to see it. `status` reads
    // the worker's mirror directly, so it flips a thread boundary earlier than the
    // loop's next pass — without this wait the assertion below is satisfied by
    // reading the counter before the loop ever looked, which is vacuous.
    // Without this wait the arm passes the very regression it exists to catch.
    //
    // Load-safe: waiting for MORE passes can only be delayed by contention, never
    // skipped, and the claim underneath is a ceiling load cannot inflate.
    let passes_at_signal = daemon.poll_loop_iterations();
    assert!(
        wait_until(Duration::from_secs(10), || daemon.poll_loop_iterations()
            >= passes_at_signal + 3),
        "the drain loop must run passes after the signal changed"
    );

    assert_eq!(
        daemon.poll_loop_layout_signal_reflows(),
        counted_under_auto,
        "a signal change under an EXPLICIT layout re-derives nothing, so it must \
         count nothing — the counter's own doc says a reflow RAN"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

/// THE STUDIO FLAGSHIP FLOW: attach a robot topic → netd mirrors it → this desk
/// renders it → the sidebar re-polls `discover`. The row it reads must report the
/// placement that is APPLIED, not the pre-drop one.
///
/// This is the arm the first `discover` fix did not have, and the shape a
/// verification probe measured against it: `discover` built the mirror row with
/// the live signals, and `fold_streaming_mirrors_into_robots` then THREW IT AWAY
/// whenever netd's catalog already listed the topic — the surviving row being
/// `remote_discovered_entry`'s, built from blank signals. So the sidebar
/// advertised a `text_document` pane the installed blueprint did not contain,
/// while `status`/`list`/both attach replies reported the applied set. One run,
/// two answers, on the one surface a separate issue governs.
///
/// Read on `robots`, deliberately: the e2e sibling reads `topics`, which is the
/// GENUINE-LOCAL section — a mirror never appears there (the provenance fold
/// moves it to its robot), so no assertion on `topics` can see this defect.
#[test]
fn a_rendering_mirror_reports_its_applied_placement_on_the_robots_section_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("mirror_row");
    let topic = "/mk/markers";
    let healthy = marker_array_frame(&0u32.to_le_bytes());

    // The MIRROR: a live publisher (netd's re-inject shape) + the provenance
    // record netd registers, so the topic folds OUT of local and INTO its robot.
    let _mirror_pub = Publisher::spawn_fixed_frame(Arc::clone(&mgr), topic, healthy);
    assert!(
        mgr.register_mirror_provenance(topic, "go2")
            .expect("register mirror provenance"),
        "the mirror is newly attributed to go2"
    );
    // …and netd's catalog ALSO lists it, which is what makes the catalog row WIN
    // the fold. Without this the mirror entry is appended verbatim and the defect
    // is unreachable — so this line is the whole precondition.
    let plane = Arc::new(GatherPlane::new(vec![oracle_catalog(
        "go2",
        &[(topic, Some("visualization_msgs/MarkerArray"))],
    )]));

    let (worker, _flush, _storage) = memory_worker("mirror_row");
    let (socket, dir) = temp_socket("mirror_row");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    let att = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(att["ok"].as_bool(), Some(true), "attach: {att}");

    // Render it until the proof latches and `status` drops the companion — the
    // state the sidebar must agree with.
    let status_views = |client: &mut Client, id: u64| -> Option<Value> {
        let st = client.request(&format!(r#"{{"id":{id},"method":"status"}}"#));
        entry_for(&st["topics"], "topic", topic).map(|r| r["view_kinds"].clone())
    };
    assert!(
        wait_until(Duration::from_secs(10), || {
            status_views(&mut client, 2) == Some(serde_json::json!(["spatial3d"]))
        }),
        "PRECONDITION: the topic must be provably rendering, or nothing below \
         discriminates — got {:?}",
        status_views(&mut client, 3)
    );

    // The sidebar's row. A CONDITION wait, because the mirror's ROBOT attribution
    // rides a cached `gather_mirror_provenance` (PROVENANCE_CACHE_TTL 3 s) and a
    // missed gather is served stale for the whole TTL — the lesson from
    // the sibling attribution arm. Load can delay the row, never invert it.
    let robot_row = |client: &mut Client, id: u64| -> Option<Value> {
        let disc = client.request(&format!(r#"{{"id":{id},"method":"discover"}}"#));
        disc["robots"].as_array()?.iter().find_map(|r| {
            (r["robot"].as_str() == Some("go2"))
                .then(|| {
                    r["topics"]
                        .as_array()?
                        .iter()
                        .find(|e| e["topic"].as_str() == Some(topic))
                        .cloned()
                })
                .flatten()
        })
    };
    let robot_row_views = |client: &mut Client, id: u64| -> Option<Value> {
        robot_row(client, id).map(|e| e["view_kinds"].clone())
    };
    assert!(
        wait_until(Duration::from_secs(10), || {
            robot_row_views(&mut client, 4) == Some(serde_json::json!(["spatial3d"]))
        }),
        "THE PIN: the ROBOTS row — what the Studio sidebar reads — must report the \
         placement this desk applied, the same one `status` reports; got {:?}",
        robot_row_views(&mut client, 5)
    );

    // …and the CAUSE travels with it. `view_kinds` on this row is computed THROUGH
    // the operator's representation choice, so a row reporting the consequence without the
    // cause shows a placement its archetype alone does not imply and nothing to
    // explain why — the failure `DiscoveredEntry::representation`'s contract exists
    // to prevent. Forced here, because ABSENT-on-`auto` is the wire default and
    // would make the assertion vacuous.
    let rep = client.request(&format!(
        r#"{{"id":6,"method":"representation","topic":"{topic}","representation":"both"}}"#
    ));
    assert_eq!(rep["ok"].as_bool(), Some(true), "representation: {rep}");
    assert!(
        wait_until(Duration::from_secs(10), || {
            robot_row(&mut client, 7).is_some_and(|e| {
                e["view_kinds"] == serde_json::json!(["spatial3d", "text_document"])
                    && e["representation"].as_str() == Some("both")
            })
        }),
        "the ROBOTS row must carry the CHOICE beside the view kinds it produced; \
         got {:?}",
        robot_row(&mut client, 8)
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

// ===========================================================================
// Monitors — the standing watchdog, over the REAL daemon
// ===========================================================================

/// The `monitors` verb answers on a daemon watching NOTHING, and the answer is a
/// positive "we looked and there is nothing" rather than an error.
///
/// Cheap, and it is the arm the capability contract rests on: the whole reason
/// this is a new VERB is that an old daemon answers the unknown method with a
/// structured error, so a client can tell "not supported" from "supported, empty".
/// If a supporting daemon ALSO errored on an empty watch set the two would be
/// indistinguishable again.
#[test]
fn monitors_answers_on_a_daemon_watching_nothing_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("cer_monitors_empty");
    let (worker, _flush, _storage) = memory_worker("monitors_empty");
    let (socket, dir) = temp_socket("monitors_empty");
    let mut daemon: RunningDaemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");

    let mut client = Client::connect(&socket);
    let resp = client.request(r#"{"id":1,"method":"monitors"}"#);
    assert_eq!(resp["ok"].as_bool(), Some(true), "{resp}");
    assert_eq!(resp["id"].as_u64(), Some(1));
    assert_eq!(resp["monitors"].as_array().map(Vec::len), Some(0));
    assert_eq!(resp["alerts"].as_array().map(Vec::len), Some(0));
    assert_eq!(resp["seq"].as_u64(), Some(0), "nothing has been minted");
    // Nothing raised, nothing withheld — the gauge agrees with the payload.
    assert_eq!(daemon.monitor_alerts_raised(), 0);
    assert_eq!(daemon.monitor_rows_ineligible(), 0);

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

/// The SAMPLER is wired: attaching a topic makes the drain loop hand samples to
/// the engine, and the row appears on the verb.
///
/// This is the sampler's no-inert-shipping arm, and it is deliberately separate from
/// (and much cheaper than) the stall arm below. The monitor engine is pure, so its
/// whole suite stays green whether or not anything ever calls it — the only thing
/// that can say the sampler RUNS is a counter that climbs on a daemon nobody is
/// polling for alerts. Both bounds are FLOORS, which contention can only delay,
/// never manufacture.
///
/// It also pins the evidence pair at its sharpest: a row inside its settle
/// window reads `samples: 0` beside a small `last_sample_age_ms`, which says
/// exactly "we are polling this row and have learned nothing from it yet" — and
/// which is why `monitor_samples_taken` counts handed-over samples rather than
/// admitted ones.
#[test]
fn attaching_a_topic_makes_the_drain_loop_sample_it_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("cer_monitors_sampler");
    let topic = "/monitors/vel";
    let _pub = Publisher::spawn(Arc::clone(&mgr), topic);

    let (worker, _flush, _storage) = memory_worker("monitors_sampler");
    let (socket, dir) = temp_socket("monitors_sampler");
    let mut daemon: RunningDaemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");

    let mut client = Client::connect(&socket);
    let att = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
    ));
    assert_eq!(att["ok"].as_bool(), Some(true), "{att}");

    // WAIT ON THE COUNTER, AND TOUCH NO HANDLER UNTIL IT MOVES. Polling `monitors`
    // to detect the row would make a sampler MOVED INTO THAT HANDLER pass this arm
    // — the handler would create the row and bump the counter, and both assertions
    // would hold on a daemon whose drain loop samples nothing. Nothing is asked of
    // the daemon here but the counter, so a zero is unambiguous.
    //
    // The sampler runs at MONITOR_SAMPLE_INTERVAL_NS, so a handful of intervals is
    // a generous bound for the FIRST sample of a freshly-attached row. Polled to a
    // deadline rather than slept, so a slow runner is slower, never wrong.
    let sampled = wait_until(Duration::from_secs(15), || {
        daemon.monitor_samples_taken() > 0
    });
    assert!(
        sampled,
        "the DRAIN LOOP must be handing samples to the engine — a zero here is the \
         whole feature shipped dead behind a green pure suite"
    );

    let resp = client.request(r#"{"id":2,"method":"monitors"}"#);
    assert!(
        row_for(&resp["monitors"], topic, None).is_some(),
        "the attached topic must become a watched row: {:#}",
        resp["monitors"]
    );

    let resp = client.request(r#"{"id":3,"method":"monitors"}"#);
    let row = row_for(&resp["monitors"], topic, None).expect("the row");
    // A genuinely LOCAL producer: `robot` ABSENT is the convention, and it
    // is what keys this row apart from any remote topic of the same name.
    assert!(
        row.get("robot").is_none(),
        "a local producer names no robot: {row}"
    );
    // `conditions` is PRESENT and empty — "we looked and nothing is raised" — while
    // an absent key would mean "no claim". The difference is what an agent reads.
    assert_eq!(row["conditions"], serde_json::json!([]), "{row}");
    // The evidence pair is always present, and meaningful at zero.
    assert!(row["samples"].is_u64(), "{row}");
    assert!(row["last_sample_age_ms"].is_u64(), "{row}");

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

/// A topic whose publisher STOPS raises `stalled`; a sibling that keeps
/// publishing does not.
///
/// THE headline arm — the whole point of the feature, driven end to end over the
/// real daemon, the real drain loop, real iceoryx2 taps and the real verb. The
/// two topics run in ONE body so the discriminator is the STIMULUS (the stop) and
/// not the passage of time: without the sibling, a monitor that raised `stalled`
/// on every watched row after 15 seconds would pass.
///
/// The wall is long by construction, and every part of it is a substrate constant
/// rather than a tuning choice: `MONITOR_SETTLE_NS` (5 s) before any evidence is
/// admitted, `MONITOR_LEARN_SAMPLES` at `MONITOR_SAMPLE_INTERVAL_NS` to freeze a
/// baseline, `LIVENESS_STREAMING_RECENCY_MS` (5 s) before a stopped stream even
/// classifies `Idle`, then `MONITOR_CONFIRM_SAMPLES` spanning
/// `MONITOR_CONFIRM_MIN_SPAN_NS`. It is polled to a deadline several times that
/// sum, so a loaded runner takes longer rather than failing — a CEILING on the
/// wall would be the load-sensitive class, and none is asserted.
///
/// The anti-tautology half asserts the sibling raises no `stalled`, NOT that it
/// raises nothing at all. That is deliberate and it is the correct bound: a runner
/// that starves the publisher thread can genuinely halve the desk's measured Hz,
/// which is a real `rate_deviation` about the RUNNER. Scoping the negative to the
/// condition under test keeps the discriminator sharp without asserting something
/// load can falsify.
#[test]
fn a_stopped_publisher_raises_stalled_while_a_live_sibling_does_not_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("cer_monitors_stall");
    let dying = "/monitors/dying";
    let living = "/monitors/living";
    // DEADLINE-paced, not `sleep`-paced, and modest rather than fast.
    // The engine freezes a rate baseline for this row and then judges it against
    // an octave around that frozen number, so the one thing this arm needs from
    // its producers is a rate a tap measures the SAME way before and after the
    // baseline is frozen — see [`Publisher::spawn_paced`]. `MONITOR_STALL_MIN_MHZ`
    // is 400 mHz, so 10 Hz clears the engine's slow-topic floor by 25x while leaving every
    // frame ~90 ms of slack for the runner to be late in.
    let mut dying_pub = Some(Publisher::spawn_paced(
        Arc::clone(&mgr),
        dying,
        STALL_ARM_PUBLISH_PERIOD,
    ));
    let _living_pub = Publisher::spawn_paced(Arc::clone(&mgr), living, STALL_ARM_PUBLISH_PERIOD);

    let (worker, _flush, _storage) = memory_worker("monitors_stall");
    let (socket, dir) = temp_socket("monitors_stall");
    let mut daemon: RunningDaemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");

    let mut client = Client::connect(&socket);
    for (id, topic) in [(1, dying), (2, living)] {
        let att = client.request(&format!(
            r#"{{"id":{id},"method":"attach","topic":"{topic}"}}"#
        ));
        assert_eq!(att["ok"].as_bool(), Some(true), "{att}");
    }

    // Both rows must LEARN a baseline before a stall can be judged at all (the stall
    // gate: a row with no trustworthy rate is ineligible and says so), AND the row
    // about to be stopped must be in a state a stall can actually be REACHED from
    // — see [`stall_reachable`] for why "a baseline exists" is not that state.
    // Waiting on what the SUBJECT reports rather than on a wall is what makes the
    // stop land where it means something.
    let ready = wait_until(STALL_READY_BOUND, || {
        let resp = client.request(r#"{"id":3,"method":"monitors"}"#);
        let both_learned = [dying, living].iter().all(|t| {
            row_for(&resp["monitors"], t, None)
                .is_some_and(|row| row["baseline_mhz"].as_u64().is_some())
        });
        both_learned
            && row_for(&resp["monitors"], dying, None)
                .is_some_and(|row| stall_reachable(row).is_ok())
    });
    if !ready {
        let resp = client.request(r#"{"id":3,"method":"monitors"}"#);
        let why = row_for(&resp["monitors"], dying, None)
            .map(|row| match stall_reachable(row) {
                Ok(()) => "the row IS stall-reachable (the sibling's baseline is what is missing)"
                    .to_string(),
                Err(reason) => reason,
            })
            .unwrap_or_else(|| format!("no row for `{dying}` at all"));
        panic!(
            "PRECONDITION: within {STALL_READY_BOUND:?} the stopped-publisher row never \
             reached a state a stall verdict can be reached from — {why}.\n\
             This is a PRECONDITION failure, not the feature's: the arm has not yet \
             applied its stimulus. Rows: {:#}\nAlerts: {:#}",
            resp["monitors"], resp["alerts"]
        );
    }
    // The ineligible GAUGE is a statement about NOW, and this is the only place it
    // can be seen FALLING: both rows spent their whole settle window withholding
    // all three conditions (`no_liveness_evidence`), and now that each has learned
    // a trustworthy baseline nothing is permanently withheld. A running TOTAL —
    // the natural slip, since every sibling counter beside it is one — would have
    // accumulated one per row per pass through that window and could never come
    // back to zero.
    assert_eq!(
        daemon.monitor_rows_ineligible(),
        0,
        "two rows with learned baselines withhold nothing — this observable is a \
         GAUGE, not a running total"
    );

    // THE STIMULUS. It follows the readiness rendezvous IMMEDIATELY — no control
    // round trip in between — because `stall_reachable`'s guarantee is about the
    // row's state AT THIS INSTANT, and every sample that lands before the stop is
    // another chance for the rate to move (see the helper).
    drop(dying_pub.take());

    let raised = wait_until(Duration::from_secs(90), || {
        let resp = client.request(r#"{"id":4,"method":"monitors"}"#);
        resp["alerts"].as_array().is_some_and(|alerts| {
            alerts
                .iter()
                .any(|a| a["condition"].as_str() == Some("stalled"))
        })
    });
    if !raised {
        // ATTRIBUTABLE, because the two ways this can fail need different fixes: a
        // row still `healthy`/`alerting` with its baseline intact is the FEATURE
        // failing to confirm, while a row back in `learning` with `baseline_mhz`
        // absent means the stall basis is not doing its job — something
        // wiped the frozen baseline after the stop AND the retained basis did not
        // survive it, which no dead publisher can ever restore.
        let resp = client.request(r#"{"id":4,"method":"monitors"}"#);
        panic!(
            "the stopped publisher's row must raise `stalled`.\nRows: {:#}\nAlerts: {:#}",
            resp["monitors"], resp["alerts"]
        );
    }

    let resp = client.request(r#"{"id":5,"method":"monitors"}"#);
    let stalls: Vec<&Value> = resp["alerts"]
        .as_array()
        .expect("alerts")
        .iter()
        .filter(|a| a["condition"].as_str() == Some("stalled"))
        .collect();
    assert_eq!(
        stalls.len(),
        1,
        "exactly ONE stall, and it is the stopped topic's: {:#}",
        resp["alerts"]
    );
    let stall = stalls[0];
    assert_eq!(stall["topic"].as_str(), Some(dying));
    assert_eq!(
        stall["cleared_at_ms"],
        Value::Null,
        "a RAISE carries no clear stamp — that field is the discriminator"
    );
    // The SERVE-TIME age is stamped on every entry (the stamps beside it belong to
    // a clock the reader does not share, so the age is the only renderable
    // quantity). ABSENT would mean UNKNOWN, which is a claim this daemon can
    // always beat.
    let age_ms = stall["age_ms"]
        .as_u64()
        .unwrap_or_else(|| panic!("every served alert carries its serve-time age: {stall}"));
    // …in MILLISECONDS, on the SAME clock as `raised_at_ms`, so their sum is the
    // serve instant in ms since this daemon's monitor origin.
    //
    // The UNIT is what this bounds, and nothing else pins it: the plane's own
    // arms call `alerts_at` with a hand-written `now_ms`, so the handler's
    // `now_ns / 1_000_000` is reachable by no oracle above. MEASURED — serving
    // `now_ns` raw survived the entire suite, and an age rendered a million times
    // too large is a silently wrong number on the one field a reader puts on
    // screen.
    //
    // The ceiling is deliberately far outside anything load can produce: this
    // test's own two poll deadlines total 150 s, so 10 minutes cannot be reached
    // by a legal run however slow the runner, while the wrong unit overshoots it
    // by ~2500x. That gap is why this is a UNIT check and not the class
    // of wall assertion whose margin contention can eat.
    let raised_at_ms = stall["raised_at_ms"]
        .as_u64()
        .unwrap_or_else(|| panic!("a raise carries its stamp: {stall}"));
    assert!(
        raised_at_ms + age_ms < 600_000,
        "`age_ms` must be MILLISECONDS on the same clock as `raised_at_ms` — \
         together they imply a serve instant {} ms into this daemon's life, which \
         no run of this test can reach: {stall}",
        raised_at_ms + age_ms
    );
    // The evidence travels with the verdict, on EITHER path. A
    // stall confirmed after a baseline re-learn wipe carries the row's last frozen baseline
    // rather than nothing, so this holds whether or not the runner's publish
    // jitter flapped a `rate_deviation` on the way down (the ROW's own
    // `baseline_mhz` may legitimately be absent in that case; the ALERT's is not).
    assert!(
        stall["baseline_mhz"].as_u64().is_some_and(|b| b > 0),
        "{stall}"
    );
    assert_eq!(stall["liveness_state"].as_str(), Some("idle"), "{stall}");

    // THE ANTI-TAUTOLOGY HALF: the sibling was watched the whole time and is NOT
    // stalled. Both clauses matter — "no alert" on a row nothing ever sampled
    // would be worthless.
    let live_row = row_for(&resp["monitors"], living, None).expect("the sibling row");
    assert!(
        live_row["samples"].as_u64().is_some_and(|s| s > 0),
        "the sibling really was being watched: {live_row}"
    );
    assert_eq!(
        live_row["conditions"]
            .as_array()
            .expect("conditions")
            .iter()
            .filter(|c| c.as_str() == Some("stalled"))
            .count(),
        0,
        "a topic that kept publishing must not be stalled: {live_row}"
    );

    // The counted twin of the payload (Principle #3), bumped inside the pass that
    // observed the raise.
    assert!(daemon.monitor_alerts_raised() >= 1);

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

/// A netd MIRROR attached through the LOCAL path is monitored as its ROBOT's row,
/// not as a desk-local one — while a genuine desk producer stays unattributed.
///
/// This is the attribution rule ("one data source = one topic; every surface reads ONE
/// map") reaching the fifth surface. The live shape:
/// netd registers a mirror BEFORE Studio checks the topic, so the
/// topic is locally visible and `attach` routes it to `attach_local` — nothing but
/// the `/__cerulion/mirrors` provenance registry can tell the daemon whose data it
/// is. That path ATTRIBUTES THE REPLY through `attribution_snapshot`; if it
/// does not also write the answer down, `monitor_snapshot` — which reads the
/// tap's own `origin_robot`, and must, because the snapshot pays a 150-600 ms
/// gather that cannot go on the poll thread every 400 ms — keys the row under
/// `None`. One attach, two answers.
///
/// The mis-keying is not cosmetic. Rows are `(topic, robot)`, so a robot's topic
/// and a desk topic of the same NAME are ONE row; every alert it mints names no
/// robot, and an agent cannot tell which machine to go and look at.
///
/// CONVERGE, THEN ATTACH. The tap's record is written ONCE, from the snapshot at
/// attach time, so this polls `discover` until the mirror is attributed — which
/// proves the provenance cache HOLDS the record — before attaching. Without that
/// the arm would be racing the same best-effort gather the sibling
/// `discover_provenance_snapshot_is_cached_within_ttl_e2e` asserts is allowed to
/// miss, and the bound is derived from `PROVENANCE_CACHE_TTL` rather than guessed.
#[test]
fn a_locally_attached_mirror_is_monitored_under_its_robot_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("cer_monitors_mirror");
    let mirror = "/monitors/mirror/scan";
    let local = "/monitors/mirror/desk";
    // The MIRROR: a live publisher (netd's re-inject publisher, from this daemon's
    // point of view) plus the provenance record netd registers beside it.
    let _mirror_pub = Publisher::spawn(Arc::clone(&mgr), mirror);
    assert!(
        mgr.register_mirror_provenance(mirror, "ubuntu")
            .expect("register mirror provenance"),
        "the mirror is newly attributed to ubuntu"
    );
    // The CONTROL: a genuine desk producer, which must stay unattributed.
    let _local_pub = Publisher::spawn(Arc::clone(&mgr), local);

    let plane = Arc::new(GatherPlane::new(vec![oracle_catalog(
        "ubuntu",
        &[(mirror, Some("geometry_msgs/Vector3"))],
    )]));
    let (worker, _flush, _storage) = memory_worker("monitors_mirror");
    let (socket, dir) = temp_socket("monitors_mirror");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // Converge FIRST: the provenance cache must hold the record before the attach
    // reads it, or this races a gather that is allowed to miss.
    let disc = discover_until(&mut client, ATTRIBUTION_CONVERGENCE_BOUND, |d| {
        d["robots"].as_array().is_some_and(|robots| {
            robots.iter().any(|r| {
                r["robot"].as_str() == Some("ubuntu")
                    && r["topics"]
                        .as_array()
                        .is_some_and(|t| t.iter().any(|e| e["topic"].as_str() == Some(mirror)))
            })
        })
    });
    assert!(
        disc["robots"].as_array().is_some_and(|r| !r.is_empty()),
        "harness precondition: the mirror must be attributed before the attach \
         reads the cache: {disc:#}"
    );

    // `attach` routes BOTH locally — the mirror is already visible here, which is
    // the whole point of this shape.
    for topic in [mirror, local] {
        let att = client.request(&format!(
            r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
        ));
        assert_eq!(att["ok"].as_bool(), Some(true), "attach '{topic}': {att}");
    }
    // The REPLY names the robot — the answer the monitor row has to agree with.
    let att = client.request(&format!(
        r#"{{"id":2,"method":"attach","topic":"{mirror}"}}"#
    ));
    assert_eq!(
        att["robot"].as_str(),
        Some("ubuntu"),
        "precondition: the attach reply attributes the mirror, so a `None` monitor \
         row below is one attach answering two ways: {att}"
    );

    let appeared = wait_until(Duration::from_secs(15), || {
        let resp = client.request(r#"{"id":3,"method":"monitors"}"#);
        row_for(&resp["monitors"], mirror, Some("ubuntu")).is_some()
            && row_for(&resp["monitors"], local, None).is_some()
    });
    assert!(appeared, "both taps must become watched rows");

    let resp = client.request(r#"{"id":4,"method":"monitors"}"#);
    // THE PIN. Rows are keyed `(topic, robot)`, so "a row for the mirror under
    // ubuntu exists" IS the attribution claim — and the `None` half is what makes
    // it airtight: the served catalog ALSO names this topic under ubuntu, so an
    // unattributed tap would leave its own row on the robot-less key beside the
    // catalog's, and a lookup by topic alone answers with whichever sorts first
    // rather than with the tap's.
    assert!(
        row_for(&resp["monitors"], mirror, Some("ubuntu")).is_some()
            && row_for(&resp["monitors"], mirror, None).is_none(),
        "a netd mirror attached through the LOCAL path must be monitored under its \
         ROBOT — the reply says ubuntu, and a row keyed `None` is the same attach \
         answering two ways, on the one surface that disagrees: {:#}",
        resp["monitors"]
    );
    // THE CONTROL, without which the pin would also pass a daemon that attributed
    // everything: a genuine desk producer names no robot.
    assert!(
        row_for(&resp["monitors"], local, None).is_some(),
        "a genuine desk producer must stay unattributed — ABSENCE is how a local \
         row is spelled: {:#}",
        resp["monitors"]
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

// ── Monitors: the DISCOVERY plane ───────────────────────────────────

/// A serving [`cerulion_vizd::DemandPlane`] whose CATALOG and DISCOVERY state a
/// test can change between gathers.
///
/// `GatherPlane` above serves a fixed catalog, which cannot express the two things
/// the discovery plane is about: a robot that STOPS naming a topic (the retirement) and a plane
/// that has not converged. It creates no mirrors — every arm here is
/// about topics NOBODY attached, which is the whole point of the plane.
struct LivenessCatalogPlane {
    catalogs: std::sync::Mutex<Vec<cerulion_core::CatalogReply>>,
    discovery: std::sync::Mutex<DiscoveryState>,
}

impl LivenessCatalogPlane {
    fn new(catalogs: Vec<cerulion_core::CatalogReply>, discovery: DiscoveryState) -> Self {
        Self {
            catalogs: std::sync::Mutex::new(catalogs),
            discovery: std::sync::Mutex::new(discovery),
        }
    }
    fn set_catalogs(&self, catalogs: Vec<cerulion_core::CatalogReply>) {
        *self.catalogs.lock().unwrap() = catalogs;
    }
    fn set_discovery(&self, discovery: DiscoveryState) {
        *self.discovery.lock().unwrap() = discovery;
    }
    fn gather(&self) -> CatalogGather {
        CatalogGather {
            catalogs: self.catalogs.lock().unwrap().clone(),
            discovery: *self.discovery.lock().unwrap(),
            unsettled_for: None,
        }
    }
}

impl cerulion_vizd::DemandPlane for LivenessCatalogPlane {
    fn demand(&self, _robot: &str, _topic: &str, _schema_hash: u64) -> Result<(), String> {
        Err("cer-monitors-c3 plane: no demand in these arms".to_string())
    }
    fn release(&self, _robot: &str, _topic: &str) -> Result<(), String> {
        Ok(())
    }
    fn query_catalog(&self, _robot: &str) -> Result<Vec<cerulion_core::CatalogReply>, String> {
        Ok(self.gather().catalogs)
    }
    fn query_catalog_with_discovery(&self, _robot: &str) -> Result<CatalogGather, String> {
        Ok(self.gather())
    }
    fn query_catalog_all(&self) -> Result<Vec<cerulion_core::CatalogReply>, String> {
        Ok(self.gather().catalogs)
    }
    fn query_catalog_all_with_discovery(&self) -> Result<CatalogGather, String> {
        Ok(self.gather())
    }
    fn query_schema(
        &self,
        _robot: &str,
        _requested: &str,
    ) -> Result<Vec<cerulion_core::SchemaReply>, String> {
        Ok(Vec::new())
    }
}

/// A catalog entry carrying the ROBOT's own liveness observation.
///
/// `rate` is the estimate the robot's observer computed, which is the ONLY
/// rate an unattached topic can have — nothing on this desk is tapping it.
fn liveness_catalog(
    robot: &str,
    entries: &[(&str, Option<cerulion_core::TopicLiveness>)],
) -> cerulion_core::CatalogReply {
    use cerulion_core::{CatalogEntry, CatalogProvenance};
    cerulion_core::CatalogReply {
        version: 1,
        robot: robot.to_string(),
        entries: entries
            .iter()
            .map(|(topic, liveness)| CatalogEntry {
                topic: (*topic).to_string(),
                schema_hash: Some(7),
                schema_name: Some("geometry_msgs/Vector3".to_string()),
                provenance: CatalogProvenance::Runtime,
                producer_count: Some(1),
                liveness: *liveness,
            })
            .collect(),
        error: None,
    }
}

/// The robot's observer reporting a healthy stream, rate included.
fn robot_streaming_liveness(mhz: u64) -> cerulion_core::TopicLiveness {
    cerulion_core::TopicLiveness {
        last_frame_age_ms: Some(0),
        observed_for_ms: 600_000,
        frames_observed: 5_000,
        rate_estimate: Some(cerulion_core::TopicRateEstimate {
            millihertz: mhz,
            is_floor: false,
        }),
    }
}

/// Poll `discover` until `predicate` holds over the `monitors` verb, or the bound
/// expires. Returns whether it held.
///
/// `discover` is what FEEDS the discovery plane (it piggybacks on the
/// gather Studio already makes), so driving it is how this plane is made to sample
/// at all. Polled to a deadline rather than slept: a loaded runner takes longer,
/// never a different answer.
fn discover_until_monitors(
    client: &mut Client,
    bound: Duration,
    mut predicate: impl FnMut(&Value) -> bool,
) -> bool {
    let deadline = Instant::now() + bound;
    loop {
        let _ = client.request(r#"{"id":90,"method":"discover"}"#);
        let mon = client.request(r#"{"id":91,"method":"monitors"}"#);
        if predicate(&mon) {
            return true;
        }
        if Instant::now() >= deadline {
            return false;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
}

/// The reported property, end to end: a topic nobody has checked gets a monitor row
/// carrying the robot's own liveness and its rate.
///
/// This is what the discovery plane exists for and what the attach-driven sampler
/// could not do: every row it watches is a row the desk ATTACHED, so a topic the user never clicked was
/// invisible to the watchdog. Here nothing is attached, no mirror exists, and no
/// tap is open — the verdict comes entirely from the catalog rows `discover`
/// already gathers.
///
/// Two FLOORS and no ceiling: `monitor_catalog_samples_taken` is the discovery plane's
/// no-inert-shipping observable (a zero is the feature shipped dead behind a green
/// unit suite), and the wall is polled to a deadline several times the substrate
/// constants it must cross. Contention can only make this slower.
///
/// The `attached` control is in the same body: `list` must report NOTHING, so the
/// row provably did not come from the attached plane.
#[test]
fn an_unattached_catalog_topic_is_monitored_from_the_discover_gather_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("cer_monitors_catalog");
    let topic = "/uslam/cloud_map";
    let plane = Arc::new(LivenessCatalogPlane::new(
        vec![liveness_catalog(
            "go2",
            &[(topic, Some(robot_streaming_liveness(500_000)))],
        )],
        DiscoveryState::Settled,
    ));

    let (worker, _flush, _storage) = memory_worker("mon_catalog");
    let (socket, dir) = temp_socket("mon_catalog");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // The row must ADMIT EVIDENCE, not merely exist: a row inside its settle
    // window exists with `samples: 0`, which correctly says "we are polling this and
    // have learned nothing yet" and would pass a much weaker feed.
    let learned = discover_until_monitors(&mut client, Duration::from_secs(60), |mon| {
        row_for(&mon["monitors"], topic, Some("go2"))
            .is_some_and(|row| row["samples"].as_u64().unwrap_or(0) > 0)
    });
    assert!(
        learned,
        "a topic NOBODY attached must become a watched row that LEARNS from the \
         robot's own observation — that is the whole of the discovery plane"
    );

    let mon = client.request(r#"{"id":2,"method":"monitors"}"#);
    let row = row_for(&mon["monitors"], topic, Some("go2")).expect("the catalog row");
    assert_eq!(
        row["robot"].as_str(),
        Some("go2"),
        "a catalog row is keyed on the robot that served it: {row}"
    );
    assert_eq!(
        row["observed_mhz"].as_u64(),
        Some(500_000),
        "the ROBOT's observed rate reaches the row — nothing on this desk is \
         measuring this topic, so a missing rate here is a row that can never \
         learn a baseline: {row}"
    );
    assert_eq!(
        row["liveness_state"].as_str(),
        Some("streaming"),
        "…classified by the substrate, not by the monitor: {row}"
    );
    assert_eq!(
        row["observed_is_floor"].as_bool(),
        Some(false),
        "and its basis is labelled, which is what the stall gate reads: {row}"
    );
    // The evidence pair rides every row: an unattached row is only as current as
    // the last `discover`, so the agent is TOLD how fresh the evidence is.
    assert!(row["samples"].is_u64(), "{row}");
    assert!(row["last_sample_age_ms"].is_u64(), "{row}");

    // THE CONTROL, in the same body: nothing is attached. Without it this arm
    // would also pass a daemon whose attached plane had somehow acquired the row,
    // which is the one other way it could appear.
    let list = client.request(r#"{"id":3,"method":"list"}"#);
    assert_eq!(
        list["attached"].as_array().map(Vec::len),
        Some(0),
        "no tap exists — this verdict came from the DISCOVERY plane alone: {list:#}"
    );
    assert_eq!(
        daemon.monitor_samples_taken(),
        0,
        "…and the drain loop's sampler handed over nothing, because it has no taps \
         to sample: a nonzero here would mean the row could have come from the sampler"
    );
    assert!(
        daemon.monitor_catalog_samples_taken() > 0,
        "the discovery plane's own no-inert-shipping observable must move"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

/// An UNCONVERGED gather teaches the monitor NOTHING, and says so — then the SAME
/// catalog, settled, teaches it everything.
///
/// A short or empty catalog from a plane that has not
/// converged proves nothing about any topic, so a row built from one must
/// admit no evidence and raise no condition. The failure mode of getting this wrong
/// is the worst kind: every cold-start `discover` — before netd has finished its
/// first harvest — would feed rows whose emptiness is an artefact of the gather,
/// and start confirming `silent` on topics that are fine.
///
/// The flip is the anti-tautology half and it is the SAME catalog, so the arm
/// cannot pass because the stimulus was inert.
#[test]
fn an_unconverged_catalog_gather_teaches_the_monitor_nothing_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("cer_monitors_c3_unconverged");
    let topic = "/go2/lowstate";
    let plane = Arc::new(LivenessCatalogPlane::new(
        vec![liveness_catalog(
            "go2",
            &[(topic, Some(robot_streaming_liveness(500_000)))],
        )],
        DiscoveryState::NotConverged,
    ));

    let (worker, _flush, _storage) = memory_worker("mon_c3_unconv");
    let (socket, dir) = temp_socket("mon_c3_unconv");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // Drive the plane well past the settle window an evidential sample would have
    // to clear. The row APPEARS (it is being polled) but must learn nothing.
    let appeared = discover_until_monitors(&mut client, Duration::from_secs(30), |mon| {
        row_for(&mon["monitors"], topic, Some("go2")).is_some()
    });
    assert!(appeared, "the row is being polled even while unconverged");
    // Past the settle window by a wide margin, driven by the same loop.
    let _ = discover_until_monitors(&mut client, Duration::from_secs(12), |_| false);

    let mon = client.request(r#"{"id":2,"method":"monitors"}"#);
    let row = row_for(&mon["monitors"], topic, Some("go2")).expect("the row");
    assert_eq!(
        row["samples"].as_u64(),
        Some(0),
        "an unconverged catalog is not evidence about any topic: {row}"
    );
    assert_eq!(
        row["state"].as_str(),
        Some("unknown"),
        "…and UNKNOWN is never conflated with healthy: {row}"
    );
    assert_eq!(row["conditions"], serde_json::json!([]), "{row}");
    // And it SAYS what it is not being told, per condition — silence would read as
    // health.
    let reasons: Vec<&str> = row["ineligible"]
        .as_array()
        .map(|a| {
            a.iter()
                .filter_map(|i| i["reason"].as_str())
                .collect::<Vec<_>>()
        })
        .unwrap_or_default();
    assert_eq!(
        reasons,
        vec![
            "no_liveness_evidence",
            "no_liveness_evidence",
            "no_liveness_evidence"
        ],
        "every condition must report that it is being judged on nothing: {row}"
    );
    assert_eq!(daemon.monitor_alerts_raised(), 0, "and nothing is raised");

    // ANTI-TAUTOLOGY: the SAME catalog, now settled, is admitted.
    plane.set_discovery(DiscoveryState::Settled);
    let learned = discover_until_monitors(&mut client, Duration::from_secs(30), |mon| {
        row_for(&mon["monitors"], topic, Some("go2"))
            .is_some_and(|row| row["samples"].as_u64().unwrap_or(0) > 0)
    });
    assert!(
        learned,
        "the stimulus was real all along — the marker is what withheld it"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

/// A SETTLED gather that stops naming a topic RETIRES its row; the sibling it still
/// names survives.
///
/// A catalog row has no lifecycle of its own. An attached row dies with its tap,
/// but a row minted from whatever a robot happened to announce would outlive the
/// robot — so a long-lived desk watching a churning LAN accumulates rows for topics
/// that no longer exist and reports each of them UNKNOWN with a forever-growing
/// age. That is the unbounded-growth class the sampler's rollback comment names, and
/// the discovery plane is what introduces it here.
///
/// The sibling is the discriminator: without it, a plane that retired EVERYTHING on
/// every gather would pass.
#[test]
fn a_settled_gather_that_stops_naming_a_topic_retires_its_monitor_row_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("cer_monitors_c3_retire");
    let (going, staying) = ("/go2/transient", "/go2/permanent");
    let plane = Arc::new(LivenessCatalogPlane::new(
        vec![liveness_catalog(
            "go2",
            &[
                (going, Some(robot_streaming_liveness(500_000))),
                (staying, Some(robot_streaming_liveness(20_000))),
            ],
        )],
        DiscoveryState::Settled,
    ));

    let (worker, _flush, _storage) = memory_worker("mon_c3_retire");
    let (socket, dir) = temp_socket("mon_c3_retire");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    let both = discover_until_monitors(&mut client, Duration::from_secs(30), |mon| {
        [going, staying]
            .iter()
            .all(|t| row_for(&mon["monitors"], t, Some("go2")).is_some())
    });
    assert!(both, "both catalog rows must open first");
    assert_eq!(
        daemon.monitor_catalog_rows_retired(),
        0,
        "precondition: nothing has been retired yet"
    );

    // The robot stops announcing one of them.
    plane.set_catalogs(vec![liveness_catalog(
        "go2",
        &[(staying, Some(robot_streaming_liveness(20_000)))],
    )]);

    let retired = discover_until_monitors(&mut client, Duration::from_secs(30), |mon| {
        row_for(&mon["monitors"], going, Some("go2")).is_none()
    });
    assert!(
        retired,
        "a SETTLED gather that stopped naming a topic must retire its row — \
         otherwise a desk watching a churning LAN grows rows without bound and \
         reports each of them UNKNOWN forever"
    );
    let mon = client.request(r#"{"id":2,"method":"monitors"}"#);
    assert!(
        row_for(&mon["monitors"], staying, Some("go2")).is_some(),
        "…and ONLY that row: the sibling the gather still names survives: {:#}",
        mon["monitors"]
    );
    assert_eq!(
        daemon.monitor_catalog_rows_retired(),
        1,
        "exactly one retirement, and it is observable without reading the verb — \
         a retirement is otherwise indistinguishable from a row never opened"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

/// A row an ATTACHED tap owns is NOT retired by a settled gather that no longer
/// lists it on the discovery plane.
///
/// This is the overlap window every attach of a catalogued topic passes through,
/// and getting it wrong is a SILENT data loss on the one row a user is actually
/// looking at: attaching a topic is exactly what removes it from the discovery
/// plane's sample set (the two sources are split by attachment), and
/// `MonitorPlane::forget` drops a row whoever owns it — so an unguarded retirement
/// would delete the attached row's learned baseline and settle state on the very
/// next `discover` the sidebar makes.
///
/// The mirror shape is the live one (netd registers the mirror before Studio checks
/// the topic, so `attach` routes it locally and the tap's `origin_robot` comes from
/// the provenance registry), cribbed from
/// `a_locally_attached_mirror_is_monitored_under_its_robot_e2e`.
#[test]
fn an_attached_row_is_not_retired_by_the_discovery_plane_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("cer_monitors_c3_attached");
    let mirror = "/go2/scan";
    let _mirror_pub = Publisher::spawn(Arc::clone(&mgr), mirror);
    assert!(
        mgr.register_mirror_provenance(mirror, "go2")
            .expect("register mirror provenance"),
        "the mirror is newly attributed to go2"
    );

    let plane = Arc::new(LivenessCatalogPlane::new(
        vec![liveness_catalog(
            "go2",
            &[(mirror, Some(robot_streaming_liveness(500_000)))],
        )],
        DiscoveryState::Settled,
    ));
    let (worker, _flush, _storage) = memory_worker("mon_c3_attach");
    let (socket, dir) = temp_socket("mon_c3_attach");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // Converge the provenance cache FIRST — the tap's record is written once, from
    // the snapshot at attach time (the sibling arm's rule).
    let disc = discover_until(&mut client, ATTRIBUTION_CONVERGENCE_BOUND, |d| {
        d["robots"].as_array().is_some_and(|robots| {
            robots.iter().any(|r| {
                r["robot"].as_str() == Some("go2")
                    && r["topics"]
                        .as_array()
                        .is_some_and(|t| t.iter().any(|e| e["topic"].as_str() == Some(mirror)))
            })
        })
    });
    assert!(
        disc["robots"].as_array().is_some_and(|r| !r.is_empty()),
        "harness precondition: the mirror is attributed before the attach: {disc:#}"
    );

    let att = client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{mirror}"}}"#
    ));
    assert_eq!(att["ok"].as_bool(), Some(true), "{att}");
    assert_eq!(
        att["robot"].as_str(),
        Some("go2"),
        "precondition: the tap knows whose data it carries, so its monitor row is \
         keyed the SAME as the catalog row it takes over: {att}"
    );

    // The attached plane now owns the row. Let it learn something, so a retirement
    // would be a visible loss rather than a no-op.
    let learned = wait_until(Duration::from_secs(30), || {
        let mon = client.request(r#"{"id":2,"method":"monitors"}"#);
        row_for(&mon["monitors"], mirror, Some("go2"))
            .is_some_and(|row| row["samples"].as_u64().unwrap_or(0) > 0)
    });
    assert!(learned, "the attached row must admit evidence first");

    // Now the robot stops announcing it — so the SETTLED gather names nothing at
    // all, and the topic is absent from the discovery plane's sample set for BOTH
    // reasons at once (attached, and un-announced).
    plane.set_catalogs(vec![liveness_catalog("go2", &[])]);
    for _ in 0..12 {
        let _ = client.request(r#"{"id":3,"method":"discover"}"#);
        std::thread::sleep(Duration::from_millis(200));
    }

    let mon = client.request(r#"{"id":4,"method":"monitors"}"#);
    let row = row_for(&mon["monitors"], mirror, Some("go2"))
        .unwrap_or_else(|| panic!("the ATTACHED row must survive: {:#}", mon["monitors"]));
    assert!(
        row["samples"].as_u64().unwrap_or(0) > 0,
        "…with everything it had learned — a retirement here deletes the baseline \
         and settle state of the one row a user is watching: {row}"
    );
    assert_eq!(
        daemon.monitor_catalog_rows_retired(),
        0,
        "the discovery plane must retire NOTHING while a tap owns the row"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

/// A COMPOSE-attached netd mirror is monitored under its robot — from the cached
/// map, never a gather.
///
/// `compose_layout` is the Studio LAYOUT path, so it is where a robot's topics are
/// most often tapped, and its local arm was the one seam that could not say whose
/// data a tap carries: a netd mirror is locally visible (netd registered it before
/// Studio composed the layout), so `attach_for_compose` opens a plain local tap and
/// the monitor row was keyed `None`. A remote robot's topic rendering as a
/// desk-local row, minting alerts that name no machine to go and look at.
///
/// The fix reads the CACHE, so this warms it first — `discover` populates
/// `mirror_provenance`, and polling until the mirror is attributed proves the entry
/// is there before the compose reads it. The cache-MISS half is the sibling arm
/// below, which is where the read-only cost contract states what it gives
/// up.
///
/// The compose is expected to REFUSE (a silent mirror resolves no archetype, the
/// same shape `a_failed_compose_releases_the_netd_demand_it_opened_e2e` uses) — and
/// that is deliberately not weakened: the taps are opened, the fill runs on them,
/// and the rollback then releases their rows. Asserting the row through a compose
/// that FAILS would test the rollback, not the fill, so the topic here PUBLISHES and
/// the compose succeeds.
#[test]
fn a_compose_attached_mirror_is_monitored_under_its_robot_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("cer_monitors_compose_attr");
    let mirror = "/monitors/compose/scan";
    let local = "/monitors/compose/desk";
    // The MIRROR: a live publisher (netd's re-inject publisher, from this daemon's
    // point of view) plus the provenance record netd registers beside it.
    let _mirror_pub = Publisher::spawn(Arc::clone(&mgr), mirror);
    assert!(
        mgr.register_mirror_provenance(mirror, "ubuntu")
            .expect("register mirror provenance"),
        "the mirror is newly attributed to ubuntu"
    );
    // The CONTROL: a genuine desk producer, which must stay unattributed.
    let _local_pub = Publisher::spawn(Arc::clone(&mgr), local);

    let plane = Arc::new(GatherPlane::new(vec![oracle_catalog(
        "ubuntu",
        &[(mirror, Some("geometry_msgs/Vector3"))],
    )]));
    let (worker, _flush, _storage) = memory_worker("monitors_compose_attr");
    let (socket, dir) = temp_socket("monitors_compose_attr");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // WARM THE CACHE. `discover` is what populates `mirror_provenance`, and the
    // compose reads that snapshot read-only — so a compose racing a cold cache would
    // legitimately stamp nothing (the sibling arm). Polling until the mirror is
    // attributed proves the entry is really there.
    let disc = discover_until(&mut client, ATTRIBUTION_CONVERGENCE_BOUND, |d| {
        d["robots"].as_array().is_some_and(|robots| {
            robots.iter().any(|r| {
                r["robot"].as_str() == Some("ubuntu")
                    && r["topics"]
                        .as_array()
                        .is_some_and(|t| t.iter().any(|e| e["topic"].as_str() == Some(mirror)))
            })
        })
    });
    assert!(
        disc["robots"].as_array().is_some_and(|r| !r.is_empty()),
        "harness precondition: the cache must hold the record before the compose \
         reads it: {disc:#}"
    );

    // THE COMPOSE — never an `attach`, which is the other seam and already pinned.
    let composed = client.request(&format!(
        r#"{{"id":2,"method":"compose_layout","intent":{{"topics":["{mirror}","{local}"]}}}}"#
    ));
    assert_eq!(
        composed["ok"].as_bool(),
        Some(true),
        "precondition: both topics publish, so the compose succeeds and its taps \
         SURVIVE — a refused compose would roll them back and this would be a test \
         of the rollback: {composed}"
    );
    assert_eq!(
        composed["attached"].as_array().map(Vec::len),
        Some(2),
        "and this call is what opened them: {composed}"
    );

    let appeared = wait_until(Duration::from_secs(15), || {
        let resp = client.request(r#"{"id":3,"method":"monitors"}"#);
        row_for(&resp["monitors"], mirror, Some("ubuntu")).is_some()
            && row_for(&resp["monitors"], local, None).is_some()
    });
    assert!(
        appeared,
        "both compose-attached taps must become watched rows"
    );

    let resp = client.request(r#"{"id":4,"method":"monitors"}"#);
    // THE PIN — keyed `(topic, robot)`, and the robot-less half is load-bearing
    // for the same reason as its `attach` twin above: the served catalog names
    // this topic under ubuntu too, so an unattributed tap leaves a second row.
    assert!(
        row_for(&resp["monitors"], mirror, Some("ubuntu")).is_some()
            && row_for(&resp["monitors"], mirror, None).is_none(),
        "a netd mirror attached by COMPOSE must be monitored under its ROBOT — this \
         is the Studio layout path, so a row keyed `None` here is the reported \
         wrongness on the seam most topics arrive through: {:#}",
        resp["monitors"]
    );
    // THE CONTROL, without which the pin would also pass a compose that attributed
    // everything it touched.
    assert!(
        row_for(&resp["monitors"], local, None).is_some(),
        "a genuine desk producer must stay unattributed — ABSENCE is how a local \
         row is spelled: {:#}",
        resp["monitors"]
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

/// A tap whose topic NAME was recently a mirror's is not left labelled with the
/// historical robot — while a genuine mirror beside it keeps its own.
///
/// THE TOMBSTONE CLASS, one step removed. The provenance map is keyed by
/// TOPIC NAME, so it describes a name rather than a tap's GENERATION, and there is
/// an ordinary window in which those disagree: retire a mirror (netd's refcount-0
/// teardown), let a desk producer take the freed single-writer slot, and attach or
/// compose it before `PROVENANCE_CACHE_TTL` expires. A snapshot taken before the
/// retirement still names the robot.
///
/// MEASURED with that answer simply RECORDED onto the
/// tap at attach time: the row reads `robot: "ubuntu"` for a topic this desk is
/// publishing itself, and 12 s of `discover` + `list` polling past the 3 s TTL does
/// not clear it — because once the record exists the HELD half of
/// `attribution_snapshot` re-asserts it to `list`/`status`/`discover` forever.
/// Affirmatively WRONG (worse than absent), permanent, and on every surface at
/// once. Both seams share the exposure: `attach` reproduces it identically.
///
/// The daemon does not pretend the map can be corroborated — the registry's whole read
/// surface is a windowed gather neither the sampler nor a handler may pay for. It
/// makes the claim FALSIFIABLE instead: the sampler re-reads the cache every pass
/// and tracks it in BOTH directions, so a hint is a per-tap cache of the live
/// answer rather than a memory of a past one.
///
/// # What this asserts, and the bound it asserts it under
///
/// The END STATE, not the transient. Inside the TTL the daemon genuinely believes
/// the stale snapshot and may hint the historical robot; that is the real cost of
/// not gathering, and asserting its absence would be asserting a race. What must
/// hold is that it CONVERGES — the arm drives `discover` (the verb that re-gathers)
/// and requires the desk topic's row to carry NO robot, bounded by
/// `ATTRIBUTION_CONVERGENCE_BOUND`.
///
/// THE ANTI-OVERCORRECTION CONTROL rides in the same body, because "never attribute
/// anything" would pass the pin above and destroy the feature: a genuine mirror
/// composed at the same moment must still be monitored under its robot at the end.
#[test]
fn a_reused_topic_name_does_not_keep_the_retired_mirrors_robot_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("cer_monitors_tombstone");
    let reused = "/monitors/tomb/reused";
    let genuine = "/monitors/tomb/genuine";

    // TWO mirrors of ubuntu's topics, both registered and both live.
    let retiring_pub = Publisher::spawn(Arc::clone(&mgr), reused);
    let _genuine_pub = Publisher::spawn(Arc::clone(&mgr), genuine);
    for topic in [reused, genuine] {
        assert!(
            mgr.register_mirror_provenance(topic, "ubuntu")
                .expect("register mirror provenance"),
            "{topic} is newly attributed to ubuntu"
        );
    }

    let plane = Arc::new(GatherPlane::new(vec![oracle_catalog(
        "ubuntu",
        &[
            (reused, Some("geometry_msgs/Vector3")),
            (genuine, Some("geometry_msgs/Vector3")),
        ],
    )]));
    let (worker, _flush, _storage) = memory_worker("monitors_tombstone");
    let (socket, dir) = temp_socket("monitors_tombstone");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // WARM the cache while BOTH are genuine mirrors — this is the snapshot that
    // goes stale under the reused name.
    let disc = discover_until(&mut client, ATTRIBUTION_CONVERGENCE_BOUND, |d| {
        d["robots"].as_array().is_some_and(|rs| {
            rs.iter().any(|r| {
                r["robot"].as_str() == Some("ubuntu")
                    && r["topics"].as_array().is_some_and(|t| {
                        [reused, genuine]
                            .iter()
                            .all(|topic| t.iter().any(|e| e["topic"].as_str() == Some(topic)))
                    })
            })
        })
    });
    assert!(
        disc["robots"].as_array().is_some_and(|r| !r.is_empty()),
        "harness precondition: the cache holds BOTH as ubuntu's before the \
         retirement makes one of them a lie: {disc:#}"
    );

    // THE RETIREMENT + THE NAME REUSE, both fast so they land well inside the TTL:
    // netd tears the mirror down, and this desk's own graph takes the freed slot.
    let retired_at = Instant::now();
    mgr.unregister_mirror_provenance(reused)
        .expect("unregister the retired mirror's provenance");
    drop(retiring_pub);
    let _desk_pub = Publisher::spawn(Arc::clone(&mgr), reused);
    let composed = client.request(&format!(
        r#"{{"id":2,"method":"compose_layout","intent":{{"topics":["{reused}","{genuine}"]}}}}"#
    ));
    assert_eq!(composed["ok"].as_bool(), Some(true), "{composed}");
    assert!(
        retired_at.elapsed() < cerulion_vizd::PROVENANCE_CACHE_TTL,
        "harness precondition: the attach must land INSIDE the TTL or the stale \
         snapshot this arm is about was never read (took {:?})",
        retired_at.elapsed()
    );

    // THE PIN: it converges to the truth. `discover` is what re-gathers, so it is
    // the verb the bound is expressed in.
    let mut last = serde_json::Value::Null;
    let converged = wait_until(ATTRIBUTION_CONVERGENCE_BOUND, || {
        client.request(r#"{"id":3,"method":"discover"}"#);
        last = client.request(r#"{"id":4,"method":"monitors"}"#);
        row_for(&last["monitors"], reused, None).is_some()
    });
    assert!(
        converged,
        "a desk producer that took a retired mirror's NAME must not stay labelled \
         with that mirror's robot — the name-keyed map describes a name, not this \
         tap's generation, and an affirmatively wrong robot is worse than none: \
         {:#}",
        last["monitors"]
    );

    // THE ANTI-OVERCORRECTION CONTROL: the sibling really is still attributed, so
    // this cannot be passed by a daemon that simply stopped attributing anything.
    assert!(
        row_for(&last["monitors"], genuine, Some("ubuntu")).is_some(),
        "a genuine mirror composed at the same moment must KEEP its robot — the guarantee \
         is falsifiability, not silence: {:#}",
        last["monitors"]
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

/// A snapshot past its TTL is not ADOPTED onto a fresh tap — and an existing hint
/// is withdrawn once a real gather says the mirror is gone.
///
/// The two halves of the freshness rule, and the first one is stated precisely
/// because the obvious statement of it is FALSE. "An expired snapshot is held rather
/// than read as an absence" describes nothing: expiry does not EMPTY the cached map,
/// and a hint is derived from that same map, so the two already agree and refreshing
/// from a stale copy changes nothing. MEASURED — neutralising the gate leaves every
/// existing hint exactly where it was.
///
/// What the gate actually decides is whether a claim that old may be ADOPTED onto a
/// tap that did not have one. That is the tail of the tombstone hazard: the older the
/// snapshot, the likelier the name has been reused since, and `compose_layout` does
/// not gather — so without the gate a compose issued long after anyone last asked
/// would take a stale name-keyed claim and put it on a desk-owned topic. With it, the
/// row is never wrong even transiently; it simply waits for someone to ask.
///
/// The second half keeps the first from being an excuse: once a verb DOES gather and
/// the mirror is genuinely gone, the hint is WITHDRAWN. Not-adopted-while-unasked and
/// withdrawn-when-answered are different rules, and a fix with only the first would
/// be the permanent mislabel again.
#[test]
fn a_snapshot_past_its_ttl_is_not_adopted_and_a_real_gather_withdraws_the_hint_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("cer_monitors_stale_hold");
    let reused = "/monitors/hold/reused";
    let held = "/monitors/hold/held";
    let retiring = Publisher::spawn(Arc::clone(&mgr), reused);
    let _held_pub = Publisher::spawn(Arc::clone(&mgr), held);
    for topic in [reused, held] {
        assert!(
            mgr.register_mirror_provenance(topic, "ubuntu")
                .expect("register mirror provenance"),
            "{topic} is newly attributed to ubuntu"
        );
    }

    let plane = Arc::new(GatherPlane::new(vec![oracle_catalog(
        "ubuntu",
        &[
            (reused, Some("geometry_msgs/Vector3")),
            (held, Some("geometry_msgs/Vector3")),
        ],
    )]));
    let (worker, _flush, _storage) = memory_worker("monitors_stale_hold");
    let (socket, dir) = temp_socket("monitors_stale_hold");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // Attach the topic that will KEEP its mirror, and warm the cache while both are
    // genuine mirrors — this is the snapshot that goes stale.
    client.request(&format!(r#"{{"id":1,"method":"attach","topic":"{held}"}}"#));
    // Monitors: the served catalog names BOTH topics under `ubuntu`, so
    // the DISCOVERY plane opens a row for each — and a topic-only lookup on this
    // verb can no longer tell a tap's row from a catalog's. What says the TAP was
    // attributed is that its row is no longer the LOCAL one: rows are keyed
    // `(topic, robot)`, so adopting `ubuntu` MOVES the tap's row off the
    // robot-less key, and the discovery plane never mints a robot-less row.
    let attributed = wait_until(ATTRIBUTION_CONVERGENCE_BOUND, || {
        client.request(r#"{"id":2,"method":"discover"}"#);
        let r = client.request(r#"{"id":3,"method":"monitors"}"#);
        row_for(&r["monitors"], held, Some("ubuntu")).is_some()
            && row_for(&r["monitors"], held, None).is_none()
    });
    assert!(
        attributed,
        "precondition: the held row is attributed to ubuntu"
    );

    // The retirement + the name reuse, then let the snapshot AGE OUT. Only the four
    // attribution-asking verbs gather and `monitors` is not one of them, so polling
    // it cannot refresh the cache: the staleness here is deterministic.
    mgr.unregister_mirror_provenance(reused)
        .expect("unregister the retired mirror's provenance");
    drop(retiring);
    let _desk_pub = Publisher::spawn(Arc::clone(&mgr), reused);
    std::thread::sleep(cerulion_vizd::PROVENANCE_CACHE_TTL + Duration::from_millis(500));

    // HALF 1 — compose the reused name. `compose_layout` does not gather, so the only
    // answer available is the expired snapshot, which still says `ubuntu`. It must
    // not be adopted: the row is never wrong, not even for one pass.
    let composed = client.request(&format!(
        r#"{{"id":4,"method":"compose_layout","intent":{{"topics":["{reused}"]}}}}"#
    ));
    assert_eq!(composed["ok"].as_bool(), Some(true), "{composed}");
    // The TAP's row is the robot-less one (see the note above): the served catalog
    // also names `reused` under `ubuntu`, so waiting for "a row with this topic"
    // would be satisfied by the DISCOVERY plane's row and would never look at the
    // tap at all.
    let appeared = wait_until(Duration::from_secs(15), || {
        let r = client.request(r#"{"id":5,"method":"monitors"}"#);
        row_for(&r["monitors"], reused, None).is_some()
    });
    assert!(appeared, "the compose-attached tap becomes a watched row");
    let deadline = Instant::now() + Duration::from_millis(1_500);
    while Instant::now() < deadline {
        let r = client.request(r#"{"id":6,"method":"monitors"}"#);
        assert!(
            row_for(&r["monitors"], reused, None).is_some(),
            "a claim older than PROVENANCE_CACHE_TTL must not be ADOPTED onto a \
             fresh tap — the older the snapshot, the likelier the name has been \
             reused since, and this topic really is the desk's own. Adopting it \
             MOVES the tap's row onto ubuntu's key, so the robot-less row \
             disappears: {:#}",
            r["monitors"]
        );
        std::thread::sleep(Duration::from_millis(100));
    }

    // HALF 2 — the still-genuine sibling was attributed BEFORE the staleness, and a
    // real gather now reports its mirror gone, so its hint is WITHDRAWN. Without
    // this, not-adopted-while-unasked would be indistinguishable from held-forever.
    mgr.unregister_mirror_provenance(held)
        .expect("unregister the held mirror's provenance");
    let withdrawn = wait_until(ATTRIBUTION_CONVERGENCE_BOUND, || {
        client.request(r#"{"id":7,"method":"discover"}"#);
        let r = client.request(r#"{"id":8,"method":"monitors"}"#);
        // A WITHDRAWAL moves the tap's row back onto the robot-less key. The
        // discovery plane's `(held, ubuntu)` row stays — the catalog still names
        // it, and that is a claim about the ROBOT, not about this tap.
        row_for(&r["monitors"], held, None).is_some()
    });
    assert!(
        withdrawn,
        "once a real gather reports the mirror gone, an existing hint is WITHDRAWN"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

/// A compose over a COLD cache stamps nothing, and a later `attach` heals it.
///
/// The costly half of the read-only cost contract, and — because the record EXISTS
/// throughout — a behavioural discriminator for read-only-ness rather than only a
/// control on fabrication. `compose_layout` reads the provenance CACHE and never
/// gathers, so a compose that runs before anything has warmed it has no answer even
/// though a gather would find one immediately. That is the whole trade: a compose
/// already spends up to `COMPOSE_PEEK_BUDGET` resolving silent topics, and buying
/// this answer with a `MIRROR_GATHER_WINDOW` on top would push a large layout toward
/// `viz_client`'s 5 s read deadline. A forced-gather implementation would stamp here
/// and fail this arm.
///
/// COLD IS DETERMINISTIC on this daemon, and the assertion rests on that: only the
/// four control handlers that ask for attribution — `discover`, `list`, `status`,
/// `attach_local` — populate `mirror_provenance`, and nothing on the poll thread
/// does. So a compose issued as the FIRST request of a fresh daemon reads an empty
/// cache by construction. (If a boot-time gather is ever added, this arm flips and
/// says so, which is the correct outcome — the cost contract would have changed.)
///
/// The heal is then driven the way a user would: `attach` pays a real gather, and
/// `retarget_origin_robot` moves the row off the `None` key it was opened under. It
/// is POLLED because that gather is best-effort; the bound is derived from
/// `PROVENANCE_CACHE_TTL`.
#[test]
fn a_compose_with_no_cached_attribution_stamps_nothing_and_is_healed_by_a_later_attach_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("cer_monitors_compose_cold");
    let mirror = "/monitors/cold/scan";
    let _mirror_pub = Publisher::spawn(Arc::clone(&mgr), mirror);
    // The record EXISTS from the start — a gather would find it. What the compose
    // must not do is go and fetch it.
    assert!(
        mgr.register_mirror_provenance(mirror, "ubuntu")
            .expect("register mirror provenance"),
        "the mirror is attributed to ubuntu BEFORE the compose — so a stamp here \
         could only come from a gather the handler must not pay for"
    );

    let plane = Arc::new(GatherPlane::new(vec![oracle_catalog(
        "ubuntu",
        &[(mirror, Some("geometry_msgs/Vector3"))],
    )]));
    let (worker, _flush, _storage) = memory_worker("monitors_compose_cold");
    let (socket, dir) = temp_socket("monitors_compose_cold");
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // THE FIRST REQUEST of this daemon's life, so nothing has warmed the provenance
    // cache. The record is there for the taking; the handler must not go and take it.
    let composed = client.request(&format!(
        r#"{{"id":1,"method":"compose_layout","intent":{{"topics":["{mirror}"]}}}}"#
    ));
    assert_eq!(
        composed["ok"].as_bool(),
        Some(true),
        "precondition: the topic publishes, so the compose succeeds: {composed}"
    );

    let appeared = wait_until(Duration::from_secs(15), || {
        let resp = client.request(r#"{"id":2,"method":"monitors"}"#);
        row_for(&resp["monitors"], mirror, None).is_some()
    });
    assert!(
        appeared,
        "the compose-attached tap must become a watched row"
    );
    let resp = client.request(r#"{"id":3,"method":"monitors"}"#);
    assert!(
        row_for(&resp["monitors"], mirror, None).is_some(),
        "the cache was COLD, so the compose must stamp NOTHING — the record exists \
         and a gather would return it, which is exactly the fetch this handler may \
         not pay for: {:#}",
        resp["monitors"]
    );

    // THE HEAL: a later `attach` DOES gather, names the robot, and the row moves off
    // the `None` key it was opened under. Polled — that gather is best-effort.
    let healed = wait_until(ATTRIBUTION_CONVERGENCE_BOUND, || {
        client.request(&format!(
            r#"{{"id":4,"method":"attach","topic":"{mirror}"}}"#
        ));
        let resp = client.request(r#"{"id":5,"method":"monitors"}"#);
        row_for(&resp["monitors"], mirror, Some("ubuntu")).is_some()
    });
    assert!(
        healed,
        "a compose that could not attribute a tap must be HEALED by the first attach \
         that names its robot — the read-only cost is a DELAY, not a permanently \
         mis-keyed row"
    );
    // And healed means MOVED, not duplicated: one row for one tap.
    let resp = client.request(r#"{"id":6,"method":"monitors"}"#);
    let rows: Vec<&Value> = resp["monitors"]
        .as_array()
        .expect("monitors")
        .iter()
        .filter(|r| r["topic"].as_str() == Some(mirror))
        .collect();
    assert_eq!(
        rows.len(),
        1,
        "the `None`-keyed row must have gone WITH the identity, not been left \
         beside the new one: {:#}",
        resp["monitors"]
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

/// A FAILED compose releases the monitor rows its own attaches opened.
///
/// `compose_layout` holds the state lock only for each individual attach, so
/// between a tap opening and a later step failing the drain loop is free to run a
/// sampling pass and OPEN a row for a topic the call is about to take away. The
/// rollback must be the inverse of what the attach did — the same rule that put
/// the stats removal and the wake-request removal in
/// `rollback_compose_attaches`, and the same one this row was missing.
///
/// What is left behind is PERMANENT, not stale-but-harmless: rows are released by
/// `MonitorPlane::forget`, whose only other caller is the `detach` verb, and there
/// is no tap left to detach. So `monitors` serves a row nothing can ever update
/// (UNKNOWN with a forever-growing age), a re-attach of the same topic inherits its
/// learned baseline and settle state instead of starting the fresh observation the
/// new tap is, and a compose loop over robot-announced names grows the row table
/// without bound.
///
/// THE WINDOW IS BUILT, NOT HOPED FOR. Three registered-but-SILENT topics: each
/// attaches (the service exists), then burns `PEEK_BOUND` (250 ms) of the shared
/// `COMPOSE_PEEK_BUDGET` resolving no archetype, so the taps are up for ~750 ms —
/// comfortably past one `MONITOR_SAMPLE_INTERVAL_NS` (400 ms). Zero placements is
/// then the legitimate refusal that takes the rollback path (the same shape
/// `a_failed_compose_releases_the_netd_demand_it_opened_e2e` uses). The precondition
/// is PROVEN rather than assumed: with no other tap attached, `monitor_snapshot`
/// mints one sample per attached tap and NOTHING otherwise, so any climb in
/// `monitor_samples_taken` over the compose is proof a pass sampled these taps and
/// therefore that the rows existed.
#[test]
fn a_failed_compose_releases_the_monitor_rows_it_opened_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("cer_monitors_rollback");
    let topics = [
        "/monitors/rb/alpha",
        "/monitors/rb/beta",
        "/monitors/rb/gamma",
    ];
    // Registered but SILENT: the service exists (so the tap attaches) and no frame
    // ever arrives (so the peek runs to its bound and no archetype resolves).
    let _silent: Vec<_> = topics
        .iter()
        .map(|t| {
            mgr.create_publisher(t, MaxSliceLen::const_new(1 << 16), 0)
                .expect("silent producer creates the service")
        })
        .collect();

    let (worker, _flush, _storage) = memory_worker("monitors_rollback");
    let (socket, dir) = temp_socket("monitors_rollback");
    let mut daemon: RunningDaemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);

    // Nothing is attached, so nothing is sampled — the baseline that makes the
    // climb below attributable to these taps and to nothing else.
    assert_eq!(
        daemon.monitor_samples_taken(),
        0,
        "precondition: an empty daemon samples nothing"
    );

    let list = topics
        .iter()
        .map(|t| format!("\"{t}\""))
        .collect::<Vec<_>>()
        .join(",");
    let composed = client.request(&format!(
        r#"{{"id":1,"method":"compose_layout","intent":{{"topics":[{list}]}}}}"#
    ));
    assert_eq!(
        composed["ok"].as_bool(),
        Some(false),
        "precondition: three silent topics resolve no archetype, so the compose \
         refuses and takes the rollback path: {composed}"
    );

    // PROOF the window was real: the only taps in existence during the compose were
    // the three it opened, so a nonzero count is a pass that sampled them.
    let sampled = daemon.monitor_samples_taken();
    assert!(
        sampled > 0,
        "harness precondition: the ~750 ms compose must span at least one \
         MONITOR_SAMPLE_INTERVAL_NS pass, or there is no phantom row to leave \
         behind and this arm proves nothing (got {sampled} samples)"
    );

    // THE PIN.
    let resp = client.request(r#"{"id":2,"method":"monitors"}"#);
    for topic in topics {
        assert!(
            row_for(&resp["monitors"], topic, None).is_none(),
            "a rolled-back compose attach must take its monitor row with it — this \
             row can never be released again, because the only other releaser is \
             `detach` and there is no tap left to detach: {:#}",
            resp["monitors"]
        );
    }
    // …and it stays gone across a sampling pass: nothing is tapped, so the sampler
    // must not re-open what the rollback released.
    std::thread::sleep(Duration::from_millis(1_200));
    let resp = client.request(r#"{"id":3,"method":"monitors"}"#);
    assert_eq!(
        resp["monitors"].as_array().map(Vec::len),
        Some(0),
        "the next sampling pass must not re-open a rolled-back row: {:#}",
        resp["monitors"]
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

/// `detach` releases the monitor row with the tap it was derived from.
///
/// Not a leak fix but a CORRECTNESS one: a re-attach genuinely begins a new
/// observation and a new baseline (the tap's whole stats entry is dropped, so its
/// Hz window and frame count start over), so anything the engine learned about the
/// old producer would describe the new one falsely. A row left behind would also
/// report UNKNOWN with a forever-growing age on a topic nobody is watching.
///
/// Cheap by construction: a row EXISTS from the first sample, long before its
/// settle window closes, so this needs no learning wall.
#[test]
fn detaching_a_topic_releases_its_monitor_row_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport("cer_monitors_detach");
    let topic = "/monitors/transient";
    let _pub = Publisher::spawn(Arc::clone(&mgr), topic);

    let (worker, _flush, _storage) = memory_worker("monitors_detach");
    let (socket, dir) = temp_socket("monitors_detach");
    let mut daemon: RunningDaemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");

    let mut client = Client::connect(&socket);
    client.request(&format!(
        r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
    ));
    let appeared = wait_until(Duration::from_secs(15), || {
        let resp = client.request(r#"{"id":2,"method":"monitors"}"#);
        row_for(&resp["monitors"], topic, None).is_some()
    });
    assert!(
        appeared,
        "the row must exist before the detach means anything"
    );

    let det = client.request(&format!(
        r#"{{"id":3,"method":"detach","topic":"{topic}"}}"#
    ));
    assert_eq!(det["ok"].as_bool(), Some(true), "{det}");

    // Released SYNCHRONOUSLY, inside the same acquisition that drops the stats
    // entry — so the very next answer is already free of it, with no sampling pass
    // in between to resurrect it.
    let resp = client.request(r#"{"id":4,"method":"monitors"}"#);
    assert!(
        row_for(&resp["monitors"], topic, None).is_none(),
        "the row must die with the tap it was derived from: {:#}",
        resp["monitors"]
    );
    // And it STAYS gone across a sampling pass — the sampler must not re-open a
    // row for a topic nothing is tapping.
    std::thread::sleep(Duration::from_millis(1_200));
    let resp = client.request(r#"{"id":5,"method":"monitors"}"#);
    assert!(
        row_for(&resp["monitors"], topic, None).is_none(),
        "and the next sampling pass must not re-open it: {:#}",
        resp["monitors"]
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

// ===========================================================================
// The `runs` verb + the local/remote fold
// ===========================================================================

/// A demand plane that answers the RUNS verb with a declared script, and nothing
/// else.
///
/// A DI double (Principle #13, not fake data — the crafted replies cross the REAL
/// `DemandPlane` trait, the REAL handler, the REAL fold and the REAL verb over a
/// REAL UDS socket). What it stands in for is netd's `query_runs`, whose own wire
/// carriage is pinned first-party in `cerulion_netd`, and whose serve half is
/// pinned over real zenoh in `cerulion_core`'s runs-query e2e.
///
/// It also COUNTS its calls and records the scope it was asked for, which is how
/// the robot-scoped arm proves the local gather was skipped rather than merely
/// unproductive.
struct RunsScriptPlane {
    answer: std::sync::Mutex<Result<cerulion_netd::RunsGather, String>>,
    asked: std::sync::Mutex<Vec<Option<String>>>,
    /// How long `query_runs` takes before answering — the REMOTE arm's span in
    /// the concurrency arm, and zero everywhere else.
    delay: Duration,
}

impl RunsScriptPlane {
    fn new(answer: Result<cerulion_netd::RunsGather, String>) -> Self {
        Self {
            answer: std::sync::Mutex::new(answer),
            asked: std::sync::Mutex::new(Vec::new()),
            delay: Duration::ZERO,
        }
    }

    /// Make the remote arm take `delay` before answering.
    fn with_delay(mut self, delay: Duration) -> Self {
        self.delay = delay;
        self
    }

    /// The scopes this plane was asked for, in order.
    fn asked(&self) -> Vec<Option<String>> {
        self.asked.lock().unwrap().clone()
    }
}

impl cerulion_vizd::DemandPlane for RunsScriptPlane {
    fn query_runs(&self, robot: Option<&str>) -> Result<cerulion_netd::RunsGather, String> {
        self.asked.lock().unwrap().push(robot.map(str::to_string));
        if self.delay > Duration::ZERO {
            std::thread::sleep(self.delay);
        }
        self.answer.lock().unwrap().clone()
    }
    fn demand(&self, _robot: &str, _topic: &str, _schema_hash: u64) -> Result<(), String> {
        Ok(())
    }
    fn release(&self, _robot: &str, _topic: &str) -> Result<(), String> {
        Ok(())
    }
    fn query_catalog(&self, _robot: &str) -> Result<Vec<cerulion_core::CatalogReply>, String> {
        Ok(Vec::new())
    }
    fn query_catalog_all(&self) -> Result<Vec<cerulion_core::CatalogReply>, String> {
        Ok(Vec::new())
    }
    fn query_schema(
        &self,
        _robot: &str,
        _requested: &str,
    ) -> Result<Vec<cerulion_core::SchemaReply>, String> {
        Ok(Vec::new())
    }
}

/// A remote gather that answered cleanly with `replies`.
fn runs_gather(replies: Vec<cerulion_core::RunsReply>) -> cerulion_netd::RunsGather {
    cerulion_netd::RunsGather {
        replies,
        unusable: Vec::new(),
        silent: Vec::new(),
        discovery: DiscoveryState::Settled,
        unsettled_for: None,
    }
}

/// A settled reply carrying one run, self-attributed to `robot`.
fn runs_reply_with(robot: &str, run_id: u128, graph: &str) -> cerulion_core::RunsReply {
    let entry = cerulion_core::RunEntry::new(
        run_id,
        graph,
        1_700_000_000_000_000_000,
        cerulion_core::transport::run_registry::RunState::Live,
        format!("name: {graph}\nnodes: []\n"),
        r#"{"version":1}"#,
    )
    .expect("a hand-built entry is well formed");
    cerulion_core::transport::cerulion_q::build_runs_reply(
        robot,
        vec![entry],
        Vec::new(),
        cerulion_core::RunsCompleteness::Settled,
    )
}

/// The row for `run_id` in a `runs` response, if present.
fn runs_row<'a>(resp: &'a Value, run_id: &str) -> Option<&'a Value> {
    resp["runs"]
        .as_array()?
        .iter()
        .find(|r| r["run_id"].as_str() == Some(run_id))
}

/// Publish a live run record on THIS manager's namespace, and answer `runs`.
///
/// **This is the arm that proves the LOCAL gather asks the right machine.** The
/// regression it guards against is one character of intent: `gather_current_runs()` reads
/// the process-GLOBAL iceoryx2 namespace, `gather_live_runs` reads the manager's
/// own. On a developer's desk both usually answer the same thing, which is exactly
/// why this is easy to get wrong and invisible afterwards — so the fixture puts a
/// run on an ISOLATED root, where the global namespace is a different machine
/// entirely and the global reader finds nothing.
///
/// The run's directory deliberately does NOT exist, and that is what keeps this arm
/// hermetic: describing a run means reading `$CERULION_HOME/runs/…`, which is
/// process-wide env this file must not mutate. An undescribable run is still
/// REPORTED (with its reason) rather than dropped, so the record's presence is
/// observable without a single filesystem write — and the arm doubles as the pin
/// that a run the desk cannot describe is named instead of silently vanishing.
#[test]
fn a_live_local_run_is_gathered_from_this_managers_namespace_e2e() {
    use cerulion_core::transport::run_registry::{RunHandle, RunRecord, RunState};

    let mgr = isolated_transport("runs_local");
    let run_id = 0x0000_0000_0000_0000_0000_0000_0000_00a1u128;
    let _run = RunHandle::publish_on_config(
        &mgr.iox_config(),
        RunRecord {
            run_id,
            supervisor_pid: std::process::id(),
            run_started_at_ns: 1_700_000_000_000_000_000,
            state: RunState::Live,
            graph_name: "perception".to_string(),
            // A directory that does not exist — see the doc above.
            run_dir: "/nonexistent/b4/local".to_string(),
        },
    )
    .expect("the run announces on this manager's namespace");

    let (worker, _flush, _storage) = memory_worker("runs_local");
    let (socket, dir) = temp_socket("runs_local");
    let plane = Arc::new(RunsScriptPlane::new(Ok(runs_gather(Vec::new()))));
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");

    let mut client = Client::connect(&socket);
    let resp = client.request(r#"{"id":1,"method":"runs"}"#);
    assert_eq!(resp["ok"].as_bool(), Some(true));

    // The LOCAL source is present and it FOUND the run — reported as undescribable
    // because its directory is not readable, which is a claim about the directory
    // and not about whether the registry was heard.
    let local = resp["sources"]
        .as_array()
        .expect("sources")
        .iter()
        .find(|s| s.get("robot").is_none())
        .expect("a LOCAL source is reported");
    let undescribable = local["undescribable"].as_array().unwrap_or_else(|| {
        panic!("the local source names the run it could not describe: {resp:#}")
    });
    assert_eq!(
        undescribable.len(),
        1,
        "the run published on THIS manager's namespace must be found — a gather on \
         the process-global namespace would see nothing here: {resp:#}"
    );
    assert_eq!(
        undescribable[0]["run_id"].as_str(),
        Some("0x000000000000000000000000000000a1"),
        "…and it is named by identity, so the row is actionable: {resp:#}"
    );
    assert!(
        !undescribable[0]["reason"]
            .as_str()
            .expect("a reason travels")
            .is_empty(),
        "a run withheld without a reason cannot be acted on: {resp:#}"
    );

    // A withheld run is never an absence claim.
    assert_eq!(
        resp["completeness"]["kind"].as_str(),
        Some("incomplete"),
        "a machine whose only run is undescribable serves an EMPTY list, and a \
         settled verdict there would be a confident 'nothing is running' about a \
         machine that is running something: {resp:#}"
    );
    assert!(resp["runs"].as_array().expect("runs").is_empty());

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

/// The FOLD, end to end: a local source and a remote robot in one answer, each
/// attributed to its own machine.
///
/// **The attribution arm on the wire.** The local source must carry NO `robot` key at
/// all — not `null`, and certainly not this desk's hostname — while the robot's row
/// names it. A response that stamped both would make a desk and a robot
/// indistinguishable in the one list whose purpose is saying where something runs.
#[test]
fn local_and_remote_runs_fold_into_one_attributed_list_e2e() {
    let mgr = isolated_transport("runs_fold");
    let (worker, _flush, _storage) = memory_worker("runs_fold");
    let (socket, dir) = temp_socket("runs_fold");
    let plane = Arc::new(RunsScriptPlane::new(Ok(runs_gather(vec![
        runs_reply_with("go2", 0xb1, "nav"),
    ]))));
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");

    let mut client = Client::connect(&socket);
    let resp = client.request(r#"{"id":1,"method":"runs"}"#);

    let go2 = runs_row(&resp, "0x000000000000000000000000000000b1")
        .unwrap_or_else(|| panic!("the robot's run is served: {resp:#}"));
    assert_eq!(go2["robot"].as_str(), Some("go2"));
    assert_eq!(go2["graph_name"].as_str(), Some("nav"));
    assert_eq!(
        go2["graph_yaml"].as_str(),
        Some("name: nav\nnodes: []\n"),
        "the effective graph travels VERBATIM — it is what the desk re-parses"
    );

    // The LOCAL source answered (this desk runs nothing), and it carries no robot.
    let sources = resp["sources"].as_array().expect("sources");
    assert_eq!(sources.len(), 2, "both machines are reported: {resp:#}");
    let local = sources
        .iter()
        .find(|s| s.get("robot").is_none())
        .unwrap_or_else(|| panic!("the LOCAL source omits `robot` entirely: {resp:#}"));
    assert_eq!(local["completeness"]["kind"].as_str(), Some("settled"));
    assert!(
        sources.iter().any(|s| s["robot"].as_str() == Some("go2")),
        "…beside the robot's own: {resp:#}"
    );

    // Nothing local ran, the LAN settled, nothing was unusable — the one shape
    // allowed to claim the list is complete.
    assert_eq!(resp["completeness"]["kind"].as_str(), Some("settled"));
    assert_eq!(resp["discovery"].as_str(), Some("settled"));
    assert!(resp.get("unusable").is_none(), "healthy wire: {resp:#}");
    assert!(resp.get("degraded").is_none(), "healthy wire: {resp:#}");

    // The unscoped request asked the LAN as a whole.
    assert_eq!(plane.asked(), vec![None]);

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

/// A robot that answered UNUSABLY renders as a REDEPLOY, never as "still
/// discovering" — the gather-to-wire hand-off, end to end.
///
/// The fixture reports discovery as SETTLED on purpose: that removes the one
/// excuse a renderer might have for treating this as a transient. Such a robot
/// answered, and it will re-serve the same undecodable bytes forever.
#[test]
fn a_robot_that_answered_unusably_reaches_the_wire_as_a_redeploy_e2e() {
    let mgr = isolated_transport("runs_unusable");
    let (worker, _flush, _storage) = memory_worker("runs_unusable");
    let (socket, dir) = temp_socket("runs_unusable");
    let plane = Arc::new(RunsScriptPlane::new(Ok(cerulion_netd::RunsGather {
        replies: vec![runs_reply_with("go2", 0xc1, "nav")],
        unusable: vec![cerulion_core::UnusableRunsAnswer {
            robot: "spot".to_string(),
            reason: "runs reply version 2 is newer than this binary understands".to_string(),
        }],
        silent: Vec::new(),
        discovery: DiscoveryState::Settled,
        unsettled_for: None,
    })));
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");

    let mut client = Client::connect(&socket);
    let resp = client.request(r#"{"id":1,"method":"runs"}"#);

    let unusable = resp["unusable"]
        .as_array()
        .unwrap_or_else(|| panic!("the skewed robot reaches the wire: {resp:#}"));
    assert_eq!(unusable.len(), 1);
    assert_eq!(unusable[0]["robot"].as_str(), Some("spot"));
    assert!(
        unusable[0]["reason"]
            .as_str()
            .expect("reason")
            .contains("newer than this binary"),
        "the REASON travels — a bare count cannot be acted on: {resp:#}"
    );
    assert_eq!(
        resp["discovery"].as_str(),
        Some("settled"),
        "discovery HAD converged, so nothing here may be rendered as 'still \
         discovering': {resp:#}"
    );
    assert_eq!(
        resp["completeness"]["kind"].as_str(),
        Some("incomplete"),
        "a robot whose bytes we could not decode may be running anything: {resp:#}"
    );
    assert!(
        runs_row(&resp, "0x000000000000000000000000000000c1").is_some(),
        "…while the robot that DID answer is still served: {resp:#}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

/// A netd failure is `degraded`, and the LOCAL half of the answer still stands.
#[test]
fn a_netd_failure_degrades_the_remote_half_and_keeps_the_local_one_e2e() {
    let mgr = isolated_transport("runs_degraded");
    let (worker, _flush, _storage) = memory_worker("runs_degraded");
    let (socket, dir) = temp_socket("runs_degraded");
    let plane = Arc::new(RunsScriptPlane::new(Err(
        "cerulion-netd unreachable".to_string()
    )));
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");

    let mut client = Client::connect(&socket);
    let resp = client.request(r#"{"id":1,"method":"runs"}"#);

    assert_eq!(
        resp["ok"].as_bool(),
        Some(true),
        "a degrade is not a failure"
    );
    assert_eq!(
        resp["degraded"].as_str(),
        Some("cerulion-netd unreachable"),
        "the reason is named, so an operator knows to look at netd: {resp:#}"
    );
    assert!(
        resp.get("discovery").is_none(),
        "and NO discovery state is claimed — a desk that could not ask cannot \
         report how much of the LAN was searched: {resp:#}"
    );
    assert_eq!(
        resp["completeness"]["kind"].as_str(),
        Some("incomplete"),
        "…so the empty list is explicitly not an absence claim: {resp:#}"
    );
    // The LOCAL arm still ran and still answered.
    assert!(
        resp["sources"]
            .as_array()
            .expect("sources")
            .iter()
            .any(|s| s.get("robot").is_none()),
        "the local half is unaffected by a remote failure: {resp:#}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

/// A ROBOT-SCOPED request asks only that robot, and contributes no local rows.
///
/// The local gather is SKIPPED rather than run-and-filtered, which the plane's own
/// call record proves: it was asked for `go2` and only `go2`. That is the attribution
/// rule showing through — a local run is attributed to NO robot, so no robot name
/// could ever have selected one.
#[test]
fn a_robot_scoped_runs_request_asks_only_that_robot_e2e() {
    use cerulion_core::transport::run_registry::{RunHandle, RunRecord, RunState};

    let mgr = isolated_transport("runs_scoped");
    // A LIVE local run the scoped request must NOT report.
    let _run = RunHandle::publish_on_config(
        &mgr.iox_config(),
        RunRecord {
            run_id: 0x00d1,
            supervisor_pid: std::process::id(),
            run_started_at_ns: 1_700_000_000_000_000_000,
            state: RunState::Live,
            graph_name: "perception".to_string(),
            run_dir: "/nonexistent/b4/scoped".to_string(),
        },
    )
    .expect("the local run announces");

    let (worker, _flush, _storage) = memory_worker("runs_scoped");
    let (socket, dir) = temp_socket("runs_scoped");
    let plane = Arc::new(RunsScriptPlane::new(Ok(runs_gather(vec![
        runs_reply_with("go2", 0xd2, "nav"),
    ]))));
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");

    let mut client = Client::connect(&socket);
    let resp = client.request(r#"{"id":1,"method":"runs","robot":"go2"}"#);

    assert_eq!(
        plane.asked(),
        vec![Some("go2".to_string())],
        "the scope reaches netd: {resp:#}"
    );
    assert!(
        resp["sources"]
            .as_array()
            .expect("sources")
            .iter()
            .all(|s| s.get("robot").is_some()),
        "NO local source appears — the local gather was skipped, not merely \
         unproductive: {resp:#}"
    );
    assert!(
        runs_row(&resp, "0x000000000000000000000000000000d2").is_some(),
        "…and the robot's own run is served: {resp:#}"
    );
    // The live local run is absent from BOTH halves of the answer.
    let local_seen = resp["runs"]
        .as_array()
        .expect("runs")
        .iter()
        .any(|r| r.get("robot").is_none());
    assert!(
        !local_seen,
        "no local row on a robot-scoped answer: {resp:#}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

/// A robot that was ASKED and stayed SILENT reaches the wire and FORBIDS the
/// absence claim — the coverage half.
///
/// A silent robot contributes no source, so its absence from `sources` is
/// indistinguishable from a robot nobody asked about: a response holding only
/// replies and unusable answers reads a half-answered LAN exactly like a fully
/// answered one. Settling there renders that robot's runs as "nothing is running
/// there" — the confident-empty class, one layer above the discovery latch that
/// closes it on netd's side, which is why the fixture holds `discovery` at SETTLED
/// and `unusable` EMPTY: the silence is the only thing that can refuse the claim.
#[test]
fn a_silent_robot_reaches_the_wire_and_forbids_the_absence_claim_e2e() {
    let mgr = isolated_transport("runs_silent");
    let (worker, _flush, _storage) = memory_worker("runs_silent");
    let (socket, dir) = temp_socket("runs_silent");
    let plane = Arc::new(RunsScriptPlane::new(Ok(cerulion_netd::RunsGather {
        replies: vec![runs_reply_with("go2", 0xe1, "nav")],
        unusable: Vec::new(),
        silent: vec!["spot".to_string()],
        discovery: DiscoveryState::Settled,
        unsettled_for: None,
    })));
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");

    let mut client = Client::connect(&socket);
    let resp = client.request(r#"{"id":1,"method":"runs"}"#);

    assert_eq!(
        resp["silent"]
            .as_array()
            .unwrap_or_else(|| panic!("the silent robot reaches the wire: {resp:#}")),
        &vec![serde_json::Value::from("spot")],
    );
    assert!(
        resp.get("unusable").is_none(),
        "silence is NOT an undecodable answer — the remedies differ: {resp:#}"
    );
    assert_eq!(
        resp["discovery"].as_str(),
        Some("settled"),
        "netd finished looking AND asked this robot, so 'still discovering' is \
         not the accurate rendering either: {resp:#}"
    );
    assert_eq!(
        resp["completeness"]["kind"].as_str(),
        Some("incomplete"),
        "the answer covers fewer machines than the question: {resp:#}"
    );
    assert!(
        runs_row(&resp, "0x000000000000000000000000000000e1").is_some(),
        "…while the robot that DID answer is served in full: {resp:#}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

/// The two arms run CONCURRENTLY: the request's wall is their MAX, not their SUM.
///
/// This is a DEADLINE property, not a speed one. `viz_client` arms a 5 s
/// `SO_RCVTIMEO` on every reply read, so past it the caller does not get a slow
/// answer — it gets an errno and `cerulion viz` dies. Run sequentially, a LOCAL
/// gather that spends its full window plus a remote arm at netd's measured worst
/// case is enough to cross that, and the local half is exactly what a robot-scoped
/// request already skips — so the desk asking the ordinary UNSCOPED question was
/// the one shape that could time out.
///
/// # Why the local arm needs an injected delay
///
/// Both real arms resist being made slow: the registry gather takes iceoryx2's
/// zero-publisher fast path on a desk running nothing, so it is near-instant, and
/// a slow REMOTE arm alone proves nothing — `max(0, D)` and `0 + D` are the same
/// number. Only a slow LOCAL arm separates the sum from the max.
///
/// # Bounds
///
/// Both arms are given the SAME span, so a sequential handler takes ~2x it and a
/// concurrent one ~1x. The assertion is a CEILING at 1.5x — halfway between, with
/// 0.5x of margin on each side — which contention can only push a passing run
/// toward, never a failing one away from (load lengthens a wall, so a sequential
/// handler cannot dip under the ceiling by being slow). The FLOOR at 1x is the
/// anti-vacuity half: it proves the delay was really injected, so the ceiling is
/// not satisfied by a handler that skipped the work.
#[test]
fn the_two_runs_arms_gather_concurrently_so_the_wall_is_their_max_e2e() {
    /// Each arm's span. Generous in absolute terms (a whole second is orders of
    /// magnitude above the ~µs of real work either arm does in this fixture), so
    /// the discriminator is the DESIGN rather than a timing race.
    const ARM: Duration = Duration::from_millis(1_000);

    let mgr = isolated_transport("runs_conc");
    let (worker, _flush, _storage) = memory_worker("runs_conc");
    let (socket, dir) = temp_socket("runs_conc");
    // The REMOTE arm sleeps for `ARM` before answering.
    let plane = Arc::new(
        RunsScriptPlane::new(Ok(runs_gather(vec![runs_reply_with("go2", 0xe2, "nav")])))
            .with_delay(ARM),
    );
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");
    // …and the LOCAL arm for the same span.
    daemon.inject_runs_local_delay_for_test(ARM.as_nanos() as u64);

    let mut client = Client::connect(&socket);
    let started = Instant::now();
    let resp = client.request(r#"{"id":1,"method":"runs"}"#);
    let wall = started.elapsed();

    // The answer is CORRECT as well as prompt — a handler that raced its arms and
    // dropped one would be fast for the wrong reason.
    assert_eq!(resp["ok"].as_bool(), Some(true));
    assert!(
        runs_row(&resp, "0x000000000000000000000000000000e2").is_some(),
        "the remote arm's run is served: {resp:#}"
    );
    assert!(
        resp["sources"]
            .as_array()
            .expect("sources")
            .iter()
            .any(|s| s.get("robot").is_none()),
        "…and the local arm ran too: {resp:#}"
    );

    // ANTI-VACUITY: the injected work really happened, so the ceiling below is not
    // satisfied by a handler that did neither arm.
    assert!(
        wall >= ARM,
        "both arms were given {ARM:?} each, so the request cannot beat one of \
         them — measured {wall:?}; the delay was not injected"
    );
    // THE PIN: concurrent ⇒ ~1x, sequential ⇒ ~2x. Load can only lengthen a wall,
    // so a sequential handler cannot pass this by being slow.
    assert!(
        wall < ARM.mul_f64(1.5),
        "the two runs arms must gather CONCURRENTLY — measured {wall:?} against \
         two {ARM:?} arms, i.e. their SUM. Run sequentially this crosses \
         viz_client's 5 s reply deadline on an ordinary unscoped request, and the \
         caller dies on an errno rather than waiting."
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

/// A daemon whose demand plane predates the verb DEGRADES rather than claiming an
/// empty LAN.
///
/// The trait's default `query_runs` is an `Err`, and this is the arm that proves
/// that choice is load-bearing at the seam rather than merely documented: the
/// plane here implements nothing beyond the older verbs, exactly as a plane without `query_runs`
/// (or any of this file's other doubles) does, and the answer says so.
#[test]
fn a_plane_that_cannot_query_runs_degrades_rather_than_claiming_an_empty_lan_e2e() {
    let mgr = isolated_transport("runs_default");
    let (worker, _flush, _storage) = memory_worker("runs_default");
    let (socket, dir) = temp_socket("runs_default");
    // `NoNetdPlane` does not implement `query_runs`, so it falls to
    // the trait default.
    let mut daemon = start_hermetic(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
    )
    .expect("daemon starts");

    let mut client = Client::connect(&socket);
    let resp = client.request(r#"{"id":1,"method":"runs"}"#);

    assert_eq!(resp["ok"].as_bool(), Some(true));
    assert!(
        resp["degraded"]
            .as_str()
            .expect("a plane that cannot ask says so")
            .contains("cannot query runs"),
        "the default is an explicit non-claim, never an empty-and-settled LAN: {resp:#}"
    );
    assert_eq!(
        resp["completeness"]["kind"].as_str(),
        Some("incomplete"),
        "{resp:#}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

// ── A REMOTE attach reports the archetype its FRAMES classify as ────────────
//
// A remote attach seeds `status` from the pinned schema NAME, so a row has an
// archetype before any frame arrives. The name table cannot see content: it maps
// every `sensor_msgs/CompressedImage` to `Image`, while the sink draws one whose
// `data` is an H.264 access unit as `VideoStream` (the shape a Go2 front camera
// arrives in). So the first decodable frame must replace the name guess, and a
// re-attach (what the Studio sidebar sends on a re-check) must not write the
// guess back over it. Without the frame verdict, `status` and `list` read `Image`
// for as long as the camera streams, and the layout keeps the views of a still
// image for a topic the sink renders as video.
//
// The demand plane is the RECORDING double: its demand succeeds without creating
// a mirror, and the test's own publisher on the SAME manager is the local service
// a netd mirror would have been. The recorded demand proves both attaches took the
// remote arm, and that the re-attach did not demand a second mirror.
#[test]
fn a_remote_h264_compressed_image_attach_reports_the_video_stream_archetype_e2e() {
    let _statics = blueprint_statics_guard();
    let mgr = isolated_transport_with_network("camremote");
    let topic = "/vizd/camremote";
    let keyframe = h264_frame(&annex_b(&[GO2_SPS, GO2_PPS, GO2_IDR_HEAD]));
    let _publisher = Publisher::spawn_fixed_frame(Arc::clone(&mgr), topic, keyframe);
    let (worker, _flush, _storage) = memory_worker("camremote");
    let (socket, dir) = temp_socket("camremote");
    let plane = Arc::new(RecordingDemandPlane::default());
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(&mgr),
        worker,
        builtin_walker(),
        None,
        Arc::clone(&plane) as Arc<dyn cerulion_vizd::DemandPlane>,
    )
    .expect("daemon starts");
    let mut client = Client::connect(&socket);
    let attach = |client: &mut Client, id: u64| -> Value {
        client.request(&format!(
            r#"{{"id":{id},"method":"attach","topic":"{topic}","robot":"go2","schema":"sensor_msgs/CompressedImage"}}"#
        ))
    };
    let status_row = |client: &mut Client, id: u64| -> Option<Value> {
        let st = client.request(&format!(r#"{{"id":{id},"method":"status"}}"#));
        entry_for(&st["topics"], "topic", topic).cloned()
    };

    let att = attach(&mut client, 1);
    assert_eq!(att["ok"].as_bool(), Some(true), "remote attach: {att}");
    assert_eq!(att["robot"].as_str(), Some("go2"), "{att}");
    assert_eq!(att["already_attached"].as_bool(), Some(false), "{att}");
    assert_eq!(
        att["schema"].as_str(),
        Some("sensor_msgs/CompressedImage"),
        "{att}"
    );
    // Both answers are correct at this instant: the name guess, or the frame
    // verdict when the poll thread drained a frame before the reply was built.
    assert!(
        matches!(att["archetype"].as_str(), Some("Image" | "VideoStream")),
        "the attach reply names the name guess or the frame verdict: {att}"
    );

    // THE pin: once a frame decodes, `status` names what the sink draws.
    let mut id = 10;
    assert!(
        wait_until(Duration::from_secs(10), || {
            id += 1;
            status_row(&mut client, id)
                .is_some_and(|r| r["archetype"].as_str() == Some("VideoStream"))
        }),
        "status must name the archetype the frames classify as: {:?}",
        status_row(&mut client, 2)
    );
    let row = status_row(&mut client, 3).expect("status row");
    assert_eq!(
        row["schema"].as_str(),
        Some("sensor_msgs/CompressedImage"),
        "the schema stays the pinned type: {row}"
    );
    let list = client.request(r#"{"id":4,"method":"list"}"#);
    let listed = entry_for(&list["attached"], "topic", topic).expect("list row");
    assert_eq!(
        listed["archetype"].as_str(),
        Some("VideoStream"),
        "list reads the same verdict: {listed}"
    );

    // A RE-ATTACH of the streaming topic must not write the name guess back over
    // the frame verdict. Nothing would replace it again: the poll thread stops
    // classifying once a frame has resolved the topic.
    let re = attach(&mut client, 5);
    assert_eq!(re["ok"].as_bool(), Some(true), "re-attach: {re}");
    assert_eq!(re["already_attached"].as_bool(), Some(true), "{re}");
    assert_eq!(
        re["archetype"].as_str(),
        Some("VideoStream"),
        "the re-attach reply names the frame verdict: {re}"
    );
    for id in 101..=110 {
        let row = status_row(&mut client, id).expect("status row");
        assert_eq!(
            row["archetype"].as_str(),
            Some("VideoStream"),
            "a re-attach must not revert status to the name guess: {row}"
        );
        std::thread::sleep(Duration::from_millis(50));
    }
    assert_eq!(
        *plane.demands.lock().unwrap(),
        vec![(
            "go2".to_string(),
            topic.to_string(),
            <CompressedImage as ShmMessage>::SCHEMA_HASH,
        )],
        "exactly one demand, for the pinned CompressedImage type"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}
